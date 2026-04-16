#!/usr/bin/env python3

"""
Centralised Goal Assigner
=========================
Runs at 1 Hz. Detects frontier clusters from the shared map, scores them
by information value (cluster size and unknown density), and assigns
non-overlapping goals using the Hungarian algorithm (scipy
linear_sum_assignment). The cost matrix is Euclidean distance minus
frontier score, so the algorithm simultaneously minimises travel distance
and maximises information gain across all drone-frontier pairs.

Publishes one goal per drone on /cfX/assigned_goal (geometry_msgs/Point).
Listens to /cfX/goal_status (std_msgs/String: "REACHED" or "FAILED") so
it only reassigns on completion or genuine failure — never on a scoring
fluctuation.

Termination conditions:

  Condition 1 — zero clusters 3 times in a row:
    The map is considered fully explored — send all drones home.

  Condition 2 — all drones FAILED with no assignable frontiers remaining:
    Send all drones home.

When done, publishes NaN to /cfX/assigned_goal as a go-home signal.
The drone_navigator interprets x=NaN as go-home-and-land.

Topic summary
─────────────
Subscribes:
  /map                       OccupancyGrid  — shared map from shared_mapper
  /cfX/odom                  Odometry       — each drone's position
  /cfX/goal_status           String         — REACHED | FAILED from drone node

Publishes:
  /cfX/assigned_goal         Point          — (x, y, 0) goal, or (NaN,NaN,0)
                                              to signal go-home
  /assigner/frontiers        MarkerArray    — frontier cluster vis in RViz
"""

import math
import numpy as np
from collections import deque
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# ── Map constants (must match shared_mapper) ──────────────────────────────────
MAP_RES = 0.1

# ── Frontier detection ────────────────────────────────────────────────────────
FRONTIER_STEP     = 1
MIN_CLUSTER_SIZE  = 3
MIN_FRONTIER_DIST = 0.4   # ignore frontiers closer than this to a drone

# ── Scoring weights ───────────────────────────────────────────────────────────
SIZE_WEIGHT    = 2.0
UNKNOWN_WEIGHT = 3.0

# ── Assignment parameters ─────────────────────────────────────────────────────
ASSIGN_RATE_HZ       = 1.0
FRONTIER_GONE_THRESH = 0.25   # reassign if <25% of original frontier remains

# ── Termination parameters ────────────────────────────────────────────────────
ZERO_CLUSTER_LIMIT = 3    # successive empty-cluster ticks before declaring done


class DroneState:
    """Tracks per-drone assignment state."""
    def __init__(self, prefix: str):
        self.prefix            = prefix
        self.position          = None    # (x, y)
        self.goal              = None    # (x, y) currently assigned
        self.status            = 'IDLE'  # IDLE | NAVIGATING | REACHED | FAILED
        self.goal_cluster_size = 0       # cluster size when goal was assigned


class GoalAssigner(Node):

    def __init__(self):
        super().__init__('goal_assigner')

        self.declare_parameter('robot_prefixes', ['/cf1', '/cf2'])
        prefixes = self.get_parameter('robot_prefixes').value

        # ── Map state ─────────────────────────────────────────────────────────
        self.map_data   = None
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        # ── Per-drone state ───────────────────────────────────────────────────
        self.drones:    dict[str, DroneState] = {}
        self.goal_pubs: dict[str, object]     = {}

        for prefix in prefixes:
            self.drones[prefix] = DroneState(prefix)

            self.create_subscription(
                Odometry, prefix + '/odom',
                lambda msg, p=prefix: self._odom_cb(msg, p), 10)

            self.create_subscription(
                String, prefix + '/goal_status',
                lambda msg, p=prefix: self._status_cb(msg, p), 10)

            self.goal_pubs[prefix] = self.create_publisher(
                Point, prefix + '/assigned_goal', 10)

        # ── Map subscription ──────────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        # ── Visualisation ─────────────────────────────────────────────────────
        self.frontier_vis_pub = self.create_publisher(
            MarkerArray, '/assigner/frontiers', 10)

        # ── Termination state ─────────────────────────────────────────────────
        self.zero_cluster_count     = 0      # successive ticks with 0 clusters
        self.exploration_done       = False  # latched once go-home sent
        self.exploration_start_time = None   # set on first non-empty tick

        # ── Assignment timer ──────────────────────────────────────────────────
        self.create_timer(1.0 / ASSIGN_RATE_HZ, self._assign_cb)

        self.get_logger().info(f'GoalAssigner started for drones: {prefixes}')

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry, prefix: str):
        self.drones[prefix].position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    def _status_cb(self, msg: String, prefix: str):
        ds     = self.drones[prefix]
        status = msg.data.strip().upper()
        if status in ('REACHED', 'FAILED'):
            self.get_logger().info(
                f'[{prefix}] reported {status} for goal {ds.goal} — clearing.')
            ds.status = status
            ds.goal   = None

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data   = np.array(msg.data, dtype=np.int8)
        self.map_width  = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y]

    # ══════════════════════════════════════════════════════════════════════════
    # Main assignment loop — runs at ASSIGN_RATE_HZ
    # ══════════════════════════════════════════════════════════════════════════

    def _assign_cb(self):
        # Once the go-home signal has been sent, do nothing further.
        if self.exploration_done:
            return

        if self.map_data is None:
            return
        if not all(ds.position is not None for ds in self.drones.values()):
            return

        now      = self.get_clock().now().nanoseconds * 1e-9
        clusters = self._get_scored_clusters()
        self._publish_frontier_markers(clusters)

        # ── Condition 1 + 2: zero clusters → BFS check ───────────────────────
        # Mirrors the original: count successive empty ticks, then on the 3rd
        # run BFS with a cooldown. Reset counter if BFS finds reachable unknowns.
        if not clusters:
            self.zero_cluster_count += 1
            self.get_logger().warn(
                f'No frontier clusters found '
                f'({self.zero_cluster_count}/{ZERO_CLUSTER_LIMIT} successive tries).')
            if self.zero_cluster_count >= ZERO_CLUSTER_LIMIT:
                self._handle_no_clusters(now)
            return

        # Clusters found — reset zero-cluster counter and record start time.
        self.zero_cluster_count = 0
        if self.exploration_start_time is None:
            self.exploration_start_time = now

        # ── Condition 3: all drones FAILED, nothing assignable ────────────────
        # If every drone has no goal and last reported FAILED, try to assign.
        # If the assignment attempt produces nothing, exploration is done.
        all_failed = all(
            ds.goal is None and ds.status == 'FAILED'
            for ds in self.drones.values())
        if all_failed:
            assigned_any = self._do_assignments(clusters, now)
            if not assigned_any:
                elapsed = self._elapsed(now)
                self.get_logger().warn(
                    f'All drones FAILED and no assignable frontiers remain. '
                    f't={elapsed:.1f}s → sending all drones home.')
                self._send_all_home()
            return

        # ── Normal assignment ─────────────────────────────────────────────────
        self._do_assignments(clusters, now)

    # ══════════════════════════════════════════════════════════════════════════
    # Termination helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _handle_no_clusters(self, now: float):
        """
        Called when zero_cluster_count >= ZERO_CLUSTER_LIMIT.
        If clusters have been empty for this many successive ticks the map
        is considered fully explored — send all drones home.
        """
        elapsed = self._elapsed(now)
        self.get_logger().info(
            f'No clusters for {ZERO_CLUSTER_LIMIT} successive ticks. '
            f't={elapsed:.1f}s → sending all drones home.')
        self._send_all_home()

    def _send_all_home(self):
        """
        Publish NaN to every drone's assigned_goal topic.
        drone_navigator interprets x=NaN as go-home-and-land, matching
        the DONE state transition in the original node.
        """
        self.exploration_done = True
        for prefix, pub in self.goal_pubs.items():
            msg   = Point()
            msg.x = float('nan')
            msg.y = float('nan')
            msg.z = 0.0
            pub.publish(msg)
            self.get_logger().info(f'[{prefix}] go-home signal sent (NaN goal).')

    def _elapsed(self, now: float) -> float:
        if self.exploration_start_time is None:
            return 0.0
        return now - self.exploration_start_time

    # ══════════════════════════════════════════════════════════════════════════
    # Assignment logic
    # ══════════════════════════════════════════════════════════════════════════

    def _do_assignments(self, clusters: list, now: float) -> bool:
        """
        Assign goals to drones that need one.
        Returns True if at least one drone was successfully given a goal,
        or if all drones are already navigating (nothing to do).
        """
        drones_needing_goal = []
        for prefix, ds in self.drones.items():
            if ds.goal is None:
                drones_needing_goal.append(prefix)
            elif self._frontier_gone(ds):
                self.get_logger().info(
                    f'[{prefix}] assigned frontier has been explored — reassigning.')
                ds.goal = None
                drones_needing_goal.append(prefix)

        if not drones_needing_goal:
            return True   # all drones navigating — not a failure

        # Goals held by drones currently navigating are off-limits
        taken = {ds.goal for ds in self.drones.values()
                 if ds.goal is not None}

        # Filter clusters to candidates valid for at least one drone:
        #   - not already taken by a navigating drone
        #   - not too close to every drone needing a goal
        candidates = []
        for wx, wy, size, unknown, score in clusters:
            if any(math.hypot(wx - tx, wy - ty) < 0.5
                   for (tx, ty) in taken if (tx, ty) is not None):
                continue
            if any(math.hypot(wx - self.drones[p].position[0],
                              wy - self.drones[p].position[1]) >= MIN_FRONTIER_DIST
                   for p in drones_needing_goal):
                candidates.append((wx, wy, size, score))

        if not candidates:
            self.get_logger().warn('No uncontested frontier candidates available.')
            return False

        n_drones     = len(drones_needing_goal)
        n_candidates = len(candidates)

        # Build cost matrix: rows = drones needing goal, cols = candidate frontiers.
        # cost[i][j] = Euclidean distance to frontier j minus its information score.
        # Hungarian minimises cost, so subtracting score makes high-value frontiers
        # cheaper and therefore preferred. Invalid pairs get a large sentinel value.
        cost = np.full((n_drones, n_candidates), fill_value=1e6)
        for i, prefix in enumerate(drones_needing_goal):
            px, py = self.drones[prefix].position
            for j, (wx, wy, size, score) in enumerate(candidates):
                dist = math.hypot(wx - px, wy - py)
                if dist >= MIN_FRONTIER_DIST:
                    cost[i][j] = dist - score

        # Run the Hungarian algorithm — finds the globally optimal assignment
        # that minimises total cost across all drone-frontier pairs simultaneously.
        row_ind, col_ind = linear_sum_assignment(cost)

        assigned_any = False
        for i, j in zip(row_ind, col_ind):
            if cost[i][j] >= 1e6:
                prefix = drones_needing_goal[i]
                self.get_logger().warn(
                    f'[{prefix}] Hungarian found no valid frontier — skipping.')
                continue

            prefix              = drones_needing_goal[i]
            wx, wy, size, score = candidates[j]
            ds                  = self.drones[prefix]
            ds.goal              = (wx, wy)
            ds.status            = 'NAVIGATING'
            ds.goal_cluster_size = size
            taken.add((wx, wy))
            assigned_any = True

            msg   = Point()
            msg.x = float(wx)
            msg.y = float(wy)
            msg.z = 0.0
            self.goal_pubs[prefix].publish(msg)
            self.get_logger().info(
                f'[{prefix}] assigned → ({wx:.2f},{wy:.2f}) '
                f'cost={cost[i][j]:.2f} size={size}')

        return assigned_any

    # ══════════════════════════════════════════════════════════════════════════
    # Frontier detection, clustering, scoring
    # ══════════════════════════════════════════════════════════════════════════

    def _get_scored_clusters(self):
        W, H = self.map_width, self.map_height
        data = self.map_data
        fc   = set()

        for row in range(1, H - 1, FRONTIER_STEP):
            for col in range(1, W - 1, FRONTIER_STEP):
                if data[row * W + col] != 0:
                    continue
                if -1 in (data[(row-1)*W+col], data[(row+1)*W+col],
                          data[row*W+(col-1)], data[row*W+(col+1)]):
                    fc.add((row, col))

        if not fc:
            return []

        result = []
        for cluster in self._cluster(fc):
            n = len(cluster)
            if n < MIN_CLUSTER_SIZE:
                continue

            best_r, best_c, best_unk = 0, 0, -1
            for r, c in cluster:
                unk = sum(
                    1 for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]
                    if 0 <= r+dr < H and 0 <= c+dc < W
                    and data[(r+dr)*W+(c+dc)] == -1)
                if unk > best_unk:
                    best_unk       = unk
                    best_r, best_c = r, c

            if data[best_r * W + best_c] != 0:
                for r, c in cluster:
                    if data[r * W + c] == 0:
                        best_r, best_c = r, c
                        break

            wx = self.map_origin[0] + (best_c + 0.5) * MAP_RES
            wy = self.map_origin[1] + (best_r + 0.5) * MAP_RES

            total_unk = 0
            for r, c in cluster:
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        nr, nc = r+dr, c+dc
                        if 0 <= nr < H and 0 <= nc < W:
                            if data[nr*W+nc] == -1:
                                total_unk += 1
            unknown = total_unk / max(1, n)

            score = SIZE_WEIGHT * n + UNKNOWN_WEIGHT * unknown
            result.append((wx, wy, n, unknown, score))

        result.sort(key=lambda x: x[4], reverse=True)
        return result

    def _cluster(self, frontier_cells):
        unvisited = set(frontier_cells)
        clusters  = []
        step      = FRONTIER_STEP
        while unvisited:
            seed    = next(iter(unvisited))
            cluster = []
            queue   = deque([seed])
            unvisited.remove(seed)
            while queue:
                r, c = queue.popleft()
                cluster.append((r, c))
                for dr in [-step, 0, step]:
                    for dc in [-step, 0, step]:
                        if dr == 0 and dc == 0:
                            continue
                        nb = (r+dr, c+dc)
                        if nb in unvisited:
                            unvisited.remove(nb)
                            queue.append(nb)
            clusters.append(cluster)
        return clusters

    def _frontier_gone(self, ds: DroneState) -> bool:
        if ds.goal is None or self.map_data is None:
            return False
        gx, gy = ds.goal
        gr     = int((gy - self.map_origin[1]) / MAP_RES)
        gc     = int((gx - self.map_origin[0]) / MAP_RES)
        W, H   = self.map_width, self.map_height
        data   = self.map_data
        radius = 5
        remaining = 0
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                nr, nc = gr+dr, gc+dc
                if 0 <= nr < H and 0 <= nc < W:
                    if data[nr*W+nc] == 0:
                        for ddr, ddc in [(-1,0),(1,0),(0,-1),(0,1)]:
                            nnr, nnc = nr+ddr, nc+ddc
                            if 0 <= nnr < H and 0 <= nnc < W:
                                if data[nnr*W+nnc] == -1:
                                    remaining += 1
                                    break
        if ds.goal_cluster_size == 0:
            return False
        return (remaining / ds.goal_cluster_size) < FRONTIER_GONE_THRESH

    # ══════════════════════════════════════════════════════════════════════════
    # Visualisation
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_frontier_markers(self, clusters):
        arr   = MarkerArray()
        clear = Marker()
        clear.action          = Marker.DELETEALL
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns              = 'frontiers'
        arr.markers.append(clear)

        for i, (wx, wy, size, unknown, score) in enumerate(clusters):
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'frontiers'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = 0.3
            m.pose.orientation.w = 1.0
            s = max(0.1, min(0.4, size * 0.02))
            m.scale.x = m.scale.y = m.scale.z = s
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.8, 0.0, 0.9
            arr.markers.append(m)

        self.frontier_vis_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = GoalAssigner()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()