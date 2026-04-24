#!/usr/bin/env python3

"""
Centralised Object-Detection Coverage Planner
==============================================
Runs at 1 Hz.  Computes a minimum set of 360-degree spin positions whose
0.5 m scan discs cover the full reachable free space of the shared map,
then assigns those scan points to available scanner drones using the
Hungarian algorithm (scipy linear_sum_assignment).

Role in the multi-drone system
------------------------------
This planner is the object-detection equivalent of goal_assigner.py.
It is designed to run alongside (or after) a set of frontier-exploration
drones + their GoalAssigner.  The frontier drones build the shared map;
scanner drones then visit hover points whose union covers everything
the frontier drones revealed.

Pipeline
--------
1.  Read the current shared occupancy grid.
2.  Subtract cells already marked scanned (i.e. within SCAN_RADIUS of a
    previously completed spin by any scanner drone in the fleet).
3.  Sample candidate spin positions on a grid over free space.
4.  Run greedy set cover to get the minimum set of candidates that
    covers all uncovered free cells.
5.  Use the Hungarian algorithm to assign candidates to drones such that
    total distance is minimised across all drone-scan-point pairs.
6.  Publish per-drone goals on /{prefix}/assigned_scan_point.

Termination conditions (same shape as goal_assigner.py)
-------------------------------------------------------
  Condition 1 - zero uncovered cells 3 times in a row  → send all home
  Condition 2 - all drones FAILED, nothing assignable  → send all home

Drone lifecycle messages
------------------------
Listens on /{prefix}/goal_status:
  "REACHED"  - drone finished a spin, mark cells, reassign
  "FAILED"   - A* failed or drone gave up, don't reassign this point
  "RECALLED" - drone is going home / landing, remove from pool permanently

Replan triggers (requirement 6)
-------------------------------
The set cover is recomputed on a fixed periodic timer - every
PERIODIC_REPLAN_INTERVAL seconds the current plan is thrown away and
rebuilt from the latest map.  This catches every kind of map change
(frontier drones extending the map, walls revealed, map shrinking
around dynamic obstacles) without needing per-change thresholds.
A first plan is always run as soon as the map and at least one drone
position are available.

Topic summary
-------------
Subscribes:
  /map                              OccupancyGrid  - shared map
  /{prefix}/odom                    Odometry       - each scanner's position
  /{prefix}/goal_status             String         - REACHED|FAILED|RECALLED

Publishes:
  /{prefix}/assigned_scan_point     Point          - (x,y,0) goal, or
                                                     (NaN,NaN,0) for go-home
  /scanner_planner/scan_points      MarkerArray    - all planned spins
  /scanner_planner/scan_discs       MarkerArray    - 0.5 m discs (req 2)
  /scanner_planner/scanned_points   MarkerArray    - completed spins
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

# ── Coverage parameters ───────────────────────────────────────────────────────
SCAN_RADIUS       = 1.0                              # metres - sensor range
SCAN_RADIUS_CELLS = int(SCAN_RADIUS / MAP_RES)       # 5 cells at 0.1 m/cell
SAMPLE_STEP_CELLS = SCAN_RADIUS_CELLS                # candidate spacing

# ── Assignment parameters ─────────────────────────────────────────────────────
ASSIGN_RATE_HZ           = 1.0
PERIODIC_REPLAN_INTERVAL = 15.0    # seconds - reconsider scan order (req 6)
MIN_GOAL_DIST            = 0.25    # ignore candidates this close to a drone

# ── Termination parameters ────────────────────────────────────────────────────
ZERO_COVERAGE_LIMIT = 3   # successive empty-coverage ticks before calling done

# ── Visualisation ─────────────────────────────────────────────────────────────
MARKER_Z = 0.3   # metres - draw markers slightly above the floor


class DroneState:
    """Tracks per-drone assignment state."""
    def __init__(self, prefix: str):
        self.prefix   = prefix
        self.position = None          # (x, y) from odom
        self.goal     = None          # (x, y) currently assigned
        self.status   = 'IDLE'        # IDLE|NAVIGATING|REACHED|FAILED|RECALLED


class ObjectDetectionPlanner(Node):

    def __init__(self):
        super().__init__('object_detection_planner')

        self.declare_parameter('robot_prefixes', ['/cf1', '/cf2'])
        prefixes = self.get_parameter('robot_prefixes').value

        # ── Map state ─────────────────────────────────────────────────────────
        self.map_data   = None
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        # ── Coverage memory (global across scanner drones) ────────────────────
        # Grid cells (row, col) already covered by a completed spin anywhere
        # in the fleet.  Subtracted from the input to set cover so replanning
        # never generates redundant scan points.
        self.scanned_cells: set = set()

        # World-space positions of completed spins (for RViz only).
        self.completed_scan_positions: list = []

        # Current set-cover output: ordered list of (wx, wy) world positions.
        # Ordering within this list does not matter - Hungarian picks per
        # drone from the full list each assignment tick.
        self.scan_points: list = []

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
                Point, prefix + '/assigned_scan_point', 10)

        # ── Map subscription ──────────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        # ── Visualisation publishers (requirement 2) ──────────────────────────
        self.scan_point_pub = self.create_publisher(
            MarkerArray, '/scanner_planner/scan_points', 10)
        self.scan_disc_pub = self.create_publisher(
            MarkerArray, '/scanner_planner/scan_discs', 10)
        self.scanned_pub = self.create_publisher(
            MarkerArray, '/scanner_planner/scanned_points', 10)

        # ── Termination state ─────────────────────────────────────────────────
        self.zero_coverage_count   = 0      # successive empty ticks
        self.planning_done         = False  # latched once go-home sent
        self.planning_start_time   = None   # first non-empty tick

        # ── Replan bookkeeping (requirement 6) ────────────────────────────────
        self.last_plan_time = 0.0
        self.have_valid_plan = False

        # ── Assignment timer ──────────────────────────────────────────────────
        self.create_timer(1.0 / ASSIGN_RATE_HZ, self._assign_cb)

        self.get_logger().info(
            f'ObjectDetectionPlanner started for drones: {prefixes}')

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

        if status == 'REACHED':
            # The drone just finished a spin at its assigned goal.
            # Mark the cells covered by that spin as scanned globally, so
            # future plans don't reassign the same disc.
            if ds.goal is not None:
                self._mark_scanned(ds.goal[0], ds.goal[1])
                self.completed_scan_positions.append(ds.goal)
                self._publish_scanned_markers()
                self.get_logger().info(
                    f'[{prefix}] REACHED ({ds.goal[0]:.2f},{ds.goal[1]:.2f}) '
                    f'- marked scanned. Total completed: '
                    f'{len(self.completed_scan_positions)}')
            ds.status = 'REACHED'
            ds.goal   = None

        elif status == 'FAILED':
            self.get_logger().warn(
                f'[{prefix}] FAILED for goal {ds.goal} - clearing.')
            ds.status = 'FAILED'
            ds.goal   = None

        elif status == 'RECALLED':
            # Requirement 3 - drone recalled or landed, remove from the pool.
            self.get_logger().info(
                f'[{prefix}] RECALLED - removing from scanner pool.')
            ds.status = 'RECALLED'
            ds.goal   = None

    def _map_cb(self, msg: OccupancyGrid):
        self.map_data   = np.array(msg.data, dtype=np.int8)
        self.map_width  = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y]

    # ══════════════════════════════════════════════════════════════════════════
    # Main assignment loop - runs at ASSIGN_RATE_HZ
    # ══════════════════════════════════════════════════════════════════════════

    def _assign_cb(self):
        if self.planning_done:
            return
        if self.map_data is None:
            return

        # Drones without position yet still count for termination checks,
        # but can't be assigned.  Require at least one drone to have position.
        if not any(ds.position is not None
                   for ds in self.drones.values()
                   if ds.status != 'RECALLED'):
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # Run coverage planning if anything has changed.
        if self._should_replan(now):
            self._run_coverage_plan()
            self.last_plan_time  = now
            self.have_valid_plan = True

        # Filter out scan points within SCAN_RADIUS of already-completed spins
        # (defence in depth - _run_coverage_plan already excludes scanned cells)
        live_points = [
            (wx, wy) for (wx, wy) in self.scan_points
            if not self._is_covered_by_completed(wx, wy)
        ]
        self.scan_points = live_points

        # ── Condition 1: no coverage points to assign ─────────────────────────
        if not self.scan_points:
            self.zero_coverage_count += 1
            self.get_logger().warn(
                f'No uncovered scan points '
                f'({self.zero_coverage_count}/{ZERO_COVERAGE_LIMIT} tries).')
            if self.zero_coverage_count >= ZERO_COVERAGE_LIMIT:
                self._handle_coverage_complete(now)
            return

        self.zero_coverage_count = 0
        if self.planning_start_time is None:
            self.planning_start_time = now

        # ── Condition 2: all drones FAILED, try once, else go home ────────────
        active_drones = [ds for ds in self.drones.values()
                         if ds.status != 'RECALLED']
        all_failed = (len(active_drones) > 0 and all(
            ds.goal is None and ds.status == 'FAILED'
            for ds in active_drones))
        if all_failed:
            assigned_any = self._do_assignments()
            if not assigned_any:
                self.get_logger().warn(
                    f'All active drones FAILED and no valid assignments. '
                    f'Sending all home.')
                self._send_all_home()
            return

        # ── Normal assignment ─────────────────────────────────────────────────
        self._do_assignments()

    def _should_replan(self, now: float) -> bool:
        """Two triggers: no plan yet, or the periodic interval has passed."""
        if not self.have_valid_plan:
            return True
        if (now - self.last_plan_time) >= PERIODIC_REPLAN_INTERVAL:
            self.get_logger().info(
                f'Periodic replan ({PERIODIC_REPLAN_INTERVAL:.0f}s elapsed) '
                f'- reconsidering optimal scan order.')
            return True
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Termination helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _handle_coverage_complete(self, now: float):
        elapsed = (0.0 if self.planning_start_time is None
                   else now - self.planning_start_time)
        self.get_logger().info(
            f'Coverage complete after {elapsed:.1f}s. '
            f'Completed spins: {len(self.completed_scan_positions)}. '
            f'Sending all drones home.')
        self._send_all_home()

    def _send_all_home(self):
        """Publish NaN to every active drone's assigned_scan_point topic."""
        self.planning_done = True
        for prefix, pub in self.goal_pubs.items():
            if self.drones[prefix].status == 'RECALLED':
                continue
            msg   = Point()
            msg.x = float('nan')
            msg.y = float('nan')
            msg.z = 0.0
            pub.publish(msg)
            self.get_logger().info(
                f'[{prefix}] go-home signal sent (NaN goal).')

    # ══════════════════════════════════════════════════════════════════════════
    # Assignment - Hungarian algorithm over (drones × scan points)
    # ══════════════════════════════════════════════════════════════════════════

    def _do_assignments(self) -> bool:
        """
        Assign scan points to drones that need a goal using the Hungarian
        algorithm over the pure Euclidean distance cost matrix.  Returns
        True if at least one drone was successfully given a goal, or if
        all active drones are already navigating.
        """
        # Who needs a new goal?  Only active (non-recalled) drones with
        # no current assignment.  (Requirement 3: recalled drones never
        # get assigned.)
        drones_needing_goal = []
        for prefix, ds in self.drones.items():
            if ds.status == 'RECALLED':
                continue
            if ds.position is None:
                continue
            if ds.goal is None:
                drones_needing_goal.append(prefix)

        if not drones_needing_goal:
            return True   # everyone busy

        # Goals already held by other (non-recalled) drones are off-limits.
        taken = {ds.goal for ds in self.drones.values()
                 if ds.goal is not None and ds.status != 'RECALLED'}

        # Candidates = scan points not taken and not too close to any
        # drone needing a goal (so a drone doesn't get assigned a point
        # essentially on top of it).
        candidates = []
        for (wx, wy) in self.scan_points:
            if (wx, wy) in taken:
                continue
            candidates.append((wx, wy))

        if not candidates:
            self.get_logger().warn('No uncontested candidates available.')
            return False

        # ── Build the cost matrix ─────────────────────────────────────────────
        # Rows = drones needing goal, cols = candidates.
        # cost[i][j] = Euclidean distance from drone i to candidate j.
        # Invalid pairs (candidate too close to the drone) get a sentinel.
        n_d = len(drones_needing_goal)
        n_c = len(candidates)
        cost = np.full((n_d, n_c), fill_value=1e6)

        for i, prefix in enumerate(drones_needing_goal):
            px, py = self.drones[prefix].position
            for j, (wx, wy) in enumerate(candidates):
                dist = math.hypot(wx - px, wy - py)
                if dist >= MIN_GOAL_DIST:
                    cost[i][j] = dist

        # Hungarian requires a square-ish matrix; scipy handles rectangular.
        # It returns an assignment that minimises total cost.
        row_ind, col_ind = linear_sum_assignment(cost)

        assigned_any = False
        for i, j in zip(row_ind, col_ind):
            if cost[i][j] >= 1e6:
                prefix = drones_needing_goal[i]
                self.get_logger().warn(
                    f'[{prefix}] Hungarian found no valid scan point.')
                continue

            prefix    = drones_needing_goal[i]
            wx, wy    = candidates[j]
            ds        = self.drones[prefix]
            ds.goal   = (wx, wy)
            ds.status = 'NAVIGATING'
            assigned_any = True

            msg   = Point()
            msg.x = float(wx)
            msg.y = float(wy)
            msg.z = 0.0
            self.goal_pubs[prefix].publish(msg)
            self.get_logger().info(
                f'[{prefix}] assigned → ({wx:.2f},{wy:.2f}) '
                f'dist={cost[i][j]:.2f}m')

        # Re-publish markers every tick so RViz reflects the current state.
        self._publish_scan_point_markers()
        self._publish_scan_disc_markers()

        return assigned_any

    # ══════════════════════════════════════════════════════════════════════════
    # Coverage planning - greedy set cover over the shared map
    # ══════════════════════════════════════════════════════════════════════════

    def _run_coverage_plan(self):
        """
        Rebuild self.scan_points from the current map state.

        1.  Find free cells not already in scanned_cells.
        2.  Sample candidates on a grid over free space.
        3.  Greedy set cover.
        4.  Save the selected positions; ordering is handled per tick
            by the Hungarian assignment.
        """
        W, H = self.map_width, self.map_height

        # Step 1 - uncovered free cells
        free_indices = np.where(self.map_data == 0)[0]
        uncovered    = set()
        for idx in free_indices:
            r, c = divmod(int(idx), W)
            if (r, c) not in self.scanned_cells:
                uncovered.add((r, c))

        if not uncovered:
            self.scan_points = []
            return

        # Step 2 - candidate positions on a coarse grid over free cells
        step = max(1, SAMPLE_STEP_CELLS)
        candidates = []
        for r in range(0, H, step):
            for c in range(0, W, step):
                if self.map_data[r * W + c] == 0:
                    candidates.append((r, c))

        if not candidates:
            self.scan_points = []
            return

        # Step 3 - greedy set cover
        sr2 = SCAN_RADIUS_CELLS ** 2

        def coverage_set(cr, cc):
            cells = set()
            for dr in range(-SCAN_RADIUS_CELLS, SCAN_RADIUS_CELLS + 1):
                nr = cr + dr
                if nr < 0 or nr >= H:
                    continue
                max_dc = int(math.sqrt(sr2 - dr * dr))
                for dc in range(-max_dc, max_dc + 1):
                    nc = cc + dc
                    if 0 <= nc < W and (nr, nc) in uncovered:
                        cells.add((nr, nc))
            return cells

        cov_map   = {(cr, cc): coverage_set(cr, cc) for cr, cc in candidates}
        remaining = set(uncovered)
        selected  = []

        while remaining:
            best_cand  = None
            best_count = 0
            for cand, covered in list(cov_map.items()):
                n = len(covered & remaining)
                if n == 0:
                    cov_map.pop(cand)
                    continue
                if n > best_count:
                    best_count = n
                    best_cand  = cand
            if best_cand is None:
                break
            selected.append(best_cand)
            remaining -= cov_map[best_cand]
            cov_map.pop(best_cand)

        # Step 4 - convert grid cells to world coords
        self.scan_points = [self._grid_to_world(r, c) for r, c in selected]

        self.get_logger().info(
            f'Coverage plan: {len(self.scan_points)} scan points for '
            f'{len(uncovered)} uncovered cells '
            f'(already scanned: {len(self.scanned_cells)} cells).')

    def _mark_scanned(self, wx: float, wy: float):
        """
        Mark all free cells within SCAN_RADIUS of (wx, wy) as scanned.
        Called whenever a scanner drone reports REACHED at its goal.
        """
        if self.map_data is None:
            return
        W, H   = self.map_width, self.map_height
        cr, cc = self._world_to_grid(wx, wy)
        sr2    = SCAN_RADIUS_CELLS ** 2

        for dr in range(-SCAN_RADIUS_CELLS, SCAN_RADIUS_CELLS + 1):
            nr = cr + dr
            if nr < 0 or nr >= H:
                continue
            max_dc = int(math.sqrt(sr2 - dr * dr))
            for dc in range(-max_dc, max_dc + 1):
                nc = cc + dc
                if 0 <= nc < W and self.map_data[nr * W + nc] == 0:
                    self.scanned_cells.add((nr, nc))

    def _is_covered_by_completed(self, wx: float, wy: float) -> bool:
        """True if (wx, wy) is within SCAN_RADIUS of any completed spin."""
        for (cx, cy) in self.completed_scan_positions:
            if math.hypot(wx - cx, wy - cy) < SCAN_RADIUS:
                return True
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Grid <-> world helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _world_to_grid(self, wx, wy):
        return (int((wy - self.map_origin[1]) / MAP_RES),
                int((wx - self.map_origin[0]) / MAP_RES))

    def _grid_to_world(self, row, col):
        return (self.map_origin[0] + (col + 0.5) * MAP_RES,
                self.map_origin[1] + (row + 0.5) * MAP_RES)

    # ══════════════════════════════════════════════════════════════════════════
    # Visualisation  (requirement 2)
    # ══════════════════════════════════════════════════════════════════════════
    #
    # Three namespaces:
    #   scan_points    yellow spheres  - pending scan queue
    #   scan_discs     translucent cylinders radius=0.5m - the area each spin
    #                                                      will cover
    #   scanned        grey spheres    - completed spins
    #
    # Each publisher starts with a DELETEALL to clear the previous frame,
    # matching the style used by goal_assigner.py.

    def _publish_scan_point_markers(self):
        arr   = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns              = 'scan_points'
        clear.action          = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (wx, wy) in enumerate(self.scan_points):
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'scan_points'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = MARKER_Z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.20
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.9
            arr.markers.append(m)

        self.scan_point_pub.publish(arr)

    def _publish_scan_disc_markers(self):
        """Requirement 2 - draw the 0.5 m disc each spin will cover."""
        arr   = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns              = 'scan_discs'
        clear.action          = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (wx, wy) in enumerate(self.scan_points):
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'scan_discs'
            m.id                 = i + 1
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = 0.05
            m.pose.orientation.w = 1.0
            # Diameter = 2 * radius; flat cylinder (~5 cm tall)
            m.scale.x = m.scale.y = 2.0 * SCAN_RADIUS
            m.scale.z = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.2, 0.25
            arr.markers.append(m)

        self.scan_disc_pub.publish(arr)

    def _publish_scanned_markers(self):
        arr   = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns              = 'scanned'
        clear.action          = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (wx, wy) in enumerate(self.completed_scan_positions):
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'scanned'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = MARKER_Z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.6, 0.6, 0.7
            arr.markers.append(m)

        self.scanned_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionPlanner()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()