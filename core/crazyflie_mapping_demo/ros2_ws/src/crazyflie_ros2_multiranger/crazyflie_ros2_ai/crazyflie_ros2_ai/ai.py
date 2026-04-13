#!/usr/bin/env python3

"""
Coverage Scanner for Crazyflie
===============================
Scans an entire known (or live-updating) map by navigating to the minimum
number of scan points and spinning 360 degrees at each one.

Sensor model:
  - Single front-facing rangefinder, 0.5 m effective distance
  - Always active — no explicit trigger needed
  - A full 360 spin at any position covers all cells within 0.5 m of that point

Coverage algorithm:
  - Greedy set cover over free cells from the shared map
  - Candidate positions sampled every SCAN_RADIUS metres across free space
  - Already-scanned cells (from completed spins) are excluded from the input
    so replanning never generates redundant scan points
  - After each spin, map_dirty is checked; if enough new free cells have
    appeared since the last plan, PLAN_COVERAGE reruns over the uncovered
    remainder only

State machine:
  TAKEOFF        — climb to cruise height
  PLAN_COVERAGE  — greedy set cover → nearest-neighbour ordered queue
  NAVIGATE       — A* path following to next scan point
  SPINNING       — 360 degree spin at scan point
  DONE           — return home via A*
  LANDING        — descend and stop
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations
import math
import heapq
import numpy as np
from enum import Enum, auto
from collections import deque

# ── Map constants ──────────────────────────────────────────────────────────────
MAP_RES = 0.1

# ── Flight parameters ──────────────────────────────────────────────────────────
TAKEOFF_HEIGHT    = 0.3
TAKEOFF_DELAY     = 3.0
CRUISE_SPEED      = 0.3
MAX_TURN_RATE     = 0.2
GOAL_REACHED_DIST = 0.15

# ── A* parameters ──────────────────────────────────────────────────────────────
WALL_INFLATION_CELLS  = 2
PROXIMITY_COST_WEIGHT = 2
PROXIMITY_COST_RADIUS = 10
WAYPOINT_SPACING      = 2

# ── Spin parameters ────────────────────────────────────────────────────────────
SPIN_RATE         = 0.5
SPIN_TARGET       = 2.0 * math.pi   # full 360

# ── Coverage parameters ────────────────────────────────────────────────────────
SCAN_RADIUS       = 0.5             # metres — sensor effective range
SCAN_RADIUS_CELLS = int(SCAN_RADIUS / MAP_RES)   # 5 cells at 0.1 m/cell
SAMPLE_STEP_CELLS = SCAN_RADIUS_CELLS             # candidate grid spacing

# A replan is triggered after a spin if this many new free cells have appeared
# since the last plan.  pi * r^2 in cells ~ 78 cells at 0.1 m/cell, r=5 cells.
REPLAN_THRESHOLD  = int(math.pi * SCAN_RADIUS_CELLS ** 2)

# ── Stuck / replan parameters ──────────────────────────────────────────────────
STUCK_PROGRESS_DIST       = 0.20
STUCK_TIMEOUT             = 5.0
MAX_STUCK_EVENTS_PER_GOAL = 2
MAX_REPLANS_PER_GOAL      = 3
REPLAN_COOLDOWN           = 3.5


class State(Enum):
    TAKEOFF        = auto()
    PLAN_COVERAGE  = auto()
    NAVIGATE       = auto()
    SPINNING       = auto()
    DONE           = auto()
    LANDING        = auto()


class GoalHealth(Enum):
    HEALTHY = auto()
    REPLAN  = auto()
    ABANDON = auto()


class CoverageScanner(Node):

    def __init__(self):
        super().__init__('coverage_scanner')

        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        # ── Flight state ───────────────────────────────────────────────────────
        self.state    = State.TAKEOFF
        self.position = [0.0, 0.0, 0.0]
        self.angles   = [0.0, 0.0, 0.0]

        self.start_pos  = None
        self.start_time = None

        # ── Map state ──────────────────────────────────────────────────────────
        self.map_data   = None
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.map_received      = False
        self.position_received = False

        # Number of free cells the last plan was built from.  Used to decide
        # whether enough new free cells have appeared to warrant a replan.
        self.free_cells_at_last_plan = 0

        # Set to True by map_callback whenever new free cells appear.
        # Cleared by PLAN_COVERAGE after replanning.
        self.map_dirty = False

        # ── Coverage memory ────────────────────────────────────────────────────
        # Grid cells (row, col) that have already been covered by a completed spin.
        self.scanned_cells: set = set()

        # World-space positions of completed scan points (for RViz only).
        self.completed_scan_positions: list = []

        # ── Scan queue ─────────────────────────────────────────────────────────
        # Ordered list of (wx, wy) world positions to visit.
        self.scan_queue: list = []

        # ── Navigation state ───────────────────────────────────────────────────
        self.goal             = None
        self.waypoints        = []
        self.current_wp       = None
        self.needs_replan     = False
        self.navigating_home  = False
        self.last_replan_time = 0.0
        self.last_replan_pos  = None
        self.last_path_cost   = None

        # Stuck detection
        self.goal_start_pos        = None
        self.goal_start_time       = 0.0
        self.last_progress_pos     = None
        self.last_progress_time    = 0.0
        self.stuck_events_for_goal = 0
        self.replan_count_for_goal = 0

        # ── Spin state ─────────────────────────────────────────────────────────
        self.spin_start_yaw      = None
        self.spin_total_rotation = 0.0
        self._last_spin_yaw      = 0.0

        # ── Logging ────────────────────────────────────────────────────────────
        self.exploration_start_time = None
        self.last_status_log_time   = 0.0
        self.status_log_period      = 2.0

        # ── ROS I/O ────────────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.cmd_pub = self.create_publisher(
            Twist, robot_prefix + '/cmd_vel_raw', 10)
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, robot_prefix + '/waypoints', 10)
        self.drone_marker_pub = self.create_publisher(
            Marker, robot_prefix + '/drone_pose', 10)
        self.scan_point_marker_pub = self.create_publisher(
            MarkerArray, robot_prefix + '/scan_points', 10)
        self.scanned_marker_pub = self.create_publisher(
            MarkerArray, robot_prefix + '/scanned_points', 10)
        self.goal_marker_pub = self.create_publisher(
            Marker, robot_prefix + '/scan_goal', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self._publish_vel(z=TAKEOFF_HEIGHT)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._info(f'Coverage scanner started. prefix={robot_prefix}')

    # ══════════════════════════════════════════════════════════════════════════
    # Logging
    # ══════════════════════════════════════════════════════════════════════════

    def _info(self, msg):
        self.get_logger().info(f'[{self.state.name}] {msg}')

    def _warn(self, msg):
        self.get_logger().warn(f'[{self.state.name}] {msg}')

    def _err(self, msg):
        self.get_logger().error(f'[{self.state.name}] {msg}')

    # ══════════════════════════════════════════════════════════════════════════
    # ROS callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.angles = list(tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]))
        if not self.position_received:
            self.position_received = True
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = self.get_clock().now().nanoseconds * 1e-9

    def map_callback(self, msg):
        self.map_data   = np.array(msg.data, dtype=np.int8)
        self.map_width  = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y]
        self.map_received = True

        # Count current free cells and flag dirty if the map has grown
        # meaningfully since the last plan.  This is deliberately cheap —
        # just a numpy comparison — so it runs on every map message without
        # stalling the main loop.
        current_free = int(np.sum(self.map_data == 0))
        new_cells    = current_free - self.free_cells_at_last_plan
        if new_cells >= REPLAN_THRESHOLD:
            self.map_dirty = True

    # ══════════════════════════════════════════════════════════════════════════
    # Main timer
    # ══════════════════════════════════════════════════════════════════════════

    def timer_callback(self):
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._state_machine(now)
        except Exception as e:
            import traceback
            self._err(f'Crash: {e}\n{traceback.format_exc()}')
            self._publish_vel()
            self.timer.cancel()

    # ══════════════════════════════════════════════════════════════════════════
    # State machine
    # ══════════════════════════════════════════════════════════════════════════

    def _state_machine(self, now):
        self._log_status(now)

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self.state == State.TAKEOFF:
            self._publish_vel(z=TAKEOFF_HEIGHT)
            airborne = self.position[2] >= TAKEOFF_HEIGHT * 0.8
            if airborne and now - self.start_time > TAKEOFF_DELAY:
                self.start_pos = [self.position[0], self.position[1]]
                self._info(
                    f'Takeoff complete. '
                    f'Home: ({self.start_pos[0]:.3f},{self.start_pos[1]:.3f})')
                self.state = State.PLAN_COVERAGE

        # ── PLAN_COVERAGE ─────────────────────────────────────────────────────
        elif self.state == State.PLAN_COVERAGE:
            if not self.map_received or not self.position_received:
                self._info('Waiting for map and odometry...')
                self._publish_vel()
                return

            self._run_coverage_plan(now)

            if not self.scan_queue:
                self._info('No uncovered scan points found. Going home.')
                self.state = State.DONE
                return

            self._info(
                f'Coverage plan ready: {len(self.scan_queue)} scan point(s).')
            self._publish_scan_point_markers(self.scan_queue)
            self._set_next_goal(now)

        # ── NAVIGATE ──────────────────────────────────────────────────────────
        elif self.state == State.NAVIGATE:
            if self.goal is None:
                self._warn('No goal — replanning coverage.')
                self.state = State.PLAN_COVERAGE
                self._publish_vel()
                return

            # Validate that the current goal cell is still free.
            # A map update may have revealed it as occupied since it was queued.
            gr, gc = self._world_to_grid(self.goal[0], self.goal[1])
            gr = max(0, min(self.map_height - 1, gr))
            gc = max(0, min(self.map_width  - 1, gc))
            if self.map_data is not None and self.map_data[gr * self.map_width + gc] == 100:
                self._warn(
                    f'Goal ({self.goal[0]:.2f},{self.goal[1]:.2f}) is now inside '
                    f'a wall — dropping and replanning.')
                self.goal = None
                self.state = State.PLAN_COVERAGE
                self._publish_vel()
                return

            # Layer 1: stuck detection
            health = self._goal_health(now)
            if health == GoalHealth.ABANDON:
                self._warn(
                    f'Goal ({self.goal[0]:.2f},{self.goal[1]:.2f}) abandoned '
                    f'after too many stuck events. Replanning coverage.')
                self.goal = None
                self.state = State.PLAN_COVERAGE
                return
            if health == GoalHealth.REPLAN:
                self._execute_replan(now)
                return

            # Layer 2: waypoint following
            if not self.waypoints and self.current_wp is None:
                # Waypoints exhausted — goal reached
                self._on_goal_reached(now)
                return

            if self._follow_waypoints(now):
                self._on_goal_reached(now)

        # ── SPINNING ──────────────────────────────────────────────────────────
        elif self.state == State.SPINNING:
            self._publish_vel(wz=SPIN_RATE)

            yaw_delta = self._wrap_angle(
                self.angles[2] - self._last_spin_yaw)
            self._last_spin_yaw       = self.angles[2]
            self.spin_total_rotation += abs(yaw_delta)

            if self.spin_total_rotation >= SPIN_TARGET:
                self._publish_vel()
                self._info(
                    f'360 spin complete at '
                    f'({self.goal[0] if self.goal else "?":.2f},'
                    f'{self.goal[1] if self.goal else "?":.2f})')

                # Mark cells covered by this spin
                if self.goal is not None:
                    self._mark_scanned(self.goal[0], self.goal[1])
                    self.completed_scan_positions.append(
                        (self.goal[0], self.goal[1]))
                    self._publish_scanned_markers(self.completed_scan_positions)

                self.goal = None

                # Check whether the map has grown enough to warrant a replan
                if self.map_dirty:
                    self._info(
                        'Map has grown since last plan — replanning coverage.')
                    self.map_dirty = False
                    self.state = State.PLAN_COVERAGE
                    return

                if self.scan_queue:
                    self._set_next_goal(now)
                else:
                    self._info('Scan queue empty. Going home.')
                    self.state = State.DONE

        # ── DONE ──────────────────────────────────────────────────────────────
        elif self.state == State.DONE:
            if self.start_pos is None:
                self.state = State.LANDING
                return

            home = (self.start_pos[0], self.start_pos[1])
            dist = math.hypot(
                self.position[0] - home[0],
                self.position[1] - home[1])

            if dist < GOAL_REACHED_DIST:
                self._info('Already at home. Landing.')
                self.state = State.LANDING
                return

            self._info(
                f'Returning home ({home[0]:.3f},{home[1]:.3f}) '
                f'distance={dist:.2f}m')

            self.goal                  = home
            self.navigating_home       = True
            self.waypoints             = []
            self.current_wp            = None
            self.needs_replan          = False
            self.last_replan_time      = 0.0
            self.last_replan_pos       = None
            self.goal_start_pos        = (self.position[0], self.position[1])
            self.goal_start_time       = now
            self.last_progress_pos     = (self.position[0], self.position[1])
            self.last_progress_time    = now
            self.stuck_events_for_goal = 0
            self.replan_count_for_goal = 0

            self._plan_path_to_goal()
            if not self.waypoints:
                self._warn('No path home found. Landing here.')
                self.state = State.LANDING
                return

            self._publish_waypoint_markers(self.waypoints)
            self._info(f'{len(self.waypoints)} waypoints to home. Navigating.')
            self.state = State.NAVIGATE

        # ── LANDING ───────────────────────────────────────────────────────────
        elif self.state == State.LANDING:
            self._publish_vel(z=-0.2)
            if self.position[2] < 0.1:
                self.timer.cancel()
                self._publish_vel()
                elapsed = (0.0 if self.exploration_start_time is None
                           else now - self.exploration_start_time)
                self._info(
                    f'Landed at ({self.position[0]:.3f},{self.position[1]:.3f}). '
                    f'Total time: {elapsed:.1f}s. '
                    f'Scan points completed: {len(self.completed_scan_positions)}.')

    # ══════════════════════════════════════════════════════════════════════════
    # Navigation helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _on_goal_reached(self, now):
        """Called when _follow_waypoints signals the goal has been reached."""
        if self.navigating_home:
            self.navigating_home = False
            self._info('Home reached. Landing.')
            self.state = State.LANDING
            return

        self._info(
            f'Arrived at scan point '
            f'({self.goal[0]:.2f},{self.goal[1]:.2f}). '
            f'Starting 360 spin.')
        self.spin_total_rotation = 0.0
        self._last_spin_yaw      = self.angles[2]
        self.state = State.SPINNING

    def _set_next_goal(self, now):
        """Pop the next scan point, reset navigation state, plan A* path."""
        wx, wy = self.scan_queue.pop(0)
        self.goal                  = (wx, wy)
        self.waypoints             = []
        self.current_wp            = None
        self.needs_replan          = False
        self.last_replan_time      = 0.0
        self.last_replan_pos       = None
        self.goal_start_pos        = (self.position[0], self.position[1])
        self.goal_start_time       = now
        self.last_progress_pos     = (self.position[0], self.position[1])
        self.last_progress_time    = now
        self.stuck_events_for_goal = 0
        self.replan_count_for_goal = 0
        self.exploration_start_time = (self.exploration_start_time or now)

        self._info(
            f'Next scan point: ({wx:.2f},{wy:.2f}). '
            f'{len(self.scan_queue)} remaining in queue.')
        self._publish_goal_marker(wx, wy)
        self._plan_path_to_goal()

        if not self.waypoints:
            self._warn(
                f'No A* path to ({wx:.2f},{wy:.2f}). '
                f'Skipping this scan point.')
            if self.scan_queue:
                self._set_next_goal(now)
            else:
                self._info('No reachable scan points remain. Going home.')
                self.goal  = None
                self.state = State.DONE
            return

        self._publish_waypoint_markers(self.waypoints)
        self.state = State.NAVIGATE

    def _goal_health(self, now):
        """Stuck detection and replan-count gating."""
        if self._is_stuck(now):
            self.stuck_events_for_goal += 1
            self._warn(
                f'No progress for {STUCK_TIMEOUT:.0f}s '
                f'(event {self.stuck_events_for_goal}/{MAX_STUCK_EVENTS_PER_GOAL})')
            if self.stuck_events_for_goal >= MAX_STUCK_EVENTS_PER_GOAL:
                return GoalHealth.ABANDON
            self.needs_replan = True

        if self.needs_replan:
            self.replan_count_for_goal += 1
            if self.replan_count_for_goal > MAX_REPLANS_PER_GOAL:
                self._warn(
                    f'Replanned {self.replan_count_for_goal} times without '
                    f'progress. Abandoning goal.')
                return GoalHealth.ABANDON
            cg = self._world_to_grid(self.position[0], self.position[1])
            if (self.last_replan_pos is not None and
                    abs(cg[0] - self.last_replan_pos[0]) <= 1 and
                    abs(cg[1] - self.last_replan_pos[1]) <= 1):
                self.last_replan_pos = None
                self._warn('Replanned but drone has not moved. Abandoning.')
                return GoalHealth.ABANDON
            self.last_replan_pos = cg
            return GoalHealth.REPLAN

        return GoalHealth.HEALTHY

    def _execute_replan(self, now):
        if (now - self.last_replan_time) < REPLAN_COOLDOWN:
            return
        self.needs_replan     = False
        self.last_replan_time = now
        self._plan_path_to_goal()
        if not self.waypoints:
            self._warn('Replan failed. Abandoning goal.')
            self.goal  = None
            self.state = State.PLAN_COVERAGE
        else:
            self._publish_waypoint_markers(self.waypoints)

    def _is_stuck(self, now):
        if self.last_progress_pos is None:
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = now
            return False
        moved = math.hypot(
            self.position[0] - self.last_progress_pos[0],
            self.position[1] - self.last_progress_pos[1])
        if moved > STUCK_PROGRESS_DIST:
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = now
            return False
        return (now - self.last_progress_time) > STUCK_TIMEOUT

    def _follow_waypoints(self, now):
        """
        Step toward self.current_wp. Pop from self.waypoints when close enough.
        Returns True when the final goal is reached.
        Reused verbatim from frontier_exploration_multiranger.py.
        """
        if self.current_wp is not None:
            dx, dy     = (self.current_wp[0] - self.position[0],
                          self.current_wp[1] - self.position[1])
            dist_to_wp = math.hypot(dx, dy)
            if dist_to_wp < GOAL_REACHED_DIST:
                if self.waypoints:
                    self.current_wp = self.waypoints.pop(0)
                    self.replan_count_for_goal = 0
                else:
                    self.current_wp = None
                    return True
        elif self.waypoints:
            self.current_wp = self.waypoints.pop(0)

        if self.current_wp is not None:
            dx, dy     = (self.current_wp[0] - self.position[0],
                          self.current_wp[1] - self.position[1])
            dist_to_wp = math.hypot(dx, dy)
            yaw        = self.angles[2]

            vx_b =  math.cos(yaw) * dx + math.sin(yaw) * dy
            vy_b = -math.sin(yaw) * dx + math.cos(yaw) * dy

            if self.waypoints:
                speed = CRUISE_SPEED / max(dist_to_wp, 1e-3)
            else:
                speed = min(CRUISE_SPEED, dist_to_wp) / max(dist_to_wp, 1e-3)

            vx = max(-CRUISE_SPEED, min(CRUISE_SPEED, vx_b * speed))
            vy = max(-CRUISE_SPEED, min(CRUISE_SPEED, vy_b * speed))

            ye = self._wrap_angle(math.atan2(dy, dx) - yaw)
            wz = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, 2.0 * ye))

            self._publish_vel(x=vx, y=vy, wz=wz)
        else:
            self._publish_vel()

        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Coverage planning
    # ══════════════════════════════════════════════════════════════════════════

    def _run_coverage_plan(self, now):
        """
        Recompute the ordered scan queue.

        1. Find all free cells not already in self.scanned_cells.
        2. Sample candidate scan positions on a grid across free space.
        3. Run greedy set cover to find the minimum set of candidates that
           covers all uncovered free cells.
        4. Order the selected candidates with a nearest-neighbour tour seeded
           from the drone's current position.
        5. Record free_cells_at_last_plan for future dirty checks.
        """
        W, H = self.map_width, self.map_height

        # Step 1: uncovered free cells
        free_indices = np.where(self.map_data == 0)[0]
        uncovered = set()
        for idx in free_indices:
            r, c = divmod(int(idx), W)
            if (r, c) not in self.scanned_cells:
                uncovered.add((r, c))

        self.free_cells_at_last_plan = len(free_indices)
        self.map_dirty = False

        if not uncovered:
            self.scan_queue = []
            return

        # Step 2: candidate positions sampled on a grid over free cells
        step = max(1, SAMPLE_STEP_CELLS)
        candidates = []
        for r in range(0, H, step):
            for c in range(0, W, step):
                if self.map_data[r * W + c] == 0:
                    candidates.append((r, c))

        if not candidates:
            self.scan_queue = []
            return

        # Step 3: greedy set cover
        # Precompute coverage sets: for each candidate, the set of uncovered
        # free cells it covers (within SCAN_RADIUS_CELLS).
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

        # Build coverage map for all candidates up front
        cov_map = {(cr, cc): coverage_set(cr, cc) for cr, cc in candidates}

        remaining   = set(uncovered)
        selected    = []

        while remaining:
            # Pick the candidate that covers the most remaining cells.
            # On equal coverage prefer the one closer to the current position
            # to keep the drone's path short as the plan is built.
            best_cand  = None
            best_count = 0
            px, py     = self.position[0], self.position[1]

            for cand in candidates:
                if cand not in cov_map:
                    continue
                covered = cov_map[cand] & remaining
                n       = len(covered)
                if n == 0:
                    cov_map.pop(cand)   # candidate is exhausted — drop it
                    continue
                if n > best_count:
                    best_count = n
                    best_cand  = cand
                elif n == best_count and best_cand is not None:
                    # Tiebreak on distance to drone
                    wx_b, wy_b = self._grid_to_world(best_cand[0], best_cand[1])
                    wx_c, wy_c = self._grid_to_world(cand[0], cand[1])
                    if (math.hypot(wx_c - px, wy_c - py) <
                            math.hypot(wx_b - px, wy_b - py)):
                        best_cand = cand

            if best_cand is None:
                break   # no candidate covers any remaining cell

            selected.append(best_cand)
            remaining -= cov_map[best_cand]
            # Invalidate coverage sets for remaining candidates so the
            # intersection is recomputed against the shrunk remaining set
            # on the next iteration.  Dropping them from cov_map forces
            # recomputation lazily at next selection.
            cov_map = {k: v for k, v in cov_map.items() if k != best_cand}

        if not selected:
            self.scan_queue = []
            return

        # Step 4: nearest-neighbour tour from current drone position
        ordered = self._nearest_neighbour_tour(selected)

        # Convert grid cells to world coordinates
        self.scan_queue = [
            self._grid_to_world(r, c) for r, c in ordered
        ]

        self._info(
            f'Coverage plan: {len(self.scan_queue)} scan points for '
            f'{len(uncovered)} uncovered free cells '
            f'(scanned_cells={len(self.scanned_cells)}).')

    def _nearest_neighbour_tour(self, cells):
        """
        Greedy nearest-neighbour ordering starting from the drone's position.
        Minimises total travel distance without solving TSP exactly.
        """
        px, py    = self.position[0], self.position[1]
        remaining = list(cells)
        ordered   = []

        cx, cy = px, py
        while remaining:
            best_idx  = 0
            best_dist = float('inf')
            for i, (r, c) in enumerate(remaining):
                wx, wy = self._grid_to_world(r, c)
                d      = math.hypot(wx - cx, wy - cy)
                if d < best_dist:
                    best_dist = d
                    best_idx  = i
            chosen = remaining.pop(best_idx)
            ordered.append(chosen)
            cx, cy = self._grid_to_world(chosen[0], chosen[1])

        return ordered

    def _mark_scanned(self, wx, wy):
        """
        Mark all free cells within SCAN_RADIUS of (wx, wy) as scanned.
        Called once per completed spin.
        """
        W, H  = self.map_width, self.map_height
        cr, cc = self._world_to_grid(wx, wy)
        sr2   = SCAN_RADIUS_CELLS ** 2

        for dr in range(-SCAN_RADIUS_CELLS, SCAN_RADIUS_CELLS + 1):
            nr = cr + dr
            if nr < 0 or nr >= H:
                continue
            max_dc = int(math.sqrt(sr2 - dr * dr))
            for dc in range(-max_dc, max_dc + 1):
                nc = cc + dc
                if 0 <= nc < W and self.map_data[nr * W + nc] == 0:
                    self.scanned_cells.add((nr, nc))

        self._info(
            f'Marked scanned cells around ({wx:.2f},{wy:.2f}). '
            f'Total scanned cells: {len(self.scanned_cells)}.')

    # ══════════════════════════════════════════════════════════════════════════
    # A* pathfinding — reused from frontier_exploration_multiranger.py
    # ══════════════════════════════════════════════════════════════════════════

    def _world_to_grid(self, wx, wy):
        return (int((wy - self.map_origin[1]) / MAP_RES),
                int((wx - self.map_origin[0]) / MAP_RES))

    def _grid_to_world(self, row, col):
        return (self.map_origin[0] + (col + 0.5) * MAP_RES,
                self.map_origin[1] + (row + 0.5) * MAP_RES)

    def _build_inflated_map(self, inflation):
        """
        Build a boolean passability array for A*.

        Unknown cells (-1) are treated as FREE here (unlike the frontier
        explorer) because this node operates on a pre-built or live-updating
        map where unknown space is navigable passageway, not hidden walls.
        Only cells explicitly marked 100 (occupied) are blocked.
        """
        W, H     = self.map_width, self.map_height
        passable = np.ones(W * H, dtype=bool)
        passable[self.map_data == 100] = False

        if inflation > 0:
            for idx in np.where(self.map_data == 100)[0]:
                r, c = divmod(int(idx), W)
                r0 = max(0, r - inflation)
                r1 = min(H - 1, r + inflation)
                c0 = max(0, c - inflation)
                c1 = min(W - 1, c + inflation)
                for rr in range(r0, r1 + 1):
                    passable[rr * W + c0: rr * W + c1 + 1] = False

        return passable

    def _build_proximity_cost(self, radius=None):
        """
        BFS proximity cost outward from occupied cells only.
        Cells near walls get a cost penalty so A* routes down corridor centres.
        """
        if radius is None:
            radius = PROXIMITY_COST_RADIUS
        W, H     = self.map_width, self.map_height
        dist_arr = np.full(W * H, -1, dtype=np.int32)
        mask     = (self.map_data == 100)
        dist_arr[mask] = 0

        queue = deque(zip(*np.where(mask.reshape(H, W))))
        dirs4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while queue:
            row, col = queue.popleft()
            d = dist_arr[row * W + col]
            if d >= radius:
                continue
            for dr, dc in dirs4:
                nr, nc = row + dr, col + dc
                if 0 <= nr < H and 0 <= nc < W:
                    idx2 = nr * W + nc
                    if dist_arr[idx2] == -1:
                        dist_arr[idx2] = d + 1
                        queue.append((nr, nc))

        cost      = np.zeros(W * H, dtype=np.float32)
        reachable = dist_arr > 0
        cost[reachable] = PROXIMITY_COST_WEIGHT * np.maximum(
            0.0, 1.0 - dist_arr[reachable] / radius)
        return cost

    def _plan_path_to_goal(self):
        """
        Run A* from current position to self.goal.
        Populates self.waypoints on success.
        No STANDOFF trimming — we want exact arrival at scan points.
        Reused from frontier_exploration_multiranger.py with minor changes.
        """
        self.waypoints  = []
        self.current_wp = None
        self.needs_replan = False

        if self.map_data is None or self.goal is None:
            self._warn('A*: no map or goal')
            return

        sr, sc = self._world_to_grid(self.position[0], self.position[1])
        gr, gc = self._world_to_grid(self.goal[0],     self.goal[1])

        sr = max(0, min(self.map_height - 1, sr))
        sc = max(0, min(self.map_width  - 1, sc))
        gr = max(0, min(self.map_height - 1, gr))
        gc = max(0, min(self.map_width  - 1, gc))

        nb8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

        found     = False
        path_cost = None

        for inflation in range(WALL_INFLATION_CELLS, 0, -1):
            passable = self._build_inflated_map(inflation)
            W, H     = self.map_width, self.map_height

            # Force start and goal cells passable so A* can always begin
            # and terminate even when inside an inflation zone.
            if 0 <= sr < H and 0 <= sc < W:
                passable[sr * W + sc] = True
            if 0 <= gr < H and 0 <= gc < W:
                passable[gr * W + gc] = True

            effective_radius = max(1, PROXIMITY_COST_RADIUS - inflation)
            prox_cost = self._build_proximity_cost(effective_radius)

            open_heap = [(0.0, 0.0, sr, sc)]
            came_from = {}
            g_score   = {(sr, sc): 0.0}
            found     = False

            while open_heap:
                _, g, row, col = heapq.heappop(open_heap)
                if (row, col) == (gr, gc):
                    found = True
                    path_cost = g
                    break
                if g > g_score.get((row, col), float('inf')):
                    continue
                for dr, dc, move_cost in nb8:
                    nr, nc = row + dr, col + dc
                    if not (0 <= nr < H and 0 <= nc < W):
                        continue
                    if not passable[nr * W + nc]:
                        continue
                    ng = g + move_cost + prox_cost[nr * W + nc]
                    if ng < g_score.get((nr, nc), float('inf')):
                        g_score[(nr, nc)]   = ng
                        came_from[(nr, nc)] = (row, col)
                        h = math.hypot(nr - gr, nc - gc)
                        heapq.heappush(open_heap, (ng + h, ng, nr, nc))

            if found:
                break

        if not found:
            self._warn(
                f'A*: no path to '
                f'({self.goal[0]:.2f},{self.goal[1]:.2f})')
            return

        path = []
        cell = (gr, gc)
        while cell in came_from:
            path.append(cell)
            cell = came_from[cell]
        path.append((sr, sc))
        path.reverse()

        # No standoff trimming — arrive exactly at the scan point centre
        wps = [self._grid_to_world(r, c)
               for i, (r, c) in enumerate(path)
               if i % WAYPOINT_SPACING == 0 or i == len(path) - 1]

        self.waypoints    = wps
        self.last_path_cost = path_cost
        self._info(
            f'A*: {len(path)} cells → {len(wps)} waypoints '
            f'cost={path_cost:.2f}')

    # ══════════════════════════════════════════════════════════════════════════
    # Utility
    # ══════════════════════════════════════════════════════════════════════════

    def _wrap_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # ══════════════════════════════════════════════════════════════════════════
    # Publishers
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_vel(self, x=0.0, y=0.0, z=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = float(x)
        msg.linear.y  = float(y)
        msg.linear.z  = float(z)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _publish_goal_marker(self, wx, wy):
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'scan_goal'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = float(wx)
        m.pose.position.y    = float(wy)
        m.pose.position.z    = float(TAKEOFF_HEIGHT)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 1.0
        self.goal_marker_pub.publish(m)

    def _publish_waypoint_markers(self, waypoints):
        arr = MarkerArray()

        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns              = 'waypoints'
        clear.id              = 0
        clear.action          = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (wx, wy) in enumerate(waypoints):
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'waypoints'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = float(TAKEOFF_HEIGHT)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            arr.markers.append(m)

        self.waypoint_marker_pub.publish(arr)

    def _publish_scan_point_markers(self, scan_queue):
        """Publish all planned scan points as yellow spheres."""
        arr = MarkerArray()

        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns              = 'scan_points'
        clear.id              = 0
        clear.action          = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (wx, wy) in enumerate(scan_queue):
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'scan_points'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = float(TAKEOFF_HEIGHT)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.20
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.8
            arr.markers.append(m)

        self.scan_point_marker_pub.publish(arr)

    def _publish_scanned_markers(self, positions):
        """Publish completed scan positions as grey spheres."""
        arr = MarkerArray()

        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns              = 'scanned'
        clear.id              = 0
        clear.action          = Marker.DELETEALL
        arr.markers.append(clear)

        for i, (wx, wy) in enumerate(positions):
            m = Marker()
            m.header.frame_id    = 'map'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'scanned'
            m.id                 = i + 1
            m.type               = Marker.SPHERE
            m.action             = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = float(TAKEOFF_HEIGHT)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.6, 0.6, 0.7
            arr.markers.append(m)

        self.scanned_marker_pub.publish(arr)

    def _publish_drone_marker(self):
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'drone'
        m.id                 = 0
        m.type               = Marker.ARROW
        m.action             = Marker.ADD
        m.pose.position.x    = float(self.position[0])
        m.pose.position.y    = float(self.position[1])
        m.pose.position.z    = float(TAKEOFF_HEIGHT)
        yaw = self.angles[2]
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x = 0.30
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 1.0
        self.drone_marker_pub.publish(m)

    def _log_status(self, now):
        if (now - self.last_status_log_time) < self.status_log_period:
            return
        self.last_status_log_time = now
        self._publish_drone_marker()

        goal_s = 'none'
        if self.goal is not None:
            d = math.hypot(
                self.goal[0] - self.position[0],
                self.goal[1] - self.position[1])
            goal_s = f'({self.goal[0]:.2f},{self.goal[1]:.2f}) {d:.2f}m away'

        elapsed = (0.0 if self.exploration_start_time is None
                   else now - self.exploration_start_time)
        self._info(
            f'pos=({self.position[0]:.2f},{self.position[1]:.2f}) '
            f'goal={goal_s} '
            f'queue={len(self.scan_queue)} '
            f'done={len(self.completed_scan_positions)} '
            f'scanned_cells={len(self.scanned_cells)} '
            f't={elapsed:.1f}s')


def main(args=None):
    rclpy.init(args=args)
    node = CoverageScanner()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()