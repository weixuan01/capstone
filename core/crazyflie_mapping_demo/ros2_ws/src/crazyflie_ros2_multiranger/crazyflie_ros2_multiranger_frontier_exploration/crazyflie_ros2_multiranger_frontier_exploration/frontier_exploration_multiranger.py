#!/usr/bin/env python3

"""
Autonomous Frontier-Based Explorer for Crazyflie
=================================================
State machine:
  1. TAKEOFF        - take off and hover
  2. SPINNING       - initial 360° scan
  3. FIND_FRONTIER  - score frontiers, pick best goal
  4. NAVIGATE       - follow A* waypoints to goal
  5. STEER_DIRECT   - direct steering fallback when A* fails
  6. WALL_AVOID     - reactive wall avoidance
  7. DONE           - exploration complete → return to (0,0) via A* → land
  8. LANDING        - descend and stop

Efficiency (v2): [1-10]
Coverage  (v3): [11-18]
Branching (v4): [19-26]
Stagnation detector (v6): [27-34]

Consecutive-stagnation gate (v7):
  Root cause of v6 failure (visible in logs):
    Stagnation fires correctly (Δ=+0/12s), branch recovery triggers, drone
    travels to the junction — but the whole map is already scanned so unknowns
    STILL don't drop after arrival.  Stagnation fires again → another branch
    point → repeat × 8.  The drone cycles forever because:
      a) Branch points are never consumed (removed) after being visited.
      b) There is no limit on how many failed recoveries are allowed.

  Fix [35-38]:
  [35] MAX_CONSECUTIVE_STAGNATIONS — after this many stagnation events in a row
       that do NOT result in renewed map progress, declare exploration done
       regardless of remaining branch points.  Default = 2:
         • 1st stagnation: maybe just a dead-end corner → try one branch visit.
         • 2nd stagnation: branch visit didn't help → map is fully covered → DONE.
       For a single-arm map (no junctions) this means exactly 1 false alarm then
       DONE.  For a two-arm map: arm-1 stagnation → branch visit → unknowns drop
       (new arm) → counter resets to 0 → arm-2 stagnation → 1 branch visit
       → still no drop → DONE.  The counter RESETS whenever the map is actively
       learning (stagnation check returns False), so a genuine second arm is
       always explored fully before the counter can reach the limit.

  [36] Branch points are now CONSUMED (removed from the deque) at the moment
       `_do_branch_recovery()` dispatches the drone toward them.  This prevents
       re-visiting the same junction repeatedly.  The unknown_history is also
       cleared so the 12-second window restarts fresh from arrival.

  [37] `self.consecutive_stagnations` — integer counter, incremented by
       `_unknown_rate_stagnant()` returning True and reset to 0 when
       `_unknown_rate_stagnant()` returns False (i.e. map is actively shrinking).
       The reset is logged so it is visible in the terminal.

  [38] Stagnation decision tree in FIND_FRONTIER (replaces v6 version):
         stagnant AND consecutive < MAX  AND branch points → branch recovery
         stagnant AND consecutive < MAX  AND no branches   → DONE
         stagnant AND consecutive >= MAX                   → DONE
         not stagnant                                      → reset counter, explore

  Two-route guarantee is preserved:
    After arm 1: stagnant (count=1) → branch recovery → arrive → unknowns drop
    → counter resets to 0 → arm 2 explored fully → stagnant (count=1) →
    branch recovery (if any left) → unknowns don't drop → stagnant (count=2)
    → count >= MAX → DONE ✓

Logging:
  - Stagnation count shown on every stagnation warning
  - Counter reset logged when map progress resumes
  - Δunk/window in every status line and frontier header
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import tf_transformations
import math
import heapq
import numpy as np
from enum import Enum, auto
from collections import deque

# ── Map constants ─────────────────────────────────────────────────────────────
GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES       = 0.1

# ── Flight parameters ─────────────────────────────────────────────────────────
TAKEOFF_HEIGHT         = 0.02
TAKEOFF_DELAY          = 5.0
CRUISE_SPEED           = 0.3
MAX_TURN_RATE          = 0.8
OBSTACLE_DIST          = 0.4
WAYPOINT_REACHED_DIST  = 0.1
FINAL_GOAL_DIST        = 0.15
MIN_FRONTIER_DIST      = 0.5
FRONTIER_STEP          = 2
REPLAN_COOLDOWN        = 1.5
WALL_INFLATION_CELLS   = 1
STEER_DIRECT_WALL_DIST = 0.8

# ── Wall avoidance parameters ─────────────────────────────────────────────────
WALL_AVOID_TIMEOUT   = 1.2
CRITICAL_FRONT_DIST  = 0.22
SIDE_TOO_CLOSE_DIST  = 0.18
AVOID_FORWARD_SPEED  = 0.08
AVOID_LATERAL_SPEED  = 0.18
AVOID_BACKWARD_SPEED = -0.05
POST_AVOID_COOLDOWN  = 0.8

# ── Frontier filtering ────────────────────────────────────────────────────────
MIN_CLUSTER_SIZE       = 1
MIN_VALID_CLUSTER_SIZE = 2

# ── Wall safety and corridor centering ───────────────────────────────────────
# The old version only added a small sideways correction, so path following
# could still dominate and let the drone hug a wall.  This version makes
# centering a higher-priority behaviour:
#   - smooth left/right ranges
#   - push away harder when a wall is too close
#   - centre only when both walls are truly visible
#   - reduce forward speed when off-centre or in tight corridors
WALL_SAFE_DIST          = 0.30
WALL_PUSH_DIST          = 0.38
WALL_CENTRE_MAX_VALID   = 0.95
TIGHT_CORRIDOR_WIDTH    = 0.95
MEDIUM_CORRIDOR_WIDTH   = 1.50
WALL_FILTER_ALPHA       = 0.35
WALL_KP_SAFETY          = 1.35
WALL_KP_CENTRE_TIGHT    = 0.95
WALL_KP_CENTRE_MEDIUM   = 0.65
WALL_KP_CENTRE_WIDE     = 0.35
MAX_LATERAL_SPEED       = 0.24
OFFCENTER_SLOW_BAND     = 0.08
OFFCENTER_HARD_BAND     = 0.16
TIGHT_SPEED_SCALE       = 0.55
MEDIUM_SPEED_SCALE      = 0.72
WIDE_SPEED_SCALE        = 0.88

# ── Initial spin ──────────────────────────────────────────────────────────────
SPIN_RATE = 0.5

# ── Coordinate memory / revisit prevention ───────────────────────────────────
VISITED_CELL_RADIUS = 2
RECENT_GOAL_MEMORY  = 8
FAILED_GOAL_MEMORY  = 10

# ── Utility scoring weights ───────────────────────────────────────────────────
DISTANCE_WEIGHT     = 1.0
SIZE_WEIGHT         = 1.6
UNKNOWN_WEIGHT      = 2.0
VISIT_PENALTY       = 0.9
RECENT_GOAL_PENALTY = 5.0
FAILED_GOAL_PENALTY = 7.0
BLACKLIST_PENALTY   = 20.0

# ── Goal commitment / hysteresis ─────────────────────────────────────────────
GOAL_KEEP_RATIO       = 0.85
GOAL_SWITCH_MIN_DELTA = 2.0

# ── Failed region blacklist ───────────────────────────────────────────────────
FAILED_REGION_RADIUS   = 0.9
FAILED_REGION_COOLDOWN = 45.0

# ── Stuck detector ────────────────────────────────────────────────────────────
STUCK_PROGRESS_DIST       = 0.20
STUCK_TIMEOUT             = 8.0
MAX_STUCK_EVENTS_PER_GOAL = 2

# [1] Promoted from local variable
WAYPOINT_SPACING = 15

# ── Scan coverage tracking [11] ───────────────────────────────────────────────
SENSOR_RANGE_CELLS = 5
COVERAGE_MIN_GAIN  = 0.15

# ── Branch / dead-end awareness [19][20] ─────────────────────────────────────
BRANCH_ANGULAR_SPREAD_DEG = 60.0
BRANCH_DEDUP_RADIUS       = 1.2
MAX_BRANCH_MEMORY         = 8

# ── Reachability BFS [23][25] ─────────────────────────────────────────────────
REACHABILITY_STRIDE         = 3
REACHABILITY_CHECK_INTERVAL = 3.0

# ── Map-stagnation detector [27-30] ──────────────────────────────────────────
# Rolling window over which the drop in unknown cell count is measured.
UNKNOWN_RATE_WINDOW = 12.0          # seconds

# Minimum cells that must disappear within the window to count as "active".
# 30 cells ≈ 0.3 m² at MAP_RES=0.1 m.
UNKNOWN_RATE_MIN_DROP = 30          # cells

# How often a (time, count) sample is appended.
UNKNOWN_SAMPLE_INTERVAL = 2.0       # seconds

# Grace period after spin before stagnation can trigger.
EXPLORATION_MIN_TIME = 25.0         # seconds

# ── Consecutive-stagnation limit [35] ────────────────────────────────────────
# How many stagnation events in a row (with no intervening progress) are
# allowed before the drone declares exploration complete and goes home.
#
# = 2 is the recommended value:
#   • Single-arm map: 1st stagnation → 1 branch attempt → no drop → 2nd
#     stagnation → DONE.
#   • Two-arm map:   1st arm stagnant → branch visit → unknowns drop (new arm
#     entered) → counter resets → 2nd arm stagnant → DONE after ≤2 attempts.
#
# Increase to 3 if your map has many dead-end corridors that look explored
# from afar but still have a thin ring of unknown cells near their tips.
MAX_CONSECUTIVE_STAGNATIONS = 2


class State(Enum):
    TAKEOFF       = auto()
    SPINNING      = auto()
    FIND_FRONTIER = auto()
    NAVIGATE      = auto()
    STEER_DIRECT  = auto()
    WALL_AVOID    = auto()
    DONE          = auto()
    LANDING       = auto()


class FrontierExplorationMultiranger(Node):

    def __init__(self):
        super().__init__('simple_mapper_multiranger')

        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        # ── Internal state ────────────────────────────────────────────────────
        self.state    = State.TAKEOFF
        self.position = [0.0, 0.0, 0.0]
        self.angles   = [0.0, 0.0, 0.0]
        self.ranges   = [0.0, 0.0, 0.0, 0.0]   # back, right, front, left

        self.map_data   = None   # [10] numpy int8
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.goal                = None
        self.goal_score          = None
        self.start_pos           = None
        self.start_time          = None
        self.spin_start_time     = None
        self.spin_start_yaw      = None
        self.spin_total_rotation = 0.0
        self._last_spin_yaw      = 0.0

        # ── A* path following ─────────────────────────────────────────────────
        self.waypoints        = []
        self.current_wp       = None
        self.needs_replan     = False
        self.last_replan_time = 0.0
        self.last_replan_pos  = None
        self.last_path_cost   = None

        # ── Wall avoidance state ──────────────────────────────────────────────
        self.wall_avoid_start_time = None
        self.avoid_turn_dir        = 1.0
        self.avoid_lateral_dir     = 1.0
        self.post_avoid_until      = 0.0

        # ── Coordinate memory ─────────────────────────────────────────────────
        self.filtered_right = None
        self.filtered_left  = None
        self.last_corridor_width = None
        self.last_centering_active = False

        self.visited_counts = {}
        self.recent_goals   = deque(maxlen=RECENT_GOAL_MEMORY)
        self.failed_goals   = deque(maxlen=FAILED_GOAL_MEMORY)
        self.failed_regions = []

        self.scanned_cells = set()                          # [12]
        self.branch_points = deque(maxlen=MAX_BRANCH_MEMORY) # [21]
        self.last_reachability_time = 0.0                   # [25]

        # [31] Unknown-count history: deque of (timestamp, count)
        self.unknown_history     = deque()
        self.last_unknown_sample = 0.0

        # [37] Consecutive stagnation counter.
        # Incremented each time stagnation fires; reset to 0 when map is
        # actively learning (stagnation check returns False).
        self.consecutive_stagnations = 0

        # ── Goal / stuck tracking ─────────────────────────────────────────────
        self.goal_start_pos        = None
        self.goal_start_time       = 0.0
        self.last_progress_pos     = None
        self.last_progress_time    = 0.0
        self.stuck_events_for_goal = 0

        # ── Timer / logging ───────────────────────────────────────────────────
        self.exploration_start_time = None
        self.last_status_log_time   = 0.0
        self.status_log_period      = 1.0

        self.position_received = False
        self.map_received      = False

        # ── ROS subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_callback, 10)
        self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            OccupancyGrid, robot_prefix + '/map', self.map_callback, map_qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(
            Trigger, robot_prefix + '/stop_exploration', self.stop_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self._publish_vel(z=TAKEOFF_HEIGHT)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'Explorer started. prefix={robot_prefix}')

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
            self.start_pos         = [self.position[0], self.position[1]]
            self.position_received = True
            self.last_progress_pos = (self.position[0], self.position[1])
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_progress_time = now
            self.get_logger().info(
                f'Home captured: '
                f'({self.start_pos[0]:.3f},{self.start_pos[1]:.3f})')

        if self.map_received:
            row, col = self._world_to_grid(self.position[0], self.position[1])
            self._mark_visited(row, col)
            self._update_scan_coverage(row, col)

    def scan_callback(self, msg):
        self.ranges = list(msg.ranges)

        right = self.ranges[1] if len(self.ranges) > 1 else 0.0
        left  = self.ranges[3] if len(self.ranges) > 3 else 0.0

        if right > 0.0:
            self.filtered_right = (right if self.filtered_right is None else
                                   WALL_FILTER_ALPHA * right +
                                   (1.0 - WALL_FILTER_ALPHA) * self.filtered_right)
        if left > 0.0:
            self.filtered_left = (left if self.filtered_left is None else
                                  WALL_FILTER_ALPHA * left +
                                  (1.0 - WALL_FILTER_ALPHA) * self.filtered_left)

    def map_callback(self, msg):
        self.map_data   = np.array(msg.data, dtype=np.int8)
        self.map_width  = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y]
        self.map_received = True

    def stop_callback(self, request, response):
        self.get_logger().info('Stop requested — landing now')
        self.timer.cancel()
        self._publish_vel(z=-0.2)
        response.success = True
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # Main state machine
    # ══════════════════════════════════════════════════════════════════════════

    def timer_callback(self):
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._state_machine(now)
        except Exception as e:
            import traceback
            self.get_logger().error(f'Crash: {e}')
            self.get_logger().error(traceback.format_exc())
            self._publish_vel()
            self.timer.cancel()

    def _state_machine(self, now):
        self._log_status(now)
        self._prune_failed_regions(now)

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self.state == State.TAKEOFF:
            self._publish_vel(z=TAKEOFF_HEIGHT)
            if now - self.start_time > TAKEOFF_DELAY:
                self.get_logger().info(
                    'Takeoff done — spinning 360° for initial map')
                self.spin_start_time     = now
                self.spin_start_yaw      = self.angles[2]
                self.spin_total_rotation = 0.0
                self._last_spin_yaw      = self.angles[2]
                self.state = State.SPINNING

        # ── SPINNING ──────────────────────────────────────────────────────────
        elif self.state == State.SPINNING:
            self._publish_vel(wz=SPIN_RATE)
            yaw_delta = self._wrap_angle(
                self.angles[2] - (self.spin_start_yaw
                                  if self.spin_total_rotation == 0.0
                                  else self._last_spin_yaw))
            self._last_spin_yaw       = self.angles[2]
            self.spin_total_rotation += abs(yaw_delta)

            if (now - self.last_status_log_time) >= self.status_log_period:
                self.get_logger().info(
                    f'Spinning '
                    f'{math.degrees(self.spin_total_rotation):.0f}°/360°')

            if self.spin_total_rotation >= 2 * math.pi:
                self._publish_vel()
                self.exploration_start_time = now
                self.get_logger().info('Spin done — starting exploration')
                self.state = State.FIND_FRONTIER

        # ── FIND_FRONTIER ─────────────────────────────────────────────────────
        elif self.state == State.FIND_FRONTIER:
            self._publish_vel(y=self._get_wall_correction())

            if not self.map_received or not self.position_received:
                self.get_logger().info('Waiting for map + pose…')
                return

            # [32] Update unknown history for stagnation window
            self._sample_unknown_history(now)

            # ── [38] STAGNATION DECISION TREE ─────────────────────────────────
            stagnant = self._unknown_rate_stagnant(now)

            if stagnant:
                # Increment consecutive counter [37]
                self.consecutive_stagnations += 1
                self.get_logger().warn(
                    f'Stagnation #{self.consecutive_stagnations}/'
                    f'{MAX_CONSECUTIVE_STAGNATIONS}')

                if self.consecutive_stagnations >= MAX_CONSECUTIVE_STAGNATIONS:
                    # [35] Hard limit reached — done regardless of branch points.
                    elapsed = self._elapsed_exploration_time(now)
                    self.get_logger().info(
                        f'Max consecutive stagnations '
                        f'({MAX_CONSECUTIVE_STAGNATIONS}) reached — '
                        f'exploration complete. t={elapsed:.1f}s → going home')
                    self.state = State.DONE
                    return

                if self.branch_points:
                    # [36] Branch recovery: consume the point, clear history,
                    # dispatch toward junction.
                    self._do_branch_recovery(now)
                    return
                else:
                    # No branches and under limit — declare done.
                    elapsed = self._elapsed_exploration_time(now)
                    self.get_logger().info(
                        f'Stagnant with no branch points — done. '
                        f't={elapsed:.1f}s → going home')
                    self.state = State.DONE
                    return

            else:
                # Map is actively learning — reset counter [37]
                if self.consecutive_stagnations > 0:
                    self.get_logger().info(
                        f'Map progress resumed — resetting stagnation '
                        f'counter (was {self.consecutive_stagnations})')
                    self.consecutive_stagnations = 0

            # ── Record junction if multiple directions visible [22] ────────────
            clusters = self._get_frontier_clusters(now)
            if len(clusters) >= 2:
                self._record_branch_point(clusters)

            # ── Frontier table log ─────────────────────────────────────────────
            unk, drop = self._unknown_window_stats()
            self.get_logger().info(
                f'--- Frontier scan: {len(clusters)} clusters | '
                f'unk={unk} Δ={drop:+d}/{UNKNOWN_RATE_WINDOW:.0f}s | '
                f'scanned={len(self.scanned_cells)} | '
                f'branches={len(self.branch_points)} | '
                f'stag={self.consecutive_stagnations}/'
                f'{MAX_CONSECUTIVE_STAGNATIONS} ---')
            for i, c in enumerate(clusters[:8]):
                cx, cy, dist, size, new_unk, total_unk, pen, score, pcost = c
                cov = 100.0 * (1.0 - new_unk / max(1.0, total_unk))
                mark = (' <- CURRENT'
                        if self.goal is not None and
                        math.hypot(cx-self.goal[0], cy-self.goal[1]) < 0.2
                        else (' <- BEST' if i == 0 else ''))
                self.get_logger().info(
                    f'  [{i+1}] ({cx:.2f},{cy:.2f}) '
                    f'dist={dist:.2f}m path={pcost:.2f} size={size} '
                    f'new_unk={new_unk:.1f} cov={cov:.0f}% '
                    f'pen={pen:.1f} score={score:.2f}{mark}')

            # No clusters → secondary BFS fallback
            if not clusters:
                self._handle_no_clusters(now)
                return

            # ── Normal goal selection ─────────────────────────────────────────
            best      = clusters[0]
            new_goal  = (best[0], best[1])
            new_score = best[7]

            keep_current = False
            if self.goal is not None and self.goal_score is not None:
                match = self._find_matching_cluster(self.goal, clusters)
                if match is not None:
                    cs = match[7]
                    if (cs >= GOAL_KEEP_RATIO * new_score or
                            (new_score - cs) < GOAL_SWITCH_MIN_DELTA):
                        keep_current = True
                        new_goal, new_score = self.goal, cs
                        self.get_logger().info(
                            f'Hysteresis: keep '
                            f'({self.goal[0]:.2f},{self.goal[1]:.2f}) '
                            f'cur={cs:.2f} best={clusters[0][7]:.2f}')

            if not keep_current:
                self.goal                  = new_goal
                self.goal_score            = new_score
                self.recent_goals.append(self.goal)
                self.goal_start_pos        = (self.position[0], self.position[1])
                self.goal_start_time       = now
                self.last_progress_pos     = (self.position[0], self.position[1])
                self.last_progress_time    = now
                self.stuck_events_for_goal = 0
                self.get_logger().info(
                    f'Goal → ({self.goal[0]:.2f},{self.goal[1]:.2f}) '
                    f'score={self.goal_score:.2f}')
            else:
                self.get_logger().info(
                    f'Continuing ({self.goal[0]:.2f},{self.goal[1]:.2f}) '
                    f'score={self.goal_score:.2f}')

            self._plan_path_to_goal()
            if not self.waypoints:
                self._mark_goal_failed(now, self.goal, 'A* no path FIND_FRONTIER')
                self.get_logger().warn('No A* path — direct steer')
                self.state = State.STEER_DIRECT
            else:
                self.state = State.NAVIGATE

        # ── NAVIGATE ──────────────────────────────────────────────────────────
        elif self.state == State.NAVIGATE:
            if self.goal is None:
                self.state = State.FIND_FRONTIER
                self._publish_vel(y=self._get_wall_correction())
                return

            if self._is_stuck(now):
                self.stuck_events_for_goal += 1
                self.get_logger().warn(
                    f'Stuck NAVIGATE #{self.stuck_events_for_goal}')
                if self.stuck_events_for_goal >= MAX_STUCK_EVENTS_PER_GOAL:
                    self._mark_goal_failed(now, self.goal, 'Stuck NAVIGATE')
                    self._abandon_current_goal()
                    return
                self._start_wall_avoid(now)
                return

            if now >= self.post_avoid_until and self._collision_risk_detected():
                self.get_logger().warn('Collision risk — wall avoid')
                self._start_wall_avoid(now)
                return

            front = self._front_range()
            if 0.0 < front < OBSTACLE_DIST:
                if (not self.needs_replan and
                        (now - self.last_replan_time) > REPLAN_COOLDOWN):
                    self.needs_replan = True

            if self.needs_replan:
                self.needs_replan     = False
                self.last_replan_time = now
                cg = self._world_to_grid(self.position[0], self.position[1])
                if self.last_replan_pos is not None:
                    if (abs(cg[0]-self.last_replan_pos[0]) <= 1 and
                            abs(cg[1]-self.last_replan_pos[1]) <= 1):
                        self.last_replan_pos = None
                        self._start_wall_avoid(now)
                        return
                self.last_replan_pos = cg
                self._plan_path_to_goal()
                if not self.waypoints:
                    self._mark_goal_failed(
                        now, self.goal, 'A* no path NAVIGATE replan')
                    self.state = State.STEER_DIRECT
                    return

            if not self.waypoints and self.current_wp is None:
                self.goal = self.goal_score = None
                self.state = State.FIND_FRONTIER
                self._publish_vel(y=self._get_wall_correction())
                return

            if self._follow_waypoints(now):
                self.goal = self.goal_score = None
                self.state = State.FIND_FRONTIER
                self._publish_vel(y=self._get_wall_correction())

        # ── STEER_DIRECT ──────────────────────────────────────────────────────
        elif self.state == State.STEER_DIRECT:
            if self.goal is None:
                self.state = State.FIND_FRONTIER
                return

            if self._is_stuck(now):
                self.stuck_events_for_goal += 1
                self.get_logger().warn(
                    f'Stuck STEER_DIRECT #{self.stuck_events_for_goal}')
                if self.stuck_events_for_goal >= MAX_STUCK_EVENTS_PER_GOAL:
                    self._mark_goal_failed(
                        now, self.goal, 'Stuck STEER_DIRECT')
                    self._abandon_current_goal()
                    return
                self._start_wall_avoid(now)
                return

            if now >= self.post_avoid_until and self._collision_risk_detected():
                self._start_wall_avoid(now)
                return

            if (now - self.last_replan_time) > REPLAN_COOLDOWN:
                self.last_replan_time = now
                self._plan_path_to_goal()
                if self.waypoints:
                    self.state = State.NAVIGATE
                    return

            dx, dy = (self.goal[0]-self.position[0],
                      self.goal[1]-self.position[1])
            dist = math.hypot(dx, dy)
            if dist < FINAL_GOAL_DIST:
                self.goal = self.goal_score = None
                self.state = State.FIND_FRONTIER
                return

            ye = self._wrap_angle(math.atan2(dy, dx) - self.angles[2])
            wall_y, speed_scale = self._get_wall_guidance()
            vx = min(CRUISE_SPEED * max(0.0, math.cos(ye)) * speed_scale, dist)
            if abs(wall_y) > OFFCENTER_HARD_BAND:
                vx *= 0.65
            wz = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, 3.0 * ye))
            if (now - self.last_status_log_time) >= self.status_log_period:
                self.get_logger().info(
                    f'Direct steer dist={dist:.2f}m wall_y={wall_y:.2f} speed={speed_scale:.2f}')
            self._publish_vel(
                x=vx,
                y=wall_y,
                wz=wz)

        # ── WALL_AVOID ────────────────────────────────────────────────────────
        elif self.state == State.WALL_AVOID:
            if self.goal is None:
                self.state = State.FIND_FRONTIER
                return

            elapsed = now - self.wall_avoid_start_time
            if (now - self.last_status_log_time) >= self.status_log_period:
                self.get_logger().info(
                    f'Wall avoid t={elapsed:.1f}/{WALL_AVOID_TIMEOUT:.1f}s '
                    f'F={self._front_range():.2f} '
                    f'R={self._right_range():.2f} L={self._left_range():.2f}')

            if elapsed > WALL_AVOID_TIMEOUT:
                self.post_avoid_until = now + POST_AVOID_COOLDOWN
                self.current_wp = None
                self.waypoints  = []
                self.state      = State.FIND_FRONTIER
                return

            if self._front_range() < CRITICAL_FRONT_DIST:
                vx = AVOID_BACKWARD_SPEED
                vy = self.avoid_lateral_dir * AVOID_LATERAL_SPEED
                wz = self.avoid_turn_dir * MAX_TURN_RATE
            else:
                vx = AVOID_FORWARD_SPEED
                vy = self.avoid_lateral_dir * AVOID_LATERAL_SPEED
                wz = self.avoid_turn_dir * 0.7 * MAX_TURN_RATE
            self._publish_vel(x=vx, y=vy, wz=wz)

        # ── DONE — return to (0,0) via A* ─────────────────────────────────────
        elif self.state == State.DONE:
            if self.start_pos is None:
                self.state = State.LANDING
                return

            if self.goal is None:
                home = (self.start_pos[0], self.start_pos[1])
                self.goal = home
                self.get_logger().info(
                    f'Returning home ({home[0]:.3f},{home[1]:.3f}) via A*')
                self._plan_path_to_goal()
                if not self.waypoints:
                    self.get_logger().warn('No A* path home — flying direct')
                    self.waypoints = [self.goal]

            if not self.waypoints and self.current_wp is None:
                self.goal  = None
                self.state = State.LANDING
                return

            if self._follow_waypoints(now):
                self.get_logger().info('Home reached — landing')
                self.goal  = None
                self.state = State.LANDING

        # ── LANDING ───────────────────────────────────────────────────────────
        elif self.state == State.LANDING:
            self._publish_vel(z=-0.2)
            if self.position[2] < 0.1:
                elapsed = self._elapsed_exploration_time(now)
                self.timer.cancel()
                self._publish_vel()
                self.get_logger().info(
                    f'Landed at '
                    f'({self.position[0]:.3f},{self.position[1]:.3f}). '
                    f'Total time={elapsed:.1f}s')

    # ══════════════════════════════════════════════════════════════════════════
    # Stagnation detector  [32][33]
    # ══════════════════════════════════════════════════════════════════════════

    def _sample_unknown_history(self, now):
        """
        [32] Append a (timestamp, unknown_count) sample throttled to
        UNKNOWN_SAMPLE_INTERVAL; prune entries older than UNKNOWN_RATE_WINDOW.
        """
        if (now - self.last_unknown_sample) < UNKNOWN_SAMPLE_INTERVAL:
            return
        self.last_unknown_sample = now
        count = (int(np.sum(self.map_data == -1))
                 if self.map_data is not None else 0)
        self.unknown_history.append((now, count))
        while (self.unknown_history and
               now - self.unknown_history[0][0] > UNKNOWN_RATE_WINDOW):
            self.unknown_history.popleft()

    def _unknown_window_stats(self):
        """Returns (latest_count, drop_over_window) for logging."""
        if not self.unknown_history:
            return 0, 0
        return (self.unknown_history[-1][1],
                self.unknown_history[0][1] - self.unknown_history[-1][1])

    def _unknown_rate_stagnant(self, now):
        """
        [33] Returns True when map learning has effectively stopped.
        Grace period + half-window fill guard prevent false positives.
        """
        if self._elapsed_exploration_time(now) < EXPLORATION_MIN_TIME:
            return False
        if len(self.unknown_history) < 3:
            return False
        oldest_t, oldest_count = self.unknown_history[0]
        newest_t, newest_count = self.unknown_history[-1]
        if (newest_t - oldest_t) < UNKNOWN_RATE_WINDOW * 0.5:
            return False
        drop = oldest_count - newest_count
        if drop < UNKNOWN_RATE_MIN_DROP:
            self.get_logger().warn(
                f'Stagnation: unknowns changed by {drop} cells over '
                f'{newest_t-oldest_t:.1f}s (threshold={UNKNOWN_RATE_MIN_DROP})')
            return True
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Branch recovery  [34][36]
    # ══════════════════════════════════════════════════════════════════════════

    def _do_branch_recovery(self, now):
        """
        [34][36] Navigate to the nearest branch point and CONSUME it (remove
        from deque) so it is never re-visited.  Clears unknown_history so the
        12-second window restarts fresh after arrival at the junction — if the
        new arm has unknowns they will start filling the history immediately
        and stagnation will NOT fire until the window fills again.
        """
        px, py  = self.position[0], self.position[1]
        nearest = min(self.branch_points,
                      key=lambda bp: math.hypot(bp[0]-px, bp[1]-py))
        dist_bp = math.hypot(nearest[0]-px, nearest[1]-py)

        # [36] Consume immediately — never revisit this exact junction
        self.branch_points.remove(nearest)

        if dist_bp < 0.4:
            # Already here — just clear history and return; stagnation counter
            # will increment again next tick if unknowns truly aren't moving.
            self.get_logger().info(
                f'Already at branch point '
                f'({nearest[0]:.2f},{nearest[1]:.2f}) — consumed, '
                f'{len(self.branch_points)} left')
            self.unknown_history.clear()
            return

        self.goal                  = nearest
        self.goal_score            = 0.0
        self.recent_goals.append(self.goal)
        self.goal_start_pos        = (px, py)
        self.goal_start_time       = now
        self.last_progress_pos     = (px, py)
        self.last_progress_time    = now
        self.stuck_events_for_goal = 0

        # [36] Clear history so the window restarts after arrival
        self.unknown_history.clear()

        self.get_logger().info(
            f'Branch recovery → ({nearest[0]:.2f},{nearest[1]:.2f}) '
            f'd={dist_bp:.2f}m | {len(self.branch_points)} branch(es) remain')
        self._plan_path_to_goal()
        if self.waypoints:
            self.state = State.NAVIGATE
        else:
            self.goal = self.goal_score = None
            self.get_logger().warn(
                'No A* path to branch point — it was already consumed')

    # ══════════════════════════════════════════════════════════════════════════
    # No-cluster fallback (secondary path)
    # ══════════════════════════════════════════════════════════════════════════

    def _handle_no_clusters(self, now):
        """
        Called only when _get_frontier_clusters() genuinely returns empty.
        Stagnation and branch recovery are handled upstream so this is a
        last-resort BFS fallback.
        """
        if self.branch_points:
            self._do_branch_recovery(now)
            return

        if (now - self.last_reachability_time) < REACHABILITY_CHECK_INTERVAL:
            self._publish_vel(y=self._get_wall_correction())
            return

        self.last_reachability_time = now
        if self._has_reachable_unknown():
            self.get_logger().warn(
                'No clusters & no branches but BFS finds reachable unknowns — '
                'hovering')
            self._publish_vel(y=self._get_wall_correction())
        else:
            elapsed = self._elapsed_exploration_time(now)
            self.get_logger().info(
                f'BFS: no reachable unknowns. t={elapsed:.1f}s → going home')
            self.state = State.DONE

    # ══════════════════════════════════════════════════════════════════════════
    # Branch point recording  [22]
    # ══════════════════════════════════════════════════════════════════════════

    def _record_branch_point(self, clusters):
        px, py   = self.position[0], self.position[1]
        bearings = [math.degrees(math.atan2(c[1]-py, c[0]-px))
                    for c in clusters[:5]]
        max_spread = 0.0
        for i in range(len(bearings)):
            for j in range(i+1, len(bearings)):
                d = abs(bearings[i] - bearings[j])
                if d > 180.0:
                    d = 360.0 - d
                if d > max_spread:
                    max_spread = d
        if max_spread < BRANCH_ANGULAR_SPREAD_DEG:
            return
        for bx, by in self.branch_points:
            if math.hypot(px-bx, py-by) < BRANCH_DEDUP_RADIUS:
                return
        self.branch_points.append((px, py))
        self.get_logger().info(
            f'Branch recorded ({px:.2f},{py:.2f}) '
            f'spread={max_spread:.0f}° — {len(self.branch_points)} total')

    # ══════════════════════════════════════════════════════════════════════════
    # Reachability BFS  [23]
    # ══════════════════════════════════════════════════════════════════════════

    def _has_reachable_unknown(self):
        if self.map_data is None:
            return False
        W, H = self.map_width, self.map_height
        s    = REACHABILITY_STRIDE
        sr, sc = self._world_to_grid(self.position[0], self.position[1])
        sr = max(0, min(H-1, round(sr/s)*s))
        sc = max(0, min(W-1, round(sc/s)*s))
        if self.map_data[sr*W+sc] != 0:
            found = False
            for dsr in range(-s, s+1, s):
                for dsc in range(-s, s+1, s):
                    nr, nc = sr+dsr, sc+dsc
                    if 0 <= nr < H and 0 <= nc < W:
                        if self.map_data[nr*W+nc] == 0:
                            sr, sc = nr, nc; found = True; break
                if found:
                    break
            if not found:
                return False
        visited = {(sr, sc)}
        queue   = deque([(sr, sc)])
        dirs4   = [(-s,0),(s,0),(0,-s),(0,s)]
        while queue:
            row, col = queue.popleft()
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    nr, nc = row+dr, col+dc
                    if 0 <= nr < H and 0 <= nc < W:
                        if self.map_data[nr*W+nc] == -1:
                            return True
            for dr, dc in dirs4:
                nr, nc = row+dr, col+dc
                if (0 <= nr < H and 0 <= nc < W and
                        (nr, nc) not in visited and
                        self.map_data[nr*W+nc] == 0):
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Shared waypoint-following helper  [9]
    # ══════════════════════════════════════════════════════════════════════════

    def _follow_waypoints(self, now):
        if self.current_wp is not None:
            dx, dy     = (self.current_wp[0]-self.position[0],
                          self.current_wp[1]-self.position[1])
            dist_to_wp = math.hypot(dx, dy)
            threshold  = (FINAL_GOAL_DIST if not self.waypoints
                          else WAYPOINT_REACHED_DIST)
            if dist_to_wp < threshold:
                if self.waypoints:
                    self.current_wp = self.waypoints.pop(0)
                    self.get_logger().info(
                        f'WP reached → '
                        f'({self.current_wp[0]:.2f},{self.current_wp[1]:.2f})'
                        f' {len(self.waypoints)} left')
                else:
                    self.current_wp = None
                    self.get_logger().info('Goal reached')
                    return True
        elif self.waypoints:
            self.current_wp = self.waypoints.pop(0)

        if self.current_wp is not None:
            dx, dy     = (self.current_wp[0]-self.position[0],
                          self.current_wp[1]-self.position[1])
            dist_to_wp = math.hypot(dx, dy)
            ye  = self._wrap_angle(math.atan2(dy, dx) - self.angles[2])
            vy, speed_scale = self._get_wall_guidance()
            vx  = min(CRUISE_SPEED * max(0.0, math.cos(ye)) * speed_scale, dist_to_wp)
            if abs(vy) > OFFCENTER_HARD_BAND:
                vx *= 0.65
            wz  = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, 3.0 * ye))
            self._publish_vel(
                x=vx,
                y=vy,
                wz=wz)
        else:
            vy, _ = self._get_wall_guidance()
            self._publish_vel(y=vy)
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Wall avoidance helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _collision_risk_detected(self):
        right, left = self._side_ranges_for_control()
        return (self._front_range() < OBSTACLE_DIST or
                right < SIDE_TOO_CLOSE_DIST or
                left  < SIDE_TOO_CLOSE_DIST)

    def _start_wall_avoid(self, now):
        r, l = self._right_range(), self._left_range()
        if r < SIDE_TOO_CLOSE_DIST and l >= SIDE_TOO_CLOSE_DIST:
            self.avoid_turn_dir, self.avoid_lateral_dir = +1.0, +1.0
        elif l < SIDE_TOO_CLOSE_DIST and r >= SIDE_TOO_CLOSE_DIST:
            self.avoid_turn_dir, self.avoid_lateral_dir = -1.0, -1.0
        else:
            if l > r:
                self.avoid_turn_dir, self.avoid_lateral_dir = +1.0, +1.0
            else:
                self.avoid_turn_dir, self.avoid_lateral_dir = -1.0, -1.0
        self.wall_avoid_start_time = now
        self.state = State.WALL_AVOID

    # ══════════════════════════════════════════════════════════════════════════
    # Scan coverage  [13]
    # ══════════════════════════════════════════════════════════════════════════

    def _update_scan_coverage(self, row, col):
        H, W = self.map_height, self.map_width
        sr   = SENSOR_RANGE_CELLS
        sr2  = sr * sr
        for dr in range(-sr, sr+1):
            nr = row + dr
            if nr < 0 or nr >= H:
                continue
            max_dc = int(math.sqrt(sr2 - dr*dr))
            for nc in range(max(0, col-max_dc), min(W-1, col+max_dc)+1):
                self.scanned_cells.add((nr, nc))

    # ══════════════════════════════════════════════════════════════════════════
    # Coordinate memory / blacklist / scoring
    # ══════════════════════════════════════════════════════════════════════════

    def _mark_visited(self, row, col):
        for dr in range(-VISITED_CELL_RADIUS, VISITED_CELL_RADIUS+1):
            for dc in range(-VISITED_CELL_RADIUS, VISITED_CELL_RADIUS+1):
                nr, nc = row+dr, col+dc
                if 0 <= nr < self.map_height and 0 <= nc < self.map_width:
                    k = (nr, nc)
                    self.visited_counts[k] = self.visited_counts.get(k, 0) + 1

    def _visited_penalty(self, row, col):
        return VISIT_PENALTY * self.visited_counts.get((row, col), 0)

    def _recent_goal_penalty(self, wx, wy):
        return sum(RECENT_GOAL_PENALTY for gx, gy in self.recent_goals
                   if math.hypot(wx-gx, wy-gy) < 0.7)

    def _failed_goal_penalty(self, wx, wy):
        return sum(FAILED_GOAL_PENALTY for gx, gy in self.failed_goals
                   if math.hypot(wx-gx, wy-gy) < 0.9)

    def _failed_region_penalty(self, now, wx, wy):
        return sum(BLACKLIST_PENALTY for r in self.failed_regions
                   if now <= r["until"] and
                   math.hypot(wx-r["x"], wy-r["y"]) < FAILED_REGION_RADIUS)

    def _prune_failed_regions(self, now):
        if self.failed_regions:
            self.failed_regions = [r for r in self.failed_regions
                                   if now <= r["until"]]

    def _mark_goal_failed(self, now, goal, reason):
        if goal is None:
            return
        gx, gy = goal
        self.failed_goals.append(goal)
        self.failed_regions.append(
            {"x": gx, "y": gy, "until": now + FAILED_REGION_COOLDOWN})
        self.get_logger().warn(
            f'Failed ({gx:.2f},{gy:.2f}) {FAILED_REGION_COOLDOWN:.0f}s: '
            f'{reason}')

    def _count_unknown_near_cluster(self, cluster):
        W, H  = self.map_width, self.map_height
        total = new = 0
        for row, col in cluster:
            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    nr, nc = row+dr, col+dc
                    if 0 <= nr < H and 0 <= nc < W:
                        if self.map_data[nr*W+nc] == -1:
                            total += 1
                            if (nr, nc) not in self.scanned_cells:
                                new += 1
        n = max(1, len(cluster))
        return total/n, new/n

    def _estimate_path_cost(self, wx, wy):
        row, col = self._world_to_grid(wx, wy)
        return (math.hypot(wx-self.position[0], wy-self.position[1]) +
                0.15 * self.visited_counts.get((row, col), 0))

    def _find_matching_cluster(self, goal, clusters):
        if goal is None:
            return None
        gx, gy   = goal
        best, bd = None, float('inf')
        for c in clusters:
            d = math.hypot(c[0]-gx, c[1]-gy)
            if d < bd:
                bd, best = d, c
        return best if bd < 0.6 else None

    # ══════════════════════════════════════════════════════════════════════════
    # Stuck detector / goal handling
    # ══════════════════════════════════════════════════════════════════════════

    def _is_stuck(self, now):
        if self.last_progress_pos is None:
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = now
            return False
        moved = math.hypot(self.position[0]-self.last_progress_pos[0],
                           self.position[1]-self.last_progress_pos[1])
        if moved > STUCK_PROGRESS_DIST:
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = now
            return False
        return (now - self.last_progress_time) > STUCK_TIMEOUT

    def _abandon_current_goal(self):
        self.goal = self.goal_score = None
        self.waypoints    = []
        self.current_wp   = None
        self.needs_replan = False
        self.last_replan_pos = None
        self.state = State.FIND_FRONTIER

    # ══════════════════════════════════════════════════════════════════════════
    # Timer / status logging
    # ══════════════════════════════════════════════════════════════════════════

    def _elapsed_exploration_time(self, now):
        return (0.0 if self.exploration_start_time is None
                else now - self.exploration_start_time)

    def _log_status(self, now):
        if (now - self.last_status_log_time) < self.status_log_period:
            return
        self.last_status_log_time = now
        elapsed = self._elapsed_exploration_time(now)
        rel_x   = self.position[0] - (self.start_pos[0] if self.start_pos else 0.0)
        rel_y   = self.position[1] - (self.start_pos[1] if self.start_pos else 0.0)
        goal_s  = ('None' if self.goal is None
                   else f'({self.goal[0]:.2f},{self.goal[1]:.2f})')
        unk, drop = self._unknown_window_stats()
        wall_y, wall_speed = self._get_wall_guidance()
        corridor = ('open' if self.last_corridor_width is None
                    else f'{self.last_corridor_width:.2f}m')
        self.get_logger().info(
            f'[t={elapsed:.1f}s] {self.state.name} '
            f'pos=({rel_x:.2f},{rel_y:.2f},{self.position[2]:.2f}) '
            f'yaw={math.degrees(self.angles[2]):.1f}° '
            f'goal={goal_s} '
            f'unk={unk} Δ={drop:+d}/{UNKNOWN_RATE_WINDOW:.0f}s '
            f'stag={self.consecutive_stagnations}/'
            f'{MAX_CONSECUTIVE_STAGNATIONS} '
            f'branches={len(self.branch_points)} '
            f'wall_y={wall_y:.2f} wall_v={wall_speed:.2f} '
            f'corridor={corridor}')

    # ══════════════════════════════════════════════════════════════════════════
    # Wall safety and corridor centering
    # ══════════════════════════════════════════════════════════════════════════

    def _side_ranges_for_control(self):
        right = self.filtered_right if self.filtered_right is not None else self._right_range()
        left  = self.filtered_left  if self.filtered_left  is not None else self._left_range()
        if right <= 0.0:
            right = 999.0
        if left <= 0.0:
            left = 999.0
        return right, left

    def _get_wall_guidance(self):
        if len(self.ranges) < 4:
            self.last_centering_active = False
            self.last_corridor_width = None
            return 0.0, 1.0

        r, l = self._side_ranges_for_control()
        vy = 0.0
        speed_scale = 1.0

        # First priority: push away hard from a nearby wall.
        if r < WALL_PUSH_DIST:
            vy += min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - r))
        if l < WALL_PUSH_DIST:
            vy -= min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - l))

        right_seen = r < WALL_CENTRE_MAX_VALID
        left_seen  = l < WALL_CENTRE_MAX_VALID

        self.last_centering_active = False
        self.last_corridor_width = None

        # Second priority: if both walls are visible, actively centre.
        if right_seen and left_seen:
            width = r + l
            self.last_corridor_width = width
            self.last_centering_active = True

            if width <= TIGHT_CORRIDOR_WIDTH:
                kp = WALL_KP_CENTRE_TIGHT
                speed_scale = TIGHT_SPEED_SCALE
            elif width <= MEDIUM_CORRIDOR_WIDTH:
                kp = WALL_KP_CENTRE_MEDIUM
                speed_scale = MEDIUM_SPEED_SCALE
            else:
                kp = WALL_KP_CENTRE_WIDE
                speed_scale = WIDE_SPEED_SCALE

            centering = kp * (r - l)
            vy += centering

            # Slow down more when clearly off-centre.
            off_center = abs(r - l)
            if off_center > OFFCENTER_HARD_BAND:
                speed_scale *= 0.65
            elif off_center > OFFCENTER_SLOW_BAND:
                speed_scale *= 0.82
        else:
            # In open spaces or one-sided walls, do not fake centring.
            # Only keep some slowdown if a side wall is still near.
            nearest = min(r, l)
            if nearest < WALL_SAFE_DIST:
                speed_scale = 0.75

        vy = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, vy))
        speed_scale = max(0.45, min(1.0, speed_scale))
        return vy, speed_scale

    def _get_wall_correction(self):
        vy, _ = self._get_wall_guidance()
        return vy

    # ══════════════════════════════════════════════════════════════════════════
    # A* pathfinding  [2]
    # ══════════════════════════════════════════════════════════════════════════

    def _world_to_grid(self, wx, wy):
        return (int((wy-self.map_origin[1]) / MAP_RES),
                int((wx-self.map_origin[0]) / MAP_RES))

    def _grid_to_world(self, row, col):
        return (self.map_origin[0] + (col+0.5)*MAP_RES,
                self.map_origin[1] + (row+0.5)*MAP_RES)

    def _build_inflated_map(self, inflation):
        W, H     = self.map_width, self.map_height
        passable = np.ones(W*H, dtype=bool)
        passable[self.map_data == 100] = False
        if inflation > 0:
            for idx in np.where(self.map_data == 100)[0]:
                r, c = divmod(int(idx), W)
                r0, r1 = max(0, r-inflation), min(H-1, r+inflation)
                c0, c1 = max(0, c-inflation), min(W-1, c+inflation)
                for rr in range(r0, r1+1):
                    passable[rr*W+c0: rr*W+c1+1] = False
        return passable

    def _plan_path_to_goal(self):
        self.waypoints = []
        self.current_wp = None
        self.needs_replan = False
        self.last_replan_time = 0.0
        self.last_replan_pos  = None
        self.last_path_cost   = None
        if self.map_data is None or self.goal is None:
            self.get_logger().warn('A*: no map or goal')
            return
        sr, sc = self._world_to_grid(self.position[0], self.position[1])
        gr, gc = self._world_to_grid(self.goal[0], self.goal[1])
        gr = max(0, min(self.map_height-1, gr))
        gc = max(0, min(self.map_width-1, gc))
        self.get_logger().info(f'A*: ({sr},{sc})→({gr},{gc})')
        nb8 = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
               (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]
        found = False; path_cost = None
        for inflation in range(WALL_INFLATION_CELLS, -1, -1):
            if inflation < WALL_INFLATION_CELLS:
                self.get_logger().warn(f'A*: reduced inflation={inflation}')
            passable  = self._build_inflated_map(inflation)
            W, H      = self.map_width, self.map_height
            open_heap = [(0.0, 0.0, sr, sc)]
            came_from = {}; g_score = {(sr, sc): 0.0}; found = False
            while open_heap:
                _, g, row, col = heapq.heappop(open_heap)
                if (row, col) == (gr, gc):
                    found = True; path_cost = g; break
                if g > g_score.get((row, col), float('inf')):
                    continue
                for dr, dc, cost in nb8:
                    nr, nc = row+dr, col+dc
                    if not (0 <= nr < H and 0 <= nc < W):
                        continue
                    if not passable[nr*W+nc]:
                        continue
                    ng = g + cost
                    if ng < g_score.get((nr, nc), float('inf')):
                        g_score[(nr, nc)]   = ng
                        came_from[(nr, nc)] = (row, col)
                        heapq.heappush(open_heap,
                            (ng+math.hypot(nr-gr, nc-gc), ng, nr, nc))
            if found:
                if inflation < WALL_INFLATION_CELLS:
                    self.get_logger().warn(
                        f'A*: tight path inflation={inflation}')
                break
        if not found:
            self.get_logger().warn('A*: no path found')
            return
        path = []
        cell = (gr, gc)
        while cell in came_from:
            path.append(cell); cell = came_from[cell]
        path.reverse()
        wps = [self._grid_to_world(r, c)
               for i, (r, c) in enumerate(path)
               if i % WAYPOINT_SPACING == 0 or i == len(path)-1]
        wps.append(self.goal)
        self.waypoints      = wps
        self.last_path_cost = path_cost
        self.get_logger().info(
            f'A*: {len(path)} cells → {len(wps)} WPs cost={path_cost:.2f}')

    # ══════════════════════════════════════════════════════════════════════════
    # Frontier detection + clustering + utility scoring
    # ══════════════════════════════════════════════════════════════════════════

    def _get_frontier_cells(self):
        if self.map_data is None:
            return set()
        W, H = self.map_width, self.map_height
        fc   = set()
        for row in range(1, H-1, FRONTIER_STEP):
            for col in range(1, W-1, FRONTIER_STEP):
                if self.map_data[row*W+col] != 0:
                    continue
                if -1 in (self.map_data[(row-1)*W+col],
                           self.map_data[(row+1)*W+col],
                           self.map_data[row*W+(col-1)],
                           self.map_data[row*W+(col+1)]):
                    fc.add((row, col))
        return fc

    def _cluster_frontier_cells(self, frontier_cells):
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

    def _get_frontier_clusters(self, now):
        fc = self._get_frontier_cells()
        if not fc:
            return []
        clusters = self._cluster_frontier_cells(fc)
        tuning_steps = [
            (MIN_FRONTIER_DIST,      MIN_CLUSTER_SIZE,             COVERAGE_MIN_GAIN),
            (MIN_FRONTIER_DIST/2.0,  MIN_CLUSTER_SIZE,             COVERAGE_MIN_GAIN/2.0),
            (MIN_FRONTIER_DIST/2.0,  max(1, MIN_CLUSTER_SIZE//2),  COVERAGE_MIN_GAIN/4.0),
            (0.0,                    1,                            0.0),
        ]
        for step_idx, (md, ms, cg) in enumerate(tuning_steps):
            valid = self._score_clusters(clusters, md, ms, cg, now)
            if valid:
                if step_idx > 0:
                    self.get_logger().warn(
                        f'Auto-tune step {step_idx}: '
                        f'min_dist={md:.2f} min_size={ms} '
                        f'cov_gain={cg:.3f} → {len(valid)} clusters')
                return valid
        return []

    def _score_clusters(self, clusters, min_dist, min_size,
                        coverage_min_gain, now):
        px, py = self.position[0], self.position[1]
        valid  = []
        for cluster in clusters:
            n = len(cluster)
            if n < min_size:
                continue
            tr = tc = 0
            for r, c in cluster:
                tr += r; tc += c
            avg_r, avg_c = tr/n, tc/n
            row = int(round(avg_r)); col = int(round(avg_c))
            wx = self.map_origin[0] + (avg_c+0.5)*MAP_RES
            wy = self.map_origin[1] + (avg_r+0.5)*MAP_RES
            dist = math.hypot(wx-px, wy-py)
            if dist < min_dist:
                continue
            total_unk, new_unk = self._count_unknown_near_cluster(cluster)
            if total_unk > 0 and coverage_min_gain > 0.0:
                if new_unk / total_unk < coverage_min_gain:
                    continue
            path_cost = self._estimate_path_cost(wx, wy)
            penalty   = (self._visited_penalty(row, col)       +
                         self._recent_goal_penalty(wx, wy)     +
                         self._failed_goal_penalty(wx, wy)     +
                         self._failed_region_penalty(now, wx, wy))
            score = (SIZE_WEIGHT * n + UNKNOWN_WEIGHT * new_unk -
                     DISTANCE_WEIGHT * path_cost - penalty)
            valid.append((wx, wy, dist, n,
                          new_unk, total_unk,
                          penalty, score, path_cost))
        valid.sort(key=lambda c: c[7], reverse=True)
        large = [c for c in valid if c[3] >= MIN_VALID_CLUSTER_SIZE]
        small = [c for c in valid if c[3] <  MIN_VALID_CLUSTER_SIZE]
        return large + small if large else valid

    # ══════════════════════════════════════════════════════════════════════════
    # Low-level helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _front_range(self):
        return (self.ranges[2]
                if len(self.ranges) > 2 and self.ranges[2] > 0.0 else 999.0)

    def _right_range(self):
        return (self.ranges[1]
                if len(self.ranges) > 1 and self.ranges[1] > 0.0 else 999.0)

    def _left_range(self):
        return (self.ranges[3]
                if len(self.ranges) > 3 and self.ranges[3] > 0.0 else 999.0)

    def _wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    def _publish_vel(self, x=0.0, y=0.0, z=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = float(x)
        msg.linear.y  = float(y)
        msg.linear.z  = float(z)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationMultiranger()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()