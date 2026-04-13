#!/usr/bin/env python3

"""
Autonomous Frontier-Based Explorer for Crazyflie
=================================================
State machine:
  1. TAKEOFF       - take off and hover
  2. SPINNING      - 100° scan; used both after takeoff and after each frontier
                     goal is reached, so every scan is identical
  3. FIND_FRONTIER - score frontiers, pick best goal
  4. NAVIGATE      - follow A* waypoints to goal
  5. DONE          - exploration complete → return to (0,0) via A* → land
  7. LANDING       - descend and stop

Efficiency (v2): [1-10]
Coverage  (v3): [11-18]

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

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
TAKEOFF_HEIGHT         = 0.3
TAKEOFF_DELAY          = 3.0
CRUISE_SPEED           = 0.3 #0.3
MAX_TURN_RATE          = 0.2
OBSTACLE_DIST          = 0.4   # outer detection radius: trigger replan when front wall within this distance
GOAL_REACHED_DIST      = 0.2   # shared threshold for waypoints, final goal, and home arrival
MIN_FRONTIER_DIST      = 0.5
FRONTIER_STEP          = 1
REPLAN_COOLDOWN        = 3.5
WALL_INFLATION_CELLS   = 2      # cells of clearance around known walls
STANDOFF_WAYPOINTS = 2          # waypoints to trim from the end of the A* path;
                                 # at WAYPOINT_SPACING=2 cells and MAP_RES=0.1m
                                 # this gives ~0.4m standoff from the frontier,
                                 # always on the correct side of any wall
PROXIMITY_COST_WEIGHT  = 2    # how strongly A* avoids cells near walls;
                                 # higher = path hugs centre more but may
                                 # fail in tight corridors; 2-6 is a good range
PROXIMITY_COST_RADIUS  = 10#10   # cells — BFS radius for proximity cost map;
                                 # cells within this radius of a wall/unknown
                                 # get a proximity penalty in A*;
                                 # at MAP_RES=0.1m, 10 cells = 1.0m clearance zone

# ── Wall avoidance parameters ─────────────────────────────────────────────────
# ── Frontier filtering ────────────────────────────────────────────────────────
MIN_CLUSTER_SIZE       = 15   # preferred minimum cluster size; relaxed to MIN_VALID_CLUSTER_SIZE if no clusters pass
MIN_VALID_CLUSTER_SIZE = 2    # absolute floor

# ── Wall safety ───────────────────────────────────────────────────────────────
# Push the drone away from any wall closer than WALL_PUSH_DIST on all four
# axes.  WALL_SAFE_DIST is kept as the threshold below which speed is reduced
# when only one side wall is visible.
WALL_PUSH_DIST          = 0.3   # inner hard-push radius: must be < OBSTACLE_DIST
WALL_SAFE_DIST          = 0.30
WALL_FILTER_ALPHA       = 0.3
WALL_KP_SAFETY          = 0.3
MAX_LATERAL_SPEED       = 0.24

# ── Initial spin ──────────────────────────────────────────────────────────────
SPIN_RATE = 0.5

# ── Coordinate memory / revisit prevention ───────────────────────────────────
VISITED_CELL_RADIUS = 2
RECENT_GOAL_MEMORY  = 8

# ── Utility scoring weights ───────────────────────────────────────────────────
DISTANCE_WEIGHT     = 1.6 
SIZE_WEIGHT         = 1.6
UNKNOWN_WEIGHT      = 2.0
VISIT_PENALTY       = 0.5 #0.9
RECENT_GOAL_PENALTY = 0.0 #10.0

# ── Peer claim coordination ───────────────────────────────────────────────────
# Gradient-based separation: frontiers far from all peer goals get a bonus,
# frontiers close to a peer goal get a penalty.  The modifier is a smooth
# tanh S-curve so there is no hard radius threshold to tune per map.
#
# Formula per peer:
#   modifier = PEER_GRADIENT_WEIGHT * tanh(dist / PEER_GRADIENT_SCALE - 1.0)
#
# At dist == PEER_GRADIENT_SCALE the modifier is zero (neutral).
# Below that distance it goes negative (penalty, min ~ -PEER_GRADIENT_WEIGHT).
# Above that distance it goes positive (bonus, max ~ +PEER_GRADIENT_WEIGHT).
#
# PEER_GRADIENT_SCALE  — crossover distance in metres; set to roughly half the
#                        expected room width.  2.0 m suits most indoor maps.
# PEER_GRADIENT_WEIGHT — maximum bonus/penalty magnitude in score units.
#                        6.0 is roughly equal to SIZE_WEIGHT * 4 cells, enough
#                        to reliably redirect a drone without hard-blocking it.
PEER_GRADIENT_SCALE     = 0.5   # metres — neutral crossover distance
PEER_GRADIENT_WEIGHT    = 10.0   # maximum score bonus/penalty magnitude
PEER_CLAIM_TIMEOUT      = 5.0   # seconds — ignore stale claims (crashed/landed)
PEER_CLAIM_PUB_INTERVAL = 0.5   # seconds between goal publications

# ── Goal commitment / hysteresis ─────────────────────────────────────────────
GOAL_KEEP_RATIO       = 0.85
GOAL_SWITCH_MIN_DELTA = 2.0

# ── Stuck detector ────────────────────────────────────────────────────────────
STUCK_PROGRESS_DIST       = 0.20
STUCK_TIMEOUT             = 5.0
MAX_STUCK_EVENTS_PER_GOAL = 2
MAX_REPLANS_PER_GOAL      = 3  # abandon goal after this many replans without reaching next waypoint

# [fix-v8] Reduced from 15 → 7 cells (0.7 m gaps) so the nav loop catches
# obstacles more frequently between waypoints.
WAYPOINT_SPACING = 2

# ── Scan coverage tracking [11] ───────────────────────────────────────────────
SENSOR_RANGE_CELLS = 5

# ── Reachability BFS [23][25] ─────────────────────────────────────────────────
REACHABILITY_STRIDE         = 3
REACHABILITY_CHECK_INTERVAL = 3.0



class State(Enum):
    TAKEOFF       = auto()
    SPINNING      = auto()
    FIND_FRONTIER = auto()
    NAVIGATE      = auto()
    DONE          = auto()
    LANDING       = auto()


class GoalHealth(Enum):
    """Return value of _navigate_goal_health().
    Encapsulates stuck detection and replan-count gating so the
    NAVIGATE block only sees a single verdict, not the raw counters."""
    HEALTHY  = auto()   # nothing wrong — continue navigating
    REPLAN   = auto()   # drone has moved; try a fresh A* path
    ABANDON  = auto()   # goal is unreachable; discard and pick a new one


class Safety(Enum):
    """Return value of _navigate_safety_check().
    Summarises sensor readings into a single action word so the
    NAVIGATE block does not need to know the threshold values."""
    CLEAR         = auto()   # no obstacle within braking distance
    REPLAN_NEEDED = auto()   # front wall close enough to warrant a new path


class FrontierExplorationMultiranger(Node):

    def __init__(self):
        super().__init__('frontier_exploration_multiranger')

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
        self.navigating_home  = False

        # ── Coordinate memory ─────────────────────────────────────────────────
        self.filtered_right = None
        self.filtered_left  = None

        self.visited_counts = {}
        self.recent_goals   = deque(maxlen=RECENT_GOAL_MEMORY)

        # ── Peer claim state ──────────────────────────────────────────────────
        # Maps sender identity hash (float) → (goal_x, goal_y, timestamp)
        self.peer_goals: dict          = {}
        self.last_claim_pub_time: float = 0.0
        # Unique float identity derived from robot_prefix so each drone can
        # filter its own echoed messages off the shared /peer_claims topic.
        self._claim_id = float(hash(robot_prefix) % 1_000_000)

        self.scanned_cells = set()                          # [12]
        self.last_reachability_time = 0.0                   # [25]
        self.zero_cluster_count = 0                         # successive FIND_FRONTIER ticks with 0 clusters
        self.no_path_failures = 0                           # successive frontiers with no A* path found

        # ── Goal / stuck tracking ─────────────────────────────────────────────
        self.goal_start_pos        = None
        self.goal_start_time       = 0.0
        self.last_progress_pos     = None
        self.last_progress_time    = 0.0
        self.stuck_events_for_goal = 0
        self.replan_count_for_goal = 0

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
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.cmd_pub = self.create_publisher(Twist, robot_prefix + '/cmd_vel_raw', 10) 
        self.marker_pub = self.create_publisher(Marker, robot_prefix + '/exploration_goal', 10)
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, robot_prefix + '/waypoints', 10)
        self.drone_marker_pub = self.create_publisher(Marker, robot_prefix + '/drone_pose', 10)
        self.possible_goal_marker_pub = self.create_publisher(Marker, robot_prefix + '/possible_goal', 10)
        self.failed_goal_marker_pub = self.create_publisher(Marker, robot_prefix + '/failed_goal', 10)

        # ── Peer claim pub/sub ────────────────────────────────────────────────
        # All drones publish to and subscribe from the single shared topic
        # /peer_claims.  Each message is a Point where:
        #   x, y = claimed goal world coordinates (0,0 means no active goal)
        #   z    = sender identity hash (float) for self-filtering
        self.claim_pub = self.create_publisher(Point, '/peer_claims', 10)
        self.create_subscription(Point, '/peer_claims', self._peer_claim_callback, 10)

        self.create_service(
            Trigger, robot_prefix + '/stop_exploration', self.stop_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self._publish_vel(z=TAKEOFF_HEIGHT)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._info(f'Explorer started. prefix={robot_prefix}')

    # ══════════════════════════════════════════════════════════════════════════
    # Logging helpers — prepend current state to every message so the state
    # is always visible without repeating the ROS timestamp or node name.
    # Format:  [STATE] message
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
            self.last_progress_pos = (self.position[0], self.position[1])
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_progress_time = now
           

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
        self._info('Stop requested — landing now')
        self.timer.cancel()
        self._publish_vel(z=-0.2)
        response.success = True
        return response

    def _peer_claim_callback(self, msg: Point):
        """Receive a peer drone's claimed goal off /peer_claims.

        msg.x, msg.y  world coordinates of the peer's current goal.
                      Both zero means the peer has no active goal.
        msg.z         sender identity hash — used to filter out our own
                      echoed messages and to key the peer_goals dict.
        """
        if abs(msg.z - self._claim_id) < 0.5:
            return   # own message echoed back — discard
        now = self.get_clock().now().nanoseconds * 1e-9
        key = msg.z
        if msg.x == 0.0 and msg.y == 0.0:
            self.peer_goals.pop(key, None)
        else:
            self.peer_goals[key] = (msg.x, msg.y, now)

    # ══════════════════════════════════════════════════════════════════════════
    # Main state machine
    # ══════════════════════════════════════════════════════════════════════════

    def timer_callback(self):
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._state_machine(now)
        except Exception as e:
            import traceback
            self._err(f'Crash: {e}')
            self._err(traceback.format_exc())
            self._publish_vel()
            self.timer.cancel()

    def _state_machine(self, now):
        self._log_status(now)

        # ── Publish peer claim ────────────────────────────────────────────────
        # Broadcast this drone's current goal to all peers at a fixed interval
        # so they can penalise nearby frontiers in their own scoring.
        # Publishing x=y=0 signals that this drone has no active claim so peers
        # do not permanently reserve the region after a goal is cleared.
        if now - self.last_claim_pub_time > PEER_CLAIM_PUB_INTERVAL:
            self.last_claim_pub_time = now
            claim = Point()
            if self.goal is not None and not self.navigating_home:
                claim.x = float(self.goal[0])
                claim.y = float(self.goal[1])
            else:
                claim.x = 0.0
                claim.y = 0.0
            claim.z = self._claim_id
            self.claim_pub.publish(claim)

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self.state == State.TAKEOFF:
            self._publish_vel(z=TAKEOFF_HEIGHT)
            # Wait until the drone has physically reached hover height AND
            # the minimum delay has elapsed. The control node requires a
            # positive linear.z to trigger its internal takeoff sequence;
            # it will not relay other commands until is_flying is True.
            airborne = self.position[2] >= TAKEOFF_HEIGHT * 0.8
            if airborne and now - self.start_time > TAKEOFF_DELAY:
                self.start_pos         = [self.position[0], self.position[1]]
                self._info( f'Home captured: ' f'({self.start_pos[0]:.3f},{self.start_pos[1]:.3f})')
                self._info(  'Takeoff complete. Rotating 100° to build initial map before exploring.')
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

            if self.spin_total_rotation >= 10/36* 2 * math.pi:
                self._publish_vel()
                self.exploration_start_time = now
                self._info('Scan complete. Starting frontier selection.')
                self.state = State.FIND_FRONTIER

        # ── FIND_FRONTIER ─────────────────────────────────────────────────────
        elif self.state == State.FIND_FRONTIER:
            self._publish_vel(y=self._get_wall_correction())

            if not self.map_received or not self.position_received:
                self._info('Waiting for map and position data from sensors...')
                return

            # ── Record junction if multiple directions visible [22] ────────────
            clusters = self._get_frontier_clusters(now)

            self._info(
                f'Choosing next frontier. Found {len(clusters)} candidate(s).')
            for i, c in enumerate(clusters[:8]):
                cx, cy, dist, size, new_unk, total_unk, pen, score, pcost = c
                cov = 100.0 * (1.0 - new_unk / max(1.0, total_unk))
                mark = (' <-- CURRENT GOAL'
                        if self.goal is not None and
                        math.hypot(cx-self.goal[0], cy-self.goal[1]) < 0.2
                        else (' <-- BEST' if i == 0 else ''))
                self._info(
                    f'  F{i+1}: straight-line={dist:.2f}m path-cost={pcost:.2f} '
                    f'size={size}cells new-unknowns={new_unk:.0f} '
                    f'already-scanned={cov:.0f}% penalty={pen:.1f} score={score:.2f}{mark}')

            # No clusters → secondary BFS fallback
            if not clusters:
                self.zero_cluster_count += 1
                self._warn(
                    f'No frontier clusters found '
                    f'({self.zero_cluster_count}/3 successive tries).')
                if self.zero_cluster_count >= 3:
                    elapsed = self._elapsed_exploration_time(now)
                    self._info(
                        f'0 clusters found 3 times in a row. t={elapsed:.1f}s → going home')
                    self.zero_cluster_count = 0
                    self.state = State.DONE
                    return
                self._handle_no_clusters(now)
                return

            # ── Normal goal selection ─────────────────────────────────────────
            self.zero_cluster_count = 0
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
                        self._info(
                            f'Keeping current goal ({self.goal[0]:.2f},{self.goal[1]:.2f}) '
                            f'— not worth switching. Current score={cs:.2f}, best available={clusters[0][7]:.2f}.')

            if not keep_current:
                self.goal                  = new_goal
                self.goal_score            = new_score
                self.recent_goals.append(self.goal)
                self.goal_start_pos        = (self.position[0], self.position[1])
                self.goal_start_time       = now
                self.last_progress_pos     = (self.position[0], self.position[1])
                self.last_progress_time    = now
                self.stuck_events_for_goal = 0
                self.replan_count_for_goal = 0
                self._info(
                    f'New goal selected: ({self.goal[0]:.2f},{self.goal[1]:.2f}) '
                    f'score={self.goal_score:.2f}. Planning path...')
            else:
                self._info(
                    f'Continuing toward ({self.goal[0]:.2f},{self.goal[1]:.2f}) '
                    f'score={self.goal_score:.2f}. Replanning path...')

            self._publish_possible_goal_marker(self.goal[0], self.goal[1])
            self._plan_path_to_goal()
            if not self.waypoints:
                self._publish_failed_goal_marker(self.goal[0], self.goal[1])
                self.no_path_failures += 1
                self._warn(
                    f'Could not find a path to ({self.goal[0]:.2f},{self.goal[1]:.2f}). '
                    f'Choosing a new frontier. '
                    f'(no_path_failures={self.no_path_failures}/{len(clusters)})')
                self.recent_goals.append(self.goal)
                self.goal = self.goal_score = None
                # If every available frontier has failed A*, there is nothing
                # left to navigate to — transition to DONE rather than looping.
                if self.no_path_failures >= len(clusters):
                    elapsed = self._elapsed_exploration_time(now)
                    self._warn(
                        f'All {len(clusters)} frontier(s) are unreachable via A*. '
                        f't={elapsed:.1f}s → going home.')
                    self.no_path_failures = 0
                    self.state = State.DONE
                else:
                    self.state = State.FIND_FRONTIER
            else:
                self.no_path_failures = 0
                self._info(
                    f'Path found with {len(self.waypoints)} waypoints. Navigating.')
                self.state = State.NAVIGATE

        # ── NAVIGATE ──────────────────────────────────────────────────────────
        elif self.state == State.NAVIGATE:
            if self.goal is None:
                self._warn('NAV-ENTRY: goal is None — redirecting to FIND_FRONTIER')
                self.state = State.FIND_FRONTIER
                self._publish_vel(y=self._get_wall_correction())
                return

            self._info(
                f'NAV-ENTRY: goal=({self.goal[0]:.2f},{self.goal[1]:.2f}) '
                f'wps={len(self.waypoints)} cur_wp={self.current_wp} '
                f'needs_replan={self.needs_replan} '
                f'replan_count={self.replan_count_for_goal} '
                f'stuck_events={self.stuck_events_for_goal} '
                f'last_replan_time={self.last_replan_time:.1f} now={now:.1f}')

            # Layer 1 — goal health: stuck detection and replan-count gating.
            health = self._navigate_goal_health(now)
            self._info(f'NAV-L1: health={health.name}')
            if health == GoalHealth.ABANDON:
                return
            if health == GoalHealth.REPLAN:
                self._navigate_execute_replan(now)
                return

            # Layer 2 — safety arbitration: front-wall replan trigger.
            safety = self._navigate_safety_check()
            cooldown_ok = self._replan_cooldown_ok(now)
            self._info(
                f'NAV-L2: safety={safety.name} '
                f'front={self._front_range():.3f}m '
                f'cooldown_ok={cooldown_ok} '
                f'time_since_replan={now - self.last_replan_time:.1f}s')
            if safety == Safety.REPLAN_NEEDED and cooldown_ok:
                self._navigate_execute_replan(now)
                return

            # Layer 3 — execution: waypoint following with wall correction.
            if not self.waypoints and self.current_wp is None:
                self._info('NAV-L3: waypoints exhausted and no current_wp — goal complete')
                self.goal = self.goal_score = None
                if self.navigating_home:
                    self.navigating_home = False
                    self._info('Home reached — landing.')
                    self.state = State.LANDING
                else:
                    self._info('Goal reached — spinning 100° to scan area')
                    self.spin_start_yaw      = self.angles[2]
                    self.spin_total_rotation = 0.0
                    self._last_spin_yaw      = self.angles[2]
                    self.state = State.SPINNING
                return

            self._info(
                f'NAV-L3: calling _follow_waypoints — '
                f'wps={len(self.waypoints)} cur_wp={self.current_wp}')
            if self._follow_waypoints(now):
                self.goal = self.goal_score = None
                if self.navigating_home:
                    self.navigating_home = False
                    self._info('Home reached — landing.')
                    self.state = State.LANDING
                else:
                    self._info('Goal reached — spinning 100° to scan area')
                    self.spin_start_yaw      = self.angles[2]
                    self.spin_total_rotation = 0.0
                    self._last_spin_yaw      = self.angles[2]
                    self.state = State.SPINNING

        # ── DONE — hand off to NAVIGATE for the return home ──────────────────
        elif self.state == State.DONE:
            if self.start_pos is None:
                self.state = State.LANDING
                return

            home = (self.start_pos[0], self.start_pos[1])
            elapsed = self._elapsed_exploration_time(now)
            self._info(
                f'Exploration done (t={elapsed:.1f}s). '
                f'Returning to start ({home[0]:.3f},{home[1]:.3f}) via A*.')
            self._publish_goal_marker(home[0], home[1], home=True)

            # If already within 10 cm, land immediately.
            dist_to_home = math.hypot(
                self.position[0] - home[0],
                self.position[1] - home[1])
            if dist_to_home < GOAL_REACHED_DIST:
                self._info(
                    f'Already at home ({dist_to_home:.2f}m). Landing immediately.')
                self.state = State.LANDING
                return

            # Set up NAVIGATE to treat home as the current goal.
            # navigating_home=True tells NAVIGATE to skip the post-goal spin
            # and go straight to LANDING, and to use a 10 cm arrival threshold.
            self.goal                  = home
            self.goal_score            = None
            self.navigating_home       = True
            self.waypoints             = []
            self.current_wp            = None
            self.needs_replan          = False
            self.last_replan_time      = 0.0
            self.last_replan_pos       = None
            self.last_path_cost        = None
            self.goal_start_pos        = (self.position[0], self.position[1])
            self.goal_start_time       = now
            self.last_progress_pos     = (self.position[0], self.position[1])
            self.last_progress_time    = now
            self.stuck_events_for_goal = 0
            self.replan_count_for_goal = 0

            self._plan_path_to_goal()
            if not self.waypoints:
                self._warn(
                    'Could not find a path home via A*. '
                    'Landing here.')
                self.state = State.LANDING
                return

            self._publish_waypoint_markers(self.waypoints)
            self._info(
                f'Return path planned: {len(self.waypoints)} waypoints. '
                f'Handing off to NAVIGATE.')
            self.state = State.NAVIGATE

        # ── LANDING ───────────────────────────────────────────────────────────
        elif self.state == State.LANDING:
            self._publish_vel(z=-0.2)
            if self.position[2] < 0.1:
                elapsed = self._elapsed_exploration_time(now)
                self.timer.cancel()
                self._publish_vel()
                self._info(
                    f'Landed at ({self.position[0]:.3f},{self.position[1]:.3f}). '
                    f'Total exploration time: {elapsed:.1f}s.')

    # ══════════════════════════════════════════════════════════════════════════
    # ══════════════════════════════════════════════════════════════════════════
    # NAVIGATE helpers — the three layers called by the NAVIGATE block
    # ══════════════════════════════════════════════════════════════════════════

    def _navigate_goal_health(self, now):
        """
        Layer 1 — goal health check.

        Encapsulates stuck detection and replan-count gating so the NAVIGATE
        block sees a single GoalHealth verdict instead of raw counter logic.

        Returns:
          GoalHealth.ABANDON  — goal is unreachable; caller must discard it.
          GoalHealth.REPLAN   — drone has moved since the last replan; caller
                                should attempt a fresh A* path.
          GoalHealth.HEALTHY  — nothing wrong; continue navigating normally.
        """
        if self._is_stuck(now):
            self.stuck_events_for_goal += 1
            self._warn(
                f'No progress toward goal for {STUCK_TIMEOUT:.0f}s '
                f'(stuck event {self.stuck_events_for_goal} of {MAX_STUCK_EVENTS_PER_GOAL}).')
            if self.stuck_events_for_goal >= MAX_STUCK_EVENTS_PER_GOAL:
                self._warn(
                    f'Stuck too many times on this goal. Abandoning '
                    f'({self.goal[0]:.2f},{self.goal[1]:.2f}) and choosing a new one.')
                self._abandon_current_goal()
                return GoalHealth.ABANDON

        if self.needs_replan:
            self.replan_count_for_goal += 1
            if self.replan_count_for_goal > MAX_REPLANS_PER_GOAL:
                self._warn(
                    f'Replanned {self.replan_count_for_goal} times without progress. '
                    f'Abandoning ({self.goal[0]:.2f},{self.goal[1]:.2f}).')
                self._abandon_current_goal()
                return GoalHealth.ABANDON
            cg = self._world_to_grid(self.position[0], self.position[1])
            if self.last_replan_pos is not None:
                if (abs(cg[0] - self.last_replan_pos[0]) <= 1 and
                        abs(cg[1] - self.last_replan_pos[1]) <= 1):
                    self.last_replan_pos = None
                    self._warn(
                        f'Replanned but drone has not moved. '
                        f'Giving up on ({self.goal[0]:.2f},{self.goal[1]:.2f}).')
                    self._abandon_current_goal()
                    return GoalHealth.ABANDON
            self.last_replan_pos = cg
            return GoalHealth.REPLAN

        return GoalHealth.HEALTHY

    def _navigate_safety_check(self):
        """
        Layer 2 — sensor-based safety arbitration.

        Reads the front ToF range and returns a Safety verdict.  Does NOT
        set needs_replan directly — the caller decides whether to act based
        on the verdict and the cooldown state.

        Returns:
          Safety.REPLAN_NEEDED — front wall within OBSTACLE_DIST; a new path
                                 should be requested if the cooldown allows.
          Safety.CLEAR         — no obstacle close enough to trigger a replan.
        """
        front = self._front_range()
        if 0.0 < front < OBSTACLE_DIST:
            self._warn(
                f'Obstacle {front:.2f}m ahead (threshold {OBSTACLE_DIST}m). '
                f'Requesting path replan.')
            return Safety.REPLAN_NEEDED
        return Safety.CLEAR

    def _replan_cooldown_ok(self, now):
        """Return True when enough time has passed since the last replan."""
        return (not self.needs_replan and
                (now - self.last_replan_time) > REPLAN_COOLDOWN)

    def _navigate_execute_replan(self, now):
        """
        Layer 2/3 bridge — run A* and update state accordingly.

        Called when either _navigate_goal_health returns REPLAN or
        _navigate_safety_check returns REPLAN_NEEDED with cooldown ok.
        Transitions to DONE on failure.
        """
        self.needs_replan     = False
        self.last_replan_time = now
        self._info(
            f'REPLAN: replanning path to ({self.goal[0]:.2f},{self.goal[1]:.2f}) '
            f'replan_count={self.replan_count_for_goal}')
        self._plan_path_to_goal()
        if not self.waypoints:
            self._warn(
                f'REPLAN: failed — no path to '
                f'({self.goal[0]:.2f},{self.goal[1]:.2f}). '
                f'Abandoning goal → FIND_FRONTIER.')
            self.no_path_failures += 1
            self._abandon_current_goal()
            self.state = State.FIND_FRONTIER
        else:
            self.no_path_failures = 0
            self._info(
                f'REPLAN: success — {len(self.waypoints)} waypoints remaining. '
                f'cur_wp reset to None, will pop on next tick.')

    # ══════════════════════════════════════════════════════════════════════════
    # No-cluster fallback (secondary path)
    # ══════════════════════════════════════════════════════════════════════════

    def _handle_no_clusters(self, now):
        """
        Called only when _get_frontier_clusters() genuinely returns empty.
        Last-resort BFS fallback — checks if any unknown cells are still
        reachable before declaring exploration complete.
        """
        if (now - self.last_reachability_time) < REACHABILITY_CHECK_INTERVAL:
            self._publish_vel(y=self._get_wall_correction())
            return

        self.last_reachability_time = now
        if self._has_reachable_unknown():
            self._warn(
                'No clusters but BFS finds reachable unknowns — '
                'hovering')
            self._publish_vel(y=self._get_wall_correction())
        else:
            elapsed = self._elapsed_exploration_time(now)
            self._info(
                f'BFS: no reachable unknowns. t={elapsed:.1f}s → going home')
            self.state = State.DONE

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
            threshold  = GOAL_REACHED_DIST
            self._info(
                f'FW: cur_wp=({self.current_wp[0]:.2f},{self.current_wp[1]:.2f}) '
                f'dist={dist_to_wp:.3f}m threshold={threshold:.3f}m '
                f'pos=({self.position[0]:.2f},{self.position[1]:.2f})')
            if dist_to_wp < threshold:
                if self.waypoints:
                    self.current_wp = self.waypoints.pop(0)
                    self.replan_count_for_goal = 0
                    self._info(
                        f'Waypoint reached. Next waypoint: '
                        f'({self.current_wp[0]:.2f},{self.current_wp[1]:.2f}), '
                        f'{len(self.waypoints)} remaining.')
                else:
                    self.current_wp = None
                    self._info('Reached goal position.')
                    return True
        elif self.waypoints:
            self.current_wp = self.waypoints.pop(0)
            self._info(
                f'FW: no current_wp, popped first waypoint '
                f'({self.current_wp[0]:.2f},{self.current_wp[1]:.2f}) '
                f'{len(self.waypoints)} remaining')

        if self.current_wp is not None:
            dx, dy     = (self.current_wp[0]-self.position[0],
                          self.current_wp[1]-self.position[1])
            dist_to_wp = math.hypot(dx, dy)
            yaw        = self.angles[2]

            # Rotate world-frame displacement into drone body frame.
            # vx_b = forward, vy_b = left.  This ensures the drone moves
            # directly toward the waypoint in both axes rather than only
            # using the forward component and letting wall guidance dictate
            # the lateral motion.
            vx_b =  math.cos(yaw) * dx + math.sin(yaw) * dy
            vy_b = -math.sin(yaw) * dx + math.cos(yaw) * dy
            # Full speed toward intermediate waypoints; ramp down only for
            # the final waypoint so the drone doesn't slow between every step.
            if self.waypoints:
                speed = CRUISE_SPEED / max(dist_to_wp, 1e-3)
            else:
                speed = min(CRUISE_SPEED, dist_to_wp) / max(dist_to_wp, 1e-3)

            # Wall guidance — single source of truth for all push-away.
            # Centering (vy corridor term) is intentionally excluded during
            # path-following to avoid lateral oscillation; _get_wall_guidance
            # only returns the push-away components here because centering
            # couples with the yaw controller.  We use vx and vy push-away
            # only, and keep speed_scale to slow down near walls.
            wall_vx, wall_vy, speed_scale = self._get_wall_guidance()

            # Scale down the waypoint-tracking vx when a front wall is close
            # so the push-away term dominates rather than fighting it.
            front = self._front_range()
            if front < OBSTACLE_DIST:
                front_scale = max(0.0, (front - WALL_PUSH_DIST) /
                                  max(OBSTACLE_DIST - WALL_PUSH_DIST, 1e-3))
            else:
                front_scale = 1.0

            vx = vx_b * speed * front_scale + wall_vx
            vx = max(-CRUISE_SPEED, min(CRUISE_SPEED, vx))
            vy = vy_b * speed

            vy = max(-MAX_LATERAL_SPEED,
                     min(MAX_LATERAL_SPEED, vy + wall_vy))

            ye = self._wrap_angle(math.atan2(dy, dx) - yaw)
            wz = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, 2.0 * ye))

            self._info(
                f'FW-VEL: vx_b={vx_b:.3f} vy_b={vy_b:.3f} speed={speed:.3f} '
                f'front={front:.3f}m front_scale={front_scale:.3f} '
                f'wall_vx={wall_vx:.3f} wall_vy={wall_vy:.3f} '
                f'vx={vx:.3f} vy={vy:.3f} wz={wz:.3f} yaw={yaw:.3f}rad '
                f'right={self._right_range():.3f}m left={self._left_range():.3f}m')

            self._publish_vel(x=vx, y=vy, wz=wz)
        else:
            self._info('FW: current_wp is None and no waypoints — publishing zero vel')
            self._publish_vel()
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Wall avoidance helpers
    # ══════════════════════════════════════════════════════════════════════════

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
    # Coordinate memory / scoring
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

    def _peer_gradient_modifier(self, wx, wy):
        """Return a signed score modifier based on distance to all active peer
        goals.  Positive = bonus (frontier is far from peers, good to explore).
        Negative = penalty (frontier is close to a peer's goal, avoid overlap).

        Uses a tanh S-curve per peer:
            modifier = PEER_GRADIENT_WEIGHT * tanh(dist / PEER_GRADIENT_SCALE - 1.0)

        At dist == PEER_GRADIENT_SCALE the contribution is zero.
        Contributions from multiple peers are summed, so a frontier equidistant
        from two peers that are both nearby gets a stronger penalty.

        Stale claims older than PEER_CLAIM_TIMEOUT are skipped so a crashed or
        landed drone does not permanently affect scoring."""
        now = self.get_clock().now().nanoseconds * 1e-9
        modifier = 0.0
        for gx, gy, t in self.peer_goals.values():
            if (now - t) > PEER_CLAIM_TIMEOUT:
                continue
            dist = math.hypot(wx - gx, wy - gy)
            modifier += PEER_GRADIENT_WEIGHT * math.tanh(
                dist / max(PEER_GRADIENT_SCALE, 1e-3) - 1.0)
        return modifier

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
        self._info(
            f'STUCK-CHK: moved={moved:.3f}m since last progress '
            f'(threshold={STUCK_PROGRESS_DIST}m) '
            f'time_without_progress={now - self.last_progress_time:.1f}s '
            f'(timeout={STUCK_TIMEOUT}s)')
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
        self.no_path_failures = 0
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

        if self.goal is not None:
            dist_to_goal = math.hypot(
                self.goal[0] - self.position[0],
                self.goal[1] - self.position[1])
            goal_s = f'{dist_to_goal:.2f}m {len(self.waypoints)}wp'
        else:
            goal_s = 'none'

        self._info(
            f'F={self._front_range():.2f} R={self._right_range():.2f} L={self._left_range():.2f} | '
            f'goal={goal_s} | '
            f't={elapsed:.1f}s')
        self._publish_drone_marker()

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
        """
        Returns (vx, vy, speed_scale) — pure push-away on all four axes.

        Centering has been removed: A* already routes through corridor centres
        via the proximity cost map, so adding a centering term here only
        introduces lateral oscillation without improving clearance.

        vx          — axial push-away (negative = retreat from front wall,
                      positive = retreat from back wall).
        vy          — lateral push-away (positive = away from right wall,
                      negative = away from left wall).
        speed_scale — reduced to 0.75 when any side wall is within
                      WALL_SAFE_DIST, giving the drone a little more reaction
                      time in tight spaces.
        """
        if len(self.ranges) < 4:
            return 0.0, 0.0, 1.0

        r, l = self._side_ranges_for_control()
        vy = 0.0
        speed_scale = 1.0

        # Lateral push-away
        if r < WALL_PUSH_DIST:
            vy += min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - r))
        if l < WALL_PUSH_DIST:
            vy -= min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - l))

        # Slow down when a side wall is close
        if min(r, l) < WALL_SAFE_DIST:
            speed_scale = 0.75

        # Axial push-away (front/back)
        front = self._front_range()
        back  = (self.ranges[0]
                 if len(self.ranges) > 0 and self.ranges[0] > 0.0
                 else 999.0)
        vx = 0.0
        if front < WALL_PUSH_DIST:
            vx -= min(CRUISE_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - front))
        if back < WALL_PUSH_DIST:
            vx += min(CRUISE_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - back))

        vy = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, vy))
        vx = max(-CRUISE_SPEED, min(CRUISE_SPEED, vx))
        speed_scale = max(0.45, min(1.0, speed_scale))
        return vx, vy, speed_scale

    def _get_wall_correction(self):
        _, vy, _ = self._get_wall_guidance()
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
        """
        Build a boolean passability array for A*.

        Cells are impassable when:
          - value == 100  (known occupied / wall)
          - value == -1   (unknown)

        Treating unknown cells as impassable means A* only routes through
        confirmed free space.  On a multi-ranger deck with 4 fixed sensors
        there is no diagonal coverage, so unknown cells adjacent to the path
        can hide walls the drone will not detect until it is already on top of
        them.  Blocking unknown cells forces the path to stay in mapped space.
        """
        W, H     = self.map_width, self.map_height
        passable = np.ones(W * H, dtype=bool)
        passable[self.map_data == 100] = False
        passable[self.map_data == -1]  = False
        if inflation > 0:
            for idx in np.where(self.map_data == 100)[0]:
                r, c = divmod(int(idx), W)
                r0, r1 = max(0, r - inflation), min(H - 1, r + inflation)
                c0, c1 = max(0, c - inflation), min(W - 1, c + inflation)
                for rr in range(r0, r1 + 1):
                    passable[rr * W + c0: rr * W + c1 + 1] = False
        return passable

    def _build_proximity_cost(self, radius=None):
        """
        Build a per-cell proximity cost array using BFS outward from every
        wall and unknown cell.

        radius defaults to PROXIMITY_COST_RADIUS but callers can pass a
        smaller value so the gradient fills the actual passable corridor
        rather than starting from inside the inflated wall boundary.
        """
        if radius is None:
            radius = PROXIMITY_COST_RADIUS
        W, H = self.map_width, self.map_height
        dist_arr = np.full(W * H, -1, dtype=np.int32)
        mask = (self.map_data == 100) | (self.map_data == -1)
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

        cost = np.zeros(W * H, dtype=np.float32)
        reachable = dist_arr > 0
        cost[reachable] = PROXIMITY_COST_WEIGHT * np.maximum(
            0.0, 1.0 - dist_arr[reachable] / radius)
        return cost

    def _plan_path_to_goal(self):
        self.waypoints = []
        self.current_wp = None
        self.needs_replan = False
        self.last_replan_pos  = None
        self.last_path_cost   = None
        if self.map_data is None or self.goal is None:
            self._warn('A*: no map or goal')
            return

        sr, sc = self._world_to_grid(self.position[0], self.position[1])
        gr, gc = self._world_to_grid(self.goal[0], self.goal[1])

        # Bug 1 fix: clamp BOTH start and goal to grid bounds.
        # Previously only goal was clamped — an out-of-bounds start causes
        # A* to silently fail with "no path found" on every call.
        sr = max(0, min(self.map_height - 1, sr))
        sc = max(0, min(self.map_width  - 1, sc))
        gr = max(0, min(self.map_height - 1, gr))
        gc = max(0, min(self.map_width  - 1, gc))

        self._info(
            f'A*: ({sr},{sc})→({gr},{gc}) frontier=({self.goal[0]:.2f},{self.goal[1]:.2f})')

        nb8 = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
               (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]
        found = False; path_cost = None

        for inflation in range(WALL_INFLATION_CELLS, 0, -1):
            if inflation < WALL_INFLATION_CELLS:
                self._warn(f'A*: reduced inflation={inflation}')

            passable = self._build_inflated_map(inflation)
            W, H     = self.map_width, self.map_height

            # Bug 2 fix: force start cell passable, not just goal.
            # If the drone sits inside an inflation zone A* would expand from
            # an impassable cell, producing an invalid first path segment.
            if 0 <= sr < H and 0 <= sc < W:
                passable[sr * W + sc] = True
            # Force goal passable so A* can always reach the standoff point.
            if 0 <= gr < H and 0 <= gc < W:
                passable[gr * W + gc] = True

            # Bug 3+5 fix: rebuild proximity cost per inflation level so the
            # gradient correctly reflects the passable region at this inflation.
            # effective_radius shrinks with inflation so the gradient fills the
            # corridor from the passable boundary inward to the centre.
            effective_radius = max(1, PROXIMITY_COST_RADIUS - inflation)
            prox_cost = self._build_proximity_cost(effective_radius)

            open_heap = [(0.0, 0.0, sr, sc)]
            came_from = {}; g_score = {(sr, sc): 0.0}; found = False

            while open_heap:
                _, g, row, col = heapq.heappop(open_heap)
                if (row, col) == (gr, gc):
                    found = True; path_cost = g; break
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
                        # Plain Euclidean heuristic — admissible because
                        # minimum step cost is 1.0 (cardinal, zero proximity)
                        h = math.hypot(nr - gr, nc - gc)
                        heapq.heappush(open_heap, (ng + h, ng, nr, nc))
            if found:
                if inflation < WALL_INFLATION_CELLS:
                    self._warn(f'A*: tight path inflation={inflation}')
                break

        if not found:
            self._warn('A*: no path found')
            return

        # Bug 4 fix: include the start cell in the path.
        # The original trace-back loop stopped at the cell whose predecessor
        # is the start, leaving (sr, sc) out of the path entirely.  With
        # WAYPOINT_SPACING=15 this could skip the first section of the path.
        path = []
        cell = (gr, gc)
        while cell in came_from:
            path.append(cell)
            cell = came_from[cell]
        path.append((sr, sc))   # add start cell
        path.reverse()

        wps = [self._grid_to_world(r, c)
               for i, (r, c) in enumerate(path)
               if i % WAYPOINT_SPACING == 0 or i == len(path) - 1]

        # Trim the last STANDOFF_WAYPOINTS waypoints so the drone stops
        # short of the frontier boundary. This replaces the old straight-line
        # walkback (_safe_nav_goal) which crossed walls in some cases.
        # At WAYPOINT_SPACING=2 and MAP_RES=0.1m, trimming 2 waypoints gives
        # ~0.4m standoff, always on the correct side of any wall.
        # Skip the trim when returning home — we want exact coordinates.
        if not self.navigating_home and len(wps) > STANDOFF_WAYPOINTS + 1:
            wps = wps[:-STANDOFF_WAYPOINTS]

        self.waypoints      = wps
        self.last_path_cost = path_cost
        self._info(
            f'A*: {len(path)} cells → {len(wps)} WPs cost={path_cost:.2f}')
        self._publish_waypoint_markers(wps)
        if self.goal is not None:
            self._publish_goal_marker(self.goal[0], self.goal[1])
    

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

        # Try the preferred minimum size first; relax to the absolute floor
        # if nothing passes so early-exploration sparse maps still get clusters.
        valid = self._score_clusters(clusters, MIN_FRONTIER_DIST, MIN_CLUSTER_SIZE, now)
        if not valid:
            valid = self._score_clusters(clusters, MIN_FRONTIER_DIST, MIN_VALID_CLUSTER_SIZE, now)
        return valid

    def _score_clusters(self, clusters, min_dist, min_size, now):
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

            # Snap centroid to the nearest free cell in the cluster if the
            # raw average lands on a wall or unknown cell. Frontier cells
            # border walls by definition, so the cluster average is often
            # pulled onto or into the wall itself. Snapping ensures the goal
            # handed to A* is always confirmed free space.
            if (self.map_data is not None and
                    not (0 <= row < self.map_height and
                        0 <= col < self.map_width and
                        self.map_data[row * self.map_width + col] == 0)):
                best_r, best_c, best_d = row, col, float('inf')
                for cr, cc in cluster:
                    if self.map_data[cr * self.map_width + cc] == 0:
                        d = (cr - avg_r)**2 + (cc - avg_c)**2
                        if d < best_d:
                            best_d = d
                            best_r, best_c = cr, cc
                row, col = best_r, best_c

            wx = self.map_origin[0] + (col+0.5)*MAP_RES
            wy = self.map_origin[1] + (row+0.5)*MAP_RES
            dist = math.hypot(wx-px, wy-py)
            if dist < min_dist:
                continue
            total_unk, new_unk = self._count_unknown_near_cluster(cluster)
            # Use total_unk so clusters are not killed just because their
            # adjacent unknowns were already marked by the scan coverage tracker.
            if total_unk * len(cluster) < 2:
                continue
            path_cost = self._estimate_path_cost(wx, wy)
            penalty   = (self._visited_penalty(row, col) +
                         self._recent_goal_penalty(wx, wy))
            peer_mod  = self._peer_gradient_modifier(wx, wy)
            score = (SIZE_WEIGHT * n + UNKNOWN_WEIGHT * new_unk -
                    DISTANCE_WEIGHT * path_cost - penalty + peer_mod)
            valid.append((wx, wy, dist, n,
                        new_unk, total_unk,
                        penalty, score, path_cost))
        valid.sort(key=lambda c: c[7], reverse=True)
        return valid

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

    def _publish_drone_marker(self):
        """
        Publish a cyan arrow marker at the drone's current position so it
        is visible in RViz without needing a URDF or TF setup.
        In RViz: Add → By topic → /drone_pose → Marker.
        Fixed frame must be 'map'.
        """
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
        # Encode yaw into the quaternion so the arrow points in the
        # drone's heading direction.
        yaw = self.angles[2]
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x = 0.30   # arrow length
        m.scale.y = 0.08   # arrow width
        m.scale.z = 0.08   # arrow height
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 1.0  # cyan
        self.drone_marker_pub.publish(m)

    def _publish_goal_marker(self, x, y, home=False):
        """
        Publish a sphere marker at (x, y) to /exploration_goal.
        Green = frontier goal.  Blue = return-home goal.
        In RViz: Add → By topic → /exploration_goal → Marker.
        Fixed frame must be 'map'.
        """
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'exploration'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = float(TAKEOFF_HEIGHT)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        if home:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.4, 1.0, 1.0
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 1.0
        self.marker_pub.publish(m)

    def _publish_possible_goal_marker(self, x, y):
        """
        Publish an orange sphere marker at (x, y) to /possible_goal.
        Shown every time a candidate frontier goal is selected for path
        planning, before the A* result is known.
        In RViz: Add > By topic > /possible_goal > Marker.
        Fixed frame must be 'map'.
        """
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'possible_goal'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = float(TAKEOFF_HEIGHT)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.30
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0  # orange
        self.possible_goal_marker_pub.publish(m)

    def _publish_failed_goal_marker(self, x, y):
        """
        Publish a red sphere marker at (x, y) to /failed_goal.
        Shown only when A* cannot find any path to the selected frontier.
        In RViz: Add > By topic > /failed_goal > Marker.
        Fixed frame must be 'map'.
        """
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'failed_goal'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = float(TAKEOFF_HEIGHT)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.30
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0  # red
        self.failed_goal_marker_pub.publish(m)

    def _publish_waypoint_markers(self, waypoints):
        """
        Publish all current A* waypoints to /waypoints as red spheres.
        In RViz: Add → By topic → /waypoints → MarkerArray.
        Fixed frame must be 'map'.
        A DELETE_ALL marker is sent first so stale waypoints from the
        previous plan are cleared before the new ones appear.
        """
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