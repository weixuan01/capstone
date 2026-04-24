#!/usr/bin/env python3

"""
Object-Detection Scanner Drone - works with ObjectDetectionPlanner
===================================================================
State machine:
  TAKEOFF      - take off and hover
  WAIT_GOAL    - hover and wait for /{prefix}/assigned_scan_point
  NAVIGATE     - follow A* waypoints to the assigned scan position
  SPINNING     - 360-degree spin at scan position (object detection)
  DONE         - return to start via A* and land (triggered by recall)
  LANDING      - descend

Relationship to explorer_drone.py
---------------------------------
This node is structurally identical to the frontier-exploration navigator:
same recall/land protocol, same goal_status reporting, same A* planner,
same wall guidance.  The differences are:

  - Listens on /{prefix}/assigned_scan_point (from ObjectDetectionPlanner)
    instead of /{prefix}/assigned_goal.
  - No initial seed-spin after takeoff - scanner drones assume the map
    already exists (built by frontier-exploration drones).  They go
    straight to WAIT_GOAL.
  - On arrival at a goal, performs a full 360-degree spin, not a short
    scanning spin.  "REACHED" is only reported after the spin completes.

Recall / Land protocol (requirement 3)
--------------------------------------
Exactly matches explorer_drone.py:
  x=NaN, z=0.0  - RECALL: return home via DONE state, then land
  x=NaN, z=1.0  - LAND:   immediate descent
On receipt of either, the drone publishes "RECALLED" on goal_status
so the planner removes it from the assignment pool permanently.

The mission-control dashboard publishes these NaN messages via the
shared mission_control node, which sends them on the frontier goal
topic /{prefix}/assigned_goal - not /{prefix}/assigned_scan_point.
To keep the Recall/Land buttons working for scanner drones without
touching mission_control, this node additionally subscribes to
/{prefix}/assigned_goal and routes any NaN Point it sees there into
the same _goal_cb handler.  Real-valued goals on /assigned_goal are
ignored, since those are for the frontier explorer.

Shared map contribution (requirement 5)
---------------------------------------
This drone runs the multiranger sensor if its deck is fitted and
contributes to the shared map continuously, same as frontier-exploration
drones.  Mapping is always on - no gating flag.  The shared_mapper node
subscribes to /{prefix}/scan separately and processes those scans the
whole time the drone is in the air.

Digit markers from AI deck
--------------------------
If the drone is fitted with an AI deck running aideck_udp_streamer, that
streamer publishes Int32 digit predictions on
/{prefix}/aideck/digit_prediction whenever its ONNX model recognises a
digit with confidence above its threshold.  This node pairs each
prediction with the current front-multiranger hit to estimate where on
the wall the digit is, then plants a TEXT_VIEW_FACING marker in the map
frame at that location, publishing the accumulating list on
/{prefix}/digit_markers for RViz.  Markers are suppressed during
TAKEOFF and LANDING (unstable ranger readings) and deduplicated within
a DIGIT_DEDUP_DIST radius to avoid stacking duplicates when the same
wall digit is seen repeatedly during a 360 deg spin.

Battery voltage (requirement 4)
-------------------------------
Battery is handled exactly like frontier-exploration drones - the
cflib backend publishes /{prefix}/status (crazyflie_interfaces/Status,
configured in crazyflies.yaml with firmware_logging.default_topics.status).
The standalone battery_monitor node subscribes to /{prefix}/status for
every drone in the fleet and displays voltage on the mission-control
dashboard.  No per-drone code is needed here.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int32
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int32
from builtin_interfaces.msg import Duration

import tf_transformations
import math
import heapq
import numpy as np
from enum import Enum, auto
from collections import deque

# ── Map constants ─────────────────────────────────────────────────────────────
MAP_RES = 0.1

# ── Flight parameters ─────────────────────────────────────────────────────────
TAKEOFF_HEIGHT     = 0.3
TAKEOFF_DELAY      = 3.0
CRUISE_SPEED       = 0.3
MAX_TURN_RATE      = 0.5
OBSTACLE_DIST      = 0.25
GOAL_REACHED_DIST  = 0.15
REPLAN_COOLDOWN    = 5.0
WALL_INFLATION_CELLS  = 2
PROXIMITY_COST_WEIGHT = 2
PROXIMITY_COST_RADIUS = 5
WAYPOINT_SPACING      = 2

# ── Wall avoidance ────────────────────────────────────────────────────────────
WALL_PUSH_DIST      = 0.2
WALL_SAFE_DIST      = 0.2
WALL_FILTER_ALPHA   = 0.3
WALL_KP_SAFETY      = 0.3
MAX_LATERAL_SPEED   = 0.24

# ── Spin ──────────────────────────────────────────────────────────────────────
SPIN_RATE   = 0.5
SPIN_TARGET = 2.0 * math.pi   # full 360 for object detection coverage

# ── Stuck detector ────────────────────────────────────────────────────────────
STUCK_PROGRESS_DIST       = 0.10
STUCK_TIMEOUT             = 15.0
MAX_STUCK_EVENTS_PER_GOAL = 4
MAX_REPLANS_PER_GOAL      = 6

# ── Digit marker placement ────────────────────────────────────────────────────
# Predictions closer than this distance to an existing marker are suppressed
# to avoid stacking multiple labels at the same wall digit.
DIGIT_DEDUP_DIST = 0.20   # metres
# Camera + multiranger max useful range for pairing predictions with walls.
# Readings beyond this distance are treated as "no wall in front" and the
# digit prediction is dropped rather than placed at max-range.
DIGIT_RANGE_MAX  = 3.5    # metres


class State(Enum):
    TAKEOFF   = auto()
    WAIT_GOAL = auto()
    NAVIGATE  = auto()
    SPINNING  = auto()
    DONE      = auto()
    LANDING   = auto()


class ObjectDetectionScanner(Node):

    def __init__(self):
        super().__init__('object_detection_scanner')

        self.declare_parameter('robot_prefix', '/cf1')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.robot_prefix = robot_prefix

        # ── Internal state ────────────────────────────────────────────────────
        self.state    = State.TAKEOFF
        self.position = [0.0, 0.0, 0.0]
        self.angles   = [0.0, 0.0, 0.0]
        self.ranges   = [0.0, 0.0, 0.0, 0.0]

        self.map_data   = None
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.goal             = None   # (x, y) from planner
        self.waypoints        = []
        self.current_wp       = None
        self.needs_replan     = False
        self.last_replan_time = 0.0
        self.last_replan_pos  = None
        self.navigating_home  = False

        # Home location (requirement 3) - captured at end of TAKEOFF
        self.start_pos  = None
        self.start_time = None

        # Spin state (full 360)
        self.spin_start_yaw      = None
        self.spin_total_rotation = 0.0
        self._last_spin_yaw      = 0.0

        # Wall-filter state
        self.filtered_right = None
        self.filtered_left  = None

        # Stuck / replan bookkeeping
        self.goal_start_time       = 0.0
        self.last_progress_pos     = None
        self.last_progress_time    = 0.0
        self.stuck_events_for_goal = 0
        self.replan_count_for_goal = 0

        self.position_received = False
        self.map_received      = False
        self._going_home       = False  # set True on RECALL or LAND

        self.scanning_start_time  = None
        self.last_status_log_time = 0.0

        # ── Digit marker state ────────────────────────────────────────────────
        # The AI deck streamer publishes digit predictions on
        # /{prefix}/aideck/digit_prediction.  We stash the latest prediction
        # here and, on each scan callback (while the drone is airborne in a
        # stable state), pair it with the front-multiranger hit to estimate
        # where on the wall the digit is.  See _maybe_place_digit_marker.
        self.pending_digit             = None
        self.placed_digit_positions    = []   # [(wx, wy), ...] for dedup
        self.digit_markers             = []   # accumulating list of Marker
        self.next_digit_marker_id      = 0

        # ── ROS interface ─────────────────────────────────────────────────────
        self.create_subscription(
            Odometry, robot_prefix + '/odom', self._odom_cb, 10)
        self.create_subscription(
            LaserScan, robot_prefix + '/scan', self._scan_cb, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)

        # Goal from planner - NOTE different topic name from explorer_drone
        self.create_subscription(
            Point, robot_prefix + '/assigned_scan_point', self._goal_cb, 10)

        # Recall / land compatibility subscription.
        # The mission_control node publishes NaN Point messages to
        # /{prefix}/assigned_goal (the frontier-exploration topic) for both
        # the Recall and Land buttons on the mission-control dashboard.
        # Scanner drones subscribe here too so those same buttons work for
        # them.  Only NaN messages are forwarded - real-valued goals on this
        # topic are silently ignored, since those are meant for a frontier
        # explorer, not a scanner.
        self.create_subscription(
            Point, robot_prefix + '/assigned_goal',
            self._recall_compat_cb, 10)

        # Digit predictions from the AI deck streamer.
        # The streamer publishes an Int32 on /{prefix}/aideck/digit_prediction
        # whenever its ONNX model sees a digit with confidence above its
        # threshold.  We stash the latest one and the scan callback decides
        # whether and where to plant it as a marker on the shared map.
        self.create_subscription(
            Int32, '/aideck/digit_prediction',
            self._digit_prediction_cb, 10)

        self.cmd_pub          = self.create_publisher(
            Twist, robot_prefix + '/cmd_vel_raw', 10)
        self.status_pub       = self.create_publisher(
            String, robot_prefix + '/goal_status', 10)
        self.marker_pub       = self.create_publisher(
            Marker, robot_prefix + '/scan_goal', 10)
        self.wp_marker_pub    = self.create_publisher(
            MarkerArray, robot_prefix + '/waypoints', 10)
        self.drone_marker_pub = self.create_publisher(
            Marker, robot_prefix + '/drone_pose', 10)
        self.digit_marker_pub = self.create_publisher(
            MarkerArray, robot_prefix + '/digit_markers', 10)

        self.create_service(
            Trigger, robot_prefix + '/stop_exploration', self._stop_cb)
        self.timer = self.create_timer(0.1, self._timer_cb)

        self._publish_vel(z=TAKEOFF_HEIGHT)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'ObjectDetectionScanner started. prefix={robot_prefix}')

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.angles = list(tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]))
        if not self.position_received:
            self.position_received = True
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = now

    def _scan_cb(self, msg):
        self.ranges = list(msg.ranges)
        right = self.ranges[1] if len(self.ranges) > 1 else 0.0
        left  = self.ranges[3] if len(self.ranges) > 3 else 0.0
        if right > 0.0:
            self.filtered_right = (right if self.filtered_right is None else
                WALL_FILTER_ALPHA * right +
                (1 - WALL_FILTER_ALPHA) * self.filtered_right)
        if left > 0.0:
            self.filtered_left = (left if self.filtered_left is None else
                WALL_FILTER_ALPHA * left +
                (1 - WALL_FILTER_ALPHA) * self.filtered_left)

        # Pair any pending digit prediction with the current front-ranger hit.
        # This runs on every scan message; the placement function itself
        # decides whether to actually plant a marker based on state and dedup.
        self._maybe_place_digit_marker()

    def _digit_prediction_cb(self, msg: Int32):
        """
        Stash the latest digit prediction from the AI deck streamer.
        The scan callback pairs it with the front-ranger reading and
        decides whether to plant a marker.
        """
        self.pending_digit = int(msg.data)

    def _map_cb(self, msg):
        self.map_data   = np.array(msg.data, dtype=np.int8)
        self.map_width  = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y]
        self.map_received = True

    def _goal_cb(self, msg: Point):
        """
        Three cases (identical protocol to explorer_drone):
          x=NaN, z=0.0 - RECALL: return home then land
          x=NaN, z=1.0 - LAND:   immediate descent
          x=real       - normal scan-point assignment
        """
        # ── Recall or land-in-place ───────────────────────────────────────────
        if math.isnan(msg.x):
            self._going_home = True
            self._report_status('RECALLED')

            self.goal         = None
            self.waypoints    = []
            self.current_wp   = None
            self.needs_replan = False

            if msg.z == 1.0:
                self.get_logger().info(
                    'Land-in-place command received. Descending now.')
                self.state = State.LANDING
            else:
                self.get_logger().info(
                    'Recall command received. Returning home.')
                self.state = State.DONE
            return

        # ── Ignore further goals if already going home ────────────────────────
        if self._going_home:
            return

        # ── Normal scan-point assignment ──────────────────────────────────────
        new_goal = (msg.x, msg.y)
        if self.goal == new_goal:
            return  # echoed back - ignore
        now = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'New scan point assigned: ({new_goal[0]:.2f},{new_goal[1]:.2f})')
        self.goal                  = new_goal
        self.waypoints             = []
        self.current_wp            = None
        self.needs_replan          = False
        self.last_replan_time      = 0.0
        self.last_replan_pos       = None
        self.goal_start_time       = now
        self.last_progress_pos     = (self.position[0], self.position[1])
        self.last_progress_time    = now
        self.stuck_events_for_goal = 0
        self.replan_count_for_goal = 0

        # Only trigger A* immediately if we're airborne and idle or moving.
        # During TAKEOFF the state machine picks it up on entering WAIT_GOAL.
        if self.state in (State.WAIT_GOAL, State.NAVIGATE):
            self._plan_path_to_goal()
            if self.waypoints:
                self.state = State.NAVIGATE
                self.get_logger().info(
                    f'Path planned: {len(self.waypoints)} waypoints.')
            else:
                self.get_logger().warn(
                    'A* failed for assigned scan point - reporting FAILED.')
                self._report_status('FAILED')
                self.goal  = None
                self.state = State.WAIT_GOAL

    def _recall_compat_cb(self, msg: Point):
        """
        Handles NaN Point messages arriving on /{prefix}/assigned_goal.

        The mission-control dashboard's Recall and Land buttons flow
        through mission_control, which publishes NaN Points to the
        frontier-exploration goal topic.  Scanner drones don't otherwise
        listen on that topic, so this small compatibility subscription
        catches the recall/land command and delegates to the normal
        _goal_cb handler, which already knows how to process NaN
        messages (x=NaN, z=0.0 = recall; x=NaN, z=1.0 = land in place).

        Real-valued goals on this topic are silently ignored - those are
        intended for frontier explorers, not scanner drones.
        """
        if math.isnan(msg.x):
            self._goal_cb(msg)

    def _stop_cb(self, request, response):
        self.get_logger().info('Stop requested - landing.')
        self.timer.cancel()
        self._publish_vel(z=-0.2)
        response.success = True
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # State machine
    # ══════════════════════════════════════════════════════════════════════════

    def _timer_cb(self):
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._state_machine(now)
        except Exception as e:
            import traceback
            self.get_logger().error(f'Crash: {e}\n{traceback.format_exc()}')
            self._publish_vel()
            self.timer.cancel()

    def _state_machine(self, now):
        self._log_status(now)

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self.state == State.TAKEOFF:
            self._publish_vel(z=TAKEOFF_HEIGHT)
            airborne = self.position[2] >= TAKEOFF_HEIGHT * 0.8
            if airborne and now - self.start_time > TAKEOFF_DELAY:
                self.start_pos = [self.position[0], self.position[1]]
                self.get_logger().info(
                    f'Takeoff complete. '
                    f'Home: ({self.start_pos[0]:.3f},{self.start_pos[1]:.3f})')
                self.scanning_start_time = now
                # No seed spin - scanner drones assume a pre-built map.
                self.state = State.WAIT_GOAL

        # ── WAIT_GOAL ─────────────────────────────────────────────────────────
        elif self.state == State.WAIT_GOAL:
            self._publish_vel(y=self._wall_correction())
            if self.goal is not None:
                self._plan_path_to_goal()
                if self.waypoints:
                    self.state = State.NAVIGATE
                else:
                    self.get_logger().warn('A* failed - reporting FAILED.')
                    self._report_status('FAILED')
                    self.goal = None

        # ── NAVIGATE ──────────────────────────────────────────────────────────
        elif self.state == State.NAVIGATE:
            if self.goal is None:
                self.state = State.WAIT_GOAL
                return

            # Layer 1 - stuck detection
            if self._is_stuck(now):
                self.stuck_events_for_goal += 1
                self.get_logger().warn(
                    f'Stuck {self.stuck_events_for_goal}/{MAX_STUCK_EVENTS_PER_GOAL}')
                if self.stuck_events_for_goal >= MAX_STUCK_EVENTS_PER_GOAL:
                    self.get_logger().warn('Too many stuck events - FAILED.')
                    self._report_status('FAILED')
                    self._clear_goal()
                    return
                self._execute_replan(now)
                return

            # Layer 2 - replan flag
            if self.needs_replan:
                self.replan_count_for_goal += 1
                if self.replan_count_for_goal > MAX_REPLANS_PER_GOAL:
                    self.get_logger().warn('Too many replans - FAILED.')
                    self._report_status('FAILED')
                    self._clear_goal()
                    return
                cg = self._world_to_grid(self.position[0], self.position[1])
                if (self.last_replan_pos is not None and
                        abs(cg[0]-self.last_replan_pos[0]) <= 1 and
                        abs(cg[1]-self.last_replan_pos[1]) <= 1):
                    self.get_logger().warn('Replanned but no movement - FAILED.')
                    self._report_status('FAILED')
                    self._clear_goal()
                    return
                self.last_replan_pos = cg
                self._execute_replan(now)
                return

            # Layer 3 - safety replan
            front = self._front_range()
            if (0.0 < front < OBSTACLE_DIST and
                    not self.needs_replan and
                    (now - self.last_replan_time) > REPLAN_COOLDOWN):
                self._execute_replan(now)
                return

            # Layer 4 - waypoint following
            if not self.waypoints and self.current_wp is None:
                self._on_arrival()
                return

            if self._follow_waypoints(now):
                self._on_arrival()

        # ── SPINNING ──────────────────────────────────────────────────────────
        elif self.state == State.SPINNING:
            self._publish_vel(wz=SPIN_RATE)
            yaw_delta = self._wrap_angle(
                self.angles[2] - self._last_spin_yaw)
            self._last_spin_yaw       = self.angles[2]
            self.spin_total_rotation += abs(yaw_delta)

            if self.spin_total_rotation >= SPIN_TARGET:
                self._publish_vel()
                self.get_logger().info(
                    f'360 spin complete at '
                    f'({self.position[0]:.2f},{self.position[1]:.2f}).')
                # Now - and only now - report REACHED so the planner marks
                # the disc as scanned.
                self._report_status('REACHED')
                self._clear_goal()

        # ── DONE (return to start via A*) ─────────────────────────────────────
        elif self.state == State.DONE:
            if self.start_pos is None:
                self.state = State.LANDING
                return
            home = (self.start_pos[0], self.start_pos[1])
            if math.hypot(self.position[0]-home[0],
                          self.position[1]-home[1]) < GOAL_REACHED_DIST:
                self.state = State.LANDING
                return
            self.goal             = home
            self.navigating_home  = True
            self.waypoints        = []
            self.current_wp       = None
            self.needs_replan     = False
            self.last_replan_time = 0.0
            self.last_replan_pos  = None
            self.goal_start_time  = now
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = now
            self.stuck_events_for_goal = 0
            self.replan_count_for_goal = 0
            self._plan_path_to_goal()
            if not self.waypoints:
                self.state = State.LANDING
            else:
                self.state = State.NAVIGATE

        # ── LANDING ───────────────────────────────────────────────────────────
        elif self.state == State.LANDING:
            self._publish_vel(z=-0.2)
            if self.position[2] < 0.1:
                self._going_home = False
                self.timer.cancel()
                self._publish_vel()
                self.get_logger().info('Landed.')

    # ══════════════════════════════════════════════════════════════════════════
    # Arrival handling
    # ══════════════════════════════════════════════════════════════════════════

    def _on_arrival(self):
        """Called when waypoints are exhausted - drone has reached its goal."""
        if self.navigating_home:
            # Arrived home (recall flow); drop into LANDING from the state machine.
            self.navigating_home = False
            self.state = State.LANDING
            return

        # Arrived at a scan point - perform the full 360 spin.
        self.get_logger().info(
            f'Arrived at scan point '
            f'({self.goal[0]:.2f},{self.goal[1]:.2f}). Starting 360 spin.')
        self.spin_start_yaw      = self.angles[2]
        self.spin_total_rotation = 0.0
        self._last_spin_yaw      = self.angles[2]
        self.state = State.SPINNING

    # ══════════════════════════════════════════════════════════════════════════
    # Navigation helpers  (identical to explorer_drone.py)
    # ══════════════════════════════════════════════════════════════════════════

    def _execute_replan(self, now):
        self.needs_replan     = False
        self.last_replan_time = now
        self._plan_path_to_goal()
        if not self.waypoints:
            self.get_logger().warn('Replan failed - reporting FAILED.')
            self._report_status('FAILED')
            self._clear_goal()

    def _clear_goal(self):
        self.goal         = None
        self.waypoints    = []
        self.current_wp   = None
        self.needs_replan = False
        self.last_replan_pos = None
        self.state        = State.WAIT_GOAL

    def _report_status(self, status: str):
        msg      = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _is_stuck(self, now):
        if self.last_progress_pos is None:
            self.last_progress_pos  = (self.position[0], self.position[1])
            self.last_progress_time = now
            return False
        if (now - self.last_replan_time) < 6.0:
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

    def _follow_waypoints(self, now):
        if self.current_wp is not None:
            dx   = self.current_wp[0] - self.position[0]
            dy   = self.current_wp[1] - self.position[1]
            dist = math.hypot(dx, dy)
            if dist < GOAL_REACHED_DIST:
                if self.waypoints:
                    self.current_wp = self.waypoints.pop(0)
                    self.replan_count_for_goal = 0
                else:
                    self.current_wp = None
                    return True
        elif self.waypoints:
            self.current_wp = self.waypoints.pop(0)

        if self.current_wp is not None:
            dx   = self.current_wp[0] - self.position[0]
            dy   = self.current_wp[1] - self.position[1]
            dist = math.hypot(dx, dy)
            yaw  = self.angles[2]

            vx_b =  math.cos(yaw)*dx + math.sin(yaw)*dy
            vy_b = -math.sin(yaw)*dx + math.cos(yaw)*dy

            if self.waypoints:
                speed = CRUISE_SPEED / max(dist, 1e-3)
            else:
                # Ramp down into the scan point - no standoff trimming here,
                # arrive exactly at the centre.
                speed = min(CRUISE_SPEED, dist) / max(dist, 1e-3)

            wall_vx, wall_vy, _ = self._get_wall_guidance()
            front = self._front_range()
            if front < OBSTACLE_DIST:
                front_scale = max(0.0, (front - WALL_PUSH_DIST) /
                                  max(OBSTACLE_DIST - WALL_PUSH_DIST, 1e-3))
            else:
                front_scale = 1.0

            vx = vx_b * speed * front_scale + wall_vx
            vx = max(-CRUISE_SPEED, min(CRUISE_SPEED, vx))
            vy = max(-MAX_LATERAL_SPEED,
                     min(MAX_LATERAL_SPEED, vy_b * speed + wall_vy))
            ye = self._wrap_angle(math.atan2(dy, dx) - yaw)
            wz = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, 2.0 * ye))
            self._publish_vel(x=vx, y=vy, wz=wz)
        else:
            self._publish_vel()
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # Wall guidance
    # ══════════════════════════════════════════════════════════════════════════

    def _get_wall_guidance(self):
        if len(self.ranges) < 4:
            return 0.0, 0.0, 1.0
        r = self.filtered_right if self.filtered_right else self._right_range()
        l = self.filtered_left  if self.filtered_left  else self._left_range()
        vy = 0.0
        speed_scale = 1.0
        if r < WALL_PUSH_DIST:
            vy += min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - r))
        if l < WALL_PUSH_DIST:
            vy -= min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - l))
        if min(r, l) < WALL_SAFE_DIST:
            speed_scale = 0.75
        front = self._front_range()
        back  = (self.ranges[0] if len(self.ranges) > 0 and self.ranges[0] > 0.0
                 else 999.0)
        vx = 0.0
        if front < WALL_PUSH_DIST:
            vx -= min(CRUISE_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - front))
        if back < WALL_PUSH_DIST:
            vx += min(CRUISE_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - back))
        vy = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, vy))
        vx = max(-CRUISE_SPEED, min(CRUISE_SPEED, vx))
        return vx, vy, speed_scale

    def _wall_correction(self):
        _, vy, _ = self._get_wall_guidance()
        return vy

    # ══════════════════════════════════════════════════════════════════════════
    # A* path planning - unknown=free since the map is pre-built
    # ══════════════════════════════════════════════════════════════════════════

    def _world_to_grid(self, wx, wy):
        return (int((wy - self.map_origin[1]) / MAP_RES),
                int((wx - self.map_origin[0]) / MAP_RES))

    def _grid_to_world(self, row, col):
        return (self.map_origin[0] + (col + 0.5) * MAP_RES,
                self.map_origin[1] + (row + 0.5) * MAP_RES)

    def _build_inflated_map(self, inflation):
        """
        Unknown cells (-1) are treated as FREE here - scanner drones run
        on a pre-built map where unknowns are navigable passageway, not
        hidden walls.  Only cells explicitly marked 100 are blocked.
        This mirrors the original coverage_scanner; it differs from the
        explorer drone's A*, which treats unknown as blocked.
        """
        W, H     = self.map_width, self.map_height
        passable = np.ones(W * H, dtype=bool)
        passable[self.map_data == 100] = False
        if inflation > 0:
            for idx in np.where(self.map_data == 100)[0]:
                r, c = divmod(int(idx), W)
                r0, r1 = max(0, r-inflation), min(H-1, r+inflation)
                c0, c1 = max(0, c-inflation), min(W-1, c+inflation)
                for rr in range(r0, r1+1):
                    passable[rr*W+c0: rr*W+c1+1] = False
        return passable

    def _build_proximity_cost(self, radius):
        W, H     = self.map_width, self.map_height
        dist_arr = np.full(W * H, -1, dtype=np.int32)
        mask     = (self.map_data == 100)
        dist_arr[mask] = 0
        queue = deque(zip(*np.where(mask.reshape(H, W))))
        dirs4 = [(-1,0),(1,0),(0,-1),(0,1)]
        while queue:
            row, col = queue.popleft()
            d = dist_arr[row*W+col]
            if d >= radius:
                continue
            for dr, dc in dirs4:
                nr, nc = row+dr, col+dc
                if 0 <= nr < H and 0 <= nc < W and dist_arr[nr*W+nc] == -1:
                    dist_arr[nr*W+nc] = d + 1
                    queue.append((nr, nc))
        cost = np.zeros(W * H, dtype=np.float32)
        ok   = dist_arr > 0
        cost[ok] = PROXIMITY_COST_WEIGHT * np.maximum(
            0.0, 1.0 - dist_arr[ok] / radius)
        return cost

    def _plan_path_to_goal(self):
        self.waypoints  = []
        self.current_wp = None
        self.needs_replan    = False
        self.last_replan_pos = None
        if self.map_data is None or self.goal is None:
            return

        sr, sc = self._world_to_grid(self.position[0], self.position[1])
        gr, gc = self._world_to_grid(self.goal[0], self.goal[1])
        sr = max(0, min(self.map_height-1, sr))
        sc = max(0, min(self.map_width-1,  sc))
        gr = max(0, min(self.map_height-1, gr))
        gc = max(0, min(self.map_width-1,  gc))

        nb8 = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
               (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]

        found = False
        for inflation in range(WALL_INFLATION_CELLS, 0, -1):
            passable = self._build_inflated_map(inflation)
            W, H     = self.map_width, self.map_height
            if 0 <= sr < H and 0 <= sc < W:
                passable[sr*W+sc] = True
            if 0 <= gr < H and 0 <= gc < W:
                passable[gr*W+gc] = True

            eff_radius = max(1, PROXIMITY_COST_RADIUS - inflation)
            prox       = self._build_proximity_cost(eff_radius)

            open_heap = [(0.0, 0.0, sr, sc)]
            came_from = {}
            g_score   = {(sr, sc): 0.0}

            while open_heap:
                _, g, row, col = heapq.heappop(open_heap)
                if (row, col) == (gr, gc):
                    found = True
                    break
                if g > g_score.get((row, col), float('inf')):
                    continue
                for dr, dc, mc in nb8:
                    nr, nc = row+dr, col+dc
                    if not (0 <= nr < H and 0 <= nc < W):
                        continue
                    if not passable[nr*W+nc]:
                        continue
                    ng = g + mc + prox[nr*W+nc]
                    if ng < g_score.get((nr, nc), float('inf')):
                        g_score[(nr, nc)]   = ng
                        came_from[(nr, nc)] = (row, col)
                        h = math.hypot(nr-gr, nc-gc)
                        heapq.heappush(open_heap, (ng+h, ng, nr, nc))
            if found:
                break

        if not found:
            self.get_logger().warn('A*: no path found.')
            return

        path = []
        cell = (gr, gc)
        while cell in came_from:
            path.append(cell)
            cell = came_from[cell]
        path.append((sr, sc))
        path.reverse()

        # No STANDOFF trimming - scanner drones must arrive exactly at
        # the centre of the scan disc.
        wps = [self._grid_to_world(r, c)
               for i, (r, c) in enumerate(path)
               if i % WAYPOINT_SPACING == 0 or i == len(path)-1]

        self.waypoints = wps
        self.get_logger().info(
            f'A*: {len(path)} cells → {len(wps)} waypoints.')
        self._publish_waypoint_markers(wps)
        if self.goal:
            self._publish_goal_marker(self.goal[0], self.goal[1])

    # ══════════════════════════════════════════════════════════════════════════
    # Range helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _front_range(self):
        return (self.ranges[2] if len(self.ranges) > 2 and self.ranges[2] > 0.0
                else 999.0)

    def _right_range(self):
        return (self.ranges[1] if len(self.ranges) > 1 and self.ranges[1] > 0.0
                else 999.0)

    def _left_range(self):
        return (self.ranges[3] if len(self.ranges) > 3 and self.ranges[3] > 0.0
                else 999.0)

    def _wrap_angle(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    # ══════════════════════════════════════════════════════════════════════════
    # Digit markers
    # ══════════════════════════════════════════════════════════════════════════
    #
    # The AI deck streamer publishes Int32 digit predictions when its ONNX
    # model is confident.  We pair each prediction with the current front-
    # multiranger hit to estimate where on the wall the digit is, then plant
    # a TEXT_VIEW_FACING marker in the map frame at that point.
    #
    # Gating:
    #   - Only runs while airborne in a stable state (not TAKEOFF or LANDING).
    #     TAKEOFF has the drone ascending with unstable ranger readings near
    #     the floor; LANDING has the same near the end.  Everywhere else
    #     (WAIT_GOAL, NAVIGATE, SPINNING, DONE) the drone is at cruise altitude
    #     and the ranger readings are usable.
    #   - The front ranger must report a valid hit within DIGIT_RANGE_MAX.
    #     If there's no wall in front (ranger reports 0.0, inf, or a value
    #     beyond the useful range) the prediction is dropped - better to
    #     miss a digit than to plant a marker at an arbitrary location.
    #
    # Dedup:
    #   Predictions whose estimated world position falls within
    #   DIGIT_DEDUP_DIST of an existing marker are suppressed.  This avoids
    #   stacking 10+ duplicate labels at the same wall digit during a spin
    #   (the streamer publishes at ~1 Hz and a 360 deg spin takes ~12 s).

    def _maybe_place_digit_marker(self):
        if self.pending_digit is None:
            return

        # State gate - skip TAKEOFF/LANDING only.
        if self.state in (State.TAKEOFF, State.LANDING):
            return

        # Front ranger must be a valid, useful-range hit.
        front = self._front_range()
        if front <= 0.0 or front >= DIGIT_RANGE_MAX:
            # No wall in front within useful range - can't place this digit.
            # Leave pending_digit set; the next scan callback may find a wall.
            return

        # Compute the front-ranger hit point in the world frame.
        # The front ranger points along the drone's body +x axis.
        yaw = self.angles[2]
        wx  = self.position[0] + math.cos(yaw) * front
        wy  = self.position[1] + math.sin(yaw) * front

        # Dedup against previously-placed markers.
        for (px, py) in self.placed_digit_positions:
            if math.hypot(wx - px, wy - py) < DIGIT_DEDUP_DIST:
                # Too close to an existing marker - drop this prediction.
                self.pending_digit = None
                return

        # Fresh location - plant a new marker.
        self._add_digit_marker(wx, wy, self.pending_digit)
        self.placed_digit_positions.append((wx, wy))
        self.get_logger().info(
            f'Digit "{self.pending_digit}" placed at '
            f'({wx:.2f}, {wy:.2f}) from {self._front_range():.2f}m front hit.')
        self.pending_digit = None
        self._publish_digit_markers()

    def _add_digit_marker(self, wx: float, wy: float, digit: int):
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'digit_labels'
        m.id                 = self.next_digit_marker_id
        self.next_digit_marker_id += 1
        m.type               = Marker.TEXT_VIEW_FACING
        m.action             = Marker.ADD
        m.pose.position.x    = float(wx)
        m.pose.position.y    = float(wy)
        m.pose.position.z    = 0.25
        m.pose.orientation.w = 1.0
        m.scale.z            = 0.25
        m.color.r            = 1.0
        m.color.g            = 1.0
        m.color.b            = 1.0
        m.color.a            = 1.0
        m.text               = str(digit)
        m.lifetime           = Duration(sec=0, nanosec=0)  # forever
        self.digit_markers.append(m)

    def _publish_digit_markers(self):
        arr = MarkerArray()
        arr.markers = self.digit_markers
        self.digit_marker_pub.publish(arr)

    # ══════════════════════════════════════════════════════════════════════════
    # Visualisation / publishing
    # ══════════════════════════════════════════════════════════════════════════

    def _publish_vel(self, x=0.0, y=0.0, z=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = float(x)
        msg.linear.y  = float(y)
        msg.linear.z  = float(z)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _publish_goal_marker(self, x, y):
        m = Marker()
        m.header.frame_id    = 'map'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns = 'scan_goal'; m.id = 0
        m.type   = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(TAKEOFF_HEIGHT)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 1.0
        self.marker_pub.publish(m)

    def _publish_waypoint_markers(self, waypoints):
        arr   = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'map'
        clear.header.stamp    = self.get_clock().now().to_msg()
        clear.ns = 'waypoints'; clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (wx, wy) in enumerate(waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = 'waypoints'; m.id = i + 1
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x    = float(wx)
            m.pose.position.y    = float(wy)
            m.pose.position.z    = float(TAKEOFF_HEIGHT)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            arr.markers.append(m)
        self.wp_marker_pub.publish(arr)

    def _log_status(self, now):
        if (now - self.last_status_log_time) < 1.0:
            return
        self.last_status_log_time = now
        goal_s = ('none' if self.goal is None else
                  f'{math.hypot(self.goal[0]-self.position[0], self.goal[1]-self.position[1]):.2f}m '
                  f'{len(self.waypoints)}wp')
        self.get_logger().info(
            f'[{self.state.name}] F={self._front_range():.2f} '
            f'R={self._right_range():.2f} L={self._left_range():.2f} | '
            f'goal={goal_s}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionScanner()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()