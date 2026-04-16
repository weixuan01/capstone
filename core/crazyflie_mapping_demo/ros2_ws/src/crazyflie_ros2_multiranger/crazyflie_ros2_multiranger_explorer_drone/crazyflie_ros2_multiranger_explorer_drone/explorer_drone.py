#!/usr/bin/env python3

"""
Drone Navigator — works with GoalAssigner
==========================================
State machine:
  TAKEOFF      — take off and hover
  SPINNING     — initial 100° scan to seed the map
  WAIT_GOAL    — hover and wait for /cfX/assigned_goal from GoalAssigner
  NAVIGATE     — follow A* waypoints to the assigned goal
  DONE         — return to start via A* and land
  LANDING      — descend

Key changes from the monolithic version:
  - No frontier detection, scoring, or peer gradient logic.
  - No FIND_FRONTIER state.
  - Goals come exclusively from GoalAssigner via /cfX/assigned_goal.
  - On arrival or failure the node publishes to /cfX/goal_status
    ("REACHED" or "FAILED") and returns to WAIT_GOAL.
  - The stuck detector and replan logic are retained but with conservative
    thresholds so a single narrow-corridor replan does not abandon the goal.
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
STANDOFF_WAYPOINTS    = 2
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
SPIN_RATE = 0.5

# ── Stuck detector — conservative so single replans don't lose the goal ───────
STUCK_PROGRESS_DIST       = 0.10
STUCK_TIMEOUT             = 15.0
MAX_STUCK_EVENTS_PER_GOAL = 4
MAX_REPLANS_PER_GOAL      = 6


class State(Enum):
    TAKEOFF   = auto()
    SPINNING  = auto()
    WAIT_GOAL = auto()   # replaces FIND_FRONTIER
    NAVIGATE  = auto()
    DONE      = auto()
    LANDING   = auto()


class DroneNavigator(Node):

    def __init__(self):
        super().__init__('drone_navigator')

        self.declare_parameter('robot_prefix', '/cf1')
        robot_prefix = self.get_parameter('robot_prefix').value

        # ── Internal state ────────────────────────────────────────────────────
        self.state    = State.TAKEOFF
        self.position = [0.0, 0.0, 0.0]
        self.angles   = [0.0, 0.0, 0.0]
        self.ranges   = [0.0, 0.0, 0.0, 0.0]

        self.map_data   = None
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.goal             = None   # (x, y) from assigner
        self.waypoints        = []
        self.current_wp       = None
        self.needs_replan     = False
        self.last_replan_time = 0.0
        self.last_replan_pos  = None
        self.last_path_cost   = None
        self.navigating_home  = False

        self.start_pos           = None
        self.start_time          = None
        self.spin_start_time     = None
        self.spin_start_yaw      = None
        self.spin_total_rotation = 0.0
        self._last_spin_yaw      = 0.0

        self.filtered_right = None
        self.filtered_left  = None

        self.goal_start_time       = 0.0
        self.last_progress_pos     = None
        self.last_progress_time    = 0.0
        self.stuck_events_for_goal = 0
        self.replan_count_for_goal = 0

        self.position_received = False
        self.map_received      = False
        self._going_home       = False  # set True when RECALL or LAND command received
        self.exploration_start_time = None
        self.last_status_log_time   = 0.0

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

        # Goal from assigner
        self.create_subscription(
            Point, robot_prefix + '/assigned_goal', self._goal_cb, 10)

        self.cmd_pub        = self.create_publisher(Twist,      robot_prefix + '/cmd_vel_raw', 10)
        self.status_pub     = self.create_publisher(String,     robot_prefix + '/goal_status',  10)
        self.marker_pub     = self.create_publisher(Marker,     robot_prefix + '/exploration_goal', 10)
        self.wp_marker_pub  = self.create_publisher(MarkerArray,robot_prefix + '/waypoints', 10)
        self.drone_marker_pub = self.create_publisher(Marker,   robot_prefix + '/drone_pose', 10)

        self.create_service(
            Trigger, robot_prefix + '/stop_exploration', self._stop_cb)
        self.timer = self.create_timer(0.1, self._timer_cb)

        self._publish_vel(z=TAKEOFF_HEIGHT)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'DroneNavigator started. prefix={robot_prefix}')

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
                WALL_FILTER_ALPHA * right + (1 - WALL_FILTER_ALPHA) * self.filtered_right)
        if left > 0.0:
            self.filtered_left = (left if self.filtered_left is None else
                WALL_FILTER_ALPHA * left + (1 - WALL_FILTER_ALPHA) * self.filtered_left)

    def _map_cb(self, msg):
        self.map_data   = np.array(msg.data, dtype=np.int8)
        self.map_width  = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y]
        self.map_received = True

    def _goal_cb(self, msg: Point):
        """Receive a message on /cfX/assigned_goal.

        Three cases distinguished by x and z values:
          x=NaN, z=0.0  — RECALL: return home then land (from assigner or mission control)
          x=NaN, z=1.0  — LAND:   land in place immediately (from mission control)
          x=real number — normal frontier goal from assigner
        """
        # ── Recall or land-in-place command ───────────────────────────────────
        if math.isnan(msg.x):
            self._going_home = True
            self._report_status('RECALLED')
            self.goal         = None
            self.waypoints    = []
            self.current_wp   = None
            self.needs_replan = False

            if msg.z == 1.0:
                # Land in place — jump straight to LANDING
                self.get_logger().info(
                    'Land-in-place command received. Descending now.')
                self.state = State.LANDING
            else:
                # Recall — navigate home first via DONE state
                self.get_logger().info(
                    'Recall command received. Returning home.')
                self.state = State.DONE
            return

        # ── Ignore new frontier goals if already recalled ─────────────────────
        if self._going_home:
            return

        # ── Normal frontier goal ──────────────────────────────────────────────
        new_goal = (msg.x, msg.y)
        if self.goal == new_goal:
            return  # same goal echoed back — ignore
        now = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'New goal assigned: ({new_goal[0]:.2f},{new_goal[1]:.2f})')
        self.goal                  = new_goal
        self.waypoints             = []
        self.current_wp            = None
        self.needs_replan          = False
        self.last_replan_time      = 0.0
        self.last_replan_pos       = None
        self.last_path_cost        = None
        self.goal_start_time       = now
        self.last_progress_pos     = (self.position[0], self.position[1])
        self.last_progress_time    = now
        self.stuck_events_for_goal = 0
        self.replan_count_for_goal = 0
        # Only switch to NAVIGATE if already in WAIT_GOAL or NAVIGATE.
        # If still taking off or spinning the state machine picks up the
        # goal naturally once it enters WAIT_GOAL.
        if self.state in (State.WAIT_GOAL, State.NAVIGATE):
            self._plan_path_to_goal()
            if self.waypoints:
                self.state = State.NAVIGATE
                self.get_logger().info(
                    f'Path planned: {len(self.waypoints)} waypoints.')
            else:
                self.get_logger().warn(
                    'A* failed for assigned goal — reporting FAILED.')
                self._report_status('FAILED')
                self.goal  = None
                self.state = State.WAIT_GOAL

    def _stop_cb(self, request, response):
        self.get_logger().info('Stop requested — landing.')
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
                    f'Takeoff complete. Home: ({self.start_pos[0]:.3f},{self.start_pos[1]:.3f})')
                self.spin_start_yaw      = self.angles[2]
                self.spin_total_rotation = 0.0
                self._last_spin_yaw      = self.angles[2]
                self.state = State.SPINNING

        # ── SPINNING ──────────────────────────────────────────────────────────
        elif self.state == State.SPINNING:
            self._publish_vel(y=self._wall_correction(), wz=SPIN_RATE)
            yaw_delta = self._wrap_angle(
                self.angles[2] - (self.spin_start_yaw
                                  if self.spin_total_rotation == 0.0
                                  else self._last_spin_yaw))
            self._last_spin_yaw       = self.angles[2]
            self.spin_total_rotation += abs(yaw_delta)
            if self.spin_total_rotation >= (10/36) * 2 * math.pi:
                self._publish_vel()
                self.exploration_start_time = now
                self.get_logger().info('Initial scan done. Waiting for goal.')
                self.state = State.WAIT_GOAL

        # ── WAIT_GOAL ─────────────────────────────────────────────────────────
        elif self.state == State.WAIT_GOAL:
            # Just hover. The assigner will call _goal_cb when ready.
            self._publish_vel(y=self._wall_correction())
            if self.goal is not None:
                # Goal arrived while we were hovering
                self._plan_path_to_goal()
                if self.waypoints:
                    self.state = State.NAVIGATE
                else:
                    self.get_logger().warn('A* failed — reporting FAILED, waiting.')
                    self._report_status('FAILED')
                    self.goal = None

        # ── NAVIGATE ──────────────────────────────────────────────────────────
        elif self.state == State.NAVIGATE:
            if self.goal is None:
                self.state = State.WAIT_GOAL
                return

            # Layer 1 — stuck detection
            if self._is_stuck(now):
                self.stuck_events_for_goal += 1
                self.get_logger().warn(
                    f'Stuck event {self.stuck_events_for_goal}/{MAX_STUCK_EVENTS_PER_GOAL}')
                if self.stuck_events_for_goal >= MAX_STUCK_EVENTS_PER_GOAL:
                    self.get_logger().warn('Too many stuck events — reporting FAILED.')
                    self._report_status('FAILED')
                    self._clear_goal()
                    return
                self._execute_replan(now)
                return

            # Layer 2 — replan flag
            if self.needs_replan:
                self.replan_count_for_goal += 1
                if self.replan_count_for_goal > MAX_REPLANS_PER_GOAL:
                    self.get_logger().warn('Too many replans — reporting FAILED.')
                    self._report_status('FAILED')
                    self._clear_goal()
                    return
                cg = self._world_to_grid(self.position[0], self.position[1])
                if (self.last_replan_pos is not None and
                        abs(cg[0]-self.last_replan_pos[0]) <= 1 and
                        abs(cg[1]-self.last_replan_pos[1]) <= 1):
                    self.get_logger().warn('Replanned but no movement — reporting FAILED.')
                    self._report_status('FAILED')
                    self._clear_goal()
                    return
                self.last_replan_pos = cg
                self._execute_replan(now)
                return

            # Layer 3 — safety replan
            front = self._front_range()
            if (0.0 < front < OBSTACLE_DIST and
                    not self.needs_replan and
                    (now - self.last_replan_time) > REPLAN_COOLDOWN):
                self._execute_replan(now)
                return

            # Layer 4 — waypoint following
            if not self.waypoints and self.current_wp is None:
                if self.navigating_home:
                    self.navigating_home = False
                    self.state = State.LANDING
                else:
                    self.get_logger().info(
                        f'Goal ({self.goal[0]:.2f},{self.goal[1]:.2f}) reached.')
                    self._report_status('REACHED')
                    self._clear_goal()
                    # Brief scan spin before waiting for next goal
                    self.spin_start_yaw      = self.angles[2]
                    self.spin_total_rotation = 0.0
                    self._last_spin_yaw      = self.angles[2]
                    self.state = State.SPINNING
                return

            if self._follow_waypoints(now):
                if self.navigating_home:
                    self.navigating_home = False
                    self.state = State.LANDING
                else:
                    self.get_logger().info(
                        f'Goal ({self.goal[0]:.2f},{self.goal[1]:.2f}) reached.')
                    self._report_status('REACHED')
                    self._clear_goal()
                    self.spin_start_yaw      = self.angles[2]
                    self.spin_total_rotation = 0.0
                    self._last_spin_yaw      = self.angles[2]
                    self.state = State.SPINNING

        # ── DONE ──────────────────────────────────────────────────────────────
        elif self.state == State.DONE:
            if self.start_pos is None:
                self.state = State.LANDING
                return
            home = (self.start_pos[0], self.start_pos[1])
            if math.hypot(self.position[0]-home[0], self.position[1]-home[1]) < GOAL_REACHED_DIST:
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
    # Navigation helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _execute_replan(self, now):
        self.needs_replan     = False
        self.last_replan_time = now
        self._plan_path_to_goal()
        if not self.waypoints:
            self.get_logger().warn('Replan failed — reporting FAILED.')
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
        # Do not fire within 6 seconds of a replan
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
        vy = speed_scale = 0.0
        speed_scale = 1.0
        if r < WALL_PUSH_DIST:
            vy += min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - r))
        if l < WALL_PUSH_DIST:
            vy -= min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - l))
        if min(r, l) < WALL_SAFE_DIST:
            speed_scale = 0.75
        front = self._front_range()
        back  = (self.ranges[0] if len(self.ranges) > 0 and self.ranges[0] > 0.0 else 999.0)
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
    # A* path planning
    # ══════════════════════════════════════════════════════════════════════════

    def _world_to_grid(self, wx, wy):
        return (int((wy - self.map_origin[1]) / MAP_RES),
                int((wx - self.map_origin[0]) / MAP_RES))

    def _grid_to_world(self, row, col):
        return (self.map_origin[0] + (col + 0.5) * MAP_RES,
                self.map_origin[1] + (row + 0.5) * MAP_RES)

    def _build_inflated_map(self, inflation):
        W, H     = self.map_width, self.map_height
        passable = np.ones(W * H, dtype=bool)
        passable[self.map_data == 100] = False
        passable[self.map_data == -1]  = False
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
        mask     = (self.map_data == 100) | (self.map_data == -1)
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

        wps = [self._grid_to_world(r, c)
               for i, (r, c) in enumerate(path)
               if i % WAYPOINT_SPACING == 0 or i == len(path)-1]

        if not self.navigating_home and len(wps) > STANDOFF_WAYPOINTS + 1:
            wps = wps[:-STANDOFF_WAYPOINTS]

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
        m.ns = 'goal'; m.id = 0
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
    node = DroneNavigator()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()