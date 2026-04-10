#!/usr/bin/env python3

"""
Simple A* Navigator for Crazyflie
===================================
State machine:
  1. TAKEOFF  - ascend to cruise height, then transition to HOVER
  2. HOVER    - hold position; wait for a goal on /nav_goal (geometry_msgs/Point)
                keyboard 'l'/'L' → LAND, 't'/'T' → TAKEOFF
  3. PLAN     - run A* to the current goal; success → NAVIGATE, failure → HOVER
  4. NAVIGATE - follow A* waypoints; waypoints exhausted → HOVER
                keyboard 'l'/'L' → LAND, 't'/'T' → TAKEOFF
  5. LAND     - descend until z < 0.1 m, then stop
                keyboard 't'/'T' → TAKEOFF

Keyboard input is read from stdin in a background thread so it never blocks
the ROS 2 timer callback.  Run the node in a terminal and press 'l'/'t'.

Goal topic: /nav_goal  (geometry_msgs/Point)
  Publish x/y in metres in the map frame; z is ignored.
  Example:
    ros2 topic pub --once /nav_goal geometry_msgs/Point "{x: 1.0, y: 0.5, z: 0.0}"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Point, PoseStamped

import tf_transformations
import math
import heapq
import numpy as np
from enum import Enum, auto
from collections import deque
import threading
import sys
import select
import tty
import termios

# ── Map constants ─────────────────────────────────────────────────────────────
GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES       = 0.1

# ── Flight parameters ─────────────────────────────────────────────────────────
TAKEOFF_HEIGHT        = 0.5       # metres; set to match your arena
TAKEOFF_DELAY         = 3.0       # seconds of ascending before HOVER
CRUISE_SPEED          = 0.3
MAX_TURN_RATE         = 0.2
OBSTACLE_DIST         = 0.4       # front-wall replan trigger distance
GOAL_REACHED_DIST     = 0.2       # waypoint / final-goal arrival threshold
REPLAN_COOLDOWN       = 3.5       # minimum seconds between replans
WALL_INFLATION_CELLS  = 2
STANDOFF_WAYPOINTS    = 2         # trim this many wps from the end of each path
PROXIMITY_COST_WEIGHT = 2
PROXIMITY_COST_RADIUS = 10
WAYPOINT_SPACING      = 2

# ── Wall safety ───────────────────────────────────────────────────────────────
WALL_PUSH_DIST     = 0.3
WALL_SAFE_DIST     = 0.30
WALL_FILTER_ALPHA  = 0.3
WALL_KP_SAFETY     = 0.3
MAX_LATERAL_SPEED  = 0.24

# ── Stuck detector ────────────────────────────────────────────────────────────
STUCK_PROGRESS_DIST = 0.20
STUCK_TIMEOUT       = 5.0


class State(Enum):
    TAKEOFF  = auto()
    HOVER    = auto()
    PLAN     = auto()
    NAVIGATE = auto()
    LAND     = auto()

    
class SimpleNavigator(Node):

    def __init__(self):
        super().__init__('map_user')

        self.declare_parameter('robot_prefix', '/crazyflie_user_real')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.robot_prefix = robot_prefix

        # ── Internal state ────────────────────────────────────────────────────
        self.state    = State.TAKEOFF
        self.position = [0.0, 0.0, 0.0]
        self.angles   = [0.0, 0.0, 0.0]
        self.ranges   = [0.0, 0.0, 0.0, 0.0]   # back, right, front, left

        self.map_data   = None
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.goal = None   # (x, y) world-frame target set by /nav_goal

        # ── A* path following ─────────────────────────────────────────────────
        self.waypoints        = []
        self.current_wp       = None
        self.needs_replan     = False
        self.last_replan_time = 0.0
        self.last_replan_pos  = None
        self.last_path_cost   = None

        # ── Wall filtering ────────────────────────────────────────────────────
        self.filtered_right = None
        self.filtered_left  = None

        # ── Stuck detector ────────────────────────────────────────────────────
        self.last_progress_pos  = None
        self.last_progress_time = 0.0

        # ── Timer / logging ───────────────────────────────────────────────────
        self.start_time           = None
        self.last_status_log_time = 0.0
        self.status_log_period    = 1.0

        self.position_received = False
        self.map_received      = False
        self._landing_initiated = False

        # ── Keyboard input ────────────────────────────────────────────────────
        # A background thread reads single keypresses without blocking the timer.
        self._key_queue = deque()
        self._kb_thread = threading.Thread(target=self._keyboard_reader, daemon=True)
        self._kb_thread.start()

        # ── ROS subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_callback, 10)
        self._scan_sub = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        # replace the existing nav_goal subscription with:
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ── ROS publishers ────────────────────────────────────────────────────
        self.cmd_pub             = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub          = self.create_publisher(Marker, '/nav_goal_marker', 10)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/waypoints', 10)
        self.drone_marker_pub    = self.create_publisher(
            Marker, robot_prefix + '/drone_pose', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self._publish_vel(z=TAKEOFF_HEIGHT)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._info(f'SimpleNavigator started. prefix={robot_prefix}')
        self._info('Press l/L to land, t/T to take off.')

    # ══════════════════════════════════════════════════════════════════════════
    # Logging helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _info(self, msg):
        self.get_logger().info(f'[{self.state.name}] {msg}')

    def _warn(self, msg):
        self.get_logger().warn(f'[{self.state.name}] {msg}')

    def _err(self, msg):
        self.get_logger().error(f'[{self.state.name}] {msg}')

    # ══════════════════════════════════════════════════════════════════════════
    # Keyboard reader (background thread)
    # ══════════════════════════════════════════════════════════════════════════

    def _keyboard_reader(self):
        """
        Reads single keypresses from stdin without echoing or buffering.
        Enqueues each character for the timer callback to consume.
        Only active when stdin is a real terminal.
        """
        if not sys.stdin.isatty():
            return
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                # Poll with a 0.1 s timeout so the thread exits cleanly on shutdown.
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    ch = sys.stdin.read(1)
                    self._key_queue.append(ch)
        except Exception:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _drain_keyboard(self):
        """
        Process all queued keypresses.  Returns True if a state transition was
        triggered so the caller knows it should return early.
        """
        while self._key_queue:
            ch = self._key_queue.popleft()
            if ch in ('l', 'L'):
                self._info(f'Keyboard: l — landing from {self.state.name}')
                self._go_land()
                return True
            if ch in ('t', 'T'):
                self._info(f'Keyboard: t — re-arming takeoff from {self.state.name}')
                self._go_takeoff()
                return True
        return False

    # ══════════════════════════════════════════════════════════════════════════
    # State transition helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _go_land(self):
        """Transition to LAND from any state."""
        self._publish_vel()          # stop motors before descending
        self._landing_initiated = False
        self.state = State.LAND

    def _go_takeoff(self):
        """Transition to TAKEOFF from any state (e.g. from LAND or HOVER)."""
        self._landing_initiated = False
        self.goal       = None
        self.waypoints  = []
        self.current_wp = None
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = State.TAKEOFF

    def _go_hover(self):
        """Stop moving and wait for a new goal."""
        self._publish_vel()
        self.goal       = None
        self.waypoints  = []
        self.current_wp = None
        self.state = State.HOVER
        self._info('Hovering. Waiting for a goal on /nav_goal')

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

    def goal_callback(self, msg):
        new_goal = (msg.pose.position.x, msg.pose.position.y)
        self._info(f'Goal received: ({new_goal[0]:.2f}, {new_goal[1]:.2f})')
        if self.state == State.HOVER:
            self.goal  = new_goal
            self.state = State.PLAN
            self._info('Transitioning to PLAN')
        else:
            self._warn(
                f'Goal ignored — not in HOVER state (current: {self.state.name}). '
                'Land or wait for HOVER before sending a new goal.')
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

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self.state == State.TAKEOFF:
            # Keyboard check in TAKEOFF: allow landing mid-ascent.
            if self._drain_keyboard():
                return
            self._publish_vel(z=TAKEOFF_HEIGHT)
            if now - self.start_time > TAKEOFF_DELAY:
                self._info('Takeoff complete.')
                self._go_hover()

        # ── HOVER ─────────────────────────────────────────────────────────────
        elif self.state == State.HOVER:
            if self._drain_keyboard():
                return
            # Maintain altitude; small wall correction keeps the drone centred.
            self._publish_vel(y=self._get_wall_correction())

        # ── PLAN ──────────────────────────────────────────────────────────────
        elif self.state == State.PLAN:
            # Allow keyboard commands during planning.
            if self._drain_keyboard():
                return

            if self.goal is None:
                self._warn('PLAN entered with no goal — returning to HOVER')
                self._go_hover()
                return

            if not self.map_received or not self.position_received:
                self._info('Waiting for map and position data...')
                self._publish_vel(y=self._get_wall_correction())
                return

            self._info(
                f'Planning path to ({self.goal[0]:.2f}, {self.goal[1]:.2f})...')
            self._publish_goal_marker(self.goal[0], self.goal[1])
            self._plan_path_to_goal()

            if not self.waypoints:
                self._err(
                    f'A* found no path to ({self.goal[0]:.2f}, {self.goal[1]:.2f}). '
                    'Returning to HOVER.')
                self.goal = None
                self._go_hover()
            else:
                self._info(
                    f'Path found: {len(self.waypoints)} waypoints. '
                    'Transitioning to NAVIGATE.')
                self._publish_waypoint_markers(self.waypoints)
                # Reset stuck detector for the new run.
                self.last_progress_pos  = (self.position[0], self.position[1])
                self.last_progress_time = now
                self.state = State.NAVIGATE

        # ── NAVIGATE ──────────────────────────────────────────────────────────
        elif self.state == State.NAVIGATE:
            if self._drain_keyboard():
                return

            if self.goal is None:
                self._warn('NAVIGATE: goal is None — returning to HOVER')
                self._go_hover()
                return

            # Front-wall replan check.
            front = self._front_range()
            cooldown_ok = (now - self.last_replan_time) > REPLAN_COOLDOWN
            if 0.0 < front < OBSTACLE_DIST and cooldown_ok:
                self._warn(
                    f'Obstacle {front:.2f} m ahead — replanning.')
                self.last_replan_time = now
                self._plan_path_to_goal()
                if not self.waypoints:
                    self._err('Replan failed — returning to HOVER.')
                    self._go_hover()
                    return
                self.current_wp = None

            # Stuck detector.
            if self._is_stuck(now):
                self._warn('Stuck detected — replanning.')
                self.last_replan_time = now
                self._plan_path_to_goal()
                if not self.waypoints:
                    self._err('Replan after stuck failed — returning to HOVER.')
                    self._go_hover()
                    return
                self.current_wp = None

            # Waypoint following.
            if not self.waypoints and self.current_wp is None:
                self._info('All waypoints reached — goal complete. Returning to HOVER.')
                self._go_hover()
                return

            done = self._follow_waypoints(now)
            if done:
                self._info('Goal reached. Returning to HOVER.')
                self._go_hover()

        # ── LAND ──────────────────────────────────────────────────────────────
        elif self.state == State.LAND:
            # Only keyboard command active in LAND is 't'/'T' (takeoff again).
            if self._drain_keyboard():
                return

            if not self._landing_initiated:
                self._landing_initiated = True
                self._info('Landing initiated.')

            self._publish_vel(z=-0.2)
            if self.position[2] < 0.1:
                self.timer.cancel()
                self._publish_vel()
                self._info(
                    f'Landed at ({self.position[0]:.3f}, {self.position[1]:.3f}).')

    # ══════════════════════════════════════════════════════════════════════════
    # Stuck detector
    # ══════════════════════════════════════════════════════════════════════════

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

    # ══════════════════════════════════════════════════════════════════════════
    # Waypoint following (carried over from frontier_exploration_multiranger.py)
    # ══════════════════════════════════════════════════════════════════════════

    def _follow_waypoints(self, now):
        """
        Advance toward the current waypoint.
        Returns True when the final waypoint has been reached.
        """
        if self.current_wp is not None:
            dx, dy     = (self.current_wp[0] - self.position[0],
                          self.current_wp[1] - self.position[1])
            dist_to_wp = math.hypot(dx, dy)
            if dist_to_wp < GOAL_REACHED_DIST:
                if self.waypoints:
                    self.current_wp = self.waypoints.pop(0)
                    self._info(
                        f'Waypoint reached. Next: '
                        f'({self.current_wp[0]:.2f}, {self.current_wp[1]:.2f}), '
                        f'{len(self.waypoints)} remaining.')
                else:
                    self.current_wp = None
                    return True   # final waypoint reached
        elif self.waypoints:
            self.current_wp = self.waypoints.pop(0)
            self._info(
                f'Popped first waypoint '
                f'({self.current_wp[0]:.2f}, {self.current_wp[1]:.2f}), '
                f'{len(self.waypoints)} remaining.')

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

            wall_vx, wall_vy, speed_scale = self._get_wall_guidance()

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
    # Wall avoidance helpers
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
        """Returns (vx, vy, speed_scale) pure push-away on all four axes."""
        if len(self.ranges) < 4:
            return 0.0, 0.0, 1.0
        r, l = self._side_ranges_for_control()
        vy = 0.0
        speed_scale = 1.0
        if r < WALL_PUSH_DIST:
            vy += min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - r))
        if l < WALL_PUSH_DIST:
            vy -= min(MAX_LATERAL_SPEED, WALL_KP_SAFETY * (WALL_PUSH_DIST - l))
        if min(r, l) < WALL_SAFE_DIST:
            speed_scale = 0.75
        front = self._front_range()
        back  = (self.ranges[0]
                 if len(self.ranges) > 0 and self.ranges[0] > 0.0 else 999.0)
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
    # A* pathfinding (carried over verbatim from frontier_exploration_multiranger.py)
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
                r0, r1 = max(0, r - inflation), min(H - 1, r + inflation)
                c0, c1 = max(0, c - inflation), min(W - 1, c + inflation)
                for rr in range(r0, r1 + 1):
                    passable[rr * W + c0: rr * W + c1 + 1] = False
        return passable

    def _build_proximity_cost(self, radius=None):
        if radius is None:
            radius = PROXIMITY_COST_RADIUS
        W, H     = self.map_width, self.map_height
        dist_arr = np.full(W * H, -1, dtype=np.int32)
        mask     = (self.map_data == 100) | (self.map_data == -1)
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
        self.waypoints  = []
        self.current_wp = None
        self.needs_replan    = False
        self.last_replan_pos = None
        self.last_path_cost  = None
        if self.map_data is None or self.goal is None:
            self._warn('A*: no map or goal available')
            return

        sr, sc = self._world_to_grid(self.position[0], self.position[1])
        gr, gc = self._world_to_grid(self.goal[0], self.goal[1])
        sr = max(0, min(self.map_height - 1, sr))
        sc = max(0, min(self.map_width  - 1, sc))
        gr = max(0, min(self.map_height - 1, gr))
        gc = max(0, min(self.map_width  - 1, gc))

        self._info(
            f'A*: ({sr},{sc}) → ({gr},{gc}) '
            f'goal=({self.goal[0]:.2f},{self.goal[1]:.2f})')

        nb8 = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
               (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]
        found = False
        path_cost = None

        for inflation in range(WALL_INFLATION_CELLS, 0, -1):
            if inflation < WALL_INFLATION_CELLS:
                self._warn(f'A*: reduced inflation={inflation}')
            passable = self._build_inflated_map(inflation)
            W, H     = self.map_width, self.map_height
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
                if inflation < WALL_INFLATION_CELLS:
                    self._warn(f'A*: tight path at inflation={inflation}')
                break

        if not found:
            self._warn('A*: no path found')
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
               if i % WAYPOINT_SPACING == 0 or i == len(path) - 1]

        # Trim standoff waypoints so the drone stops slightly short of the goal.
        if len(wps) > STANDOFF_WAYPOINTS + 1:
            wps = wps[:-STANDOFF_WAYPOINTS]

        self.waypoints      = wps
        self.last_path_cost = path_cost
        self._info(
            f'A*: {len(path)} cells → {len(wps)} waypoints, cost={path_cost:.2f}')

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
        return (angle + math.pi) % (2 * math.pi) - math.pi

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
        m.ns                 = 'nav_goal'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = float(TAKEOFF_HEIGHT)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 1.0
        self.marker_pub.publish(m)

    def _publish_waypoint_markers(self, waypoints):
        arr   = MarkerArray()
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

    # ══════════════════════════════════════════════════════════════════════════
    # Status logging
    # ══════════════════════════════════════════════════════════════════════════

    def _log_status(self, now):
        if (now - self.last_status_log_time) < self.status_log_period:
            return
        self.last_status_log_time = now
        if self.goal is not None:
            dist = math.hypot(
                self.goal[0] - self.position[0],
                self.goal[1] - self.position[1])
            goal_s = f'({self.goal[0]:.2f},{self.goal[1]:.2f}) {dist:.2f}m {len(self.waypoints)}wp'
        else:
            goal_s = 'none'
        self._info(
            f'F={self._front_range():.2f} R={self._right_range():.2f} '
            f'L={self._left_range():.2f} | goal={goal_s}')
        self._publish_drone_marker()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
