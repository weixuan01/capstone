#!/usr/bin/env python3

"""
Autonomous Frontier-Based Explorer

1. Takes off and waits for the map to start building
2. Finds "frontiers" - cells on the edge of known/unknown space
3. Flies toward the nearest frontier to map new area
4. Repeats until no frontiers remain (area fully mapped)
5. Returns to start and lands

Topics:
  Subscribes: /{robot_prefix}/map    - OccupancyGrid from simple_mapper
  Subscribes: /{robot_prefix}/odom   - Drone position
  Subscribes: /{robot_prefix}/scan   - Laser ranges for obstacle avoidance
  Publishes:  /cmd_vel               - Velocity commands

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from .wall_following.wall_following import WallFollowing

import tf_transformations
import math
import numpy as np
from enum import Enum, auto

# ── Map constants (must match simple_mapper_multiranger.py) ──────────────────
GLOBAL_SIZE_X = 20.0   # metres
GLOBAL_SIZE_Y = 20.0   # metres
MAP_RES = 0.1          # metres per cell

# ── Tuning parameters ────────────────────────────────────────────────────────
TAKEOFF_HEIGHT       = 0.3    # metres
GOAL_REACHED_DIST    = 0.1    # metres - how close counts as "reached" frontier
MIN_FRONTIER_DIST    = 0.8    # metres - ignore tiny frontiers closer than this
FRONTIER_SEARCH_STEP = 1      # check every Nth cell for speed (increase if slow)

class State(Enum):
    TAKEOFF        = auto()
    WAITING        = auto()   # hovering, waiting for map data
    FIND_FRONTIER  = auto()   # computing next goal
    NAVIGATE       = auto()   # flying toward frontier
    AVOID_OBSTACLE = auto()   # spinning away from obstacle
    ESCAPE         = auto()
    WALL_FOLLOW_ESCAPE = auto()
    DONE           = auto()   # fully mapped, returning home
    LANDING        = auto()


class FrontierExplorationMultiranger(Node):

    def __init__(self):
        super().__init__('simple_mapper_multiranger')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value

        self.declare_parameter('max_forward_speed', 0.5) # m/s forward speed
        self.forward_speed = self.get_parameter('max_forward_speed').value

        self.declare_parameter('max_turn_rate', 0.5) # rad/s turning speed
        self.turn_rate = self.get_parameter('max_turn_rate').value

        self.declare_parameter('target_altitude', 0.5)
        self.target_altitude = float(self.get_parameter('target_altitude').value)

        self.declare_parameter('alt_kp', 1.2)
        self.alt_kp = float(self.get_parameter('alt_kp').value)

        self.declare_parameter('max_vz', 0.4)
        self.max_vz = float(self.get_parameter('max_vz').value)

        self.declare_parameter('max_obstacle_distance', 0.3) # metres - stop and turn if obstacle closer than this
        self.obstacle_distance = self.get_parameter('max_obstacle_distance').value

        # ── State ─────────────────────────────────────────────────────────────
        self.state = State.TAKEOFF
        self.position  = [0.0, 0.0, 0.0]   # x, y, z in metres
        self.angles    = [0.0, 0.0, 0.0]   # roll, pitch, yaw in radians
        self.ranges    = [0.0, 0.0, 0.0, 0.0]  # back, right, front, left
        self.map_data  = None               # flat list from OccupancyGrid
        self.map_width = 0
        self.map_height= 0
        self.map_origin= [0.0, 0.0]
        self.goal      = None               # (x, y) in metres
        self.start_pos = None               # where we took off from
        self.start_time= None
        self.avoid_start_time = None
        self.escape_start_time = None
        self.position_received = False
        self.map_received = False
        self.no_frontier_count = 0          # consecutive times no frontier found
        self.stuck_count = 0
        self.last_goal = None
        self.wall_follow_mode = False
        self.wall_follow_start_time = None
        self.last_status_log_time = 0.0
        self.current_mode = "TAKEOFF"

        # ── ROS subscribers / publishers ─────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.map_sub = self.create_subscription(
            OccupancyGrid, robot_prefix + '/map', self.map_callback, map_qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.stop_srv = self.create_service(
            Trigger, robot_prefix + '/stop_exploration', self.stop_cb)

        # ── Main loop timer (10 Hz) ───────────────────────────────────────────
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.wall_following = WallFollowing(
            max_turn_rate=self.turn_rate,
            max_forward_speed=self.forward_speed,
            init_state=WallFollowing.StateWallFollowing.FORWARD
        )

        # Take off immediately like wall following
        msg = Twist()
        msg.linear.z = TAKEOFF_HEIGHT
        self.cmd_pub.publish(msg)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'Autonomous explorer started for {robot_prefix}. Taking off...')

    # ══════════════════════════════════════════════════════════════════════════
    # Subscriber callbacks
    # ══════════════════════════════════════════════════════════════════════════
    
    def _log_status(self, now, extra=""):
        if now - self.last_status_log_time > 1.0:
            self.get_logger().info(
                f'Mode={self.current_mode} Pos=({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}) '
                f'Ranges(front={self.ranges[2]:.2f}, left={self.ranges[3]:.2f}, right={self.ranges[1]:.2f}) {extra}'
            )
            self.last_status_log_time = now
    
    def _hold_altitude_vz(self):
        err = self.target_altitude - self.position[2]
        vz = self.alt_kp * err
        if vz > self.max_vz:
            vz = self.max_vz
        if vz < -self.max_vz:
            vz = -self.max_vz
        return vz

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles = list(euler)
        if not self.position_received:
            self.start_pos = [self.position[0], self.position[1]]
            self.position_received = True

    def scan_callback(self, msg):
        self.ranges = list(msg.ranges)

    def _wall_follow_cmd(self, now):
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0
        state_wf = "HOVER"

        actual_yaw_rad = self.angles[2]

        right_range = self.ranges[1]
        front_range = self.ranges[2]
        left_range = self.ranges[3]

        # same setup as working wall_following script
        wf_dir = WallFollowing.WallFollowingDirection.RIGHT
        side_range = left_range

        if side_range > 0.1:
            velocity_x, velocity_y, yaw_rate, state_wf = self.wall_following.wall_follower(
                front_range, side_range, actual_yaw_rad, wf_dir, now
        )

        return velocity_x, velocity_y, yaw_rate, state_wf

    def map_callback(self, msg):
        self.map_data   = list(msg.data)
        self.map_width  = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x,
                           msg.info.origin.position.y]
        self.map_received = True

    # ══════════════════════════════════════════════════════════════════════════
    # Service callback - stop exploration
    # ══════════════════════════════════════════════════════════════════════════

    def stop_cb(self, request, response):
        self.get_logger().info('Stop requested - landing now')
        self.timer.cancel()
        self._publish_vel(z=-0.2)
        response.success = True
        return response

    # ══════════════════════════════════════════════════════════════════════════
    # Main state machine (runs at 10 Hz)
    # ══════════════════════════════════════════════════════════════════════════

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # ── TAKEOFF: hover until delay passes ────────────────────────────────
        if self.state == State.TAKEOFF:
            self.current_mode = "TAKEOFF"
            if now - self.start_time > self.delay:
                self.get_logger().info('Takeoff complete. Starting exploration.')
                self.state = State.WAITING
            return

        # ── WAITING: wait until we have map and position data ────────────────
        elif self.state == State.WAITING:
            self.current_mode = "WAITING"
            self._publish_vel(z=self._hold_altitude_vz())
            if self.map_received and self.position_received:
                self.state = State.FIND_FRONTIER

        # ── FIND_FRONTIER: scan map and pick nearest frontier ─────────────────
        elif self.state == State.FIND_FRONTIER:
            self._publish_vel(z=self._hold_altitude_vz())

            frontier = self._find_nearest_frontier()

            if frontier is not None:
                self.goal = frontier
                self.last_goal = frontier
                self.no_frontier_count = 0
                self.get_logger().info(
                    f'Frontier detected. Switching to exploration mode: ({frontier[0]:.2f}, {frontier[1]:.2f})')
                self.state = State.NAVIGATE
            else:
                self.no_frontier_count += 1
                self._log_status(now, extra="Searching for frontier")

                if self.no_frontier_count >= 10:
                    self.get_logger().info('No frontiers found - area fully mapped!')
                    self.state = State.DONE

        # ── NAVIGATE: fly toward goal frontier ───────────────────────────────
        elif self.state == State.NAVIGATE:
            self.current_mode = "NAVIGATE"

            frontier = self._find_nearest_frontier()

            if frontier is None:
                self.no_frontier_count += 1
                self._log_status(now, extra="No frontier currently visible")

                if self.no_frontier_count >= 20:
                    self.get_logger().info('No frontiers found for a while - area likely mapped.')
                    self.state = State.DONE
                else:
                    self._publish_vel(z=self._hold_altitude_vz())
                return

            self.no_frontier_count = 0
            self.goal = frontier
            self.last_goal = frontier

            vx, vy, wz, wf_state = self._wall_follow_cmd(now)

            self._log_status(
                now,
                extra=f'Goal=({frontier[0]:.2f}, {frontier[1]:.2f}) WF={wf_state} Cmd=({vx:.2f},{vy:.2f},{wz:.2f})'
            )

            self._publish_vel(x=vx, y=vy, z=self._hold_altitude_vz(), wz=wz)

        # ── AVOID_OBSTACLE: spin in place to clear obstacle ──────────────────
        elif self.state == State.AVOID_OBSTACLE:
            # turn in place first
            self._publish_vel(z=self._hold_altitude_vz(), wz=self.turn_rate)
            if now - self.avoid_start_time > 1.0:
                self.get_logger().info('Turning complete. Escaping...')
                self.escape_start_time = now
                self.state = State.ESCAPE
     
        elif self.state == State.ESCAPE:
            # move a little backward to get away from wall
            self._publish_vel(x=-0.15, z=self._hold_altitude_vz(), wz=0.4)

            if now - self.escape_start_time > 1.0:
                self.get_logger().info('Escape complete. Finding new frontier...')
                self.goal = None
                self.last_goal = None
                self.state = State.FIND_FRONTIER

        elif self.state == State.WALL_FOLLOW_ESCAPE:
            # Temporary wall-following style movement:
            # move forward slowly while turning left slightly
            self._publish_vel(x=0.15, z=self._hold_altitude_vz(), wz=0.5)

            if now - self.wall_follow_start_time > 3.0:
                self.get_logger().info('Wall-follow escape complete. Returning to frontier search...')
                self.stuck_count = 0
                self.goal = None
                self.last_goal = None
                self.state = State.FIND_FRONTIER

        # ── DONE: return to start position and land ───────────────────────────
        elif self.state == State.DONE:
            self.current_mode = "DONE"
            if self.start_pos is not None:
                dx = self.start_pos[0] - self.position[0]
                dy = self.start_pos[1] - self.position[1]
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > 0.3:
                    vx, vy, wz = self._steer_to_goal(dx, dy, dist)
                    self._publish_vel(x=vx, y=vy, z=self._hold_altitude_vz(), wz=wz)
                else:
                    self.get_logger().info('Home reached. Landing.')
                    self.state = State.LANDING

        # ── LANDING ───────────────────────────────────────────────────────────
        elif self.state == State.LANDING:
            self.current_mode = "LANDING"
            self._publish_vel(z=-0.2)
            if self.position[2] < 0.1:
                self.timer.cancel()
                self._publish_vel()
                self.get_logger().info('Landed. Exploration complete!')

    # ══════════════════════════════════════════════════════════════════════════
    # Frontier finding
    # ══════════════════════════════════════════════════════════════════════════

    def _find_nearest_frontier(self):
        """
        Scan the occupancy grid for frontier cells.
        A frontier is a FREE cell (value=0) that has at least one
        UNKNOWN neighbour (value=-1).
        Returns the (x, y) world coordinates of the nearest frontier,
        or None if none found.
        """
        if self.map_data is None:
            return None

        px = self.position[0]
        py = self.position[1]
        W  = self.map_width
        H  = self.map_height

        best_dist = float('inf')
        best_world = None

        # Step through cells (skip every N for speed)
        for row in range(1, H - 1, FRONTIER_SEARCH_STEP):
            for col in range(1, W - 1, FRONTIER_SEARCH_STEP):
                idx = row * W + col

                # Only consider free cells
                if self.map_data[idx] != 0:
                    continue

                # Check 4-connected neighbours for unknown cells
                neighbours = [
                    self.map_data[(row-1) * W + col],
                    self.map_data[(row+1) * W + col],
                    self.map_data[row * W + (col-1)],
                    self.map_data[row * W + (col+1)],
                ]
                if -1 not in neighbours:
                    continue

                # Convert cell to world coordinates
                wx = self.map_origin[0] + (col + 0.5) * MAP_RES
                wy = self.map_origin[1] + (row + 0.5) * MAP_RES

                dx = wx - px
                dy = wy - py
                dist = math.sqrt(dx*dx + dy*dy)

                if dist < MIN_FRONTIER_DIST:
                    continue

                if dist < best_dist:
                    best_dist = dist
                    best_world = (wx, wy)

        return best_world

    # ══════════════════════════════════════════════════════════════════════════
    # Navigation helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _steer_to_goal(self, dx, dy, dist):
        """
        Compute velocity commands to steer toward a goal offset (dx, dy).
        Uses the drone's current yaw so commands are in the drone's body frame.
        """
        yaw = self.angles[2]

        # Desired heading
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self._wrap_angle(desired_yaw - yaw)

        # If pointing roughly toward goal, move forward
        if abs(yaw_error) < 0.6:
            speed = min(self.forward_speed, dist)
            vx = speed * math.cos(yaw_error)
            vy = 0.0
        else:
            # Turn on the spot first
            vx = 0.0
            vy = 0.0

        # Proportional turn rate, capped at max
        wz = max(-self.turn_rate, min(self.turn_rate, 4.0 * yaw_error))

        return vx, vy, wz

    def _wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _publish_vel(self, x=0.0, y=0.0, z=0.0, wz=0.0):
        """Publish a Twist velocity command."""
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
