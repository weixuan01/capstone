#!/usr/bin/env python3

"""
Autonomous Frontier-Based Explorer for Crazyflie
=================================================
State machine:
  1. TAKEOFF        - take off and hover
  2. FIND_FRONTIER  - scan map, cluster frontiers, list valid ones, pick closest
  3. NAVIGATE       - fly toward the chosen frontier
  4. WALL_FOLLOW    - wall follow on left side for 4 seconds, then replan
  5. DONE           - no frontiers left, return home and land

Wall safety (active in FIND_FRONTIER, NAVIGATE, WALL_FOLLOW, DONE):
  - Always keeps at least WALL_SAFE_DIST (0.3m) from any known side wall
  - In corridors where walls are detected on both sides, nudges toward the
    midpoint to fly down the centre

Topics:
  Subscribes: /crazyflie/map   - OccupancyGrid from simple_mapper
  Subscribes: /crazyflie/odom  - drone position and orientation
  Subscribes: /crazyflie/scan  - laser ranges (back, right, front, left)
  Publishes:  /cmd_vel         - velocity commands

To stop the drone safely:
  ros2 service call /crazyflie/stop_exploration std_srvs/srv/Trigger
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
from enum import Enum, auto
from collections import deque

from .wall_following.wall_following import WallFollowing

# ── Map constants — must match simple_mapper_multiranger.py ──────────────────
GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES       = 0.1

# ── Flight parameters ────────────────────────────────────────────────────────
TAKEOFF_HEIGHT      = 0.05   # metres to hover at
TAKEOFF_DELAY       = 5.0   # seconds to wait after takeoff before exploring
CRUISE_SPEED        = 0.3   # m/s forward speed toward frontier
MAX_TURN_RATE       = 0.8   # rad/s maximum yaw rate
OBSTACLE_DIST       = 0.4   # metres — start wall following if obstacle closer
GOAL_REACHED_DIST   = 0.3   # metres — frontier counts as reached within this
MIN_FRONTIER_DIST   = 0.1   # metres — ignore frontiers closer than this
FRONTIER_STEP       = 3     # scan every Nth map cell (increase if CPU is slow)
WALL_FOLLOW_TIMEOUT = 4.0   # seconds to wall follow before replanning

# ── Frontier filtering ────────────────────────────────────────────────────────
MIN_CLUSTER_SIZE    = 1     # minimum frontier cluster size to be considered real

# ── Wall safety and corridor centering ───────────────────────────────────────
WALL_SAFE_DIST      = 0.2   # metres — minimum distance to keep from side walls
WALL_CENTRE_RANGE   = 0.5   # metres — max range to consider a wall for centering
                            # (only centre if BOTH sides detect a wall within this)
WALL_KP_SAFETY      = 0.8   # proportional gain for safety push (stronger)
WALL_KP_CENTRE      = 0.4   # proportional gain for centering nudge (gentler)
MAX_LATERAL_SPEED   = 0.2   # m/s maximum sideways correction speed


class State(Enum):
    TAKEOFF       = auto()
    FIND_FRONTIER = auto()
    NAVIGATE      = auto()
    WALL_FOLLOW   = auto()
    DONE          = auto()
    LANDING       = auto()


class FrontierExplorationMultiranger(Node):

    def __init__(self):
        super().__init__('simple_mapper_multiranger')

        # ── ROS parameter ─────────────────────────────────────────────────────
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        # ── Internal state ────────────────────────────────────────────────────
        self.state    = State.TAKEOFF
        self.position = [0.0, 0.0, 0.0]
        self.angles   = [0.0, 0.0, 0.0]
        self.ranges   = [0.0, 0.0, 0.0, 0.0]  # back, right, front, left

        self.map_data   = None
        self.map_width  = 0
        self.map_height = 0
        self.map_origin = [0.0, 0.0]

        self.goal                   = None
        self.start_pos              = None
        self.start_time             = None
        self.wall_follow_start_time = None

        self.position_received = False
        self.map_received      = False

        # ── Wall following ────────────────────────────────────────────────────
        self.wall_follower = WallFollowing(
            max_turn_rate=MAX_TURN_RATE,
            max_forward_speed=CRUISE_SPEED,
            init_state=WallFollowing.StateWallFollowing.FORWARD
        )
        self.wf_direction = WallFollowing.WallFollowingDirection.LEFT

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

        # ── ROS publisher ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Stop service ──────────────────────────────────────────────────────
        self.create_service(
            Trigger, robot_prefix + '/stop_exploration', self.stop_callback)

        # ── Main loop at 10 Hz ────────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self.timer_callback)

        self._publish_vel(z=TAKEOFF_HEIGHT)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'Explorer started. Robot prefix: {robot_prefix}')

    # ══════════════════════════════════════════════════════════════════════════
    # ROS callbacks
    # ══════════════════════════════════════════════════════════════════════════

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

    def map_callback(self, msg):
        self.map_data   = list(msg.data)
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
            self.get_logger().error(f'Crash in state machine: {e}')
            self.get_logger().error(traceback.format_exc())
            self._publish_vel()
            self.timer.cancel()

    def _state_machine(self, now):

        # ── STATE 1: TAKEOFF ──────────────────────────────────────────────────
        # No wall correction during takeoff — drone is climbing vertically
        if self.state == State.TAKEOFF:
            self._publish_vel(z=TAKEOFF_HEIGHT)
            if now - self.start_time > TAKEOFF_DELAY:
                self.get_logger().info('Takeoff complete — starting exploration')
                self.state = State.FIND_FRONTIER

        # ── STATE 2: FIND_FRONTIER ────────────────────────────────────────────
        # Hover with wall correction while computing the next goal
        elif self.state == State.FIND_FRONTIER:
            vy_correction = self._get_wall_correction()
            self._publish_vel(y=vy_correction)

            if not self.map_received or not self.position_received:
                self.get_logger().info('Waiting for map and position data...')
                return

            clusters = self._get_frontier_clusters()

            self.get_logger().info(
                f'--- Frontier scan: {len(clusters)} valid clusters found ---')
            for i, (cx, cy, dist, size) in enumerate(clusters):
                self.get_logger().info(
                    f'  [{i+1}] centre=({cx:.2f}, {cy:.2f})  '
                    f'dist={dist:.2f}m  size={size} cells')

            if len(clusters) == 0:
                self.get_logger().info('No valid frontiers — area fully mapped!')
                self.state = State.DONE
                return

            self.goal = (clusters[0][0], clusters[0][1])
            self.get_logger().info(
                f'Goal set: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
            self.state = State.NAVIGATE

        # ── STATE 3: NAVIGATE ─────────────────────────────────────────────────
        # Fly toward frontier with wall safety correction applied on top
        elif self.state == State.NAVIGATE:
            if self.goal is None:
                self.get_logger().warn('No goal — going back to find frontier')
                self.state = State.FIND_FRONTIER
                return

            front_range = self._front_range()

            if 0.0 < front_range < OBSTACLE_DIST:
                self.get_logger().info(
                    f'Obstacle at {front_range:.2f}m — starting wall follow')
                self.wall_follow_start_time = now
                self.state = State.WALL_FOLLOW
                return

            dx = self.goal[0] - self.position[0]
            dy = self.goal[1] - self.position[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < GOAL_REACHED_DIST:
                self.get_logger().info('Frontier reached — finding next')
                self.goal = None
                self.state = State.FIND_FRONTIER
                return

            # Get base steering toward goal
            vx, vy, wz = self._steer_toward(dx, dy, dist)

            # Add wall correction on top of base lateral velocity
            vy_correction = self._get_wall_correction()
            vy_total = max(-MAX_LATERAL_SPEED,
                           min(MAX_LATERAL_SPEED, vy + vy_correction))

            self._publish_vel(x=vx, y=vy_total, wz=wz)

        # ── STATE 4: WALL_FOLLOW ──────────────────────────────────────────────
        # Wall follow with safety correction — prevents hugging walls too tightly
        elif self.state == State.WALL_FOLLOW:
            if self.goal is None:
                self.state = State.FIND_FRONTIER
                return

            time_wall_following = now - self.wall_follow_start_time

            if time_wall_following > WALL_FOLLOW_TIMEOUT:
                self.get_logger().info(
                    f'Wall follow complete ({time_wall_following:.1f}s) — replanning')
                self.goal = None
                self.state = State.FIND_FRONTIER
                return

            front_range = self._front_range()
            right_range = self.ranges[1] if len(self.ranges) > 1 else 999.0
            side_range  = right_range

            self.get_logger().info(
                f'Wall following — time={time_wall_following:.1f}s/{WALL_FOLLOW_TIMEOUT:.0f}s '
                f'front={front_range:.2f}m side={side_range:.2f}m')

            if side_range > 0.1:
                vx, vy, wz, wf_state = self.wall_follower.wall_follower(
                    front_range, side_range, self.angles[2],
                    self.wf_direction, now)

                # Add safety correction on top of wall follower output
                vy_correction = self._get_wall_correction()
                vy_total = max(-MAX_LATERAL_SPEED,
                               min(MAX_LATERAL_SPEED, vy + vy_correction))

                self._publish_vel(x=vx, y=vy_total, wz=wz)
            else:
                self._publish_vel()

        # ── STATE 5: DONE ─────────────────────────────────────────────────────
        # Return home with wall correction active
        elif self.state == State.DONE:
            if self.start_pos is None:
                self.state = State.LANDING
                return

            dx = self.start_pos[0] - self.position[0]
            dy = self.start_pos[1] - self.position[1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > 0.3:
                vx, vy, wz = self._steer_toward(dx, dy, dist)
                vy_correction = self._get_wall_correction()
                vy_total = max(-MAX_LATERAL_SPEED,
                               min(MAX_LATERAL_SPEED, vy + vy_correction))
                self._publish_vel(x=vx, y=vy_total, wz=wz)
                self.get_logger().info(f'Returning home — {dist:.2f}m remaining')
            else:
                self.get_logger().info('Home reached — landing')
                self.state = State.LANDING

        # ── LANDING ───────────────────────────────────────────────────────────
        # No wall correction during landing
        elif self.state == State.LANDING:
            self._publish_vel(z=-0.2)
            if self.position[2] < 0.1:
                self.timer.cancel()
                self._publish_vel()
                self.get_logger().info('Landed. Exploration complete!')

    # ══════════════════════════════════════════════════════════════════════════
    # Wall safety and corridor centering
    # ══════════════════════════════════════════════════════════════════════════

    def _get_wall_correction(self):
        """
        Returns a lateral velocity correction (vy) to:
          1. Keep at least WALL_SAFE_DIST from side walls (priority)
          2. Centre the drone in corridors where both sides have walls nearby

        Positive vy = push left (in drone body frame)
        Negative vy = push right (in drone body frame)

        The ranges array is: [back=0, right=1, front=2, left=3]
        """
        if len(self.ranges) < 4:
            return 0.0

        right_range = self.ranges[1] if self.ranges[1] > 0.0 else 999.0
        left_range  = self.ranges[3] if self.ranges[3] > 0.0 else 999.0

        # ── Priority 1: Safety push ───────────────────────────────────────────
        # If either side is too close, push directly away from that wall
        # This overrides centering entirely
        if right_range < WALL_SAFE_DIST:
            # Too close to right wall — push left (positive vy)
            error = WALL_SAFE_DIST - right_range   # how far inside safe zone
            correction = WALL_KP_SAFETY * error
            self.get_logger().warn(
                f'Wall safety: right={right_range:.2f}m < {WALL_SAFE_DIST}m '
                f'→ pushing left by {correction:.2f} m/s')
            return min(correction, MAX_LATERAL_SPEED)

        if left_range < WALL_SAFE_DIST:
            # Too close to left wall — push right (negative vy)
            error = WALL_SAFE_DIST - left_range
            correction = WALL_KP_SAFETY * error
            self.get_logger().warn(
                f'Wall safety: left={left_range:.2f}m < {WALL_SAFE_DIST}m '
                f'→ pushing right by {correction:.2f} m/s')
            return -min(correction, MAX_LATERAL_SPEED)

        # ── Priority 2: Corridor centering ───────────────────────────────────
        # Only activate if BOTH sides detect a wall within WALL_CENTRE_RANGE
        # This avoids trying to centre in open space where there's nothing to
        # centre between
        if right_range < WALL_CENTRE_RANGE and left_range < WALL_CENTRE_RANGE:
            # Positive error = closer to right wall = push left
            # Negative error = closer to left wall = push right
            error = right_range - left_range
            correction = WALL_KP_CENTRE * error
            correction = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, correction))
            if abs(correction) > 0.02:   # ignore tiny corrections (dead zone)
                self.get_logger().info(
                    f'Corridor centering: right={right_range:.2f}m '
                    f'left={left_range:.2f}m → vy correction={correction:.2f}')
            return correction

        # Open space — no correction needed
        return 0.0

    # ══════════════════════════════════════════════════════════════════════════
    # Frontier detection with clustering
    # ══════════════════════════════════════════════════════════════════════════

    def _get_frontier_cells(self):
        """
        Find every individual frontier cell in the map.
        A frontier cell is FREE (0) with at least one UNKNOWN (-1) neighbour.
        Returns a set of (row, col) grid positions.
        """
        if self.map_data is None:
            return set()

        W = self.map_width
        H = self.map_height
        frontier_cells = set()

        for row in range(1, H - 1, FRONTIER_STEP):
            for col in range(1, W - 1, FRONTIER_STEP):
                if self.map_data[row * W + col] != 0:
                    continue
                neighbours = [
                    self.map_data[(row - 1) * W + col],
                    self.map_data[(row + 1) * W + col],
                    self.map_data[row * W + (col - 1)],
                    self.map_data[row * W + (col + 1)],
                ]
                if -1 in neighbours:
                    frontier_cells.add((row, col))

        return frontier_cells

    def _cluster_frontier_cells(self, frontier_cells):
        """
        Group frontier cells into connected clusters using BFS.
        Returns a list of clusters, each cluster is a list of (row, col).
        """
        unvisited = set(frontier_cells)
        clusters  = []
        step      = FRONTIER_STEP

        while unvisited:
            seed    = next(iter(unvisited))
            cluster = []
            queue   = deque([seed])
            unvisited.remove(seed)

            while queue:
                row, col = queue.popleft()
                cluster.append((row, col))

                for dr in [-step, 0, step]:
                    for dc in [-step, 0, step]:
                        if dr == 0 and dc == 0:
                            continue
                        neighbour = (row + dr, col + dc)
                        if neighbour in unvisited:
                            unvisited.remove(neighbour)
                            queue.append(neighbour)

            clusters.append(cluster)

        return clusters

    def _get_frontier_clusters(self):
        """
        Find, cluster, and filter frontiers.
        Returns a list of (x, y, distance, size) sorted closest first.
        Clusters smaller than MIN_CLUSTER_SIZE are rejected as sensor noise.
        """
        px = self.position[0]
        py = self.position[1]

        frontier_cells = self._get_frontier_cells()
        if not frontier_cells:
            return []

        clusters = self._cluster_frontier_cells(frontier_cells)
        valid_clusters = []

        for cluster in clusters:
            if len(cluster) < MIN_CLUSTER_SIZE:
                self.get_logger().info(
                    f'  Rejected cluster of {len(cluster)} cells (too small)')
                continue

            avg_row = sum(r for r, c in cluster) / len(cluster)
            avg_col = sum(c for r, c in cluster) / len(cluster)
            wx = self.map_origin[0] + (avg_col + 0.5) * MAP_RES
            wy = self.map_origin[1] + (avg_row + 0.5) * MAP_RES

            dx = wx - px
            dy = wy - py
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < MIN_FRONTIER_DIST:
                continue

            valid_clusters.append((wx, wy, dist, len(cluster)))

        valid_clusters.sort(key=lambda c: c[2])
        return valid_clusters

    # ══════════════════════════════════════════════════════════════════════════
    # Navigation helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _steer_toward(self, dx, dy, dist):
        """
        Return (vx, vy, wz) to steer toward an offset (dx, dy).
        Turns on the spot until roughly facing the goal, then moves forward.
        vy is 0 here — wall correction is added on top by the caller.
        """
        yaw         = self.angles[2]
        desired_yaw = math.atan2(dy, dx)
        yaw_error   = self._wrap_angle(desired_yaw - yaw)

        if abs(yaw_error) < 0.5:
            vx = min(CRUISE_SPEED, dist)
        else:
            vx = 0.0

        wz = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, 3.0 * yaw_error))
        return vx, 0.0, wz

    def _front_range(self):
        if len(self.ranges) > 2 and self.ranges[2] > 0.0:
            return self.ranges[2]
        return 999.0

    def _wrap_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

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