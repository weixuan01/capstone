#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import tf_transformations

from .wall_following.wall_following import WallFollowing


class WallFollowingMultiranger(Node):
    def __init__(self):
        super().__init__('wall_following_multiranger')

        self.declare_parameter('robot_prefix', 'crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        self.declare_parameter('delay', 5.0)
        self.delay = float(self.get_parameter('delay').value)

        self.declare_parameter('max_turn_rate', 0.5)
        max_turn_rate = float(self.get_parameter('max_turn_rate').value)

        self.declare_parameter('max_forward_speed', 0.5)
        max_forward_speed = float(self.get_parameter('max_forward_speed').value)

        self.declare_parameter('wall_following_direction', 'right')
        self.wall_following_direction = self.get_parameter('wall_following_direction').value

        self.declare_parameter('invert_yaw_command', False)
        self.invert_yaw_command = bool(self.get_parameter('invert_yaw_command').value)

        # trigger: delayed one-shot positive-z command, then silence during startup.
        #          This matches the simulator/control_services and real/vel_mux behavior.
        # direct:  continuous positive-z command until odom height is reached.
        self.declare_parameter('takeoff_mode', 'trigger')
        self.takeoff_mode = str(self.get_parameter('takeoff_mode').value).strip().lower()

        self.declare_parameter('takeoff_ready_time', 1.0)
        self.takeoff_ready_time = float(self.get_parameter('takeoff_ready_time').value)

        self.declare_parameter('takeoff_speed', 0.5)
        self.takeoff_speed = float(self.get_parameter('takeoff_speed').value)

        self.declare_parameter('takeoff_height', 0.55)
        self.takeoff_height = float(self.get_parameter('takeoff_height').value)

        self.declare_parameter('takeoff_height_tolerance', 0.03)
        self.takeoff_height_tolerance = float(self.get_parameter('takeoff_height_tolerance').value)

        self.declare_parameter('takeoff_timeout', 6.0)
        self.takeoff_timeout = float(self.get_parameter('takeoff_timeout').value)

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)

        self.srv = self.create_service(
            Trigger,
            robot_prefix + '/stop_wall_following',
            self.stop_wall_following_cb,
        )

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]

        self.position_update = False
        self.last_odom_time = None
        self.ready_since = None

        self.takeoff_start_time = None
        self.takeoff_trigger_sent = False
        self.takeoff_finished = False

        self.wait_for_start = True
        self.start_clock = None

        self.last_debug_time = 0.0
        self.last_state = None

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.wall_following = WallFollowing(
            max_turn_rate=max_turn_rate,
            max_forward_speed=max_forward_speed,
            init_state=WallFollowing.StateWallFollowing.FORWARD,
        )

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(
            f'Wall following configured for {robot_prefix}; '
            f'direction={self.wall_following_direction}, '
            f'takeoff_mode={self.takeoff_mode}, delay={self.delay:.1f}s'
        )

    def stop_wall_following_cb(self, request, response):
        self.get_logger().info('Stopping wall following')
        self.timer.cancel()

        msg = Twist()
        msg.linear.z = -0.2
        self.twist_publisher.publish(msg)

        response.success = True
        response.message = 'Wall following stopped'
        return response

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_is_fresh(self, time_now):
        return (
            self.position_update
            and self.last_odom_time is not None
            and (time_now - self.last_odom_time) < 0.2
        )

    def _takeoff_height_reached(self):
        return self.position[2] >= (self.takeoff_height - self.takeoff_height_tolerance)

    def _publish_takeoff_twist(self):
        msg = Twist()
        msg.linear.z = self.takeoff_speed
        self.twist_publisher.publish(msg)

    def _handle_takeoff(self, time_now):
        if not self._odom_is_fresh(time_now):
            self.ready_since = None
            return False

        if self.ready_since is None:
            self.ready_since = time_now
            return False

        if (time_now - self.ready_since) < self.takeoff_ready_time:
            return False

        if self.takeoff_start_time is None:
            self.takeoff_start_time = time_now

        # For simulator control_services and real vel_mux, a delayed one-shot
        # positive-z trigger is enough. Do NOT publish startup zero twists after it,
        # otherwise the simulator can lose the takeoff latch before it sets is_flying.
        if self.takeoff_mode == 'trigger':
            if not self.takeoff_trigger_sent:
                self.get_logger().info('Estimator ready, sending delayed one-shot takeoff trigger')
                self._publish_takeoff_twist()
                self.takeoff_trigger_sent = True
                self.takeoff_finished = True
                self.start_clock = time_now
                return True
            return False

        if self.takeoff_mode == 'direct':
            if self.takeoff_start_time == time_now:
                self.get_logger().info('Estimator ready, starting direct takeoff')
            self._publish_takeoff_twist()
        else:
            self.get_logger().warn(
                f"Unknown takeoff_mode '{self.takeoff_mode}', falling back to trigger"
            )
            self.takeoff_mode = 'trigger'
            if not self.takeoff_trigger_sent:
                self._publish_takeoff_twist()
                self.takeoff_trigger_sent = True
                self.takeoff_finished = True
                self.start_clock = time_now
                return True

        reached_altitude = self._takeoff_height_reached()
        timed_out = (time_now - self.takeoff_start_time) >= self.takeoff_timeout

        if reached_altitude or timed_out:
            # In direct mode, it is safe to publish a zero hold command.
            hold = Twist()
            self.twist_publisher.publish(hold)

            reason = 'height reached' if reached_altitude else 'timeout reached'
            self.get_logger().info(
                f'Takeoff phase complete ({reason}), z={self.position[2]:.2f} m'
            )

            self.takeoff_finished = True
            self.start_clock = time_now
            return True

        return False

    def timer_callback(self):
        time_now = self._now()

        # Phase 1: wait for odometry and complete takeoff
        if not self.takeoff_finished:
            self._handle_takeoff(time_now)
            return

        # Phase 2: settle before starting wall following
        if self.wait_for_start:
            # Important: stay silent in trigger mode so the upstream control node
            # keeps its internally latched takeoff/hover state.
            if self.takeoff_mode == 'direct':
                hold = Twist()
                self.twist_publisher.publish(hold)

            if (time_now - self.start_clock) > self.delay:
                self.get_logger().info('Starting wall following')
                self.wait_for_start = False

            return

        # Phase 3: wall following
        velocity_x = 0.0
        velocity_y = 0.0
        yaw_rate = 0.0
        state_wf = WallFollowing.StateWallFollowing.HOVER

        actual_yaw_rad = self.angles[2]

        right_range = self.ranges[1]
        front_range = self.ranges[2]
        left_range = self.ranges[3]

        if self.wall_following_direction == 'right':
            wf_dir = WallFollowing.WallFollowingDirection.RIGHT
            side_range = left_range
        else:
            wf_dir = WallFollowing.WallFollowingDirection.LEFT
            side_range = right_range

        if side_range > 0.1:
            velocity_x, velocity_y, yaw_rate, state_wf = self.wall_following.wall_follower(
                front_range,
                side_range,
                actual_yaw_rad,
                wf_dir,
                time_now,
            )

        if state_wf != self.last_state or (time_now - self.last_debug_time) > 0.5:
            self.get_logger().info(
                f"state={state_wf.name} front={front_range:.2f} side={side_range:.2f} "
                f"z={self.position[2]:.2f} yaw={actual_yaw_rad:.2f} "
                f"vx={velocity_x:.2f} vy={velocity_y:.2f} cmd_yaw={yaw_rate:.2f}"
            )
            self.last_state = state_wf
            self.last_debug_time = time_now

        cmd_yaw = -yaw_rate if self.invert_yaw_command else yaw_rate

        msg = Twist()
        msg.linear.x = velocity_x
        msg.linear.y = velocity_y
        msg.angular.z = cmd_yaw
        self.twist_publisher.publish(msg)

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]

        self.position_update = True
        self.last_odom_time = self._now()

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingMultiranger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()