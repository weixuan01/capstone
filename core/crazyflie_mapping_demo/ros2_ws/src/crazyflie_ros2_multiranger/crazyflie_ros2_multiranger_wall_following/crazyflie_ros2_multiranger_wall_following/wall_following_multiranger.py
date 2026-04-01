#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Trigger

import tf_transformations
import math
import numpy as np
from .wall_following.wall_following import WallFollowing
import time

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1


class WallFollowingMultiranger(Node):
    def __init__(self):

        super().__init__('simple_mapper_multiranger')
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value
        self.declare_parameter('max_turn_rate', 0.5)
        max_turn_rate = self.get_parameter('max_turn_rate').value
        self.declare_parameter('max_forward_speed', 0.5)
        max_forward_speed = self.get_parameter('max_forward_speed').value
        self.declare_parameter('wall_following_direction', 'right')
        self.wall_following_direction = self.get_parameter('wall_following_direction').value
        self.declare_parameter('invert_yaw_command', False)
        self.invert_yaw_command = self.get_parameter('invert_yaw_command').value

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)

        self.declare_parameter('takeoff_ready_time', 1.0)
        self.takeoff_ready_time = float(self.get_parameter('takeoff_ready_time').value)

        self.last_odom_time = None
        self.ready_since = None
        self.takeoff_sent = False

        self.wait_for_start = True
        self.start_clock = None

        # add service to stop wall following and make the crazyflie land
        self.srv = self.create_service(Trigger, robot_prefix + '/stop_wall_following', self.stop_wall_following_cb)

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]        
        self.last_debug_time = 0.0
        self.last_state = None

        self.position_update = False

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(f"Wall following set for crazyflie " + robot_prefix +
                               f" using the scan topic with a delay of {self.delay} seconds")

        # Create a timer to run the wall following state machine
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Initialize wall following state machine
        self.wall_following = WallFollowing(
                max_turn_rate=max_turn_rate,
                max_forward_speed=max_forward_speed,
                init_state=WallFollowing.StateWallFollowing.FORWARD)

        # Give a take off command but wait for the delay to start the wall following
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9

    def stop_wall_following_cb(self, request, response):
        self.get_logger().info('Stopping wall following')
        self.timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = -0.2
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

        response.success = True

        return response

    def timer_callback(self):
        time_now = self.get_clock().now().nanoseconds * 1e-9

        # Phase 1: wait for fresh odom before takeoff
        if not self.takeoff_sent:
            odom_fresh = (
                self.position_update and
                self.last_odom_time is not None and
                (time_now - self.last_odom_time) < 0.2
            )

            if odom_fresh:
                if self.ready_since is None:
                    self.ready_since = time_now
                elif (time_now - self.ready_since) >= self.takeoff_ready_time:
                    self.get_logger().info('Estimator ready, sending takeoff trigger')
                    msg = Twist()
                    msg.linear.z = 0.5
                    self.twist_publisher.publish(msg)

                    self.takeoff_sent = True
                    self.start_clock = time_now
            else:
                self.ready_since = None

            return

        # Phase 2: hover and settle before wall following
        if self.wait_for_start:
            hold = Twist()
            self.twist_publisher.publish(hold)

            if time_now - self.start_clock > self.delay:
                self.get_logger().info('Starting wall following')
                self.wait_for_start = False

            return

        # Phase 3: actual wall following
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
                front_range, side_range, actual_yaw_rad, wf_dir, time_now
            )
        
        # for debugging
        if state_wf != self.last_state or (time_now - self.last_debug_time) > 0.5:
            self.get_logger().info(
                f"state={state_wf.name} front={front_range:.2f} side={side_range:.2f} "
                f"yaw={actual_yaw_rad:.2f} cmd_yaw={yaw_rate:.2f}"
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
        self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges

def main(args=None):

    rclpy.init(args=args)
    wall_following_multiranger = WallFollowingMultiranger()
    rclpy.spin(wall_following_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
