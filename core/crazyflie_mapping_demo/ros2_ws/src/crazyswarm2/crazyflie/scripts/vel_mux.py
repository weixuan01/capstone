#!/usr/bin/env python3

"""
A Twist message handler that get incoming twist messages from 
    external packages and handles proper takeoff, landing and
    hover commands of connected crazyflie in the crazyflie_server
    node

    2022 - K. N. McGuire (Bitcraze AB)
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from crazyflie_interfaces.srv import Takeoff, Land, NotifySetpointsStop
from crazyflie_interfaces.msg import Hover
from std_msgs.msg import Float32
import time

class VelMux(Node):
    def __init__(self):
        super().__init__('vel_mux')
        self.declare_parameter('hover_height', 0.5)
        self.declare_parameter('robot_prefix', '/cf')
        self.declare_parameter('incoming_twist_topic', '/cmd_vel_safe')

        self.hover_height  = self.get_parameter('hover_height').value
        robot_prefix  = self.get_parameter('robot_prefix').value
        incoming_twist_topic  = self.get_parameter('incoming_twist_topic').value

        # Current altitude setpoint. Starts at hover_height and is overridden
        # by the CA node via /{prefix}/target_height during resolution.
        self.current_hover_height = self.hover_height
        
        self.subscription = self.create_subscription(
            Twist,
            robot_prefix + incoming_twist_topic,
            self.cmd_vel_callback,
            10)

        # CA node publishes altitude separation targets here.
        self.create_subscription(
            Float32,
            robot_prefix + '/target_height',
            self._target_height_callback,
            10)

        self.msg_cmd_vel = Twist()
        self.received_first_cmd_vel = False
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.takeoff_until = 0.0
        self.takeoff_client = self.create_client(Takeoff, robot_prefix + '/takeoff')
        self.publisher_hover = self.create_publisher(Hover, robot_prefix + '/cmd_hover', 10)
        self.land_client = self.create_client(Land, robot_prefix + '/land')
        self.notify_client = self.create_client(NotifySetpointsStop, robot_prefix + '/notify_setpoints_stop')
        self.cf_has_taken_off = False

        self.takeoff_client.wait_for_service()
        self.land_client.wait_for_service()

        self.get_logger().info(f"Velocity Multiplexer set for {robot_prefix}"+
                               f" with height {self.hover_height} m using the {incoming_twist_topic} topic")

    def _target_height_callback(self, msg: Float32):
        """Override the hover altitude when the CA node requests separation.
        Only acts after takeoff — ignored on the ground."""
        if self.cf_has_taken_off:
            self.current_hover_height = float(msg.data)
            self.get_logger().debug(
                f'[VelMux] target_height override → {self.current_hover_height:.2f}m')

    def cmd_vel_callback(self, msg):
        self.msg_cmd_vel = msg
        msg_is_zero = msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0 and msg.linear.z == 0.0
        if msg_is_zero is False and self.received_first_cmd_vel is False and msg.linear.z >= 0.0:
            self.received_first_cmd_vel = True

    def timer_callback(self):
        now = time.monotonic()

        if self.received_first_cmd_vel and self.cf_has_taken_off is False:
            req = Takeoff.Request()
            req.height = self.hover_height
            req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
            self.takeoff_client.call_async(req)

            self.cf_has_taken_off = True
            self.takeoff_until = now + 2.0
            # Sync current_hover_height to actual takeoff height
            self.current_hover_height = self.hover_height
            return

        if self.received_first_cmd_vel and self.cf_has_taken_off:
            if self.msg_cmd_vel.linear.z >= 0.0:
                msg = Hover()
                msg.z_distance = self.current_hover_height  # CA node can override this

                # During takeoff, force pure hover
                if now < self.takeoff_until:
                    msg.vx = 0.0
                    msg.vy = 0.0
                    msg.yaw_rate = 0.0
                else:
                    msg.vx = self.msg_cmd_vel.linear.x
                    msg.vy = self.msg_cmd_vel.linear.y
                    msg.yaw_rate = self.msg_cmd_vel.angular.z

                self.publisher_hover.publish(msg)
            else:
                req = NotifySetpointsStop.Request()
                self.notify_client.call_async(req)

                req = Land.Request()
                req.height = 0.1
                req.duration = rclpy.duration.Duration(seconds=2.0).to_msg()
                self.land_client.call_async(req)

                self.cf_has_taken_off = False
                self.received_first_cmd_vel = False
                self.takeoff_until = 0.0
                # Reset hover height for next flight
                self.current_hover_height = self.hover_height

def main(args=None):
    rclpy.init(args=args)

    vel_mux = VelMux()

    rclpy.spin(vel_mux)

    vel_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
