#!/usr/bin/env python3

""" 
Simple square pattern node for Crazyflie in ROS 2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import tf_transformations
import math
import time

class SquarePatternMultiranger(Node):
    def __init__(self):
        super().__init__('simple_mapper_multiranger')
        
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value
        
        self.declare_parameter('max_forward_speed', 0.5)
        self.forward_speed = self.get_parameter('max_forward_speed').value
        
        self.declare_parameter('max_turn_rate', 0.5)
        self.turn_rate = self.get_parameter('max_turn_rate').value

        # New parameter for the square dimensions
        self.declare_parameter('square_size', 1.0) # in meters
        self.square_size = self.get_parameter('square_size').value

        # Odometry subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)

        # Service to stop the drone
        self.srv = self.create_service(Trigger, robot_prefix + '/stop_pattern', self.stop_square_pattern_cb)

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.position_update = False

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(f"Square pattern set for {robot_prefix} " +
                               f"with a delay of {self.delay} seconds")

        # Create a timer to run the state machine
        self.timer = self.create_timer(0.01, self.timer_callback)

        # State machine initialization
        self.state = 'WAIT_TAKEOFF'
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        self.side_count = 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_yaw = 0.0
        

        # Give a take off command initially
        msg = Twist()
        msg.linear.z = 0.3
        self.twist_publisher.publish(msg)
        
    def stop_square_pattern_cb(self, request, response):
        self.get_logger().info('Stopping square pattern')
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
        msg = Twist()
        actual_yaw_rad = self.angles[2]

        # STATE: Wait for takeoff delay
        if self.state == 'WAIT_TAKEOFF':
            if self.get_clock().now().nanoseconds * 1e-9 - self.start_clock > self.delay:
                self.get_logger().info('Starting square pattern: FORWARD')
                self.state = 'FORWARD'
                self.start_x = self.position[0]
                self.start_y = self.position[1]
            else:
                return # Keep hovering until delay is done

        # STATE: Fly forward until distance is reached
        elif self.state == 'FORWARD':
            msg.linear.x = self.forward_speed
            
            # Calculate distance traveled from the start of this side
            dist = math.hypot(self.position[0] - self.start_x, self.position[1] - self.start_y)
            
            if dist >= self.square_size:
                self.get_logger().info('Distance reached. Turning 90 degrees.')
                self.state = 'TURN'
                # Set target yaw to current yaw + 90 degrees (pi/2)
                self.target_yaw = actual_yaw_rad + (math.pi / 2.0)
                # Normalize target yaw to be between -pi and pi
                while self.target_yaw > math.pi: self.target_yaw -= 2 * math.pi
                while self.target_yaw < -math.pi: self.target_yaw += 2 * math.pi

        # STATE: Turn until target yaw is reached
        elif self.state == 'TURN':
            msg.angular.z = self.turn_rate

            # Calculate the difference to the target angle
            yaw_diff = self.target_yaw - actual_yaw_rad
            # Normalize yaw difference
            while yaw_diff > math.pi: yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi: yaw_diff += 2 * math.pi

            # If we are within ~5.7 degrees (0.1 radians) of the target, stop turning
            if abs(yaw_diff) < 0.1:
                self.side_count += 1
                self.get_logger().info(f'Turn complete. Total sides finished: {self.side_count}')
                
                if self.side_count >= 4:
                    self.get_logger().info('Square complete! Starting next lap.')   
                    self.side_count = 0      
                              
                self.state = 'FORWARD'
                self.start_x = self.position[0]
                self.start_y = self.position[1]
	
        # Publish the velocity command continuously
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

def main(args=None):
    rclpy.init(args=args)
    square_pattern = SquarePatternMultiranger()
    rclpy.spin(square_pattern)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
