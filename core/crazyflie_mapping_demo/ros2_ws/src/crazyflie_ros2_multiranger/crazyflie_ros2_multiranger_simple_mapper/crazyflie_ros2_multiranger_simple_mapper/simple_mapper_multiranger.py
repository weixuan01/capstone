#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import tf_transformations
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class SimpleMapperMultiranger(Node):
    def __init__(self):
        super().__init__('simple_mapper_multiranger')

        self.declare_parameter('robot_prefix', '/crazyflie')
        self.robot_prefix = self.get_parameter('robot_prefix').value

        # Map geometry
        self.declare_parameter('map_size_x', 40.0)
        self.declare_parameter('map_size_y', 40.0)
        self.declare_parameter('map_resolution', 0.1)

        self.map_size_x = float(self.get_parameter('map_size_x').value)
        self.map_size_y = float(self.get_parameter('map_size_y').value)
        self.map_resolution = float(self.get_parameter('map_resolution').value)

        if self.map_size_x <= 0.0 or self.map_size_y <= 0.0 or self.map_resolution <= 0.0:
            raise ValueError('map_size_x, map_size_y, and map_resolution must be > 0')

        self.map_width = int(round(self.map_size_x / self.map_resolution))
        self.map_height = int(round(self.map_size_y / self.map_resolution))

        # Startup/mapping gating
        self.declare_parameter('min_mapping_height', 0.15)
        self.declare_parameter('mapping_start_delay', 1.0)
        self.declare_parameter('require_fresh_odom', True)
        self.declare_parameter('recenter_initial_yaw', False)

        self.min_mapping_height = float(self.get_parameter('min_mapping_height').value)
        self.mapping_start_delay = float(self.get_parameter('mapping_start_delay').value)
        self.require_fresh_odom = bool(self.get_parameter('require_fresh_odom').value)
        self.recenter_initial_yaw = bool(self.get_parameter('recenter_initial_yaw').value)

        self.odom_subscriber = self.create_subscription(
            Odometry, self.robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, self.robot_prefix + '/scan', self.scan_subscribe_callback, 10)
        self.prediction_subscriber = self.create_subscription(
            Int32, '/aideck/digit_prediction', self.prediction_callback, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.robot_prefix + '/digit_markers', 10)

        self.pending_digit = None
        self.digit_markers = []
        self.next_marker_id = 0

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]
        self.range_max = 3.5
        self.position_update = False
        self.last_odom_time = None

        self.map = [-1] * (self.map_width * self.map_height)

        # Keep OccupancyGrid centered in the map frame.
        self.map_origin_x = -self.map_size_x / 2.0
        self.map_origin_y = -self.map_size_y / 2.0

        # Initialize map->odom transform later, once startup pose is stable.
        self.map_initialized = False
        self.map_ready_since = None
        self.tfbr = TransformBroadcaster(self)
        self.tf_translation_x = 0.0
        self.tf_translation_y = 0.0
        self.tf_yaw = 0.0

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            self.robot_prefix + '/map',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        self.get_logger().info(
            f"Simple mapper set for {self.robot_prefix}. Map {self.map_size_x:.1f}m x {self.map_size_y:.1f}m @ {self.map_resolution:.2f}m/cell"
        )

    def prediction_callback(self, msg):
        self.pending_digit = int(msg.data)

    def get_wall_points(self):
        points = {}
        o = self.position
        yaw = self.angles[2]

        roll = 0.0
        pitch = 0.0

        r_back = self.ranges[0]
        r_right = self.ranges[1]
        r_front = self.ranges[2]
        r_left = self.ranges[3]

        if r_left < self.range_max and r_left != 0.0 and not math.isinf(r_left):
            left = [o[0], o[1] + r_left, o[2]]
            points["left"] = self.rot(roll, pitch, yaw, o, left)

        if r_right < self.range_max and r_right != 0.0 and not math.isinf(r_right):
            right = [o[0], o[1] - r_right, o[2]]
            points["right"] = self.rot(roll, pitch, yaw, o, right)

        if r_front < self.range_max and r_front != 0.0 and not math.isinf(r_front):
            front = [o[0] + r_front, o[1], o[2]]
            points["front"] = self.rot(roll, pitch, yaw, o, front)

        if r_back < self.range_max and r_back != 0.0 and not math.isinf(r_back):
            back = [o[0] - r_back, o[1], o[2]]
            points["back"] = self.rot(roll, pitch, yaw, o, back)

        return points

    def add_digit_marker(self, x_map, y_map, digit_text):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'digit_labels'
        marker.id = self.next_marker_id
        self.next_marker_id += 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = float(x_map)
        marker.pose.position.y = float(y_map)
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = str(digit_text)
        marker.lifetime = Duration(sec=0, nanosec=0)

        self.digit_markers.append(marker)

    def publish_digit_markers(self):
        arr = MarkerArray()
        arr.markers = self.digit_markers
        self.marker_publisher.publish(arr)

    def publish_map_to_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.robot_prefix + '/odom'
        t.transform.translation.x = float(self.tf_translation_x)
        t.transform.translation.y = float(self.tf_translation_y)
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.tf_yaw)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tfbr.sendTransform(t)

    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return points

    def transform_odom_to_map_xy(self, x_odom, y_odom):
        cos_yaw = math.cos(self.tf_yaw)
        sin_yaw = math.sin(self.tf_yaw)
        x_map = cos_yaw * x_odom - sin_yaw * y_odom + self.tf_translation_x
        y_map = sin_yaw * x_odom + cos_yaw * y_odom + self.tf_translation_y
        return x_map, y_map

    def world_to_grid(self, x_map, y_map):
        mx = int((x_map - self.map_origin_x) / self.map_resolution)
        my = int((y_map - self.map_origin_y) / self.map_resolution)
        return mx, my

    def in_bounds(self, mx, my):
        return 0 <= mx < self.map_width and 0 <= my < self.map_height

    def map_index(self, mx, my):
        return my * self.map_width + mx

    def ready_to_map(self):
        if not self.position_update:
            self.map_ready_since = None
            return False

        now = self.get_clock().now().nanoseconds * 1e-9
        odom_fresh = (
            self.last_odom_time is not None and
            (now - self.last_odom_time) < 0.2
        )

        if self.require_fresh_odom and not odom_fresh:
            self.map_ready_since = None
            return False

        if self.position[2] < self.min_mapping_height:
            self.map_ready_since = None
            return False

        if self.map_initialized:
            return True

        if self.map_ready_since is None:
            self.map_ready_since = now
            return False

        if (now - self.map_ready_since) < self.mapping_start_delay:
            return False

        # Freeze startup pose and recenter odom into map so startup pose becomes map (0, 0).
        x0 = float(self.position[0])
        y0 = float(self.position[1])
        yaw0 = float(self.angles[2]) if self.recenter_initial_yaw else 0.0

        self.tf_yaw = -yaw0
        cos_yaw = math.cos(self.tf_yaw)
        sin_yaw = math.sin(self.tf_yaw)
        self.tf_translation_x = -(cos_yaw * x0 - sin_yaw * y0)
        self.tf_translation_y = -(sin_yaw * x0 + cos_yaw * y0)

        self.map = [-1] * (self.map_width * self.map_height)
        self.map_initialized = True
        self.publish_map_to_odom_tf()

        self.get_logger().info(
            'Map initialized. '
            f'startup odom=({x0:.2f}, {y0:.2f}, {self.position[2]:.2f}), '
            f'map->odom tx={self.tf_translation_x:.2f}, ty={self.tf_translation_y:.2f}, yaw={self.tf_yaw:.2f}'
        )
        return True

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
        self.publish_map_to_odom_tf()

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges
        self.range_max = msg.range_max

        if not self.ready_to_map():
            return

        wall_points = self.get_wall_points()
        data = list(wall_points.values())

        robot_x_map, robot_y_map = self.transform_odom_to_map_xy(self.position[0], self.position[1])
        robot_mx, robot_my = self.world_to_grid(robot_x_map, robot_y_map)
        if not self.in_bounds(robot_mx, robot_my):
            self.get_logger().warn(
                f'Robot pose fell outside map bounds: mx={robot_mx}, my={robot_my}, '
                f'x_map={robot_x_map:.2f}, y_map={robot_y_map:.2f}'
            )
            return

        for px_odom, py_odom, _ in data:
            point_x_map, point_y_map = self.transform_odom_to_map_xy(px_odom, py_odom)
            point_mx, point_my = self.world_to_grid(point_x_map, point_y_map)
            if not self.in_bounds(point_mx, point_my):
                continue

            for line_x, line_y in self.bresenham_line(robot_mx, robot_my, point_mx, point_my):
                if self.in_bounds(line_x, line_y):
                    self.map[self.map_index(line_x, line_y)] = 0

            self.map[self.map_index(point_mx, point_my)] = 100

        front_point = wall_points.get("front", None)

        if self.pending_digit is not None and front_point is not None:
            px_odom, py_odom, _ = front_point
            px_map, py_map = self.transform_odom_to_map_xy(px_odom, py_odom)
            self.add_digit_marker(px_map, py_map, str(self.pending_digit))
            self.pending_digit = None

        self.publish_map()
        self.publish_digit_markers()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map
        self.map_publisher.publish(msg)

    def rotate_and_create_points(self):
        data = []
        o = self.position

        # Yaw-only is more robust for a 2D occupancy map.
        roll = 0.0
        pitch = 0.0
        yaw = self.angles[2]

        r_back = self.ranges[0]
        r_right = self.ranges[1]
        r_front = self.ranges[2]
        r_left = self.ranges[3]

        if r_left < self.range_max and r_left != 0.0 and not math.isinf(r_left):
            left = [o[0], o[1] + r_left, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, left))

        if r_right < self.range_max and r_right != 0.0 and not math.isinf(r_right):
            right = [o[0], o[1] - r_right, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, right))

        if r_front < self.range_max and r_front != 0.0 and not math.isinf(r_front):
            front = [o[0] + r_front, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, front))

        if r_back < self.range_max and r_back != 0.0 and not math.isinf(r_back):
            back = [o[0] - r_back, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, back))

        return data

    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos(roll)
        cosp = math.cos(pitch)
        cosy = math.cos(yaw)

        sinr = math.sin(roll)
        sinp = math.sin(pitch)
        siny = math.sin(yaw)

        roty = np.array([
            [cosy, -siny, 0],
            [siny,  cosy, 0],
            [0,        0, 1],
        ])

        rotp = np.array([
            [ cosp, 0, sinp],
            [    0, 1,    0],
            [-sinp, 0, cosp],
        ])

        rotr = np.array([
            [1,    0,     0],
            [0, cosr, -sinr],
            [0, sinr,  cosr],
        ])

        rot_first = np.dot(rotr, rotp)
        rot = np.array(np.dot(rot_first, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapperMultiranger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()