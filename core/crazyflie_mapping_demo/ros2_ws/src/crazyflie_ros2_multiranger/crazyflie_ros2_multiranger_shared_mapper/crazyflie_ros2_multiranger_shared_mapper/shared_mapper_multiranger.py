#!/usr/bin/env python3

"""
Shared mapper for multiple Crazyflie drones using a common lighthouse
coordinate frame. Each drone's odom and scan topics are subscribed to
independently, but all drones write into a single shared OccupancyGrid.

The map origin is set dynamically by averaging the first 10 odometry
readings received from any drone, so transient startup noise cannot
lock in a bad origin and cause RViz to flicker. Once committed, the
origin never changes for the lifetime of the node.

Map saving behaviour:
  - Auto-saves to ~/map.pgm and ~/map.yaml every 30 seconds (overwrites)
  - Final save is triggered automatically on node shutdown via destroy_node
  - Manual save available at any time:
      ros2 service call /save_map std_srvs/srv/Trigger {}

Based on simple_mapper_multiranger.py by K. N. McGuire (Bitcraze AB).
"""

import os
import math
import threading
import time
from functools import partial

import numpy as np
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

import tf_transformations

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1
AUTO_SAVE_INTERVAL = 30.0   # seconds

# Log-odds occupancy update parameters.
# L_OCC is added to a cell's log-odds score on each sensor hit.
# L_FREE is added (negative, so it subtracts) along each Bresenham ray.
# L_MIN / L_MAX clamp accumulated evidence so the map can recover from
# both false positives and false negatives within a bounded number of scans.
# Occupied threshold: cells above this value are published as occupied (100).
# Free threshold: cells below this value are published as free (0).
L_OCC   =  0.85
L_FREE  = -0.40
L_MIN   = -4.0
L_MAX   =  4.0
L_OCC_THRESH  =  0.5   # log-odds above this -> publish as 100
L_FREE_THRESH = -0.5   # log-odds below this -> publish as 0
MAP_PUBLISH_HZ = 5.0   # decouple map publishing from scan rate


class SharedMapperMultiranger(Node):
    def __init__(self):
        super().__init__('shared_mapper_multiranger')

        self.declare_parameter('robot_prefixes', ['/crazyflie']) #use the same names as in the launch file
        robot_prefixes = self.get_parameter('robot_prefixes').value

        # Optional path to a previously saved map yaml.  When provided the
        # mapper pre-populates its grid from that file and skips the origin-
        # averaging step, so new scans are added on top of the existing map.
        # Example:
        #   ros2 run <pkg> shared_mapper_multiranger --ros-args -p map_file:=/home/user/map.yaml
        self.declare_parameter('map_file', '')
        map_file = self.get_parameter('map_file').value

        self.get_logger().info(f"Shared mapper starting for drones: {robot_prefixes}")

        # Single shared map and lock
        self.map_width = int(GLOBAL_SIZE_X / MAP_RES)
        self.map_height = int(GLOBAL_SIZE_Y / MAP_RES)
        n_cells = self.map_width * self.map_height
        # Log-odds grid: float32, 0.0 = unknown, positive = likely occupied,
        # negative = likely free.  Converted to 0/100/-1 only at publish time.
        self.log_odds = np.zeros(n_cells, dtype=np.float32)
        self.map_lock = threading.Lock()

        # Dynamic map origin — committed once enough odom samples have been
        # averaged, so a single unstable startup reading cannot lock in a
        # bad origin and cause RViz to flicker on every subsequent publish.
        self.map_origin_x = None
        self.map_origin_y = None
        self.origin_set = False
        self._origin_samples = []
        self._ORIGIN_SAMPLE_COUNT = 10

        # Load a prior map if one was provided.  This sets origin_set = True
        # so scans are accepted immediately without waiting for odom samples.
        if map_file:
            self._load_map_from_file(map_file)

        # Per-drone state dictionaries
        self.positions = {}
        self.angles = {}
        self.position_updates = {}
        self.range_maxes = {}
        self.mapping_active = {}  # per-drone flag — set to False when drone lands

        # Subscribe to each drone's odom and scan topics
        for prefix in robot_prefixes:
            self.positions[prefix] = [0.0, 0.0, 0.0]
            self.angles[prefix] = [0.0, 0.0, 0.0]
            self.position_updates[prefix] = False
            self.range_maxes[prefix] = 3.5
            self.mapping_active[prefix] = True

            self.create_subscription(
                Odometry,
                prefix + '/odom',
                partial(self._odom_callback, prefix=prefix),
                10
            )
            self.create_subscription(
                LaserScan,
                prefix + '/scan',
                partial(self._scan_callback, prefix=prefix),
                10
            )
            self.create_subscription(
                Bool,
                prefix + '/mapping_active',
                partial(self._mapping_active_callback, prefix=prefix),
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST)
            )
            self.get_logger().info(f"  Subscribed to {prefix}/odom and {prefix}/scan")

        # Broadcast one static transform per drone: map -> prefix/odom
        self.tfbr = StaticTransformBroadcaster(self)
        transforms = []
        for prefix in robot_prefixes:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = prefix + '/odom'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.tfbr.sendTransform(transforms)

        # Single shared map publisher
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST
            )
        )

        # Manual save service
        self.create_service(Trigger, '/save_map', self._save_map_callback)

        # Auto-save timer — fires every 30 seconds, overwrites the same file
        self.create_timer(AUTO_SAVE_INTERVAL, self._auto_save_callback)

        # Publish timer — decoupled from scan rate to avoid hammering the lock
        # with the numpy conversion on every single scan callback.
        self.create_timer(1.0 / MAP_PUBLISH_HZ, self._publish_map)

        self.get_logger().info("Shared mapper ready. Publishing to /map")
        self.get_logger().info(
            f"Auto-saving to ~/map every {AUTO_SAVE_INTERVAL:.0f}s. "
            "Manual save: ros2 service call /save_map std_srvs/srv/Trigger {}"
        )

    # ------------------------------------------------------------------ #
    # Odometry callback                                                    #
    # ------------------------------------------------------------------ #

    def _odom_callback(self, msg, prefix=''):
        self.positions[prefix][0] = msg.pose.pose.position.x
        self.positions[prefix][1] = msg.pose.pose.position.y
        self.positions[prefix][2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[prefix][0] = euler[0]
        self.angles[prefix][1] = euler[1]
        self.angles[prefix][2] = euler[2]
        self.position_updates[prefix] = True

        # Accumulate odom samples from any drone until we have enough to
        # compute a stable average, then commit the origin exactly once.
        # This prevents a single noisy startup reading from locking in a
        # bad origin and causing RViz to reinitialise on every map publish.
        if not self.origin_set:
            self._origin_samples.append(
                (msg.pose.pose.position.x, msg.pose.pose.position.y)
            )
            if len(self._origin_samples) >= self._ORIGIN_SAMPLE_COUNT:
                avg_x = sum(s[0] for s in self._origin_samples) / self._ORIGIN_SAMPLE_COUNT
                avg_y = sum(s[1] for s in self._origin_samples) / self._ORIGIN_SAMPLE_COUNT
                self.map_origin_x = avg_x - GLOBAL_SIZE_X / 2.0
                self.map_origin_y = avg_y - GLOBAL_SIZE_Y / 2.0
                self.origin_set = True
                self.get_logger().info(
                    f"Map origin committed to "
                    f"({self.map_origin_x:.3f}, {self.map_origin_y:.3f}) "
                    f"from {self._ORIGIN_SAMPLE_COUNT}-sample average "
                    f"(last sample from {prefix})"
                )

    def _mapping_active_callback(self, msg, prefix=''):
        if not msg.data and self.mapping_active[prefix]:
            self.get_logger().info(f'{prefix} is landing — scan processing disabled.')
        self.mapping_active[prefix] = msg.data

    # ------------------------------------------------------------------ #
    # Scan callback                                                        #
    # ------------------------------------------------------------------ #

    def _scan_callback(self, msg, prefix=''):
        if not self.mapping_active[prefix]:
            return
        if not self.position_updates[prefix]:
            return

        if not self.origin_set:
            return

        self.range_maxes[prefix] = msg.range_max
        hit_points = self._rotate_and_create_points(msg.ranges, msg.range_max, prefix)

        position_x_map = int(
            (self.positions[prefix][0] - self.map_origin_x) / MAP_RES)
        position_y_map = int(
            (self.positions[prefix][1] - self.map_origin_y) / MAP_RES)

        with self.map_lock:
            for point in hit_points:
                point_x = int((point[0] - self.map_origin_x) / MAP_RES)
                point_y = int((point[1] - self.map_origin_y) / MAP_RES)

                if not self._in_bounds(point_x, point_y):
                    continue

                # Apply L_FREE along the ray (all cells the beam passed through
                # are evidence of free space).
                for line_x, line_y in self._bresenham_line(
                        position_x_map, position_y_map, point_x, point_y):
                    if self._in_bounds(line_x, line_y):
                        idx = line_y * self.map_width + line_x
                        self.log_odds[idx] = max(
                            L_MIN, self.log_odds[idx] + L_FREE)

                # Apply L_OCC at the endpoint (sensor detected an obstacle here).
                idx = point_y * self.map_width + point_x
                self.log_odds[idx] = min(L_MAX, self.log_odds[idx] + L_OCC)

    # ------------------------------------------------------------------ #
    # Map publisher                                                        #
    # ------------------------------------------------------------------ #

    def _publish_map(self):
        if not self.origin_set:
            return

        with self.map_lock:
            lo = self.log_odds.copy()

        # Convert log-odds to OccupancyGrid values.
        # Cells above L_OCC_THRESH  -> 100 (occupied)
        # Cells below L_FREE_THRESH -> 0   (free)
        # Everything else           -> -1  (unknown)
        grid = np.full(len(lo), -1, dtype=np.int8)
        grid[lo >= L_OCC_THRESH]  = 100
        grid[lo <= L_FREE_THRESH] = 0

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.info.resolution = MAP_RES
        out.info.width = self.map_width
        out.info.height = self.map_height
        out.info.origin.position.x = self.map_origin_x
        out.info.origin.position.y = self.map_origin_y
        out.data = grid.tolist()

        self.map_publisher.publish(out)

    # ------------------------------------------------------------------ #
    # Load map from a previously saved yaml + pgm pair                   #
    # ------------------------------------------------------------------ #

    def _load_map_from_file(self, yaml_path):
        """
        Pre-populate the shared map grid from a nav2_map_server yaml + pgm pair.

        After a successful load:
          - map_origin_x / map_origin_y are taken from the yaml
          - origin_set is True so scan callbacks start writing immediately
          - The in-memory grid reflects the loaded occupancy data

        If the file cannot be read, a warning is logged and the mapper
        continues in the normal origin-averaging startup mode.
        """
        try:
            yaml_path = os.path.expanduser(yaml_path)
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)

            pgm_path = meta.get('image', '')
            if not os.path.isabs(pgm_path):
                # Resolve relative image path against the yaml directory.
                pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)
            pgm_path = os.path.expanduser(pgm_path)

            resolution  = float(meta.get('resolution', MAP_RES))
            origin      = meta.get('origin', [0.0, 0.0, 0.0])
            negate      = int(meta.get('negate', 0))
            occ_thresh  = float(meta.get('occupied_thresh', 0.65))
            free_thresh = float(meta.get('free_thresh', 0.196))

            img = np.array(Image.open(pgm_path).convert('L'), dtype=np.uint8)
            # nav2_map_server stores the image flipped vertically.
            img = np.flipud(img)

            loaded_h, loaded_w = img.shape

            # Convert pixel values to occupancy.
            # Pixel 0 (black) -> occupied, 254 (white) -> free, 205 (grey) -> unknown.
            # negate flag inverts the interpretation.
            if negate:
                occupancy_float = img.astype(np.float32) / 255.0
            else:
                occupancy_float = (255.0 - img.astype(np.float32)) / 255.0

            # Convert float occupancy -> integer occupancy class
            grid = np.full(loaded_h * loaded_w, -1, dtype=np.int8)
            occ_flat = occupancy_float.flatten()
            grid[occ_flat >= occ_thresh]  = 100
            grid[occ_flat <= free_thresh] = 0

            # Convert to log-odds
            lo = np.zeros(loaded_h * loaded_w, dtype=np.float32)
            lo[grid == 100] = L_MAX
            lo[grid == 0]   = L_MIN

            # Accept the loaded origin immediately — do not wait for odom.
            self.map_origin_x = float(origin[0])
            self.map_origin_y = float(origin[1])

            # If the loaded map dimensions match our configured size, copy
            # directly.  Otherwise centre the loaded data inside our grid.
            if loaded_w == self.map_width and loaded_h == self.map_height:
                with self.map_lock:
                    self.log_odds = lo
            else:
                self.get_logger().warn(
                    f'Loaded map size ({loaded_w}x{loaded_h}) differs from '
                    f'configured size ({self.map_width}x{self.map_height}). '
                    'Centering loaded data inside grid.'
                )
                col_off = (self.map_width  - loaded_w) // 2
                row_off = (self.map_height - loaded_h) // 2
                loaded_lo   = lo.reshape(loaded_h, loaded_w)
                full_lo     = np.zeros((self.map_height, self.map_width), dtype=np.float32)
                r0 = max(0, row_off)
                c0 = max(0, col_off)
                r1 = min(self.map_height, row_off + loaded_h)
                c1 = min(self.map_width,  col_off + loaded_w)
                lr0 = r0 - row_off
                lc0 = c0 - col_off
                full_lo[r0:r1, c0:c1] = loaded_lo[lr0:lr0 + (r1 - r0),
                                                    lc0:lc0 + (c1 - c0)]
                with self.map_lock:
                    self.log_odds = full_lo.flatten()

            self.origin_set = True
            self.get_logger().info(
                f'Loaded map from {yaml_path}  '
                f'origin=({self.map_origin_x:.3f}, {self.map_origin_y:.3f})  '
                f'size={loaded_w}x{loaded_h}'
            )

        except Exception as exc:
            self.get_logger().warn(
                f'Failed to load map from "{yaml_path}": {exc}. '
                'Falling back to origin-averaging startup mode.'
            )

    # ------------------------------------------------------------------ #
    # Save map — core logic used by all three save paths                  #
    # ------------------------------------------------------------------ #

    def _save_map_to_disk(self):
        """
        Write the current map to ~/map.pgm and ~/map.yaml.
        Returns (success, message) tuple.
        Overwrites existing files so ~/map always reflects the latest state.
        """
        if not self.origin_set:
            return False, 'Map origin not set yet, no data to save'

        save_path = os.path.expanduser('~/map')

        with self.map_lock:
            lo = self.log_odds.copy()

        # Convert log-odds to occupancy values for saving.
        grid = np.full(len(lo), -1, dtype=np.int8)
        grid[lo >= L_OCC_THRESH]  = 100
        grid[lo <= L_FREE_THRESH] = 0
        # Convert occupancy values to greyscale image.
        # free (0)        -> white (254)
        # occupied (100)  -> black (0)
        # unknown (-1)    -> grey  (205)
        arr = grid.reshape((self.map_height, self.map_width))
        img_arr = np.full((self.map_height, self.map_width), 205, dtype=np.uint8)
        img_arr[arr == 0] = 254
        img_arr[arr == 100] = 0

        # Flip vertically so the image y-axis matches ROS convention
        img_arr = np.flipud(img_arr)
        Image.fromarray(img_arr).save(save_path + '.pgm')

        # Write yaml metadata in nav2_map_server format
        yaml_data = {
            'image': save_path + '.pgm',
            'resolution': float(MAP_RES),
            'origin': [
                float(self.map_origin_x),
                float(self.map_origin_y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(save_path + '.yaml', 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

        msg = f'Map saved to {save_path}.pgm and {save_path}.yaml'
        return True, msg

    # ------------------------------------------------------------------ #
    # Auto-save timer callback                                             #
    # ------------------------------------------------------------------ #

    def _auto_save_callback(self):
        success, msg = self._save_map_to_disk()
        if success:
            self.get_logger().info(f'[Auto-save] {msg}')
        else:
            self.get_logger().warn(f'[Auto-save] {msg}')

    # ------------------------------------------------------------------ #
    # Manual save service callback                                         #
    # ------------------------------------------------------------------ #

    def _save_map_callback(self, request, response):
        success, msg = self._save_map_to_disk()
        if success:
            self.get_logger().info(f'[Manual save] {msg}')
        else:
            self.get_logger().warn(f'[Manual save] {msg}')
        response.success = success
        response.message = msg
        return response

    # ------------------------------------------------------------------ #
    # Final save on shutdown                                               #
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        self.get_logger().info('Node shutting down — performing final map save...')
        success, msg = self._save_map_to_disk()
        if success:
            self.get_logger().info(f'[Final save] {msg}')
        else:
            self.get_logger().warn(f'[Final save] {msg}')
        super().destroy_node()

    # ------------------------------------------------------------------ #
    # Geometry helpers                                                     #
    # ------------------------------------------------------------------ #

    def _in_bounds(self, x, y):
        return 0 <= x < self.map_width and 0 <= y < self.map_height

    def _bresenham_line(self, x0, y0, x1, y1):
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

    def _rotate_and_create_points(self, ranges, range_max, prefix):
        data = []
        o = self.positions[prefix]
        roll = self.angles[prefix][0]
        pitch = self.angles[prefix][1]
        yaw = self.angles[prefix][2]

        r_back = ranges[0]
        r_right = ranges[1]
        r_front = ranges[2]
        r_left = ranges[3]

        def valid(r):
            return r < range_max and r != 0.0 and not math.isinf(r)

        if valid(r_left):
            data.append(self._rot(roll, pitch, yaw, o, [o[0], o[1] + r_left, o[2]]))
        if valid(r_right):
            data.append(self._rot(roll, pitch, yaw, o, [o[0], o[1] - r_right, o[2]]))
        if valid(r_front):
            data.append(self._rot(roll, pitch, yaw, o, [o[0] + r_front, o[1], o[2]]))
        if valid(r_back):
            data.append(self._rot(roll, pitch, yaw, o, [o[0] - r_back, o[1], o[2]]))

        return data

    def _rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos(roll)
        cosp = math.cos(pitch)
        cosy = math.cos(yaw)
        sinr = math.sin(roll)
        sinp = math.sin(pitch)
        siny = math.sin(yaw)

        roty = np.array([[cosy, -siny, 0],
                         [siny,  cosy, 0],
                         [0,     0,    1]])
        rotp = np.array([[cosp,  0, sinp],
                         [0,     1, 0],
                         [-sinp, 0, cosp]])
        rotr = np.array([[1, 0,    0],
                         [0, cosr, -sinr],
                         [0, sinr,  cosr]])

        rot = np.dot(np.dot(rotr, rotp), roty)
        return np.add(np.dot(rot, np.subtract(point, origin)), origin)


def main(args=None):
    rclpy.init(args=args)
    time.sleep(3)
    node = SharedMapperMultiranger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
