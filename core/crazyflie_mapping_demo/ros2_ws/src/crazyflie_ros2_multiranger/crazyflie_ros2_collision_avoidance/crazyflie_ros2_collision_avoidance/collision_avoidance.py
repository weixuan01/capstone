#!/usr/bin/env python3

"""
Collision Avoidance Node for Crazyflie Multi-Drone System
==========================================================
Sits between the exploration node and the hardware control node,
intercepting velocity commands and overriding them when a peer drone
enters the safety radius.  Transparent during normal flight.

Topic chain:
  exploration_node  →  /{prefix}/cmd_vel_raw   (planner output)
  this node         →  /{prefix}/cmd_vel        (safe output to control node)

Peer positions are shared via the global /peer_poses topic.
Each drone publishes its own position as a PointStamped where:
    point.x, point.y  = world x/y position
    point.z           = world z position (actual altitude)
    header.frame_id   = identity hash as string (for self-filtering)

Resolution manoeuvre — altitude separation:
  When two drones come within PEER_SAFE_DIST of each other, they
  independently compare their x positions to decide who climbs and
  who descends.  Higher x climbs by ALTITUDE_STEP, lower x descends
  by ALTITUDE_STEP.  If x values are within X_TIE_TOLERANCE, y is
  used instead.  If y is also tied, identity hash breaks the tie.
  Horizontal planner commands are passed through unchanged during
  resolution so each drone keeps navigating toward its goal.
  Once the peer clears PEER_SAFE_DIST, altitude returns to cruise.

States:
  CLEAR      — no peer within PEER_SAFE_DIST; commands passed through unchanged
  RESOLUTION — peer within PEER_SAFE_DIST; altitude separation active
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry

import math
from enum import Enum, auto

# ── Safety radii ──────────────────────────────────────────────────────────────
PEER_SAFE_DIST    = 0.3   # metres — resolution zone; altitude separation activates
PEER_POSE_TIMEOUT = 5.0   # seconds — ignore stale peer poses (crashed / landed)

# ── Altitude separation ───────────────────────────────────────────────────────
CRUISE_ALTITUDE   = 0.3   # metres — normal flight altitude (match TAKEOFF_HEIGHT)
ALTITUDE_STEP     = 0.15  # metres — each drone moves this far from cruise altitude
X_TIE_TOLERANCE   = 0.05  # metres — x positions closer than this use y as tiebreaker

# ── Position publish rate ─────────────────────────────────────────────────────
POSE_PUB_INTERVAL = 0.1   # seconds — publish own pose at 10 Hz

# ── Velocity limits (match exploration node) ─────────────────────────────────
MAX_SPEED         = 0.3


class CAState(Enum):
    CLEAR      = auto()
    RESOLUTION = auto()


class CollisionAvoidanceNode(Node):

    def __init__(self):
        super().__init__('collision_avoidance_node')

        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        # ── Identity ──────────────────────────────────────────────────────────
        # Stored both as float (for arithmetic comparison) and string (for
        # embedding in header.frame_id so peers can filter our messages).
        self._identity     = float(hash(robot_prefix) % 1_000_000)
        self._identity_str = str(int(self._identity))

        # ── Internal state ────────────────────────────────────────────────────
        self.position      = [0.0, 0.0, 0.0]
        self.yaw           = 0.0
        self.ca_state      = CAState.CLEAR

        # Target altitude — normally CRUISE_ALTITUDE, adjusted during resolution.
        self.target_altitude = CRUISE_ALTITUDE

        # Maps identity_str → (x, y, z, timestamp)
        self.peer_poses: dict = {}

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Odometry,
            robot_prefix + '/odom',
            self._odom_callback,
            10)

        self.create_subscription(
            Twist,
            robot_prefix + '/cmd_vel_raw',
            self._cmd_callback,
            10)

        self.create_subscription(
            PointStamped,
            '/peer_poses',
            self._peer_pose_callback,
            10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist, robot_prefix + '/cmd_vel_safe', 10)

        self.pose_pub = self.create_publisher(
            PointStamped, '/peer_poses', 10)

        # ── Timer — pose broadcast ────────────────────────────────────────────
        self.create_timer(POSE_PUB_INTERVAL, self._timer_callback)

        self.get_logger().info(
            f'[CA] Started. prefix={robot_prefix} id={self._identity_str}')

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def _odom_callback(self, msg: Odometry):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def _cmd_callback(self, msg: Twist):
        """Receive a velocity command from the exploration node and filter it."""
        self._process_and_publish(msg)

    def _peer_pose_callback(self, msg: PointStamped):
        """Receive a peer drone's position off /peer_poses.
        Identity hash is stored as a string in header.frame_id."""
        if msg.header.frame_id == self._identity_str:
            return  # own message — discard
        now = self.get_clock().now().nanoseconds * 1e-9
        key = msg.header.frame_id
        self.peer_poses[key] = (msg.point.x, msg.point.y, msg.point.z, now)

    def _timer_callback(self):
        """Publish own position to /peer_poses at POSE_PUB_INTERVAL."""
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._identity_str   # identity in frame_id
        msg.point.x = float(self.position[0])
        msg.point.y = float(self.position[1])
        msg.point.z = float(self.position[2])      # actual altitude in z
        self.pose_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # Core CA logic
    # ══════════════════════════════════════════════════════════════════════════

    def _closest_peer(self):
        """Return (dist, peer_x, peer_y, peer_z) for the closest active peer,
        or None if no active peers exist."""
        now    = self.get_clock().now().nanoseconds * 1e-9
        px, py = self.position[0], self.position[1]
        closest  = None
        min_dist = float('inf')

        for peer_x, peer_y, peer_z, t in self.peer_poses.values():
            if (now - t) > PEER_POSE_TIMEOUT:
                continue
            dist = math.hypot(px - peer_x, py - peer_y)
            if dist < min_dist:
                min_dist = dist
                closest  = (dist, peer_x, peer_y, peer_z)

        return closest

    def _should_climb(self, peer_x, peer_y):
        """Return True if this drone should climb, False if it should descend.

        Comparison rules (in order):
          1. Higher x climbs.
          2. If x values within X_TIE_TOLERANCE, higher y climbs.
          3. If y also tied, higher identity hash climbs.

        Both drones run this independently and always reach opposite conclusions
        because each compares its own value against the peer's value."""
        my_x, my_y = self.position[0], self.position[1]

        if abs(my_x - peer_x) > X_TIE_TOLERANCE:
            return my_x > peer_x

        if abs(my_y - peer_y) > X_TIE_TOLERANCE:
            return my_y > peer_y

        # Tiebreaker — identity hash, always deterministic and opposite for
        # the two drones since they have different prefixes.
        return self._identity > float(
            list(self.peer_poses.keys())[0]) if self.peer_poses else True

    def _repulsion_body_frame(self, dx_world, dy_world, magnitude):
        pass  # retained for future use

    def _process_and_publish(self, planner_cmd: Twist):
        """Apply CA filter to planner_cmd and publish the result.

        CLEAR      — pass through unchanged. Target altitude is cruise.
        RESOLUTION — peer within PEER_SAFE_DIST. Set target altitude based on
                     x/y position comparison. Horizontal planner commands passed
                     through so the drone keeps navigating while separating
                     vertically. Once the peer clears, altitude returns to cruise.
        """
        peer = self._closest_peer()

        if peer is None:
            self._set_state(CAState.CLEAR)
            self.target_altitude = CRUISE_ALTITUDE
            self.cmd_pub.publish(self._with_altitude(planner_cmd, self.target_altitude))
            return

        dist, peer_x, peer_y, peer_z = peer

        # ── RESOLUTION ────────────────────────────────────────────────────────
        if dist < PEER_SAFE_DIST:
            self._set_state(CAState.RESOLUTION)
            if self._should_climb(peer_x, peer_y):
                self.target_altitude = CRUISE_ALTITUDE + ALTITUDE_STEP
            else:
                self.target_altitude = CRUISE_ALTITUDE - ALTITUDE_STEP
            self.cmd_pub.publish(self._with_altitude(planner_cmd, self.target_altitude))
            return

        # ── CLEAR — peer exists but is outside resolution range ───────────────
        self._set_state(CAState.CLEAR)
        self.target_altitude = CRUISE_ALTITUDE
        self.cmd_pub.publish(self._with_altitude(planner_cmd, self.target_altitude))

    # ══════════════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _with_altitude(self, cmd: Twist, altitude: float) -> Twist:
        """Return a copy of cmd with linear.z set to the target altitude.
        The vel_mux interprets linear.z as a hover height setpoint."""
        out = Twist()
        out.linear.x  = cmd.linear.x
        out.linear.y  = cmd.linear.y
        out.linear.z  = float(altitude)
        out.angular.z = cmd.angular.z
        return out

    def _set_state(self, new_state: CAState):
        if new_state != self.ca_state:
            self.get_logger().info(
                f'[CA] {self.ca_state.name} → {new_state.name} '
                f'pos=({self.position[0]:.2f},{self.position[1]:.2f}) '
                f'target_alt={self.target_altitude:.2f}m')
            self.ca_state = new_state


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()