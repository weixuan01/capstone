#!/usr/bin/env python3

"""
Collision Avoidance Node for Crazyflie Multi-Drone System
==========================================================
Sits between the exploration node and the hardware control node,
intercepting velocity commands and overriding them when a peer drone
enters the safety radius.  Transparent during normal flight.

Topic chain:
  exploration_node  →  /{prefix}/cmd_vel_raw   (planner output)
  this node         →  /{prefix}/cmd_vel_safe   (safe output to control node)
  this node         →  /{prefix}/target_height  (altitude setpoint to control node)

Peer positions are shared via the global /peer_poses topic.
Each drone publishes its own position as a PointStamped where:
    point.x, point.y  = world x/y position
    point.z           = world z position (actual altitude)
    header.frame_id   = robot_prefix string (for self-filtering)

Resolution manoeuvre — altitude separation:
  When two drones come within PEER_SAFE_DIST of each other, they
  independently compare their x positions to decide who climbs and
  who descends.  Higher x climbs by ALTITUDE_STEP, lower x descends
  by ALTITUDE_STEP.  If x values are within X_TIE_TOLERANCE, y is
  used instead.  If y is also tied, lexicographic comparison of the
  robot_prefix string breaks the tie — deterministic and always
  produces opposite results for two different prefixes.
  Horizontal planner commands are passed through unchanged during
  resolution so each drone keeps navigating toward its goal.
  Once the peer clears PEER_SAFE_DIST, altitude returns to cruise.

  Altitude separation is commanded via /{prefix}/target_height
  (std_msgs/Float32).  control_services subscribes to this topic and
  overrides desired_height when a message arrives.  linear.z in
  cmd_vel_safe is always 0.0 so control_services height PID is not
  disrupted.

States:
  CLEAR      — no peer within PEER_SAFE_DIST; commands passed through unchanged
  RESOLUTION — peer within PEER_SAFE_DIST; altitude separation active
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

import math
from enum import Enum, auto

# ── Safety radii ──────────────────────────────────────────────────────────────
PEER_SAFE_DIST    = 0.45   # metres — resolution zone; altitude separation activates
PEER_POSE_TIMEOUT = 5.0   # seconds — ignore stale peer poses (crashed / landed)

# ── Altitude separation ───────────────────────────────────────────────────────
CRUISE_ALTITUDE   = 0.3   # metres — normal flight altitude (match hover_height)
ALTITUDE_STEP     = 0.15  # metres — each drone moves this far from cruise altitude
X_TIE_TOLERANCE   = 0.05  # metres — x positions closer than this use y as tiebreaker

# ── Position publish rate ─────────────────────────────────────────────────────
POSE_PUB_INTERVAL = 0.1   # seconds — publish own pose at 10 Hz

# ── Takeoff detection threshold ───────────────────────────────────────────────
# Must be ABOVE CRUISE_ALTITUDE so that control_services sees
# position.z > takeoff_height and sets is_flying=True before this node
# starts zeroing linear.z in _strip_z.
#
# If FLYING_THRESHOLD is below or equal to CRUISE_ALTITUDE (takeoff_height),
# the CA node zeros linear.z before control_services crosses its takeoff
# threshold, so is_flying never becomes True, the height PID never activates,
# and each new planner linear.z > 0 restarts the takeoff loop — causing
# continuous upward drift.
#
# By setting this above CRUISE_ALTITUDE, the drone is guaranteed to have
# already crossed takeoff_height (and completed the takeoff sequence in
# control_services) before _strip_z begins zeroing linear.z.
FLYING_THRESHOLD  = CRUISE_ALTITUDE + 0.05  # metres — must be > takeoff_height


class CAState(Enum):
    CLEAR      = auto()
    RESOLUTION = auto()


class CollisionAvoidanceNode(Node):

    def __init__(self):
        super().__init__('collision_avoidance_node')

        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        self._identity_str = robot_prefix

        # ── Internal state ────────────────────────────────────────────────────
        self.position      = [0.0, 0.0, 0.0]
        self.yaw           = 0.0
        self.ca_state      = CAState.CLEAR
        self.target_altitude = CRUISE_ALTITUDE
        self._is_flying    = False
        # Locked climb/descend role for the current RESOLUTION encounter.
        # Set once on entry to RESOLUTION, cleared on exit.
        # None means no lock held (CLEAR state).
        self._climb_locked: 'bool | None' = None

        # Maps identity_str (robot_prefix) → (x, y, z, timestamp)
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

        # Altitude setpoint — control_services subscribes to this and overrides
        # desired_height when a message arrives.
        self.height_pub = self.create_publisher(
            Float32, robot_prefix + '/target_height', 10)

        # ── Timer — pose broadcast ────────────────────────────────────────────
        self.create_timer(POSE_PUB_INTERVAL, self._timer_callback)

        self.get_logger().info(
            f'[CA] Started. prefix={robot_prefix} id={self._identity_str} '
            f'flying_threshold={FLYING_THRESHOLD:.2f}m')

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

        # FLYING_THRESHOLD is set above CRUISE_ALTITUDE so control_services
        # completes its takeoff sequence (is_flying=True) before this node
        # starts zeroing linear.z in _strip_z.
        self._is_flying = self.position[2] > FLYING_THRESHOLD

    def _cmd_callback(self, msg: Twist):
        self._process_and_publish(msg)

    def _peer_pose_callback(self, msg: PointStamped):
        if msg.header.frame_id == self._identity_str:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        key = msg.header.frame_id
        self.peer_poses[key] = (msg.point.x, msg.point.y, msg.point.z, now)

    def _timer_callback(self):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._identity_str
        msg.point.x = float(self.position[0])
        msg.point.y = float(self.position[1])
        msg.point.z = float(self.position[2])
        self.pose_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # Core CA logic
    # ══════════════════════════════════════════════════════════════════════════

    def _closest_peer(self):
        now    = self.get_clock().now().nanoseconds * 1e-9
        px, py = self.position[0], self.position[1]
        closest  = None
        min_dist = float('inf')

        for peer_id, (peer_x, peer_y, peer_z, t) in self.peer_poses.items():
            if (now - t) > PEER_POSE_TIMEOUT:
                continue
            dist = math.hypot(px - peer_x, py - peer_y)
            if dist < min_dist:
                min_dist = dist
                closest  = (dist, peer_x, peer_y, peer_z, peer_id)

        return closest

    def _should_climb(self, peer_x, peer_y, peer_id: str) -> bool:
        my_x, my_y = self.position[0], self.position[1]

        if abs(my_x - peer_x) > X_TIE_TOLERANCE:
            return my_x > peer_x

        if abs(my_y - peer_y) > X_TIE_TOLERANCE:
            return my_y > peer_y

        return self._identity_str > peer_id

    def _process_and_publish(self, planner_cmd: Twist):
        """Apply CA filter to planner_cmd and publish the result.

        cmd_vel_safe always has linear.z = 0.0 so control_services height PID
        is undisturbed.  Altitude separation is commanded separately via the
        target_height topic, which control_services uses to override
        desired_height.

        CLEAR      — pass x/y/yaw through unchanged. Publish cruise altitude.
        RESOLUTION — pass x/y/yaw through unchanged. Publish climb/descend
                     altitude so the drones separate vertically while continuing
                     to navigate toward their goals.  The climb/descend role is
                     evaluated once on entry and locked for the duration of the
                     encounter to prevent role thrashing as positions cross.
        """
        peer = self._closest_peer()

        if peer is None:
            self._climb_locked = None
            self._set_state(CAState.CLEAR)
            self._publish_target_height(CRUISE_ALTITUDE)
            self.cmd_pub.publish(self._strip_z(planner_cmd))
            return

        dist, peer_x, peer_y, peer_z, peer_id = peer

        if dist < PEER_SAFE_DIST:
            if self.ca_state != CAState.RESOLUTION:
                # Entering resolution — evaluate and lock role once.
                # Never re-evaluate while the peer stays within range.
                self._climb_locked = self._should_climb(peer_x, peer_y, peer_id)
                self._set_state(CAState.RESOLUTION)

            if self._climb_locked:
                self.target_altitude = CRUISE_ALTITUDE + ALTITUDE_STEP
            else:
                self.target_altitude = CRUISE_ALTITUDE - ALTITUDE_STEP
            self._publish_target_height(self.target_altitude)
            self.cmd_pub.publish(self._strip_z(planner_cmd))
            return

        # Peer exists but outside resolution range — clear lock, return to cruise
        self._climb_locked = None
        self._set_state(CAState.CLEAR)
        self.target_altitude = CRUISE_ALTITUDE
        self._publish_target_height(CRUISE_ALTITUDE)
        self.cmd_pub.publish(self._strip_z(planner_cmd))

    # ══════════════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _strip_z(self, cmd: Twist) -> Twist:
        """Return a copy of cmd with linear.z forced to 0.0 during flight.
        Before takeoff, linear.z is passed through unchanged so control_services
        can detect the takeoff signal and complete its takeoff sequence.

        _is_flying uses FLYING_THRESHOLD = CRUISE_ALTITUDE + 0.05, which is
        above takeoff_height. This guarantees control_services has already set
        is_flying=True and activated the height PID before this node starts
        zeroing linear.z. Setting FLYING_THRESHOLD below takeoff_height causes
        the CA node to cut the takeoff signal before control_services finishes,
        leaving is_flying=False permanently and causing upward drift."""
        out = Twist()
        out.linear.x  = cmd.linear.x
        out.linear.y  = cmd.linear.y
        out.linear.z  = 0.0 if self._is_flying else cmd.linear.z
        out.angular.z = cmd.angular.z
        return out

    def _publish_target_height(self, height: float):
        msg = Float32()
        msg.data = float(height)
        self.height_pub.publish(msg)

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
    rclpy.shutdown()


if __name__ == '__main__':
    main()