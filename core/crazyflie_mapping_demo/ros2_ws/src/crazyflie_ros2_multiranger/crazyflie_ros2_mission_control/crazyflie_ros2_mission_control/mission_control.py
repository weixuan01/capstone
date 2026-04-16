#!/usr/bin/env python3
"""
battery_monitor.py

The single ROS 2 node that bridges the mission control server and the drones.
Handles two responsibilities:

  1. Battery monitoring
     Subscribes to /<drone>/status for each active drone, aggregates voltage
     data, and publishes a JSON summary to /battery_status at a fixed rate.
     The mission control server reads this topic and streams it to the browser.

  2. Land commands
     Subscribes to /land_command (std_msgs/String), expects a JSON payload:
       {"prefixes": ["cf1", "cf2"]}
     For each prefix, publishes a Hover message with z_distance=0.0 to
     /<prefix>/cmd_hover, which commands the drone to descend and land.

Published topics
----------------
/battery_status  (std_msgs/String)
    JSON string, e.g.:
    {
        "cf1": {"voltage": 3.95, "state": "ok"},
        "cf2": {"voltage": 3.71, "state": "warning"}
    }
    state values: "ok" | "warning" | "critical" | "unknown"

Subscribed topics
-----------------
/<drone>/status   (crazyflie_interfaces/msg/Status)  — one per drone
/land_command     (std_msgs/String)                  — from mission control server

Parameters
----------
drone_names  (list[str])  Names of active drones, e.g. ['cf1', 'cf2']
                          Passed in by shared_real.launch.py — do not hardcode.
publish_rate (float)      Battery summary publish rate in Hz. Default: 1.0
"""

import json

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import Status, Hover
from std_msgs.msg import String
from geometry_msgs.msg import Point


VOLTAGE_WARNING  = 3.8  # V — matches crazyflies.yaml default
VOLTAGE_CRITICAL = 3.7  # V — matches crazyflies.yaml default


def voltage_state(voltage: float) -> str:
    if voltage <= 0.0:
        return 'unknown'
    if voltage < VOLTAGE_CRITICAL:
        return 'critical'
    if voltage < VOLTAGE_WARNING:
        return 'warning'
    return 'ok'


class DroneManager(Node):

    def __init__(self):
        super().__init__('drone_manager')

        self.declare_parameter('drone_names',  ['cf1'])
        self.declare_parameter('publish_rate', 1.0)

        drone_names  = self.get_parameter('drone_names').value
        publish_rate = self.get_parameter('publish_rate').value

        # ── Battery state ─────────────────────────────────────────────────────
        self.battery_data: dict[str, dict] = {}

        for name in drone_names:
            self.battery_data[name] = {
                'voltage':   0.0,
                'state':     'unknown',
                'last_seen': None,
            }
            self.create_subscription(
                Status,
                f'/{name}/status',
                self._make_battery_callback(name),
                1,
            )
            self.get_logger().info(f'Subscribed to /{name}/status')

        self.battery_pub = self.create_publisher(String, '/battery_status', 10)
        self.create_timer(1.0 / publish_rate, self._publish_battery_summary)

        # ── Land command ──────────────────────────────────────────────────────
        # Publishers created on demand per drone prefix
        self._hover_pubs: dict[str, object] = {}

        # ── Assigned goal publishers — used for recall and land-in-place ──────
        # Publishes NaN goals to /cfX/assigned_goal so the drone_navigator
        # handles the command directly via its existing _goal_cb logic.
        self._assigned_goal_pubs: dict[str, object] = {}
        for name in drone_names:
            self._assigned_goal_pubs[name] = self.create_publisher(
                Point, f'/{name}/assigned_goal', 10)

        self.create_subscription(
            String,
            '/recall_command',
            self._recall_callback,
            10,
        )
        self.get_logger().info('Subscribed to /recall_command')

        self.create_subscription(
            String,
            '/land_command',
            self._land_callback,
            10,
        )
        self.get_logger().info('Subscribed to /land_command')

    # ── Battery ───────────────────────────────────────────────────────────────

    def _make_battery_callback(self, drone_name: str):
        def callback(msg: Status):
            voltage = msg.battery_voltage
            state   = voltage_state(voltage)
            self.battery_data[drone_name] = {
                'voltage':   round(voltage, 3),
                'state':     state,
                'last_seen': self.get_clock().now().nanoseconds,
            }
            if state == 'critical':
                self.get_logger().error(
                    f'[{drone_name}] CRITICAL battery: {voltage:.2f} V'
                )
            elif state == 'warning':
                self.get_logger().warn(
                    f'[{drone_name}] Low battery: {voltage:.2f} V'
                )
        return callback

    def _publish_battery_summary(self):
        payload = {
            name: {'voltage': data['voltage'], 'state': data['state']}
            for name, data in self.battery_data.items()
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.battery_pub.publish(msg)

    # ── Land ──────────────────────────────────────────────────────────────────

    def _get_hover_pub(self, prefix: str):
        if prefix not in self._hover_pubs:
            self._hover_pubs[prefix] = self.create_publisher(
                Hover, f'/{prefix}/cmd_hover', 10
            )
        return self._hover_pubs[prefix]

    def _recall_callback(self, msg: String):
        """Return home then land. Publishes NaN goal with z=0.0 to each
        specified drone. The drone_navigator transitions to DONE and navigates
        back to its recorded start position before landing."""
        try:
            data     = json.loads(msg.data)
            prefixes = data.get('prefixes', [])
        except Exception as e:
            self.get_logger().error(f'[recall] bad message: {e}')
            return

        for prefix in prefixes:
            try:
                goal   = Point()
                goal.x = float('nan')
                goal.y = float('nan')
                goal.z = 0.0   # z=0.0 → recall (return home then land)
                self._assigned_goal_pubs[prefix].publish(goal)
                self.get_logger().info(f'[recall] sent to {prefix}')
            except Exception as e:
                self.get_logger().error(f'[recall] failed for {prefix}: {e}')

    def _land_callback(self, msg: String):
        """Land in place immediately. Publishes NaN goal with z=1.0 to each
        specified drone. The drone_navigator transitions directly to LANDING
        and descends without returning home first."""
        try:
            data     = json.loads(msg.data)
            prefixes = data.get('prefixes', [])
        except Exception as e:
            self.get_logger().error(f'[land] bad message: {e}')
            return

        for prefix in prefixes:
            try:
                goal   = Point()
                goal.x = float('nan')
                goal.y = float('nan')
                goal.z = 1.0   # z=1.0 → land in place
                self._assigned_goal_pubs[prefix].publish(goal)
                self.get_logger().info(f'[land] sent to {prefix}')
            except Exception as e:
                self.get_logger().error(f'[land] failed for {prefix}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DroneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()