import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# square_sim.launch.py
#
# Per-drone launch file for square pattern flight in sim mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - square    Flies a square pattern
#
# NOT launched here (handled by shared_sim.launch.py):
#   - Gazebo simulator
#   - shared_mapper
#   - rviz
#
# Note: vel_mux is not needed in sim mode — Gazebo receives cmd_vel directly
# via the ros_gz bridge.
#
# Launch arguments:
#   robot_prefix   ROS 2 namespace for this drone, e.g. /cf1
# =============================================================================


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_prefix',
            default_value='/crazyflie',
            description='ROS 2 namespace for this drone, e.g. /cf1'
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    robot_prefix = context.launch_configurations['robot_prefix']

    # ── Square pattern ────────────────────────────────────────────────────────
    # Commands the drone to fly a square pattern of a fixed size.
    square = Node(
        package='crazyflie_ros2_multiranger_square',
        executable='square_multiranger',
        name=f'square_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=[
            {'robot_prefix':      robot_prefix},
            {'use_sim_time':      True},
            {'delay':             5.0},
            {'max_turn_rate':     0.7},
            {'max_forward_speed': 0.5},
            {'square_size':       1.5},
        ],
    )

    return [square]
