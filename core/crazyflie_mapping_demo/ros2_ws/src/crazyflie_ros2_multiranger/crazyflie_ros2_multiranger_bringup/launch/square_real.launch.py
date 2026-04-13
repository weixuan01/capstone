import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# square_real.launch.py
#
# Per-drone launch file for square pattern flight in real mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - vel_mux   Translates cmd_vel into drone-safe velocity commands
#   - square    Flies a square pattern
#
# NOT launched here (handled by shared_real.launch.py):
#   - crazyflie_server
#   - shared_mapper
#   - rviz
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
    p = robot_prefix.lstrip('/')

    # ── Velocity multiplexer ──────────────────────────────────────────────────
    # Translates incoming cmd_vel commands into drone-safe velocity setpoints.
    vel_mux = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name=f'vel_mux_{p}',
        output='screen',
        parameters=[
            {'hover_height':         0.3},
            {'incoming_twist_topic': '/cmd_vel'},
            {'robot_prefix':         robot_prefix},
        ],
    )

    # ── Square pattern ────────────────────────────────────────────────────────
    # Commands the drone to fly a square pattern of a fixed size.
    square = Node(
        package='crazyflie_ros2_multiranger_square',
        executable='square_multiranger',
        name=f'square_{p}',
        output='screen',
        parameters=[
            {'robot_prefix':      robot_prefix},
            {'use_sim_time':      False},
            {'delay':             5.0},
            {'max_turn_rate':     0.5},
            {'max_forward_speed': 0.3},
            {'square_size':       0.5},
        ],
    )

    return [vel_mux, square]
