import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# frontier_exploration_swarm_real.launch.py
#
# Per-drone launch file for swarm frontier exploration in real mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - vel_mux           Translates cmd_vel into drone-safe velocity commands
#   - drone_navigator   Navigates to goals assigned by the centralised goal_assigner
#   - collision_avoidance
#
# NOT launched here (handled by shared_real.launch.py):
#   - crazyflie_server
#   - shared_mapper
#   - goal_assigner
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

    # ── Velocity multiplexer ──────────────────────────────────────────────────
    # Translates incoming cmd_vel commands into drone-safe velocity setpoints.
    vel_mux = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name=f'vel_mux_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=[
            {'hover_height':         0.3},
            {'incoming_twist_topic': '/cmd_vel_safe'},
            {'robot_prefix':         robot_prefix},
        ],
    )

    # ── Drone navigator ───────────────────────────────────────────────────────
    # Receives goal assignments from the centralised goal_assigner and navigates
    # to them via A*. Reports REACHED or FAILED back to the assigner.
    explorer_drone = Node(
        package='crazyflie_ros2_multiranger_explorer_drone',
        executable='explorer_drone',
        name=f'{robot_prefix.lstrip("/")}_explorer_drone',
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
            {'use_sim_time': False},
        ],
    )

    # ── Collision avoidance ───────────────────────────────────────────────────
    # Sits between the navigator and vel_mux. Passes cmd_vel_raw through
    # unchanged during normal flight. Overrides with a resolution manoeuvre
    # when a peer drone enters the safety radius, then hands back control.
    collision_avoidance = Node(
        package='crazyflie_ros2_collision_avoidance',
        executable='collision_avoidance',
        name=f'collision_avoidance_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
        ],
    )

    return [vel_mux, explorer_drone, collision_avoidance]
