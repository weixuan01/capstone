import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# ai_real.launch.py
#
# Per-drone launch file for the object-detection scanner in real mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - vel_mux                    Translates cmd_vel into drone-safe velocity commands
#   - object_detection_scanner   Navigates to scan points assigned by the
#                                centralised object_detection_planner and
#                                performs a 360-degree spin at each one
#   - collision_avoidance        Peer-repulsion layer between navigator and vel_mux
#
# NOT launched here (handled by shared_real.launch.py):
#   - crazyflie_server
#   - shared_mapper
#   - object_detection_planner
#   - rviz
#
# Launch arguments:
#   robot_prefix   ROS 2 namespace for this drone, e.g. /cf3
# =============================================================================


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_prefix',
            default_value='/crazyflie',
            description='ROS 2 namespace for this drone, e.g. /cf3'
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

    # ── Object-detection scanner ──────────────────────────────────────────────
    # Receives scan-point assignments from the centralised
    # object_detection_planner and navigates to them via A*.  On arrival,
    # performs a full 360-degree spin for object detection, then reports
    # REACHED so the planner marks the disc as scanned.
    object_detection_scanner = Node(
        package='crazyflie_ros2_object_detection_scanner',
        executable='object_detection_scanner',
        name=f'{robot_prefix.lstrip("/")}_object_detection_scanner',
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
            {'use_sim_time': False},
        ],
    )

    # ── Collision avoidance ───────────────────────────────────────────────────
    # Sits between the navigator and vel_mux.  Passes cmd_vel_raw through
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

    return [vel_mux, object_detection_scanner, collision_avoidance]
