import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# =============================================================================
# frontier_exploration_swarm_sim.launch.py
#
# Per-drone launch file for swarm frontier exploration in sim mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - control_services  Sim equivalent of vel_mux for Gazebo
#   - drone_navigator   Navigates to goals assigned by the centralised goal_assigner
#   - collision_avoidance
#
# NOT launched here (handled by shared_sim.launch.py):
#   - Gazebo simulator
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

    # ── Control services ──────────────────────────────────────────────────────
    # Sim equivalent of vel_mux. Translates cmd_vel_safe into Gazebo velocity
    # commands and manages hover height.
    control_services = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        name=f"{robot_prefix.strip('/')}_control_services",
        output='screen',
        parameters=[
            {'hover_height':         0.3},
            {'robot_prefix':         robot_prefix},
            {'incoming_twist_topic': '/cmd_vel_safe'},
            {'max_ang_z_rate':       0.4},
            {'use_sim_time':         True},
        ],
    )

    # ── Drone navigator ───────────────────────────────────────────────────────
    # Receives goal assignments from the centralised goal_assigner and navigates
    # to them via A*. Reports REACHED or FAILED back to the assigner.
    explorer_drone = Node(
        package='crazyflie_ros2_multiranger_explorer_drone',
        executable='explorer_drone',
        name=f"{robot_prefix.strip('/')}_explorer_drone",
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
            {'use_sim_time': True},
        ],
    )

    # ── Collision avoidance ───────────────────────────────────────────────────
    # Sits between the navigator and control_services. Passes cmd_vel_raw
    # through unchanged during normal flight. Overrides with a resolution
    # manoeuvre when a peer drone enters the safety radius.
    collision_avoidance = Node(
        package='crazyflie_ros2_collision_avoidance',
        executable='collision_avoidance',
        name=f'collision_avoidance_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
        ],
    )

    return [control_services, explorer_drone, collision_avoidance]
