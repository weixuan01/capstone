import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# map_user_sim.launch.py
#
# Per-drone launch file for map-user navigation in sim mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - map_user   Navigates using the shared occupancy map
#
# NOT launched here (handled by shared_sim.launch.py):
#   - Gazebo simulator
#   - shared_mapper
#   - rviz
#
# Note: vel_mux is not needed in sim mode — Gazebo receives cmd_vel directly
# via the ros_gz bridge. nav2 map_server is real-mode only.
#
# Launch arguments:
#   robot_prefix   ROS 2 namespace for this drone, e.g. /cf1
#   map_file       (optional) Absolute path to a saved map.yaml.
#                  Leave empty to start with a blank map.
# =============================================================================


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_prefix',
            default_value='/crazyflie',
            description='ROS 2 namespace for this drone, e.g. /cf1'
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Absolute path to a saved map.yaml. Leave empty for a blank map.'
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    robot_prefix = context.launch_configurations['robot_prefix']
    map_file     = context.launch_configurations['map_file']

    # ── Map user ──────────────────────────────────────────────────────────────
    # Navigates point-to-point using the occupancy map from shared_mapper.
    map_user_params = [
        {'robot_prefix':          robot_prefix},
        {'use_sim_time':          True},
        {'delay':                 0.0},
        {'max_turn_rate':         0.7},
        {'max_forward_speed':     0.5},
        {'target_altitude':       0.5},
        {'alt_kp':                1.2},
        {'max_vz':                0.4},
        {'max_obstacle_distance': 0.3},
    ]
    if map_file:
        map_user_params.append({'map_file': map_file})

    map_user = Node(
        package='crazyflie_ros2_multiranger_map_user',
        executable='map_user_multiranger',
        name=f'map_user_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=map_user_params,
    )

    return [map_user]
