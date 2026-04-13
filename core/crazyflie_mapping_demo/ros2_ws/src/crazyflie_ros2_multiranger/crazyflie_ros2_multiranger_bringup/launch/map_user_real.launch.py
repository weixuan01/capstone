import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# map_user_real.launch.py
#
# Per-drone launch file for map-user navigation in real mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - vel_mux           Translates cmd_vel into drone-safe velocity commands
#   - map_user          Navigates using the shared occupancy map
#   - map_server        (only if robot_prefix_map is provided) Loads a saved
#                       map.yaml and publishes it on /map
#   - lifecycle_manager (only if map_server is launched) Activates map_server
#
# NOT launched here (handled by shared_real.launch.py):
#   - crazyflie_server
#   - shared_mapper
#   - rviz
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
    p = robot_prefix.lstrip('/')

    nodes = []

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
    nodes.append(vel_mux)

    # ── Map user ──────────────────────────────────────────────────────────────
    # Navigates point-to-point using the occupancy map from shared_mapper.
    map_user_params = [
        {'robot_prefix':          robot_prefix},
        {'use_sim_time':          False},
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
        name=f'map_user_{p}',
        output='screen',
        parameters=map_user_params,
    )
    nodes.append(map_user)

    # ── Nav2 map server (only when a saved map file is provided) ──────────────
    # map_server loads the .yaml map and publishes it on /map.
    # lifecycle_manager brings map_server into its active state on startup.
    if map_file:
        map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name=f'map_server_{p}',
            output='screen',
            parameters=[
                {'yaml_filename': map_file},
                {'use_sim_time':  False},
            ],
        )
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=f'lifecycle_manager_map_{p}',
            output='screen',
            parameters=[
                {'autostart':  True},
                {'node_names': [f'map_server_{p}']},
            ],
        )
        nodes.append(map_server)
        nodes.append(lifecycle_manager)

    return nodes
