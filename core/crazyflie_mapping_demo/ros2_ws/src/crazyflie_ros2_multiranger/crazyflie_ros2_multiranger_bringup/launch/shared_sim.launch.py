import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# =============================================================================
# shared_sim.launch.py
#
# Launched once per session by launch-universal.sh (sim mode).
# Starts the nodes that are shared across all drones:
#   - Gazebo simulator  (spawns the world and all drone models)
#   - shared_mapper     (one unified occupancy map for all drones)
#   - rviz              (visualiser, using the sim-mode config)
#
# This file is NOT responsible for any per-drone behaviour nodes.
# Those are launched separately by the per-drone launch files.
#
# Launch arguments:
#   robot_prefixes   Comma-separated list of all drone namespaces, e.g. [/cf1,/cf2]
#   world            Gazebo world name, e.g. maze | crazyflie_world | circle-maze
# =============================================================================


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_prefixes',
            default_value='[/crazyflie]',
            description='Comma-separated list of all drone namespaces, e.g. [/cf1,/cf2]'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='maze',
            description='Gazebo world name, e.g. maze, crazyflie_world, circle-maze'
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    prefixes_str  = context.launch_configurations['robot_prefixes']
    world         = context.launch_configurations['world']
    prefixes_list = [p.strip() for p in prefixes_str.strip('[]').split(',') if p.strip()]

    pkg_gz      = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')

    # ── Gazebo simulator ──────────────────────────────────────────────────────
    # Launches Gazebo, loads the chosen world, and spawns all drone models.
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'crazyflie_simulation.launch.py')
        ),
        launch_arguments={
            'robot_prefixes': prefixes_str,
            'world':          world,
        }.items(),
    )

    # ── Shared mapper ─────────────────────────────────────────────────────────
    # One instance subscribes to all drone scan topics and maintains a single
    # unified occupancy map published on /map.
    shared_mapper = Node(
        package='crazyflie_ros2_multiranger_shared_mapper',
        executable='shared_mapper_multiranger',
        name='shared_mapper',
        output='screen',
        parameters=[
            {'robot_prefixes': prefixes_list},
            {'use_sim_time':   True},
        ],
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'sim_mapping.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return [simulator, shared_mapper, rviz]
