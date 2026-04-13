import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# =============================================================================
# shared_real.launch.py
#
# Launched once per session by launch-universal.sh (real mode).
# Starts the nodes that are shared across all drones:
#   - crazyflie_server  (Crazyswarm2 — connects to all physical drones)
#   - shared_mapper     (one unified occupancy map for all drones)
#   - rviz              (visualiser, using the real-mode config)
#
# This file is NOT responsible for any per-drone behaviour nodes.
# Those are launched separately by the per-drone launch files.
#
# Launch arguments:
#   robot_prefixes   Comma-separated list of all drone namespaces, e.g. [/cf1,/cf2]
# =============================================================================


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_prefixes',
            default_value='[/crazyflie]',
            description='Comma-separated list of all drone namespaces, e.g. [/cf1,/cf2]'
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    prefixes_str  = context.launch_configurations['robot_prefixes']
    prefixes_list = [p.strip() for p in prefixes_str.strip('[]').split(',') if p.strip()]

    pkg_crazyswarm2 = get_package_share_directory('crazyflie')
    pkg_bringup     = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    crazyflies_yaml = os.path.join(pkg_bringup, 'config', 'crazyflie_real_crazyswarm2.yaml')

    # ── Crazyswarm2 server ────────────────────────────────────────────────────
    # One instance connects to all physical drones listed in crazyflies_yaml.
    crazyflie_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyswarm2, 'launch', 'launch.py')
        ),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml,
            'backend': 'cflib',
            'mocap':   'False',
            'rviz':    'False',
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
            {'use_sim_time':   False},
        ],
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'real_mapping.rviz')],
        parameters=[{'use_sim_time': False}],
    )

    return [crazyflie_server, shared_mapper, rviz]
