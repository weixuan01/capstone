import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Absolute path to a previously saved map.yaml. '
                    'When set the shared mapper loads this map on startup '
                    'instead of waiting for odom samples.'
    )
    map_file = LaunchConfiguration('map_file')

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_crazyswarm2 = get_package_share_directory('crazyflie')
    pkg_bringup     = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    crazyflies_yaml = os.path.join(pkg_bringup, 'config', 'crazyflie_real_crazyswarm2.yaml')

    # ── Crazyswarm2 server ────────────────────────────────────────────────────
    crazyflie_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyswarm2, 'launch', 'launch.py')),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml,
            'backend': 'cflib',
            'mocap': 'False',
            'rviz': 'False'
        }.items()
    )

    # ── Velocity multiplexer ──────────────────────────────────────────────────
    crazyflie_vel_mux = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name='vel_mux',
        output='screen',
        parameters=[
            {'hover_height': 0.3},
            {'incoming_twist_topic': '/cmd_vel'},
            {'robot_prefix': 'crazyflie_real'},
        ]
    )

    # ── Shared mapper ─────────────────────────────────────────────────────────
    shared_mapper = Node(
        package='crazyflie_ros2_multiranger_shared_mapper',
        executable='shared_mapper_multiranger',
        name='shared_mapper',
        output='screen',
        parameters=[
            {'robot_prefixes': ['crazyflie_real']},
            {'map_file': map_file},
            {'use_sim_time': False},
        ]
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'config', 'real_mapping.rviz')],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        map_file_arg,
        crazyflie_server,
        crazyflie_vel_mux,
        shared_mapper,
        rviz,
    ])
