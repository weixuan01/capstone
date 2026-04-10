import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_crazyswarm2 = get_package_share_directory('crazyflie')
    pkg_multiranger_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    crazyflies_yaml = os.path.join(
        pkg_multiranger_bringup,
        'config',
        'crazyflie_real_crazyswarm2.yaml',
    )

    crazyflie_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_project_crazyswarm2, 'launch'),
            '/launch.py',
        ]),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml,
            'backend': 'cflib',
            'mocap': 'False',
            'rviz': 'False',
        }.items(),
    )

    crazyflie_vel_mux = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name='vel_mux',
        output='screen',
        parameters=[
            {'hover_height': 0.4},
            {'incoming_twist_topic': '/cmd_vel'},
            {'robot_prefix': 'crazyflie_real'},
        ],
    )

    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie_real'},
            {'use_sim_time': False},
            {'map_size_x': 40.0},
            {'map_size_y': 40.0},
            {'map_resolution': 0.1},
            {'min_mapping_height': 0.15},
            {'mapping_start_delay': 3.0},
            {'require_fresh_odom': True},
            {'recenter_initial_yaw': False},
        ],
    )

    frontier_exploration = Node(
        package='crazyflie_ros2_multiranger_frontier_exploration',
        executable='frontier_exploration_multiranger',
        name='frontier_exploration',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie_real'},
            {'use_sim_time': False},
            {'delay': 5.0},
            {'invert_yaw_command': True},
            {'takeoff_mode': 'trigger'},
            {'takeoff_ready_time': 1.0},
            {'takeoff_height': 0.30},
            {'takeoff_speed': 0.35},
            {'takeoff_timeout': 4.0},
        ],
    )

    aideck_udp_streamer = Node(
        package='crazyflie',
        executable='aideck_udp_streamer.py',
        name='aideck_udp_streamer',
        output='screen',
        parameters=[
            {'deck_ip': '192.168.4.1'},
            {'deck_port': 5000},
            {'listen_ip': '0.0.0.0'},
            {'listen_port': 5001},
            {'image_topic': '/aideck/image_raw'},
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'real_mapping.rviz',
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        crazyflie_real,
        crazyflie_vel_mux,
        simple_mapper,
        frontier_exploration,
        aideck_udp_streamer,
        rviz,
    ])