import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_project_crazyflie_gazebo,
                'launch',
                'crazyflie_simulation.launch.py',
            )
        )
    )

    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True},
            {'map_size_x': 40.0},
            {'map_size_y': 40.0},
            {'map_resolution': 0.1},
            {'min_mapping_height': 0.15},
            {'mapping_start_delay': 3.0},
            {'require_fresh_odom': True},
            {'recenter_initial_yaw': False},
        ],
    )

    wall_following = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name='wall_following',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True},
            {'delay': 5.0},
            {'max_turn_rate': 0.7},
            {'max_forward_speed': 0.5},
            {'wall_following_direction': 'right'},
            {'takeoff_mode': 'trigger'},
            {'takeoff_ready_time': 1.0},
            {'takeoff_speed': 0.5},
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'sim_mapping.rviz',
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        crazyflie_simulation,
        simple_mapper,
        wall_following,
        rviz,
    ])