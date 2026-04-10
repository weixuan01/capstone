import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    # Pass map_file:=/path/to/map.yaml to pre-load a saved map into the shared
    # mapper.  Leave empty to start with a blank map (origin-averaging mode).
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Absolute path to a previously saved map.yaml. '
                    'When set the shared mapper loads this map on startup '
                    'instead of waiting for odom samples.'
    )
    map_file = LaunchConfiguration('map_file')

    # Configure ROS nodes for launch

    # Setup project paths'''
    pkg_project_crazyswarm2 = get_package_share_directory('crazyflie')
    pkg_multiranger_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    crazyflies_yaml = os.path.join(
        pkg_multiranger_bringup,
        'config',
        'crazyflie_real_crazyswarm2.yaml')

    # Start up a crazyflie server through the Crazyswarm2 project
    crazyflie_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_project_crazyswarm2, 'launch'), '/launch.py']),
        launch_arguments={'crazyflies_yaml_file': crazyflies_yaml, 'backend': 'cflib', 'mocap': 'False', 'rviz': 'False'}.items()
    )

    # Start a velocity multiplexer node for the crazyflie
    crazyflie_vel_mux = Node(
            package='crazyflie',
            executable='vel_mux.py',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.3},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': '/crazyflie_user_real'},]    # Unique identifier
        )

    #=======================================================================
    # if launching with an already made map, use these 2 nodes
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': '/home/ryan/map.yaml'},
            {'use_sim_time': False}
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )
    #=======================================================================

    # start a map_user node
    map_user = Node(
        package='crazyflie_ros2_multiranger_map_user',
        executable='map_user',
        name='map_user',
        output='screen',
        parameters=[
            {'robot_prefix': '/crazyflie_user_real'},
            {'use_sim_time': False},
            {'delay': 0.0},
            {'max_turn_rate': 0.7},
            {'max_forward_speed': 0.5},
            {'target_altitude': 0.5},
            {'alt_kp': 1.2},
            {'max_vz': 0.4},
            {'max_obstacle_distance': 0.3}
        ]
    )
    
    shared_mapper = Node(
    package='crazyflie_ros2_multiranger_shared_mapper',
    executable='shared_mapper_multiranger',
    name='shared_mapper',
    output='screen',
    parameters=[
        {'robot_prefixes': ['/crazyflie_user_real']},
        {'map_file': map_file},
        {'use_sim_time': False},
    ]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie_ros2_multiranger_bringup'),
        'config',
        'real_mapping.rviz')

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                "use_sim_time": False
            }]
            )

    return LaunchDescription([
        crazyflie_real,
        crazyflie_vel_mux,
        #simple_mapper,
        shared_mapper,
        #frontier_exploration,

        map_user,
        rviz,

        map_server,
        lifecycle_manager
        ])
