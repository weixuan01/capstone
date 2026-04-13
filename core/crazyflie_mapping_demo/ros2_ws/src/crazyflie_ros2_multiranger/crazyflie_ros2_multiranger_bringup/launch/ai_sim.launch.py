import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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

    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    
    control_services = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        name=f"{robot_prefix.strip('/')}_control_services",
        output='screen',
        parameters=[
            {'hover_height': 0.3},
            {'robot_prefix': robot_prefix},
            {'incoming_twist_topic': '/cmd_vel_safe'},
            {'max_ang_z_rate': 0.4},
            {'use_sim_time': True},
        ]
    )
    
    # ── Collision avoidance ───────────────────────────────────────────────────
    # Sits between the exploration node and control services. Passes cmd_vel_raw through as cmd_vel_safe
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
	
    ai = Node(
        package='crazyflie_ros2_ai',
        executable='ai',
        name=f"{robot_prefix.strip('/')}_ai",
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
            {'use_sim_time': True}
        ]
    )

    
    return [
        control_services,
        ai,
        collision_avoidance
    ]
