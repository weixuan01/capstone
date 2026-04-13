import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# frontier_exploration_real.launch.py
#
# Per-drone launch file for frontier exploration in real mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - vel_mux               Translates cmd_vel into drone-safe velocity commands
#   - frontier_exploration  Autonomously explores unknown space
#
# NOT launched here (handled by shared_real.launch.py):
#   - crazyflie_server
#   - shared_mapper
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

    # ── Velocity multiplexer ──────────────────────────────────────────────────
    # Translates incoming cmd_vel commands into drone-safe velocity setpoints.
    vel_mux = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name=f'vel_mux_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=[
            {'hover_height':         0.3},
            {'incoming_twist_topic': '/cmd_vel_safe'},
            {'robot_prefix':         robot_prefix},
        ],
    )

    # ── AI ──────────────────────────────────────────────────
    # Autonomously scans the shared map.
    ai = Node(
        package='crazyflie_ros2_ai',
        executable='ai',
        name=f'ai_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=[
            {'robot_prefix':         robot_prefix},
            {'use_sim_time':         False},
            {'delay':                5.0},
            {'max_turn_rate':        0.7},
            {'max_forward_speed':    0.5},
            {'target_altitude':      0.5},
            {'alt_kp':               1.2},
            {'max_vz':               0.4},
            {'max_obstacle_distance':0.3},
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

    # ── Collision avoidance ───────────────────────────────────────────────────
    # Sits between the exploration node and vel_mux. Passes cmd_vel_raw through
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

    return [vel_mux, ai, aideck_udp_streamer, collision_avoidance]
