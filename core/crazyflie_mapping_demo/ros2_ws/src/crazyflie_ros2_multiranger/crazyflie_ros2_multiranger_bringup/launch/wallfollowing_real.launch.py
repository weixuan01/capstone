import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# wallfollowing_real.launch.py
#
# Per-drone launch file for wall following in real mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - vel_mux           Translates cmd_vel into drone-safe velocity commands
#   - wall_following    Follows walls autonomously
#   - aideck_udp_streamer  Streams video from the AI deck over UDP
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
    p = robot_prefix.lstrip('/')

    # ── Velocity multiplexer ──────────────────────────────────────────────────
    # Translates incoming cmd_vel commands into drone-safe velocity setpoints.
    vel_mux = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name=f'vel_mux_{p}',
        output='screen',
        parameters=[
            {'hover_height':         0.4},
            {'incoming_twist_topic': '/cmd_vel'},
            {'robot_prefix':         robot_prefix},
        ],
    )

    # ── Wall following ────────────────────────────────────────────────────────
    # Follows walls autonomously using multiranger sensor readings.
    wall_following = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name=f'wall_following_{p}',
        output='screen',
        parameters=[
            {'robot_prefix':             robot_prefix},
            {'use_sim_time':             False},
            {'delay':                    5.0},
            {'max_turn_rate':            0.5},
            {'max_forward_speed':        0.3},
            {'wall_following_direction': 'right'},
            {'invert_yaw_command':       True},
            {'takeoff_mode':             'trigger'},
            {'takeoff_height':           0.30},
            {'takeoff_speed':            0.35},
            {'takeoff_timeout':          4.0},
        ],
    )

    # ── AI deck UDP streamer ──────────────────────────────────────────────────
    # Receives the video stream from the AI deck and publishes it as a ROS topic.
    aideck_udp_streamer = Node(
        package='crazyflie',
        executable='aideck_udp_streamer.py',
        name=f'aideck_udp_streamer_{p}',
        output='screen',
        parameters=[
            {'deck_ip':    '192.168.4.1'},
            {'deck_port':  5000},
            {'listen_ip':  '0.0.0.0'},
            {'listen_port':5001},
            {'image_topic':'/aideck/image_raw'},
        ],
    )

    return [vel_mux, wall_following, aideck_udp_streamer]
