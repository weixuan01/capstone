import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# wallfollowing_sim.launch.py
#
# Per-drone launch file for wall following in sim mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - wall_following    Follows walls autonomously
#
# NOT launched here (handled by shared_sim.launch.py):
#   - Gazebo simulator
#   - shared_mapper
#   - rviz
#
# Note: vel_mux is not needed in sim mode — Gazebo receives cmd_vel directly
# via the ros_gz bridge. aideck_udp_streamer is real-hardware only.
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

    # ── Wall following ────────────────────────────────────────────────────────
    # Follows walls autonomously using multiranger sensor readings.
    wall_following = Node(
        package='crazyflie_ros2_multiranger_wall_following',
        executable='wall_following_multiranger',
        name=f'wall_following_{robot_prefix.lstrip("/")}',
        output='screen',
        parameters=[
            {'robot_prefix':             robot_prefix},
            {'use_sim_time':             True},
            {'delay':                    5.0},
            {'max_turn_rate':            0.7},
            {'max_forward_speed':        0.5},
            {'wall_following_direction': 'right'},
            {'takeoff_mode':             'trigger'},
            {'takeoff_ready_time':       1.0},
            {'takeoff_speed':            0.5},
        ],
    )

    return [wall_following]
