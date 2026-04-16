import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


# =============================================================================
# object_detection_scanner_real.launch.py
#
# Per-drone launch file for the object-detection scanner in real mode.
# Called once per drone by launch-universal.sh.
#
# Launches:
#   - vel_mux                    Translates cmd_vel into drone-safe velocity commands
#   - aideck_udp_streamer        Streams AI deck camera + runs digit inference
#   - object_detection_scanner   Navigates to scan points assigned by the
#                                centralised object_detection_planner and
#                                performs a 360-degree spin at each one
#   - collision_avoidance        Peer-repulsion layer between navigator and vel_mux
#
# NOT launched here (handled by shared_real.launch.py):
#   - crazyflie_server
#   - shared_mapper
#   - object_detection_planner
#   - rviz
#
# Launch arguments:
#   robot_prefix   ROS 2 namespace for this drone, e.g. /cf3
# =============================================================================


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_prefix',
            default_value='/crazyflie',
            description='ROS 2 namespace for this drone, e.g. /cf3'
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    robot_prefix = context.launch_configurations['robot_prefix']
    # Pre-compute the bare drone name once so f-strings stay simple.
    drone_id = robot_prefix.lstrip('/')

    # ── Velocity multiplexer ──────────────────────────────────────────────────
    # Translates incoming cmd_vel commands into drone-safe velocity setpoints.
    vel_mux = Node(
        package='crazyflie',
        executable='vel_mux.py',
        name=f'vel_mux_{drone_id}',
        output='screen',
        parameters=[
            {'hover_height':         0.3},
            {'incoming_twist_topic': '/cmd_vel_safe'},
            {'robot_prefix':         robot_prefix},
        ],
    )
    
    # ── AI deck UDP streamer ──────────────────────────────────────────────────
    # Streams the AI deck camera feed to the companion computer over UDP and
    # runs ONNX digit inference.  Publishes:
    #   /{drone_id}/aideck/image_raw   — overlay image for debugging
    #   /aideck/mnist_input            — 28x28 MNIST-preprocessed crop
    #   /aideck/digit_prediction       — Int32 digit (0..9) when confident
    #
    # NOTE: the prediction topic is currently hard-coded to /aideck/digit_prediction
    # in the streamer source.  If you ever run more than one AI-deck drone at
    # once, the streamer needs to be patched to prefix its publishers — otherwise
    # both scanners would publish to the same topic and both would see each
    # other's predictions.
    aideck_udp_streamer = Node(
        package='crazyflie',
        executable='aideck_udp_streamer.py',
        name=f'{drone_id}_aideck_udp_streamer',
        output='screen',
        parameters=[
            {'deck_ip':                   '192.168.4.1'},
            {'deck_port':                 5000},
            {'listen_ip':                 '0.0.0.0'},
            {'listen_port':               5001},
            {'image_topic':               f'/{drone_id}/aideck/image_raw'},
            {'robot_prefix':              robot_prefix},
            {'start_after_takeoff':       False},
            {'start_height_threshold':    0.24},
            {'start_stable_delay':        1.0},
            {'require_fresh_odom':        False},
            {'odom_timeout_sec':          0.3},
            {'start_retry_seconds':       2.0},
            {'restart_backoff_sec':       1.0},
            {'prediction_conf_threshold': 0.85},
            {'enable_prediction':         True},
            {'publish_mnist_image':       True},
            {'log_fps':                   False},
        ],
    )
    
    # ── Object-detection scanner ──────────────────────────────────────────────
    # Receives scan-point assignments from the centralised
    # object_detection_planner and navigates to them via A*.  On arrival,
    # performs a full 360-degree spin for object detection, then reports
    # REACHED so the planner marks the disc as scanned.
    object_detection_scanner = Node(
        package='crazyflie_ros2_object_detection_scanner',
        executable='object_detection_scanner',
        name=f'{drone_id}_object_detection_scanner',
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
            {'use_sim_time': False},
        ],
    )

    # ── Collision avoidance ───────────────────────────────────────────────────
    # Sits between the navigator and vel_mux.  Passes cmd_vel_raw through
    # unchanged during normal flight. Overrides with a resolution manoeuvre
    # when a peer drone enters the safety radius, then hands back control.
    collision_avoidance = Node(
        package='crazyflie_ros2_collision_avoidance',
        executable='collision_avoidance',
        name=f'collision_avoidance_{drone_id}',
        output='screen',
        parameters=[
            {'robot_prefix': robot_prefix},
        ],
    )

    return [
        vel_mux,
        aideck_udp_streamer,
        object_detection_scanner,
        collision_avoidance,
    ]
