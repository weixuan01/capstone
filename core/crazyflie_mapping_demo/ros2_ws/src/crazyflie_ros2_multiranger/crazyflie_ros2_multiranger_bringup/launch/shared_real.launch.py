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
#   - crazyflie_server           (Crazyswarm2 — connects to all physical drones)
#   - shared_mapper              (one unified occupancy map for all drones)
#   - drone_manager              (battery monitoring + land command handling)
#   - goal_assigner              (only when launch_goal_assigner:=true)
#   - object_detection_planner   (only when launch_object_detection_planner:=true)
#   - rviz                       (visualiser, using the real-mode config)
#
# This file is NOT responsible for any per-drone behaviour nodes.
# Those are launched separately by the per-drone launch files.
#
# Launch arguments:
#   robot_prefixes                   Comma-separated list of all drone namespaces, e.g. [/cf1,/cf2]
#   launch_goal_assigner             true | false — start the centralised goal assigner
#   swarm_prefixes                   Comma-separated list of swarm drone namespaces only, e.g. [/cf1,/cf2]
#                                    Only used when launch_goal_assigner is true.
#   launch_object_detection_planner  true | false — start the centralised object-detection planner
#   scanner_prefixes                 Comma-separated list of scanner drone namespaces only, e.g. [/cf3,/cf4]
#                                    Only used when launch_object_detection_planner is true.
# =============================================================================


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_prefixes',
            default_value='[/crazyflie]',
            description='Comma-separated list of all drone namespaces, e.g. [/cf1,/cf2]'
        ),
        DeclareLaunchArgument(
            'launch_goal_assigner',
            default_value='false',
            description='Set true to launch the centralised goal assigner for swarm exploration'
        ),
        DeclareLaunchArgument(
            'swarm_prefixes',
            default_value='[]',
            description='Namespaces of drones using frontier-exploration-swarm, e.g. [/cf1,/cf2]'
        ),
        DeclareLaunchArgument(
            'launch_object_detection_planner',
            default_value='false',
            description='Set true to launch the centralised object-detection planner for scanner drones'
        ),
        DeclareLaunchArgument(
            'scanner_prefixes',
            default_value='[]',
            description='Namespaces of drones using object-detection, e.g. [/cf3,/cf4]'
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Optional path to a previously saved map.yaml to pre-load into the shared mapper'
        ),
        OpaqueFunction(function=_launch_setup),
    ])


def _launch_setup(context, *args, **kwargs):
    prefixes_str          = context.launch_configurations['robot_prefixes']
    launch_goal_assigner  = context.launch_configurations['launch_goal_assigner'].lower() == 'true'
    swarm_prefixes_str    = context.launch_configurations['swarm_prefixes']
    launch_od_planner     = context.launch_configurations['launch_object_detection_planner'].lower() == 'true'
    scanner_prefixes_str  = context.launch_configurations['scanner_prefixes']
    map_file              = context.launch_configurations['map_file']

    prefixes_list = [p.strip() for p in prefixes_str.strip('[]').split(',') if p.strip()]
    swarm_list    = [p.strip() for p in swarm_prefixes_str.strip('[]').split(',') if p.strip()]
    scanner_list  = [p.strip() for p in scanner_prefixes_str.strip('[]').split(',') if p.strip()]

    drone_names = [p.lstrip('/') for p in prefixes_list]

    pkg_crazyswarm2 = get_package_share_directory('crazyflie')
    pkg_bringup     = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    crazyflies_yaml = os.path.join(pkg_bringup, 'config', 'crazyflie_real_crazyswarm2.yaml')

    # ── Crazyswarm2 server ────────────────────────────────────────────────────
    crazyflie_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyswarm2, 'launch', 'launch.py')
        ),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml,
            'backend': 'cflib',
            'mocap':   'False',
            'rviz':    'False',
            'gui':     'False',
            'teleop':  'False',
        }.items(),
    )

    # ── Shared mapper ─────────────────────────────────────────────────────────
    shared_mapper_params = [
        {'robot_prefixes': prefixes_list},
        {'use_sim_time':   False},
    ]
    if map_file:
        shared_mapper_params.append({'map_file': map_file})

    shared_mapper = Node(
        package='crazyflie_ros2_multiranger_shared_mapper',
        executable='shared_mapper_multiranger',
        name='shared_mapper',
        output='screen',
        parameters=shared_mapper_params,
    )

    # ── Mission control ─────────────────────────────────────────────────────────
    mission_control = Node(
        package='crazyflie_ros2_mission_control',
        executable='mission_control',
        name='mission_control',
        output='screen',
        parameters=[
            {'drone_names':  drone_names},
            {'publish_rate': 1.0},
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

    nodes = [crazyflie_server, shared_mapper, mission_control, rviz]

    # ── Goal assigner (swarm mode only) ───────────────────────────────────────
    # Only launched when launch-universal.sh passes launch_goal_assigner:=true,
    # which happens when at least one drone uses frontier-exploration-swarm.
    if launch_goal_assigner:
        goal_assigner = Node(
            package='crazyflie_ros2_multiranger_goal_assigner',
            executable='goal_assigner',
            name='goal_assigner',
            output='screen',
            parameters=[
                {'robot_prefixes': swarm_list},
                {'use_sim_time':   False},
            ],
        )
        nodes.append(goal_assigner)

    # ── Object-detection planner (scanner mode only) ──────────────────────────
    # Only launched when launch-universal.sh passes
    # launch_object_detection_planner:=true, which happens when at least one
    # drone uses object-detection.  Runs greedy set cover + Hungarian
    # assignment over the shared map and publishes per-drone scan points.
    if launch_od_planner:
        object_detection_planner = Node(
            package='crazyflie_ros2_object_detection_planner',
            executable='object_detection_planner',
            name='object_detection_planner',
            output='screen',
            parameters=[
                {'robot_prefixes': scanner_list},
                {'use_sim_time':   False},
            ],
        )
        nodes.append(object_detection_planner)

    return nodes
