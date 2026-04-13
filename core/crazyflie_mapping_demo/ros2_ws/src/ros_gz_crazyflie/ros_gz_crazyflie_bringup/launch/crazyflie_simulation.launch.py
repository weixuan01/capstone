# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node


# Spacing between drones at spawn (metres)
SPAWN_SPACING = 0.5


def _generate_bridge_yaml(prefixes, tmp_dir):
    """Generate a ros_gz bridge YAML covering all drones. Returns file path."""
    entries = []

    for prefix in prefixes:
        p = prefix.lstrip('/')
        entries += [
            f'- ros_topic_name: "{prefix}/cmd_vel"\n'
            f'  gz_topic_name: "/{p}/gazebo/command/twist"\n'
            f'  ros_type_name: "geometry_msgs/msg/Twist"\n'
            f'  gz_type_name: "ignition.msgs.Twist"\n'
            f'  direction: ROS_TO_GZ\n',

            f'- ros_topic_name: "{prefix}/odom"\n'
            f'  gz_topic_name: "/model/{p}/odometry"\n'
            f'  ros_type_name: "nav_msgs/msg/Odometry"\n'
            f'  gz_type_name: "gz.msgs.Odometry"\n'
            f'  direction: GZ_TO_ROS\n',

            f'- ros_topic_name: "/tf"\n'
            f'  gz_topic_name: "/model/{p}/pose"\n'
            f'  ros_type_name: "tf2_msgs/msg/TFMessage"\n'
            f'  gz_type_name: "gz.msgs.Pose_V"\n'
            f'  direction: GZ_TO_ROS\n',

            f'- ros_topic_name: "{prefix}/scan"\n'
            f'  gz_topic_name: "/{p}/lidar"\n'
            f'  ros_type_name: "sensor_msgs/msg/LaserScan"\n'
            f'  gz_type_name: "ignition.msgs.LaserScan"\n'
            f'  direction: GZ_TO_ROS\n',
        ]

    entries.append(
        '- ros_topic_name: "/clock"\n'
        '  gz_topic_name: "/clock"\n'
        '  ros_type_name: "rosgraph_msgs/msg/Clock"\n'
        '  gz_type_name: "gz.msgs.Clock"\n'
        '  direction: GZ_TO_ROS\n'
    )

    yaml_path = os.path.join(tmp_dir, 'ros_gz_crazyflie_bridge.yaml')
    with open(yaml_path, 'w') as f:
        f.write('---\n')
        for entry in entries:
            f.write(entry)
    return yaml_path


def _write_drone_sdf(model_sdf_path, prefix, tmp_dir):
    """
    Copy crazyflie_drone.sdf with all namespace-bearing 'crazyflie' tokens
    replaced by the drone prefix (e.g. 'cf1'), and mesh URIs resolved to
    absolute file:// paths so Gazebo can find them from the temp directory.

    Only SDF structural tokens are replaced — the plugin class names
    (gz::sim::systems::*) and free-text comments are left untouched.
    Returns path to the written temp file.
    """
    import re
    p = prefix.lstrip('/')
    model_dir = os.path.dirname(model_sdf_path)

    with open(model_sdf_path, 'r') as f:
        sdf = f.read()

    # ── Targeted namespace replacement ────────────────────────────────────────
    patterns = [
        (r'(<(?:model|link|joint|visual|collision|sensor)\s+name=")crazyflie(/)', rf'\g<1>{p}\2'),
        (r'(<(?:model|link|joint|visual|collision|sensor)\s+name=")crazyflie(")', rf'\g<1>{p}\2'),
        (r'(<(?:robotNamespace|comLinkName|jointName|linkName|child|parent)>)crazyflie(/)', rf'\g<1>{p}\2'),
        (r'(<(?:robotNamespace|comLinkName|jointName|linkName|child|parent)>)crazyflie(<)', rf'\g<1>{p}\2'),
        (r'<topic>lidar</topic>', f'<topic>{p}/lidar</topic>'),
    ]
    for pat, repl in patterns:
        sdf = re.sub(pat, repl, sdf)

    # ── Resolve relative mesh URIs to absolute file:// paths ─────────────────
    def _abs_uri(match):
        rel = match.group(1)
        abs_path = os.path.normpath(os.path.join(model_dir, rel))
        return f'<uri>file://{abs_path}</uri>'
    sdf = re.sub(r'<uri>((?:\.\./)*meshes/[^<]+)</uri>', _abs_uri, sdf)

    out = os.path.join(tmp_dir, f'model_{p}.sdf')
    with open(out, 'w') as f:
        f.write(sdf)
    return out


def _write_world_sdf(world_sdf_path, prefixes, tmp_dir):

    world_dir = os.path.dirname(world_sdf_path)

    with open(world_sdf_path, 'r') as f:
        world = f.read()

    # Resolve all relative URIs in the world SDF to absolute file:// paths
    def _abs_uri(match):
        rel = match.group(1)
        abs_path = os.path.normpath(os.path.join(world_dir, rel))
        return f'<uri>file://{abs_path}</uri>'
    world = re.sub(r'<uri>(?!file://)([^<]+)</uri>', _abs_uri, world)

    includes = ''
    for i, prefix in enumerate(prefixes):
        p = prefix.lstrip('/')
        sdf_path = os.path.join(tmp_dir, f'model_{p}.sdf')
        spawn_x = float(i) * SPAWN_SPACING
        includes += f'''
    <include>
      <uri>file://{sdf_path}</uri>
      <name>{p}</name>
      <pose>{spawn_x} 0.0 0.0 0 0 0</pose>
    </include>'''

    world = world.replace('</world>', includes + '\n</world>')

    out = os.path.join(tmp_dir, 'world_with_drones.sdf')
    with open(out, 'w') as f:
        f.write(world)
    return out


def _launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    model_sdf_path = os.path.join(pkg_gazebo, 'models', 'crazyflie', 'crazyflie_drone.sdf')

    robot_prefixes_str = context.launch_configurations.get('robot_prefixes', '[/crazyflie]')
    prefixes = [p.strip() for p in robot_prefixes_str.strip('[]').split(',') if p.strip()]

    world_arg = context.launch_configurations.get('world', 'maze')
    tmp_dir = tempfile.mkdtemp(prefix='crazyflie_sim_')

    actions = []

    # ── Write all drone SDFs first ────────────────────────────────────────────
    for prefix in prefixes:
        _write_drone_sdf(model_sdf_path, prefix, tmp_dir)

    # ── Write world SDF with drones embedded ─────────────────────────────────
    world_sdf_path = os.path.join(pkg_gazebo, 'worlds', world_arg + '.sdf')
    world_with_drones = _write_world_sdf(world_sdf_path, prefixes, tmp_dir)

    # ── Gazebo world ──────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': world_with_drones + ' -r'
        }.items(),
    )
    actions.append(gz_sim)

    # ── Bridge (one node, all drones) ─────────────────────────────────────────
    bridge_yaml = _generate_bridge_yaml(prefixes, tmp_dir)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen',
    )
    actions.append(TimerAction(period=3.0, actions=[bridge]))

    # ── Per-drone: send enable signal to arm all 4 motors ────────────────────
    # Without this the MulticopterVelocityControl plugin only drives 2 of 4
    # rotors when the drone model is loaded at world start rather than spawned
    # dynamically. The delay gives Gazebo time to fully initialise the plugin.
    for i, prefix in enumerate(prefixes):
        p = prefix.lstrip('/')
        enable = ExecuteProcess(
            cmd=[
                'gz', 'topic',
                '-t', f'/{p}/enable',
                '-m', 'gz.msgs.Boolean',
                '-p', 'data: true',
            ],
            output='screen',
        )
        actions.append(TimerAction(period=6.0 + i * 0.5, actions=[enable]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_prefixes', default_value='[/crazyflie]'),
        DeclareLaunchArgument('world',          default_value='maze'),
        OpaqueFunction(function=_launch_setup),
    ])
