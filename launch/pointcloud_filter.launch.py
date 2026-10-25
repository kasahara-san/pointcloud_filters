"""Launch the filter, the bundled recording and an optional RViz comparison."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from pointcloud_filters.bag import resolve_bag_path


def _configured_values(context, overrides):
    params_file = LaunchConfiguration('params_file').perform(context)
    with open(params_file, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    values = dict(input_topic='/merged_pointcloud', pose_topic='/zx200/global_pose',
                  joint_states_topic='/zx200/joint_states',
                  use_dynamic_tf=False,
                  output_topic='/merged_pointcloud_filtered', output_format='pointcloud2')
    for name, entry in document.items():
        if name in ('/**', 'pointcloud_filter', '/pointcloud_filter'):
            values.update(entry.get('ros__parameters', {}))
    values.update(overrides)
    return values


def _rviz_node(context, overrides):
    values = _configured_values(context, overrides)
    config = LaunchConfiguration('rviz_config').perform(context)
    if not config:
        filename = ('pointcloud_filter_mesh.rviz' if values['output_format'] == 'mesh'
                    else 'pointcloud_filter.rviz')
        config = os.path.join(
            get_package_share_directory('pointcloud_filters'), 'rviz', filename)
    return Node(
        package='rviz2', executable='rviz2', name='pointcloud_filter_rviz', output='screen',
        arguments=['-d', config, '-f', LaunchConfiguration('fixed_frame')],
        parameters=[{'use_sim_time': overrides['use_sim_time']}],
        remappings=[('/merged_pointcloud', values['input_topic']),
                    ('/merged_pointcloud_filtered', values['output_topic']),
                    ('/merged_pointcloud_filtered/self_removed',
                     values['output_topic'] + '/self_removed'),
                    ('/merged_pointcloud_filtered/marker', values['output_topic'] + '/marker')])


def _bag_after_ready(filter_node, player):
    """Start playback once the filter finishes loading geometry and subscriptions."""
    sentinel = b'Ready for point clouds.'
    tails = {True: b'', False: b''}
    started = False

    def on_output(event):
        nonlocal started
        if started:
            return None
        stream = event.from_stdout
        text = tails[stream] + event.text
        if sentinel in text:
            started = True
            return [player]
        # ProcessIO boundaries can split any byte of a log line. Retain only
        # enough bytes for a cross-event match, independently for each stream.
        tails[stream] = text[-len(sentinel) + 1:]
        return None

    return RegisterEventHandler(OnProcessIO(
        target_action=filter_node, on_stdout=on_output, on_stderr=on_output))


def _launch_node(context):
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }
    types = dict(mesh_resolution=int, radius_m=float, self_filter_padding=float,
                 publish_period_sec=float, self_filter_scale=float)
    for name in ('output_format', 'mesh_resolution', 'output_topic', 'pose_topic', 'radius_m',
                 'joint_states_topic', 'self_filter', 'self_filter_padding',
                 'robot_description_path', 'publish_period_sec', 'use_dynamic_tf',
                 'self_filter_scale'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            if name in ('self_filter', 'use_dynamic_tf'):
                value = IfCondition(value).evaluate(context)
            elif name in types:
                value = types[name](value)
            overrides[name] = value
    filter_node = Node(
        package='pointcloud_filters', executable='pointcloud_filter',
        name='pointcloud_filter', output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides])
    nodes = [filter_node]
    if IfCondition(LaunchConfiguration('use_rviz')).evaluate(context):
        nodes.append(_rviz_node(context, overrides))
    if IfCondition(LaunchConfiguration('play_bag')).evaluate(context):
        bag_path = resolve_bag_path(LaunchConfiguration('bag_path').perform(context))
        rate = float(LaunchConfiguration('bag_rate').perform(context))
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('bag_rate must be finite and > 0')
        values = _configured_values(context, overrides)
        command = [
            'ros2', 'bag', 'play', bag_path, '--clock', '100', '--delay', '2',
            '--rate', str(rate), '--disable-keyboard-controls',
        ]
        if IfCondition(LaunchConfiguration('bag_loop')).evaluate(context):
            command.append('--loop')
        command.extend(['--topics', values['input_topic'], values['pose_topic'],
                        values['joint_states_topic'], '/tf_static'])
        if values['use_dynamic_tf']:
            command.append('/tf')
        player = ExecuteProcess(cmd=command, output='screen')
        # Register before the node starts, including fast starts without meshes.
        nodes.insert(0, _bag_after_ready(filter_node, player))
    return nodes


def generate_launch_description():
    """Provide filter settings and an RViz view selected by the output format."""
    default_config = os.path.join(
        get_package_share_directory('pointcloud_filters'),
        'config', 'pointcloud_filter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument(
            'play_bag', default_value='true',
            description='Replay the bundled recording; false uses live topics.'),
        DeclareLaunchArgument(
            'bag_path', default_value='',
            description='Rosbag directory/file; empty finds the bundled recording.'),
        DeclareLaunchArgument(
            'bag_loop', default_value='false', description='Loop the recording.'),
        DeclareLaunchArgument(
            'bag_rate', default_value='1.0', description='Positive playback rate.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value=LaunchConfiguration('play_bag'),
            description='Use the bag clock by default; live mode uses wall time.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', description='Start RViz2 for visual checks.'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='Custom RViz configuration; empty selects by output format.'),
        DeclareLaunchArgument(
            'fixed_frame', default_value='map', description='RViz fixed frame.'),
        DeclareLaunchArgument(
            'output_format', default_value='', choices=['', 'pointcloud2', 'mesh'],
            description='Output format; empty uses params_file.'),
        DeclareLaunchArgument(
            'mesh_resolution', default_value='', choices=['', '1', '2', '3', '4', '5'],
            description='Mesh resolution: 1 coarse, 5 fine; empty uses params_file.'),
        DeclareLaunchArgument(
            'output_topic', default_value='',
            description='Output topic; empty uses params_file.'),
        DeclareLaunchArgument(
            'pose_topic', default_value='',
            description='Backhoe PoseStamped topic; empty uses params_file.'),
        DeclareLaunchArgument(
            'radius_m', default_value='',
            description='XY extraction radius in metres; empty uses params_file.'),
        DeclareLaunchArgument(
            'joint_states_topic', default_value='',
            description='Backhoe JointState topic; empty uses params_file.'),
        DeclareLaunchArgument(
            'use_dynamic_tf', default_value='', choices=['', 'true', 'false'],
            description='Listen to and replay /tf; static world/map TF is sufficient by default.'),
        DeclareLaunchArgument(
            'self_filter', default_value='', choices=['', 'true', 'false'],
            description='Enable robot self-point removal; empty uses params_file.'),
        DeclareLaunchArgument(
            'self_filter_padding', default_value='',
            description='Padding around robot geometry in metres; empty uses params_file.'),
        DeclareLaunchArgument(
            'self_filter_scale', default_value='',
            description='Per-link geometry enlargement, >=1; empty uses params_file.'),
        DeclareLaunchArgument(
            'robot_description_path', default_value='',
            description='Robot URDF path; empty uses params_file.'),
        DeclareLaunchArgument(
            'publish_period_sec', default_value='',
            description='Wall-time processing period in seconds; empty uses params_file.'),
        OpaqueFunction(function=_launch_node),
    ])
