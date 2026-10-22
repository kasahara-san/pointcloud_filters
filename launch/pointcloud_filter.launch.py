"""Launch the point cloud filter and an optional RViz comparison view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _rviz_node(context, overrides):
    params_file = LaunchConfiguration('params_file').perform(context)
    with open(params_file, encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    values = dict(input_topic='/merged_pointcloud',
                  output_topic='/merged_pointcloud_filtered', output_format='pointcloud2')
    for name, entry in document.items():
        if name in ('/**', 'pointcloud_filter', '/pointcloud_filter'):
            values.update(entry.get('ros__parameters', {}))
    values.update(overrides)
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
                    ('/merged_pointcloud_filtered/marker', values['output_topic'] + '/marker')])


def _launch_node(context):
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }
    for name in ('output_format', 'mesh_resolution', 'output_topic', 'pose_topic', 'radius_m'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            if name == 'mesh_resolution':
                value = int(value)
            elif name == 'radius_m':
                value = float(value)
            overrides[name] = value
    nodes = [Node(
        package='pointcloud_filters', executable='pointcloud_filter',
        name='pointcloud_filter', output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides])]
    if IfCondition(LaunchConfiguration('use_rviz')).evaluate(context):
        nodes.append(_rviz_node(context, overrides))
    return nodes


def generate_launch_description():
    """Provide filter settings and an RViz view selected by the output format."""
    default_config = os.path.join(
        get_package_share_directory('pointcloud_filters'),
        'config', 'pointcloud_filter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
        OpaqueFunction(function=_launch_node),
    ])
