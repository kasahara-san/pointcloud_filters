"""Launch the D37PXi onboard filter and its optional live RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


OVERRIDES = dict(
    input_topic=str, pose_topic=str, joint_states_topic=str, output_topic=str,
    output_format=str, mesh_resolution=int, radius_m=float, voxel_size=float,
    self_filter=bool, self_filter_padding=float, self_filter_scale=float,
    self_filter_geometry=str, robot_description_path=str, publish_period_sec=float)


def _configured_values(context, overrides):
    values = {}
    share = get_package_share_directory('pointcloud_filters')
    for path in (os.path.join(share, 'config', 'd37pxi.yaml'),
                 LaunchConfiguration('params_file').perform(context)):
        with open(path, encoding='utf-8') as stream:
            for name, entry in (yaml.safe_load(stream) or {}).items():
                if name in ('/**', 'pointcloud_filter', '/pointcloud_filter'):
                    values.update(entry.get('ros__parameters', {}))
    values.update(overrides)
    return values


def _rviz_node(context, values, overrides):
    config = LaunchConfiguration('rviz_config').perform(context)
    if not config:
        config = os.path.join(
            get_package_share_directory('pointcloud_filters'), 'rviz',
            'd37pxi_mesh.rviz' if values['output_format'] == 'mesh' else 'd37pxi.rviz')
    base_topic = '/d37pxi/lidar_points_filtered'
    return Node(
        package='rviz2', executable='rviz2', name='pointcloud_filter_rviz', output='screen',
        arguments=['-d', config, '-f', LaunchConfiguration('fixed_frame')],
        parameters=[{'use_sim_time': overrides['use_sim_time']}],
        remappings=[('/tf', '/d37pxi/filter_visualization_tf'),
                    ('/lidar_points', values['input_topic']),
                    (base_topic, values['output_topic']),
                    *[(base_topic + suffix, values['output_topic'] + suffix)
                      for suffix in ('/self_removed', '/robot_model', '/marker')]])


def _launch_node(context):
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }
    for name, kind in OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = (IfCondition(value).evaluate(context) if kind is bool
                               else kind(value))
    nodes = [Node(
        package='pointcloud_filters', executable='pointcloud_filter',
        name='pointcloud_filter', output='screen',
        parameters=[LaunchConfiguration('params_file'), overrides])]
    if IfCondition(LaunchConfiguration('use_rviz')).evaluate(context):
        nodes.append(_rviz_node(context, _configured_values(context, overrides), overrides))
    return nodes


def generate_launch_description():
    """Use the D37PXi profile and already published sensor topics by default."""
    share = get_package_share_directory('pointcloud_filters')
    defaults = {
        'params_file': os.path.join(share, 'config', 'd37pxi.yaml'),
        'use_rviz': 'true',
        'use_sim_time': 'false',
        'fixed_frame': 'd37pxi/filter_base',
        'rviz_config': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        *[DeclareLaunchArgument(
            name, default_value='', description=f'Override {name}; empty uses params_file.')
          for name in OVERRIDES],
        OpaqueFunction(function=_launch_node),
    ])
