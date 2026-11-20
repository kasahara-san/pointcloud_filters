"""Run the D37PXi onboard profile against topics already being published."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Reuse the generic filter launch with onboard-specific defaults."""
    share = get_package_share_directory('pointcloud_filters')
    defaults = {
        'params_file': os.path.join(share, 'config', 'd37pxi.yaml'),
        'play_bag': 'false',
        'use_rviz': 'true',
        'fixed_frame': 'd37pxi/filter_base',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'pointcloud_filter.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in defaults}.items()),
    ])
