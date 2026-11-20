"""Verify that the only launch path uses the D37PXi onboard profile."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest
import yaml


@pytest.fixture
def launcher(monkeypatch, tmp_path):
    """Load the source D37PXi launch without starting ROS processes or a player."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'd37pxi_launch', root / 'launch/d37pxi.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(root))
    return module


def _context(launcher, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for entity in launcher.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def test_default_launch_uses_live_d37pxi_topics_without_recording(launcher):
    """Construction needs no external recording and starts the filter plus RViz."""
    context = _context(launcher)
    actions = launcher._launch_node(context)
    assert len(actions) == 2 and all(isinstance(action, Node) for action in actions)
    assert context.launch_configurations['use_sim_time'] == 'false'
    assert context.launch_configurations['fixed_frame'] == 'd37pxi/filter_base'
    assert Path(context.launch_configurations['params_file']).name == 'd37pxi.yaml'
    values = launcher._configured_values(context, {})
    assert values['input_topic'] == '/lidar_points'
    assert values['pose_topic'] == '/d37pxi/global_pose'
    assert values['joint_states_topic'] == '/d37pxi/joint_states'
    assert values['output_topic'] == '/d37pxi/lidar_points_filtered'


def test_headless_mode_starts_only_the_filter(launcher):
    """Support unattended filtering without starting a GUI or bag process."""
    actions = launcher._launch_node(_context(launcher, use_rviz='false'))
    assert len(actions) == 1 and isinstance(actions[0], Node)


@pytest.mark.parametrize('output_format', ['pointcloud2', 'mesh'])
def test_custom_topics_reach_rviz_and_filter_configuration(launcher, tmp_path, output_format):
    """Keep the visualization connected when the profile or output format changes."""
    params = tmp_path / 'settings.yaml'
    params.write_text('pointcloud_filter:\n  ros__parameters:\n    input_topic: /custom/cloud\n')
    context = _context(launcher, params_file=str(params), output_format=output_format,
                       output_topic='/custom/filtered')
    received = []
    sentinel = object()
    launcher._rviz_node = lambda context, values, overrides: received.append(values) or sentinel
    actions = launcher._launch_node(context)
    assert len(actions) == 2 and actions[-1] is sentinel
    assert received[0]['input_topic'] == '/custom/cloud'
    assert received[0]['output_topic'] == '/custom/filtered'
    assert received[0]['output_format'] == output_format
    assert received[0]['pose_topic'] == '/d37pxi/global_pose'


def test_rviz_profiles_use_d37pxi_topics_and_visualization_frame():
    """Both output formats must show the D37PXi model and filter diagnostics."""
    root = Path(__file__).resolve().parents[1]
    for name in ('d37pxi.rviz', 'd37pxi_mesh.rviz'):
        config = yaml.safe_load((root / 'rviz' / name).read_text())['Visualization Manager']
        assert config['Global Options']['Fixed Frame'] == 'd37pxi/filter_base'
        topics = [display['Topic']['Value'] for display in config['Displays']
                  if 'Topic' in display]
        assert '/lidar_points' in topics
        assert '/d37pxi/lidar_points_filtered/robot_model' in topics
        assert '/d37pxi/lidar_points_filtered/self_removed' in topics
