"""Verify bag/live launch behavior and replay configuration before starting ROS."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.events.process import ProcessStderr, ProcessStdout
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest


@pytest.fixture
def launcher(monkeypatch, tmp_path):
    """Load the source launch file using the source package as its share directory."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'pointcloud_filter_launch', root / 'launch' / 'pointcloud_filter.launch.py')
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


def _command(action, context):
    return [perform_substitutions(context, part) for part in action.cmd]


def _output_event(action, text, event_type=ProcessStderr):
    return event_type(text=text, action=action, name='filter', cmd=['filter'],
                      cwd=None, env=None, pid=123)


def _ready_player(actions, context):
    registration, filter_node = actions[:2]
    assert isinstance(registration, RegisterEventHandler)
    assert isinstance(filter_node, Node)
    event = _output_event(filter_node, b'[INFO] Ready for point clouds.\n')
    handler = registration.event_handler
    assert handler.matches(event)
    result = handler.handle(event, context)
    assert len(result) == 1 and isinstance(result[0], ExecuteProcess)
    return result[0]


def test_default_replays_recording_with_clock_and_selected_inputs(launcher):
    """The no-argument launch must actually replay all robot filtering inputs."""
    context = _context(launcher, use_rviz='false')
    actions = launcher._launch_node(context)
    assert len(actions) == 2
    command = _command(_ready_player(actions, context), context)
    assert command[:3] == ['ros2', 'bag', 'play']
    assert Path(command[3]).name == 'rosbag2_2026_10_25-08_46_47'
    assert command[4:6] == ['--clock', '100']
    assert command[command.index('--topics') + 1:] == [
        '/merged_pointcloud', '/zx200/global_pose', '/zx200/joint_states', '/tf_static']
    assert '--loop' not in command
    assert context.launch_configurations['use_sim_time'] == 'true'


def test_live_mode_needs_no_recording_and_defaults_to_wall_time(launcher):
    """A live launch cannot require a bundled bag or wait for an absent bag clock."""
    context = _context(
        launcher, play_bag='false', use_rviz='false', bag_path='/nonexistent/recording')
    actions = launcher._launch_node(context)
    assert len(actions) == 1 and isinstance(actions[0], Node)
    assert context.launch_configurations['use_sim_time'] == 'false'


def test_replay_path_loop_rate_and_yaml_topic_overrides(launcher, tmp_path):
    """Replay options and configured input names reach the bag player intact."""
    bag = tmp_path / 'replacement'
    bag.mkdir()
    (bag / 'metadata.yaml').touch()
    params = tmp_path / 'settings.yaml'
    params.write_text(
        'pointcloud_filter:\n  ros__parameters:\n'
        '    input_topic: /custom/cloud\n    joint_states_topic: /custom/joints\n')
    context = _context(
        launcher, use_rviz='false', bag_path=str(bag), bag_loop='true', bag_rate='2.5',
        params_file=str(params), pose_topic='/custom/pose', self_filter='false')
    actions = launcher._launch_node(context)
    command = _command(_ready_player(actions, context), context)
    assert command[3] == str(bag)
    assert '--loop' in command
    assert command[command.index('--rate') + 1] == '2.5'
    assert command[command.index('--topics') + 1:] == [
        '/custom/cloud', '/custom/pose', '/custom/joints', '/tf_static']


@pytest.mark.parametrize('yaml_value,override,expected', [
    (False, '', False), (True, '', True), (True, 'false', False), (False, 'true', True),
])
def test_dynamic_tf_replay_requires_explicit_opt_in(
        launcher, tmp_path, yaml_value, override, expected):
    """Dynamic bag TF must be absent unless enabled in YAML or a launch override."""
    params = tmp_path / 'settings.yaml'
    params.write_text(
        'pointcloud_filter:\n  ros__parameters:\n'
        f'    use_dynamic_tf: {str(yaml_value).lower()}\n')
    context = _context(
        launcher, use_rviz='false', params_file=str(params), use_dynamic_tf=override)
    actions = launcher._launch_node(context)
    command = _command(_ready_player(actions, context), context)
    topics = command[command.index('--topics') + 1:]
    assert ('/tf' in topics) == expected
    assert '/tf_static' in topics


@pytest.mark.parametrize('stream', [ProcessStdout, ProcessStderr])
def test_readiness_handles_split_bytes_and_only_starts_player_once(launcher, stream):
    """Readiness logs can be fragmented; repeated logs cannot create more players."""
    context = _context(launcher, use_rviz='false')
    registration, filter_node = launcher._launch_node(context)
    handler = registration.event_handler
    for text in (b'loading geometry\n' * 100, b'[INFO] Rea', b'dy for point '):
        assert handler.handle(_output_event(filter_node, text, stream), context) is None
    result = handler.handle(_output_event(filter_node, b'clouds.\n', stream), context)
    assert len(result) == 1 and isinstance(result[0], ExecuteProcess)
    assert handler.handle(
        _output_event(filter_node, b'Ready for point clouds.\n', stream), context) is None


def test_readiness_does_not_mix_streams_or_match_other_processes(launcher):
    """Only a complete sentinel from the filter's own output can begin playback."""
    context = _context(launcher, use_rviz='false')
    registration, filter_node = launcher._launch_node(context)
    handler = registration.event_handler
    other = ExecuteProcess(cmd=['false'])
    assert not handler.matches(_output_event(other, b'Ready for point clouds.'))
    assert handler.handle(
        _output_event(filter_node, b'Open3D initialization failed\n'), context) is None
    assert handler.handle(_output_event(filter_node, b'Ready for point '), context) is None
    assert handler.handle(
        _output_event(filter_node, b'clouds.', ProcessStdout), context) is None
    result = handler.handle(_output_event(filter_node, b'clouds.'), context)
    assert len(result) == 1 and isinstance(result[0], ExecuteProcess)


@pytest.mark.parametrize('rate', ['0', '-1', 'nan', 'inf'])
def test_invalid_replay_rate_fails_before_starting_processes(launcher, rate):
    """Reject rates that cannot describe forward, finite playback."""
    context = _context(launcher, use_rviz='false', bag_rate=rate)
    with pytest.raises(ValueError, match='bag_rate'):
        launcher._launch_node(context)
