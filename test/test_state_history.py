"""Check measurement-time matching, partial joint updates and replay resets."""

from geometry_msgs.msg import PoseStamped
import pytest
from sensor_msgs.msg import JointState

from pointcloud_filters.state_history import stamp_ns, StateHistory


def _pose(seconds):
    message = PoseStamped()
    message.header.stamp.sec = seconds
    return message


def _joints(seconds, names=('boom', 'arm'), values=(0.3, 0.7)):
    message = JointState(name=list(names), position=list(values))
    message.header.stamp.sec = seconds
    return message


def test_nearest_measurement_not_latest_received():
    """A timer processing an older cloud uses its measured state."""
    history = StateHistory()
    first, middle, latest = [_pose(t) for t in (10, 11, 12)]
    for pose in (first, latest, middle):
        history.add_pose(pose)
    assert history.pose_at(10_100_000_000, .5) is first
    assert history.pose_at(11_800_000_000, .5) is latest
    with pytest.raises(ValueError, match='stamp difference'):
        history.pose_at(15_000_000_000, .5)


def test_partial_joints_have_independent_freshness():
    """A fresh arm update never makes an old boom angle appear fresh."""
    history = StateHistory()
    history.add_joints(_joints(10))
    history.add_joints(_joints(12, ('arm',), (1.2,)))
    assert history.joints_at(12_000_000_000, ('arm',), .1) == {'arm': 1.2}
    with pytest.raises(ValueError, match='joint boom/cloud stamp difference'):
        history.joints_at(12_000_000_000, ('boom', 'arm'), .1)
    with pytest.raises(ValueError, match='Waiting for joint missing'):
        history.joints_at(12_000_000_000, ('missing',), .1)


def test_bounded_history_and_bag_loop_drop_old_pass():
    """A backward clock jump cannot reuse joints left over from the previous pass."""
    history = StateHistory(duration_sec=2)
    for seconds in (10, 11, 12, 13):
        history.add_pose(_pose(seconds))
        history.add_joints(_joints(seconds))
    with pytest.raises(ValueError):
        history.pose_at(10_000_000_000, .1)
    history.add_pose(_pose(1))
    history.add_joints(_joints(1, ('arm',), (2.,)))
    assert history.pose_at(1_000_000_000, .1).header.stamp.sec == 1
    with pytest.raises(ValueError, match='Waiting for joint boom'):
        history.joints_at(1_000_000_000, ('boom',), .1)
    with pytest.raises(ValueError):
        history.pose_at(13_000_000_000, .1)


@pytest.mark.parametrize('names,positions', [
    (['a'], []), (['a', 'a'], [0., 1.]), (['a'], [float('nan')]),
])
def test_malformed_joint_messages_are_rejected(names, positions):
    """Missing angles, duplicate names and nonfinite values must not become defaults."""
    with pytest.raises(ValueError):
        StateHistory().add_joints(_joints(1, names, positions))


def test_integer_nanosecond_precision():
    """Preserve nanoseconds even at epoch-sized measurement timestamps."""
    pose = _pose(1792739277)
    pose.header.stamp.nanosec = 123456789
    assert stamp_ns(pose.header.stamp) == 1792739277123456789


def test_unstamped_pose_only_allowed_with_explicit_opt_out():
    """The crop-only path supports fixed centers; robot states require valid timing."""
    history = StateHistory()
    pose = _pose(0)
    history.add_pose(pose)
    with pytest.raises(ValueError):
        history.pose_at(10_000_000_000, .5)
    assert history.pose_at(10_000_000_000, .5, allow_unstamped=True) is pose


def test_unstamped_cloud_uses_latest_pose_only_in_crop_mode():
    """Maintain the explicit crop-only timestamp opt-out for an unstamped cloud."""
    history = StateHistory()
    history.add_pose(_pose(10))
    latest = _pose(11)
    history.add_pose(latest)
    assert history.pose_at(0, .5, allow_unstamped=True) is latest
    with pytest.raises(ValueError):
        history.pose_at(0, .5)
