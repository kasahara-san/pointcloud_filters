"""Check mounting rotations, horizontal cropping and separate sensor clocks."""

from copy import deepcopy
from pathlib import Path

from geometry_msgs.msg import PoseStamped
import numpy as np
import pytest
from std_msgs.msg import Header

from pointcloud_filters.onboard import OnboardGeometry, synchronized_header
from pointcloud_filters.self_filter import SelfFilter
from pointcloud_filters.state_history import stamp_ns


def test_sideways_mount_keeps_horizontal_radius_independent_of_sensor_axes():
    """A tall return stays near the base; a distant forward return stays distant."""
    mount = OnboardGeometry('lidar', [1., 0., 2.], [-np.pi / 2, 0., -np.pi / 2], .5)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y = -19268., -138681.
    pose.pose.orientation.w = 1.
    # LiDAR X=base -Y, LiDAR Y=base -Z, LiDAR Z=base X.
    points = np.array([[0., -98., -1.], [0., 2., 10.], [0., 0., 0.]])
    offsets = mount.map_offsets(points, pose, 'map')
    np.testing.assert_allclose(offsets[:2], [[0., 0., 100.], [11., 0., 0.]], atol=1e-12)
    np.testing.assert_array_equal(mount.valid_returns(points), [True, True, False])
    np.testing.assert_allclose(mount.cloud_from_base @ mount.base_from_cloud, np.eye(4),
                               atol=1e-12)


def test_clock_offset_preserves_original_header_and_nanosecond_normalization():
    """Lookup uses the configured epoch while output retains the sensor header."""
    header = Header(frame_id='hesai_lidar')
    header.stamp.sec, header.stamp.nanosec = 364, 753422000
    original = deepcopy(header)
    corrected = synchronized_header(header, 1795148958.7430618)
    assert stamp_ns(corrected.stamp) == stamp_ns(header.stamp) + round(1795148958.7430618e9)
    assert 0 <= corrected.stamp.nanosec < 1_000_000_000
    assert header == original and corrected.frame_id == header.frame_id


def test_unexpected_frames_and_invalid_pose_do_not_pass_silently():
    """A mounting calibration only applies to its documented sensor and pose frames."""
    mount = OnboardGeometry('hesai_lidar', [0., 0., 0.], [0., 0., 0.], .5)
    with pytest.raises(ValueError, match='sensor_frame'):
        mount.check_frame(Header(frame_id='other_lidar'))
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    with pytest.raises(ValueError, match='must be in map'):
        mount.map_offsets(np.zeros((1, 3)), pose, 'map')
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = float('nan')
    with pytest.raises(ValueError, match='orientation'):
        mount.map_offsets(np.zeros((1, 3)), pose, 'map')


def test_bundled_d37pxi_mimic_joints_follow_measured_blade_angles():
    """Load every collision and ensure blade geometry actually moves with lift/tilt."""
    path = Path(__file__).resolve().parents[1] / 'robot/d37pxi/d37pxi.urdf'
    model = SelfFilter(path)
    assert set(model.required_joints) == {'lift_joint', 'tilt_joint', 'angle_joint'}
    assert len(model.collisions) == 7
    flat = model.link_transforms(dict(lift_joint=0., tilt_joint=0., angle_joint=0.))
    raised = model.link_transforms(dict(lift_joint=-.2, tilt_joint=.1, angle_joint=.1))
    assert raised['blade_link'][2, 3] > flat['blade_link'][2, 3] + .2
    assert not np.allclose(flat['blade_arm_piston_link'], raised['blade_arm_piston_link'])
