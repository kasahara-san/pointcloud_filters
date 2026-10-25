"""Check pose timestamps and coordinate transforms using a real TF buffer."""

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import pytest
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException

from pointcloud_filters.position import (
    model_origin_matrix, pose_matrix_in_cloud_frame, position_in_cloud_frame)


def _pose(frame='map', seconds=100):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp.sec = seconds
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 2.0, 3.0, 4.0
    return pose


def _header(frame='map', seconds=100):
    header = Header(frame_id=frame)
    header.stamp.sec = seconds
    return header


def test_same_frame_needs_no_tf_or_pose_orientation():
    """Position alone determines the crop center; the machine's heading is unused."""
    np.testing.assert_equal(position_in_cloud_frame(_pose(), _header(), None), [2, 3, 4])


def test_transform_rotation_and_translation():
    """Convert the pose position into the point cloud's map frame."""
    buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'world'
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 20.0
    transform.transform.translation.z = 30.0
    transform.transform.rotation.z = math.sqrt(0.5)
    transform.transform.rotation.w = math.sqrt(0.5)
    buffer.set_transform_static(transform, 'test')
    pose = _pose('world')
    original = deepcopy(pose)
    np.testing.assert_allclose(position_in_cloud_frame(pose, _header(), buffer), [7, 22, 34])
    assert pose == original


def test_dynamic_tf_uses_pose_timestamp():
    """Interpolate TF at measurement time rather than applying the latest transform."""
    buffer = Buffer()
    for seconds, offset in [(100, 0.0), (110, 10.0)]:
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'world'
        transform.header.stamp.sec = seconds
        transform.transform.translation.x = offset
        transform.transform.rotation.w = 1.0
        buffer.set_transform(transform, 'test')
    np.testing.assert_allclose(position_in_cloud_frame(
        _pose('world', 105), _header(seconds=105), buffer), [7, 3, 4])


def test_missing_transform_is_rejected():
    """Different coordinate frames require an actual TF relationship."""
    with pytest.raises(TransformException):
        position_in_cloud_frame(_pose('world'), _header(), Buffer())


@pytest.mark.parametrize('pose_frame,cloud_frame', [('', 'map'), ('map', '')])
def test_missing_frame_is_rejected(pose_frame, cloud_frame):
    """Unknown frames must not be treated as equivalent coordinates."""
    with pytest.raises(ValueError, match='frame_id'):
        position_in_cloud_frame(_pose(pose_frame), _header(cloud_frame), None)


@pytest.mark.parametrize('seconds', [97, 103])
def test_old_or_future_pose_is_rejected(seconds):
    """Apply the same timestamp tolerance to past and future position samples."""
    with pytest.raises(ValueError, match='stamp difference'):
        position_in_cloud_frame(_pose(seconds=seconds), _header(), None)


def test_time_check_can_be_disabled_and_zero_stamp_supports_static_testing():
    """Allow explicit time-check opt-out and conventionally unstamped poses."""
    np.testing.assert_equal(position_in_cloud_frame(
        _pose(seconds=1), _header(), None, max_time_diff_sec=0), [2, 3, 4])
    np.testing.assert_equal(position_in_cloud_frame(
        _pose(seconds=0), _header(), None), [2, 3, 4])


def test_nonfinite_position_is_rejected():
    """Do not send invalid coordinates to TF or spatial filtering."""
    pose = _pose()
    pose.pose.position.x = float('nan')
    with pytest.raises(ValueError, match='finite'):
        position_in_cloud_frame(pose, _header(), None)


def test_world_map_inverse_and_pose_orientation_are_composed():
    """Use the inverse world->map TF with the full measured base pose."""
    buffer = Buffer()
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'world', 'map'
    transform.transform.translation.x = 10000.0
    transform.transform.translation.y = 20000.0
    transform.transform.translation.z = 28.558
    transform.transform.rotation.z = math.sqrt(.5)
    transform.transform.rotation.w = math.sqrt(.5)
    buffer.set_transform_static(transform, 'test')
    pose = _pose(frame='')
    pose.pose.position.x = 9997.0
    pose.pose.position.y = 20002.0
    pose.pose.position.z = 32.496
    pose.pose.orientation.z = 1.0
    pose.pose.orientation.w = 0.0
    matrix = pose_matrix_in_cloud_frame(pose, _header(), buffer)
    np.testing.assert_allclose(matrix[:3, 3], [2., 3., 3.938], atol=1e-9)
    np.testing.assert_allclose(
        matrix[:3, :3], [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]], atol=1e-9)
    assert pose.header.frame_id == ''


def test_invalid_orientation_rejected_for_robot_geometry():
    """A zero quaternion cannot define the orientation of the articulated model."""
    pose = _pose()
    pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError):
        pose_matrix_in_cloud_frame(pose, _header(), None)


def test_explicit_cad_origin_uses_base_axes_and_preserves_base_position():
    """A calibrated model offset rotates with the measured base, not world axes."""
    pose = _pose()
    pose.pose.orientation.z, pose.pose.orientation.w = math.sqrt(.5), math.sqrt(.5)
    base = pose_matrix_in_cloud_frame(pose, _header(), None)
    model = base @ model_origin_matrix([0., 1., -2.], [0., 0., math.pi / 2])
    np.testing.assert_allclose(base[:3, 3], [2., 3., 4.])
    np.testing.assert_allclose(model[:3, 3], [1., 3., 2.])
    np.testing.assert_allclose(model[:3, :3], np.diag([-1., -1., 1.]), atol=1e-10)
