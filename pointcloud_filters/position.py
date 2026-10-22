"""Resolve the backhoe position into a point cloud's coordinate frame."""

import numpy as np
from rclpy.time import Time
from scipy.spatial.transform import Rotation


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def position_in_cloud_frame(pose, cloud_header, tf_buffer, max_time_diff_sec=2.0):
    """Return XYZ in the cloud frame, rejecting missing frames and stale poses."""
    source = pose.header.frame_id
    target = cloud_header.frame_id
    if not source or not target:
        raise ValueError('Both global_pose and PointCloud2 must have a header.frame_id')
    pose_time = _stamp_seconds(pose.header.stamp)
    cloud_time = _stamp_seconds(cloud_header.stamp)
    if max_time_diff_sec > 0 and pose_time != 0 and cloud_time != 0:
        difference = abs(pose_time - cloud_time)
        if difference > max_time_diff_sec:
            raise ValueError(
                f'Pose/cloud stamp difference is {difference:.3f}s; '
                f'max_pose_time_diff_sec={max_time_diff_sec:.3f}s')
    position = pose.pose.position
    xyz = np.array([position.x, position.y, position.z])
    if not np.isfinite(xyz).all():
        raise ValueError('global_pose position must be finite')
    if source != target:
        transform = tf_buffer.lookup_transform(
            target, source, Time.from_msg(pose.header.stamp)).transform
        rotation = transform.rotation
        translation = transform.translation
        xyz = Rotation.from_quat([
            rotation.x, rotation.y, rotation.z, rotation.w]).apply(xyz)
        xyz += [translation.x, translation.y, translation.z]
        if not np.isfinite(xyz).all():
            raise ValueError('Transformed global_pose position must be finite')
    return xyz
