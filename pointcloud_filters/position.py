"""Resolve the backhoe position into a point cloud's coordinate frame."""

import numpy as np
from rclpy.time import Time
from scipy.spatial.transform import Rotation


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def model_origin_matrix(xyz, rpy):
    """Describe the CAD root relative to the measured base, in metres/radians."""
    xyz, rpy = np.asarray(xyz, dtype=float), np.asarray(rpy, dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not (
            np.isfinite(xyz).all() and np.isfinite(rpy).all()):
        raise ValueError('model_origin_xyz and model_origin_rpy need three finite values')
    matrix = np.eye(4)
    matrix[:3, 3] = xyz
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    return matrix


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


def pose_matrix_in_cloud_frame(pose, cloud_header, tf_buffer,
                               max_time_diff_sec=0.5, pose_frame='world'):
    """
    Compose cloud<-world and world<-base without publishing robot TF.

    The recorded GNSS poses have empty frame IDs. Only that missing field uses
    pose_frame; an explicitly supplied frame always takes precedence.
    """
    from copy import deepcopy

    pose = deepcopy(pose)
    if not pose.header.frame_id:
        pose.header.frame_id = pose_frame
    xyz = position_in_cloud_frame(pose, cloud_header, tf_buffer, max_time_diff_sec)
    quaternion = pose.pose.orientation
    rotation = Rotation.from_quat([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    if pose.header.frame_id != cloud_header.frame_id:
        transform = tf_buffer.lookup_transform(
            cloud_header.frame_id, pose.header.frame_id,
            Time.from_msg(pose.header.stamp)).transform
        q = transform.rotation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]) * rotation
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = xyz
    return matrix
