"""Geometry and explicit clock conversion for a machine-mounted LiDAR."""

from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from pointcloud_filters.position import model_origin_matrix
from pointcloud_filters.state_history import stamp_ns


def synchronized_header(header, offset_sec):
    """Convert the LiDAR clock for state lookup without changing the input header."""
    if not math.isfinite(offset_sec):
        raise ValueError('cloud_time_offset_sec must be finite')
    result = deepcopy(header)
    timestamp = stamp_ns(header.stamp) + round(offset_sec * 1e9)
    if timestamp < 0:
        raise ValueError('Corrected cloud timestamp must be nonnegative')
    result.stamp.sec, result.stamp.nanosec = divmod(timestamp, 1_000_000_000)
    return result


class OnboardGeometry:
    """Keep local calculations precise even with large map coordinates."""

    def __init__(self, sensor_frame, xyz, rpy, min_range):
        """Configure base_link<-LiDAR in metres and fixed XYZ Euler radians."""
        if not sensor_frame:
            raise ValueError('sensor_frame is required in onboard mode')
        if not math.isfinite(min_range) or min_range < 0:
            raise ValueError('min_sensor_range_m must be finite and >= 0')
        self.sensor_frame = sensor_frame
        self.base_from_cloud = model_origin_matrix(xyz, rpy)
        self.cloud_from_base = np.linalg.inv(self.base_from_cloud)
        self.min_range = min_range

    def check_frame(self, header):
        """Reject other frames instead of silently applying the wrong mounting pose."""
        if header.frame_id != self.sensor_frame:
            raise ValueError(f'Expected sensor_frame={self.sensor_frame}, '
                             f'got {header.frame_id}')

    def map_offsets(self, points, pose, pose_frame):
        """Return map-axis offsets from measured base, without subtracting huge XYZ."""
        if (pose.header.frame_id or pose_frame) != pose_frame:
            raise ValueError(f'Onboard global_pose must be in {pose_frame}')
        position = pose.pose.position
        if not np.isfinite([position.x, position.y, position.z]).all():
            raise ValueError('global_pose position must be finite')
        q = pose.pose.orientation
        if not np.isfinite([q.x, q.y, q.z, q.w]).all():
            raise ValueError('global_pose orientation must be finite')
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        base_points = (points @ self.base_from_cloud[:3, :3].T
                       + self.base_from_cloud[:3, 3])
        return base_points @ rotation.T

    def valid_returns(self, points):
        """Discard zero placeholders and near-sensor returns before spatial filters."""
        return (np.isfinite(points).all(axis=1)
                & (np.linalg.norm(points, axis=1) >= self.min_range))
