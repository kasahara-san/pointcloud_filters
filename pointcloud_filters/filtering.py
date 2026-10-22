"""Crop a point cloud in XY around the machine and remove statistical outliers."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class FilterParameters:
    """Spatial distances are in metres; neighbor counts exclude the query point."""

    radius_m: float = 10.0
    voxel_size: float = 0.1
    sor_mean_k: int = 20
    sor_stddev_mul: float = 2.0

    def __post_init__(self):
        """Reject invalid configuration before processing any messages."""
        for name in ('voxel_size', 'sor_stddev_mul'):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and >= 0')
        if not math.isfinite(self.radius_m) or self.radius_m <= 0:
            raise ValueError('radius_m must be finite and > 0')
        if (isinstance(self.sor_mean_k, bool) or not isinstance(self.sor_mean_k, int)
                or self.sor_mean_k < 1):
            raise ValueError('sor_mean_k must be an integer >= 1')


def filter_indices(points, center_xy, parameters):
    """Return retained input indices and stage counts without changing input data."""
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points must have shape (N, 3)')
    center_xy = np.asarray(center_xy, dtype=np.float64)
    if center_xy.shape != (2,) or not np.isfinite(center_xy).all():
        raise ValueError('center_xy must contain two finite coordinates')

    indices = np.flatnonzero(np.isfinite(points).all(axis=1))
    distances_xy = np.hypot(points[indices, 0] - center_xy[0],
                            points[indices, 1] - center_xy[1])
    stats = dict(input=len(points), finite=len(indices), radius=0,
                 voxel=0, sor=0, output=0)
    indices = indices[distances_xy <= parameters.radius_m]
    stats['radius'] = len(indices)
    if parameters.voxel_size > 0 and len(indices):
        keys = np.floor(points[indices] / parameters.voxel_size)
        _, first = np.unique(keys, axis=0, return_index=True)
        # Keep an original point (including its attributes) from each voxel.
        indices = indices[np.sort(first)]
    stats['voxel'] = len(indices)
    if len(indices) >= 3:
        candidates = points[indices]
        k = min(parameters.sor_mean_k, len(indices) - 1)
        distances, _ = cKDTree(candidates).query(candidates, k=k + 1)
        mean_distances = distances[:, 1:].mean(axis=1)
        threshold = (mean_distances.mean()
                     + parameters.sor_stddev_mul * mean_distances.std())
        indices = indices[mean_distances <= threshold]
    stats['sor'] = len(indices)

    indices = np.sort(indices)
    stats['output'] = len(indices)
    return indices, stats
