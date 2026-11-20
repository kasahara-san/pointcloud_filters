"""Build rigid mounting and CAD-origin transforms for the D37PXi."""

import numpy as np
from scipy.spatial.transform import Rotation


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
