"""Check rigid mounting and CAD-origin transforms used by the D37PXi."""

import math

import numpy as np
import pytest

from pointcloud_filters.position import model_origin_matrix


def test_origin_translation_and_fixed_xyz_rotation():
    """Apply translation in parent axes and rotate local points about the origin."""
    matrix = model_origin_matrix([1., 2., 3.], [0., 0., math.pi / 2])
    np.testing.assert_allclose(matrix @ [1., 0., 0., 1.], [1., 3., 3., 1.], atol=1e-12)
    np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-12)


@pytest.mark.parametrize('xyz,rpy', [
    ([0., 0.], [0., 0., 0.]),
    ([0., 0., 0.], [0., 0.]),
    ([float('nan'), 0., 0.], [0., 0., 0.]),
    ([0., 0., 0.], [float('inf'), 0., 0.]),
])
def test_invalid_mounting_values_are_rejected(xyz, rpy):
    """Reject incomplete and nonfinite mounting values before filtering starts."""
    with pytest.raises(ValueError, match='three finite values'):
        model_origin_matrix(xyz, rpy)
