"""Exercise machine-centered XY cropping, outliers and input edge cases."""

import numpy as np
import pytest

from pointcloud_filters.filtering import FilterParameters, filter_indices


def _grid(side, offset=0.0):
    x, y = np.meshgrid(np.arange(side) * 0.05 + offset, np.arange(side) * 0.05)
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(side * side)))


def test_xy_disk_includes_boundary_and_ignores_height():
    """Crop a circular cylinder, with no Z cutoff or square bounding-box shortcut."""
    points = np.array([
        [10, 20, 0], [15, 20, 100], [10, 15, -100], [13, 24, 10],
        [15.001, 20, 0], [15, 25, 0], [np.nan, 20, 0], [10, np.inf, 0]])
    original = points.copy()
    indices, stats = filter_indices(points, [10, 20], FilterParameters(
        radius_m=5.0, voxel_size=0.0, sor_stddev_mul=1e6))
    np.testing.assert_equal(indices, [0, 1, 2, 3])
    assert stats['radius'] == 4 and stats['finite'] == 6
    np.testing.assert_equal(points, original)


def test_crop_follows_machine_and_keeps_separate_regions_inside_radius():
    """Moving the center changes the selection; a large radius retains both groups."""
    first, second = _grid(15), _grid(15, offset=10.0)
    points = np.vstack((first, second))
    parameters = FilterParameters(radius_m=1.0, voxel_size=0.0, sor_mean_k=8)
    left, _ = filter_indices(points, [0.35, 0.35], parameters)
    right, _ = filter_indices(points, [10.35, 0.35], parameters)
    assert len(left) > 100 and np.all(left < len(first))
    assert len(right) > 100 and np.all(right >= len(first))
    both, _ = filter_indices(points, [5.35, 0.35], FilterParameters(
        radius_m=6.0, voxel_size=0.0, sor_mean_k=8))
    assert np.count_nonzero(both < len(first)) > 100
    assert np.count_nonzero(both >= len(first)) > 100


def test_statistical_filter_removes_outlier_inside_crop():
    """Passing the radius criterion must not bypass statistical outlier removal."""
    cluster = _grid(15)
    points = np.vstack((cluster, [[0.9, 0.35, 0.0]]))
    indices, stats = filter_indices(points, [0.35, 0.35], FilterParameters(
        radius_m=2.0, voxel_size=0.0, sor_mean_k=8))
    assert stats['radius'] == len(points)
    assert len(cluster) not in indices
    assert stats['sor'] < stats['radius']
    assert len(indices) > 100


def test_self_removal_precedes_voxel_and_statistical_filter():
    """A robot representative cannot displace nearby terrain in the same voxel."""
    points = np.array([[.01, .01, .01], [.02, .02, .02], [1., 0., 0.]])
    indices, counts = filter_indices(
        points, [0, 0], FilterParameters(), self_mask=np.array([True, False, False]))
    np.testing.assert_array_equal(indices, [1, 2])
    assert counts['radius'] == 3
    assert counts['self_removed'] == 1
    assert counts['self_filter'] == 2


@pytest.mark.parametrize('points', [
    np.empty((0, 3)),
    np.array([[np.nan, 0.0, 0.0], [0.0, np.inf, 0.0]]),
    np.array([[100.0, 100.0, 0.0], [-100.0, -100.0, 0.0]]),
])
def test_empty_invalid_and_outside_inputs_return_empty(points):
    """No points survive when the crop has no finite input points."""
    indices, stats = filter_indices(points, [0.0, 0.0], FilterParameters())
    assert indices.dtype == np.intp
    assert len(indices) == stats['output'] == 0


@pytest.mark.parametrize('count', [1, 2])
def test_small_input_has_no_cluster_size_cutoff(count):
    """Keep small valid inputs when there are too few points to apply SOR."""
    points = np.column_stack((np.arange(count), np.zeros(count), np.zeros(count)))
    indices, _ = filter_indices(points, [0, 0], FilterParameters(voxel_size=0.0))
    np.testing.assert_equal(indices, np.arange(count))


def test_voxel_preserves_original_representative():
    """Select an original point from each occupied voxel after cropping."""
    points = np.repeat(_grid(10), 5, axis=0)
    indices, stats = filter_indices(points, [0, 0], FilterParameters(voxel_size=0.025))
    assert stats['radius'] == len(points) and stats['voxel'] == 100
    assert len(indices) > 20
    assert np.all(indices % 5 == 0)


@pytest.mark.parametrize('overrides', [
    {'radius_m': 0.0}, {'radius_m': -1.0}, {'radius_m': float('nan')},
    {'radius_m': float('inf')}, {'voxel_size': -0.1},
    {'sor_stddev_mul': float('inf')}, {'sor_mean_k': 0},
    {'sor_mean_k': 1.5}, {'sor_mean_k': True},
])
def test_invalid_parameters_are_rejected(overrides):
    """Invalid settings must fail during startup instead of a timer callback."""
    with pytest.raises(ValueError):
        FilterParameters(**overrides)


@pytest.mark.parametrize('center', [[0], [0, 0, 0], [np.nan, 0], [0, np.inf]])
def test_invalid_center_is_rejected(center):
    """A missing or invalid position must never silently become the origin."""
    with pytest.raises(ValueError, match='center_xy'):
        filter_indices(np.zeros((1, 3)), center, FilterParameters())
