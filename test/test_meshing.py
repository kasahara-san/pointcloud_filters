"""Check resolution levels and genuine 3D Mesh reconstruction."""

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from shape_msgs.msg import Mesh

from pointcloud_filters.meshing import MeshConverter, mesh_voxel_size


def _assert_valid_mesh(message):
    vertices = np.array([[p.x, p.y, p.z] for p in message.vertices])
    faces = np.array([t.vertex_indices for t in message.triangles])
    assert len(vertices) >= 3 and len(faces) >= 1
    assert np.isfinite(vertices).all()
    assert faces.min() >= 0 and faces.max() < len(vertices)
    assert len(np.unique(faces)) == len(vertices)
    corners = vertices[faces]
    assert np.all(np.linalg.norm(np.cross(
        corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]), axis=1) > 0)
    return vertices, faces


def test_five_resolutions_reconstruct_a_vertical_plane_and_serialize():
    """Finer settings retain more detail without assuming a height map."""
    y, z = np.meshgrid(np.arange(41) * 0.2, np.arange(41) * 0.2)
    points = np.column_stack((np.full(y.size, 4.2), y.ravel(), z.ravel()))
    triangle_counts = []
    for level in range(1, 6):
        message, stats = MeshConverter(level).convert(points)
        vertices, faces = _assert_valid_mesh(message)
        np.testing.assert_allclose(vertices[:, 0], 4.2)
        assert vertices[:, 1:].min() >= 0 and vertices[:, 1:].max() <= 8.0
        assert stats['vertices'] == len(vertices)
        assert stats['triangles'] == len(faces)
        triangle_counts.append(len(faces))
        restored = deserialize_message(serialize_message(message), Mesh)
        assert restored == message
    assert triangle_counts == sorted(triangle_counts)
    assert triangle_counts[-1] > 3 * triangle_counts[0]


def test_curved_surface_reconstructs_in_three_dimensions():
    """Reconstruct a sphere using oriented estimated surface normals."""
    count = 500
    z = 1 - 2 * (np.arange(count) + 0.5) / count
    angle = np.arange(count) * np.pi * (3 - np.sqrt(5))
    radius = np.sqrt(1 - z * z)
    points = np.column_stack((radius * np.cos(angle), radius * np.sin(angle), z))
    message, _ = MeshConverter(5).convert(points)
    vertices, faces = _assert_valid_mesh(message)
    assert len(faces) > 100
    assert np.all(np.ptp(vertices, axis=0) > 1.8)


def test_disconnected_mesh_patches_are_preserved():
    """Keep all valid surfaces instead of choosing just the largest cluster."""
    x, y = np.meshgrid(np.arange(11) * 0.2, np.arange(11) * 0.2)
    large = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    x, y = np.meshgrid(np.arange(5) * 0.2 + 5, np.arange(5) * 0.2)
    small = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    message, _ = MeshConverter(5).convert(np.vstack((large, small)))
    vertices, faces = _assert_valid_mesh(message)
    assert vertices[:, 0].min() == 0.0 and vertices[:, 0].max() >= 5.0
    assert np.count_nonzero(vertices[:, 0] < 3.0) > 50
    assert np.count_nonzero(vertices[:, 0] > 4.0) > 10
    assert np.all(np.ptp(vertices[faces, 0], axis=1) < 1.0)


@pytest.mark.parametrize('points', [
    np.empty((0, 3)), np.zeros((10, 3)), np.eye(3)[:2],
    np.column_stack((np.arange(10), np.zeros(10), np.zeros(10))),
    np.array([[np.nan, 0, 0], [0, np.inf, 0]]),
])
def test_unreconstructable_inputs_produce_empty_mesh(points):
    """Empty, repeated, collinear and nonfinite points cannot define a surface."""
    message, stats = MeshConverter(3).convert(points)
    assert message == Mesh()
    assert stats['triangles'] == stats['vertices'] == 0


@pytest.mark.parametrize('resolution', [0, 6, -1, 2.5, True, '3'])
def test_invalid_resolution_rejected(resolution):
    """Reject settings outside the five integer levels before loading Open3D."""
    with pytest.raises(ValueError, match='mesh_resolution'):
        mesh_voxel_size(resolution)
