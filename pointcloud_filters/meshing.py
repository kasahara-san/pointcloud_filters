"""Reconstruct triangle surfaces from a filtered point cloud."""

from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial import Delaunay
from shape_msgs.msg import Mesh, MeshTriangle


MESH_VOXEL_SIZES = (0.8, 0.6, 0.4, 0.2, 0.1)


def mesh_voxel_size(resolution):
    """Map an integer level from 1 (coarse) to 5 (fine) to metres."""
    if (isinstance(resolution, bool) or not isinstance(resolution, int)
            or not 1 <= resolution <= len(MESH_VOXEL_SIZES)):
        raise ValueError('mesh_resolution must be an integer from 1 to 5')
    return MESH_VOXEL_SIZES[resolution - 1]


class MeshConverter:
    """Use voxel sampling and surface triangulation for planar and 3D surfaces."""

    def __init__(self, resolution=3):
        """Load Open3D only when mesh output is requested."""
        self.voxel_size = mesh_voxel_size(resolution)
        try:
            import open3d
        except ImportError as error:
            raise RuntimeError(
                'Mesh output requires Open3D. Install dependencies with rosdep '
                'or install python3-open3d.') from error
        self._o3d = open3d

    def convert(self, points, excluded=None, exclusion_step=0.075):
        """Return Mesh and stage counts; degenerate inputs yield an empty Mesh."""
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('points must have shape (N, 3)')
        points = points[np.isfinite(points).all(axis=1)]
        stats = dict(mesh_sampled=0, vertices=0, triangles=0)
        if len(points) < 3:
            return Mesh(), stats

        o3d = self._o3d
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        cloud = cloud.voxel_down_sample(self.voxel_size)
        sampled = np.asarray(cloud.points)
        if excluded is not None and len(sampled):
            if not np.isfinite(exclusion_step) or exclusion_step <= 0:
                raise ValueError('exclusion_step must be finite and > 0')
            cloud = cloud.select_by_index(np.flatnonzero(~excluded(sampled)))
            sampled = np.asarray(cloud.points)
        stats['mesh_sampled'] = len(sampled)
        if len(sampled) < 3:
            return Mesh(), stats
        _, singular_values, axes = np.linalg.svd(
            sampled - sampled.mean(axis=0), full_matrices=False)
        if singular_values[1] <= singular_values[0] * 1e-10:
            return Mesh(), stats

        spacing = max(self.voxel_size, float(np.median(
            cloud.compute_nearest_neighbor_distance())))
        if singular_values[2] <= singular_values[0] * 1e-6:
            # Project onto the fitted plane, including vertical planes. Avoid
            # ball-pivoting ambiguities on coplanar regular grids.
            projected = (sampled - sampled.mean(axis=0)) @ axes[:2].T
            surface = o3d.geometry.TriangleMesh(
                cloud.points, o3d.utility.Vector3iVector(Delaunay(projected).simplices))
        else:
            cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
                radius=3.0 * spacing, max_nn=30))
            if len(sampled) < 5:
                cloud.orient_normals_to_align_with_direction(axes[2])
            else:
                cloud.orient_normals_consistent_tangent_plane(
                    min(30, len(sampled) - 1))
            surface = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                cloud, o3d.utility.DoubleVector([1.2 * spacing, 2.0 * spacing]))
        surface.remove_duplicated_vertices()
        surface.remove_duplicated_triangles()
        surface.remove_degenerate_triangles()
        if not len(surface.triangles):
            return Mesh(), stats

        corners = np.asarray(surface.vertices)[np.asarray(surface.triangles)]
        edges = np.linalg.norm(corners - np.roll(corners, 1, axis=1), axis=2)
        areas_twice = np.linalg.norm(np.cross(
            corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]), axis=1)
        surface.remove_triangles_by_mask(
            np.any(edges > 3.0 * spacing, axis=1)
            | (areas_twice <= np.finfo(float).eps * spacing * spacing))
        if excluded is not None and len(surface.triangles):
            corners = np.asarray(surface.vertices)[np.asarray(surface.triangles)]
            surface.remove_triangles_by_mask(
                _excluded_triangles(corners, excluded, exclusion_step))
        if not len(surface.triangles):
            return Mesh(), stats

        # Preserve separate surfaces inside the crop instead of selecting a cluster.
        surface.remove_unreferenced_vertices()
        surface.orient_triangles()
        result = Mesh()
        result.vertices = [Point(x=float(x), y=float(y), z=float(z))
                           for x, y, z in np.asarray(surface.vertices)]
        result.triangles = [MeshTriangle(vertex_indices=triangle.astype(np.uint32))
                            for triangle in np.asarray(surface.triangles)]
        stats.update(vertices=len(result.vertices), triangles=len(result.triangles))
        return result, stats


def _excluded_triangles(corners, excluded, step):
    """Reject faces spanning the robot, sampling edges and interiors at <=step."""
    lengths = np.linalg.norm(corners - np.roll(corners, 1, axis=1), axis=2).max(axis=1)
    divisions = np.maximum(1, np.ceil(lengths / step).astype(int))
    removed = np.zeros(len(corners), dtype=bool)
    for count in np.unique(divisions):
        indices = np.flatnonzero(divisions == count)
        weights = np.array([(i / count, j / count, 1 - (i + j) / count)
                            for i in range(count + 1) for j in range(count + 1 - i)])
        # Odd divisions miss edge midpoints, and divisions not divisible by
        # three miss the centroid. Always inspect these independent probes.
        weights = np.vstack((weights, [
            [.5, .5, 0.], [.5, 0., .5], [0., .5, .5], [1. / 3, 1. / 3, 1. / 3]]))
        # Bound temporary arrays even for large, high-resolution surfaces.
        batch = max(1, 100_000 // len(weights))
        for start in range(0, len(indices), batch):
            selected = indices[start:start + batch]
            queries = np.einsum('ij,kjl->kil', weights, corners[selected])
            hits = excluded(queries.reshape(-1, 3))
            removed[selected] = np.asarray(hits).reshape(len(selected), -1).any(axis=1)
    return removed
