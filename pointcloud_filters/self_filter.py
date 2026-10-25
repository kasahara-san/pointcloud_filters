"""Remove articulated robot surfaces using URDF kinematics and collision meshes."""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial.transform import Rotation


def _numbers(text, count):
    values = np.fromstring(text, sep=' ')
    if values.shape != (count,) or not np.isfinite(values).all():
        raise ValueError(f'Expected {count} finite numbers: {text}')
    return values


def _origin(element):
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = _numbers(element.get('xyz', '0 0 0'), 3)
        matrix[:3, :3] = Rotation.from_euler(
            'xyz', _numbers(element.get('rpy', '0 0 0'), 3)).as_matrix()
    return matrix


def _apply(matrix, points):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def _collada_mesh(path):
    """Read COLLADA triangle meshes including scene nodes, unit and up axis."""
    root = ET.parse(path).getroot()
    namespace = {'c': root.tag.split('}')[0].lstrip('{')}
    geometries = {}
    for geometry in root.findall('c:library_geometries/c:geometry', namespace):
        mesh = geometry.find('c:mesh', namespace)
        if mesh is None:
            continue
        sources = {}
        for source in mesh.findall('c:source', namespace):
            array = source.find('c:float_array', namespace)
            accessor = source.find('c:technique_common/c:accessor', namespace)
            if array is not None and accessor is not None:
                stride = int(accessor.get('stride', '1'))
                offset = int(accessor.get('offset', '0'))
                count = int(accessor.get('count'))
                values = np.fromstring(array.text or '', sep=' ')
                sources[source.get('id')] = values[
                    offset:offset + count * stride].reshape(count, stride)
        vertex_sources = {
            vertex.get('id'): next(
                item.get('source')[1:] for item in vertex.findall('c:input', namespace)
                if item.get('semantic') == 'POSITION')
            for vertex in mesh.findall('c:vertices', namespace)}
        vertices, triangles = [], []
        for primitive in mesh:
            kind = primitive.tag.rsplit('}', 1)[-1]
            if kind not in ('triangles', 'polylist'):
                if kind in ('polygons', 'trifans', 'tristrips'):
                    raise ValueError(f'Unsupported COLLADA primitive {kind}: {path}')
                continue
            inputs = primitive.findall('c:input', namespace)
            vertex_input = next(item for item in inputs if item.get('semantic') == 'VERTEX')
            stride = 1 + max(int(item.get('offset', '0')) for item in inputs)
            raw = np.fromstring(
                primitive.find('c:p', namespace).text or '', sep=' ', dtype=np.int64)
            indices = raw.reshape(-1, stride)[:, int(vertex_input.get('offset', '0'))]
            positions = sources[vertex_sources[vertex_input.get('source')[1:]]][:, :3]
            if kind == 'triangles':
                faces = indices.reshape(-1, 3)
            else:
                counts = np.fromstring(
                    primitive.find('c:vcount', namespace).text, sep=' ', dtype=int)
                faces, start = [], 0
                for count in counts:
                    polygon = indices[start:start + count]
                    faces.extend([polygon[0], polygon[i], polygon[i + 1]]
                                 for i in range(1, count - 1))
                    start += count
                faces = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
            triangles.append(faces + sum(len(item) for item in vertices))
            vertices.append(positions)
        if vertices:
            geometries[geometry.get('id')] = (
                np.concatenate(vertices), np.concatenate(triangles))
    visual_scenes = {scene.get('id'): scene for scene in root.findall(
        'c:library_visual_scenes/c:visual_scene', namespace)}
    instance = root.find('c:scene/c:instance_visual_scene', namespace)
    if instance is None or instance.get('url', '')[1:] not in visual_scenes:
        raise ValueError(f'COLLADA has no valid visual scene: {path}')
    all_vertices, all_triangles = [], []

    def visit(node, parent):
        local = np.eye(4)
        for item in node:
            kind = item.tag.rsplit('}', 1)[-1]
            if kind == 'matrix':
                local = local @ _numbers(item.text, 16).reshape(4, 4)
            elif kind in ('translate', 'scale', 'rotate'):
                transform = np.eye(4)
                values = _numbers(item.text, 4 if kind == 'rotate' else 3)
                if kind == 'translate':
                    transform[:3, 3] = values
                elif kind == 'scale':
                    transform[:3, :3] = np.diag(values)
                else:
                    axis = values[:3] / np.linalg.norm(values[:3])
                    transform[:3, :3] = Rotation.from_rotvec(
                        axis * np.deg2rad(values[3])).as_matrix()
                local = local @ transform
        combined = parent @ local
        for item in node.findall('c:instance_geometry', namespace):
            vertices, triangles = geometries[item.get('url')[1:]]
            all_triangles.append(triangles + sum(len(v) for v in all_vertices))
            all_vertices.append(_apply(combined, vertices))
        for child in node.findall('c:node', namespace):
            visit(child, combined)

    scene = visual_scenes[instance.get('url')[1:]]
    for node in scene.findall('c:node', namespace):
        visit(node, np.eye(4))
    if not all_vertices:
        raise ValueError(f'COLLADA scene has no triangles: {path}')
    vertices = np.concatenate(all_vertices)
    unit = root.find('c:asset/c:unit', namespace)
    vertices *= float(unit.get('meter', '1')) if unit is not None else 1.0
    axis = root.find('c:asset/c:up_axis', namespace)
    up = axis.text.strip() if axis is not None else 'Y_UP'
    if up == 'Y_UP':
        vertices = vertices @ Rotation.from_euler('x', 90, degrees=True).as_matrix().T
    elif up == 'X_UP':
        vertices = vertices @ Rotation.from_euler('y', -90, degrees=True).as_matrix().T
    elif up != 'Z_UP':
        raise ValueError(f'Unsupported COLLADA up_axis: {up}')
    return vertices, np.concatenate(all_triangles)


def _clean_mesh(vertices, triangles):
    """Weld coincident CAD vertices and remove duplicate/degenerate triangles."""
    if not np.isfinite(vertices).all():
        raise ValueError('Collision mesh has nonfinite vertices')
    _, first, inverse = np.unique(
        np.round(vertices, 7), axis=0, return_index=True, return_inverse=True)
    vertices, triangles = vertices[first], inverse[triangles]
    _, first = np.unique(np.sort(triangles, axis=1), axis=0, return_index=True)
    triangles = triangles[np.sort(first)]
    xyz = vertices[triangles]
    areas = np.linalg.norm(np.cross(xyz[:, 1] - xyz[:, 0], xyz[:, 2] - xyz[:, 0]), axis=1)
    triangles = triangles[areas > 1e-12]
    if not len(triangles):
        raise ValueError('Collision mesh has no nondegenerate triangles')
    return vertices, triangles


def _closed_components(vertices, triangles):
    """Return closed components; open CAD surfaces have no defined interior."""
    edges = np.concatenate((triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]))
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices)))
    _, labels = connected_components(graph, directed=False)
    triangle_labels = labels[triangles[:, 0]]
    for label in np.unique(triangle_labels):
        component = triangles[triangle_labels == label]
        edges = np.sort(np.concatenate((component[:, [0, 1]], component[:, [1, 2]],
                                        component[:, [2, 0]])), axis=1)
        _, counts = np.unique(edges, axis=0, return_counts=True)
        if len(component) >= 4 and np.all(counts == 2):
            yield component


@dataclass
class _Joint:
    """One URDF joint's fixed origin and variable motion."""

    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    mimic: object


class _Collision:
    """Cache a triangle surface and independently closed components in local space."""

    def __init__(self, o3d, link, origin, vertices, triangles):
        """Build immutable acceleration structures once for each collision."""
        self.link, self.origin = link, origin
        self.vertices, self.triangles = _clean_mesh(vertices, triangles)
        self.lower, self.upper = self.vertices.min(0), self.vertices.max(0)
        self._o3d = o3d
        self.scene = self._scene(self.triangles)
        self.volumes = []
        for component in _closed_components(self.vertices, self.triangles):
            points = self.vertices[np.unique(component)]
            self.volumes.append((self._scene(component), points.min(0), points.max(0)))

    def _scene(self, triangles):
        o3d = self._o3d
        scene = o3d.t.geometry.RaycastingScene(nthreads=2)
        scene.add_triangles(
            o3d.core.Tensor(self.vertices.astype(np.float32)),
            o3d.core.Tensor(triangles.astype(np.uint32)))
        return scene

    def contains(self, points, padding):
        """Select shell points and interiors whose component defines a closed volume."""
        result = np.zeros(len(points), dtype=bool)
        indices = np.flatnonzero(np.all(
            (points >= self.lower - padding) & (points <= self.upper + padding), axis=1))
        if not len(indices):
            return result
        query = self._o3d.core.Tensor(points[indices].astype(np.float32))
        result[indices] = self.scene.compute_distance(query, nthreads=2).numpy() <= padding + 1e-6
        for scene, lower, upper in self.volumes:
            candidates = indices[~result[indices] & np.all(
                (points[indices] >= lower) & (points[indices] <= upper), axis=1)]
            if len(candidates):
                query = self._o3d.core.Tensor(points[candidates].astype(np.float32))
                result[candidates] |= scene.compute_occupancy(query, nthreads=2).numpy() > 0.5
        return result


class SelfFilter:
    """Apply all URDF collision meshes at measured joint angles in the cloud frame."""

    def __init__(self, urdf_path, padding=0.15, scale=1.0):
        """Load geometry once; grow each collision about its own bounding-box center."""
        if not np.isfinite(padding) or padding < 0:
            raise ValueError('self_filter_padding must be finite and >= 0')
        if not np.isfinite(scale) or scale < 1:
            raise ValueError('self_filter_scale must be finite and >= 1')
        import open3d as o3d

        self.padding = float(padding)
        self.scale = float(scale)
        path = Path(urdf_path)
        root = ET.parse(path).getroot()
        links = {link.get('name') for link in root.findall('link')}
        self.joints, self.collisions = [], []
        for joint in root.findall('joint'):
            kind = joint.get('type')
            if kind not in ('fixed', 'continuous', 'revolute', 'prismatic'):
                raise ValueError(f'Unsupported URDF joint type: {kind}')
            axis_element = joint.find('axis')
            axis = _numbers(axis_element.get('xyz', '1 0 0'), 3) if (
                axis_element is not None) else np.array([1., 0., 0.])
            if kind != 'fixed' and np.linalg.norm(axis) == 0:
                raise ValueError(f'Joint {joint.get("name")} has zero axis')
            axis = axis / np.linalg.norm(axis)
            mimic = joint.find('mimic')
            self.joints.append(_Joint(
                joint.get('name'), kind, joint.find('parent').get('link'),
                joint.find('child').get('link'), _origin(joint.find('origin')), axis,
                None if mimic is None else (mimic.get('joint'),
                                            float(mimic.get('multiplier', '1')),
                                            float(mimic.get('offset', '0')))))
        roots = links - {joint.child for joint in self.joints}
        if len(roots) != 1:
            raise ValueError('Robot URDF must have exactly one root link')
        self.base_link = roots.pop()
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                mesh = geometry.find('mesh')
                if mesh is None:
                    raise ValueError('Self filter currently requires URDF collision mesh geometry')
                filename = mesh.get('filename')
                if filename.startswith('package://'):
                    from ament_index_python.packages import get_package_share_directory
                    package, relative = filename[len('package://'):].split('/', 1)
                    mesh_path = Path(get_package_share_directory(package)) / relative
                else:
                    mesh_path = path.parent / filename
                if mesh_path.suffix.lower() == '.dae':
                    vertices, triangles = _collada_mesh(mesh_path)
                else:
                    legacy = o3d.io.read_triangle_mesh(str(mesh_path))
                    vertices, triangles = np.asarray(legacy.vertices), np.asarray(legacy.triangles)
                mesh_scale = _numbers(mesh.get('scale', '1 1 1'), 3)
                self.collisions.append(_Collision(
                    o3d, link.get('name'), _origin(collision.find('origin')),
                    vertices * mesh_scale, triangles))
        if not self.collisions:
            raise ValueError('Robot URDF has no collision meshes')
        # Joints on collision-free leaves (for example bucket_end_joint) are optional.
        needed_links = {collision.link for collision in self.collisions}
        changed = True
        while changed:
            previous = len(needed_links)
            needed_links.update(j.parent for j in self.joints if j.child in needed_links)
            changed = len(needed_links) != previous
        self.joints = [j for j in self.joints if j.child in needed_links]
        self.required_joints = tuple(j.name for j in self.joints
                                     if j.kind != 'fixed' and j.mimic is None)

    def link_transforms(self, joint_positions):
        """Return base_from_link transforms, rejecting absent or nonfinite angles."""
        missing = set(self.required_joints) - set(joint_positions)
        if missing:
            raise ValueError(f'Missing required joints: {sorted(missing)}')
        values = {name: float(value) for name, value in joint_positions.items()}
        if not all(np.isfinite(values[name]) for name in self.required_joints):
            raise ValueError('Required joint positions must be finite')
        transforms = {self.base_link: np.eye(4)}
        pending = self.joints.copy()
        while pending:
            advanced = False
            for joint in pending.copy():
                if joint.parent not in transforms:
                    continue
                if joint.mimic:
                    source, multiplier, offset = joint.mimic
                    if source not in values:
                        continue
                    values[joint.name] = values[source] * multiplier + offset
                movement = np.eye(4)
                if joint.kind != 'fixed':
                    value = values[joint.name]
                    if not np.isfinite(value):
                        raise ValueError('Joint positions must be finite')
                    if joint.kind == 'prismatic':
                        movement[:3, 3] = joint.axis * value
                    else:
                        movement[:3, :3] = Rotation.from_rotvec(joint.axis * value).as_matrix()
                transforms[joint.child] = transforms[joint.parent] @ joint.origin @ movement
                pending.remove(joint)
                advanced = True
            if not advanced:
                raise ValueError('URDF has disconnected links, cyclic joints or unresolved mimic')
        return transforms

    def _growth_scales(self):
        """Combine the fixed 0.1 scale grid and the exact requested final scale."""
        step = 11
        while step / 10 < self.scale - 1e-12:
            yield step / 10
            step += 1
        yield self.scale

    def mask(self, points, cloud_from_base, joint_positions):
        """Return True for robot points; preserve input order and reject invalid poses."""
        points = np.asarray(points, dtype=np.float64)
        matrix = np.asarray(cloud_from_base, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('points must have shape (N, 3)')
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError('cloud_from_base must be a finite 4x4 rigid transform')
        if (not np.allclose(matrix[3], [0, 0, 0, 1])
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0)):
            raise ValueError('cloud_from_base must be a rigid transform')
        transforms = self.link_transforms(joint_positions)
        result = np.zeros(len(points), dtype=bool)
        finite = np.isfinite(points).all(axis=1)
        for collision in self.collisions:
            indices = np.flatnonzero(finite & ~result)
            if not len(indices):
                break
            cloud_from_mesh = matrix @ transforms[collision.link] @ collision.origin
            local = (points[indices] - cloud_from_mesh[:3, 3]) @ cloud_from_mesh[:3, :3]
            selected = collision.contains(local, self.padding)
            if self.scale > 1:
                # Inverse-scale queries instead of rebuilding triangle scenes.
                # Divide padding as well so the final shell stays in metres.
                # Retain intermediate shells as well: an open surface moving
                # outward must not reveal points removed at a previous 0.1 step.
                center = (collision.lower + collision.upper) / 2
                for scale in self._growth_scales():
                    remaining = np.flatnonzero(~selected)
                    if not len(remaining):
                        break
                    scaled_queries = center + (local[remaining] - center) / scale
                    selected[remaining] = collision.contains(
                        scaled_queries, self.padding / scale)
            result[indices] = selected
        return result
