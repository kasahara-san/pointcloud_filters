"""Adapt filtered geometry to RViz displays without losing its source header."""

from copy import deepcopy

from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker, MarkerArray


def mesh_marker(mesh, header):
    """Expand mesh indices into a persistent triangle marker, or delete an old one."""
    marker = Marker()
    marker.header = deepcopy(header)
    marker.ns = 'filtered_mesh'
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD if mesh.triangles else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.15, 0.95, 0.35, 1.0
    marker.points = [mesh.vertices[index]
                     for triangle in mesh.triangles for index in triangle.vertex_indices]
    return marker


def robot_markers(model, cloud_from_model, joints, header):
    """Show actual articulated CAD surfaces in the same frame as the input cloud."""
    transforms = model.link_transforms(joints)
    result = MarkerArray()
    for index, collision in enumerate(model.collisions):
        matrix = cloud_from_model @ transforms[collision.link] @ collision.origin
        marker = Marker()
        marker.header = deepcopy(header)
        marker.ns, marker.id = 'robot_model', index
        marker.type, marker.action = Marker.MESH_RESOURCE, Marker.ADD
        marker.mesh_resource = collision.mesh_resource
        marker.mesh_use_embedded_materials = False
        p, q = matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, p)
        marker.pose.orientation.x, marker.pose.orientation.y, \
            marker.pose.orientation.z, marker.pose.orientation.w = map(float, q)
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, collision.mesh_scale)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .75, .15, .3
        result.markers.append(marker)
    return result
