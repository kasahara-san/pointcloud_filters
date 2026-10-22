"""Adapt filtered geometry to RViz displays without losing its source header."""

from copy import deepcopy

from visualization_msgs.msg import Marker


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
