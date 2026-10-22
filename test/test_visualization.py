"""Check that RViz receives the actual mesh and clears old geometry."""

from geometry_msgs.msg import Point
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from pointcloud_filters.visualization import mesh_marker


def test_marker_preserves_triangle_order_coordinates_and_header():
    """Expand indexed faces with unit scale and preserve the input coordinate frame."""
    mesh = Mesh(
        vertices=[Point(x=1.0), Point(y=2.0), Point(z=3.0), Point(x=4.0)],
        triangles=[MeshTriangle(vertex_indices=[0, 2, 1]),
                   MeshTriangle(vertex_indices=[2, 3, 1])])
    header = Header(frame_id='map')
    header.stamp.sec = 123
    marker = mesh_marker(mesh, header)
    assert marker.header == header
    assert marker.type == Marker.TRIANGLE_LIST and marker.action == Marker.ADD
    assert marker.points == [mesh.vertices[i] for i in [0, 2, 1, 2, 3, 1]]
    assert marker.scale.x == marker.scale.y == marker.scale.z == 1.0
    assert marker.pose.orientation.w == 1.0
    assert marker.color.a == 1.0
    restored = deserialize_message(serialize_message(marker), Marker)
    assert restored.header == marker.header and restored.points == marker.points
    assert restored.scale == marker.scale and restored.pose == marker.pose
    assert [restored.color.r, restored.color.g, restored.color.b, restored.color.a] == \
        pytest.approx([marker.color.r, marker.color.g, marker.color.b, marker.color.a])
    marker.header.frame_id = 'other'
    assert header.frame_id == 'map'


def test_empty_mesh_deletes_previous_marker():
    """Reuse the marker identity when removing a previously visible surface."""
    previous = mesh_marker(Mesh(), Header(frame_id='map'))
    assert previous.action == Marker.DELETE
    assert previous.ns == 'filtered_mesh' and previous.id == 0
    assert previous.points == []
