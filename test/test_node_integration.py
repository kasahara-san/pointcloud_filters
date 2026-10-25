"""Exercise articulated self removal through real ROS publishers and subscribers."""

from copy import deepcopy
import math
import os
import threading
import time
import uuid

from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField
from shape_msgs.msg import Mesh
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker

from pointcloud_filters.cloud_io import read_cloud
from pointcloud_filters.pointcloud_filter_node import PointCloudFilterNode


def _robot_description(directory):
    vertices = [(-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5),
                (-0.5, 0.5, -0.5), (-0.5, -0.5, 0.5), (0.5, -0.5, 0.5),
                (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5)]
    triangles = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
                 (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
                 (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    text = ''.join(f'v {x} {y} {z}\n' for x, y, z in vertices)
    text += ''.join(f'f {a + 1} {b + 1} {c + 1}\n' for a, b, c in triangles)
    (directory / 'box.obj').write_text(text)
    path = directory / 'robot.urdf'
    path.write_text(
        '<robot name="integration_robot"><link name="base_link"/>'
        '<link name="arm_link"><collision><origin xyz="1 0 0"/>'
        '<geometry><mesh filename="box.obj"/></geometry></collision></link>'
        '<joint name="arm_joint" type="continuous"><parent link="base_link"/>'
        '<child link="arm_link"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/>'
        '</joint></robot>')
    return path


def _cloud(seconds=100):
    # world->map has yaw 90 degrees, world->base has yaw 180 degrees,
    # and the joint has yaw 90 degrees. The articulated cube center must
    # therefore be map (2, 5, 0), independently of production FK code.
    robot = np.array([[2, 5, 0.5], [2, 5, 0], [2.5, 5, 0], [2, 5, 0.6]])
    x, y = np.meshgrid(np.linspace(6, 8, 9), np.linspace(2, 4, 9))
    terrain = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    points = np.vstack((robot, terrain, [[100, 100, 0], [np.nan, 0, 0]]))
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'id'],
                      'formats': ['<f4', '<f4', '<f4', '<u2', '<u4'],
                      'offsets': [0, 4, 8, 12, 16], 'itemsize': 20})
    data = bytearray([0xa5] * (len(points) * dtype.itemsize))
    records = np.ndarray(len(points), dtype=dtype, buffer=data)
    for index, name in enumerate(('x', 'y', 'z')):
        records[name] = points[:, index]
    records['intensity'] = np.arange(len(points)) + 100
    records['id'] = np.arange(len(points)) + 1000
    message = PointCloud2()
    message.header.frame_id = 'map'
    message.header.stamp.sec = seconds
    message.height, message.width = 1, len(points)
    message.point_step, message.row_step = 20, 20 * len(points)
    message.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in [('x', 0), ('y', 4), ('z', 8)]
    ] + [PointField(name='intensity', offset=12, datatype=PointField.UINT16, count=1),
         PointField(name='id', offset=16, datatype=PointField.UINT32, count=1)]
    message.data = bytes(data)
    return message


def _pose(seconds=100):
    message = PoseStamped()
    # The real recording has empty frame IDs; pose_frame supplies world.
    message.header.stamp.sec = seconds
    message.pose.position.x = 99996.0
    message.pose.position.y = -29997.0
    message.pose.position.z = 50.0
    message.pose.orientation.z = 1.0
    message.pose.orientation.w = 0.0
    return message


def _joints(seconds=100, name='arm_joint'):
    message = JointState(name=[name], position=[math.pi / 2])
    message.header.stamp.sec = seconds
    return message


class _RunningFilter:

    def __init__(self, output_format):
        self.outputs, self.removed, self.headers, self.markers = [], [], [], []
        self.failures = []
        self.filter = PointCloudFilterNode()
        self.peer = Node('filter_test_peer_' + uuid.uuid4().hex[:8])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cloud_publisher = self.peer.create_publisher(PointCloud2, '/test/cloud', qos)
        self.pose_publisher = self.peer.create_publisher(PoseStamped, '/test/pose', qos)
        self.joint_publisher = self.peer.create_publisher(JointState, '/test/joints', qos)
        self.subscriptions = [
            self.peer.create_subscription(
                Mesh if output_format == 'mesh' else PointCloud2,
                '/test/filtered', self.outputs.append, qos),
            self.peer.create_subscription(
                PointCloud2, '/test/filtered/self_removed', self.removed.append, qos),
        ]
        if output_format == 'mesh':
            self.subscriptions.extend([
                self.peer.create_subscription(
                    Header, '/test/filtered/header', self.headers.append, qos),
                self.peer.create_subscription(
                    Marker, '/test/filtered/marker', self.markers.append,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            ])
        self.tf_broadcaster = StaticTransformBroadcaster(self.peer)
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.filter)
        self.executor.add_node(self.peer)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        self.wait(lambda: all(publisher.get_subscription_count() for publisher in (
            self.cloud_publisher, self.pose_publisher, self.joint_publisher)))
        self.wait(lambda: self.filter.count_subscribers('/test/filtered') >= 1
                  and self.filter.count_subscribers('/test/filtered/self_removed') >= 1)

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as error:
            self.failures.append(error)

    def wait(self, condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.failures:
                raise AssertionError(f'Executor failed: {self.failures}')
            if condition():
                return
            time.sleep(0.01)
        raise AssertionError('Timed out waiting for ROS message delivery')

    def assert_quiet(self, duration=0.35):
        count = len(self.outputs)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert not self.failures
            assert len(self.outputs) == count
            time.sleep(0.01)

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'world', 'map'
        transform.transform.translation.x = 100000.0
        transform.transform.translation.y = -30000.0
        transform.transform.translation.z = 50.0
        transform.transform.rotation.z = math.sqrt(0.5)
        transform.transform.rotation.w = math.sqrt(0.5)
        self.tf_broadcaster.sendTransform(transform)

    def publish_state(self, seconds=100):
        self.pose_publisher.publish(_pose(seconds))
        self.joint_publisher.publish(_joints(seconds))

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        self.filter.destroy_node()
        self.peer.destroy_node()
        assert not self.thread.is_alive()
        assert not self.failures


@pytest.fixture
def running_filter(tmp_path, monkeypatch):
    """Provide isolated DDS nodes with a 0.1-second processing timer."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    path = _robot_description(tmp_path)
    active = []

    def start(output_format='pointcloud2', **overrides):
        arguments = ['--ros-args']
        values = dict(
            robot_description_path=str(path), output_format=output_format,
            input_topic='/test/cloud', pose_topic='/test/pose',
            joint_states_topic='/test/joints', output_topic='/test/filtered',
            publish_period_sec=0.1, voxel_size=0.0, sor_stddev_mul=1000000.0,
            max_pose_time_diff_sec=0.2, max_joint_time_diff_sec=0.2,
        )
        values.update(overrides)
        for name, value in values.items():
            arguments.extend(['-p', f'{name}:={value}'])
        rclpy.init(args=arguments)
        instance = _RunningFilter(output_format)
        active.append(instance)
        return instance

    yield start
    for instance in active:
        instance.close()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize('output_format', ['pointcloud2', 'mesh'])
def test_world_pose_joint_kinematics_and_both_outputs(running_filter, output_format):
    """Remove the measured robot while preserving terrain and output metadata."""
    running = running_filter(output_format)
    running.publish_transform()
    running.publish_state()
    cloud = _cloud()
    original = deepcopy(cloud)
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: bool(running.outputs) and bool(running.removed))
    removed = running.removed[0]
    assert removed.width == 4
    assert removed.header == cloud.header and removed.fields == cloud.fields
    assert bytes(removed.data) == bytes(cloud.data[:4 * cloud.point_step])
    if output_format == 'pointcloud2':
        output = running.outputs[0]
        assert output.width == 81 and output.height == 1
        assert output.header == cloud.header and output.fields == cloud.fields
        assert output.point_step == cloud.point_step
        assert bytes(output.data) == bytes(cloud.data[4 * 20:85 * 20])
        xyz, _ = read_cloud(output)
        assert np.all(xyz[:, 0] >= 6)
    else:
        running.wait(lambda: bool(running.headers) and bool(running.markers))
        output = running.outputs[0]
        assert len(output.vertices) > 10 and len(output.triangles) > 10
        assert all(point.x >= 6 and point.z == 0 for point in output.vertices)
        assert running.headers[0] == cloud.header
        marker = running.markers[0]
        assert marker.header == cloud.header and marker.action == Marker.ADD
        assert marker.points == [output.vertices[index] for triangle in output.triangles
                                 for index in triangle.vertex_indices]
    assert cloud == original
    running.assert_quiet()


def test_missing_and_stale_partial_joints_wait_then_recover(running_filter):
    """Missing/stale joint samples must hold the cloud until every joint is current."""
    running = running_filter()
    running.publish_transform()
    running.pose_publisher.publish(_pose())
    running.cloud_publisher.publish(_cloud())
    running.assert_quiet()
    running.joint_publisher.publish(_joints(98))
    running.assert_quiet()
    running.joint_publisher.publish(_joints())
    running.wait(lambda: len(running.outputs) == 1 and len(running.removed) == 1)
    assert running.removed[0].width == 4

    running.pose_publisher.publish(_pose(102))
    running.cloud_publisher.publish(_cloud(102))
    running.joint_publisher.publish(_joints(102, 'unrelated_joint'))
    running.assert_quiet()
    running.joint_publisher.publish(_joints(102))
    running.wait(lambda: len(running.outputs) == 2)
    assert running.outputs[-1].header.stamp.sec == 102
    assert running.outputs[-1].width == 81


def test_missing_static_transform_waits_then_recovers(running_filter):
    """A later world/map static transform must release the pending measured cloud."""
    running = running_filter()
    running.publish_state()
    running.cloud_publisher.publish(_cloud())
    running.assert_quiet()
    running.publish_transform()
    running.wait(lambda: len(running.outputs) == 1)
    assert running.outputs[0].width == 81
    assert running.outputs[0].header.frame_id == 'map'


def test_self_filter_disabled_preserves_legacy_unstamped_crop(running_filter):
    """Disabling self removal must allow a position-only pose with no joints or TF."""
    running = running_filter(self_filter=False)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x, pose.pose.position.y = 3.0, 4.0
    pose.pose.orientation.w = 0.0
    cloud = _cloud()
    running.pose_publisher.publish(pose)
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: len(running.outputs) == 1)
    assert running.outputs[0].width == 85
    assert bytes(running.outputs[0].data) == bytes(cloud.data[:85 * 20])
    assert running.outputs[0].header == cloud.header
    assert running.removed == []


def test_calibrated_model_origin_is_applied_in_base_coordinates(running_filter):
    """An explicit CAD offset changes geometry while the crop center stays measured."""
    running = running_filter(model_origin_xyz=[0.0, 1.0, 0.0])
    running.publish_transform()
    running.publish_state()
    cloud = _cloud()
    data = bytearray(cloud.data)
    x = np.ndarray(cloud.width, dtype='<f4', buffer=data, strides=(cloud.point_step,))
    # map<-base has yaw90, so +1m along base Y moves the CAD -1m along map X.
    x[:4] -= 1.0
    cloud.data = bytes(data)
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: bool(running.outputs) and bool(running.removed))
    assert running.removed[0].width == 4
    assert bytes(running.removed[0].data) == bytes(cloud.data[:4 * cloud.point_step])
    assert running.outputs[0].width == 81


def test_default_ignores_irrelevant_tf_from_mixed_time_domains(running_filter, capfd):
    """Live and recorded robot TF cannot pollute the default static-only buffer."""
    running = running_filter()
    current = running.peer.create_publisher(TFMessage, '/tf', 10)
    recorded = running.peer.create_publisher(TFMessage, '/tf', 10)
    running.publish_transform()
    for _ in range(5):
        for publisher, seconds in ((current, 2000), (recorded, 1000)):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'world', 'base_link_3d'
            transform.header.stamp.sec = seconds
            publisher.publish(TFMessage(transforms=[transform]))
            time.sleep(0.01)
    running.publish_state()
    running.cloud_publisher.publish(_cloud())
    running.wait(lambda: bool(running.outputs) and bool(running.removed))
    assert current.get_subscription_count() == recorded.get_subscription_count() == 0
    assert running.outputs[0].width == 81
    assert running.removed[0].width == 4
    running.assert_quiet(0.2)
    captured = capfd.readouterr()
    assert 'TF_OLD_DATA' not in captured.out + captured.err


def test_dynamic_tf_opt_in_interpolates_at_measurement_time(running_filter):
    """Explicit dynamic TF supports measured interpolation without static world/map."""
    running = running_filter(use_dynamic_tf=True)
    publisher = running.peer.create_publisher(TFMessage, '/tf', 10)
    running.wait(lambda: publisher.get_subscription_count() >= 1)
    for seconds, offset in ((100, -2.0), (102, 2.0)):
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'world', 'map'
        transform.header.stamp.sec = seconds
        transform.transform.translation.x = 100000.0 + offset
        transform.transform.translation.y = -30000.0
        transform.transform.translation.z = 50.0
        transform.transform.rotation.z = math.sqrt(0.5)
        transform.transform.rotation.w = math.sqrt(0.5)
        publisher.publish(TFMessage(transforms=[transform]))
    running.publish_state(101)
    cloud = _cloud(101)
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: bool(running.outputs) and bool(running.removed))
    assert running.outputs[0].width == 81
    assert running.removed[0].width == 4
    assert running.outputs[0].header == cloud.header


def test_expanded_model_scale_reaches_the_self_filter(running_filter):
    """The requested scale removes a point outside the original mesh and padding."""
    running = running_filter(self_filter_scale=1.3)
    running.publish_transform()
    running.publish_state()
    cloud = _cloud()
    data = bytearray(cloud.data)
    z = np.ndarray(cloud.width, dtype='<f4', buffer=data, offset=8,
                   strides=(cloud.point_step,))
    # Original half-height .50 + padding .15 excludes .75; scaling the
    # collision around its own center gives .65 + .15 and includes it.
    z[3] = 0.75
    cloud.data = bytes(data)
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: bool(running.outputs) and bool(running.removed))
    assert running.removed[0].width == 4
    assert bytes(running.removed[0].data) == bytes(cloud.data[:4 * cloud.point_step])
    assert running.outputs[0].width == 81
