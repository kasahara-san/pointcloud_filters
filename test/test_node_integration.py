"""Exercise the bundled D37PXi model through real ROS publishers and subscribers."""

from copy import deepcopy
import math
import os
from pathlib import Path
import threading
import time
import uuid

from geometry_msgs.msg import PoseStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2, PointField
from shape_msgs.msg import Mesh
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

import pointcloud_filters.pointcloud_filter_node as filter_module


def _cloud(seconds=100):
    # Four points inside the actual D37PXi body envelope, followed by visible terrain.
    robot = np.array([[0., 0., 1.], [1., 0., 1.], [-1., 0., .5], [0., 0., 2.5]])
    x, y = np.meshgrid(np.linspace(5., 7., 9), np.linspace(2., 4., 9))
    terrain = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    base = np.vstack((robot, terrain, [[11., 0., 0.], [0., 0., 0.], [np.nan, 0., 0.]]))
    # Independent inverse of base<-LiDAR: XYZ=[1,0,2], RPY=[-pi/2,0,-pi/2].
    points = np.column_stack((-base[:, 1], 2. - base[:, 2], base[:, 0] - 1.))
    points[86] = 0.  # Hesai's unmeasured zero placeholder.
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
    message.header.frame_id = 'hesai_lidar'
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


def _pose(seconds=1100, frame='map'):
    message = PoseStamped()
    message.header.frame_id = frame
    message.header.stamp.sec = seconds
    message.pose.position.x, message.pose.position.y, message.pose.position.z = \
        -19268., -138681., 88.
    message.pose.orientation.w = 1.
    return message


def _joints(seconds=1100, names=('lift', 'tilt', 'angle')):
    message = JointState(name=list(names), position=[0.] * len(names))
    message.header.stamp.sec = seconds
    return message


class _RunningFilter:

    def __init__(self, output_format):
        self.outputs, self.removed, self.headers, self.markers, self.models = [], [], [], [], []
        self.failures = []
        self.filter = filter_module.PointCloudFilterNode()
        self.peer = Node('d37_filter_test_peer_' + uuid.uuid4().hex[:8])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        def topic(name):
            return self.filter.get_parameter(name).value

        self.cloud_publisher = self.peer.create_publisher(PointCloud2, topic('input_topic'), qos)
        self.pose_publisher = self.peer.create_publisher(PoseStamped, topic('pose_topic'), qos)
        self.joint_publisher = self.peer.create_publisher(
            JointState, topic('joint_states_topic'), qos)
        output = topic('output_topic')
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            self.peer.create_subscription(Mesh if output_format == 'mesh' else PointCloud2,
                                          output, self.outputs.append, qos),
            self.peer.create_subscription(PointCloud2, output + '/self_removed',
                                          self.removed.append, qos),
            self.peer.create_subscription(MarkerArray, output + '/robot_model',
                                          self.models.append, transient),
        ]
        if output_format == 'mesh':
            self.subscriptions.extend([
                self.peer.create_subscription(
                    Header, output + '/header', self.headers.append, qos),
                self.peer.create_subscription(Marker, output + '/marker',
                                              self.markers.append, transient),
            ])
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.filter)
        self.executor.add_node(self.peer)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        self.wait(lambda: all(publisher.get_subscription_count() for publisher in (
            self.cloud_publisher, self.pose_publisher, self.joint_publisher)))
        self.wait(lambda: self.filter.count_subscribers(output) >= 1)

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
            time.sleep(.01)
        raise AssertionError('Timed out waiting for ROS message delivery')

    def assert_quiet(self, duration=.35):
        count = len(self.outputs)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert not self.failures and len(self.outputs) == count
            time.sleep(.01)

    def publish_state(self, seconds=100):
        self.pose_publisher.publish(_pose(seconds + 1000))
        self.joint_publisher.publish(_joints(seconds + 1000))

    def close(self):
        self.executor.shutdown(timeout_sec=5.)
        self.thread.join(timeout=5.)
        self.filter.destroy_node()
        self.peer.destroy_node()
        assert not self.thread.is_alive() and not self.failures


@pytest.fixture
def running_filter(tmp_path, monkeypatch):
    """Use the real D37PXi description in an isolated ROS domain with a short timer."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    root = Path(__file__).resolve().parents[1]
    monkeypatch.setattr(filter_module, 'get_package_share_directory', lambda _: str(root))
    active = []

    def start(output_format='pointcloud2', default_topics=False, **overrides):
        arguments = ['--ros-args']
        values = dict(output_format=output_format, publish_period_sec=.1, voxel_size=0.,
                      max_pose_time_diff_sec=.2, max_joint_time_diff_sec=.2,
                      sensor_xyz=[1., 0., 2.], sensor_rpy=[-math.pi / 2, 0., -math.pi / 2],
                      cloud_time_offset_sec=1000.,
                      visualization_base_frame='test_d37_filter_base')
        if not default_topics:
            values.update(input_topic='/test/cloud', pose_topic='/test/pose',
                          joint_states_topic='/test/joints', output_topic='/test/filtered')
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
def test_d37pxi_onboard_filter_preserves_terrain_and_source_records(running_filter, output_format):
    """Remove the bundled machine using mapped joints and shifted sensor time."""
    running = running_filter(output_format)
    running.publish_state()
    cloud = _cloud()
    original = deepcopy(cloud)
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: bool(running.outputs) and bool(running.removed) and bool(running.models))
    removed = running.removed[0]
    assert removed.width == 4 and removed.header == cloud.header
    assert bytes(removed.data) == bytes(cloud.data[:4 * cloud.point_step])
    if output_format == 'pointcloud2':
        output = running.outputs[0]
        assert output.width == 81 and output.height == 1
        assert output.header == cloud.header and output.fields == cloud.fields
        assert bytes(output.data) == bytes(cloud.data[4 * 20:85 * 20])
    else:
        running.wait(lambda: bool(running.headers) and bool(running.markers))
        output = running.outputs[0]
        assert len(output.vertices) > 10 and len(output.triangles) > 10
        assert all(point.z >= 4. for point in output.vertices)
        assert running.headers[0] == cloud.header
        marker = running.markers[0]
        assert marker.header == cloud.header and marker.action == Marker.ADD
        assert marker.points == [output.vertices[i] for face in output.triangles
                                 for i in face.vertex_indices]
    assert len(running.models[0].markers) == 7
    assert all(marker.header == cloud.header for marker in running.models[0].markers)
    assert cloud == original
    running.assert_quiet()


def test_missing_and_stale_partial_blade_joints_wait_then_recover(running_filter):
    """Every D37PXi blade joint needs a current sample, even in partial updates."""
    running = running_filter()
    running.pose_publisher.publish(_pose())
    running.cloud_publisher.publish(_cloud())
    running.assert_quiet()
    running.joint_publisher.publish(_joints(1098))
    running.joint_publisher.publish(_joints(names=('lift', 'angle')))
    running.assert_quiet()
    running.joint_publisher.publish(_joints(names=('tilt',)))
    running.wait(lambda: len(running.outputs) == 1)
    running.pose_publisher.publish(_pose(1102))
    running.cloud_publisher.publish(_cloud(102))
    running.joint_publisher.publish(_joints(1102, ('unrelated',)))
    running.assert_quiet()
    running.joint_publisher.publish(_joints(1102))
    running.wait(lambda: len(running.outputs) == 2)


def test_mismatched_state_clock_waits_for_synchronized_samples(running_filter):
    """Without the configured epoch shift, distant state timestamps cannot pass."""
    running = running_filter(cloud_time_offset_sec=0.)
    running.publish_state()
    running.cloud_publisher.publish(_cloud())
    running.assert_quiet()
    running.pose_publisher.publish(_pose(100))
    running.joint_publisher.publish(_joints(100))
    running.wait(lambda: len(running.outputs) == 1)
    assert running.outputs[0].header.stamp.sec == 100


def test_crop_uses_map_horizontal_plane_instead_of_sideways_sensor_xy(running_filter):
    """Keep a high nearby return while excluding a far forward return."""
    running = running_filter()
    running.publish_state()
    cloud = _cloud()
    data = bytearray(cloud.data)
    for axis, value in enumerate([0., -98., -1.]):
        field = np.ndarray(cloud.width, dtype='<f4', buffer=data, offset=axis * 4,
                           strides=(cloud.point_step,))
        field[-1] = value  # base XYZ=(0,0,100): within the horizontal crop.
    cloud.data = bytes(data)
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: bool(running.outputs))
    output = running.outputs[0]
    assert output.width == 82
    assert bytes(output.data[-20:]) == bytes(cloud.data[-20:])


def test_self_filter_disabled_still_applies_onboard_range_and_crop(running_filter):
    """The diagnostic opt-out preserves machine returns but excludes invalid scans."""
    running = running_filter(self_filter=False)
    running.pose_publisher.publish(_pose())
    cloud = _cloud()
    running.cloud_publisher.publish(cloud)
    running.wait(lambda: bool(running.outputs))
    assert running.outputs[0].width == 85
    assert bytes(running.outputs[0].data) == bytes(cloud.data[:85 * 20])
    assert running.removed == [] and running.models == []


def test_direct_run_defaults_use_d37pxi_topics_and_bundled_model(running_filter):
    """Starting without a params file must require only D37PXi sensor topics."""
    running = running_filter(default_topics=True)
    assert running.cloud_publisher.topic_name == '/lidar_points'
    assert running.pose_publisher.topic_name == '/d37pxi/global_pose'
    assert running.joint_publisher.topic_name == '/d37pxi/joint_states'
    running.publish_state()
    running.cloud_publisher.publish(_cloud())
    running.wait(lambda: bool(running.outputs))
    assert running.outputs[0].width == 81
