"""Periodically crop the latest PointCloud2 around the backhoe's XY position."""

from copy import deepcopy
from dataclasses import asdict
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, PointCloud2
from shape_msgs.msg import Mesh
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException
from visualization_msgs.msg import Marker

from pointcloud_filters.cloud_io import read_cloud, select_cloud
from pointcloud_filters.filtering import FilterParameters, filter_indices
from pointcloud_filters.meshing import MeshConverter, mesh_voxel_size
from pointcloud_filters.position import (
    model_origin_matrix, pose_matrix_in_cloud_frame, position_in_cloud_frame)
from pointcloud_filters.self_filter import SelfFilter
from pointcloud_filters.state_history import stamp_ns, StateHistory
from pointcloud_filters.tf_listener import FilterTransformListener
from pointcloud_filters.visualization import mesh_marker


class PointCloudFilterNode(Node):
    """Keep one pending input and filter it on a steady-clock timer."""

    def __init__(self):
        """Declare startup parameters and configure compatible topic QoS."""
        super().__init__('pointcloud_filter')
        descriptor = ParameterDescriptor(read_only=True)
        defaults = dict(input_topic='/merged_pointcloud',
                        pose_topic='/zx200/global_pose', pose_frame='world',
                        joint_states_topic='/zx200/joint_states',
                        max_pose_time_diff_sec=0.5, max_joint_time_diff_sec=0.5,
                        state_history_sec=30.0, self_filter=True,
                        use_dynamic_tf=False,
                        self_filter_padding=0.15, self_filter_scale=1.0,
                        robot_description_path='',
                        model_origin_xyz=[0.0, 0.0, 0.0], model_origin_rpy=[0.0, 0.0, 0.0],
                        output_topic='/merged_pointcloud_filtered',
                        output_format='pointcloud2', mesh_resolution=3,
                        publish_period_sec=10.0, **asdict(FilterParameters()))
        values = {
            name: self.declare_parameter(name, default, descriptor).value
            for name, default in defaults.items()
        }
        self._parameters_config = FilterParameters(**{
            name: values[name] for name in asdict(FilterParameters())})
        self._period = values['publish_period_sec']
        if not math.isfinite(self._period) or self._period <= 0:
            raise ValueError('publish_period_sec must be finite and > 0')
        self._max_pose_time_diff = values['max_pose_time_diff_sec']
        if not math.isfinite(self._max_pose_time_diff) or self._max_pose_time_diff < 0:
            raise ValueError('max_pose_time_diff_sec must be finite and >= 0')
        self._max_joint_time_diff = values['max_joint_time_diff_sec']
        if not math.isfinite(self._max_joint_time_diff) or self._max_joint_time_diff < 0:
            raise ValueError('max_joint_time_diff_sec must be finite and >= 0')
        self._pose_frame = values['pose_frame']
        self._base_from_model = model_origin_matrix(
            values['model_origin_xyz'], values['model_origin_rpy'])
        self._self_filter = None
        if values['self_filter']:
            robot_path = values['robot_description_path'] or os.path.join(
                get_package_share_directory('pointcloud_filters'), 'robot', 'zx200.urdf')
            self._self_filter = SelfFilter(
                robot_path, padding=values['self_filter_padding'],
                scale=values['self_filter_scale'])
        self._output_format = values['output_format']
        if self._output_format not in ('pointcloud2', 'mesh'):
            raise ValueError('output_format must be pointcloud2 or mesh')
        mesh_voxel_size(values['mesh_resolution'])
        self._mesh_converter = (MeshConverter(values['mesh_resolution'])
                                if self._output_format == 'mesh' else None)
        input_topic = self.resolve_topic_name(values['input_topic'])
        output_topic = self.resolve_topic_name(values['output_topic'])
        if input_topic == output_topic:
            raise ValueError('input_topic and output_topic must differ')
        self._lock = threading.Lock()
        self._latest_cloud = None
        self._state_history = StateHistory(values['state_history_sec'])
        self._last_pose_warning = None
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = FilterTransformListener(
            self._tf_buffer, self, use_dynamic_tf=values['use_dynamic_tf'])
        self._input_group = MutuallyExclusiveCallbackGroup()
        self._subscription = self.create_subscription(
            PointCloud2, values['input_topic'], self._receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
            callback_group=self._input_group)
        self._pose_subscription = self.create_subscription(
            PoseStamped, values['pose_topic'], self._receive_pose,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
            callback_group=self._input_group)
        self._joint_subscription = self.create_subscription(
            JointState, values['joint_states_topic'], self._receive_joints,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
            callback_group=self._input_group)
        output_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self._publisher = self.create_publisher(
            Mesh if self._mesh_converter else PointCloud2,
            values['output_topic'], output_qos)
        self._header_publisher = None
        self._removed_publisher = (self.create_publisher(
            PointCloud2, output_topic + '/self_removed', output_qos)
            if self._self_filter else None)
        self._marker_publisher = None
        if self._mesh_converter:
            self._header_publisher = self.create_publisher(
                Header, self._publisher.topic_name + '/header', output_qos)
            self._marker_publisher = self.create_publisher(
                Marker, self._publisher.topic_name + '/marker',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Filtering does not block the subscription callback's executor thread.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            self._period, self._process_latest, clock=self._steady_clock)
        self.get_logger().info(
            f'{input_topic} -> {output_topic}, '
            f"output_format={self._output_format}, "
            f"mesh_resolution={values['mesh_resolution']}, "
            f'period={self._period:.3f}s (wall time), '
            f'parameters={self._parameters_config}')
        self.get_logger().info(
            f"XY center: {self.resolve_topic_name(values['pose_topic'])} "
            f'(geometry_msgs/msg/PoseStamped); '
            f'max_pose_time_diff_sec={self._max_pose_time_diff}')
        self.get_logger().info(
            f"self_filter={values['self_filter']}; "
            f"joint_states={values['joint_states_topic']}; "
            f"missing pose frame={self._pose_frame}; padding={values['self_filter_padding']}m; "
            f"scale={values['self_filter_scale']}")
        self.get_logger().info(
            f"base->CAD origin: xyz={values['model_origin_xyz']}m "
            f"rpy={values['model_origin_rpy']}rad")
        self.get_logger().info(
            f"TF input: /tf_static; use_dynamic_tf={values['use_dynamic_tf']}")
        if self._header_publisher:
            self.get_logger().info(
                f'Mesh coordinates use the input frame; metadata topic: '
                f'{self._header_publisher.topic_name}')
        self.get_logger().info('Ready for point clouds.')

    def _publish_result(self, result, header):
        self._publisher.publish(result)
        if self._header_publisher:
            self._header_publisher.publish(deepcopy(header))
        if self._marker_publisher:
            self._marker_publisher.publish(mesh_marker(result, header))

    def _receive(self, message):
        with self._lock:
            self._latest_cloud = message

    def _receive_pose(self, message):
        with self._lock:
            self._state_history.add_pose(message)

    def _receive_joints(self, message):
        try:
            with self._lock:
                self._state_history.add_joints(message)
        except ValueError as error:
            self._warn_pose(f'Invalid JointState: {error}')

    def _warn_pose(self, message):
        now = time.monotonic()
        if self._last_pose_warning is None or now - self._last_pose_warning >= 10.0:
            self.get_logger().warning(message)
            self._last_pose_warning = now

    def _process_latest(self):
        with self._lock:
            message = self._latest_cloud
        if message is None:
            return
        started = time.monotonic()
        try:
            with self._lock:
                timestamp = stamp_ns(message.header.stamp)
                pose = self._state_history.pose_at(
                    timestamp, self._max_pose_time_diff, allow_unstamped=not self._self_filter)
                joints = (self._state_history.joints_at(
                    timestamp, self._self_filter.required_joints, self._max_joint_time_diff)
                    if self._self_filter else {})
            if self._self_filter:
                cloud_from_base = pose_matrix_in_cloud_frame(
                    pose, message.header, self._tf_buffer, self._max_pose_time_diff,
                    self._pose_frame)
            else:
                pose = deepcopy(pose)
                if not pose.header.frame_id:
                    pose.header.frame_id = self._pose_frame
                cloud_from_base = np.eye(4)
                cloud_from_base[:3, 3] = position_in_cloud_frame(
                    pose, message.header, self._tf_buffer, self._max_pose_time_diff)
            center = cloud_from_base[:3, 3]
            cloud_from_model = cloud_from_base @ self._base_from_model
        except (ValueError, TransformException) as error:
            self._warn_pose(f'Cannot resolve machine state; output skipped: {error}')
            return
        with self._lock:
            if self._latest_cloud is message:
                self._latest_cloud = None
        try:
            xyz, records = read_cloud(message)
            self_mask = None
            if self._self_filter:
                self_mask = np.zeros(len(xyz), dtype=bool)
                candidates = np.flatnonzero(
                    np.isfinite(xyz).all(axis=1)
                    & (np.hypot(xyz[:, 0] - center[0], xyz[:, 1] - center[1])
                       <= self._parameters_config.radius_m))
                self_mask[candidates] = self._self_filter.mask(
                    xyz[candidates], cloud_from_model, joints)
            indices, stats = filter_indices(
                xyz, center[:2], self._parameters_config, self_mask)
            if self._mesh_converter:
                excluded = (lambda points: self._self_filter.mask(
                    points, cloud_from_model, joints)) if self._self_filter else None
                result, mesh_stats = self._mesh_converter.convert(
                    xyz[indices], excluded=excluded,
                    exclusion_step=max(0.01, min(0.075, self._self_filter.padding / 2))
                    if self._self_filter else 0.075)
                stats.update(mesh_stats)
            else:
                result = select_cloud(message, records, indices)
        except (ValueError, TypeError, AssertionError, KeyError, RuntimeError) as error:
            self.get_logger().error(f'Cannot filter PointCloud2: {error}')
            if self._mesh_converter:
                # Clear a previous surface if this frame could not be reconstructed.
                self._publish_result(Mesh(), message.header)
            return
        self._publish_result(result, message.header)
        if self._removed_publisher:
            self._removed_publisher.publish(select_cloud(
                message, records, np.flatnonzero(self_mask)))
        elapsed = time.monotonic() - started
        mesh_summary = (
            f" mesh_sampled={stats['mesh_sampled']} vertices={stats['vertices']} "
            f"triangles={stats['triangles']};" if self._mesh_converter else '')
        self.get_logger().info(
            f"points: input={stats['input']} finite={stats['finite']} "
            f"radius={stats['radius']} self_removed={stats['self_removed']} "
            f"voxel={stats['voxel']} "
            f"sor={stats['sor']} output={stats['output']}; "
            f'center_xy=({center[0]:.3f}, {center[1]:.3f}); '
            f'{mesh_summary} processing={elapsed:.3f}s')
        if not len(indices):
            self.get_logger().warning(
                'No retained points within radius; published an empty output. '
                'Check radius_m, global_pose and outlier parameters.')
        elif self._mesh_converter and not len(result.triangles):
            self.get_logger().warning(
                'No reconstructable surface; published an empty Mesh. '
                'Check point spacing and mesh_resolution.')
        if elapsed > self._period:
            self.get_logger().warning(
                'Processing exceeded the publication period; increase voxel_size '
                'or publish_period_sec, or lower mesh_resolution in mesh mode.')


def main(args=None):
    """Run subscription and filtering callbacks on separate executor threads."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = PointCloudFilterNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            # Launch may forward a second SIGINT while cleanup is in progress.
            pass
        except RuntimeError:
            # Humble's signal handler can shut down between the native checks.
            if rclpy.ok():
                raise
