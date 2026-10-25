"""Subscribe only to the TF streams required by the configured filter."""

import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class FilterTransformListener:
    """Keep static field coordinates independent of unrelated dynamic robot TF."""

    def __init__(self, buffer, node, use_dynamic_tf=False):
        """Subscribe to static TF, and opt in to dynamic TF only when needed."""
        self.buffer = buffer
        self.node = node
        self._lock = threading.RLock()
        self._group = ReentrantCallbackGroup()
        self.tf_static_sub = node.create_subscription(
            TFMessage, '/tf_static', self.static_callback,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._group)
        self.tf_sub = None
        self._jump_handle = None
        if use_dynamic_tf:
            self.tf_sub = node.create_subscription(
                TFMessage, '/tf', self.callback,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE),
                callback_group=self._group)
            self._jump_handle = node.get_clock().create_jump_callback(
                JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                              on_clock_change=True),
                post_callback=self._clock_jump)

    def static_callback(self, message):
        """Keep the static world/map relation without accepting robot motion TF."""
        with self._lock:
            for transform in message.transforms:
                self.buffer.set_transform_static(transform, 'pointcloud_filter_static')

    def callback(self, message):
        """Accept dynamic coordinate transforms only in the explicitly enabled mode."""
        with self._lock:
            for transform in message.transforms:
                self.buffer.set_transform(transform, 'pointcloud_filter_dynamic')

    def _clock_jump(self, _jump):
        # BufferCore.clear() discards dynamic history while preserving static TF.
        with self._lock:
            self.buffer.clear()
        self.node.get_logger().info('ROS clock changed; cleared dynamic TF history.')
