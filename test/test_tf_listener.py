"""Regress TF bag rewind handling using a real buffer without DDS transport."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from rclpy.clock import ClockChange, TimeJump
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException

from pointcloud_filters.tf_listener import FilterTransformListener


class _FakeClock:
    """Capture the same jump callback registered on a real ROS node clock."""

    def __init__(self):
        """Retain callbacks without requiring a ROS context or /clock publisher."""
        self.registrations = []

    def create_jump_callback(self, threshold, *, pre_callback=None, post_callback=None):
        """Provide a removable handle matching the rclpy Clock API."""
        handle = SimpleNamespace(threshold=threshold, pre_callback=pre_callback,
                                 post_callback=post_callback, active=True)
        handle.unregister = lambda: setattr(handle, 'active', False)
        self.registrations.append(handle)
        return handle

    def rewind(self, seconds):
        """Deliver a real TimeJump object as rclpy does after a backward clock jump."""
        jump = TimeJump(ClockChange.ROS_TIME_NO_CHANGE, Duration(seconds=-seconds))
        for handle in self.registrations:
            if handle.active and handle.pre_callback:
                handle.pre_callback()
            if handle.active and handle.post_callback:
                handle.post_callback(jump)


class _FakeNode:
    """Record topic registration and manually deliver only subscribed messages."""

    def __init__(self):
        """Use plain handles and a fake clock to isolate listener behavior."""
        self.subscriptions = []
        self.clock = _FakeClock()
        self.messages = []

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        """Preserve QoS and callback group so tests can inspect actual registration."""
        subscription = SimpleNamespace(message_type=message_type, topic=topic,
                                       callback=callback, qos=qos, **kwargs)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        """Match node teardown without making transport calls."""
        self.subscriptions.remove(subscription)
        return True

    def get_clock(self):
        """Expose the fake clock used by the listener's jump handler."""
        return self.clock

    def get_logger(self):
        """Capture diagnostics without prescribing their wording."""
        return SimpleNamespace(warning=self.messages.append, warn=self.messages.append,
                               info=self.messages.append, debug=self.messages.append,
                               error=self.messages.append)

    def dispatch(self, topic, message):
        """Deliver exactly once per registered subscription and return the count."""
        matching = [subscription for subscription in self.subscriptions
                    if subscription.topic == topic]
        for subscription in matching:
            subscription.callback(message)
        return len(matching)


def _transform(parent='map', child='base_link_3d', seconds=100, xyz=(0., 0., 0.), yaw=0.):
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = parent, child
    transform.header.stamp.sec = seconds
    (transform.transform.translation.x, transform.transform.translation.y,
     transform.transform.translation.z) = xyz
    transform.transform.rotation.z = math.sin(yaw / 2)
    transform.transform.rotation.w = math.cos(yaw / 2)
    return transform


def _translation(transform):
    return [transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z]


def test_default_listener_registers_only_retained_static_tf():
    """An ordinary launch must never subscribe to the bag's unrelated robot /tf."""
    node, buffer = _FakeNode(), Buffer()
    listener = FilterTransformListener(buffer, node)
    assert listener is not None
    assert [subscription.topic for subscription in node.subscriptions] == ['/tf_static']
    subscription = node.subscriptions[0]
    assert subscription.message_type is TFMessage
    assert subscription.qos.depth == 100
    assert subscription.qos.history == HistoryPolicy.KEEP_LAST
    assert subscription.qos.reliability == ReliabilityPolicy.RELIABLE
    assert subscription.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_static_world_map_inverse_remains_valid_at_any_recording_time():
    """Resolve the actual GNSS/world relationship with inverse rotation and translation."""
    node, buffer = _FakeNode(), Buffer()
    listener = FilterTransformListener(buffer, node)
    transform = _transform('world', 'map', 200, (21395.1661, 14034.4613, 28.558), math.pi / 2)
    assert node.dispatch('/tf_static', TFMessage(transforms=[transform])) == 1
    for seconds in (0, 100, 200, 1792911725):
        inverse = buffer.lookup_transform('map', 'world', Time(seconds=seconds))
        np.testing.assert_allclose(_translation(inverse),
                                   [-14034.4613, 21395.1661, -28.558], atol=1e-8)
    assert listener is not None


def test_default_listener_ignores_future_then_past_base_transforms():
    """Replaying old bag TF cannot poison a static-only buffer or emit old-data errors."""
    node, buffer = _FakeNode(), Buffer()
    listener = FilterTransformListener(buffer, node, use_dynamic_tf=False)
    node.dispatch('/tf_static', TFMessage(transforms=[
        _transform('world', 'map', 200, (100., 200., 30.))]))
    for seconds in (200, 201, 100, 101, 200, 100):
        assert node.dispatch('/tf', TFMessage(transforms=[
            _transform(seconds=seconds, xyz=(20., 21., 1.2))])) == 0
    assert 'base_link_3d' not in buffer.all_frames_as_yaml()
    with pytest.raises(TransformException):
        buffer.lookup_transform('map', 'base_link_3d', Time())
    np.testing.assert_allclose(_translation(
        buffer.lookup_transform('map', 'world', Time(seconds=100))), [-100., -200., -30.])
    assert listener is not None


def test_opt_in_dynamic_tf_preserves_measurement_time_interpolation():
    """An explicitly enabled dynamic frame still interpolates at the cloud timestamp."""
    node, buffer = _FakeNode(), Buffer()
    listener = FilterTransformListener(buffer, node, use_dynamic_tf=True)
    assert {subscription.topic for subscription in node.subscriptions} == {'/tf_static', '/tf'}
    node.dispatch('/tf_static', TFMessage(transforms=[
        _transform('world', 'map', 100, (100., 200., 30.))]))
    for seconds, offset in ((100, 0.), (110, 10.)):
        assert node.dispatch('/tf', TFMessage(transforms=[
            _transform(seconds=seconds, xyz=(offset, 0., 1.2))])) == 1
    result = buffer.lookup_transform('world', 'base_link_3d', Time(seconds=105))
    np.testing.assert_allclose(_translation(result), [105., 200., 31.2])
    assert listener is not None


def test_clock_rewind_clears_dynamic_history_and_restores_all_static_frames():
    """A new bag loop accepts older measurements while static latched TF survives."""
    node, buffer = _FakeNode(), Buffer()
    listener = FilterTransformListener(buffer, node, use_dynamic_tf=True)
    node.dispatch('/tf_static', TFMessage(transforms=[
        _transform('world', 'map', 200, (100., 200., 30.)),
        _transform('map', 'field_origin', 200, (2., 3., 4.))]))
    node.dispatch('/tf', TFMessage(transforms=[
        _transform(seconds=200, xyz=(20., 0., 1.2))]))
    np.testing.assert_allclose(_translation(
        buffer.lookup_transform('world', 'base_link_3d', Time(seconds=200))), [120., 200., 31.2])
    assert any(handle.post_callback and handle.threshold.min_backward is not None
               and handle.threshold.min_backward.nanoseconds < 0
               for handle in node.clock.registrations)
    node.clock.rewind(100)
    with pytest.raises(TransformException):
        buffer.lookup_transform('map', 'base_link_3d', Time(seconds=200))
    np.testing.assert_allclose(_translation(
        buffer.lookup_transform('world', 'field_origin', Time(seconds=100))), [102., 203., 34.])
    node.dispatch('/tf', TFMessage(transforms=[
        _transform(seconds=100, xyz=(1., 0., 1.2))]))
    np.testing.assert_allclose(_translation(
        buffer.lookup_transform('world', 'base_link_3d', Time(seconds=100))), [101., 200., 31.2])
    assert listener is not None


def test_clock_rewind_restores_latest_static_value_for_each_child():
    """Keep updated static calibration, including frames from separate TF messages."""
    node, buffer = _FakeNode(), Buffer()
    listener = FilterTransformListener(buffer, node, use_dynamic_tf=True)
    node.dispatch('/tf_static', TFMessage(transforms=[
        _transform('world', 'map', 200, (100., 200., 30.))]))
    node.dispatch('/tf_static', TFMessage(transforms=[
        _transform('map', 'field_origin', 200, (2., 3., 4.))]))
    node.dispatch('/tf_static', TFMessage(transforms=[
        _transform('world', 'map', 201, (101., 202., 30.))]))
    node.clock.rewind(101)
    np.testing.assert_allclose(_translation(
        buffer.lookup_transform('world', 'field_origin', Time(seconds=100))), [103., 205., 34.])
    assert listener is not None
