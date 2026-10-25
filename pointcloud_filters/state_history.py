"""Keep time-indexed machine states, including partial JointState messages."""

from bisect import bisect_left
from collections import deque
import math


def stamp_ns(stamp):
    """Convert a ROS stamp to integer nanoseconds without floating point loss."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class StateHistory:
    """
    Resolve the nearest measured state for each cloud, never a default angle.

    Histories are bounded by measurement time. A backward jump exceeding one
    second resets the history so rosbag loops cannot reuse the previous pass.
    Callers serialize access while subscribers append and filtering snapshots.
    """

    def __init__(self, duration_sec=30.0):
        """Configure the retained interval in seconds."""
        if not math.isfinite(duration_sec) or duration_sec <= 0:
            raise ValueError('state_history_sec must be finite and > 0')
        self._duration_ns = int(duration_sec * 1e9)
        self._poses = deque()
        self._joints = {}
        self._latest_pose = None
        self._latest_joint = None

    def _append(self, history, timestamp, value):
        if history and timestamp < history[-1][0]:
            # Minor transport reordering is retained in timestamp order.
            entries = list(history)
            index = bisect_left([entry[0] for entry in entries], timestamp)
            entries.insert(index, (timestamp, value))
            history.clear()
            history.extend(entries)
        else:
            history.append((timestamp, value))
        cutoff = history[-1][0] - self._duration_ns
        while history and history[0][0] < cutoff:
            history.popleft()

    def add_pose(self, pose):
        """Retain a PoseStamped with its original measurement timestamp."""
        timestamp = stamp_ns(pose.header.stamp)
        if self._latest_pose is not None and timestamp < self._latest_pose - 1_000_000_000:
            self._poses.clear()
        self._append(self._poses, timestamp, pose)
        self._latest_pose = max(timestamp, self._poses[-1][0])

    def add_joints(self, message):
        """Merge partial joint updates while tracking every joint's own timestamp."""
        if len(message.name) != len(message.position):
            raise ValueError('JointState name/position lengths differ')
        if len(set(message.name)) != len(message.name):
            raise ValueError('JointState contains duplicate names')
        if not all(math.isfinite(value) for value in message.position):
            raise ValueError('JointState positions must be finite')
        timestamp = stamp_ns(message.header.stamp)
        if self._latest_joint is not None and timestamp < self._latest_joint - 1_000_000_000:
            self._joints.clear()
        for name, value in zip(message.name, message.position):
            history = self._joints.setdefault(name, deque())
            self._append(history, timestamp, value)
        self._latest_joint = max(timestamp, self._latest_joint or timestamp)
        # A loop reset must also reset the comparison timestamp.
        if self._latest_joint - timestamp > 1_000_000_000:
            self._latest_joint = timestamp

    @staticmethod
    def _nearest(history, timestamp, tolerance, label, allow_unstamped=False):
        if not history:
            raise ValueError(f'Waiting for {label}')
        if allow_unstamped and timestamp == 0:
            return history[-1][1]
        times = [entry[0] for entry in history]
        index = bisect_left(times, timestamp)
        candidates = list(history)[max(0, index - 1):index + 1]
        chosen_time, value = min(candidates, key=lambda entry: abs(entry[0] - timestamp))
        difference = abs(chosen_time - timestamp) * 1e-9
        if tolerance > 0 and difference > tolerance and not (allow_unstamped and chosen_time == 0):
            raise ValueError(f'{label}/cloud stamp difference {difference:.3f}s '
                             f'exceeds {tolerance:.3f}s')
        return value

    def pose_at(self, timestamp, tolerance, allow_unstamped=False):
        """Return the nearest actual pose within the configured time tolerance."""
        return self._nearest(self._poses, timestamp, tolerance, 'global_pose', allow_unstamped)

    def joints_at(self, timestamp, names, tolerance):
        """Return all required joints, rejecting missing or stale partial updates."""
        return {name: self._nearest(self._joints.get(name), timestamp, tolerance,
                                    f'joint {name}') for name in names}
