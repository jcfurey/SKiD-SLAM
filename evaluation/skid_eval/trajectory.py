"""Trajectories in the TUM convention, and time association between them.

TUM format: one pose per line, ``timestamp tx ty tz qx qy qz qw``, timestamps
in seconds, translation in metres, quaternion in (x, y, z, w) order and
normalised. Comment lines start with ``#``. That convention is used for both
ground truth and estimates so the two are never accidentally mismatched.
"""

import bisect
import math

from . import linalg


class Pose:
    """A rigid transform with a timestamp."""

    __slots__ = ("time", "translation", "rotation")

    def __init__(self, time, translation, rotation):
        self.time = float(time)
        self.translation = tuple(float(value) for value in translation)
        self.rotation = tuple(tuple(float(v) for v in row) for row in rotation)

    @classmethod
    def from_quaternion(cls, time, translation, quaternion):
        return cls(time, translation, linalg.quaternion_to_matrix(quaternion))

    def inverse(self):
        rotation = linalg.transpose(self.rotation)
        translation = linalg.scale(
            linalg.matvec(rotation, self.translation), -1.0)
        return Pose(self.time, translation, rotation)

    def compose(self, other):
        """self * other, keeping self's timestamp."""
        rotation = linalg.matmul(self.rotation, other.rotation)
        translation = linalg.add(
            linalg.matvec(self.rotation, other.translation), self.translation)
        return Pose(self.time, translation, rotation)

    def between(self, other):
        """The transform taking self's frame to other's: self^-1 * other."""
        return self.inverse().compose(other)

    def quaternion(self):
        return linalg.matrix_to_quaternion(self.rotation)


class Trajectory:
    """A time-ordered sequence of poses."""

    def __init__(self, poses):
        self.poses = sorted(poses, key=lambda pose: pose.time)
        self._times = [pose.time for pose in self.poses]

    def __len__(self):
        return len(self.poses)

    def __iter__(self):
        return iter(self.poses)

    def __getitem__(self, index):
        return self.poses[index]

    @property
    def times(self):
        return list(self._times)

    def duration(self):
        if len(self.poses) < 2:
            return 0.0
        return self._times[-1] - self._times[0]

    def path_length(self):
        total = 0.0
        for previous, current in zip(self.poses, self.poses[1:]):
            total += linalg.norm(
                linalg.subtract(current.translation, previous.translation))
        return total

    def nearest(self, time, max_difference):
        """The pose closest in time, or None when nothing is close enough."""
        if not self.poses:
            return None
        index = bisect.bisect_left(self._times, time)
        candidates = []
        if index < len(self.poses):
            candidates.append(self.poses[index])
        if index > 0:
            candidates.append(self.poses[index - 1])
        best = min(candidates, key=lambda pose: abs(pose.time - time))
        if abs(best.time - time) > max_difference:
            return None
        return best


def parse_tum(lines):
    """Parse TUM-format lines into a Trajectory.

    Raises ValueError naming the offending line, because a silently skipped
    malformed row turns into a quietly wrong metric.
    """
    poses = []
    for number, raw in enumerate(lines, start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(
                f"line {number}: expected 8 fields "
                f"(timestamp tx ty tz qx qy qz qw), found {len(fields)}")
        try:
            values = [float(field) for field in fields]
        except ValueError as error:
            raise ValueError(f"line {number}: {error}") from error

        quaternion = tuple(values[4:8])
        if math.sqrt(sum(v * v for v in quaternion)) < 1e-9:
            raise ValueError(f"line {number}: quaternion has zero norm")
        poses.append(
            Pose.from_quaternion(values[0], tuple(values[1:4]), quaternion))

    if not poses:
        raise ValueError("no poses found")
    return Trajectory(poses)


def read_tum(path):
    with open(path, "r", encoding="utf-8") as handle:
        try:
            return parse_tum(handle)
        except ValueError as error:
            raise ValueError(f"{path}: {error}") from error


def write_tum(path, trajectory):
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("# timestamp tx ty tz qx qy qz qw\n")
        for pose in trajectory:
            x, y, z, w = pose.quaternion()
            handle.write(
                f"{pose.time:.9f} "
                f"{pose.translation[0]:.9f} {pose.translation[1]:.9f} "
                f"{pose.translation[2]:.9f} "
                f"{x:.9f} {y:.9f} {z:.9f} {w:.9f}\n")


def associate(estimate, reference, max_time_difference=0.02):
    """Pair each estimated pose with the nearest reference pose in time.

    Each reference pose is used at most once, so a stalled estimate cannot
    inflate its own score by matching the same reference repeatedly.
    """
    pairs = []
    used = set()
    for pose in estimate:
        match = reference.nearest(pose.time, max_time_difference)
        if match is None or match.time in used:
            continue
        used.add(match.time)
        pairs.append((pose, match))
    return pairs
