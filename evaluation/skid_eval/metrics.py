"""Trajectory and registration error metrics."""

import math

from . import linalg
from .alignment import align_trajectories


class Statistics:
    """Summary of an error sample."""

    def __init__(self, values, unit):
        self.values = list(values)
        self.unit = unit

    def __len__(self):
        return len(self.values)

    @property
    def count(self):
        return len(self.values)

    @property
    def rmse(self):
        if not self.values:
            return float("nan")
        return math.sqrt(
            sum(value * value for value in self.values) / len(self.values))

    @property
    def mean(self):
        if not self.values:
            return float("nan")
        return sum(self.values) / len(self.values)

    @property
    def median(self):
        if not self.values:
            return float("nan")
        ordered = sorted(self.values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])

    @property
    def std(self):
        if len(self.values) < 2:
            return float("nan")
        mean = self.mean
        variance = sum((value - mean) ** 2 for value in self.values)
        return math.sqrt(variance / (len(self.values) - 1))

    @property
    def minimum(self):
        return min(self.values) if self.values else float("nan")

    @property
    def maximum(self):
        return max(self.values) if self.values else float("nan")

    def percentile(self, fraction):
        """Linear-interpolated percentile; fraction in [0, 1]."""
        if not self.values:
            return float("nan")
        ordered = sorted(self.values)
        if len(ordered) == 1:
            return ordered[0]
        position = fraction * (len(ordered) - 1)
        lower = int(math.floor(position))
        upper = min(lower + 1, len(ordered) - 1)
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    def as_dict(self):
        return {
            "count": self.count,
            "unit": self.unit,
            "rmse": self.rmse,
            "mean": self.mean,
            "median": self.median,
            "std": self.std,
            "min": self.minimum,
            "max": self.maximum,
            "p95": self.percentile(0.95),
        }


def absolute_trajectory_error(pairs, alignment=None, estimate_scale=False):
    """ATE and ARE for associated (estimate, reference) pose pairs.

    The estimate is expressed in the reference frame first, because an
    absolute error measured in the wrong frame measures the frame, not the
    trajectory. Pass an alignment to reuse one, or let it be solved here.
    """
    if len(pairs) < 3:
        raise ValueError(
            f"need at least 3 associated poses, got {len(pairs)}")
    if alignment is None:
        alignment = align_trajectories(pairs, estimate_scale=estimate_scale)

    translation_errors = []
    rotation_errors = []
    for estimate, reference in pairs:
        aligned_translation = alignment.apply(estimate.translation)
        translation_errors.append(
            linalg.norm(
                linalg.subtract(aligned_translation, reference.translation)))

        aligned_rotation = alignment.apply_rotation(estimate.rotation)
        difference = linalg.matmul(
            linalg.transpose(aligned_rotation), reference.rotation)
        rotation_errors.append(math.degrees(linalg.rotation_angle(difference)))

    return {
        "alignment": alignment,
        "translation": Statistics(translation_errors, "m"),
        "rotation": Statistics(rotation_errors, "deg"),
    }


def relative_pose_error(pairs, delta=1):
    """RPE over a fixed index step.

    Alignment-free by construction: both sides are relative motions, so any
    rigid transform between the two frames cancels. That is what makes RPE the
    right drift metric and ATE the right global-consistency metric.
    """
    if delta < 1:
        raise ValueError("delta must be at least 1")
    if len(pairs) <= delta:
        raise ValueError(
            f"need more than delta={delta} associated poses, got {len(pairs)}")

    translation_errors = []
    rotation_errors = []
    for index in range(len(pairs) - delta):
        estimate_from, reference_from = pairs[index]
        estimate_to, reference_to = pairs[index + delta]

        estimated_motion = estimate_from.between(estimate_to)
        reference_motion = reference_from.between(reference_to)
        error = estimated_motion.inverse().compose(reference_motion)

        translation_errors.append(linalg.norm(error.translation))
        rotation_errors.append(math.degrees(linalg.rotation_angle(error.rotation)))

    return {
        "translation": Statistics(translation_errors, "m"),
        "rotation": Statistics(rotation_errors, "deg"),
    }


def registration_errors(measurements):
    """Per-loop RTE and RRE.

    `measurements` is a sequence of (estimated, reference) relative Poses: what
    the registration reported for a loop, and what the ground truth says that
    relative transform actually was.
    """
    translation_errors = []
    rotation_errors = []
    for estimated, reference in measurements:
        error = estimated.inverse().compose(reference)
        translation_errors.append(linalg.norm(error.translation))
        rotation_errors.append(math.degrees(linalg.rotation_angle(error.rotation)))

    return {
        "translation": Statistics(translation_errors, "m"),
        "rotation": Statistics(rotation_errors, "deg"),
    }


def success_rate(errors, translation_threshold_m, rotation_threshold_deg):
    """Fraction of registrations inside both thresholds.

    A registration counts only when translation and rotation are both
    acceptable: a loop with the right position and the wrong heading is not a
    successful registration.
    """
    translations = errors["translation"].values
    rotations = errors["rotation"].values
    if len(translations) != len(rotations):
        raise ValueError("translation and rotation samples must correspond")
    if not translations:
        return {"successes": 0, "total": 0, "rate": float("nan")}

    successes = sum(
        1 for t, r in zip(translations, rotations)
        if t <= translation_threshold_m and r <= rotation_threshold_deg)
    return {
        "successes": successes,
        "total": len(translations),
        "rate": successes / len(translations),
        "translation_threshold_m": translation_threshold_m,
        "rotation_threshold_deg": rotation_threshold_deg,
    }
