"""Convert LoopDiagnostic messages into the evaluation harness CSV rows.

This module deliberately has no ROS imports. The bag-facing executable handles
deserialization; keeping row conversion here makes the transform convention
and filtering rules testable with ordinary Python objects.
"""

import csv
import math
import pathlib


CANDIDATE_FIELDS = (
    "query_time", "match_time", "score",
    "query_robot_id", "query_keyframe_index",
    "match_robot_id", "match_keyframe_index",
    "candidate_rank", "sector_shift", "decision", "reason",
)

REGISTRATION_FIELDS = (
    "query_time", "match_time", "tx", "ty", "tz",
    "qx", "qy", "qz", "qw",
    "query_robot_id", "query_keyframe_index",
    "match_robot_id", "match_keyframe_index",
    "candidate_rank", "score", "sector_shift",
    "registration_status", "registration_detail",
    "source_points", "target_points", "coarse_correspondences",
    "coarse_rotation_inliers", "coarse_translation_inliers",
    "fine_inliers", "fine_converged", "fine_iterations", "fine_error",
    "metric_inliers", "overlap_ratio", "truncated_mse_m2",
    "uncertainty_variance_scale", "uncertainty_condition_number",
    "uncertainty_clamped_modes", "coarse_time_ms", "fine_time_ms",
    "metric_time_ms",
)


def candidate_row(message):
    """Return one descriptor candidate row, or None for another stage.

    Rejected descriptor candidates are intentional inputs to a threshold sweep.
    Position-search fallbacks carry a NaN rather than inventing a descriptor
    score and are excluded.
    """
    if message.stage != "descriptor":
        return None
    score = float(message.descriptor_distance)
    if math.isnan(score):
        return None
    return {
        "query_time": float(message.query_time),
        "match_time": float(message.match_time),
        "score": score,
        "query_robot_id": message.query_robot_id,
        "query_keyframe_index": int(message.query_keyframe_index),
        "match_robot_id": message.match_robot_id,
        "match_keyframe_index": int(message.match_keyframe_index),
        "candidate_rank": int(message.candidate_rank),
        "sector_shift": int(message.sector_shift),
        "decision": message.decision,
        "reason": message.reason,
    }


def registration_row(message):
    """Return an accepted registration row in match -> query convention."""
    if message.stage != "registration" or not message.registration_accepted:
        return None
    pose = message.relative_pose.pose
    position = pose.position
    orientation = pose.orientation
    return {
        "query_time": float(message.query_time),
        "match_time": float(message.match_time),
        "tx": float(position.x),
        "ty": float(position.y),
        "tz": float(position.z),
        "qx": float(orientation.x),
        "qy": float(orientation.y),
        "qz": float(orientation.z),
        "qw": float(orientation.w),
        "query_robot_id": message.query_robot_id,
        "query_keyframe_index": int(message.query_keyframe_index),
        "match_robot_id": message.match_robot_id,
        "match_keyframe_index": int(message.match_keyframe_index),
        "candidate_rank": int(message.candidate_rank),
        "score": float(message.descriptor_distance),
        "sector_shift": int(message.sector_shift),
        "registration_status": message.registration_status,
        "registration_detail": message.registration_detail,
        "source_points": int(message.source_points),
        "target_points": int(message.target_points),
        "coarse_correspondences": int(message.coarse_correspondences),
        "coarse_rotation_inliers": int(message.coarse_rotation_inliers),
        "coarse_translation_inliers": int(message.coarse_translation_inliers),
        "fine_inliers": int(message.fine_inliers),
        "fine_converged": bool(message.fine_converged),
        "fine_iterations": int(message.fine_iterations),
        "fine_error": float(message.fine_error),
        "metric_inliers": int(message.metric_inliers),
        "overlap_ratio": float(message.overlap_ratio),
        "truncated_mse_m2": float(message.truncated_mse_m2),
        "uncertainty_variance_scale":
            float(message.uncertainty_variance_scale),
        "uncertainty_condition_number":
            float(message.uncertainty_condition_number),
        "uncertainty_clamped_modes": int(message.uncertainty_clamped_modes),
        "coarse_time_ms": float(message.coarse_time_ms),
        "fine_time_ms": float(message.fine_time_ms),
        "metric_time_ms": float(message.metric_time_ms),
    }


def rows_from_messages(messages):
    """Return (candidate rows, accepted-registration rows)."""
    candidates = []
    registrations = []
    for message in messages:
        candidate = candidate_row(message)
        if candidate is not None:
            candidates.append(candidate)
        registration = registration_row(message)
        if registration is not None:
            registrations.append(registration)
    return candidates, registrations


def write_rows(path, fieldnames, rows):
    """Write a deterministic, header-bearing CSV and return its row count."""
    output = pathlib.Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)
