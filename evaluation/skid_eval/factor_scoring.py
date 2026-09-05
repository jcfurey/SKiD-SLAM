"""Audit symmetric loop-factor delivery and score factors against TUM poses.

The ROS bag reader lives in ``score_loop_factors_from_bag.py``.  This module
contains only message-independent policy and geometry so the important parts
remain deterministic and unit testable without a ROS installation.
"""

import bisect
import collections
import math
from dataclasses import dataclass

from . import linalg
from .metrics import Statistics


@dataclass(frozen=True, order=True)
class Endpoint:
    robot_id: str
    keyframe_index: int
    trajectory_epoch: int = 0

    def label(self):
        suffix = f"@{self.trajectory_epoch}" if self.trajectory_epoch else ""
        return f"{self.robot_id}/{self.keyframe_index}{suffix}"


@dataclass(frozen=True)
class Factor:
    recipient: str
    from_endpoint: Endpoint
    to_endpoint: Endpoint
    relative_pose: object
    authority_id: str = ""
    authority_epoch: int = 0
    revision: int = 0
    retracted: bool = False
    covariance: tuple = ()


@dataclass(frozen=True)
class EndpointTimestamp:
    endpoint: Endpoint
    time: float


def canonical_factor(factor):
    """Return ``((low_endpoint, high_endpoint), low_from_high_pose)``."""
    if factor.from_endpoint <= factor.to_endpoint:
        endpoints = (factor.from_endpoint, factor.to_endpoint)
        pose = factor.relative_pose
    else:
        endpoints = (factor.to_endpoint, factor.from_endpoint)
        pose = factor.relative_pose.inverse()
    return endpoints, pose


def factor_label(identity):
    return f"{identity[0].label()} <-> {identity[1].label()}"


def pose_difference(first, second):
    """Translation and rotation separating two estimates of one transform."""
    difference = first.inverse().compose(second)
    return (
        linalg.norm(difference.translation),
        math.degrees(linalg.rotation_angle(difference.rotation)),
    )


def index_factor_deliveries(factors, translation_tolerance_m=1.0e-6,
                            rotation_tolerance_deg=1.0e-5):
    """Canonicalize one topic and report duplicate measurement conflicts."""
    indexed = {}
    duplicate_count = 0
    duplicate_conflicts = []
    for factor in factors:
        identity, pose = canonical_factor(factor)
        previous = indexed.get(identity)
        if previous is None:
            indexed[identity] = pose
            continue
        duplicate_count += 1
        translation, rotation = pose_difference(previous, pose)
        if (translation > translation_tolerance_m or
                rotation > rotation_tolerance_deg):
            duplicate_conflicts.append({
                "factor": factor_label(identity),
                "translation_difference_m": translation,
                "rotation_difference_deg": rotation,
            })
    return indexed, duplicate_count, duplicate_conflicts


def resolve_factor_revisions(factors, translation_tolerance_m=1e-6,
                             rotation_tolerance_deg=1e-5):
    """Resolve final graph state while retaining evidence of conflicting replays.

    Legacy messages remain strict: duplicates without a revision are errors.
    Modern messages use per-authority generations and per-pair revisions.
    """
    epochs = {}
    owner_epochs = {}
    for factor in factors:
        if factor.revision:
            epochs[factor.authority_id] = max(
                epochs.get(factor.authority_id, 0), factor.authority_epoch)
            for endpoint in (factor.from_endpoint, factor.to_endpoint):
                owner_epochs[endpoint.robot_id] = max(
                    owner_epochs.get(endpoint.robot_id, 0), endpoint.trajectory_epoch)
    latest = {}
    seen = {}
    legacy = []
    conflicts = []
    replays = 0
    for factor in factors:
        if not factor.revision:
            legacy.append(factor)
            continue
        identity, pose = canonical_factor(factor)
        claim = identity, factor.authority_id
        version = claim, factor.authority_epoch, factor.revision
        previous = seen.get(version)
        if previous is not None:
            replays += 1
            _, previous_pose = canonical_factor(previous)
            translation, rotation = pose_difference(previous_pose, pose)
            if (factor.retracted != previous.retracted or
                    translation > translation_tolerance_m or
                    rotation > rotation_tolerance_deg or
                    factor.covariance != previous.covariance):
                conflicts.append({"factor": factor_label(identity),
                                  "reason": "conflicting_revision_payload"})
        else:
            seen[version] = factor
        if factor.authority_epoch != epochs[factor.authority_id]:
            continue
        if any(endpoint.trajectory_epoch != owner_epochs[endpoint.robot_id]
               for endpoint in (factor.from_endpoint, factor.to_endpoint)):
            continue
        previous = latest.get(claim)
        if previous is None or factor.revision > previous.revision:
            latest[claim] = factor
    active = {}
    for (identity, authority), factor in sorted(latest.items()):
        if not factor.retracted and identity not in active:
            active[identity] = factor
    modern_ids = {claim[0] for claim in latest}
    resolved = [factor for factor in legacy
                if canonical_factor(factor)[0] not in modern_ids] + list(active.values())
    versions = [(factor_label(identity), authority, factor.authority_epoch,
                 factor.revision, factor.retracted, factor.covariance)
                for (identity, authority), factor in sorted(latest.items())]
    return resolved, replays, conflicts, versions


def compare_factor_deliveries(deliveries, translation_tolerance_m=1.0e-6,
                              rotation_tolerance_deg=1.0e-5):
    """Verify that every topic received the same oriented measurements."""
    if len(deliveries) < 2:
        raise ValueError("at least two factor topics are required")

    indexed = {}
    topic_summary = {}
    duplicate_conflicts = []
    duplicate_messages = 0
    replay_messages = 0
    revision_states = {}
    recipients_by_factor = collections.defaultdict(set)
    recipient_errors = []
    for topic, raw_factors in sorted(deliveries.items()):
        factors, replays, revision_conflicts, revisions = resolve_factor_revisions(
            raw_factors, translation_tolerance_m, rotation_tolerance_deg)
        replay_messages += replays
        revision_states[topic] = revisions
        values, duplicates, conflicts = index_factor_deliveries(
            factors, translation_tolerance_m, rotation_tolerance_deg)
        conflicts.extend(revision_conflicts)
        indexed[topic] = values
        duplicate_messages += duplicates
        topic_recipients = sorted(set(factor.recipient for factor in factors))
        topic_summary[topic] = {
            "messages": len(raw_factors),
            "replays": replays,
            "unique_factors": len(values),
            "duplicates": duplicates,
            "recipients": topic_recipients,
        }
        if len(topic_recipients) > 1:
            recipient_errors.append({
                "topic": topic,
                "reason": "multiple_recipients_on_one_topic",
                "recipients": topic_recipients,
            })
        for factor in factors:
            identity, _ = canonical_factor(factor)
            recipients_by_factor[identity].add(factor.recipient)
            expected = {
                factor.from_endpoint.robot_id, factor.to_endpoint.robot_id}
            if factor.recipient not in expected:
                recipient_errors.append({
                    "topic": topic,
                    "factor": factor_label(identity),
                    "reason": "recipient_is_not_an_endpoint",
                    "recipient": factor.recipient,
                    "expected": sorted(expected),
                })
        for conflict in conflicts:
            duplicate_conflicts.append({"topic": topic, **conflict})

    baseline_topic = sorted(indexed)[0]
    baseline = indexed[baseline_topic]
    missing = {}
    extra = {}
    measurement_mismatches = []
    for topic in sorted(indexed):
        if topic == baseline_topic:
            continue
        topic_values = indexed[topic]
        absent = sorted(set(baseline) - set(topic_values))
        additional = sorted(set(topic_values) - set(baseline))
        if absent:
            missing[topic] = [factor_label(identity) for identity in absent]
        if additional:
            extra[topic] = [factor_label(identity) for identity in additional]
        for identity in sorted(set(baseline) & set(topic_values)):
            translation, rotation = pose_difference(
                baseline[identity], topic_values[identity])
            if (translation > translation_tolerance_m or
                    rotation > rotation_tolerance_deg):
                measurement_mismatches.append({
                    "topic": topic,
                    "factor": factor_label(identity),
                    "translation_difference_m": translation,
                    "rotation_difference_deg": rotation,
                })

    for identity in sorted(set().union(*(set(value) for value in indexed.values()))):
        expected = {identity[0].robot_id, identity[1].robot_id}
        actual = recipients_by_factor[identity]
        if actual != expected:
            recipient_errors.append({
                "factor": factor_label(identity),
                "reason": "factor_not_delivered_to_both_endpoints",
                "recipients": sorted(actual),
                "expected": sorted(expected),
            })

    revision_mismatches = [topic for topic in sorted(revision_states)
                           if revision_states[topic] != revision_states[baseline_topic]]
    symmetric = not (
        missing or extra or measurement_mismatches or duplicate_conflicts or
        duplicate_messages or recipient_errors or revision_mismatches)
    return {
        "symmetric": symmetric,
        "baseline_topic": baseline_topic,
        "topics": topic_summary,
        "missing_from_topic": missing,
        "extra_on_topic": extra,
        "measurement_mismatches": measurement_mismatches,
        "duplicate_conflicts": duplicate_conflicts,
        "duplicate_messages": duplicate_messages,
        "replay_messages": replay_messages,
        "revision_mismatches": revision_mismatches,
        "recipient_errors": recipient_errors,
    }, baseline


def collect_endpoint_timestamps(observations, tolerance_s=1.0e-6):
    """Build the keyframe clock map and expose conflicts and index regressions."""
    endpoint_times = {}
    conflicts = []
    for observation in observations:
        previous = endpoint_times.get(observation.endpoint)
        if previous is not None and abs(previous - observation.time) > tolerance_s:
            conflicts.append({
                "endpoint": observation.endpoint.label(),
                "first_time": previous,
                "later_time": observation.time,
                "difference_s": abs(previous - observation.time),
            })
            continue
        endpoint_times[observation.endpoint] = float(observation.time)

    by_robot = collections.defaultdict(list)
    for endpoint, timestamp in endpoint_times.items():
        by_robot[(endpoint.robot_id, endpoint.trajectory_epoch)].append((endpoint.keyframe_index, timestamp))
    monotonicity_violations = []
    for (robot_id, epoch), values in sorted(by_robot.items()):
        values.sort()
        for previous, current in zip(values, values[1:]):
            if current[1] + tolerance_s < previous[1]:
                monotonicity_violations.append({
                    "robot_id": robot_id,
                    "previous_keyframe_index": previous[0],
                    "previous_time": previous[1],
                    "keyframe_index": current[0],
                    "time": current[1],
                })
    return endpoint_times, conflicts, monotonicity_violations


def _nearest_with_gap(trajectory, timestamp):
    if not trajectory.poses:
        return None, float("inf")
    index = bisect.bisect_left(trajectory.times, timestamp)
    candidates = []
    if index < len(trajectory):
        candidates.append(trajectory[index])
    if index > 0:
        candidates.append(trajectory[index - 1])
    pose = min(candidates, key=lambda item: abs(item.time - timestamp))
    return pose, abs(pose.time - timestamp)


def _statistics(values, unit):
    if not values:
        return None
    statistics = Statistics(values, unit)
    result = statistics.as_dict()
    result["p90"] = statistics.percentile(0.90)
    for key, value in result.items():
        if isinstance(value, float) and not math.isfinite(value):
            result[key] = None
    return result


def score_factors(factors, endpoint_times, ground_truth,
                  max_time_difference_s=0.03,
                  position_only_ground_truth=False):
    """Score canonical factors using their original endpoint timestamps."""
    scored = []
    unassociated = []
    for identity, estimate in sorted(factors.items()):
        from_endpoint, to_endpoint = identity
        missing_times = [
            endpoint.label() for endpoint in identity
            if endpoint not in endpoint_times]
        missing_ground_truth = [
            endpoint.robot_id for endpoint in identity
            if endpoint.robot_id not in ground_truth]
        if missing_times or missing_ground_truth:
            unassociated.append({
                "factor": factor_label(identity),
                "reason": "missing_endpoint_time" if missing_times
                          else "missing_ground_truth",
                "missing": missing_times or sorted(set(missing_ground_truth)),
            })
            continue

        from_pose, from_gap = _nearest_with_gap(
            ground_truth[from_endpoint.robot_id],
            endpoint_times[from_endpoint])
        to_pose, to_gap = _nearest_with_gap(
            ground_truth[to_endpoint.robot_id], endpoint_times[to_endpoint])
        maximum_gap = max(from_gap, to_gap)
        if maximum_gap > max_time_difference_s:
            unassociated.append({
                "factor": factor_label(identity),
                "reason": "ground_truth_time_gap",
                "maximum_time_difference_s": maximum_gap,
                "from_time_difference_s": from_gap,
                "to_time_difference_s": to_gap,
            })
            continue

        reference = from_pose.between(to_pose)
        separation_error = abs(
            linalg.norm(estimate.translation) -
            linalg.norm(reference.translation))
        row = {
            "factor": factor_label(identity),
            "from_time": endpoint_times[from_endpoint],
            "to_time": endpoint_times[to_endpoint],
            "maximum_time_difference_s": maximum_gap,
            "separation_error_m": separation_error,
        }
        if position_only_ground_truth:
            row["translation_error_m"] = None
            row["rotation_error_deg"] = None
        else:
            error = estimate.inverse().compose(reference)
            row["translation_error_m"] = linalg.norm(error.translation)
            row["rotation_error_deg"] = math.degrees(
                linalg.rotation_angle(error.rotation))
        scored.append(row)

    translation_errors = [
        row["translation_error_m"] for row in scored
        if row["translation_error_m"] is not None]
    rotation_errors = [
        row["rotation_error_deg"] for row in scored
        if row["rotation_error_deg"] is not None]
    separation_errors = [row["separation_error_m"] for row in scored]
    return {
        "total_factors": len(factors),
        "associated_factors": len(scored),
        "unassociated_factors": len(unassociated),
        "max_time_difference_s": max_time_difference_s,
        "reference_mode": (
            "position_only" if position_only_ground_truth else "full_pose"),
        "translation": _statistics(translation_errors, "m"),
        "rotation": _statistics(rotation_errors, "deg"),
        "separation": _statistics(separation_errors, "m"),
        "factors": scored,
        "unassociated": unassociated,
    }
