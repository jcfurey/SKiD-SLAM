"""Runs the metrics a manifest asks for over the results it points at.

Every section is optional: a manifest that names no candidate file simply
reports no place-recognition figures, with the reason stated, rather than
failing. A missing input is a gap in the evaluation, not an error in it.
"""

import math

from . import inputs, metrics, place_recognition, resources, trajectory
from .alignment import align_trajectories, yaw_only_alignment

# label, relative tolerance, absolute tolerance. Both are needed: a purely
# relative tolerance is vacuous when the expected value is zero, and a purely
# absolute one does not scale between a 0.1 m and a 10 m dataset.
_COMPARISONS = {
    "ate_rmse_m": ("ATE RMSE (m)", 0.10, 0.01),
    "are_rmse_deg": ("ARE RMSE (deg)", 0.10, 0.10),
    "rte_rmse_m": ("RTE RMSE (m)", 0.10, 0.01),
    "rre_rmse_deg": ("RRE RMSE (deg)", 0.10, 0.10),
    "success_rate": ("Registration success rate", 0.05, 0.01),
    "max_f1": ("Place recognition max F1", 0.05, 0.01),
}


def _alignment_for(manifest, pairs):
    if manifest.alignment == "yaw":
        return yaw_only_alignment(pairs), None
    scaled = manifest.alignment == "sim3"
    alignment = align_trajectories(pairs, estimate_scale=scaled)
    return alignment, (alignment.scale if scaled else None)


def _evaluate_trajectory(manifest, robot, results_root, skipped):
    estimate_path = manifest.resolve(robot.estimate, results_root)
    reference_path = manifest.resolve(robot.ground_truth, results_root)
    if estimate_path is None or reference_path is None:
        skipped.append("no estimate or ground-truth trajectory in the manifest")
        return None, None
    if not estimate_path.exists() or not reference_path.exists():
        missing = [str(p) for p in (estimate_path, reference_path)
                   if not p.exists()]
        skipped.append(f"trajectory file(s) not found: {', '.join(missing)}")
        return None, None

    estimate = trajectory.read_tum(estimate_path)
    reference = trajectory.read_tum(reference_path)
    pairs = trajectory.associate(
        estimate, reference, manifest.association_max_time_difference_s)
    if len(pairs) < 3:
        skipped.append(
            f"only {len(pairs)} poses associated within "
            f"{manifest.association_max_time_difference_s} s; check the clocks")
        return None, reference

    alignment, scale = _alignment_for(manifest, pairs)
    absolute = metrics.absolute_trajectory_error(pairs, alignment=alignment)
    relative = metrics.relative_pose_error(pairs, manifest.relative_pose_delta)

    return {
        "estimate_poses": len(estimate),
        "reference_poses": len(reference),
        "associated": len(pairs),
        "alignment": manifest.alignment,
        "scale": scale,
        "delta": manifest.relative_pose_delta,
        "ate_translation": absolute["translation"],
        "ate_rotation": absolute["rotation"],
        "rpe_translation": relative["translation"],
        "rpe_rotation": relative["rotation"],
    }, reference


def _evaluate_place_recognition(manifest, robot, results_root, reference,
                                skipped):
    path = manifest.resolve(robot.candidates, results_root)
    if path is None:
        skipped.append("no loop-candidate file in the manifest")
        return None
    if not path.exists():
        skipped.append(f"loop-candidate file not found: {path}")
        return None

    candidates = inputs.read_candidates(path)
    unlabelled = [c for c in candidates if c.is_true_loop is None]
    if unlabelled:
        if reference is None:
            skipped.append(
                "candidates are unlabelled and no ground truth is available "
                "to label them")
            return None
        place_recognition.label_candidates(
            candidates, reference, manifest.revisit_distance_m,
            manifest.place_min_time_gap_s)

    labelled = [c for c in candidates if c.is_true_loop is not None]
    if not labelled:
        skipped.append("no candidate could be labelled against ground truth")
        return None

    total_positives = (manifest.total_positives
                       if manifest.total_positives is not None
                       else len([c for c in labelled if c.is_true_loop]))
    curve = place_recognition.precision_recall_curve(labelled, total_positives)
    summary = place_recognition.summarize(curve, total_positives)
    summary["labelled"] = len(labelled)
    summary["candidates"] = len(candidates)
    return summary


def _evaluate_registration(manifest, robot, results_root, reference, skipped):
    path = manifest.resolve(robot.registrations, results_root)
    if path is None:
        skipped.append("no registration file in the manifest")
        return None
    if not path.exists():
        skipped.append(f"registration file not found: {path}")
        return None
    if reference is None:
        skipped.append("registration needs ground truth to score against")
        return None

    registrations = inputs.read_registrations(path)
    measurements = []
    unmatched = 0
    for query_time, match_time, estimated in registrations:
        query = reference.nearest(
            query_time, manifest.association_max_time_difference_s)
        match = reference.nearest(
            match_time, manifest.association_max_time_difference_s)
        if query is None or match is None:
            unmatched += 1
            continue
        measurements.append((estimated, query.between(match)))

    if not measurements:
        skipped.append(
            "no registration could be located in the ground-truth trajectory")
        return None

    errors = metrics.registration_errors(measurements)
    rate = metrics.success_rate(
        errors,
        manifest.registration_translation_threshold_m,
        manifest.registration_rotation_threshold_deg)
    return {
        "evaluated": len(measurements),
        "unmatched": unmatched,
        "translation": errors["translation"],
        "rotation": errors["rotation"],
        "success": rate,
    }


def _evaluate_resources(manifest, robot, results_root, trajectory_summary):
    keyframes = (trajectory_summary["estimate_poses"]
                 if trajectory_summary else 0)
    return resources.descriptor_memory(
        keyframes, manifest.knn_feature_dim, manifest.num_sectors,
        manifest.scan_context_rings)


def _evaluate_communication(manifest, robot, results_root, skipped):
    path = manifest.resolve(robot.comms_log, results_root)
    if path is None:
        skipped.append("no communication log in the manifest")
        return None
    if not path.exists():
        skipped.append(f"communication log not found: {path}")
        return None

    with open(path, "r", encoding="utf-8") as handle:
        records = resources.parse_comms_log(handle)
    if not records:
        skipped.append(
            f"no map-fusion diagnostics lines found in {path}")
        return None
    return resources.summarize_communication(records)


def _measured_values(robot_results):
    measured = {}
    trajectory_summary = robot_results.get("trajectory")
    if trajectory_summary:
        measured["ate_rmse_m"] = trajectory_summary["ate_translation"].rmse
        measured["are_rmse_deg"] = trajectory_summary["ate_rotation"].rmse
    registration = robot_results.get("registration")
    if registration:
        measured["rte_rmse_m"] = registration["translation"].rmse
        measured["rre_rmse_deg"] = registration["rotation"].rmse
        measured["success_rate"] = registration["success"]["rate"]
    place = robot_results.get("place_recognition")
    if place:
        measured["max_f1"] = place["max_f1"]
    return measured


def compare_to_expected(manifest, robot_results):
    """Compare measurements against the manifest's expected values."""
    expected = {key: value for key, value in manifest.expected.items()
                if value is not None}
    if not expected:
        return []

    measured = {}
    for robot in robot_results:
        measured.update(_measured_values(robot))

    rows = []
    for key, value in expected.items():
        label, relative, absolute = _COMPARISONS.get(key, (key, 0.10, 0.0))
        actual = measured.get(key)
        if actual is None or (isinstance(actual, float) and math.isnan(actual)):
            verdict = "not measured"
        elif abs(actual - value) <= relative * abs(value) + absolute:
            verdict = "within tolerance"
        else:
            verdict = f"outside {relative:.0%} + {absolute:g} tolerance"
        rows.append({"metric": label, "measured": actual,
                     "expected": value, "verdict": verdict})
    return rows


def evaluate(manifest, results_root=None):
    """Evaluate every robot the manifest names."""
    robot_results = []
    for robot in manifest.robots:
        skipped = []
        trajectory_summary, reference = _evaluate_trajectory(
            manifest, robot, results_root, skipped)
        entry = {
            "id": robot.id,
            "trajectory": trajectory_summary,
            "place_recognition": _evaluate_place_recognition(
                manifest, robot, results_root, reference, skipped),
            "registration": _evaluate_registration(
                manifest, robot, results_root, reference, skipped),
            "resources": _evaluate_resources(
                manifest, robot, results_root, trajectory_summary),
            "communication": _evaluate_communication(
                manifest, robot, results_root, skipped),
            "skipped": skipped,
        }
        robot_results.append(entry)

    declared = any(value is not None for value in manifest.expected.values())
    return {
        "dataset": manifest.dataset,
        "title": manifest.title,
        "config": manifest.config,
        "robots": robot_results,
        "expected_declared": declared,
        "comparison": compare_to_expected(manifest, robot_results),
        "notes": [],
    }
