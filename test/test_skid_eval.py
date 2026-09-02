"""Tests for the evaluation harness.

Every case uses data whose correct answer is known analytically, so a metric
that drifts is caught by the value, not by a golden file nobody re-derives.
"""

import math
import pathlib
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(
    0, str(pathlib.Path(__file__).resolve().parent.parent / "evaluation"))

from skid_eval import linalg, metrics, place_recognition, resources  # noqa: E402
from skid_eval.diagnostic_extraction import (  # noqa: E402
    CANDIDATE_FIELDS, REGISTRATION_FIELDS, candidate_row,
    registration_row, rows_from_messages, write_rows)
from skid_eval.factor_scoring import (  # noqa: E402
    Endpoint, EndpointTimestamp, Factor, collect_endpoint_timestamps,
    compare_factor_deliveries, score_factors)
from skid_eval.inputs import read_candidates, read_registrations  # noqa: E402
from skid_eval.alignment import (  # noqa: E402
    horn_alignment, yaw_only_alignment)
from skid_eval.manifest import Manifest, ManifestError  # noqa: E402
from skid_eval.report import render_text  # noqa: E402
from skid_eval.runner import evaluate  # noqa: E402
from skid_eval.trajectory import (  # noqa: E402
    Pose, Trajectory, associate, parse_tum, write_tum)

IDENTITY = linalg.identity3()


def _loop_diagnostic(**changes):
    """Small ROS-message-shaped object for extraction tests."""
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    values = {
        "stage": "descriptor",
        "decision": "selected",
        "reason": "within_descriptor_threshold",
        "query_time": 12.5,
        "match_time": 4.25,
        "descriptor_distance": 0.125,
        "query_robot_id": "jackal1",
        "query_keyframe_index": 42,
        "match_robot_id": "jackal0",
        "match_keyframe_index": 7,
        "candidate_rank": 1,
        "sector_shift": -2,
        "registration_accepted": False,
        "registration_status": "success",
        "registration_detail": "accepted",
        "relative_pose": SimpleNamespace(pose=pose),
        "source_points": 100,
        "target_points": 120,
        "coarse_correspondences": 80,
        "coarse_rotation_inliers": 70,
        "coarse_translation_inliers": 60,
        "fine_inliers": 55,
        "fine_converged": True,
        "fine_iterations": 9,
        "fine_error": 0.02,
        "metric_inliers": 50,
        "overlap_ratio": 0.5,
        "truncated_mse_m2": 0.04,
        "uncertainty_variance_scale": 1.5,
        "uncertainty_condition_number": 20.0,
        "uncertainty_clamped_modes": 1,
        "coarse_time_ms": 3.0,
        "fine_time_ms": 4.0,
        "metric_time_ms": 1.0,
    }
    values.update(changes)
    return SimpleNamespace(**values)


def rotation_z(angle):
    return linalg.quaternion_to_matrix(
        (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)))


# ---------------------------------------------------------------------------
# linalg
# ---------------------------------------------------------------------------

def test_quaternion_matrix_round_trip():
    quaternion = (0.1, -0.3, 0.2, 0.927)
    matrix = linalg.quaternion_to_matrix(quaternion)
    recovered = linalg.matrix_to_quaternion(matrix)
    again = linalg.quaternion_to_matrix(recovered)
    for row_a, row_b in zip(matrix, again):
        for a, b in zip(row_a, row_b):
            assert a == pytest.approx(b, abs=1e-12)


def test_rotation_angle_matches_the_constructed_angle():
    for angle in (0.0, 0.25, 1.0, math.pi - 1e-6):
        assert linalg.rotation_angle(rotation_z(angle)) == pytest.approx(
            angle, abs=1e-9)


def test_rotation_angle_clamps_a_numerically_imperfect_matrix():
    # A trace fractionally above 3 must not push acos out of its domain.
    nearly_identity = ((1.0 + 1e-15, 0.0, 0.0),
                       (0.0, 1.0, 0.0),
                       (0.0, 0.0, 1.0))
    assert linalg.rotation_angle(nearly_identity) == pytest.approx(0.0, abs=1e-6)


def test_jacobi_eigen_sorts_by_descending_eigenvalue():
    matrix = ((2.0, 1.0, 0.0), (1.0, 2.0, 0.0), (0.0, 0.0, 5.0))
    values, vectors = linalg.jacobi_eigen(matrix)
    assert values[0] == pytest.approx(5.0)
    assert values[1] == pytest.approx(3.0)
    assert values[2] == pytest.approx(1.0)
    # Eigenvectors are columns and satisfy A v = lambda v.
    for index, value in enumerate(values):
        vector = tuple(vectors[row][index] for row in range(3))
        product = linalg.matvec(matrix, vector)
        for a, b in zip(product, linalg.scale(vector, value)):
            assert a == pytest.approx(b, abs=1e-9)


# ---------------------------------------------------------------------------
# trajectory
# ---------------------------------------------------------------------------

def test_parse_tum_reads_poses_and_skips_comments():
    trajectory = parse_tum([
        "# a comment",
        "",
        "1.0 1 2 3 0 0 0 1",
        "0.0 0 0 0 0 0 0 1  # trailing comment",
    ])
    assert len(trajectory) == 2
    # Sorted by time regardless of file order.
    assert trajectory[0].time == 0.0
    assert trajectory[1].translation == (1.0, 2.0, 3.0)


@pytest.mark.parametrize("line, reason", [
    ("1.0 1 2 3", "8 fields"),
    ("1.0 1 2 3 0 0 0 0", "zero norm"),
    ("1.0 1 2 3 0 0 0 x", "could not convert"),
])
def test_parse_tum_rejects_malformed_lines(line, reason):
    with pytest.raises(ValueError) as error:
        parse_tum([line])
    assert reason in str(error.value)


def test_parse_tum_rejects_an_empty_file():
    with pytest.raises(ValueError, match="no poses"):
        parse_tum(["# nothing but a comment"])


def test_tum_round_trip_through_a_file(tmp_path):
    original = Trajectory([
        Pose.from_quaternion(0.0, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0)),
        Pose.from_quaternion(1.0, (4.0, 5.0, 6.0),
                             (0.0, 0.0, math.sin(0.5), math.cos(0.5))),
    ])
    path = tmp_path / "trajectory.tum"
    write_tum(path, original)
    reloaded = parse_tum(path.read_text().splitlines())

    assert len(reloaded) == 2
    for before, after in zip(original, reloaded):
        assert before.time == pytest.approx(after.time)
        for a, b in zip(before.translation, after.translation):
            assert a == pytest.approx(b, abs=1e-9)


def test_association_uses_each_reference_pose_only_once():
    reference = Trajectory([Pose(float(i), (float(i), 0, 0), IDENTITY)
                            for i in range(3)])
    # Three estimates all sitting on the same timestamp must not all match.
    estimate = Trajectory([Pose(1.0, (0, 0, 0), IDENTITY) for _ in range(3)])
    assert len(associate(estimate, reference, 0.5)) == 1


def test_association_respects_the_time_window():
    reference = Trajectory([Pose(0.0, (0, 0, 0), IDENTITY)])
    estimate = Trajectory([Pose(5.0, (0, 0, 0), IDENTITY)])
    assert associate(estimate, reference, 0.02) == []
    assert len(associate(estimate, reference, 10.0)) == 1


def test_pose_between_is_the_relative_transform():
    a = Pose(0.0, (1.0, 0.0, 0.0), rotation_z(0.0))
    b = Pose(1.0, (1.0, 2.0, 0.0), rotation_z(math.pi / 2))
    relative = a.between(b)
    assert relative.translation == pytest.approx((0.0, 2.0, 0.0), abs=1e-12)
    assert linalg.rotation_angle(relative.rotation) == pytest.approx(
        math.pi / 2, abs=1e-12)
    # Composing it back reproduces b.
    restored = a.compose(relative)
    assert restored.translation == pytest.approx(b.translation, abs=1e-12)


# ---------------------------------------------------------------------------
# alignment
# ---------------------------------------------------------------------------

def test_horn_recovers_a_known_rigid_transform():
    rotation = rotation_z(math.pi / 2)
    translation = (1.0, 2.0, 3.0)
    source = [(1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 3.0),
              (1.0, 1.0, 1.0)]
    target = [linalg.add(linalg.matvec(rotation, p), translation)
              for p in source]

    alignment = horn_alignment(source, target)
    assert alignment.translation == pytest.approx(translation, abs=1e-9)
    assert alignment.scale == pytest.approx(1.0)
    for s, t in zip(source, target):
        assert alignment.apply(s) == pytest.approx(t, abs=1e-9)


def test_horn_recovers_scale_when_asked():
    source = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
              (1.0, 1.0, 0.0)]
    target = [linalg.scale(p, 2.5) for p in source]
    alignment = horn_alignment(source, target, estimate_scale=True)
    assert alignment.scale == pytest.approx(2.5, abs=1e-9)


def test_horn_needs_three_correspondences():
    with pytest.raises(ValueError, match="at least 3"):
        horn_alignment([(0, 0, 0), (1, 0, 0)], [(0, 0, 0), (1, 0, 0)])


def test_horn_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match="equal length"):
        horn_alignment([(0, 0, 0)] * 4, [(0, 0, 0)] * 3)


def test_yaw_alignment_recovers_a_planar_rotation():
    angle = 0.7
    rotation = rotation_z(angle)
    reference = [Pose(float(i), (float(i), 0.5 * i, 0.0), IDENTITY)
                 for i in range(5)]
    estimate = [
        Pose(pose.time,
             linalg.add(linalg.matvec(rotation, pose.translation),
                        (3.0, -1.0, 0.0)),
             IDENTITY)
        for pose in reference]
    pairs = list(zip(estimate, reference))

    alignment = yaw_only_alignment(pairs)
    # The alignment must undo the rotation, so it is the inverse angle.
    assert linalg.rotation_angle(alignment.rotation) == pytest.approx(
        angle, abs=1e-9)
    for est, ref in pairs:
        assert alignment.apply(est.translation) == pytest.approx(
            ref.translation, abs=1e-9)


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------

def _reference_line(count=10):
    return Trajectory([Pose(float(i), (float(i), 0.0, 0.0), IDENTITY)
                       for i in range(count)])


def test_ate_is_zero_for_a_pure_frame_difference():
    reference = _reference_line()
    rotation = rotation_z(0.4)
    estimate = Trajectory([
        Pose(pose.time,
             linalg.add(linalg.matvec(rotation, pose.translation),
                        (5.0, -2.0, 1.0)),
             linalg.matmul(rotation, pose.rotation))
        for pose in reference])

    result = metrics.absolute_trajectory_error(associate(estimate, reference))
    assert result["translation"].rmse == pytest.approx(0.0, abs=1e-9)
    assert result["rotation"].rmse == pytest.approx(0.0, abs=1e-7)


def test_rigid_alignment_cannot_absorb_a_scale_error_but_sim3_can():
    reference = _reference_line()
    estimate = Trajectory([Pose(float(i), (1.1 * i, 0.0, 0.0), IDENTITY)
                           for i in range(10)])
    pairs = associate(estimate, reference)

    rigid = metrics.absolute_trajectory_error(pairs)
    similarity = metrics.absolute_trajectory_error(pairs, estimate_scale=True)
    assert rigid["translation"].rmse > 0.2
    assert similarity["translation"].rmse == pytest.approx(0.0, abs=1e-9)


def test_relative_pose_error_measures_per_step_drift():
    reference = _reference_line()
    estimate = Trajectory([Pose(float(i), (1.1 * i, 0.0, 0.0), IDENTITY)
                           for i in range(10)])
    result = metrics.relative_pose_error(associate(estimate, reference))
    # Each step is 1.1 m instead of 1.0 m.
    assert result["translation"].rmse == pytest.approx(0.1, abs=1e-9)
    assert result["rotation"].rmse == pytest.approx(0.0, abs=1e-9)


def test_relative_pose_error_is_unaffected_by_the_frame():
    reference = _reference_line()
    drifting = Trajectory([Pose(float(i), (1.1 * i, 0.0, 0.0), IDENTITY)
                           for i in range(10)])
    rotation = rotation_z(1.2)
    moved = Trajectory([
        Pose(pose.time,
             linalg.add(linalg.matvec(rotation, pose.translation),
                        (9.0, 9.0, 9.0)),
             linalg.matmul(rotation, pose.rotation))
        for pose in drifting])

    plain = metrics.relative_pose_error(associate(drifting, reference))
    shifted = metrics.relative_pose_error(associate(moved, reference))
    assert shifted["translation"].rmse == pytest.approx(
        plain["translation"].rmse, abs=1e-9)


def test_absolute_trajectory_error_needs_enough_associations():
    reference = _reference_line(2)
    with pytest.raises(ValueError, match="at least 3"):
        metrics.absolute_trajectory_error(associate(reference, reference))


def test_registration_errors_and_success_rate():
    truth = Pose(0.0, (5.0, 0.0, 0.0), rotation_z(0.0))
    good = Pose(0.0, (5.1, 0.0, 0.0), rotation_z(math.radians(1.0)))
    bad = Pose(0.0, (12.0, 0.0, 0.0), rotation_z(math.radians(30.0)))

    errors = metrics.registration_errors([(good, truth), (bad, truth)])
    assert errors["translation"].values[0] == pytest.approx(0.1, abs=1e-9)
    assert errors["rotation"].values[0] == pytest.approx(1.0, abs=1e-6)

    rate = metrics.success_rate(errors, 2.0, 5.0)
    assert rate == {"successes": 1, "total": 2, "rate": 0.5,
                    "translation_threshold_m": 2.0,
                    "rotation_threshold_deg": 5.0}


def test_success_requires_both_thresholds():
    truth = Pose(0.0, (0.0, 0.0, 0.0), rotation_z(0.0))
    # Right position, badly wrong heading: not a successful registration.
    misaligned = Pose(0.0, (0.0, 0.0, 0.0), rotation_z(math.radians(45.0)))
    errors = metrics.registration_errors([(misaligned, truth)])
    assert metrics.success_rate(errors, 2.0, 5.0)["successes"] == 0


def test_statistics_summary():
    stats = metrics.Statistics([1.0, 2.0, 3.0, 4.0], "m")
    assert stats.mean == pytest.approx(2.5)
    assert stats.median == pytest.approx(2.5)
    assert stats.rmse == pytest.approx(math.sqrt(7.5))
    assert stats.minimum == 1.0 and stats.maximum == 4.0
    assert stats.percentile(0.0) == pytest.approx(1.0)
    assert stats.percentile(1.0) == pytest.approx(4.0)
    assert stats.percentile(0.5) == pytest.approx(2.5)


# ---------------------------------------------------------------------------
# place recognition
# ---------------------------------------------------------------------------

def _candidates(scored):
    return [place_recognition.Candidate(0.0, 0.0, score, label)
            for score, label in scored]


def test_perfect_separation_scores_a_perfect_curve():
    candidates = _candidates([(0.1, True), (0.2, True),
                              (0.8, False), (0.9, False)])
    summary = place_recognition.summarize(
        place_recognition.precision_recall_curve(candidates), 2)
    assert summary["max_f1"] == pytest.approx(1.0)
    assert summary["max_f1_threshold"] == pytest.approx(0.2)
    assert summary["average_precision"] == pytest.approx(1.0)
    assert summary["recall_at_full_precision"] == pytest.approx(1.0)


def test_interleaved_scores_reduce_average_precision():
    candidates = _candidates([(0.1, True), (0.2, False), (0.3, True)])
    summary = place_recognition.summarize(
        place_recognition.precision_recall_curve(candidates), 2)
    # Precision is 1.0 at the first hit and 2/3 at the second.
    assert summary["average_precision"] == pytest.approx(
        0.5 * 1.0 + 0.5 * (2.0 / 3.0))
    assert summary["recall_at_full_precision"] == pytest.approx(0.5)


def test_total_positives_governs_recall():
    candidates = _candidates([(0.1, True), (0.2, True)])
    over_proposals = place_recognition.summarize(
        place_recognition.precision_recall_curve(candidates), 2)
    over_truth = place_recognition.summarize(
        place_recognition.precision_recall_curve(candidates, 4), 4)
    assert over_proposals["recall_at_max_f1"] == pytest.approx(1.0)
    assert over_truth["recall_at_max_f1"] == pytest.approx(0.5)


def test_ties_produce_a_single_operating_point():
    candidates = _candidates([(0.5, True), (0.5, False)])
    curve = place_recognition.precision_recall_curve(candidates)
    assert len(curve) == 1
    assert curve[0]["precision"] == pytest.approx(0.5)


def test_labelling_uses_distance_and_a_time_gap():
    # A loop: the platform returns to the origin at t=100.
    reference = Trajectory([
        Pose(0.0, (0.0, 0.0, 0.0), IDENTITY),
        Pose(50.0, (100.0, 0.0, 0.0), IDENTITY),
        Pose(100.0, (1.0, 0.0, 0.0), IDENTITY),
    ])
    candidates = [
        place_recognition.Candidate(100.0, 0.0, 0.1),    # true revisit
        place_recognition.Candidate(50.0, 0.0, 0.2),     # far apart in space
        place_recognition.Candidate(50.0, 50.0, 0.3),    # no time gap
        place_recognition.Candidate(999.0, 0.0, 0.4),    # not in the reference
    ]
    labelled = place_recognition.label_candidates(
        candidates, reference, revisit_distance_m=5.0, min_time_gap_s=30.0)

    assert labelled == 3
    assert candidates[0].is_true_loop is True
    assert candidates[1].is_true_loop is False
    assert candidates[2].is_true_loop is False
    assert candidates[3].is_true_loop is None


def test_curve_requires_labels():
    with pytest.raises(ValueError, match="no labelled"):
        place_recognition.precision_recall_curve(
            [place_recognition.Candidate(0.0, 0.0, 0.1)])


# ---------------------------------------------------------------------------
# resources
# ---------------------------------------------------------------------------

def test_descriptor_sizes_and_the_scan_context_baseline():
    summary = resources.descriptor_memory(
        1000, knn_feature_dim=40, num_sectors=60, scan_context_rings=20)
    assert summary["solid_bytes_per_descriptor"] == (40 + 60) * 4
    assert summary["scan_context_bytes_per_descriptor"] == 20 * 60 * 4
    assert summary["reduction_factor"] == pytest.approx(12.0)
    assert summary["solid_total_bytes"] == 400 * 1000


def test_descriptor_dimensions_must_be_positive():
    with pytest.raises(ValueError):
        resources.solid_descriptor_bytes(0, 60)
    with pytest.raises(ValueError):
        resources.scan_context_bytes(20, 0)


# The exact line MapFusion::commsMaintenance emits. If that format string
# changes, this test fails and the parser has to follow it.
COMMS_LINE = (
    "[INFO] [liorf_mapFusion]: comms announcements: sent 1200 msg / "
    "480.500 kiB, received 900 msg / 360.250 kiB, no latency samples | "
    "scans: sent 40 msg / 40960.000 kiB, received 38 msg / 38912.000 kiB, "
    "latency mean 120.500 ms max 512.000 ms over 38 samples | "
    "cache 500 scans / 480.5 MiB | requests in flight 2, retried 5, "
    "abandoned 1, throttled 3 | deferred 4 (dropped 0, expired 2) | "
    "announcement backlog dropped 7/9 | cache evictions 12")


def test_parses_the_map_fusion_diagnostics_line():
    record = resources.parse_comms_line(COMMS_LINE)
    assert record is not None
    assert record["announcements"]["sent_messages"] == 1200
    assert record["announcements"]["sent_kib"] == pytest.approx(480.5)
    assert "latency_mean_ms" not in record["announcements"]
    assert record["scans"]["latency_mean_ms"] == pytest.approx(120.5)
    assert record["scans"]["latency_samples"] == 38
    assert record["cached_scans"] == 500
    assert record["retried"] == 5
    assert record["abandoned"] == 1
    assert record["deferred_expired"] == 2
    assert record["cache_evictions"] == 12


def test_ignores_unrelated_log_lines():
    assert resources.parse_comms_line("[INFO] something else entirely") is None
    assert resources.parse_comms_log(["nothing", "here"]) == []


def test_communication_summary_uses_the_last_cumulative_report():
    early = COMMS_LINE.replace("sent 40 msg / 40960.000 kiB",
                               "sent 10 msg / 10240.000 kiB")
    records = resources.parse_comms_log([early, COMMS_LINE])
    summary = resources.summarize_communication(records)
    assert summary["reports"] == 2
    assert summary["scan_messages_sent"] == 40
    # Scans dominate the bytes even though announcements dominate the count.
    assert summary["scan_share_of_sent"] > 0.98
    assert summary["announcement_messages_sent"] == 1200


def test_communication_summary_needs_a_record():
    with pytest.raises(ValueError, match="no communication records"):
        resources.summarize_communication([])


# ---------------------------------------------------------------------------
# manifest
# ---------------------------------------------------------------------------

def _minimal_manifest(**overrides):
    document = {
        "dataset": "park", "title": "Park", "config": "config/park.yaml",
        "robots": [{"id": "jackal0"}],
    }
    document.update(overrides)
    return document


def test_manifest_defaults():
    manifest = Manifest(_minimal_manifest())
    assert manifest.alignment == "rigid"
    assert manifest.relative_pose_delta == 1
    assert manifest.knn_feature_dim == 40
    assert manifest.revisit_distance_m == pytest.approx(10.0)


@pytest.mark.parametrize("overrides, reason", [
    ({"robots": []}, "non-empty"),
    ({"alignment": "affine"}, "alignment must be one of"),
    ({"relative_pose_delta": 0}, "at least 1"),
    ({"association_max_time_difference_s": 0.0}, "must be positive"),
    ({"expected": []}, "must be a mapping"),
])
def test_manifest_rejects_bad_values(overrides, reason):
    with pytest.raises(ManifestError, match=reason):
        Manifest(_minimal_manifest(**overrides))


def test_manifest_reports_every_missing_field():
    with pytest.raises(ManifestError) as error:
        Manifest({"dataset": "park"})
    message = str(error.value)
    assert "title" in message and "config" in message and "robots" in message


def test_shipped_manifests_all_load():
    root = (pathlib.Path(__file__).resolve().parent.parent
            / "evaluation" / "manifests")
    manifests = sorted(root.glob("*.yaml"))
    assert len(manifests) == 6
    from skid_eval.manifest import load_manifest
    for path in manifests:
        manifest = load_manifest(path)
        assert manifest.robots
        # Every shipped manifest points at a configuration that exists.
        config = root.parent.parent / manifest.config
        assert config.exists(), f"{path.name} names a missing config"


# ---------------------------------------------------------------------------
# runner, end to end
# ---------------------------------------------------------------------------

def _write_loop_dataset(directory):
    """A circle of radius 10 traversed twice, so t and t+50 are the same place."""
    poses = []
    for step in range(100):
        angle = 2.0 * math.pi * step / 50.0
        poses.append(Pose(float(step),
                          (10.0 * math.cos(angle), 10.0 * math.sin(angle), 0.0),
                          rotation_z(angle)))
    reference = Trajectory(poses)
    write_tum(directory / "gt.tum", reference)

    # The estimate differs from truth only by a frame, so ATE must vanish.
    frame = rotation_z(0.6)
    estimate = Trajectory([
        Pose(pose.time,
             linalg.add(linalg.matvec(frame, pose.translation),
                        (7.0, -3.0, 2.0)),
             linalg.matmul(frame, pose.rotation))
        for pose in reference])
    write_tum(directory / "estimate.tum", estimate)

    (directory / "candidates.csv").write_text(
        "query_time,match_time,score\n"
        "50.0,0.0,0.05\n"      # exact revisit, best score
        "51.0,1.0,0.10\n"      # exact revisit
        "25.0,0.0,0.60\n"      # opposite side of the circle
        "26.0,1.0,0.70\n")     # opposite side of the circle

    # One registration that is exactly right, one that is 5 m out.
    truth = reference[50].between(reference[0])
    x, y, z, w = linalg.matrix_to_quaternion(truth.rotation)
    tx, ty, tz = truth.translation
    (directory / "registrations.csv").write_text(
        "query_time,match_time,tx,ty,tz,qx,qy,qz,qw\n"
        f"50.0,0.0,{tx},{ty},{tz},{x},{y},{z},{w}\n"
        f"51.0,1.0,{tx + 5.0},{ty},{tz},{x},{y},{z},{w}\n")

    (directory / "mapfusion.log").write_text(COMMS_LINE + "\n")
    return reference


def _manifest_for(directory):
    return Manifest(_minimal_manifest(
        place_recognition={"revisit_distance_m": 1.0, "min_time_gap_s": 30.0},
        registration={"translation_threshold_m": 2.0,
                      "rotation_threshold_deg": 5.0},
        resources={"knn_feature_dim": 40, "num_sectors": 60,
                   "scan_context_rings": 20},
        robots=[{
            "id": "jackal0",
            "ground_truth": "gt.tum",
            "estimate": "estimate.tum",
            "candidates": "candidates.csv",
            "registrations": "registrations.csv",
            "comms_log": "mapfusion.log",
        }]))


def test_end_to_end_evaluation_of_a_known_run(tmp_path):
    _write_loop_dataset(tmp_path)
    results = evaluate(_manifest_for(tmp_path), results_root=tmp_path)

    assert len(results["robots"]) == 1
    robot = results["robots"][0]
    assert robot["skipped"] == []

    trajectory_summary = robot["trajectory"]
    assert trajectory_summary["associated"] == 100
    # The estimate is the truth in another frame, so alignment removes it all.
    assert trajectory_summary["ate_translation"].rmse == pytest.approx(
        0.0, abs=1e-6)
    assert trajectory_summary["ate_rotation"].rmse == pytest.approx(
        0.0, abs=1e-4)
    assert trajectory_summary["rpe_translation"].rmse == pytest.approx(
        0.0, abs=1e-6)

    # The two revisits score better than the two non-revisits.
    place = robot["place_recognition"]
    assert place["labelled"] == 4
    assert place["max_f1"] == pytest.approx(1.0)
    assert place["average_precision"] == pytest.approx(1.0)

    # One registration exact, one 5 m out: half succeed at a 2 m threshold.
    registration = robot["registration"]
    assert registration["evaluated"] == 2
    assert registration["translation"].values[0] == pytest.approx(0.0, abs=1e-6)
    assert registration["translation"].values[1] == pytest.approx(5.0, abs=1e-6)
    assert registration["success"]["rate"] == pytest.approx(0.5)

    assert robot["resources"]["solid_bytes_per_descriptor"] == 400
    assert robot["communication"]["scan_messages_sent"] == 40

    # No expected values are declared, so nothing is compared.
    assert results["comparison"] == []
    assert results["expected_declared"] is False

    report = render_text(results)
    assert "Park" in report and "ATE" in report and "max F1" in report


def test_missing_inputs_are_reported_rather_than_fatal(tmp_path):
    manifest = Manifest(_minimal_manifest(robots=[{
        "id": "jackal0", "estimate": "absent.tum", "ground_truth": "gone.tum"}]))
    results = evaluate(manifest, results_root=tmp_path)

    robot = results["robots"][0]
    assert robot["trajectory"] is None
    assert robot["place_recognition"] is None
    assert any("not found" in reason for reason in robot["skipped"])
    # Resource figures need no run data and are still produced.
    assert robot["resources"]["solid_bytes_per_descriptor"] == 400
    assert "skipped" in render_text(results)


def test_expected_values_are_compared_when_declared(tmp_path):
    _write_loop_dataset(tmp_path)
    manifest = _manifest_for(tmp_path)
    manifest.expected = {"ate_rmse_m": 0.0, "success_rate": 0.9}

    results = evaluate(manifest, results_root=tmp_path)
    verdicts = {row["metric"]: row["verdict"] for row in results["comparison"]}
    assert verdicts["ATE RMSE (m)"] == "within tolerance"
    # Measured 0.5 against an expected 0.9 is well outside tolerance.
    assert "outside" in verdicts["Registration success rate"]
    assert "Comparison against expected" in render_text(results)


# ---------------------------------------------------------------------------
# structured diagnostic extraction
# ---------------------------------------------------------------------------

def test_candidate_extraction_keeps_rejected_descriptor_scores():
    message = _loop_diagnostic(
        decision="rejected", reason="descriptor_threshold",
        descriptor_distance=0.75, candidate_rank=6)
    row = candidate_row(message)

    assert row["score"] == pytest.approx(0.75)
    assert row["candidate_rank"] == 6
    assert row["decision"] == "rejected"
    assert row["query_time"] == pytest.approx(12.5)
    assert row["match_time"] == pytest.approx(4.25)


def test_candidate_extraction_drops_position_search_without_a_score():
    message = _loop_diagnostic(
        descriptor_distance=float("nan"), reason="position_search")
    assert candidate_row(message) is None


def test_registration_extraction_uses_match_into_query_pose():
    message = _loop_diagnostic(
        stage="registration", decision="accepted",
        registration_accepted=True)
    row = registration_row(message)

    assert (row["tx"], row["ty"], row["tz"]) == (1.0, 2.0, 3.0)
    assert (row["qx"], row["qy"], row["qz"], row["qw"]) == (
        0.0, 0.0, 0.0, 1.0)
    assert row["metric_inliers"] == 50
    assert row["truncated_mse_m2"] == pytest.approx(0.04)


def test_registration_extraction_omits_rejections_and_pcm_copies():
    rejected = _loop_diagnostic(
        stage="registration", decision="rejected",
        registration_accepted=False)
    pcm = _loop_diagnostic(
        stage="pcm", decision="accepted", registration_accepted=True)
    assert registration_row(rejected) is None
    assert registration_row(pcm) is None


def test_extracted_csvs_are_consumed_by_the_existing_readers(tmp_path):
    descriptor = _loop_diagnostic()
    registration = _loop_diagnostic(
        stage="registration", decision="accepted",
        registration_accepted=True)
    candidates, registrations = rows_from_messages(
        [descriptor, registration])
    candidates_path = tmp_path / "candidates.csv"
    registrations_path = tmp_path / "registrations.csv"

    assert write_rows(
        candidates_path, CANDIDATE_FIELDS, candidates) == 1
    assert write_rows(
        registrations_path, REGISTRATION_FIELDS, registrations) == 1

    parsed_candidates = read_candidates(candidates_path)
    parsed_registrations = read_registrations(registrations_path)
    assert parsed_candidates[0].score == pytest.approx(0.125)
    assert parsed_registrations[0][0:2] == pytest.approx((12.5, 4.25))
    assert parsed_registrations[0][2].translation == pytest.approx(
        (1.0, 2.0, 3.0))


# ---------------------------------------------------------------------------
# distributed-factor scoring
# ---------------------------------------------------------------------------

def test_factor_delivery_comparison_normalizes_opposite_orientations():
    first = Endpoint("jackal0", 4)
    second = Endpoint("jackal1", 9)
    transform = Pose(0.0, (2.0, -1.0, 0.5), rotation_z(0.4))
    deliveries = {
        "/jackal0/context/loop_info": [
            Factor("jackal0", first, second, transform)],
        "/jackal1/context/loop_info": [
            Factor("jackal1", second, first, transform.inverse())],
    }

    report, canonical = compare_factor_deliveries(deliveries)

    assert report["symmetric"] is True
    assert len(canonical) == 1
    assert canonical[(first, second)].translation == pytest.approx(
        transform.translation)


def test_factor_delivery_comparison_exposes_measurement_disagreement():
    first = Endpoint("jackal0", 4)
    second = Endpoint("jackal1", 9)
    deliveries = {
        "left": [Factor(
            "jackal0", first, second,
            Pose(0.0, (1.0, 0.0, 0.0), IDENTITY))],
        "right": [Factor(
            "jackal1", first, second,
            Pose(0.0, (1.1, 0.0, 0.0), IDENTITY))],
    }

    report, _ = compare_factor_deliveries(deliveries)

    assert report["symmetric"] is False
    assert report["measurement_mismatches"][0][
        "translation_difference_m"] == pytest.approx(0.1)


def test_factor_delivery_comparison_requires_both_endpoint_recipients():
    first = Endpoint("jackal0", 4)
    second = Endpoint("jackal1", 9)
    transform = Pose(0.0, (1.0, 0.0, 0.0), IDENTITY)
    deliveries = {
        "left": [Factor("jackal0", first, second, transform)],
        "right": [Factor("jackal0", first, second, transform)],
    }

    report, _ = compare_factor_deliveries(deliveries)

    assert report["symmetric"] is False
    assert any(
        error["reason"] == "factor_not_delivered_to_both_endpoints"
        for error in report["recipient_errors"])


def test_endpoint_timestamp_audit_exposes_conflicts_and_regressions():
    observations = [
        EndpointTimestamp(Endpoint("jackal0", 1), 10.0),
        EndpointTimestamp(Endpoint("jackal0", 1), 10.0),
        EndpointTimestamp(Endpoint("jackal0", 2), 9.0),
        EndpointTimestamp(Endpoint("jackal0", 2), 9.5),
    ]

    timestamps, conflicts, regressions = collect_endpoint_timestamps(
        observations)

    assert len(timestamps) == 2
    assert conflicts[0]["endpoint"] == "jackal0/2"
    assert conflicts[0]["difference_s"] == pytest.approx(0.5)
    assert regressions[0]["previous_keyframe_index"] == 1
    assert regressions[0]["keyframe_index"] == 2


def test_factor_scoring_uses_original_endpoint_times_and_relative_pose():
    first = Endpoint("jackal0", 4)
    second = Endpoint("jackal1", 9)
    first_pose = Pose(5.0, (1.0, 2.0, 0.0), rotation_z(0.5))
    second_pose = Pose(8.0, (4.0, 4.0, 0.0), rotation_z(0.8))
    reference = first_pose.between(second_pose)
    factors = {(first, second): reference}
    endpoint_times = {first: 5.001, second: 7.999}
    ground_truth = {
        "jackal0": Trajectory([first_pose]),
        "jackal1": Trajectory([second_pose]),
    }

    report = score_factors(
        factors, endpoint_times, ground_truth, max_time_difference_s=0.01)

    assert report["associated_factors"] == 1
    assert report["unassociated_factors"] == 0
    assert report["translation"]["max"] == pytest.approx(0.0, abs=1e-12)
    assert report["rotation"]["max"] == pytest.approx(0.0, abs=1e-12)
    assert report["separation"]["max"] == pytest.approx(0.0, abs=1e-12)


def test_factor_scoring_reports_ground_truth_gaps_without_reusing_bad_poses():
    first = Endpoint("jackal0", 4)
    second = Endpoint("jackal1", 9)
    factors = {(first, second): Pose(0.0, (1.0, 0.0, 0.0), IDENTITY)}
    endpoint_times = {first: 1.0, second: 2.0}
    ground_truth = {
        "jackal0": Trajectory([Pose(0.0, (0.0, 0.0, 0.0), IDENTITY)]),
        "jackal1": Trajectory([Pose(0.0, (1.0, 0.0, 0.0), IDENTITY)]),
    }

    report = score_factors(
        factors, endpoint_times, ground_truth, max_time_difference_s=0.1)

    assert report["associated_factors"] == 0
    assert report["translation"] is None
    assert report["unassociated"][0]["reason"] == "ground_truth_time_gap"
