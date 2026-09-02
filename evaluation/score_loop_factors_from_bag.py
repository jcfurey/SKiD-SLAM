#!/usr/bin/env python3
"""Audit two-sided LoopConstraint delivery and score it against TUM ground truth.

Example:

    ./evaluation/score_loop_factors_from_bag.py diagnostics \
        --factor-topic /jackal0/context/loop_info \
        --factor-topic /jackal1/context/loop_info \
        --diagnostic-topic /jackal1/solid/loop_diagnostics \
        --ground-truth jackal0=jackal0_ground_truth.txt \
        --ground-truth jackal1=jackal1_ground_truth.txt \
        --max-time-difference 0.03 --max-rte 2.0 \
        --output factor_report.json

The endpoint timestamps come from accepted registration diagnostics, not the
factor publication time.  Factors are canonicalized by endpoint, so opposite
message orientations can be compared without hiding a transform-direction
error.  The report includes standard RTE/RRE and an orientation-independent
endpoint-separation error. Use ``--position-only-ground-truth`` when a
dataset's orientation channel is not a valid reference, and
``--max-separation`` to make that mode an executable regression gate.
"""

import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from skid_eval.factor_scoring import (  # noqa: E402
    Endpoint, EndpointTimestamp, Factor, collect_endpoint_timestamps,
    compare_factor_deliveries, score_factors)
from skid_eval.trajectory import Pose, read_tum  # noqa: E402


def _ground_truth_assignment(value):
    if "=" not in value:
        raise argparse.ArgumentTypeError(
            "expected ROBOT_ID=/path/to/ground_truth.txt")
    robot_id, path = value.split("=", 1)
    if not robot_id or not path:
        raise argparse.ArgumentTypeError(
            "expected non-empty ROBOT_ID and ground-truth path")
    return robot_id, path


def _pose_from_message(message):
    return Pose.from_quaternion(
        0.0,
        (message.position.x, message.position.y, message.position.z),
        (message.orientation.x, message.orientation.y,
         message.orientation.z, message.orientation.w))


def _read_messages(bag, storage, factor_topics, diagnostic_topic):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            f"this command needs a sourced ROS 2 environment ({error})") \
            from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"))
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = list(factor_topics) + [diagnostic_topic]
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise ValueError(
            "bag is missing required topic(s): " + ", ".join(missing))
    for topic in factor_topics:
        if topic_types[topic] != "liorf/msg/LoopConstraint":
            raise ValueError(
                f"{topic} has type {topic_types[topic]}, expected "
                "liorf/msg/LoopConstraint")
    if topic_types[diagnostic_topic] != "liorf/msg/LoopDiagnostic":
        raise ValueError(
            f"{diagnostic_topic} has type {topic_types[diagnostic_topic]}, "
            "expected liorf/msg/LoopDiagnostic")

    reader.set_filter(rosbag2_py.StorageFilter(topics=required))
    message_types = {
        topic: get_message(topic_types[topic]) for topic in required}
    deliveries = {topic: [] for topic in factor_topics}
    timestamp_observations = []
    accepted_diagnostics = 0
    while reader.has_next():
        record = (reader.read_next_ext()
                  if hasattr(reader, "read_next_ext") else reader.read_next())
        topic, data = record[0], record[1]
        message = deserialize_message(data, message_types[topic])
        if topic in deliveries:
            deliveries[topic].append(Factor(
                recipient=message.robot_id,
                from_endpoint=Endpoint(
                    message.from_robot_id, int(message.index_from)),
                to_endpoint=Endpoint(
                    message.to_robot_id, int(message.index_to)),
                relative_pose=_pose_from_message(message.relative_pose.pose)))
            continue

        if (message.stage != "registration" or
                not message.registration_accepted):
            continue
        accepted_diagnostics += 1
        timestamp_observations.extend((
            EndpointTimestamp(
                Endpoint(message.query_robot_id,
                         int(message.query_keyframe_index)),
                float(message.query_time)),
            EndpointTimestamp(
                Endpoint(message.match_robot_id,
                         int(message.match_keyframe_index)),
                float(message.match_time)),
        ))
    return deliveries, timestamp_observations, accepted_diagnostics


def _parse_ground_truth(assignments):
    trajectories = {}
    paths = {}
    for robot_id, path in assignments:
        if robot_id in trajectories:
            raise ValueError(f"ground truth specified twice for {robot_id}")
        trajectories[robot_id] = read_tum(path)
        paths[robot_id] = str(pathlib.Path(path).resolve())
    return trajectories, paths


def _write_report(path, report):
    destination = pathlib.Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")


def _print_summary(report):
    delivery = report["delivery"]
    score = report["score"]
    counts = ", ".join(
        f"{topic}: {values['unique_factors']} unique/{values['messages']} msg"
        for topic, values in delivery["topics"].items())
    print(f"factor delivery symmetric={delivery['symmetric']} ({counts})")
    print(
        f"endpoint timestamps={report['endpoint_timestamps']['unique']} "
        f"conflicts={len(report['endpoint_timestamps']['conflicts'])} "
        f"monotonicity_violations="
        f"{len(report['endpoint_timestamps']['monotonicity_violations'])}")
    print(
        f"ground-truth associated={score['associated_factors']}/"
        f"{score['total_factors']} within "
        f"{score['max_time_difference_s']:.3f} s")
    if score["translation"] is not None:
        values = score["translation"]
        print(
            f"RTE median={values['median']:.3f} m "
            f"p90={values['p90']:.3f} m max={values['max']:.3f} m")
    if score["rotation"] is not None:
        values = score["rotation"]
        print(
            f"RRE median={values['median']:.3f} deg "
            f"p90={values['p90']:.3f} deg max={values['max']:.3f} deg")
    if score["separation"] is not None:
        values = score["separation"]
        print(
            f"separation error median={values['median']:.3f} m "
            f"p90={values['p90']:.3f} m max={values['max']:.3f} m")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="diagnostic bag directory or file")
    parser.add_argument(
        "--factor-topic", action="append", required=True,
        help="LoopConstraint topic; pass once per endpoint robot")
    parser.add_argument("--diagnostic-topic", required=True,
                        help="LoopDiagnostic topic carrying endpoint times")
    parser.add_argument(
        "--ground-truth", action="append", required=True,
        type=_ground_truth_assignment, metavar="ROBOT_ID=PATH",
        help="retimed TUM trajectory; pass once per robot")
    parser.add_argument("--storage", default="mcap",
                        help="rosbag2 storage plugin (default: mcap)")
    parser.add_argument("--max-time-difference", type=float, default=0.03,
                        help="maximum endpoint-to-ground-truth gap in seconds")
    parser.add_argument(
        "--position-only-ground-truth", action="store_true",
        help=("do not use ground-truth orientations; report only the "
              "orientation-independent endpoint-separation error"))
    parser.add_argument("--max-rte", type=float,
                        help="fail if any associated factor exceeds this RTE")
    parser.add_argument("--max-rre", type=float,
                        help="fail if any associated factor exceeds this RRE")
    parser.add_argument(
        "--max-separation", type=float,
        help=("fail if any associated factor exceeds this "
              "orientation-independent endpoint-separation error"))
    parser.add_argument("--min-associated", type=int, default=1,
                        help="minimum number of ground-truth-scored factors")
    parser.add_argument("--output", help="write the complete JSON report")
    parser.add_argument("--json", action="store_true",
                        help="print the complete JSON report to stdout")
    arguments = parser.parse_args(argv)

    if len(arguments.factor_topic) < 2:
        parser.error("--factor-topic must be supplied at least twice")
    if arguments.max_time_difference < 0.0:
        parser.error("--max-time-difference must be non-negative")
    if arguments.min_associated < 0:
        parser.error("--min-associated must be non-negative")
    if arguments.max_rte is not None and arguments.max_rte < 0.0:
        parser.error("--max-rte must be non-negative")
    if arguments.max_rre is not None and arguments.max_rre < 0.0:
        parser.error("--max-rre must be non-negative")
    if arguments.max_separation is not None and arguments.max_separation < 0.0:
        parser.error("--max-separation must be non-negative")
    if arguments.position_only_ground_truth and (
            arguments.max_rte is not None or arguments.max_rre is not None):
        parser.error(
            "--max-rte/--max-rre require full-pose ground truth")

    try:
        ground_truth, ground_truth_paths = _parse_ground_truth(
            arguments.ground_truth)
        deliveries, observations, accepted_diagnostics = _read_messages(
            arguments.bag, arguments.storage, arguments.factor_topic,
            arguments.diagnostic_topic)
        delivery_report, canonical_factors = compare_factor_deliveries(
            deliveries)
        endpoint_times, conflicts, monotonicity_violations = \
            collect_endpoint_timestamps(observations)
        score = score_factors(
            canonical_factors, endpoint_times, ground_truth,
            arguments.max_time_difference,
            position_only_ground_truth=arguments.position_only_ground_truth)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    report = {
        "schema": "skid_slam/loop_factor_audit/v1",
        "bag": str(pathlib.Path(arguments.bag).resolve()),
        "storage": arguments.storage,
        "factor_topics": arguments.factor_topic,
        "diagnostic_topic": arguments.diagnostic_topic,
        "ground_truth": ground_truth_paths,
        "accepted_registration_diagnostics": accepted_diagnostics,
        "delivery": delivery_report,
        "endpoint_timestamps": {
            "unique": len(endpoint_times),
            "conflicts": conflicts,
            "monotonicity_violations": monotonicity_violations,
        },
        "score": score,
        "thresholds": {
            "max_rte_m": arguments.max_rte,
            "max_rre_deg": arguments.max_rre,
            "max_separation_m": arguments.max_separation,
            "min_associated": arguments.min_associated,
        },
    }
    if arguments.output:
        _write_report(arguments.output, report)
    if arguments.json:
        json.dump(report, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
        print()
    else:
        _print_summary(report)

    failures = []
    if not delivery_report["symmetric"]:
        failures.append("factor delivery is not symmetric")
    if conflicts:
        failures.append("endpoint timestamps conflict")
    if monotonicity_violations:
        failures.append("endpoint timestamps regress with keyframe index")
    if score["associated_factors"] < arguments.min_associated:
        failures.append(
            f"only {score['associated_factors']} factors were associated")
    if arguments.max_rte is not None and score["translation"] is not None:
        if score["translation"]["max"] > arguments.max_rte:
            failures.append(
                f"maximum RTE {score['translation']['max']:.6f} m exceeds "
                f"{arguments.max_rte:.6f} m")
    if arguments.max_rre is not None and score["rotation"] is not None:
        if score["rotation"]["max"] > arguments.max_rre:
            failures.append(
                f"maximum RRE {score['rotation']['max']:.6f} deg exceeds "
                f"{arguments.max_rre:.6f} deg")
    if arguments.max_separation is not None and score["separation"] is not None:
        if score["separation"]["max"] > arguments.max_separation:
            failures.append(
                "maximum separation error "
                f"{score['separation']['max']:.6f} m exceeds "
                f"{arguments.max_separation:.6f} m")
    if failures:
        print("error: " + "; ".join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
