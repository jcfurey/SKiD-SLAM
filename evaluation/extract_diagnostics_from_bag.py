#!/usr/bin/env python3
"""Extract loop-candidate and accepted-registration CSVs from a ROS 2 bag.

Record a map-fusion node's per-observer LoopDiagnostic topic, then run:

    ./evaluation/extract_diagnostics_from_bag.py my_bag \
        --topic /jackal1/solid/loop_diagnostics \
        --candidates-output results/park/candidates.csv \
        --registrations-output results/park/registrations.csv
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from skid_eval.diagnostic_extraction import (  # noqa: E402
    CANDIDATE_FIELDS, REGISTRATION_FIELDS, rows_from_messages, write_rows)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="bag directory or file")
    parser.add_argument("--topic", required=True,
                        help="liorf/msg/LoopDiagnostic topic to read")
    parser.add_argument("--candidates-output", required=True,
                        help="candidate CSV to write")
    parser.add_argument("--registrations-output", required=True,
                        help="accepted-registration CSV to write")
    parser.add_argument("--storage", default="sqlite3",
                        help="rosbag2 storage plugin (default: sqlite3)")
    arguments = parser.parse_args(argv)

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        print(f"error: this script needs a ROS 2 environment ({error})",
              file=sys.stderr)
        return 2

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=arguments.bag, storage_id=arguments.storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"))

    types = {topic.name: topic.type
             for topic in reader.get_all_topics_and_types()}
    if arguments.topic not in types:
        print(f"error: {arguments.topic} is not in the bag. Available: "
              f"{', '.join(sorted(types))}", file=sys.stderr)
        return 2
    actual_type = types[arguments.topic]
    expected_type = "liorf/msg/LoopDiagnostic"
    if actual_type != expected_type:
        print(f"error: {arguments.topic} has type {actual_type}, expected "
              f"{expected_type}", file=sys.stderr)
        return 2

    message_type = get_message(actual_type)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[arguments.topic]))

    messages = []
    while reader.has_next():
        record = (reader.read_next_ext()
                  if hasattr(reader, "read_next_ext") else reader.read_next())
        data = record[1]
        messages.append(deserialize_message(data, message_type))

    candidate_rows, registration_rows = rows_from_messages(messages)
    candidate_count = write_rows(
        arguments.candidates_output, CANDIDATE_FIELDS, candidate_rows)
    registration_count = write_rows(
        arguments.registrations_output, REGISTRATION_FIELDS,
        registration_rows)

    if not messages:
        print(f"error: no messages on {arguments.topic}", file=sys.stderr)
        return 1
    print(f"read {len(messages)} diagnostics; wrote {candidate_count} "
          f"candidates and {registration_count} accepted registrations")
    return 0


if __name__ == "__main__":
    sys.exit(main())
