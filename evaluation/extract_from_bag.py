#!/usr/bin/env python3
"""Convert an odometry topic in a ROS 2 bag into a TUM trajectory.

NOT EXECUTED DURING DEVELOPMENT. This repository's development environment has
no ROS installation, so this script is written from the rosbag2_py and
nav_msgs APIs but has never been run. Check its output against the bag before
trusting a metric computed from it.

    ./evaluation/extract_from_bag.py my_bag \
        --topic /jackal0/liorf/mapping/odometry \
        --output results/park/estimate.tum
"""

import argparse
import pathlib
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="bag directory or file")
    parser.add_argument("--topic", required=True,
                        help="nav_msgs/msg/Odometry topic to read")
    parser.add_argument("--output", required=True, help="TUM file to write")
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
        rosbag2_py.StorageOptions(uri=arguments.bag,
                                  storage_id=arguments.storage),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"))

    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if arguments.topic not in types:
        print(f"error: {arguments.topic} is not in the bag. Available: "
              f"{', '.join(sorted(types))}", file=sys.stderr)
        return 2

    message_type = get_message(types[arguments.topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[arguments.topic]))

    output = pathlib.Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with open(output, "w", encoding="utf-8") as handle:
        handle.write("# timestamp tx ty tz qx qy qz qw\n")
        while reader.has_next():
            _, data, _ = reader.read_next()
            message = deserialize_message(data, message_type)
            stamp = message.header.stamp
            time = stamp.sec + stamp.nanosec * 1e-9
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            handle.write(
                f"{time:.9f} {position.x:.9f} {position.y:.9f} "
                f"{position.z:.9f} {orientation.x:.9f} {orientation.y:.9f} "
                f"{orientation.z:.9f} {orientation.w:.9f}\n")
            written += 1

    if written == 0:
        print(f"error: no messages on {arguments.topic}", file=sys.stderr)
        return 1
    print(f"wrote {written} poses to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
