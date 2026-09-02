#!/usr/bin/env python3
"""Exercise one ZeroMQ bridge link across two isolated ROS 2 domains.

This is an opt-in integration bench, not a CTest: it binds two loopback TCP
ports and starts ROS graph participants in two caller-selected domains. Source
the workspace first, then pass domain IDs and ports that are not in use.
"""

import argparse
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time


_STARTED = "ZeroMQ bridge started"
_HEALTHY_REPORT = re.compile(
    r"sent 1 msg .* received 1 msg .* dropped own 0, topic mismatch 0, "
    r"oversize 0, malformed 0 .* send failures 0 .* echoes suppressed 1")


def _domain(value):
    parsed = int(value)
    if not 0 <= parsed <= 232:
        raise argparse.ArgumentTypeError("domain IDs must be between 0 and 232")
    return parsed


def _port(value):
    parsed = int(value)
    if not 1 <= parsed <= 65535:
        raise argparse.ArgumentTypeError("ports must be between 1 and 65535")
    return parsed


def _environment(domain):
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain)
    # A shared ros2cli daemon can retain the domain from the first invocation.
    environment["ROS2CLI_DISABLE_DAEMON"] = "1"
    return environment


def _bridge_command(name, port, peer_port, topic, message_type):
    return [
        "ros2", "run", "liorf", "liorf_zmqBridge", "--ros-args",
        "-r", f"__node:=liorf_zmqBridge_{name}",
        "-p", f"robot_id:={name}",
        "-p", f"zmq.bind_endpoint:=tcp://127.0.0.1:{port}",
        "-p", f'zmq.peer_endpoints:=["tcp://127.0.0.1:{peer_port}"]',
        "-p", f'zmq.topics:=["{topic}"]',
        "-p", f'zmq.topic_types:=["{message_type}"]',
        "-p", "zmq.report_period_s:=1.0",
    ]


def _wait_for(predicate, timeout, description, processes=()):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for process in processes:
            status = process.poll()
            if status is not None:
                raise RuntimeError(
                    f"{description}: process {process.args[0]!r} exited {status}")
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(f"timed out waiting for {description}")


def _read(path):
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return ""


def _stop(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)


def _exchange(source_domain, destination_domain, payload, topic,
              message_type, output_path, error_path, publish_path, timeout):
    with output_path.open("w", encoding="utf-8") as output, \
            error_path.open("w", encoding="utf-8") as error:
        echo = subprocess.Popen(
            ["ros2", "topic", "echo", "--once", topic, message_type],
            env=_environment(destination_domain), stdout=output, stderr=error,
            text=True, start_new_session=True)
        try:
            time.sleep(1.0)
            with publish_path.open("w", encoding="utf-8") as publish_output:
                completed = subprocess.run(
                    ["ros2", "topic", "pub", "--once", topic, message_type,
                     "{data: '" + payload + "'}"],
                    env=_environment(source_domain), stdout=publish_output,
                    stderr=subprocess.STDOUT, text=True, timeout=timeout,
                    check=False)
            if completed.returncode:
                raise RuntimeError(
                    f"publisher in domain {source_domain} exited "
                    f"{completed.returncode}")
            _wait_for(
                lambda: echo.poll() is not None, timeout,
                f"payload from domain {source_domain} in domain "
                f"{destination_domain}")
            if echo.returncode:
                raise RuntimeError(
                    f"subscriber in domain {destination_domain} exited "
                    f"{echo.returncode}")
        finally:
            _stop(echo)

    if payload not in _read(output_path):
        raise RuntimeError(
            f"domain {destination_domain} did not print payload {payload!r}")


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-a", type=_domain, required=True)
    parser.add_argument("--domain-b", type=_domain, required=True)
    parser.add_argument("--port-a", type=_port, required=True)
    parser.add_argument("--port-b", type=_port, required=True)
    parser.add_argument("--topic", default="/skid_zmq_bench")
    parser.add_argument("--message-type", default="std_msgs/msg/String")
    parser.add_argument("--startup-timeout", type=float, default=12.0)
    parser.add_argument("--message-timeout", type=float, default=12.0)
    parser.add_argument(
        "--artifact-dir",
        help="new directory for logs (default: a unique directory in /tmp)")
    return parser.parse_args()


def main():
    arguments = _arguments()
    if arguments.domain_a == arguments.domain_b:
        raise SystemExit("--domain-a and --domain-b must be different")
    if arguments.port_a == arguments.port_b:
        raise SystemExit("--port-a and --port-b must be different")
    if arguments.startup_timeout <= 0.0 or arguments.message_timeout <= 0.0:
        raise SystemExit("timeouts must be positive")
    if shutil.which("ros2") is None:
        raise SystemExit("ros2 is not on PATH; source ROS and this workspace")

    if arguments.artifact_dir:
        artifact = Path(arguments.artifact_dir).expanduser().resolve()
        artifact.mkdir(parents=True, exist_ok=False)
    else:
        artifact = Path(tempfile.mkdtemp(
            prefix=(f"skid_zmq_bridge_domains{arguments.domain_a}_"
                    f"{arguments.domain_b}_")))

    names = (f"bench{arguments.domain_a}", f"bench{arguments.domain_b}")
    bridge_logs = (artifact / "bridge_a.log", artifact / "bridge_b.log")
    bridge_files = []
    bridges = []
    try:
        for name, domain, port, peer_port, log_path in (
                (names[0], arguments.domain_a, arguments.port_a,
                 arguments.port_b, bridge_logs[0]),
                (names[1], arguments.domain_b, arguments.port_b,
                 arguments.port_a, bridge_logs[1])):
            handle = log_path.open("w", encoding="utf-8")
            bridge_files.append(handle)
            bridges.append(subprocess.Popen(
                _bridge_command(name, port, peer_port, arguments.topic,
                                arguments.message_type),
                env=_environment(domain), stdout=handle,
                stderr=subprocess.STDOUT, text=True,
                start_new_session=True))

        _wait_for(
            lambda: all(_STARTED in _read(path) for path in bridge_logs),
            arguments.startup_timeout, "both bridges to start", bridges)

        # PUB/SUB drops messages sent before its asynchronous connection has
        # settled. Give the two loopback peers a bounded slow-joiner interval.
        time.sleep(2.0)
        _exchange(
            arguments.domain_a, arguments.domain_b,
            f"from-domain-{arguments.domain_a}", arguments.topic,
            arguments.message_type, artifact / "received_on_b.txt",
            artifact / "echo_b.err", artifact / "publish_a.log",
            arguments.message_timeout)
        _exchange(
            arguments.domain_b, arguments.domain_a,
            f"from-domain-{arguments.domain_b}", arguments.topic,
            arguments.message_type, artifact / "received_on_a.txt",
            artifact / "echo_a.err", artifact / "publish_b.log",
            arguments.message_timeout)

        _wait_for(
            lambda: all(_HEALTHY_REPORT.search(_read(path))
                        for path in bridge_logs),
            4.0, "healthy bidirectional bridge reports", bridges)
    except Exception as error:  # keep the logs for diagnosis
        print(f"FAIL: {error}", file=sys.stderr)
        print(f"artifacts: {artifact}", file=sys.stderr)
        return 1
    finally:
        for bridge in bridges:
            _stop(bridge)
        for handle in bridge_files:
            handle.close()

    print(
        f"PASS: domains {arguments.domain_a} and {arguments.domain_b} "
        "exchanged one message in each direction")
    print("PASS: both bridges reported clean delivery and echo suppression")
    print(f"artifacts: {artifact}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
