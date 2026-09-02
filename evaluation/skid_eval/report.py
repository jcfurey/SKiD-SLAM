"""Rendering of evaluation results."""

import json


def _format(value, digits=4):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        if value != value:  # NaN
            return "n/a"
        return f"{value:.{digits}f}"
    return str(value)


def _statistics_lines(name, statistics, indent="    "):
    data = statistics.as_dict()
    return [
        f"{indent}{name} ({data['unit']}): "
        f"rmse {_format(data['rmse'])}  mean {_format(data['mean'])}  "
        f"median {_format(data['median'])}  p95 {_format(data['p95'])}  "
        f"max {_format(data['max'])}  n={data['count']}"
    ]


def render_text(results):
    lines = []
    lines.append(f"SKiD-SLAM evaluation: {results['title']} "
                 f"({results['dataset']})")
    lines.append("=" * 72)

    for note in results.get("notes", []):
        lines.append(f"note: {note}")
    if results.get("notes"):
        lines.append("")

    for robot in results["robots"]:
        lines.append(f"robot {robot['id']}")
        if robot.get("skipped"):
            for reason in robot["skipped"]:
                lines.append(f"    skipped: {reason}")

        trajectory = robot.get("trajectory")
        if trajectory:
            lines.append(f"  trajectory  associated {trajectory['associated']} "
                         f"of {trajectory['estimate_poses']} estimated poses "
                         f"against {trajectory['reference_poses']} reference "
                         f"poses")
            lines.append(f"    alignment: {trajectory['alignment']}"
                         + (f" (scale {_format(trajectory['scale'])})"
                            if trajectory.get('scale') is not None else ""))
            lines += _statistics_lines("ATE", trajectory["ate_translation"])
            lines += _statistics_lines("ARE", trajectory["ate_rotation"])
            lines += _statistics_lines(
                f"RPE t (delta={trajectory['delta']})", trajectory["rpe_translation"])
            lines += _statistics_lines(
                f"RPE r (delta={trajectory['delta']})", trajectory["rpe_rotation"])

        place = robot.get("place_recognition")
        if place:
            lines.append(f"  place recognition  {place['labelled']} labelled "
                         f"candidates, {place['total_positives']} positives")
            lines.append(
                f"    max F1 {_format(place['max_f1'])} at threshold "
                f"{_format(place['max_f1_threshold'])} "
                f"(P {_format(place['precision_at_max_f1'])}, "
                f"R {_format(place['recall_at_max_f1'])})")
            lines.append(
                f"    average precision {_format(place['average_precision'])}  "
                f"recall at full precision "
                f"{_format(place['recall_at_full_precision'])}")

        registration = robot.get("registration")
        if registration:
            lines.append(f"  registration  {registration['evaluated']} loops")
            lines += _statistics_lines("RTE", registration["translation"])
            lines += _statistics_lines("RRE", registration["rotation"])
            rate = registration["success"]
            lines.append(
                f"    success {rate['successes']}/{rate['total']} = "
                f"{_format(rate['rate'])} "
                f"(<= {_format(rate['translation_threshold_m'], 2)} m, "
                f"<= {_format(rate['rotation_threshold_deg'], 2)} deg)")

        resources = robot.get("resources")
        if resources:
            lines.append("  resources")
            lines.append(
                f"    descriptor {resources['solid_bytes_per_descriptor']} B "
                f"x {resources['keyframes']} keyframes = "
                f"{resources['solid_total_bytes'] / 1024.0:.1f} kiB")
            if "reduction_factor" in resources:
                lines.append(
                    f"    Scan Context baseline "
                    f"{resources['scan_context_bytes_per_descriptor']} B "
                    f"({_format(resources['reduction_factor'], 1)}x larger)")

        communication = robot.get("communication")
        if communication:
            lines.append(f"  communication  over {communication['reports']} "
                         f"diagnostic reports")
            lines.append(
                f"    announcements {communication['announcement_messages_sent']} "
                f"msg / {_format(communication['announcement_kib_sent'], 1)} kiB")
            lines.append(
                f"    scans {communication['scan_messages_sent']} msg / "
                f"{_format(communication['scan_kib_sent'], 1)} kiB "
                f"({_format(communication['scan_share_of_sent'])} of bytes sent)")
            if communication.get("scan_latency_samples"):
                lines.append(
                    f"    scan latency mean "
                    f"{_format(communication['scan_latency_mean_ms'], 1)} ms  "
                    f"max {_format(communication['scan_latency_max_ms'], 1)} ms")
        lines.append("")

    comparison = results.get("comparison")
    if comparison:
        lines.append("Comparison against expected values")
        lines.append("-" * 72)
        for row in comparison:
            lines.append(
                f"  {row['metric']:<32} measured {_format(row['measured'])}  "
                f"expected {_format(row['expected'])}  "
                f"{row['verdict']}")
        lines.append("")
    elif results.get("expected_declared") is False:
        lines.append("No expected values are recorded in the manifest, so "
                     "nothing is compared. Fill in the `expected` block with "
                     "the paper's published figures to make this a "
                     "regression check.")
        lines.append("")

    return "\n".join(lines)


def render_json(results):
    def encode(value):
        if hasattr(value, "as_dict"):
            return value.as_dict()
        if isinstance(value, dict):
            return {key: encode(item) for key, item in value.items()}
        if isinstance(value, list):
            return [encode(item) for item in value]
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        return str(value)

    return json.dumps(encode(results), indent=2, sort_keys=True)
