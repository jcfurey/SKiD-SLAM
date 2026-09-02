"""Descriptor memory and communication cost.

The paper's resource claim is that a lightweight descriptor makes continuous
inter-robot exchange affordable. The descriptor side of that is exactly
computable from the configuration; the communication side is read back from
the diagnostics the map-fusion node logs.
"""

import re

_FLOAT_BYTES = 4


def solid_descriptor_bytes(knn_feature_dim, num_sectors):
    """Bytes in one SOLiD descriptor: the R-SOLiD and A-SOLiD keys."""
    if knn_feature_dim <= 0 or num_sectors <= 0:
        raise ValueError("descriptor dimensions must be positive")
    return (knn_feature_dim + num_sectors) * _FLOAT_BYTES


def scan_context_bytes(num_rings, num_sectors):
    """Bytes in one Scan Context descriptor: the full ring-by-sector matrix."""
    if num_rings <= 0 or num_sectors <= 0:
        raise ValueError("descriptor dimensions must be positive")
    return num_rings * num_sectors * _FLOAT_BYTES


def descriptor_memory(keyframes, knn_feature_dim, num_sectors,
                      scan_context_rings=None):
    """Descriptor memory for a run, and the Scan Context baseline beside it."""
    if keyframes < 0:
        raise ValueError("keyframes cannot be negative")
    per_descriptor = solid_descriptor_bytes(knn_feature_dim, num_sectors)
    summary = {
        "keyframes": keyframes,
        "solid_bytes_per_descriptor": per_descriptor,
        "solid_total_bytes": per_descriptor * keyframes,
    }
    if scan_context_rings:
        baseline = scan_context_bytes(scan_context_rings, num_sectors)
        summary["scan_context_bytes_per_descriptor"] = baseline
        summary["scan_context_total_bytes"] = baseline * keyframes
        summary["reduction_factor"] = baseline / per_descriptor
    return summary


# The diagnostics line emitted by MapFusion::commsMaintenance. These patterns
# read the numbers out of it; they must track that format string.
_CHANNEL = re.compile(
    r"(?P<channel>announcements|scans): "
    r"sent (?P<sent_messages>\d+) msg / (?P<sent_kib>[\d.]+) kiB, "
    r"received (?P<received_messages>\d+) msg / (?P<received_kib>[\d.]+) kiB"
    r"(?:, latency mean (?P<latency_mean_ms>[\d.]+) ms "
    r"max (?P<latency_max_ms>[\d.]+) ms over (?P<latency_samples>\d+) samples)?")

_CACHE = re.compile(
    r"cache (?P<cached_scans>\d+) scans / (?P<cached_mib>[\d.]+) MiB")
_REQUESTS = re.compile(
    r"requests in flight (?P<in_flight>\d+), retried (?P<retried>\d+), "
    r"abandoned (?P<abandoned>\d+), throttled (?P<throttled>\d+)")
_DEFERRED = re.compile(
    r"deferred (?P<deferred>\d+) \(dropped (?P<deferred_dropped>\d+), "
    r"expired (?P<deferred_expired>\d+)\)")
_BACKLOG = re.compile(
    r"announcement backlog dropped (?P<backlog_dropped_1>\d+)/"
    r"(?P<backlog_dropped_2>\d+)")
_EVICTIONS = re.compile(r"cache evictions (?P<cache_evictions>\d+)")


def parse_comms_line(line):
    """Parse one diagnostics line, or return None when it is not one."""
    channels = {match.group("channel"): match.groupdict()
                for match in _CHANNEL.finditer(line)}
    if "announcements" not in channels or "scans" not in channels:
        return None

    def numbers(group):
        result = {}
        for key, value in group.items():
            if key == "channel" or value is None:
                continue
            result[key] = float(value) if "." in value else int(value)
        return result

    record = {
        "announcements": numbers(channels["announcements"]),
        "scans": numbers(channels["scans"]),
    }
    for pattern in (_CACHE, _REQUESTS, _DEFERRED, _BACKLOG, _EVICTIONS):
        match = pattern.search(line)
        if match:
            for key, value in match.groupdict().items():
                record[key] = float(value) if "." in value else int(value)
    return record


def parse_comms_log(lines):
    """Every diagnostics record in a log, oldest first."""
    records = []
    for line in lines:
        record = parse_comms_line(line)
        if record is not None:
            records.append(record)
    return records


def summarize_communication(records):
    """Headline communication cost from the last record in a log.

    The counters are cumulative, so the final record is the run total. Reading
    them that way avoids double counting the periodic reports.
    """
    if not records:
        raise ValueError("no communication records found")
    last = records[-1]

    announcements = last["announcements"]
    scans = last["scans"]
    total_sent_kib = announcements["sent_kib"] + scans["sent_kib"]
    summary = {
        "reports": len(records),
        "announcement_messages_sent": announcements["sent_messages"],
        "announcement_kib_sent": announcements["sent_kib"],
        "scan_messages_sent": scans["sent_messages"],
        "scan_kib_sent": scans["sent_kib"],
        "total_kib_sent": total_sent_kib,
        "scan_share_of_sent": (scans["sent_kib"] / total_sent_kib
                               if total_sent_kib > 0 else float("nan")),
        "scan_latency_mean_ms": scans.get("latency_mean_ms"),
        "scan_latency_max_ms": scans.get("latency_max_ms"),
        "scan_latency_samples": scans.get("latency_samples", 0),
    }
    for key in ("cached_scans", "cached_mib", "retried", "abandoned",
                "throttled", "deferred_dropped", "deferred_expired",
                "cache_evictions"):
        if key in last:
            summary[key] = last[key]
    return summary
