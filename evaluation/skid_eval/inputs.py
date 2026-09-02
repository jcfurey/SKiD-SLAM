"""Readers for the per-run result files the harness consumes.

These are deliberately plain CSV so they can be produced by a bag extraction
script, by a log scraper, or by hand, and diffed in review.
"""

import csv

from .place_recognition import Candidate
from .trajectory import Pose


def read_candidates(path):
    """Loop candidates: query_time, match_time, score[, is_true_loop]."""
    candidates = []
    with open(path, newline="", encoding="utf-8") as handle:
        for number, row in enumerate(csv.DictReader(handle), start=2):
            missing = [key for key in ("query_time", "match_time", "score")
                       if row.get(key) in (None, "")]
            if missing:
                raise ValueError(
                    f"{path} line {number}: missing {', '.join(missing)}")
            label = row.get("is_true_loop")
            if label in (None, ""):
                is_true_loop = None
            else:
                is_true_loop = str(label).strip().lower() in {
                    "1", "true", "yes"}
            candidates.append(Candidate(
                row["query_time"], row["match_time"], row["score"],
                is_true_loop))
    if not candidates:
        raise ValueError(f"{path}: no candidates")
    return candidates


def read_registrations(path):
    """Registered loops: query_time, match_time and the estimated transform.

    The transform maps the match frame into the query frame, matching the
    convention of the relative pose the pose graph consumes.
    """
    registrations = []
    fields = ("query_time", "match_time", "tx", "ty", "tz",
              "qx", "qy", "qz", "qw")
    with open(path, newline="", encoding="utf-8") as handle:
        for number, row in enumerate(csv.DictReader(handle), start=2):
            missing = [key for key in fields if row.get(key) in (None, "")]
            if missing:
                raise ValueError(
                    f"{path} line {number}: missing {', '.join(missing)}")
            values = {key: float(row[key]) for key in fields}
            pose = Pose.from_quaternion(
                values["query_time"],
                (values["tx"], values["ty"], values["tz"]),
                (values["qx"], values["qy"], values["qz"], values["qw"]))
            registrations.append((values["query_time"], values["match_time"],
                                  pose))
    if not registrations:
        raise ValueError(f"{path}: no registrations")
    return registrations
