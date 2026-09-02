"""Place-recognition precision and recall."""

from . import linalg


class Candidate:
    """One retrieved place match awaiting judgement.

    `score` is the descriptor distance, so smaller is a better match and a
    candidate is accepted when its score is at or below the threshold.
    """

    __slots__ = ("query_time", "match_time", "score", "is_true_loop")

    def __init__(self, query_time, match_time, score, is_true_loop=None):
        self.query_time = float(query_time)
        self.match_time = float(match_time)
        self.score = float(score)
        self.is_true_loop = is_true_loop


def label_candidates(candidates, reference, revisit_distance_m,
                     min_time_gap_s=30.0, max_time_difference=0.05):
    """Label candidates against ground truth, in place.

    A candidate is a true loop when the two ground-truth positions are within
    `revisit_distance_m` and the observations are at least `min_time_gap_s`
    apart: without the time gap every consecutive pose is trivially a "loop".

    A candidate whose timestamps cannot be located in the reference is left
    unlabelled rather than guessed at, and is excluded from the curve.
    """
    labelled = 0
    for candidate in candidates:
        query = reference.nearest(candidate.query_time, max_time_difference)
        match = reference.nearest(candidate.match_time, max_time_difference)
        if query is None or match is None:
            candidate.is_true_loop = None
            continue
        if abs(query.time - match.time) < min_time_gap_s:
            candidate.is_true_loop = False
            labelled += 1
            continue
        distance = linalg.norm(
            linalg.subtract(query.translation, match.translation))
        candidate.is_true_loop = distance <= revisit_distance_m
        labelled += 1
    return labelled


def precision_recall_curve(candidates, total_positives=None):
    """Precision and recall as the acceptance threshold sweeps.

    Returns points ordered by increasing threshold. `total_positives` is the
    number of revisits that genuinely exist in the run; it defaults to the
    number of positive candidates, which measures recall over what retrieval
    proposed rather than over what was there. Pass the true figure when it is
    known, and say which was used when reporting.
    """
    judged = [c for c in candidates if c.is_true_loop is not None]
    if not judged:
        raise ValueError("no labelled candidates")

    positives = (len([c for c in judged if c.is_true_loop])
                 if total_positives is None else total_positives)

    ordered = sorted(judged, key=lambda c: c.score)
    points = []
    accepted_true = 0
    accepted_false = 0
    for index, candidate in enumerate(ordered):
        if candidate.is_true_loop:
            accepted_true += 1
        else:
            accepted_false += 1

        # Only emit a point at the end of a run of equal scores: a threshold
        # cannot separate candidates that score identically.
        if index + 1 < len(ordered) and ordered[index + 1].score == candidate.score:
            continue

        accepted = accepted_true + accepted_false
        precision = accepted_true / accepted if accepted else 1.0
        recall = accepted_true / positives if positives else float("nan")
        f1 = (0.0 if precision + recall == 0.0
              else 2.0 * precision * recall / (precision + recall))
        points.append({
            "threshold": candidate.score,
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "true_positives": accepted_true,
            "false_positives": accepted_false,
        })
    return points


def summarize(points, total_positives):
    """Headline place-recognition figures from a curve."""
    if not points:
        raise ValueError("empty curve")

    best = max(points, key=lambda point: point["f1"])

    # Average precision: precision weighted by the recall it gained.
    average_precision = 0.0
    previous_recall = 0.0
    for point in points:
        gain = point["recall"] - previous_recall
        if gain > 0.0:
            average_precision += point["precision"] * gain
            previous_recall = point["recall"]

    # The strictest threshold that still admits no false positive at all.
    perfect = [p for p in points if p["precision"] >= 1.0]
    recall_at_full_precision = max(
        (p["recall"] for p in perfect), default=0.0)

    return {
        "total_positives": total_positives,
        "max_f1": best["f1"],
        "max_f1_threshold": best["threshold"],
        "precision_at_max_f1": best["precision"],
        "recall_at_max_f1": best["recall"],
        "average_precision": average_precision,
        "recall_at_full_precision": recall_at_full_precision,
    }
