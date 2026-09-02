"""Rigid and similarity alignment of one trajectory onto another.

Absolute trajectory error is only meaningful after the estimate is expressed in
the reference frame, so the alignment is part of the metric definition, not a
preprocessing detail. Horn's closed form is used: it needs only the dominant
eigenvector of a 4x4 symmetric matrix, which the local Jacobi routine supplies.
"""

import math

from . import linalg


class Alignment:
    """The similarity transform taking source points onto target points."""

    def __init__(self, rotation, translation, scale=1.0):
        self.rotation = rotation
        self.translation = translation
        self.scale = scale

    def apply(self, point):
        rotated = linalg.matvec(self.rotation, point)
        return linalg.add(linalg.scale(rotated, self.scale), self.translation)

    def apply_rotation(self, rotation):
        return linalg.matmul(self.rotation, rotation)


def _centroid(points):
    count = float(len(points))
    return tuple(sum(point[axis] for point in points) / count
                 for axis in range(3))


def horn_alignment(source, target, estimate_scale=False):
    """Least-squares similarity transform mapping source onto target.

    Both arguments are sequences of 3-vectors of equal length. Returns an
    Alignment. Raises ValueError for fewer than three correspondences, where
    the rotation is not determined.
    """
    if len(source) != len(target):
        raise ValueError("source and target must have equal length")
    if len(source) < 3:
        raise ValueError(
            f"need at least 3 correspondences to determine a rotation, "
            f"got {len(source)}")

    source_centroid = _centroid(source)
    target_centroid = _centroid(target)
    source_centred = [linalg.subtract(p, source_centroid) for p in source]
    target_centred = [linalg.subtract(p, target_centroid) for p in target]

    # Cross-covariance.
    covariance = [[0.0] * 3 for _ in range(3)]
    for s, t in zip(source_centred, target_centred):
        for i in range(3):
            for j in range(3):
                covariance[i][j] += s[i] * t[j]

    sxx, sxy, sxz = covariance[0]
    syx, syy, syz = covariance[1]
    szx, szy, szz = covariance[2]

    # Horn's symmetric 4x4; its dominant eigenvector is the optimal rotation
    # as a quaternion in (w, x, y, z) order.
    n = (
        (sxx + syy + szz, syz - szy, szx - sxz, sxy - syx),
        (syz - szy, sxx - syy - szz, sxy + syx, szx + sxz),
        (szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy),
        (sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz),
    )
    _, eigenvectors = linalg.jacobi_eigen(n)
    dominant = tuple(eigenvectors[row][0] for row in range(4))
    quaternion = (dominant[1], dominant[2], dominant[3], dominant[0])
    if linalg.norm(quaternion) < 1e-12:
        raise ValueError("degenerate correspondences; rotation undetermined")
    rotation = linalg.quaternion_to_matrix(quaternion)

    scale = 1.0
    if estimate_scale:
        numerator = 0.0
        denominator = 0.0
        for s, t in zip(source_centred, target_centred):
            numerator += sum(a * b for a, b in zip(t, linalg.matvec(rotation, s)))
            denominator += sum(component * component for component in s)
        if denominator <= 0.0:
            raise ValueError("degenerate correspondences; scale undetermined")
        scale = numerator / denominator

    rotated_centroid = linalg.scale(
        linalg.matvec(rotation, source_centroid), scale)
    translation = linalg.subtract(target_centroid, rotated_centroid)
    return Alignment(rotation, translation, scale)


def align_trajectories(pairs, estimate_scale=False):
    """Alignment taking the estimated poses in `pairs` onto the reference."""
    source = [estimate.translation for estimate, _ in pairs]
    target = [reference.translation for _, reference in pairs]
    return horn_alignment(source, target, estimate_scale=estimate_scale)


def yaw_only_alignment(pairs):
    """Alignment restricted to a rotation about z, for planar platforms.

    Solved in closed form: the optimal yaw is the argument of the summed
    cross-covariance in the xy plane.
    """
    source = [estimate.translation for estimate, _ in pairs]
    target = [reference.translation for _, reference in pairs]
    if len(source) < 2:
        raise ValueError("need at least 2 correspondences for a yaw alignment")

    source_centroid = _centroid(source)
    target_centroid = _centroid(target)

    numerator = 0.0
    denominator = 0.0
    for s, t in zip(source, target):
        sx, sy = s[0] - source_centroid[0], s[1] - source_centroid[1]
        tx, ty = t[0] - target_centroid[0], t[1] - target_centroid[1]
        numerator += sx * ty - sy * tx
        denominator += sx * tx + sy * ty
    yaw = math.atan2(numerator, denominator)

    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0))
    translation = linalg.subtract(
        target_centroid, linalg.matvec(rotation, source_centroid))
    return Alignment(rotation, translation, 1.0)
