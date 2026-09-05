"""Small dense linear algebra, enough for pose evaluation.

Matrices are tuples of row tuples and vectors are plain tuples. Everything here
works on 3x3 and 4x4 problems, so the naive implementations are exact enough
and fast enough, and they keep the harness free of a numeric dependency.
"""

import math

Vector3 = tuple
Matrix3 = tuple


def identity3():
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def transpose(matrix):
    return tuple(zip(*matrix))


def matmul(lhs, rhs):
    rhs_columns = transpose(rhs)
    return tuple(
        tuple(sum(a * b for a, b in zip(row, column)) for column in rhs_columns)
        for row in lhs)


def matvec(matrix, vector):
    return tuple(sum(a * b for a, b in zip(row, vector)) for row in matrix)


def add(lhs, rhs):
    return tuple(a + b for a, b in zip(lhs, rhs))


def subtract(lhs, rhs):
    return tuple(a - b for a, b in zip(lhs, rhs))


def scale(vector, factor):
    return tuple(component * factor for component in vector)


def norm(vector):
    return math.sqrt(sum(component * component for component in vector))


def outer(lhs, rhs):
    return tuple(tuple(a * b for b in rhs) for a in lhs)


def matrix_add(lhs, rhs):
    return tuple(tuple(a + b for a, b in zip(row_l, row_r))
                 for row_l, row_r in zip(lhs, rhs))


def matrix_scale(matrix, factor):
    return tuple(tuple(value * factor for value in row) for row in matrix)


def jacobi_eigen(matrix, max_sweeps=100, tolerance=1e-14):
    """Eigen decomposition of a real symmetric matrix.

    Returns (eigenvalues, eigenvectors) with eigenvectors as columns, sorted by
    descending eigenvalue. The cyclic Jacobi method converges quadratically and
    is unconditionally stable for symmetric input, which is all this harness
    ever forms.
    """
    size = len(matrix)
    a = [list(row) for row in matrix]
    v = [[1.0 if i == j else 0.0 for j in range(size)] for i in range(size)]

    for _ in range(max_sweeps):
        off_diagonal = math.sqrt(sum(
            a[i][j] * a[i][j]
            for i in range(size) for j in range(size) if i != j))
        if off_diagonal <= tolerance:
            break

        for p in range(size - 1):
            for q in range(p + 1, size):
                if abs(a[p][q]) <= tolerance:
                    continue
                theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
                sign = 1.0 if theta >= 0.0 else -1.0
                t = sign / (abs(theta) + math.sqrt(theta * theta + 1.0))
                c = 1.0 / math.sqrt(t * t + 1.0)
                s = t * c

                for k in range(size):
                    a_kp = a[k][p]
                    a_kq = a[k][q]
                    a[k][p] = c * a_kp - s * a_kq
                    a[k][q] = s * a_kp + c * a_kq
                for k in range(size):
                    a_pk = a[p][k]
                    a_qk = a[q][k]
                    a[p][k] = c * a_pk - s * a_qk
                    a[q][k] = s * a_pk + c * a_qk
                for k in range(size):
                    v_kp = v[k][p]
                    v_kq = v[k][q]
                    v[k][p] = c * v_kp - s * v_kq
                    v[k][q] = s * v_kp + c * v_kq

    eigenvalues = [a[i][i] for i in range(size)]
    order = sorted(range(size), key=lambda i: eigenvalues[i], reverse=True)
    sorted_values = tuple(eigenvalues[i] for i in order)
    sorted_vectors = tuple(tuple(v[row][i] for i in order)
                           for row in range(size))
    return sorted_values, sorted_vectors


def quaternion_to_matrix(quaternion):
    """Rotation matrix for a quaternion given as (x, y, z, w)."""
    x, y, z, w = quaternion
    length = math.hypot(x, y, z, w)
    if not math.isfinite(length):
        raise ValueError("quaternion must be finite")
    if length == 0.0:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / length, y / length, z / length, w / length
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def matrix_to_quaternion(matrix):
    """Quaternion (x, y, z, w) for a rotation matrix, via the largest branch."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (matrix[2][1] - matrix[1][2]) * s
        y = (matrix[0][2] - matrix[2][0]) * s
        z = (matrix[1][0] - matrix[0][1]) * s
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = 2.0 * math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2])
        w = (matrix[2][1] - matrix[1][2]) / s
        x = 0.25 * s
        y = (matrix[0][1] + matrix[1][0]) / s
        z = (matrix[0][2] + matrix[2][0]) / s
    elif matrix[1][1] > matrix[2][2]:
        s = 2.0 * math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2])
        w = (matrix[0][2] - matrix[2][0]) / s
        x = (matrix[0][1] + matrix[1][0]) / s
        y = 0.25 * s
        z = (matrix[1][2] + matrix[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1])
        w = (matrix[1][0] - matrix[0][1]) / s
        x = (matrix[0][2] + matrix[2][0]) / s
        y = (matrix[1][2] + matrix[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def rotation_angle(matrix):
    """Geodesic angle of a rotation matrix, in radians, in [0, pi]."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    # Clamp: a matrix that is only numerically orthonormal can push the
    # argument a hair outside the domain of acos.
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.acos(cosine)
