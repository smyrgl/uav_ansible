"""PX4 (NED world, FRD body) -> ROS (ENU world, FLU body). Pure functions.

Same composition as px4_ros_com's frame_transforms: the world change is the
NED<->ENU permutation (x<->y, z negated), the body change is a half-turn about
x (y and z negated), and an orientation q (FRD->NED, PX4 order w,x,y,z) becomes
    q_ros = NED_ENU_Q * q * AIRCRAFT_BASELINK_Q
which in matrix form is R_ros = R_ned_enu @ R(q) @ R_frd_flu. ROS quaternions
are returned as (x, y, z, w).
"""
import math

R_NED_ENU = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))   # own inverse
R_FRD_FLU = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))  # own inverse


def ned_to_enu(v):
    """World vector NED -> ENU."""
    return (float(v[1]), float(v[0]), -float(v[2]))


def frd_to_flu(v):
    """Body vector FRD -> FLU."""
    return (float(v[0]), -float(v[1]), -float(v[2]))


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def rotation_from_px4_quaternion(q):
    """PX4 quaternion (w, x, y, z), Hamilton, body FRD -> world NED, as a 3x3 matrix."""
    w, x, y, z = (float(c) for c in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if not n or not math.isfinite(n):
        raise ValueError("invalid quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def quaternion_from_rotation(r):
    """3x3 rotation -> ROS quaternion (x, y, z, w), Hamilton."""
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2
        w, x, y, z = (r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2
        w, x, y, z = (r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2
        w, x, y, z = (r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    return (x, y, z, w)


def px4_to_ros_rotation(q):
    """R(body FLU -> world ENU) from a PX4 attitude quaternion (FRD -> NED)."""
    return _matmul(_matmul(R_NED_ENU, rotation_from_px4_quaternion(q)), R_FRD_FLU)


def px4_to_ros_orientation(q):
    """PX4 attitude (w, x, y, z; FRD->NED) -> ROS orientation (x, y, z, w; FLU->ENU)."""
    return quaternion_from_rotation(px4_to_ros_rotation(q))


def world_to_body(r_body_world, v_world):
    """Express a world (ENU) vector in the body (FLU) frame given R(body->world)."""
    rt = _transpose(r_body_world)
    return tuple(sum(rt[i][k] * v_world[k] for k in range(3)) for i in range(3))


def yaw_from_rotation(r):
    return math.atan2(r[1][0], r[0][0])


def diag_covariance(variances, order=(0, 1, 2)):
    """Row-major 3x3 covariance with the given per-axis variances, reordered
    (NED (n,e,d) variances become ENU (e,n,d): order (1, 0, 2))."""
    v = [float(variances[i]) for i in order]
    return [v[0], 0.0, 0.0, 0.0, v[1], 0.0, 0.0, 0.0, v[2]]


def finite_or_nan(values):
    return [float(x) if math.isfinite(float(x)) else math.nan for x in values]
