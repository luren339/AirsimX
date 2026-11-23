"""Frame conventions shared by the AirSim clients.

AirSim uses a North-East-Down (NED) world and a Forward-Right-Down (FRD)
body frame. ROS and most learning tooling use East-North-Up (ENU) for the
world and Forward-Left-Up (FLU) for the body. Both frames are related by
axis permutations, but they are *different* permutations: the world swap
exchanges north/east and flips vertical, while the body transform keeps
forward and flips the lateral and vertical axes.

Keeping the conversion in the SDK gives the ROS bridge, Gymnasium
environment and any user code one implementation to rely on.
"""

import math

import numpy as np

# World: NED -> ENU.
NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
# Body: FRD -> FLU.
FRD_TO_FLU = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def ned_to_enu_vector(x, y, z):
    """Return the (east, north, up) components of a NED vector."""
    return float(y), float(x), float(-z)


def enu_to_ned_vector(east, north, up):
    """Return the (north, east, down) components of an ENU vector."""
    return float(north), float(east), float(-up)


def frd_to_flu_vector(x, y, z):
    """Convert a body-frame vector from FRD to FLU (lateral and vertical flip)."""
    return float(x), float(-y), float(-z)


def flu_to_frd_vector(x, y, z):
    """Convert a body-frame vector from FLU to FRD (lateral and vertical flip)."""
    return float(x), float(-y), float(-z)


def quaternion_to_matrix(w, x, y, z):
    """Convert a (w, x, y, z) quaternion to a 3x3 rotation matrix."""
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError('quaternion must be finite with non-zero norm')
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(matrix):
    """Convert a 3x3 rotation matrix to a normalized (w, x, y, z) quaternion."""
    m = np.asarray(matrix, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0 or not math.isfinite(norm):
        raise ValueError('matrix produced a degenerate quaternion')
    return w / norm, x / norm, y / norm, z / norm


def ned_quaternion_to_enu(w, x, y, z):
    """Express a NED/FRD orientation quaternion in the ENU/FLU convention."""
    rotation = quaternion_to_matrix(w, x, y, z)
    return matrix_to_quaternion(NED_TO_ENU @ rotation @ FRD_TO_FLU)


def enu_quaternion_to_ned(w, x, y, z):
    """Express an ENU/FLU orientation quaternion in the NED/FRD convention."""
    rotation = quaternion_to_matrix(w, x, y, z)
    return matrix_to_quaternion(NED_TO_ENU.T @ rotation @ FRD_TO_FLU)


def enu_yaw_to_ned_quaternion(yaw):
    """Build the AirSim NED quaternion for a ROS (ENU, CCW from east) yaw."""
    half = 0.5 * (math.pi / 2.0 - float(yaw))
    return math.cos(half), 0.0, 0.0, math.sin(half)


def ned_yaw_to_enu(quaternion):
    """Return the ROS yaw (CCW from east) of an AirSim NED orientation."""
    w, x, y, z = quaternion
    enu = ned_quaternion_to_enu(w, x, y, z)
    return math.atan2(2.0 * (enu[0] * enu[3] + enu[1] * enu[2]),
                      1.0 - 2.0 * (enu[2] * enu[2] + enu[3] * enu[3]))


def twist_enu_to_ned(linear, angular):
    """Convert ROS ENU linear and FLU angular twist tuples into AirSim NED."""
    north, east, down = enu_to_ned_vector(*linear)
    roll, pitch, yaw_rate = flu_to_frd_vector(*angular)
    return (north, east, down), (roll, pitch, yaw_rate)


def stamp_from_nanoseconds(timestamp_ns):
    """Split an AirSim nanosecond timestamp into (sec, nanosec)."""
    if timestamp_ns < 0:
        raise ValueError('timestamp cannot be negative')
    return int(timestamp_ns // 1_000_000_000), int(timestamp_ns % 1_000_000_000)
