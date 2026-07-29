#!/usr/bin/env python3
# utils_np.py
# Versioni NumPy-only delle utility geometriche di common.py.
# Usare questo modulo nei nodi che girano FUORI dal container Docker
# (GCS, logger, supervisor) dove CasADi non è installato.
# common.py rimane invariato per l'MPC e gli altri nodi del guardrone.

import numpy as np
from math import atan2

g0 = 9.80665  # [m/s^2]


def wrap_pi(a):
    """Normalizza un angolo nell'intervallo [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def min_angle(alpha):
    """Equivalente NumPy di ca.atan2(ca.sin(a), ca.cos(a))."""
    return np.arctan2(np.sin(alpha), np.cos(alpha))


def quat_to_R(q):
    """
    Converte quaternione [qw, qx, qy, qz] in matrice di rotazione 3x3 (NumPy).
    Equivalente NumPy di utils_pkg.common.quat_to_R (che usa CasADi).
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    return np.array([
        [1 - 2*(y**2 + z**2),  2*(x*y - z*w),      2*(x*z + y*w)     ],
        [2*(x*y + z*w),        1 - 2*(x**2 + z**2), 2*(y*z - x*w)     ],
        [2*(x*z - y*w),        2*(y*z + x*w),        1 - 2*(x**2 + y**2)]
    ])


def quat_to_RPY(q):
    """
    Converte quaternione [qw, qx, qy, qz] in angoli RPY (roll, pitch, yaw) in NumPy.
    Equivalente NumPy di utils_pkg.common.quat_to_RPY (che usa CasADi).
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    # Roll
    sinr_cosp = 2 * (qw*qx + qy*qz)
    cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2 * (qw*qy - qz*qx)
    pitch = np.arcsin(np.clip(sinp, -0.9999, 0.9999))

    # Yaw
    siny_cosp = 2 * (qw*qz + qx*qy)
    cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])
