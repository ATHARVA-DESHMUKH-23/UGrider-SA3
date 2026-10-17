#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation


def inverse_transform(xyz, rpy):
    """
    Input:
        xyz = [x, y, z]
        rpy = [roll, pitch, yaw] in radians

    Returns:
        xyz and rpy for the inverse transform.
    """

    # ROS URDF RPY convention
    R = Rotation.from_euler('xyz', rpy).as_matrix()

    # Homogeneous transform:
    #
    # [ R  t ]
    # [ 0  1 ]
    #
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz

    # Inverse:
    T_inv = np.linalg.inv(T)

    # Extract inverse translation
    xyz_inv = T_inv[:3, 3]

    # Extract inverse rotation
    rpy_inv = Rotation.from_matrix(T_inv[:3, :3]).as_euler('xyz')

    return xyz_inv, rpy_inv


# ============================================================
# ENTER YOUR URDF TRANSFORM HERE
# ============================================================

xyz = [0.02339233017, 0.01322843686, 0.0119545759]

rpy = [
    3.673205355e-06,
    -0.0003667921803,
    1.570796325
]


# Calculate inverse
xyz_offset, rpy_offset = inverse_transform(xyz, rpy)


# ============================================================
# OUTPUT
# ============================================================

print("\nOffset to put in base_link:\n")

print(
    'xyz="'
    + " ".join(f"{x:.12f}" for x in xyz_offset)
    + '"'
)

print(
    'rpy="'
    + " ".join(f"{x:.12f}" for x in rpy_offset)
    + '"'
)

print("\nComplete origin tag:\n")

print(
    '<origin '
    + 'xyz="'
    + " ".join(f"{x:.12f}" for x in xyz_offset)
    + '" '
    + 'rpy="'
    + " ".join(f"{x:.12f}" for x in rpy_offset)
    + '" />'
)