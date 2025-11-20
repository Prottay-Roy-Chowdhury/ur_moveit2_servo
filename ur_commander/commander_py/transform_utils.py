"""
Transform utilities using SciPy for ROS geometry_msgs/Pose.

This module provides:
- Pose <-> 4x4 transformation matrix
- Simple helpers for using poses mathematically
"""

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from scipy.spatial.transform import Rotation


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """
    Convert a geometry_msgs/Pose to a 4x4 transformation matrix.
    """
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R = Rotation.from_quat(q).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    """
    Convert a 4x4 transformation matrix to a geometry_msgs/Pose.
    """
    pose = Pose()
    pose.position = Point(x=T[0, 3], y=T[1, 3], z=T[2, 3])
    quat = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return pose


def transform_to_matrix(transform: Transform) -> np.ndarray:
    """
    Convert a geometry_msgs/Transform to a 4x4 transformation matrix.
    """
    # Convert Transform to Pose (they have similar fields)
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation = transform.rotation
    return pose_to_matrix(pose)



