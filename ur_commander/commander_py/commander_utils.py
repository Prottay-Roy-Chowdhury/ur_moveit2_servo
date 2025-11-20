"""
Utility functions for manipulating poses and quaternions in ROS.

This module provides functions to translate poses, rotate poses using Euler angles,
and convert quaternions to Euler angles.
"""

from typing import List

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Transform
from scipy.spatial.transform import Rotation


def translate_pose(pose: Pose, translation: List[float]) -> Pose:
    """
    Translate a pose by a given translation vector.

    :param pose: A Pose object representing the original pose.
    :param translation: A list of 3 elements representing the translation vector [tx, ty, tz].
    :return: A new pose with the translation applied.
    """
    new_pose = Pose()
    new_pose.position = Point(
        x=pose.position.x + translation[0],
        y=pose.position.y + translation[1],
        z=pose.position.z + translation[2],
    )
    new_pose.orientation = pose.orientation
    return new_pose


def rotate_pose_euler(pose: Pose, euler_deg: List, frame: str = "local") -> Pose:
    """
    Rotate a pose by Euler angles in degrees (XYZ convention).

    :param frame: 'local' or 'world' frame of reference
    """
    rotation = Rotation.from_euler("xyz", euler_deg, degrees=True)
    q_orig = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rot_orig = Rotation.from_quat(q_orig)

    if frame == "local":
        rot_new = rot_orig * rotation
    else:
        rot_new = rotation * rot_orig

    new_pose = Pose()
    new_pose.position = pose.position
    q_new = rot_new.as_quat()
    new_pose.orientation = Quaternion(x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])
    return new_pose


def quat_to_euler(quat: Quaternion) -> List[float]:
    """
    Convert a quaternion to Euler angles in degrees (XYZ convention).

    :param quat: A Quaternion message.
    :return: A list of Euler angles [roll, pitch, yaw] in degrees.
    """
    rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
    euler_angles = rotation.as_euler("xyz", degrees=True)
    return list(euler_angles)


def euler_to_quat(euler_deg: List[float]) -> Quaternion:
    """
    Convert Euler angles in degrees to a Quaternion.

    :param euler_deg: A list of Euler angles [roll, pitch, yaw] in degrees.
    :return: A Quaternion message representing the rotation.
    """
    rotation = Rotation.from_euler("xyz", euler_deg, degrees=True)
    q = rotation.as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


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


def apply_transform_to_points(xyz: np.ndarray, transform: Transform | Pose) -> np.ndarray:
    """
    Apply a geometry_msgs/Transform or geometry_msgs/Pose to a set of 3D points.

    Args:
        xyz: Nx3 array of points.
        transform: Transform or Pose to apply.

    Returns:
        Nx3 array of transformed points.
    """
    # Convert to 4x4 matrix
    if isinstance(transform, Transform):
        T = transform_to_matrix(transform)
    elif isinstance(transform, Pose):
        T = pose_to_matrix(transform)
    else:
        raise TypeError("transform must be geometry_msgs/Transform or Pose")

    # Convert points to homogeneous coordinates
    n_points = xyz.shape[0]
    xyz_h = np.hstack([xyz, np.ones((n_points, 1))])

    # Apply transform
    xyz_transformed_h = (T @ xyz_h.T).T

    # Return only x,y,z
    return xyz_transformed_h[:, :3]
