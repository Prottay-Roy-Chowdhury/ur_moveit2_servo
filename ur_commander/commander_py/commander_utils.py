"""
Utility functions for manipulating poses and quaternions in ROS.

This module provides functions to translate poses, rotate poses using Euler angles,
and convert quaternions to Euler angles.
"""

from typing import List

from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Point, Pose, Quaternion


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
