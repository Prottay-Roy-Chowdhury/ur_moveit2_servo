"""
ROS TF utilities:
Helpers for querying transforms and converting them to matrices.
"""

from geometry_msgs.msg import Pose
from rclpy.time import Time
from .transform_utils import pose_to_matrix
import tf2_ros


def create_tf_buffer_and_listener(node):
    """
    Create a TF2 buffer and listener tied to the given node.

    :param node: rclpy Node
    :return: (tf_buffer, tf_listener)
    """
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    return tf_buffer, tf_listener




def get_transform(tf_buffer: tf2_ros.Buffer,
                  frame_id: str,
                  ee_link: str,
                  time: Time = Time(),
                  logger=None):
    """
    Query a TF transform between frame_id and ee_link.

    :param tf_buffer: TF2 Buffer
    :param frame_id: Target frame
    :param ee_link: Source frame
    :param time: Time to query (default: latest)
    :param logger: Optional ROS logger
    :return: geometry_msgs/TransformStamped or None
    """
    try:
        return tf_buffer.lookup_transform(frame_id, ee_link, time)
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        if logger:
            logger.error(f"TF lookup failed: {e}")
        return None


def get_transform_matrix(tf_buffer: tf2_ros.Buffer,
                         frame_id: str,
                         ee_link: str,
                         time: Time = Time(),
                         logger=None):
    """
    Get the transform between frame_id and ee_link as a 4x4 matrix.

    :param tf_buffer: A tf2_ros.Buffer object
    :param frame_id: Target frame
    :param ee_link: Source frame
    :param time: Time to query (default latest)
    :param logger: Optional ROS logger for errors
    :return: 4x4 numpy matrix or None
    """
    transform = get_transform(tf_buffer, frame_id, ee_link, time, logger)
    if transform is None:
        return None

    # Convert to Pose
    pose = Pose()
    pose.position.x = transform.transform.translation.x
    pose.position.y = transform.transform.translation.y
    pose.position.z = transform.transform.translation.z
    pose.orientation = transform.transform.rotation

    return pose_to_matrix(pose)