import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseArray
import tf2_ros
import math
import tf_transformations
import numpy as np

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # TF broadcasters
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish static transform: base_link -> camera_link
        self.publish_static_transform()

    def publish_static_transform(self):
        """Publish static transform from base_link to camera_link"""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"
        static_transform.child_frame_id = "camera_link"

        # Predefined camera offset and orientation
        static_transform.transform.translation.x = 1.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.2

        # Camera rotation (assuming 90-degree pitch)
        roll, pitch, yaw = map(math.radians, [0, 90, 0])
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        static_transform.transform.rotation.x = quaternion[0]
        static_transform.transform.rotation.y = quaternion[1]
        static_transform.transform.rotation.z = quaternion[2]
        static_transform.transform.rotation.w = quaternion[3]

        self.tf_static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Published static transform: base_link -> camera_link")

    # def publish_static_transform(self):
    #     """Publish static transform from base_link to camera_link"""
    #     static_transform = TransformStamped()
    #     static_transform.header.stamp = self.get_clock().now().to_msg()
    #     static_transform.header.frame_id = "base_link"
    #     static_transform.child_frame_id = "camera_link"

    #     # Predefined camera offset and orientation
    #     static_transform.transform.translation.x = 0.237070
    #     static_transform.transform.translation.y = 0.092182
    #     static_transform.transform.translation.z = 0.429082

    #     # Camera rotation (assuming 90-degree pitch)
    #     roll, pitch, yaw = map(math.radians, [0, 90, 0])
    #     quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

    #     static_transform.transform.rotation.x = 0.037239
    #     static_transform.transform.rotation.y = 0.688763
    #     static_transform.transform.rotation.z = 0.271204
    #     static_transform.transform.rotation.w = 0.671317

    #     self.tf_static_broadcaster.sendTransform(static_transform)
    #     self.get_logger().info("Published static transform: base_link -> camera_link")

def main():
    rclpy.init()
    node = TFPublisher()
    rclpy.spin(node)  # Keep node running
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
