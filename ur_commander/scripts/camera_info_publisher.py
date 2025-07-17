#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__("single_camera_info_publisher")
        self.pub = self.create_publisher(CameraInfo, "/mechmind/camera_info", 10)

        # Replace with your actual image resolution
        self.width = 1920
        self.height = 1200

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_rgb_optical_frame"
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height
        self.camera_info_msg.distortion_model = "plumb_bob"

        # Directly assign arrays (fix for array.array object)
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [
            2419.304814724099,
            0.0,
            967.438095439022,
            0.0,
            2419.3003402251647,
            597.8902870014496,
            0.0,
            0.0,
            1.0,
        ]
        self.camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info_msg.p = [
            2419.304814724099,
            0.0,
            967.438095439022,
            0.0,
            0.0,
            2419.3003402251647,
            597.8902870014496,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        self.timer = self.create_timer(0.1, self.publish_info)

    def publish_info(self):
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
