#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mecheye_ros_interface.srv import CaptureColorImage
import time  # For simple delay, you can also use timers


class ImageCaptureClient(Node):
    def __init__(self):
        super().__init__("image_capture_client")
        self.cli = self.create_client(CaptureColorImage, "/capture_color_image")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /capture_color_image service...")

    def send_request(self):
        req = CaptureColorImage.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("CaptureColorImage service called successfully")
        else:
            self.get_logger().error("Service call failed")


def main():
    rclpy.init()
    node = ImageCaptureClient()

    try:
        while rclpy.ok():
            node.send_request()
            time.sleep(1.0)  # adjust interval as needed
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
