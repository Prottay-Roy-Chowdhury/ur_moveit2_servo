#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_commander.srv import GetTransform  # custom service
import time  # for delay between calls


class TFClient(Node):
    def __init__(self):
        super().__init__("tf_client")

        # Create client for the /get_transform service
        self.cli = self.create_client(GetTransform, "/get_transform")

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_transform service...")

    def send_request(self, frame_id="base_link", ee_link="tool0"):
        req = GetTransform.Request()
        req.frame_id = frame_id
        req.ee_link = ee_link

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            tf_msg = future.result().transform
            self.get_logger().info(
                f"Transform {ee_link} -> {frame_id}: "
                f"translation=({tf_msg.transform.translation.x:.3f}, "
                f"{tf_msg.transform.translation.y:.3f}, "
                f"{tf_msg.transform.translation.z:.3f})"
            )
        else:
            error_msg = future.result().error_msg if future.result() else "No result"
            self.get_logger().error(f"Failed to get transform: {error_msg}")


def main():
    rclpy.init()
    node = TFClient()

    try:
        while rclpy.ok():
            node.send_request("base_link", "tcp_ee")
            time.sleep(1.0)  # call every 1 second
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()