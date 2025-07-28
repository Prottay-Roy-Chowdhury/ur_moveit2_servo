#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from ur_commander.srv import GetTransform


class TFService(Node):
    def __init__(self):
        super().__init__("tf_service")

        # TF2 buffer and listener
        self.tf_buffer    = Buffer()
        self.tf_listener  = TransformListener(self.tf_buffer, self)

        # Create the service
        self.srv = self.create_service(GetTransform, "/get_transform", self.handle_get_transform)

        self.get_logger().info("Service /get_transform ready.")

    def handle_get_transform(self, request, response):
        try:
            tf = self.tf_buffer.lookup_transform(
                request.frame_id,
                request.ee_link,
                Time()
            )
            response.transform  = tf
            response.success    = True
            response.error_msg  = ""
        except Exception as e:
            response.success    = False
            response.error_msg  = str(e)
            self.get_logger().error(f"Failed to lookup transform {request.ee_link}->{request.frame_id}: {e}")
        return response


def main():
    rclpy.init()
    node = TFService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()