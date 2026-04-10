#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class HandToTwistNode(Node):
    def __init__(self):
        super().__init__("hand_to_twist")

        self.publisher_ = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )

        self.timer = self.create_timer(0.1, self.publish_zero_twist)
        self.get_logger().info("hand_to_twist node started")

    def publish_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandToTwistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()