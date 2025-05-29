#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from ur_commander.srv import VisualizePoses  # Make sure this matches your .srv file


class CommanderViz(Node):
    def __init__(self):
        super().__init__("commander_viz")

        # Publisher for visualization
        self.pose_pub = self.create_publisher(PoseArray, "/commander_viz/pose_array", 10)

        # Service definition
        self.srv = self.create_service(
            VisualizePoses, "/commander_viz/pose_visualization", self.visualize_callback
        )

        self.get_logger().info("Service server /commander_viz/pose_visualization ready")

    def visualize_callback(self, request=VisualizePoses.Request, response=VisualizePoses.Response):
        # Construct PoseArray message
        pose_array = PoseArray()
        pose_array.header.frame_id = request.frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = request.poses

        # Publish to RViz or other tools
        self.pose_pub.publish(pose_array)
        self.get_logger().info(f"Published {len(pose_array.poses)} poses")

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CommanderViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
