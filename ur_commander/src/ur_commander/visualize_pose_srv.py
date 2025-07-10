#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import PoseArray
from ur_commander.srv import VisualizePoses


class VisualizePoseSrv(Node):
    def __init__(self):
        super().__init__("visualize_pose_srv")
        self.publisher = self.create_publisher(
            msg_type=PoseArray,
            topic="/pose_visualizer",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self.pose_visualizer_service = self.create_service(
            VisualizePoses, "visualize_poses", self.visualize_poses_callback
        )

        self.pose_array = PoseArray()

    def visualize_poses_callback(self, request, response):
        self.pose_array.poses = request.poses
        self.pose_array.header.frame_id = request.frame_id
        self.pose_array.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pose_array)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    visualize_pose_srv = VisualizePoseSrv()
    rclpy.spin(visualize_pose_srv)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
