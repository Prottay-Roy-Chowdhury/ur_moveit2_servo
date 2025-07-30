#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from ur_commander.srv import SavePointCloud
import open3d as o3d
import numpy as np
import struct

class SavePointCloudNode(Node):
    def __init__(self):
        super().__init__("save_pointcloud_service")

        self.latest_cloud = None
        self.latest_cloud_textured = None

        # Subscribers
        self.create_subscription(PointCloud2, "/mechmind/point_cloud", self.cloud_cb, 10)
        self.create_subscription(PointCloud2, "/mechmind/textured_point_cloud", self.textured_cb, 10)

        # Service
        self.srv = self.create_service(SavePointCloud, "save_pointcloud", self.save_cloud_callback)
        self.get_logger().info("SavePointCloud service ready.")

    def cloud_cb(self, msg):
        self.latest_cloud = msg

    def textured_cb(self, msg):
        self.latest_cloud_textured = msg

    def save_cloud_callback(self, request, response):
        cloud_msg = self.latest_cloud_textured if request.textured else self.latest_cloud
        if cloud_msg is None:
            response.success = False
            response.message = "No cloud received yet!"
            return response

        # Extract points from PointCloud2
        points = []
        colors = []
        has_rgb = any(f.name == "rgb" for f in cloud_msg.fields)

        for p in point_cloud2.read_points(cloud_msg, skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            points.append([x, y, z])
            if request.textured and has_rgb:
                rgb = p[3]
                # Convert packed float32 RGB to r,g,b (0-1)
                s = struct.pack('f', rgb)
                i = struct.unpack('I', s)[0]
                r = (i & 0x00FF0000) >> 16
                g = (i & 0x0000FF00) >> 8
                b = (i & 0x000000FF)
                colors.append([r/255.0, g/255.0, b/255.0])

        if len(points) == 0:
            response.success = False
            response.message = "Point cloud is empty!"
            return response

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        if request.textured and has_rgb and colors:
            pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

        # Save to file
        o3d.io.write_point_cloud(request.filename, pcd)
        self.get_logger().info(f"Saved point cloud to {request.filename}")
        response.success = True
        response.message = f"Saved to {request.filename}"
        return response


def main():
    rclpy.init()
    node = SavePointCloudNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()