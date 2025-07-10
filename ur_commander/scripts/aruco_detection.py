#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from mecheye_ros_interface.srv import CaptureColorImage

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Parameters
        self.declare_parameter("aruco_dictionary_name", "DICT_6X6_250")
        self.declare_parameter("aruco_marker_side_length", 0.15)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("marker_prefix", "aruco_marker")

        self.marker_length = self.get_parameter("aruco_marker_side_length").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.marker_prefix = self.get_parameter("marker_prefix").value

        # ArUco dictionary setup
        dict_name = self.get_parameter("aruco_dictionary_name").value
        if dict_name not in ARUCO_DICT:
            self.get_logger().error(f"Unsupported ArUco dictionary: {dict_name}")
            raise ValueError("Invalid ArUco dictionary")

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

        # Dummy camera intrinsics
        self.mtx = np.array(
            [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float32
        )
        self.dst = np.zeros((5,), dtype=np.float32)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for marker pose
        self.marker_pub = self.create_publisher(TransformStamped, "aruco_marker", 10)

        # Image subscriber
        self.subscription = self.create_subscription(
            Image, "/mechmind/color_image", self.image_callback, 10
        )

        # Capture service client
        self.image_client = self.create_client(CaptureColorImage, "/capture_color_image")

        # Start checking for service availability
        self.service_ready = False
        self.create_timer(1.0, self.check_service_available)

    def check_service_available(self):
        if self.image_client.wait_for_service(timeout_sec=0.5):
            if not self.service_ready:
                self.service_ready = True
                self.get_logger().info("Service /capture_color_image is available.")
                self.create_timer(1.0, self.capture_image)  # Start capture loop
        else:
            self.get_logger().info("Waiting for /capture_color_image service...")

    def capture_image(self):
        if not self.service_ready:
            return

        req = CaptureColorImage.Request()
        future = self.image_client.call_async(req)
        future.add_done_callback(self.capture_callback)

    def capture_callback(self, future):
        try:
            _ = future.result()
            self.get_logger().info("Capture triggered")
        except Exception as e:
            self.get_logger().error(f"Capture service call failed: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_image(cv_image, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def process_image(self, image, stamp):
        corners, ids, _ = self.aruco_detector.detectMarkers(image)

        if ids is None:
            self.get_logger().info("No ArUco markers detected.")
            return

        # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        #     corners, self.marker_length, self.mtx, self.dst
        # )

        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
            corners, 0.15, self.mtx, self.dst
        )

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            rotation_matrix = cv2.Rodrigues(rvec)[0]
            quat = R.from_matrix(rotation_matrix).as_quat()

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f"{self.marker_prefix}_{marker_id}"
            t.transform.translation.x = tvec[0]
            t.transform.translation.y = tvec[1]
            t.transform.translation.z = tvec[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(t)
            self.marker_pub.publish(t)

        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i in range(len(ids)):
            cv2.drawFrameAxes(image, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)

        cv2.imshow("Aruco Detection", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
