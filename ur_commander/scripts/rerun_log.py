#!/usr/bin/env python3
from __future__ import annotations

import sys

import numpy as np
import rclpy
import rclpy.executors
import rerun as rr
import rerun_urdf
import trimesh
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Duration, Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener


class RerunTFStreamer(Node):
    """Streams URDF and live TFs to Rerun."""

    def __init__(self):
        super().__init__("rr_tf_stream")

        # Latching QoS for URDF
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Map Rerun paths to TF frames
        self.path_to_frame = {
            "deco2": "base_link",
            "deco2_camera": "camera_color_optical_frame",
        }

        # Log a bounding box for reference
        rr.log(
            "bounding_box",
            rr.Boxes3D(
                half_sizes=[3, 3, 3],
                centers=[0, 0, 1],
                colors=[255, 255, 255, 255],
            ),
            static=True,
        )

        # Subscriptions
        self.create_subscription(
            CameraInfo,
            "/mechmind/camera_info",
            self.cam_info_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile=latching_qos,
            callback_group=self.callback_group,
        )

    def urdf_callback(self, msg: String):
        """Load and log the URDF once received."""
        self.get_logger().info("Received URDF → loading in Rerun…")

        # Safely load the URDF from the ROS parameter string
        try:
            urdf = rerun_urdf.load_urdf_from_msg(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")
            return

        # Optional: scale camera frame for visualization if exists
        if "camera_color_optical_frame" in urdf.scene.graph.nodes:
            from trimesh.transformations import scale_matrix

            orig = urdf.scene.graph.get("camera_color_optical_frame")[0]
            scale = scale_matrix(0.00254)
            urdf.scene.graph.update(frame_to="camera_color_optical_frame", matrix=orig.dot(scale))

        # Log the URDF to Rerun
        try:
            rerun_urdf.log_scene(
                scene=urdf.scene,
                node=urdf.base_link,
                path="map/robot/urdf",
                static=True,
            )
            self.get_logger().info("URDF successfully logged to Rerun.")
        except Exception as e:
            self.get_logger().error(f"Failed to log URDF scene: {e}")

    def log_tf_as_transform3d(self, path: str, time: Time):
        """Look up a TF transform and log it in Rerun."""
        parent_path = path.rsplit("/", 1)[0]
        child_frame = self.path_to_frame.get(path)
        parent_frame = self.path_to_frame.get(parent_path)

        if not child_frame or not parent_frame:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, time, timeout=Duration(seconds=0.1)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(
                path,
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                ),
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def cam_info_callback(self, info: CameraInfo):
        """Log camera intrinsics."""
        time = Time.from_msg(info.header.stamp)
        rr.set_time("ros_time", np.datetime64(time.nanoseconds, "ns"))

        from image_geometry import PinholeCameraModel

        model = PinholeCameraModel()
        model.fromCameraInfo(info)

        rr.log(
            "map/robot/camera/img",
            rr.Pinhole(
                resolution=[model.width, model.height],
                image_from_camera=model.intrinsicMatrix(),
            ),
        )


def main():
    rr.init("rerun_deco2_stream")

    rclpy.init(args=sys.argv)
    node = RerunTFStreamer()

    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
