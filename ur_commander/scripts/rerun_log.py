#!/usr/bin/env python3
from __future__ import annotations

import sys
from email.mime import application

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
    """Streams URDF (static geometry) and TF (dynamic transforms) to Rerun."""

    def __init__(self):
        super().__init__("rr_tf_stream")

        # Latching QoS so URDF arrives even if node starts late
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.path_to_frame = {
            "/robot/urdf/world": "world",
            "/robot/urdf/world/base_link": "base_link",
            "/robot/urdf/world/base_link/base_link_inertia": "base_link_inertia",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link": "shoulder_link",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link": "upper_arm_link",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link": "forearm_link",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link": "wrist_1_link",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link": "wrist_2_link",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link": "wrist_3_link",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/flange": "flange",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/flange/tool0": "tool0",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/flange/tool0/camera_color_optical_frame": "camera_color_optical_frame",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/flange/tool0/tcp_ee": "tcp_ee",
            "/robot/urdf/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/forearm_link/wrist_1_link/wrist_2_link/wrist_3_link/flange/tool0/tcp_ee/suction_cup_frame": "suction_cup_frame",
        }

        self.urdf_sub = self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile=latching_qos,
            callback_group=self.callback_group,
        )

        self.create_timer(0.1, self.timer_callback, callback_group=self.callback_group)

        rr.log(
            "map/box",
            rr.Boxes3D(half_sizes=[1, 1, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            static=True,
        )

    # -----------------------------------------------------------------------

    def log_tf_as_transform3d(self, path: str, time: Time) -> None:
        parent_path = path.rsplit("/", 1)[0]

        child_frame = self.path_to_frame.get(path, None)
        parent_frame = self.path_to_frame.get(parent_path, None)

        if child_frame is None:
            print(f"No frame mapping for {path}, skipping")
            return

        if parent_frame is None:
            if child_frame == "world":
                return
            else:
                parent_frame = "world"

        try:
            tf = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, time, timeout=Duration(seconds=0.1)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(
                path,
                rr.Transform3D(
                    translation=[t.x, t.y, t.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])
                ),
            )

        except TransformException as ex:
            print(f"Failed to get transform: {ex}")

    def urdf_callback(self, urdf_msg: String) -> None:
        """Log a URDF using log_scene from rerun_urdf."""
        urdf = rerun_urdf.load_urdf_from_msg(urdf_msg)
        print(f"Loaded URDF with {len(urdf.scene.geometry)} geometries")
        rerun_urdf.log_scene(scene=urdf.scene, node=urdf.base_link, path="robot/urdf", static=True)

    def timer_callback(self):
        """Stream TF transforms every tick."""
        now = self.get_clock().now()
        rr.set_time(timeline="ros_time", timestamp=now.nanoseconds * 1e-9)
        for path in self.path_to_frame.keys():
            self.log_tf_as_transform3d(path, now)


# ---------------------------------------------------------------------------


def main():
    rr.init("rerun_robot_stream", spawn=True)

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
