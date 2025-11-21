#!/usr/bin/env python3
from __future__ import annotations

import sys
from typing import Final

import cv_bridge
import numpy as np
import rclpy
import rclpy.executors
import rclpy.time
import rerun as rr
import rerun.blueprint as rrb
import rerun_urdf
from commander_py import commander_utils
from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Duration, Time
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener

DESCRIPTION = """
# UR10e + Mech-Mind Structured Light Sensor Example
This example demonstrates logging data from a UR10e robot equipped with a Mech-Mind structured light sensor.
We use Rerun to visualize:
- The robot's trajectory and end-effector frames
- Camera intrinsics of the Mech-Mind sensor
- The resulting 3D point cloud
- Synchronized RGB images from the camera
""".strip()

FILTER_MIN_VISIBLE: Final = 500


class RerunTFStreamer(Node):
    """Streams URDF (static geometry) and TF (dynamic transforms) to Rerun."""

    def __init__(self):
        super().__init__("rr_tf_stream")

        # Latching QoS so URDF arrives even if node starts late
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.cv_bridge = cv_bridge.CvBridge()

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
            # Add more mappings as needed
            "/map/camera_frame": "camera_color_optical_frame",
            "map/tcp_frame": "tcp_ee",
        }

        self.urdf_sub = self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile=latching_qos,
            callback_group=self.callback_group,
        )

        self.img_sub = self.create_subscription(
            Image,
            "/mechmind/color_image",
            self.img_callback,
            qos_profile=QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            ),
            callback_group=self.callback_group,
        )

        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            "/mechmind/textured_point_cloud",
            self.point_cloud_callback,
            qos_profile=QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            ),
            callback_group=self.callback_group,
        )

        self.create_timer(0.05, self.timer_callback, callback_group=self.callback_group)

        rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        rr.log(
            "map/box",
            rr.Boxes3D(half_sizes=[1, 1, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            static=True,
        )

        rr.log(
            "map/world",
            rr.Transform3D(
                translation=[0, 0, 0],
                rotation=rr.Quaternion(xyzw=[0, 0, 0, 1]),
                axis_length=0.4,
            ),
            static=True,
        )

        rr.log(
            "map/camera_frame/image",
            rr.Pinhole(
                width=1920,
                height=1200,
                focal_length=[2419.304814724099, 2419.3003402251647],  # fx, fy
                principal_point=[967.438095439022, 597.8902870014496],  # cx, cy
                image_plane_distance=0.3,
            ),
            static=True,
        )

        rr.log(
            "/description",
            rr.TextDocument(DESCRIPTION, media_type=rr.MediaType.MARKDOWN),
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
                parent_frame, child_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            t = tf.transform.translation
            q = tf.transform.rotation

            # if path == "robot/urdf/world":
            #     rr.log(
            #         path,
            #         rr.Transform3D(
            #             translation=[t.x, t.y, t.z],
            #             rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            #             axis_length=0.4,
            #         ),
            #     )
            if path.endswith("tcp_frame") or path.endswith("camera_frame"):
                rr.log(
                    path,
                    rr.Transform3D(
                        translation=[t.x, t.y, t.z],
                        rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                        axis_length=0.15,
                    ),
                )
            else:
                rr.log(
                    path,
                    rr.Transform3D(
                        translation=[t.x, t.y, t.z],
                        rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
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
        # now = self.get_clock().now()
        # rr.set_time(timeline="ros_time", timestamp=now.nanoseconds * 1e-9)
        for path in self.path_to_frame.keys():
            self.log_tf_as_transform3d(path, rclpy.time.Time())

    def img_callback(self, img_msg: Image) -> None:
        """Log camera image to Rerun."""
        time = Time.from_msg(img_msg.header.stamp)
        rr.set_time(timeline="ros_time", timestamp=time.nanoseconds * 1e-9)
        print("Logging image of size", img_msg.width, "x", img_msg.height)
        rr.log(
            "map/camera_frame/image",
            rr.Image(self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")),
        )

    def point_cloud_callback(self, pc_msg: PointCloud2) -> None:
        """Log point cloud to Rerun."""
        time = Time.from_msg(pc_msg.header.stamp)
        rr.set_time(timeline="ros_time", timestamp=time.nanoseconds * 1e-9)

        print("Logging point cloud with", pc_msg.width * pc_msg.height, "points")

        print([f.name for f in pc_msg.fields])

        pts = list(
            point_cloud2.read_points(pc_msg, field_names=["x", "y", "z", "rgb"], skip_nans=False)
        )
        pts = np.array(pts)

        xyz = structured_to_unstructured(pts[["x", "y", "z"]])

        rgb_float = pts["rgb"]
        rgb = self.unpack_rgb_float(rgb_float).astype(np.uint8)

        try:
            tf_to_base = self.tf_buffer.lookup_transform(
                "base",
                "camera_color_optical_frame",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )

            xyz = commander_utils.apply_transform_to_points(xyz, tf_to_base.transform)
        except TransformException as ex:
            print(f"Failed to get transform: {ex}")

        rr.log(
            "map/point_cloud",
            rr.Points3D(
                xyz,
                colors=rgb,
            ),
            static=False,
        )

    @staticmethod
    def unpack_rgb_float(rgb_float):
        rgb_int = rgb_float.view(np.uint32)
        r = (rgb_int >> 16) & 255
        g = (rgb_int >> 8) & 255
        b = rgb_int & 255
        return np.stack([r, g, b], axis=-1)


# ---------------------------------------------------------------------------


def main():
    rr.init("rerun_robot_stream", spawn=True)

    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="Robot View",
                origin="/",
                eye_controls=rrb.EyeControls3D(
                    position=(0.02255, -3.6032, 1.1292),
                    look_target=(0.08612, 0.30224, 0.98864),
                    eye_up=(0.0, 0.0, 1.0),
                    spin_speed=0.0,
                    kind=rrb.Eye3DKind.Orbital,
                    speed=3.3388,
                ),
                line_grid=rrb.archetypes.LineGrid3D(
                    visible=True, spacing=0.1, plane=rr.components.Plane3D.XY
                ),
            ),
            rrb.Vertical(
                rrb.Spatial2DView(name="Camera View", origin="map/camera_frame/image"),
                rrb.TextDocumentView(name="Description", origin="/description"),
            ),
            column_shares=[3, 2],
        ),
        rrb.BlueprintPanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
        rrb.TimePanel(state="expanded"),
    )

    rr.send_blueprint(blueprint)

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
