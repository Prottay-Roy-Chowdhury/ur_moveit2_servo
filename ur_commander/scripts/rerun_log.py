#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from operator import call
from turtle import color

import cv_bridge
import numpy as np
import rclpy
import rerun as rr
import rerun_urdf
import trimesh
from image_geometry import PinholeCameraModel
from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Duration, Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RerunLogNode(Node):
    def __init__(self) -> None:
        super().__init__("rr_deco2")

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.callback_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.path_to_frame = {
            "deco2": "base_link",
            "deco2_camera": "camera_color_optical_frame",
        }

        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        rr.log(
            "bounding_box",
            rr.Boxes3D(half_sizes=[3, 3, 3], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
            static=True,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            "/mechmind/camera_info",
            self.cam_info_callback,
            10,
            callback_group=self.callback_group,
        )

        self.urdf_sub = self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile=latching_qos,
            callback_group=self.callback_group,
        )

    def urdf_callback(self, msg: String) -> None:
        urdf = rerun_urdf.load_urdf_from_msg(msg)
        orig, _ = urdf.scene.graph.get("camera_link")
        scale = trimesh.transformations.scale_matrix(0.00254)
        urdf.scene.graph.update(frame_to="camera_link", matrix=orig.dot(scale))
        scaled = urdf.scene.scaled(1.0)

        rerun_urdf.log_scene(scene=scaled, node=urdf.base_link, path="map/robot/urdf", static=True)
