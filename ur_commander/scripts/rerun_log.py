#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys

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
