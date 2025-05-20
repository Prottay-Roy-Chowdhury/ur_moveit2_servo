#! usr#!/usr/bin/env python3

import time
import rclpy

from rclpy.logging import get_logger

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


class Commander(Node):
    """
    UR Commander Node for controlling the UR robot using MoveItPy.

    This class initializes the MoveItPy interface and sets up the necessary
    parameters for planning and executing motions.
    """

    def __init__(self) -> None:
        super().__init__("ur_commander")
        self.get_logger().info("UR Commander Node Initialized")

        # Initialize MoveItPy
        self.moveit_py = MoveItPy(node_name="ur_commander")
        self.ur_robot = self.moveit_py.get_planning_component("ur_manipulator")
        self.robot_state = RobotState()
        self.pipeline_plan_request_parameters = MultiPipelinePlanRequestParameters()

        # Set up QoS profile for communication
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.RMW_QOS
