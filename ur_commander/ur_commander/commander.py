#! usr#!/usr/bin/env python3

import time
import rclpy

from rclpy.logging import get_logger

from rclpy.node import Node
from rclpy.service import Service
from rclpy.client import Client
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import (
    GetMotionPlan,
    ApplyPlanningScene,
    GetMotionSequence,
    GetCartesianPath,
    GetPositionFK,
    GetPositionIK,
)

from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters


class Commander:
    """
    UR Commander Node for controlling the UR robot using MoveItPy.

    This class initializes the MoveItPy interface and sets up the necessary
    parameters for planning and executing motions.
    """

    def __init__(self, node: Node, callback_group=CallbackGroup) -> None:
        self._node = node
        self._callback_group = callback_group
        self._node.get_logger().info("UR Commander Node Initialized")
        # Initialize MoveItPy
        self.moveit_py = MoveItPy(node_name="ur_commander")
        self.ur_robot = self.moveit_py.get_planning_component("ur_manipulator")
        self.robot_model = self.ur_robot.get_robot_model()
        self.pipeline_plan_request_parameters = MultiPipelinePlanRequestParameters()

        # Initialize the service client for planning

    def _init_service_clients(self):
        """
        Initialize the service clients for motion planning and execution.
        """
        self._node.get_logger().info("Initializing services...")
        self._plan_kinematic_path_srv = self._node.create_service(
            GetMotionPlan,
            "/plan_kinematic_path",
            self._plan_kinematic_path_callback,
        )
        self.get_plan_sequcene_client = self.create_client(GetMotionSequence, "/plan_sequence_path")

    def _plan_kinematic_path_callback(self, request, response):
        """
        Callback for the /plan_kinematic_path service.
        """
        self._node.get_logger().info("Received plan kinematic path request")
        # Extract the request parameters
        start_state = RobotState()
        goal_state = RobotState()
        constraints = []
        max_velocity_scaling_factor = 1.0
        max_acceleration_scaling_factor = 1.0

        # Call the planning function
        plan_result = self.plan_kinematic_path(
            start_state,
            goal_state,
            constraints,
            max_velocity_scaling_factor,
            max_acceleration_scaling_factor,
        )

        # Set the response
        response.planning_time = plan_result.planning_time
        response.error_code = plan_result.error_code
        return response
