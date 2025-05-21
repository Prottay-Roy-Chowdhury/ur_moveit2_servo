#!/usr/bin/env python3

import rclpy

import moveit.core.kinematic_constraints as kinematic_constraints

from typing import Optional, List, Dict, Any, Tuple, Union

from rclpy.logging import get_logger

from rclpy.node import Node
from rclpy.service import Service
from rclpy.client import Client
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.srv import (
    GetMotionPlan,
    ApplyPlanningScene,
    GetMotionSequence,
    GetCartesianPath,
    GetPositionFK,
    GetPositionIK,
)

from moveit.core.robot_state import RobotState
from moveit.core.controller_manager import ExecutionStatus
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    MultiPipelinePlanRequestParameters,
    PlanRequestParameters,
    TrajectoryExecutionManager,
)


class Commander:
    """
    UR Commander Node for controlling the UR robot using MoveItPy.

    This class initializes the MoveItPy interface and sets up the necessary
    parameters for planning and executing motions.
    """

    def __init__(self, node: Node, callback_group: Optional[CallbackGroup] = None) -> None:
        self._node = node
        self._callback_group = callback_group
        self._node.get_logger().info("UR Commander Node Initialized")
        # Initialize MoveItPy
        self.moveit_py = MoveItPy(node_name="ur_commander")
        self.ur_robot = self.moveit_py.get_planning_component("ur_manipulator")
        self.scene = self.moveit_py.get_planning_scene_monitor()
        self.pipeline_plan_request_parameters = MultiPipelinePlanRequestParameters(
            self.moveit_py, ["ompl", "pilz_lin"]
        )

        # Initialize the service client for planning
        self._plan_kinematic_path_srv = self._node.create_client(
            srv_type=GetMotionPlan,
            srv_name="/plan_kinematic_path",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self._plan_sequence_srv = self._node.create_client(
            srv_type=GetMotionSequence,
            srv_name="/plan_motion_sequence",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self._plan_cartician_path_srv = self._node.create_client(
            srv_type=GetCartesianPath,
            srv_name="/compute_cartesian_path",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self._plan_ik_srv = self._node.create_client(
            srv_type=GetPositionIK,
            srv_name="/compute_ik",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self._plan_fk_srv = self._node.create_client(
            srv_type=GetPositionFK,
            srv_name="/compute_fk",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self._apply_planning_scene_srv = self._node.create_client(
            srv_type=ApplyPlanningScene,
            srv_name="/apply_planning_scene",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

    def set_pose_goal(
        self,
        pose: Optional[Union[Pose, PoseStamped]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[Union[Quaternion, Tuple[float, float, float, float]]] = None,
        ee_link: Optional[str] = None,
        tolerance_potion: float = 0.01,
        tolerance_orientation: float = 0.01,
        weight_position: float = 1.0,
        weight_orientation: float = 1.0,
    ):
        """
        Set the pose goal for the robot.

        Args:
            pose (Pose or PoseStamped, optional): The target pose.
            position (Point or tuple, optional): The target position.
            quat_xyzw (Quaternion or tuple, optional): The target orientation.
            ee_link (str, optional): The end effector link name.
            tolerance_potion (float, optional): Position tolerance.
            tolerance_orientation (float, optional): Orientation tolerance.
            weight_position (float, optional): Weight for position constraint.
            weight_orientation (float, optional): Weight for orientation constraint.
        """

        if pose is None:
            if position is None or quat_xyzw is None:
                raise ValueError("Either pose or both position and quat_xyzw must be provided.")
            pose = Pose()
            pose.position = Point(*position)
            pose.orientation = Quaternion(*quat_xyzw)

        if ee_link is None:
            ee_link = self.ur_robot.get_end_effector_link()

        self.ur_robot.set_pose_target(
            pose,
            end_effector_link=ee_link,
            tolerance_position=tolerance_potion,
            tolerance_orientation=tolerance_orientation,
            weight_position=weight_position,
            weight_orientation=weight_orientation,
        )


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("ur_commander_node")
    commander = Commander(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
