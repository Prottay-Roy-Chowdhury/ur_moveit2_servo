#!/usr/bin/env python3

import rclpy

import moveit.core.kinematic_constraints as kinematic_constraints

from typing import Optional, List, Dict, Any, Tuple, Union

from rclpy.logging import get_logger

from rclpy.node import Node
from rclpy.service import Service
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import (
    RobotState,
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import (
    GetMotionPlan,
    ApplyPlanningScene,
    GetMotionSequence,
    GetCartesianPath,
    GetPositionFK,
    GetPositionIK,
)

from moveit_msgs.action import ExecuteTrajectory, MoveGroup, MoveGroupSequence

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
    UR Commander Node for controlling the UR robot using python

    The Movietpy library is too dumb to use, so we are using the basic services
    provided by the moveit interface to control the robot.
    """

    def __init__(
        self,
        node: Node,
        callback_group: Optional[CallbackGroup] = None,
        move_group: str = "ur_manipulator",
        base_frame: str = "base_link",
        end_effector_frame: list = ["tool0"],
    ) -> None:

        self._node = node
        self._callback_group = callback_group
        self._node.get_logger().info("UR Commander Node Initialized")

        self._planning_scene = None
        # Initialize action client for trajectory execution
        self._execute_trajectory_action_client = ActionClient(
            node=self._node,
            action_type=ExecuteTrajectory,
            action_name="execute_trajectory",
            goal_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            result_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            cancel_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            feedback_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            status_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
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

    def _plan_kinematic_path(
        self,
        request: GetMotionPlan.Request,
        timeout: float = 5.0,
    ) -> Union[GetMotionPlan.Response, None]:
        """
        Plan a kinematic path using the MoveIt service
        """
        if not self._plan_kinematic_path_srv.wait_for_service(timeout_sec=timeout):
            return None

        future = self._plan_kinematic_path_srv.call_async(request)
        if future.result() is None:
            self._node.get_logger().error("No result in kinematic path!")
            return None
        return future.result()

    def _plan_cartesian_path(
        self,
        request: GetCartesianPath.Request,
        timeout: float = 5.0,
    ) -> Union[GetCartesianPath.Response, None]:
        """
        Plan a cartesian path using the MoveIt service
        """
        if not self._plan_cartician_path_srv.wait_for_service(timeout_sec=timeout):
            return None

        future = self._plan_cartician_path_srv.call_async(request)
        if future.result() is None:
            self._node.get_logger().error("No result in cartesian path!")
            return None
        return future.result()

    def _plan_ik(
        self,
        request: GetPositionIK.Request,
        timeout: float = 2.0,
    ) -> Union[GetPositionIK.Response, None]:
        """
        Plan an inverse kinematic path using the MoveIt service
        """
        if not self._plan_ik_srv.wait_for_service(timeout_sec=timeout):
            return None

        future = self._plan_ik_srv.call_async(request)
        if future.result() is None:
            self._node.get_logger().error("No result in inverse kinematic path!")
            return None
        return future.result()

    def _plan_fk(
        self,
        request: GetPositionFK.Request,
        timeout: float = 2.0,
    ) -> Union[GetPositionFK.Response, None]:
        """
        Plan a forward kinematic path using the MoveIt service
        """
        if not self._plan_fk_srv.wait_for_service(timeout_sec=timeout):
            return None

        future = self._plan_fk_srv.call_async(request)
        if future.result() is None:
            self._node.get_logger().error("No result in forward kinematic path!")
            return None
        return future.result()

    def set_pose_target(
        self,
        pose: Pose,
        group_name: str = "ur_manipulator",
        base_frame: str = "base_link",
        end_effector_frame: str = "tool0",
        timeout: float = 5.0,
    ) -> Union[GetMotionPlan.Response, None]:
        """
        Set the pose target for the robot using the MoveIt service
        """


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
