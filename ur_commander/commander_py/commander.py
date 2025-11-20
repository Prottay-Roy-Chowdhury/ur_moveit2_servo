#!/usr/bin/env python3

from typing import List, Literal, Optional, Tuple, Union

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup, MoveGroupSequence
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MotionPlanResponse,
    MotionSequenceItem,
    MotionSequenceRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetMotionSequence,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.task import Future
from requests import request
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray


class Commander:
    """
    UR Commander Node for controlling the UR robot using python.

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
        self._move_group = move_group
        self._base_frame = base_frame
        self._ee_frame = end_effector_frame
        self._node.get_logger().info("UR Commander Node Initialized")
        self._planning_scene = None
        self._joint_state = None
        self._joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Initialize the subscriber for the joint state
        self._joint_state_subscriber = self._node.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            callback=self._joint_state_callback,
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

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

        # Initialize the action client for move group,
        # apparently we can only animate the motion through this action client
        self._move_action_client = ActionClient(
            node=self._node,
            action_type=MoveGroup,
            action_name="move_action",
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

        # Initialize the action client for sequence move group
        self._move_sequence_action_client = ActionClient(
            node=self._node,
            action_type=MoveGroupSequence,
            action_name="sequence_move_group",
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
            srv_name="/plan_sequence_path",
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

        self._get_planning_scene_srv = self._node.create_client(
            srv_type=GetPlanningScene,
            srv_name="/get_planning_scene",
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

        self.trajectory_visualization_publisher = self._node.create_publisher(
            msg_type=MarkerArray,
            topic="/commander_viz/trajectory",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self._collision_object_publisher = self._node.create_publisher(
            msg_type=CollisionObject,
            topic="/collision_object",
            qos_profile=10,
        )

        self._attached_collision_object_publisher = self._node.create_publisher(
            msg_type=AttachedCollisionObject,
            topic="/attached_collision_object",
            qos_profile=10,
        )

    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        wait_until_executed: bool = True,
    ) -> Optional[Future]:
        """
        Execute the given trajectory using the ExecuteTrajectory action client.

        :param trajectory: The trajectory to execute.
        :param wait: If True, waits for the execution to complete.
        :return: Future object if wait is False, otherwise None.
        """
        if trajectory is None:
            self._node.get_logger().error("No trajectory provided for execution")
            return None

        if not self._execute_trajectory_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error("Action server /execute_trajectory is not available")
            return None

        execute_goal = self._construct_execute_trajectory_goal(trajectory)

        if execute_goal is None:
            return None

        send_goal_future = self._execute_trajectory_action_client.send_goal_async(
            goal=execute_goal,
            feedback_callback=None,
        )

        self._node.get_logger().info(
            "Sending goal to action server /execute_trajectory with trajectory"
        )

        while not send_goal_future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error(
                "Goal was rejected by the action server /execute_trajectory"
            )
            return None

        self._node.get_logger().info("Goal accepted by action server /execute_trajectory")

        get_result_future = goal_handle.get_result_async()

        if not wait_until_executed:
            return get_result_future

        while rclpy.ok() and not get_result_future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.get_logger().info("Execution completed")
        return get_result_future.result()

    def plan(
        self,
        pose_goal: MotionPlanRequest = None,
        joint_goal: MotionPlanRequest = None,
        pipeline_id: Literal["ompl", "pilz_industrial_motion_planner", "stomp", "chomp"] = "ompl",
        planner_id: Literal["", "PTP", "LIN", "CIRC", "CHOMP", "STOMP"] = "",
        ee_frame: Optional[str] = None,
        acc_scale: float = 0.1,
        vel_scale: float = 0.1,
    ) -> MotionPlanResponse:
        """
        Plans a motion based on the provided pose or joint goal.

        :param pose_goal: MotionPlanRequest containing pose constraints.
        :param joint_goal: MotionPlanRequest containing joint constraints.
        :param planner_id: The planner to use for the motion plan.
        :param pipeline_id: The pipeline to use for the motion plan.
        :param acc_scale: Acceleration scaling factor.
        :param vel_scale: Velocity scaling factor.
        :return: MotionPlanResponse containing the planned trajectory.
        """
        if pose_goal is None and joint_goal is None:
            self._node.get_logger().error("No goal provided for planning")
            return None

        if pose_goal is not None and joint_goal is not None:
            self._node.get_logger().error(
                "Both pose and joint goals provided, only one should be set"
            )
            return None

        if not self._plan_kinematic_path_srv.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(
                "Service /plan_kinematic_path is not available, cannot plan motion"
            )
            return None

        if not self._move_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(
                "Action server /move_action is not available, cannot plan motion"
            )
            return None

        # Use the provided goal type
        goal_req = pose_goal if pose_goal is not None else joint_goal

        # Set planner and pipeline IDs
        goal_req.planner_id = planner_id
        goal_req.pipeline_id = pipeline_id
        goal_req.max_acceleration_scaling_factor = acc_scale
        goal_req.max_velocity_scaling_factor = vel_scale

        move_action_goal = MoveGroup.Goal()
        move_action_goal.request = goal_req
        move_action_goal.planning_options.plan_only = True

        send_goal_future = self._move_action_client.send_goal_async(
            goal=move_action_goal,
            feedback_callback=None,
        )

        while not send_goal_future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self._node.get_logger().error("Goal was rejected by the action server /move_action")
            return None

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f"Planning failed with error code: {result.error_code.val}"
            )
            return None
        self._node.get_logger().info("Planning successful")

        if pose_goal is not None:
            self._visualize_trajectory(
                result.planned_trajectory,
                ee_frame=goal_req.goal_constraints[-1].position_constraints[0].link_name,
            )
        else:
            self._visualize_trajectory(
                result.planned_trajectory,
                ee_frame=ee_frame if ee_frame is not None else self._ee_frame[0],
            )
        return result.planned_trajectory

    def plan_sequence(
        self,
        pose_goals: Optional[List[MotionPlanRequest]] = None,
        joint_goals: Optional[List[MotionPlanRequest]] = None,
        pipeline_id: Literal[
            "ompl", "pilz_industrial_motion_planner"
        ] = "pilz_industrial_motion_planner",
        planner_ids: Optional[List[Literal["PTP", "LIN", "CIRC", ""]]] = None,
        blends: Optional[List[float]] = None,
        ee_frame: Optional[str] = None,
        acc_scale: float = 0.1,
        vel_scale: float = 0.1,
    ) -> Optional[RobotTrajectory]:
        """
        Plans a motion sequence based on the provided pose or joint goals.

        :param pose_goals: List of MotionPlanRequest containing pose constraints.
        :param joint_goals: List of MotionPlanRequest containing joint constraints.
        :param pipeline_id: The pipeline to use for the motion plan ("ompl" or "pilz_industrial_motion_planner").
        :param planner_ids: List of planner IDs for each goal.
        :param blends: List of blend radii for each goal.
        :param acc_scale: Acceleration scaling factor.
        :param vel_scale: Velocity scaling factor.
        :return: RobotTrajectory containing the planned trajectory.
        """
        if pose_goals is None and joint_goals is None:
            self._node.get_logger().error("No goals provided for planning sequence")
            return None

        if pose_goals is not None and joint_goals is not None:
            self._node.get_logger().error(
                "Both pose and joint goals provided, only one should be set"
            )
            return None

        if not self._plan_sequence_srv.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(
                "Service /plan_sequence_path is not available, cannot plan motion sequence"
            )
            return None

        # Use the provided goal type
        goal_req = pose_goals if pose_goals is not None else joint_goals

        # Set planner and pipeline IDs, defaulting if not provided
        if planner_ids is None:
            # Default planner_id for pilz is "PTP", for ompl is ""
            default_id = "PTP" if pipeline_id == "pilz_industrial_motion_planner" else ""
            planner_ids = [default_id] * len(goal_req)
        if blends is None:
            blends = [0.0] * len(goal_req)

        sequence_items = []
        for i, goal in enumerate(goal_req):
            sequence_item = MotionSequenceItem()
            sequence_item.req = goal
            sequence_item.req.planner_id = planner_ids[i]
            sequence_item.req.pipeline_id = pipeline_id
            sequence_item.req.max_acceleration_scaling_factor = acc_scale
            sequence_item.req.max_velocity_scaling_factor = vel_scale
            sequence_item.blend_radius = blends[i]
            sequence_items.append(sequence_item)

        # The last blend value has to be 0.0
        sequence_items[-1].blend_radius = 0.0

        # Remove the start state except the first item
        if len(sequence_items) > 1:
            for item in sequence_items[1:]:
                item.req.start_state = RobotState()

        sequence_request = GetMotionSequence.Request()
        sequence_request.request.items = sequence_items

        # Call the planning service
        future = self._plan_sequence_srv.call_async(sequence_request)

        while not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        try:
            response = future.result()
        except Exception as e:
            self._node.get_logger().error(f"Service call failed: {e}")
            return None

        if response.response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._node.get_logger().error(
                f"Planning segment failed with error code: {response.response.error_code.val}"
            )
            return None

        self._node.get_logger().info("Planning sequence successful")
        self._visualize_trajectory(
            trajectory=response.response.planned_trajectories[0],
            ee_frame=ee_frame if ee_frame is not None else self._ee_frame[0],
        )
        return response.response.planned_trajectories[0]

    def set_pose_goal(
        self,
        pose: Optional[Union[Pose, PoseStamped]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[Union[Quaternion, Tuple[float, float, float, float]]] = None,
        frame_id: Optional[str] = None,
        ee_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> MotionPlanRequest:
        """
        Construct a MotionPlanRequest with the provided parameters.

        :param pose: Pose of the end effector (as Pose or PoseStamped).
        :param position: Position in 3D space (as Point or tuple).
        :param quat_xyzw: Quaternion representing the orientation (as Quaternion or tuple).
        :param frame_id: The frame ID for the constraint.
        :param ee_link: The end effector link name.
        :param tolerance: Tolerance for the constraints.
        :param weight: Weight for the constraints.
        :return: MotionPlanRequest object.
        """
        request = self._consctruct_plan_goal_request(
            group_name=self._move_group,
            frame_id=frame_id,
            ee_frame=ee_link,
        )

        # construct pose stamped if pose is provided
        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(
                stamp=self._node.get_clock().now().to_msg(), frame_id=frame_id or self._base_frame
            )
            pose_stamped.pose = pose

        else:
            if not isinstance(position, Point):
                position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(
                stamp=self._node.get_clock().now().to_msg(), frame_id=frame_id or self._base_frame
            )
            pose_stamped.pose = Pose(
                position=position,
                orientation=quat_xyzw,
            )

        # Set the goal pose in the request
        request.goal_constraints[-1].position_constraints.append(
            self.construct_position_constraint(
                frame_id=pose_stamped.header.frame_id,
                ee_link=ee_link,
                position_xyz=pose_stamped.pose.position,
                tolerance=tolerance,
                weight=weight,
            )
        )
        request.goal_constraints[-1].orientation_constraints.append(
            self.construct_orientation_constraint(
                frame_id=pose_stamped.header.frame_id,
                ee_link=ee_link,
                quat_xyzw=pose_stamped.pose.orientation,
                tolerance=tolerance,
                weight=weight,
            )
        )

        return request

    def set_joint_goal(
        self,
        joint_names: Optional[List[str]] = None,
        joint_values: Optional[List[float]] = None,
        frame_id: Optional[str] = None,
        ee_link: Optional[str] = None,
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> MotionPlanRequest:
        """
        Construct a MotionPlanRequest with joint constraints.

        :param joint_names: List of joint names.
        :param joint_values: List of joint values corresponding to the joint names.
        :param tolerance: Tolerance for the joint constraints.
        :param weight: Weight for the joint constraints.
        :return: MotionPlanRequest object.
        """
        request = self._consctruct_plan_goal_request(
            group_name=self._move_group,
            frame_id=frame_id,
            ee_frame=ee_link,
        )

        # If no joint names are provided, use the default ones
        if joint_names is None:
            joint_names = self._joint_names

        # Construct joint constraints
        request.goal_constraints[-1].joint_constraints = self.construct_joint_constraints(
            joint_names=joint_names,
            joint_values=joint_values,
            tolerance=tolerance,
            weight=weight,
        )

        return request

    def construct_joint_constraints(
        self,
        joint_names: List[str],
        joint_values: List[float],
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> List[JointConstraint]:
        """
        Construct a list of JointConstraint objects based on the provided joint names and values.

        :param joint_names: List of joint names.
        :param joint_values: List of joint values corresponding to the joint names.
        :param tolerance: Tolerance for the joint constraints.
        :param weight: Weight for the joint constraints.
        :return: List of JointConstraint objects.
        """
        constraints = []
        if joint_names is None:
            joint_names = self._joint_names

        for name, value in zip(joint_names, joint_values):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = value
            constraint.tolerance_above = tolerance
            constraint.tolerance_below = tolerance
            constraint.weight = weight
            constraints.append(constraint)

        return constraints

    def construct_orientation_constraint(
        self,
        frame_id: str,
        ee_link: str,
        quat_xyzw: Union[Quaternion, Tuple[float, float, float, float]],
        tolerance: float = 0.001,
        weight: float = 1.0,
        orientation_type: int = 0,
    ) -> OrientationConstraint:
        """
        Construct an OrientationConstraint object based on the provided parameters.

        :param frame_id: The frame ID for the constraint.
        :param ee_link: The end effector link name.
        :param quat_xyzw: Quaternion representing the orientation (as Quaternion or tuple).
        :param tolerance: Tolerance for the orientation constraint.
        :param weight: Weight for the orientation constraint.
        :param orientation_type: Type of orientation constraint (0 for absolute, 1 for relative).
        :return: OrientationConstraint object.
        """
        constraint = OrientationConstraint()
        constraint.header.frame_id = frame_id if frame_id is not None else self._base_frame
        constraint.link_name = ee_link if ee_link is not None else self._ee_frame[0]
        if isinstance(quat_xyzw, Quaternion):
            constraint.orientation = quat_xyzw
        else:
            constraint.orientation.x = float(quat_xyzw[0])
            constraint.orientation.y = float(quat_xyzw[1])
            constraint.orientation.z = float(quat_xyzw[2])
            constraint.orientation.w = float(quat_xyzw[3])

        constraint.absolute_x_axis_tolerance = tolerance
        constraint.absolute_y_axis_tolerance = tolerance
        constraint.absolute_z_axis_tolerance = tolerance

        constraint.weight = weight
        constraint.parameterization = orientation_type
        return constraint

    def construct_position_constraint(
        self,
        frame_id: str,
        ee_link: str,
        position_xyz: Union[Point, Tuple[float, float, float]],
        tolerance: float = 0.001,
        weight: float = 1.0,
    ) -> PositionConstraint:
        """
        Construct a PositionConstraint object based on the provided parameters.

        :param frame_id: The frame ID for the constraint.
        :param ee_link: The end effector link name.
        :param position_xyz: Position in 3D space (as Point or tuple).
        :param tolerance: Tolerance for the position constraint.
        :param weight: Weight for the position constraint.
        :return: PositionConstraint object.
        """
        constraint = PositionConstraint()
        constraint.header.frame_id = frame_id if frame_id is not None else self._base_frame
        constraint.link_name = ee_link if ee_link is not None else self._ee_frame[0]

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [tolerance]
        constraint.constraint_region.primitives.append(primitive)

        pose = Pose()
        if isinstance(position_xyz, Point):
            pose.position = position_xyz
        else:
            pose.position = Point(*position_xyz)
        pose.orientation.w = 1.0  # Identity quaternion
        constraint.constraint_region.primitive_poses.append(pose)

        constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        constraint.weight = weight

        return constraint

    def compute_fk(
        self,
        joint_state: Optional[Union[JointState, List[float]]] = None,
        ee_link: Optional[str] = None,
    ) -> Optional[Future]:
        """
        Computes the forward kinematics for the given joint state.

        :param joint_state: JointState message or list of joint values.
        :param ee_link: The end effector link name.
        :return: Future object containing the computed pose.
        """
        if not self._plan_fk_srv.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("Service /compute_fk is not available")
            return None

        request = GetPositionFK.Request()
        request.header.frame_id = self._base_frame
        request.header.stamp = self._node.get_clock().now().to_msg()
        request.fk_link_names = [ee_link] if ee_link is not None else [self._ee_frame]

        if joint_state is not None:
            if isinstance(joint_state, JointState):
                request.robot_state.joint_state = joint_state
                request.robot_state.joint_state.name = self._joint_names
            elif isinstance(joint_state, list):
                joint_state_msg = JointState()
                joint_state_msg.name = self._joint_names
                joint_state_msg.position = joint_state
                request.robot_state.joint_state = joint_state_msg
            else:
                self._node.get_logger().error("Invalid joint state provided for FK computation")
                return None
        else:
            self._node.get_logger().error("No joint state provided for FK computation")
            return None

        future = self._plan_fk_srv.call_async(request)

        while not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if future.result() is None:
            self._node.get_logger().error(
                f"Failed to call service /compute_fk: {future.exception()}"
            )
            return None

        return future.result().pose_stamped[0]

    def add_collision_object(
        self,
        object_id: str,
        object_type: int,
        dimensions: Tuple[float, float, float],
        pose: Pose,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add a collision object to the planning scene.

        :param object_id: Unique identifier for the collision object.
        :param object_type: Type of the collision object (1=box, 2=sphere, 3=cylinder, 4=cone).
        :param dimensions: Dimensions of the collision object (length, width, height).
        :param pose: Pose of the collision object in the planning scene.
        :param frame_id: Frame ID for the pose (optional).
        :param operation: Operation to perform (add, remove, attach).
        """
        if pose is not None:
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=frame_id if frame_id else self._base_frame,
                ),
                pose=pose,
            )

            collision_msg = CollisionObject(
                id=object_id,
                header=pose_stamped.header,
                pose=pose_stamped.pose,
                operation=operation,
            )
            collision_msg.primitives.append(SolidPrimitive(type=object_type, dimensions=dimensions))

            self._collision_object_publisher.publish(collision_msg)

    def remove_collision_object(self, object_id: str):
        """
        Remove a collision object from the planning scene.
        """
        collision_msg = CollisionObject(
            id=object_id,
            operation=CollisionObject.REMOVE,
        )
        collision_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._collision_object_publisher.publish(collision_msg)

    def attach_collision_object(
        self,
        object_id: str,
        link_name: Optional[str] = None,
        touch_links: List[str] = [],
        weight: float = 0.0,
    ) -> None:
        """
        Attach a collision object to a link in the planning scene.
        """

        attach_msg = AttachedCollisionObject(
            object=CollisionObject(id=object_id, operation=CollisionObject.ADD),
            link_name=link_name,
            touch_links=touch_links,
            weight=weight,
        )

        self._attached_collision_object_publisher.publish(attach_msg)

    def detach_collision_object(self) -> None:
        """
        Detach a collision object from a link in the planning scene.
        """

        detach_msg = AttachedCollisionObject(
            object=CollisionObject(operation=CollisionObject.REMOVE),
        )

        self._attached_collision_object_publisher.publish(detach_msg)

    def _joint_state_callback(self, msg: JointState) -> None:
        """
        Callback for the joint state subscriber.

        Updates the internal joint state variable with the latest message.
        """
        for joint_name in self._joint_names:
            if joint_name not in msg.name:
                return
        self._joint_state = msg

    def _consctruct_plan_goal_request(
        self,
        group_name: str,
        frame_id: str,
        ee_frame: str,
    ) -> MotionPlanRequest:
        """
        Initialize a MotionPlanRequest with the given parameters.

        """
        if not group_name:
            group_name = self._move_group
        if not frame_id:
            frame_id = self._base_frame
        if not ee_frame:
            ee_frame = self._ee_frame[0]

        request = MotionPlanRequest()
        request.group_name = group_name
        # request.cartesian_speed_limited_link = ee_frame
        request.num_planning_attempts = 5
        request.allowed_planning_time = 1.0
        request.max_velocity_scaling_factor = 0.0
        request.max_acceleration_scaling_factor = 0.0
        request.max_cartesian_speed = 0.0

        request.planner_id = ""
        request.pipeline_id = ""

        # Set the start state to the current joint state
        if self._joint_state is not None:
            robot_state = RobotState()
            robot_state.joint_state = self._joint_state
            request.start_state = robot_state

        # Set the planning frame and end effector frame
        request.workspace_parameters.header.frame_id = frame_id
        request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
        # Constraints
        request.path_constraints = Constraints()
        request.goal_constraints = [Constraints()]
        return request

    def _construct_execute_trajectory_goal(
        self,
        trajectory: RobotTrajectory,
    ) -> ExecuteTrajectory.Goal:
        """
        Constructs an ExecuteTrajectory goal from the provided trajectory.

        :param trajectory: The trajectory to execute.
        :return: ExecuteTrajectory.Goal object.
        """
        if trajectory is None:
            self._node.get_logger().error("No trajectory provided for execution")
            return None

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory.joint_trajectory = trajectory.joint_trajectory

        return execute_goal

    def _visualize_trajectory(
        self,
        trajectory: RobotTrajectory,
        ee_frame: Optional[str] = "tool0",
    ) -> None:
        """
        Publish the trajectory for visualization.

        :param trajectory: The trajectory to visualize.
        """
        if trajectory is None:
            self._node.get_logger().error("No trajectory provided for visualization")

        n = len(trajectory.joint_trajectory.points)

        # Clear previous markers
        marker_array = MarkerArray()
        marker = Marker()
        marker.header = Header(
            stamp=self._node.get_clock().now().to_msg(),
            frame_id=self._base_frame,
        )
        marker.ns = "trajectory"
        marker.id = -1
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.trajectory_visualization_publisher.publish(marker_array)

        # render the trajectory as a marker array with gradient color

        for i, point in enumerate(trajectory.joint_trajectory.points):
            ee_pose_stamped = self.compute_fk(
                joint_state=list(point.positions),
                ee_link=ee_frame,
            )
            marker = Marker()
            marker.header = Header(
                stamp=self._node.get_clock().now().to_msg(),
                frame_id=self._base_frame,
            )
            marker.ns = "trajectory"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = Pose()
            marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.02, 0.02

            marker.pose = ee_pose_stamped.pose

            r = 0.8  # Red stays at maximum throughout
            g = (1.0 - i / (n - 1)) * 0.5 if n > 1 else 0.0  # Green decreases from 0.5 to 0
            b = 0.0  # No blue
            marker.color = ColorRGBA(
                r=r,
                g=g,
                b=b,
                a=0.8,
            )

            marker_array.markers.append(marker)
        self.trajectory_visualization_publisher.publish(marker_array)

    @property
    def joint_state(self) -> Optional[JointState]:
        """Return the current joint state of the robot."""
        return self._joint_state
