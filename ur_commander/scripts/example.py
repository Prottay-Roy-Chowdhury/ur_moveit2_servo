#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    RobotState,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
)
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient


class PalletizingRobot(Node):
    def __init__(self):
        super().__init__("palletizing_robot")

        # Service client for motion planning
        self.client = self.create_client(GetMotionPlan, "plan_kinematic_path")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for plan_kinematic_path service...")
        self.get_logger().info("Motion planning service available!")

        # Action client for trajectory execution
        self.execute_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        while not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Waiting for execute_trajectory action server...")
        self.get_logger().info("Trajectory execution server available!")

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )
        self.current_joint_state = None

        # Robot configuration
        self.group_name = "fairino10_v6_group"
        self.base_frame = "base_link"
        self.gripper_frame = "grasp_frame"

        # Define target poses (gripper frame relative to base_link)
        self.target_poses = {
            "prepick_pos": Pose(
                position=Point(x=0.800, y=-0.050, z=0.510),
                orientation=Quaternion(x=0.000, y=0.000, z=0.000, w=1.000),
            ),
            "pick_pos": Pose(
                position=Point(x=0.800, y=-0.050, z=0.365),  # 0.315 + 0.05
                orientation=Quaternion(x=0.000, y=0.000, z=0.000, w=1.000),
            ),
            "postpick_pos": Pose(
                position=Point(x=0.800, y=-0.050, z=0.510),
                orientation=Quaternion(x=0.000, y=0.000, z=0.000, w=1.000),
            ),
            "preplace_pos": Pose(
                position=Point(x=0.763, y=0.007, z=0.518),
                orientation=Quaternion(x=-0.052, y=-0.009, z=0.661, w=0.748),
            ),
            "place_pos": Pose(
                position=Point(x=0.799, y=-0.005, z=0.278),
                orientation=Quaternion(x=-0.002, y=0.030, z=0.661, w=0.750),
            ),
            "postplace_pos": Pose(
                position=Point(x=0.763, y=0.007, z=0.518),
                orientation=Quaternion(x=-0.052, y=-0.009, z=0.661, w=0.748),
            ),
            "home_pos": Pose(
                position=Point(x=0.092, y=-0.760, z=0.257),
                orientation=Quaternion(x=-0.004, y=0.003, z=0.049, w=0.999),
            ),
        }

        # Motion parameters
        self.max_velocity = 0.3  # m/s (adjustable)
        self.max_acceleration = 0.3  # m/s^2 (adjustable)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def plan_cartesian_motion(self, target_pose, stage_name):
        if self.current_joint_state is None:
            self.get_logger().error("No current joint state available!")
            return None

        # Motion plan request
        request = GetMotionPlan.Request()
        motion_req = request.motion_plan_request

        # Configure motion request
        motion_req.group_name = self.group_name
        motion_req.planner_id = "pilz_industrial_motion_planner/LIN"
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 5.0
        motion_req.max_velocity_scaling_factor = 1.0  # Adjusted in execution
        motion_req.max_acceleration_scaling_factor = 1.0  # Adjusted in execution

        # Workspace parameters
        motion_req.workspace_parameters.header.frame_id = self.base_frame
        motion_req.workspace_parameters.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
        motion_req.workspace_parameters.max_corner = Vector3(x=2.0, y=2.0, z=2.0)

        # Start state
        motion_req.start_state.joint_state = self.current_joint_state

        # Goal constraints
        constraints = Constraints()

        # Position constraint (tight tolerance for Cartesian precision)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = self.gripper_frame
        pos_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        )
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)

        # Orientation constraint (strict to preserve gripper orientation)
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.base_frame
        ori_constraint.link_name = self.gripper_frame
        ori_constraint.orientation = target_pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.005
        ori_constraint.absolute_y_axis_tolerance = 0.005
        ori_constraint.absolute_z_axis_tolerance = 0.005
        ori_constraint.weight = 1.0
        constraints.orientation_constraints.append(ori_constraint)

        motion_req.goal_constraints.append(constraints)

        # Call motion planning service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if (
            future.result() is None
            or future.result().motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            self.get_logger().error(
                f'Failed to plan motion to {stage_name}: {future.result().motion_plan_response.error_code.val if future.result() else "No response"}'
            )
            return None

        self.get_logger().info(f"Successfully planned motion to {stage_name}")
        return future.result().motion_plan_response.trajectory

    def execute_trajectory(self, trajectory, stage_name):
        # Adjust trajectory speed and acceleration
        for point in trajectory.joint_trajectory.points:
            if point.velocities:
                point.velocities = [
                    min(max(v, -self.max_velocity), self.max_velocity) for v in point.velocities
                ]
            if point.accelerations:
                point.accelerations = [
                    min(max(a, -self.max_acceleration), self.max_acceleration)
                    for a in point.accelerations
                ]

        # Execute trajectory
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        if not send_goal_future.result():
            self.get_logger().error(f"Failed to send trajectory goal for {stage_name}")
            return False

        goal_handle = send_goal_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result():
            self.get_logger().info(f"Successfully executed motion to {stage_name}")
            return True
        else:
            self.get_logger().error(f"Trajectory execution to {stage_name} failed")
            return False

    def move_to(self, stage_name):
        target_pose = self.target_poses[stage_name]
        trajectory = self.plan_cartesian_motion(target_pose, stage_name)
        if trajectory is None:
            return False
        return self.execute_trajectory(trajectory, stage_name)


def main(args=None):
    rclpy.init(args=args)
    node = PalletizingRobot()

    # Wait for joint state
    while node.current_joint_state is None and rclpy.ok():
        rclpy.spin_once(node)
        node.get_logger().info("Waiting for joint states...")

    # Palletizing sequence
    sequence = [
        "home_pos",
        "prepick_pos",
        "pick_pos",
        "postpick_pos",
        "preplace_pos",
        "place_pos",
        "postplace_pos",
        "home_pos",
    ]

    try:
        while rclpy.ok():
            for stage in sequence:
                node.get_logger().info(f"Moving to {stage}...")
                if not node.move_to(stage):
                    node.get_logger().error(
                        f"Failed to complete motion to {stage}. Aborting cycle."
                    )
                    break
            else:
                node.get_logger().info("Completed one palletizing cycle. Repeating...")
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
