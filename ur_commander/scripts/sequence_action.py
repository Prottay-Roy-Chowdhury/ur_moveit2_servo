from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroupSequence, ExecuteTrajectory
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PositionConstraint,
    RobotTrajectory,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from moveit_msgs.msg import RobotTrajectory


class SequencedMotion(Node):
    def __init__(self):
        super().__init__("sequenced_motion")
        self._action_client_plan = ActionClient(self, MoveGroupSequence, "/sequence_move_group")
        self._action_client_execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        while not self._action_client_plan.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {self._action_client_plan._action_name}...")

        self._base = "shoulder_pan_joint"
        self._end_effector = "tool0"
        self._move_group_name = "ur_manipulator"
        self._planned_trajectory = None

    def _build_motion_plan_request(self, target_pose: Pose) -> MotionPlanRequest:
        req = MotionPlanRequest()

        # General config
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"  # Pilz PTP, LIN, or CIRC
        req.allowed_planning_time = 10.0
        req.group_name = self._move_group_name
        req.max_acceleration_scaling_factor = 0.1
        req.max_velocity_scaling_factor = 0.1
        req.num_planning_attempts = 1000

        # Goal constraints
        req.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id=self._base),
                        link_name=self._end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses=[Pose(position=target_pose.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id=self._base),
                        link_name=self._end_effector,
                        orientation=target_pose.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        return req

    def plan_sequence(self, target_poses: List[Pose]) -> RobotTrajectory:
        goal = MoveGroupSequence.Goal()
        for target_pose in target_poses:
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius=0.1,
                    req=self._build_motion_plan_request(target_pose),
                )
            )
        goal.planning_options.plan_only = True  # Plan only, don't execute
        goal.request.items[-1].blend_radius = 0.0  # Last radius must be 0

        self.get_logger().info("Sending plan request to action server...")
        send_goal_future = self._action_client_plan.send_goal_async(goal)
        send_goal_future.add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by plan action server.")
            return

        self.get_logger().info("Plan accepted. Waiting for trajectory...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.trajectory_callback)

    def trajectory_callback(self, future):
        result = future.result().result
        if result.response.error_code.val == 1:  # Success
            self.get_logger().info("Trajectory planned successfully.")
            self._planned_trajectory = result.response.planned_trajectories[0]
            # self.execute_trajectory(trajectory)
        else:
            self.get_logger().error(f"Planning failed with error code: {result.response.error_code.val}")

    def execute_trajectory(self, trajectory: RobotTrajectory) -> None:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory  # Set the planned trajectory
        self.get_logger().info("Sending trajectory to action server for execution...")
        send_goal_future = self._action_client_execute.send_goal_async(goal)
        send_goal_future.add_done_callback(self.execution_response_callback)

    def execution_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by execution action server.")
            return

        self.get_logger().info("Execution accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.execution_result_callback)

    def execution_result_callback(self, future):
        result = future.result()
        if result.val == 1:  # Success
            self.get_logger().info("Trajectory executed successfully.")
        else:
            self.get_logger().error(f"Execution failed with error code: {result.val}")


def main() -> None:
    rclpy.init()
    sequenced_motion = SequencedMotion()
    target_poses = [
        Pose(
            position=Point(x=0.6, y=0.1, z=0.8),
            orientation=Quaternion(x=0.0, y=0.7071067811865475, z=0.0, w=0.7071067811865476),
        ),
        Pose(
            position=Point(x=0.6, y=0.3, z=0.5),
            orientation=Quaternion(x=0.5, y=0.5, z=0.5, w=0.5),
        ),
        Pose(
            position=Point(x=0.6, y=-0.3, z=0.5),
            orientation=Quaternion(x=0.0, y=0.7071067811865475, z=0.0, w=0.7071067811865476),
        ),
    ]
    sequenced_motion.plan_sequence(target_poses)

    while rclpy.ok() and sequenced_motion._planned_trajectory is None:
        rclpy.spin_once(sequenced_motion)

    if sequenced_motion._planned_trajectory is not None:
        sequenced_motion.execute_trajectory(sequenced_motion._planned_trajectory)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
