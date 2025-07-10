from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class SequencedMotion(Node):
    def __init__(self):
        super().__init__("sequenced_motion")
        self._action_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {self._action_client._action_name}...")

        self._base = "shoulder_pan_joint"
        self._end_effector = "tool0"
        self._move_group_name = "ur_manipulator"

    def _build_motion_plan_request(self, target_pose: Pose) -> MotionPlanRequest:
        req = MotionPlanRequest()

        # General configuration
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

    def execute_sequence(self, target_poses: List[Pose]) -> None:
        goal = MoveGroupSequence.Goal()
        for target_pose in target_poses:
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius=0.1,
                    req=self._build_motion_plan_request(target_pose),
                )
            )
        goal.planning_options.plan_only = True
        goal.request.items[-1].blend_radius = 0.0  # Last radius must be 0

        self.get_logger().info("Sending goal to action server...")
        send_goal_future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            return

        self.get_logger().info("Goal accepted by action server. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.response.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Motion sequence executed successfully.")
        else:
            self.get_logger().error(f"Execution failed with error code: {result.response.error_code.val}")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback.state}")


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
    sequenced_motion.execute_sequence(target_poses)

    rclpy.spin(sequenced_motion)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
