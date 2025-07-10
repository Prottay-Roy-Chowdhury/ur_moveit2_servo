from threading import Thread
from math import pi

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur

from geometry_msgs.msg import Pose, Quaternion, Point
import rclpy.wait_for_message
from ur_msgs.srv import SetIO
from ur_commander.srv import VisualizePoses

rclpy.init()

# Create node for this example
node = Node("notebook_example")
callback_group = ReentrantCallbackGroup()


def display_poses(poses: list[Pose], frame_id: str = "base_link") -> None:
    client = node.create_client(VisualizePoses, "/visualize_poses")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("service not available, waiting again...")
    client.call(VisualizePoses.Request(poses=poses, frame_id=frame_id))


moveit2 = MoveIt2(
    node=node,
    joint_names=ur.joint_names(),
    base_link_name=ur.base_link_name(),
    end_effector_name=ur.end_effector_name(),
    group_name=ur.MOVE_GROUP_ARM,
    callback_group=callback_group,
)

# Spin the node in background thread(s) and wait a bit for initialization
executor = rclpy.executors.MultiThreadedExecutor(2)
executor.add_node(node)
executor_thread = Thread(target=executor.spin, daemon=True, args=())
executor_thread.start()
node.create_rate(1.0).sleep()

# Scale down velocity and acceleration of joints (percentage of maximum)
moveit2.max_velocity = 0.1
moveit2.max_acceleration = 0.1
synchronous = True
cancel_after_secs = 0.0
cartesian = False
cartesian_max_step = 0.0025
cartesian_fraction_threshold = 0.0
cartesian_jump_threshold = 0.0
cartesian_avoid_collisions = False
moveit2.planner_id = "PTP"

# Add collision objects
moveit2.add_collision_box(
    id="table",
    size=[2.0, 1.0, 0.05],
    position=[0.0, 0.0, -0.025],
    quat_xyzw=[0.0, 0.0, 0.0, 0.0],
)


home_position = [0.0, -pi / 2, pi / 2, 0.0, 0.0, 0.0]
node.get_logger().info("Moving to home position")
traj = moveit2.move_to_configuration(home_position)
if traj is None:
    node.get_logger().error("Failed to move to home position")


moveit2.execute(traj)
success = moveit2.wait_until_executed()
if not success:
    node.get_logger().error("Failed to execute trajectory")
else:
    node.get_logger().info("Moved to home position")


target = Pose(
    position=Point(x=0.6, y=0.0, z=0.5),
    orientation=Quaternion(x=0.0, y=0.7071067811865475, z=0.0, w=0.7071067811865476),
)

display_poses([target])








