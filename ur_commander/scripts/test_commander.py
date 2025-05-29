from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from commander_py import commander
from geometry_msgs.msg import Pose, Point, Quaternion

rclpy.init()
node = Node("ex_pose_goal")

callback_group = ReentrantCallbackGroup()
commander = commander.Commander(
    node=node, callback_group=callback_group, move_group="ur_manipulator"
)

# executor = rclpy.executors.MultiThreadedExecutor(2)
# executor.add_node(node)``
# executor_thread = Thread(target=executor.spin, daemon=True)
# executor_thread.start()
# node.create_rate(1.0).sleep()

# joint_values = [-1.57, -1.57, -1.57, 0.0, 1.57, 0.0]
# goal = commander.set_joint_goal(joint_values=joint_values)

# pose = Pose()
# pose.position = Point(x=0.0, y=1.0, z=0.5)
# pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
goal = commander.set_pose_goal(
    position=Point(x=0.0, y=0.5, z=0.0),
    quat_xyzw=Quaternion(x=0.0, y=0.0, z=0.0, w=-1.0),
    frame_id="base_link",
)

print("Pose Goal:", goal)

traj = commander.plan(
    pose_goal=goal,
    planner_id="",
    pipeline_id="ompl",
    acc_scale=0.2,
    vel_scale=0.2,
)

# result = commander.compute_fk(
#     joint_state=[-1.57, -1.57, -1.57, 0.0, 1.57, 0.0],
# )

# print("FK Result:", result)


# commander.execute_trajectory(
#     trajectory=traj,
#     wait_until_executed=True,
# )
