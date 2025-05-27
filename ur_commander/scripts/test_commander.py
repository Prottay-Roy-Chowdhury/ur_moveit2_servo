from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from commander_py import commander

rclpy.init()
node = Node("ex_pose_goal")

callback_group = ReentrantCallbackGroup()
commander = commander.Commander(
    node=node, callback_group=callback_group, move_group="ur_manipulator"
)

# executor = rclpy.executors.MultiThreadedExecutor(2)
# executor.add_node(node)
# executor_thread = Thread(target=executor.spin, daemon=True)
# executor_thread.start()
# node.create_rate(1.0).sleep()

joint_values = [-1.57, -1.57, -1.57, 0.0, 1.57, 0.0]
goal = commander.set_joint_goal(joint_values=joint_values)

traj = commander.plan(
    joint_goal=goal,
    planner_id="CHOMP",
    pipeline_id="chomp",
    acc_scale=0.2,
    vel_scale=0.2,
)

# result = commander.compute_fk(
#     joint_state=[-1.57, -1.57, -1.57, 0.0, 1.57, 0.0],
# )

# print("FK Result:", result)


commander.execute_trajectory(
    trajectory=traj,
    wait_until_executed=True,
)
