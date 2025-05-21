import os
import sys
import yaml
import rclpy
import numpy as np

from pathlib import Path

# message libraries
from geometry_msgs.msg import PoseStamped, Pose

# moveit_py
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# config file libraries
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


# we need to specify our moveit_py config at the top of each notebook we use.
# this is since we will start spinning a moveit_py node within this notebook.

moveit_config = (
    MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
    .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur10e"})
    .moveit_cpp(Path("config") / "moveit_cpp.yaml")
    .to_moveit_configs()
).to_dict()

# initialise rclpy (only for logging purposes)
rclpy.init()

panda = MoveItPy(node_name="ur", config_dict=moveit_config)
panda_arm = panda.get_planning_component("ur_manipulator")
