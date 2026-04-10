"""Launch file for IAAC UR10e robot with MoveIt, Servo, vision and visualization nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="Launch in simulation mode if true, real robot mode if false",
        ),
        DeclareLaunchArgument(
            "launch_servo",
            default_value="true",
            description="Launch MoveIt Servo",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="iaac_ur_description",
            description="Package containing the URDF description of the robot.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="deco2_mosaic.xacro",
            description="URDF description file of the robot.",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=os.path.join(
                get_package_share_directory("iaac_ur_description"),
                "config",
                "initial_positions.yaml",
            ),
            description="YAML file with initial joint positions for fake hardware",
        ),
    ]

    sim = LaunchConfiguration("sim")
    launch_servo = LaunchConfiguration("launch_servo")
    ur_description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")

    ur_bringup_launch_file = os.path.join(
        FindPackageShare("ur_robot_driver").find("ur_robot_driver"),
        "launch",
        "ur10e.launch.py",
    )

    moveit_launch_file = os.path.join(
        FindPackageShare("ur_moveit_config").find("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    )

    sim_arguments = {
        "initial_joint_controller": "scaled_joint_trajectory_controller",
        "robot_ip": "xxx.xxx.xxx.xxx",
        "use_fake_hardware": "true",
        "fake_sensor_commands": "true",
        "activate_joint_controller": "true",
        "launch_rviz": "false",
        "description_package": ur_description_package,
        "description_file": description_file,
        "initial_positions_file": initial_positions_file,
    }

    real_robot_arguments = {
        "initial_joint_controller": "scaled_joint_trajectory_controller",
        "robot_ip": "192.168.56.101",
        "use_fake_hardware": "false",
        "fake_sensor_commands": "false",
        "activate_joint_controller": "true",
        "launch_rviz": "false",
        "description_package": ur_description_package,
        "description_file": description_file,
    }

    moveit_arguments = {
        "ur_type": "ur10e",
        "description_package": ur_description_package,
        "description_file": description_file,
        "launch_rviz": "true",
        "launch_servo": launch_servo,
    }

    ur_bringup_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_bringup_launch_file),
                launch_arguments=sim_arguments.items(),
                condition=IfCondition(sim),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_bringup_launch_file),
                launch_arguments=real_robot_arguments.items(),
                condition=UnlessCondition(sim),
            ),
        ]
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments=moveit_arguments.items(),
    )

    visualize_pose_srv_node = Node(
        package="ur_commander",
        executable="commander_viz.py",
        name="visualize_pose_srv_node",
        output="screen",
    )

    hand_to_twist_node = Node(
        package="ur_servo_vision",
        executable="hand_to_twist",
        name="hand_to_twist",
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            ur_bringup_launch,
            moveit_launch,
            visualize_pose_srv_node,
            hand_to_twist_node,
        ]
    )