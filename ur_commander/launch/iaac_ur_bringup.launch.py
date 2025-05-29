import os

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():  # pylint: disable=missing-function-docstring
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="Set to true to launch in simulation mode, false for real robot.",
    )
    kinematics_params_arg = DeclareLaunchArgument(
        "kinematics_params_file",
        default_value=os.path.join(
            get_package_share_directory("ur_commander"),
            "config",
            "ur10e_iaac_calibration.yaml",
        ),
        description="Path to the kinematics parameters file.",
    )

    sim = LaunchConfiguration("sim")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")

    ur_robot_driver_share = get_package_share_directory("ur_robot_driver")
    ur_moveit_share = get_package_share_directory("ur_moveit_config")

    rviz_config_file = os.path.join(
        get_package_share_directory("ur_commander"),
        "config",
        "robot_viz.rviz",
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur10e"})
        .to_moveit_configs()
    )

    sim_launch_args = {
        "robot_ip": "xxx.xxx.xxx",
        "ur_type": "ur10e",
        "use_mock_hardware": "true",
        "mock_sensor_commands": "true",
        "kinematics_params_file": kinematics_params_file,
        "launch_rviz": "false",
    }

    real_launch_args = {
        "robot_ip": "192.168.56.101",
        "ur_type": "ur10e",
        "use_mock_hardware": "false",
        "mock_sensor_commands": "false",
        "kinematics_params_file": kinematics_params_file,
        "launch_rviz": "false",
    }

    ur_robot_driver_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_robot_driver_share, "launch", "ur10e.launch.py")
        ),
        launch_arguments=sim_launch_args.items(),
        condition=IfCondition(sim),
    )

    ur_robot_driver_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_robot_driver_share, "launch", "ur10e.launch.py")
        ),
        launch_arguments=real_launch_args.items(),
        condition=UnlessCondition(sim),
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_moveit_share, "launch", "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur10e",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    commander_viz = Node(
        package="ur_commander",
        executable="commander_viz.py",
        name="commander_viz",
        output="screen",
    )

    return LaunchDescription(
        [
            sim_arg,
            kinematics_params_arg,
            ur_robot_driver_sim,
            ur_robot_driver_real,
            ur_moveit,
            rviz_node,
            commander_viz,
        ]
    )
