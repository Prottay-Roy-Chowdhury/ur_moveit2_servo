"""Launch file for IAAC UR10e robot with MoveIt, Servo and visualization nodes."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
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
    ]

    sim = LaunchConfiguration("sim")
    launch_servo = LaunchConfiguration("launch_servo")
    ur_description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

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

    # Start with scaled controller on startup
    sim_arguments = {
        "initial_joint_controller": "scaled_joint_trajectory_controller",
        "robot_ip": "xxx.xxx.xxx.xxx",
        "use_fake_hardware": "true",
        "fake_sensor_commands": "true",
        "activate_joint_controller": "true",
        "launch_rviz": "false",
        "description_package": ur_description_package,
        "description_file": description_file,
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

    # Deactivate scaled_joint_trajectory_controller
    unspawn_scaled = Node(
        package="controller_manager",
        executable="unspawner",
        arguments=[
            "scaled_joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Activate forward_position_controller
    spawn_forward_position = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Start Servo after controllers have had time to settle
    start_servo = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/servo_node/start_servo",
            "std_srvs/srv/Trigger",
            "{}",
        ],
        shell=False,
        output="screen",
    )

    # # Optional: switch Servo to Twist command mode
    # switch_servo_to_twist = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "service",
    #         "call",
    #         "/servo_node/switch_command_type",
    #         "moveit_msgs/srv/ServoCommandType",
    #         "{command_type: 1}",
    #     ],
    #     shell=False,
    #     output="screen",
    # )

    # Chain the steps after MoveIt starts
    controller_switch_sequence = RegisterEventHandler(
        OnProcessStart(
            target_action=visualize_pose_srv_node,
            on_start=[
                TimerAction(period=3.0, actions=[unspawn_scaled]),
                TimerAction(period=5.0, actions=[spawn_forward_position]),
                TimerAction(period=7.0, actions=[start_servo]),
                # TimerAction(period=8.0, actions=[switch_servo_to_twist]),
            ],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            ur_bringup_launch,
            moveit_launch,
            visualize_pose_srv_node,
            controller_switch_sequence,
        ]
    )