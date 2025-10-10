"""Launch file for IAAC UR10e Handeye calibration and MoveIt integration."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the 'sim' argument to toggle between simulation and real robot mode
    declared_arguments = [
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="Launch in simulation mode if true, real robot mode if false",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Specify the joint controller to use",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
            ],
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
    # Define launch configurations for use in the IncludeLaunchDescription
    sim = LaunchConfiguration("sim")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    ur_description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    # pipeline = LaunchConfiguration("pipeline")

    # Define the path to the UR launch file
    ur_bringup_launch_file = os.path.join(
        FindPackageShare("ur_robot_driver").find("ur_robot_driver"), "launch", "ur10e.launch.py"
    )

    moveit_launch_file = os.path.join(
        FindPackageShare("ur_moveit_config").find("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    )

    # Define the arguments for both simulation and real robot modes
    sim_arguments = {
        "initial_joint_controller": initial_joint_controller,
        "robot_ip": "xxx.xxx.xxx",
        "use_fake_hardware": "true",
        "fake_sensor_commands": "true",
        "activate_joint_controller": "true",
        "launch_rviz": "false",
    }

    real_robot_arguments = {
        "initial_joint_controller": initial_joint_controller,
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
    }

    # Define the IncludeLaunchDescription with conditional arguments
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

    # Launch moveit
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments=moveit_arguments.items(),
    )

    # Launch the pose visualization node
    visualize_pose_srv_node = Node(
        package="ur_commander",
        executable="commander_viz.py",
        name="commander_viz",
        output="screen",
    )

    camera_node = Node(
        package="mecheye_ros_interface",
        executable="start",
        name="mechmind_camera_publisher_service",
        output="screen",
        prefix="xterm -e",
        parameters=[
            {"save_file": True},
            {"camera_ip": "172.20.112.1"},  # change to your camera ip
            {"user_external_intri": False},
            {"fx": 1727.4641025602748},
            {"fy": 1727.4586926701952},
            {"u": 655.8180825729554},
            {"v": 516.6306500606158},
        ],
    )

    # # add static transfrorm publisher for camera to base link with quaternion
    # camera_tf_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="camera_to_base_link_publisher",
    #     output="screen",
    #     arguments=[
    #         "0.074355",  # x
    #         "-0.103473",  # y
    #         "0.020089",  # z
    #         "-0.002435",  # qx
    #         "0.003356",  # qy
    #         "-0.005016",  # qz
    #         "0.999979",  # qw
    #         "tool0",  # parent frame
    #         "camera_color_optical_frame",  # child frame
    #     ],
    # )

    # The camera nodes for mechmind camera
    static_tf_point_cloud_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "/camera_color_optical_frame",
            "/mechmind_camera/point_cloud",
        ],
    )
    static_tf_textured_point_cloud_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "/camera_color_optical_frame",
            "/mechmind_camera/textured_point_cloud",
        ],
    )

    # Launch the tf_service_server node
    tf_service_node = Node(
        package="ur_commander",
        executable="tf_service_server.py",
        name="tf_service_server",
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            ur_bringup_launch,
            moveit_launch,
            visualize_pose_srv_node,
            tf_service_node,
            camera_node,
            static_tf_point_cloud_node,
            static_tf_textured_point_cloud_node,
        ]
    )
