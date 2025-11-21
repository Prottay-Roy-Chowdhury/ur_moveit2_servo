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
            default_value="false",
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
    ]

    # Define launch configurations for use in the IncludeLaunchDescription
    sim = LaunchConfiguration("sim")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
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
    }

    moveit_arguments = {
        "ur_type": "ur10e",
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
        name="visualize_pose_srv_node",
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
            {"camera_ip": "192.168.56.100"},  # change to your came ip
            {"user_external_intri": False},
            {"fx": 1727.4641025602748},
            {"fy": 1727.4586926701952},
            {"u": 655.8180825729554},
            {"v": 516.6306500606158},
        ],
    )

    # Custom the camera info publisher to match the camera frame
    camera_info_publisher = Node(
        package="ur_commander",
        executable="camera_info_publisher.py",
        name="camera_info_publisher",
        output="screen",
    )

    # Launch aruco_ros single marker detection node
    aruco_single_node = Node(
        package="aruco_ros",
        executable="single",
        parameters=[
            {
                "image_is_rectified": True,
                "marker_size": 0.15,  # Marker size in meters
                "marker_id": 10,  # Marker ID
                "reference_frame": "",
                "camera_frame": "camera_color_optical_frame",
                "marker_frame": "aruco_marker_frame",
                "corner_refinement": "LINES",
            },
        ],
        remappings=[
            ("/camera_info", "/mechmind/camera_info"),
            ("/image", "/mechmind/color_image"),
        ],
    )

    # Include easy_handeye2 calibration launch
    calibrate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("easy_handeye2").find("easy_handeye2"),
                "launch",
                "calibrate.launch.py",
            )
        ),
        launch_arguments={
            "calibration_type": "eye_in_hand",
            "name": "deco2",
            "robot_base_frame": "base_link",
            "robot_effector_frame": "tool0",
            "tracking_base_frame": "camera_color_optical_frame",
            "tracking_marker_frame": "aruco_marker_frame",
        }.items(),
    )

    # Include easy_handeye2 calibration publish launch for eye-on-bases
    eih_publish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("easy_handeye2").find("easy_handeye2"),
                "launch",
                "publish.launch.py",
            )
        ),
        launch_arguments={
            "name": "deco2",
        }.items(),
    )

    # add static transfrorm publisher for camera to base link with quaternion
    camera_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base_link_publisher",
        output="screen",
        arguments=[
            "0.096507",  # x
            "-0.087104",  # y
            "0.017135",  # z
            "0.035378",  # qx
            "0.002747",  # qy
            "-0.008516",  # qz
            "0.999334",  # qw
            "tool0",  # parent frame
            "camera_color_optical_frame",  # child frame
        ],
    )

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

    return LaunchDescription(
        declared_arguments
        + [
            ur_bringup_launch,
            moveit_launch,
            visualize_pose_srv_node,
            camera_node,
            camera_info_publisher,
            aruco_single_node,
            calibrate_launch,
            eih_publish_launch,
            camera_tf_publisher,
            static_tf_point_cloud_node,
            static_tf_textured_point_cloud_node,
        ]
    )
