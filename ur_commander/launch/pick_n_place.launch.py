from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    

    # Declare the 'sim' argument to toggle between simulation and real robot mode
    declared_arguments = [
        DeclareLaunchArgument(
            "sim", default_value="false", description="Launch in simulation mode if true, real robot mode if false"
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
            "pipeline",
            default_value="pilz",
            description="Specify the planning pipeline to use",
            choices=["ompl", "pilz"],
        ),
    ]

    # Define launch configurations for use in the IncludeLaunchDescription
    sim = LaunchConfiguration("sim")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    pipeline = LaunchConfiguration("pipeline")

    # Define the path to the UR launch file
    ur_bringup_launch_file = os.path.join(
        FindPackageShare("ur_robot_driver").find("ur_robot_driver"), "launch", "ur10e.launch.py"
    )

    moveit_ompl_launch_file = os.path.join(
        FindPackageShare("ur_moveit_config").find("ur_moveit_config"), "launch", "ur_moveit.launch.py"
    )

    moveit_pilz_launch_file = os.path.join(
        FindPackageShare("ur_moveit_config").find("ur_moveit_config"), "launch", "ur_moveit_pilz.launch.py"
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

    # Conditionally include the appropriate MoveIt launch file based on pipeline choice
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PythonExpression(
                ["'", moveit_ompl_launch_file, "' if '", pipeline, "' == 'ompl' else '", moveit_pilz_launch_file, "'"]
            )
        ),
        launch_arguments=moveit_arguments.items(),
    )

    # Launch the pose visualization node
    visualize_pose_srv_node = Node(
        package="ur_commander",
        executable="visualize_pose_srv.py",
        name="visualize_pose_srv_node",
        output="screen",
    )
    
# # Include easy_handeye2 calibration launch
#     calibrate_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 FindPackageShare("easy_handeye2").find("easy_handeye2"), "launch", "calibrate.launch.py"
#             )
#         ),
#         launch_arguments={
#             "calibration_type": "eye_on_base",
#             "name": "pick_n_place_calibration",
#             "robot_base_frame": "base_link",
#             "robot_effector_frame": "tcp",
#             "tracking_base_frame": "camera_color_optical_frame",
#             "tracking_marker_frame": "aruco_marker",
#         }.items(),
#     )

    # Include easy_handeye2 calibration publish launch for eye-on-base
    eob_publish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("easy_handeye2").find("easy_handeye2"), "launch", "publish.launch.py"
            )
        ),
        launch_arguments={
            "name": "pick_n_place_calibration",
        }.items(),
    )

    # Return the full launch description
    return LaunchDescription(declared_arguments + [ur_bringup_launch, moveit_launch, visualize_pose_srv_node,eob_publish_launch])
#  handeye_rqt_evaluator_launch, calibrate_launch,eob_publish_launch,ros


# [handeye_server-13] [INFO] [1744806788.462713510] [handeye_server]: Calibration saved to /home/shu/.ros2/easy_handeye2/calibrations/pick_n_place_calibration.calib
