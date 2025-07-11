# deco2_robotic_mosaic

## Before Installation

Change your IP address to:
   ```bash
   192.168.56.1
   ```

## 📦 Installation

1. Clone this repository:
   ```bash
   git clone -b vision [https://github.com/Shu980101/AIbotix.git](https://github.com/HuanyuL/deco2_robotic_mosaic.git)
   cd deco2_robotic_mosaic
   ```
   
2. Build image:
   ```bash
   .docker build_image.sh
   ```

3. Run image:
   ```bash
   .docker run_user_nvidia.sh
   ```

---

## 🧰 Launch Instructions

### Start the Camera Node
```bash
ros2 launch dev_ws/src/mecheye_ros2_interface/launch/start_camera.py
```
In a new terminal and select the default camera(Default: 0):

```bash
ros2 run mecheye_ros_interface start
```
### Run Aruco Detection

In a new terminal:

```bash
ros2 run ur_commander aruco_detection.py
```

### Run Hand_eye_calibration

Open deco2_robotic_mosaic/ur_commander/launch/iaac_ur10e.launch.py and check whether calibrate_launch is uncommented. If it is not, uncomment it.

Example launch script:

```bash
    # Include easy_handeye2 calibration launch
    calibrate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("easy_handeye2").find("easy_handeye2"), "launch", "calibrate.launch.py"
            )
        ),
        launch_arguments={
            "calibration_type": "eye_in_hand",
            "name": "pick_n_place_calibration",
            "robot_base_frame": "base_link",
            "robot_effector_frame": "tool0",
            "tracking_base_frame": "camera_color_optical_frame",
            "tracking_marker_frame": "aruco_marker_10",
        }.items(),
    )
```

Once the launch file is ready, build the workspace and source it with the following code:

```bash
    cd dev_ws/
    colcon build
    source install/setup.bash
```

Run:
```bash
    ros2 launch ur_commander iaac_ur10e.launch.py sim:=true pipeline:=pilz
```

Once you have Aruco detected on the screen, a calibration window will pop up, and follow the instructions to do the calibration.
After saving the calibration data, you need to go to the launch file again to comment out the calibrate_launch and uncomment the eob_publish_launch.

Example launch script:

```bash
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
```

After this, launch the following command again—you should see the camera frame appear in the calibrated position.
Run:
```bash
    ros2 launch ur_commander iaac_ur10e.launch.py sim:=true pipeline:=pilz
```
