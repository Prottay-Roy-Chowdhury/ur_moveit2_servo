import cv2
import time
import os
import glob
from math import sin, cos

from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import find_and_connect, confirm_capture_3d


def get_input_int(min: int, max: int, warning_message: str):
    while True:
        user_input = input()
        if user_input.isdigit() and min <= int(user_input) <= max:
            return int(user_input)
        print(warning_message)


def get_input_float():
    while True:
        user_input = input()
        try:
            return float(user_input)
        except:
            print("Please enter a number.")


def show_and_save_image(image, file_name: str, window_name: str):
    if image.is_empty():
        return
    cv2.imwrite(file_name, image.data())
    print("Save the image to file", file_name)


def save_extrinsic_parameters(extrinsic_parameters: str):
    # Generate a timestamped file name
    curr_time = time.localtime()
    file_name = "ExtrinsicParameters{:04d}{:02d}{:02d}{:02d}{:02d}{:02d}.txt".format(
        curr_time.tm_year,
        curr_time.tm_mon,
        curr_time.tm_mday,
        curr_time.tm_hour,
        curr_time.tm_min,
        curr_time.tm_sec,
    )

    # Write the extrinsic parameters to the file
    with open(file_name, "w") as out_file:
        out_file.write("ExtrinsicParameters:\n")
        out_file.write(extrinsic_parameters)

    print(f"Save result in file {file_name}")


def parse_poses_from_data():
    """Parse the robot poses from the provided data"""
    poses = [
        (-525.0, -490.38, 413.33, 176.98, -1.64, 173.48),
        (-524.98, -634.54, 413.28, 166.18, -2.83, 173.9),
        (-444.76, -634.56, 413.26, 160.99, -1.81, -159.78),
        (-430.83, -511.26, 496.19, 177.13, -4.18, 173.68),
        (-600.24, -315.63, 496.22, -175.02, -4.2, -173.84),
        (-532.98, -238.56, 411.44, 171.4, -8.36, -172.5),
        (-651.29, -238.57, 411.45, -166.91, -0.94, -172.62),
        (-651.3, -566.37, 411.44, 170.25, 2.02, -172.84),
        (-651.29, -632.89, 341.29, 164.56, 7.0, 164.28),
        (-590.54, -632.88, 389.14, 164.56, 7.0, -150.19),
        (-551.44, -626.35, 440.08, 163.11, 3.46, -157.75),
        (-593.65, -529.45, 422.53, 170.59, -0.2, -162.47),
        (-656.05, -546.19, 492.91, 170.59, -0.19, 142.46),
        (-362.61, -546.17, 492.89, 179.26, -11.3, 141.59),
        (-312.24, -489.29, 428.89, -177.45, -15.27, 152.65),
    ]
    return poses


class AutomatedHandEyeCalibrationSample:
    def __init__(self):
        self.camera = Camera()
        self.calibration = HandEyeCalibration()
        self.image_path = "/dev_ws/src/ur_commander/images"
        self.poses = parse_poses_from_data()

    def input_calib_type(self):
        print("\nEnter the number that represents the camera mounting method.")
        print("1: eye-in-hand")
        print("2: eye-to-hand")
        input_type = get_input_int(
            1, 2, "Unknown calibrateType, please enter correct calibrateType number."
        )
        if input_type == 1:
            self.mounting_mode = HandEyeCalibration.CameraMountingMode_EyeInHand
        elif input_type == 2:
            self.mounting_mode = HandEyeCalibration.CameraMountingMode_EyeToHand

    def input_board_type(self):
        print(
            "\nEnter the number that represent the model of your calibration board (the model is labeled on the calibration board)"
        )
        print("1: BDB-5\n2:BDB-6\n3:BDB-7")
        print("4: CGB-020\n5: CGB-035\n6: CGB-050")
        print("7: OCB-005\n8: OCB-010\n9: OCB-015\n10: OCB-020")
        input_type = get_input_int(
            1, 10, "Unknown boardType, please enter correct boardType number."
        )
        if input_type == 1:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_BDB_5
        elif input_type == 2:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_BDB_6
        elif input_type == 3:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_BDB_7
        elif input_type == 4:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_CGB_20
        elif input_type == 5:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_CGB_35
        elif input_type == 6:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_CGB_50
        elif input_type == 7:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_OCB_5
        elif input_type == 8:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_OCB_10
        elif input_type == 9:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_OCB_15
        elif input_type == 10:
            self.board_model = HandEyeCalibration.CalibrationBoardModel_OCB_20

    def input_euler_type(self):
        print("\nEnter the number that represents the Euler angle convention of your robot.")
        print(
            "1: Z-Y'-X'' (intrinsic rotations) : the intrinsic rotations are known as: yaw, pitch and roll"
        )
        print("2: Z-Y'-Z''/OAT (intrinsic rotations) ")
        print("3: X-Y'-Z''(intrinsic rotations) ")
        print("4: Z-X'-Z'' (intrinsic rotations) ")
        print("5: X-Y-Z/WPR (extrinsic rotations) ")
        input_type = get_input_int(
            1, 5, "Unknown eulerType, please enter correct eulerType number."
        )
        self.euler_type_code = input_type

    def euler_to_quad(self, pose_x, pose_y, pose_z, pose_r1, pose_r2, pose_r3):
        a1 = pose_r1 * PI / 180 / 2
        a2 = pose_r2 * PI / 180 / 2
        a3 = pose_r3 * PI / 180 / 2
        if self.euler_type_code == 1:  # Z-Y'-X''
            quad_w = sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a2) * cos(a3)
            quad_x = -sin(a1) * sin(a2) * cos(a3) + sin(a3) * cos(a1) * cos(a2)
            quad_y = sin(a1) * sin(a3) * cos(a2) + sin(a2) * cos(a1) * cos(a3)
            quad_z = sin(a1) * cos(a2) * cos(a3) - sin(a2) * sin(a3) * cos(a1)
        elif self.euler_type_code == 2:  # Z-Y'-Z''
            quad_w = cos(a2) * cos(a1 + a3)
            quad_x = -sin(a2) * sin(a1 - a3)
            quad_y = sin(a2) * cos(a1 - a3)
            quad_z = cos(a2) * sin(a1 + a3)
        elif self.euler_type_code == 3:  # X-Y'-Z''
            quad_w = -sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a2) * cos(a3)
            quad_x = sin(a1) * cos(a2) * cos(a3) + sin(a2) * sin(a3) * cos(a1)
            quad_y = -sin(a1) * sin(a3) * cos(a2) + sin(a2) * cos(a1) * cos(a3)
            quad_z = sin(a1) * sin(a2) * cos(a3) + sin(a3) * cos(a1) * cos(a2)
        elif self.euler_type_code == 4:  # Z-X'-Z''
            quad_w = cos(a2) * cos(a1 + a3)
            quad_x = sin(a2) * cos(a1 - a3)
            quad_y = sin(a2) * sin(a1 - a3)
            quad_z = cos(a2) * sin(a1 + a3)
        elif self.euler_type_code == 5:  # X-Y-Z
            a1 = pose_r3 * PI / 180 / 2
            a3 = pose_r1 * PI / 180 / 2
            quad_w = sin(a1) * sin(a2) * sin(a3) + cos(a1) * cos(a2) * cos(a3)
            quad_x = -sin(a1) * sin(a2) * cos(a3) + sin(a3) * cos(a1) * cos(a2)
            quad_y = sin(a1) * sin(a3) * cos(a2) + sin(a2) * cos(a1) * cos(a3)
            quad_z = sin(a1) * cos(a2) * cos(a3) - sin(a2) * sin(a3) * cos(a1)

        print(f"\nPose {pose_x}, {pose_y}, {pose_z}, {pose_r1}, {pose_r2}, {pose_r3}")
        print(f"Quaternion: {pose_x}, {pose_y}, {pose_z}, {quad_w}, {quad_x}, {quad_y}, {quad_z}")

        return HandEyeTransformation(pose_x, pose_y, pose_z, quad_w, quad_x, quad_y, quad_z)

    def get_image_files(self):
        """Get all image files from the specified directory"""
        image_extensions = ["*.png", "*.jpg", "*.jpeg", "*.tiff", "*.bmp"]
        image_files = []

        for extension in image_extensions:
            image_files.extend(glob.glob(os.path.join(self.image_path, extension)))

        image_files.sort()  # Sort to ensure consistent ordering
        return image_files

    def load_image_as_frame(self, image_path):
        """Load an image file and convert it to the appropriate format for the calibration system"""
        try:
            # Load the image using OpenCV
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                print(f"Failed to load image: {image_path}")
                return None

            # Convert to the format expected by the calibration system
            # Note: You may need to adjust this based on your specific image format requirements
            color_image = Color2DImage()
            # Assuming there's a method to set data from numpy array
            # This part may need adjustment based on your API
            color_image.set_data(cv_image)
            return color_image

        except Exception as e:
            print(f"Error loading image {image_path}: {e}")
            return None

    def automated_calibrate(self):
        print("\n******************************************************************************")
        print("Starting automated hand-eye calibration process...")
        print("Processing images from:", self.image_path)
        print("Number of poses available:", len(self.poses))
        print("******************************************************************************")

        # Get list of image files
        image_files = self.get_image_files()

        if not image_files:
            print(f"No image files found in {self.image_path}")
            return

        print(f"Found {len(image_files)} image files")

        # Process each image with corresponding pose
        successful_poses = 0
        camera_to_base = HandEyeTransformation()

        for i, image_file in enumerate(image_files):
            if i >= len(self.poses):
                print(f"Warning: More images ({len(image_files)}) than poses ({len(self.poses)})")
                break

            pose_index = i + 1
            pose_data = self.poses[i]

            print(f"\n--- Processing pose {pose_index} ---")
            print(f"Image: {os.path.basename(image_file)}")
            print(f"Pose: {pose_data}")

            # Load the image from file instead of capturing
            print("Loading image from file...")
            loaded_image = self.load_image_as_frame(image_file)

            if loaded_image is None:
                print(f"Failed to load image for pose {pose_index}")
                continue

            # Test recognition using the loaded image
            print("Testing feature recognition on loaded image...")
            color_image = Color2DImage()

            # Use the loaded image for recognition instead of capturing
            # Note: This assumes you can pass the loaded image to the recognition system
            # You may need to modify this based on your specific API
            recognition_result = self.calibration.test_recognition_with_image(
                loaded_image, color_image
            )

            if not recognition_result.is_ok():
                print(f"Feature recognition failed for pose {pose_index}")
                continue

            show_and_save_image(
                color_image,
                file_name=f"FeatureRecognitionResultForTest_{pose_index}.png",
                window_name="Feature Recognition Result For Test",
            )

            # Step A: Add robot pose
            print("Step A: Adding robot pose...")
            pose_x, pose_y, pose_z, pose_r1, pose_r2, pose_r3 = pose_data
            robot_pose = self.euler_to_quad(pose_x, pose_y, pose_z, pose_r1, pose_r2, pose_r3)

            color_image_result = Color2DImage()

            # Use the loaded image for pose detection instead of capturing
            error_status = self.calibration.add_pose_and_detect_with_image(
                loaded_image, robot_pose, color_image_result
            )

            show_error(error_status)
            show_and_save_image(
                color_image_result,
                file_name=f"FeatureRecognitionResult_{pose_index}.png",
                window_name="Feature Recognition Result",
            )

            if error_status.is_ok():
                successful_poses += 1
                print(f"Successfully processed pose {pose_index}")
            else:
                print(f"Failed to process pose {pose_index}")

            # Small delay to prevent overwhelming the system
            time.sleep(0.1)

        print(f"\n--- Processing Complete ---")
        print(f"Successfully processed {successful_poses} poses")

        # Step C: Calculate extrinsic parameters
        if successful_poses >= 15:
            print("\nStep C: Calculating extrinsic parameters...")
            error_status = self.calibration.calculate_extrinsics(self.camera, camera_to_base)
            show_error(error_status)

            if error_status.is_ok():
                print("The extrinsic parameters are:")
                print(camera_to_base.to_string())
                save_extrinsic_parameters(camera_to_base.to_string())
            else:
                print("Failed to calculate extrinsic parameters")
        else:
            print(
                f"Insufficient successful poses ({successful_poses}). Need at least 15 for calibration."
            )

    def main(self):
        if not find_and_connect(self.camera):
            print("Failed to connect to camera")
            return

        print("Connected to camera successfully")

        # Get user input for calibration parameters
        self.input_calib_type()
        self.input_board_type()

        # Initialize calibration
        init_result = self.calibration.initialize_calibration(
            self.camera, self.mounting_mode, self.board_model
        )
        show_error(init_result)

        if not init_result.is_ok():
            print("Failed to initialize calibration")
            self.camera.disconnect()
            return

        self.input_euler_type()

        # Run automated calibration
        self.automated_calibrate()

        # Disconnect camera
        self.camera.disconnect()
        print("Disconnected from the camera successfully.")


if __name__ == "__main__":
    calibration_sample = AutomatedHandEyeCalibrationSample()
    calibration_sample.main()
