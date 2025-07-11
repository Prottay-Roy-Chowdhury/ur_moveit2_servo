import cv2
import time
import os
import glob
from math import sin, cos

from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import find_and_connect

class AutomatedHandEyeCalibrationSample:
    def __init__(self):
        self.camera = Camera()
        self.calibration = HandEyeCalibration()
        self.image_path = "/dev_ws/src/ur_commander/images"
        self.poses = self.parse_poses_from_data()
        self.euler_type_code = 1  # default; set later via input

    def parse_poses_from_data(self):
        return [
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
            (-312.24, -489.29, 428.89, -177.45, -15.27, 152.65)
        ]

    def euler_to_quad(self, x, y, z, r1, r2, r3):
        a1, a2, a3 = r1 * PI / 360, r2 * PI / 360, r3 * PI / 360
        w = sin(a1)*sin(a2)*sin(a3) + cos(a1)*cos(a2)*cos(a3)
        qx = -sin(a1)*sin(a2)*cos(a3) + sin(a3)*cos(a1)*cos(a2)
        qy = sin(a1)*sin(a3)*cos(a2) + sin(a2)*cos(a1)*cos(a3)
        qz = sin(a1)*cos(a2)*cos(a3) - sin(a2)*sin(a3)*cos(a1)
        return HandEyeTransformation(x, y, z, w, qx, qy, qz)

    def show_and_save_image(self, image, file_name):
        if image.is_empty():
            return
        cv2.imwrite(file_name, image.data())
        print("Saved:", file_name)

    def load_image(self, file_path):
        img_cv = cv2.imread(file_path)
        if img_cv is None:
            print("Failed to read:", file_path)
            return None
        img = Color2DImage()
        img.set_data(img_cv)
        return img

    def automated_calibrate(self):
        image_files_png = sorted(glob.glob(os.path.join(self.image_path, "Original2DImage_*.png")))
        image_files_tiff = sorted(glob.glob(os.path.join(self.image_path, "DepthMap_*.tiff")))
        if len(image_files_png) == 0 or len(image_files_tiff) == 0:
            print("No image files found.")
            return

        success = 0
        for i, pose in enumerate(self.poses):
            if i >= len(image_files_png):
                break

            color_img = self.load_image(image_files_png[i])
            if not color_img:
                continue

            # Recognition
            out_img = Color2DImage()
            if not self.calibration.test_recognition_with_image(color_img, out_img).is_ok():
                print(f"Recognition failed at index {i}")
                continue

            self.show_and_save_image(out_img, f"FeatureRecognitionResultForTest_{i+1}.png")

            # Pose addition
            robot_pose = self.euler_to_quad(*pose)
            out_img2 = Color2DImage()
            err = self.calibration.add_pose_and_detect_with_image(color_img, robot_pose, out_img2)
            self.show_and_save_image(out_img2, f"FeatureRecognitionResult_{i+1}.png")
            if err.is_ok():
                success += 1
                print(f"Pose {i+1} OK")

        print(f"\n{success}/{len(self.poses)} poses processed")

        if success >= 15:
            result = HandEyeTransformation()
            err = self.calibration.calculate_extrinsics(self.camera, result)
            if err.is_ok():
                print("Extrinsics:\n", result.to_string())
                self.save_extrinsic_parameters(result.to_string())

    def save_extrinsic_parameters(self, txt):
        t = time.localtime()
        fname = f"ExtrinsicParameters{t.tm_year:04d}{t.tm_mon:02d}{t.tm_mday:02d}{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.txt"
        with open(fname, "w") as f:
            f.write("ExtrinsicParameters:\n")
            f.write(txt)
        print("Saved extrinsics to", fname)

    def main(self):
        if not find_and_connect(self.camera):
            print("Failed to connect to camera")
            return

        self.automated_calibrate()
        self.camera.disconnect()


if __name__ == "__main__":
    AutomatedHandEyeCalibrationSample().main()