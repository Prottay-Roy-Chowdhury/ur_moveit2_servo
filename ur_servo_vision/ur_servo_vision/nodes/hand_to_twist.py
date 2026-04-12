#!/usr/bin/env python3

import math
import os
from typing import List, Tuple

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from rclpy.node import Node


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


class HandToTwistNode(Node):
    def __init__(self):
        super().__init__("hand_to_twist")

        self.publisher_ = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )

        pkg_share = get_package_share_directory("ur_servo_vision")
        model_path = os.path.join(pkg_share, "models", "hand_landmarker.task")

        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.get_logger().info(f"Using model: {model_path}")

        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            running_mode=vision.RunningMode.VIDEO,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera /dev/video0")

        self.frame_count = 0
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)

        # Linear motion tuning
        self.max_linear_speed_x = 0.05
        self.max_linear_speed_y = 0.04
        self.max_linear_speed_z = 0.05

        # Angular motion tuning
        self.max_angular_speed_x = 0.03
        self.max_angular_speed_y = 0.03
        self.max_angular_speed_z = 0.03

        self.deadband_xy = 0.08
        self.deadband_area = 0.10
        self.alpha = 0.30

        # Area calibration
        self.reference_area = None
        self.min_valid_radius_px = 25.0

        self.filtered_lin_x = 0.0
        self.filtered_lin_y = 0.0
        self.filtered_lin_z = 0.0

        self.filtered_ang_x = 0.0
        self.filtered_ang_y = 0.0
        self.filtered_ang_z = 0.0

        self.get_logger().info("hand_to_twist node started")

    def apply_deadband(self, value: float, threshold: float) -> float:
        return 0.0 if abs(value) < threshold else value

    def low_pass(self, previous: float, current: float) -> float:
        return self.alpha * current + (1.0 - self.alpha) * previous

    def publish_twist(
        self,
        vx: float,
        vy: float,
        vz: float,
        wx: float = 0.0,
        wy: float = 0.0,
        wz: float = 0.0,
    ):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = float(wx)
        msg.twist.angular.y = float(wy)
        msg.twist.angular.z = float(wz)
        self.publisher_.publish(msg)

    def publish_zero(self):
        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def landmarks_to_pixels(
        self, hand_landmarks, width: int, height: int
    ) -> List[Tuple[int, int]]:
        pts = []
        for lm in hand_landmarks:
            px = int(lm.x * width)
            py = int(lm.y * height)
            pts.append((px, py))
        return pts

    def is_open_palm(self, hand_landmarks) -> bool:
        finger_pairs = [
            (8, 6),
            (12, 10),
            (16, 14),
            (20, 18),
        ]

        extended_count = 0
        for tip_idx, pip_idx in finger_pairs:
            if hand_landmarks[tip_idx].y < hand_landmarks[pip_idx].y:
                extended_count += 1

        return extended_count >= 3

    def is_fist(self, hand_landmarks) -> bool:
        finger_pairs = [
            (8, 6),
            (12, 10),
            (16, 14),
            (20, 18),
        ]

        folded_count = 0
        for tip_idx, pip_idx in finger_pairs:
            if hand_landmarks[tip_idx].y > hand_landmarks[pip_idx].y:
                folded_count += 1

        return folded_count >= 3

    def compute_hand_circle(
        self, points: List[Tuple[int, int]]
    ) -> Tuple[Tuple[float, float], float]:
        pts_np = np.array(points, dtype=np.int32)
        (cx, cy), radius = cv2.minEnclosingCircle(pts_np)
        return (cx, cy), radius

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Failed to read from camera")
            self.publish_zero()
            return

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = frame.shape

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        timestamp_ms = int(self.frame_count * 50)
        self.frame_count += 1

        result = self.landmarker.detect_for_video(mp_image, timestamp_ms)

        if not result.hand_landmarks:
            self.publish_zero()
            cv2.imshow("hand_to_twist", frame)
            cv2.waitKey(1)
            return

        hand = result.hand_landmarks[0]
        points_px = self.landmarks_to_pixels(hand, w, h)
        (cx, cy), radius = self.compute_hand_circle(points_px)

        if radius < self.min_valid_radius_px:
            self.publish_zero()
            cv2.imshow("hand_to_twist", frame)
            cv2.waitKey(1)
            return

        area = math.pi * radius * radius

        if self.reference_area is None:
            self.reference_area = area
            self.get_logger().info(
                f"Reference hand area initialized: {self.reference_area:.1f}"
            )

        x_offset = (cx - (w / 2.0)) / (w / 2.0)
        z_offset = ((h / 2.0) - cy) / (h / 2.0)
        area_ratio = (area - self.reference_area) / self.reference_area

        raw_x = clamp(-x_offset, -1.0, 1.0)
        raw_y = clamp(area_ratio * 2.0, -1.0, 1.0)
        raw_z = clamp(z_offset, -1.0, 1.0)

        raw_x = self.apply_deadband(raw_x, self.deadband_xy)
        raw_y = self.apply_deadband(raw_y, self.deadband_area)
        raw_z = self.apply_deadband(raw_z, self.deadband_xy)

        mode_text = "STOP"

        if self.is_open_palm(hand):
            vx = raw_x * self.max_linear_speed_x
            vy = raw_y * self.max_linear_speed_y
            vz = raw_z * self.max_linear_speed_z

            self.filtered_lin_x = self.low_pass(self.filtered_lin_x, vx)
            self.filtered_lin_y = self.low_pass(self.filtered_lin_y, vy)
            self.filtered_lin_z = self.low_pass(self.filtered_lin_z, vz)

            self.filtered_ang_x = self.low_pass(self.filtered_ang_x, 0.0)
            self.filtered_ang_y = self.low_pass(self.filtered_ang_y, 0.0)
            self.filtered_ang_z = self.low_pass(self.filtered_ang_z, 0.0)

            self.publish_twist(
                self.filtered_lin_x,
                self.filtered_lin_y,
                self.filtered_lin_z,
                0.0,
                0.0,
                0.0,
            )
            mode_text = "OPEN PALM -> LINEAR"

        elif self.is_fist(hand):
            wx = raw_z * self.max_angular_speed_x
            wy = raw_y * self.max_angular_speed_y
            wz = raw_x * self.max_angular_speed_z

            self.filtered_ang_x = self.low_pass(self.filtered_ang_x, wx)
            self.filtered_ang_y = self.low_pass(self.filtered_ang_y, wy)
            self.filtered_ang_z = self.low_pass(self.filtered_ang_z, wz)

            self.filtered_lin_x = self.low_pass(self.filtered_lin_x, 0.0)
            self.filtered_lin_y = self.low_pass(self.filtered_lin_y, 0.0)
            self.filtered_lin_z = self.low_pass(self.filtered_lin_z, 0.0)

            self.publish_twist(
                0.0,
                0.0,
                0.0,
                self.filtered_ang_x,
                self.filtered_ang_y,
                self.filtered_ang_z,
            )
            mode_text = "FIST -> ANGULAR"

        else:
            self.filtered_lin_x = self.low_pass(self.filtered_lin_x, 0.0)
            self.filtered_lin_y = self.low_pass(self.filtered_lin_y, 0.0)
            self.filtered_lin_z = self.low_pass(self.filtered_lin_z, 0.0)
            self.filtered_ang_x = self.low_pass(self.filtered_ang_x, 0.0)
            self.filtered_ang_y = self.low_pass(self.filtered_ang_y, 0.0)
            self.filtered_ang_z = self.low_pass(self.filtered_ang_z, 0.0)
            self.publish_zero()

        cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 255), -1)

        for px, py in points_px:
            cv2.circle(frame, (px, py), 3, (255, 0, 0), -1)

        cv2.line(frame, (w // 2, 0), (w // 2, h), (150, 150, 150), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (150, 150, 150), 1)

        cv2.putText(
            frame,
            mode_text,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if mode_text != "STOP" else (0, 0, 255),
            2,
        )
        cv2.putText(
            frame,
            f"center=({int(cx)}, {int(cy)}) area={area:.0f}",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"lin x:{self.filtered_lin_x:+.3f} y:{self.filtered_lin_y:+.3f} z:{self.filtered_lin_z:+.3f}",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"ang x:{self.filtered_ang_x:+.3f} y:{self.filtered_ang_y:+.3f} z:{self.filtered_ang_z:+.3f}",
            (20, 115),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        cv2.imshow("hand_to_twist", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandToTwistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()