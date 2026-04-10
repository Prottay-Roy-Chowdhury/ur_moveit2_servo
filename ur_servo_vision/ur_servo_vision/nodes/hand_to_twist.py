#!/usr/bin/env python3

import math
import os

import cv2
import mediapipe as mp
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

        self.max_linear_speed = 0.09
        self.deadband = 0.08
        self.alpha = 0.25

        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_z = 0.0

        self.get_logger().info("hand_to_twist node started")

    def apply_deadband(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadband else value

    def low_pass(self, previous: float, current: float) -> float:
        return self.alpha * current + (1.0 - self.alpha) * previous

    def publish_twist(self, vx: float, vy: float, vz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool0"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)

    def publish_zero(self):
        self.publish_twist(0.0, 0.0, 0.0)

    def timer_callback(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Failed to read from camera")
            self.publish_zero()
            return

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

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

        wrist = hand[0]
        index_tip = hand[8]
        thumb_tip = hand[4]

        # Image-centered offsets
        # MediaPipe coordinates are normalized: x,y in [0,1]
        x_offset = index_tip.x - 0.5
        y_offset = index_tip.y - 0.5

        # Simple pinch distance as z control proxy
        pinch_distance = math.sqrt(
            (index_tip.x - thumb_tip.x) ** 2 +
            (index_tip.y - thumb_tip.y) ** 2
        )

        # Map hand motion to linear twist
        raw_vx = clamp(-y_offset, -1.0, 1.0)
        raw_vy = clamp(-x_offset, -1.0, 1.0)
        raw_vz = clamp((pinch_distance - 0.10) * 2.0, -1.0, 1.0)

        raw_vx = self.apply_deadband(raw_vx)
        raw_vy = self.apply_deadband(raw_vy)
        raw_vz = self.apply_deadband(raw_vz)

        vx = raw_vx * self.max_linear_speed
        vy = raw_vy * self.max_linear_speed
        vz = raw_vz * self.max_linear_speed

        self.filtered_x = self.low_pass(self.filtered_x, vx)
        self.filtered_y = self.low_pass(self.filtered_y, vy)
        self.filtered_z = self.low_pass(self.filtered_z, vz)

        self.publish_twist(self.filtered_x, self.filtered_y, self.filtered_z)

        h, w, _ = frame.shape
        px = int(index_tip.x * w)
        py = int(index_tip.y * h)
        cv2.circle(frame, (px, py), 8, (0, 255, 0), -1)
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