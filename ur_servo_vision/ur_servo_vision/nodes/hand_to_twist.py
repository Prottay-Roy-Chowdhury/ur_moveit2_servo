#!/usr/bin/env python3

import math
import os
import threading
import time
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


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

        # ---------------------------------------------------------
        # Shared-state synchronization
        # ---------------------------------------------------------

        self.state_lock = threading.Lock()

        # ---------------------------------------------------------
        # GUI frame shared with main thread
        # ---------------------------------------------------------

        self.display_lock = threading.Lock()
        self.latest_display_frame = None

        # ---------------------------------------------------------
        # Callback groups
        # ---------------------------------------------------------

        self.vision_callback_group = ReentrantCallbackGroup()
        self.control_callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------
        # Perception loop
        # ---------------------------------------------------------

        self.vision_frequency = 20.0

        self.vision_timer = self.create_timer(
            1.0 / self.vision_frequency,
            self.vision_callback,
            callback_group=self.vision_callback_group,
        )

        # ---------------------------------------------------------
        # Servo command loop
        # ---------------------------------------------------------

        self.control_frequency = 100.0
        self.control_dt = 1.0 / self.control_frequency

        self.control_timer = self.create_timer(
            self.control_dt,
            self.control_callback,
            callback_group=self.control_callback_group,
        )

        # ---------------------------------------------------------
        # Linear motion tuning
        # ---------------------------------------------------------

        self.max_linear_speed_x = 0.08
        self.max_linear_speed_y = 0.08
        self.max_linear_speed_z = 0.08

        # ---------------------------------------------------------
        # Angular motion tuning
        # ---------------------------------------------------------

        self.max_angular_speed_x = 0.50
        self.max_angular_speed_y = 0.50
        self.max_angular_speed_z = 0.50

        # ---------------------------------------------------------
        # Input deadbands
        # ---------------------------------------------------------

        self.deadband_xy = 0.08
        self.deadband_area = 0.10

        # ---------------------------------------------------------
        # Area calibration
        # ---------------------------------------------------------

        self.reference_area = None
        self.min_valid_radius_px = 25.0

        # ---------------------------------------------------------
        # Measurement filtering
        # ---------------------------------------------------------

        self.measurement_alpha = 0.25

        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_radius = None

        # ---------------------------------------------------------
        # Gesture hysteresis
        # ---------------------------------------------------------

        self.active_mode = "STOP"
        self.candidate_mode = "STOP"
        self.candidate_count = 0
        self.gesture_confirm_frames = 3

        # ---------------------------------------------------------
        # Target velocities generated by vision
        # ---------------------------------------------------------

        self.target_lin_x = 0.0
        self.target_lin_y = 0.0
        self.target_lin_z = 0.0

        self.target_ang_x = 0.0
        self.target_ang_y = 0.0
        self.target_ang_z = 0.0

        # ---------------------------------------------------------
        # Actual velocities sent to Servo
        # ---------------------------------------------------------

        self.filtered_lin_x = 0.0
        self.filtered_lin_y = 0.0
        self.filtered_lin_z = 0.0

        self.filtered_ang_x = 0.0
        self.filtered_ang_y = 0.0
        self.filtered_ang_z = 0.0

        # ---------------------------------------------------------
        # Acceleration limits
        # ---------------------------------------------------------

        self.max_linear_acceleration = 0.20
        self.max_angular_acceleration = 0.40

        self.get_logger().info(
            "hand_to_twist node started "
            f"(vision={self.vision_frequency:.0f} Hz, "
            f"control={self.control_frequency:.0f} Hz)"
        )


    # =============================================================
    # Utility functions
    # =============================================================

    def apply_deadband(self, value: float, threshold: float) -> float:
        """
        Rescaled deadband.

        Values inside the deadband become zero. Values outside the
        deadband are rescaled continuously from 0 to 1.
        """

        if abs(value) <= threshold:
            return 0.0

        sign = 1.0 if value > 0.0 else -1.0

        return sign * (abs(value) - threshold) / (1.0 - threshold)

    def rate_limit(
        self,
        current: float,
        target: float,
        max_acceleration: float,
    ) -> float:
        """
        Limit how quickly a velocity command can change.
        """

        max_change = max_acceleration * self.control_dt

        delta = target - current
        delta = clamp(delta, -max_change, max_change)

        return current + delta

    def set_zero_targets(self):
        """
        Request a smooth stop from the control loop.
        """

        with self.state_lock:

            self.target_lin_x = 0.0
            self.target_lin_y = 0.0
            self.target_lin_z = 0.0

            self.target_ang_x = 0.0
            self.target_ang_y = 0.0
            self.target_ang_z = 0.0

    def immediate_stop(self):
        """
        Immediately send zero velocity.

        Used for camera/tracking failures rather than normal
        gesture transitions.
        """

        self.set_zero_targets()

        self.filtered_lin_x = 0.0
        self.filtered_lin_y = 0.0
        self.filtered_lin_z = 0.0

        self.filtered_ang_x = 0.0
        self.filtered_ang_y = 0.0
        self.filtered_ang_z = 0.0

        self.publish_zero()

    # =============================================================
    # Gesture handling
    # =============================================================

    def update_gesture_mode(self, detected_mode: str):
        """
        Require several consecutive frames before switching modes.
        """

        if detected_mode == self.active_mode:
            self.candidate_mode = detected_mode
            self.candidate_count = 0
            return

        if detected_mode == self.candidate_mode:
            self.candidate_count += 1
        else:
            self.candidate_mode = detected_mode
            self.candidate_count = 1

        if self.candidate_count >= self.gesture_confirm_frames:
            self.active_mode = self.candidate_mode
            self.candidate_count = 0

    # =============================================================
    # ROS publishing
    # =============================================================

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
        self.publish_twist(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    # =============================================================
    # Hand processing
    # =============================================================

    def landmarks_to_pixels(
        self,
        hand_landmarks,
        width: int,
        height: int,
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
        self,
        points: List[Tuple[int, int]],
    ) -> Tuple[Tuple[float, float], float]:

        pts_np = np.array(points, dtype=np.int32)

        (cx, cy), radius = cv2.minEnclosingCircle(pts_np)

        return (cx, cy), radius

    # =============================================================
    # Vision loop - 20 Hz
    # =============================================================

    def vision_callback(self):

        ok, frame = self.cap.read()

        # ---------------------------------------------------------
        # Camera failure -> immediate stop
        # ---------------------------------------------------------

        if not ok:
            self.get_logger().warning("Failed to read from camera")
            self.immediate_stop()
            return

        frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB,
        )

        h, w, _ = frame.shape

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=rgb_frame,
        )

        timestamp_ms = int(self.frame_count * (1000.0 / self.vision_frequency))
        self.frame_count += 1

        result = self.landmarker.detect_for_video(
            mp_image,
            timestamp_ms,
        )

        # ---------------------------------------------------------
        # No hand detected
        # ---------------------------------------------------------

        if not result.hand_landmarks:

            # Tracking loss should stop immediately.
            self.immediate_stop()

            self.active_mode = "STOP"
            self.candidate_mode = "STOP"
            self.candidate_count = 0

            cv2.putText(
                frame,
                "NO HAND -> STOP",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

            # cv2.imshow("hand_to_twist", frame)
            # cv2.waitKey(1)
            self.update_display_frame(frame)

            return

        hand = result.hand_landmarks[0]

        points_px = self.landmarks_to_pixels(
            hand,
            w,
            h,
        )

        (raw_cx, raw_cy), raw_radius = self.compute_hand_circle(
            points_px
        )

        # ---------------------------------------------------------
        # Invalid / very small hand
        # ---------------------------------------------------------

        if raw_radius < self.min_valid_radius_px:

            self.immediate_stop()

            self.active_mode = "STOP"
            self.candidate_mode = "STOP"
            self.candidate_count = 0

            cv2.putText(
                frame,
                "HAND TOO SMALL -> STOP",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

            self.update_display_frame(frame)

            return

        # ---------------------------------------------------------
        # Filter measured hand center and radius
        # ---------------------------------------------------------

        if self.filtered_cx is None:

            self.filtered_cx = raw_cx
            self.filtered_cy = raw_cy
            self.filtered_radius = raw_radius

        else:

            a = self.measurement_alpha

            self.filtered_cx = (
                a * raw_cx
                + (1.0 - a) * self.filtered_cx
            )

            self.filtered_cy = (
                a * raw_cy
                + (1.0 - a) * self.filtered_cy
            )

            self.filtered_radius = (
                a * raw_radius
                + (1.0 - a) * self.filtered_radius
            )

        cx = self.filtered_cx
        cy = self.filtered_cy
        radius = self.filtered_radius

        # ---------------------------------------------------------
        # Calculate filtered hand area
        # ---------------------------------------------------------

        area = math.pi * radius * radius

        if self.reference_area is None:

            self.reference_area = area

            self.get_logger().info(
                f"Reference hand area initialized: "
                f"{self.reference_area:.1f}"
            )

        # ---------------------------------------------------------
        # Convert hand position into normalized control input
        # ---------------------------------------------------------

        x_offset = (
            cx - (w / 2.0)
        ) / (w / 2.0)

        z_offset = (
            (h / 2.0) - cy
        ) / (h / 2.0)

        area_ratio = (
            area - self.reference_area
        ) / self.reference_area

        raw_x = clamp(
            -x_offset,
            -1.0,
            1.0,
        )

        raw_y = clamp(
            area_ratio * 2.0,
            -1.0,
            1.0,
        )

        raw_z = clamp(
            z_offset,
            -1.0,
            1.0,
        )

        # ---------------------------------------------------------
        # Apply smooth deadband
        # ---------------------------------------------------------

        raw_x = self.apply_deadband(
            raw_x,
            self.deadband_xy,
        )

        raw_y = self.apply_deadband(
            raw_y,
            self.deadband_area,
        )

        raw_z = self.apply_deadband(
            raw_z,
            self.deadband_xy,
        )

        # ---------------------------------------------------------
        # Determine current gesture
        # ---------------------------------------------------------

        if self.is_open_palm(hand):

            detected_mode = "LINEAR"

        elif self.is_fist(hand):

            detected_mode = "ANGULAR"

        else:

            detected_mode = "STOP"

        # ---------------------------------------------------------
        # Apply gesture hysteresis
        # ---------------------------------------------------------

        self.update_gesture_mode(
            detected_mode
        )

        mode_text = "STOP"

        # ---------------------------------------------------------
        # LINEAR MODE
        # ---------------------------------------------------------

        if self.active_mode == "LINEAR":

            with self.state_lock:

                self.target_lin_x = (
                    raw_x * self.max_linear_speed_x
                )

                self.target_lin_y = (
                    raw_y * self.max_linear_speed_y
                )

                self.target_lin_z = (
                    raw_z * self.max_linear_speed_z
                )

                self.target_ang_x = 0.0
                self.target_ang_y = 0.0
                self.target_ang_z = 0.0

            mode_text = "OPEN PALM -> LINEAR"

        # ---------------------------------------------------------
        # ANGULAR MODE
        # ---------------------------------------------------------

        elif self.active_mode == "ANGULAR":

            with self.state_lock:

                self.target_ang_x = (
                    raw_z * self.max_angular_speed_x
                )

                self.target_ang_y = (
                    raw_y * self.max_angular_speed_y
                )

                self.target_ang_z = (
                    raw_x * self.max_angular_speed_z
                )

                self.target_lin_x = 0.0
                self.target_lin_y = 0.0
                self.target_lin_z = 0.0

            mode_text = "FIST -> ANGULAR"

        # ---------------------------------------------------------
        # STOP MODE
        # ---------------------------------------------------------

        else:

            # Intentional STOP is smoothly decelerated.
            self.set_zero_targets()

            mode_text = "STOP"

        # ---------------------------------------------------------
        # Visualization
        # ---------------------------------------------------------

        # Raw hand circle
        cv2.circle(
            frame,
            (int(raw_cx), int(raw_cy)),
            int(raw_radius),
            (100, 100, 100),
            1,
        )

        # Filtered hand circle
        cv2.circle(
            frame,
            (int(cx), int(cy)),
            int(radius),
            (0, 255, 0),
            2,
        )

        cv2.circle(
            frame,
            (int(cx), int(cy)),
            6,
            (0, 255, 255),
            -1,
        )

        for px, py in points_px:
            cv2.circle(
                frame,
                (px, py),
                3,
                (255, 0, 0),
                -1,
            )

        cv2.line(
            frame,
            (w // 2, 0),
            (w // 2, h),
            (150, 150, 150),
            1,
        )

        cv2.line(
            frame,
            (0, h // 2),
            (w, h // 2),
            (150, 150, 150),
            1,
        )

        cv2.putText(
            frame,
            mode_text,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0)
            if mode_text != "STOP"
            else (0, 0, 255),
            2,
        )

        cv2.putText(
            frame,
            f"center=({int(cx)}, {int(cy)}) "
            f"area={area:.0f}",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            frame,
            f"lin x:{self.filtered_lin_x:+.3f} "
            f"y:{self.filtered_lin_y:+.3f} "
            f"z:{self.filtered_lin_z:+.3f}",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            frame,
            f"ang x:{self.filtered_ang_x:+.3f} "
            f"y:{self.filtered_ang_y:+.3f} "
            f"z:{self.filtered_ang_z:+.3f}",
            (20, 115),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        # cv2.imshow(
        #     "hand_to_twist",
        #     frame,
        # )

        # cv2.waitKey(1)

        self.update_display_frame(frame)

    # =============================================================
    # Servo control loop - 100 Hz
    # =============================================================

    def control_callback(self):

        # ---------------------------------------------------------
        # Take one consistent snapshot of vision targets
        # ---------------------------------------------------------

        with self.state_lock:

            target_lin_x = self.target_lin_x
            target_lin_y = self.target_lin_y
            target_lin_z = self.target_lin_z

            target_ang_x = self.target_ang_x
            target_ang_y = self.target_ang_y
            target_ang_z = self.target_ang_z

        # ---------------------------------------------------------
        # Linear acceleration limiting
        # ---------------------------------------------------------

        self.filtered_lin_x = self.rate_limit(
            self.filtered_lin_x,
            target_lin_x,
            self.max_linear_acceleration,
        )

        self.filtered_lin_y = self.rate_limit(
            self.filtered_lin_y,
            target_lin_y,
            self.max_linear_acceleration,
        )

        self.filtered_lin_z = self.rate_limit(
            self.filtered_lin_z,
            target_lin_z,
            self.max_linear_acceleration,
        )

        # ---------------------------------------------------------
        # Angular acceleration limiting
        # ---------------------------------------------------------

        self.filtered_ang_x = self.rate_limit(
            self.filtered_ang_x,
            target_ang_x,
            self.max_angular_acceleration,
        )

        self.filtered_ang_y = self.rate_limit(
            self.filtered_ang_y,
            target_ang_y,
            self.max_angular_acceleration,
        )

        self.filtered_ang_z = self.rate_limit(
            self.filtered_ang_z,
            target_ang_z,
            self.max_angular_acceleration,
        )

        # ---------------------------------------------------------
        # Publish continuously to MoveIt Servo
        # ---------------------------------------------------------

        self.publish_twist(
            self.filtered_lin_x,
            self.filtered_lin_y,
            self.filtered_lin_z,
            self.filtered_ang_x,
            self.filtered_ang_y,
            self.filtered_ang_z,
        )

    def update_display_frame(self, frame):
        """
        Store the newest annotated frame for the main GUI thread.
        """

        with self.display_lock:
            self.latest_display_frame = frame.copy()


    def get_display_frame(self):
        """
        Return the newest display frame safely.
        """

        with self.display_lock:

            if self.latest_display_frame is None:
                return None

            return self.latest_display_frame.copy()

    # =============================================================
    # Cleanup
    # =============================================================

    def destroy_node(self):

        # Stop robot command before shutting down
        self.immediate_stop()

        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()

        if hasattr(self, "landmarker"):
            self.landmarker.close()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = HandToTwistNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # ---------------------------------------------------------
    # Run ROS executor in a background thread.
    # Main thread remains available for OpenCV / Qt GUI.
    # ---------------------------------------------------------

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )

    executor_thread.start()

    try:

        while rclpy.ok():

            frame = node.get_display_frame()

            if frame is not None:
                cv2.imshow(
                    "hand_to_twist",
                    frame,
                )

            key = cv2.waitKey(1) & 0xFF

            # Press Q or ESC to quit
            if key == ord("q") or key == 27:
                break

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass

    finally:

        # Immediately stop command before shutdown
        node.immediate_stop()

        executor.shutdown()

        executor_thread.join(timeout=2.0)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()