#!/usr/bin/env python3
"""Refined MediaPipe hand-to-Twist controller for ROS 2 Humble.

Keep the original hand_to_twist.py unchanged. Test with motion disabled first.
"""

import math
import os
import threading
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def clamp(value, low, high):
    return max(low, min(value, high))


class HandToTwistRefined(Node):
    def __init__(self):
        super().__init__('hand_to_twist_refined')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('vision_hz', 20.0)
        self.declare_parameter('control_hz', 100.0)
        self.declare_parameter('average_window', 3)
        self.declare_parameter('tracking_timeout_sec', 0.10)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('show_gui', True)

        self.vision_hz = float(self.get_parameter('vision_hz').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        window = int(self.get_parameter('average_window').value)
        self.tracking_timeout = float(self.get_parameter('tracking_timeout_sec').value)
        self.enable_motion = bool(self.get_parameter('enable_motion').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        if self.vision_hz <= 0 or self.control_hz <= 0 or window < 1 or self.tracking_timeout <= 0:
            raise ValueError('Invalid frequency, averaging window, or tracking timeout')
        if self.tracking_timeout <= 1.0 / self.vision_hz:
            self.get_logger().warning('Tracking timeout is <= one nominal camera interval')

        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.raw_pub = self.create_publisher(Float64MultiArray, '~/raw_target', 10)
        self.avg_pub = self.create_publisher(Float64MultiArray, '~/averaged_target', 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '~/published_command', 10)
        self.limited_pub = self.create_publisher(Float64MultiArray, '~/limited_command', 10)
        self.mode_pub = self.create_publisher(String, '~/gesture_mode', 10)
        self.tracking_pub = self.create_publisher(String, '~/tracking_state', 10)

        pkg_share = get_package_share_directory('ur_servo_vision')
        model_path = os.path.join(pkg_share, 'models', 'hand_landmarker.task')
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'MediaPipe model missing: {model_path}')
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            num_hands=1,
            running_mode=vision.RunningMode.VIDEO,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        camera_index = int(self.get_parameter('camera_index').value)
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.landmarker.close()
            raise RuntimeError(f'Could not open camera index {camera_index}')

        self.lock = threading.Lock()
        self.display_lock = threading.Lock()
        self.display_frame = None
        self.history = deque(maxlen=window)
        self.raw_target = np.zeros(6)
        self.avg_target = np.zeros(6)
        self.output = np.zeros(6)
        self.last_valid_time = None
        self.last_detection_valid = False
        self.last_control_time = time.monotonic()
        self.mode = 'STOP'
        self.candidate = 'STOP'
        self.candidate_count = 0
        self.gesture_confirm_frames = 3
        self.measurement_alpha = 0.25
        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_radius = None
        self.reference_area = None
        self.min_valid_radius_px = 25.0
        self.deadband_xy = 0.08
        self.deadband_area = 0.10
        self.linear_speed = np.array([0.08, 0.08, 0.08])
        self.angular_speed = np.array([0.50, 0.50, 0.50])
        self.acceleration = np.array([0.20] * 3 + [0.40] * 3)
        self.start_monotonic = time.monotonic()
        self.last_mp_timestamp_ms = -1
        self._warned_timeout = False

        # Mutually exclusive vision callbacks prevent overlapping camera reads.
        self.vision_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.vision_timer = self.create_timer(
            1.0 / self.vision_hz, self.vision_callback, callback_group=self.vision_group
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_hz, self.control_callback, callback_group=self.control_group
        )
        self.get_logger().warning(
            f'Refined node started: motion_enabled={self.enable_motion}; '
            f'vision={self.vision_hz:g} Hz, output={self.control_hz:g} Hz, '
            f'average_window={window}, tracking_timeout={self.tracking_timeout:.3f}s'
        )

    @staticmethod
    def deadband(value, threshold):
        if abs(value) <= threshold:
            return 0.0
        return math.copysign((abs(value) - threshold) / (1.0 - threshold), value)

    @staticmethod
    def gesture(hand):
        pairs = ((8, 6), (12, 10), (16, 14), (20, 18))
        extended = sum(hand[tip].y < hand[pip].y for tip, pip in pairs)
        folded = sum(hand[tip].y > hand[pip].y for tip, pip in pairs)
        if extended >= 3:
            return 'LINEAR'
        if folded >= 3:
            return 'ANGULAR'
        return 'STOP'

    def update_mode(self, detected):
        if detected == self.mode:
            self.candidate = detected
            self.candidate_count = 0
            return False
        if detected == self.candidate:
            self.candidate_count += 1
        else:
            self.candidate = detected
            self.candidate_count = 1
        if self.candidate_count >= self.gesture_confirm_frames:
            self.mode = detected
            self.candidate_count = 0
            return True
        return False

    def reset_tracking(self):
        """Reset motion and measurement history; do not resume stale commands."""
        with self.lock:
            self.history.clear()
            self.raw_target[:] = 0
            self.avg_target[:] = 0
            self.output[:] = 0
            self.last_valid_time = None
        self.mode = 'STOP'
        self.candidate = 'STOP'
        self.candidate_count = 0
        self.last_detection_valid = False
        self.filtered_cx = None
        self.filtered_cy = None
        self.filtered_radius = None
        # Retain initial reference area across brief losses to avoid recalibration jumps.

    def publish_diagnostic(self, publisher, values):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in values]
        publisher.publish(msg)

    def vision_callback(self):
        try:
            self.process_vision()
        except Exception as exc:
            self.get_logger().error(f'Vision error: {exc}')
            self.reset_tracking()

    def process_vision(self):
        ok, frame = self.cap.read()
        if not ok:
            self.reset_tracking()
            return
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((time.monotonic() - self.start_monotonic) * 1000)
        timestamp_ms = max(timestamp_ms, self.last_mp_timestamp_ms + 1)
        self.last_mp_timestamp_ms = timestamp_ms
        result = self.landmarker.detect_for_video(mp_image, timestamp_ms)

        valid = bool(result.hand_landmarks)
        raw_center = None
        if valid:
            hand = result.hand_landmarks[0]
            pts = np.array([(int(p.x * w), int(p.y * h)) for p in hand], dtype=np.int32)
            (raw_cx, raw_cy), raw_radius = cv2.minEnclosingCircle(pts)
            valid = raw_radius >= self.min_valid_radius_px
            raw_center = (raw_cx, raw_cy, raw_radius)

        if valid:
            # Require fresh gesture confirmation after a watchdog timeout.
            with self.lock:
                expired = (self.last_valid_time is None or
                           time.monotonic() - self.last_valid_time >= self.tracking_timeout)
            if expired and self.last_valid_time is not None:
                self.reset_tracking()
            if self.filtered_cx is None:
                self.filtered_cx, self.filtered_cy, self.filtered_radius = raw_center
            else:
                a = self.measurement_alpha
                self.filtered_cx = a * raw_cx + (1 - a) * self.filtered_cx
                self.filtered_cy = a * raw_cy + (1 - a) * self.filtered_cy
                self.filtered_radius = a * raw_radius + (1 - a) * self.filtered_radius
            cx, cy, radius = self.filtered_cx, self.filtered_cy, self.filtered_radius
            area = math.pi * radius ** 2
            if self.reference_area is None:
                self.reference_area = area
                self.get_logger().info(f'Reference hand area: {area:.1f}')

            raw_x = clamp(-(cx - w / 2) / (w / 2), -1.0, 1.0)
            raw_y = clamp(2.0 * (area - self.reference_area) / self.reference_area, -1.0, 1.0)
            raw_z = clamp((h / 2 - cy) / (h / 2), -1.0, 1.0)
            xyz = np.array([
                self.deadband(raw_x, self.deadband_xy),
                self.deadband(raw_y, self.deadband_area),
                self.deadband(raw_z, self.deadband_xy),
            ])

            detected = self.gesture(hand)
            changed = self.update_mode(detected)
            with self.lock:
                if changed:
                    self.history.clear()
                    self.raw_target[:] = 0
                    self.avg_target[:] = 0
                    self.output[:] = 0
                raw = np.zeros(6)
                if self.mode == 'LINEAR':
                    raw[:3] = xyz * self.linear_speed
                elif self.mode == 'ANGULAR':
                    raw[3:] = xyz[[2, 1, 0]] * self.angular_speed
                # Never blend STOP measurements with motion samples.
                if self.mode == 'STOP':
                    self.history.clear()
                    self.avg_target[:] = 0
                else:
                    self.history.append(raw.copy())
                    self.avg_target[:] = np.mean(np.stack(self.history), axis=0)
                self.raw_target[:] = raw
                self.last_valid_time = time.monotonic()
                raw_snapshot = self.raw_target.copy()
                avg_snapshot = self.avg_target.copy()
            self.last_detection_valid = True
            self.publish_diagnostic(self.raw_pub, raw_snapshot)
            self.publish_diagnostic(self.avg_pub, avg_snapshot)
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (int(raw_cx), int(raw_cy)), int(raw_radius), (100, 100, 100), 1)
        else:
            self.last_detection_valid = False
            # Do not insert false zero measurements into the averaging buffer.
            # The control watchdog stops output after tracking_timeout.
            with self.lock:
                age = (time.monotonic() - self.last_valid_time
                       if self.last_valid_time is not None else float('inf'))
            if age >= self.tracking_timeout:
                self.reset_tracking()

        cv2.line(frame, (w // 2, 0), (w // 2, h), (150, 150, 150), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (150, 150, 150), 1)
        label = self.mode if valid else 'TRACKING LOST / TIMEOUT PENDING'
        cv2.putText(frame, label, (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0) if valid else (0, 0, 255), 2)
        cv2.putText(frame, 'MOTION ENABLED' if self.enable_motion else 'DRY RUN - ZERO OUTPUT',
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 100, 255), 2)
        with self.display_lock:
            self.display_frame = frame.copy()

    def control_callback(self):
        now = time.monotonic()
        dt = clamp(now - self.last_control_time, 0.0, 0.05)
        self.last_control_time = now
        with self.lock:
            stale = (self.last_valid_time is None or
                     now - self.last_valid_time >= self.tracking_timeout)
            if stale:
                self.output[:] = 0.0
                self.avg_target[:] = 0.0
                self.history.clear()
            else:
                delta = self.avg_target - self.output
                max_step = self.acceleration * dt
                self.output += np.clip(delta, -max_step, max_step)
            limited = self.output.copy()
            cmd = limited.copy() if self.enable_motion else np.zeros(6)
            mode = self.mode
            if stale:
                tracking = 'TRACKING_LOST'
            elif self.last_detection_valid:
                tracking = 'TRACKING'
            else:
                tracking = 'TEMPORARY_LOSS' 
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, cmd[:3])
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, cmd[3:])
        self.pub.publish(msg)
        self.publish_diagnostic(self.cmd_pub, cmd)
        self.publish_diagnostic(self.limited_pub, limited)
        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        tracking_msg = String()
        tracking_msg.data = tracking
        self.tracking_pub.publish(tracking_msg)

    def get_frame(self):
        with self.display_lock:
            return None if self.display_frame is None else self.display_frame.copy()

    def destroy_node(self):
        # Best-effort zero command, not a substitute for a robot safety stop.
        try:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            for _ in range(3):
                self.pub.publish(msg)
        except Exception:
            pass
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'landmarker'):
            self.landmarker.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    thread = None
    try:
        node = HandToTwistRefined()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        if node.show_gui:
            while rclpy.ok():
                frame = node.get_frame()
                if frame is not None:
                    cv2.imshow('hand_to_twist_refined', frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                time.sleep(0.005)
        else:
            while rclpy.ok():
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()