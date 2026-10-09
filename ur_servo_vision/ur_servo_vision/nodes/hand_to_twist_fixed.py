#!/usr/bin/env python3
"""Fixed-speed hand-gesture Twist selector for ROS 2 Humble.

Original axis mapping is preserved:
  LINEAR:  horizontal -> X (inverted), area -> Y, vertical -> Z
  ANGULAR: vertical -> X, area -> Y, horizontal -> Z (inverted)

This is experimental teleoperation, not a safety-rated control system.
"""
import math
import os
import threading
import time

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


class HandToTwistFixed(Node):
    def __init__(self):
        super().__init__('hand_to_twist_fixed')
        defaults = {
            'camera_index': 0,
            'vision_hz': 20.0,
            'control_hz': 100.0,
            'enable_motion': False,
            'show_gui': True,
            'linear_velocity_x': 0.03,
            'linear_velocity_y': 0.03,
            'linear_velocity_z': 0.03,
            'angular_velocity_x': 0.20,
            'angular_velocity_y': 0.20,
            'angular_velocity_z': 0.20,
            'xy_activate_threshold': 0.15,
            'xy_deactivate_threshold': 0.10,
            'area_activate_threshold': 0.25,
            'area_deactivate_threshold': 0.20,
            'measurement_alpha': 0.25,
            'gesture_confirmation_frames': 3,
            'axis_confirmation_frames': 2,
            'tracking_timeout_sec': 0.10,
            'min_valid_radius_px': 25.0,
            'reset_reference_on_tracking_loss': False,
            'twist_topic': '/servo_node/delta_twist_cmds',
            'command_frame': 'base_link',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.vision_hz = float(p('vision_hz'))
        self.control_hz = float(p('control_hz'))
        self.enable_motion = bool(p('enable_motion'))
        self.show_gui = bool(p('show_gui'))
        self.xy_on = float(p('xy_activate_threshold'))
        self.xy_off = float(p('xy_deactivate_threshold'))
        self.area_on = float(p('area_activate_threshold'))
        self.area_off = float(p('area_deactivate_threshold'))
        self.alpha = float(p('measurement_alpha'))
        self.gesture_frames = int(p('gesture_confirmation_frames'))
        self.axis_frames = int(p('axis_confirmation_frames'))
        self.tracking_timeout = float(p('tracking_timeout_sec'))
        self.min_radius = float(p('min_valid_radius_px'))
        self.reset_reference_on_loss = bool(p('reset_reference_on_tracking_loss'))
        self.command_frame = str(p('command_frame'))
        self.speeds = np.array([
            float(p('linear_velocity_x')), float(p('linear_velocity_y')),
            float(p('linear_velocity_z')), float(p('angular_velocity_x')),
            float(p('angular_velocity_y')), float(p('angular_velocity_z')),
        ], dtype=float)
        if not (self.vision_hz > 0 and self.control_hz > 0 and
                self.tracking_timeout > 0 and self.min_radius > 0 and
                0 < self.alpha <= 1 and self.gesture_frames >= 1 and
                self.axis_frames >= 1 and np.all(np.isfinite(self.speeds)) and
                np.all(self.speeds >= 0) and
                0 <= self.xy_off < self.xy_on <= 1 and
                0 <= self.area_off < self.area_on):
            raise ValueError('Invalid fixed-speed, threshold, filter, or frequency parameters')
        if self.tracking_timeout <= 1.0 / self.vision_hz:
            self.get_logger().warning('tracking_timeout_sec <= one nominal vision interval')

        self.pub = self.create_publisher(TwistStamped, str(p('twist_topic')), 10)
        self.selected_pub = self.create_publisher(Float64MultiArray, '~/selected_command', 10)
        self.published_pub = self.create_publisher(Float64MultiArray, '~/published_command', 10)
        self.mode_pub = self.create_publisher(String, '~/gesture_mode', 10)
        self.tracking_pub = self.create_publisher(String, '~/tracking_state', 10)
        self.axes_pub = self.create_publisher(Float64MultiArray, '~/axis_states', 10)
        self.measurement_pub = self.create_publisher(Float64MultiArray, '~/measurements', 10)

        model = os.path.join(get_package_share_directory('ur_servo_vision'),
                             'models', 'hand_landmarker.task')
        if not os.path.isfile(model):
            raise FileNotFoundError(f'MediaPipe model not found: {model}')
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            num_hands=1, running_mode=vision.RunningMode.VIDEO,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5)
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.cap = cv2.VideoCapture(int(p('camera_index')))
        if not self.cap.isOpened():
            self.landmarker.close()
            raise RuntimeError('Could not open configured camera')

        self.lock = threading.Lock()
        self.display_lock = threading.Lock()
        self.display_frame = None
        self.selected = np.zeros(6, dtype=float)
        self.axis_states = np.zeros(6, dtype=float)
        self.measurements = np.zeros(3, dtype=float)  # x, area ratio, vertical
        self.mode = 'STOP'
        self.tracking = 'TRACKING_LOST'
        self.last_valid = None
        self.filtered = None
        self.reference_area = None
        self.candidate_mode = 'STOP'
        self.candidate_count = 0
        self.axis_candidates = [0] * 3
        self.axis_counts = [0] * 3
        self.axes = [0] * 3  # horizontal, area, vertical
        self.start_monotonic = time.monotonic()
        self.last_mp_ms = -1
        self.vision_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1 / self.vision_hz, self.vision_callback,
                          callback_group=self.vision_group)
        self.create_timer(1 / self.control_hz, self.control_callback,
                          callback_group=self.control_group)
        self.get_logger().warning(
            f'Fixed-speed controller started: enable_motion={self.enable_motion}; '
            f'vision={self.vision_hz:g} Hz; publishing={self.control_hz:g} Hz. '
            'No acceleration limiting. Test disconnected from physical motion first.')

    @staticmethod
    def detect_gesture(hand):
        pairs = ((8, 6), (12, 10), (16, 14), (20, 18))
        extended = sum(hand[t].y < hand[p].y for t, p in pairs)
        folded = sum(hand[t].y > hand[p].y for t, p in pairs)
        if extended >= 3:
            return 'LINEAR'
        if folded >= 3:
            return 'ANGULAR'
        return 'STOP'

    @staticmethod
    def hysteretic_axis(value, previous, on, off):
        # A reversal must pass the opposite activation threshold.
        if previous == 0:
            if value >= on:
                return 1
            if value <= -on:
                return -1
            return 0
        if previous > 0:
            if value <= -on:
                return -1
            if value <= off:
                return 0
            return 1
        if value >= on:
            return 1
        if value >= -off:
            return 0
        return -1

    def zero_and_reset(self, reset_reference=False):
        with self.lock:
            self.selected[:] = 0
            self.axis_states[:] = 0
            self.axes = [0] * 3
            self.axis_candidates = [0] * 3
            self.axis_counts = [0] * 3
            self.mode = 'STOP'
            self.candidate_mode = 'STOP'
            self.candidate_count = 0
            self.tracking = 'TRACKING_LOST'
            self.last_valid = None
            self.filtered = None
            if reset_reference:
                self.reference_area = None

    def update_mode_locked(self, detected):
        if detected == self.mode:
            self.candidate_mode = detected
            self.candidate_count = 0
            return
        if detected == self.candidate_mode:
            self.candidate_count += 1
        else:
            self.candidate_mode = detected
            self.candidate_count = 1
        if self.candidate_count >= self.gesture_frames:
            self.mode = detected
            self.candidate_count = 0
            self.axes = [0] * 3
            self.axis_candidates = [0] * 3
            self.axis_counts = [0] * 3
            self.selected[:] = 0
            self.axis_states[:] = 0

    def update_axis_locked(self, idx, measurement, on, off):
        candidate = self.hysteretic_axis(measurement, self.axes[idx], on, off)
        if candidate == self.axes[idx]:
            self.axis_candidates[idx] = candidate
            self.axis_counts[idx] = 0
            return
        if candidate == self.axis_candidates[idx]:
            self.axis_counts[idx] += 1
        else:
            self.axis_candidates[idx] = candidate
            self.axis_counts[idx] = 1
        if self.axis_counts[idx] >= self.axis_frames:
            self.axes[idx] = candidate
            self.axis_counts[idx] = 0

    def vision_callback(self):
        try:
            self.process_vision()
        except Exception as exc:
            self.get_logger().error(f'Vision failure: {exc}')
            self.zero_and_reset(self.reset_reference_on_loss)

    def process_vision(self):
        ok, frame = self.cap.read()
        if not ok:
            self.zero_and_reset(self.reset_reference_on_loss)
            return
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        ms = max(int((time.monotonic() - self.start_monotonic) * 1000),
                 self.last_mp_ms + 1)
        self.last_mp_ms = ms
        result = self.landmarker.detect_for_video(mp_image, ms)
        hand = result.hand_landmarks[0] if result.hand_landmarks else None
        center = None
        if hand is not None:
            pts = np.array([(int(pt.x * w), int(pt.y * h)) for pt in hand],
                           dtype=np.int32)
            (cx, cy), radius = cv2.minEnclosingCircle(pts)
            if radius >= self.min_radius:
                center = np.array([cx, cy, radius], dtype=float)

        if center is None:
            # A missing detection must not continue commanding stale motion.
            self.zero_and_reset(self.reset_reference_on_loss)
        else:
            now = time.monotonic()
            with self.lock:
                # Reacquisition after a timeout must confirm a gesture afresh.
                if self.last_valid is not None and now - self.last_valid >= self.tracking_timeout:
                    self.selected[:] = 0
                    self.mode = 'STOP'
                    self.axes = [0] * 3
                    self.axis_candidates = [0] * 3
                    self.axis_counts = [0] * 3
                    self.candidate_mode = 'STOP'
                    self.candidate_count = 0
                    self.filtered = None
                    if self.reset_reference_on_loss:
                        self.reference_area = None
                if self.filtered is None:
                    self.filtered = center.copy()
                else:
                    self.filtered = self.alpha * center + (1 - self.alpha) * self.filtered
                fx, fy, fr = self.filtered
                area = math.pi * fr * fr
                if self.reference_area is None:
                    self.reference_area = area
                    self.get_logger().info(f'Calibrated reference hand area: {area:.1f} px^2')
                # Preserve previous direction conventions.
                horizontal = float(np.clip(-(fx - w / 2) / (w / 2), -1, 1))
                area_ratio = float((area - self.reference_area) / self.reference_area)
                vertical = float(np.clip((h / 2 - fy) / (h / 2), -1, 1))
                self.measurements[:] = [horizontal, area_ratio, vertical]
                self.update_mode_locked(self.detect_gesture(hand))
                if self.mode != 'STOP':
                    self.update_axis_locked(0, horizontal, self.xy_on, self.xy_off)
                    self.update_axis_locked(1, area_ratio, self.area_on, self.area_off)
                    self.update_axis_locked(2, vertical, self.xy_on, self.xy_off)
                else:
                    self.axes = [0] * 3
                x, a, z = self.axes
                self.selected[:] = 0
                if self.mode == 'LINEAR':
                    self.selected[:3] = [x * self.speeds[0],
                                         a * self.speeds[1],
                                         z * self.speeds[2]]
                elif self.mode == 'ANGULAR':
                    self.selected[3:] = [z * self.speeds[3],
                                          a * self.speeds[4],
                                          x * self.speeds[5]]
                self.axis_states[:] = np.sign(self.selected)
                self.last_valid = now
                self.tracking = 'TRACKING'
                mode = self.mode
            cv2.circle(frame, (int(fx), int(fy)), int(fr), (0, 255, 0), 2)
            cv2.putText(frame, f'Area change: {area_ratio:+.1%}', (20, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.line(frame, (w // 2, 0), (w // 2, h), (150, 150, 150), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (150, 150, 150), 1)
        with self.lock:
            mode = self.mode
            cmd = self.selected.copy()
        cv2.putText(frame, f'Mode: {mode}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        cv2.putText(frame, 'MOTION ENABLED' if self.enable_motion else 'DRY RUN',
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 80, 255), 2)
        cv2.putText(frame, 'Selected: ' + np.array2string(cmd, precision=2),
                    (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1)
        with self.display_lock:
            self.display_frame = frame.copy()

    @staticmethod
    def publish_array(pub, data):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in data]
        pub.publish(msg)

    @staticmethod
    def publish_string(pub, value):
        msg = String()
        msg.data = str(value)
        pub.publish(msg)

    def control_callback(self):
        now = time.monotonic()
        with self.lock:
            stale = (self.last_valid is None or
                     now - self.last_valid >= self.tracking_timeout)
            if stale:
                # Watchdog also protects against a stalled vision callback.
                self.selected[:] = 0
                self.axis_states[:] = 0
                self.axes = [0] * 3
                self.axis_candidates = [0] * 3
                self.axis_counts = [0] * 3
                self.mode = 'STOP'
                self.candidate_mode = 'STOP'
                self.candidate_count = 0
                self.filtered = None
                self.last_valid = None
                self.tracking = 'TRACKING_LOST'
            selected = self.selected.copy()
            states = self.axis_states.copy()
            measurements = self.measurements.copy()
            mode = self.mode
            tracking = self.tracking
        published = selected if self.enable_motion else np.zeros(6)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z) = map(float, published[:3])
        (msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z) = map(float, published[3:])
        self.pub.publish(msg)
        self.publish_array(self.selected_pub, selected)
        self.publish_array(self.published_pub, published)
        self.publish_array(self.axes_pub, states)
        self.publish_array(self.measurement_pub, measurements)
        self.publish_string(self.mode_pub, mode)
        self.publish_string(self.tracking_pub, tracking)

    def get_frame(self):
        with self.display_lock:
            return None if self.display_frame is None else self.display_frame.copy()

    def destroy_node(self):
        try:
            msg = TwistStamped()
            msg.header.frame_id = self.command_frame
            for _ in range(3):
                msg.header.stamp = self.get_clock().now().to_msg()
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
        node = HandToTwistFixed()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        if node.show_gui:
            while rclpy.ok():
                frame = node.get_frame()
                if frame is not None:
                    cv2.imshow('hand_to_twist_fixed', frame)
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
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
            thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
