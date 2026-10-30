#!/usr/bin/env python3
"""Fixed-speed Grasshopper directional controller for ROS 2 Humble.

Input:
    std_msgs/msg/Int8MultiArray
    [X, Y, Z, RX, RY, RZ]

    Each value must be -1, 0 or +1.

Output:
    geometry_msgs/msg/TwistStamped

Direction selection comes from Grasshopper through ROSBridge.
Velocity magnitudes, frame_id, and ROS timestamps are generated locally.

This is experimental teleoperation, not a safety-rated controller.
"""

import math
import time

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

from std_msgs.msg import (
    Int8MultiArray,
    Float64MultiArray,
    String,
)


class GHDirectionToTwist(Node):

    # ==================================================
    # INITIALIZATION
    # ==================================================

    def __init__(self):

        super().__init__("gh_direction_to_twist")

        # ----------------------------------------------
        # Parameter defaults
        # ----------------------------------------------

        defaults = {
            "direction_topic": "/gh_servo/directions",

            "control_hz": 100.0,
            "input_timeout_sec": 0.30,

            "enable_motion": False,

            "linear_velocity_x": 0.30,
            "linear_velocity_y": 0.30,
            "linear_velocity_z": 0.30,

            "angular_velocity_x": 0.20,
            "angular_velocity_y": 0.20,
            "angular_velocity_z": 0.20,

            "twist_topic": "/gh_servo/test_twist",
            "command_frame": "base_link",
        }

        for key, value in defaults.items():
            self.declare_parameter(key, value)

        def p(name):
            return self.get_parameter(name).value

        # ----------------------------------------------
        # General configuration
        # ----------------------------------------------

        self.direction_topic = str(
            p("direction_topic")
        )

        self.twist_topic = str(
            p("twist_topic")
        )

        self.command_frame = str(
            p("command_frame")
        )

        self.control_hz = float(
            p("control_hz")
        )

        self.input_timeout = float(
            p("input_timeout_sec")
        )

        self.enable_motion = bool(
            p("enable_motion")
        )

        # ----------------------------------------------
        # Fixed velocity magnitudes
        # ----------------------------------------------

        self.speeds = [
            float(p("linear_velocity_x")),
            float(p("linear_velocity_y")),
            float(p("linear_velocity_z")),

            float(p("angular_velocity_x")),
            float(p("angular_velocity_y")),
            float(p("angular_velocity_z")),
        ]

        # ----------------------------------------------
        # Parameter validation
        # ----------------------------------------------

        if (
            not math.isfinite(self.control_hz)
            or self.control_hz <= 0
        ):
            raise ValueError(
                "control_hz must be finite and positive"
            )

        if (
            not math.isfinite(self.input_timeout)
            or self.input_timeout <= 0
        ):
            raise ValueError(
                "input_timeout_sec must be finite and positive"
            )

        if (
            any(
                not math.isfinite(value) or value < 0
                for value in self.speeds
            )
        ):
            raise ValueError(
                "Fixed speeds must be finite and non-negative"
            )

        if (
            not self.direction_topic
            or not self.twist_topic
            or not self.command_frame
        ):
            raise ValueError(
                "ROS topics and command_frame cannot be empty"
            )

        # ----------------------------------------------
        # State variables
        # ----------------------------------------------

        # Most recently received valid directions.
        self.directions = [0] * 6

        # Time of the latest valid input message.
        self.last_valid_input = None

        # Current input status.
        self.input_state = "WAITING"

        # State before the latest received message.
        self.previous_state = None

        # ----------------------------------------------
        # ROS publishers
        # ----------------------------------------------

        # Final TwistStamped output.
        self.pub = self.create_publisher(
            TwistStamped,
            self.twist_topic,
            10,
        )

        # Diagnostics follow the structure used by
        # the existing fixed-speed vision controller.

        self.selected_pub = self.create_publisher(
            Float64MultiArray,
            "~/selected_command",
            10,
        )

        self.published_pub = self.create_publisher(
            Float64MultiArray,
            "~/published_command",
            10,
        )

        self.axes_pub = self.create_publisher(
            Float64MultiArray,
            "~/axis_states",
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            "~/input_state",
            10,
        )

        # ----------------------------------------------
        # ROS subscriber
        # ----------------------------------------------

        self.direction_sub = self.create_subscription(
            Int8MultiArray,
            self.direction_topic,
            self.direction_callback,
            10,
        )

        # ----------------------------------------------
        # Independent control timer
        # ----------------------------------------------

        self.control_timer = self.create_timer(
            1.0 / self.control_hz,
            self.control_callback,
        )

        # ----------------------------------------------
        # Startup information
        # ----------------------------------------------

        self.get_logger().info(
            "GH directional controller initialized"
        )

        self.get_logger().info(
            f"Input topic: {self.direction_topic}"
        )

        self.get_logger().info(
            f"Output topic: {self.twist_topic}"
        )

        self.get_logger().info(
            f"Control frequency: {self.control_hz:g} Hz"
        )

        self.get_logger().info(
            f"Input timeout: {self.input_timeout:g} s"
        )

        self.get_logger().info(
            f"Command frame: {self.command_frame}"
        )

        self.get_logger().warning(
            f"Motion enabled: {self.enable_motion}. "
            "No acceleration limiting is implemented."
        )

    # ==================================================
    # INPUT VALIDATION
    # ==================================================

    @staticmethod
    def validate_directions(values):

        if len(values) != 6:
            return False

        for value in values:

            if type(value) is not int:
                return False

            if value not in (-1, 0, 1):
                return False

        return True

    # ==================================================
    # DIRECTION SUBSCRIBER CALLBACK
    # ==================================================

    def direction_callback(self, msg):

        # Preserve the six-value input convention.
        values = list(msg.data)

        # ----------------------------------------------
        # Reject malformed direction messages
        # ----------------------------------------------

        if not self.validate_directions(values):

            self.directions = [0] * 6
            self.last_valid_input = None
            self.input_state = "INVALID"

            self.get_logger().warning(
                "Invalid GH direction message received"
            )

            return

        # ----------------------------------------------
        # Accept the newest direction message
        # ----------------------------------------------

        self.directions = values

        # Local monotonic clock, independent of
        # ROS simulation or system clock changes.
        self.last_valid_input = time.monotonic()

        self.input_state = "ACTIVE"

    # ==================================================
    # INPUT WATCHDOG
    # ==================================================

    def input_is_fresh(self):

        if self.last_valid_input is None:
            return False

        age = (
            time.monotonic()
            - self.last_valid_input
        )

        return age < self.input_timeout

    # ==================================================
    # FIXED-SPEED CONVERSION
    # ==================================================

    def calculate_selected_command(self):

        # Directions always represent normalized
        # selectors: -1, 0, +1.
        #
        # Physical velocities come entirely
        # from the ROS-side configuration.

        return [
            float(direction) * speed
            for direction, speed in zip(
                self.directions,
                self.speeds,
            )
        ]

    # ==================================================
    # DIAGNOSTIC PUBLISHING
    # ==================================================

    @staticmethod
    def publish_array(publisher, values):

        msg = Float64MultiArray()

        msg.data = [
            float(value)
            for value in values
        ]

        publisher.publish(msg)

    @staticmethod
    def publish_string(publisher, value):

        msg = String()
        msg.data = str(value)

        publisher.publish(msg)

    # ==================================================
    # CONTROL CALLBACK
    # ==================================================

    def control_callback(self):

        # ----------------------------------------------
        # Input timeout
        # ----------------------------------------------

        fresh = self.input_is_fresh()

        if not fresh:

            # Do not retain stale directional motion.
            self.directions = [0] * 6

            if self.input_state != "INVALID":
                self.input_state = (
                    "STALE"
                    if self.last_valid_input is not None
                    else "WAITING"
                )

        # ----------------------------------------------
        # Fixed velocity selection
        # ----------------------------------------------

        selected = self.calculate_selected_command()

        # ----------------------------------------------
        # Final output gate
        # ----------------------------------------------

        if self.enable_motion and fresh:

            published = list(selected)

        else:

            published = [0.0] * 6

        # ----------------------------------------------
        # Generate ROS TwistStamped
        # ----------------------------------------------

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = self.command_frame

        msg.twist.linear.x = published[0]
        msg.twist.linear.y = published[1]
        msg.twist.linear.z = published[2]

        msg.twist.angular.x = published[3]
        msg.twist.angular.y = published[4]
        msg.twist.angular.z = published[5]

        self.pub.publish(msg)

        # ----------------------------------------------
        # Diagnostic topics
        # ----------------------------------------------

        self.publish_array(
            self.selected_pub,
            selected,
        )

        self.publish_array(
            self.published_pub,
            published,
        )

        self.publish_array(
            self.axes_pub,
            self.directions,
        )

        self.publish_string(
            self.state_pub,
            self.input_state,
        )

        # ----------------------------------------------
        # State transition logging
        # ----------------------------------------------

        if self.input_state != self.previous_state:

            self.get_logger().info(
                f"Input state: {self.input_state}"
            )

            self.previous_state = self.input_state

    # ==================================================
    # SHUTDOWN
    # ==================================================

    def publish_zero(self):

        msg = TwistStamped()

        msg.header.frame_id = self.command_frame

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # Twist values default to zero.
        self.pub.publish(msg)

    def destroy_node(self):

        try:
            # Same shutdown principle as the
            # existing hand_to_twist_fixed node.

            for _ in range(3):
                self.publish_zero()

        except Exception:
            pass

        super().destroy_node()


# ======================================================
# MAIN
# ======================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = GHDirectionToTwist()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()