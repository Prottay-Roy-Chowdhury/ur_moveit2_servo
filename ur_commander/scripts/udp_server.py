import json
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class TFUDPStreamer(Node):
    def __init__(self):
        super().__init__("tf_udp_streamer")

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Joint state subscriber
        self.joint_angles = [0.0] * 6  # default 6 DOF
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10,
        )

        # UDP setup
        self.udp_ip = "192.168.1.44"
        self.udp_port = 5012
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.source_frame = "base"
        self.target_frame = "camera_color_optical_frame"

    def joint_callback(self, msg: JointState):
        if len(msg.position) >= 6:
            self.joint_angles = list(msg.position[:6])

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time(),
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            data = {
                "translation": {
                    "x": float(t.x),
                    "y": float(t.y),
                    "z": float(t.z),
                },
                "rotation": {
                    "x": float(r.x),
                    "y": float(r.y),
                    "z": float(r.z),
                    "w": float(r.w),
                },
                "joints": self.joint_angles,
            }

            message = json.dumps(data)
            self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TFUDPStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
