#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node

class UDPServer(Node):
    def __init__(self):
        super().__init__('udp_server')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))

        self.timer = self.create_timer(0.01, self.receive_udp)
        self.get_logger().info('UDP server listening on port 5005')

    def receive_udp(self):
        self.sock.settimeout(0.0)
        try:
            data, addr = self.sock.recvfrom(4096)
            msg = data.decode(errors='ignore')
            self.get_logger().info(f'Received from {addr}: {msg}')

            # TODO: convert UDP command into MoveIt / ROS 2 action/service/topic call

        except BlockingIOError:
            pass

def main():
    rclpy.init()
    node = UDPServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()