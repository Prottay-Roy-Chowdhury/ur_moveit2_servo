import socket
import time

udp_ip = "192.168.1.44"  # change if needed
udp_port = 5008

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    message = "hello world"
    sock.sendto(message.encode(), (udp_ip, udp_port))
    print(f"Sent: {message}")
    time.sleep(1)
