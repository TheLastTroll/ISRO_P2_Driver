#!/usr/bin/env python3
"""
NTRIP Client for RTK corrections.

Subscribes to /nmea (GGA from driver) and publishes /rtcm (corrections to driver).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import socket
import base64
import time
import threading


class SimpleNTRIP(Node):
    def __init__(self):
        super().__init__('simple_ntrip_node')

        # --- Parameter Declaration ---
        self.declare_parameter('host', 'rts2.ngii.go.kr')
        self.declare_parameter('port', 2101)
        self.declare_parameter('mountpoint', 'VRS-RTCM32')
        self.declare_parameter('username', '')
        self.declare_parameter('password', 'ngii')

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.mountpoint = self.get_parameter('mountpoint').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        # Prompt for username if not provided
        if not self.username or self.username == 'YOUR_ID':
            print("\n" + "=" * 40)
            print(" [!] NTRIP credentials required.")
            self.username = input(" - Enter ID: ").strip()

        # --- Publishers / Subscribers ---
        self.create_subscription(String, '/nmea', self.nmea_callback, 10)
        self.rtcm_pub = self.create_publisher(ByteMultiArray, '/rtcm', 10)

        self.sock = None
        self.connected = False
        self.buffer = bytearray()
        self.lock = threading.Lock()

        # Start receive thread
        self.recv_thread = threading.Thread(target=self.receive_rtcm_loop)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        self.get_logger().info(
            f"NTRIP Client Started. Target: {self.host}:{self.port}/{self.mountpoint}")
        self.connect_to_server()

    def connect_to_server(self):
        """Establish connection to NTRIP caster."""
        try:
            with self.lock:
                if self.sock:
                    self.sock.close()

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(10.0)
                self.sock.connect((self.host, self.port))

                # NTRIP HTTP Request
                user_pw = f"{self.username}:{self.password}"
                auth_str = base64.b64encode(user_pw.encode()).decode()

                request = (
                    f"GET /{self.mountpoint} HTTP/1.0\r\n"
                    f"User-Agent: NTRIP ROS2 Client/1.0\r\n"
                    f"Authorization: Basic {auth_str}\r\n"
                    f"Accept: */*\r\n"
                    f"Connection: close\r\n\r\n"
                )

                self.sock.sendall(request.encode())
                self.sock.settimeout(None)  # Remove timeout after connect
                self.connected = True
                self.buffer = bytearray()
                self.get_logger().info("Connected to NTRIP Caster! Waiting for RTCM...")

        except Exception as e:
            self.get_logger().error(f"Connection Failed: {e}")
            self.connected = False

    def nmea_callback(self, msg):
        """Forward NMEA GGA to caster for VRS position."""
        if self.connected and self.sock:
            try:
                nmea_data = msg.data.strip() + "\r\n"
                with self.lock:
                    self.sock.sendall(nmea_data.encode())
            except Exception as e:
                self.get_logger().warn(f"Failed to send NMEA: {e}")
                self.connected = False
                self.connect_to_server()

    def receive_rtcm_loop(self):
        """Background thread to receive and parse RTCM messages."""
        while rclpy.ok():
            if self.connected and self.sock:
                try:
                    data = self.sock.recv(1024)
                    if not data:
                        self.get_logger().warn("Connection closed by server.")
                        self.connected = False
                        time.sleep(2)
                        self.connect_to_server()
                        continue

                    self.buffer.extend(data)

                    # Parse RTCM3 messages (0xD3 preamble)
                    while len(self.buffer) >= 3:
                        if self.buffer[0] != 0xD3:
                            self.buffer.pop(0)
                            continue

                        # RTCM3 header: [D3] [reserved + length high] [length low]
                        length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
                        total_len = 3 + length + 3  # header + payload + CRC

                        if len(self.buffer) < total_len:
                            break

                        rtcm_packet = bytes(self.buffer[:total_len])
                        del self.buffer[:total_len]

                        # Publish as ByteMultiArray
                        msg = ByteMultiArray()
                        msg.data = list(rtcm_packet)
                        self.rtcm_pub.publish(msg)

                except socket.timeout:
                    continue
                except Exception as e:
                    self.get_logger().warn(f"Receive Error: {e}")
                    self.connected = False
                    time.sleep(2)
            else:
                time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNTRIP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
