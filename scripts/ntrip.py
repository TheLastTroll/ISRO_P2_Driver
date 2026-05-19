#!/usr/bin/env python3
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

        # --- 파라미터 선언 (기본값: 국토지리정보원) ---
        self.declare_parameter('host', 'rts2.ngii.go.kr')
        self.declare_parameter('port', 2101)
        self.declare_parameter('mountpoint', 'VRS-RTCM32')
        self.declare_parameter('username', '')
        self.declare_parameter('password', 'ngii')
        self.declare_parameter('interactive', True)

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.mountpoint = self.get_parameter('mountpoint').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        interactive = self.get_parameter('interactive').value

        if interactive and (not self.username or self.username == 'YOUR_ID'):
            self._interactive_setup()

        # --- Pub/Sub ---
        # 드라이버에서 NMEA(GGA)를 받아 서버로 전송
        self.nmea_sub = self.create_subscription(String, '/nmea', self.nmea_callback, 10)
        # 서버에서 받은 RTCM을 드라이버로 전송
        self.rtcm_pub = self.create_publisher(ByteMultiArray, '/rtcm', 10)

        self.sock = None
        self.connected = False
        self.buffer = bytearray()
        self.reconnect_lock = threading.Lock()  # 재접속 동시성 보호

        self.recv_thread = threading.Thread(target=self.receive_rtcm_loop)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        self.get_logger().info("NTRIP Client Started.")
        self.get_logger().info(f"  Host: {self.host}:{self.port}")
        self.get_logger().info(f"  Mountpoint: {self.mountpoint}")
        self.get_logger().info(f"  Username: {self.username}")
        self.connect_to_server()

    def _interactive_setup(self):
        """대화형으로 NTRIP 설정 입력."""
        print("\n" + "=" * 50)
        print("       NTRIP 클라이언트 설정")
        print("=" * 50)

        print("\n[?] 국토지리정보원 VRS-RTCM32 서비스를 사용하시겠습니까?")
        print("    (Host: rts2.ngii.go.kr, Port: 2101, Mount: VRS-RTCM32)")
        print()

        while True:
            use_ngii = input("    VRS-RTCM32 사용 (Y/n): ").strip().lower()
            if use_ngii in ['', 'y', 'yes', 'ㅛ']:
                self.host = 'rts2.ngii.go.kr'
                self.port = 2101
                self.mountpoint = 'VRS-RTCM32'
                self.password = 'ngii'

                print("\n    국토지리정보원 VRS-RTCM32 설정 적용")
                print()

                self.username = input("    사용자 ID (국토지리정보원 가입 ID): ").strip()
                if not self.username:
                    print("    [!] ID가 입력되지 않았습니다.")
                    continue
                break

            elif use_ngii in ['n', 'no', 'ㅜ']:
                print("\n    [i] 커스텀 NTRIP Caster 설정을 입력하세요.")
                print("-" * 50)

                host_input = input(f"    Host [{self.host}]: ").strip()
                if host_input:
                    self.host = host_input

                while True:
                    port_input = input(f"    Port [{self.port}]: ").strip()
                    if not port_input:
                        break
                    try:
                        self.port = int(port_input)
                        break
                    except ValueError:
                        print("    [!] 숫자를 입력하세요.")

                mount_input = input(f"    Mountpoint [{self.mountpoint}]: ").strip()
                if mount_input:
                    self.mountpoint = mount_input

                user_input = input("    Username: ").strip()
                if user_input:
                    self.username = user_input

                pass_input = input(f"    Password [{self.password}]: ").strip()
                if pass_input:
                    self.password = pass_input

                break
            else:
                print("    [!] Y 또는 N을 입력하세요.")

        print()
        print("-" * 50)
        print("    [설정 완료]")
        print(f"    Host:       {self.host}")
        print(f"    Port:       {self.port}")
        print(f"    Mountpoint: {self.mountpoint}")
        print(f"    Username:   {self.username}")
        print(f"    Password:   {'*' * len(self.password)}")
        print("=" * 50 + "\n")

    def connect_to_server(self):
        """재접속을 단일 락으로 직렬화."""
        with self.reconnect_lock:
            try:
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(10)
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(None)

                user_pw = f"{self.username}:{self.password}"
                auth_str = base64.b64encode(user_pw.encode()).decode()

                headers = (
                    f"GET /{self.mountpoint} HTTP/1.0\r\n"
                    f"User-Agent: NTRIP ROS2 Client\r\n"
                    f"Authorization: Basic {auth_str}\r\n"
                    f"Accept: */*\r\n"
                    f"Connection: close\r\n\r\n"
                )

                self.sock.sendall(headers.encode())
                self.connected = True
                self.buffer = bytearray()
                self.get_logger().info("Connected to NTRIP Caster. Waiting for RTCM...")

            except socket.timeout:
                self.get_logger().error(f"Connection timeout: {self.host}:{self.port}")
                self.connected = False
            except Exception as e:
                self.get_logger().error(f"Connection Failed: {e}")
                self.connected = False

    def nmea_callback(self, msg):
        # VRS 모드는 클라이언트의 GGA 위치를 보내야 보정 데이터를 받을 수 있음
        if not (self.connected and self.sock):
            return

        try:
            nmea_data = msg.data.strip() + "\r\n"
            self.sock.sendall(nmea_data.encode())
        except Exception as e:
            self.get_logger().warn(f"Failed to send NMEA: {e}")
            # 재접속은 receive 루프에 맡김 (여기서 호출하면 경합)
            self.connected = False

    def receive_rtcm_loop(self):
        while rclpy.ok():
            if not (self.connected and self.sock):
                time.sleep(1)
                self.connect_to_server()
                continue

            try:
                # 데이터 수신 및 버퍼 누적
                data = self.sock.recv(1024)
                if not data:
                    self.get_logger().warn("Disconnected from server.")
                    self.connected = False
                    time.sleep(3)
                    continue

                self.buffer.extend(data)

                # RTCMv3 메시지 추출 (Preamble 0xD3)
                while len(self.buffer) >= 3:
                    if self.buffer[0] != 0xD3:
                        self.buffer.pop(0)
                        continue

                    # 길이: 2바이트 중 뒤 10비트
                    # [D3] [0000 00LL] [LLLL LLLL]
                    length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
                    total_msg_len = length + 3 + 3  # Header(3) + Body + CRC(3)

                    if len(self.buffer) < total_msg_len:
                        break

                    rtcm_packet = self.buffer[:total_msg_len]
                    del self.buffer[:total_msg_len]

                    msg = ByteMultiArray()
                    msg.data = [bytes([b]) for b in rtcm_packet]
                    self.rtcm_pub.publish(msg)

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f"Receive Error: {e}")
                self.connected = False
                time.sleep(3)


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
