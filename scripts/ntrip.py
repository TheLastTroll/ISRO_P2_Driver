import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import socket
import base64
import time
import threading
import getpass
import struct

class SimpleNTRIP(Node):
    def __init__(self):
        super().__init__('simple_ntrip_node')

        # --- 파라미터 설정 (국토지리정보원 정보) ---
        self.declare_parameter('host', 'rts2.ngii.go.kr')
        self.declare_parameter('port', 2101)
        self.declare_parameter('mountpoint', 'VRS-RTCM32') # PIM222A는 3.2 권장
        self.declare_parameter('username', '')      # 본인 ID
        self.declare_parameter('password', 'ngii')      # 국토지리정보원 ngii

        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.mountpoint = self.get_parameter('mountpoint').value
        
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        # 파라미터가 비어있다면 터미널에서 직접 입력을 받음
        if not self.username or self.username == 'YOUR_ID':
            print("\n" + "="*40)
            print(" [!] NTRIP 접속 정보가 필요합니다.")
            self.username = input(" - 아이디(ID) 입력: ").strip()
        
        # --- Pub/Sub 설정 ---
        # 1. Driver에서 NMEA(GGA)를 받아서 서버로 전송
        self.create_subscription(String, '/nmea', self.nmea_callback, 10)
        
        # 2. 서버에서 받은 RTCM을 Driver로 전송
        self.rtcm_pub = self.create_publisher(ByteMultiArray, '/rtcm', 10)

        self.sock = None
        self.connected = False
        self.buffer = bytearray() # 데이터 조립용 버퍼
        
        # RTCM 수신 스레드 시작
        self.recv_thread = threading.Thread(target=self.receive_rtcm_loop)
        self.recv_thread.daemon = True
        self.recv_thread.start()

        self.get_logger().info(f"NTRIP Client Started. Connecting to {self.host}:{self.mountpoint}")
        self.connect_to_server()

    def connect_to_server(self):
        try:
            if self.sock:
                self.sock.close()
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            
            # NTRIP HTTP Header 전송
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
            self.buffer = bytearray() # 재접속 시 버퍼 초기화
            self.get_logger().info("Connected to NTRIP Caster! Waiting for RTCM...")
            
        except Exception as e:
            self.get_logger().error(f"Connection Failed: {e}")
            self.connected = False

    def nmea_callback(self, msg):
        # PIM222A 드라이버가 보내준 $GPGGA 문장을 서버로 전송
        # (VRS는 내 위치를 보내야 보정정보를 줍니다)
        if self.connected and self.sock:
            try:
                # 줄바꿈 확실히 추가
                nmea_data = msg.data.strip() + "\r\n"
                self.sock.sendall(nmea_data.encode())
                # self.get_logger().info(f"Sent NMEA: {nmea_data.strip()}") # 디버그용
            except Exception as e:
                self.get_logger().warn(f"Failed to send NMEA: {e}")
                self.connected = False
                self.connect_to_server() # 재접속 시도

    # RTCM 패킷 파싱 로직
    def receive_rtcm_loop(self):
        while rclpy.ok():
            if self.connected and self.sock:
                try:
                    # 1. 데이터 수신 및 버퍼 추가
                    data = self.sock.recv(1024)
                    if not data:
                        self.get_logger().warn("Disconnected.")
                        self.connected = False
                        self.connect_to_server()
                        continue
                    
                    self.buffer.extend(data)

                    # 2. 버퍼에서 RTCM 메시지 추출 (0xD3으로 시작)
                    while len(self.buffer) >= 3:
                        # RTCMv3 Preamble 확인 (0xD3)
                        if self.buffer[0] != 0xD3:
                            self.buffer.pop(0) # 쓰레기 데이터 버림
                            continue
                        
                        # 길이 추출 (2바이트 중 뒤 10비트)
                        # RTCM 헤더: [D3] [0000 00LL] [LLLL LLLL]
                        length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
                        total_msg_len = length + 3 + 3 # Header(3) + Body(Length) + CRC(3)
                        
                        if len(self.buffer) < total_msg_len:
                            break # 데이터가 다 안 왔으면 대기
                        
                        # 3. 완전한 메시지 하나 잘라내기
                        rtcm_packet = self.buffer[:total_msg_len]
                        del self.buffer[:total_msg_len] # 버퍼에서 제거
                        
                        # 4. ROS 토픽으로 발행
                        msg = ByteMultiArray()
                        msg.data = [bytes([b]) for b in rtcm_packet]
                        self.rtcm_pub.publish(msg)
                        
                        # self.get_logger().info(f"Pub RTCM Msg: {len(rtcm_packet)} bytes")

                except Exception as e:
                    self.get_logger().warn(f"Receive Error: {e}")
                    self.connected = False
                    time.sleep(1)
            else:
                time.sleep(1) # 접속 대기

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNTRIP()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()