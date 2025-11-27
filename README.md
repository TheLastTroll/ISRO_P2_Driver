# ISRO_P2_Driver

PIMTP (PIM Transfer Protocol) 기반 GNSS/INS 장비용 ROS2 드라이버입니다.

GNSS/INS 수신기로부터 PVA (Position, Velocity, Attitude) 및 IMU 데이터를 파싱하여 표준 ROS2 메시지로 발행합니다.

## 주요 기능

- **다중 연결 모드**: Serial, TCP Client, TCP Server 지원
- **표준 ROS2 메시지**: NavSatFix, Imu, TwistWithCovarianceStamped
- **RTK 지원**: NTRIP 클라이언트 연동을 통한 RTK 보정
- **GPS 시간 동기화**: GPS Week/Milliseconds 기반 정밀 타임스탬프
- **ENU/NED 좌표 변환**: 설정 가능한 좌표계
- **고속 IMU 콜백**: 100Hz 이상의 IMU 데이터 콜백 지원

## 지원 장비

- PIM222A (PIMTP 프로토콜)
- NovAtel OEM7 Automotive 메시지 포맷 사용 장비

## 요구사항

- ROS2 Humble / Iron / Jazzy
- Ubuntu 22.04 / 24.04

## 설치

```bash
# 워크스페이스에 클론
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/ISRO_P2_Driver.git

# 빌드
cd ~/ros2_ws
colcon build --packages-select ISRO_P2_Driver

# 환경 설정
source install/setup.bash
```

## 디렉토리 구조

```
ISRO_P2_Driver/
├── CMakeLists.txt
├── package.xml
├── README.md                 	 # 한글 매뉴얼
├── include/
│   └── ISRO_P2_Driver.h         # C 드라이버 헤더
├── src/
│   ├── ISRO_P2_Driver.c         # C 드라이버 (PIMTP 파서)
│   └── ISRO_P2_Driver_node.cpp  # ROS2 노드
├── scripts/
│   └── ntrip.py                 # NTRIP 클라이언트 노드
├── launch/
│   └── ISRO_P2_Driver_launch.py
└── config/
    ├── serial_mode.yaml
    ├── client_mode.yaml
    └── server_mode.yaml
```

## 사용법

### Serial 모드 (기본)

```bash
ros2 launch ISRO_P2_Driver ISRO_P2_Driver_launch.py mode:=serial
```

### TCP Client 모드

장비에 TCP로 접속:

```bash
ros2 launch ISRO_P2_Driver ISRO_P2_Driver_launch.py mode:=client
```

### TCP Server 모드

장비의 접속을 대기:

```bash
ros2 launch ISRO_P2_Driver ISRO_P2_Driver_launch.py mode:=server
```

## 설정

`config/` 디렉토리의 YAML 파일을 수정하여 설정합니다.

### serial_mode.yaml

```yaml
ISRO_P2_Driver_node:
  ros__parameters:
    mode: "serial"
    serial:
      port: "/dev/ttyUSB0"    # 시리얼 포트
      baud: 921600            # 통신 속도
    frame_id: "gps_link"      # GPS 프레임 ID
    imu_frame_id: "imu_link"  # IMU 프레임 ID
    publish_rate: 20.0        # 발행 주기 (Hz)
    publish_raw_imu: true     # Raw IMU 발행 여부
    use_enu: true             # ENU 좌표계 사용 (ROS 표준)
```

### client_mode.yaml

```yaml
ISRO_P2_Driver_node:
  ros__parameters:
    mode: "tcp_client"
    tcp:
      ip: "192.168.1.100"     # 장비 IP 주소
      port: 3000              # 장비 포트
    frame_id: "gps_link"
    publish_rate: 20.0
    use_enu: true
```

### server_mode.yaml

```yaml
ISRO_P2_Driver_node:
  ros__parameters:
    mode: "tcp_server"
    tcp:
      port: 3000              # 대기할 포트 번호
```

## 토픽

### 발행 토픽 (Published)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/fix` | sensor_msgs/NavSatFix | GPS 위치 (위도, 경도, 고도) |
| `/vel` | geometry_msgs/TwistWithCovarianceStamped | 속도 (ENU 또는 NED) |
| `/imu/data` | sensor_msgs/Imu | PVA 기반 자세 (Orientation) |
| `/imu/raw` | sensor_msgs/Imu | Raw IMU 가속도 & 자이로 |
| `/nmea` | std_msgs/String | NMEA GGA 문장 (NTRIP용) |
| `/pva/status_debug` | std_msgs/String | 전체 디버그 정보 |

### 구독 토픽 (Subscribed)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/rtcm` | std_msgs/ByteMultiArray | NTRIP으로부터 RTK 보정 데이터 |

## RTK 설정 (NTRIP)

드라이버와 함께 NTRIP 클라이언트 노드를 실행합니다:

```bash
# 터미널 1: 드라이버 실행
ros2 launch ISRO_P2_Driver ISRO_P2_Driver_launch.py mode:=serial

# 터미널 2: NTRIP 클라이언트 실행 (국토지리정보원 예시)
ros2 run ISRO_P2_Driver ntrip.py --ros-args \
  -p host:="rts2.ngii.go.kr" \
  -p port:=2101 \
  -p mountpoint:="VRS-RTCM32" \
  -p username:="사용자ID" \
  -p password:="ngii"
```

### RTK 데이터 흐름

```
장비 → NMEA GGA → /nmea 토픽 → NTRIP 클라이언트 → Caster 서버
                                      ↓
장비 ← PIMTP Type2 ← /rtcm 토픽 ← RTCM3 보정 데이터
```

### 국토지리정보원 VRS 서비스

- **호스트**: `rts2.ngii.go.kr`
- **포트**: `2101`
- **마운트포인트**: `VRS-RTCM32` (RTCM 3.2 권장)
- **비밀번호**: `ngii` (고정)
- **아이디**: 국토지리정보원 회원가입 필요

## 메시지 상세

### PVA 메시지 (MSG_ID: 2379)

위치, 속도, 자세 데이터와 각 표준편차를 포함합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| latitude | double | 위도 (degrees) |
| longitude | double | 경도 (degrees) |
| height | double | 타원체고 (m) |
| velocity_x/y/z | double | 속도 (m/s) |
| attitude_roll/pitch/azimuth | double | 자세 (degrees) |
| lat/lon/height_std_dev | float | 위치 표준편차 |
| position_type | uint32 | 측위 타입 |
| ins_status | uint32 | INS 상태 |

### IMU 메시지 (MSG_ID: 2389)

100Hz Raw 가속도계 및 자이로스코프 데이터.

## Position Type (측위 타입)

| 코드 | 이름 | 설명 |
|------|------|------|
| 0 | NONE | 측위 불가 |
| 16 | SINGLE | 단독 측위 |
| 17 | PSRDIFF | DGPS |
| 34 | NARROW_FLOAT | RTK Float |
| 50 | NARROW_INT | RTK Fixed (최상) |
| 56 | INS_RTKFIXED | INS + RTK Fixed |

## INS Status (INS 상태)

| 코드 | 이름 | 설명 |
|------|------|------|
| 0 | INACTIVE | INS 비활성 |
| 1 | ALIGNING | 초기 정렬 중 |
| 2 | HIGH_VARIANCE | 높은 분산 (정확도 낮음) |
| 3 | SOLUTION_GOOD | 정상 INS 솔루션 |
| 7 | ALIGNMENT_COMPLETE | 정렬 완료 |
| 8 | DETERMINING_ORIENTATION | 방위 결정 중 |

## 좌표계

### use_enu: true (ROS 표준)
- X = 동쪽 (East)
- Y = 북쪽 (North)  
- Z = 위쪽 (Up)
- Yaw = 동쪽이 0°, 반시계 방향 양수

### use_enu: false (NED)
- X = 북쪽 (North)
- Y = 동쪽 (East)
- Z = 아래쪽 (Down)
- Yaw = 북쪽이 0°, 시계 방향 양수

## 문제 해결

### 데이터가 수신되지 않음

```bash
# 시리얼 포트 권한 확인
sudo chmod 666 /dev/ttyUSB0

# dialout 그룹에 사용자 추가
sudo usermod -aG dialout $USER
# 로그아웃 후 다시 로그인 필요

# 시리얼 포트 확인
ls -la /dev/ttyUSB*
```

### RTK가 작동하지 않음

1. NTRIP 인증 정보 확인
2. NMEA 발행 확인: `ros2 topic echo /nmea`
3. 장비의 RTK 입력 설정 확인
4. 인터넷 연결 상태 확인

### Position Type이 NARROW_FLOAT에서 멈춤

- 안테나 설치 환경 확인 (멀티패스)
- 기준국과의 거리 확인 (VRS 사용 권장)
- 위성 수신 상태 확인
- 충분한 시간 대기 (수 분 소요 가능)

### CPU 사용량이 높음

- `publish_rate` 파라미터 조정
- Serial 설정의 VMIN/VTIME 확인

## C 드라이버 API

```c
// 장비 연결 열기
ISRO_P2_T* P2_Open(const P2_Config_T* config);

// 연결 닫기
void P2_Close(ISRO_P2_T* device);

// 최신 PVA 데이터 가져오기 (Non-blocking)
int P2_GetPVA(ISRO_P2_T* device, PVA_MESSAGE_T* pva);

// 최신 IMU 데이터 가져오기 (Non-blocking)
int P2_GetIMU(ISRO_P2_T* device, IMU_MESSAGE_T* imu);

// 최신 NMEA 문장 가져오기
int P2_GetNMEA(ISRO_P2_T* device, char* buffer, int buffer_len);

// RTCM 보정 데이터 전송
int P2_SendRTCM(ISRO_P2_T* device, const uint8_t* rtcm_data, uint32_t len);

// 고속 IMU 콜백 설정
void P2_SetIMUCallback(ISRO_P2_T* device, IMU_Callback callback, void* user_data);
```

## 디버그 토픽 활용

`/pva/status_debug` 토픽에서 상세 정보를 확인할 수 있습니다:

```bash
ros2 topic echo /pva/status_debug
```

출력 예시:
```
========== PVA FULL DEBUG INFO ==========
[Time] GPS Week: 2345, MS: 123456000
[Position] Lat: 37.12345678, Lon: 127.12345678, Alt: 50.123 m
[Velocity] Vx: 0.01, Vy: 0.02, Vz: 0.00 m/s
[Attitude] Roll: 0.5, Pitch: -0.3, Azimuth: 45.2 deg
-----------------------------------------
[Status]   Pos Type: 50 (NARROW_INT)
           INS Status: 3 (SOLUTION_GOOD)
[Extended] Hex Code: 0x02000143
-----------------------------------------
  [O] Position Update Used
  [O] Phase Update Used
  [O] INS Solution Converged (GOOD)
  [*] Alignment State: Dual Antenna
=========================================
```

## 라이선스

MIT License

## 작성자

이정환 (jhlee@insungsys.kr)  
인성 기술연구소

## 참고

- PIMTP 프로토콜: NovAtel OEM7 Automotive Message Format 기반
- NTRIP: 국토지리정보원(NGII) VRS 서비스 연동
