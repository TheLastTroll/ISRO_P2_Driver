# ISRO_P2_Driver

PIMTP (PIM Transfer Protocol) 기반 GNSS/INS 장비 ISRO-P2/ISRO-P2-E용 ROS2 드라이버입니다.

GNSS/INS 수신기로부터 PVA (Position, Velocity, Attitude) 및 IMU 데이터를 파싱하여 표준 ROS2 메시지로 발행합니다.

## 주요 기능

- **다중 연결 모드**: Serial(ISRO-P2), TCP Client(ISRO-P2-E), TCP Server(ISRO-P2-E) 지원
- **표준 ROS2 메시지**: NavSatFix, Imu, TwistWithCovarianceStamped
- **RTK 지원**: NTRIP 클라이언트 연동을 통한 RTK 보정 (외부망 필요)
- **GPS 시간 동기화**: GPS Week/Milliseconds 기반 정밀 타임스탬프
- **ENU/NED 좌표 변환**: 설정 가능한 좌표계
- **하드웨어 상태 모니터링**: 온도, 전압, CPU 사용률 실시간 모니터링
- **원격 리셋 서비스**: ROS2 서비스를 통한 장비 리셋

## 지원 장비

- PIM222A (PIMTP 프로토콜)

## 장비 설정 (PIM222A)

드라이버 사용 전 장비에 아래 설정이 적용되어 있어야 합니다.

### 권장 설정 예시

```
IMU:INTERNAL
NMEATALKER:AUTO                       # NMEA Talker ID (AUTO/GP)

# === 통신 속도 설정 ===
BaudRate:COM1,921600                  # COM1: 고속 데이터 (PIMTP)
BaudRate:COM2,115200                  # COM2: NMEA 출력 (모니터링/RTK용)

# === PIMTP Automotive 로그 설정 (COM1) ===
LogAutomotive:COM1,PVA,TIME,0.02,0     # PVA 메시지: 50Hz (또는 원하는 주기 1.0, 0.2, 0.1, 0.05, 0.02)
LogAutomotive:COM1,IMU,CHANGE,0,0     # IMU 메시지 (100Hz)
LogAutomotive:COM1,STATUS,CHANGE,0,0  # STATUS 메시지: 상태 변경 시

# === NMEA 로그 설정 ===
LogNMEA:COM1,GGA,TIME,1.0,0           # COM1: GGA 1Hz (NTRIP용)
LogNMEA:COM2,GGA,TIME,1.0,0           # COM2: GGA 1Hz (모니터링/RTK용)
LogNMEA:COM2,GSV,TIME,1.0,0           # COM2: 위성 정보
LogNMEA:COM2,PASHR,TIME,1.0,0         # COM2: 자세 정보
LogNMEA:COM2,HDT,CHANGE,0,0           # COM2: DUALANTENNA Heading (계산될경우)

# === INS 레버암 설정 (차량별 측정 필요) ===
INSRotation:RBV,0.0,0.0,0.0,3.0,3.0,3.0           # 권장 측정오차 3.0 입력
# 형식: INSRotation:RBV,x,y,z,xStd,yStd,zStd
# - IMU 설치 각도 (degrees)
# - 표준편차 (degrees)
# - 0 0 0 -> y 전진 x 우측 z 상향

INSTranslation:Ant1,0.00,-1.20,1.00,0.05,0.05,0.05
# 형식: INSTranslation:Ant1,X,Y,Z,XStd,YStd,ZStd
# - IMU 중심 → 주 안테나 오프셋 (meters)
# - 축 기준으로 거리 입력

INSTranslation:Ant2,0.00,1.80,1.00,0.05,0.05,0.05
# 형식: INSTranslation:Ant2,X,Y,Z,XStd,YStd,ZStd
# - IMU 중심 → 보조 안테나 오프셋 (meters)
# - 듀얼 안테나 헤딩 사용 시 필수
```

## 요구사항

- ROS2 Humble / Iron / Jazzy
- Ubuntu 22.04 / 24.04

## 설치

```bash
# 워크스페이스에 클론
cd ~/ros2_ws/src
git clone https://github.com/TheLastTroll/ISRO_P2_Driver.git

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
├── README.md                    # 한글 매뉴얼
├── include/
│   └── ISRO_P2_Driver.h         # C 드라이버 헤더
├── src/
│   ├── ISRO_P2_Driver.c         # C 드라이버 (PIMTP 파서)
│   └── ISRO_P2_Driver_node.cpp  # ROS2 노드
├── scripts/
│   └── ntrip.py                 # NTRIP 클라이언트 노드
├── launch/
│   └── ISRO_P2_Driver.launch.py
└── config/
    ├── serial_mode.yaml
    ├── client_mode.yaml
    └── server_mode.yaml
```

## 사용법

### Serial 모드 (기본)

```bash
ros2 launch ISRO_P2_Driver ISRO_P2_Driver.launch.py mode:=serial
```

### TCP Client 모드

장비에 TCP로 접속:

```bash
ros2 launch ISRO_P2_Driver ISRO_P2_Driver.launch.py mode:=client
```

### TCP Server 모드

장비의 접속을 대기:

```bash
ros2 launch ISRO_P2_Driver ISRO_P2_Driver.launch.py mode:=server
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
      baud: 921600            # 통신 속도 (115200, 460800, 921600 지원)
    frame_id: "gps_link"      # GPS 프레임 ID
    imu_frame_id: "imu_link"  # IMU 프레임 ID
    publish_rate: 100.0       # 발행 주기 (Hz) - 장비 출력과 맞춤
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
    imu_frame_id: "imu_link"
    publish_rate: 100.0
    publish_raw_imu: true
    use_enu: true
```

### server_mode.yaml

```yaml
ISRO_P2_Driver_node:
  ros__parameters:
    mode: "tcp_server"
    tcp:
      port: 3000              # 대기할 포트 번호
    frame_id: "gps_link"
    imu_frame_id: "imu_link"
    publish_rate: 100.0
    publish_raw_imu: true
    use_enu: true
```

## ROS2 인터페이스

### 발행 토픽 (Published Topics)

| 토픽 | 타입 | 주기 | 설명 |
|------|------|------|------|
| `/fix` | sensor_msgs/NavSatFix | PVA | GPS 위치 (위도, 경도, 고도) |
| `/vel` | geometry_msgs/TwistWithCovarianceStamped | PVA | Body Frame 속도 (NEU속도에서 FLU로 회전변환) |
| `/imu/data` | sensor_msgs/Imu | PVA | PVA 기반 자세 (Orientation) |
| `/imu/raw` | sensor_msgs/Imu | 100Hz | Raw IMU 가속도 & 자이로 (콜백 기반) |
| `/nmea` | std_msgs/String | 1Hz | NMEA GGA 문장 (NTRIP용) |
| `/pva/status_debug` | std_msgs/String | PVA | 전체 디버그 정보 |
| `/pva/hardware_status` | std_msgs/String | STATUS | 하드웨어 상태 (온도, 전압, CPU) |

### 구독 토픽 (Subscribed Topics)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/rtcm` | std_msgs/ByteMultiArray | NTRIP으로부터 RTK 보정 데이터 |

### 서비스 (Services)

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/isro_p2/reset` | std_srvs/Trigger | 장비 리셋 명령 전송 |

**리셋 서비스 사용 예시:**

```bash
ros2 service call /isro_p2/reset std_srvs/srv/Trigger
```

## RTK 설정 (NTRIP)

드라이버와 함께 NTRIP 클라이언트 노드를 실행합니다.

### 대화형 모드 (권장)

```bash
# 터미널 1: 드라이버 실행
ros2 launch ISRO_P2_Driver ISRO_P2_Driver_launch.py mode:=serial

# 터미널 2: NTRIP 클라이언트 실행 (대화형)
ros2 run ISRO_P2_Driver ntrip.py
```

실행 시 대화형으로 설정을 입력받습니다:

```
==================================================
       NTRIP 클라이언트 설정
==================================================

[?] 국토지리정보원 VRS-RTCM32 서비스를 사용하시겠습니까?
    (Host: rts2.ngii.go.kr, Port: 2101, Mount: VRS-RTCM32)

    VRS-RTCM32 사용 (Y/n): y

    [√] 국토지리정보원 VRS-RTCM32 설정 적용

    사용자 ID (국토지리정보원 가입 ID): myusername

--------------------------------------------------
    [설정 완료]
    Host:       rts2.ngii.go.kr
    Port:       2101
    Mountpoint: VRS-RTCM32
    Username:   myusername
    Password:   ****
==================================================
```

### 커스텀 NTRIP Caster 사용 시

`n`을 입력하면 개별 설정을 입력받습니다:

```
    VRS-RTCM32 사용 (Y/n): n

    [i] 커스텀 NTRIP Caster 설정을 입력하세요.
--------------------------------------------------
    Host [rts2.ngii.go.kr]: rtk.example.com
    Port [2101]: 2102
    Mountpoint [VRS-RTCM32]: RTCM3_MSM
    Username: user123
    Password [ngii]: mypassword
```

### 비대화형 모드 (스크립트/자동화용)

파라미터를 모두 지정하면 대화형 입력을 건너뜁니다:

```bash
ros2 run ISRO_P2_Driver ntrip.py --ros-args \
  -p host:="rts2.ngii.go.kr" \
  -p port:=2101 \
  -p mountpoint:="VRS-RTCM32" \
  -p username:="사용자ID" \
  -p password:="ngii" \
  -p interactive:=false
```

### NTRIP 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `host` | `rts2.ngii.go.kr` | NTRIP Caster 호스트 |
| `port` | `2101` | NTRIP Caster 포트 |
| `mountpoint` | `VRS-RTCM32` | 마운트포인트 |
| `username` | (없음) | 사용자 ID |
| `password` | `ngii` | 비밀번호 |
| `interactive` | `true` | 대화형 모드 활성화 |

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

## PIMTP 프로토콜

### 패킷 구조

```
+------------+---------------+---------+-------+
| PIMTP Hdr  | Payload       | Payload | CRC32 |
| (12 bytes) | Header        | Data    | (4B)  |
+------------+---------------+---------+-------+
```

**PIMTP 헤더 (12 bytes):**

| 오프셋 | 크기 | 필드 | 설명 |
|--------|------|------|------|
| 0 | 4 | Sync | `0xAC 0x55 0x96 0x83` |
| 4 | 2 | Payload Type | 페이로드 종류 |
| 6 | 2 | Reserved | 예약 |
| 8 | 4 | Payload Length | 페이로드 길이 |

**Payload Type:**

| 값 | 이름 | 방향 | 설명 |
|----|------|------|------|
| 1 | AUTOMOTIVE | 수신 | PVA, IMU, STATUS 메시지 |
| 2 | RTK_CORRECTIONS | 송신 | RTCM 보정 데이터 |
| 3 | UPDATE_DATA | 송신 | 펌웨어 업데이트 |
| 4 | NMEA | 수신 | NMEA 문장 |

### Automotive 메시지 헤더 (16 bytes)

| 오프셋 | 크기 | 필드 | 설명 |
|--------|------|------|------|
| 0 | 2 | Message ID | 메시지 종류 |
| 2 | 2 | Data Size | 데이터 크기 |
| 4 | 2 | Time Status | GPS 시간 상태 |
| 6 | 2 | GPS Week | GPS 주차 |
| 8 | 4 | GPS Millisecond | GPS 밀리초 |
| 12 | 4 | Reserved | 예약 |

### 메시지 ID

| ID | 이름 | 크기 | 설명 |
|----|------|------|------|
| 2379 | PVA | 126B | 위치, 속도, 자세 |
| 2389 | IMU | 40B | Raw IMU 데이터 |
| 2393 | STATUS | 60B | 하드웨어 상태 |
| 2410 | RESET | 12B | 리셋 명령 |

## 메시지 상세

### PVA 메시지 (MSG_ID: 2379, 126 bytes)

위치, 속도, 자세 데이터와 각 표준편차를 포함합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| ins_status | uint32 | INS 상태 |
| position_type | uint32 | 측위 타입 |
| latitude | double | 위도 (degrees) |
| longitude | double | 경도 (degrees) |
| height | double | 타원체고도 (m) |
| undulation | float | 지오이드 높이 (m) |
| velocity_x | double | 북방향 속도 (m/s) |
| velocity_y | double | 동방향 속도 (m/s) |
| velocity_z | double | 상방향 속도 (m/s) |
| attitude_roll | double | 롤 (degrees) |
| attitude_pitch | double | 피치 (degrees) |
| attitude_azimuth | double | 방위각 (degrees, CW from North) |
| lat_std_dev | float | 위도 표준편차 (m) |
| lon_std_dev | float | 경도 표준편차 (m) |
| height_std_dev | float | 고도 표준편차 (m) |
| vel_x/y/z_std_dev | float | 속도 표준편차 (m/s) |
| roll/pitch/azimuth_std_dev | float | 자세 표준편차 (deg) |
| extended_solution_status | uint32 | 확장 솔루션 상태 |
| time_since_update | uint16 | 마지막 위치 업데이트 이후 시간 (s) |

### IMU 메시지 (MSG_ID: 2389, 40 bytes)

100Hz Raw 가속도계 및 자이로스코프 데이터.

| 필드 | 타입 | 설명 |
|------|------|------|
| gps_second | double | GPS 초 |
| scaled_accel_z/y/x | int32 | 스케일링된 가속도 |
| scaled_gyro_z/y/x | int32 | 스케일링된 각속도 |
| imu_status_mask | uint32 | IMU 상태 마스크 |

**IMU 스케일 팩터:**
- 가속도: `value * 1.0e-9 * 100` (m/s², 100Hz 기준)
- 각속도: `value * 1.0e-10 * 100` (rad/s, 100Hz 기준)

**IMU Status Mask 비트:**
- Bit 1 (0x02): IMU Corrected (바이어스 보정 완료)

### STATUS 메시지 (MSG_ID: 2393, 60 bytes)

하드웨어 상태 모니터링 데이터.

| 필드 | 타입 | 설명 |
|------|------|------|
| error_word | uint32 | 에러 상태 비트필드 |
| status_word | uint32 | 상태 비트필드 |
| aux1~5_word | uint32 | 보조 상태 |
| rx_idle_time | uint8 | 수신기 유휴 시간 (0.5% 단위) |
| me1/2_idle_time | uint8 | 측정 엔진 유휴 시간 |
| temperature | float | 내부 온도 (°C) |
| voltage_rtc | float | RTC 전압 (V) |
| voltage_3v3 | float | 3.3V 레일 전압 (V) |
| voltage_3v0 | float | 입력 전압 (V) |
| voltage_2v85 | float | 2.85V 레일 전압 (V) |
| voltage_1v35 | float | 1.35V 레일 전압 (V) |
| voltage_1v20 | float | 1.20V 레일 전압 (V) |

## Position Type (측위 타입)

| 코드 | 이름 | 설명 |
|------|------|------|
| 0 | NONE | 측위 불가 |
| 16 | SINGLE | 단독 측위 |
| 19 | PROPAGATED | 전파 측위 |
| 32 | L1_FLOAT | L1 RTK Float |
| 34 | NARROW_FLOAT | RTK Float |
| 48 | L1_INT | L1 RTK Fixed |
| 49 | WIDE_INT | Wide Lane RTK Fixed |
| 50 | NARROW_INT | RTK Fixed (권장) |
| 51 | RTK_DIRECT_INS | RTK Direct INS |
| 53 | INS_PSRSP | INS + Pseudorange |
| 55 | INS_RTKFLOAT | INS + RTK Float |
| 56 | INS_RTKFIXED | INS + RTK Fixed (최상) |

## INS Status (INS 상태)

| 코드 | 이름 | 설명 |
|------|------|------|
| 0 | INACTIVE | INS 비활성 |
| 1 | ALIGNING | 초기 정렬 중 |
| 2 | HIGH_VARIANCE | 높은 분산 (정확도 낮음) |
| 3 | SOLUTION_GOOD | 정상 INS 솔루션 |
| 6 | SOLUTION_FREE | INS 독립 솔루션 (정확도 낮아지는 중) |
| 7 | ALIGNMENT_COMPLETE | 정렬 완료 |
| 8 | DETERMINING_ORIENTATION | 방위 결정 중 |
| 9 | WAITING_INITIAL_POS | 초기 위치 대기 (주 안테나의 위치 정확도가 낮은 경우 발생) |
| 10 | WAITING_AZIMUTH | 동체 전진방향 대기 |
| 11 | INITIALIZING_BIASES | 바이어스 초기화 중 |
| 12 | MOTION_DETECT | 움직임 감지 |

## Extended Solution Status (확장 솔루션 상태)

비트필드로 구성된 상세 솔루션 정보입니다.

| 비트 | 설명 |
|------|------|
| 0x00000001 | Position Update Used |
| 0x00000002 | Phase Update Used |
| 0x00000004 | Zero Velocity Update Used |
| 0x00000008 | Wheel Sensor Update Used |
| 0x00000010 | ALIGN (Heading) Update Used |
| 0x00000020 | External POS Update Used |
| 0x00000040 | INS Solution Converged |
| 0x00000080 | Doppler Update Used |
| 0x00000100 | Pseudorange Update Used |
| 0x00000200 | Velocity Update Used |
| 0x00000800 | Dead Reckoning Update Used |
| 0x00001000 | Phase Wind Up Update Used |
| 0x00002000 | Course Over Ground Update Used |
| 0x00004000 | External Velocity Update Used |
| 0x00008000 | External Attitude Update Used |
| 0x00010000 | External Heading Update Used |
| 0x00020000 | External Height Update Used |
| 0x00100000 | Rover Position Update Used |
| 0x00200000 | Rover Position Update Type (RTK Int) |
| 0x01000000 | Static Turn-on Biases Estimated |
| 0x02000000 | Alignment Direction Verified |

**Alignment Indicator (비트 26-28):**

| 값 | 설명 |
|----|------|
| 0 | Incomplete Alignment |
| 1 | Static |
| 2 | Kinematic |
| 3 | Dual Antenna |
| 4 | User Command |

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
- 정렬되지 않은경우 /fix에서는 single로 표기될 수 있음

### CPU 사용량이 높음

- `publish_rate` 파라미터 조정
- Serial 설정의 VMIN/VTIME은 드라이버에서 최적화됨 (VMIN=32, VTIME=1)

### CRC 오류가 발생함

- 시리얼 케이블 연결 상태 확인
- 통신 속도 설정 확인 (장비와 동일해야 함)
- 로그에서 CRC 불일치 메시지 확인

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

// 하드웨어 상태 가져오기
int P2_GetStatus(ISRO_P2_T* device, STATUS_MESSAGE_T* status);

// 장비 리셋 명령 전송
int P2_SendReset(ISRO_P2_T* device);

// 고속 IMU 콜백 설정
void P2_SetIMUCallback(ISRO_P2_T* device, IMU_Callback callback, void* user_data);
```

### 연결 설정 구조체

```c
typedef enum {
    CONN_TYPE_SERIAL = 0,
    CONN_TYPE_TCP_CLIENT = 1,
    CONN_TYPE_TCP_SERVER = 2
} P2_CONN_TYPE_E;

typedef struct {
    P2_CONN_TYPE_E type;
    char serial_port[64];
    int serial_baud;
    char tcp_ip[64]; 
    int tcp_port;
} P2_Config_T;
```

## 디버그 토픽 활용

`/pva/status_debug` 토픽에서 상세 정보를 확인할 수 있습니다:

```bash
ros2 topic echo /pva/status_debug --field data
```

출력 예시:

```
========== PVA FULL DEBUG INFO ==========
[Time] GPS Week: 2345, MS: 123456000
[Position] Lat: 37.12345678, Lon: 127.12345678, Alt: 50.123 m
[Velocity] VN: 0.01, VE: 0.02, VU: 0.00 m/s
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

### 하드웨어 상태 모니터링

`/pva/hardware_status` 토픽에서 하드웨어 상태를 확인할 수 있습니다:

```bash
ros2 topic echo /pva/hardware_status
```

출력 예시:

```
=== System Health ===
Temp: 45.2 C
Voltages: [3.3V: 3.31V] [Input: 12.1V]
CPU Idle: 85.0 %
Error Word: 0x0
```

## 기술 참고사항

### GPS 시간 동기화

드라이버는 GPS Week와 Milliseconds를 사용하여 정밀 ROS 타임스탬프를 생성합니다:

- GPS Epoch: 1980-01-06
- Unix Epoch 오프셋: 315964800초
- 현재 윤초(Leap Seconds): 18초 (2017년 이후)

### 링 버퍼

- 크기: 128KB (131072 bytes)
- 오버플로우 시 가장 오래된 데이터 덮어쓰기
- 스레드 안전 (mutex + condition variable)

### 파서 상태 머신

```
STATE_SEARCH_SYNC → STATE_READ_PIMTP_HEADER → STATE_READ_PAYLOAD → STATE_READ_CRC
        ↓
STATE_PROCESS_NMEA (Raw NMEA '$' 감지 시)
```

## 라이선스

MIT License

## 작성자

이정환 (rnd@insungsys.kr)  
인성 기술연구소
www.insungsys.kr


## 참고

- PIMTP 프로토콜: NovAtel OEM7 Automotive Message Format 기반
- NTRIP: 국토지리정보원(NGII) VRS 서비스 연동
- CRC-32: NovAtel 표준 다항식 (0xEDB88320)