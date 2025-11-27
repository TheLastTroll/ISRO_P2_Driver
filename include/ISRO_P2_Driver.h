#ifndef ISRO_P2_DRIVER_H
#define ISRO_P2_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// [복구됨] 상수 및 Enum 정의
// ---------------------------------------------------------

// Time Status Values (Table 44)
typedef enum {
    GPSTIME_UNKNOWN = 20,       // Time is unknown
    GPSTIME_COARSE = 100,       // Time downloaded from GNSS, Week valid, ranges not accurate
    GPSTIME_FINESTEERING = 180  // Time is accurate
} TIME_STATUS_E;

// Position Type (Table 48)
typedef enum {
    POS_TYPE_NONE = 0,
    POS_TYPE_SINGLE = 16,
    POS_TYPE_PSRDIFF = 17,
    POS_TYPE_NARROW_FLOAT = 34,
    POS_TYPE_L1_INT = 48,
    POS_TYPE_WIDE_INT = 49,
    POS_TYPE_NARROW_INT = 50,   // RTK Fixed (Multi-frequency)
    POS_TYPE_RTK_DIRECT_INS = 51,
    POS_TYPE_INS_PSRSP = 53,
    POS_TYPE_INS_RTKFLOAT = 55,
    POS_TYPE_INS_RTKFIXED = 56  // INS RTK Fixed
} POSITION_TYPE_E;

// INS Status Types (Table 47) - 참고용
typedef enum {
    INS_INACTIVE = 0,
    INS_ALIGNING = 1,
    INS_HIGH_VARIANCE = 2,
    INS_SOLUTION_GOOD = 3,
    INS_SOLUTION_FREE = 6,
    INS_ALIGNMENT_COMPLETE = 7,
    INS_DETERMINING_ORIENTATION = 8,
    INS_WAITING_INITIAL_POS = 9,
    INS_WAITING_AZIMUTH = 10,
    INS_INITIALIZING_BIASES = 11,
    INS_MOTION_DETECT = 12
} INS_STATUS_E;

// ---------------------------------------------------------
// 데이터 구조체 정의
// ---------------------------------------------------------

// PVA 메시지 구조체 (Message ID: 2379)
// Table 46: PVA Message Definition
typedef struct __attribute__((packed)) {
    // --- Wire Payload (장비에서 오는 실제 데이터 126 bytes) ---
    uint32_t ins_status;              // Field 1: INS Status (INS_STATUS_E)
    uint32_t position_type;           // Field 2: Position Type (POSITION_TYPE_E)
    double   latitude;                // Field 3: Latitude (degrees)
    double   longitude;               // Field 4: Longitude (degrees)
    double   height;                  // Field 5: Height (m)
    float    undulation;              // Field 6: Undulation (m)
    double   velocity_x;              // Field 7: Velocity X component (m/s)
    double   velocity_y;              // Field 8: Velocity Y component (m/s)
    double   velocity_z;              // Field 9: Velocity Z component (m/s)
    double   attitude_roll;           // Field 10: Attitude roll (degrees)
    double   attitude_pitch;          // Field 11: Attitude pitch (degrees)
    double   attitude_azimuth;        // Field 12: Attitude azimuth (degrees)
    float    lat_std_dev;             // Field 13: Latitude standard deviation
    float    lon_std_dev;             // Field 14: Longitude standard deviation
    float    height_std_dev;          // Field 15: Height standard deviation
    float    vel_x_std_dev;           // Field 16: Velocity X standard deviation
    float    vel_y_std_dev;           // Field 17: Velocity Y standard deviation
    float    vel_z_std_dev;           // Field 18: Velocity Z standard deviation
    float    roll_std_dev;            // Field 19: Attitude roll standard deviation
    float    pitch_std_dev;           // Field 20: Attitude pitch standard deviation
    float    azimuth_std_dev;         // Field 21: Attitude azimuth standard deviation
    uint32_t extended_solution_status; // Field 22: Extended solution status (Table 49)
    uint16_t time_since_update;       // Field 23: Time since last position update (s)

    // --- Internal Use Only (드라이버가 채우는 필드) ---
    uint16_t header_gps_week;         // From Automotive Header
    uint32_t header_gps_ms;           // From Automotive Header
} PVA_MESSAGE_T;

// IMU 메시지 구조체 (Message ID: 2389)
// Table 52: IMU Message Definition
typedef struct __attribute__((packed)) {
    double   gps_second;              // Field 1: GPS Second
    int32_t  scaled_accel_z;          // Field 2: Scaled Z Acceleration
    int32_t  scaled_accel_y;          // Field 3: Scaled Y Acceleration
    int32_t  scaled_accel_x;          // Field 4: Scaled X Acceleration
    int32_t  scaled_gyro_z;           // Field 5: Scaled Z Gyro
    int32_t  scaled_gyro_y;           // Field 6: Scaled Y Gyro
    int32_t  scaled_gyro_x;           // Field 7: Scaled X Gyro
    uint32_t imu_status_mask;         // Field 8: IMU Status Mask
} IMU_MESSAGE_T;

// ---------------------------------------------------------
// 설정 및 API 정의
// ---------------------------------------------------------

// 연결 모드 정의
typedef enum {
    CONN_TYPE_SERIAL = 0,
    CONN_TYPE_TCP_CLIENT = 1,
    CONN_TYPE_TCP_SERVER = 2
} P2_CONN_TYPE_E;

// 설정 구조체
typedef struct {
    P2_CONN_TYPE_E type;
    char serial_port[64];
    int serial_baud;
    char tcp_ip[64]; 
    int tcp_port;
} P2_Config_T;

typedef void (*IMU_Callback)(const IMU_MESSAGE_T* imu, void* user_data);
typedef struct ISRO_P2_T ISRO_P2_T;

// API 함수
ISRO_P2_T* P2_Open(const P2_Config_T* config);
void P2_Close(ISRO_P2_T* device);

int P2_GetPVA(ISRO_P2_T* device, PVA_MESSAGE_T* pva);
int P2_GetIMU(ISRO_P2_T* device, IMU_MESSAGE_T* imu);

// [NTRIP용] NMEA 데이터 가져오기
int P2_GetNMEA(ISRO_P2_T* device, char* buffer, int buffer_len);

// [RTK용] RTCM 데이터 전송하기
int P2_SendRTCM(ISRO_P2_T* device, const uint8_t* rtcm_data, uint32_t len);

void P2_SetIMUCallback(ISRO_P2_T* device, IMU_Callback callback, void* user_data);

#ifdef __cplusplus
}
#endif
#endif // ISRO_P2_DRIVER_H