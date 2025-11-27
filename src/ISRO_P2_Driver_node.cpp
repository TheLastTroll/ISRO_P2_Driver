#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
// [수정] 헤더 파일명은 소문자(byte_multi_array.hpp)여야 한다
#include "std_msgs/msg/byte_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp" // 리셋 서비스용

#include <sstream>
#include <string>
#include <cmath>
#include "ISRO_P2_Driver.h"

using namespace std::chrono_literals;

class ISRO_P2_DriverNode : public rclcpp::Node {
public:
    ISRO_P2_DriverNode() : Node("isro_p2_driver_node") {
        // ---------------------------------------------------------
        // 1. 파라미터 선언
        // ---------------------------------------------------------
        this->declare_parameter("mode", "serial"); 
        this->declare_parameter("serial.port", "/dev/ttyUSB0");
        this->declare_parameter("serial.baud", 921600);
        this->declare_parameter("tcp.ip", "192.168.1.100");
        this->declare_parameter("tcp.port", 3000);
        
        this->declare_parameter("frame_id", "gps_link");
        this->declare_parameter("imu_frame_id", "imu_link");
        this->declare_parameter("publish_rate", 100.0);
        this->declare_parameter("publish_raw_imu", true);
        this->declare_parameter("use_enu", true); // True: ENU(ROS표준), False: NED(장비표준)

        // ---------------------------------------------------------
        // 2. 드라이버 설정 및 연결
        // ---------------------------------------------------------
        P2_Config_T config = {CONN_TYPE_SERIAL}; 
        std::string mode = this->get_parameter("mode").as_string();

        if (mode == "serial") {
            config.type = CONN_TYPE_SERIAL;
            std::string p = this->get_parameter("serial.port").as_string();
            strncpy(config.serial_port, p.c_str(), sizeof(config.serial_port)-1);
            config.serial_baud = this->get_parameter("serial.baud").as_int();
            RCLCPP_INFO(this->get_logger(), "Mode: SERIAL (%s @ %d)", config.serial_port, config.serial_baud);
        } else if (mode == "tcp_client") {
            config.type = CONN_TYPE_TCP_CLIENT;
            std::string ip = this->get_parameter("tcp.ip").as_string();
            strncpy(config.tcp_ip, ip.c_str(), sizeof(config.tcp_ip)-1);
            config.tcp_port = this->get_parameter("tcp.port").as_int();
            RCLCPP_INFO(this->get_logger(), "Mode: TCP CLIENT (%s:%d)", config.tcp_ip, config.tcp_port);
        } else if (mode == "tcp_server") {
            config.type = CONN_TYPE_TCP_SERVER;
            config.tcp_port = this->get_parameter("tcp.port").as_int();
            strcpy(config.tcp_ip, "0.0.0.0");
            RCLCPP_INFO(this->get_logger(), "Mode: TCP SERVER (Listening on port %d)", config.tcp_port);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown mode: %s", mode.c_str());
            throw std::runtime_error("Invalid mode parameter");
        }

        device_ = P2_Open(&config);
        if (!device_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open driver.");
            throw std::runtime_error("Driver initialization failed");
        }

        // 설정값 로드
        frame_id_ = this->get_parameter("frame_id").as_string();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
        use_enu_ = this->get_parameter("use_enu").as_bool();
        publish_raw_imu_ = this->get_parameter("publish_raw_imu").as_bool();
        double rate = this->get_parameter("publish_rate").as_double();

        // ---------------------------------------------------------
        // 3. 퍼블리셔 생성
        // ---------------------------------------------------------
        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vel", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
		// 하드웨어 상태 퍼블리셔 (STATUS ID 2393)
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/pva/hardware_status", 10);
        
        // [Debug] 상세 상태 모니터링 (v4 기능 완벽 이식)
        debug_pub_ = this->create_publisher<std_msgs::msg::String>("/pva/status_debug", 10);
        
        // [NMEA] NTRIP Client 연동용 (GGA 전송)
        nmea_pub_ = this->create_publisher<std_msgs::msg::String>("/nmea", 10); 
        
        if (publish_raw_imu_) {
            raw_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/raw", 10);
			P2_SetIMUCallback(device_, c_imu_callback_wrapper, this);
        }

        // ---------------------------------------------------------
        // 4. 서브스크라이버 생성
        // ---------------------------------------------------------
        // [RTK] 외부(NTRIP)에서 RTCM 데이터를 받아 장비로 전송
        rtcm_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "/rtcm", 10, 
            [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
                if (device_) {
                    // PIMTP Type 2 패킷으로 포장하여 전송
                    P2_SendRTCM(device_, msg->data.data(), msg->data.size());
                }
            });
			
		// 원격 리셋 서비스 서버 생성
        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/isro_p2/reset",
            std::bind(&ISRO_P2_DriverNode::reset_service_callback, this, std::placeholders::_1, std::placeholders::_2)
        );	

        // ---------------------------------------------------------
        // 5. 타이머 설정
        // ---------------------------------------------------------
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&ISRO_P2_DriverNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "ISRO P2 Driver Running. RTK Ready (/rtcm -> Device).");
    }

    ~ISRO_P2_DriverNode() {
        if (device_) {
            P2_Close(device_);
            RCLCPP_INFO(this->get_logger(), "Driver closed");
        }
    }
	//  C언어 드라이버에서 호출할 정적 래퍼 함수
    static void c_imu_callback_wrapper(const IMU_MESSAGE_T* imu, void* user_data) {
        ISRO_P2_DriverNode* node = (ISRO_P2_DriverNode*)user_data;
        if (node) {
            node->imu_data_callback(imu);
        }
    }

    //  실제 IMU 데이터 처리 멤버 함수
    void imu_data_callback(const IMU_MESSAGE_T* imu) {
        // 데이터 도착 즉시 현재 시간으로 발행 (또는 imu->gps_second 활용 가능)
        auto now = this->get_clock()->now();
        publishIMUData(*imu, now);
    }
	
	//  리셋 서비스 콜백 함수
    void reset_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // 사용 안 함
        if (device_) {
            if (P2_SendReset(device_) == 0) {
                response->success = true;
                response->message = "Reset command sent successfully.";
                RCLCPP_WARN(this->get_logger(), "Reset command sent to device!");
            } else {
                response->success = false;
                response->message = "Failed to send reset command (Connection issue?).";
                RCLCPP_ERROR(this->get_logger(), "Failed to send reset command.");
            }
        } else {
            response->success = false;
            response->message = "Driver not initialized.";
        }
    }
	

private:
    // [Time Sync] GPS Week/MS를 이용해 정밀 ROS Timestamp 생성
    rclcpp::Time calculateGPSTime(uint16_t week, uint32_t ms) {
        // GPS Epoch (1980-01-06) ~ Unix Epoch (1970-01-01) 차이: 315964800초
        const uint64_t GPS_TO_UNIX_OFFSET = 315964800; 
        // 현재 윤초 (Leap Seconds): 18초 (2017년 이후)
        const uint64_t LEAP_SECONDS = 18;             
        
        uint64_t gps_seconds = (uint64_t)week * 604800 + (ms / 1000);
        uint64_t gps_nanos = (ms % 1000) * 1000000;
        
        // Unix Time = GPS Time + Offset - Leap Seconds
        uint64_t unix_seconds = gps_seconds + GPS_TO_UNIX_OFFSET - LEAP_SECONDS;
        
        return rclcpp::Time(unix_seconds, gps_nanos);
    }

    void timer_callback() {
        if (!device_) return;
        auto now = this->get_clock()->now();

        // 1. PVA (Position, Velocity, Attitude) 처리
        PVA_MESSAGE_T pva;
        if (P2_GetPVA(device_, &pva) == 0) {
            // [중복 방지] 새로운 데이터가 들어왔을 때만 발행
            if (pva.header_gps_ms != last_gps_ms_) {
                last_gps_ms_ = pva.header_gps_ms; // 업데이트

                rclcpp::Time msg_time = now;
                // GPS Time Sync
                if (pva.header_gps_week > 0) {
                    msg_time = calculateGPSTime(pva.header_gps_week, pva.header_gps_ms);
                }
                publishPVAData(pva, msg_time);
                
                // Debug
                auto debug_msg = std_msgs::msg::String();
                debug_msg.data = generateFullDebugString(pva);
                debug_pub_->publish(debug_msg);
            }
        }

        // 2. NMEA (GGA) 처리 -> NTRIP 전송용
        char nmea_buf[1024];
        if (P2_GetNMEA(device_, nmea_buf, sizeof(nmea_buf)) == 0) {
            auto nmea_msg = std_msgs::msg::String();
            nmea_msg.data = std::string(nmea_buf);
            nmea_pub_->publish(nmea_msg);
        }

        // 3. IMU (Raw Sensor) 이제 콜백이 처리함.
        //if (publish_raw_imu_) {
        //    IMU_MESSAGE_T imu;
        //    if (P2_GetIMU(device_, &imu) == 0) {
        //        // IMU 시간 동기화는 PVA와 동일하거나, 수신 시간 사용
        //        publishIMUData(imu, now); 
        //    }
        //}
		
		//  하드웨어 상태(STATUS) 처리
        STATUS_MESSAGE_T status;
        if (P2_GetStatus(device_, &status) == 0) {
            auto status_msg = std_msgs::msg::String();
            std::stringstream ss;
            ss << "=== System Health ===\n";
            ss << "Temp: " << status.temperature << " C\n";
            ss << "Voltages: [3.3V: " << status.voltage_3v3 << "V] [Input: " << status.voltage_3v0 << "V]\n"; // 3v0 is usually input rail
            ss << "CPU Idle: " << (int)status.rx_idle_time * 0.5 << " %\n";
            ss << "Error Word: 0x" << std::hex << status.error_word << std::dec;
            // Error word가 0이 아니면 경고 표시
            if (status.error_word != 0) ss << " (WARNING!)";
            
            status_msg.data = ss.str();
            status_pub_->publish(status_msg);
        }
		
    }

    // Helper: Position Type to String
    std::string getPosTypeString(uint32_t type) {
        switch(type) {
            case 0: return "NONE";
            case 16: return "SINGLE";
            case 17: return "PSRDIFF";
            case 19: return "PROPAGATED";
            case 32: return "L1_FLOAT";
            case 34: return "NARROW_FLOAT";
            case 48: return "L1_INT";
            case 49: return "WIDE_INT";
            case 50: return "NARROW_INT"; 
            case 51: return "RTK_DIRECT_INS";
            case 53: return "INS_PSRSP";
            case 55: return "INS_RTKFLOAT";
            case 56: return "INS_RTKFIXED";
            default: return "UNKNOWN(" + std::to_string(type) + ")";
        }
    }
    
    // Helper: INS Status to String
    std::string getInsStatusString(uint32_t status) {
        switch(status) {
            case 0: return "INACTIVE";
            case 1: return "ALIGNING";
            case 2: return "HIGH_VARIANCE";
            case 3: return "SOLUTION_GOOD";
            case 6: return "SOLUTION_FREE";
            case 7: return "ALIGNMENT_COMPLETE";
            case 8: return "DETERMINING_ORIENTATION";
            case 9: return "WAITING_INITIAL_POS";
            case 10: return "WAITING_AZIMUTH";
            case 11: return "INITIALIZING_BIASES";
            case 12: return "MOTION_DETECT";
            default: return "UNKNOWN(" + std::to_string(status) + ")";
        }
    }

    // [Full Logic Restoration] v4 코드의 모든 비트 체크 로직 복원
    std::string generateFullDebugString(const PVA_MESSAGE_T& pva) {
        std::stringstream ss;
        ss.precision(9);

        ss << "========== PVA FULL DEBUG INFO ==========\n";
        ss << "[Time] GPS Week: " << pva.header_gps_week << ", MS: " << pva.header_gps_ms << "\n";
        ss << "[Position] Lat: " << std::fixed << pva.latitude 
           << ", Lon: " << pva.longitude 
           << ", Alt: " << std::setprecision(3) << pva.height << " m\n";
        ss << "[Velocity] Vx: " << pva.velocity_x 
           << ", Vy: " << pva.velocity_y 
           << ", Vz: " << pva.velocity_z << " m/s\n";
        ss << "[Attitude] Roll: " << pva.attitude_roll 
           << ", Pitch: " << pva.attitude_pitch 
           << ", Azimuth: " << pva.attitude_azimuth << " deg\n";

        ss << "-----------------------------------------\n";
        ss << "[Status]   Pos Type: " << pva.position_type << " (" << getPosTypeString(pva.position_type) << ")\n";
        ss << "           INS Status: " << pva.ins_status << " (" << getInsStatusString(pva.ins_status) << ")\n";
        
        uint32_t ext = pva.extended_solution_status;
        ss << "[Extended] Hex Code: 0x" << std::hex << std::uppercase << ext << std::dec << "\n";

        ss << "-----------------------------------------\n";
        // Table 49: INS Extended Solution Status 비트 전체 체크
        if (ext & 0x00000001) ss << "  [O] Position Update Used\n";
        if (ext & 0x00000002) ss << "  [O] Phase Update Used\n";
        if (ext & 0x00000004) ss << "  [O] Zero Velocity Update Used\n";
        if (ext & 0x00000008) ss << "  [O] Wheel Sensor Update Used\n";
        if (ext & 0x00000010) ss << "  [O] ALIGN(Heading) Update Used\n";
        if (ext & 0x00000020) ss << "  [O] External POS Update Used\n";
        if (ext & 0x00000040) ss << "  [O] INS Solution Converged (GOOD)\n";
        if (ext & 0x00000080) ss << "  [O] Doppler Update Used\n";
        if (ext & 0x00000100) ss << "  [O] Pseudorange Update Used\n";
        if (ext & 0x00000200) ss << "  [O] Velocity Update Used\n";
        if (ext & 0x00000800) ss << "  [O] Dead Reckoning Update Used\n";
        if (ext & 0x00001000) ss << "  [O] Phase Wind Up Update Used\n";
        if (ext & 0x00002000) ss << "  [O] Course Over Ground Update Used\n";
        if (ext & 0x00004000) ss << "  [O] External Velocity Update Used\n";
        if (ext & 0x00008000) ss << "  [O] External Attitude Update Used\n";
        if (ext & 0x00010000) ss << "  [O] External Heading Update Used\n";
        if (ext & 0x00020000) ss << "  [O] External Height Update Used\n";
        if (ext & 0x00100000) ss << "  [O] Rover Position Update Used\n";
        if (ext & 0x00200000) ss << "  [O] Rover Position Update Type (RTK Int)\n";
        if (ext & 0x01000000) ss << "  [O] Static Turn-on Biases Estimated\n";
        if (ext & 0x02000000) ss << "  [O] Alignment Direction Verified\n";

        uint32_t align_ind = (ext & 0x1C000000) >> 26;
        ss << "  [*] Alignment State: ";
        switch(align_ind) {
            case 0x0: ss << "Incomplete Alignment"; break;
            case 0x1: ss << "Static"; break;
            case 0x2: ss << "Kinematic"; break;
            case 0x3: ss << "Dual Antenna"; break;
            case 0x4: ss << "User Command"; break;
            case 0x5: ss << "Reserved"; break;
            default:  ss << "Unknown (" << align_ind << ")"; break;
        }
        ss << "\n=========================================";

        return ss.str();
    }

    void publishPVAData(const PVA_MESSAGE_T& pva, const rclcpp::Time& timestamp) {
        auto fix_msg = sensor_msgs::msg::NavSatFix();
        fix_msg.header.stamp = timestamp;
        fix_msg.header.frame_id = frame_id_;
        fix_msg.latitude = pva.latitude;
        fix_msg.longitude = pva.longitude;
        fix_msg.altitude = pva.height;
        
        // Status Mapping
        if (pva.position_type == 50 || pva.position_type == 56) {
            fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else if (pva.position_type == 34) {
            fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        } else if (pva.position_type == 16 || pva.position_type == 17) {
            fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        } else {
            fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        
        fix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS | sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
        fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        fix_msg.position_covariance[0] = pva.lat_std_dev * pva.lat_std_dev;    
        fix_msg.position_covariance[4] = pva.lon_std_dev * pva.lon_std_dev;    
        fix_msg.position_covariance[8] = pva.height_std_dev * pva.height_std_dev; 
        fix_pub_->publish(fix_msg);

        // Velocity Mapping
        auto vel_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
        vel_msg.header.stamp = timestamp;
        vel_msg.header.frame_id = frame_id_;
        
        // ENU vs NED
        if (use_enu_) {
            vel_msg.twist.twist.linear.x = pva.velocity_y; // North -> Y -> East -> X ?? (ENU: X=East, Y=North, Z=Up)
            // Caution: NED (North, East, Down) -> ENU (East, North, Up)
            // pva.velocity_x = North, pva.velocity_y = East (Based on NovAtel spec)
            // Correct mapping:
            vel_msg.twist.twist.linear.x = pva.velocity_y; // East
            vel_msg.twist.twist.linear.y = pva.velocity_x; // North
            vel_msg.twist.twist.linear.z = pva.velocity_z; // Up (NovAtel gives Up velocity directly in SPAN)
        } else {
            // NED Mapping
            vel_msg.twist.twist.linear.x = pva.velocity_x;
            vel_msg.twist.twist.linear.y = pva.velocity_y;
            vel_msg.twist.twist.linear.z = -pva.velocity_z; // Down -> Up conversion
        }
        
        vel_msg.twist.covariance[0] = pva.vel_x_std_dev * pva.vel_x_std_dev;
        vel_msg.twist.covariance[7] = pva.vel_y_std_dev * pva.vel_y_std_dev;
        vel_msg.twist.covariance[14] = pva.vel_z_std_dev * pva.vel_z_std_dev;
        vel_msg.twist.covariance[21] = 0.01; // Rotation rate cov (unknown)
        vel_msg.twist.covariance[28] = 0.01;
        vel_msg.twist.covariance[35] = 0.01;
        vel_pub_->publish(vel_msg);

        // Attitude Mapping
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = timestamp;
        imu_msg.header.frame_id = imu_frame_id_;
        
        tf2::Quaternion q;
        double roll = pva.attitude_roll * M_PI / 180.0;
        double pitch = pva.attitude_pitch * M_PI / 180.0;
        double yaw = pva.attitude_azimuth * M_PI / 180.0;
        
        if (use_enu_) {
            // NovAtel Azimuth: North = 0, Clockwise
            // ROS Yaw: East = 0, Counter-Clockwise
            // Conversion: Yaw = 90 - Azimuth
            yaw = (90.0 - pva.attitude_azimuth) * M_PI / 180.0;
            // Normalize
            while (yaw > M_PI) yaw -= 2*M_PI;
            while (yaw < -M_PI) yaw += 2*M_PI;
            
            q.setRPY(roll, -pitch, yaw); // Check pitch sign carefully for ENU
        } else {
            q.setRPY(roll, pitch, yaw);
        }
        
        imu_msg.orientation = tf2::toMsg(q);
        double roll_var = pva.roll_std_dev * pva.roll_std_dev * (M_PI/180.0) * (M_PI/180.0);
        double pitch_var = pva.pitch_std_dev * pva.pitch_std_dev * (M_PI/180.0) * (M_PI/180.0);
        double yaw_var = pva.azimuth_std_dev * pva.azimuth_std_dev * (M_PI/180.0) * (M_PI/180.0);
        
        imu_msg.orientation_covariance[0] = roll_var;
        imu_msg.orientation_covariance[4] = pitch_var;
        imu_msg.orientation_covariance[8] = yaw_var;
        
        // PVA message doesn't have rates/accels
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;
        
        // Mark rates as unknown/invalid covariance
        for (int i = 0; i < 9; i++) {
            imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 1.0 : 0.0;
            imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 1.0 : 0.0;
        }
        
        imu_pub_->publish(imu_msg);
        
        // Simple Terminal Log
        if (++log_counter_ % 100 == 0) {  
            RCLCPP_INFO(this->get_logger(), 
                "PVA: Lat=%.8f, Lon=%.8f, Alt=%.2f, Type=%d, INS=%d",
                pva.latitude, pva.longitude, pva.height, 
                pva.position_type, pva.ins_status);
        }
    }
	
    // [수정됨] IMU_MESSAGE_T를 const reference로 받도록 수정
    void publishIMUData(const IMU_MESSAGE_T& imu, const rclcpp::Time& timestamp) {
        auto raw_imu_msg = sensor_msgs::msg::Imu();
        raw_imu_msg.header.stamp = timestamp;
        raw_imu_msg.header.frame_id = imu_frame_id_;
        
        const double accel_scale = 1.0e-9;
        const double gyro_scale = 1.0e-10;
        double rate_factor = 100.0; // Assuming 100Hz IMU
        
        if (use_enu_) {
            raw_imu_msg.linear_acceleration.x = imu.scaled_accel_x * accel_scale * rate_factor;
            raw_imu_msg.linear_acceleration.y = imu.scaled_accel_y * accel_scale * rate_factor;
            raw_imu_msg.linear_acceleration.z = imu.scaled_accel_z * accel_scale * rate_factor;
            
            raw_imu_msg.angular_velocity.x = imu.scaled_gyro_x * gyro_scale * rate_factor;
            raw_imu_msg.angular_velocity.y = imu.scaled_gyro_y * gyro_scale * rate_factor;
            raw_imu_msg.angular_velocity.z = imu.scaled_gyro_z * gyro_scale * rate_factor;
        } else {
            raw_imu_msg.linear_acceleration.x = imu.scaled_accel_x * accel_scale * rate_factor;
            raw_imu_msg.linear_acceleration.y = imu.scaled_accel_y * accel_scale * rate_factor;
            raw_imu_msg.linear_acceleration.z = -imu.scaled_accel_z * accel_scale * rate_factor; // Z down to up
            
            raw_imu_msg.angular_velocity.x = imu.scaled_gyro_x * gyro_scale * rate_factor;
            raw_imu_msg.angular_velocity.y = imu.scaled_gyro_y * gyro_scale * rate_factor;
            raw_imu_msg.angular_velocity.z = -imu.scaled_gyro_z * gyro_scale * rate_factor;
        }
        
        bool imu_corrected = (imu.imu_status_mask & 0x02) != 0;
        double accel_variance = imu_corrected ? 0.01 : 0.1;
        double gyro_variance = imu_corrected ? 0.001 : 0.01;
        
        for (int i = 0; i < 9; i++) {
            raw_imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? accel_variance : 0.0;
            raw_imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? gyro_variance : 0.0;
        }
        
        // Orientation unknown in raw msg
        raw_imu_msg.orientation_covariance[0] = -1;
        
        raw_imu_pub_->publish(raw_imu_msg);
        
        if (++imu_log_counter_ % 1000 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "IMU: GPS_Sec=%.3f, Status=0x%02X (Corrected=%d)",
                imu.gps_second, imu.imu_status_mask, imu_corrected);
        }
    }

    // --- Member Variables ---
    ISRO_P2_T* device_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmea_pub_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr rtcm_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    std::string frame_id_;
    std::string imu_frame_id_;
    bool use_enu_ = true;
    bool publish_raw_imu_ = false;
    uint32_t log_counter_ = 0;
    uint32_t imu_log_counter_ = 0;
    uint32_t no_pva_counter_ = 0;
	uint32_t last_gps_ms_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ISRO_P2_DriverNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}