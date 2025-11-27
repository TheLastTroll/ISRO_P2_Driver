#include "ISRO_P2_Driver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// --- Constants ---
#define UART_BUFFER_SIZE    4096
#define RING_BUFFER_SIZE    131072  
#define MAX_PACKET_SIZE     4096
#define PVA_PAYLOAD_SIZE    126    // Table 46의 순수 데이터 크기
#define PACKET_TIMEOUT_MS   1000

#define PIMTP_SYNC1         0xAC
#define PIMTP_SYNC2         0x55
#define PIMTP_SYNC3         0x96
#define PIMTP_SYNC4         0x83
#define CRC32_POLYNOMIAL    0xEDB88320L

// --- Internal Enums ---
typedef enum {
    PIMTP_PAYLOAD_AUTOMOTIVE = 1,
    PIMTP_PAYLOAD_RTK_CORRECTIONS = 2,
    PIMTP_PAYLOAD_UPDATE_DATA = 3,
    PIMTP_PAYLOAD_NMEA = 4
} PIMTP_PAYLOAD_TYPE_E;

typedef enum {
    MSG_ID_PVA = 2379,
    MSG_ID_CONFIGFILE = 2388,
    MSG_ID_IMU = 2389,
    MSG_ID_STATUS = 2393,
    MSG_ID_EVENT = 2394,
    MSG_ID_PASSTHROUGH = 2396,
    MSG_ID_VERSION = 2409,
    MSG_ID_RESET = 2410
} AUTOMOTIVE_MSG_ID_E;

// --- Internal Structures ---
typedef struct __attribute__((packed)) {
    uint8_t  sync[4];
    uint16_t payload_type;
    uint16_t reserved;
    uint32_t payload_length;
} PIMTP_HEADER_T;

typedef struct __attribute__((packed)) {
    uint16_t message_id;
    uint16_t message_data_size;
    uint16_t time_status;
    uint16_t gps_week;
    uint32_t gps_millisecond;
    uint32_t reserved;
} AUTOMOTIVE_HEADER_T;

typedef struct {
    uint8_t* buffer;
    uint32_t size;
    uint32_t write_pos;
    uint32_t read_pos;
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
} RING_BUFFER_T;

typedef enum {
    STATE_SEARCH_SYNC,
    STATE_READ_PIMTP_HEADER,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC,
    STATE_PROCESS_NMEA
} PARSER_STATE_E;

typedef struct {
    PARSER_STATE_E state;
    uint8_t header_buffer[12];
    uint32_t header_index;
    uint16_t payload_type;
    uint32_t payload_length;
    uint8_t* payload_buffer;
    uint32_t payload_index;
    uint8_t crc_buffer[4];
    uint32_t crc_index;
    uint8_t nmea_buffer[1024];
    uint32_t nmea_index;
    struct timespec last_byte_time;
} PARSER_CONTEXT_T;

// Main Device Structure
struct ISRO_P2_T {
    P2_Config_T config;
    int main_fd;
    int client_fd; 
    
    pthread_t read_thread;
    pthread_t process_thread;
    RING_BUFFER_T* ring_buffer;
    bool running;
    pthread_mutex_t data_mutex;
    
    PVA_MESSAGE_T latest_pva;
    bool pva_valid;
    struct timespec pva_timestamp;
    
    IMU_MESSAGE_T latest_imu;
    bool imu_valid;
    struct timespec imu_timestamp;
    IMU_Callback imu_callback;
    void* imu_user_data;

    char latest_nmea[1024];
    bool nmea_valid;
};

// --- Utilities ---
static inline uint32_t ReadLE32(const uint8_t* buf) {
    return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
}

static uint32_t crc32_table[256];
static int crc32_table_initialized = 0;

static void InitNovAtelCRC32Table(void) {
    if (crc32_table_initialized) return;
    for (int i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 8; j > 0; j--) {
            if (crc & 1) crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            else crc >>= 1;
        }
        crc32_table[i] = crc;
    }
    crc32_table_initialized = 1;
}

static uint32_t CalculateNovAtelCRC32(const uint8_t* data, uint32_t len) {
    InitNovAtelCRC32Table();
    uint32_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t temp1 = (crc >> 8) & 0x00FFFFFFL;
        uint32_t temp2 = crc32_table[((int)crc ^ data[i]) & 0xff];
        crc = temp1 ^ temp2;
    }
    return crc;
}

static void ProcessNMEASentence(struct ISRO_P2_T* device, const uint8_t* sentence, uint32_t len);

// --- Ring Buffer Implementation ---
static RING_BUFFER_T* RingBuffer_Create(uint32_t size) {
    RING_BUFFER_T* rb = (RING_BUFFER_T*)calloc(1, sizeof(RING_BUFFER_T));
    if (!rb) return NULL;
    rb->buffer = (uint8_t*)malloc(size);
    if (!rb->buffer) { free(rb); return NULL; }
    rb->size = size;
    pthread_mutex_init(&rb->mutex, NULL);
    pthread_cond_init(&rb->not_empty, NULL);
    pthread_cond_init(&rb->not_full, NULL);
    return rb;
}

static void RingBuffer_Destroy(RING_BUFFER_T* rb) {
    if (!rb) return;
    pthread_mutex_destroy(&rb->mutex);
    pthread_cond_destroy(&rb->not_empty);
    pthread_cond_destroy(&rb->not_full);
    free(rb->buffer);
    free(rb);
}

static int RingBuffer_Write(RING_BUFFER_T* rb, const uint8_t* data, uint32_t len) {
    pthread_mutex_lock(&rb->mutex);
    uint32_t available = (rb->size + rb->read_pos - rb->write_pos - 1) % rb->size;
    if (available < len) rb->read_pos = (rb->write_pos + len + 1) % rb->size; // Overwrite oldest
    for (uint32_t i = 0; i < len; i++) {
        rb->buffer[rb->write_pos] = data[i];
        rb->write_pos = (rb->write_pos + 1) % rb->size;
    }
    pthread_cond_signal(&rb->not_empty);
    pthread_mutex_unlock(&rb->mutex);
    return len;
}

static int RingBuffer_Read(RING_BUFFER_T* rb, uint8_t* data, uint32_t max_len) {
    pthread_mutex_lock(&rb->mutex);
    while (rb->read_pos == rb->write_pos) {
        pthread_cond_wait(&rb->not_empty, &rb->mutex);
    }
    uint32_t available = (rb->size + rb->write_pos - rb->read_pos) % rb->size;
    uint32_t to_read = (available < max_len) ? available : max_len;
    for (uint32_t i = 0; i < to_read; i++) {
        data[i] = rb->buffer[rb->read_pos];
        rb->read_pos = (rb->read_pos + 1) % rb->size;
    }
    pthread_cond_signal(&rb->not_full);
    pthread_mutex_unlock(&rb->mutex);
    return to_read;
}

// --- Message Processors ---
static void ProcessPVAMessage(struct ISRO_P2_T* device, const uint8_t* data, uint32_t len, uint16_t week, uint32_t ms) {
    if (len < PVA_PAYLOAD_SIZE) {
        // printf("[WARN] PVA Payload too short: %d < %d\n", len, PVA_PAYLOAD_SIZE);
        return; 
    }
    
    PVA_MESSAGE_T pva = {0};
    memcpy(&pva, data, PVA_PAYLOAD_SIZE); 
    pva.header_gps_week = week;
    pva.header_gps_ms = ms;

    pthread_mutex_lock(&device->data_mutex);
    memcpy(&device->latest_pva, &pva, sizeof(PVA_MESSAGE_T));
    device->pva_valid = true;
    clock_gettime(CLOCK_REALTIME, &device->pva_timestamp);
    pthread_mutex_unlock(&device->data_mutex);
}

static void ProcessIMUMessage(struct ISRO_P2_T* device, const uint8_t* data, uint32_t len) {
    if (len < sizeof(IMU_MESSAGE_T)) return;
    IMU_MESSAGE_T* imu = (IMU_MESSAGE_T*)data;
    
    pthread_mutex_lock(&device->data_mutex);
    memcpy(&device->latest_imu, imu, sizeof(IMU_MESSAGE_T));
    device->imu_valid = true;
    clock_gettime(CLOCK_REALTIME, &device->imu_timestamp);
    if (device->imu_callback) device->imu_callback(imu, device->imu_user_data);
    pthread_mutex_unlock(&device->data_mutex);
}

static void ProcessAutomotiveMessage(struct ISRO_P2_T* device, const uint8_t* payload, uint32_t len) {
    if (len < sizeof(AUTOMOTIVE_HEADER_T)) return;
    AUTOMOTIVE_HEADER_T* header = (AUTOMOTIVE_HEADER_T*)payload;
    const uint8_t* msg_data = payload + sizeof(AUTOMOTIVE_HEADER_T);
    
    uint16_t week = header->gps_week;
    uint32_t ms = header->gps_millisecond;

    switch(header->message_id) {
        case MSG_ID_PVA:
            ProcessPVAMessage(device, msg_data, header->message_data_size, week, ms);
            break;
        case MSG_ID_IMU:
            ProcessIMUMessage(device, msg_data, header->message_data_size);
            break;
        case MSG_ID_STATUS:
            // 추후 구현: 장비 상태(전압, 온도 등) 처리
            // printf("[INFO] STATUS Message Received (Size: %d)\n", header->message_data_size);
            break;
        case MSG_ID_VERSION:
            // 추후 구현: 버전 정보 처리
            break;
        default:
            // printf("[INFO] Unknown AutoMsg ID: %d\n", header->message_id);
            break;
    }
}

static void ProcessNMEASentence(struct ISRO_P2_T* device, const uint8_t* sentence, uint32_t len) {
    pthread_mutex_lock(&device->data_mutex);
    uint32_t copy_len = (len < sizeof(device->latest_nmea) - 1) ? len : sizeof(device->latest_nmea) - 1;
    memcpy(device->latest_nmea, sentence, copy_len);
    device->latest_nmea[copy_len] = '\0';
    
    // Remove CR/LF
    char* p = device->latest_nmea;
    while (*p) {
        if (*p == '\r' || *p == '\n') *p = '\0';
        p++;
    }
    device->nmea_valid = true;
    pthread_mutex_unlock(&device->data_mutex);
}

static void ProcessPIMTPPacket(struct ISRO_P2_T* device, PARSER_CONTEXT_T* ctx) {
    switch (ctx->payload_type) {
        case PIMTP_PAYLOAD_AUTOMOTIVE: // Type 1
            ProcessAutomotiveMessage(device, ctx->payload_buffer, ctx->payload_length);
            break;
        case PIMTP_PAYLOAD_NMEA:       // Type 4
			// [디버깅용] NMEA 패킷 수신 확인
            //printf("[DEBUG] NMEA Packet Received inside PIMTP (Len: %d)\n", ctx->payload_length);		
            ProcessNMEASentence(device, ctx->payload_buffer, ctx->payload_length);
            break;
        case PIMTP_PAYLOAD_RTK_CORRECTIONS: // Type 2 (Input Only usually)
            // printf("[INFO] RTK Corrections Loopback Received?\n");
            break;
        case PIMTP_PAYLOAD_UPDATE_DATA:     // Type 3
            // Firmware update packet - ignore for driver
            break;
        default:
            // printf("[WARN] Unknown Payload Type: %d\n", ctx->payload_type);
            break;
    }
}

static void ParseByte(struct ISRO_P2_T* device, PARSER_CONTEXT_T* ctx, uint8_t byte) {
    switch (ctx->state) {
        case STATE_SEARCH_SYNC:
            if (byte == '$') { // Start of Raw NMEA
                ctx->nmea_buffer[0] = byte;
                ctx->nmea_index = 1;
                ctx->state = STATE_PROCESS_NMEA;
            }
            else if (byte == PIMTP_SYNC1) { // Start of PIMTP
                ctx->header_buffer[0] = byte;
                ctx->header_index = 1;
                ctx->state = STATE_READ_PIMTP_HEADER;
            }
            break;

        case STATE_READ_PIMTP_HEADER:
            ctx->header_buffer[ctx->header_index++] = byte;
            
            // Check Sync Bytes
            if (ctx->header_index == 2 && byte != PIMTP_SYNC2) ctx->state = STATE_SEARCH_SYNC;
            else if (ctx->header_index == 3 && byte != PIMTP_SYNC3) ctx->state = STATE_SEARCH_SYNC;
            else if (ctx->header_index == 4 && byte != PIMTP_SYNC4) ctx->state = STATE_SEARCH_SYNC;
            else if (ctx->header_index == 12) {
                PIMTP_HEADER_T* header = (PIMTP_HEADER_T*)ctx->header_buffer;
                ctx->payload_type = header->payload_type;
                ctx->payload_length = header->payload_length;
                
                if (ctx->payload_length > 0 && ctx->payload_length < MAX_PACKET_SIZE) {
                    ctx->payload_buffer = (uint8_t*)malloc(ctx->payload_length);
                    if (ctx->payload_buffer) {
                        ctx->payload_index = 0;
                        ctx->state = STATE_READ_PAYLOAD;
                    } else {
                        // Malloc Fail
                        ctx->state = STATE_SEARCH_SYNC;
                    }
                } else {
                    // Invalid Length
                    ctx->state = STATE_SEARCH_SYNC;
                }
            }
            break;

        case STATE_READ_PAYLOAD:
            ctx->payload_buffer[ctx->payload_index++] = byte;
            if (ctx->payload_index >= ctx->payload_length) {
                ctx->crc_index = 0;
                ctx->state = STATE_READ_CRC;
            }
            break;

        case STATE_READ_CRC:
            ctx->crc_buffer[ctx->crc_index++] = byte;
            if (ctx->crc_index >= 4) {
                uint32_t received_crc = ReadLE32(ctx->crc_buffer);
                
                uint8_t* verify_buf = (uint8_t*)malloc(12 + ctx->payload_length);
                if (verify_buf) {
                    memcpy(verify_buf, ctx->header_buffer, 12);
                    memcpy(verify_buf + 12, ctx->payload_buffer, ctx->payload_length);
                    
                    uint32_t calc_crc = CalculateNovAtelCRC32(verify_buf, 12 + ctx->payload_length);
                    free(verify_buf);

                    // [수정됨] Type 4 (NMEA)는 CRC가 틀려도 강제로 통과시킴 (일단 데이터를 보기 위함)
                    if (received_crc == calc_crc || ctx->payload_type == 4) {
                        
                        // 만약 CRC가 틀렸는데 Type 4라서 들어온 경우 경고 로그 출력
                        if (received_crc != calc_crc) {
                            printf("[WARN] CRC Failed but Forcing NMEA! Type: %d, Recv: 0x%08X vs Calc: 0x%08X\n", 
                                   ctx->payload_type, received_crc, calc_crc);
                        } else {
                            // 정상 통과 시 디버그 로그 (너무 많으면 주석 처리)
                            //printf("[DEBUG] Packet Valid! Type: %d, Len: %d\n", ctx->payload_type, ctx->payload_length);
                        }

                        ProcessPIMTPPacket(device, ctx);

                    } else {
                        // 진짜 에러 (Type 1 등이 깨진 경우)
                        printf("[ERROR] CRC Mismatch & Dropped! Type: %d, Len: %d, Recv: 0x%08X vs Calc: 0x%08X\n", 
                               ctx->payload_type, ctx->payload_length, received_crc, calc_crc);
                    }
                }
                
                if (ctx->payload_buffer) {
                    free(ctx->payload_buffer);
                    ctx->payload_buffer = NULL;
                }
                ctx->state = STATE_SEARCH_SYNC;
            }
            break;

        case STATE_PROCESS_NMEA:
            ctx->nmea_buffer[ctx->nmea_index++] = byte;
            if (byte == '\n' || ctx->nmea_index >= sizeof(ctx->nmea_buffer)-1) {
                ProcessNMEASentence(device, ctx->nmea_buffer, ctx->nmea_index);
                ctx->state = STATE_SEARCH_SYNC;
            }
            break;
    }
}

// --- Threads ---
static void* ProcessThread(void* arg) {
    struct ISRO_P2_T* device = (struct ISRO_P2_T*)arg;
    uint8_t buffer[1024];
    PARSER_CONTEXT_T ctx = {0};
    
    // printf("[Driver] Process Thread Started.\n");
    
    while (device->running) {
        int bytes = RingBuffer_Read(device->ring_buffer, buffer, sizeof(buffer));
        if (bytes > 0) {
            for(int i=0; i<bytes; i++) ParseByte(device, &ctx, buffer[i]);
        }
    }
    if(ctx.payload_buffer) free(ctx.payload_buffer);
    return NULL;
}

static void* ReadThread(void* arg) {
    struct ISRO_P2_T* device = (struct ISRO_P2_T*)arg;
    uint8_t buffer[4096];
    
    // printf("[Driver] Read Thread Started.\n");

    while (device->running) {
        int target_fd = -1;

        // TCP Connection Management
        if (device->config.type == CONN_TYPE_TCP_SERVER) {
            if (device->client_fd < 0) {
                // Accept logic
                struct sockaddr_in cli_addr;
                socklen_t clilen = sizeof(cli_addr);
                fd_set readfds;
                struct timeval tv = {1, 0};
                FD_ZERO(&readfds);
                FD_SET(device->main_fd, &readfds);
                
                int ret = select(device->main_fd + 1, &readfds, NULL, NULL, &tv);
                if (ret > 0 && FD_ISSET(device->main_fd, &readfds)) {
                    int newsockfd = accept(device->main_fd, (struct sockaddr*)&cli_addr, &clilen);
                    if (newsockfd >= 0) device->client_fd = newsockfd;
                }
                if (device->client_fd < 0) continue;
            }
            target_fd = device->client_fd;
        } else if (device->config.type == CONN_TYPE_TCP_CLIENT) {
            if (device->main_fd < 0) {
                // Connect logic
                device->main_fd = socket(AF_INET, SOCK_STREAM, 0);
                if (device->main_fd >= 0) {
                    struct sockaddr_in serv_addr = {0};
                    serv_addr.sin_family = AF_INET;
                    serv_addr.sin_port = htons(device->config.tcp_port);
                    inet_pton(AF_INET, device->config.tcp_ip, &serv_addr.sin_addr);
                    
                    if (connect(device->main_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                         close(device->main_fd);
                         device->main_fd = -1;
                         sleep(2);
                         continue;
                    }
                } else {
                    sleep(2);
                    continue;
                }
            }
            target_fd = device->main_fd;
        } else {
            target_fd = device->main_fd; // Serial
        }

        // Reading
        if (target_fd >= 0) {
            int bytes = read(target_fd, buffer, sizeof(buffer));
            if (bytes > 0) {
                RingBuffer_Write(device->ring_buffer, buffer, bytes);
            } else if (bytes == 0) {
                // EOF Handling
                if (device->config.type == CONN_TYPE_TCP_SERVER) {
                    close(device->client_fd); device->client_fd = -1;
                } else if (device->config.type == CONN_TYPE_TCP_CLIENT) {
                    close(device->main_fd); device->main_fd = -1;
                }
            } else {
                // Error Handling
                if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                    if (device->config.type == CONN_TYPE_TCP_SERVER) {
                         close(device->client_fd); device->client_fd = -1;
                    } else if (device->config.type == CONN_TYPE_TCP_CLIENT) {
                         close(device->main_fd); device->main_fd = -1;
                    }
                }
                usleep(1000);
            }
        } else {
            usleep(10000);
        }
    }
    return NULL;
}

static int ConfigureSerial(int fd, int baudrate_param) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;
    
    speed_t speed = B921600; // default
    
    // [복구됨] Full Standard Baudrate List
    switch(baudrate_param) {
        case 4800:   speed = B4800;   break;
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 500000: speed = B500000; break;
        case 576000: speed = B576000; break;
        case 921600: speed = B921600; break;
        default:     speed = B921600; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
	
	// =====================================================
    // Raw Mode 설정
    // 이 함수가 없으면 바이너리 데이터 중 일부(0x0D, 0x11, 0x13 등)가
    // 특수 문자로 인식되어 사라지거나 변조됩니다.
    // =====================================================
    cfmakeraw(&tty);	
	    
	// 추가 세부 설정 (Raw 모드를 더 확실하게 보장)
    tty.c_cflag |= (CLOCAL | CREAD); // 수신 가능, 모뎀 제어 무시
    tty.c_cflag &= ~CSTOPB;          // 1 Stop bit
    tty.c_cflag &= ~CRTSCTS;         // 하드웨어 흐름 제어 끄기 (중요!)
    tty.c_cflag &= ~PARENB;          // Parity 없음

        // VMIN > 0 으로 설정하여 CPU 인터럽트 빈도를 줄임
    // 데이터가 최소 32바이트 쌓이거나, 0.1초(VTIME 1)가 지날 때까지 대기
    tty.c_cc[VMIN] = 32; 
    tty.c_cc[VTIME] = 1; 

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    
    // 입력/출력 버퍼 플러시 (기존 쓰레기 데이터 제거)
    tcflush(fd, TCIOFLUSH);
    
    return 0;
}

// --- API Implementation ---
ISRO_P2_T* P2_Open(const P2_Config_T* config) {
    struct ISRO_P2_T* device = (struct ISRO_P2_T*)calloc(1, sizeof(struct ISRO_P2_T));
    if (!device) return NULL;

    memcpy(&device->config, config, sizeof(P2_Config_T));
    device->main_fd = -1;
    device->client_fd = -1;

    if (config->type == CONN_TYPE_SERIAL) {
        device->main_fd = open(config->serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
        if (device->main_fd < 0) { free(device); return NULL; }
        ConfigureSerial(device->main_fd, config->serial_baud);
        fcntl(device->main_fd, F_SETFL, O_NONBLOCK);
    } 
    else if (config->type == CONN_TYPE_TCP_SERVER) {
        device->main_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (device->main_fd < 0) { free(device); return NULL; }
        int opt = 1;
        setsockopt(device->main_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in serv_addr = {0};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(config->tcp_port);
        if (bind(device->main_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            close(device->main_fd); free(device); return NULL;
        }
        if (listen(device->main_fd, 5) < 0) {
            close(device->main_fd); free(device); return NULL;
        }
    }
    
    device->ring_buffer = RingBuffer_Create(RING_BUFFER_SIZE);
    pthread_mutex_init(&device->data_mutex, NULL);
    device->running = true;
    pthread_create(&device->read_thread, NULL, ReadThread, device);
    pthread_create(&device->process_thread, NULL, ProcessThread, device);
    return device;
}

void P2_Close(ISRO_P2_T* device) {
    if (!device) return;
    device->running = false;
    if (device->main_fd >= 0 && device->config.type != CONN_TYPE_SERIAL) {
        shutdown(device->main_fd, SHUT_RDWR);
    }
    pthread_join(device->read_thread, NULL);
    pthread_join(device->process_thread, NULL);
    RingBuffer_Destroy(device->ring_buffer);
    pthread_mutex_destroy(&device->data_mutex);
    if (device->client_fd >= 0) close(device->client_fd);
    if (device->main_fd >= 0) close(device->main_fd);
    free(device);
}

int P2_GetPVA(ISRO_P2_T* device, PVA_MESSAGE_T* pva) {
    if (!device || !pva) return -1;
    pthread_mutex_lock(&device->data_mutex);
    if (!device->pva_valid) { pthread_mutex_unlock(&device->data_mutex); return -1; }
    memcpy(pva, &device->latest_pva, sizeof(PVA_MESSAGE_T));
    pthread_mutex_unlock(&device->data_mutex);
    return 0;
}

int P2_GetIMU(ISRO_P2_T* device, IMU_MESSAGE_T* imu) {
    if (!device || !imu) return -1;
    pthread_mutex_lock(&device->data_mutex);
    if (!device->imu_valid) { pthread_mutex_unlock(&device->data_mutex); return -1; }
    memcpy(imu, &device->latest_imu, sizeof(IMU_MESSAGE_T));
    pthread_mutex_unlock(&device->data_mutex);
    return 0;
}

int P2_GetNMEA(ISRO_P2_T* device, char* buffer, int buffer_len) {
    if (!device || !buffer) return -1;
    pthread_mutex_lock(&device->data_mutex);
    if (!device->nmea_valid) { pthread_mutex_unlock(&device->data_mutex); return -1; }
    strncpy(buffer, device->latest_nmea, buffer_len);
    device->nmea_valid = false;
    pthread_mutex_unlock(&device->data_mutex);
    return 0;
}

int P2_SendRTCM(ISRO_P2_T* device, const uint8_t* rtcm_data, uint32_t len) {
    if (!device || !rtcm_data || len == 0) return -1;
    int fd = -1;
    if (device->config.type == CONN_TYPE_SERIAL) fd = device->main_fd;
    else if (device->config.type == CONN_TYPE_TCP_CLIENT) fd = device->main_fd;
    else return -1;
    if (fd < 0) return -1;

    uint8_t header[12] = {0xAC, 0x55, 0x96, 0x83};
    uint16_t type = 2; // RTK_CORRECTIONS
    memcpy(&header[4], &type, 2);
    memcpy(&header[8], &len, 4);

    uint8_t* full_packet = (uint8_t*)malloc(12 + len);
    if (!full_packet) return -1;
    memcpy(full_packet, header, 12);
    memcpy(full_packet + 12, rtcm_data, len);
    uint32_t crc = CalculateNovAtelCRC32(full_packet, 12 + len);
    free(full_packet);

    write(fd, header, 12);
    write(fd, rtcm_data, len);
    write(fd, &crc, 4);
    return 0;
}

void P2_SetIMUCallback(ISRO_P2_T* device, IMU_Callback callback, void* user_data) {
    if (!device) return;
    pthread_mutex_lock(&device->data_mutex);
    device->imu_callback = callback;
    device->imu_user_data = user_data;
    pthread_mutex_unlock(&device->data_mutex);
}