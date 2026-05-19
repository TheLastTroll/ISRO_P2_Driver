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
#define PVA_PAYLOAD_SIZE    126    // Table 46 순수 데이터 크기
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

    STATUS_MESSAGE_T latest_status;
    bool status_valid;
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
// Overwrite-on-full 정책. 가장 오래된 데이터를 덮어쓰며 producer는 절대 블록되지 않음.
static RING_BUFFER_T* RingBuffer_Create(uint32_t size) {
    RING_BUFFER_T* rb = (RING_BUFFER_T*)calloc(1, sizeof(RING_BUFFER_T));
    if (!rb) return NULL;
    rb->buffer = (uint8_t*)malloc(size);
    if (!rb->buffer) { free(rb); return NULL; }
    rb->size = size;
    pthread_mutex_init(&rb->mutex, NULL);
    pthread_cond_init(&rb->not_empty, NULL);
    return rb;
}

static void RingBuffer_Destroy(RING_BUFFER_T* rb) {
    if (!rb) return;
    pthread_mutex_destroy(&rb->mutex);
    pthread_cond_destroy(&rb->not_empty);
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
    pthread_mutex_unlock(&rb->mutex);
    return to_read;
}

// --- Message Processors ---
static void ProcessPVAMessage(struct ISRO_P2_T* device, const uint8_t* data, uint32_t len, uint16_t week, uint32_t ms) {
    if (len < PVA_PAYLOAD_SIZE) return;

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

    IMU_MESSAGE_T imu;
    memcpy(&imu, data, sizeof(IMU_MESSAGE_T));

    pthread_mutex_lock(&device->data_mutex);
    memcpy(&device->latest_imu, &imu, sizeof(IMU_MESSAGE_T));
    device->imu_valid = true;
    clock_gettime(CLOCK_REALTIME, &device->imu_timestamp);
    if (device->imu_callback) device->imu_callback(&imu, device->imu_user_data);
    pthread_mutex_unlock(&device->data_mutex);
}

static void ProcessStatusMessage(struct ISRO_P2_T* device, const uint8_t* data, uint32_t len) {
    if (len < sizeof(STATUS_MESSAGE_T)) return;

    STATUS_MESSAGE_T status;
    memcpy(&status, data, sizeof(STATUS_MESSAGE_T));

    pthread_mutex_lock(&device->data_mutex);
    memcpy(&device->latest_status, &status, sizeof(STATUS_MESSAGE_T));
    device->status_valid = true;
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
            ProcessStatusMessage(device, msg_data, header->message_data_size);
            break;
        case MSG_ID_VERSION:
            // TODO: 버전 정보 처리
            break;
        default:
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
        case PIMTP_PAYLOAD_AUTOMOTIVE:
            ProcessAutomotiveMessage(device, ctx->payload_buffer, ctx->payload_length);
            break;
        case PIMTP_PAYLOAD_NMEA:
            ProcessNMEASentence(device, ctx->payload_buffer, ctx->payload_length);
            break;
        case PIMTP_PAYLOAD_RTK_CORRECTIONS:
            // 입력 전용 채널. 장비에서 루프백되는 경우는 무시.
            break;
        case PIMTP_PAYLOAD_UPDATE_DATA:
            // Firmware update 패킷은 드라이버에서 처리하지 않음.
            break;
        default:
            break;
    }
}

static void ParseByte(struct ISRO_P2_T* device, PARSER_CONTEXT_T* ctx, uint8_t byte) {
    switch (ctx->state) {
        case STATE_SEARCH_SYNC:
            if (byte == '$') { // Raw NMEA 시작
                ctx->nmea_buffer[0] = byte;
                ctx->nmea_index = 1;
                ctx->state = STATE_PROCESS_NMEA;
            }
            else if (byte == PIMTP_SYNC1) { // PIMTP 시작
                ctx->header_buffer[0] = byte;
                ctx->header_index = 1;
                ctx->state = STATE_READ_PIMTP_HEADER;
            }
            break;

        case STATE_READ_PIMTP_HEADER:
            ctx->header_buffer[ctx->header_index++] = byte;

            // Sync 바이트 확인
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
                        ctx->state = STATE_SEARCH_SYNC;
                    }
                } else {
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

                    // NMEA(Type 4)는 CRC 불일치여도 통과시키되 경고를 남김.
                    // 일부 펌웨어 버전에서 NMEA 패킷의 CRC 계산이 잘못 들어오는 케이스가 관찰되었음.
                    if (received_crc == calc_crc) {
                        ProcessPIMTPPacket(device, ctx);
                    } else if (ctx->payload_type == PIMTP_PAYLOAD_NMEA) {
                        printf("[WARN] NMEA CRC mismatch (passing through): Recv 0x%08X vs Calc 0x%08X\n",
                               received_crc, calc_crc);
                        ProcessPIMTPPacket(device, ctx);
                    } else {
                        printf("[ERROR] CRC mismatch dropped. Type: %d, Len: %d, Recv: 0x%08X vs Calc: 0x%08X\n",
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
            if (byte == '\n' || ctx->nmea_index >= sizeof(ctx->nmea_buffer) - 1) {
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

    while (device->running) {
        int bytes = RingBuffer_Read(device->ring_buffer, buffer, sizeof(buffer));
        if (bytes > 0) {
            for (int i = 0; i < bytes; i++) ParseByte(device, &ctx, buffer[i]);
        }
    }
    if (ctx.payload_buffer) free(ctx.payload_buffer);
    return NULL;
}

static void* ReadThread(void* arg) {
    struct ISRO_P2_T* device = (struct ISRO_P2_T*)arg;
    uint8_t buffer[4096];

    while (device->running) {
        int target_fd = -1;

        // TCP Connection 관리
        if (device->config.type == CONN_TYPE_TCP_SERVER) {
            if (device->client_fd < 0) {
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
                // EOF
                if (device->config.type == CONN_TYPE_TCP_SERVER) {
                    close(device->client_fd); device->client_fd = -1;
                } else if (device->config.type == CONN_TYPE_TCP_CLIENT) {
                    close(device->main_fd); device->main_fd = -1;
                }
            } else {
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

    speed_t speed = B921600;
    switch (baudrate_param) {
        case 115200: speed = B115200; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:     speed = B921600; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Raw 모드: 바이너리 데이터 중 0x0D/0x11/0x13 등 제어 문자의 변조 방지
    cfmakeraw(&tty);

    tty.c_cflag |= (CLOCAL | CREAD); // 수신 가능, 모뎀 제어 무시
    tty.c_cflag &= ~CSTOPB;          // 1 Stop bit
    tty.c_cflag &= ~CRTSCTS;         // 하드웨어 흐름 제어 OFF
    tty.c_cflag &= ~PARENB;          // Parity 없음

    // 32바이트 또는 0.1초마다 깨우도록 설정해 CPU 인터럽트 빈도 감소
    tty.c_cc[VMIN] = 32;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;

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

    uint8_t header[12] = {PIMTP_SYNC1, PIMTP_SYNC2, PIMTP_SYNC3, PIMTP_SYNC4};
    uint16_t type = PIMTP_PAYLOAD_RTK_CORRECTIONS;
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

int P2_GetStatus(ISRO_P2_T* device, STATUS_MESSAGE_T* status) {
    if (!device || !status) return -1;
    pthread_mutex_lock(&device->data_mutex);
    if (!device->status_valid) { pthread_mutex_unlock(&device->data_mutex); return -1; }
    memcpy(status, &device->latest_status, sizeof(STATUS_MESSAGE_T));
    pthread_mutex_unlock(&device->data_mutex);
    return 0;
}

// Reset 명령 전송 (MsgID 2410)
int P2_SendReset(ISRO_P2_T* device) {
    if (!device) return -1;
    int fd = -1;
    if (device->config.type == CONN_TYPE_SERIAL) fd = device->main_fd;
    else if (device->config.type == CONN_TYPE_TCP_CLIENT) fd = device->main_fd;
    else return -1;
    if (fd < 0) return -1;

    // PIMTP Header(12) + Auto Header(16) + Reset Body(12) + CRC(4) = 44 bytes
    uint8_t packet[44];
    memset(packet, 0, sizeof(packet));

    // 1. PIMTP Header
    PIMTP_HEADER_T* pim_hdr = (PIMTP_HEADER_T*)packet;
    pim_hdr->sync[0] = PIMTP_SYNC1; pim_hdr->sync[1] = PIMTP_SYNC2;
    pim_hdr->sync[2] = PIMTP_SYNC3; pim_hdr->sync[3] = PIMTP_SYNC4;
    pim_hdr->payload_type = PIMTP_PAYLOAD_AUTOMOTIVE;
    pim_hdr->payload_length = 16 + 12; // AutoHeader + ResetBody

    // 2. Automotive Header
    AUTOMOTIVE_HEADER_T* auto_hdr = (AUTOMOTIVE_HEADER_T*)(packet + 12);
    auto_hdr->message_id = MSG_ID_RESET;
    auto_hdr->message_data_size = 12;
    auto_hdr->time_status = 0;
    auto_hdr->gps_week = 0;
    auto_hdr->gps_millisecond = 0;
    auto_hdr->reserved = 0;

    // 3. Reset Body (Table 66, 67): Type(4) + Param1(4) + Param2(4)
    uint32_t* body = (uint32_t*)(packet + 12 + 16);
    body[0] = 0; // NORMAL RESET
    body[1] = 0;
    body[2] = 0;

    // 4. CRC
    uint32_t crc = CalculateNovAtelCRC32(packet, 40);
    memcpy(packet + 40, &crc, 4);

    // 5. Send
    write(fd, packet, sizeof(packet));
    return 0;
}
