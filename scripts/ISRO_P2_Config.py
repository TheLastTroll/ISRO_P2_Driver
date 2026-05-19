#!/usr/bin/env python3
"""
ISRO-P2 Config Updater - SerialLoad Protocol
=============================================
Winload 로그 시퀀스 기반 구현

실행 순서:
  Phase 1: Autobaud (9600) - SerialLoad 배너 대기
  Phase 2: Baud Switch (460800)
  Phase 3: SSL Transfer - 'Y' ACK, "SERLOAD2" 대기
  Phase 4: Device Query (3C C3 -> 5A A5)
  Phase 5: Config Download
  Phase 6: Flash & Reset

사용법:
  1. ssl_data.bin 파일이 같은 폴더에 있어야 함
  2. 스크립트 실행
  3. "장비 전원을 켜세요" 메시지가 나오면 장비 전원 ON
"""

import struct
import serial
import time
import sys
import os
import locale

# ==============================================================================
# 설정
# ==============================================================================
TARGET_PORT = '/dev/ttyUSB0'
INITIAL_BAUD = 9600
DATA_BAUD = 460800

DEBUG = True

# SSL 파일 (pcapng에서 추출)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SSL_FILE = os.path.join(SCRIPT_DIR, 'ssl_data.bin')

# Config 메타데이터
CONFIG_FILE = None  # 외부 파일 경로 (None이면 기본값 사용)
CONFIG_NAME = "CONFIG_FILE"
CONFIG_VERSION = "1.0.0"
CONFIG_SN = "SN"

# 기본 Config
USER_CONFIG_TEXT = """IMU:INTERNAL
NMEATALKER:AUTO
BaudRate:COM1,921600
BaudRate:COM2,115200
LogAutomotive:COM1,PVA,TIME,1,0
LogAutomotive:COM1,IMU,CHANGE,0,0
LogAutomotive:COM1,STATUS,CHANGE,0,0
LogNMEA:COM1,GGA,TIME,1.0,0
LogNMEA:COM2,GGA,TIME,1.0,0
LogNMEA:COM2,PASHR,TIME,1.0,0
LogNMEA:COM2,VTG,TIME,1.0,0
LogNMEA:COM2,GSV,TIME,1.0,0
LogNMEA:COM2,HDT,CHANGE,0,0
INSRotation:RBV,0.0,0.0,0.0,3.0,3.0,3.0
INSTranslation:Ant1,0.00,-1.20,1.00,0.05,0.05,0.05
INSTranslation:Ant2,0.00,1.80,1.00,0.05,0.05,0.05"""

# CANDMI Config (참고용 샘플)
USER_CONFIG_TEXT2 = """IMU:internal
BaudRate:COM1,921600
LogNMEA:COM1,GGA,TIME,1.0,0
LogAutomotive:COM1,PVA,TIME,1.0,0
LogAutomotive:COM1,IMU,CHANGE,0,0
LogAutomotive:COM1,STATUS,CHANGE,0,0
BaudRate:COM2,115200
LogNMEA:COM2,GGA,TIME,1.0,0
INSRotation:RBV,0,0,90,1.0,1.0,1.0
INSTranslation:Ant1,-0.4,0.0,0.9,0.05,0.05,0.05
INSTranslation:Ant2,-1.4,0.0,0.9,0.05,0.05,0.05
INSTranslation:DMI,-0.25,0.00,-0.33,0.03,0.03,0.03
SetVehicleParameters:2.670,1.54,1.54,FRONT_WHEEL_ACKERMANN,0
CAN:CAN1,500K
DMI:CAN1,DMI_LUA
LUA_CAN_SCRIPT:
MESSAGE_WHEELSPEED = 0x200
messageIDTable = {}
messageIDTable[0] = MESSAGE_WHEELSPEED
GEAR_PARK = 0x0
GEAR_REVERSE = 0x2
GEAR_NEUTRAL = 0x0
GEAR_DRIVE = 0x1

function applyScaleFactor(value)
   return value >> 5
end

function processCANData(inputTable)
   local returnTable = {}
   if inputTable['MessageID'] == MESSAGE_WHEELSPEED then
      local bin_data = string.unpack('I8', inputTable['Data'])
      local gear = ( (bin_data & 0x000000000000C000) >> 14 )
      local direction = 0
      if gear == GEAR_DRIVE then
         direction = 1
      elseif gear == GEAR_REVERSE then
         direction = -1
      else
         direction = 1
      end

      fl_data = ( (bin_data & 0x0000000000003FFF) >> 0 ) * direction * 0.03125
      rr_data = ( (bin_data & 0x000000003FFF0000) >> 16 ) * direction * 0.03125
      fr_data = ( (bin_data & 0x00003FFF00000000) >> 32 ) * direction * 0.03125
      rl_data = ( (bin_data & 0x3FFF000000000000) >> 48 ) * direction * 0.03125

      returnTable['front left'] = fl_data
      returnTable['rear right'] = rr_data
      returnTable['front right'] = fr_data
      returnTable['rear left'] = rl_data

   end

   return returnTable

end"""

# ==============================================================================
# 상수
# ==============================================================================
DATABLK_COMPONENT = 0x3A7A000B
SYNC_BYTE = 0x16

# ==============================================================================
# 유틸리티
# ==============================================================================
def debug(msg):
    if DEBUG:
        print(f"    [DEBUG] {msg}")

def calculate_crc32(data):
    """NovAtel CRC32 - Data Block 내부용"""
    crc = 0
    for byte in data:
        temp = (crc ^ byte) & 0xFF
        for _ in range(8):
            if temp & 1:
                temp = (temp >> 1) ^ 0xEDB88320
            else:
                temp >>= 1
        crc = (crc >> 8) ^ temp
    return crc & 0xFFFFFFFF

def calculate_jamcrc(data):
    """JAMCRC - 3C C3 패킷용 (Big-Endian으로 저장됨), final XOR 없음"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc

def calc_novatel_crc32(data):
    """표준 NovAtel CRC32 (PIMTP 패킷용) - calculate_crc32와 동일"""
    return calculate_crc32(data)

def send_pimtp_reset(target_port):
    """
    PIMTP 프로토콜로 강제 리셋 명령(MsgID 2410) 전송.
    SerialLoad 모드로 진입시키기 위해 미리 보내는 용도.
    """
    print(f"[Reset] Sending PIMTP RESET packet to {target_port}...")

    try:
        # PIMTP Header (Sync + Type 1 + Len 28)
        pimtp_header = struct.pack('<4sHH I', b'\xAC\x55\x96\x83', 1, 0, 28)
        # Auto Header (MsgID 2410, Size 12)
        auto_header = struct.pack('<HHHHII', 2410, 12, 0, 0, 0, 0)
        # Body (0 = Normal Reset)
        reset_body = struct.pack('<III', 0, 0, 0)

        payload = pimtp_header + auto_header + reset_body
        crc_val = calc_novatel_crc32(payload)
        final_packet = payload + struct.pack('<I', crc_val)

        # 현재 baudrate를 모르므로 후보 baudrate를 순회하며 전송.
        # 리셋이 성공하면 장비가 즉시 재부팅되므로 빠르게 보내고 빠져나옴.
        baud_candidates = [460800, 921600, 115200]
        sent = False
        for baud in baud_candidates:
            try:
                with serial.Serial(target_port, baud, timeout=0.1) as ser:
                    ser.write(final_packet)
                    time.sleep(0.05)
                sent = True
            except Exception:
                pass

        if sent:
            print("    -> PIMTP Reset packet sent.")
        else:
            print("    [!] Could not open port for reset (might be busy or wrong port).")

    except Exception as e:
        print(f"    [!] Reset failed: {e}")

# ==============================================================================
# Data Block 생성
# ==============================================================================
def create_data_block(config_text):
    """140-byte header + config content"""
    content = config_text.replace('\r\n', '\n').replace('\n', '\r\n')
    content_bytes = content.encode('ascii')

    data_len = len(content_bytes)
    data_crc = calculate_crc32(content_bytes)

    b_name = CONFIG_NAME.encode('ascii')[:15].ljust(16, b'\x00')
    b_ver = CONFIG_VERSION.encode('ascii')[:15].ljust(16, b'\x00')
    b_sn = CONFIG_SN.encode('ascii')[:15].ljust(16, b'\x00')
    b_res1 = b'\x00' * 16
    b_res2 = b'\x00' * 16
    locale.setlocale(locale.LC_TIME, 'C')  # 영어 로케일 강제
    b_date = time.strftime("%Y/%b/%d").encode('ascii')[:11].ljust(12, b'\x00')
    b_time = time.strftime("%H:%M:%S").encode('ascii')[:11].ljust(12, b'\x00')
    b_padding = b'\x00' * 8

    header_fmt = '<IIII HHHH I 16s16s16s16s16s12s12s8s'

    temp = struct.pack(header_fmt,
        DATABLK_COMPONENT, data_len, data_crc, data_len,
        2, 140, 0, 0, 0,
        b_name, b_ver, b_sn, b_res1, b_res2, b_date, b_time, b_padding
    )
    header_crc = calculate_crc32(temp)

    final = struct.pack(header_fmt,
        DATABLK_COMPONENT, data_len, data_crc, data_len,
        2, 140, 0, 0, header_crc,
        b_name, b_ver, b_sn, b_res1, b_res2, b_date, b_time, b_padding
    )

    return final + content_bytes

# ==============================================================================
# 3C C3 패킷 생성
# ==============================================================================
def build_3cc3_packet(direction, flags, seq, msg_type, payload):
    """
    3C C3 패킷 구조:
      [0-1] 3C C3 - Sync
      [2]   Direction (0x01=REQ, 0x80=RESP)
      [3]   Flags
      [4-5] Length (little-endian)
      [6]   Sequence
      [7]   Message Type
      [8+]  Payload
      [-4]  CRC32 (JAMCRC, Big-Endian)
    """
    length = len(payload)
    header = struct.pack('<BBBBHBB',
        0x3C, 0xC3,
        direction,
        flags,
        length,
        seq,
        msg_type
    )
    packet = header + payload
    crc = calculate_jamcrc(packet)
    return packet + struct.pack('>I', crc)

def build_config_packet(data_block, seq=10):
    """
    Config 전송용 3C C3 패킷

    pcapng 분석 결과 (pcab4~pcab8):
      기본 공식:
        base_length = config_data_len - 362
        base_prefix = config_data_len - 372

      다중 오버플로우 처리:
        overflow_count = base_prefix // 256
        Flags  = 0x02 + overflow_count
        Prefix[7] = 0x02 + overflow_count
        Length = base_length - (overflow_count * 256)
        Prefix[8] = base_prefix % 256

      CRC: JAMCRC + Big-Endian 저장

    검증:
      501B  -> Flags=0x02, Length=139, Prefix=02 81 (overflow=0)
      502B  -> Flags=0x02, Length=140, Prefix=02 82 (overflow=0)
      547B  -> Flags=0x02, Length=185, Prefix=02 AF (overflow=0)
      693B  -> Flags=0x03, Length=75,  Prefix=03 41 (overflow=1)
      1784B -> Flags=0x07, Length=142, Prefix=07 84 (overflow=5)
    """
    # Config 데이터 길이 (Data Block = 140B header + config data)
    config_data_len = len(data_block) - 140

    base_length = config_data_len - 362
    base_prefix = config_data_len - 372

    overflow_count = base_prefix // 256
    flags = 0x02 + overflow_count
    prefix_7 = 0x02 + overflow_count
    length_field = base_length - (overflow_count * 256)
    prefix_8 = base_prefix % 256

    prefix = bytes([0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, prefix_7, prefix_8])
    payload = prefix + data_block

    header = struct.pack('<BBBBHBB',
        0x3C, 0xC3,
        0x80,         # Direction (RESP)
        flags,        # Flags (동적)
        length_field, # Length (동적)
        seq,
        0x1E          # MsgType
    )

    packet = header + payload
    crc = calculate_jamcrc(packet)
    return packet + struct.pack('>I', crc)  # Big-Endian

# ==============================================================================
# Phase 1: Autobaud - SerialLoad 배너 + UPDATE_BEGIN
# ==============================================================================

# UPDATE_BEGIN 패킷 (pcapng Frame 10759에서 추출)
# 9600 baud에서 이 패킷을 보내야 장비가 업데이트 모드로 진입한다.
UPDATE_BEGIN_PACKET = bytes([
    0x16,  # SYNC
    0x55, 0xAA, 0x02,
    0x00, 0x04,  # Seq
    0x00, 0x04,  # Length
    0x00, 0x00, 0x00, 0x0F,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x07, 0x08, 0x00
])

def phase1_autobaud(ser, timeout=30.0):
    """
    Winload 로그:
      "Searching for card..."
      "Card Detected"
      "V4.00 COM2 PIM222A"

    1. SerialLoad 배너 수신 (9600)
    2. 'D' 수신 (Device Ready)
    3. UPDATE_BEGIN 패킷 전송 (9600)
    4. 'Y' ACK 수신
    """
    print("[Phase 1] Autobaud - Searching for card...")

    start = time.time()
    buffer = b''
    banner = None
    d_received = False

    while time.time() - start < timeout:
        ser.write(bytes([SYNC_BYTE] * 20))
        ser.flush()
        time.sleep(0.05)

        if ser.in_waiting:
            buffer += ser.read(ser.in_waiting)

            if b'SerialLoad' in buffer and banner is None:
                idx = buffer.find(b'SerialLoad')
                end = buffer.find(b'\n', idx)
                if end == -1:
                    end = min(idx + 50, len(buffer))
                banner = buffer[idx:end].decode('ascii', errors='replace').strip()
                print(f"    Card Detected")
                print(f"    {banner}")

                if 'PIM222A' in banner:
                    print(f"    Found platform: PIM222A")

            if banner and b'D' in buffer:
                d_received = True
                debug("'D' received (Device Ready)")
                break

    if not banner:
        print("    [!] Card not detected")
        return False, None

    print("    Sending UPDATE_BEGIN...")
    ser.write(bytes([SYNC_BYTE]))
    time.sleep(0.01)
    ser.write(UPDATE_BEGIN_PACKET)
    ser.flush()

    # 'Y' ACK 대기
    start = time.time()
    while time.time() - start < 3.0:
        if ser.in_waiting:
            resp = ser.read(ser.in_waiting)
            if b'Y' in resp:
                print("    UPDATE_BEGIN ACK received ('Y')")
                return True, buffer
        time.sleep(0.02)

    print("    [!] UPDATE_BEGIN ACK not received")
    return True, buffer  # 일단 계속 진행

# ==============================================================================
# Phase 2: Baud Switch
# ==============================================================================
def phase2_baud_switch(ser, target_port):
    """
    Winload 로그:
      "Changing Baud Rate to: 460800"
      "baud rate changed"

    Phase 1에서 UPDATE_BEGIN + 'Y' ACK 완료 후 baud 전환.
    """
    print(f"\n[Phase 2] Changing Baud Rate to: {DATA_BAUD}")

    ser.close()
    time.sleep(0.1)

    try:
        new_ser = serial.Serial(target_port, DATA_BAUD, timeout=0.1)
        print(f"    baud rate changed")
        return new_ser
    except Exception as e:
        print(f"    [!] Failed to change baud rate: {e}")
        return None

# ==============================================================================
# Phase 3: SSL Transfer
# ==============================================================================

# SSL 전송 완료 후 마무리 패킷 (pcapng Frame 11387, 11393에서 추출)
SSL_FINISH_PACKET_1 = bytes.fromhex('1655aa02460100080000055620020824dbec8132bec18d82')
SSL_FINISH_PACKET_2 = bytes.fromhex('1655aa0247020000000000a120010080')  # 종료 신호

def phase3_ssl_transfer(ser, ssl_data):
    """
    Winload 로그:
      "Initializing download"
      "Using PIM222 Second Stage Loader"
      "Stage 1 Done"

    1. SSL 데이터 전송 (996B chunks)
    2. 각 청크마다 'Y' ACK 수신
    3. 마무리 패킷 #1 전송 -> 'Y' ACK
    4. 마무리 패킷 #2 전송 -> 'Y' ACK
    5. SERLOAD2 수신 (Stage 1 Done)
    """
    print(f"\n[Phase 3] Initializing download")
    print(f"    Using PIM222 Second Stage Loader")

    chunk_size = 996
    offset = 0
    total = len(ssl_data)
    y_count = 0

    while offset < total:
        chunk = ssl_data[offset:offset + chunk_size]
        ser.write(chunk)
        ser.flush()

        offset += len(chunk)
        pct = int(offset / total * 100)

        time.sleep(0.005)
        if ser.in_waiting:
            resp = ser.read(ser.in_waiting)
            y_count += resp.count(b'Y')

        sys.stdout.write(f"\r    Downloading SSL: {pct}% (ACK: {y_count})")
        sys.stdout.flush()

    print()

    # 마무리 패킷 #1
    debug("Sending SSL finish packet #1...")
    ser.write(SSL_FINISH_PACKET_1)
    ser.flush()
    time.sleep(0.05)

    if ser.in_waiting:
        resp = ser.read(ser.in_waiting)
        if b'Y' in resp:
            y_count += 1
            debug("Finish #1 ACK received")

    # 마무리 패킷 #2 (종료 신호)
    debug("Sending SSL finish packet #2 (end signal)...")
    ser.write(SSL_FINISH_PACKET_2)
    ser.flush()
    time.sleep(0.05)

    if ser.in_waiting:
        resp = ser.read(ser.in_waiting)
        if b'Y' in resp:
            y_count += 1
            debug("Finish #2 ACK received")

    print(f"    SSL transfer complete (ACK: {y_count})")

    # SERLOAD2 대기 (Stage 1 Done)
    print("    Waiting for Stage 1 Done...")

    start = time.time()
    buffer = b''
    serload2_found = False

    while time.time() - start < 10.0:
        if ser.in_waiting:
            buffer += ser.read(ser.in_waiting)
            if b'SERLOAD2' in buffer:
                serload2_found = True
                break
        time.sleep(0.05)

    if serload2_found:
        print("    Stage 1 Done")
        return True
    else:
        print("    [!] SERLOAD2 not received")
        debug(f"Buffer: {buffer}")
        return False

# ==============================================================================
# Phase 4, 5, 6: 3C C3 Protocol (Device Query + Config + Flash)
# ==============================================================================

# pcapng에서 추출한 정확한 3C C3 패킷들
PKT_INITIAL          = bytes.fromhex('3cc3800002000f07009311f02b')     # Seq=15, Msg=0x07
PKT_ACK_1            = bytes.fromhex('3cc3010001000201839b0566')        # Seq=2,  Msg=0x01
PKT_QUERY_PSN        = bytes.fromhex('3cc380000200020900054a4ef6')      # Seq=2,  Msg=0x09
PKT_ACK_2            = bytes.fromhex('3cc3010001000401d5c1a2e0')        # Seq=4,  Msg=0x01
PKT_QUERY_HW         = bytes.fromhex('3cc3800002000427000ac03b68')      # Seq=4,  Msg=0x27
PKT_ACK_3            = bytes.fromhex('3cc3010001000601e7f7c062')        # Seq=6,  Msg=0x01
PKT_QUERY_SW         = bytes.fromhex('3cc380000200062d00f3ab078c')      # Seq=6,  Msg=0x2D
PKT_ACK_4            = bytes.fromhex('3cc30100010008017974edec')        # Seq=8,  Msg=0x01
PKT_QUERY_CARDS      = bytes.fromhex('3cc380000200081f3c3b2a0270')      # Seq=8,  Msg=0x1F
PKT_RESET_DATA       = bytes.fromhex('3cc380000c00091600050000000000000282007949e571')  # Seq=9, Msg=0x16
PKT_POST_CONFIG_ACK  = bytes.fromhex('3cc3800001000b0187b57952')        # Seq=11, Msg=0x01
PKT_FINAL_ACK        = bytes.fromhex('3cc3010001000e012f2e4a6a')        # Seq=14, Msg=0x01

def wait_5aa5_response(ser, timeout=0.5, flush_first=False):
    """5A A5 응답 대기."""
    if flush_first:
        time.sleep(0.02)
        if ser.in_waiting:
            ser.read(ser.in_waiting)

    start = time.time()
    buffer = b''
    while time.time() - start < timeout:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            buffer += chunk
            if b'\x5a\xa5' in buffer:
                # 후속 데이터가 더 올 수 있으므로 잠시 더 읽음
                time.sleep(0.02)
                if ser.in_waiting:
                    buffer += ser.read(ser.in_waiting)
                return buffer
        time.sleep(0.005)
    return buffer

def phase4_5_6_protocol(ser, data_block):
    """
    pcapng 분석 기반 시퀀스.
    """
    print(f"\n[Phase 4] Device Query")
    device_info = {}

    # ---------- Phase 4: Device Query ----------

    # Initial
    debug("TX Initial (Seq=15, Msg=0x07)")
    ser.write(PKT_INITIAL)
    ser.flush()
    resp = wait_5aa5_response(ser, timeout=1.0)
    debug(f"RX: {len(resp)}B - {resp[:30].hex(' ') if resp else 'empty'}")
    if b'DNA' in resp:
        idx = resp.find(b'DNA')
        psn = resp[idx:idx+16].decode('ascii', errors='replace').strip('\x00')
        device_info['PSN'] = psn
        print(f"    Requesting PSN... {psn}")

    # ACK #1 + Query PSN
    debug("TX ACK #1 (Seq=2, Msg=0x01)")
    ser.write(PKT_ACK_1); ser.flush(); time.sleep(0.02)

    debug("TX Query PSN (Seq=2, Msg=0x09)")
    ser.write(PKT_QUERY_PSN); ser.flush()
    resp = wait_5aa5_response(ser, timeout=1.0)
    debug(f"RX: {len(resp)}B - {resp[:30].hex(' ') if resp else 'empty'}")
    if b'PIM222A-' in resp:
        idx = resp.find(b'PIM222A-')
        hw = resp[idx:idx+12].decode('ascii', errors='replace').strip('\x00')
        device_info['HW'] = hw
        print(f"    Requesting HW_Ver... {hw}")

    # ACK #2 + Query HW
    debug("TX ACK #2 (Seq=4, Msg=0x01)")
    ser.write(PKT_ACK_2); ser.flush(); time.sleep(0.02)

    debug("TX Query HW (Seq=4, Msg=0x27)")
    ser.write(PKT_QUERY_HW); ser.flush()
    resp = wait_5aa5_response(ser, timeout=1.0)
    debug(f"RX: {len(resp)}B - {resp[:30].hex(' ') if resp else 'empty'}")
    print(f"    Requesting SW Platform...")

    # ACK #3 + Query SW
    debug("TX ACK #3 (Seq=6, Msg=0x01)")
    ser.write(PKT_ACK_3); ser.flush(); time.sleep(0.02)

    debug("TX Query SW (Seq=6, Msg=0x2D)")
    ser.write(PKT_QUERY_SW); ser.flush()
    resp = wait_5aa5_response(ser, timeout=1.0)
    debug(f"RX: {len(resp)}B - {resp[:30].hex(' ') if resp else 'empty'}")
    print(f"    Requesting Number of Cards...")

    # ACK #4 + Query Cards
    debug("TX ACK #4 (Seq=8, Msg=0x01)")
    ser.write(PKT_ACK_4); ser.flush(); time.sleep(0.02)

    debug("TX Query Cards (Seq=8, Msg=0x1F)")
    ser.write(PKT_QUERY_CARDS); ser.flush()
    resp = wait_5aa5_response(ser, timeout=1.0)
    debug(f"RX: {len(resp)}B - {resp[:30].hex(' ') if resp else 'empty'}")

    # Reset Data (pcapng와 동일하게 ACK 없이 바로 전송)
    debug("TX Reset Data (Seq=9, Msg=0x16)")
    ser.write(PKT_RESET_DATA); ser.flush()
    resp = wait_5aa5_response(ser, timeout=1.0)
    debug(f"RX: {len(resp)}B - {resp[:30].hex(' ') if resp else 'empty'}")
    print(f"    Resetting Data...")

    # ---------- Phase 5: Config Download ----------
    print(f"\n[Phase 5] Config Download")
    print(f"    Using config: USER_CONFIG")
    print(f"    Downloading...")

    config_pkt = build_config_packet(data_block, seq=0x0A)
    debug(f"TX Config (Seq=10, Msg=0x1E) - {len(config_pkt)}B")
    debug(f"   Header: {config_pkt[:12].hex(' ')}")
    debug(f"   CRC: {config_pkt[-4:].hex(' ')}")

    # 전송 전 버퍼 정리 및 안정화
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)

    # 큰 패킷은 청크로 나눠 전송 (USB-시리얼 버퍼 문제 방지)
    CHUNK_SIZE = 64
    for i in range(0, len(config_pkt), CHUNK_SIZE):
        chunk = config_pkt[i:i+CHUNK_SIZE]
        ser.write(chunk)
        ser.flush()
        time.sleep(0.002)

    # Config ACK 대기
    debug("Waiting for Config ACK...")
    resp = wait_5aa5_response(ser, timeout=2.0)
    debug(f"RX: {len(resp)}B - {resp[:30].hex(' ') if resp else 'empty'}")

    if b'\x5a\xa5' in resp:
        debug("Config 5A A5 ACK received")
        print(f"    Download Complete")
    else:
        print(f"    [!] Config ACK not received")
        time.sleep(0.5)
        if ser.in_waiting:
            extra = ser.read(ser.in_waiting)
            debug(f"Extra data in buffer: {len(extra)}B - {extra[:30].hex(' ')}")
        return False, device_info

    # ---------- Phase 6: Flash & Reset ----------
    print(f"\n[Phase 6] Flash & Reset")
    print(f"    Queueing reset request...")

    # Post Config ACK
    debug("TX Post Config ACK (Seq=11, Msg=0x01)")
    ser.write(PKT_POST_CONFIG_ACK); ser.flush()

    # ACK 전송 후 장비 응답 대기
    time.sleep(0.2)

    resp = wait_5aa5_response(ser, timeout=1.0)

    # 버퍼에 남은 추가 데이터 흡수
    time.sleep(0.1)
    if ser.in_waiting:
        extra = ser.read(ser.in_waiting)
        resp += extra
        debug(f"Extra data read: {len(extra)}B")

    debug(f"RX Post Config: {len(resp)}B - {resp[:50].hex(' ') if resp else 'empty'}")

    # Erase/Reset 메시지 대기 (Post Config 응답에 이미 포함될 수도 있음)
    debug("Waiting for Erase/Reset message...")
    start = time.time()
    buffer = resp
    erase_reset_received = False
    erase_time_shown = False

    while time.time() - start < 15.0:
        time.sleep(0.05)
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            buffer += chunk
            debug(f"Received: {len(chunk)}B, total: {len(buffer)}B")

        if not erase_time_shown and b'Erase Time' in buffer:
            try:
                erase_idx = buffer.find(b'Erase Time')
                msg = buffer[erase_idx:erase_idx+50].decode('ascii', errors='replace')
                msg = ''.join(c for c in msg if c.isprintable() or c == ' ')
                print(f"    Total {msg.strip()}")
            except Exception:
                print(f"    Total Erase Time: OK")
            erase_time_shown = True

        if b'Resetting' in buffer or b'Reset' in buffer:
            erase_reset_received = True
            print(f"    Resetting...")
            break

        # Erase Time 받은 뒤 2초 더 대기 후 종료 (Reset 메시지가 없는 펌웨어도 있음)
        if erase_time_shown and time.time() - start > 2.0:
            erase_reset_received = True
            break

    # Final ACK (성공/실패 무관하게 전송)
    debug("TX Final ACK (Seq=14, Msg=0x01)")
    ser.write(PKT_FINAL_ACK); ser.flush()

    if erase_reset_received or erase_time_shown:
        print(f"    Done.")
        return True, device_info
    else:
        print(f"    [!] Erase/Reset message not received")
        debug(f"Buffer contents: {buffer[:100].hex(' ') if buffer else 'empty'}")
        return False, device_info

# ==============================================================================
# 메인
# ==============================================================================
def perform_update(target_port=TARGET_PORT, config_text=None):
    print("=" * 60)
    print("PIM222A Config Updater (SerialLoad Protocol)")
    print("=" * 60)

    # SSL 로드
    print("\n[Init] Loading SSL...")
    if not os.path.exists(SSL_FILE):
        print(f"    [!] SSL file not found: {SSL_FILE}")
        print("    ssl_data.bin 파일이 필요합니다.")
        return False

    with open(SSL_FILE, 'rb') as f:
        ssl_data = f.read()
    print(f"    SSL: {len(ssl_data)} bytes")

    # Config Data Block
    print("\n[Init] Creating config data block...")

    if config_text is None:
        config_text = USER_CONFIG_TEXT
        if CONFIG_FILE and os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                config_text = f.read()
            print(f"    Config file: {CONFIG_FILE}")
        else:
            print(f"    Using built-in config")
    else:
        print(f"    Using configuration from GUI")

    data_block = create_data_block(config_text)
    print(f"    Data Block: {len(data_block)} bytes")

    # PIMTP Reset 시도 (자동 재부팅)
    send_pimtp_reset(target_port)

    print("    Waiting for device reboot (SerialLoad)...")
    print("    (자동 리셋이 실패하면 수동으로 껐다 켜주세요)")
    time.sleep(0.5)

    print(f"\n[Init] Opening {target_port} at {INITIAL_BAUD} baud...")
    try:
        ser = serial.Serial(target_port, INITIAL_BAUD, timeout=0.1)
    except Exception as e:
        print(f"    [!] Error: {e}")
        return False

    print("\n" + "=" * 60)
    print("    *** 지금 장비 전원을 켜세요! ***")
    print("    *** Power the unit ON now ***")
    print("=" * 60 + "\n")

    # Phase 1: Autobaud
    success, _ = phase1_autobaud(ser, timeout=30.0)
    if not success:
        ser.close()
        return False

    # Phase 2: Baud Switch
    ser = phase2_baud_switch(ser, target_port)
    if ser is None:
        return False

    # Phase 3: SSL Transfer
    success = phase3_ssl_transfer(ser, ssl_data)
    if not success:
        ser.close()
        return False

    # Phase 4, 5, 6
    success, device_info = phase4_5_6_protocol(ser, data_block)
    ser.close()

    print("\n" + "=" * 60)
    if success:
        print("SUCCESS! 장비가 새 config로 재부팅됩니다.")
        if device_info:
            print(f"Device: {device_info}")
    else:
        print("완료 메시지 미수신 - 장비 확인 필요")
    print("=" * 60)

    return success

# ==============================================================================
# Entry Point
# ==============================================================================
if __name__ == "__main__":
    print("""
┌─────────────────────────────────────────────────────────────┐
│  ISRO-P2 SerialLoad Config Updater                          │
│                                                             │
│  필요 파일: ssl_data.bin (같은 폴더)                        │
│                                                             │
│  순서:                                                      │
│    1. Enter                                                 │
│    2. "장비 전원을 켜세요" 메시지 나오면                    │
│    3. 장비 전원 ON                                          │
└─────────────────────────────────────────────────────────────┘
""")

    input("Press Enter to start...")
    print()
    perform_update()
