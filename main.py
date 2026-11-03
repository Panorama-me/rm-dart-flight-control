import sensor
import time
from pyb import UART
import image
import os
import gc

print("=== BOOT ===")

# =========================
# UART
# =========================
UART_ID = 3
UART_BAUDRATE = 115200
uart = UART(UART_ID, UART_BAUDRATE, read_buf_len=2048)

# =========================
# 配置
# =========================
RECORD_ENABLE = True
# Native RGB565 ImageIO file stream; no whole-video buffer in RAM.
DESCENT_RECORD_TAIL_MS = 3000
MAX_RECORD_MS = 30000
RECORD_INTERVAL_MS = 0  # 0: write every captured frame; >0: minimum interval
RECORD_SYNC_MS = 1000
TELEM_TIMEOUT_MS = 300
TELEM_LOG_ENABLE = True
TELEM_LOG_SYNC_MS = 1000

# 模式：由下位机握手决定
MODE_DEFAULT = 0
MODE_PITCH = 1
MODE_YAW = 2
MODE_BOTH = 3
MODE_NAMES = ("D", "P", "Y", "B")

# 飞行状态：由下位机遥测给出
STATE_READY = 0
STATE_IMU_READY = 1
STATE_SERVO_READY = 2
STATE_LAUNCHED = 3
STATE_ASCENT = 4
STATE_DESCENT = 5
STATE_VISION_ERROR = 6
STATE_NAMES = ("R", "I", "S", "L", "A", "D", "E")

mode = MODE_BOTH

# =========================
# 图像 / ROI
# =========================
BLOB_ROI = (0, 0, 320, 240)  # snapshot 输出画面；动态 cur_roi 只限制识别搜索
INIT_ROI = (0, 0, 320, 240)
READOUT_ROI = (0, 700, 640, 480)  # 传感器读出窗口，决定取景范围和放大效果
FOUND_RANGE_X = 40
FOUND_RANGE_Y = 40
THRESHOLDS1 = [(9, 100, -102, -25, 0, 127)]

FRAME_SIZE = sensor.QVGA
PIXFORMAT = sensor.RGB565
EXPOSURE_US = 800
SKIP_FRAMES_MS = 1500

GUIDE_CX = BLOB_ROI[0] + BLOB_ROI[2] // 2
GUIDE_CY = BLOB_ROI[1] + BLOB_ROI[3] // 2

# =========================
# 通信协议
# =========================
VISION_SOF = 0xAA
VISION_EOF = 0xFF
VISION_CRC8_INIT = 0x00
VISION_CRC8_POLY = 0x8C

# 握手：F103 -> OpenMV: A5 MODE FLAGS FE CRC8
#       OpenMV -> F103: 5A MODE FE CRC8
HS_SOF = 0xA5
HS_EOF = 0xFE
HS_FRAME_LEN = 5
ACK_SOF = 0x5A
ACK_FRAME_LEN = 4
RESET_SOF = 0x5B
RESET_FRAME_LEN = 4

# 遥测：F103 -> OpenMV，24 字节
TELEM_SOF = 0x55
TELEM_EOF = 0xFE
TELEM_FRAME_LEN = 24
TELEM_ANGLE_SCALE = 100.0
TELEM_GYRO_SCALE = 10.0

T_GYRO_X_H = 1
T_GYRO_X_L = 2
T_GYRO_Y_H = 3
T_GYRO_Y_L = 4
T_GYRO_Z_H = 5
T_GYRO_Z_L = 6
T_PITCH_H = 7
T_PITCH_L = 8
T_YAW_H = 9
T_YAW_L = 10
T_ROLL_H = 11
T_ROLL_L = 12
T_OUT_PITCH_H = 13
T_OUT_PITCH_L = 14
T_OUT_YAW_H = 15
T_OUT_YAW_L = 16
T_OUT_ROLL_H = 17
T_OUT_ROLL_L = 18
T_MODE_INDEX = 19
T_STATE_INDEX = 20
T_FLAGS_INDEX = 21
T_EOF_INDEX = 22
T_CRC_INDEX = 23

F_LINK_VALID = 0x01
F_TARGET_VALID = 0x02
F_IMU_READY = 0x04
F_HANDSHAKE_DONE = 0x08
F_OUTPUT_VALID = 0x10

# [DEBUG] 发送统计
TX_SKIP_COUNT = 0
TX_SLOW_COUNT = 0
TX_SLOW_MS = 20

TELEM_BUF_LIMIT = 200


# =========================
# 数据结构
# =========================
class BlobData:
    def __init__(self, x=0, y=0, w=0, h=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def area(self):
        return self.w * self.h

    def cx(self):
        return self.x + self.w // 2

    def cy(self):
        return self.y + self.h // 2


# =========================
# CRC8
# =========================
def crc8_lsb(data, start=0, end=None):
    if end is None:
        end = len(data)
    crc = VISION_CRC8_INIT
    for i in range(start, end):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ VISION_CRC8_POLY) & 0xFF
            else:
                crc >>= 1
    return crc


def s16(hi, lo):
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v


def mode_name(m):
    if 0 <= m < len(MODE_NAMES):
        return MODE_NAMES[m]
    return "?"


def state_name(s):
    if 0 <= s < len(STATE_NAMES):
        return STATE_NAMES[s]
    return "?"


# =========================
# 握手：等待下位机，回 ACK
# =========================
def wait_for_handshake():
    global mode

    # 通知 F103 清除上一次握手锁存，允许重新握手
    reset = bytearray([RESET_SOF, RESET_SOF, RESET_SOF])
    reset.append(crc8_lsb(reset))
    for _ in range(3):
        uart.write(reset)
        time.sleep_ms(5)
    time.sleep_ms(5)
    while uart.any():
        uart.read()

    buf = bytearray()
    idx = 0
    last_rx_print = 0
    last_wait_print = time.ticks_ms()
    print("waiting handshake...")

    while True:
        n = uart.any()
        if n:
            data = uart.read(n)
            if data:
                buf.extend(data)
                nowp = time.ticks_ms()
                if time.ticks_diff(nowp, last_rx_print) > 500:
                    print("rx first bytes:", data[:8])
                    last_rx_print = nowp

        while len(buf) - idx >= HS_FRAME_LEN:
            if buf[idx] != HS_SOF:
                idx += 1
                continue

            if (buf[idx + 3] != HS_EOF or
                    crc8_lsb(buf, idx, idx + 4) != buf[idx + 4]):
                idx += 1
                continue

            m = buf[idx + 1]
            if m > MODE_BOTH:
                idx += HS_FRAME_LEN
                continue

            mode = m
            ack = bytearray([ACK_SOF, mode, HS_EOF])
            ack.append(crc8_lsb(ack))

            # 连发 3 次，降低 ACK 丢失概率
            for _ in range(3):
                uart.write(ack)
                time.sleep_ms(2)

            # 清掉握手残留
            time.sleep_ms(5)
            while uart.any():
                uart.read()

            print("handshake ok, mode =", mode)
            return mode

        if idx:
            buf = buf[idx:]
            idx = 0

        if time.ticks_diff(time.ticks_ms(), last_wait_print) > 1000:
            print("still waiting, buf_len =", len(buf))
            last_wait_print = time.ticks_ms()

        time.sleep_ms(5)


# =========================
# 遥测解析
# =========================
telem_buf = bytearray()
telem_idx = 0

telem_gyro_x = 0.0
telem_gyro_y = 0.0
telem_gyro_z = 0.0
telem_pitch = 0.0
telem_yaw = 0.0
telem_roll = 0.0
telem_out_pitch = 0.0
telem_out_yaw = 0.0
telem_out_roll = 0.0
telem_mode = MODE_BOTH
telem_state = STATE_READY
telem_flags = 0
telem_link = 0
telem_last_ms = 0
telem_ok_printed = False
last_no_link_print = 0
telem_diag_count = 0
telem_frames = 0
flight_seen = False
descent_seen_ms = None


def poll_telemetry():
    global telem_buf, telem_idx
    global telem_gyro_x, telem_gyro_y, telem_gyro_z
    global telem_pitch, telem_yaw, telem_roll
    global telem_out_pitch, telem_out_yaw, telem_out_roll
    global telem_mode, telem_state, telem_flags
    global telem_link, telem_last_ms
    global telem_ok_printed, last_no_link_print, telem_diag_count
    global telem_frames
    global mode
    global flight_seen, descent_seen_ms

    got = 0
    n = uart.any()
    if n:
        data = uart.read(n)
        if data:
            telem_buf.extend(data)
            if (not telem_link) and (
                    time.ticks_diff(time.ticks_ms(), last_no_link_print) > 500):
                print("telem rx:", data[:8])
                last_no_link_print = time.ticks_ms()

    while len(telem_buf) - telem_idx >= TELEM_FRAME_LEN:
        if telem_buf[telem_idx] != TELEM_SOF:
            telem_idx += 1
            continue

        if telem_diag_count < 3:
            print("telem cand:", telem_buf[telem_idx:telem_idx + TELEM_FRAME_LEN])
            print("eof/calc/recv:",
                  telem_buf[telem_idx + T_EOF_INDEX],
                  crc8_lsb(telem_buf, telem_idx,
                           telem_idx + T_CRC_INDEX),
                  telem_buf[telem_idx + T_CRC_INDEX])
            telem_diag_count += 1

        if (telem_buf[telem_idx + T_EOF_INDEX] != TELEM_EOF or
                crc8_lsb(telem_buf, telem_idx,
                         telem_idx + T_CRC_INDEX) !=
                telem_buf[telem_idx + T_CRC_INDEX]):
            telem_idx += 1
            continue

        b = telem_idx
        telem_gyro_x = s16(telem_buf[b + T_GYRO_X_H],
                           telem_buf[b + T_GYRO_X_L]) / TELEM_GYRO_SCALE
        telem_gyro_y = s16(telem_buf[b + T_GYRO_Y_H],
                           telem_buf[b + T_GYRO_Y_L]) / TELEM_GYRO_SCALE
        telem_gyro_z = s16(telem_buf[b + T_GYRO_Z_H],
                           telem_buf[b + T_GYRO_Z_L]) / TELEM_GYRO_SCALE

        telem_pitch = s16(telem_buf[b + T_PITCH_H],
                          telem_buf[b + T_PITCH_L]) / TELEM_ANGLE_SCALE
        telem_yaw = s16(telem_buf[b + T_YAW_H],
                        telem_buf[b + T_YAW_L]) / TELEM_ANGLE_SCALE
        telem_roll = s16(telem_buf[b + T_ROLL_H],
                         telem_buf[b + T_ROLL_L]) / TELEM_ANGLE_SCALE

        telem_out_pitch = s16(telem_buf[b + T_OUT_PITCH_H],
                              telem_buf[b + T_OUT_PITCH_L]) / TELEM_ANGLE_SCALE
        telem_out_yaw = s16(telem_buf[b + T_OUT_YAW_H],
                            telem_buf[b + T_OUT_YAW_L]) / TELEM_ANGLE_SCALE
        telem_out_roll = s16(telem_buf[b + T_OUT_ROLL_H],
                             telem_buf[b + T_OUT_ROLL_L]) / TELEM_ANGLE_SCALE

        telem_mode = telem_buf[b + T_MODE_INDEX]
        telem_state = telem_buf[b + T_STATE_INDEX]
        telem_flags = telem_buf[b + T_FLAGS_INDEX]

        # Latch the flight transitions from every valid UART frame. The camera
        # loop may skip a short LAUNCHED state while writing an image frame.
        if STATE_LAUNCHED <= telem_state <= STATE_DESCENT:
            flight_seen = True
        if telem_state == STATE_DESCENT and descent_seen_ms is None:
            descent_seen_ms = time.ticks_ms()

        if telem_mode <= MODE_BOTH:
            mode = telem_mode

        telem_idx += TELEM_FRAME_LEN
        got += 1
        append_telemetry_log(time.ticks_ms(), telem_frames + got)

    if telem_idx:
        telem_buf = telem_buf[telem_idx:]
        telem_idx = 0

    if len(telem_buf) > TELEM_BUF_LIMIT:
        print("[DEBUG] telem_buf overflow:", len(telem_buf))
        telem_buf = bytearray()
        telem_idx = 0

    now = time.ticks_ms()
    if got:
        telem_frames += got
        telem_last_ms = now
        telem_link = 1
        if not telem_ok_printed:
            print("telemetry ok")
            telem_ok_printed = True
    elif telem_link and (time.ticks_diff(now, telem_last_ms) > TELEM_TIMEOUT_MS):
        telem_link = 0

    update_telemetry_log(now)


# =========================
# 上行目标帧
# =========================
def send_target(dx, dy):
    global TX_SKIP_COUNT, TX_SLOW_COUNT

    if not uart.txdone():
        TX_SKIP_COUNT += 1
        return

    frame = bytearray([VISION_SOF,
                       (dx >> 8) & 0xFF, dx & 0xFF,
                       (dy >> 8) & 0xFF, dy & 0xFF,
                       1, VISION_EOF])
    frame.append(crc8_lsb(frame))

    t0 = time.ticks_ms()
    uart.write(frame)
    dt = time.ticks_diff(time.ticks_ms(), t0)

    if dt > TX_SLOW_MS:
        TX_SLOW_COUNT += 1
        print("[DEBUG] write slow:", dt, "ms, tx_skip:", TX_SKIP_COUNT)


def send_no_target():
    global TX_SKIP_COUNT, TX_SLOW_COUNT

    if not uart.txdone():
        TX_SKIP_COUNT += 1
        return

    frame = bytearray([VISION_SOF, 0, 0, 0, 0, 0, VISION_EOF])
    frame.append(crc8_lsb(frame))

    t0 = time.ticks_ms()
    uart.write(frame)
    dt = time.ticks_diff(time.ticks_ms(), t0)

    if dt > TX_SLOW_MS:
        TX_SLOW_COUNT += 1
        print("[DEBUG] write slow:", dt, "ms, tx_skip:", TX_SKIP_COUNT)


# =========================
# 时间域中值滤波
# =========================
MED_WIN = 5
cx_hist = []
cy_hist = []


def median_of(vals):
    s = sorted(vals)
    return s[len(s) // 2]


def temporal_median(cx, cy):
    cx_hist.append(cx)
    cy_hist.append(cy)
    if len(cx_hist) > MED_WIN:
        cx_hist.pop(0)
        cy_hist.pop(0)
    if len(cx_hist) < MED_WIN:
        return cx, cy
    return median_of(cx_hist), median_of(cy_hist)


# =========================
# 内录
# =========================
led = None
record_dir = None
record_path = None
record_name = None
record_stream = None
recording = False
record_stop_at = 0
record_done = False
record_start_ms = None
record_last_write_ms = None
record_last_sync_ms = None
telem_log_stream = None
telem_log_path = None
telem_log_start_ms = None
telem_log_last_sync_ms = None
telem_log_done = False


def next_telemetry_path(dir_path):
    base = dir_path.rstrip("/") if dir_path else "."
    n = 1
    while n < 100000:
        path = "%s/telem_%05d.csv" % (base, n)
        try:
            os.stat(path)
            n += 1
        except OSError:
            return path
    raise OSError("telemetry log file numbers exhausted")


def stop_telemetry_log():
    global telem_log_stream
    if telem_log_stream is not None:
        try:
            telem_log_stream.close()
        except Exception as e:
            print("[DEBUG] telem log close err:", e)
        telem_log_stream = None


def append_telemetry_log(now_ms, frame_number):
    global telem_log_stream, telem_log_path
    global telem_log_start_ms, telem_log_last_sync_ms, telem_log_done

    if not TELEM_LOG_ENABLE or record_dir is None or telem_log_done:
        return
    if telem_log_stream is None:
        if not (STATE_LAUNCHED <= telem_state <= STATE_DESCENT):
            return
        stream = None
        try:
            path = next_telemetry_path(record_dir)
            stream = open(path, "w")
            stream.write("t_ms,frame,mode,state,flags,gx_dps,gy_dps,gz_dps,"
                         "pitch_deg,yaw_deg,roll_deg,out_pitch_deg,"
                         "out_yaw_deg,out_roll_deg\n")
        except Exception as e:
            if stream is not None:
                try:
                    stream.close()
                except Exception:
                    pass
            telem_log_done = True
            print("[DEBUG] telem log open err:", e)
            return
        telem_log_stream = stream
        telem_log_path = path
        telem_log_start_ms = now_ms
        telem_log_last_sync_ms = now_ms
        print("telemetry log start:", path)

    try:
        telem_log_stream.write(
            "%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n"
            % (time.ticks_diff(now_ms, start), frame_number,
               telem_mode, telem_state, telem_flags,
               telem_gyro_x, telem_gyro_y, telem_gyro_z,
               telem_pitch, telem_yaw, telem_roll,
               telem_out_pitch, telem_out_yaw, telem_out_roll))
    except Exception as e:
        print("[DEBUG] telem log write err:", e)
        stop_telemetry_log()
        telem_log_done = True


def update_telemetry_log(now_ms):
    global telem_log_last_sync_ms, telem_log_done
    if telem_log_stream is None:
        return

    if ((descent_seen_ms is not None and
         time.ticks_diff(now_ms, descent_seen_ms) >= DESCENT_RECORD_TAIL_MS) or
            time.ticks_diff(now_ms, telem_log_start_ms) >= MAX_RECORD_MS):
        stop_telemetry_log()
        telem_log_done = True
        print("telemetry log stop")
        return

    if time.ticks_diff(now_ms, telem_log_last_sync_ms) >= TELEM_LOG_SYNC_MS:
        try:
            telem_log_stream.flush()
            telem_log_last_sync_ms = now_ms
        except Exception as e:
            print("[DEBUG] telem log flush err:", e)
            stop_telemetry_log()
            telem_log_done = True


def pick_record_dir():
    # QVGA RGB565 uses about 154 KB/frame. Record to a writable SD card.
    for d in ("/sdcard", "/sd"):
        try:
            os.listdir(d)
            probe = d + "/.record_probe"
            with open(probe, "wb") as f:
                f.write(b"ok")
            os.remove(probe)
            return d
        except Exception:
            continue
    return None


def next_record_path(dir_path):
    base = dir_path.rstrip("/") if dir_path else "."
    n = 1
    while n < 100000:
        name = "%s/rec_%05d.bin" % (base, n)
        try:
            os.stat(name)
            n += 1
        except OSError:
            return name
    raise OSError("record file numbers exhausted")


def start_recording():
    global record_path, record_name, record_stream
    global recording, record_stop_at, record_start_ms
    global record_last_write_ms, record_last_sync_ms

    if not RECORD_ENABLE or record_dir is None:
        return

    path = next_record_path(record_dir)
    try:
        stream = image.ImageIO(path, "w")
    except Exception as e:
        print("[DEBUG] record open fail:", e)
        return

    record_stream = stream
    record_path = path
    record_name = path.split("/")[-1]
    recording = True
    record_stop_at = 0
    record_start_ms = time.ticks_ms()
    record_last_write_ms = None
    record_last_sync_ms = record_start_ms

    if led is not None:
        led.on()

    print("record start:", path)


def stop_recording():
    global record_stream, recording

    if record_stream is not None:
        try:
            record_stream.close()
        except Exception as e:
            print("[DEBUG] record close err:", e)

    record_stream = None
    recording = False

    if led is not None:
        led.off()

    print("record stop")


def update_recording(img):
    global recording, record_stop_at, record_done
    global record_last_write_ms, record_last_sync_ms

    if not RECORD_ENABLE:
        return

    if not recording:
        if (not record_done) and flight_seen:
            start_recording()
        if not recording:
            return

    if record_stream is None:
        recording = False
        return

    now = time.ticks_ms()
    if descent_seen_ms is not None:
        record_stop_at = time.ticks_add(descent_seen_ms,
                                        DESCENT_RECORD_TAIL_MS)
    if ((record_stop_at and time.ticks_diff(now, record_stop_at) >= 0) or
            time.ticks_diff(now, record_start_ms) >= MAX_RECORD_MS):
        stop_recording()
        record_done = True
        return

    if (RECORD_INTERVAL_MS > 0 and record_last_write_ms is not None and
            time.ticks_diff(now, record_last_write_ms) < RECORD_INTERVAL_MS):
        return
    try:
        # Append the native image and its inter-frame interval to the SD file.
        record_stream.write(img)
        record_last_write_ms = time.ticks_ms()
        if time.ticks_diff(record_last_write_ms,
                           record_last_sync_ms) >= RECORD_SYNC_MS:
            if hasattr(record_stream, "sync"):
                record_stream.sync()
            elif hasattr(os, "sync"):
                os.sync()
            record_last_sync_ms = record_last_write_ms
    except Exception as e:
        print("[DEBUG] record write err:", e)
        stop_recording()
        record_done = True


# =========================
# 传感器
# =========================
def init_sensor():
    for attempt in range(3):
        try:
            sensor.reset()
            sensor.set_pixformat(PIXFORMAT)
            sensor.set_framesize(FRAME_SIZE)
            sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
            sensor.set_windowing(BLOB_ROI)
            sensor.set_auto_gain(False, gain_db=20)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)

            time.sleep_ms(200)
            sensor.skip_frames(time=SKIP_FRAMES_MS)

            print("[DEBUG] sensor init OK, attempt", attempt + 1)
            return
        except Exception as e:
            print("[DEBUG] sensor init fail, attempt", attempt + 1, ":", e)
            time.sleep_ms(500)

    raise OSError("sensor init failed after 3 attempts")


def find_max_blob(img, roi):
    max_blob = BlobData()
    cnt = 0
    found_flag = 0

    blobs = img.find_blobs([THRESHOLDS1[0]], roi=roi)

    for blob in blobs:
        rect = blob.rect

        if cnt == 0:
            max_blob.x, max_blob.y, max_blob.w, max_blob.h = rect
            cnt += 1

        if cnt > 0:
            c_blob = BlobData(*rect)
            if c_blob.area() > max_blob.area():
                max_blob = c_blob

        found_flag = 1

    return found_flag, max_blob


def build_found_roi(blob):
    x1 = max(BLOB_ROI[0], blob.x - FOUND_RANGE_X)
    y1 = max(BLOB_ROI[1], blob.y - FOUND_RANGE_Y)
    x2 = min(BLOB_ROI[0] + BLOB_ROI[2],
             blob.x + blob.w + FOUND_RANGE_X)
    y2 = min(BLOB_ROI[1] + BLOB_ROI[3],
             blob.y + blob.h + FOUND_RANGE_Y)
    return (
        x1, y1, x2 - x1, y2 - y1
    )


# =========================
# 显示
# =========================
def draw_overlay(img, blob, found_flag, cur_roi, fps, t_ms,
                 link, mode_value, state, flags,
                 att_p, att_y, att_r,
                 gx, gy, gz,
                 op, oy, or_,
                 file_name, rx_frames):

    if found_flag == 1:
        img.draw_rectangle((blob.x, blob.y, blob.w, blob.h),
                           color=(255, 0, 0))
        img.draw_string((2, 18),
                        "Found! X=%d Y=%d" % (blob.cx(), blob.cy()),
                        color=(0, 255, 0), scale=1)

    img.draw_string((2, 2), "FPS:%.1f" % fps, color=(255, 0, 0), scale=1)
    img.draw_string((2, 10), "t=%d ms T:%d" % (t_ms, rx_frames),
                    color=(255, 255, 0), scale=1)

    if link:
        img.draw_string((2, 26),
                        "P:%.1f Y:%.1f R:%.1f" % (att_p, att_y, att_r),
                        color=(255, 255, 0), scale=1)
        img.draw_string((2, 34),
                        "GX:%.1f GY:%.1f GZ:%.1f" % (gx, gy, gz),
                        color=(255, 255, 0), scale=1)
        if flags & F_OUTPUT_VALID:
            img.draw_string((2, 42),
                            "OP:%.1f OY:%.1f OR:%.1f" % (op, oy, or_),
                            color=(255, 255, 0), scale=1)
        else:
            img.draw_string((2, 42), "OP:-- OY:-- OR:--",
                            color=(255, 255, 0), scale=1)
        img.draw_string((2, 50),
                        "M:%s S:%s L:1 V:%d"
                        % (mode_name(mode_value), state_name(state),
                           1 if (flags & F_LINK_VALID) else 0),
                        color=(255, 255, 0), scale=1)
    else:
        img.draw_string((2, 26), "P:-- Y:-- R:--",
                        color=(255, 255, 0), scale=1)
        img.draw_string((2, 34), "GX:-- GY:-- GZ:--",
                        color=(255, 255, 0), scale=1)
        img.draw_string((2, 42), "OP:-- OY:-- OR:--",
                        color=(255, 255, 0), scale=1)
        img.draw_string((2, 50),
                        "M:%s S:-- L:0" % mode_name(mode_value),
                        color=(255, 255, 0), scale=1)

    if file_name:
        img.draw_string((2, 58), file_name, color=(255, 255, 0), scale=1)

    img.draw_rectangle(cur_roi, color=(0, 0, 255))


# =========================
# 主程序
# =========================
init_sensor()
print("init successful")
clock = time.clock()

try:
    led = sensor.LED("LED_RED")
except Exception:
    led = None

mode = wait_for_handshake()
print("entered main loop, mode =", mode)
telem_buf = bytearray()
telem_idx = 0

record_dir = pick_record_dir() if (RECORD_ENABLE or TELEM_LOG_ENABLE) else None
print("RECORD_DIR =", record_dir)

start = time.ticks_ms()
frame_cnt = 0
gc_counter = 0
cur_roi = INIT_ROI

try:
    while True:
        clock.tick()
        poll_telemetry()

        t_ms = time.ticks_diff(time.ticks_ms(), start)

        try:
            img = sensor.snapshot()
        except Exception as e:
            print("[DEBUG] snapshot err:", e)
            time.sleep_ms(10)
            continue

        frame_cnt += 1

        found_flag, max_blob = find_max_blob(img, cur_roi)

        if found_flag == 1:
            cur_roi = build_found_roi(max_blob)
        else:
            cur_roi = INIT_ROI

        if found_flag == 1:
            cx = max_blob.cx()
            cy = max_blob.cy()
            cx_f, cy_f = temporal_median(cx, cy)

            dx = cx_f - GUIDE_CX
            dy = cy_f - GUIDE_CY

            # 下位机模式：pitch 只控 pitch，yaw 只控 yaw
            if mode == MODE_PITCH:
                dx = 0
            elif mode == MODE_YAW:
                dy = 0

            send_target(dx, dy)
            draw_found_flag = 1
        else:
            cx_hist.clear()
            cy_hist.clear()
            send_no_target()
            draw_found_flag = 0

        fps = clock.fps()

        draw_overlay(img, max_blob, draw_found_flag, cur_roi, fps, t_ms,
                     telem_link, mode, telem_state, telem_flags,
                     telem_pitch, telem_yaw, telem_roll,
                     telem_gyro_x, telem_gyro_y, telem_gyro_z,
                     telem_out_pitch, telem_out_yaw, telem_out_roll,
                     record_name if recording else None, telem_frames)

        # 录当前帧（在绘制之后，视频里带叠加信息）
        update_recording(img)

        gc_counter += 1
        if gc_counter >= 3:
            gc.collect()
            gc_counter = 0

except KeyboardInterrupt:
    print("[DEBUG] KeyboardInterrupt")

finally:
    if recording:
        stop_recording()
    stop_telemetry_log()

    if led is not None:
        led.off()

    print("frames:", frame_cnt,
          "tx_skip:", TX_SKIP_COUNT,
          "tx_slow:", TX_SLOW_COUNT)

    if record_path:
        print("record:", record_path)
    if telem_log_path:
        print("telemetry log:", telem_log_path)
