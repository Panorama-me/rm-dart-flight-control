# OpenMV 端：把相机切成 USB 摄像头（UVC），供上位机实时标定使用。
#
# 用法
#   1. OpenMV IDE 打开本文件，点「运行」（不要断开 USB）
#   2. 电脑设备里会多出一个摄像头（名字通常是 OpenMV Cam）
#   3. 浏览器打开 tools/focal_calib.html，点「连接相机」
#
# !!! 本文件的传感器配置逐行对齐 main.py 的 init_sensor() !!!
#   标定的是「飞行时那套光学系统」，任何一项不一致，标出来的焦距就对不上：
#     READOUT_ROI    决定取景放大倍率 -> 直接决定焦距
#     auto_gain 关   固定 20 dB，否则画面亮度随环境变，LAB 阈值行为跟着变
#     auto_whitebal  关，否则色相漂移，绿色阈值判定不稳
#     auto_exposure  固定 100 us，同上
#   改 main.py 的这几项时，这里必须同步改。
#
# 前提
#   - 固件 >= 4.5.0（IDE 菜单 工具 -> 关于 可查版本）。低版本没有 UVC，
#     改用「图片模式」：IDE 里截图保存，再用页面上的文件选择。
#   - 运行本脚本时不要再跑 main.py，两者抢同一个 sensor。

import sensor, time

# ---- 与 main.py 保持一致 ----
READOUT_ROI = (0, 0, 1280, 960)   # main.py: READOUT_ROI
BLOB_ROI = (0, 0, 320, 240)       # main.py: BLOB_ROI
EXPOSURE_US = 100                 # main.py: EXPOSURE_US
GAIN_DB = 20                      # main.py: set_auto_gain(False, gain_db=20)
SKIP_MS = 1500                    # main.py: SKIP_FRAMES_MS


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    try:
        sensor.set_framebuffers(1)
    except Exception:
        pass
    sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
    sensor.set_windowing(BLOB_ROI)
    sensor.set_auto_gain(False, gain_db=GAIN_DB)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    time.sleep_ms(200)
    sensor.skip_frames(time=SKIP_MS)


init_sensor()
print("[CAL] sensor ready: QVGA RGB565, readout", READOUT_ROI, "window", BLOB_ROI)
print("[CAL] gain", GAIN_DB, "dB, exposure", EXPOSURE_US, "us")

# 持续采集；固件会把帧推给 USB 视频类
while True:
    sensor.snapshot()
