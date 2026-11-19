# OpenMV 端：把相机切成 USB 摄像头（UVC），供上位机实时标定使用。
#
# 用法
#   1. OpenMV IDE 打开本文件，点「运行」（不要断开 USB）
#   2. 电脑设备里会多出一个摄像头（名字通常是 OpenMV Cam）
#   3. 浏览器打开 tools/focal_calib.html，点「连接相机」
#
# 前提
#   - 固件 >= 4.5.0（IDE 菜单 工具 -> 关于 可查版本）。低版本没有 UVC，
#     只能改用「图片模式」：IDE 里截图保存，再用页面上的文件选择。
#   - 运行本脚本时不要再跑 main.py，两者抢同一个 sensor。
#
# 注意
#   这里刻意不做任何识别。识别放在 PC 上做，标定时才能随时改阈值、
#   看到质心像素，而不是被固件的判定逻辑挡住。

import sensor, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# 持续采集；固件会把帧推给 USB 视频类
while True:
    sensor.snapshot()
