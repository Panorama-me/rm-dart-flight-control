# -*- coding: utf-8 -*-
"""
OpenMV 端主循环分段耗时实测（台架脚本，不依赖 STM32）

为什么要有这个脚本
    main.py 注释里的预算是「snapshot 25.4 + find_blobs/叠加/UART 17.9 +
    JPEG 4.6 = 47.8 ms -> 20.9 fps」，但那几个数是 bench_encode.py 在别的配置下
    测的，不是当前主循环（READOUT_ROI 1280x960、160x120 池化、OVERLAY_LEVEL 1）的实测。
    想加帧率，必须先知道这 48 ms 花在哪一格，否则改错地方只是白折腾。

怎么跑
    1. 插好 FAT32 SD 卡（写卡那一项需要）
    2. OpenMV IDE 里打开本文件直接运行（不需要 main.py 在跑）
    3. 把打印出来的表格贴回来

输出什么
    每一段的单帧耗时，以及「只优化这一段」时主循环会变成多少 ms / 多少 fps。

注意
    脚本会在 SD 根目录建 .bench_stage.mjpeg，跑完自动删除。
"""

import gc
import os
import time

import sensor

try:
    import mjpeg
except ImportError:
    mjpeg = None

# =========================
# 与 main.py 保持一致的参数
# =========================
FRAME_SIZE = sensor.QVGA
PIXFORMAT = sensor.RGB565
READOUT_ROI = (0, 0, 1280, 960)
BLOB_ROI = (0, 0, 320, 240)
INIT_ROI = (0, 0, 320, 240)
EXPOSURE_US = 300
SKIP_MS = 1500
THRESHOLDS1 = (9, 100, -102, -25, 0, 127)
QUALITY = 60          # RECORD_JPEG_QUALITY
POOL_DIV = 2          # RECORD_POOL_DIV

FRAMES = 20
WARMUP = 5
BENCH_PATH = "/sdcard/.bench_stage.mjpeg"


def measure(fn):
    for _ in range(WARMUP):
        fn()
    t0 = time.ticks_ms()
    for _ in range(FRAMES):
        fn()
    return time.ticks_diff(time.ticks_ms(), t0) / float(FRAMES)


def report(name, ms):
    print("%-26s %7.2f ms   -> %5.1f fps" % (name, ms, (1000.0 / ms) if ms > 0 else 0))


def find_blob(img, roi, y_stride=1):
    blobs = img.find_blobs([THRESHOLDS1], roi=roi, x_stride=2, y_stride=y_stride,
                           pixels_threshold=10, area_threshold=10)
    best = None
    area = 0
    for b in blobs:
        r = b.rect
        a = r[2] * r[3]
        if a > area:
            area = a
            best = b
    return best


def draw_like_main(img, blob, found):
    if found and blob is not None:
        img.draw_rectangle(blob.rect, color=(255, 0, 0))
    img.draw_string((2, 2), "FPS:20.0", color=(255, 0, 0), scale=1)
    img.draw_string((2, 12), "M:B S:D 20fps", color=(255, 255, 255), scale=1)


print("=== bench_stages ===")
sensor.reset()
sensor.set_pixformat(PIXFORMAT)
sensor.set_framesize(FRAME_SIZE)
try:
    sensor.set_framebuffers(1)
except Exception as e:
    print("set_framebuffers:", e)
try:
    sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
    print("readout window OK", READOUT_ROI)
except Exception as e:
    print("readout window 失败:", e)
try:
    sensor.set_windowing(BLOB_ROI)
    sensor.set_auto_gain(False, gain_db=20)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
except Exception as e:
    print("sensor 参数设置失败:", e)
time.sleep_ms(200)
sensor.skip_frames(time=SKIP_MS)

gc.collect()

print()
print("--- 分段 ---")
t_snap = measure(lambda: sensor.snapshot())
report("1 snapshot", t_snap)

img = sensor.snapshot()
t_blob = measure(lambda: find_blob(img, INIT_ROI, 1))
report("2 find_blobs 全屏 y1", t_blob)

t_blob2 = measure(lambda: find_blob(img, INIT_ROI, 2))
report("3 find_blobs 全屏 y2", t_blob2)

b = find_blob(img, INIT_ROI, 1)
roi_small = INIT_ROI
if b is not None:
    x, y, w, h = b.rect
    x1 = max(0, x - 40)
    y1 = max(0, y - 40)
    x2 = min(320, x + w + 40)
    y2 = min(240, y + h + 40)
    roi_small = (x1, y1, x2 - x1, y2 - y1)
print("   锁定 ROI =", roi_small, " 目标 =", "有" if b is not None else "无")
t_blob3 = measure(lambda: find_blob(img, roi_small, 1))
report("4 find_blobs 锁定ROI", t_blob3)

found = b is not None
t_draw = measure(lambda: draw_like_main(img, b, found))
report("5 overlay(L1)", t_draw)

if mjpeg is not None:
    try:
        st = mjpeg.Mjpeg(BENCH_PATH)
        img2 = img.copy()
        if POOL_DIV > 1:
            img2.mean_pool(POOL_DIV, POOL_DIV)

        t_enc = measure(lambda: st.write(img2, quality=QUALITY))
        st.close()
        report("6 编码+写卡", t_enc)
    except Exception as e:
        print("mjpeg 测试失败:", e)
        t_enc = 0.0
else:
    print("固件无 mjpeg，跳过编码测试")
    t_enc = 0.0

print()
print("--- 合成 ---")
full = t_snap + t_blob + t_draw + t_enc
report("现在(全屏扫描 y1)", full)
report("若 y_stride=2", t_snap + t_blob2 + t_draw + t_enc)
report("若锁定后小 ROI", t_snap + t_blob3 + t_draw + t_enc)
report("若同时 y2 + 小 ROI", t_snap + t_blob2 + t_blob3 - t_blob + t_draw + t_enc)

print()
print("读法：哪一项最大就先动哪一项")
print("  snapshot 最大   -> 传感器/读出窗口问题，别动检测")
print("  find_blobs 最大 -> 加 y_stride=2 或收缩 ROI")
print("  叠加最大        -> OVERLAY_LEVEL 降到 0（会丢录像里的调试信息）")
print("  编码+写卡最大   -> 降 JPEG 质量或继续池化")

try:
    os.remove(BENCH_PATH)
    print("临时文件已删除")
except Exception:
    pass
