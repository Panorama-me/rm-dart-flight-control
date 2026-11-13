# -*- coding: utf-8 -*-
"""
OpenMV 端编码 / 写卡耗时实测（台架脚本，不依赖 STM32）

用途
    决定录像该用哪种存法。核心未知量是 JPEG 编码到底多贵——
    如果 `mjpeg.Mjpeg.write()` 比现在直接写原始帧还慢，换格式就没意义。

怎么跑
    1. 插好 FAT32 SD 卡
    2. 在 OpenMV IDE 里打开本文件，直接运行（不需要 main.py 在跑）
    3. 等它打印完表格，把结果贴回来

输出什么
    每种方案的「每帧耗时 / 上限帧率 / 单帧字节 / 带宽」，最后给一句结论。

注意
    脚本会在 SD 卡根目录建 `.bench_*` 临时文件，跑完自动删除。
"""

import gc
import os
import time

import image
import sensor

try:
    import mjpeg
except ImportError:
    mjpeg = None      # 固件没带该模块时跳过 MJPEG 测试，其余照跑

# =========================
# 参数
# =========================
FRAMES = 30              # 每种方案测多少帧
WARMUP = 5               # 预热帧数，丢掉（首帧总是偏慢）
QUALITIES = (50, 60, 75)  # 要对比的 JPEG 质量

RAW_BYTES = 320 * 240 * 2  # QVGA RGB565 单帧字节数


# =========================
# 工具
# =========================
def pick_dir():
    """找一个可写的 SD 卡目录，和 main.py 的 pick_record_dir 同逻辑。"""
    for d in ("/sdcard", "/sd"):
        try:
            os.listdir(d)
            probe = d + "/.bench_probe"
            with open(probe, "wb") as f:
                f.write(b"ok")
            os.remove(probe)
            return d
        except Exception:
            continue
    return None


def cleanup(path):
    try:
        os.remove(path)
    except Exception:
        pass


def bench(name, frames, fn, bytes_per_frame=None):
    """跑 frames 次 fn()，返回每帧平均毫秒数。

    fn() 内部负责写卡；bytes_per_frame 用于算带宽，None 表示不统计。
    """
    gc.collect()
    t0 = time.ticks_us()
    for _ in range(frames):
        fn()
    dt_us = time.ticks_diff(time.ticks_us(), t0)
    ms = dt_us / 1000.0 / frames

    fps = 1000.0 / ms if ms > 0 else 0.0
    line = "  %-34s %7.1f ms/frame   %5.1f fps" % (name, ms, fps)
    if bytes_per_frame:
        kb = bytes_per_frame / 1024.0
        mb_s = bytes_per_frame * fps / 1024.0 / 1024.0
        line += "   %7.1f KB/frame  %5.2f MB/s" % (kb, mb_s)
    print(line)
    return ms


# =========================
# 主流程
# =========================
def main():
    print("")
    print("=" * 78)
    print("OpenMV 编码 / 写卡耗时实测")
    print("=" * 78)

    sd = pick_dir()
    if sd is None:
        print("!! 找不到可写的 SD 卡（/sdcard 或 /sd），无法测写入。")
        print("   只能测纯编码耗时，结果仅供参考。")
    else:
        print("SD 目录:", sd)

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=1500)

    print("相机: QVGA RGB565，原始帧 %d 字节 (%.1f KB)"
          % (RAW_BYTES, RAW_BYTES / 1024.0))
    print("每种方案 %d 帧，预热 %d 帧" % (FRAMES, WARMUP))

    print("")
    print("预热中...")
    for _ in range(WARMUP):
        sensor.snapshot()

    results = {}

    # ---------- [1] 基线：只 snapshot ----------
    print("")
    print("[1] 基线（只 sensor.snapshot()，作为对照下限）")
    gc.collect()
    t0 = time.ticks_us()
    for _ in range(FRAMES):
        sensor.snapshot()
    base_ms = time.ticks_diff(time.ticks_us(), t0) / 1000.0 / FRAMES
    print("  %-34s %7.1f ms/frame   %5.1f fps" % ("snapshot", base_ms, 1000.0 / base_ms))
    results["snapshot"] = base_ms

    # 基线跑完再取固定样本：snapshot 可能复用帧缓冲，提前取会被覆盖。
    # 后续所有测试都不再调用 snapshot，所以这一帧能一直用。
    sample = sensor.snapshot()

    # ---------- [2] 当前方案：ImageIO 原始帧 ----------
    if sd:
        print("")
        print("[2] 当前方案：image.ImageIO.write(img)  原始 RGB565")
        path = sd + "/.bench_raw.bin"
        cleanup(path)
        stream = image.ImageIO(path, "w")

        def _raw():
            stream.write(sample)

        results["raw"] = bench("ImageIO.write(原始)", FRAMES, _raw, RAW_BYTES)
        stream.close()
        size = os.stat(path)[6]
        print("  %-34s %7.1f KB/frame（实测文件大小）"
              % ("", size / 1024.0 / FRAMES))
        cleanup(path)

    # ---------- [3] 纯 JPEG 编码 ----------
    print("")
    print("[3] 纯编码：img.compress(quality=Q)，不写卡")
    for q in QUALITIES:
        def _enc(q=q):
            sample.compress(quality=q)
        ms = bench("compress(q=%d)" % q, FRAMES, _enc)
        results["enc_%d" % q] = ms

    # ---------- [4] MJPEG 编码 + 写卡 ----------
    if sd and mjpeg is not None:
        print("")
        print("[4] 方案 A：mjpeg.Mjpeg.write(img, quality=Q)  编码+写卡")
        for q in QUALITIES:
            path = sd + "/.bench_q%d.mjpeg" % q
            cleanup(path)
            out = mjpeg.Mjpeg(path)

            def _mj(q=q, out=out):
                out.write(sample, quality=q)

            ms = bench("Mjpeg.write(q=%d)" % q, FRAMES, _mj)
            out.close()
            size = os.stat(path)[6]
            per = size / FRAMES
            print("  %-34s %7.1f KB/frame  %5.2f MB/s（按实测大小）"
                  % ("", per / 1024.0, per * (1000.0 / ms) / 1024.0 / 1024.0))
            results["mjpeg_%d" % q] = ms
            cleanup(path)

    # ---------- [5] 方案 B：缩到 QQVGA 再存原始 ----------
    # 不同固件缩放 API 不同（mean_pool / scale），缺了就跳过，不影响其他组。
    if sd:
        print("")
        print("[5] 方案 B：缩到 QQVGA 再写原始帧")
        small = None
        for meth in ("mean_pool", "scale"):
            if hasattr(sample, meth):
                try:
                    small = getattr(sample, meth)(2, 2) if meth == "mean_pool" \
                        else getattr(sample, meth)(x_size=160, y_size=120)
                    print("  使用 %s() 缩放" % meth)
                    break
                except Exception as e:
                    print("  %s() 失败: %s" % (meth, e))
                    small = None
        if small is None:
            print("  跳过：本固件没有可用的缩放接口")
        else:
            path = sd + "/.bench_small.bin"
            cleanup(path)
            stream = image.ImageIO(path, "w")

            def _small(img=small):
                stream.write(img)

            small_bytes = 160 * 120 * 2
            results["small"] = bench("缩放 + ImageIO.write", FRAMES,
                                     _small, small_bytes)
            stream.close()
            cleanup(path)

    if mjpeg is None:
        print("")
        print("[4] 跳过：本固件没有 mjpeg 模块")

    # ---------- 结论 ----------
    print("")
    print("=" * 78)
    print("结论")
    print("=" * 78)

    print("  下面各数字都【只算存储环节】，不含 sensor.snapshot()。")
    print("  snapshot 单独耗时 %.1f ms，是任何方案都要付的固定成本。" % base_ms)
    print("")

    raw_ms = results.get("raw")
    if raw_ms:
        print("  当前 ImageIO 原始帧: %.1f ms/帧（对照基准）" % raw_ms)

    for q in QUALITIES:
        ms = results.get("mjpeg_%d" % q)
        if not ms:
            continue
        verdict = "值得换" if ms < 25 else ("可换，但收益有限" if ms < 40 else "不划算")
        print("  MJPEG q=%d 每帧 %.1f ms → %s" % (q, ms, verdict))

    small_ms = results.get("small")
    if small_ms:
        verdict = "值得换" if small_ms < 25 else "不划算"
        print("  QQVGA 原始每帧 %.1f ms → %s" % (small_ms, verdict))

    print("")
    print("判据（看 MJPEG 的存储耗时，与上面的 ImageIO 原始帧直接对比）：")
    print("  比 ImageIO 原始帧更短  → 换 MJPEG，白得一个能直接播的文件")
    print("  略长 5~15 ms           → 换 MJPEG + 录制帧率降到 10 fps 仍划算")
    print("  长 15 ms 以上          → 不划算，走 QQVGA 原始 或 直接降帧率")
    print("")


main()
