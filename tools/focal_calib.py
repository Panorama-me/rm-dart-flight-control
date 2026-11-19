#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标定相机焦距：识别绿色目标的质心，用已知真实角度反推焦距。

为什么要单独标：飞行数据里目标一直在动，任何 d(rel)/d(yaw) 都会被目标运动污染，
两种算法能差出一倍（k=0.70 与 1.52）。只有「目标静止 + 角度已知」的台架数据
才能定死焦距。

识别部分刻意复刻 main.py 的 LAB 阈值（默认 THRESHOLDS1 = (9,100,-102,-25,0,127)），
所以量到的是「OpenMV 会看到什么」，不是「人眼觉得哪里绿」。

用法
----
单张（已知真实角度，立刻给出建议焦距）:

    python tools/focal_calib.py calib/left.jpg --angle -9.46

多张（不同角度，最小二乘拟合，最稳）:

    python tools/focal_calib.py calib/ --angles calib.csv
    # calib.csv 两列: 文件名,真实角度(度)
    # left.jpg,-9.46
    # center.jpg,0
    # right.jpg,9.46

只量质心（不给角度，用来看某一帧目标在哪）:

    python tools/focal_calib.py rec_00012.mjpeg --frame 40

录像逐帧导出质心（供后续和遥测对齐）:

    python tools/focal_calib.py rec_00012.mjpeg --dump cx.csv
"""

import argparse
import csv
import math
import os
import sys
from io import BytesIO

try:
    from PIL import Image
except ImportError:
    sys.exit("需要 Pillow：python -m pip install pillow")

SOI = b"\xff\xd8\xff"

# 与 ctl_task.h 一致
IMG_W, IMG_H = 320, 240
GUIDE_CX, GUIDE_CY = 160.0, 115.0
FOCAL_CODE = 480.0
THRESHOLD_DEFAULT = (9, 100, -102, -25, 0, 127)


def split_mjpeg(data):
    """按 SOI/EOI 切分裸 MJPEG 流（与 tools/mjpeg_player.py 同逻辑）。"""
    starts = []
    i = 0
    while True:
        j = data.find(SOI, i)
        if j < 0:
            break
        starts.append(j)
        i = j + 3
    frames = []
    for k, s in enumerate(starts):
        e = starts[k + 1] if k + 1 < len(starts) else len(data)
        chunk = data[s:e]
        p = chunk.rfind(b"\xff\xd9")
        if p > 0:
            chunk = chunk[:p + 2]
        if len(chunk) > 4:
            frames.append(chunk)
    return frames


# sRGB -> 线性，查表避免逐像素做 pow
_LIN = []
for _i in range(256):
    _c = _i / 255.0
    _LIN.append(_c / 12.92 if _c <= 0.04045 else ((_c + 0.055) / 1.055) ** 2.4)


def rgb_to_lab(r, g, b):
    """D65 白点，输出 L in 0..100, a/b in -128..127，与 OpenMV 的量纲一致。"""
    rl, gl, bl = _LIN[r], _LIN[g], _LIN[b]
    x = (0.4124 * rl + 0.3576 * gl + 0.1805 * bl) / 0.95047
    y = 0.2126 * rl + 0.7152 * gl + 0.0722 * bl
    z = (0.0193 * rl + 0.1192 * gl + 0.9505 * bl) / 1.08883

    def f(t):
        return t ** (1.0 / 3.0) if t > 0.008856 else 7.787 * t + 16.0 / 116.0

    fx, fy, fz = f(x), f(y), f(z)
    return 116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz)


def find_green(img, thresh=None, min_pixels=8):
    """返回 (cx, cy, count)。img 统一为 320x240 RGB。"""
    if thresh is None:
        thresh = THRESHOLD_DEFAULT
    l0, l1, a0, a1, b0, b1 = thresh
    px = img.load()
    sx = sy = n = 0
    for y in range(IMG_H):
        for x in range(IMG_W):
            r, g, b = px[x, y][:3]
            # 粗筛：绿色分量必须明显高，省掉大部分像素的 LAB 计算
            if g < 40 or g <= r or g <= b:
                continue
            L, A, B = rgb_to_lab(r, g, b)
            if l0 <= L <= l1 and a0 <= A <= a1 and b0 <= B <= b1:
                sx += x
                sy += y
                n += 1
    if n < min_pixels:
        return None
    return sx / n, sy / n, n


def to_320(img):
    if img.size != (IMG_W, IMG_H):
        img = img.convert('RGB').resize((IMG_W, IMG_H), Image.BILINEAR)
    else:
        img = img.convert('RGB')
    return img


def load_image(path):
    if path.lower().endswith('.mjpeg') or path.lower().endswith('.bin'):
        return None
    return to_320(Image.open(path))


def load_frame(path, index):
    data = open(path, 'rb').read()
    frames = split_mjpeg(data)
    if not frames:
        return None, 0
    index = max(0, min(index, len(frames) - 1))
    return to_320(Image.open(BytesIO(frames[index]))), len(frames)


def angle_of(cx, focal):
    """像素 -> 角度（度）。未乘 CAMERA_YAW_SIGN，标定只看幅值。"""
    return math.degrees(math.atan((cx - GUIDE_CX) / focal))


def suggest_focal(cx, true_deg):
    dt = math.tan(math.radians(true_deg))
    if abs(dt) < 1e-6:
        return None
    return (cx - GUIDE_CX) / dt


def report(cx, cy, n, true_deg=None, focal=FOCAL_CODE):
    print('  质心 (%.1f, %.1f)  有效像素 %d' % (cx, cy, n))
    print('  当前焦距 %.0f px 下角度 = %+.2f 度' % (focal, angle_of(cx, focal)))
    if true_deg is not None:
        f_new = suggest_focal(cx, true_deg)
        if f_new is None:
            print('  真实角度为 0，无法反推焦距（这是中心基准点）')
        else:
            print('  真实角度 %+.2f 度  ->  建议焦距 %.0f px（当前 %.0f，比例 %.3f）'
                  % (true_deg, f_new, focal, f_new / focal))


def fit_focal(samples):
    """cx = GUIDE_CX + f * tan(theta) 的最小二乘解。"""
    num = den = 0.0
    for cx, th in samples:
        t = math.tan(math.radians(th))
        num += t * (cx - GUIDE_CX)
        den += t * t
    if den < 1e-9:
        return None
    return num / den


def _interp(ts, vs, tq):
    if tq <= ts[0]:
        return vs[0]
    if tq >= ts[-1]:
        return vs[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= tq:
            lo = mid
        else:
            hi = mid
    span = ts[hi] - ts[lo]
    if span <= 0:
        return vs[lo]
    w = (tq - ts[lo]) / span
    return vs[lo] * (1 - w) + vs[hi] * w


def _corr(a, b):
    n = len(a)
    if n < 5:
        return 0.0
    ma = sum(a) / n
    mb = sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((y - mb) ** 2 for y in b))
    if sa < 1e-9 or sb < 1e-9:
        return 0.0
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def fit_from_telem(samples, telem_path, fps_lo=14.0, fps_hi=45.0):
    """录像 + 遥测联合标定。

    目标静止时，像素位移与机头转角满足 d(cx) = -f * d(yaw)。
    录像没有时间戳，所以先搜索 (fps, offset) 让 cx 与 yaw 相关性最大，
    再回归 cx ~ yaw，斜率换算成焦距。

    samples: [(帧序号, cx)]，帧序号用于保持时间轴。
    """
    rows = list(csv.DictReader(open(telem_path, encoding='utf-8', errors='replace')))
    ts, yaws = [], []
    for r in rows:
        try:
            ts.append(float(r['t_ms']) / 1000.0)
            yaws.append(float(r['yaw_deg']))
        except (ValueError, KeyError):
            continue
    if len(ts) < 20 or len(samples) < 10:
        return None

    idxs = [s[0] for s in samples]
    cxs = [s[1] for s in samples]
    span = (idxs[-1] - idxs[0])

    best = None
    fps = fps_lo
    while fps <= fps_hi + 1e-9:
        dt = 1.0 / fps
        off = max(0.0, ts[0] - 0.5)
        while off + span * dt <= ts[-1]:
            ys = [_interp(ts, yaws, off + (i - idxs[0]) * dt) for i in idxs]
            c = _corr(cxs, ys)
            if best is None or abs(c) > abs(best[0]):
                best = (c, fps, off)
            off += 0.02
        fps += 0.5

    if best is None:
        return None

    c, fps, off = best
    dt = 1.0 / fps
    ys = [_interp(ts, yaws, off + (i - idxs[0]) * dt) for i in idxs]
    mx = sum(cxs) / len(cxs)
    my = sum(ys) / len(ys)
    sxx = sum((y - my) ** 2 for y in ys)
    if sxx < 1e-9:
        return None
    slope = sum((x - mx) * (y - my) for x, y in zip(cxs, ys)) / sxx
    pred = [my + slope * (y - my) for y in ys]
    ss_res = sum((x - p) ** 2 for x, p in zip(cxs, pred))
    ss_tot = sum((x - mx) ** 2 for x in cxs)
    r2 = 1 - ss_res / max(ss_tot, 1e-9)
    f = -slope / math.pi * 180.0
    return {'corr': c, 'fps': fps, 'offset': off, 'slope': slope,
            'r2': r2, 'focal': f}


def main():
    ap = argparse.ArgumentParser(description='识别绿色目标质心，标定相机焦距')
    ap.add_argument('path', help='图片 / 图片目录 / .mjpeg')
    ap.add_argument('--angle', type=float, help='该图的真实角度（度），用于反推焦距')
    ap.add_argument('--angles', help='CSV：文件名,真实角度（批量拟合用）')
    ap.add_argument('--frame', type=int, default=0, help='mjpeg 取第几帧（默认 0）')
    ap.add_argument('--dump', help='mjpeg 逐帧质心导出到 CSV')
    ap.add_argument('--telem', help='配套遥测 CSV；给出则用「机头转角 vs 像素位移」联合标定')
    ap.add_argument('--focal', type=float, default=480.0, help='当前焦距（默认 480）')
    ap.add_argument('--center', type=float, default=160.0, help='画面中心 x（默认 160）')
    ap.add_argument('--thresh', help='LAB 阈值，逗号分隔，如 9,100,-102,-25,0,127')
    ap.add_argument('--stride', type=int, default=1, help='mjpeg 逐帧导出时的步长')
    args = ap.parse_args()

    global GUIDE_CX, FOCAL_CODE
    GUIDE_CX = args.center
    FOCAL_CODE = args.focal

    thresh = THRESHOLD_DEFAULT
    if args.thresh:
        thresh = tuple(int(v) for v in args.thresh.split(','))
        if len(thresh) != 6:
            sys.exit('--thresh 需要 6 个数')

    # --- mjpeg + 遥测联合标定 ---
    if args.telem:
        data = open(args.path, 'rb').read()
        frames = split_mjpeg(data)
        if not frames:
            sys.exit('没有解析到 JPEG 帧')
        samples = []
        for i in range(0, len(frames), max(1, args.stride)):
            img = to_320(Image.open(BytesIO(frames[i])))
            r = find_green(img, thresh)
            if r is not None:
                samples.append((i, r[0]))
        print('%s: %d 帧，识别到目标 %d 帧' % (
            os.path.basename(args.path), len(frames), len(samples)))
        if len(samples) < 10:
            sys.exit('识别帧太少，换一段录像或调 --thresh')

        res = fit_from_telem(samples, args.telem)
        if res is None:
            sys.exit('拟合失败（遥测太短或 yaw 无变化）')

        print('  最佳对齐: %.1f fps, 起点偏移 %.2f s, 相关 %.3f' % (
            res['fps'], res['offset'], res['corr']))
        print('  回归: d(cx)/d(yaw) = %.2f px/度   R2 = %.3f' % (res['slope'], res['r2']))
        print('  -> 焦距 %.0f px（当前 %.0f，比例 %.3f）' % (
            res['focal'], FOCAL_CODE, res['focal'] / FOCAL_CODE))
        print()
        if res['r2'] < 0.3 or res['focal'] < 100 or res['focal'] > 3000:
            print('  结论：这段数据标不出来。')
            print('  原因通常是目标在画面里几乎不动（机头一直跟着它），')
            print('  缺少「机头转动 -> 像素位移」的对应关系。')
            print('  请上台架：目标固定，机头缓慢左右各转 20 度以上再录。')
            return
        if res['r2'] > 0.85:
            print('  可信：像素位移基本由机头转动解释，目标近似静止。')
        elif res['r2'] > 0.5:
            print('  存疑：目标有运动，焦距估计可能偏（回归会被稀释）。建议台架静止目标复测。')
        else:
            print('  不可用：目标运动或对齐失败主导，这次估计不能采信。')
        print('  把 ctl_task.h 的 DART_CAMERA_FOCAL_X_PX / Y_PX 改成上面的值，')
        print('  同时把 KP/KD 乘以 %.3f 保持等效增益不变。' % (
            FOCAL_CODE / max(res['focal'], 1e-6)))
        return

    # --- mjpeg 逐帧导出 ---
    if args.dump:
        data = open(args.path, 'rb').read()
        frames = split_mjpeg(data)
        if not frames:
            sys.exit('没有解析到 JPEG 帧')
        out = csv.writer(open(args.dump, 'w', newline='', encoding='utf-8'))
        out.writerow(['frame', 'cx', 'cy', 'pixels', 'angle_deg_at_%d' % int(FOCAL_CODE)])
        hit = 0
        for i in range(0, len(frames), max(1, args.stride)):
            img = to_320(Image.open(BytesIO(frames[i])))
            r = find_green(img, thresh)
            if r is None:
                out.writerow([i, '', '', 0, ''])
                continue
            cx, cy, n = r
            hit += 1
            out.writerow([i, '%.1f' % cx, '%.1f' % cy, n, '%.2f' % angle_of(cx, FOCAL_CODE)])
        print('共 %d 帧，识别到目标 %d 帧 -> %s' % (len(frames), hit, args.dump))
        return

    # --- mjpeg 单帧 ---
    if args.path.lower().endswith(('.mjpeg', '.bin')):
        img, total = load_frame(args.path, args.frame)
        if img is None:
            sys.exit('读不到帧')
        print('%s  第 %d/%d 帧' % (os.path.basename(args.path), args.frame, total))
        r = find_green(img, thresh)
        if r is None:
            sys.exit('这一帧没有识别到绿色目标（可试 --thresh 调整）')
        report(*r, true_deg=args.angle, focal=FOCAL_CODE)
        return

    # --- 批量：目录 + CSV ---
    if args.angles:
        rows = [r for r in csv.reader(open(args.angles, encoding='utf-8')) if len(r) >= 2]
        samples = []
        print('%-22s %8s %8s %10s %10s' % ('file', 'cx', 'pixels', '角度(code)', '建议焦距'))
        for name, ang in rows:
            name = name.strip()
            ang = float(ang)
            p = name if os.path.isabs(name) else os.path.join(args.path, name)
            img = load_image(p)
            if img is None:
                print('%-22s  跳过（不是图片）' % name)
                continue
            r = find_green(img, thresh)
            if r is None:
                print('%-22s  未识别到目标' % name)
                continue
            cx, cy, n = r
            f_new = suggest_focal(cx, ang) if abs(ang) > 1e-6 else None
            print('%-22s %8.1f %8d %10.2f %10s' % (
                name, cx, n, angle_of(cx, FOCAL_CODE),
                '%.0f' % f_new if f_new else '-'))
            if f_new and f_new > 50:
                samples.append((cx, ang))
        if samples:
            f = fit_focal(samples)
            print()
            print('最小二乘拟合：焦距 = %.0f px（当前 %.0f，比例 %.3f）' % (
                f, FOCAL_CODE, f / FOCAL_CODE))
            print('把 ctl_task.h 的 DART_CAMERA_FOCAL_X_PX / Y_PX 改成这个值，')
            print('同时把 KP/KD 乘以 %.3f 保持等效增益不变。' % (FOCAL_CODE / f))
        else:
            print('\n没有可用样本（角度为 0 的点只作中心基准，不参与拟合）')
        return

    # --- 单张图片 ---
    img = load_image(args.path)
    if img is None:
        sys.exit('不支持的输入')
    print(os.path.basename(args.path))
    r = find_green(img, thresh)
    if r is None:
        sys.exit('没有识别到绿色目标（可试 --thresh 调整）')
    report(*r, true_deg=args.angle, focal=FOCAL_CODE)


if __name__ == '__main__':
    main()
