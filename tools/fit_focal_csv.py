#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从标定 CSV 拟合焦距。

配合 tools/calib_main.py 使用：那份副本在遥测 CSV 末尾追加了 vis_dx / vis_dy。
台架上目标固定、机体转动时，像素位移只由机体转动引起：

    d(vis_dx) = -f * d(yaw_rad)      =>      f = -d(vis_dx)/d(yaw) * 57.2958

用法：
    python tools/fit_focal_csv.py 内录/xxx/telem_00001.csv
    python tools/fit_focal_csv.py calib.csv --focal 480
"""

import argparse
import csv
import glob
import math
import os
import sys


def mean(a):
    return sum(a) / len(a) if a else 0.0


def unwrap_deg(vals):
    """把 0~360 的绝对角展开成连续序列，避免跨界跳变。"""
    if not vals:
        return []
    out = [vals[0]]
    for v in vals[1:]:
        d = v - out[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(out[-1] + d)
    return out


def main():
    ap = argparse.ArgumentParser(description='从标定 CSV 求焦距')
    ap.add_argument('csv_path')
    ap.add_argument('--focal', type=float, default=480.0, help='当前焦距（默认 480）')
    ap.add_argument('--min-step', type=float, default=0.03,
                    help='相邻帧 dx 或 yaw 的最小变化（过滤冻结段）')
    ap.add_argument('--dump', help='把参与拟合的点导出到 CSV，便于画图')
    args = ap.parse_args()

    path = args.csv_path
    if os.path.isdir(path):
        cands = sorted(glob.glob(os.path.join(path, 'telem_*.csv')))
        if not cands:
            sys.exit('目录里没有 telem_*.csv：%s' % path)
        path = cands[-1]
        print('使用目录里最新的一份：%s' % os.path.basename(path))
    if not os.path.exists(path):
        sys.exit('找不到文件：%s\n'
                 '两点提示：\n'
                 '  1) 内录数据在仓库外面：rm_dart_test/内录/日期/telem_xxxxx.csv\n'
                 '  2) 中文路径用引号包住、给绝对路径最稳，例如：\n'
                 '     python tools/fit_focal_csv.py '
                 '"D:/Study/rmElectric/rm_dart_test/内录/11.19/telem_00012.csv"\n'
                 '  3) 也可以直接把整个目录传进来，自动取里面最新的一份\n'
                 % path)

    rows = list(csv.DictReader(open(path, encoding='utf-8', errors='replace')))
    if not rows:
        sys.exit('空文件')
    if 'vis_dx' not in rows[0]:
        sys.exit('这份 CSV 没有 vis_dx 列。\n'
                 '它必须是 tools/calib_main.py 记录的标定数据；\n'
                 '普通 main.py 记的飞行数据没有像素列，标不了焦距。')

    rec = []
    for r in rows:
        try:
            rec.append((float(r['t_ms']) / 1000.0,
                        float(r['yaw_deg']),
                        float(r['vis_dx']),
                        float(r['vis_dy']) if r.get('vis_dy') not in (None, '') else 0.0))
        except (ValueError, KeyError):
            continue
    if len(rec) < 30:
        sys.exit('有效行太少（%d），至少转 2 秒' % len(rec))

    yaw = unwrap_deg([r[1] for r in rec])

    pts = []
    for k in range(1, len(rec)):
        ddx = rec[k][2] - rec[k - 1][2]
        dyaw = yaw[k] - yaw[k - 1]
        if abs(ddx) < args.min_step or abs(dyaw) < args.min_step:
            continue
        if ddx * dyaw > 0:
            # 目标固定时 dx 与 yaw 必然反向；同向多半是目标在动或识别跳变
            continue
        pts.append((rec[k][0], rec[k][2], math.tan(math.radians(yaw[k]))))

    if len(pts) < 20:
        sys.exit('可用点太少（%d）。检查：目标是否固定？有没有真的转动机体 20 度以上？'
                 % len(pts))

    xs = [p[2] for p in pts]     # tan(yaw)，针孔模型 dx = -f*tan(yaw)
    ys = [p[1] for p in pts]
    mx, my = mean(xs), mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx < 1e-9:
        sys.exit('yaw 几乎没变，转动机体再录一次')
    slope = sxy / sxx
    pred = [my + slope * (x - mx) for x in xs]
    ss_res = sum((y - p) ** 2 for y, p in zip(ys, pred))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1 - ss_res / max(ss_tot, 1e-9)

    f = -slope
    span = math.degrees(math.atan(max(xs))) - math.degrees(math.atan(min(xs)))
    print('样本 %d 点，yaw 跨度 %.1f 度，dx 跨度 %.1f px' % (len(pts), span, max(ys) - min(ys)))
    print('回归 d(dx)/d(tan yaw) = %.1f px   R2 = %.3f' % (slope, r2))
    print()

    if r2 < 0.85:
        print('R2 偏低（<0.85），结果不可信。常见原因：')
        print('  - 目标没固定住，或者转的时候目标跟着动了')
        print('  - 转得太快，视觉帧率跟不上，采样点稀疏')
        print('  - 视觉丢帧多，vis_dx 出现冻结段')
        print()
    if f < 100 or f > 3000:
        print('焦距 %.0f px 明显不合理，检查符号或数据。' % f)
        return 1

    k = f / args.focal
    print('=> 建议焦距 %.0f px   （当前 %.0f）' % (f, args.focal))
    print('   比例 k = %.3f' % k)
    print()
    print('改 ctl_task.h：')
    print('  DART_CAMERA_FOCAL_X_PX  %.0f' % f)
    print('  DART_CAMERA_FOCAL_Y_PX  %.0f' % f)
    print()
    print('同步缩放 PID（保持等效增益）：')
    print('  KP = 14 x %.3f = %.1f' % (1 / k, 14 / k))
    print('  KD = 3.0 x %.3f = %.2f' % (1 / k, 3.0 / k))

    if args.dump:
        out = csv.writer(open(args.dump, 'w', newline='', encoding='utf-8'))
        out.writerow(['t_s', 'vis_dx', 'tan_yaw'])
        for p in pts:
            out.writerow(['%.3f' % p[0], '%.2f' % p[1], '%.2f' % p[2]])
        print('\n拟合点已导出：%s' % args.dump)
    return 0


if __name__ == '__main__':
    sys.exit(main())
