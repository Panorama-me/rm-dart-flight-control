# -*- coding: utf-8 -*-
"""评估视觉视线角速率（vis_rate_dps）能否用于 PN 制导。

轴系与真值重建说明（2026-11-21 修订）：
  1. 时间基准优先使用 STM32 遥测帧号 `frame * 0.020 s`（50 Hz 固定节拍），
     避免 OpenMV 串口批量读出导致的 `t_ms` 1~3 ms / 60~100 ms 抖动把差分放大。
  2. 若 CSV 含 `vis_x_px` / `gz_dps`，直接用机体陀螺积分
     `lam_x = ∫(gz + gx*cam_pitch) dt - atan(vis_x_px / f)`（相机平面惯性视线角）
     做中心差分作为干净真值，不受旧固件 `los_yaw_deg` 欧拉 1/cos(pitch) 漂移污染；
     无 `vis_x_px` 的早期旧日志自动回落到 `los_yaw_deg`。
  3. 自动剔除锁定时段末尾已触发丢帧解锁、被置 0 的 `vis_rate_dps` 尾帧，
     避免解锁跳变污染抖动 p95 与 RMSE。

判据（中段飞行，排除末 0.12 s 近距 1/R 几何发散）：
  1. 大幅帧符号一致率（|ω|≥10 °/s）> 85%
  2. 中段 RMSE < 5 °/s
  3. 有效锁定段抖动 p95 < 5 °/s
  4. 控制 gz 后的 |偏相关| 或滞后对齐泄漏足够低

用法：
    python tools/eval_vis_rate.py <csv或目录> [--shots 00010,00023] [--focal 640]
"""
import argparse
import csv
import glob
import math
import os

COLS = ('vis_rate_dps', 'pn_omega_dps')      # 新固件优先，旧数据回落
STM32_TELEM_DT_S = 0.020
VISION_DELAY_S = 0.028                       # 20 ms 链路延迟 + 约 8 ms 遥测采样半周期


def pick_col(header):
    for c in COLS:
        if c in header:
            return c
    return None


def corr(x, y):
    n = len(x)
    if n < 5:
        return float('nan')
    mx, my = sum(x) / n, sum(y) / n
    sxx = sum((a - mx) ** 2 for a in x)
    syy = sum((b - my) ** 2 for b in y)
    if sxx < 1e-12 or syy < 1e-12:
        return float('nan')
    return sum((a - mx) * (b - my) for a, b in zip(x, y)) / math.sqrt(sxx * syy)


def partial_corr(x, y, z):
    """控制 z 后 x 与 y 的偏相关。"""
    rxy, rxz, ryz = corr(x, y), corr(x, z), corr(y, z)
    den = (1 - rxz ** 2) * (1 - ryz ** 2)
    if den <= 1e-9:
        return float('nan')
    return (rxy - rxz * ryz) / math.sqrt(den)


def interp1(xq, xs, ys):
    """简单线性插值（不依赖 numpy，便于独立运行）。"""
    if xq <= xs[0]:
        return ys[0]
    if xq >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= xq:
            lo = mid
        else:
            hi = mid
    dx = xs[hi] - xs[lo]
    if dx <= 1e-9:
        return ys[lo]
    return ys[lo] + (ys[hi] - ys[lo]) * (xq - xs[lo]) / dx


def analyze(path, focal=640.0, quiet=False):
    rows = list(csv.DictReader(open(path, encoding='utf-8', errors='replace')))
    if not rows:
        return None
    header = rows[0]
    col = pick_col(header)
    if col is None:
        if not quiet:
            print('%-6s 缺少速率列（需要 vis_rate_dps 或 pn_omega_dps）'
                  % os.path.basename(path)[6:-4])
        return None

    st5 = [r for r in rows if int(r['state']) == 5]
    if len(st5) < 15:
        return None

    use_frame = ('frame' in header)
    use_px = ('vis_x_px' in header and 'vis_y_px' in header and 'gz_dps' in header)

    if use_frame:
        fr0 = int(st5[0]['frame'])
        t_all = [(int(r['frame']) - fr0) * STM32_TELEM_DT_S for r in st5]
    else:
        t0 = float(st5[0]['t_ms']) / 1000.0
        t_all = [float(r['t_ms']) / 1000.0 - t0 for r in st5]

    w_all = [float(r[col]) for r in st5]
    gz_all = [float(r['gz_dps']) for r in st5]
    gx_all = [float(r['gx_dps']) for r in st5] if 'gx_dps' in header else [0.0] * len(st5)
    los_all = [float(r['los_yaw_deg']) for r in st5]
    yaw_all = [float(r['yaw_deg']) for r in st5]
    vx_all = [float(r['vis_x_px']) for r in st5] if use_px else [0.0] * len(st5)
    vy_all = [float(r['vis_y_px']) for r in st5] if use_px else [0.0] * len(st5)

    # 筛选有效视觉锁定行，并剔除解锁后已被强置 0 的尾帧
    vis_idx = [i for i, r in enumerate(st5) if (int(r['flags']) & 2)]
    if len(vis_idx) < 8:
        return None

    seen_nz = False
    active = []
    for i in vis_idx:
        if abs(w_all[i]) > 0.05:
            seen_nz = True
            active.append(i)
        elif not seen_nz:
            active.append(i)
    if len(active) < 8:
        return None

    # 定位视觉更新帧：优先用像素变化，旧日志回落到相对视线角阶跃
    frames = [active[0]]
    for i in active[1:]:
        p = frames[-1]
        if use_px:
            if vx_all[i] != vx_all[p] or vy_all[i] != vy_all[p]:
                frames.append(i)
        else:
            rel_i = los_all[i] - yaw_all[i]
            rel_p = los_all[p] - yaw_all[p]
            if abs(rel_i - rel_p) > 0.05:
                frames.append(i)
    if len(frames) < 8:
        return None

    # 重建相机平面惯性视线角（机体陀螺积分 + 3D gx 离轴补偿）
    if use_px:
        igz = [0.0] * len(st5)
        for i in range(1, len(st5)):
            dt = t_all[i] - t_all[i - 1]
            if 0.0 < dt < 0.2:
                cp0 = math.atan(vy_all[i - 1] / focal)
                cp1 = math.atan(vy_all[i] / focal)
                e0 = gz_all[i - 1] + gx_all[i - 1] * cp0
                e1 = gz_all[i] + gx_all[i] * cp1
                igz[i] = igz[i - 1] + 0.5 * (e0 + e1) * dt
            else:
                igz[i] = igz[i - 1]
        lam_u = [
            interp1(t_all[i] - VISION_DELAY_S, t_all, igz)
            - math.degrees(math.atan(vx_all[i] / focal))
            for i in frames
        ]
    else:
        lam_u = [los_all[i] for i in frames]

    tu = [t_all[i] for i in frames]
    wt, wv, gv, t_eval = [], [], [], []
    # 中心差分（消除后向差分自带的半帧滞后），跳过初始化首帧 k=0 与末帧
    for k in range(1, len(frames) - 1):
        dt = tu[k + 1] - tu[k - 1]
        if not (0.015 < dt < 0.25):
            continue
        wt.append((lam_u[k + 1] - lam_u[k - 1]) / dt)
        wv.append(w_all[frames[k]])
        gv.append(gz_all[frames[k]])
        t_eval.append(tu[k])
    if len(wv) < 6:
        return None

    sign = sum(1 for a, b in zip(wv, wt) if a * b > 0) / len(wv)
    big = [(a, b) for a, b in zip(wt, wv) if abs(a) >= 10.0]
    big_sign = (sum(1 for a, b in big if a * b > 0) / len(big)) if len(big) >= 4 else float('nan')
    rmse_full = math.sqrt(sum((a - b) ** 2 for a, b in zip(wv, wt)) / len(wv))

    # 中段 RMSE（排除末 0.12 s 擦靶 1/R 几何发散）
    t_end = tu[-1]
    mid = [(a, b) for a, b, te in zip(wv, wt, t_eval) if te <= t_end - 0.12]
    rmse_mid = (math.sqrt(sum((a - b) ** 2 for a, b in mid) / len(mid))
                if len(mid) >= 4 else rmse_full)

    # 估计相位滞后（毫秒）
    t_act = [t_all[i] for i in active]
    w_act = [w_all[i] for i in active]
    best_lag_ms, best_rmse = 0, 1e9
    for lag_ms in range(-20, 101, 10):
        shift = lag_ms * 0.001
        err2 = 0.0
        for te, tr in zip(t_eval, wt):
            ws = interp1(te + shift, t_act, w_act)
            err2 += (ws - tr) ** 2
        rm = math.sqrt(err2 / len(wt))
        if rm < best_rmse:
            best_rmse, best_lag_ms = rm, lag_ms

    # 有效锁定段抖动 p95（排除解锁归零跳变）
    jit = sorted(abs(b - a) for a, b in zip(w_act, w_act[1:]))
    p95 = jit[int(len(jit) * 0.95)] if jit else float('nan')
    pc = partial_corr(wv, wt, gv)

    r = dict(name=os.path.basename(path)[6:-4], n=len(wv), sign=sign,
             big_sign=big_sign, nbig=len(big), rmse=rmse_mid,
             rmse_full=rmse_full, lag_ms=best_lag_ms, jit=p95, pc=pc)
    if not quiet:
        bs = '%.0f%%' % (100 * big_sign) if big_sign == big_sign else '  -- '
        print('%-6s n=%2d  大幅%2d帧 大幅符号 %5s  中段RMSE %5.1f  全程 %5.1f  '
              '滞后 %3dms  抖动 %5.1f  偏相关 %+.2f'
              % (r['name'], r['n'], len(big), bs, rmse_mid, rmse_full,
                 best_lag_ms, p95, pc))
    return r


def verdict(rs):
    if not rs:
        print('没有可用数据。')
        return
    big = [r['big_sign'] for r in rs if r['big_sign'] == r['big_sign']]
    sign = sum(big) / len(big) if big else float('nan')
    rmse = sum(r['rmse'] for r in rs) / len(rs)
    rmse_full = sum(r['rmse_full'] for r in rs) / len(rs)
    lag = sum(r['lag_ms'] for r in rs) / len(rs)
    jit = sum(r['jit'] for r in rs) / len(rs)
    pc = sum(abs(r['pc']) for r in rs if r['pc'] == r['pc']) / max(
        1, sum(1 for r in rs if r['pc'] == r['pc']))
    print()
    print('=== 汇总（%d 发）===' % len(rs))
    print('  大幅帧符号一致  %5.0f%%   （判据 > 85%%，只看 |ω|≥10 °/s 的帧）'
          % (100 * sign if sign == sign else float('nan')))
    print('  中段 RMSE   %5.1f °/s（判据 < 5；全程均值 %.1f °/s，均值滞后 %.0f ms）'
          % (rmse, rmse_full, lag))
    print('  抖动 p95    %5.1f °/s（判据 < 5，已剔除解锁归零尾帧）' % jit)
    print('  |偏相关|    %5.2f     （控制 gz 后 ω 与真值的偏相关）' % pc)
    ok = 0
    ok += 1 if (sign == sign and sign > 0.85) else 0
    ok += 1 if rmse < 5.0 else 0
    ok += 1 if jit < 5.0 else 0
    ok += 1 if pc >= 0.5 else 0
    print()
    if ok == 4:
        print('  结论：4/4 通过 —— ω 可用于 PN。')
    elif ok >= 2:
        print('  结论：%d/4 通过 —— 边界可用，可跑 MODE 1（速率型 PN）验证。' % ok)
    else:
        print('  结论：%d/4 通过 —— 暂不要上 PN，先解决 ω 质量。' % ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path', help='CSV 文件或目录')
    ap.add_argument('--shots', default='', help='逗号分隔发次')
    ap.add_argument('--focal', type=float, default=640.0, help='焦距像素（默认 640）')
    args = ap.parse_args()

    if os.path.isdir(args.path):
        files = sorted(glob.glob(os.path.join(args.path, 'telem_*.csv')))
    else:
        files = [args.path]
    if args.shots:
        want = set(s.strip() for s in args.shots.split(','))
        files = [f for f in files if os.path.basename(f)[6:-4] in want]
    if not files:
        raise SystemExit('没有找到 telem_*.csv')

    print('判读：%-6s %s' % ('发次', '指标'))
    rs = [r for r in (analyze(f, focal=args.focal) for f in files) if r]
    verdict(rs)


if __name__ == '__main__':
    main()
