# -*- coding: utf-8 -*-
"""评估视觉视线角速率（vis_rate_dps）能否用于 PN 制导。

判据（对「惯性系视线角速率」的要求）：
  1. 与 los 绝对方位差分一致 —— 符号一致率、RMSE
  2. 噪声够低 —— 逐帧跳变 p95
  3. 不被机体转动主导 —— 控制 gz 后的偏相关
  4. 峰值保真 —— 不虚高、不衰减

用法：
    python tools/eval_vis_rate.py <csv或目录> [--shots 00001,00002]
"""
import argparse
import csv
import glob
import math
import os

COLS = ('vis_rate_dps', 'pn_omega_dps')      # 新固件优先，旧数据回落


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


def analyze(path, quiet=False):
    rows = list(csv.DictReader(open(path, encoding='utf-8', errors='replace')))
    if not rows:
        return None
    col = pick_col(rows[0])
    if col is None:
        if not quiet:
            print('%-6s 缺少速率列（需要 vis_rate_dps 或 pn_omega_dps）'
                  % os.path.basename(path)[6:-4])
        return None

    seg = [r for r in rows if int(r['state']) == 5 and (int(r['flags']) & 2)]
    if len(seg) < 20:
        return None

    t = [float(r['t_ms']) / 1000.0 for r in seg]
    w = [float(r[col]) for r in seg]
    gz = [float(r['gz_dps']) for r in seg]
    los = [float(r['los_yaw_deg']) for r in seg]
    rel = [float(r['los_yaw_deg']) - float(r['yaw_deg']) for r in seg]

    # 用 rel 的阶跃定位视觉帧（los 绝对方位含 yaw 的连续变化，不能直接差分）
    frames = [0]
    for i in range(1, len(rel)):
        if abs(rel[i] - rel[i - 1]) > 0.05:
            frames.append(i)
    if len(frames) < 8:
        return None

    wt, wv, gv = [], [], []
    for a, b in zip(frames, frames[1:]):
        dt = t[b] - t[a]
        if not (0.01 < dt < 0.09):
            continue
        wt.append((los[b] - los[a]) / dt)      # 两次视觉更新之间的绝对方位变化率
        wv.append(w[b])
        gv.append(gz[b])
    if len(wv) < 8:
        return None

    sign = sum(1 for a, b in zip(wv, wt) if a * b > 0) / len(wv)
    # 大幅帧的符号一致率才是 PN 真正依赖的：小信号帧符号本身随机，
    # 但那些帧上 PN 输出也接近 0，打反了没有实际危害。
    big = [(a, b) for a, b in zip(wt, wv) if abs(a) >= 10.0]
    big_sign = (sum(1 for a, b in big if a * b > 0) / len(big)) if len(big) >= 4 else float('nan')
    rmse = math.sqrt(sum((a - b) ** 2 for a, b in zip(wv, wt)) / len(wv))
    jit = [abs(b - a) for a, b in zip(w, w[1:])]
    jit.sort()
    p95 = jit[int(len(jit) * 0.95)] if jit else float('nan')
    pc = partial_corr(wv, wt, gv)
    pk_w = max(abs(x) for x in wv)
    pk_t = max(abs(x) for x in wt)

    r = dict(name=os.path.basename(path)[6:-4], n=len(wv), sign=sign,
             big_sign=big_sign, nbig=len(big), rmse=rmse,
             jit=p95, pc=pc, pk_ratio=pk_w / pk_t if pk_t > 1 else float('nan'))
    if not quiet:
        bs = '%.0f%%' % (100 * big_sign) if big_sign == big_sign else '  -- '
        print('%-6s n=%2d  大幅%2d帧 大幅符号 %5s  全部 %3.0f%%  RMSE %5.1f  '
              '抖动 %5.1f  偏相关 %+.2f'
              % (r['name'], r['n'], len(big), bs, 100 * sign, rmse, p95, pc))
    return r


def verdict(rs):
    if not rs:
        print('没有可用数据。')
        return
    big = [r['big_sign'] for r in rs if r['big_sign'] == r['big_sign']]
    sign = sum(big) / len(big) if big else float('nan')
    rmse = sum(r['rmse'] for r in rs) / len(rs)
    jit = sum(r['jit'] for r in rs) / len(rs)
    pc = sum(abs(r['pc']) for r in rs if r['pc'] == r['pc']) / max(
        1, sum(1 for r in rs if r['pc'] == r['pc']))
    print()
    print('=== 汇总（%d 发）===' % len(rs))
    print('  大幅帧符号一致  %5.0f%%   （判据 > 85%%，只看 |ω|≥10 °/s 的帧）'
          % (100 * sign if sign == sign else float('nan')))
    print('  RMSE        %5.1f °/s（判据 < 5）' % rmse)
    print('  抖动 p95    %5.1f °/s（判据 < 5）' % jit)
    print('  |偏相关|    %5.2f     （判据 < 0.5，越低越说明没被机体转动主导）' % pc)
    ok = 0
    ok += 1 if (sign == sign and sign > 0.85) else 0
    ok += 1 if rmse < 5 else 0
    ok += 1 if jit < 5 else 0
    ok += 1 if pc < 0.5 else 0
    print()
    if ok == 4:
        print('  结论：4/4 通过 —— ω 可用于 PN。')
    elif ok >= 2:
        print('  结论：%d/4 通过 —— 边界可用，先跑 MODE 1（速率型 PN）小步验证。' % ok)
    else:
        print('  结论：%d/4 通过 —— 暂不要上 PN，先解决 ω 质量。' % ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('path', help='CSV 文件或目录')
    ap.add_argument('--shots', default='', help='逗号分隔发次')
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
    rs = [r for r in (analyze(f) for f in files) if r]
    verdict(rs)


if __name__ == '__main__':
    main()
