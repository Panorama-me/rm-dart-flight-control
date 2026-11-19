# -*- coding: utf-8 -*-
"""把下降段的 yaw / out_yaw（角度）与 gy / gz（角速率）画成折线图。

用法：
    python tools/plot_attitude.py <数据目录> [--shots 00031,00030] [--n 3] [--out 路径]

不指定 --shots 时，自动挑「rel 变化最频繁」的 n 发（视觉最有效的那些）。
"""
import argparse
import csv
import glob
import json
import os


def load(path):
    rows = list(csv.DictReader(open(path, encoding='utf-8', errors='replace')))
    d = {k: [] for k in rows[0]}
    for r in rows:
        for k in rows[0]:
            try:
                d[k].append(float(r[k]))
            except (ValueError, TypeError):
                d[k].append(float('nan'))
    return d


def out_abs(d, i, vis):
    """out_yaw 记录的是「相对机身的控制误差 + 前视」，换算成绝对方位。

    out_yaw = (los - yaw) + lead  ->  绝对指令 = yaw + out_yaw = los + lead
    视觉无效时它会跳到 yaw - ref（-88、-99.9 这类），幅值远超 ±30，直接断点。
    """
    if not vis:
        return None
    o = d['out_yaw_deg'][i]
    if abs(o) > 30.0:
        return None
    return round(d['yaw_deg'][i] + o, 2)


def series(path):
    d = load(path)
    idx = [i for i in range(len(d['t_ms'])) if int(d['state'][i]) == 5]
    if len(idx) < 12:
        return None
    t0 = d['t_ms'][idx[0]]
    rel = [d['los_yaw_deg'][i] - d['yaw_deg'][i] for i in idx]
    chg = sum(1 for a, b in zip(rel, rel[1:]) if abs(b - a) > 0.01)
    # 丢目标时 out_yaw 会跳到默认指令值（接近 0），画成断点，别当成真实指令
    vis = [bool(int(d['flags'][i]) & 2) for i in idx]
    return dict(
        name=os.path.basename(path)[6:-4],
        n=len(idx),
        score=100.0 * chg / (len(idx) - 1),
        t=[round((d['t_ms'][i] - t0) / 1000.0, 3) for i in idx],
        yaw=[round(d['yaw_deg'][i], 2) for i in idx],
        out=[out_abs(d, i, v) for i, v in zip(idx, vis)],
        gy=[round(d['gy_dps'][i], 1) for i in idx],
        gz=[round(d['gz_dps'][i], 1) for i in idx],
        rel=[round(x, 2) for x in rel],
    )


TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>下降段姿态折线</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.js"></script>
<style>
  body { margin:0; padding:20px 24px; background:#fff; color:#2C2C2A;
         font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif; }
  h1 { font-size:16px; font-weight:500; margin:0 0 4px; }
  p.sub { font-size:12px; color:#5F5E5A; margin:0 0 18px; line-height:1.6; }
  .panel { border:0.5px solid rgba(0,0,0,0.15); border-radius:12px;
           padding:14px 16px 8px; margin-bottom:18px; }
  .panel h2 { font-size:13px; font-weight:500; margin:0 0 10px; }
  .lg { display:flex; flex-wrap:wrap; gap:14px; font-size:12px; color:#5F5E5A; margin-bottom:8px; }
  .lg span { display:flex; align-items:center; gap:5px; }
  .sw { width:14px; height:0; border-top-width:2px; border-top-style:solid; }
  .wrap { position:relative; height:300px; }
</style>
</head>
<body>
<h1>下降段：yaw / out_yaw 与角速率</h1>
<p class="sub">左轴为角度（度），右轴为角速率（度/秒）。<b>yaw 是实际航向，out_yaw 已由「相对机身的指令」换算为绝对方位</b>，两者之差即控制误差加前视项。<br>
gy 是俯仰轴角速率（按要求一并画出），gz 才是与 yaw 同轴的那一路，用灰色虚线作参照。</p>
<div id="panels"></div>
<script>
const DATA = __DATA__;
const COL = { yaw:'#378ADD', out:'#D85A30', gy:'#1D9E75', gz:'#888780' };
const panels = document.getElementById('panels');
DATA.forEach(function (d, i) {
  const box = document.createElement('div');
  box.className = 'panel';
  box.innerHTML =
    '<h2>' + d.name + '　下降段 ' + d.n + ' 帧　视觉有效 ' + d.score.toFixed(0) + '%</h2>' +
    '<div class="lg">' +
      '<span><i class="sw" style="border-top-color:' + COL.yaw + '"></i>yaw 实际</span>' +
      '<span><i class="sw" style="border-top-color:' + COL.out + '"></i>out_yaw 指令</span>' +
      '<span><i class="sw" style="border-top-color:' + COL.gy + '"></i>gy 俯仰角速率</span>' +
      '<span><i class="sw" style="border-top-color:' + COL.gz + ';border-top-style:dashed"></i>gz 偏航角速率（参照）</span>' +
    '</div><div class="wrap"><canvas id="c' + i + '"></canvas></div>';
  panels.appendChild(box);

  new Chart(document.getElementById('c' + i), {
    type: 'line',
    data: {
      labels: d.t,
      datasets: [
        { label:'yaw', data:d.yaw, borderColor:COL.yaw, borderWidth:2,
          pointRadius:0, tension:0.15, yAxisID:'y' },
        { label:'out_yaw', data:d.out, borderColor:COL.out, borderWidth:2,
          pointRadius:0, tension:0.15, yAxisID:'y' },
        { label:'gy', data:d.gy, borderColor:COL.gy, borderWidth:1.5,
          pointRadius:0, tension:0.1, yAxisID:'y1' },
        { label:'gz', data:d.gz, borderColor:COL.gz, borderWidth:1.2,
          borderDash:[4,4], pointRadius:0, tension:0.1, yAxisID:'y1' }
      ]
    },
    options: {
      responsive:true, maintainAspectRatio:false, animation:false,
      interaction:{ mode:'index', intersect:false },
      plugins:{ legend:{ display:false } },
      scales:{
        x:{ ticks:{ color:'#5F5E5A', font:{size:11}, maxTicksLimit:10 },
            grid:{ color:'rgba(0,0,0,0.06)' },
            title:{ display:true, text:'下降段耗时 (s)', color:'#5F5E5A', font:{size:11} } },
        y:{ position:'left', ticks:{ color:COL.yaw, font:{size:11} },
            grid:{ color:'rgba(0,0,0,0.06)' },
            title:{ display:true, text:'角度 (°)', color:COL.yaw, font:{size:11} } },
        y1:{ position:'right', ticks:{ color:COL.gy, font:{size:11} },
             grid:{ drawOnChartArea:false },
             title:{ display:true, text:'角速率 (°/s)', color:COL.gy, font:{size:11} } }
      }
    }
  });
});
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--shots', help='逗号分隔的发次，如 00031,00030')
    ap.add_argument('--n', type=int, default=3, help='不指定 --shots 时画几发')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, 'telem_*.csv')))
    if not files:
        raise SystemExit('目录里没有 telem_*.csv：%s' % args.dir)

    want = None
    if args.shots:
        want = set(s.strip() for s in args.shots.split(','))
        files = [f for f in files if os.path.basename(f)[6:-4] in want]

    data = [s for s in (series(f) for f in files) if s]
    if not data:
        raise SystemExit('没有可用的下降段数据')

    if not args.shots:
        data.sort(key=lambda x: -x['score'])
        data = data[:args.n]
    data.sort(key=lambda x: x['name'])

    out = args.out or os.path.join(args.dir, 'attitude_plot.html')
    html = TEMPLATE.replace('__DATA__', json.dumps(data, ensure_ascii=False))
    open(out, 'w', encoding='utf-8', newline='').write(html)
    print('已生成 %s' % out)
    for d in data:
        ov = [v for v in d['out'] if v is not None]
        print('  %s  %d 帧  视觉有效 %.0f%%  yaw %.1f~%.1f  out_abs %.1f~%.1f  gy %.0f~%.0f' %
              (d['name'], d['n'], d['score'],
               min(d['yaw']), max(d['yaw']),
               min(ov) if ov else float('nan'), max(ov) if ov else float('nan'),
               min(d['gy']), max(d['gy'])))


if __name__ == '__main__':
    main()
