#!/usr/bin/env python3
"""把 OpenMV 录制的 .mjpeg 拆成 JPEG 序列，并生成可调速的 HTML 播放器。

MJPEG 裸流没有帧率元数据，播放器通常按 25/30 fps 播放；而实际录制帧率
只有 16~26 fps，所以看起来偏快。本脚本按真实帧率生成播放器，并支持
0.25x ~ 4x 调速与逐帧查看。

用法:
    python tools/mjpeg_player.py 内录/11.11/rec_00016.mjpeg
    python tools/mjpeg_player.py rec.mjpeg --fps 18
    python tools/mjpeg_player.py rec.mjpeg --out D:/tmp/rec16
"""

import argparse
import os
import sys

SOI = b"\xff\xd8\xff"
EOI = b"\xff\xd9"


def split_frames(data):
    """按 SOI 标记切分 JPEG 帧，并按 EOI 裁掉帧间填充。

    OpenMV 的 ImageIO 流在每帧 EOI 之后会补 0x00 对齐，所以不能简单
    用「下一帧起点」当上一帧终点，必须回退到最近的 EOI。

    返回 (帧数据列表, 带填充的帧数, 找不到 EOI 的帧数)。
    """
    starts = []
    i = 0
    while True:
        j = data.find(SOI, i)
        if j < 0:
            break
        starts.append(j)
        i = j + 3

    frames = []
    padded = 0
    broken = 0
    for k, start in enumerate(starts):
        end = starts[k + 1] if k + 1 < len(starts) else len(data)
        blob = data[start:end]
        e = blob.rfind(EOI)
        if e > 0:
            if e + 2 < len(blob):
                padded += 1
            blob = blob[:e + 2]
        else:
            broken += 1
        frames.append(blob)
    return frames, padded, broken


HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>{name} - MJPEG 播放器</title>
<style>
  body {{ margin: 0; background: #14161a; color: #d8d6d0;
         font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }}
  header {{ padding: 12px 16px; border-bottom: 1px solid #2a2d33;
           display: flex; flex-wrap: wrap; gap: 12px; align-items: center; }}
  h1 {{ font-size: 14px; font-weight: 500; margin: 0; }}
  .meta {{ font-size: 12px; color: #8b8983; }}
  main {{ display: flex; justify-content: center; padding: 16px; }}
  img {{ max-width: 100%; max-height: 70vh; image-rendering: pixelated;
        background: #000; border: 1px solid #2a2d33; }}
  .bar {{ display: flex; flex-wrap: wrap; gap: 8px; align-items: center;
         padding: 0 16px 16px; }}
  button {{ background: #22262c; color: #d8d6d0; border: 1px solid #34383f;
           border-radius: 6px; padding: 6px 12px; font-size: 13px; cursor: pointer; }}
  button:hover {{ background: #2c3138; }}
  button.on {{ background: #2f6fb5; border-color: #3f86d6; color: #fff; }}
  input[type=range] {{ flex: 1 1 240px; min-width: 160px; accent-color: #3f86d6; }}
  .readout {{ font-size: 12px; color: #8b8983; min-width: 200px; }}
</style>
</head>
<body>
<header>
  <h1>{name}</h1>
  <span class="meta">{count} 帧 · 录制 {fps} fps · 时长 {duration} s</span>
</header>
<main><img id="view" alt="frame"></main>
<div class="bar">
  <button id="play">播放</button>
  <button id="prev">上一帧</button>
  <button id="next">下一帧</button>
  <span class="meta">速度</span>
  <button class="spd" data-s="0.25">0.25x</button>
  <button class="spd" data-s="0.5">0.5x</button>
  <button class="spd on" data-s="1">1x</button>
  <button class="spd" data-s="2">2x</button>
  <button class="spd" data-s="4">4x</button>
</div>
<div class="bar">
  <input id="seek" type="range" min="0" max="{last}" value="0" step="1">
  <span class="readout" id="readout"></span>
</div>
<script>
const TOTAL = {count};
const FPS = {fps};
const DIR = "{dir}";
const view = document.getElementById('view');
const seek = document.getElementById('seek');
const readout = document.getElementById('readout');
const playBtn = document.getElementById('play');
let idx = 0, playing = false, speed = 1, timer = null, acc = 0, last = 0;

function src(i) {{
  return DIR + 'frame_' + String(i).padStart(4, '0') + '.jpg';
}}
function show(i) {{
  idx = Math.max(0, Math.min(TOTAL - 1, i));
  view.src = src(idx);
  seek.value = idx;
  readout.textContent = '第 ' + (idx + 1) + ' / ' + TOTAL + ' 帧 · '
    + (idx / FPS).toFixed(2) + ' s';
}}
function tick(ts) {{
  if (!playing) return;
  if (!last) last = ts;
  acc += (ts - last) * speed;
  last = ts;
  const step = 1000 / FPS;
  while (acc >= step) {{
    acc -= step;
    if (idx + 1 >= TOTAL) {{ pause(); return; }}
    idx += 1;
  }}
  view.src = src(idx);
  seek.value = idx;
  readout.textContent = '第 ' + (idx + 1) + ' / ' + TOTAL + ' 帧 · '
    + (idx / FPS).toFixed(2) + ' s';
  timer = requestAnimationFrame(tick);
}}
function play() {{
  if (playing) return;
  if (idx + 1 >= TOTAL) idx = 0;
  playing = true; last = 0; acc = 0;
  playBtn.textContent = '暂停';
  timer = requestAnimationFrame(tick);
}}
function pause() {{
  playing = false;
  playBtn.textContent = '播放';
  if (timer) cancelAnimationFrame(timer);
  timer = null;
}}
playBtn.onclick = () => playing ? pause() : play();
document.getElementById('prev').onclick = () => {{ pause(); show(idx - 1); }};
document.getElementById('next').onclick = () => {{ pause(); show(idx + 1); }};
seek.oninput = e => {{ pause(); show(parseInt(e.target.value, 10)); }};
document.querySelectorAll('.spd').forEach(b => {{
  b.onclick = () => {{
    speed = parseFloat(b.dataset.s);
    document.querySelectorAll('.spd').forEach(x => x.classList.remove('on'));
    b.classList.add('on');
  }};
}});
document.addEventListener('keydown', e => {{
  if (e.code === 'Space') {{ e.preventDefault(); playing ? pause() : play(); }}
  if (e.code === 'ArrowLeft') {{ pause(); show(idx - 1); }}
  if (e.code === 'ArrowRight') {{ pause(); show(idx + 1); }}
}});
show(0);
</script>
</body>
</html>
"""


def build_player(name, frames, fps, out_dir, base_dir):
    os.makedirs(out_dir, exist_ok=True)
    for i, blob in enumerate(frames):
        with open(os.path.join(out_dir, "frame_%04d.jpg" % i), "wb") as fp:
            fp.write(blob)

    html_path = os.path.splitext(out_dir.rstrip("/\\"))[0] + ".html"
    html = HTML_TEMPLATE.format(
        name=name,
        count=len(frames),
        fps=fps,
        last=max(0, len(frames) - 1),
        duration="%.2f" % (len(frames) / fps),
        dir=os.path.basename(out_dir.rstrip("/\\")) + "/",
    )
    with open(html_path, "w", encoding="utf-8") as fp:
        fp.write(html)
    return html_path


def main():
    ap = argparse.ArgumentParser(description="拆分 MJPEG 并生成可调速播放器")
    ap.add_argument("path", help=".mjpeg 文件路径")
    ap.add_argument("--fps", type=float, default=16.0,
                    help="录制帧率（默认 16，11.11 实测 16~18）")
    ap.add_argument("--out", default=None,
                    help="帧目录（默认与源文件同名的 _frames 目录）")
    args = ap.parse_args()

    if not os.path.isfile(args.path):
        print("找不到文件:", args.path)
        return 1

    data = open(args.path, "rb").read()
    frames, padded, broken = split_frames(data)
    if not frames:
        print("没有找到 JPEG 帧")
        return 1

    stem = os.path.splitext(args.path)[0]
    out_dir = args.out or (stem + "_frames")
    html_path = build_player(os.path.basename(args.path), frames, args.fps,
                             out_dir, os.path.dirname(stem))

    print("源文件 : %s (%.0f KB)" % (args.path, len(data) / 1024))
    print("帧数   : %d（带填充 %d，缺 EOI %d）" % (len(frames), padded, broken))
    print("帧率   : %g fps，时长 %.2f s" % (args.fps, len(frames) / args.fps))
    print("帧目录 : %s" % out_dir)
    print("播放器 : %s" % html_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
