#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 main.py 派生标定副本 tools/calib_main.py。

只加 4 处，全部与「把视觉像素偏移写进遥测 CSV」有关：
  1. 视觉主循环：保存 vis_dx / vis_dy
  2. 遥测表头：追加 vis_dx,vis_dy
  3. 遥测格式串：追加两个 %.2f
  4. 遥测参数：追加 vis_dx, vis_dy

为什么不手工复制：main.py 有 1300 行，手抄一份必然失同步。
每次 main.py 改动后重跑本脚本即可，diff 一眼能看出只有这 4 处不同。

用法：
    python tools/make_calib_copy.py          # 生成
    python tools/make_calib_copy.py --check  # 只检查是否已同步
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'main.py')
DST = os.path.join(ROOT, 'tools', 'calib_main.py')

HEADER = '''# =============================================================================
# 本文件由 tools/make_calib_copy.py 从 main.py 自动生成，不要手改。
#
# 与 main.py 的差别只有 4 处，都为了把视觉像素偏移写进遥测 CSV：
#   1. 视觉循环里保存 vis_dx / vis_dy
#   2. 遥测表头追加 vis_dx,vis_dy
#   3. 遥测格式串追加两个 %.2f
#   4. 遥测参数追加 vis_dx, vis_dy
#
# 用途：目标固定不动，手动转动机体，让陀螺仪 yaw 与像素偏移同步落盘，
#       再用 tools/fit_focal_csv.py 回归出焦距（f = -d(dx)/d(yaw) * 57.3）。
#
# 台架流程：
#   0. OpenMV 必须插 SD 卡。没有卡时 record_dir 为 None，遥测 CSV 根本不会生成。
#   1. STM32 烧正常固件，DART_MODE = DEBUG（只算不发舵，遥测照发）
#   2. 本文件在 OpenMV 上运行
#   3. 目标灯固定，机体绕竖轴左右各转 20 度以上，转 3~5 个来回
#   4. 停录，把 telem_XXXXX.csv 拷到电脑，跑 tools/fit_focal_csv.py
#      注意内录数据在仓库外面（rm_dart_test/内录/...），用绝对路径最稳
# =============================================================================

'''

REPLACES = [
    (
        'cur_roi = INIT_ROI\n\ntry:',
        'cur_roi = INIT_ROI\n\n'
        '# --- 标定副本新增：最近一帧的视觉像素偏移，供遥测 CSV 记录 ---\n'
        'vis_dx = 0.0\n'
        'vis_dy = 0.0\n\n'
        'try:',
        '视觉循环前初始化 vis_dx/vis_dy',
    ),
    (
        '            send_target(dx, dy)',
        '            vis_dx = dx\n'
        '            vis_dy = dy\n'
        '            send_target(dx, dy)',
        '视觉帧保存 dx/dy',
    ),
    (
        '"a_meas_ms2,a_cmd_ms2,pn_omega_dps,yaw_u_us\\n")',
        '"a_meas_ms2,a_cmd_ms2,pn_omega_dps,yaw_u_us,vis_dx,vis_dy\\n")',
        '遥测表头追加两列',
    ),
    (
        '"%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f\\n"',
        '"%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f\\n"',
        '遥测格式串追加两个字段',
    ),
    (
        'telem_a_meas, telem_a_cmd, telem_pn_omega, telem_yaw_u))',
        'telem_a_meas, telem_a_cmd, telem_pn_omega, telem_yaw_u,\n'
        '               vis_dx, vis_dy))',
        '遥测参数追加 vis_dx/vis_dy',
    ),
]


def main():
    if not os.path.exists(SRC):
        sys.exit('找不到 main.py：%s\n本脚本必须放在仓库的 tools/ 目录下运行。' % SRC)
    src = open(SRC, 'rb').read().decode('utf-8')
    out = src
    for old, new, desc in REPLACES:
        n = out.count(old)
        if n != 1:
            sys.exit('替换失败「%s」：main.py 里匹配到 %d 次（应为 1），'
                     '可能 main.py 改过了，需要同步更新本脚本' % (desc, n))
        out = out.replace(old, new)

    header = HEADER
    content = header + out

    if '--check' in sys.argv:
        try:
            cur = open(DST, 'rb').read().decode('utf-8')
        except FileNotFoundError:
            print('calib_main.py 不存在，需要生成')
            return 1
        if cur == content:
            print('已同步')
            return 0
        print('不同步，需要重新生成')
        return 1

    open(DST, 'wb').write(content.encode('utf-8'))
    print('已生成 %s（%d 行）' % (os.path.relpath(DST, ROOT),
                                content.count('\n') + 1))
    print('与 main.py 的差异：%d 处' % len(REPLACES))
    return 0


if __name__ == '__main__':
    sys.exit(main())
