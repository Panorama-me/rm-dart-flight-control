# AGENTS.md —— AI 修改规范

本文件约束所有 AI 助手对本仓库的修改。**开始改代码前必须读完第 1、2 节。**

项目：RoboMaster 飞镖飞控。STM32F103C8T6 下位机（Keil MDK）+ OpenMV 上位机（MicroPython）。
代码跑在**会飞的硬件**上，改错不是报错，是炸机。所以本规范的重点是「别引入不可逆的破坏」，
而不是代码风格。

---

## 1. 文件编码与换行（最高优先级）

### 1.1 实际分布

不要凭经验猜，下面是实测结果：

| 路径 | 编码 | 换行 | 数量 |
|---|---|---|---:|
| `rm_dart/MDK-ARM/Control_Task.c` | **GBK** | **CRLF** | 1 |
| `rm_dart/MDK-ARM/USER/TASK/Inc/ctl_task.h` | **GBK** | **CRLF** | 1 |
| `rm_dart/MDK-ARM/USER/**`（其余） | GBK 47 个 / UTF-8 55 个，**混合** | 以 LF 为主 | 102 |
| `rm_dart/Core/**` | UTF-8 | CRLF 为主 | 20 |
| `rm_dart/Drivers/**`、`Middlewares/**` | UTF-8 | LF | 107 |
| `main.py`、`docs/`、`tests/`、`tools/` | UTF-8 | LF | 7 |

**只有两个文件是 GBK + CRLF**，但偏偏是本项目最核心的两个。`USER/` 下其余文件编码混杂，
**动手前必须先探测**：

```python
b = open(p, 'rb').read()
try:    b.decode('utf-8'); enc = 'utf8'
except: b.decode('gbk');   enc = 'gbk'
```

### 1.2 禁止事项

- **禁止** `open(path, 'w')` 或 `open(path, 'w', encoding='utf-8')` 去写 GBK 文件。
  这会直接抛 `UnicodeDecodeError` 或把中文写成乱码。
- **禁止**用 `Edit` 工具改 GBK 文件的多行中文片段。`Edit` 按 UTF-8 读写，会毁掉全文。
  单行 ASCII 片段可以，但仍有风险，一律走 1.3 的方式。
- **禁止**用 Python 文本模式（默认 `newline=None`）写 `.md`/`.py`。Windows 上会把 `\n`
  全转成 `\r\n`，git 里会显示整文件重写。写文档用 `newline=''` 或二进制模式。

### 1.3 正确写法：先编码、再原子替换

```python
import os
p = 'rm_dart/MDK-ARM/Control_Task.c'
text = open(p, 'rb').read().decode('gbk')      # 1. 按原编码读

# ... 改 text ...

out = text.encode('gbk')                        # 2. 先编码，失败就抛在这里，源文件没动
tmp = p + '.tmp'
open(tmp, 'wb').write(out)
os.replace(tmp, p)                              # 3. 原子替换
```

顺序不能反。曾经因为 `open(p,'wb')` 先截断、随后 `encode` 抛异常，导致
`Control_Task.c` 变成 0 字节、一天的工作全部丢失。

### 1.4 GBK 无法编码的字符

以下字符**不能出现在 `Control_Task.c` / `ctl_task.h` 里**，注释也不行：

| 字符 | 码位 | 替代写法 |
|---|---|---|
| `µ` | U+00B5 | `us` 或「微秒」 |
| `−`（数学减号） | U+2212 | ASCII `-` |
| `→` | U+2192 | `->` |
| `²` | U+00B2 | `^2` |
| `①` `②` | U+2460… | `1)` `2)` |

写注释时用中文「负」「平方」更稳妥。

### 1.5 改完必须自检

```bash
python -c "
for p in ['rm_dart/MDK-ARM/Control_Task.c','rm_dart/MDK-ARM/USER/TASK/Inc/ctl_task.h']:
    b=open(p,'rb').read(); c=b.count(b'\r\n'); l=b.count(b'\n')
    b.decode('gbk')
    assert c==l, p+' 换行被破坏'
    print(p, len(b), 'B  GBK OK  纯 CRLF')
"
```

`CRLF 数 == LF 数` 才说明没有裸 LF 混进来。

---

## 2. 不能动的东西

这些地方改错会直接导致舵机反向、时序崩坏或上位机解析错位。改动前必须说明理由并得到确认。

| 位置 | 为什么 |
|---|---|
| `write_servo_outputs()` 的混控公式 `mixed[0..3]` | X 尾翼四舵分配，改一个符号四个舵面全反 |
| `DART_PITCH/YAW/ROLL_OUTPUT_SIGN` | 机械安装方向，只能靠台架确认，不能靠推理 |
| `DART_DESCENT_PITCH/YAW_OUTPUT_SIGN` | 下降段方向修正，与上面**叠加**，共两次取反 |
| `DART_CONTROL_PERIOD_MS = 2` 与 200 Hz PWM | 5 ms PWM 周期内只有最后一次 CCR 写入生效 |
| 遥测 37 字节布局（`T_*_INDEX`） | 改了必须同步 `main.py` 的解析，否则上位机全乱 |
| `gyro_rate_derivative()` | PID 的 D 项来源是陀螺角速率，不是误差微分 |
| 中断服务函数 | 禁止引入浮点运算和阻塞调用；F103 无 FPU |

**`DART_MODE`** 默认必须是 `DART_MODE_BOTH`。台架调试改 `DART_MODE_DEBUG`（只算不驱动舵机）
后，**飞行前必须改回**，并在提交信息里注明。

---

## 3. 改参数的规矩

### 3.1 每个参数都要有实测依据

参数注释里写清楚「这个数从哪来」。参考现有写法：

```c
#define DART_ASCENT_PITCH_LIFT_US  100.0f  /* 阶段1 固定抬升；对齐 11.12 全部 14 发实测 */
```

**不要把推测写成实测。** 如果只是估计，写「待验证」或「估计值」。

### 3.2 先反推数据，再改参数

`内录/` 下有 11.10 / 11.11 / 11.12 三批飞行数据（`telem_*.csv` + `rec_*.mjpeg`）。
改参数前先用数据验证旧值，别凭直觉。

**舵机三通道反解公式**（X 尾翼 `mixed = ±p ±y +r`，`s*_deg` 为相对中位角，0.09 °/us）：

```
p_us = (s1 + s2 - s3 - s4) / 4 / 0.09
y_us = (s1 - s2 - s3 + s4) / 4 / 0.09
r_us = (s1 + s2 + s3 + s4) / 4 / 0.09
```

前提是舵机没被步进限幅截断（`DART_SERVO_MAX_STEP_US = 30`）。大机动段反解会失真，需剔除。

### 3.3 参数改动要能解释「为什么这一发不一样」

同一套参数两次飞行结果不同是常态（初速、风、装配）。归因前先排除初速差异，
别急着改 PID。

---

## 4. 提交前必须跑

```bash
# 1. 编译（在 rm_dart/MDK-ARM 目录下）
"/c/Keil_v5/UV4/UV4.exe" -b MA_DartGuide.uvprojx -j0 -o build_log.txt
# 要求 0 Error(s), 0 Warning(s)，并记录 Program Size

# 2. 主机测试（在仓库根目录）
python tests/test_main_logic.py      # 16 项
python tests/test_playback_paths.py  # 5 项

# 3. 编码自检（见 1.5）
```

`build_log.txt` 已在 `.gitignore` 中，不要提交。

---

## 5. 排查方法论（这个项目踩过的坑）

### 5.1 闭环遥测测不出被控对象符号

想从飞行数据判断「正舵面让姿态往哪转」是**做不到的**——控制器会把舵面与响应解耦。
实测 `corr(y_us, Δyaw)` 中位 **−0.01**。

符号只能靠台架：固定机体 + `DART_MODE_DEBUG`，给固定指令看舵面方向。

### 5.2 遥测里的 `los_yaw_deg` 列不是控制器用的那个值

```c
// main.py 收到的 CSV 列 = (yaw - ref_yaw) + 相对视线角
los_yaw_deg_out = wrap_180(yaw_deg - debug_ref_yaw_deg) + vision.los_yaw_deg;
```

控制器内部用的是 `vision.los_yaw_deg`（**相对**相机光轴的角度）。
拿 CSV 列直接和 `yaw` 比较会得出错误结论。

### 5.3 步进限幅会污染舵面反解

`DART_SERVO_MAX_STEP_US = 30` 让四个通道各自独立限幅，限幅期间 `s1..s4` 不再正比于
目标混控量，反解出的 `p_us` 会跳变。看到 `-100 / -50 / -100` 这种锯齿，先怀疑限幅。

### 5.4 状态枚举

```
0 READY  1 IMU_READY  2 SERVO_READY  3 LAUNCHED  4 ASCENT
5 DESCENT  6 VISION_ERROR  7 DEBUG  8 LANDED
```

`state=8` 之后 `p_us/y_us/r_us` 会冻结在最后一帧（PWM 已停），不是实时值。

---

## 6. 禁止的操作

- 删除或修改 `.workbuddy-ai/`（项目记忆，不是缓存）
- 批量重排 `DartControlRuntime_t` 字段顺序（有 51 个字段，重排容易漏改）
- `git push --force`、`git reset --hard`、`git commit --amend` 改已推送的历史
- 未经确认修改 `DART_MODE`、`*_OUTPUT_SIGN`、遥测帧布局
- 在未编译通过的情况下提交

---

## 7. 目录速查

```
rm_dart/MDK-ARM/
  Control_Task.c              ★ 控制主逻辑（GBK+CRLF）
  USER/TASK/Inc/ctl_task.h    ★ 全部参数与数据结构（GBK+CRLF）
  USER/Components/Controller/controller.c   PID 实现
  USER/Components/Devices/BMI088driver.c    IMU 驱动
  MA_DartGuide.uvprojx        Keil 工程
rm_dart/Core/                 CubeMX 生成（UTF-8+CRLF，尽量不改）
main.py                       OpenMV 上位机（UTF-8+LF）
tools/mjpeg_player.py         MJPEG 拆帧 + 可调播放器
tools/bench_encode.py         录像存储方案耗时实测（OpenMV 上跑）
tests/                        主机单元测试
docs/技术手册.md              完整技术手册（改代码要同步）
内录/                         飞行数据，不入库
```

---

## 8. 文档同步

改了参数或行为，**同一次提交里**更新：

- `docs/技术手册.md` 对应章节 + 参数总表 + 附：变更记录
- `README.md` 参数速查表

文档写了但代码没实现，比不写更糟——会误导下一个人。反过来，代码实现了但文档没写，
下一个人会重新发明一遍。
