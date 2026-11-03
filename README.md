# rm_dart：视觉引导小型飞行器

STM32F103 + OpenMV 的双控制器原型工程。F103 负责姿态解算、起飞检测、上升段姿态保持与下降段视觉引导，输出四路舵机 X 型尾翼混控；OpenMV 负责绿色目标识别、串口帧收发、屏幕叠加与机载录像。

![固件](https://img.shields.io/badge/firmware-STM32F103C8-blue) ![IDE](https://img.shields.io/badge/IDE-Keil%20MDK%20ARMCC%205.06-orange) ![vision](https://img.shields.io/badge/vision-OpenMV%20QVGA-green) ![status](https://img.shields.io/badge/status-bench%20only-red)

> **状态：未完成上板验证。** 本文所有参数均为代码默认值，未经真实飞行整定。首次测试必须拆除动力或牢固约束机体。

最后更新：2026-11-03

---

## 1. 主要特性

| 能力 | 实现 | 位置 |
|---|---|---|
| 姿态解算 | BMI088 + 四元数 EKF，输出 yaw/pitch/roll | `ins_task.c`、`QuaternionEKF.c` |
| 起飞检测 | 去重力机体 X 轴加速度阈值 + 连续 5 周期确认 | `Control_Task.c` |
| 上升段 | 保持起飞 yaw/roll；pitch 两阶段补偿后锁存；识别到目标时 yaw 切视觉 | `run_ascent_stabilization()` |
| 下降段 | 前视追踪（视线角 + 前视时间×惯性系视线角速率）+ 视觉 PID + roll 坐标投影 | `run_descent_guidance()` |
| 视觉滤波 | 默认 RAW_RATE（位置不过滤、仅轻滤角速率），可切回 EKF | `update_vision_filter()` |
| 模式握手 | 下位机决定模式，握手帧携带 default/pitch/yaw/both | `openmv_handshake_send()` |
| 遥测 | 37 字节、50 Hz，含陀螺、姿态、输出目标角、LOS 增量、四路舵机角、状态、flags | `attitude_telemetry_send()` |
| 目标识别 | 绿色色块 + 跟踪 ROI + 5 帧时间中值 | `main.py` |
| 机载记录 | ImageIO 原生 RGB565 录像 + 每帧遥测 CSV | `main.py` |
| 回放导出 | OpenMV 上回放，可导出 MJPEG | `imageio_playback.py` |
| 地面站 / GPS / 测距 | **无** | — |

## 2. 仓库结构

```text
.
├── README.md                 本文件：总览、接线、协议速查、构建与验证
├── docs/
│   └── 技术手册.md            原理与实现细节、参数总表、排障、风险与后续计划
├── main.py                   OpenMV 视觉端主程序
├── imageio_playback.py       OpenMV 录像回放 / 导出脚本
├── tests/
│   ├── test_main_logic.py    协议与录像状态机的主机测试
│   └── test_playback_paths.py 回放路径查找的主机测试
└── rm_dart/                  STM32F103 Keil 工程
    ├── MA_DartGuide.ioc      CubeMX 配置
    ├── Core/                 HAL 初始化、FreeRTOS、中断、时钟
    ├── Drivers/              STM32F1 HAL + CMSIS（原厂库）
    ├── Middlewares/          FreeRTOS 10.0.1 + CMSIS-RTOS v2
    └── MDK-ARM/
        ├── MA_DartGuide.uvprojx      Keil 工程
        ├── Control_Task.c            ★ 飞控核心（GBK 编码）
        ├── USER/TASK/Inc/ctl_task.h  ★ 参数与协议定义（GBK 编码）
        ├── USER/TASK/ins_task.c      IMU 采集与姿态解算
        ├── USER/Components/          PID、EKF、BMI088 驱动
        └── Libraries/                CMSIS-DSP Cortex-M3 库
```

编译产物（`rm_dart/MDK-ARM/MA_DartGuide/` 下的 `.o`、`.axf`、`.hex`、`.map`、`.lst`）、`__pycache__`、Keil 本地界面设置和构建日志均已在 `.gitignore` 中排除。

## 3. 快速开始

### 3.1 接线

| OpenMV | F103 | 方向 |
|---|---|---|
| UART3 TX（常见 P4） | PA10 / USART1_RX | OpenMV → F103，视觉帧 |
| UART3 RX（常见 P5） | PA9 / USART1_TX | F103 → OpenMV，握手 + 遥测 |
| GND | GND | 必须共地 |

115200-8-N-1，3.3 V TTL，**不要接 RS232 电平**。只接 TX 不接 RX 时 OpenMV 会一直停在 `waiting handshake...`，F103 也收不到 ACK、不会发遥测。

### 3.2 编译固件

用 Keil uVision 打开 `rm_dart/MDK-ARM/MA_DartGuide.uvprojx`，目标 `MA_DartGuide`，器件 `STM32F103C8`，预定义宏 `USE_HAL_DRIVER, STM32F103xB`，按 F7 编译。产物在 `rm_dart/MDK-ARM/MA_DartGuide/MA_DartGuide.hex`。

### 3.3 烧录

用 Keil 下载或任意 SWD 工具烧写上述 HEX。烧录后上电，LED 应从蓝（就绪）→ 绿（IMU 就绪）→ 青（舵机就绪）。

### 3.4 部署视觉端

1. 把 `main.py` 拷到 OpenMV 盘符，或在 OpenMV IDE 中打开运行。
2. 插入 FAT 格式 SD 卡（`/sdcard` 或 `/sd` 可写），否则录像与 CSV 会被跳过。
3. 先给 F103 上电，再运行 `main.py`。终端应依次出现：

```text
=== BOOT ===
[DEBUG] sensor init OK, attempt 1
init successful
waiting handshake...
handshake ok, mode = 3
entered main loop, mode = 3
RECORD_DIR = /sdcard
```

4. 飞行后可见 `record start: /sdcard/rec_000XX.bin` 与 `record stop`，遥测日志为同目录 `telem_000XX.csv`。

### 3.5 主机测试

```bash
python -m unittest discover -s tests -v
```

14 项用例，覆盖 8 字节视觉帧字节序与 CRC、忙时丢帧、跟踪 ROI 裁剪、最大色块选择、YPR 刷新、录像触发/尾录/上限/写失败、遥测 CSV 记录、回放路径查找。测试通过 AST 抽取脚本顶层定义并注入假对象，**不接触 OpenMV 硬件，也不覆盖 STM32 控制逻辑**。

## 4. 通信协议速查

完整字段、时序与状态机见 [docs/技术手册.md](docs/技术手册.md#4-通信协议)。

### 4.1 握手、应答与复位

| 方向 | 帧 | 长度 | 说明 |
|---|---|---|---|
| F103 → OpenMV | `A5 MODE FLAGS FE CRC8` | 5 B | 每 100 ms 一次，直到收到 ACK；FLAGS bit0 = IMU 就绪 |
| OpenMV → F103 | `5A MODE FE CRC8` | 4 B | 连发 3 次；MODE 必须与 F103 的 `DART_MODE` 一致 |
| OpenMV → F103 | `5B 5B 5B CRC8` | 4 B | 上电时连发 3 次，清除握手锁存，允许重新握手 |

MODE：`0` default、`1` pitch-only、`2` yaw-only、`3` both（默认 `DART_MODE_BOTH`）。收到合法 ACK 后 F103 置 `attitude_tx_armed`，开始 50 Hz 遥测。

### 4.2 视觉上行帧（OpenMV → F103，8 字节）

| 字节 | 内容 |
|---|---|
| 0 | `0xAA` |
| 1-2 | X 偏移，int16 大端，相对制导中心，向右为正 |
| 3-4 | Y 偏移，int16 大端，相对制导中心，向下为正 |
| 5 | `recognized`：1 识别到目标，0 未识别 |
| 6 | `0xFF` |
| 7 | CRC8，覆盖字节 0..6 |

偏移超出 `|dx| ≤ 160`、`|dy| ≤ 120` 时按无目标处理。未识别时发送全零。

### 4.3 遥测下行帧（F103 → OpenMV，37 字节，50 Hz）

| 字节 | 内容 |
|---|---|
| 0 | `0x55` |
| 1-6 | 陀螺 X/Y/Z，int16 大端，0.1 °/s |
| 7-12 | 当前 pitch/yaw/roll，int16 大端，0.01° |
| 13-18 | 输出目标 pitch/yaw/roll，int16 大端，0.01° |
| 19 | mode（0..4，4 = debug） |
| 20 | state（0..7，7 = debug） |
| 21 | flags |
| 22-23 | LOS yaw 增量，int16 大端，0.01°（相对上电 1 s 参考角） |
| 24-25 | LOS pitch 增量，int16 大端，0.01°（同上） |
| 26-27 | 舵机 1 角度，int16 大端，0.1°（1550 µs = 0°） |
| 28-29 | 舵机 2 角度，int16 大端，0.1° |
| 30-31 | 舵机 3 角度，int16 大端，0.1° |
| 32-33 | 舵机 4 角度，int16 大端，0.1° |
| 34 | 调试标志：bit0 = 参考角已捕获 |
| 35 | `0xFE` |
| 36 | CRC8，覆盖字节 0..35 |

flags：bit0 视觉链路有效、bit1 目标有效、bit2 IMU 就绪、bit3 握手完成、bit4 输出角有效。舵机角按 1550 µs = 0°、±1000 µs = ±90° 线性换算，仅作调试观察。CRC8 统一为初值 `0x00`、多项式 `0x8C` 的反射算法。

## 5. 关键参数速查

| 参数 | 值 | 宏 / 位置 |
|---|---|---|
| 控制周期 | 2 ms（标称 500 Hz） | `DART_CONTROL_PERIOD_MS` |
| 最大 dt | 20 ms | `DART_MAX_CONTROL_DT_MS` |
| 起飞加速度阈值 | 22.0 m/s²，连续 5 周期 | `DART_LAUNCH_ACCEL_THRESHOLD` |
| 起飞姿态采样 | 5 帧，yaw 圆均值 | `DART_LAUNCH_ATTITUDE_SAMPLE_COUNT` |
| 上升 pitch 目标 | +25°（映射后角度） | `DART_ASCENT_PITCH_TARGET_DEG` |
| 上升固定抬升 | 100 µs，低头到目标后转 PID | `DART_ASCENT_PITCH_LIFT_US` |
| 下降判据 | 映射后 pitch < 5°，连续 25 周期，且已上升 300 ms | `DART_DESCENT_PITCH_THRESHOLD_DEG` |
| 前视时间 / 前视限幅 | 0.15 s / ±5° | `DART_LEAD_TIME_S`、`DART_LEAD_MAX_DEG` |
| 视线角速率 / 指令限幅 | ±30 °/s、±20° | `DART_LOS_RATE_LIMIT_DEG_S`、`DART_LEAD_COMMAND_LIMIT_DEG` |
| 自稳 PID | yaw 5/0.01/0.5，pitch 10/0.05/0.5，roll 10/0.01/1.0 | `DART_*_STAB_*` |
| 视觉 PID | yaw 5/0.1/0.5，pitch 10/0.1/5.0 | `DART_*_VISION_*` |
| 舵机 | 中位 1550 µs，行程 550–2550 µs，步进 30 µs/周期 | `DART_SERVO_*` |
| 图像 | QVGA 320×240；640×480 读出窗口，等效水平视场约 19° | `DART_IMAGE_*`、`DART_CAMERA_FOCAL_PX` |
| 视觉链路 / 目标保持 | 250 ms / 120 ms，500 ms 复位 EKF | `DART_VISION_LINK_TIMEOUT_MS` 等 |
| 录像 | 起飞开始，下降 +3 s 停止，上限 30 s | `main.py` |

完整参数表（含含义、单位与代码位置）见 [docs/技术手册.md](docs/技术手册.md#7-参数总表)。

## 6. 构建与验证状态

| 项目 | 状态 | 日期 |
|---|---|---|
| Keil 全量编译 | 0 Error / 0 Warning；Code=38964、RO=2768、RW=548、ZI=18604 | 2026-11-02 22:49 |
| RAM 占用 | 19152 / 20480 B，余量 1328 B | 2026-11-02 |
| 主机测试 | 14 项全部通过（Python 3.12.14） | 2026-11-03 |
| 实机验证 | **未完成**：舵机方向、IMU 轴向、PID 与引导参数均未上板确认 | — |

> 上述编译结果来自 `rm_dart/MDK-ARM/build_vision_filter_final.log`。该日志早于 `ctl_task.h` 的最后一次改动（2026-11-02 23:08），重新编译后请以新日志为准。

## 7. 版本管理

- **不再使用 V2、V3 等目录名或文档后缀。** 版本信息由 Git 提交历史与标签表达，工作目录和文档标题只保留工程名 `rm_dart`。
- 当前基线标签：`v0.1.0`（2026-11-03，首次纳入 Git 管理）。
- 标签格式：`v<主>.<次>.<修订>`，主版本用于不兼容的协议或控制结构变更，次版本用于功能变更，修订用于参数与文档修正。
- 提交信息格式：`类型: 摘要`，类型取 `feat`、`fix`、`docs`、`refactor`、`test`、`chore`。
- `main` 分支保持可编译；实验性改动在 `feature/*` 分支完成后再合并。

## 8. 安全须知

1. 这是飞行控制代码。**不得**在未做台架验证的情况下直接飞行。
2. 首次上电拆动力或牢固约束机体，逐项确认舵机中位、方向、限位、LED 颜色和 IMU 轴向。
3. 上电时保持机体静止：BMI088 做 6000 点零偏标定，标定期间晃动会得到错误零偏。
4. 从小增益、短时、低速开始整定，确认舵面始终阻碍对应轴的人工转动。
5. 系统没有磁力计、测距、着陆检测和失控保护，yaw 会漂移，下降段结束后没有终止状态。

## 9. 文档

| 文档 | 内容 |
|---|---|
| `README.md` | 总览、结构、接线、协议速查、构建与验证、版本管理 |
| [`docs/技术手册.md`](docs/技术手册.md) | 硬件与引脚、启动时序、协议细节、状态机与控制算法、视觉端、参数总表、排障、风险与后续计划 |

发现文档与代码不一致时，以源码为准，并在提交中同时修正文档与代码。
