/**
 ******************************************************************************
 * @file    ctl_task.h
 * @brief   引导飞行器控制模块的配置、类型定义和公共接口。
 *          该文件包含飞行状态机、视觉处理、PID 控制、舵机输出等相关的
 *          宏定义、数据结构和函数声明。
 ******************************************************************************
 */
#ifndef __CTL_TASK_H
#define __CTL_TASK_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "controller.h"

/* ------------------------------ 数学常量 ------------------------------- */
/* 数学常量：角度与弧度互转 */
#define DART_DEG_TO_RAD                     0.01745329251994329577f  /* 度 -> 弧度 */
#define DART_RAD_TO_DEG                    57.295779513082320876f   /* 弧度 -> 度 */

/* ------------------------------ 板级配置 ------------------------------- */
/* 板级硬件配置 */
#define DART_LED_GPIO_PORT                  GPIOA                   /* RGB LED 所在 GPIO 端口 */
#define DART_LED_RED_PIN                    GPIO_PIN_4              /* 红色 LED 引脚 */
#define DART_LED_GREEN_PIN                  GPIO_PIN_5              /* 绿色 LED 引脚 */
#define DART_LED_BLUE_PIN                   GPIO_PIN_6              /* 蓝色 LED 引脚 */
#define DART_LED_ACTIVE_LEVEL               GPIO_PIN_RESET          /* LED 低电平点亮 */

#define DART_SERVO_CENTER_US                1550U                   /* 舵机中位脉宽（微秒） */
#define DART_SERVO_MIN_US                    550U                   /* 舵机最小脉宽（微秒） */
#define DART_SERVO_MAX_US                   2550U                   /* 舵机最大脉宽（微秒） */
#define DART_SERVO_MAX_STEP_US                30U                   /* 单周期舵机脉宽最大变化量（微秒），保护舵机 */

/* ---------------------------- 视觉 UART 协议 --------------------------- */
/* 视觉 UART 通信协议 */
/* 帧格式：AA xH xL yH yL recognized FF CRC8。CRC 覆盖字节 0..6。
   x/y 为相对引导中心（ROI 中心）的有符号像素偏移，int16 大端，
   正值分别表示向右、向下。 */
#define DART_VISION_FRAME_LEN                  8U                   /* 帧总长度 */
#define DART_VISION_SOF_INDEX                  0U                   /* 帧头索引 */
#define DART_VISION_X_H_INDEX                  1U                   /* X 偏移高字节索引 */
#define DART_VISION_X_L_INDEX                  2U                   /* X 偏移低字节索引 */
#define DART_VISION_Y_H_INDEX                  3U                   /* Y 偏移高字节索引 */
#define DART_VISION_Y_L_INDEX                  4U                   /* Y 偏移低字节索引 */
#define DART_VISION_RECOGNIZED_INDEX           5U                   /* 识别标志索引 */
#define DART_VISION_EOF_INDEX                  6U                   /* 帧尾索引 */
#define DART_VISION_CRC_INDEX                  7U                   /* CRC 校验字节索引 */
#define DART_VISION_SOF                     0xAAU                   /* 帧头常量 */
#define DART_VISION_EOF                     0xFFU                   /* 帧尾常量 */
#define DART_VISION_CRC8_INIT               0x00U                   /* CRC8 初始值 */
#define DART_VISION_CRC8_POLY               0x8CU                   /* CRC8 多项式 */

/* ---------------------------- 姿态遥测协议 ----------------------------- */
/* 姿态遥测 UART 协议（F103 -> OpenMV，49 字节大端）：
   55 gyro(6) att(6) out(6) mode state flags los(4) servo(8) dbgFlags
      dpsi(2) a_cmd(2) aoa(2) vis_rate(2) vis_x(2) vis_y(2) FE CRC8
   gyro 0.1 deg/s，角度/LOS/速率 0.01，舵机角 0.1 deg；CRC 覆盖 0..47。 */
#define DART_ATT_FRAME_LEN                    49U
#define DART_ATT_SOF_INDEX                     0U
#define DART_ATT_GYRO_X_H_INDEX                1U
#define DART_ATT_GYRO_X_L_INDEX                2U
#define DART_ATT_GYRO_Y_H_INDEX                3U
#define DART_ATT_GYRO_Y_L_INDEX                4U
#define DART_ATT_GYRO_Z_H_INDEX                5U
#define DART_ATT_GYRO_Z_L_INDEX                6U
#define DART_ATT_PITCH_H_INDEX                 7U
#define DART_ATT_PITCH_L_INDEX                 8U
#define DART_ATT_YAW_H_INDEX                   9U
#define DART_ATT_YAW_L_INDEX                  10U
#define DART_ATT_ROLL_H_INDEX                 11U
#define DART_ATT_ROLL_L_INDEX                 12U
#define DART_ATT_OUTPUT_PITCH_H_INDEX         13U
#define DART_ATT_OUTPUT_PITCH_L_INDEX         14U
#define DART_ATT_OUTPUT_YAW_H_INDEX           15U
#define DART_ATT_OUTPUT_YAW_L_INDEX           16U
#define DART_ATT_OUTPUT_ROLL_H_INDEX          17U
#define DART_ATT_OUTPUT_ROLL_L_INDEX          18U
#define DART_ATT_MODE_INDEX                   19U
#define DART_ATT_STATE_INDEX                  20U
#define DART_ATT_FLAGS_INDEX                  21U
#define DART_ATT_LOS_YAW_H_INDEX              22U
#define DART_ATT_LOS_YAW_L_INDEX              23U
#define DART_ATT_LOS_PITCH_H_INDEX            24U
#define DART_ATT_LOS_PITCH_L_INDEX            25U
#define DART_ATT_SERVO1_H_INDEX               26U
#define DART_ATT_SERVO1_L_INDEX               27U
#define DART_ATT_SERVO2_H_INDEX               28U
#define DART_ATT_SERVO2_L_INDEX               29U
#define DART_ATT_SERVO3_H_INDEX               30U
#define DART_ATT_SERVO3_L_INDEX               31U
#define DART_ATT_SERVO4_H_INDEX               32U
#define DART_ATT_SERVO4_L_INDEX               33U
#define DART_ATT_DBG_FLAGS_INDEX              34U
/* 2026-11-18 新增：yaw 制导调试量（PN + 加速度内环）。
   帧长 37 -> 45，EOF/CRC 后移到 43/44。上位机 main.py 必须同步。
   2026-11-20：YAW_U 槽改报惯性视线角速率（度/秒），并新增绿灯像素坐标 X/Y，
   帧长 45 -> 49，EOF/CRC 后移到 47/48。main.py 必须同步。 */
#define DART_ATT_A_MEAS_H_INDEX               35U
#define DART_ATT_A_MEAS_L_INDEX               36U
#define DART_ATT_A_CMD_H_INDEX                37U
#define DART_ATT_A_CMD_L_INDEX                38U
#define DART_ATT_PN_OMEGA_H_INDEX             39U
#define DART_ATT_PN_OMEGA_L_INDEX             40U
#define DART_ATT_VIS_RATE_H_INDEX             41U
#define DART_ATT_VIS_RATE_L_INDEX             42U
#define DART_ATT_VIS_X_H_INDEX                43U
#define DART_ATT_VIS_X_L_INDEX                44U
#define DART_ATT_VIS_Y_H_INDEX                45U
#define DART_ATT_VIS_Y_L_INDEX                46U
#define DART_ATT_EOF_INDEX                    47U
#define DART_ATT_CRC_INDEX                    48U
#define DART_ATT_SOF                        0x55U
#define DART_ATT_EOF                        0xFEU
#define DART_ATT_ANGLE_SCALE               100.0f
#define DART_ATT_GYRO_SCALE                 10.0f
#define DART_ATT_LOS_SCALE                 100.0f   /* LOS 增量 0.01 度 */
#define DART_ATT_SERVO_SCALE                10.0f   /* 舵机角 0.1 度 */
#define DART_ATT_ACCEL_SCALE               100.0f   /* 加速度 0.01 m/s^2 */
#define DART_ATT_RATE_SCALE                100.0f   /* 角速率 0.01 度/秒 */
#define DART_ATT_SERVO_DEG_PER_US           0.09f   /* 1550 us = 0 度，+-1000 us = +-90 度 */
#define DART_DBG_FLAG_REF_VALID             0x01U   /* 调试标志 bit0：上电参考角已捕获 */
#define DART_ATT_FLAG_LINK_VALID            0x01U
#define DART_ATT_FLAG_TARGET_VALID          0x02U
#define DART_ATT_FLAG_IMU_READY             0x04U
#define DART_ATT_FLAG_HANDSHAKE_DONE        0x08U
#define DART_ATT_FLAG_OUTPUT_VALID          0x10U
#define DART_ATT_FLAG_DBG_ALWAYS_ON         0x40U   /* 制导调试量每周期更新标记 */
#define DART_ATT_TX_TIMEOUT_MS                 5U

/* ------------------------- 握手 / 模式协议 ----------------------------- */
/* F103 -> OpenMV 握手帧：A5 MODE FLAGS FE CRC8（5 字节）。
   OpenMV -> F103 ACK：5A MODE FE CRC8（4 字节）。
   握手成功（收到 ACK）后 F103 才开始发送姿态遥测。 */
#define DART_HANDSHAKE_SOF                  0xA5U
#define DART_HANDSHAKE_EOF                  0xFEU
#define DART_HANDSHAKE_FRAME_LEN               5U
#define DART_HANDSHAKE_TX_PERIOD_MS          100U
#define DART_ACK_SOF                        0x5AU
#define DART_ACK_FRAME_LEN                     4U
#define DART_RESET_SOF                      0x5BU
#define DART_RESET_FRAME_LEN                   4U

/* 工作模式（编译期选择，握手帧通知 OpenMV；0 保留未用） */
#define DART_MODE_PITCH                       1U
#define DART_MODE_YAW                         2U
#define DART_MODE_BOTH                        3U
#define DART_MODE_DEBUG                       4U   /* 台架调试：不检起飞、不驱舵机，仅上报指令 */
#define DART_MODE                            DART_MODE_BOTH

#define DART_IMAGE_WIDTH_PX                  320.0f    /* OpenMV 输出 QVGA 宽度 */
#define DART_IMAGE_HEIGHT_PX                 240.0f    /* OpenMV 输出 QVGA 高度 */
/* 等效焦距（像素）：f_out = 1921 * 320 / W（W=640 -> 960, W=1280 -> 480）。
   两轴必须相等，避免 roll 旋转后各向异性。 */
#define DART_CAMERA_FOCAL_X_PX                640.0f
#define DART_CAMERA_FOCAL_Y_PX                640.0f
/* 相机安装固定视线偏置（度），默认 0 */
#define DART_CAMERA_YAW_BIAS_DEG               0.0f
#define DART_CAMERA_PITCH_BIAS_DEG             0.0f
#define DART_VISION_LINK_TIMEOUT_MS           250U                  /* 视觉链路超时时间（毫秒） */
#define DART_VISION_STARTUP_GRACE_MS         1000U                  /* 启动宽限期，期间不报链路错误（毫秒） */
#define DART_TARGET_LOST_HOLD_MS              120U                  /* 目标丢失后保持有效的时间（毫秒） */
#define DART_TARGET_REACQUIRE_RESET_MS        500U                  /* 目标丢失超过此时间后复位 EKF 重新捕获（毫秒） */
#define DART_DEBUG_REF_DELAY_MS              1000U                  /* IMU 就绪后采上电参考角的延时（毫秒） */
#define DART_ENABLE_PRELAUNCH_LINK_ERROR_LED    1U                  /* 使能起飞前视觉链路错误 LED 指示 */

/* 1：目标丢失后最多保持下降段最后一次视觉指令 DART_TARGET_REACQUIRE_RESET_MS 毫秒；
   0：vision.valid 清零后立即退出视觉引导。 */
#ifndef DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE
#define DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE 1U
#endif

#if ((DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE != 0U) && \
     (DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE != 1U))
#error "DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE must be 0 or 1"
#endif

/* 视觉滤波（方案 A 双轴）：yaw/pitch 均用机体陀螺积分角延迟对齐 + 惯性系线性 KF，
   彻底消除大俯冲角下欧拉 yaw 的 1/cos(pitch) 放大泄漏与 pitch 未对齐毛刺。 */
#define DART_VISION_DELAY_MS                  20U                   /* 视觉链路延迟补偿（毫秒，MED_WIN=1 实测约 20） */
#define DART_YAW_HIST_LEN                     64U                   /* yaw/pitch 陀螺积分历史缓冲（2 ms * 64 = 128 ms） */
#define DART_VISION_INERT_Q                  1200.0f                /* 惯性系 KF 连续过程噪声 ((度/秒)^2/秒) */
#define DART_VISION_INERT_R_DEG2               0.25f                /* 惯性系 KF 量测方差（度^2，约 0.5 度） */
#define DART_VISION_INERT_GATE_DEG             6.0f                 /* 惯性系创新门限（度） */

/* 视觉滤波模式：0 = EKF/方案 A（默认），1 = RAW_RATE（备用帧间差分） */
#define DART_VISION_FILTER_EKF                  0U
#define DART_VISION_FILTER_RAW_RATE             1U
#ifndef DART_VISION_FILTER_MODE
#define DART_VISION_FILTER_MODE                 DART_VISION_FILTER_EKF
#endif
#define DART_VISION_RATE_LPF_RC_S               0.015f /* RAW_RATE 低通时间常数（秒，约 1/3 帧周期） */
#define DART_VISION_RATE_MIN_DT_MS              5U
#define DART_VISION_RATE_MAX_DT_MS              100U
#define DART_VISION_RAW_RATE_LIMIT_DEG_S         90.0f /* RAW_RATE 帧间速率限幅（度/秒），防框跳变 */
#define DART_VISION_RATE_LOST_DECAY             0.5f   /* 丢帧时速率保留系数，每丢一帧乘一次 */
#define DART_VISION_RATE_LOST_FLOOR_DEG_S       1.0f   /* 衰减到此值以下直接归零（度/秒） */

#if ((DART_VISION_FILTER_MODE != DART_VISION_FILTER_EKF) && \
     (DART_VISION_FILTER_MODE != DART_VISION_FILTER_RAW_RATE))
#error "Invalid DART_VISION_FILTER_MODE"
#endif

/* ---------------------------- 控制周期与状态机 --------------------------- */
#define DART_CONTROL_PERIOD_MS                  2U                  /* 标称控制周期（毫秒） */
#define DART_MAX_CONTROL_DT_MS                 20U                  /* 控制周期最大允许值，防止 dt 过大（毫秒） */
#define DART_MIN_ASCENT_TIME_MS               300U                  /* 上升段最短持续时间（毫秒） */
#define DART_LED_ERROR_BLINK_HALF_PERIOD_MS   200U                  /* 错误闪烁半周期（毫秒） */

/* ---------------------------- 上电 yaw 零偏标定 -------------------------- */
/* IMU 就绪后静止采集一段时间，测出 yaw 漂移速率并补偿。
   漂移量 = 采集期总偏移 + 速率 × 之后经过的时间；两个都减掉。
   采集期间若平均角速率超限（有人在动），放弃本次标定。 */
#define DART_YAW_CALIB_ENABLE                    1U
#define DART_YAW_CALIB_MS                        2000U   /* 采集时长（毫秒） */
#define DART_YAW_CALIB_MIN_DPS                   0.2f    /* 漂移速率低于此值不补偿（度/秒） */
#define DART_YAW_CALIB_MAX_RATE_RAD_S            0.05f   /* 采集期平均角速率上限，约 2.9 °/s */

/* ------------------------------ 起飞检测 -------------------------------- */
#define DART_IMU_LED_HOLD_MS                  500U                  /* IMU 就绪后等待舵机初始化的时间（毫秒）；关闭 yaw 标定时生效 */
#define DART_LAUNCH_LED_HOLD_MS               250U                  /* 起飞后姿态采集完成到进入上升段的保持时间（毫秒） */
#define DART_LAUNCH_ATTITUDE_SAMPLE_COUNT       5U                  /* 起飞瞬间姿态采样次数 */
#define DART_LAUNCH_ACCEL_THRESHOLD           22.0f                 /* 起飞加速度阈值（m/s2） */
#define DART_LAUNCH_CONFIRM_COUNT                5U                  /* 连续超过阈值次数确认起飞 */

/* ------------------------------ 上升段 ---------------------------------- */
/* 阶段1：pitch >= TARGET 时固定抬升；阶段2：跌破后按 LIFT_PID/LIFT_BANG 回目标角；到达即停。 */
#define DART_ASCENT_PITCH_TARGET_DEG          (23.0f)               /* 阶段1/2 分界角（度） */
#define DART_ASCENT_PITCH_LIFT_US             150.0f                /* 阶段1 固定抬升输出（微秒） */

/* ------------------------------ 下降段 ---------------------------------- */
#define DART_DESCENT_PITCH_THRESHOLD_DEG      (5.0f)                /* 俯仰角低于此值判入下降段（度） */
#define DART_DESCENT_CONFIRM_COUNT               25U                /* 连续满足次数确认进入下降 */
#define DART_DESCENT_GLIDE_ENABLE                1U                 /* 无目标时启用滑翔速率环 */
#define DART_YAW_ABS_LOCK_ENABLE                 0U                 /* 1=脱锁保持最佳航向，0=回发射航向 */
#define DART_YAW_BEST_MARGIN_DEG                0.15f               /* 刷新最佳航向所需 |LOS| 改进量（度） */
#ifndef DART_YAW_BEST_TRACK_ENABLE
#define DART_YAW_BEST_TRACK_ENABLE               1U                 /* 独立于 ABS_LOCK：跟踪记录最佳航向 */
#endif
#define DART_YAW_BEST_CONFIRM_FRAMES             2U                 /* 连续改善帧数门限，防单帧跳变 */
#define DART_DESCENT_UNLOCK_LOST_FRAMES          2U                 /* 连续丢视觉帧达此值即脱锁（约 60 ms） */
/* 俯仰速率环（滑翔/视觉共用）：rate_cmd = clamp(GAIN * (pitch - target)), out = KD * (rate_cmd - gy) */
#define DART_GLIDE_PITCH_DEG                (-45.0f)   /* 基准滑翔俯仰角（度） */
#define DART_GLIDE_RATE_GAIN                  5.0f     /* 滑翔段角度->速率增益（1/秒） */
#define DART_GLIDE_MAX_RATE                  80.0f     /* 滑翔段最大低头速率（度/秒） */
#define DART_VISION_PITCH_RATE_GAIN           8.0f     /* 视觉段角度->速率增益（1/秒） */
#define DART_VISION_PITCH_MAX_RATE          80.0f     /* 视觉段最大低头速率（度/秒） */
#define DART_GLIDE_RATE_KD                    2.5f     /* 俯仰速率环增益（微秒/(度/秒)） */
/* 不对称限幅：低头侧实测 >17 度力矩反转，150 us=13.5 度留余量；抬头侧刹车允许至 250 us */
#define DART_GLIDE_MAX_OUTPUT_DOWN_US       150.0f   /* 低头侧限幅（微秒） */
#define DART_GLIDE_MAX_OUTPUT_UP_US         250.0f   /* 抬头侧限幅（微秒） */

/* ------------------------------ 落地检测 -------------------------------- */
/* 下降段三轴合成角速率突增。
   11.11 实测 21 发：飞行中最大 497 °/s，落地帧 510~3326 °/s（跳变 10~149 倍）。
   单帧 >500 立即判定；>400 需连续两帧，覆盖软着陆（28 发仅 510）。
   进入 LANDED 后停四路 PWM，舵机自由。 */
#define DART_LANDING_DETECT_ENABLE               1U
#define DART_LANDING_OMEGA_DPS                 500.0f
#define DART_LANDING_OMEGA_SOFT_DPS            400.0f
/* 换算成 (rad/s)^2，避免运行时开方：x^2 * (pi/180)^2 */
#define DART_LANDING_W2_HARD  (DART_LANDING_OMEGA_DPS * DART_LANDING_OMEGA_DPS * 0.00030462f)
#define DART_LANDING_W2_SOFT  (DART_LANDING_OMEGA_SOFT_DPS * DART_LANDING_OMEGA_SOFT_DPS * 0.00030462f)

/* ------------------------ 前视追踪（lead pursuit）调参 ------------------- */
/* 指令 = 视线角 + 前视时间 × 惯性系视线角速率。
   目标静止时前视项只补偿本机转弯与舵机滞后，不承担“导航比”的作用。 */
#define DART_LEAD_TIME_S                  0.20f   /* 前视时间（秒）；实测回路延迟约 50~80 ms，0.20 留足余量 */
#define DART_LEAD_MAX_DEG                  3.0f   /* 前视项单独限幅（度）；= LEAD_TIME*LOS_RATE_LIMIT，两者同时卡住 */
#define DART_LEAD_RATE_DEADBAND_DEG_S      1.0f   /* 视线角速率死区（度/秒），抑制画面噪声引起的抖动 */
#define DART_LOS_RATE_LIMIT_DEG_S         15.0f   /* 视线角速率限幅（度/秒）；与 LEAD_TIME 相乘应略小于 LEAD_MAX */
/* yaw 总指令限幅（度）：需小于半视场（18.45 度），15 度可覆盖末段高速扫出 */
#define DART_LEAD_COMMAND_LIMIT_DEG       15.0f
/* pitch 指令单独限幅（度）：经速率环已足够覆盖目标相对视线率，防止极端帧过驱 */
#define DART_PITCH_COMMAND_LIMIT_DEG      10.0f

/* 下降段引导输出反向力矩：只影响 DESCENT 的 pitch PID 输出。
   yaw 没有这个符号——视觉 yaw 与上升段自稳走完全相同的输出路径，
   坐标系差异统一由 DART_CAMERA_YAW_SIGN 一处负责。 */
#define DART_DESCENT_PITCH_OUTPUT_SIGN   (-1.0f)

/* -------------------------------- PID 调参 ----------------------------- */
#define DART_PID_MAX_OUTPUT_US                  420.0f               /* PID 输出限幅（微秒） */
#define DART_PID_INTEGRAL_LIMIT_US              120.0f               /* 积分项限幅（微秒） */
#define DART_PID_DEADBAND_DEG                     0.20f              /* 死区（度） */
#define DART_PID_CHANGING_A                      10.0f               /* 自稳变速积分 A（|e| > A+B 停积分） */
#define DART_PID_CHANGING_B                       5.0f               /* 自稳变速积分 B（|e| <= B 全额积分） */
#define DART_PID_OUTPUT_LPF_RC                    0.003f             /* 输出低通时间常数（秒，2026-11-20 降延迟） */
#define DART_PID_DERIVATIVE_LPF_RC                0.01f              /* 微分低通时间常数（秒） */
#define DART_PID_OLS_ORDER                        0U                 /* 0 = 用陀螺角速率代替误差微分 */
#define DART_PID_IMPROVE_FLAGS  (Integral_Limit | Trapezoid_Intergral | \
                                 DerivativeFilter | OutputFilter)
/* 11.20 41 发实测：yaw 20/0/2.5 阻尼良好（2ndHalf_pp 中位 0.76 度）但存在 -2.5~-4.2 度静差，
   补 KI=5 + 变速积分消静差；roll 原 10/8/1.0 偏软（下降段被偏航耦合拉偏至 18~32 度），
   提到 15/5/1.8 + 变速积分增刚度防饱和。 */
#define DART_YAW_KP                              20.0f               /* 偏航自稳 KP */
#define DART_YAW_KI                               5.0f               /* 偏航自稳 KI（配变速积分） */
#define DART_YAW_KD                               2.5f               /* 偏航自稳 KD（>4 易 2 Hz 极限环） */
#define DART_PITCH_KP                            10.0f               /* 俯仰自稳 KP */
#define DART_PITCH_KI                             0.0f               /* 俯仰自稳 KI */
#define DART_PITCH_KD                             2.5f               /* 俯仰自稳 KD */
#define DART_ROLL_KP                             15.0f               /* 横滚自稳 KP */
#define DART_ROLL_KI                              5.0f               /* 横滚自稳 KI（配变速积分） */
#define DART_ROLL_KD                              1.8f               /* 横滚自稳 KD */

#define DART_YAW_STAB_KP                         DART_YAW_KP
#define DART_YAW_STAB_KI                         DART_YAW_KI
#define DART_YAW_STAB_KD                         DART_YAW_KD
#define DART_PITCH_STAB_KP                       DART_PITCH_KP
#define DART_PITCH_STAB_KI                       DART_PITCH_KI
#define DART_PITCH_STAB_KD                       DART_PITCH_KD

/* 视觉 yaw PID（MODE 0 / MODE 2 使用）：KI=25 维持稳态攻角，A/B=10/10 覆盖下降段 5~15 度常态误差 */
#define DART_YAW_VISION_KP                       14.0f
#define DART_YAW_VISION_KI                       25.0f
#define DART_YAW_VISION_KD                       3.0f
#define DART_YAW_VISION_CHANGING_A               10.0f
#define DART_YAW_VISION_CHANGING_B               10.0f

/* yaw 制导模式：0=角度 PID / 1=速率型 PN（默认，配方案 A 惯性 KF）/ 2=PID+攻角增益 */
#define DART_YAW_PN_MODE_OFF                     0
#define DART_YAW_PN_MODE_RATE                    1
#define DART_YAW_PN_MODE_AOA                     2
#define DART_YAW_PN_MODE                         DART_YAW_PN_MODE_RATE
#define DART_PN_N                                2.5f   /* 比例导引系数（2026-11-21 由 3.0 收敛防过中心甩头） */
#define DART_PN_SIGN                             1.0f   /* +1 = INS 系左转为正，与 yaw_u 同号 */
#define DART_PN_RATE_MAX                       150.0f   /* 速率指令限幅（度/秒，放宽防大 gz 时过早松舵） */
#define DART_PN_K_RATE                           8.0f   /* 速率环增益（微秒/(度/秒)） */
#define DART_PN_K_LOS                            1.5f   /* 纯视线角居中项（1/秒，不再叠前视项） */
#define DART_PN_OUT_MAX                        250.0f   /* 速率型 PN 偏航输出限幅（微秒） */
#define DART_PN_RATE_LPF_RC_S                    0.0f   /* 可选视线速率低通（秒）；方案 A 下设 0 省滞后 */

/* 下降段 pitch 制导模式：0 = 原速率环（vision_pitch_output），1 = Yaw 同款 PN + 中心对齐 */
#define DART_PITCH_GUIDANCE_RATE_LOOP            0U
#define DART_PITCH_GUIDANCE_PN                   1U
#ifndef DART_PITCH_GUIDANCE_MODE
#define DART_PITCH_GUIDANCE_MODE                 DART_PITCH_GUIDANCE_PN
#endif
#if ((DART_PITCH_GUIDANCE_MODE != DART_PITCH_GUIDANCE_RATE_LOOP) && \
     (DART_PITCH_GUIDANCE_MODE != DART_PITCH_GUIDANCE_PN))
#error "Invalid DART_PITCH_GUIDANCE_MODE"
#endif
#define DART_PITCH_PN_N                          2.5f   /* 俯仰比例导引系数 */
#define DART_PITCH_PN_K_LOS                      3.0f   /* 俯仰纯视线角居中项（1/秒） */
#define DART_PITCH_PN_K_RATE                     2.5f   /* 俯仰速率环增益（微秒/(度/秒)，与滑翔速率环一致） */
#define DART_PITCH_PN_RATE_MAX                 120.0f   /* 俯仰期望速率限幅（度/秒）；输出仍受 -150/+250 us 保护 */

/* yaw 攻角增益（MODE 2）：G(|dpsi|) = MIN + (MAX - MIN)*|dpsi|/(|dpsi| + HALF)，叠加于 PID 输入 */
#define DART_YAW_AOA_GAIN_MIN                   0.8f
#define DART_YAW_AOA_GAIN_MAX                   2.0f
#define DART_YAW_AOA_RAW_HALF                   3.5f
#define DART_YAW_AOA_MAX_DEG                   15.0f
#define DART_PITCH_VISION_KP                     26.0f
#define DART_PITCH_VISION_KI                     0.00f
#define DART_PITCH_VISION_KD                     3.00f

/* 上升段 pitch 策略，由 DART_ASCENT_PITCH_MODE 编译期选择：
     0 HOLD      : 不补偿也不自稳，pitch 输出恒为 0；
     1 LIFT_PID  : 固定抬升到低头，再用角度+速率串级把 pitch 拉回停止角，到达即完成；
     2 LIFT_BANG : 固定抬升到低头，再固定抬头到停止角，之后输出 0，全程开环。 */
#define DART_PITCH_MODE_HOLD                    0U
#define DART_PITCH_MODE_LIFT_PID                1U
#define DART_PITCH_MODE_LIFT_BANG               2U
#ifndef DART_ASCENT_PITCH_MODE
#define DART_ASCENT_PITCH_MODE                  DART_PITCH_MODE_LIFT_PID
#endif
/* 阶段2 参数：PULL_US 仅 LIFT_BANG 用；STOP_DEG 为两种策略共用的停止/目标角 */
#define DART_ASCENT_PITCH_PULL_US               0.0f    /* LIFT_BANG 固定抬头输出（微秒） */
#define DART_ASCENT_PITCH_STOP_DEG              23.0f   /* 抬回此角即停止补偿（度） */
#define DART_ASCENT_PITCH_RATE_GAIN             1.0f    /* LIFT_PID 角度->速率增益（1/秒） */
#define DART_ASCENT_PITCH_MAX_RATE             35.0f    /* LIFT_PID 目标速率限幅（度/秒） */
#define DART_ASCENT_PITCH_RATE_KD               2.0f    /* LIFT_PID 速率环增益（微秒/(度/秒)） */

/* 横滚补偿与水平自稳目标 */
#define DART_ASCENT_ROLL_COMPENSATION_ENABLE    0U      /* 1: 启用 roll 初始补偿，0: 走 roll 自稳 */
#define DART_ASCENT_ROLL_TARGET_DEG             0.0f    /* roll 补偿目标角（度） */
#define DART_ASCENT_ROLL_LIFT_US                100.0f  /* roll 固定抬升输出（微秒） */
#define DART_ROLL_HOLD_DEG                      0.0f    /* roll 自稳目标角（0 度=机翼水平） */

/* ------------------------------ 轴向校准 ------------------------------- */
#define DART_LAUNCH_X_SIGN                        1.0f               /* 起飞加速度 X 轴符号 */
/* 相机右正 -> INS yaw 左正（yaw_ins = -psi, gz = -psi_dot），全系统唯一视觉 yaw 符号位 */
#define DART_CAMERA_YAW_SIGN                     (-1.0f)
#define DART_CAMERA_PITCH_SIGN                   1.0f               /* 与取反后的机体俯仰角一致 */
#define DART_ROLL_OUTPUT_SIGN                     -1.0f               /* 横滚输出符号 */
#define DART_PITCH_OUTPUT_SIGN                     1.0f               /* 坐标取反后保持原舵机方向 */
#define DART_YAW_OUTPUT_SIGN                      -1.0f               /* 偏航输出符号 */

/* 四元数 EKF 的俯仰/横滚定义与本机架相反，在此统一映射。
   偏航仍为陀螺 Z 轴，俯仰角速率为陀螺 Y 轴，横滚角速率为陀螺 X 轴。 */
#define DART_SWAP_INS_PITCH_ROLL                    1U               /* 是否交换 INS 俯仰与横滚 */
#if DART_SWAP_INS_PITCH_ROLL
#define DART_INS_PITCH_DEG(ins_ptr)             (-(ins_ptr)->Roll)   /* 交换后：俯仰角取 -Roll */
#define DART_INS_ROLL_DEG(ins_ptr)              ((ins_ptr)->Pitch)   /* 交换后：横滚角取 Pitch */
#else
#define DART_INS_PITCH_DEG(ins_ptr)             (-(ins_ptr)->Pitch)  /* 不交换：俯仰角取 -Pitch */
#define DART_INS_ROLL_DEG(ins_ptr)              ((ins_ptr)->Roll)    /* 不交换：横滚角取 Roll */
#endif
#define DART_INS_PITCH_RATE_RAD_S(ins_ptr)      (-(ins_ptr)->Gyro[1]) /* 俯仰角速率与角度同号 */
#define DART_INS_ROLL_RATE_RAD_S(ins_ptr)       ((ins_ptr)->Gyro[0]) /* 横滚角速率：陀螺 X 轴 */
#define DART_INS_YAW_RATE_RAD_S(ins_ptr)        ((ins_ptr)->Gyro[2]) /* 偏航角速率：陀螺 Z 轴 */

/**
 * @brief 飞行状态枚举
 */
typedef enum
{
    DART_STATE_READY = 0,       /* 就绪：上电初始状态 */
    DART_STATE_IMU_READY,       /* IMU 就绪：等待舵机初始化 */
    DART_STATE_SERVO_READY,     /* 舵机就绪：等待起飞检测 */
    DART_STATE_LAUNCHED,        /* 已起飞：采集起飞姿态 */
    DART_STATE_ASCENT,          /* 上升段：保持起飞姿态稳定 */
    DART_STATE_DESCENT,         /* 下降段：视觉引导 */
    DART_STATE_VISION_ERROR,    /* 视觉错误：起飞前链路异常指示 */
    DART_STATE_DEBUG,           /* 调试：未发射，识别目标并计算指令，不驱动舵机 */
    DART_STATE_LANDED           /* 已落地：停止 PWM 输出，舵机自由 */
} DartFlightState_e;

/**
 * @brief 单轴视觉 EKF 状态
 */
typedef struct
{
    float angle;                /* 估计角度（弧度） */
    float rate;                 /* 估计角速度（弧度/秒） */
    float p00;                  /* 协方差矩阵 P[0][0] */
    float p01;                  /* 协方差矩阵 P[0][1] */
    float p10;                  /* 协方差矩阵 P[1][0] */
    float p11;                  /* 协方差矩阵 P[1][1] */
    uint8_t initialized;        /* 是否已初始化 */
} DartVisionAxisEkf_t;

/**
 * @brief 视觉邮箱：用于中断与任务之间传递最新视觉帧数据
 */
typedef struct
{
    volatile int16_t x;         /* 目标 X 偏移（相对引导中心，像素） */
    volatile int16_t y;         /* 目标 Y 偏移（相对引导中心，像素） */
    volatile uint8_t recognized;/* 是否识别到目标 */
    volatile uint32_t tick_ms;  /* 帧时间戳（毫秒） */
    volatile uint32_t sequence; /* 帧序号，用于无锁读取一致性判断 */
} DartVisionMailbox_t;

/**
 * @brief 视觉目标遥测数据
 */
typedef struct
{
    int16_t raw_x;              /* 原始 X 偏移（相对引导中心，像素） */
    int16_t raw_y;              /* 原始 Y 偏移（相对引导中心，像素） */
    float los_yaw_deg;          /* 视线偏航角（度） */
    float los_pitch_deg;        /* 视线俯仰角（度） */
    uint32_t last_frame_ms;     /* 最近一帧时间戳 */
    uint32_t last_target_ms;    /* 最近一次识别到目标的时间戳 */
    uint32_t frame_count;       /* 总帧数 */
    uint32_t lost_frame_count;  /* 丢失帧计数 */
    uint32_t crc_error_count;   /* CRC 错误计数 */
    uint8_t link_valid;         /* 链路有效标志 */
    uint8_t recognized;         /* 当前帧是否识别到目标 */
    uint8_t lost_frames;        /* 连续丢失的视觉帧数（识别到即清零） */
    uint8_t valid;              /* 目标是否有效（用于引导） */
} DartVisionTarget_t;

/**
 * @brief 控制遥测数据
 */
typedef struct
{
    DartFlightState_e state;    /* 当前飞行状态 */
    float launch_yaw_deg;       /* 起飞时平均偏航角（度） */
    float launch_pitch_deg;     /* 起飞时平均俯仰角（度） */
    float launch_roll_deg;      /* 起飞时平均横滚角（度） */
    float yaw_output_us;        /* 偏航舵机输出（微秒） */
    float pitch_output_us;      /* 俯仰舵机输出（微秒） */
    float roll_output_us;       /* 横滚舵机输出（微秒） */
    float target_yaw_deg;       /* 引导目标偏航角（机体坐标，度） */
    float target_pitch_deg;     /* 引导目标俯仰角（机体坐标，度） */
    float output_yaw_deg;       /* 当前控制目标偏航角（含自稳/视觉，度） */
    float output_pitch_deg;     /* 当前控制目标俯仰角（含自稳/视觉，度） */
    float output_roll_deg;      /* 当前控制目标横滚角（含自稳，度） */
    uint8_t output_valid;       /* 输出目标角是否有效 */
    uint16_t servo_us[4];       /* 四个舵机实际脉宽（微秒） */
    DartVisionTarget_t vision;  /* 视觉目标数据 */
} DartControlTelemetry_t;

/**
 * @brief 控制运行时数据结构：保存所有全局状态和中间变量
 */
typedef struct
{
    /* --- PID 控制器 --- */
    PID_t yaw_pid;                          /* 偏航自稳 PID */
    PID_t yaw_vision_pid;                   /* 偏航视觉 PID */
    PID_t pitch_pid;                        /* 俯仰自稳/补偿 PID */
    PID_t pitch_vision_pid;                 /* 俯仰视觉 PID */
    PID_t roll_pid;                         /* 横滚 PID */

    /* --- 视觉处理 --- */
    DartVisionAxisEkf_t vision_x_ekf;       /* X 轴视觉 EKF */
    DartVisionAxisEkf_t vision_y_ekf;       /* Y 轴视觉 EKF */
    DartVisionMailbox_t vision_mailbox;     /* 视觉邮箱 */

    /* --- 遥测与运行状态 --- */
    DartControlTelemetry_t telemetry;       /* 遥测数据 */
    volatile DartFlightState_e flight_state;/* 当前飞行状态 */
    volatile uint8_t vision_link_error;     /* 视觉链路错误标志 */
    volatile uint8_t imu_ready;             /* IMU 就绪标志 */
    volatile uint8_t attitude_tx_armed;     /* 收到过有效视觉帧后置 1，之后持续发送姿态 */
    uint8_t servo_ready;                    /* 舵机就绪标志 */
    uint8_t initialized;                    /* 控制模块初始化标志 */

    /* --- 串口接收 --- */
    uint8_t uart_rx_byte;                   /* 串口接收单字节缓冲 */
    uint8_t uart_frame[DART_VISION_FRAME_LEN]; /* 视觉帧接收缓冲 */
    uint8_t uart_frame_index;               /* 当前接收帧索引 */

    /* --- 时间戳 --- */
    uint32_t state_enter_ms;                /* 进入当前状态的时间戳 */
    uint32_t control_start_ms;              /* 控制启动时间戳 */
    uint32_t last_control_ms;               /* 上次控制任务执行时间戳 */
    uint32_t processed_vision_sequence;     /* 已处理的最新视觉帧序号 */

    /* --- 起飞与上升段 --- */
    uint8_t launch_confirm_count;           /* 起飞确认计数 */
    uint8_t attitude_sample_count;          /* 起飞姿态采样计数 */
    float yaw_sin_sum;                      /* 偏航角正弦累加，用于圆均值 */
    float yaw_cos_sum;                      /* 偏航角余弦累加，用于圆均值 */
    float roll_sum;                         /* 横滚角累加，用于算术平均 */
    float pitch_sum;                        /* 俯仰角累加，用于算术平均 */
    uint8_t ascent_pitch_done;              /* 本次起飞的 pitch 补偿已停止 */
    uint8_t ascent_pitch_below_target;      /* 当前 pitch 曾低于补偿目标（固定抬升转为 PID 闭环） */
    uint8_t ascent_pitch_below_stop;        /* 阶段2 曾把 pitch 压到停止角以下，用于判定补偿完成 */
    uint8_t ascent_roll_done;               /* 本次起飞的 roll 补偿已停止 */
    uint8_t ascent_roll_below_target;       /* 当前 roll 曾低于补偿目标（固定抬升转为 PID 闭环） */
    uint8_t ascent_vision_yaw_active;       /* 上升段当前由视觉控制 yaw（脱锁后回自稳） */

    /* --- 上电 yaw 零偏标定 --- */
    uint8_t yaw_calib_active;               /* yaw 零偏标定进行中 */
    uint8_t yaw_calib_done;                 /* yaw 零偏标定完成 */
    uint32_t yaw_calib_end_ms;              /* 标定结束时间戳 */
    uint32_t yaw_calib_count;               /* 标定采样次数 */
    float yaw_calib_start_deg;              /* 标定起始 yaw */
    float yaw_calib_drift_deg;              /* 标定期总漂移 */
    float yaw_bias_dps;                     /* 漂移速率（度/秒） */
    float yaw_calib_gz_sum;                 /* 标定期 |gz| 累加，用于运动检测 */

    /* --- 下降段与落地 --- */
    uint8_t descent_confirm_count;          /* 下降确认计数 */
    uint8_t descent_target_seen;            /* 本次下降已首次识别有效目标 */
    uint8_t descent_vision_active;          /* 下降段当前正在使用视觉制导 */
    uint8_t landed;                         /* 已检测到落地（角速率突增） */
    float landing_prev_w2;                  /* 上一周期角速率平方，用于连两帧判据 */
    float yaw_best_deg;                     /* 目标最居中时的机体航向（度） */
    float yaw_best_los_deg;                 /* 上述时刻的 |视线偏航角|（度） */
    uint8_t yaw_best_valid;                 /* 已记录到最佳航向 */
    uint8_t yaw_best_streak;                /* 连续「更居中」的帧数，防单帧识别跳变 */
} DartControlRuntime_t;

/* 全局运行时数据实例 */
extern DartControlRuntime_t DartControl;

/* ------------------------------- 公共函数 ------------------------------- */
/* 公共函数声明 */
void control_Init(void);                    /* 初始化控制模块 */
void control_Task(void);                    /* 控制主任务，周期调用 */
void servo_Init(void);                      /* 初始化舵机 PWM 并回中 */
void led_Task(void);                        /* LED 状态指示任务 */
void attitude_telemetry_send(void);         /* 向 OpenMV 发送 pitch/yaw/roll 姿态帧 */
void openmv_handshake_send(void);           /* 周期发送握手帧，收到 ACK 后停止 */
void control_notify_imu_ready(uint8_t ready); /* 通知 IMU 就绪状态 */
void control_uart_rx_callback(UART_HandleTypeDef *huart); /* 串口接收回调 */
void control_uart_error_callback(UART_HandleTypeDef *huart); /* 串口错误回调 */
DartFlightState_e control_get_state(void);  /* 获取当前飞行状态 */
const DartControlTelemetry_t *control_get_telemetry(void); /* 获取遥测数据指针 */

#ifdef __cplusplus
}
#endif

#endif
