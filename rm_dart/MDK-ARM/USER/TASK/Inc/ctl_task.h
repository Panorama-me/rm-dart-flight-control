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
/* 姿态遥测 UART 协议（F103 -> OpenMV），帧格式：
   55
      gyroXH gyroXL gyroYH gyroYL gyroZH gyroZL
      pitchH pitchL yawH yawL rollH rollL
      outPitchH outPitchL outYawH outYawL outRollH outRollL
      mode state flags
      losYawH losYawL losPitchH losPitchL
      servo1H servo1L servo2H servo2L servo3H servo3L servo4H servo4L
      dbgFlags FE CRC8
   gyro 单位 0.1 deg/s，角度单位 0.01 deg；
   LOS 为相对上电 1 s 参考角的增量（0.01 deg），舵机角 0.1 deg；
   CRC 覆盖字节 0..35。 */
#define DART_ATT_FRAME_LEN                    37U
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
#define DART_ATT_EOF_INDEX                    35U
#define DART_ATT_CRC_INDEX                    36U
#define DART_ATT_SOF                        0x55U
#define DART_ATT_EOF                        0xFEU
#define DART_ATT_ANGLE_SCALE               100.0f
#define DART_ATT_GYRO_SCALE                 10.0f
#define DART_ATT_LOS_SCALE                 100.0f   /* LOS 增量 0.01 度 */
#define DART_ATT_SERVO_SCALE                10.0f   /* 舵机角 0.1 度 */
#define DART_ATT_SERVO_DEG_PER_US           0.09f   /* 1550 us = 0 度，±1000 us = ±90 度 */
#define DART_DBG_FLAG_REF_VALID             0x01U   /* 调试标志 bit0：上电参考角已捕获 */
#define DART_ATT_FLAG_LINK_VALID            0x01U
#define DART_ATT_FLAG_TARGET_VALID          0x02U
#define DART_ATT_FLAG_IMU_READY             0x04U
#define DART_ATT_FLAG_HANDSHAKE_DONE        0x08U
#define DART_ATT_FLAG_OUTPUT_VALID          0x10U
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

/* 模式：由下位机编译期决定，握手帧携带给 OpenMV。
   default 与 both 当前功能一致（测试/比赛区别的标记）。 */
#define DART_MODE_DEFAULT                     0U
#define DART_MODE_PITCH                       1U
#define DART_MODE_YAW                         2U
#define DART_MODE_BOTH                        3U
#define DART_MODE_DEBUG                       4U
/* 编译期选择工作模式；台架调试时改成 DART_MODE_DEBUG：
   不检测起飞、不驱动舵机，只识别目标并计算/上报视觉指令。 */
#define DART_MODE                            DART_MODE_BOTH

#define DART_IMAGE_WIDTH_PX                  320.0f    /* OpenMV 输出 QVGA，宽度 320 */
#define DART_IMAGE_HEIGHT_PX                 240.0f    /* OpenMV 输出 QVGA，高度 240 */
/* 视觉焦距（像素）。OV5640 全幅 2592x1944、水平 68°：
   f_sensor = 1296/tan(34°) = 1921 px。
   注意：main.py 的 IOCTL_SET_READOUT_WINDOW 排在 set_framesize 之后，
   实测未生效，实际读出窗口是 OpenMV 在 QVGA 下的默认值——传感器中心 640x480。
   640x480 窗口缩放到 320x240 输出，f = (320/2)/tan(atan(320/1921)) = 961 px。
   台架实测：目标偏 5° 时旧值 480 算出 10°（正好 2 倍），改为 961 后吻合。
   若将来修正 ioctl 顺序让 1280x960 真正生效，f 应改回 480（视场 36.8°x28.1°）。
   两轴必须相等，否则 roll 旋转后的指令会出现各向异性（见 9.x）。 */
#define DART_CAMERA_FOCAL_X_PX                961.0f
#define DART_CAMERA_FOCAL_Y_PX                961.0f
/* 相机安装角与读出窗口偏移造成的固定视线偏置（度），默认 0。
   当前 640x480 窗口在垂直方向相对光轴偏移约 20 度；
   若机身轴对准目标时图像不在 (160,120)，把实测偏置填到这里。 */
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

/* 视觉 EKF 调参 */
#define DART_VISION_EKF_PROCESS_NOISE          5.0f                 /* 过程噪声强度 */
#define DART_VISION_EKF_MEAS_NOISE_PX          9.0f                 /* 测量噪声方差（像素平方） */
#define DART_VISION_EKF_GATE_PX              180.0f                 /* 创新量限幅门限（像素） */

/* 原视觉 EKF 仍保留在 Control_Task.c 中，修改本编译期开关即可恢复；
   RAW_RATE 模式不对位置滤波，只对视线角速率做轻量低通。 */
#define DART_VISION_FILTER_EKF                  0U
#define DART_VISION_FILTER_RAW_RATE             1U
#ifndef DART_VISION_FILTER_MODE
#define DART_VISION_FILTER_MODE                 DART_VISION_FILTER_RAW_RATE
#endif
#define DART_VISION_RATE_LPF_RC_S               0.055f  /* 视线角速率低通；应大于相机帧周期，否则几乎不滤波 */
#define DART_VISION_RATE_MIN_DT_MS              5U
#define DART_VISION_RATE_MAX_DT_MS              100U
#define DART_VISION_RAW_RATE_LIMIT_DEG_S        360.0f

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
#define DART_ASCENT_PITCH_TARGET_DEG          (30.0f)               /* 阶段1/2 分界角：pitch 跌破此值转入阶段2 */
/* 阶段1：pitch 尚未低于目标角时输出固定抬升量，抵消发射低头。
   阶段2：角度低于目标后按 LIFT_PID / LIFT_BANG 各自的策略回目标角。
   阶段3：到达目标或进入下降段即停止补偿。 */
#define DART_ASCENT_PITCH_LIFT_US             100.0f               /* 阶段1 固定抬升输出（微秒）；对齐 11.12-15 实测 */
/* 11.12 全部 14 发可见阶段1=≈100us、阶段2=≈110us，切换点无一例外落在 pitch 穿过 30° 时。
   11.11 那批是 LIFT=130 / PULL=100，与 git 基线一致，说明中途改过一次且未提交。 */

/* ------------------------------ 下降段 ---------------------------------- */
#define DART_DESCENT_PITCH_THRESHOLD_DEG      (5.0f)                 /* 取反后的俯仰角低于此值判断进入下降段（度） */
#define DART_DESCENT_CONFIRM_COUNT               25U                 /* 连续满足下降条件次数确认进入下降 */
/* 滑翔总开关：下降段无目标时启用滑翔速率环（见 DART_GLIDE_* 参数）。 */
#define DART_DESCENT_GLIDE_ENABLE                1U
/* 脱锁航向锁定：跟踪期间记录「目标最居中时的机体 yaw」，脱锁后以该角为目标。
   判据是 |视线偏航角| 变小：越接近 0 说明目标越靠近画面中心，此时的 yaw 就是目标方位。
   只记录 yaw、不参与视线角的加减，因此不存在符号歧义。
   1 = 启用（0则保持原行为：脱锁后回发射航向）。 */
#define DART_YAW_ABS_LOCK_ENABLE                 0U
/* 更新最佳航向需要的改进量（度）。防止噪声尖峰把锁定角钉在错误位置：
   只有 |视线角| 比历史最优值再小 0.15 度才更新。 */
#define DART_YAW_BEST_MARGIN_DEG                0.15f
/* 脱锁判据：连续丢失的视觉帧数达到此值即切脱锁锁定，不再等
   DART_TARGET_LOST_HOLD_MS 的 120 ms 超时。按 26 fps 算：
   2 帧 ≈ 77 ms；调成 1 则丢一帧就切（约 38 ms）。+
	 
   注意：帧数按视觉帧累计，不是 2 ms 的控制周期。 */
#define DART_DESCENT_UNLOCK_LOST_FRAMES          2U
/* 滑翔速率环：不直接给目标角，而是控制「低头速率」，让 pitch 缓慢逼近基准角。
   期望速率 = GAIN × (当前角 - 目标角)，限幅 ±MAX_RATE；
   舵面 = RATE_KD × (期望速率 - 实际速率)。
   接近目标时速率自动减小，到达即稳定（gyro ≈ 0），不会过冲。 */
#define DART_GLIDE_PITCH_DEG                (-45.0f)   /* 基准滑翔俯仰角（度） */

/* 滑翔段（无目标）：把机头拉向基准滑翔角，GAIN 5.0 对应时间常数约 0.2 s。
   有了下面的输出限幅兜底，滑翔段不会再把舵面推进力矩反转区。 */
#define DART_GLIDE_RATE_GAIN                  5.0f     /* 每度偏差对应的目标角速率（1/秒） */
#define DART_GLIDE_MAX_RATE                  80.0f     /* 最大低头速率（度/秒） */

/* 视觉段（有目标）：比滑翔段更激进，GAIN 8.0 对应时间常数约 0.125 s。
   与滑翔段共用同一个速率环，但增益和限幅独立——改一段不影响另一段。 */
#define DART_VISION_PITCH_RATE_GAIN           8.0f     /* 视觉指令 -> 目标角速率（1/秒） */
#define DART_VISION_PITCH_MAX_RATE          100.0f     /* 视觉段最大低头速率（度/秒） */

/* 速率环增益（微秒 / (度/秒)），两段共用。 */
#define DART_GLIDE_RATE_KD                    2.5f

/* 舵面输出限幅，两段共用，两侧不对称。
   低头侧：11.14 实测俯仰通道超过约 17 度后力矩反转——00012 推到 -22.1 度，
     机头反而从 -20 抬到 -17，卡住下不去。150 us = 13.5 度，留 3 度余量。
   抬头侧：刹车要靠大偏度，实测 +24 度（00005）仍然正常，不跟着收紧，
     只防极端值。 */
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
#define DART_LEAD_COMMAND_LIMIT_DEG       10.0f   /* 总指令限幅（度），应小于半视场 */

/* 下降段引导输出反向力矩：只影响 DESCENT 的 pitch PID 输出。
   yaw 没有这个符号——视觉 yaw 与上升段自稳走完全相同的输出路径，
   坐标系差异统一由 DART_CAMERA_YAW_SIGN 一处负责。 */
#define DART_DESCENT_PITCH_OUTPUT_SIGN   (-1.0f)

/* -------------------------------- PID 调参 ----------------------------- */
#define DART_PID_MAX_OUTPUT_US                  420.0f               /* PID 输出限幅（微秒） */
#define DART_PID_INTEGRAL_LIMIT_US              120.0f               /* 积分项限幅（微秒） */
#define DART_PID_DEADBAND_DEG                     0.20f              /* 死区（度） */
#define DART_PID_CHANGING_A                      10.0f               /* 积分分离参数 A */
#define DART_PID_CHANGING_B                       2.0f               /* 积分分离参数 B */
#define DART_PID_OUTPUT_LPF_RC                    0.01f              /* 输出低通滤波时间常数 */
#define DART_PID_DERIVATIVE_LPF_RC                0.01f              /* 微分低通滤波时间常数 */
#define DART_PID_OLS_ORDER                        0U  /* 用陀螺角速率代替误差微分 */
#define DART_PID_IMPROVE_FLAGS  (Integral_Limit | Trapezoid_Intergral | \
                                 DerivativeFilter | OutputFilter)      /* PID 改进标志位 */
#define DART_YAW_KP                              20.0f               /* 偏航 PID 比例系数 */
#define DART_YAW_KI                               0.00f               /* 偏航 PID 积分系数 */
/* Kd 的单位是“微秒 /（度/秒）”，符号需在实机上确认。
   11.15 六发实测：舵面 -> 角速率互相关峰值在 198~264 ms（6~8 帧），
   回路增益 KP*0.09*3.5 约 6.3，在 1/(2*0.21)=2.4 Hz 处转满 180 度，
   表现为升降两段都存在的 ±2~4 度、约 2 Hz 极限环。
   按一阶对象 + 纯延迟模型扫参数：KD 1.5 -> 2.5 时残余摆幅
   从 2.5 度降到 2.0 度，而对外部扰动的峰值偏差不变（2.19 度）；
   KD 到 4.0 以上会发散（12 度），2.5 是留足余量的折中。 */
#define DART_YAW_KD                               2.5f              /* 偏航 PID 微分系数 */
#define DART_PITCH_KP                            10.0f               /* 俯仰 PID 比例系数 */
#define DART_PITCH_KI                             0.0f               /* 俯仰 PID 积分系数 */
#define DART_PITCH_KD                             2.5f              /* 俯仰 PID 微分系数 */
#define DART_ROLL_KP                             10.0f               /* 横滚 PID 比例系数 */
#define DART_ROLL_KI                              0.00f               /* 横滚 PID 积分系数 */
#define DART_ROLL_KD                              1.0f    
/* 横滚 PID 微分系数 */

/* 俯仰/偏航在自稳与视觉两种工况下各用一组独立的 PID 参数。 */
#define DART_YAW_STAB_KP                         DART_YAW_KP
#define DART_YAW_STAB_KI                         DART_YAW_KI
#define DART_YAW_STAB_KD                         DART_YAW_KD
#define DART_PITCH_STAB_KP                       DART_PITCH_KP
#define DART_PITCH_STAB_KI                       DART_PITCH_KI
#define DART_PITCH_STAB_KD                       DART_PITCH_KD

/* 视觉 PID 的输入是角度误差（度）。当前焦距 480 px（1280x960 读出窗口），
   同样的像素偏移对应的角度约为全幅（237 px）的 1/2；
   若跟踪偏慢，优先按比例增大 DART_*_VISION_KP。 */
#define DART_YAW_VISION_KP                       20.0f
#define DART_YAW_VISION_KI                       0.0f
#define DART_YAW_VISION_KD                       3.5f
#define DART_PITCH_VISION_KP                     20.0f
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
/* 阶段2 参数。PULL_US 仅 LIFT_BANG 使用；STOP_DEG 两种策略共用：
   LIFT_BANG 拿它当固定抬头的终止角，LIFT_PID 拿它当停止判据与速率环的角度基准。
   调低 STOP_DEG 可提前收手，靠惯性补足剩余角度，减小过冲。 */
#define DART_ASCENT_PITCH_PULL_US               110.0f  /* 阶段2 固定抬头输出（微秒）；对齐 11.12-15 实测 */

#define DART_ASCENT_PITCH_STOP_DEG              23.0f   /* 抬回此角度即停止补偿（度）；两种策略共用 */

/* LIFT_PID 阶段2 的速率环参数（方案 B：角度差 -> 目标速率 -> 速率误差 -> 舵面）
   工作点（target=23）：pitch=20/gy=120 -> 246 us，pitch=12/gy=60 -> 142 us，
   pitch=8/gy=20 -> 70 us。11.12 的 680 帧回放：平均 197 us，3% 打满 420。
   注意：pitch 低于 23 时角度差最大只有 15 度，rate_cmd 远达不到限幅，
   所以 GAIN 与 MAX_RATE 影响很小，真正决定力度的是 RATE_KD。 */
#define DART_ASCENT_PITCH_RATE_GAIN             1.0f    /* 角度差换算成目标速率（1/秒） */
#define DART_ASCENT_PITCH_MAX_RATE             35.0f    /* 目标低头速率限幅（度/秒） */
#define DART_ASCENT_PITCH_RATE_KD               2.0f    /* 速率误差换算成舵面（微秒/(度/秒)） */

//roll补偿
#define DART_ASCENT_ROLL_COMPENSATION_ENABLE    0U      /* 1: 启用 roll 初始补偿，0: 走 roll 自稳 */
#define DART_ASCENT_ROLL_TARGET_DEG             0.0f    /* roll 补偿目标角（度） */
#define DART_ASCENT_ROLL_LIFT_US                100.0f  /* roll 固定抬升输出（微秒），符号需实机确认 */
/* roll 自稳目标角：固定 0°，即机翼水平。
   原逻辑在未补偿时保持起飞横滚角，会把发射架上的初始倾斜带入整段飞行。 */
#define DART_ROLL_HOLD_DEG                      0.0f
/* ------------------------------ 轴向校准 ------------------------------- */
/* 轴向校准：符号与坐标映射 */
#define DART_LAUNCH_X_SIGN                        1.0f               /* 起飞加速度 X 轴符号 */
/* 相机偏航角 -> INS yaw 系的符号。
   本机 IMU 的 pitch/roll 互换且带负号（见 DART_SWAP_INS_PITCH_ROLL），
   INS 的 yaw 轴整体与机体反向：yaw_ins = -psi，gz = -psi_dot。
   相机是正向安装的（画面右 = 机体右），所以相机角度必须取反才能进入
   INS 系，与 yaw_ins / gz 同号。11.14 录像实测相邻帧 dX/dyaw_ins =
   +1.08 / +0.79（正常相机应为 -1），确认反向。

   全系统只保留这一处 yaw 符号：视觉误差、视线角速率、PID 输出都不再
   单独乘符号，与上升段 calculate_yaw_hold 的结构完全一致。
   台架若发现 yaw 朝目标反方向转，只改这一处。 */
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
    float filtered_x;           /* 滤波后 X 偏移（像素） */
    float filtered_y;           /* 滤波后 Y 偏移（像素） */
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
    uint8_t descent_glide_active;           /* 下降段滑翔补偿已锁存 */
    float descent_glide_pitch_deg;          /* 锁存的滑翔俯仰角 */
    uint8_t landed;                         /* 已检测到落地（角速率突增） */
    float landing_prev_w2;                  /* 上一周期角速率平方，用于连两帧判据 */
    float yaw_best_deg;                     /* 目标最居中时的机体航向（度） */
    float yaw_best_los_deg;                 /* 上述时刻的 |视线偏航角|（度） */
    uint8_t yaw_best_valid;                 /* 已记录到最佳航向 */
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
