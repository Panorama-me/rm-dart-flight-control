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
#define DART_MODE                            DART_MODE_DEBUG

#define DART_IMAGE_WIDTH_PX                  320.0f    /* OpenMV 输出 QVGA，宽度 320 */
#define DART_IMAGE_HEIGHT_PX                 240.0f    /* OpenMV 输出 QVGA，高度 240 */
/* 视觉焦距（像素），x/y 两轴分开标定。
   x：暂用 640x480 读出窗口的几何值 961 px，待转台标定后修正；
   y：纵向 240 px 实测约 44.6 度，f = 120 / tan(22.3 度) 约 292.6 px。
   两轴焦距不一致时，roll 旋转后的指令会出现各向异性（见 9.x 说明）；
   若 x 轴实测纵向跨度同样是 44.6 度，应把 x 也改成 292.6。 */
#define DART_CAMERA_FOCAL_X_PX               961.0f
#define DART_CAMERA_FOCAL_Y_PX               292.6f
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
#define DART_VISION_FILTER_EKF                  1U
#define DART_VISION_FILTER_RAW_RATE             0U
#ifndef DART_VISION_FILTER_MODE
#define DART_VISION_FILTER_MODE                 DART_VISION_FILTER_RAW_RATE
#endif
#define DART_VISION_RATE_LPF_RC_S               0.020f
#define DART_VISION_RATE_MIN_DT_MS              5U
#define DART_VISION_RATE_MAX_DT_MS              100U
#define DART_VISION_RAW_RATE_LIMIT_DEG_S        360.0f

#if ((DART_VISION_FILTER_MODE != DART_VISION_FILTER_EKF) && \
     (DART_VISION_FILTER_MODE != DART_VISION_FILTER_RAW_RATE))
#error "Invalid DART_VISION_FILTER_MODE"
#endif

/* ------------------------------ 飞行检测 ------------------------------- */
/* 飞行检测与状态机参数 */
#define DART_CONTROL_PERIOD_MS                  2U                  /* 标称控制周期（毫秒） */
#define DART_MAX_CONTROL_DT_MS                 20U                  /* 控制周期最大允许值，防止 dt 过大（毫秒） */
#define DART_IMU_LED_HOLD_MS                  500U                  /* IMU 就绪后等待舵机初始化的时间（毫秒） */
#define DART_LAUNCH_LED_HOLD_MS               250U                  /* 起飞后姿态采集完成到进入上升段的保持时间（毫秒） */
#define DART_LAUNCH_ATTITUDE_SAMPLE_COUNT       5U                  /* 起飞瞬间姿态采样次数 */
#define DART_LAUNCH_ACCEL_THRESHOLD           22.0f                 /* 起飞加速度阈值（m/s2） */
#define DART_LAUNCH_CONFIRM_COUNT                5U                  /* 连续超过阈值次数确认起飞 */
#define DART_ASCENT_PITCH_TARGET_DEG          (25.0f)               /* 使用取反后的机体俯仰角，保持原物理目标 */
/* 发射后俯仰角尚未低于补偿目标时，输出固定抬升量抵消发射低头；正值表示抬升。
   角度低于目标后改由 pitch PID 闭环拉回目标，到达目标或进入下降段即停止补偿。 */
#define DART_ASCENT_PITCH_LIFT_US             100.0f               /* 发射后固定 pitch 抬升输出（微秒） */
#define DART_DESCENT_PITCH_THRESHOLD_DEG      (5.0f)                 /* 取反后的俯仰角低于此值判断进入下降段（度） */
#define DART_DESCENT_CONFIRM_COUNT               25U                 /* 连续满足下降条件次数确认进入下降 */
#define DART_MIN_ASCENT_TIME_MS                 300U                 /* 上升段最短持续时间（毫秒） */
#define DART_LED_ERROR_BLINK_HALF_PERIOD_MS     200U                 /* 错误闪烁半周期（毫秒） */

/* ------------------------ 前视追踪（lead pursuit）调参 ------------------- */
/* 指令 = 视线角 + 前视时间 × 惯性系视线角速率。
   目标静止时前视项只补偿本机转弯与舵机滞后，不承担“导航比”的作用。 */
#define DART_LEAD_TIME_S                  0.15f   /* 前视时间（秒），按控制滞后取 0.1-0.3 s */
#define DART_LEAD_MAX_DEG                  5.0f   /* 前视项单独限幅（度），防止把目标推出视场 */
#define DART_LEAD_RATE_DEADBAND_DEG_S      2.0f   /* 视线角速率死区（度/秒），抑制画面噪声引起的抖动 */
#define DART_LOS_RATE_LIMIT_DEG_S         30.0f   /* 视线角速率限幅（度/秒） */
#define DART_LEAD_COMMAND_LIMIT_DEG       20.0f   /* 总指令限幅（度），应小于半视场 */

/* 下降段引导输出反向力矩：只影响 DESCENT 的 pitch/yaw PID 输出 */
#define DART_DESCENT_YAW_OUTPUT_SIGN     (-1.0f)
#define DART_DESCENT_PITCH_OUTPUT_SIGN   (-1.0f)


/* 上升段视觉 yaw 输出符号：视觉只控 yaw，默认与下降段视觉输出一致。
   实机上若 yaw 朝目标反方向转，优先单独修改这里。 */
#define DART_ASCENT_VISION_YAW_OUTPUT_SIGN (-1.0f)

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
#define DART_YAW_KP                              5.0f               /* 偏航 PID 比例系数 */
#define DART_YAW_KI                               0.01f               /* 偏航 PID 积分系数 */
/* Kd 的单位是“微秒 /（度/秒）”，符号需在实机上确认。 */
#define DART_YAW_KD                               0.5f              /* 偏航 PID 微分系数 */
#define DART_PITCH_KP                            10.0f               /* 俯仰 PID 比例系数 */
#define DART_PITCH_KI                             0.05f               /* 俯仰 PID 积分系数 */
#define DART_PITCH_KD                             0.5f              /* 俯仰 PID 微分系数 */
#define DART_ROLL_KP                             10.0f               /* 横滚 PID 比例系数 */
#define DART_ROLL_KI                              0.01f               /* 横滚 PID 积分系数 */
#define DART_ROLL_KD                              1.0f              /* 横滚 PID 微分系数 */

/* 俯仰/偏航在自稳与视觉两种工况下各用一组独立的 PID 参数。 */
#define DART_YAW_STAB_KP                         DART_YAW_KP
#define DART_YAW_STAB_KI                         DART_YAW_KI
#define DART_YAW_STAB_KD                         DART_YAW_KD
#define DART_PITCH_STAB_KP                       DART_PITCH_KP
#define DART_PITCH_STAB_KI                       DART_PITCH_KI
#define DART_PITCH_STAB_KD                       DART_PITCH_KD

/* 视觉 PID 的输入是角度误差（度）。焦距从 237 px 改成 961 px 后，
   同样的像素偏移对应的角度约为原来的 1/4，视觉回路等效增益随之降低；
   首次上板若跟踪偏慢，优先按比例增大 DART_*_VISION_KP。 */
#define DART_YAW_VISION_KP                       5.0f
#define DART_YAW_VISION_KI                       0.1f
#define DART_YAW_VISION_KD                       0.5f
#define DART_PITCH_VISION_KP                     10.0f
#define DART_PITCH_VISION_KI                     0.1f
#define DART_PITCH_VISION_KD                     5.0f

/* 上升段初始补偿开关：pitch 轴和 roll 轴分别独立选择是否启用初始补偿。
   启用补偿时沿用现有 pitch 补偿思路：先输出固定抬升量，越过目标角后由 PID 闭环；
   不启用补偿时沿用现有 roll/yaw 自稳思路：保持起飞瞬间的姿态角。 */
#define DART_ASCENT_PITCH_COMPENSATION_ENABLE   1U      /* 1: 启用 pitch 初始补偿，0: 走 pitch 自稳 */
#define DART_ASCENT_ROLL_COMPENSATION_ENABLE    0U      /* 1: 启用 roll 初始补偿，0: 走 roll 自稳 */
#define DART_ASCENT_ROLL_TARGET_DEG             0.0f    /* roll 补偿目标角（度） */
#define DART_ASCENT_ROLL_LIFT_US                100.0f  /* roll 固定抬升输出（微秒），符号需实机确认 */
/* ------------------------------ 轴向校准 ------------------------------- */
/* 轴向校准：符号与坐标映射 */
#define DART_LAUNCH_X_SIGN                        1.0f               /* 起飞加速度 X 轴符号 */
#define DART_CAMERA_YAW_SIGN                      1.0f               /* 相机偏航角符号 */
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
    DART_STATE_DEBUG            /* 调试：未发射，识别目标并计算指令，不驱动舵机 */
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
    PID_t yaw_pid;                          /* 偏航自稳 PID */
    PID_t yaw_vision_pid;                   /* 偏航视觉 PID */
    PID_t pitch_pid;                        /* 俯仰自稳/补偿 PID */
    PID_t pitch_vision_pid;                 /* 俯仰视觉 PID */
    PID_t roll_pid;                         /* 横滚 PID */
    DartVisionAxisEkf_t vision_x_ekf;       /* X 轴视觉 EKF */
    DartVisionAxisEkf_t vision_y_ekf;       /* Y 轴视觉 EKF */
    DartVisionMailbox_t vision_mailbox;     /* 视觉邮箱 */
    DartControlTelemetry_t telemetry;       /* 遥测数据 */
    volatile DartFlightState_e flight_state;/* 当前飞行状态 */
    volatile uint8_t vision_link_error;     /* 视觉链路错误标志 */
    volatile uint8_t imu_ready;             /* IMU 就绪标志 */
    volatile uint8_t attitude_tx_armed;     /* 收到过有效视觉帧后置 1，之后持续发送姿态 */
    uint8_t servo_ready;                    /* 舵机就绪标志 */
    uint8_t initialized;                    /* 控制模块初始化标志 */
    uint8_t uart_rx_byte;                   /* 串口接收单字节缓冲 */
    uint8_t uart_frame[DART_VISION_FRAME_LEN]; /* 视觉帧接收缓冲 */
    uint8_t uart_frame_index;               /* 当前接收帧索引 */
    uint32_t state_enter_ms;                /* 进入当前状态的时间戳 */
    uint32_t control_start_ms;              /* 控制启动时间戳 */
    uint32_t last_control_ms;               /* 上次控制任务执行时间戳 */
    uint32_t processed_vision_sequence;     /* 已处理的最新视觉帧序号 */
    uint8_t launch_confirm_count;           /* 起飞确认计数 */
    uint8_t descent_confirm_count;          /* 下降确认计数 */
    uint8_t ascent_pitch_done;              /* 本次起飞的 pitch 补偿已停止 */
    uint8_t ascent_pitch_below_target;      /* 当前 pitch 曾低于补偿目标（固定抬升转为 PID 闭环） */
    uint8_t ascent_roll_done;               /* 本次起飞的 roll 补偿已停止 */
    uint8_t ascent_roll_below_target;       /* 当前 roll 曾低于补偿目标（固定抬升转为 PID 闭环） */
    uint8_t ascent_vision_yaw_active;       /* 上升段当前由视觉控制 yaw（脱锁后回自稳） */
    uint8_t descent_target_seen;            /* 本次下降已首次识别有效目标 */
    uint8_t descent_vision_active;          /* 下降段当前正在使用视觉制导 */
    uint8_t attitude_sample_count;          /* 起飞姿态采样计数 */
    float yaw_sin_sum;                      /* 偏航角正弦累加，用于圆均值 */
    float yaw_cos_sum;                      /* 偏航角余弦累加，用于圆均值 */
    float roll_sum;                         /* 横滚角累加，用于算术平均 */
    float pitch_sum;                        /* 俯仰角累加，用于算术平均 */
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
