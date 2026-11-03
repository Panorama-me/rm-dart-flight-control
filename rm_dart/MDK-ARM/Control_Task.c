#include "ctl_task.h"

#include "ins_task.h"
#include "tim.h"
#include "usart.h"

#include <math.h>
#include <string.h>

/* 全局飞控运行时数据结构，保存状态、PID、遥测、视觉滤波等信息 */
DartControlRuntime_t DartControl;

/* 在 control_Init() 中只赋值一次，避免每个控制周期重复计算。 */
static float VisionFocalPxX;
static float VisionFocalPxY;
static float debug_ref_yaw_deg;
static float debug_ref_pitch_deg;
static uint32_t debug_imu_ready_ms;
static uint8_t debug_ref_valid;

/* 握手 ACK 接收缓冲（OpenMV -> F103） */
static uint8_t ack_buf[DART_ACK_FRAME_LEN];
static uint8_t ack_index = 0U;
static uint8_t reset_buf[DART_RESET_FRAME_LEN];
static uint8_t reset_index = 0U;

/**
 * @brief 浮点数限幅
 * @param value 输入值
 * @param minimum 下限
 * @param maximum 上限
 * @return 限幅后的值
 */
static float constrain_float(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

/**
 * @brief 将角度归一化到 [-180, 180] 度
 * @param angle 输入角度（度）
 * @return 归一化后的角度
 */
static float wrap_180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

/**
 * @brief 设置飞行状态并记录进入该状态的时间戳
 * @param state 目标飞行状态
 */
static void set_state(DartFlightState_e state)
{
    DartControl.flight_state = state;
    DartControl.telemetry.state = state;
    DartControl.state_enter_ms = HAL_GetTick();
}

/**
 * @brief 复位单个 PID 控制器的内部状态
 * @param pid PID 结构体指针
 */
static void reset_pid(PID_t *pid)
{
    pid->Iout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->DWT_CNT = DWT->CYCCNT;
}

/**
 * @brief 复位所有控制轴（偏航、俯仰、横滚）的 PID
 */
static void reset_all_control_pids(void)
{
    reset_pid(&DartControl.yaw_pid);
    reset_pid(&DartControl.yaw_vision_pid);
    reset_pid(&DartControl.pitch_pid);
    reset_pid(&DartControl.pitch_vision_pid);
    reset_pid(&DartControl.roll_pid);
}

/* 陀螺角速率（输入单位 rad/s）作为 PID 的微分反馈。 */
static void gyro_rate_derivative(PID_t *pid)
{
    pid->Dout = -pid->Kd * pid->Measure;
}

static float calculate_axis_pid(PID_t *pid, float error, float rate_rad_s)
{
    float rate_deg_s = rate_rad_s * DART_RAD_TO_DEG;

    /* Ref - Measure 仍等于角度误差；Measure 输入的是陀螺角速率。 */
    return PID_Calculate(pid, rate_deg_s, rate_deg_s + error);
}

/**
 * @brief 初始化单轴 PID 参数
 * @param pid PID 结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
static void init_axis_pid(PID_t *pid, float kp, float ki, float kd)
{
    PID_Init(pid,
             DART_PID_MAX_OUTPUT_US,
             DART_PID_INTEGRAL_LIMIT_US,
             DART_PID_DEADBAND_DEG,
             kp,
             ki,
             kd,
             DART_PID_CHANGING_A,
             DART_PID_CHANGING_B,
             DART_PID_OUTPUT_LPF_RC,
             DART_PID_DERIVATIVE_LPF_RC,
             DART_PID_OLS_ORDER,
             DART_PID_IMPROVE_FLAGS);
    pid->User_Func2_f = gyro_rate_derivative;
}

/**
 * @brief 设置指定舵机通道的 PWM 比较值（脉宽，单位 us）
 * @param index 舵机索引 0~3
 * @param pulse_us 脉宽微秒值
 *
 * 注意：索引与定时器通道的映射关系在 switch 中定义，需与硬件连接一致。
 */
static void set_servo_compare(uint8_t index, uint16_t pulse_us)
{
    DartControl.telemetry.servo_us[index] = pulse_us;
    switch (index)
    {
    case 0U:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_us);
        break;
    case 1U:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us);
        break;
    case 2U:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
        break;
    default:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_us);
        break;
    }
}

/**
 * @brief 舵机混控输出
 * @param pitch_us 俯仰通道控制量（us）
 * @param yaw_us 偏航通道控制量（us）
 * @param roll_us 横滚通道控制量（us）
 *
 * 说明：采用 X 型尾翼布局进行混控，将三个轴的指令分配到四个舵机。
 *       输出前进行限幅和步进限制，防止舵机行程过大或变化过快。
 */
static void write_servo_outputs(float pitch_us, float yaw_us, float roll_us)
{
    float mixed[4];
    uint16_t requested;
    uint16_t previous;
    uint8_t i;

    /* 应用各轴输出符号，适配机械安装方向 */
    pitch_us *= DART_PITCH_OUTPUT_SIGN;
    yaw_us *= DART_YAW_OUTPUT_SIGN;
    roll_us *= DART_ROLL_OUTPUT_SIGN;

    /* X 型尾翼混控分配。舵机编号与方向必须在约束状态下逐项验证。 */
    /* X 型尾翼分配：四个舵机分别组合俯仰、偏航、横滚指令 */
    mixed[0] = pitch_us + yaw_us + roll_us;
    mixed[1] = pitch_us - yaw_us + roll_us;
    mixed[2] = -pitch_us - yaw_us + roll_us;
    mixed[3] = -pitch_us + yaw_us + roll_us;

    for (i = 0U; i < 4U; i++)
    {
        /* 将混控量限制在舵机机械行程范围内 */
        mixed[i] = constrain_float(mixed[i],
                                   (float)DART_SERVO_MIN_US - DART_SERVO_CENTER_US,
                                   (float)DART_SERVO_MAX_US - DART_SERVO_CENTER_US);
        requested = (uint16_t)((float)DART_SERVO_CENTER_US + mixed[i]);
        previous = DartControl.telemetry.servo_us[i];
        if (previous == 0U)
            previous = DART_SERVO_CENTER_US;

        /* 限制每个控制周期内舵机脉宽的最大变化量，保护舵机 */
        if (requested > (uint16_t)(previous + DART_SERVO_MAX_STEP_US))
            requested = (uint16_t)(previous + DART_SERVO_MAX_STEP_US);
        else if ((uint16_t)(requested + DART_SERVO_MAX_STEP_US) < previous)
            requested = (uint16_t)(previous - DART_SERVO_MAX_STEP_US);

        DartControl.telemetry.servo_us[i] = requested;
        /* 调试模式只算不驱动：遥测照常更新，PWM 保持中位。 */
        if (DART_MODE != DART_MODE_DEBUG)
            set_servo_compare(i, requested);
    }
}

/**
 * @brief 将所有舵机回中
 */
static void center_servos(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
        set_servo_compare(i, DART_SERVO_CENTER_US);
}

/**
 * @brief 复位视觉 EKF 滤波器
 * @param ekf EKF 结构体指针
 */
static void vision_ekf_reset(DartVisionAxisEkf_t *ekf)
{
    memset(ekf, 0, sizeof(*ekf));
}

#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_EKF
/**
 * @brief 初始化视觉 EKF 滤波器
 * @param ekf EKF 结构体指针
 * @param pixel 当前像素坐标
 * @param center 图像中心像素坐标
 * @param focal 焦距（像素）
 *
 * 根据针孔相机模型，由像素偏差计算初始角度。
 */
static void vision_ekf_init(DartVisionAxisEkf_t *ekf,
                            float pixel,
                            float center,
                            float focal)
{
    ekf->angle = atanf((pixel - center) / focal);
    ekf->rate = 0.0f;
    ekf->p00 = 0.02f;
    ekf->p01 = 0.0f;
    ekf->p10 = 0.0f;
    ekf->p11 = 0.5f;
    ekf->initialized = 1U;
}

/**
 * @brief 视觉 EKF 预测步
 * @param ekf EKF 结构体指针
 * @param dt 时间步长（秒）
 *
 * 状态量为 [角度, 角速度]，采用匀角速度模型，过程噪声为连续白噪声。
 */
static void vision_ekf_predict(DartVisionAxisEkf_t *ekf, float dt)
{
    float p00;
    float p01;
    float p10;
    float p11;
    float dt2;
    float dt3;
    float dt4;

    if (!ekf->initialized)
        return;

    dt2 = dt * dt;
    dt3 = dt2 * dt;
    dt4 = dt2 * dt2;

    /* 状态预测：角度 += 角速度 * dt */
    ekf->angle += ekf->rate * dt;

    /* 协方差预测 */
    p00 = ekf->p00 + dt * (ekf->p01 + ekf->p10) + dt2 * ekf->p11 +
          0.25f * DART_VISION_EKF_PROCESS_NOISE * dt4;
    p01 = ekf->p01 + dt * ekf->p11 +
          0.5f * DART_VISION_EKF_PROCESS_NOISE * dt3;
    p10 = ekf->p10 + dt * ekf->p11 +
          0.5f * DART_VISION_EKF_PROCESS_NOISE * dt3;
    p11 = ekf->p11 + DART_VISION_EKF_PROCESS_NOISE * dt2;

    ekf->p00 = p00;
    ekf->p01 = p01;
    ekf->p10 = p10;
    ekf->p11 = p11;
}

/* 任一轴与预测不一致时，整帧视觉数据都丢弃。 */
static uint8_t vision_ekf_measurement_in_gate(const DartVisionAxisEkf_t *ekf,
                                              float pixel,
                                              float center,
                                              float focal)
{
    float innovation;

    if (!ekf->initialized)
        return 1U;

    innovation = pixel - (center + focal * tanf(ekf->angle));
    return ((innovation >= -DART_VISION_EKF_GATE_PX) &&
            (innovation <= DART_VISION_EKF_GATE_PX)) ? 1U : 0U;
}

/**
 * @brief 视觉 EKF 更新步
 * @param ekf EKF 结构体指针
 * @param pixel 测量像素坐标
 * @param center 图像中心像素坐标
 * @param focal 焦距（像素）
 *
 * 量测模型为：pixel = center + focal * tan(angle)
 * 使用一阶泰勒展开得到雅可比矩阵。
 */
static void vision_ekf_update(DartVisionAxisEkf_t *ekf,
                              float pixel,
                              float center,
                              float focal)
{
    float tangent;
    float innovation;
    float measurement_jacobian;
    float innovation_covariance;
    float gain_angle;
    float gain_rate;
    float p00;
    float p01;
    float p10;
    float p11;

    if (!ekf->initialized)
    {
        vision_ekf_init(ekf, pixel, center, focal);
        return;
    }

    tangent = tanf(ekf->angle);
    innovation = pixel - (center + focal * tangent);

    measurement_jacobian = focal * (1.0f + tangent * tangent);
    innovation_covariance = measurement_jacobian * measurement_jacobian * ekf->p00 +
                            DART_VISION_EKF_MEAS_NOISE_PX;
    if (innovation_covariance < 0.000001f)
        return;

    /* 计算卡尔曼增益 */
    gain_angle = ekf->p00 * measurement_jacobian / innovation_covariance;
    gain_rate = ekf->p10 * measurement_jacobian / innovation_covariance;

    /* 状态更新 */
    ekf->angle += gain_angle * innovation;
    ekf->rate += gain_rate * innovation;

    /* 协方差更新 */
    p00 = ekf->p00;
    p01 = ekf->p01;
    p10 = ekf->p10;
    p11 = ekf->p11;
    ekf->p00 = (1.0f - gain_angle * measurement_jacobian) * p00;
    ekf->p01 = (1.0f - gain_angle * measurement_jacobian) * p01;
    ekf->p10 = p10 - gain_rate * measurement_jacobian * p00;
    ekf->p11 = p11 - gain_rate * measurement_jacobian * p01;
    /* 强制协方差对称 */
    ekf->p01 = 0.5f * (ekf->p01 + ekf->p10);
    ekf->p10 = ekf->p01;
}
#endif

#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_RAW_RATE
/* 保留最新视线角不做滤波，只对帧间角速率做平滑。 */
static void vision_raw_rate_update(DartVisionAxisEkf_t *state,
                                   float measured_angle,
                                   float inv_frame_dt,
                                   float rate_alpha,
                                   uint8_t rate_valid)
{
    float raw_rate;
    const float rate_limit =
        DART_VISION_RAW_RATE_LIMIT_DEG_S * DART_DEG_TO_RAD;

    if (rate_valid && state->initialized)
    {
        raw_rate = (measured_angle - state->angle) * inv_frame_dt;
        raw_rate = constrain_float(raw_rate, -rate_limit, rate_limit);
        state->rate += rate_alpha * (raw_rate - state->rate);
    }
    else
    {
        state->rate = 0.0f;
    }

    state->angle = measured_angle;
    state->initialized = 1U;
}
#endif

/**
 * @brief 更新视觉滤波器，包括预测、读取邮箱数据、更新、链路状态判断
 * @param dt 时间步长（秒）
 * @param now_ms 当前系统时间（毫秒）
 */
static void update_vision_filter(float dt, uint32_t now_ms)
{
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint32_t sample_tick;
    int16_t sample_x;
    int16_t sample_y;
    uint8_t sample_recognized;
    uint8_t measurement_valid;
#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_RAW_RATE
    uint8_t previous_recognized;
    uint8_t rate_valid;
    uint32_t frame_dt_ms;
    float frame_dt_s;
    float inv_frame_dt;
    float rate_alpha;
    float measured_x_angle;
    float measured_y_angle;
#endif
    /* 引导中心固定为 (0,0)：OpenMV 发送的是相对 ROI 中心的偏移。
       焦距仍用图像半宽计算，不能跟着引导中心变化。 */
    float center_x = 0.0f;
    float center_y = 0.0f;
    float focal_x = VisionFocalPxX;
    float focal_y = VisionFocalPxY;

#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_EKF
    /* 先进行 EKF 预测 */
    vision_ekf_predict(&DartControl.vision_x_ekf, dt);
    vision_ekf_predict(&DartControl.vision_y_ekf, dt);
#else
    /* RAW_RATE 模式只在新视觉帧到达时更新。 */
    (void)dt;
#endif

    /* 以无锁方式读取视觉邮箱，确保数据一致性 */
    do
    {
        sequence_before = DartControl.vision_mailbox.sequence;
        sample_x = DartControl.vision_mailbox.x;
        sample_y = DartControl.vision_mailbox.y;
        sample_recognized = DartControl.vision_mailbox.recognized;
        sample_tick = DartControl.vision_mailbox.tick_ms;
        sequence_after = DartControl.vision_mailbox.sequence;
    } while (sequence_before != sequence_after);

    /* 如果有新帧，则更新当前视觉状态。 */
    if (sequence_after != DartControl.processed_vision_sequence)
    {
        DartControl.processed_vision_sequence = sequence_after;
        DartControl.telemetry.vision.last_frame_ms = sample_tick;
        DartControl.telemetry.vision.frame_count = sequence_after;
#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_RAW_RATE
        previous_recognized = DartControl.telemetry.vision.recognized;
#endif
        DartControl.telemetry.vision.recognized = 0U;

        measurement_valid =
            (sample_recognized &&
             (fabsf((float)sample_x) <= (DART_IMAGE_WIDTH_PX * 0.5f)) &&
             (fabsf((float)sample_y) <= (DART_IMAGE_HEIGHT_PX * 0.5f))) ? 1U : 0U;

#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_EKF
        if (measurement_valid)
        {
            measurement_valid =
                (vision_ekf_measurement_in_gate(&DartControl.vision_x_ekf,
                                                (float)sample_x,
                                                center_x, focal_x) &&
                 vision_ekf_measurement_in_gate(&DartControl.vision_y_ekf,
                                                (float)sample_y,
                                                center_y, focal_y)) ? 1U : 0U;
        }
#endif

        if (measurement_valid)
        {
#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_EKF
            /* 两轴量测均有效，才更新 EKF 和目标有效时间戳。 */
            vision_ekf_update(&DartControl.vision_x_ekf,
                              (float)sample_x, center_x, focal_x);
            vision_ekf_update(&DartControl.vision_y_ekf,
                              (float)sample_y, center_y, focal_y);
#else
            /* 位置直接跟随当前帧，只对视线角速率做轻量低通，
               以保留前视追踪所需的速率分量。 */
            frame_dt_ms = sample_tick -
                          DartControl.telemetry.vision.last_target_ms;
            rate_valid =
                (previous_recognized &&
                 DartControl.vision_x_ekf.initialized &&
                 DartControl.vision_y_ekf.initialized &&
                 (frame_dt_ms >= DART_VISION_RATE_MIN_DT_MS) &&
                 (frame_dt_ms <= DART_VISION_RATE_MAX_DT_MS)) ? 1U : 0U;

            inv_frame_dt = 0.0f;
            rate_alpha = 1.0f;
            if (rate_valid)
            {
                frame_dt_s = (float)frame_dt_ms * 0.001f;
                inv_frame_dt = 1.0f / frame_dt_s;
                rate_alpha = frame_dt_s /
                             (DART_VISION_RATE_LPF_RC_S + frame_dt_s);
            }

            measured_x_angle = atanf(((float)sample_x - center_x) / focal_x);
            measured_y_angle = atanf(((float)sample_y - center_y) / focal_y);
            vision_raw_rate_update(&DartControl.vision_x_ekf,
                                   measured_x_angle,
                                   inv_frame_dt, rate_alpha, rate_valid);
            vision_raw_rate_update(&DartControl.vision_y_ekf,
                                   measured_y_angle,
                                   inv_frame_dt, rate_alpha, rate_valid);
#endif
            DartControl.telemetry.vision.recognized = 1U;
            DartControl.telemetry.vision.raw_x = sample_x;
            DartControl.telemetry.vision.raw_y = sample_y;
            DartControl.telemetry.vision.last_target_ms = sample_tick;
        }
        else
        {
            /* 未识别到有效目标，丢失帧计数增加 */
            DartControl.telemetry.vision.lost_frame_count++;
#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_RAW_RATE
            DartControl.vision_x_ekf.rate = 0.0f;
            DartControl.vision_y_ekf.rate = 0.0f;
#endif
        }
    }

    /* 判断视觉链路是否有效：有帧且未超时 */
    DartControl.telemetry.vision.link_valid =
        ((DartControl.telemetry.vision.frame_count != 0U) &&
         ((now_ms - DartControl.telemetry.vision.last_frame_ms) <=
          DART_VISION_LINK_TIMEOUT_MS)) ? 1U : 0U;

    if (DartControl.vision_x_ekf.initialized &&
        DartControl.vision_y_ekf.initialized)
    {
#if DART_VISION_FILTER_MODE == DART_VISION_FILTER_EKF
        /* 将 EKF 角度转换为滤波后的像素坐标和视线角 */
        DartControl.telemetry.vision.filtered_x =
            center_x + focal_x * tanf(DartControl.vision_x_ekf.angle);
        DartControl.telemetry.vision.filtered_y =
            center_y + focal_y * tanf(DartControl.vision_y_ekf.angle);
#else
        /* RAW_RATE 不做位置滤波，省去两次软件 tanf 调用。 */
        DartControl.telemetry.vision.filtered_x =
            (float)DartControl.telemetry.vision.raw_x;
        DartControl.telemetry.vision.filtered_y =
            (float)DartControl.telemetry.vision.raw_y;
#endif
        DartControl.telemetry.vision.los_yaw_deg =
            DART_CAMERA_YAW_SIGN * DartControl.vision_x_ekf.angle * DART_RAD_TO_DEG +
            DART_CAMERA_YAW_BIAS_DEG;
        DartControl.telemetry.vision.los_pitch_deg =
            DART_CAMERA_PITCH_SIGN * DartControl.vision_y_ekf.angle * DART_RAD_TO_DEG +
            DART_CAMERA_PITCH_BIAS_DEG;

        /* 目标保持有效：链路有效且目标未超时保持时间 */
        DartControl.telemetry.vision.valid =
            (DartControl.telemetry.vision.link_valid &&
             ((now_ms - DartControl.telemetry.vision.last_target_ms) <=
              DART_TARGET_LOST_HOLD_MS)) ? 1U : 0U;

        /* 目标丢失过久，复位 EKF，重新捕获 */
        if ((now_ms - DartControl.telemetry.vision.last_target_ms) >
            DART_TARGET_REACQUIRE_RESET_MS)
        {
            vision_ekf_reset(&DartControl.vision_x_ekf);
            vision_ekf_reset(&DartControl.vision_y_ekf);
            DartControl.telemetry.vision.valid = 0U;
        }
    }
    else
    {
        DartControl.telemetry.vision.valid = 0U;
    }

    /* 视觉链路错误标志：启动宽限期后仍无有效链路 */
    DartControl.vision_link_error =
        (((now_ms - DartControl.control_start_ms) > DART_VISION_STARTUP_GRACE_MS) &&
         !DartControl.telemetry.vision.link_valid) ? 1U : 0U;
    DartControl.telemetry.state = control_get_state();
}

/**
 * @brief 捕获起飞瞬间的姿态（偏航、横滚和俯仰），用于上升段稳定
 * @param ins 惯性导航数据
 *
 * 对多次采样的偏航角做圆均值，横滚角和俯仰角做算术平均。
 */
static void capture_launch_attitude(const INS_t *ins)
{
    float yaw_rad;

    if (DartControl.attitude_sample_count >= DART_LAUNCH_ATTITUDE_SAMPLE_COUNT)
        return;

    yaw_rad = ins->Yaw * DART_DEG_TO_RAD;
    DartControl.yaw_sin_sum += sinf(yaw_rad);
    DartControl.yaw_cos_sum += cosf(yaw_rad);
    DartControl.roll_sum += DART_INS_ROLL_DEG(ins);
    DartControl.pitch_sum += DART_INS_PITCH_DEG(ins);
    DartControl.attitude_sample_count++;

    if (DartControl.attitude_sample_count == DART_LAUNCH_ATTITUDE_SAMPLE_COUNT)
    {
        /* 圆均值计算平均偏航角 */
        DartControl.telemetry.launch_yaw_deg =
            atan2f(DartControl.yaw_sin_sum, DartControl.yaw_cos_sum) * DART_RAD_TO_DEG;
        DartControl.telemetry.launch_roll_deg =
            DartControl.roll_sum / (float)DART_LAUNCH_ATTITUDE_SAMPLE_COUNT;
        DartControl.telemetry.launch_pitch_deg =
            DartControl.pitch_sum / (float)DART_LAUNCH_ATTITUDE_SAMPLE_COUNT;
    }
}

static float calculate_yaw_hold(const INS_t *ins)
{
    float error = wrap_180(DartControl.telemetry.launch_yaw_deg - ins->Yaw);
    return calculate_axis_pid(&DartControl.yaw_pid, error,
                              DART_INS_YAW_RATE_RAD_S(ins));
}

/* 未启用 pitch 补偿时的俯仰自稳：保持起飞瞬间的俯仰角。
   与 yaw/roll 自稳使用同一套思路，只是各轴角度不跨 ±180 度归一化。 */
static float calculate_pitch_hold(const INS_t *ins)
{
    float error = DartControl.telemetry.launch_pitch_deg -
                  DART_INS_PITCH_DEG(ins);
    return calculate_axis_pid(&DartControl.pitch_pid, error,
                              DART_INS_PITCH_RATE_RAD_S(ins));
}

/* 横滚参考角：启用 roll 初始补偿时锁定补偿目标角，否则保持起飞横滚角。 */
static float roll_reference_deg(void)
{
    return DART_ASCENT_ROLL_COMPENSATION_ENABLE ? DART_ASCENT_ROLL_TARGET_DEG
                                                : DartControl.telemetry.launch_roll_deg;
}

static float calculate_roll_hold(const INS_t *ins)
{
    float error = wrap_180(roll_reference_deg() - DART_INS_ROLL_DEG(ins));
    return calculate_axis_pid(&DartControl.roll_pid, error,
                              DART_INS_ROLL_RATE_RAD_S(ins));
}

/* 前视追踪指令：视线角 + 前视时间 × 惯性系视线角速率。
   前视项单独限幅并设速率死区，避免静止目标下把图像噪声放大成指令抖动。 */
static float lead_pursuit_command(float los_deg, float los_rate_deg_s)
{
    float lead_deg = 0.0f;

    if (fabsf(los_rate_deg_s) > DART_LEAD_RATE_DEADBAND_DEG_S)
        lead_deg = DART_LEAD_TIME_S * los_rate_deg_s;

    lead_deg = constrain_float(lead_deg,
                               -DART_LEAD_MAX_DEG,
                               DART_LEAD_MAX_DEG);

    return constrain_float(los_deg + lead_deg,
                           -DART_LEAD_COMMAND_LIMIT_DEG,
                           DART_LEAD_COMMAND_LIMIT_DEG);
}

/* 上升段视觉 yaw 指令：视线角 + 前视修正，再按横滚投影到机体 yaw 轴。
   与下降段 yaw 引导使用同一套视觉量，但只取 yaw 分量，不参与 pitch。 */
static float vision_yaw_command(const INS_t *ins)
{
    float camera_yaw_rate;
    float camera_yaw_command;
    float roll_rad;

    camera_yaw_rate = DART_CAMERA_YAW_SIGN *
                      (DartControl.vision_x_ekf.rate +
                       DART_INS_YAW_RATE_RAD_S(ins)) * DART_RAD_TO_DEG;
    camera_yaw_rate = constrain_float(camera_yaw_rate,
                                      -DART_LOS_RATE_LIMIT_DEG_S,
                                      DART_LOS_RATE_LIMIT_DEG_S);

    camera_yaw_command = lead_pursuit_command(
        DartControl.telemetry.vision.los_yaw_deg, camera_yaw_rate);

    roll_rad = DART_INS_ROLL_DEG(ins) * DART_DEG_TO_RAD;
    camera_yaw_command = cosf(roll_rad) * camera_yaw_command;

    return constrain_float(camera_yaw_command,
                           -DART_LEAD_COMMAND_LIMIT_DEG,
                           DART_LEAD_COMMAND_LIMIT_DEG);
}

/* 上升段姿态控制：
   - yaw：识别到目标时由视觉 PID 参数控制 yaw，脱锁后由自稳 PID 参数保持起飞 yaw；
   - pitch：
     启用补偿时沿用现有 pitch 补偿思路，分两段：
       1) 发射后俯仰角尚未低于目标角：输出固定抬升量，抵消发射低头，不做 PID；
       2) 低头越过目标角后：PID 闭环把俯仰拉回目标角；
       俯仰重新到达目标角或进入下降段即停止补偿（ascent_pitch_done 置位），
       之后上升段不再输出 pitch，下降段只由视觉引导控制 pitch；
     未启用补偿时沿用 roll/yaw 自稳思路，保持起飞瞬间的俯仰角；
   - roll：
     启用补偿时先输出固定抬升量，越过补偿目标后由 PID 闭环并保持该目标角；
     未启用补偿时沿用现有自稳思路，保持起飞 roll。 */
static void run_ascent_stabilization(const INS_t *ins)
{
    float pitch_deg = DART_INS_PITCH_DEG(ins);
    float pitch_output = 0.0f;
    float output_pitch_deg = 0.0f;
    float yaw_command_deg;

    /* 上升段 yaw：识别到目标时只由视觉控制 yaw；脱锁后回到起飞 yaw 自稳。 */
    if ((DART_MODE != DART_MODE_PITCH) && DartControl.telemetry.vision.valid)
    {
        if (!DartControl.ascent_vision_yaw_active)
        {
            DartControl.ascent_vision_yaw_active = 1U;
            reset_pid(&DartControl.yaw_vision_pid);
        }

        yaw_command_deg = vision_yaw_command(ins);
        DartControl.telemetry.yaw_output_us =
            DART_ASCENT_VISION_YAW_OUTPUT_SIGN *
            calculate_axis_pid(&DartControl.yaw_vision_pid, yaw_command_deg,
                               DART_INS_YAW_RATE_RAD_S(ins));
        /* 等效目标 yaw = 当前 yaw + 视觉修正角。 */
        DartControl.telemetry.output_yaw_deg = ins->Yaw + yaw_command_deg;
    }
    else
    {
        if (DartControl.ascent_vision_yaw_active)
        {
            DartControl.ascent_vision_yaw_active = 0U;
            reset_pid(&DartControl.yaw_vision_pid);
        }

        DartControl.telemetry.yaw_output_us = calculate_yaw_hold(ins);
        DartControl.telemetry.output_yaw_deg = DartControl.telemetry.launch_yaw_deg;
    }

    /* ---------- pitch：启用补偿走补偿思路，未启用走自稳思路 ---------- */
    if (!DART_ASCENT_PITCH_COMPENSATION_ENABLE)
    {
        /* 未启用 pitch 补偿：保持起飞瞬间的俯仰角，与 roll/yaw 自稳一致。 */
        pitch_output = calculate_pitch_hold(ins);
        output_pitch_deg = DartControl.telemetry.launch_pitch_deg;
    }
    else if (!DartControl.ascent_pitch_done)
    {
        if (pitch_deg < DART_ASCENT_PITCH_TARGET_DEG)
            DartControl.ascent_pitch_below_target = 1U;
        else if (DartControl.ascent_pitch_below_target)
        {
            /* 从下方回到目标角：补偿完成，锁存到本次飞行结束。 */
            DartControl.ascent_pitch_done = 1U;
            reset_pid(&DartControl.pitch_pid);
        }

        if (!DartControl.ascent_pitch_done)
        {
            if (!DartControl.ascent_pitch_below_target)
            {
                /* 尚未低头：固定抬升输出，避免角度高于目标时 PID 反而压头。 */
                pitch_output = DART_ASCENT_PITCH_LIFT_US;
            }
            else
            {
                /* 低头后：PID 闭环控制到目标角。 */
                float pitch_error = DART_ASCENT_PITCH_TARGET_DEG - pitch_deg;
                pitch_output = calculate_axis_pid(&DartControl.pitch_pid,
                                                  pitch_error,
                                                  DART_INS_PITCH_RATE_RAD_S(ins));
            }
            output_pitch_deg = DART_ASCENT_PITCH_TARGET_DEG;
        }
    }
    DartControl.telemetry.pitch_output_us = pitch_output;

    /* ---------- roll：启用补偿则先固定抬升再闭环，未启用则自稳 ---------- */
    if (DART_ASCENT_ROLL_COMPENSATION_ENABLE && !DartControl.ascent_roll_done)
    {
        float roll_deg = DART_INS_ROLL_DEG(ins);

        if (roll_deg < DART_ASCENT_ROLL_TARGET_DEG)
            DartControl.ascent_roll_below_target = 1U;
        else if (DartControl.ascent_roll_below_target)
        {
            /* 从下方回到目标角：固定抬升阶段结束，交回 PID 闭环保持目标角。 */
            DartControl.ascent_roll_done = 1U;
            reset_pid(&DartControl.roll_pid);
        }

        if (!DartControl.ascent_roll_below_target)
            DartControl.telemetry.roll_output_us = DART_ASCENT_ROLL_LIFT_US;
        else
            DartControl.telemetry.roll_output_us = calculate_roll_hold(ins);
    }
    else
    {
        DartControl.telemetry.roll_output_us = calculate_roll_hold(ins);
    }

    /* ---------- 回传当前控制目标角（yaw 已在上方按视觉/自稳赋值） ---------- */
    DartControl.telemetry.output_pitch_deg = output_pitch_deg;
    DartControl.telemetry.output_roll_deg = roll_reference_deg();
    DartControl.telemetry.output_valid = 1U;

    /* ---------- 混控输出 ---------- */
    write_servo_outputs(DartControl.telemetry.pitch_output_us,
                        DartControl.telemetry.yaw_output_us,
                        DartControl.telemetry.roll_output_us);
}

/**
 * @brief 下降段引导：基于视觉前视追踪生成俯仰/偏航指令
 * @param ins 惯性导航数据
 *
 * 使用视线角（LOS）和惯性系视线角速率，按前视追踪（lead pursuit）计算期望姿态，
 * 再经当前横滚角进行坐标变换，最终由视觉 PID 参数转换为舵机输出。
 */
static void run_descent_guidance(const INS_t *ins)
{
    float camera_yaw_rate;
    float camera_pitch_rate;
    float camera_yaw_command;
    float camera_pitch_command;
    float roll_rad;
    float cos_roll;
    float sin_roll;
    float yaw_command;
    float pitch_command;
#if DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE
    uint32_t target_age_ms;
#endif

    /* 下降段 roll 保持补偿目标（启用补偿）或起飞横滚角（未启用补偿）。 */
    DartControl.telemetry.roll_output_us = calculate_roll_hold(ins);
    DartControl.telemetry.output_roll_deg = roll_reference_deg();
    DartControl.telemetry.output_valid = 1U;

#if DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE
    if (!DartControl.telemetry.vision.valid)
#else
    /* 关闭锁存时，收到明确的无目标帧就立即退出视觉引导；
       原有超时逻辑仍可兜底串口帧丢失。 */
    if (!DartControl.telemetry.vision.valid ||
        !DartControl.telemetry.vision.recognized)
#endif
    {
#if DART_DESCENT_LOCK_LAST_VISUAL_COMMAND_ENABLE
        target_age_ms = HAL_GetTick() -
                        DartControl.telemetry.vision.last_target_ms;
        if (DartControl.descent_target_seen &&
            (target_age_ms <= DART_TARGET_REACQUIRE_RESET_MS))
        {
            /* 可选锁存：保持最后一次相对视觉指令，
               但不超过视觉状态重捕获超时。 */
            DartControl.telemetry.yaw_output_us = 0.0f;
            if (DART_MODE != DART_MODE_PITCH)
                DartControl.telemetry.yaw_output_us =
                    DART_DESCENT_YAW_OUTPUT_SIGN *
                    calculate_axis_pid(&DartControl.yaw_vision_pid,
                                       DartControl.telemetry.target_yaw_deg,
                                       DART_INS_YAW_RATE_RAD_S(ins));
            DartControl.telemetry.pitch_output_us = 0.0f;
            if (DART_MODE != DART_MODE_YAW)
                DartControl.telemetry.pitch_output_us =
                    DART_DESCENT_PITCH_OUTPUT_SIGN *
                    calculate_axis_pid(&DartControl.pitch_vision_pid,
                                       DartControl.telemetry.target_pitch_deg,
                                       DART_INS_PITCH_RATE_RAD_S(ins));
            DartControl.telemetry.output_yaw_deg =
                DartControl.telemetry.target_yaw_deg;
            DartControl.telemetry.output_pitch_deg =
                DART_DESCENT_PITCH_OUTPUT_SIGN *
                DartControl.telemetry.target_pitch_deg;
            write_servo_outputs(DartControl.telemetry.pitch_output_us,
                                DartControl.telemetry.yaw_output_us,
                                DartControl.telemetry.roll_output_us);
            return;
        }
#endif

        if (DartControl.descent_vision_active)
        {
            DartControl.descent_vision_active = 0U;
            reset_pid(&DartControl.yaw_vision_pid);
            reset_pid(&DartControl.pitch_vision_pid);
        }

        if (DartControl.descent_target_seen)
            reset_pid(&DartControl.yaw_pid);
        DartControl.descent_target_seen = 0U;

        /* 锁存关闭或已超时：丢弃旧视觉指令，改为保持起飞偏航、俯仰回中。 */
        DartControl.telemetry.target_yaw_deg = 0.0f;
        DartControl.telemetry.target_pitch_deg = 0.0f;
        DartControl.telemetry.yaw_output_us = calculate_yaw_hold(ins);
        DartControl.telemetry.output_yaw_deg =
            DartControl.telemetry.launch_yaw_deg;
        DartControl.telemetry.pitch_output_us = 0.0f;
        DartControl.telemetry.output_pitch_deg = 0.0f;
        write_servo_outputs(0.0f,
                            DartControl.telemetry.yaw_output_us,
                            DartControl.telemetry.roll_output_us);
        return;
    }

    if (!DartControl.descent_vision_active)
    {
        reset_pid(&DartControl.yaw_vision_pid);
        reset_pid(&DartControl.pitch_vision_pid);
        DartControl.descent_vision_active = 1U;
    }
    DartControl.descent_target_seen = 1U;

    /* 计算相机坐标系下的视线角速率（补偿机体角速率） */
    camera_yaw_rate = DART_CAMERA_YAW_SIGN *
                      (DartControl.vision_x_ekf.rate +
                       DART_INS_YAW_RATE_RAD_S(ins)) * DART_RAD_TO_DEG;
    /* 像素速率取相机 Y 轴；俯仰约定取反后，机体俯仰角速率与之符号相反。 */
    camera_pitch_rate = DART_CAMERA_PITCH_SIGN *
                        (DartControl.vision_y_ekf.rate -
                         DART_INS_PITCH_RATE_RAD_S(ins)) * DART_RAD_TO_DEG;
    camera_yaw_rate = constrain_float(camera_yaw_rate,
                                      -DART_LOS_RATE_LIMIT_DEG_S,
                                      DART_LOS_RATE_LIMIT_DEG_S);
    camera_pitch_rate = constrain_float(camera_pitch_rate,
                                        -DART_LOS_RATE_LIMIT_DEG_S,
                                        DART_LOS_RATE_LIMIT_DEG_S);

    /* 前视追踪：指令 = 视线角 + 前视时间 × 视线角速率（前视项已在函数内限幅） */
    camera_yaw_command = lead_pursuit_command(
        DartControl.telemetry.vision.los_yaw_deg, camera_yaw_rate);
    camera_pitch_command = lead_pursuit_command(
        DartControl.telemetry.vision.los_pitch_deg, camera_pitch_rate);

    /* 根据当前横滚角将相机平面指令旋转到机体俯仰/偏航 */
    roll_rad = DART_INS_ROLL_DEG(ins) * DART_DEG_TO_RAD;
    cos_roll = cosf(roll_rad);
    sin_roll = sinf(roll_rad);
    yaw_command = cos_roll * camera_yaw_command + sin_roll * camera_pitch_command;
    pitch_command = -sin_roll * camera_yaw_command + cos_roll * camera_pitch_command;
    yaw_command = constrain_float(yaw_command,
                                  -DART_LEAD_COMMAND_LIMIT_DEG,
                                  DART_LEAD_COMMAND_LIMIT_DEG);
    pitch_command = constrain_float(pitch_command,
                                    -DART_LEAD_COMMAND_LIMIT_DEG,
                                    DART_LEAD_COMMAND_LIMIT_DEG);

    /* 下位机模式：pitch/yaw 单轴时另一轴复位并输出 0；both/default 两轴 */
    if (DART_MODE == DART_MODE_PITCH)
    {
        yaw_command = 0.0f;
        reset_pid(&DartControl.yaw_vision_pid);
    }
    else if (DART_MODE == DART_MODE_YAW)
    {
        pitch_command = 0.0f;
        reset_pid(&DartControl.pitch_vision_pid);
    }

    /* 记录下发给 OpenMV 的目标角（机体坐标，PID 前的指令） */
    DartControl.telemetry.target_yaw_deg = yaw_command;
    DartControl.telemetry.target_pitch_deg = pitch_command;
    DartControl.telemetry.output_yaw_deg = yaw_command;
    /* 遥测里的 OP 是经过下降段方向修正后的俯仰指令。 */
    DartControl.telemetry.output_pitch_deg =
        DART_DESCENT_PITCH_OUTPUT_SIGN * pitch_command;

    /* 视觉控制 yaw/pitch（视觉 PID 参数）；roll 继续由补偿/自稳保持。 */
    DartControl.telemetry.yaw_output_us = 0.0f;
    if (DART_MODE != DART_MODE_PITCH)
        DartControl.telemetry.yaw_output_us =
            DART_DESCENT_YAW_OUTPUT_SIGN *
            calculate_axis_pid(&DartControl.yaw_vision_pid, yaw_command,
                               DART_INS_YAW_RATE_RAD_S(ins));

    DartControl.telemetry.pitch_output_us = 0.0f;
    if (DART_MODE != DART_MODE_YAW)
        DartControl.telemetry.pitch_output_us =
            DART_DESCENT_PITCH_OUTPUT_SIGN *
            calculate_axis_pid(&DartControl.pitch_vision_pid, pitch_command,
                               DART_INS_PITCH_RATE_RAD_S(ins));
    write_servo_outputs(DartControl.telemetry.pitch_output_us,
                        DartControl.telemetry.yaw_output_us,
                        DartControl.telemetry.roll_output_us);
}

/* 调试模式引导：与下降段同一套前视追踪，只计算指令，不驱动舵机。 */
static void run_debug_guidance(const INS_t *ins)
{
    float camera_yaw_rate = 0.0f;
    float camera_pitch_rate = 0.0f;
    float camera_yaw_command = 0.0f;
    float camera_pitch_command = 0.0f;
    float roll_rad;
    float cos_roll;
    float sin_roll;
    float yaw_command;
    float pitch_command;

    if (DartControl.telemetry.vision.valid)
    {
        camera_yaw_rate = DART_CAMERA_YAW_SIGN *
                          (DartControl.vision_x_ekf.rate +
                           DART_INS_YAW_RATE_RAD_S(ins)) * DART_RAD_TO_DEG;
        camera_pitch_rate = DART_CAMERA_PITCH_SIGN *
                            (DartControl.vision_y_ekf.rate -
                             DART_INS_PITCH_RATE_RAD_S(ins)) * DART_RAD_TO_DEG;
        camera_yaw_rate = constrain_float(camera_yaw_rate,
                                          -DART_LOS_RATE_LIMIT_DEG_S,
                                          DART_LOS_RATE_LIMIT_DEG_S);
        camera_pitch_rate = constrain_float(camera_pitch_rate,
                                            -DART_LOS_RATE_LIMIT_DEG_S,
                                            DART_LOS_RATE_LIMIT_DEG_S);

        camera_yaw_command = lead_pursuit_command(
            DartControl.telemetry.vision.los_yaw_deg, camera_yaw_rate);
        camera_pitch_command = lead_pursuit_command(
            DartControl.telemetry.vision.los_pitch_deg, camera_pitch_rate);
    }

    roll_rad = DART_INS_ROLL_DEG(ins) * DART_DEG_TO_RAD;
    cos_roll = cosf(roll_rad);
    sin_roll = sinf(roll_rad);
    yaw_command = cos_roll * camera_yaw_command + sin_roll * camera_pitch_command;
    pitch_command = -sin_roll * camera_yaw_command + cos_roll * camera_pitch_command;
    yaw_command = constrain_float(yaw_command,
                                  -DART_LEAD_COMMAND_LIMIT_DEG,
                                  DART_LEAD_COMMAND_LIMIT_DEG);
    pitch_command = constrain_float(pitch_command,
                                    -DART_LEAD_COMMAND_LIMIT_DEG,
                                    DART_LEAD_COMMAND_LIMIT_DEG);

    DartControl.telemetry.target_yaw_deg = yaw_command;
    DartControl.telemetry.target_pitch_deg = pitch_command;
    DartControl.telemetry.output_yaw_deg = yaw_command;
    DartControl.telemetry.output_pitch_deg =
        DART_DESCENT_PITCH_OUTPUT_SIGN * pitch_command;
    DartControl.telemetry.output_roll_deg = DART_INS_ROLL_DEG(ins);
    DartControl.telemetry.output_valid = 1U;
    DartControl.telemetry.roll_output_us = 0.0f;

    if (DartControl.telemetry.vision.valid)
    {
        DartControl.telemetry.yaw_output_us =
            DART_DESCENT_YAW_OUTPUT_SIGN *
            calculate_axis_pid(&DartControl.yaw_vision_pid, yaw_command,
                               DART_INS_YAW_RATE_RAD_S(ins));
        DartControl.telemetry.pitch_output_us =
            DART_DESCENT_PITCH_OUTPUT_SIGN *
            calculate_axis_pid(&DartControl.pitch_vision_pid, pitch_command,
                               DART_INS_PITCH_RATE_RAD_S(ins));
    }
    else
    {
        DartControl.telemetry.yaw_output_us = 0.0f;
        DartControl.telemetry.pitch_output_us = 0.0f;
        reset_pid(&DartControl.yaw_vision_pid);
        reset_pid(&DartControl.pitch_vision_pid);
    }

    write_servo_outputs(DartControl.telemetry.pitch_output_us,
                        DartControl.telemetry.yaw_output_us,
                        DartControl.telemetry.roll_output_us);
}

/**
 * @brief 计算 CRC8 校验值
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC8 结果
 */
static uint8_t vision_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = DART_VISION_CRC8_INIT;
    uint8_t byte_index;
    uint8_t bit_index;

    for (byte_index = 0U; byte_index < length; byte_index++)
    {
        crc ^= data[byte_index];
        for (bit_index = 0U; bit_index < 8U; bit_index++)
        {
            if (crc & 0x01U)
                crc = (uint8_t)((crc >> 1U) ^ DART_VISION_CRC8_POLY);
            else
                crc >>= 1U;
        }
    }
    return crc;
}

/**
 * @brief 向 OpenMV 发送姿态遥测帧（pitch/yaw/roll）
 *
 * 帧格式：55 pitchH pitchL yawH yawL rollH rollL flags FE CRC8
 * 角度单位 0.01 度，大端；CRC8 覆盖字节 0..8。
 * 只在收到过有效视觉帧且 IMU 就绪后发送，由低优先级 LED 任务以 50 Hz 调用。
 */
void attitude_telemetry_send(void)
{
    const INS_t *ins;
    float pitch_deg;
    float yaw_deg;
    float roll_deg;
    float gyro_x_deg;
    float gyro_y_deg;
    float gyro_z_deg;
    float output_pitch_deg;
    float output_yaw_deg;
    float output_roll_deg;
    int16_t pitch_raw;
    int16_t yaw_raw;
    int16_t roll_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t output_pitch_raw;
    int16_t output_yaw_raw;
    int16_t output_roll_raw;
    int16_t los_yaw_raw;
    int16_t los_pitch_raw;
    int16_t servo_raw[4];
    float los_yaw_deg_out;
    float los_pitch_deg_out;
    uint8_t dbg_flags;
    uint8_t i;
    uint8_t output_valid;
    uint32_t primask;
    static uint8_t tx_frame[DART_ATT_FRAME_LEN];

    /* 握手成功后开始发送姿态遥测；IMU 未就绪时不发。 */
    if (!DartControl.attitude_tx_armed)
        return;
    if (!DartControl.imu_ready)
        return;
    /* HAL_UART_Transmit_IT 在发送完成前一直使用这块静态缓冲。 */
    if (huart1.gState != HAL_UART_STATE_READY)
        return;

    ins = get_gyro_data_Point();
    primask = __get_PRIMASK();
    __disable_irq();
    pitch_deg = DART_INS_PITCH_DEG(ins);
    yaw_deg = ins->Yaw;
    roll_deg = DART_INS_ROLL_DEG(ins);
    gyro_x_deg = ins->Gyro[0] * DART_RAD_TO_DEG;
    gyro_y_deg = ins->Gyro[1] * DART_RAD_TO_DEG;
    gyro_z_deg = ins->Gyro[2] * DART_RAD_TO_DEG;
    output_pitch_deg = DartControl.telemetry.output_pitch_deg;
    output_yaw_deg = DartControl.telemetry.output_yaw_deg;
    output_roll_deg = DartControl.telemetry.output_roll_deg;
    output_valid = DartControl.telemetry.output_valid;
    if (primask == 0U)
        __enable_irq();

    /* LOS 增量：相对上电 1 s 参考角；舵机角按 1550 us = 0 度换算。 */
    if (debug_ref_valid)
    {
        los_yaw_deg_out = wrap_180(yaw_deg - debug_ref_yaw_deg) +
                          DartControl.telemetry.vision.los_yaw_deg;
        los_pitch_deg_out = (pitch_deg - debug_ref_pitch_deg) +
                            DartControl.telemetry.vision.los_pitch_deg;
        dbg_flags = DART_DBG_FLAG_REF_VALID;
    }
    else
    {
        los_yaw_deg_out = 0.0f;
        los_pitch_deg_out = 0.0f;
        dbg_flags = 0U;
    }

    pitch_raw = (int16_t)(pitch_deg * DART_ATT_ANGLE_SCALE);
    yaw_raw = (int16_t)(yaw_deg * DART_ATT_ANGLE_SCALE);
    roll_raw = (int16_t)(roll_deg * DART_ATT_ANGLE_SCALE);
    gyro_x_raw = (int16_t)(gyro_x_deg * DART_ATT_GYRO_SCALE);
    gyro_y_raw = (int16_t)(gyro_y_deg * DART_ATT_GYRO_SCALE);
    gyro_z_raw = (int16_t)(gyro_z_deg * DART_ATT_GYRO_SCALE);
    output_pitch_raw = (int16_t)(output_pitch_deg * DART_ATT_ANGLE_SCALE);
    output_yaw_raw = (int16_t)(output_yaw_deg * DART_ATT_ANGLE_SCALE);
    output_roll_raw = (int16_t)(output_roll_deg * DART_ATT_ANGLE_SCALE);
    los_yaw_raw = (int16_t)(los_yaw_deg_out * DART_ATT_LOS_SCALE);
    los_pitch_raw = (int16_t)(los_pitch_deg_out * DART_ATT_LOS_SCALE);
    for (i = 0U; i < 4U; i++)
        servo_raw[i] = (int16_t)((DartControl.telemetry.servo_us[i] -
                                  (float)DART_SERVO_CENTER_US) *
                                 DART_ATT_SERVO_DEG_PER_US *
                                 DART_ATT_SERVO_SCALE);

    tx_frame[DART_ATT_SOF_INDEX] = DART_ATT_SOF;
    tx_frame[DART_ATT_GYRO_X_H_INDEX] = (uint8_t)((uint16_t)gyro_x_raw >> 8U);
    tx_frame[DART_ATT_GYRO_X_L_INDEX] = (uint8_t)gyro_x_raw;
    tx_frame[DART_ATT_GYRO_Y_H_INDEX] = (uint8_t)((uint16_t)gyro_y_raw >> 8U);
    tx_frame[DART_ATT_GYRO_Y_L_INDEX] = (uint8_t)gyro_y_raw;
    tx_frame[DART_ATT_GYRO_Z_H_INDEX] = (uint8_t)((uint16_t)gyro_z_raw >> 8U);
    tx_frame[DART_ATT_GYRO_Z_L_INDEX] = (uint8_t)gyro_z_raw;
    tx_frame[DART_ATT_PITCH_H_INDEX] = (uint8_t)((uint16_t)pitch_raw >> 8U);
    tx_frame[DART_ATT_PITCH_L_INDEX] = (uint8_t)pitch_raw;
    tx_frame[DART_ATT_YAW_H_INDEX] = (uint8_t)((uint16_t)yaw_raw >> 8U);
    tx_frame[DART_ATT_YAW_L_INDEX] = (uint8_t)yaw_raw;
    tx_frame[DART_ATT_ROLL_H_INDEX] = (uint8_t)((uint16_t)roll_raw >> 8U);
    tx_frame[DART_ATT_ROLL_L_INDEX] = (uint8_t)roll_raw;
    tx_frame[DART_ATT_OUTPUT_PITCH_H_INDEX] =
        (uint8_t)((uint16_t)output_pitch_raw >> 8U);
    tx_frame[DART_ATT_OUTPUT_PITCH_L_INDEX] = (uint8_t)output_pitch_raw;
    tx_frame[DART_ATT_OUTPUT_YAW_H_INDEX] =
        (uint8_t)((uint16_t)output_yaw_raw >> 8U);
    tx_frame[DART_ATT_OUTPUT_YAW_L_INDEX] = (uint8_t)output_yaw_raw;
    tx_frame[DART_ATT_OUTPUT_ROLL_H_INDEX] =
        (uint8_t)((uint16_t)output_roll_raw >> 8U);
    tx_frame[DART_ATT_OUTPUT_ROLL_L_INDEX] = (uint8_t)output_roll_raw;
    tx_frame[DART_ATT_MODE_INDEX] = (uint8_t)DART_MODE;
    tx_frame[DART_ATT_STATE_INDEX] = (uint8_t)control_get_state();
    tx_frame[DART_ATT_FLAGS_INDEX] = (uint8_t)(
        (DartControl.telemetry.vision.link_valid ? DART_ATT_FLAG_LINK_VALID : 0U) |
        (DartControl.telemetry.vision.valid ? DART_ATT_FLAG_TARGET_VALID : 0U) |
        (DartControl.imu_ready ? DART_ATT_FLAG_IMU_READY : 0U) |
        (DartControl.attitude_tx_armed ? DART_ATT_FLAG_HANDSHAKE_DONE : 0U) |
        (output_valid ? DART_ATT_FLAG_OUTPUT_VALID : 0U));
    tx_frame[DART_ATT_LOS_YAW_H_INDEX] = (uint8_t)((uint16_t)los_yaw_raw >> 8U);
    tx_frame[DART_ATT_LOS_YAW_L_INDEX] = (uint8_t)los_yaw_raw;
    tx_frame[DART_ATT_LOS_PITCH_H_INDEX] = (uint8_t)((uint16_t)los_pitch_raw >> 8U);
    tx_frame[DART_ATT_LOS_PITCH_L_INDEX] = (uint8_t)los_pitch_raw;
    tx_frame[DART_ATT_SERVO1_H_INDEX] = (uint8_t)((uint16_t)servo_raw[0] >> 8U);
    tx_frame[DART_ATT_SERVO1_L_INDEX] = (uint8_t)servo_raw[0];
    tx_frame[DART_ATT_SERVO2_H_INDEX] = (uint8_t)((uint16_t)servo_raw[1] >> 8U);
    tx_frame[DART_ATT_SERVO2_L_INDEX] = (uint8_t)servo_raw[1];
    tx_frame[DART_ATT_SERVO3_H_INDEX] = (uint8_t)((uint16_t)servo_raw[2] >> 8U);
    tx_frame[DART_ATT_SERVO3_L_INDEX] = (uint8_t)servo_raw[2];
    tx_frame[DART_ATT_SERVO4_H_INDEX] = (uint8_t)((uint16_t)servo_raw[3] >> 8U);
    tx_frame[DART_ATT_SERVO4_L_INDEX] = (uint8_t)servo_raw[3];
    tx_frame[DART_ATT_DBG_FLAGS_INDEX] = dbg_flags;
    tx_frame[DART_ATT_EOF_INDEX] = DART_ATT_EOF;
    tx_frame[DART_ATT_CRC_INDEX] =
        vision_crc8(tx_frame, DART_ATT_CRC_INDEX);

    /* 37 字节在低优先级任务里阻塞发送会被高优先级任务抢占并超时，
       改由 USART1 中断连续发送；gState 忙时跳过本帧。 */
    (void)HAL_UART_Transmit_IT(&huart1, tx_frame, DART_ATT_FRAME_LEN);
}

/**
 * @brief 周期发送握手帧，直到收到 OpenMV 的 ACK
 *
 * 帧格式：A5 MODE FLAGS FE CRC8（5 字节）。
 * FLAGS bit0 = IMU 就绪。
 */
void openmv_handshake_send(void)
{
    static uint32_t last_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    uint8_t tx_frame[DART_HANDSHAKE_FRAME_LEN];

    if (DartControl.attitude_tx_armed)
        return;
    if ((now_ms - last_ms) < DART_HANDSHAKE_TX_PERIOD_MS)
        return;
    last_ms = now_ms;

    tx_frame[0] = DART_HANDSHAKE_SOF;
    tx_frame[1] = (uint8_t)DART_MODE;
    tx_frame[2] = (uint8_t)(DartControl.imu_ready ? 0x01U : 0U);
    tx_frame[3] = DART_HANDSHAKE_EOF;
    tx_frame[4] = vision_crc8(tx_frame, 4U);
    (void)HAL_UART_Transmit(&huart1, tx_frame,
                            DART_HANDSHAKE_FRAME_LEN,
                            DART_ATT_TX_TIMEOUT_MS);
}

/**
 * @brief 将解析后的视觉帧发布到邮箱，供控制任务读取
 */
static void publish_vision_frame(void)
{
    /* x/y 为相对引导中心的有符号像素偏移（int16，大端） */
    DartControl.vision_mailbox.x = (int16_t)(
        ((uint16_t)DartControl.uart_frame[DART_VISION_X_H_INDEX] << 8U) |
        DartControl.uart_frame[DART_VISION_X_L_INDEX]);
    DartControl.vision_mailbox.y = (int16_t)(
        ((uint16_t)DartControl.uart_frame[DART_VISION_Y_H_INDEX] << 8U) |
        DartControl.uart_frame[DART_VISION_Y_L_INDEX]);
    DartControl.vision_mailbox.recognized =
        DartControl.uart_frame[DART_VISION_RECOGNIZED_INDEX] ? 1U : 0U;
    DartControl.vision_mailbox.tick_ms = HAL_GetTick();
    DartControl.vision_mailbox.sequence++;   /* 序号递增，用于无锁读取 */
}

/**
 * @brief 初始化视觉串口接收（USART1），启动中断接收
 */
static void vision_uart_init(void)
{
    DartControl.uart_frame_index = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    (void)HAL_UART_Receive_IT(&huart1, &DartControl.uart_rx_byte, 1U);
}

/**
 * @brief 控制模块初始化
 *
 * 清零运行时结构体（保留 IMU 就绪标志），初始化 PID，设置初始状态，
 * 初始化视觉串口，标记初始化完成。
 */
void control_Init(void)
{
    uint8_t imu_ready;
    uint8_t i;
    uint32_t primask;

    /* 关中断保护，避免清零时被中断修改 */
    primask = __get_PRIMASK();
    __disable_irq();
    imu_ready = DartControl.imu_ready;
    memset(&DartControl, 0, sizeof(DartControl));
    DartControl.imu_ready = imu_ready;
    if (primask == 0U)
        __enable_irq();

    VisionFocalPxX = DART_CAMERA_FOCAL_X_PX;
    VisionFocalPxY = DART_CAMERA_FOCAL_Y_PX;

    /* 舵机遥测初始化为中位 */
    for (i = 0U; i < 4U; i++)
        DartControl.telemetry.servo_us[i] = DART_SERVO_CENTER_US;

    /* 初始化各轴 PID：yaw/pitch 分别使用自稳与视觉两套参数 */
    init_axis_pid(&DartControl.yaw_pid,
                  DART_YAW_STAB_KP, DART_YAW_STAB_KI, DART_YAW_STAB_KD);
    init_axis_pid(&DartControl.yaw_vision_pid,
                  DART_YAW_VISION_KP, DART_YAW_VISION_KI, DART_YAW_VISION_KD);
    init_axis_pid(&DartControl.pitch_pid,
                  DART_PITCH_STAB_KP, DART_PITCH_STAB_KI, DART_PITCH_STAB_KD);
    init_axis_pid(&DartControl.pitch_vision_pid,
                  DART_PITCH_VISION_KP, DART_PITCH_VISION_KI, DART_PITCH_VISION_KD);
    init_axis_pid(&DartControl.roll_pid,
                  DART_ROLL_KP, DART_ROLL_KI, DART_ROLL_KD);

    set_state(DART_STATE_READY);
    DartControl.last_control_ms = HAL_GetTick();
    DartControl.control_start_ms = DartControl.last_control_ms;
    vision_uart_init();
    DartControl.initialized = 1U;
}

/**
 * @brief 舵机初始化：启动 PWM 输出并回中
 */
void servo_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
        status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
        status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
        status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
        status = HAL_ERROR;
    center_servos();
    DartControl.servo_ready = (status == HAL_OK) ? 1U : 0U;
}

/**
 * @brief 控制主任务，按固定周期调用
 *
 * 实现飞行状态机：
 * READY -> IMU_READY -> SERVO_READY -> LAUNCHED -> ASCENT -> DESCENT
 * 各状态执行相应的初始化、检测和控制逻辑。
 */
void control_Task(void)
{
    const INS_t *shared_ins;
    INS_t ins_snapshot;
    uint32_t primask;
    uint32_t now_ms;
    uint32_t elapsed_ms;
    float dt;
    float launch_accel;

    if (!DartControl.initialized)
        return;

    /* 计算控制周期，限制最大 dt */
    now_ms = HAL_GetTick();
    elapsed_ms = now_ms - DartControl.last_control_ms;
    if (elapsed_ms == 0U)
        elapsed_ms = DART_CONTROL_PERIOD_MS;
    if (elapsed_ms > DART_MAX_CONTROL_DT_MS)
        elapsed_ms = DART_MAX_CONTROL_DT_MS;
    dt = (float)elapsed_ms * 0.001f;
    DartControl.last_control_ms = now_ms;

    /* 更新视觉滤波器（预测 + 更新） */
    update_vision_filter(dt, now_ms);

    if (!DartControl.imu_ready)
        return;

    /* 若舵机未就绪，等待一段时间后尝试初始化 */
    if (!DartControl.servo_ready)
    {
        if (DartControl.flight_state == DART_STATE_READY)
            set_state(DART_STATE_IMU_READY);
        if ((now_ms - DartControl.state_enter_ms) >= DART_IMU_LED_HOLD_MS)
        {
            servo_Init();
            if (DartControl.servo_ready)
                set_state(DART_STATE_SERVO_READY);
            else
                DartControl.state_enter_ms = now_ms;
        }
        return;
    }

    /* 获取 IMU 数据快照（关中断保护） */
    shared_ins = get_gyro_data_Point();
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(&ins_snapshot, shared_ins, sizeof(ins_snapshot));
    if (primask == 0U)
        __enable_irq();

    /* 上电静止参考角：IMU 就绪 1 s 后采样一次，调试模式的 LOS 增量以此为零点。 */
    if (!debug_ref_valid &&
        ((now_ms - debug_imu_ready_ms) >= DART_DEBUG_REF_DELAY_MS))
    {
        debug_ref_yaw_deg = ins_snapshot.Yaw;
        debug_ref_pitch_deg = DART_INS_PITCH_DEG(&ins_snapshot);
        debug_ref_valid = 1U;
    }

    /* 调试模式：不做起飞检测、不驱动舵机，只运行视觉引导并上报。 */
    if (DART_MODE == DART_MODE_DEBUG)
    {
        if (DartControl.flight_state != DART_STATE_DEBUG)
        {
            reset_all_control_pids();
            set_state(DART_STATE_DEBUG);
        }
        run_debug_guidance(&ins_snapshot);
        return;
    }

    /* 状态：SERVO_READY —— 检测起飞加速度，确认起飞 */
    if (DartControl.flight_state == DART_STATE_SERVO_READY)
    {
        launch_accel = DART_LAUNCH_X_SIGN * ins_snapshot.MotionAccel_b[0];
        if (launch_accel >= DART_LAUNCH_ACCEL_THRESHOLD)
        {
            if (DartControl.launch_confirm_count < DART_LAUNCH_CONFIRM_COUNT)
                DartControl.launch_confirm_count++;
        }
        else
        {
            DartControl.launch_confirm_count = 0U;
        }

        if (DartControl.launch_confirm_count >= DART_LAUNCH_CONFIRM_COUNT)
        {
            /* 先播种偏航/横滚参考角，保证本周期三轴都参与控制。 */
            reset_all_control_pids();
            DartControl.attitude_sample_count = 0U;
            DartControl.yaw_sin_sum = 0.0f;
            DartControl.yaw_cos_sum = 0.0f;
            DartControl.roll_sum = 0.0f;
            DartControl.pitch_sum = 0.0f;
            DartControl.ascent_pitch_done = 0U;
            DartControl.ascent_pitch_below_target = 0U;
            DartControl.ascent_roll_done = 0U;
            DartControl.ascent_roll_below_target = 0U;
            DartControl.ascent_vision_yaw_active = 0U;
            DartControl.descent_target_seen = 0U;
            DartControl.descent_vision_active = 0U;
            DartControl.telemetry.launch_yaw_deg = ins_snapshot.Yaw;
            DartControl.telemetry.launch_pitch_deg =
                DART_INS_PITCH_DEG(&ins_snapshot);
            DartControl.telemetry.launch_roll_deg =
                DART_INS_ROLL_DEG(&ins_snapshot);
            set_state(DART_STATE_LAUNCHED);
            run_ascent_stabilization(&ins_snapshot);
        }
        return;
    }

    /* 状态：LAUNCHED —— 采集起飞姿态，短暂稳定后进入 ASCENT */
    if (DartControl.flight_state == DART_STATE_LAUNCHED)
    {
        capture_launch_attitude(&ins_snapshot);
        run_ascent_stabilization(&ins_snapshot);
        if ((DartControl.attitude_sample_count >= DART_LAUNCH_ATTITUDE_SAMPLE_COUNT) &&
            ((now_ms - DartControl.state_enter_ms) >= DART_LAUNCH_LED_HOLD_MS))
        {
            set_state(DART_STATE_ASCENT);
        }
        return;
    }

    /* 状态：ASCENT —— 上升段稳定，检测俯仰角判断是否进入下降 */
    if (DartControl.flight_state == DART_STATE_ASCENT)
    {
        run_ascent_stabilization(&ins_snapshot);
        if (((now_ms - DartControl.state_enter_ms) >= DART_MIN_ASCENT_TIME_MS) &&
            (DART_INS_PITCH_DEG(&ins_snapshot) < DART_DESCENT_PITCH_THRESHOLD_DEG))
        {
            if (DartControl.descent_confirm_count < DART_DESCENT_CONFIRM_COUNT)
                DartControl.descent_confirm_count++;
        }
        else
        {
            DartControl.descent_confirm_count = 0U;
        }

        if (DartControl.descent_confirm_count >= DART_DESCENT_CONFIRM_COUNT)
        {
            /* 进入下降段即停止上升段 pitch 补偿，此后 pitch 只由视觉引导控制。
               保留 yaw/roll PID 状态，姿态保持不中断。 */
            DartControl.ascent_pitch_done = 1U;
            DartControl.telemetry.pitch_output_us = 0.0f;
            DartControl.telemetry.output_pitch_deg = 0.0f;
            reset_pid(&DartControl.pitch_pid);
            reset_pid(&DartControl.yaw_vision_pid);
            reset_pid(&DartControl.pitch_vision_pid);
            DartControl.descent_target_seen = 0U;
            DartControl.descent_vision_active = 0U;
            DartControl.telemetry.target_yaw_deg = 0.0f;
            DartControl.telemetry.target_pitch_deg = 0.0f;
            set_state(DART_STATE_DESCENT);
        }
        return;
    }

    /* 状态：DESCENT —— 下降段视觉引导 */
    if (DartControl.flight_state == DART_STATE_DESCENT)
        run_descent_guidance(&ins_snapshot);
}

/**
 * @brief 通知控制模块 IMU 是否就绪
 * @param ready 1 表示就绪，0 表示未就绪
 */
void control_notify_imu_ready(uint8_t ready)
{
    if (ready && !DartControl.imu_ready)
        debug_imu_ready_ms = HAL_GetTick();
    DartControl.imu_ready = ready ? 1U : 0U;
}

/**
 * @brief 串口接收回调（USART1），逐字节解析视觉帧
 * @param huart UART 句柄
 *
 * 帧格式：SOF + 数据 + CRC + EOF，校验通过后发布到邮箱。
 */
void control_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    /* 上位机启动复位帧：5B 5B 5B CRC8，清除握手锁存，允许重新握手 */
    if (reset_index == 0U)
    {
        if (DartControl.uart_rx_byte == DART_RESET_SOF)
            reset_buf[reset_index++] = DartControl.uart_rx_byte;
    }
    else
    {
        reset_buf[reset_index++] = DartControl.uart_rx_byte;
        if (reset_index >= DART_RESET_FRAME_LEN)
        {
            if ((reset_buf[1] == DART_RESET_SOF) &&
                (reset_buf[2] == DART_RESET_SOF) &&
                (reset_buf[3] == vision_crc8(reset_buf, 3U)))
            {
                DartControl.attitude_tx_armed = 0U;
                ack_index = 0U;
            }
            reset_index = 0U;
        }
    }

    /* 握手 ACK：5A MODE FE CRC8；收到后允许发送姿态遥测 */
    if (!DartControl.attitude_tx_armed)
    {
        if (ack_index == 0U)
        {
            if (DartControl.uart_rx_byte == DART_ACK_SOF)
                ack_buf[ack_index++] = DartControl.uart_rx_byte;
        }
        else
        {
            ack_buf[ack_index++] = DartControl.uart_rx_byte;
            if (ack_index >= DART_ACK_FRAME_LEN)
            {
                if ((ack_buf[2] == DART_HANDSHAKE_EOF) &&
                    (ack_buf[1] == (uint8_t)DART_MODE) &&
                    (ack_buf[3] == vision_crc8(ack_buf, 3U)))
                {
                    DartControl.attitude_tx_armed = 1U;
                }
                ack_index = 0U;
            }
        }
    }

    if (DartControl.uart_frame_index == 0U)
    {
        /* 等待帧头 SOF */
        if (DartControl.uart_rx_byte == DART_VISION_SOF)
            DartControl.uart_frame[DartControl.uart_frame_index++] =
                DartControl.uart_rx_byte;
    }
    else
    {
        /* 接收后续字节 */
        DartControl.uart_frame[DartControl.uart_frame_index++] =
            DartControl.uart_rx_byte;
        if (DartControl.uart_frame_index >= DART_VISION_FRAME_LEN)
        {
            /* 一帧接收完毕，校验 SOF、EOF 和 CRC */
            if ((DartControl.uart_frame[DART_VISION_SOF_INDEX] == DART_VISION_SOF) &&
                (DartControl.uart_frame[DART_VISION_EOF_INDEX] == DART_VISION_EOF) &&
                (DartControl.uart_frame[DART_VISION_CRC_INDEX] ==
                 vision_crc8(DartControl.uart_frame, DART_VISION_CRC_INDEX)))
            {
                publish_vision_frame();
            }
            else
            {
                DartControl.telemetry.vision.crc_error_count++;
            }
            DartControl.uart_frame_index = 0U;
        }
    }

    /* 重新启动接收中断 */
    (void)HAL_UART_Receive_IT(&huart1, &DartControl.uart_rx_byte, 1U);
}

/**
 * @brief 串口错误回调，复位接收状态并重新启动接收
 * @param huart UART 句柄
 */
void control_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    DartControl.uart_frame_index = 0U;
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    (void)HAL_UART_Receive_IT(&huart1, &DartControl.uart_rx_byte, 1U);
}

/**
 * @brief HAL 串口接收完成回调，转发到控制模块处理
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    control_uart_rx_callback(huart);
}

/**
 * @brief HAL 串口错误回调，转发到控制模块处理
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    control_uart_error_callback(huart);
}

/**
 * @brief 根据逻辑电平写入 RGB LED
 * @param red 红色分量（0/1）
 * @param green 绿色分量（0/1）
 * @param blue 蓝色分量（0/1）
 */
static void led_write(uint8_t red, uint8_t green, uint8_t blue)
{
    GPIO_PinState inactive =
        (DART_LED_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    HAL_GPIO_WritePin(DART_LED_GPIO_PORT, DART_LED_RED_PIN,
                      red ? DART_LED_ACTIVE_LEVEL : inactive);
    HAL_GPIO_WritePin(DART_LED_GPIO_PORT, DART_LED_GREEN_PIN,
                      green ? DART_LED_ACTIVE_LEVEL : inactive);
    HAL_GPIO_WritePin(DART_LED_GPIO_PORT, DART_LED_BLUE_PIN,
                      blue ? DART_LED_ACTIVE_LEVEL : inactive);
}

/**
 * @brief LED 任务：根据当前飞行状态显示不同颜色
 */
void led_Task(void)
{
    switch (control_get_state())
    {
    case DART_STATE_READY:
        led_write(0U, 0U, 1U);          /* 蓝：就绪 */
        break;
    case DART_STATE_IMU_READY:
        led_write(0U, 1U, 0U);          /* 绿：IMU 就绪 */
        break;
    case DART_STATE_SERVO_READY:
        led_write(0U, 1U, 1U);          /* 青：舵机就绪 */
        break;
    case DART_STATE_LAUNCHED:
        led_write(1U, 0U, 0U);          /* 红：已起飞 */
        break;
    case DART_STATE_ASCENT:
        led_write(1U, 1U, 0U);          /* 黄：上升段 */
        break;
    case DART_STATE_DESCENT:
        led_write(1U, 0U, 1U);          /* 紫：下降段 */
        break;
    case DART_STATE_DEBUG:
        /* 调试模式：蓝绿闪烁，提示未发射、舵机不输出 */
        if (((HAL_GetTick() / DART_LED_ERROR_BLINK_HALF_PERIOD_MS) & 0x01U) != 0U)
            led_write(0U, 1U, 1U);
        else
            led_write(0U, 0U, 0U);
        break;
    case DART_STATE_VISION_ERROR:
        /* 视觉错误：白色闪烁 */
        if (((HAL_GetTick() / DART_LED_ERROR_BLINK_HALF_PERIOD_MS) & 0x01U) != 0U)
            led_write(1U, 1U, 1U);
        else
            led_write(0U, 0U, 0U);
        break;
    default:
        led_write(0U, 0U, 0U);          /* 其他状态：熄灭 */
        break;
    }
}

/**
 * @brief 获取当前对外显示的飞行状态
 * @return 飞行状态枚举
 *
 * 若使能了预起飞链路错误指示，且视觉链路错误发生在起飞前，
 * 则返回 VISION_ERROR 状态，用于 LED 闪烁提示。
 */
DartFlightState_e control_get_state(void)
{
    if (DART_ENABLE_PRELAUNCH_LINK_ERROR_LED &&
        DartControl.vision_link_error &&
        (DartControl.flight_state < DART_STATE_LAUNCHED))
    {
        return DART_STATE_VISION_ERROR;
    }
    return DartControl.flight_state;
}

/**
 * @brief 获取遥测数据结构指针
 * @return 指向遥测数据的常量指针
 */
const DartControlTelemetry_t *control_get_telemetry(void)
{
    return &DartControl.telemetry;
}

