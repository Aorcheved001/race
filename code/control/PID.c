/*
 * PID.c
 *
 * 级联式 PID 控制器
 * 支持：位置环 + 速度环 + 偏航角速度环（转弯控制）
 *
 * 控制框图
 * +------------------------------------------------------------------------------+
 * |                          PID 级联控制结构图                                  |
 * +------------------------------------------------------------------------------+
 * |                                                                              |
 * |  +----------------+   +----------------+   +----------------+   +--------+  |
 * |  |   位置环         |   |   速度环       |   |   偏航角速度环  |   | 输出PWM|  |
 * |  | (外环)          |   | (中环)         |   | (内环)        |   |        |  |
 * |  |                |   |               |   |              |   |        |  |
 * |  | 输入: 目标位置    |   | 输入: 目标速度   |   | 输入: 目标角速  |   |        |  |
 * |  | - 当前位置       |   | - 当前速度      |   | - 当前角速度    |   |        |  |
 * |  | = 误差          |   | = 误差         |   | = 误差        |   |        |  |
 * |  |                |   |               |   |              |   |        |  |
 * |  | 输出: 目标速度    |   | 输出: PWM      |   | 输出: 转向PWM  |   | 最终PWM|  |
 * |  +----------------+   +----------------+   +----------------+   +--------+  |
 * |        |                   |                   |                   |        |
 * |        v                   v                   v                   v        |
 * |  +--------------------------------------------------------------+            |
 * |  |  PID公式: out = Kp*e + Ki*Σ(e*dt) + Kd*de/dt               |            |
 * |  +--------------------------------------------------------------+            |
 * |                                                                              |
 * +------------------------------------------------------------------------------+
 *
 * 差速转向原理
 *    右轮速度 + 左轮速度 = 直线速度
 *    右轮速度 - 左轮速度 = 转向角速度
 *
 * Created on: 2026-03-15
 *     Author: Daydreamer
 */

#include "zf_common_headfile.h"

//------------------------------------------- 宏定义 ------------------------------------------------------------

/* PID 输出 PWM 百分比最大值（与 Motor.c 接口匹配） */
#define WHEEL_PID_PWM_PERCENT_MAX    60.0f
#define WHEEL_PID_DEADZONE  4.0f     // 死区阈值（最大死区PWM=5）    5.0f
#define WHEEL_PID_MIN_START_PWM   10.0f     // 最小启动PWM值（关键参数）

/* 包含车辆配置参数 */
#include "vehicle_config.h"


//------------------------------------------- 全局变量 ----------------------------------------------------------

/* 全局 PID 控制器结构体 */
WHEEL_PID_LAYER g_wheel_pid;

/* 速度环参考速度限幅值 (m/s) */
static float s_speed_ref_abs_max_mps = 1.0f;

/* 偏航角速度环参考限幅值 (rad/s) */
static float s_yaw_rate_ref_abs_max_rps = 5.0f;

int16 origin_pidout = 0;

int16 origin_pid_sum_error = 0;
int16 origin_error_right = 0;
int16 origin_error_left = 0;
//------------------------------------------- 内部函数声明 ------------------------------------------------------

static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s);


//------------------------------------------- PID 核心算法 ------------------------------------------------------

// 初始化 PID 状态变量
static void pid_para_init(PID_INFO *pid_info)
{
    if (!pid_info) return;

    pid_info->iError    = 0.0f;
    pid_info->LastError = 0.0f;
    pid_info->PrevError = 0.0f;
    pid_info->SumError  = 0.0f;
    pid_info->LastData  = 0.0f;
}

// 标准位置式 PID 计算
static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s)
{
    if (!pid_info || !param) return 0.0f;
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    /* 记录当前误差 */
    pid_info->iError = error;

    /* 积分项（带限幅） */
    pid_info->SumError += error * dt_s;
    origin_pid_sum_error = pid_info->SumError;

    if (param[PID_PARAM_I_LIMIT] > 0.0f)
    {
        pid_info->SumError = func_limit(pid_info->SumError, param[PID_PARAM_I_LIMIT]);
    }

    /* 微分项 */
    float diff = (error - pid_info->LastError) / dt_s;

    /* PID 输出 */
    float out = param[PID_PARAM_KP] * error
              + param[PID_PARAM_KI] * pid_info->SumError
              + param[PID_PARAM_KD] * diff;

    origin_pidout = out;


    /* 更新历史误差 */
    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = error;

    return out;
}


//------------------------------------------- 初始化函数 -------------------------------------------------------

// PID 控制器初始化
void wheel_pid_init(void)
{
    /* 清空全局结构体 */
    memset(&g_wheel_pid, 0, sizeof(g_wheel_pid));

    /* 初始化各环 PID 状态 */
    pid_para_init(&g_wheel_pid.pid_pos_left);
    pid_para_init(&g_wheel_pid.pid_pos_right);
    pid_para_init(&g_wheel_pid.pid_spd_left);
    pid_para_init(&g_wheel_pid.pid_spd_right);
    pid_para_init(&g_wheel_pid.pid_yaw_rate);

    /* 默认 PID 参数（建议根据实际车辆调试） */

    // 位置环参数
    g_wheel_pid.position_param[PID_PARAM_KP]      = 10.0f;
    g_wheel_pid.position_param[PID_PARAM_KI]      = 0.0f;
    g_wheel_pid.position_param[PID_PARAM_KD]      = 0.4f;
    g_wheel_pid.position_param[PID_PARAM_I_LIMIT] = 2.5f;

    // 速度环参数（左轮）
    g_wheel_pid.speed_param_left[PID_PARAM_KP]       = 10.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_KI]       = 1.8f;
    g_wheel_pid.speed_param_left[PID_PARAM_KD]       = 0.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_I_LIMIT]  = 200.0f;

    // 速度环参数（右轮）
    g_wheel_pid.speed_param_right[PID_PARAM_KP]      = 10.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_KI]      = 1.8f;
    g_wheel_pid.speed_param_right[PID_PARAM_KD]      = 0.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_I_LIMIT] = 200.0f;

    // 偏航角速度环参数
    g_wheel_pid.yaw_rate_param[PID_PARAM_KP]      = 2.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KI]      = 0.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KD]      = 0.1f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_I_LIMIT] = 100.0f;

    /* 初始化目标值 */
    g_wheel_pid.cmd_speed_left_mps  = 0.0f;
    g_wheel_pid.cmd_speed_right_mps = 0.0f;
    g_wheel_pid.ref_speed_left_mps  = 0.0f;
    g_wheel_pid.ref_speed_right_mps = 0.0f;
    g_wheel_pid.cmd_yaw_rate_rps    = 0.0f;
    g_wheel_pid.ref_yaw_rate_rps    = 0.0f;

    /* 默认关闭所有环 */
    g_wheel_pid.enable_output        = 0u;
    g_wheel_pid.enable_speed_loop    = 0u;
    g_wheel_pid.enable_position_loop = 0u;
    g_wheel_pid.enable_yaw_rate_loop = 0u;

    g_wheel_pid.initialized = 1u;

    wheel_pid_cmd_init();
}

// PID 各环使能控制
void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,
                      uint8 enable_position_loop, uint8 enable_yaw_rate_loop)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    g_wheel_pid.enable_output        = enable_output ? 1u : 0u;
    g_wheel_pid.enable_speed_loop    = enable_speed_loop ? 1u : 0u;
    g_wheel_pid.enable_position_loop = enable_position_loop ? 1u : 0u;
    g_wheel_pid.enable_yaw_rate_loop = enable_yaw_rate_loop ? 1u : 0u;
}

//------------------------------------------- 设置目标值 -------------------------------------------------------

// 设置左右轮目标速度 (m/s)
void wheel_pid_set_speed_target(float left_mps, float right_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_speed_left_mps  = left_mps;
    g_wheel_pid.cmd_speed_right_mps = right_mps;
}

// 设置相同目标速度（直线行驶快捷接口）
void wheel_pid_set_target_speed(float target_speed_mps)
{
    wheel_pid_set_speed_target(target_speed_mps, target_speed_mps);
}

// 设置左右轮目标位置 (m)
void wheel_pid_set_position_target(float left_m, float right_m)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.target_pos_left_m  = left_m;
    g_wheel_pid.target_pos_right_m = right_m;
}

// 设置目标偏航角速度 (rad/s)
void wheel_pid_set_yaw_target(float yaw_rate_rps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_yaw_rate_rps = yaw_rate_rps;
}

//------------------------------------------- 参数整定接口 -----------------------------------------------------

void wheel_pid_set_speed_param_left(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.speed_param_left[PID_PARAM_KP] = kp;
    g_wheel_pid.speed_param_left[PID_PARAM_KI] = ki;
    g_wheel_pid.speed_param_left[PID_PARAM_KD] = kd;
    g_wheel_pid.speed_param_left[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_speed_param_right(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.speed_param_right[PID_PARAM_KP] = kp;
    g_wheel_pid.speed_param_right[PID_PARAM_KI] = ki;
    g_wheel_pid.speed_param_right[PID_PARAM_KD] = kd;
    g_wheel_pid.speed_param_right[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit)
{
    wheel_pid_set_speed_param_left(kp, ki, kd, i_limit);
    wheel_pid_set_speed_param_right(kp, ki, kd, i_limit);
}

void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.position_param[PID_PARAM_KP] = kp;
    g_wheel_pid.position_param[PID_PARAM_KI] = ki;
    g_wheel_pid.position_param[PID_PARAM_KD] = kd;
    g_wheel_pid.position_param[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.yaw_rate_param[PID_PARAM_KP] = kp;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KI] = ki;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KD] = kd;
    g_wheel_pid.yaw_rate_param[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (abs_max_mps > 1e-6f)
        s_speed_ref_abs_max_mps = abs_max_mps;
}

void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (abs_max_rps > 1e-6f)
        s_yaw_rate_ref_abs_max_rps = abs_max_rps;
}

//------------------------------------------- 主控制更新函数 ---------------------------------------------------

// 级联 PID 更新核心函数（内部使用）
static void wheel_pid_update_cascade(const EncoderLayerState *enc, float yaw_rps, float dt_s)
{
    if (!enc) return;
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    const uint8 pos_on = g_wheel_pid.enable_position_loop;
    const uint8 spd_on = g_wheel_pid.enable_speed_loop;
    const uint8 yaw_on = g_wheel_pid.enable_yaw_rate_loop;

    float out_left = 0.0f;
    float out_right = 0.0f;
    float out_steer = 0.0f;

    // 位置环使能时，累加位置目标（轨迹跟踪）
    if (pos_on)
    {
        g_wheel_pid.target_pos_left_m  += g_wheel_pid.cmd_speed_left_mps * dt_s;
        g_wheel_pid.target_pos_right_m += g_wheel_pid.cmd_speed_right_mps * dt_s;
    }

    /* ======================== 模式1：位置 + 速度  (偏航角速度) ======================== */
    if (pos_on && spd_on && yaw_on)
    {
        // 位置环
        float err_pos_l = g_wheel_pid.target_pos_left_m - enc->odom_left_m;
        float err_pos_r = g_wheel_pid.target_pos_right_m - enc->odom_right_m;

        float v_l = pid_step(&g_wheel_pid.pid_pos_left, g_wheel_pid.position_param, err_pos_l, dt_s);
        float v_r = pid_step(&g_wheel_pid.pid_pos_right, g_wheel_pid.position_param, err_pos_r, dt_s);

        // 限幅速度参考值
        v_l = func_limit_ab(v_l, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);
        v_r = func_limit_ab(v_r, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);

        g_wheel_pid.ref_speed_left_mps = v_l;
        g_wheel_pid.ref_speed_right_mps = v_r;

        // 速度环
        float err_spd_l = v_l - enc->speed_left_mps;
        float err_spd_r = v_r - enc->speed_right_mps;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

        // 偏航角速度环
//        float target_yaw = (v_r - v_l) / COMPAT_WHEELBASE_M;
//        g_wheel_pid.ref_yaw_rate_rps = target_yaw;
//
//        float err_yaw = target_yaw - yaw_rps;
//        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);

        out_steer = 0;
    }
    /* ======================== 模式2：速度 + 偏航角速度 ======================== */
    else if (!pos_on && spd_on && yaw_on)
    {
        g_wheel_pid.ref_speed_left_mps  = g_wheel_pid.cmd_speed_left_mps;
        g_wheel_pid.ref_speed_right_mps = g_wheel_pid.cmd_speed_right_mps;

        float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
        float err_spd_r = g_wheel_pid.cmd_speed_right_mps - enc->speed_right_mps;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

        float target_yaw = (g_wheel_pid.cmd_speed_right_mps - g_wheel_pid.cmd_speed_left_mps) / COMPAT_WHEELBASE_M;
        g_wheel_pid.ref_yaw_rate_rps = target_yaw;

        float err_yaw = target_yaw - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /* ======================== 模式3：仅速度环 ======================== */
    else if (!pos_on && spd_on && !yaw_on)
    {
        g_wheel_pid.ref_speed_left_mps  = g_wheel_pid.cmd_speed_left_mps;
        g_wheel_pid.ref_speed_right_mps = g_wheel_pid.cmd_speed_right_mps;

        float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
        //float err_spd_l = enc->speed_left_mps - g_wheel_pid.cmd_speed_left_mps;     // actual - target
        float err_spd_r = g_wheel_pid.cmd_speed_right_mps + enc->speed_right_mps;

        origin_error_right = err_spd_r;
        origin_error_left  = err_spd_l;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

        out_steer = 0.0f;

    }
    /* ======================== 模式4：仅偏航角速度环 ======================== */
    else if (!pos_on && !spd_on && yaw_on)
    {
        out_left = 0.0f;
        out_right = 0.0f;
        g_wheel_pid.ref_yaw_rate_rps = g_wheel_pid.cmd_yaw_rate_rps;

        float err_yaw = g_wheel_pid.cmd_yaw_rate_rps - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /* ======================== 模式5：全部关闭 ======================== */
    else
    {
        g_wheel_pid.ref_speed_left_mps = 0.0f;
        g_wheel_pid.ref_speed_right_mps = 0.0f;
        g_wheel_pid.ref_yaw_rate_rps = 0.0f;
        out_left = out_right = out_steer = 0.0f;
    }

    /* PWM 限幅 */
    out_left  = func_limit(out_left,  WHEEL_PID_PWM_PERCENT_MAX);
    out_right = func_limit(out_right, WHEEL_PID_PWM_PERCENT_MAX);
    out_steer = func_limit(out_steer, STEER_PWM_ABS_MAX);

//    /* 死区控制 */
//    if (fabs(out_left) < WHEEL_PID_DEADZONE)
//        out_left = 0.0f;
//
//    if (fabs(out_right) < WHEEL_PID_DEADZONE)
//        out_right = 0.0f;

//    // 左轮
//    if (out_left > 0 && out_left < WHEEL_PID_MIN_START_PWM)
//        out_left = WHEEL_PID_MIN_START_PWM;
//    else if (out_left < 0 && out_left > -WHEEL_PID_MIN_START_PWM)
//        out_left = -WHEEL_PID_MIN_START_PWM;
//
//    // 右轮
//    if (out_right > 0 && out_right < WHEEL_PID_MIN_START_PWM)
//        out_right = WHEEL_PID_MIN_START_PWM;
//    else if (out_right < 0 && out_right > -WHEEL_PID_MIN_START_PWM)
//        out_right = -WHEEL_PID_MIN_START_PWM;

    /* 保存输出值 */
    g_wheel_pid.out_left_pwm  = out_left;
    g_wheel_pid.out_right_pwm = out_right;
    g_wheel_pid.out_steer_pwm = out_steer;

    /* 执行电机控制 */
    if (g_wheel_pid.enable_output)
    {
//----------------------------正常pid--------------------------------------------------------//
        if (out_left > 0)
            motor_control(motor_LB, MOTOR_DIR_FORWARD, (uint8)out_left);
        else if (out_left < 0)
            motor_control(motor_LB, MOTOR_DIR_REVERSE, (uint8)(-out_left));
        else
            motor_control(motor_LB, MOTOR_DIR_BRAKE, 0);

        if (out_right > 0)
            motor_control(motor_RB, MOTOR_DIR_FORWARD, (uint8)out_right);
        else if (out_right < 0)
            motor_control(motor_RB, MOTOR_DIR_REVERSE, (uint8)(-out_right));
        else
            motor_control(motor_RB, MOTOR_DIR_BRAKE, 0);

//----------------------------测试-------------------------------------------------------------//
//        if (out_left > 0)
//            motor_control(motor_LB, MOTOR_DIR_FORWARD, 50);
//        else if (out_left < 0)
//            motor_control(motor_LB, MOTOR_DIR_REVERSE, 50);
//        else
//            motor_control(motor_LB, MOTOR_DIR_BRAKE, 0);
//
//        if (out_right > 0)
//            motor_control(motor_RB, MOTOR_DIR_FORWARD,50);
//        else if (out_right < 0)
//            motor_control(motor_RB, MOTOR_DIR_REVERSE, 50);
//        else
//            motor_control(motor_RB, MOTOR_DIR_BRAKE, 0);

        /* 转向 PWM 可由外部模块使用 g_wheel_pid.out_steer_pwm */
    }
}

// 外部调用接口（推荐使用此函数）
void wheel_pid_update(const EncoderLayerState *enc, float dt_s)
{
    /* 使用编码器速度估算偏航角速度 */
    float estimated_yaw = (enc->speed_right_mps - enc->speed_left_mps) / COMPAT_WHEELBASE_M;
    wheel_pid_update_cascade(enc, estimated_yaw, dt_s);
}
