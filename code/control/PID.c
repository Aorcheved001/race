/*
 * PID.c
 *
 *  链式级联 PID 控制层
 *  支持：位置环 + 速度环 + 角速度环 三级级联
 *
 *  控制流程图：
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │                        PID 控制流程图                                │
 *  ├──────────────────────────────────────────────────────────────────────┤
 *  │                                                                      │
 *  │  ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐    │
 *  │  │ 位置环    │ --> │ 速度环    │ --> │ 角速度环  │ --> │ 驱动PWM  │    │
 *  │  │(外环)    │     │(中环)    │     │(内环)    │     │          │    │
 *  │  │          │     │          │     │          │     │          │    │
 *  │  │ 输入:    │     │ 输入:    │     │ 输入:    │     │ 输入:    │    │
 *  │  │ target   │     │ 位置环   │     │ 速度环   │     │ 角速度环 │    │
 *  │  │ - curr   │     │ 输出     │     │ 输出     │     │ 限幅    │    │
 *  │  │ = error  │     │ - 实际   │     │ - 实际   │     │          │    │
 *  │  │          │     │ = error  │     │ = error  │     │          │    │
 *  │  │          │     │          │     │          │     │          │    │
 *  │  │ 输出:    │     │ 输出:    │     │ 输出:    │     │ 输出:    │    │
 *  │  │ 目标速度 │     │ 目标PWM  │     │ 转向PWM  │     │ 驱动PWM  │    │
 *  │  └──────────┘     └──────────┘     └──────────┘     └──────────┘    │
 *  │       │                │                │                           │
 *  │       v                v                v                           │
 *  │  ┌──────────────────────────────────────────────────────────┐       │
 *  │  │  PID公式: out = Kp*e + Ki*∫e*dt + Kd*de/dt              │       │
 *  │  └──────────────────────────────────────────────────────────┘       │
 *  │                                                                      │
 *  └──────────────────────────────────────────────────────────────────────┘
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │  差速转向原理:                                                        │
 *  │    左轮速度 + 右轮速度 = 直线速度                                     │
 *  │    左轮速度 - 右轮速度 = 转向速度（差速）                             │
 *  │                                                                      │
 *  │    角速度 = (V右 - V左) / 轮距                                       │
 *  │    所以角速度环可以直接控制转向                                       │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */

//-------------------------------------------头文件引入------------------------------------------------------------
#include "zf_common_headfile.h"
#include "PID.h"
#include "motor.h"

//-------------------------------------------宏定义------------------------------------------------------------

/* PID输出限幅 - 与Motor.c的MOTOR_DRIVE_PWM_MAX保持一致 */
#define WHEEL_PID_PWM_ABS_MAX    2000.0f

/* 兼容接口估算yaw_rate时使用的轮距（m） */
#define COMPAT_WHEELBASE_M       0.2f


//-------------------------------------------全局变量------------------------------------------------------------

/* 全局PID控制层结构体 */
WHEEL_PID_LAYER g_wheel_pid;

/* 运行时速度环速度参考限幅值(m/s) */
static float s_speed_ref_abs_max_mps = 2.5f;

/* 运行时角速度环速度参考限幅值(rad/s) */
static float s_yaw_rate_ref_abs_max_rps = 5.0f;


//-------------------------------------------内部函数声明------------------------------------------------------------

static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s);


//-------------------------------------------内部核心函数实现------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     PID内部初始化
// 函数说明     pid_info    - PID状态结构体指针
// 返回参数     void
// 使用示例     pid_para_init(&pid_info);
// 备注信息     清零所有误差和累积历史值
//-------------------------------------------------------------------------------------------------------------------
void pid_para_init(PID_INFO *pid_info)
{
    if (!pid_info) return;

    pid_info->iError = 0.0f;
    pid_info->LastError = 0.0f;
    pid_info->PrevError = 0.0f;
    pid_info->SumError = 0.0f;
    pid_info->LastData = 0.0f;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数名称     标准PID一步计算
// 函数说明     pid_info    - PID状态结构体指针
// 函数说明     param       - PID参数数组 [Kp, Ki, Kd, 积分限幅]
// 函数说明     error       - 当前误差（目标值 - 实际值）
// 函数说明     dt_s        - 控制周期（秒）
// 返回参数     PID输出值
// 使用示例     out = pid_step(&pid_info, param, error, 0.004f);
// 备注信息     通用位置式PID实现
//              out = Kp*e + Ki*∫e*dt + Kd*de/dt
//-------------------------------------------------------------------------------------------------------------------
static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s)
{
    if (!pid_info || !param) return 0.0f;
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    /* 记录当前误差 */
    pid_info->iError = error;

    /* 积分项：累积误差（带限幅） */
    pid_info->SumError += error * dt_s;
    if (param[PID_PARAM_I_LIMIT] > 0.0f)
    {
        pid_info->SumError = func_limit(pid_info->SumError, param[PID_PARAM_I_LIMIT]);
    }

    /* 微分项：误差变化率 */
    float diff = (error - pid_info->LastError) / dt_s;

    /* PID输出 = 比例 + 积分 + 微分 */
    float out = param[PID_PARAM_KP] * error
              + param[PID_PARAM_KI] * pid_info->SumError
              + param[PID_PARAM_KD] * diff;

    /* 更新误差历史 */
    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = error;

    return out;
}


//-------------------------------------------初始化相关函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     链式PID初始化
// 函数说明     void
// 返回参数     void
// 使用示例     wheel_pid_init();
// 备注信息     设置默认PID参数并清零所有PID状态
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_init(void)
{
    /* 清零全局PID结构体 */
    memset(&g_wheel_pid, 0, sizeof(g_wheel_pid));

    /* 初始化各个PID状态 */
    pid_para_init(&g_wheel_pid.pid_pos_left);
    pid_para_init(&g_wheel_pid.pid_pos_right);
    pid_para_init(&g_wheel_pid.pid_spd_left);
    pid_para_init(&g_wheel_pid.pid_spd_right);
    pid_para_init(&g_wheel_pid.pid_yaw_rate);

    /*
     * 默认PID参数（需要根据实际车辆调整）
     *
     * 位置环参数:
     *   - Kp: 较大值以快速响应速度
     *   - Ki: 保持（消除静态误差）
     *   - Kd: 微分（抑制超调）
     *   - 积分限幅: 限制位置环输出速度目标）
     */
    g_wheel_pid.position_param[PID_PARAM_KP]       = 10.0f;
    g_wheel_pid.position_param[PID_PARAM_KI]       = 0.0f;
    g_wheel_pid.position_param[PID_PARAM_KD]       = 0.4f;
    g_wheel_pid.position_param[PID_PARAM_I_LIMIT]  = 2.5f;

    /*
     * 速度环参数:
     *   - Kp: 保持
     *   - Ki: 积分（主要控制作用）
     *   - Kd: 微分（抑制抖动）
     *   - 积分限幅: 限制速度环输出PWM值）
     */
    g_wheel_pid.speed_param[PID_PARAM_KP]          = 0.0f;
    g_wheel_pid.speed_param[PID_PARAM_KI]          = 50.0f;
    g_wheel_pid.speed_param[PID_PARAM_KD]          = 0.0f;
    g_wheel_pid.speed_param[PID_PARAM_I_LIMIT]     = 1000.0f;

    /*
     * 角速度环参数:
     *   - Kp: 较大值以快速响应转向
     *   - Ki: 积分（消除静态偏差）
     *   - Kd: 微分（抑制转向抖动）
     *   - 积分限幅: 限制角速度环输出转向PWM值）
     */
    g_wheel_pid.yaw_rate_param[PID_PARAM_KP]       = 2.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KI]       = 0.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KD]       = 0.1f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_I_LIMIT] = 100.0f;

    /* 初始化控制目标 */
    g_wheel_pid.cmd_speed_left_mps = 0.0f;
    g_wheel_pid.cmd_speed_right_mps = 0.0f;
    g_wheel_pid.ref_speed_left_mps = 0.0f;
    g_wheel_pid.ref_speed_right_mps = 0.0f;
    g_wheel_pid.cmd_yaw_rate_rps = 0.0f;
    g_wheel_pid.ref_yaw_rate_rps = 0.0f;

    /* 初始化使能状态（默认全部关闭） */
    g_wheel_pid.enable_output = 0u;
    g_wheel_pid.enable_speed_loop = 0u;
    g_wheel_pid.enable_position_loop = 0u;
    g_wheel_pid.enable_yaw_rate_loop = 0u;

    /* 标记已初始化 */
    g_wheel_pid.initialized = 1u;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数名称     使能PID控制状态（级联接口，4参数完整版）
// 函数说明     enable_output         - 总输出使能
// 函数说明     enable_speed_loop     - 速度环使能
// 函数说明     enable_position_loop  - 位置环使能
// 函数说明     enable_yaw_rate_loop  - 角速度环使能
// 返回参数     void
// 使用示例     wheel_pid_enable_cascade(1, 1, 0, 1);  // 输出? 速度环? 位置环? 角速度环?
// 备注信息     动态使能状态，支持多种模式
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_enable_cascade(uint8 enable_output, uint8 enable_speed_loop,
                      uint8 enable_position_loop, uint8 enable_yaw_rate_loop)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.enable_output = enable_output ? 1u : 0u;
    g_wheel_pid.enable_speed_loop = enable_speed_loop ? 1u : 0u;
    g_wheel_pid.enable_position_loop = enable_position_loop ? 1u : 0u;
    g_wheel_pid.enable_yaw_rate_loop = enable_yaw_rate_loop ? 1u : 0u;
}


//-------------------------------------------------------------------------------------------------------------------
// 兼容接口：wheel_pid_enable(单参数)
// 内部映射：wheel_pid_enable_cascade(enable_output, 1, 0, 0) — 仅使能速度环
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_enable(uint8 enable_output)
{
    wheel_pid_enable_cascade(enable_output, 1, 0, 0);
}


//-------------------------------------------参数设置相关函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置速度目标（m/s）（级联接口，左右轮独立控制）
// 函数说明     left_mps   - 左轮目标速度
// 函数说明     right_mps  - 右轮目标速度
// 返回参数     void
// 使用示例     wheel_pid_set_speed_target_cascade(1.0f, 1.0f);
// 备注信息     用于速度环或串级控制的速度目标
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_speed_target_cascade(float left_mps, float right_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_speed_left_mps = left_mps;
    g_wheel_pid.cmd_speed_right_mps = right_mps;
}


//-------------------------------------------------------------------------------------------------------------------
// 兼容接口：wheel_pid_set_target_speed(单参数)
// 内部映射：wheel_pid_set_speed_target_cascade(speed, speed) — 左右轮使用相同速度
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_target_speed(float target_speed_mps)
{
    wheel_pid_set_speed_target_cascade(target_speed_mps, target_speed_mps);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置位置目标（m）
// 函数说明     left_m     - 左轮目标位置
// 函数说明     right_m    - 右轮目标位置
// 返回参数     void
// 使用示例     wheel_pid_set_position_target(1.0f, 1.0f);
// 备注信息     位置环累积cmd_speed生成轨迹
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_position_target(float left_m, float right_m)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.target_pos_left_m = left_m;
    g_wheel_pid.target_pos_right_m = right_m;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置角速度目标（rad/s）
// 函数说明     yaw_rate_rps - 目标角速度
// 返回参数     void
// 使用示例     wheel_pid_set_yaw_target(1.5f);  // 1.5 rad/s
// 备注信息     正值右转，负值左转
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_yaw_target(float yaw_rate_rps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_yaw_rate_rps = yaw_rate_rps;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置速度环PID参数
// 函数说明     kp       - 比例系数
// 函数说明     ki       - 积分系数
// 函数说明     kd       - 微分系数
// 函数说明     i_limit  - 积分限幅
// 返回参数     void
// 使用示例     wheel_pid_set_speed_param(0.0f, 50.0f, 0.0f, 1000.0f);
// 备注信息     支持实时调整速度环PID
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.speed_param[PID_PARAM_KP] = kp;
    g_wheel_pid.speed_param[PID_PARAM_KI] = ki;
    g_wheel_pid.speed_param[PID_PARAM_KD] = kd;
    g_wheel_pid.speed_param[PID_PARAM_I_LIMIT] = i_limit;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置位置环PID参数
// 函数说明     kp       - 比例系数
// 函数说明     ki       - 积分系数
// 函数说明     kd       - 微分系数
// 函数说明     i_limit  - 积分限幅
// 返回参数     void
// 使用示例     wheel_pid_set_position_param(10.0f, 0.0f, 0.4f, 2.5f);
// 备注信息     支持实时调整位置环PID
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.position_param[PID_PARAM_KP] = kp;
    g_wheel_pid.position_param[PID_PARAM_KI] = ki;
    g_wheel_pid.position_param[PID_PARAM_KD] = kd;
    g_wheel_pid.position_param[PID_PARAM_I_LIMIT] = i_limit;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置角速度环PID参数
// 函数说明     kp       - 比例系数
// 函数说明     ki       - 积分系数
// 函数说明     kd       - 微分系数
// 函数说明     i_limit  - 积分限幅
// 返回参数     void
// 使用示例     wheel_pid_set_yaw_rate_param(2.0f, 0.0f, 0.1f, 100.0f);
// 备注信息     支持实时调整角速度环PID
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.yaw_rate_param[PID_PARAM_KP] = kp;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KI] = ki;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KD] = kd;
    g_wheel_pid.yaw_rate_param[PID_PARAM_I_LIMIT] = i_limit;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置速度参考限幅值
// 函数说明     abs_max_mps - 最大速度(m/s)
// 返回参数     void
// 使用示例     wheel_pid_set_speed_ref_limit_mps(2.5f);
// 备注信息     限制位置环输出速度目标
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (abs_max_mps > 1e-6f)
    {
        s_speed_ref_abs_max_mps = abs_max_mps;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     设置角速度参考限幅值
// 函数说明     abs_max_rps - 最大角速度(rad/s)
// 返回参数     void
// 使用示例     wheel_pid_set_yaw_rate_ref_limit_rps(5.0f);
// 备注信息     限制角速度环输出的转向PWM目标
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (abs_max_rps > 1e-6f)
    {
        s_yaw_rate_ref_abs_max_rps = abs_max_rps;
    }
}


//-------------------------------------------核心控制函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     链式PID更新（级联接口，完整版，支持yaw反馈）
// 函数说明     enc       - 编码器状态指针
// 函数说明     yaw_rps   - 实际角速度(rad/s)，如果传感器无法获取可传0
// 函数说明     dt_s      - 控制周期（秒）
// 返回参数     void
// 使用示例     wheel_pid_update_cascade(enc, imu660.data_Ripen.gyro_z, 0.004f);
// 备注信息     在4ms中断中调用，输出PWM到电机
//              支持：位置+速度+角速度串级、速度+角速度、仅速度、仅角速度
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_update_cascade(const EncoderLayerState *enc, float yaw_rps, float dt_s)
{
    if (!enc) return;
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    const uint8 pos_on = g_wheel_pid.enable_position_loop;
    const uint8 spd_on = g_wheel_pid.enable_speed_loop;
    const uint8 yaw_on = g_wheel_pid.enable_yaw_rate_loop;

    /*
     * 位置环使能时：目标位置累积速度目标（轨迹跟踪）
     * 相当于：target_pos += cmd_speed * dt
     * 确保位置环有足够的累积误差
     */
    if (pos_on)
    {
        g_wheel_pid.target_pos_left_m += g_wheel_pid.cmd_speed_left_mps * dt_s;
        g_wheel_pid.target_pos_right_m += g_wheel_pid.cmd_speed_right_mps * dt_s;
    }

    float out_left = 0.0f;
    float out_right = 0.0f;
    float out_steer = 0.0f;

    /*================================================================
     *                    模式1：全串级（位置 + 速度 + 角速度）
     * ---------------------------------------------------------------
     *   位置环输出 = 速度目标（m/s）
     *   速度环输出 = PWM直接驱动
     *   角速度环输出 = 转向PWM
     * ================================================================*/
    if (pos_on && spd_on && yaw_on)
    {
        /* 外环：位置环 */
        float err_pos_l = g_wheel_pid.target_pos_left_m - enc->odom_left_m;
        float err_pos_r = g_wheel_pid.target_pos_right_m - enc->odom_right_m;

        float v_l = pid_step(&g_wheel_pid.pid_pos_left, g_wheel_pid.position_param, err_pos_l, dt_s);
        float v_r = pid_step(&g_wheel_pid.pid_pos_right, g_wheel_pid.position_param, err_pos_r, dt_s);

        /* 限制速度目标 */
        v_l = func_limit_ab(v_l, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);
        v_r = func_limit_ab(v_r, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);

        g_wheel_pid.ref_speed_left_mps = v_l;
        g_wheel_pid.ref_speed_right_mps = v_r;

        /* 中环：速度环 */
        float err_spd_l = v_l - enc->speed_left_mps;
        float err_spd_r = v_r - enc->speed_right_mps;

        out_left = pid_step(&g_wheel_pid.pid_spd_left, g_wheel_pid.speed_param, err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param, err_spd_r, dt_s);

        /* 转向计算目标角速度 */
        float target_yaw = (v_r - v_l) / 0.2f;  /* 假设轮距0.2m，可根据实际修改 */
        g_wheel_pid.ref_yaw_rate_rps = target_yaw;

        /* 内环：角速度环 */
        float err_yaw = target_yaw - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /*================================================================
     *                    模式2：速度环 + 角速度环
     * ---------------------------------------------------------------
     *   速度环输出 = PWM直接驱动
     *   角速度环输出 = 转向PWM
     * ================================================================*/
    else if (!pos_on && spd_on && yaw_on)
    {
        g_wheel_pid.ref_speed_left_mps = g_wheel_pid.cmd_speed_left_mps;
        g_wheel_pid.ref_speed_right_mps = g_wheel_pid.cmd_speed_right_mps;

        /* 速度环 */
        float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
        float err_spd_r = g_wheel_pid.cmd_speed_right_mps - enc->speed_right_mps;

        out_left = pid_step(&g_wheel_pid.pid_spd_left, g_wheel_pid.speed_param, err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param, err_spd_r, dt_s);

        /* 转向计算目标角速度 */
        float target_yaw = (g_wheel_pid.cmd_speed_right_mps - g_wheel_pid.cmd_speed_left_mps) / 0.2f;
        g_wheel_pid.ref_yaw_rate_rps = target_yaw;

        /* 角速度环 */
        float err_yaw = target_yaw - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /*================================================================
     *                    模式3：仅速度环
     * ---------------------------------------------------------------
     *   速度环直接输出PWM
     * ================================================================*/
    else if (!pos_on && spd_on && !yaw_on)
    {
        g_wheel_pid.ref_speed_left_mps = g_wheel_pid.cmd_speed_left_mps;
        g_wheel_pid.ref_speed_right_mps = g_wheel_pid.cmd_speed_right_mps;

        float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
        float err_spd_r = g_wheel_pid.cmd_speed_right_mps - enc->speed_right_mps;

        out_left = pid_step(&g_wheel_pid.pid_spd_left, g_wheel_pid.speed_param, err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param, err_spd_r, dt_s);

        out_steer = 0.0f;
    }
    /*================================================================
     *                    模式4：仅角速度环
     * ---------------------------------------------------------------
     *   直接用角速度环控制转向
     * ================================================================*/
    else if (!pos_on && !spd_on && yaw_on)
    {
        /* 纯转向控制（速度为0） */
        out_left = 0.0f;
        out_right = 0.0f;

        g_wheel_pid.ref_yaw_rate_rps = g_wheel_pid.cmd_yaw_rate_rps;

        float err_yaw = g_wheel_pid.cmd_yaw_rate_rps - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /*================================================================
     *                    模式5：全部关闭
     * ================================================================*/
    else
    {
        g_wheel_pid.ref_speed_left_mps = 0.0f;
        g_wheel_pid.ref_speed_right_mps = 0.0f;
        g_wheel_pid.ref_yaw_rate_rps = 0.0f;
        out_left = 0.0f;
        out_right = 0.0f;
        out_steer = 0.0f;
    }

    /* PWM输出限幅 */
    out_left = func_limit(out_left, WHEEL_PID_PWM_ABS_MAX);
    out_right = func_limit(out_right, WHEEL_PID_PWM_ABS_MAX);
    out_steer = func_limit(out_steer, STEER_PWM_ABS_MAX);

    /* 保存输出 */
    g_wheel_pid.out_left_pwm = out_left;
    g_wheel_pid.out_right_pwm = out_right;
    g_wheel_pid.out_steer_pwm = out_steer;

    /* 电机输出 */
    if (g_wheel_pid.enable_output)
    {
        motor_control(motor_LB, (int16)out_left);
        motor_control(motor_RB, (int16)out_right);
        /* 角速度环输出 out_steer_pwm 供外部转向模块使用 */
    }
}


//=================================================================================================
// 兼容性接口实现（供旧代码调用，内部映射到级联PID）
// 旧代码使用单参数wheel_pid_enable、单参数wheel_pid_set_target_speed、2参数wheel_pid_update
// 这些函数内部调用对应的级联接口，保持旧代码无需修改即可工作
//=================================================================================================

//-------------------------------------------------------------------------------------------------------------------
// 兼容接口：wheel_pid_set_target_pos(占位)
// 旧工程未实际使用位置环，此接口兼容保留
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_target_pos(float target_pos_m)
{
    (void)target_pos_m;
}

//-------------------------------------------------------------------------------------------------------------------
// 兼容接口：wheel_pid_update(2参数，无yaw)
// 内部根据左右轮命令速度差自动估算yaw_rate，然后调用级联PID
// 当左右轮速度相同时（直行），估算yaw_rate = 0，与原单环PID行为一致
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_update(const EncoderLayerState *enc, float dt_s)
{
    /*
     * 根据左右轮命令速度差估算车身角速度：
     *   yaw_rate = (v_right - v_left) / wheelbase
     * 当左右轮速度相同时（直行），yaw_rate = 0
     * 当左右轮有速度差时，产生转向角速度
     */
    float estimated_yaw = (g_wheel_pid.cmd_speed_right_mps - g_wheel_pid.cmd_speed_left_mps) / COMPAT_WHEELBASE_M;

    wheel_pid_update_cascade(enc, estimated_yaw, dt_s);
}
