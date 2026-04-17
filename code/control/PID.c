/*
 * PID.c
 *
 *  Created on: 2026-03-24
 *      Author: Daydreamer
 */
//-------------------------------------------头文件声明区------------------------------------------------------------
#include "zf_common_headfile.h"
#include "PID.h"
#include "motor.h"

//-------------------------------------------变量定义区------------------------------------------------------------
WHEEL_PID_LAYER g_wheel_pid;                                         // 后轮速度PID全局对象

//-------------------------------------------内部函数定义区------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     速度环PID计算
//  参数说明     pid_info      PID 状态结构体
//  参数说明     pid_param     参数数组 [Kp, Ki, Kd, I_limit]
//  参数说明     target        目标速度
//  参数说明     current       当前速度
//  参数说明     dt_s          采样周期（s）
//  返回参数     float         PWM输出
//-------------------------------------------------------------------------------------------------------------------
static float pid_speed_step(PID_INFO *pid_info,
                            const float *pid_param,
                            float target,
                            float current,
                            float dt_s)
{
    if (!pid_info || !pid_param) return 0.0f;
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    pid_info->iError = target - current;                             // 当前误差 e(k)

    pid_info->SumError += pid_info->iError * dt_s;                   // 积分项累加
    if (pid_param[3] > 0.0f)
    {
        pid_info->SumError = func_limit(pid_info->SumError, pid_param[3]); // 积分限幅
    }

    float diff = (pid_info->iError - pid_info->LastError) / dt_s;    // 微分项

    float out = pid_param[0] * pid_info->iError                      // P
              + pid_param[1] * pid_info->SumError                    // I
              + pid_param[2] * diff;                                 // D

    pid_info->LastError = pid_info->iError;                          // 保存误差用于下次微分
    return out;
}

//-------------------------------------------对外函数定义区------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     清零 PID 参数结构体
//  参数说明     pid_info  PID 状态结构体指针
//-------------------------------------------------------------------------------------------------------------------
void pid_para_init(PID_INFO *pid_info)
{
    if (!pid_info) return;
    pid_info->iError = 0.0f;
    pid_info->LastError = 0.0f;
    pid_info->SumError = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     后轮速度PID初始化
//  参数说明     无
//  备注信息     仅速度环，用于纵向控制
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_init(void)
{
    memset(&g_wheel_pid, 0, sizeof(g_wheel_pid));

    pid_para_init(&g_wheel_pid.pid_spd);                             // 速度环状态清零

    g_wheel_pid.speed_param[0] = 10.0f;                             // 速度环 Kp（输出PWM）
    g_wheel_pid.speed_param[1] = 40.0f;                              // 速度环 Ki
    g_wheel_pid.speed_param[2] = 0.0f;                               // 速度环 Kd
    g_wheel_pid.speed_param[3] = 2.0f;                               // 速度环积分限幅（m/s·s）

    g_wheel_pid.target_speed_mps = 0.0f;                             // 目标速度
    g_wheel_pid.speed_limit = 2.0f;                                  // 速度限幅 2 m/s
    g_wheel_pid.out_pwm = 0.0f;                                      // 输出PWM

    g_wheel_pid.enable_output = 0u;                                  // 默认不下发电机输出
    g_wheel_pid.initialized = 1u;                                    // 初始化完成
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     设置 PID 输出使能
//  参数说明     enable_output 1=输出PWM到电机 0=不输出
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_enable(uint8 enable_output)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    if (enable_output && !g_wheel_pid.enable_output)
    {
        pid_para_init(&g_wheel_pid.pid_spd);
    }

    g_wheel_pid.enable_output = enable_output ? 1u : 0u;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     设置后轮中心位置目标
//  参数说明     target_pos_m 目标位置（m）
//  备注信息     保留接口兼容性，实际不使用位置控制
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_target_pos(float target_pos_m)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    (void)target_pos_m;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     设置后轮中心速度目标
//  参数说明     target_speed_mps 目标速度
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_target_speed(float target_speed_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    g_wheel_pid.target_speed_mps = func_limit(target_speed_mps, g_wheel_pid.speed_limit);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     设置位置环参数
//  参数说明     kp, ki, kd, i_limit
//  备注信息     保留接口兼容性，实际不使用位置控制
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    (void)kp; (void)ki; (void)kd; (void)i_limit;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     设置速度环参数
//  参数说明     kp, ki, kd, i_limit
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    g_wheel_pid.speed_param[0] = kp;
    g_wheel_pid.speed_param[1] = ki;
    g_wheel_pid.speed_param[2] = kd;
    g_wheel_pid.speed_param[3] = i_limit;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     速度PID周期更新
//  参数说明     enc   编码器层状态
//  参数说明     dt_s  周期时间
//  备注信息     仅速度环，用于纵向控制。横向控制由Pure Pursuit负责。
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_update(const EncoderLayerState *enc, float dt_s)
{
    if (!enc) return;
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    float current_speed_mps = enc->speed_average_mps;

    float spd_out = pid_speed_step(&g_wheel_pid.pid_spd,
                                   g_wheel_pid.speed_param,
                                   g_wheel_pid.target_speed_mps,
                                   current_speed_mps,
                                   dt_s);

    spd_out = func_limit(spd_out, 2000.0f);
    g_wheel_pid.out_pwm = spd_out;

    if (g_wheel_pid.enable_output)
    {
        motor_control(motor_LB, (int16)spd_out);
        motor_control(motor_RB, (int16)spd_out);
    }
}
