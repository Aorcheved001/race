/*
 * PID.h
 *
 *  Created on: 2026-03-24
 *      Author: Daydreamer
 */
#ifndef _FLY_MOTOR_PID_h
#define _FLY_MOTOR_PID_h

//-------------------------------------------头文件声明区------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------结构体定义区------------------------------------------------------------
typedef struct
{
    float iError;                          // 当前误差 e(k)
    float LastError;                       // 上一拍误差 e(k-1)
    float SumError;                        // 误差积分 Σe
} PID_INFO;

typedef struct
{
    PID_INFO pid_spd;                      // 速度环 PID 状态

    float speed_param[4];                  // 速度环参数：[0]=Kp [1]=Ki [2]=Kd [3]=积分限幅

    float target_speed_mps;                // 目标速度

    float speed_limit;                     // 速度限幅

    float out_pwm;                         // 最终输出 PWM（左右后轮同值）

    uint8 enable_output;                   // 输出使能：1=下发电机PWM，0=不下发
    uint8 initialized;                     // 模块初始化标志：1=已初始化
} WHEEL_PID_LAYER;

//-------------------------------------------变量声明区------------------------------------------------------------
extern WHEEL_PID_LAYER g_wheel_pid;

//-------------------------------------------函数声明区------------------------------------------------------------
void pid_para_init(PID_INFO *pid_info);                     // 清零 PID 内部状态

void wheel_pid_init(void);                                  // 初始化速度PID

void wheel_pid_enable(uint8 enable_output);                 // 设置输出使能

void wheel_pid_set_target_pos(float target_pos_m);          // 保留接口兼容性

void wheel_pid_set_target_speed(float target_speed_mps);    // 设置目标速度

void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit); // 保留接口兼容性

void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit);    // 设置速度环参数

void wheel_pid_update(const EncoderLayerState *enc, float dt_s);                 // 速度PID周期更新

#endif
