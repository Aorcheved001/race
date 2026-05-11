/*
 * PID.h
 *
 *  级联式 PID 控制层
 *  支持：位置环 + 速度环 + 角速度环 三级控制
 *
 *  架构说明：
 *  +------------------------------------------------------------------+
 *  |                    级联 PID 控制架构                             |
 *  +------------------------------------------------------------------+
 *  |                                                                  |
 *  |   外环(位置环)   -->   中环(速度环)   -->   内环(角速度环)   -->   输出PWM   |
 *  |   target_pos          ref_speed          ref_yaw_rate           out_steer_pwm   |
 *  |   (m)                 (m/s)              (rad/s)                转向舵机          |
 *  |                                                                  |
 *  |   输入：              输入：             输入：                                |
 *  |   累积位置误差        速度目标误差       角速度误差/转向误差                  |
 *  |                                                                  |
 *  |   输出参数            输出参数           输出参数                              |
 *  |   position_param      speed_param        yaw_rate_param                       |
 *  |                                                                  |
 *  +------------------------------------------------------------------+
 *  |   速度环直接输出驱动电机PWM                                          |
 *  |   角速度偏差 = 转向角速度 - 实际转向                                 |
 *  +------------------------------------------------------------------+
 *
 *  使用方法：
 *    1. wheel_pid_init()              - 初始化（在main中调用一次）
 *    2. wheel_pid_enable(...)         - 使能控制环
 *    3. wheel_pid_set_speed_target()  - 设置速度目标（m/s）
 *    4. wheel_pid_set_yaw_target()    - 设置角速度目标（rad/s）
 *    5. wheel_pid_update(enc, dt)     - 4ms周期调用（核心计算）
 *
 *  兼容接口（可从旧代码调用，内部映射到级联PID）：
 *    - wheel_pid_enable(1)           ：使能全部环
 *    - wheel_pid_set_target_speed(v) ：设置两轮速度为v
 *    - wheel_pid_update(enc, dt)     ：内部估计yaw_rate，调用级联PID
 */

#ifndef CODE_CONTROL_PID_H_
#define CODE_CONTROL_PID_H_

//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"
#include "vehicle_config.h"

//-------------------------------------------宏定义-----------------------------------------------------------------

/* PID参数索引 */
#define PID_PARAM_KP         0       /* 比例系数 */
#define PID_PARAM_KI         1       /* 积分系数 */
#define PID_PARAM_KD         2       /* 微分系数 */
#define PID_PARAM_I_LIMIT    3       /* 积分限幅 */

/* PID输出限幅 */
#define WHEEL_PID_PWM_ABS_MAX       2000.0f   /* 驱动PWM最大值（绝对值） */
#define SPEED_REF_ABS_MAX_MPS       2.5f      /* 速度参考限幅值(m/s) */
#define YAW_RATE_REF_ABS_MAX_RPS    5.0f      /* 角速度参考限幅值(rad/s) */
#define STEER_PWM_ABS_MAX           200.0f    /* 转向PWM限幅值（舵机转向最大值） */

//-------------------------------------------结构体定义-------------------------------------------------------------

/**
 * PID控制器内部状态（增量式、帧微分的各种历史）
 */
typedef struct
{
    float iError;        /* 当前误差（用于微分计算） */
    float LastError;     /* 上一帧误差 */
    float PrevError;     /* 上上一帧误差（用于微分） */
    float SumError;      /* 误差累积（用于积分） */
    float LastData;      /* 上一帧数据（用于微分计算） */
} PID_INFO;

/**
 * 级联式多环PID控制器结构体
 * 包含位置环、速度环、角速度环的PID状态
 */
typedef struct
{
    /*--------- PID内部状态 --------*/
    PID_INFO pid_pos_left;           /* 左轮位置环PID状态 */
    PID_INFO pid_pos_right;          /* 右轮位置环PID状态 */
    PID_INFO pid_spd_left;           /* 左轮速度环PID状态 */
    PID_INFO pid_spd_right;          /* 右轮速度环PID状态 */
    PID_INFO pid_yaw_rate;           /* 角速度环PID状态（统一处理） */

    /*--------- PID参数 --------*/
    float position_param[4];         /* 位置环PID参数 [Kp, Ki, Kd, 积分限幅] */
    float speed_param[4];            /* 速度环PID参数 [Kp, Ki, Kd, 积分限幅] */
    float yaw_rate_param[4];         /* 角速度环PID参数 [Kp, Ki, Kd, 积分限幅] */

    /*--------- 控制目标 --------*/
    float cmd_speed_left_mps;        /* 控制速度-左轮(m/s) */
    float cmd_speed_right_mps;       /* 控制速度-右轮(m/s) */
    float target_pos_left_m;         /* 目标位置-左轮(m) */
    float target_pos_right_m;        /* 目标位置-右轮(m) */
    float ref_speed_left_mps;        /* 速度参考-左轮(m/s) */
    float ref_speed_right_mps;       /* 速度参考-右轮(m/s) */
    float cmd_yaw_rate_rps;          /* 控制角速度(rad/s) */
    float ref_yaw_rate_rps;          /* 角速度参考值(rad/s) */

    /*--------- PID输出 --------*/
    float out_left_pwm;              /* 左轮PWM输出（前轮电机） */
    float out_right_pwm;             /* 右轮PWM输出（前轮电机） */
    float out_steer_pwm;             /* 转向PWM输出（舵机转向） */

    /*--------- 使能控制 --------*/
    uint8 enable_output;             /* 输出使能 */
    uint8 enable_speed_loop;         /* 速度环使能 */
    uint8 enable_position_loop;      /* 位置环使能 */
    uint8 enable_yaw_rate_loop;      /* 角速度环使能 */
    uint8 initialized;               /* 初始化标志 */
} WHEEL_PID_LAYER;


//-------------------------------------------外部变量声明------------------------------------------------------------
extern WHEEL_PID_LAYER g_wheel_pid;


//-------------------------------------------函数声明---------------------------------------------------------------

void wheel_pid_init(void);                                                    /* 初始化PID控制器 */

void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,           /* 使能控制环 */
        uint8 enable_position_loop, uint8 enable_yaw_rate_loop);

void wheel_pid_set_speed_target(float left_mps, float right_mps);             /* 设置速度目标，单位m/s */

void wheel_pid_set_position_target(float left_m, float right_m);              /* 设置位置目标，单位m */

void wheel_pid_set_yaw_target(float yaw_rate_rps);                            /* 设置角速度目标，单位rad/s */

void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit);  /* 设置速度环PID参数 */

void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit);    /* 设置位置环PID参数 */

void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit);    /* 设置角速度环PID参数 */

void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps);                    /* 设置速度参考限幅值，单位m/s */

void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps);                 /* 设置角速度参考限幅值，单位rad/s */

void wheel_pid_update(const EncoderLayerState *enc, float dt_s);              /* 更新PID计算，周期调用 */

void wheel_pid_set_target_speed(float target_speed_mps);                      /* 设置目标速度（兼容接口），单位m/s */


#endif /* CODE_CONTROL_PID_H_ */
