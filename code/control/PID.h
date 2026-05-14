/*
 * PID.h
 *
 * 级联式 PID 控制器
 * 支持：位置环 + 速度环 + 偏航角速度环
 *
 * 整体说明
 * +------------------------------------------------------------------+
 * |                    差速底盘 PID 控制架构                         |
 * +------------------------------------------------------------------+
 * |                                                                  |
 * |   外环(位置环)   -->   中环(速度环)   -->   内环(偏航角速度环)   -->   输出PWM   |
 * |   target_pos          ref_speed          ref_yaw_rate           out_steer_pwm   |
 * |   (m)                 (m/s)              (rad/s)                转向PWM         |
 * |                                                                  |
 * |   输入：              输入：             输入：                                |
 * |   累加位置指令        速度目标值         偏航角速度指令 / 转向指令            |
 * |                                                                  |
 * |   参数：              参数：             参数：                                |
 * |   position_param      speed_param        yaw_rate_param                       |
 * |                                                                  |
 * +------------------------------------------------------------------+
 * |   速度环直接输出左右轮 PWM                                        |
 * |   偏航角速度偏差 = 目标转弯角速度 - 实际转弯角速度                 |
 * +------------------------------------------------------------------+
 *
 * 使用方法：
 *   1. wheel_pid_init()              - 初始化（main中调用一次）
 *   2. wheel_pid_enable(...)         - 使能各控制环
 *   3. wheel_pid_set_speed_target()  - 设置速度目标（m/s）
 *   4. wheel_pid_set_yaw_target()    - 设置偏航角速度目标（rad/s）
 *   5. wheel_pid_update(enc, dt)     - 周期调用（推荐4ms）
 *
 * 兼容旧接口：
 *   - wheel_pid_enable(1,1,0,1)      使能对应环
 *   - wheel_pid_set_target_speed(v)  设置相同左右轮速度
 *   - wheel_pid_update(enc, dt)      内部自动估算偏航角速度
 */

#ifndef CODE_CONTROL_PID_H_
#define CODE_CONTROL_PID_H_

//------------------------------------------- 头文件引用 ------------------------------------------------------------

#include "zf_common_headfile.h"
#include "vehicle_config.h"

//------------------------------------------- 宏定义 -----------------------------------------------------------------

/* PID 参数数组索引 */
#define PID_PARAM_KP         0       /* 比例系数 */
#define PID_PARAM_KI         1       /* 积分系数 */
#define PID_PARAM_KD         2       /* 微分系数 */
#define PID_PARAM_I_LIMIT    3       /* 积分限幅值 */

/* PWM 和参考值限幅 */
#define WHEEL_PID_PWM_ABS_MAX       2000.0f   /* PWM绝对最大值（原始范围） */
#define SPEED_REF_ABS_MAX_MPS       2.5f      /* 速度参考限幅 (m/s) */
#define YAW_RATE_REF_ABS_MAX_RPS    5.0f      /* 偏航角速度参考限幅 (rad/s) */
#define STEER_PWM_ABS_MAX           200.0f    /* 转向PWM限幅值 */

#define WHEEL_PID_PWM_LIMIT_PERCENT 70.0f     /* PWM输出百分比限幅（默认70%） */

//------------------------------------------- 结构体定义 -------------------------------------------------------------

/**
 * PID 内部状态结构体（位置式PID）
 */
typedef struct
{
    float iError;        /* 当前误差 */
    float LastError;     /* 上一次误差 */
    float PrevError;     /* 上上一次误差（用于微分） */
    float SumError;      /* 误差积分值 */
    float LastData;      /* 上一次输出值 */
} PID_INFO;

/**
 * 差速底盘级联 PID 总控制器结构体
 */
typedef struct
{
    /*--------- PID 内部状态 --------*/
    PID_INFO pid_pos_left;           /* 左轮位置环 PID 状态 */
    PID_INFO pid_pos_right;          /* 右轮位置环 PID 状态 */
    PID_INFO pid_spd_left;           /* 左轮速度环 PID 状态 */
    PID_INFO pid_spd_right;          /* 右轮速度环 PID 状态 */
    PID_INFO pid_yaw_rate;           /* 偏航角速度环 PID 状态 */

    /*--------- PID 参数 --------*/
    float position_param[4];         /* 位置环 PID 参数 [Kp, Ki, Kd, I_limit] */
    float speed_param_left[4];       /* 左轮速度环 PID 参数 */
    float speed_param_right[4];      /* 右轮速度环 PID 参数 */
    float yaw_rate_param[4];         /* 偏航角速度环 PID 参数 */

    /*--------- 目标与参考值 --------*/
    float cmd_speed_left_mps;        /* 指令速度 - 左轮 (m/s) */
    float cmd_speed_right_mps;       /* 指令速度 - 右轮 (m/s) */
    float target_pos_left_m;         /* 目标位置 - 左轮 (m) */
    float target_pos_right_m;        /* 目标位置 - 右轮 (m) */
    float ref_speed_left_mps;        /* 参考速度 - 左轮 (m/s) */
    float ref_speed_right_mps;       /* 参考速度 - 右轮 (m/s) */
    float cmd_yaw_rate_rps;          /* 指令偏航角速度 (rad/s) */
    float ref_yaw_rate_rps;          /* 参考偏航角速度 (rad/s) */

    /*--------- PID 输出 --------*/
    float out_left_pwm;              /* 左轮 PWM 输出值 */
    float out_right_pwm;             /* 右轮 PWM 输出值 */
    float out_steer_pwm;             /* 转向 PWM 输出值 */

    /*--------- 使能标志 --------*/
    uint8 enable_output;             /* 总输出使能 */
    uint8 enable_speed_loop;         /* 速度环使能 */
    uint8 enable_position_loop;      /* 位置环使能 */
    uint8 enable_yaw_rate_loop;      /* 偏航角速度环使能 */
    uint8 initialized;               /* 初始化完成标志 */
} WHEEL_PID_LAYER;


//------------------------------------------- 外部变量声明 -----------------------------------------------------------

extern WHEEL_PID_LAYER g_wheel_pid;
extern int16 origin_pidout;
extern int16 origin_pid_sum_error;
extern int16 origin_error_right;
extern int16 origin_error_left;


//------------------------------------------- 函数声明 ---------------------------------------------------------------

void wheel_pid_init(void);                                                    /* 初始化 PID 控制器 */

void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,           /* 使能控制环 */
                      uint8 enable_position_loop, uint8 enable_yaw_rate_loop);

void wheel_pid_set_speed_target(float left_mps, float right_mps);             /* 设置左右轮目标速度 (m/s) */

void wheel_pid_set_target_speed(float target_speed_mps);                      /* 设置相同目标速度（直线快捷接口） */

void wheel_pid_set_position_target(float left_m, float right_m);              /* 设置左右轮目标位置 (m) */

void wheel_pid_set_yaw_target(float yaw_rate_rps);                            /* 设置目标偏航角速度 (rad/s) */

void wheel_pid_set_speed_param_left(float kp, float ki, float kd, float i_limit);   /* 设置左轮速度环参数 */

void wheel_pid_set_speed_param_right(float kp, float ki, float kd, float i_limit);  /* 设置右轮速度环参数 */

void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit);        /* 同时设置左右轮速度环参数 */

void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit);     /* 设置位置环参数 */

void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit);     /* 设置偏航角速度环参数 */

void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps);                    /* 设置速度参考限幅 (m/s) */

void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps);                 /* 设置偏航角速度参考限幅 (rad/s) */

void wheel_pid_update(const EncoderLayerState *enc, float dt_s);              /* 主更新函数（推荐调用） */

#endif /* CODE_CONTROL_PID_H_ */
