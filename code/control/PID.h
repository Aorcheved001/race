/*
 * PID.h
 *
 *  链式级联 PID 控制层
 *  支持：位置环 + 速度环 + 角速度环 三级级联
 *
 *  控制架构说明：
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │                        级联 PID 控制架构                             │
 *  ├──────────────────────────────────────────────────────────────────────┤
 *  │                                                                      │
 *  │   外环(位置环)   →   中环(速度环)   →   内环(角速度环)  →   驱动PWM    │
 *  │   target_pos        ref_speed         ref_yaw_rate    out_steer_pwm   │
 *  │   (m)               (m/s)             (rad/s)         差速转向          │
 *  │                                                                      │
 *  │   作用：              作用：            作用：                        │
 *  │   累积位置误差        跟踪目标速度       控制转向/差速转向              │
 *  │                                                                      │
 *  │   调参数：            调参数：          调参数：                      │
 *  │   position_param      speed_param       yaw_rate_param               │
 *  │                                                                      │
 *  ├──────────────────────────────────────────────────────────────────────┤
 *  │   速度环输出直接控制左右轮PWM                                          │
 *  │   角速度偏差 = 转向角速度（差速转向）                                   │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  使用方法：
 *    1. wheel_pid_init()              - 初始化（main中调用一次）
 *    2. wheel_pid_enable(...)          - 使能控制环
 *    3. wheel_pid_set_speed_target()  - 设置速度目标（m/s）
 *    4. wheel_pid_set_yaw_target()    - 设置角速度目标（rad/s）
 *    5. wheel_pid_update(enc, dt)     - 4ms周期调用（核心计算）
 *
 *  兼容性接口（供旧代码调用，内部映射到级联PID）：
 *    - wheel_pid_enable(1)            → 使能全部环
 *    - wheel_pid_set_target_speed(v)   → 设置左右速度为v
 *    - wheel_pid_update(enc, dt)       → 内部估算yaw_rate后调用级联PID
 */

#ifndef CODE_CONTROL_PID_H_
#define CODE_CONTROL_PID_H_

//-------------------------------------------头文件引用------------------------------------------------------------
#include "zf_common_headfile.h"
#include "vehicle_config.h"          // 车辆参数统一配置

//-------------------------------------------宏定义------------------------------------------------------------

/* PID参数索引 */
#define PID_PARAM_KP         0       // 比例系数
#define PID_PARAM_KI         1       // 积分系数
#define PID_PARAM_KD         2       // 微分系数
#define PID_PARAM_I_LIMIT    3       // 积分限幅

/* PWM和参考值限幅 */
#define WHEEL_PID_PWM_ABS_MAX       2000.0f   // 驱动PWM绝对值最大值
#define SPEED_REF_ABS_MAX_MPS       2.5f      // 速度参考限幅值(m/s)
#define YAW_RATE_REF_ABS_MAX_RPS    5.0f      // 角速度参考限幅值(rad/s)
#define STEER_PWM_ABS_MAX           200.0f    // 转向PWM限幅值（差速转向差值）

#define WHEEL_PID_PWM_LIMIT_PERCENT 60.0f     // PWM输出百分比限幅

 //-------------------------------------------结构体定义--------------------------------------------------------

 /**
  * PID控制器内部状态（积分、微分历史）
  */
 typedef struct
 {
     float iError;        // 当前误差（用于微分计算）
     float LastError;     // 上一次误差
     float PrevError;     // 上上一次误差（用于微分）
     float SumError;      // 误差累积和
     float LastData;      // 上一次输出（用于微分计算）
 } PID_INFO;

 /**
  * 链式级联PID控制层结构体
  * 包含位置环、速度环、角速度环的PID状态
  */
 typedef struct
 {
     /*--------- PID内部状态 --------*/
     PID_INFO pid_pos_left;           // 左轮位置环PID状态
     PID_INFO pid_pos_right;          // 右轮位置环PID状态
     PID_INFO pid_spd_left;           // 左轮速度环PID状态
     PID_INFO pid_spd_right;          // 右轮速度环PID状态
     PID_INFO pid_yaw_rate;           // 角速度环PID状态（统一车身）

     /*--------- PID参数 --------*/
     float position_param[4];         // 位置环PID参数 [Kp, Ki, Kd, 积分限幅]
     float speed_param_left[4];       // 左轮速度环PID参数 [Kp, Ki, Kd, 积分限幅]
     float speed_param_right[4];      // 右轮速度环PID参数 [Kp, Ki, Kd, 积分限幅]
     float yaw_rate_param[4];         // 角速度环PID参数 [Kp, Ki, Kd, 积分限幅]

     /*--------- 控制目标 --------*/
     float cmd_speed_left_mps;        // 命令速度-左轮(m/s)
     float cmd_speed_right_mps;       // 命令速度-右轮(m/s)
     float target_pos_left_m;         // 目标位置-左轮(m)
     float target_pos_right_m;        // 目标位置-右轮(m)
     float ref_speed_left_mps;        // 速度参考-左轮(m/s)
     float ref_speed_right_mps;       // 速度参考-右轮(m/s)
     float cmd_yaw_rate_rps;          // 命令角速度(rad/s)
     float ref_yaw_rate_rps;          // 角速度参考值(rad/s)

     /*--------- PID输出 --------*/
     float out_left_pwm;              // 左轮PWM（向前驱动）
     float out_right_pwm;             // 右轮PWM（向前驱动）
     float out_steer_pwm;             // 转向PWM（差速转向）

     /*--------- 使能开关 --------*/
     uint8 enable_output;              // 总输出使能
     uint8 enable_speed_loop;          // 速度环使能
     uint8 enable_position_loop;      // 位置环使能
     uint8 enable_yaw_rate_loop;       // 角速度环使能
     uint8 initialized;               // 初始化标志
 } WHEEL_PID_LAYER;


 //-------------------------------------------外部变量声明--------------------------------------------------------
 extern WHEEL_PID_LAYER g_wheel_pid;
 extern int16 origin_pidout;
 extern int16 origin_pid_sum_error;
 extern int16 origin_error_right;
 extern int16 origin_error_left;


 //-------------------------------------------函数声明------------------------------------------------------------

 void wheel_pid_init(void);                                                    // 初始化PID控制器
 
 void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,           // 使能控制环                       
         uint8 enable_position_loop, uint8 enable_yaw_rate_loop);   
 
 void wheel_pid_set_speed_target(float left_mps, float right_mps);             // 设置速度目标，单位：m/s
 
 void wheel_pid_set_position_target(float left_m, float right_m);              // 设置位置目标，单位：m
 
 void wheel_pid_set_yaw_target(float yaw_rate_rps);                            // 设置角速度目标，单位：rad/s
 
 void wheel_pid_set_speed_param_left(float kp, float ki, float kd, float i_limit);   // 设置左轮速度环参数
 void wheel_pid_set_speed_param_right(float kp, float ki, float kd, float i_limit);  // 设置右轮速度环参数
 void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit);        // 同时设置左右轮速度环参数
 
 void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit);    // 设置位置环PID参数
 
 void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit);    // 设置角速度环PID参数
 
 void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps);                    // 设置速度参考限幅值，单位：m/s
 
 void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps);                 // 设置角速度参考限幅值，单位：rad/s
 
 void wheel_pid_update(const EncoderLayerState *enc, float dt_s);              // 更新PID计算，周期调用
 
 void wheel_pid_set_target_speed(float target_speed_mps);                      // 设置目标速度（兼容接口），单位：m/s


 #endif /* CODE_CONTROL_PID_H_ */
