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

 //-------------------------------------------头文件------------------------------------------------------------
 #include "zf_common_headfile.h"

 //-------------------------------------------宏定义------------------------------------------------------------

 /* PID参数索引 */
 #define PID_PARAM_KP         0       // 比例系数
 #define PID_PARAM_KI         1       // 积分系数
 #define PID_PARAM_KD         2       // 微分系数
 #define PID_PARAM_I_LIMIT    3       // 积分限幅

 /* PID输出限幅 */
 #define WHEEL_PID_PWM_ABS_MAX       2000.0f   // 驱动PWM绝对值最大值
 #define SPEED_REF_ABS_MAX_MPS       2.5f      // 速度参考限幅值(m/s)
 #define YAW_RATE_REF_ABS_MAX_RPS    5.0f     // 角速度参考限幅值(rad/s)
 #define STEER_PWM_ABS_MAX           200.0f   // 转向PWM限幅值（差速转向差值）

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
     float speed_param[4];            // 速度环PID参数 [Kp, Ki, Kd, 积分限幅]
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


 //-------------------------------------------函数声明------------------------------------------------------------

 /**
  * @brief PID内部初始化
  * @param pid_info PID状态结构体指针
  */
 void pid_para_init(PID_INFO *pid_info);

 /**
  * @brief 链式PID初始化
  * @note  设置默认PID参数并清零所有状态
  */
 void wheel_pid_init(void);

 /**
  * @brief 使能PID控制（兼容接口，旧代码调用此函数）
  * @note  内部等价于 wheel_pid_enable_cascade(1, 1, 0, 0)，仅使能速度环
  */
 void wheel_pid_enable(uint8 enable_output);

 /**
  * @brief 使能PID控制（级联接口，4参数完整版）
  * @param enable_output         总输出使能
  * @param enable_speed_loop      速度环使能
  * @param enable_position_loop   位置环使能
  * @param enable_yaw_rate_loop   角速度环使能
  */
 void wheel_pid_enable_cascade(uint8 enable_output, uint8 enable_speed_loop,
                       uint8 enable_position_loop, uint8 enable_yaw_rate_loop);

 /**
  * @brief 设置速度目标（m/s）（兼容接口，旧代码调用此函数）
  * @note  左右轮使用相同速度值，内部调用 wheel_pid_set_speed_target_cascade
  */
 void wheel_pid_set_speed_target(float target_speed_mps);

 /**
  * @brief 设置速度目标（m/s）（级联接口，左右轮独立控制）
  * @param left_mps  左轮目标速度
  * @param right_mps 右轮目标速度
  */
 void wheel_pid_set_speed_target_cascade(float left_mps, float right_mps);

 /**
  * @brief 设置位置目标（m）
  * @param left_m  左轮目标位置
  * @param right_m 右轮目标位置
  */
 void wheel_pid_set_position_target(float left_m, float right_m);

 /**
  * @brief 设置角速度目标（rad/s）
  * @param yaw_rate_rps 目标角速度
  */
 void wheel_pid_set_yaw_target(float yaw_rate_rps);

 /**
  * @brief 设置速度环PID参数
  * @param kp       比例系数
  * @param ki       积分系数
  * @param kd       微分系数
  * @param i_limit  积分限幅
  */
 void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit);

 /**
  * @brief 设置位置环PID参数
  * @param kp       比例系数
  * @param ki       积分系数
  * @param kd       微分系数
  * @param i_limit  积分限幅
  */
 void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit);

 /**
  * @brief 设置角速度环PID参数
  * @param kp       比例系数
  * @param ki       积分系数
  * @param kd       微分系数
  * @param i_limit  积分限幅
  */
 void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit);

 /**
  * @brief 设置速度参考限幅值
  * @param abs_max_mps  最大速度(m/s)
  */
 void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps);

 /**
  * @brief 设置角速度参考限幅值
  * @param abs_max_rps 最大角速度(rad/s)
  */
 void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps);

 /**
  * @brief 链式PID更新（兼容接口，旧代码调用此函数）
  * @param enc       编码器状态指针
  * @param dt_s      控制周期（秒），通常4ms=0.004s
  * @note  在4ms中断中调用，内部根据左右轮速度差自动估算yaw_rate
  */
 void wheel_pid_update(const EncoderLayerState *enc, float dt_s);

 /**
  * @brief 链式PID更新（级联接口，完整版，支持yaw反馈）
  * @param enc       编码器状态指针
  * @param yaw_rps   实际角速度(rad/s)，如果传感器无法获取可传0
  * @param dt_s      控制周期（秒），通常4ms=0.004s
  * @note  在4ms中断中调用，输出PWM到电机
  */
 void wheel_pid_update_cascade(const EncoderLayerState *enc, float yaw_rps, float dt_s);

 /**
  * @brief 兼容接口：设置目标位置（旧接口，占位）
  * @note  旧工程未实际使用位置环，此接口兼容保留
  */
 void wheel_pid_set_target_pos(float target_pos_m);

 #endif /* CODE_CONTROL_PID_H_ */
