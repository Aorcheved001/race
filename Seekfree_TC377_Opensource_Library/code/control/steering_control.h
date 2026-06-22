/*
 * steering_control.h
 *
 *  Created on: 2026年4月12日
 *      Author: A
 *  Description: 转向控制模块 —— 绝对编码器 + PD + 陀螺仪前馈
 */

#ifndef CODE_CONTROL_STEERING_CONTROL_H_
#define CODE_CONTROL_STEERING_CONTROL_H_
//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------硬件引脚定义（DRV8701驱动）------------------------------------------------------------
#define STEER_DIR_PIN           P13_2              // 方向引脚（GPIO输出）
#define STEER_PWM_PIN           ATOM3_CH0_P13_3    // PWM引脚（速度控制）
// DRV8701: DIR高=右转(正), DIR低=左转(负), PWM控制速度

//-------------------------------------------编码器参数------------------------------------------------------------
#define STEER_ENCODER_RES       4096               // 编码器分辨率（12位，底层已右移4位）
#define STEER_HOME_RAW          631                // 机械原位对应编码器原始值
#define STEER_LEFT_MAX_RAW      1793               // 左转物理打死raw值（按20260621 steerpid实测修正）
#define STEER_RIGHT_MAX_RAW     3645               // 右转物理打死raw值（按20260621 steerpid实测修正，跨零点）

// 左右转最大变化量（用于角度映射）
#define STEER_LEFT_MAX_DIFF     (STEER_LEFT_MAX_RAW - STEER_HOME_RAW)                      // 957
#define STEER_RIGHT_MAX_DIFF    (STEER_ENCODER_RES - STEER_RIGHT_MAX_RAW + STEER_HOME_RAW) // 1150

//-------------------------------------------PID 控制参数------------------------------------------------------------
#define STEER_KP                320.0f             // 转向比例系数（实测端点映射修正后保留当前响应力度）
#define STEER_KD                0.5f               // 转向微分系数
#define STEER_GKD               0.0f               // 陀螺仪前馈系数
#define STEER_CONTROL_DT        0.004f             // 控制周期（秒），与中断一致

//-------------------------------------------差速转向助力参数（陀螺仪角速率闭环）------------------------------------------------------------
#define DIFF_STEER_KP               20.0f             // 差速角速率比例系数（rad/s误差→脉冲/4ms）
#define DIFF_STEER_KI               1.0f              // 差速角速率积分系数（消除稳态角速率误差）
// 从3.0降到1.0：3.0积分过快→弯道中积分累积→出弯后积分未释放→继续转弯→震荡
// 1.0积分缓慢：稳态仍有修正能力，但不会在弯道中过度累积
#define DIFF_STEER_KD               0.0f              // 差速角速率微分系数
#define DIFF_STEER_MAX_DELTA        2.0f              // 差速最大偏移量（脉冲/4ms，浮点）
#define DIFF_STEER_DEADZONE_RAD_S   0.01f             // 差速死区（rad/s），小于此值不启用差速
#define DIFF_STEER_LOW_SPEED_THRESH 0.3f              // 低速阈值（m/s），低于此值差速增益放大
#define DIFF_STEER_LOW_SPEED_GAIN   1.5f              // 低速差速增益倍数
// 差速助力原理（陀螺仪闭环版）：
//   输入 = 目标角速率 - 实际角速率（陀螺仪测量）
//   目标角速率 = v * tan(δ) / L （自行车模型/单轨模型）
//   实际角速率 = imu660.gyro_z（陀螺仪直接测量，不受打滑影响）
//   输出 = 差速脉冲偏移，左轮+delta，右轮-delta
//   优势：
//     1. 稳态仍有输出：转弯中目标ω持续存在，差速持续辅助，真正缩小转弯半径
//     2. 打滑鲁棒：陀螺仪不受轮子打滑影响，差速判断准确
//     3. 倒车支持：倒车时v<0，目标ω自动反向，差速方向正确
//   死区0.05rad/s：直行时陀螺仪噪声不触发差速
//-------------------------------------------差速曲率前馈参数------------------------------------------------------------
#define DIFF_STEER_FF_GAIN          2.0f              // 曲率前馈增益（1/m → 脉冲/4ms）
// 曲率前馈原理：
//   PP算法计算出路径曲率κ后，直接预加差速偏移，不等航向偏差出现再PID修正
//   前馈量 = KFF * |κ| * sign(κ) * v
//   弯道入口即给差速，消除转向延迟导致的切角
//   KFF=2.0：κ=0.5(1/m), v=0.46m/s → 前馈≈0.46脉冲，约占MAX_DELTA的23%
//   仿真含电机延迟(tau=40ms)扫描34组数据：FF=2最优(avg|max|=0.0262m)
//   FF=2~8差异<3%，但FF=2更保守，过校正风险更低
#define DIFF_STEER_FF_MAX          2.0f              // 前馈最大偏移量（脉冲/4ms）
// 前馈限幅：防止极端曲率下前馈过大导致振荡
// 与PID的MAX_DELTA独立限幅，总差速 = 前馈 + PID反馈
//-------------------------------------------控制限幅------------------------------------------------------------
#define STEER_MAX_ANGLE         VEHICLE_MAX_STEER_ANGLE_DEG  // 最大转角（度）
#define STEER_PWM_MAX           8500               // PWM 输出限幅
#define STEER_PWM_MIN_EFFECTIVE 1300               // 克服静摩擦的最小有效PWM
#define STEER_ERROR_DEADBAND_DEG 0.40f              // 目标附近死区，避免回中抖动

//-------------------------------------------外部变量------------------------------------------------------------
extern imu660_struct imu660;
typedef struct
{
    float target_deg;          // 目标转向角，左负右正
    float current_deg;         // 实际转向角，左负右正
    float error_deg;           // target - current
    float derivative_deg_s;    // 误差微分
    float output_pwm;          // PID输出，正=右转，负=左转
    float diff_delta;          // 后轮差速助力输出
    float diff_ff_delta;       // 后轮差速前馈输出
    float omega_target_rad_s;  // 差速闭环目标角速度，右转为正
    float omega_actual_rad_s;  // 陀螺测得实际角速度，右转为正
    int16 raw;                 // 绝对编码器原始值
    int32 diff;                // 相对原位差值，左正右负
} SteeringPidDebug;

//-------------------------------------------函数接口------------------------------------------------------------
void steering_init(void);
void steering_control(void);                       // 每个控制周期调用一次
void steering_set_target(float angle_deg);         // 设置目标角度（左负右正）
float steering_get_current_angle(void);            // 获取当前转向角度（左负右正）
void   steering_get_raw_and_diff(int16 *raw, int32 *diff);  // 获取编码器原始值和差值
void steering_send_angle_to_host(void);
void steering_get_pid_debug(SteeringPidDebug *out); // 获取转向PID调试快照

//-------------------------------------------差速转向助力接口------------------------------------------------------------
float steering_get_diff_delta(void);              // 获取当前差速偏移量（脉冲/4ms，浮点）
void steering_diff_assist_enable(uint8 enable);     // 启用/禁用差速助力（1=启用，0=禁用）
void steering_diff_set_speed_dir(float dir);        // 设置行驶方向（1.0=前进，-1.0=倒车）
void steering_diff_set_curvature_ff(float kappa);  // 设置路径曲率前馈（1/m，正值=左转，负值=右转）

#endif /* CODE_CONTROL_STEERING_CONTROL_H_ */
