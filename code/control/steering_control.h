/*
 * steering_control.h
 *
 *  Created on: 2026年4月12日
 *      Author: A
 */

#ifndef CODE_CONTROL_STEERING_CONTROL_H_
#define CODE_CONTROL_STEERING_CONTROL_H_
//-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------硬件引脚定义------------------------------------------------------------
#define STEER_MOTOR_PWM_PIN     ATOM1_CH1_P33_9     // 转向电机 PWM     (要修改)
#define STEER_MOTOR_DIR_PIN     P33_11               // 转向电机 方向引脚   (要修改)

//-------------------------------------------控制参数标定区------------------------------------------------------------
// 调参指南：先调KP使响应快速且无明显超调，再加KD抑制超调，最后加GKD补偿车身旋转
#define KP                      50.0f               // 比例增益
#define KD                      0.5f                // 微分增益
#define GKD                     10.0f               // 陀螺仪前馈增益

#define STEER_MAX_ANGLE         30.0f               // 默认角度限制（度）
#define STEER_MAX_ANGLE_U_TURN  45.0f               // 掉头模式角度限制（度）
#define STEER_PWM_MAX           2000                // PWM 输出限幅

#define STEER_ZERO_SAMPLES      10                  // 零位标定时采样次数
#define STEER_GEAR_RATIO        (30.0f / 56.0f)     //齿轮比
//-------------------------------------------外部结构体声明------------------------------------------------------------
extern imu660_struct imu660;                       // IMU 数据结构体

//-------------------------------------------函数声明区------------------------------------------------------------
void steering_init(void);                          // 转向模块初始化
void steering_control(void);                       // 转向控制（4ms 周期调用）
void steering_set_target(float angle_deg);         // 设置目标角度（单位：度）

float steering_get_current_angle(void);            // 获取当前转向角度（单位：度）
void steering_set_zero_at_current(void);           // 以当前方向为零点
void steering_set_zero_offset(int16 raw_offset);   // 手动设置零位偏移（原始值偏移量）
void steering_set_output_enabled(uint8 enable);        // 使能/关闭转向输出
void steering_set_max_angle(float max_angle_deg);  // 设置角度限制（默认30度，掉头可用45度）





#endif /* CODE_CONTROL_STEERING_CONTROL_H_ */
