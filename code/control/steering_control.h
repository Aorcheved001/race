/*
 * steering_control.h
 *
 *  Created on: 2026年4月12日
 *      Author: A
 *  Description: 转向控制模块 —— 绝对值编码器 + PD + 陀螺仪前馈
 *               零点由 STEER_HOME_RAW 固定标定，上电不变
 *               左转为负角度，右转为正角度
 */

#ifndef CODE_CONTROL_STEERING_CONTROL_H_
#define CODE_CONTROL_STEERING_CONTROL_H_
//-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------硬件引脚定义------------------------------------------------------------
#define STEER_MOTOR_PWM_PIN1    ATOM4_CH2_P21_4    // 正转PWM
#define STEER_MOTOR_PWM_PIN2    ATOM4_CH3_P21_5    // 反转PWM

//-------------------------------------------编码器参数------------------------------------------------------------
#define STEER_ENCODER_RES       4096               // 编码器分辨率（12位，底层已右移4位）
#define STEER_HOME_RAW          2139               // 机械原位对应编码器原始值
#define STEER_LEFT_MAX_RAW      3200               // 左转极限对应的raw值（左负）
#define STEER_RIGHT_MAX_RAW     875                // 右转极限对应的raw值（右正）

//-------------------------------------------PID 控制参数------------------------------------------------------------
// 调参顺序：KP → KD → GKD
#define STEER_KP                250.0f              // 比例增益
#define STEER_KD                0.5f               // 微分增益
#define STEER_GKD               0.0f              // 陀螺仪前馈增益
#define STEER_CONTROL_DT        0.004f             // 控制周期（秒），需与调用周期一致

//-------------------------------------------限制参数------------------------------------------------------------
#define STEER_MAX_ANGLE         VEHICLE_MAX_STEER_ANGLE_DEG  // 最大转向角度（度）
#define STEER_PWM_MAX           7500               // PWM 输出限幅

//-------------------------------------------外部声明------------------------------------------------------------
extern imu660_struct imu660;

//-------------------------------------------公开接口------------------------------------------------------------
void steering_init(void);
void steering_control(void);                       // 每个控制周期调用一次
void steering_set_target(float angle_deg);         // 设置目标角度，左负右正
float steering_get_current_angle(void);            // 获取当前相对角度，左负右正
void steering_send_angle_to_host(void);

#endif /* CODE_CONTROL_STEERING_CONTROL_H_ */
