/*
 * servo.h
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */

#ifndef CODE_CONTROLPART_SERVO_H_
#define CODE_CONTROLPART_SERVO_H_

#include "zf_common_headfile.h"

// ================= 用户配置区 =================
// 请根据实际硬件连接修改此处
#define SERVO_PIN             ATOM1_CH1_P33_9   // 舵机引脚
#define SERVO_FREQ            50                // 舵机频率 (50Hz - 300Hz)
#define SERVO_ANGLE_MIN       75.0f             // 软件限制的最小角度
#define SERVO_ANGLE_MAX       105.0f            // 软件限制的最大角度
// =============================================

// 编译时检查频率范围
#if (SERVO_FREQ < 50 || SERVO_FREQ > 300)
    #error "SERVO_FREQ 必须在 50 到 300 之间!"
#endif

#define     limit_value(x, a, b)    ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))


//------------------------------------------函数声明区------------------------------------------------------------

void servo_init(void);                                      // 初始化舵机模块

void servo_set_angle(float angle);                    // 设置舵机角度

float servo_get_angle(void);                          // 获取当前角度

float servo_get_angle_rad_relative(void);             // 获取当前角度 (弧度)

#endif /* CODE_CONTROLPART_SERVO_H_ */
