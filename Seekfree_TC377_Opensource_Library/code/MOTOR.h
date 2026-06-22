
#include "zf_common_headfile.h"

#ifndef _MOTOR_CTRL_H
#define _MOTOR_CTRL_H

#define DIR_L              (P11_9)                          // 左电机方向脚
#define PWM_L              (ATOM2_CH5_P11_10)                // 左电机PWM脚
#define DIR_R              (P11_11)                          // 右电机方向脚
#define PWM_R              (ATOM2_CH7_P11_12)                // 右电机PWM脚

typedef enum{
    MOTOR_L = 0,
    MOTOR_R
};

// 函数声明
int32 range_protect( int32 current_value , int32 min , int32 max );
void Motor_Init(void);
void Motor_Ctrl( uint8 motor , int32 pwm );

#endif
