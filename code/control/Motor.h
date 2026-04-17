#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

// ========================== 电机引脚定义 ==========================
// 假设 C 是左后，D 是右后
#define MOTORC_PWM_PIN      ATOM0_CH2_P21_4    // 左后电机 PWM
#define MOTORC_DIR_PIN      P21_3               // 左后电机 方向

#define MOTORD_PWM_PIN      ATOM0_CH3_P21_5     // 右后电机 PWM
#define MOTORD_DIR_PIN      P21_2               // 右后电机 方向

// ========================== 类型定义 ==========================
typedef enum
{
    motor_LB, // 左后 (Left Back)
    motor_RB, // 右后 (Right Back)
} MOTOR_TYPE;

// ========================== 函数声明 ==========================
void motor_init(void);                              // 电机初始化
void motor_control(MOTOR_TYPE motor, int16 duty);   // 单个电机控制

#endif /* CODE_MOTOR_H_ */
