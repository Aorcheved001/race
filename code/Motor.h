#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

// ========================== 引脚定义 ==========================
//ATOM0_CH1_P33_9
//#define MOTOR1_PWM_PIN1     ATOM1_CH1_P33_9
#define MOTOR1_PWM_PIN1     ATOM0_CH1_P33_9
#define MOTOR1_PWM_PIN2     ATOM1_CH2_P33_11

#define MOTOR2_PWM_PIN1     ATOM0_CH3_P14_2
#define MOTOR2_PWM_PIN2     ATOM0_CH2_P14_3

// ========================== 类型定义 ==========================
typedef enum
{
    motor_LB,   // 左后 (Left Back)
    motor_RB,   // 右后 (Right Back)
} MOTOR_TYPE;

typedef enum
{
    MOTOR_DIR_FORWARD   =  1,   // 正转
    MOTOR_DIR_BRAKE     =  0,   // 刹车
    MOTOR_DIR_REVERSE   = -1,   // 反转
} MotorDir;

// ========================== 函数声明 ==========================
void motor_init(void);
void motor_control(MOTOR_TYPE motor, MotorDir dir, uint8 percent);

#endif /* CODE_MOTOR_H_ */
