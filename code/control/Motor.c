#include "motor.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机硬件初始化
// 参数说明     void
// 返回参数     void
// 备注信息     初始化4个电机的PWM及方向引脚，PWM频率设置为 17kHz，方向引脚配置为推挽输出
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    // 左后 (LB)
    pwm_init(MOTORC_PWM_PIN, 17000, 0);
    gpio_init(MOTORC_DIR_PIN, GPO, 1, GPO_PUSH_PULL);

    // 右后 (RB)
    pwm_init(MOTORD_PWM_PIN, 17000, 0);
    gpio_init(MOTORD_DIR_PIN, GPO, 1, GPO_PUSH_PULL);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单个电机控制底层函数
// 参数说明     motor: 电机通道枚举值 (motor_LF/RF/LB/RB)
//              duty:  PWM占空比 (带符号整数)
// 返回参数     void
// 备注信息     duty > 0 时方向引脚置1，duty < 0 时方向引脚置0，取绝对值输出PWM
//              PWM限制范围：最小1500，最大2500
//-------------------------------------------------------------------------------------------------------------------
void motor_control(MOTOR_TYPE motor, int16 duty)
{
    uint8 dir = (duty > 0) ? 1 : 0;
    int16 abs_duty = abs(duty);

    if(abs_duty > 2000) abs_duty = 2000;
    if(abs_duty < 1200 && abs_duty != 0) abs_duty = 1200;

    switch(motor)
    {
        case motor_LB: // 左后
            pwm_set_duty(MOTORC_PWM_PIN, (uint16)abs_duty);
            gpio_set_level(MOTORC_DIR_PIN, dir);
            break;

        case motor_RB: // 右后
            pwm_set_duty(MOTORD_PWM_PIN, (uint16)abs_duty);
            gpio_set_level(MOTORD_DIR_PIN, dir);
            break;
    }
}

