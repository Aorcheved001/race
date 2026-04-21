#include "motor.h"

//-------------------------------------------------------------------------------------------------------------------
//  函数名称     motor_init
//  参数说明     void
//  返回值       void
//  备注信息     初始化MOTOR1/MOTOR2的PWM引脚，PWM频率17kHz，占空比初始值为0
//              每个电机使用2个PWM引脚
//              MOTOR1: ATOM1 CH1(P33.9) + CH2(P33.11)
//              MOTOR2: ATOM0 CH3(P14.2) + CH2(P14.3)
//-------------------------------------------------------------------------------------------------------------------
void motor_init(void)
{
    // MOTOR1: 前臂高臂(S1) + 前臂低臂(S2) + 后臂高臂(S3) + 后臂低臂(S4)
    pwm_init(MOTOR1_PWM_PIN1, 10000, 0);    // P33.9 - 前臂高臂 (S1)
    pwm_init(MOTOR1_PWM_PIN2, 10000, 0);    // P33.11 - 后臂高臂 (S3)
    //gpio_init(P33_11, GPO, 1, GPO_PUSH_PULL);
    //gpio_set_dir(P33_11, GPO, GPO_PUSH_PULL);
    // MOTOR2: 前臂高臂(S1) + 前臂低臂(S2) + 后臂高臂(S3) + 后臂低臂(S4)
    pwm_init(MOTOR2_PWM_PIN1, 10000, 0);    // P14.2 - 前臂高臂 (S1)
    pwm_init(MOTOR2_PWM_PIN2, 10000, 0);    // P14.3 - 后臂高臂 (S3)

    // 初始为刹车模式，全桥低侧导通
    motor_control(motor_RB ,0);
    motor_control(motor_LB ,0);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名称     motor_control
//  参数说明     motor:     电机通道枚举 (motor_LB / motor_RB)
//              dir:       方向 (MOTOR_DIR_FORWARD / BRAKE / REVERSE)
//              percent:   PWM占空比百分比 (0~100)
//  返回值       void
//  备注信息     以STM32 TIM1 H桥逻辑实现：
//
//              全桥结构：
//                S1 (前臂高臂)    S2 (前臂低臂)    S3 (后臂高臂)    S4 (后臂低臂)
//              PWM自动互补S1/S2、S3/S4
//
//              正转模式 dir = FORWARD
//                S1/S2 互补PWM，S1 = duty, S2 = complement
//                S3 = 0(关断)，S4 = 1
//                电流: S1(PWM)→电机→S4(常通)→S2(续流) S1/S4为正转方向
//
//              反转模式 dir = REVERSE
//                S3/S4 互补PWM，S3 = duty, S4 = complement
//                S1 = 0(关断)，S2 = 1
//                电流: S2(常通)→电机→S3(PWM)→S4(续流) S2/S3为反向电流
//
//              刹车模式 dir = BRAKE
//                S1 = 0, S2 = 1, S3 = 0, S4 = 1
//                电机两端接地，形成短路回路，快速制动
//
//              PWM占空比范围：
//                最小 0% (0)
//                最大 100% (PWM_DUTY_MAX)
//-------------------------------------------------------------------------------------------------------------------
void my_motor_control(MOTOR_TYPE motor, MotorDir dir, uint8 percent)
{
    uint32 duty;


    if(percent > 100) percent = 100;
    duty = (uint32)percent * PWM_DUTY_MAX / 100U;

    if( duty == 0)
    {
        // 刹车模式：S1=0, S2=1, S3=0, S4=1
        // 电机两端接地，形成短路回路，快速制动
        switch(motor)
        {
            case motor_LB:
                pwm_set_duty(MOTOR1_PWM_PIN1, 0);
                pwm_set_duty(MOTOR1_PWM_PIN2, 0);

                break;

            case motor_RB:
                pwm_set_duty(MOTOR2_PWM_PIN1, 0);         // P14.2 S1=0
                pwm_set_duty(MOTOR2_PWM_PIN2, 0);         // P14.3 S3=0
                break;
        }
    }
    else if(dir == MOTOR_DIR_FORWARD)
    {
        // 正转模式：前臂(S1/S2)互补PWM，后臂(S3/S4)固定
        // S1 = PWM, S2 = complement, S3 = 0, S4 = 1
        // 电流: S1/S4 导通 S1(PWM)/S2(续流)
        switch(motor)
        {
            case motor_LB:
                pwm_set_duty(MOTOR1_PWM_PIN1, duty);      // P33.9 S1 PWM
                pwm_set_duty(MOTOR1_PWM_PIN2, 0);         // P33.11 S3 = 0, 则 S4 = 1
                //gpio_set_level(P33_11,1);
                //gpio_low(P33_11);
                break;

            case motor_RB:
                pwm_set_duty(MOTOR2_PWM_PIN1, duty);      // P14.2 S1 PWM
                pwm_set_duty(MOTOR2_PWM_PIN2, 0);         // P14.3 S3 = 0, 则 S4 = 1
                break;
        }
    }
    else if(dir == MOTOR_DIR_REVERSE)
    {
        // 反转模式：后臂(S3/S4)互补PWM，前臂(S1/S2)固定
        // S1 = 0, S2 = 1, S3 = PWM, S4 = complement
        // 电流: S2(常通)/S3(PWM) 形成反向电流
        switch(motor)
        {
            case motor_LB:
                pwm_set_duty(MOTOR1_PWM_PIN1, 0);         // P33.9 S1 = 0, 则 S2 = 1
                pwm_set_duty(MOTOR1_PWM_PIN2, duty);      // P33.11 S3 PWM
                break;

            case motor_RB:
                pwm_set_duty(MOTOR2_PWM_PIN1, 0);         // P14.2 S1 = 0, 则 S2 = 1
                pwm_set_duty(MOTOR2_PWM_PIN2, duty);      // P14.3 S3 PWM
                break;
        }
    }
}


void motor_control(MOTOR_TYPE motor, int16 duty)
{
    int8 dir = (duty > 0) ? 1 : -1;
    int16 abs_duty = abs(duty);

    if(duty == 0)
    {
        // 刹车模式：S1=0, S2=1, S3=0, S4=1
        // 电机两端接地，形成短路回路，快速制动
        switch(motor)
        {
            case motor_LB:
                pwm_set_duty(MOTOR1_PWM_PIN1, 0);
                pwm_set_duty(MOTOR1_PWM_PIN2, 0);

                break;

            case motor_RB:
                pwm_set_duty(MOTOR2_PWM_PIN1, 0);         // P14.2 S1=0
                pwm_set_duty(MOTOR2_PWM_PIN2, 0);         // P14.3 S3=0
                break;
        }
    }
    else if(dir == MOTOR_DIR_FORWARD)
    {
        // 正转模式：前臂(S1/S2)互补PWM，后臂(S3/S4)固定
        // S1 = PWM, S2 = complement, S3 = 0, S4 = 1
        // 电流: S1/S4 导通 S1(PWM)/S2(续流)
        switch(motor)
        {
            case motor_LB:
                pwm_set_duty(MOTOR1_PWM_PIN1, abs_duty);      // P33.9 S1 PWM
                pwm_set_duty(MOTOR1_PWM_PIN2, 0);         // P33.11 S3 = 0, 则 S4 = 1
                //gpio_set_level(P33_11,1);
                //gpio_low(P33_11);
                break;

            case motor_RB:
                pwm_set_duty(MOTOR2_PWM_PIN1, abs_duty);      // P14.2 S1 PWM
                pwm_set_duty(MOTOR2_PWM_PIN2, 0);         // P14.3 S3 = 0, 则 S4 = 1
                break;
        }
    }
    else if(dir == MOTOR_DIR_REVERSE)
    {
        // 反转模式：后臂(S3/S4)互补PWM，前臂(S1/S2)固定
        // S1 = 0, S2 = 1, S3 = PWM, S4 = complement
        // 电流: S2(常通)/S3(PWM) 形成反向电流
        switch(motor)
        {
            case motor_LB:
                pwm_set_duty(MOTOR1_PWM_PIN1, 0);         // P33.9 S1 = 0, 则 S2 = 1
                pwm_set_duty(MOTOR1_PWM_PIN2, abs_duty);      // P33.11 S3 PWM
                break;

            case motor_RB:
                pwm_set_duty(MOTOR2_PWM_PIN1, 0);         // P14.2 S1 = 0, 则 S2 = 1
                pwm_set_duty(MOTOR2_PWM_PIN2, abs_duty);      // P14.3 S3 PWM
                break;
        }
    }
}
