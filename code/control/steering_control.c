/*
 * steering_control.c
 *
 *  Created on: 2026年4月13日
 *      Author: A
 *  Description: 转向控制 —— 绝对编码器 + PD + 陀螺仪前馈
 *               HOME 为固定机械原位，上电不变
 *               左负右正，PID 按 (目标 - 当前) 误差驱 PWM
 */

#include "zf_common_headfile.h"
#include "steering_control.h" 

//-------------------------------------------内部变量------------------------------------------------------------
static float g_target_angle = 0.0f;             // 目标角度（度）
static float g_last_error  = 0.0f;             // 上一次误差，用于微分

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取当前转向角度（相对机械原位）
//  @return     角度（度），0=原位，左负右正
//  @note       raw增大 = 左转（负角度），raw减小 = 右转（正角度）
//              左右转的raw变化幅度不同，使用不对称的比例因子
//-------------------------------------------------------------------------------------------------------------------
float steering_get_current_angle(void)
{
    int16 raw = absolute_encoder_get_location();
    int32 diff = (int32)raw - (int32)STEER_HOME_RAW;

    // 跨越零点：超过半圈视为越界，反向修补一圈
    if(diff > (STEER_ENCODER_RES / 2))
        diff -= STEER_ENCODER_RES;
    else if(diff < -(STEER_ENCODER_RES / 2))
        diff += STEER_ENCODER_RES;

    // 不对称映射：
    // 左转：diff>0, raw从2139到3200(+1061)对应0到-45°
    // 右转：diff<0, raw从2139到875(-1264)对应0到+45°
    float angle;
    if(diff >= 0)
    {
        // 左转（负角度）
        angle = -(float)diff * (STEER_MAX_ANGLE / (float)(STEER_LEFT_MAX_RAW - STEER_HOME_RAW));
    }
    else
    {
        // 右转（正角度）
        angle = -(float)diff * (STEER_MAX_ANGLE / (float)(STEER_HOME_RAW - STEER_RIGHT_MAX_RAW));
    }
    
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      转向模块初始化
//-------------------------------------------------------------------------------------------------------------------
void steering_init(void)
{
    absolute_encoder_init();
    system_delay_ms(10);

    pwm_init(STEER_MOTOR_PWM_PIN1, 10000, 0);
    pwm_init(STEER_MOTOR_PWM_PIN2, 10000, 0);

    g_target_angle = 0.0f;
    g_last_error  = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置目标角度
//  @param      angle_deg   目标角度（度），左负右正
//-------------------------------------------------------------------------------------------------------------------
void steering_set_target(float angle_deg)
{
    g_target_angle = func_limit(angle_deg, STEER_MAX_ANGLE);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      转向控制（每个控制周期调用一次）
//  @note       PD + 陀螺仪前馈，误差 = 目标 - 当前
//-------------------------------------------------------------------------------------------------------------------
void steering_control(void)
{
    // 1. 获取当前角度
    float current = steering_get_current_angle();

    // 2. 误差 = 目标 - 当前
    float error = g_target_angle - current;

    // 3. PD + 陀螺仪前馈
    float derivative = (error - g_last_error) / STEER_CONTROL_DT;
    float output = STEER_KP * error
                 + STEER_KD * derivative
                 + STEER_GKD * imu660.data_Ripen.gyro_z;

    // 4. 限幅
    output = func_limit(output, (float)STEER_PWM_MAX);

    // 5. 双 PWM 驱动
    // output > 0: 需要右转（角度往正方向），raw应该减小 → PIN2输出
    // output < 0: 需要左转（角度往负方向），raw应该增大 → PIN1输出
    int16 abs_duty = (int16)fabsf(output);
    if(output >= 0.0f)
    {
        pwm_set_duty(STEER_MOTOR_PWM_PIN1, 0);
        pwm_set_duty(STEER_MOTOR_PWM_PIN2, (uint16)abs_duty);
    }
    else
    {
        pwm_set_duty(STEER_MOTOR_PWM_PIN1, (uint16)abs_duty);
        pwm_set_duty(STEER_MOTOR_PWM_PIN2, 0);
    }

    // 6. 保存误差
    g_last_error = error;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送当前转向角度、目标角度与原始编码器值到上位机
//-------------------------------------------------------------------------------------------------------------------
void steering_send_angle_to_host(void)
{
    int16 raw = absolute_encoder_get_location();
    printf("steer:%.2f,target:%.2f,raw:%d\r\n", steering_get_current_angle(), g_target_angle, raw);
}
