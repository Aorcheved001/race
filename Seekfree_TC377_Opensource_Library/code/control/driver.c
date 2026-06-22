/*
 * driver.c
 * 踏板驱动 - 加速踏板(ADC)与减速踏板(GPIO)
 *
 * Created on: 2026-04-25
 */

#include "driver.h"
#include "MOTOR.h"
#include "yaokong.h"
#include <stdio.h>

//==============================================================================
//                              宏定义
//==============================================================================

/* 加速踏板参数 */
#define PEDAL_ADC_DEADZONE_VOLTAGE     1.2f    /* 死区电压 (V) - 低于此值不加速 */
#define PEDAL_ADC_MAX_VOLTAGE          1.88f    /* 最大电压 (V) - 踏板踩到底约1.9V */
#define PEDAL_MAX_DUTY_PERCENT         80      /* 最大占空比 (%) */

/* ADC 分辨率相关 (12位 ADC) */
#define ADC_RESOLUTION_12BIT           4095    /* 12位 ADC 最大值 */
#define ADC_REF_VOLTAGE                5.0f    /* ADC 参考电压 (V) */

/* 电压转 ADC 值 */
#define VOLTAGE_TO_ADC(volt)           ((uint16)((volt) / ADC_REF_VOLTAGE * ADC_RESOLUTION_12BIT))

//==============================================================================
//                              私有变量
//==============================================================================

/* 加速踏板 ADC 通道 (A12 = ADC1_CH4_A12) */
static adc_channel_enum accel_pedal_adc_channel = ADC1_CH4_A12;

/* 减速踏板 GPIO 引脚 (P22_0) */
static gpio_pin_enum brake_pedal_gpio_pin = P22_0;

//==============================================================================
//                              函数实现
//==============================================================================

/**
 * @brief 初始化踏板驱动
 * @param void
 * @retval void
 * @note 初始化加速踏板 ADC 和减速踏板 GPIO
 */
void pedal_init(void)
{
    /* 初始化加速踏板 ADC (12位分辨率) */
    adc_init(accel_pedal_adc_channel, ADC_12BIT);

    /* 初始化减速踏板 GPIO (上拉输入，未踩下时高电平) */
    gpio_init(brake_pedal_gpio_pin, GPI, GPIO_HIGH, GPI_PULL_UP);
}

/**
 * @brief 读取加速踏板 ADC 值
 * @param void
 * @retval uint16 ADC 原始值 (0~4095)
 */
uint16 pedal_read_accel_adc(void)
{
    return adc_convert(accel_pedal_adc_channel);
}

/**
 * @brief 读取加速踏板电压
 * @param void
 * @retval float 电压值 (0~5.0V)
 */
float pedal_read_accel_voltage(void)
{
    uint16 adc_value = adc_convert(accel_pedal_adc_channel);
    return (float)adc_value / ADC_RESOLUTION_12BIT * ADC_REF_VOLTAGE;
}

/**
 * @brief 读取减速踏板状态
 * @param void
 * @retval uint8 1: 踩下(低电平), 0: 未踩下(高电平)
 */
uint8 pedal_read_brake_state(void)
{
    /* 减速踏板: 未踩下=高电平, 踩下=低电平 */
    /* 返回 1 表示踩下, 0 表示未踩下 */
    return (gpio_get_level(brake_pedal_gpio_pin) == GPIO_LOW) ? 1 : 0;
}

/**
 * @brief 根据加速踏板计算 PWM 占空比
 * @param void
 * @retval uint8 占空比百分比 (0~80%)
 * @note 死区 1.2V 以下输出 0，1.2V~1.88V 线性映射到 0~80%
 */
uint8 pedal_calc_accel_duty(void)
{
    uint16 adc_value = adc_convert(accel_pedal_adc_channel);
    uint16 deadzone_adc = VOLTAGE_TO_ADC(PEDAL_ADC_DEADZONE_VOLTAGE);  /* ~1229 */
    uint16 max_adc = VOLTAGE_TO_ADC(PEDAL_ADC_MAX_VOLTAGE);            /* 4095 */

    uint8 duty_percent = 0;

    /* 死区处理: 低于 1.2V 输出 0 */
    if (adc_value <= deadzone_adc)
    {
        duty_percent = 0;
    }
    else
    {
        /* 线性映射: 1.2V~1.88V -> 0~80% */
        uint16 effective_adc = adc_value - deadzone_adc;
        uint16 effective_range = max_adc - deadzone_adc;  /* ~2866 */

        /* 计算占空比百分比 */
        duty_percent = (uint8)((uint32)effective_adc * PEDAL_MAX_DUTY_PERCENT / effective_range);

        /* 限幅 */
        if (duty_percent > PEDAL_MAX_DUTY_PERCENT)
        {
            duty_percent = PEDAL_MAX_DUTY_PERCENT;
        }
    }

    return duty_percent;
}

/**
 * @brief 踏板控制处理函数 (主循环调用)
 * @param void
 * @retval void
 * @note 根据踏板状态控制电机:
 *       - 减速踏板踩下: 刹车 (两个下管导通)
 *       - 减速踏板未踩下: 根据加速踏板线性控制电机
 */
void pedal_process(void)
{
    /* 遥控激活时，踏板不控制电机，防止冲突 */
    if (yaokong_active)
    {
        return;
    }

    uint8 brake_pressed = pedal_read_brake_state();
    uint8 accel_duty = pedal_calc_accel_duty();

    if (brake_pressed)
    {
        /* 减速踏板踩下 - 刹车模式 */
        Motor_Ctrl(MOTOR_L, 0);
        Motor_Ctrl(MOTOR_R, 0);
    }
    else
    {
        /* 减速踏板未踩下 - 根据加速踏板控制电机 */
        if (accel_duty > 0)
        {
            /* 有加速输入 - 正转前进 */
            int32 pwm_val = (int32)accel_duty * 100;
            Motor_Ctrl(MOTOR_L, pwm_val);
            Motor_Ctrl(MOTOR_R, pwm_val);
        }
        else
        {
            /* 无加速输入 - 刹车 */
            Motor_Ctrl(MOTOR_L, 0);
            Motor_Ctrl(MOTOR_R, 0);
        }
    }
}

/**
 * @brief 获取加速踏板最大占空比限制
 * @param void
 * @retval uint8 最大占空比百分比
 */
uint8 pedal_get_max_duty_limit(void)
{
    return PEDAL_MAX_DUTY_PERCENT;
}

/**
 * @brief 发送踏板数据到上位机
 * @param void
 * @retval void
 * @note 发送格式: pedal:brake_state,accel_adc,accel_voltage,accel_duty\r\n
 *       - brake_state: 0=未踩下, 1=踩下
 *       - accel_adc: ADC原始值 (0~4095)
 *       - accel_voltage: 电压值 (0~5.0V)
 *       - accel_duty: 计算的占空比 (0~80%)
 */
void pedal_send_to_host(void)
{
    uint8 brake_state = pedal_read_brake_state();
    uint16 accel_adc = pedal_read_accel_adc();
    float accel_voltage = pedal_read_accel_voltage();
    uint8 accel_duty = pedal_calc_accel_duty();

    printf("pedal:%d,%d,%.2f,%d\r\n",
           brake_state, accel_adc, accel_voltage, accel_duty);
}
