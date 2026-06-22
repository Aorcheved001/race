/*
 * driver.h
 * 踏板驱动 - 加速踏板(ADC)与减速踏板(GPIO)
 *
 * Created on: 2026-04-25
 */

#ifndef CODE_CONTROL_DRIVER_H_
#define CODE_CONTROL_DRIVER_H_

#include "zf_common_headfile.h"

//==============================================================================
//                              函数声明
//==============================================================================

/**
 * @brief 初始化踏板驱动
 */
void pedal_init(void);

/**
 * @brief 读取加速踏板 ADC 原始值
 * @return ADC 原始值 (0~4095)
 */
uint16 pedal_read_accel_adc(void);

/**
 * @brief 读取加速踏板电压
 * @return 电压值 (0~5.0V)
 */
float pedal_read_accel_voltage(void);

/**
 * @brief 读取减速踏板状态
 * @return 1: 踩下(低电平), 0: 未踩下(高电平)
 */
uint8 pedal_read_brake_state(void);

/**
 * @brief 根据加速踏板计算 PWM 占空比
 * @return 占空比百分比 (0~60%)
 * @note 死区 1.5V 以下输出 0，1.5V~5V 线性映射到 0~60%
 */
uint8 pedal_calc_accel_duty(void);

/**
 * @brief 踏板控制处理函数 (主循环调用)
 * @note 根据踏板状态控制电机
 */
void pedal_process(void);

/**
 * @brief 获取加速踏板最大占空比限制
 * @return 最大占空比百分比
 */
uint8 pedal_get_max_duty_limit(void);

/**
 * @brief 发送踏板数据到上位机
 * @note 发送格式: pedal:brake_state,accel_adc,accel_voltage,accel_duty\r\n
 *       - brake_state: 0=未踩下, 1=踩下
 *       - accel_adc: ADC原始值 (0~4095)
 *       - accel_voltage: 电压值 (0~5.0V)
 *       - accel_duty: 计算的占空比 (0~60%)
 */
void pedal_send_to_host(void);

#endif /* CODE_CONTROL_DRIVER_H_ */
