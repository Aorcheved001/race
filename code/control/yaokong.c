/*
 * yaokong.c
 */

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"
#include "motor.h"
#include "PID.h"

 //-------------------------------------------全局变量区------------------------------------------------------------
bool flag_stop = 0;                                                       // 控制标志位，1表示停止，0表示运行
float speed_yk = 0.8f;                                                    // 遥控速度指令，单位：m/s
uint8 yaokong_active = 0;                                                 // 遥控器激活标志，1表示有输入

 //-------------------------------------------内部变量区------------------------------------------------------------
static uint8 s_control_enabled = 1u;                                      // 控制使能标志，1表示允许控制

 //-------------------------------------------函数定义区------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      初始化遥控
 ////  @param      speed_mps       初始速度（m/s）
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void yaokong_init(float speed_mps)
{
    speed_yk = fabsf(speed_mps);                                          // 设置速度绝对值
    flag_stop = 0;                                                        // 清零停止标志
    s_control_enabled = 1u;                                               // 使能控制
    wheel_pid_enable(1);                                                  // 使能轮速PID
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      设置遥控是否允许直接下发执行器指令
 ////  @param      enabled         1表示允许，0表示禁止
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void yaokong_set_control_enabled(uint8 enabled)
{
    s_control_enabled = enabled ? 1u : 0u;                                // 设置控制使能标志
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      处理遥控数据
 ////  @param      void
 ////  @return     uint8           1表示处理成功，0表示无数据
 ////  @note       解析遥控器数据并控制电机和舵机
 ////-------------------------------------------------------------------------------------------------------------------
uint8 yaokong_data_deal(void)
{
    if (lora3a22_state_flag != 1u)
    {
        return 0u;
    }

    if (lora3a22_finsh_flag != 1u)
    {
        return 0u;
    }

    flag_stop = (lora3a22_uart_transfer.key[0] == 1u) ? 1u : 0u;          // 解析停止按键

    float target_speed = 0.0f;
    uint8 has_input = 0u;

    if (!flag_stop)
    {
        if (lora3a22_uart_transfer.joystick[2] > 300)
        {
            target_speed = speed_yk;                                      // 前进
            has_input = 1u;
        }
        else if (lora3a22_uart_transfer.joystick[2] < -300)
        {
            target_speed = -speed_yk;                                     // 后退
            has_input = 1u;
        }
    }

    float steer_offset_deg = 0.0f;
    int16 steer_raw = lora3a22_uart_transfer.joystick[3];
    if ((steer_raw > 300) || (steer_raw < -300))
    {
        steer_offset_deg = 0.008f * (float)steer_raw;                     // 计算转向偏移
        steer_offset_deg = func_limit(steer_offset_deg, 35.0f);           // 限幅转向角
        has_input = 1u;
    }

    if (lora3a22_uart_transfer.key[1] == 1u)
    {
        steer_offset_deg = 0.0f;                                          // 按键1按下时回正
    }

    if (s_control_enabled)
    {
        wheel_pid_set_target_speed(target_speed);                         // 设置目标速度
        servo_set_angle(90.0f + steer_offset_deg);                        // 设置舵机角度
    }

    yaokong_active = has_input;                                           // 更新激活标志
    lora3a22_finsh_flag = 0u;                                             // 清除接收完成标志
    return 1u;
}
