/*
 * yaokong.c
 */

 //-------------------------------------------头文件引用------------------------------------------------------------
#include "zf_common_headfile.h"
#include "MOTOR.h"
#include "PID.h"
#include "steering_control.h"
#include "track.h"

 //-------------------------------------------全局变量定义------------------------------------------------------------
bool flag_stop = 0;                                                       // 停止标志位，1表示停止，0表示运行
int16 speed_yk = 15;                                                    // 遥控速度指令，单位：脉冲/4ms
uint8 yaokong_active = 0;                                                 // 遥控器激活标志，1表示激活中

 //-------------------------------------------内部变量定义------------------------------------------------------------
static uint8 s_control_enabled = 1u;                                      // 控制使能标志，1表示允许控制

 //-------------------------------------------函数定义------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      初始化遥控
 ////  @param      speed_mps       初始速度（m/s）
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void yaokong_init(int16 max_pulse)
{
    speed_yk = (max_pulse > 0) ? max_pulse : (int16)(-max_pulse);       // 设置脉冲绝对值
    flag_stop = 0;                                                        // 清除停止标志
    s_control_enabled = 1u;                                               // 使能控制
    Yao.flag_motor_start_user = 1;                                         // 使能电机（速度环）
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      设置遥控是否直接或间接路执行指令
 ////  @param      enabled         1表示激活，0表示禁止
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void yaokong_set_control_enabled(uint8 enabled)
{
    s_control_enabled = enabled ? 1u : 0u;                                // 设置控制使能标志
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      处理遥控数据
 ////  @param      void
 ////  @return     uint8           1表示处理成功，0表示未收到
 ////  @note       处理遥控器数据并控制电机和转向
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

    flag_stop = (lora3a22_uart_transfer.key[0] == 1u) ? 1u : 0u;          // 更新停止按键

    int16 target_speed = 0;
    uint8 has_input = 0u;

    if (!flag_stop)
    {
        int16 js2 = lora3a22_uart_transfer.joystick[2];                   // 油门摇杆值
        if (js2 > 300)
        {
            // 前进：300~1500 线性映射到 0~speed_yk（整型，避免小数点）
            int32 temp = (int32)speed_yk * (js2 - 300) / 1200;
            if (temp > speed_yk) temp = speed_yk;
            target_speed = (int16)temp;
            has_input = 1u;
        }
        else if (js2 < -300)
        {
            // 后退：-300~-1500 线性映射到 0~-speed_yk（整型，避免小数点）
            int32 temp = (int32)speed_yk * (-js2 - 300) / 1200;
            if (temp > speed_yk) temp = speed_yk;
            target_speed = (int16)(-temp);
            has_input = 1u;
        }
    }

    float steer_offset_deg = 0.0f;
    int16 steer_raw = lora3a22_uart_transfer.joystick[3];
    if ((steer_raw > 300) || (steer_raw < -300))
    {
        steer_offset_deg = -0.03f * (float)steer_raw;                     // 计算转向偏移
        steer_offset_deg = func_limit(steer_offset_deg, VEHICLE_MAX_STEER_ANGLE_DEG); // 限幅转向角
        has_input = 1u;
    }


    if (has_input)
    {
        wheel_pid_set_target_speed(target_speed);                  // 设置目标速度（直接传脉冲值，int16）
        steering_set_target(steer_offset_deg);                            // 设置转向角度
    }
    else
    {
        // 无有效遥控数据时，转向回归原点
        steering_set_target(0.0f);
    }

    // 管理电机使能标志：有遥控输入且未停止时使能
    if (!flag_stop && has_input)
    {
        Yao.flag_motor_start_user = 1;
    }
    else if (flag_stop)
    {
        Yao.flag_motor_start_user = 0;
    }
    else if (!has_input && track_follow_flag == 0 && track_follow_reverse_flag == 0)
    {
        Yao.flag_motor_start_user = 0;
    }

    yaokong_active = has_input;                                           // 更新激活标志
    lora3a22_finsh_flag = 0u;                                             // 清除数据完成标志
    return 1u;
}


