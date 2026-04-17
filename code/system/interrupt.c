/*
 * imu660.c
 *
 *  Created on: 2024年1月29日
 *      Author: 牢威
 *第二次修改 2025年3月21日   Daydreaer
 *
 */


 //-------------------------------------------头文件声明区------------------------------------------------------------

 #include "zf_common_headfile.h"

 //-------------------------------------------内部变量声明区------------------------------------------------------------

static volatile uint16 s_pending_1ms = 0;                           // 1ms 中断标志位
static volatile uint16 s_pending_2ms = 0;                           // 2ms 中断标志位
static volatile uint16 s_pending_4ms = 0;                           // 4ms 中断标志位
static volatile uint16 s_pending_8ms = 0;                           // 8ms 中断标志位
static volatile uint16 s_pending_16ms = 0;                          // 16ms 中断标志位
static volatile uint16 s_pending_40ms = 0;                          // 40ms 中断标志位
static volatile uint16 s_pending_100ms = 0;                         // 100ms 中断标志位

volatile uint8 g_key_scan_flag = 0;                                 // 按键扫描标志位
volatile uint8 yaokong_flag = 0;                                    // 按键扫描标志位


//static INS_Input s_ins_input;                                       // INS 输入结构体
static uint8 s_ins_counter = 0;                                     // INS 输入计数器

 //-------------------------------------------函数定义区------------------------------------------------------------
void Interrupt_1ms(void)   { if (s_pending_1ms   < 500u) s_pending_1ms++;   }             // 1ms 中断服务函数
void Interrupt_2ms(void)   { if (s_pending_2ms   < 500u) s_pending_2ms++;   }             // 2ms 中断服务函数
void Interrupt_4ms(void)   { if (s_pending_4ms   < 500u) s_pending_4ms++;   }             // 4ms 中断服务函数
void Interrupt_8ms(void)   { if (s_pending_8ms   < 500u) s_pending_8ms++;   }             // 8ms 中断服务函数
void Interrupt_16ms(void)  { if (s_pending_16ms  < 500u) s_pending_16ms++;  }             // 16ms 中断服务函数
void Interrupt_40ms(void)  { if (s_pending_40ms  < 500u) s_pending_40ms++;  }             // 40ms 中断服务函数
void Interrupt_100ms(void) { if (s_pending_100ms < 500u) s_pending_100ms++; }             // 100ms 中断服务函数


////-------------------------------------------------------------------------------------------------------------------
////  @brief      2ms任务函数
////  @param      无
////-------------------------------------------------------------------------------------------------------------------
static void run_2ms_tasks(void)
{
//    yaokong_flag++;
//    if (yaokong_flag == 5)
//    {
//        lora3a22_response_time++;
//        if (lora3a22_response_time > 500 / 10)   // 500ms 没有接受到数据判断位发送端异常
//        {
//            lora3a22_state_flag = 0;             // 遥控器状态位清零
//            lora3a22_response_time = 0;
//        }
//        yaokong_flag %= 5;
//    }
}


 ////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      4ms任务函数
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
static void run_4ms_tasks(void)
{
//
//    date_handle(&imu_date);                                      // 处理IMU数据
//
//    encoder_layer_update();                                      // 编码器数据更新
//
//    const EncoderLayerState* enc = encoder_layer_get_state();    // 获取编码器状态指针
//
//    wheel_pid_update(enc, 0.004f);                               // 更新PID控制器
//
//    //更新ins输入结构体数据
//    s_ins_input.wheel_speed = enc->speed_average_mps;            // 平均速度
//    s_ins_input.steer_angle = servo_get_angle_rad_relative();    // 相对角度
//    s_ins_input.gyro_z      = imu660.data_Ripen.gyro_z;          // 陀螺仪Z轴数据
//
//    s_ins_counter = 0;                                            // 重置INS输入计数器
//    ins_ekf_predict_step_with_ins_input(&g_ins_state, &s_ins_input, 0.004f); // INS 状态预测

}
 ////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      8ms任务函数
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
static void run_8ms_tasks(void)
{
//    yaokong_data_deal();                                        // 遥控数据处理（更新速度/转向目标）
//
//    g_key_scan_flag = 1;                                        // 按键扫描标志位
//
//    menu();                                                     // 菜单函数
//
//    ins_ekf_update_step(&g_ins_state, NULL, NULL, 0);           // INS 状态更新

}


 ////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      40ms任务函数
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
static void run_40ms_tasks(void)
{
//    float x_gps = 0.0f;                                         // GPS X轴坐标
//
//    float y_gps = 0.0f;                                         // GPS Y轴坐标
//
//    if (gnss_get_position_xy(&x_gps, &y_gps))                   // 获取GPS位置
//    {
//        ins_ekf_update_with_gps_position(&g_ins_state,          // gps位置观测更新
//                                         x_gps, y_gps,
//                                         GPS_VAR_DEFAULT, GPS_VAR_DEFAULT);
//    }
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      100ms任务函数
////  @param      无
////-------------------------------------------------------------------------------------------------------------------

// 声明外部函数

static void run_100ms_tasks(void)
{
    /**科目二灯光计时**/
//    if(project_task.task2_state[speech_ok] == ON
//    && project_task.task2_state[gps_num_state] == ON
//    && project_task.task2_state[go_num_state] == ON
//    && Led_time_state == 1)
//    {
//        if(Led_time < 100)  Led_time++;
//        if(Led_time<=50 && Led_time%10 == 0)
//        {
//            switch(brightness_state)
//            {
//                case 0:
//                    dot_matrix_screen_set_brightness(5000);
//                    brightness_state = 1;
//                    break;
//                case 1:
//                    if(transimit_cmd_led = CMD_LEFT_TURN ||
//                       transimit_cmd_led = CMD_RIGHT_TURN ||
//                       transimit_cmd_led = CMD_HAZARD_LAMP )
//                    {
//                        dot_matrix_screen_set_brightness(0);
//                    }
//                    brightness_state = 0;
//                    break;
//            }
//        }
//    }
}

 ////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      轮询执行中断任务函数
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
void InterruptTasks_Poll(void)
{
    while (s_pending_4ms)
    {
        s_pending_4ms--;
        run_4ms_tasks();
    }
    while (s_pending_2ms)
    {
        s_pending_2ms--;
        run_2ms_tasks();
    }

    while (s_pending_8ms)
    {
        s_pending_8ms--;
        run_8ms_tasks();
    }

    while (s_pending_100ms)
    {
        s_pending_100ms--;
        run_100ms_tasks();
    }

    while (s_pending_40ms)
    {
        s_pending_40ms--;
        run_40ms_tasks();
    }

    s_pending_1ms  = 0;
    s_pending_16ms = 0;
}
