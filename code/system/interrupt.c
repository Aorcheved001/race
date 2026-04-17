/*
 * interrupt.c
 */

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------内部变量区------------------------------------------------------------
static volatile uint16 s_pending_1ms = 0;                                 // 1ms任务挂起计数
static volatile uint16 s_pending_2ms = 0;                                 // 2ms任务挂起计数
static volatile uint16 s_pending_4ms = 0;                                 // 4ms任务挂起计数
static volatile uint16 s_pending_8ms = 0;                                 // 8ms任务挂起计数
static volatile uint16 s_pending_16ms = 0;                                // 16ms任务挂起计数
static volatile uint16 s_pending_40ms = 0;                                // 40ms任务挂起计数

volatile uint8 g_key_scan_flag = 0;                                       // 按键扫描标志
static INS_Input s_ins_input = {0};                                       // INS输入数据结构体

 //-------------------------------------------函数声明区------------------------------------------------------------
static float normalize_angle_rad_local(float angle)
{
    while (angle > INS_PI) angle -= 2.0f * INS_PI;
    while (angle < -INS_PI) angle += 2.0f * INS_PI;
    return angle;
}

static uint8 get_mag_yaw_measurement(float *yaw_rad)
{
    float mag_x = imu660.data_Ripen.mag_x;
    float mag_y = imu660.data_Ripen.mag_y;

    if(yaw_rad == NULL)
    {
        return 0u;
    }

    if((fabsf(mag_x) + fabsf(mag_y)) < 1e-4f)
    {
        return 0u;
    }

    *yaw_rad = normalize_angle_rad_local(-atan2f(mag_y, mag_x));
    return 1u;
}

void Interrupt_1ms(void)  { if (s_pending_1ms  < 500u) s_pending_1ms++; }
void Interrupt_2ms(void)  { if (s_pending_2ms  < 500u) s_pending_2ms++; }
void Interrupt_4ms(void)  { if (s_pending_4ms  < 500u) s_pending_4ms++; }
void Interrupt_8ms(void)  { if (s_pending_8ms  < 500u) s_pending_8ms++; }
void Interrupt_16ms(void) { if (s_pending_16ms < 500u) s_pending_16ms++; }
void Interrupt_40ms(void) { if (s_pending_40ms < 500u) s_pending_40ms++; }

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      4ms周期任务
 ////  @param      void
 ////  @return     void
 ////  @note       执行IMU数据处理、编码器更新、PID控制、INS更新、轨迹处理
 ////-------------------------------------------------------------------------------------------------------------------
static void run_4ms_tasks(void)
{
    date_handle(&imu_date);                                               // IMU数据处理
    encoder_layer_update();                                               // 编码器层更新

    const EncoderLayerState* enc = encoder_layer_get_state();             // 获取编码器状态
    wheel_pid_update(enc, 0.004f);                                        // 轮速PID更新

    s_ins_input.v_mps = enc->speed_average_mps;                           // 设置INS输入速度
    s_ins_input.gyro_z_rad_s = imu660.data_Ripen.gyro_z;                  // 设置INS输入角速度
    s_ins_input.mag_valid = get_mag_yaw_measurement(&s_ins_input.mag_yaw_rad);   // 获取磁航向

    float steer_rad = servo_get_angle_rad_relative();                     // 获取转向角
    s_ins_input.omega_rad_s = s_ins_input.v_mps * tanf(steer_rad) / INS_WHEELBASE_M;   // 计算角速度

    Ins_update(&s_ins_input, 0.004f);                                     // INS更新
    track_proc();                                                         // 轨迹处理
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      8ms周期任务
 ////  @param      void
 ////  @return     void
 ////  @note       执行按键扫描、遥控处理、INS导航任务
 ////-------------------------------------------------------------------------------------------------------------------
static void run_8ms_tasks(void)
{
    g_key_scan_flag = 1;                                                  // 设置按键扫描标志

    yaokong_set_control_enabled((track_follow_flag == 0u) ? 1u : 0u);     // 设置遥控使能
    yaokong_data_deal();                                                  // 遥控数据处理

    key_scanner();
    INS_NavigationTask();                                                 // INS导航任务
}


static void send_position_to_host(void)
{
    printf("(%.2f,%.2f)\r\n", INS.cod_RealTime.x, INS.cod_RealTime.y);
}

static void send_pos_to_host(void)
{
    float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
    Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

    // 将弧度转换为角度方便上位机查看
    yaw_gyro *= 57.29578f;
    yaw_mag_rel *= 57.29578f;
    yaw_ekf *= 57.29578f;

    char send_buf[64];

    // 分三次独立发送数据
    sprintf(send_buf, "imu_yaw:%.2f,%.2f,%.2f\r\n", yaw_gyro,yaw_mag_rel,yaw_ekf);
    wireless_uart_send_string(send_buf);

//    sprintf(send_buf, "mag_yaw:%.2f\r\n", yaw_mag_rel);
//    wireless_uart_send_string(send_buf);
//
//    sprintf(send_buf, "fin_yaw:%.2f\r\n", yaw_ekf);
//    wireless_uart_send_string(send_buf);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      40ms周期任务
 ////  @param      void
 ////  @return     void
 ////  @note       执行INS显示任务
 ////-------------------------------------------------------------------------------------------------------------------
static void run_40ms_tasks(void)
{
//    key_scanner();                                                        // 按键扫描
    INS_Display();                                                        // INS显示任务
//    send_position_to_host();
    imu_mag_send_raw_data_to_pc();
//    send_pos_to_host();
//    menu();
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      中断任务轮询
 ////  @param      void
 ////  @return     void
 ////  @note       在主循环中调用，处理挂起的周期任务
 ////-------------------------------------------------------------------------------------------------------------------
void InterruptTasks_Poll(void)
{
    while (s_pending_4ms)
    {
        s_pending_4ms--;
        run_4ms_tasks();
    }

    while (s_pending_8ms)
    {
        s_pending_8ms--;
        run_8ms_tasks();
    }

    while (s_pending_40ms)
    {
        s_pending_40ms--;
        run_40ms_tasks();
    }

    s_pending_1ms = 0;
    s_pending_2ms = 0;
    s_pending_16ms = 0;
}
