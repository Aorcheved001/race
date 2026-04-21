/*
 * interrupt.c
 */

//-------------------------------------------头文件引用------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------内部变量定义------------------------------------------------------------
static volatile uint16 s_pending_1ms = 0;                                 // 1ms 任务计数器
static volatile uint16 s_pending_2ms = 0;                                 // 2ms 任务计数器
static volatile uint16 s_pending_4ms = 0;                                 // 4ms 任务计数器
static volatile uint16 s_pending_8ms = 0;                                 // 8ms 任务计数器
static volatile uint16 s_pending_16ms = 0;                                // 16ms 任务计数器
static volatile uint16 s_pending_40ms = 0;                                // 40ms 任务计数器

volatile uint8 g_key_scan_flag = 0;                                       // 按键扫描标志
static INS_Input s_ins_input = {0};                                       // INS 输入缓存
static float s_roll_zero_offset_deg = 0.0f;
static float s_pitch_zero_offset_deg = 0.0f;
static float s_roll_zero_sum_deg = 0.0f;
static float s_pitch_zero_sum_deg = 0.0f;
static uint16 s_attitude_zero_sample_count = 0u;
static uint8 s_attitude_zero_ready = 0u;

//-------------------------------------------内部工具函数------------------------------------------------------------
static float normalize_angle_rad_local(float angle)
{
    while (angle > INS_PI) angle -= 2.0f * INS_PI;
    while (angle < -INS_PI) angle += 2.0f * INS_PI;
    return angle;
}

static void update_attitude_zero_offset(float roll_deg, float pitch_deg)
{
    const uint16 required_samples = 50u;

    if(s_attitude_zero_ready != 0u)
    {
        return;
    }

    s_roll_zero_sum_deg += roll_deg;
    s_pitch_zero_sum_deg += pitch_deg;
    s_attitude_zero_sample_count++;

    if(s_attitude_zero_sample_count >= required_samples)
    {
        s_roll_zero_offset_deg = s_roll_zero_sum_deg / (float)s_attitude_zero_sample_count;
        s_pitch_zero_offset_deg = s_pitch_zero_sum_deg / (float)s_attitude_zero_sample_count;
        s_attitude_zero_ready = 1u;
    }
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
////  @brief      4ms 周期任务函数
////  @param      void
////  @return     void
////  @note       执行包括 IMU 数据处理、编码器更新、PID 控制、INS 更新、轨迹处理
////-------------------------------------------------------------------------------------------------------------------
static void run_4ms_tasks(void)
{
    date_handle(&imu_date);                                               // 处理 IMU 数据
    encoder_layer_update();                                               // 更新编码器状态

    const EncoderLayerState* enc = encoder_layer_get_state();             // 获取编码器状态
    wheel_pid_update(enc, 0.004f);                                        // 更新电机 PID

    steering_control();                                                   // 更新转向控制（4ms周期调用）

    s_ins_input.v_mps = enc->speed_average_mps;                           // 写入 INS 速度数据
    s_ins_input.gyro_z_rad_s = imu660.data_Ripen.gyro_z;                  // 写入 INS 角速度数据
    s_ins_input.mag_valid = get_mag_yaw_measurement(&s_ins_input.mag_yaw_rad);   // 更新磁信号观测

    float steer_rad = steering_get_current_angle() * 0.017453292519943295f;  // 当前转向角（弧度）
    s_ins_input.omega_rad_s = s_ins_input.v_mps * tanf(steer_rad) / INS_WHEELBASE_M;   // 计算车身角速度

    Ins_update(&s_ins_input, 0.004f);                                     // 更新 INS
    track_proc();                                                         // 执行循迹处理
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      8ms 周期任务函数
////  @param      void
////  @return     void
////  @note       处理遥控、按键和 INS 导航任务
////-------------------------------------------------------------------------------------------------------------------
static void run_8ms_tasks(void)
{
    g_key_scan_flag = 1;                                                  // 置位按键扫描标志

    yaokong_set_control_enabled((track_follow_flag == 0u) ? 1u : 0u);     // 设置遥控是否接管
    yaokong_data_deal();                                                  // 处理遥控数据

    key_scanner();
    INS_NavigationTask();                                                 // 执行 INS 导航任务
}


static void send_position_to_host(void)
{
    printf("(%.2f,%.2f)\r\n", INS.cod_RealTime.x, INS.cod_RealTime.y);
}

static void send_pos_to_host(void)
{
    float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
    Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

    // 转成角度和百分度单位，方便直接查看
    yaw_gyro *= 57.29578f;
    yaw_mag_rel *= 57.29578f;
    yaw_ekf *= 57.29578f;

    yaw_gyro = -yaw_gyro;
    yaw_mag_rel = -yaw_mag_rel;
    yaw_ekf = -yaw_ekf;

    char send_buf[64];

    // 当前方向角统一为顺时针为正
    sprintf(send_buf, "imu_yaw:%.2f,%.2f,%.2f\r\n", yaw_gyro,yaw_mag_rel,yaw_ekf);
    wireless_uart_send_string(send_buf);

//    sprintf(send_buf, "mag_yaw:%.2f\r\n", yaw_mag_rel);
//    wireless_uart_send_string(send_buf);
//
//    sprintf(send_buf, "fin_yaw:%.2f\r\n", yaw_ekf);
//    wireless_uart_send_string(send_buf);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      发送姿态和三轴磁力计数据到上位机
////  @param      void
////  @return     void
////  @note       输出格式为 imu_att:roll,pitch,yaw_gyro,yaw_mag_raw,yaw_mag_rel,yaw_ekf
////-------------------------------------------------------------------------------------------------------------------
static void send_attitude_to_host(void)
{
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    float body_roll_rad = 0.0f;
    float body_pitch_rad = 0.0f;
    float yaw_gyro = 0.0f;
    float yaw_mag_raw = 0.0f;
    float yaw_mag_rel = 0.0f;
    float yaw_ekf = 0.0f;
    float mag_x = 0.0f;
    float mag_y = 0.0f;
    float mag_z = 0.0f;
    char send_buf[128];
    char mag_buf[128];

    Ins_get_attitude(&roll_rad, &pitch_rad, &yaw_rad);
    Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);
    Ins_get_mag_vector(&mag_x, &mag_y, &mag_z);

    (void)yaw_rad;

    // 将载体姿态映射到车体坐标系
    // 车体 roll 取当前 pitch
    // 车体 pitch 取当前 -roll
    body_roll_rad = pitch_rad;
    body_pitch_rad = -roll_rad;

    body_roll_rad *= INS_RAD2DEG;
    body_pitch_rad *= INS_RAD2DEG;
    yaw_gyro *= INS_RAD2DEG;
    yaw_mag_raw *= INS_RAD2DEG;
    yaw_mag_rel *= INS_RAD2DEG;
    yaw_ekf *= INS_RAD2DEG;

    yaw_gyro = -yaw_gyro;
    yaw_mag_raw = -yaw_mag_raw;
    yaw_mag_rel = -yaw_mag_rel;
    yaw_ekf = -yaw_ekf;

    update_attitude_zero_offset(body_roll_rad, body_pitch_rad);

    if(s_attitude_zero_ready != 0u)
    {
        body_roll_rad -= s_roll_zero_offset_deg;
        body_pitch_rad -= s_pitch_zero_offset_deg;
    }

    sprintf(send_buf,
            "imu_att:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
            body_roll_rad,
            body_pitch_rad,
            yaw_gyro,
            yaw_mag_raw,
            yaw_mag_rel,
            yaw_ekf);
    wireless_uart_send_string(send_buf);

    sprintf(mag_buf, "imu_mag3:%.3f,%.3f,%.3f\r\n", mag_x, mag_y, mag_z);
    wireless_uart_send_string(mag_buf);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      40ms 周期任务函数
////  @param      void
////  @return     void
////  @note       执行显示和上位机通信
////-------------------------------------------------------------------------------------------------------------------
static void run_40ms_tasks(void)
{
//    key_scanner();                                                        // 按键扫描
    INS_Display();                                                        // INS 信息显示
//    send_position_to_host();
//    imu_mag_send_raw_data_to_pc();
//    send_pos_to_host();
    send_attitude_to_host();
//    menu();
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      中断任务轮询函数
////  @param      void
////  @return     void
////  @note       在主循环中调用，根据计数器执行各任务
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
