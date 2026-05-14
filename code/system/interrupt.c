/*
 * interrupt.c
 */

//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"


//-------------------------------------------全局变量定义------------------------------------------------------------
static volatile uint16 s_pending_1ms = 0;                                 // 1ms 任务计数器
static volatile uint16 s_pending_2ms = 0;                                 // 2ms 任务计数器
static volatile uint16 s_pending_4ms = 0;                                 // 4ms 任务计数器
static volatile uint16 s_pending_8ms = 0;                                 // 8ms 任务计数器
static volatile uint16 s_pending_16ms = 0;                                // 16ms 任务计数器
static volatile uint16 s_pending_40ms = 0;                                // 40ms 任务计数器

volatile uint8 g_key_scan_flag = 0;                                       // 按键扫描标志
static INS_Input s_ins_input = {0};                                       // INS 输入缓存
static float s_roll_zero_offset_deg = 0.0f;                               // 横滚角零点偏移
static float s_pitch_zero_offset_deg = 0.0f;                              // 俯仰角零点偏移
static float s_roll_zero_sum_deg = 0.0f;                                  // 横滚角累加值
static float s_pitch_zero_sum_deg = 0.0f;                                 // 俯仰角累加值
static uint16 s_attitude_zero_sample_count = 0u;                          // 姿态零点采样计数
static uint8 s_attitude_zero_ready = 0u;                                  // 姿态零点校准完成标志

//-------------------------------------------局部函数声明------------------------------------------------------------
/**
 * @brief  角度归一化（弧度），限制在 [-PI, PI]
 */
static float normalize_angle_rad_local(float angle)
{
    while (angle > INS_PI) angle -= 2.0f * INS_PI;
    while (angle < -INS_PI) angle += 2.0f * INS_PI;
    return angle;
}

/**
 * @brief  更新姿态零点偏移（自动校准）
 */
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

/**
 * @brief  获取磁力计计算的偏航角
 * @param  yaw_rad: 输出偏航角（弧度）
 * @return 有效标志
 */
static uint8 get_mag_yaw_measurement(float *yaw_rad)
{
    float mag_x = imu660.data_Ripen.mag_x;
    float mag_y = imu660.data_Ripen.mag_y;

    if(yaw_rad == NULL)
    {
        return 0u;
    }

    // 磁力计数据无效，直接返回
    if((fabsf(mag_x) + fabsf(mag_y)) < 1e-4f)
    {
        return 0u;
    }

    // 有效时输出yaw_rad，实际值由INS模块计算
    *yaw_rad = 0.0f;
    return 1u;
}

// 定时器中断计数函数
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
////  @note       执行：IMU数据处理、编码器更新、车轮PID、转向控制、INS更新、轨迹处理、数据上传
////-------------------------------------------------------------------------------------------------------------------
static void run_4ms_tasks(void)
{
//    date_handle(&imu_date);                                               // 处理 IMU 数据
//    encoder_layer_update();                                               // 更新编码器层状态 .
//
//    const EncoderLayerState* enc = encoder_layer_get_state();             // 获取编码器状态 .
//    wheel_pid_update(enc, 0.004f);                                        // 车轮 PID 计算 .

//    steering_control();                                                   // 转向控制（4ms周期执行）

//    s_ins_input.v_mps = enc->speed_average_mps;                           // 写入 INS 速度输入
//    s_ins_input.gyro_z_rad_s = imu660.data_Ripen.gyro_z;                  // 写入 INS 陀螺仪Z轴输入
//    s_ins_input.mag_valid = get_mag_yaw_measurement(&s_ins_input.mag_yaw_rad);   // 获取磁偏角观测值
//    s_ins_input.delta_left_m = enc->delta_left_m;                         // 写入 INS 左轮位移
//    s_ins_input.delta_right_m = enc->delta_right_m;                       // 写入 INS 右轮位移

//    float steer_rad = steering_get_current_angle() * 0.017453292519943295f;  // 当前转向角转弧度
//    s_ins_input.omega_rad_s = s_ins_input.v_mps * tanf(steer_rad) / INS_WHEELBASE_M;   // 计算车辆角速度
//
//    Ins_update(&s_ins_input, 0.004f);                                     // 更新 INS 数据
//    track_proc();                                                         // 轨迹处理

    // VOFA+ 数据发送与 PID 在线调节
    ///vofa_update(enc);                  //.

    // 从FIFO读取并处理无线指令
    uint8 rx_byte;
    while (wireless_uart_read_buffer(&rx_byte, 1) > 0)
    {
        wheel_pid_cmd_process_byte(rx_byte);
    }

    // 车轮指令处理与调试
    wheel_pid_cmd_poll();
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      8ms 周期任务函数
////  @param      void
////  @return     void
////  @note       执行：按键扫描、遥控器数据处理、INS导航任务
////-------------------------------------------------------------------------------------------------------------------
static void run_8ms_tasks(void)
{
    g_key_scan_flag = 1;                                                  // 置位按键扫描标志

    yaokong_set_control_enabled((track_follow_flag == 0u) ? 1u : 0u);     // 设置遥控器使能状态
    yaokong_data_deal();                                                  // 处理遥控器数据

    key_scanner();

    // 按键控制目标速度
//    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
//    {
//        key_clear_state(KEY_1);
//        g_wheel_pid.cmd_speed_left_mps  -= 1.0f;
//        g_wheel_pid.cmd_speed_right_mps -= 1.0f;
//    }

    INS_NavigationTask();                                                 // 执行 INS 导航任务
}

/**
 * @brief  发送坐标位置给上位机
 */
static void send_position_to_host(void)
{
    printf("(%.2f,%.2f)\r\n", INS.cod_RealTime.x, INS.cod_RealTime.y);
}

/**
 * @brief  发送偏航角数据给上位机
 */
static void send_pos_to_host(void)
{
    float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
    Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

    // 弧度转角度，方便查看
    yaw_gyro *= 57.29578f;
    yaw_mag_rel *= 57.29578f;
    yaw_ekf *= 57.29578f;

    yaw_gyro = -yaw_gyro;
    yaw_mag_rel = -yaw_mag_rel;
    yaw_ekf = -yaw_ekf;

    char send_buf[64];
    sprintf(send_buf, "imu_yaw:%.2f,%.2f,%.2f\r\n", yaw_gyro,yaw_mag_rel,yaw_ekf);
    wireless_uart_send_string(send_buf);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      发送姿态数据到上位机
////  @param      void
////  @return     void
////  @note       输出格式：imu_att:横滚,俯仰,陀螺偏航,磁原始偏航,磁相对偏航,EKF融合偏航
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

    // 姿态坐标系转换
    body_roll_rad = -pitch_rad;
    body_pitch_rad = roll_rad;

    // 弧度转角度
    body_roll_rad *= INS_RAD2DEG;
    body_pitch_rad *= INS_RAD2DEG;
    yaw_gyro *= INS_RAD2DEG;
    yaw_mag_raw *= INS_RAD2DEG;
    yaw_mag_rel *= INS_RAD2DEG;
    yaw_ekf *= INS_RAD2DEG;

    // 方向校准
    yaw_gyro = -yaw_gyro;
    yaw_mag_raw = -yaw_mag_raw;
    yaw_mag_rel = -yaw_mag_rel;
    yaw_ekf = -yaw_ekf;

    // 零点校准
    update_attitude_zero_offset(body_roll_rad, body_pitch_rad);
    if(s_attitude_zero_ready != 0u)
    {
        body_roll_rad -= s_roll_zero_offset_deg;
        body_pitch_rad -= s_pitch_zero_offset_deg;
    }

    // 发送姿态数据
    sprintf(send_buf,
            "imu_att:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
            body_roll_rad,
            body_pitch_rad,
            yaw_gyro,
            yaw_mag_raw,
            yaw_mag_rel,
            yaw_ekf);
    wireless_uart_send_string(send_buf);

    // 发送磁力计原始数据
    sprintf(mag_buf, "imu_mag3:%.3f,%.3f,%.3f\r\n", mag_x, mag_y, mag_z);
    wireless_uart_send_string(mag_buf);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      40ms 周期任务函数
////  @param      void
////  @return     void
////  @note       执行：信息显示、姿态数据上传
////-------------------------------------------------------------------------------------------------------------------
static void run_40ms_tasks(void)
{
    INS_Display();                                                        // INS 信息显示
    send_attitude_to_host();                                              // 发送姿态数据到上位机
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      中断任务轮询（主循环调用）
////  @param      void
////  @return     void
////  @note       主循环中调用，按周期执行任务
////  @note       使用中断计数，避免在ISR中执行耗时循环
////-------------------------------------------------------------------------------------------------------------------
void InterruptTasks_Poll(void)
{
    boolean int_state;
    uint32 pending_snapshot;
    
    // 读取并执行4ms任务
    int_state = disableInterrupts();
    pending_snapshot = s_pending_4ms;
    s_pending_4ms = 0;
    restoreInterrupts(int_state);
    
    while (pending_snapshot > 0)
    {
        pending_snapshot--;
        run_4ms_tasks();
    }

    // 读取并执行8ms任务
    int_state = disableInterrupts();
    pending_snapshot = s_pending_8ms;
    s_pending_8ms = 0;
    restoreInterrupts(int_state);
    
    while (pending_snapshot > 0)
    {
        pending_snapshot--;
        run_8ms_tasks();
    }

    // 读取并执行40ms任务
    int_state = disableInterrupts();
    pending_snapshot = s_pending_40ms;
    s_pending_40ms = 0;
    restoreInterrupts(int_state);
    
    while (pending_snapshot > 0)
    {
        pending_snapshot--;
        run_40ms_tasks();
    }

    // 清空未使用的计数器
    int_state = disableInterrupts();
    s_pending_1ms = 0;
    s_pending_2ms = 0;
    s_pending_16ms = 0;
    restoreInterrupts(int_state);
}
