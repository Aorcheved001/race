/*
 * interrupt.c
 */

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"
#include "rtk.h"

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

    *yaw_rad = normalize_angle_rad_local(atan2f(mag_y, mag_x));
    return 1u;
}

void Interrupt_1ms(void)  { debug_layer_on_1ms_tick(); if (s_pending_1ms  < 500u) s_pending_1ms++; }
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
    date_handle(&imu_date);
    steering_control();

    // 从环形缓冲区消费编码器采样（而非直接读 g_state，避免跨核数据丢失/重复）
    EncoderRingEntry enc_sample;
    if (encoder_ring_pop(&enc_sample))
    {
        s_ins_input.v_mps = enc_sample.speed_avg_mps;
        s_ins_input.gyro_z_rad_s = imu660.data_Ripen.gyro_z;

        // 航向源选择：
        // [方案A] 纯陀螺积分：无磁力计修正，短时精度高但长时漂移
        // s_ins_input.mag_valid = 0u;
        // s_ins_input.mag_yaw_rad = 0.0f;

        // [方案B] EKF融合磁力计：磁力计修正陀螺漂移，长时稳定但受磁干扰
        s_ins_input.mag_valid = 1u;
        s_ins_input.mag_yaw_rad = 0.0f;  // INS内部自行读取imu660磁力计原始数据，此字段未使用
        s_ins_input.delta_left_m = enc_sample.delta_left_m;
        s_ins_input.delta_right_m = enc_sample.delta_right_m;
        float steer_deg = steering_get_current_angle();
        float steer_rad = steer_deg * 0.017453292519943295f;
        float omega_rad_s = s_ins_input.v_mps * tanf(steer_rad) / INS_WHEELBASE_M;

        if(steer_deg < -INS_STEER_MODEL_DEADBAND_DEG)
        {
            omega_rad_s *= INS_STEER_MODEL_LEFT_SCALE;
        }
        else if(steer_deg > INS_STEER_MODEL_DEADBAND_DEG)
        {
            omega_rad_s *= INS_STEER_MODEL_RIGHT_SCALE;
        }
        s_ins_input.omega_rad_s = omega_rad_s;

        Ins_update(&s_ins_input, 0.004f);
        track_proc();                                             // 轨迹处理（内部自行检查flag）
    }
    rtk_update();                                                 // RTK数据更新（GNSS解析+ENU转换+INS对齐）
    // 缓冲区空：说明主循环还没追上 ISR 节拍，跳过本周期
    //（总比用重复/丢失的数据好）
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      8ms周期任务
 ////  @param      void
 ////  @return     void
 ////  @note       执行按键扫描、遥控处理、INS导航任务
 ////-------------------------------------------------------------------------------------------------------------------
static void run_8ms_tasks(void)
{
//    yaokong_data_deal();
    key_scanner();
    //    subject1_update();
    subject3_update();
}


static void send_position_to_host(void)
{
    printf("(%.2f,%.2f)\r\n", INS.cod_RealTime.x, INS.cod_RealTime.y);
}


static void send_ring_count_to_host(void)
{
    printf("ring:%lu\r\n", (unsigned long)encoder_ring_count());
}

static void send_ring_count_wireless(void)
{
    static char buf[64];
    snprintf(buf, sizeof(buf), "ring:%lu\r\n", (unsigned long)encoder_ring_count());
    wireless_uart_send_buffer(buf, 18);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      40ms周期任务
 ////  @param      void
 ////  @return     void
 ////  @note       执行INS显示任务
 ////-------------------------------------------------------------------------------------------------------------------
static void run_40ms_tasks(void)
{
    // subject1_display();                                                        // INS显示任务
    subject3_display();                                                        // 科目三显示任务
    // 统一调试输出层：位置发送、无线队列刷新等所有周期性输出集中在此
    debug_layer_output_40ms();
//    imu_send_gyro_yaw_compare_to_pc();
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      中断任务轮询
 ////  @param      void
 ////  @return     void
 ////  @note       在主循环中调用，处理挂起的周期任务
 ////-------------------------------------------------------------------------------------------------------------------
void InterruptTasks_Poll(void)
{
    debug_layer_record_pending4(s_pending_4ms);
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
