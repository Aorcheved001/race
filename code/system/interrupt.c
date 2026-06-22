/*
 * interrupt.c
 */

//-------------------------------------------头文件引用------------------------------------------------------------
#include "zf_common_headfile.h"
#include "subject_003.h"

//-------------------------------------------内部变量定义------------------------------------------------------------
static volatile uint16 s_pending_1ms = 0;                                 // 1ms 任务计数器
static volatile uint16 s_pending_2ms = 0;                                 // 2ms 任务计数器
static volatile uint16 s_pending_4ms = 0;                                 // 4ms 任务计数器
static volatile uint16 s_pending_8ms = 0;                                 // 8ms 任务计数器
static volatile uint16 s_pending_16ms = 0;                                // 16ms 任务计数器
static volatile uint16 s_pending_40ms = 0;                                // 40ms 任务计数器

static volatile uint32 s_debug_count_1ms = 0;                             // 调试：1ms中断计数
static volatile uint32 s_debug_count_40ms = 0;                            // 调试：40ms中断计数

volatile uint8 g_key_scan_flag = 0;                                       // 按键扫描标志
static INS_Input s_ins_input = {0};                                       // INS 输入缓存

//-------------------------------------------内部工具函数------------------------------------------------------------
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

    // 仅检查磁力计数据有效性，不计算磁航向
    // 实际的倾斜补偿磁航向由 INS 内部的 compute_tilt_compensated_mag_yaw() 计算
    if((fabsf(mag_x) + fabsf(mag_y)) < 1e-4f)
    {
        return 0u;
    }

    // 返回有效标志，yaw_rad 不再使用简化计算（由 INS 内部处理）
    *yaw_rad = 0.0f;  // 占位值，实际值由 INS 内部计算
    return 1u;
}

void Interrupt_1ms(void)  { if (s_pending_1ms  < 500u) s_pending_1ms++; s_debug_count_1ms++; }
void Interrupt_2ms(void)  { if (s_pending_2ms  < 500u) s_pending_2ms++; }
void Interrupt_4ms(void)  { if (s_pending_4ms  < 500u) s_pending_4ms++; }
void Interrupt_8ms(void)  { if (s_pending_8ms  < 500u) s_pending_8ms++; }
void Interrupt_16ms(void) { if (s_pending_16ms < 500u) s_pending_16ms++; }
void Interrupt_40ms(void) { if (s_pending_40ms < 500u) s_pending_40ms++; s_debug_count_40ms++; }

////-------------------------------------------------------------------------------------------------------------------
////  @brief      1ms 周期任务函数
////  @param      void
////  @return     void
////  @note       [P7-3-3] 从 isr.c 移至中断层，由 InterruptTasks_Poll() 轮询执行
////  @note       包含编码器更新和车轮 PID 控制
////-------------------------------------------------------------------------------------------------------------------
static void run_1ms_tasks(void)
{
    encoder_layer_update();                                               // 更新编码器层状态

    const EncoderLayerState* enc = encoder_layer_get_state();             // 获取编码器状态
    if(enc != NULL)                                                       // [P7-1-6] 判空保护
    {
        wheel_pid_update(enc, 0.001f);                                    // 车轮 PID 计算
    }
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      4ms 周期任务函数
////  @param      void
////  @return     void
////  @note       执行包括 IMU 数据处理、编码器更新、PID 控制、INS 更新、轨迹处理
////-------------------------------------------------------------------------------------------------------------------
static void run_4ms_tasks(void)
{
   date_handle(&imu_date);                                               // 处理 IMU 数据 (未初始化)

   const EncoderLayerState* enc = encoder_layer_get_state();             // 获取编码器状态（更新在 run_1ms_tasks 中执行）

    steering_control();                                                   // 更新转向控制

   if(enc == NULL) return;                                               // [P7-1-6] 数据未就绪，跳过本周期
   s_ins_input.v_mps = enc->speed_average_mps;                           // 写入 INS 速度数据
   s_ins_input.gyro_z_rad_s = imu660.data_Ripen.gyro_z;                  // 写入 INS 角速度数据
   s_ins_input.mag_valid = get_mag_yaw_measurement(&s_ins_input.mag_yaw_rad);   // 更新磁信号观测
   s_ins_input.delta_left_m = enc->delta_left_m;                         // 写入 INS 左轮位移增量
   s_ins_input.delta_right_m = enc->delta_right_m;                       // 写入 INS 右轮位移增量

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
    yaokong_data_deal();      
                                                // 处理遥控数据
        key_scanner();
    subject1_update();                                                    // 执行科目一状态机更新
//    subject3_update();                                                    // 执行科目三状态机更新（默认关闭）
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
    subject1_display();                                                   // 科目一INS信息显示
//    menu();

    // 调用调试层统一输出接口
    debug_layer_output_40ms();
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      中断任务轮询函数
////  @param      void
////  @return     void
////  @note       在主循环中调用，根据计数器执行各任务
////  @note       使用临界区保护共享计数器，避免ISR与主循环并发竞争
////-------------------------------------------------------------------------------------------------------------------
void InterruptTasks_Poll(void)
{
    boolean int_state;
    uint32 pending_snapshot;

    // [P7-3-3] 1ms 任务：编码器更新 + PID 控制（从 isr.c 移入）
    // 必须在 4ms 任务之前执行，确保编码器数据最新
    int_state = disableInterrupts();
    pending_snapshot = s_pending_1ms;
    s_pending_1ms = 0;
    restoreInterrupts(int_state);

    while (pending_snapshot > 0)
    {
        pending_snapshot--;
        run_1ms_tasks();
    }

    // 4ms 任务：原子取快照后批量消费
    int_state = disableInterrupts();
    pending_snapshot = s_pending_4ms;
    s_pending_4ms = 0;
    restoreInterrupts(int_state);
    
    while (pending_snapshot > 0)
    {
        pending_snapshot--;
        run_4ms_tasks();
    }

    // 8ms 任务：原子取快照后批量消费
    int_state = disableInterrupts();
    pending_snapshot = s_pending_8ms;
    s_pending_8ms = 0;
    restoreInterrupts(int_state);
    
    while (pending_snapshot > 0)
    {
        pending_snapshot--;
        run_8ms_tasks();
    }

    // 40ms 任务：原子取快照后批量消费
    int_state = disableInterrupts();
    pending_snapshot = s_pending_40ms;
    s_pending_40ms = 0;
    restoreInterrupts(int_state);
    
    while (pending_snapshot > 0)
    {
        pending_snapshot--;
        run_40ms_tasks();
    }

    // 清除不使用的计数器（1ms 已在上方处理，不再清除）
    int_state = disableInterrupts();
    s_pending_2ms = 0;
    s_pending_16ms = 0;
    restoreInterrupts(int_state);
}

uint32 get_interrupt_count_1ms(void)
{
    return s_debug_count_1ms;
}

uint32 get_interrupt_count_40ms(void)
{
    return s_debug_count_40ms;
}
