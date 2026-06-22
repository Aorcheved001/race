/*
 * encoder.c
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */
#include "zf_common_headfile.h"

 //-------------------------------------------内部定义区------------------------------------------------------------

static float g_tick_to_meter_left  = ENCODER_TICK_TO_METER_DEFAULT;   // 每脉冲对应米数 m/tick
static float g_tick_to_meter_right = ENCODER_TICK_TO_METER_DEFAULT;   // 每脉冲对应米数 m/tick
static float g_sample_dt_s = 0.004f;               // 采样周期（秒），>0 有效；应与 encoder_layer_update

// tick_left_pid/tick_right_pid 已废弃，改用 g_state.tick_left_4ms/tick_right_4ms
 //-------------------------------------------内部结构体区------------------------------------------------------------
EncoderLayerState g_state;
static volatile uint8 g_update_ready = 0u;                             // [P7-1-6] 原子更新完成标志

// === 环形缓冲区（SPSC 无锁，位于共享 RAM，Core1 ISR 写 / Core0 主循环读） ===
static EncoderRingEntry g_enc_ring[ENC_RING_BUF_SIZE];
// write_idx: 仅 Core1 ISR 独占写 → Core0 只读
// read_idx : 仅 Core0 主循环独占写 → Core1 ISR 只读
// TriCore 的 uint32 对齐读写是硬件原子的，无需关中断
static volatile uint32 g_enc_write_idx = 0;
static volatile uint32 g_enc_read_idx  = 0;
 //-------------------------------------------函数定义区------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      编码器初始化
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    encoder_quad_init(ENCODER_LEFT_ID, ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B);        //初始化左轮正交编码器
    encoder_quad_init(ENCODER_RIGHT_ID, ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B);     //初始化右轮正交编码器

    encoder_clear_count(ENCODER_LEFT_ID);                                               //清除左轮计数
    encoder_clear_count(ENCODER_RIGHT_ID);                                              //清除右轮计数
    memset(&g_state, 0, sizeof(g_state));                                               //对内存进行操作，使得g_state中的所有字节都被设置为0（含累计脉冲清零）
    g_update_ready = 0u;                                                                // [P7-1-6] 重置更新标志
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      设置编码器模型参数
 ////  @param      tick_to_meter_left  左轮“每 tick 位移”（m/tick），>0 有效；可为负以修正方向
 ////  @param      tick_to_meter_right 右轮“每 tick 位移”（m/tick），>0 有效；可为负以修正方向
 ////  @param      sample_dt_s         采样周期（秒），>0 有效；应与 encoder_layer_update 的调用周期一致
 ////-------------------------------------------------------------------------------------------------------------------
void encoder_layer_set_model(float tick_to_meter_left, float tick_to_meter_right, float sample_dt_s)
{
    if (tick_to_meter_left != 0.0f) g_tick_to_meter_left = tick_to_meter_left;                  //设置左轮“每 tick 位移”（m/tick）不等于0 有效；可为负以修正方向
    if (tick_to_meter_right != 0.0f) g_tick_to_meter_right = tick_to_meter_right;              //设置右轮“每 tick 位移”（m/tick）不等于0 有效；可为负以修正方向
    if (sample_dt_s > 1e-6f) g_sample_dt_s = sample_dt_s;                                      //设置采样周期（秒）不等于0 有效；应与 encoder_layer_update 的调用周期一致
}


////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      周期更新（一次采样与计算）
 ////  @param      无
 ////  @note        读硬件→立即清零（原子操作，不丢脉冲），推入环形缓冲区；同步更新 g_state
 ////-------------------------------------------------------------------------------------------------------------------
void encoder_layer_update(void)
{
    // 1. 读硬件 + 立即清零（原子，保证每周期读数独立）
    int16 tick_left  = -encoder_get_count(ENCODER_LEFT_ID);
    int16 tick_right =  encoder_get_count(ENCODER_RIGHT_ID);
    encoder_clear_count(ENCODER_LEFT_ID);
    encoder_clear_count(ENCODER_RIGHT_ID);

    // 2. 计算物理量
    float dl = (float)tick_left  * g_tick_to_meter_left;
    float dr = (float)tick_right * g_tick_to_meter_right;
    float speed_avg = 0.5f * (dl + dr) / g_sample_dt_s;

    // 3. 推入环形缓冲区（Core0 消费端从缓冲区读取，不再读 g_state）
    {
        uint32 w = g_enc_write_idx;
        uint32 next_w = (w + 1) & (ENC_RING_BUF_SIZE - 1);

        // 缓冲区满 → 丢弃本次采样（保护已存数据不被覆盖）
        if (next_w != g_enc_read_idx)
        {
            EncoderRingEntry* entry = &g_enc_ring[w];
            entry->tick_left     = tick_left;
            entry->tick_right    = tick_right;
            entry->delta_left_m  = dl;
            entry->delta_right_m = dr;
            entry->speed_avg_mps = speed_avg;
            g_enc_write_idx = next_w;  // 推进写指针（uint32 对齐硬件原子）
        }
    }

    // 4. 同步更新 g_state（Core1 自身的电机 PID 仍通过 g_state 读脉冲）
    g_state.tick_left   = tick_left;
    g_state.tick_right  = tick_right;
    g_state.delta_left_m  = dl;
    g_state.delta_right_m = dr;
    g_state.odom_left_m  += dl;
    g_state.odom_right_m += dr;
    g_state.speed_left_mps   = dl / g_sample_dt_s;
    g_state.speed_right_mps  = dr / g_sample_dt_s;
    g_state.speed_average_mps = speed_avg;
    g_state.total_tick_left  += (int32)tick_left;
    g_state.total_tick_right += (int32)tick_right;

    g_update_ready = 1u;                                              // [P7-1-6] 标记更新完成，通知读取端
}
////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      从环形缓冲区弹出一个编码器采样（SPSC 无锁，仅 Core0 主循环调用）
 ////  @param      out      输出参数，存放弹出的采样数据
 ////  @return     uint8    1=成功取出数据, 0=缓冲区空（主循环追上了 ISR 节拍）
 ////  @note       缓冲区空时不阻塞，调用方检查返回值
 ////-------------------------------------------------------------------------------------------------------------------
uint8 encoder_ring_pop(EncoderRingEntry* out)
{
    if (out == NULL) return 0u;

    uint32 r = g_enc_read_idx;

    // 缓冲区空（read 追上了 write）
    if (r == g_enc_write_idx)
    {
        return 0u;
    }

    *out = g_enc_ring[r];  // 拷贝整条记录
    g_enc_read_idx = (r + 1) & (ENC_RING_BUF_SIZE - 1);  // 推进读指针
    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      获取环形缓冲区当前积压数量
 ////  @param      无
 ////  @return     uint32   未消费的采样条数（0 = Consumer 已追上 Producer）
 ////-------------------------------------------------------------------------------------------------------------------
uint32 encoder_ring_count(void)
{
    uint32 w = g_enc_write_idx;
    uint32 r = g_enc_read_idx;

    if (w >= r)
    {
        return w - r;
    }
    else
    {
        // write 已绕回，write 在 read 前面
        return (ENC_RING_BUF_SIZE - r) + w;
    }
}
////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      返回当前的g_state的地址
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
const EncoderLayerState* encoder_layer_get_state(void)
{
    // [P7-1-6] 检查是否已完成一次完整更新，防止读取到部分更新的数据
    if(g_update_ready == 0u)
    {
        return NULL;                                                   // 数据尚未就绪，调用者需判空
    }
    return &g_state;
}
////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      返回当前的左轮脉冲值
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
int32 encoder_layer_get_raw_count_left(void)
{
    return -encoder_get_count(ENCODER_LEFT_ID);
}
////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      返回当前的左轮脉冲值
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
int32 encoder_layer_get_raw_count_right(void)
{
    return encoder_get_count(ENCODER_RIGHT_ID);                       // 右轮不取反，原始脉冲前进时为正
}
//////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      清零里程计脉冲
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
void encoder_layer_clear_raw_counts(void)
{
    encoder_clear_count(ENCODER_LEFT_ID);
    encoder_clear_count(ENCODER_RIGHT_ID);
}
//////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      清零累计里程
 ////  @param      无
 ////-------------------------------------------------------------------------------------------------------------------
void encoder_layer_clear_odom(void)
{
    g_state.odom_left_m = 0.0f;
    g_state.odom_right_m = 0.0f;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      发送编码器速度数据到上位机
 ////  @param      无
 ////  @note       使用 printf 发送，格式: enc:tick_left,tick_right,speed_left,speed_right,speed_avg
 ////-------------------------------------------------------------------------------------------------------------------
void encoder_send_speed_to_host(void)
{
//    // 方法1: 发送处理后的速度数据
//    printf("enc:%.3f,%.3f,%.3f\r\n",
//           g_state.speed_left_mps,
//           g_state.speed_right_mps,
//           g_state.speed_average_mps);
//
//    // 方法2: 发送原始 tick 数据（用于调试硬件）
//    int32 raw_left = encoder_get_count(ENCODER_LEFT_ID);
//    int32 raw_right = encoder_get_count(ENCODER_RIGHT_ID);
//    printf("raw:%ld,%ld\r\n", (long)raw_left, (long)raw_right);

    // 方法3: 累计里程脉冲
    printf("odom:%.4f,%.4f\r\n", g_state.odom_left_m, g_state.odom_right_m);
}
