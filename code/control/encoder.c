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
static float g_sample_dt_s = 0.001f;               // 采样周期（秒），>0 有效；应与 encoder_layer_update 的调用周期一致
volatile int16 tick_left_pid = 0;          //周期更新一次编码器获得的数值  左轮取反值，约定前进方向速度为正
volatile int16 tick_right_pid = 0;
 //-------------------------------------------内部结构体区------------------------------------------------------------
static EncoderLayerState g_state;
static volatile uint8 g_update_ready = 0u;                             // [P7-1-6] 原子更新完成标志

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
    memset(&g_state, 0, sizeof(g_state));                                               //对内存进行操作，使得g_state中的所有字节都被设置为0
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
 ////-------------------------------------------------------------------------------------------------------------------
void encoder_layer_update(void)
{
    int16 tick_left = -encoder_get_count(ENCODER_LEFT_ID);          //周期更新一次编码器获得的数值  左轮取反值，约定前进方向速度为正
    int16 tick_right = encoder_get_count(ENCODER_RIGHT_ID);         //周期更新一次编码器获得的数值  右轮值，约定前进方向速度为正

    tick_left_pid = tick_left;
    tick_right_pid = tick_right;

    encoder_clear_count(ENCODER_LEFT_ID);                           //周期清除左轮计数
    encoder_clear_count(ENCODER_RIGHT_ID);                          //周期清除右轮计数

    g_state.tick_left = tick_left;                                  //缓冲区存入结构体  左轮值，约定前进方向速度为正
    g_state.tick_right = tick_right;                                //缓冲区存入结构体  右轮值，约定前进方向速度为正

    g_state.delta_left_m = (float)tick_left * g_tick_to_meter_left;     //脉冲数转化为左轮位移
    g_state.delta_right_m = (float)tick_right * g_tick_to_meter_right;  //脉冲数转化为右轮位移

    g_state.odom_left_m += g_state.delta_left_m;                                              //累计左轮位移
    g_state.odom_right_m += g_state.delta_right_m;                                            //累计右轮位移

    g_state.speed_left_mps = g_state.delta_left_m / g_sample_dt_s;                           //计算左轮速度
    g_state.speed_right_mps = g_state.delta_right_m / g_sample_dt_s;                         //计算右轮速度

    g_state.speed_average_mps = 0.5f * (g_state.speed_left_mps + g_state.speed_right_mps);  //计算后轴中心点平均速度

    g_update_ready = 1u;                                              // [P7-1-6] 标记更新完成，通知读取端
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
    return encoder_get_count(ENCODER_RIGHT_ID);
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
    // 方法1: 发送处理后的速度数据
    printf("enc:%.3f,%.3f,%.3f\r\n", 
           g_state.speed_left_mps, 
           g_state.speed_right_mps, 
           g_state.speed_average_mps);
    
    // 方法2: 发送原始 tick 数据（用于调试硬件）
    int32 raw_left = encoder_get_count(ENCODER_LEFT_ID);
    int32 raw_right = encoder_get_count(ENCODER_RIGHT_ID);
    printf("raw:%ld,%ld\r\n", (long)raw_left, (long)raw_right);
}
