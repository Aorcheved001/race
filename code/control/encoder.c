/*
 * encoder.c
 *
 *  编码器驱动与里程计计算
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */
#include "zf_common_headfile.h"

//------------------------------------------- 全局变量 ------------------------------------------------------------
static float g_tick_to_meter_left  = ENCODER_TICK_TO_METER_DEFAULT;   // 每脉冲对应米数 m/tick
static float g_tick_to_meter_right = ENCODER_TICK_TO_METER_DEFAULT;   // 每脉冲对应米数 m/tick
static float g_sample_dt_s = 0.004f;                                   // 采样时间 单位：s

//------------------------------------------- 全局结构体 ------------------------------------------------------------
static EncoderLayerState g_state;

//------------------------------------------- 函数实现 ------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器初始化
//  @param      无
//-------------------------------------------------------------------------------------------------------------------
void encoder_init(void)
{
    // 初始化左右编码器
    encoder_quad_init(ENCODER_LEFT_ID,  ENCODER_LEFT_PIN_A,  ENCODER_LEFT_PIN_B);
    encoder_quad_init(ENCODER_RIGHT_ID, ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B);

    // 清空计数值
    encoder_clear_count(ENCODER_LEFT_ID);
    encoder_clear_count(ENCODER_RIGHT_ID);

    // 状态结构体清零
    memset(&g_state, 0, sizeof(g_state));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置编码器参数
//  @param      tick_to_meter_left   左编码器脉冲->米
//  @param      tick_to_meter_right  右编码器脉冲->米
//  @param      sample_dt_s          采样周期
//-------------------------------------------------------------------------------------------------------------------
void encoder_layer_set_model(float tick_to_meter_left, float tick_to_meter_right, float sample_dt_s)
{
    if (tick_to_meter_left  != 0.0f)  g_tick_to_meter_left  = tick_to_meter_left;
    if (tick_to_meter_right != 0.0f)  g_tick_to_meter_right = tick_to_meter_right;
    if (sample_dt_s > 1e-6f)          g_sample_dt_s        = sample_dt_s;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器数据更新（放在定时中断里）
//  @param      无
//-------------------------------------------------------------------------------------------------------------------
void encoder_layer_update(void)
{
    // 读取脉冲值（注意方向取反）
    int16 tick_left  = -encoder_get_count(ENCODER_LEFT_ID);
    int16 tick_right =  encoder_get_count(ENCODER_RIGHT_ID);

    // 清空计数器
    encoder_clear_count(ENCODER_LEFT_ID);
    encoder_clear_count(ENCODER_RIGHT_ID);

    // 保存原始脉冲
    g_state.tick_left   = tick_left;
    g_state.tick_right  = tick_right;

    // 计算位移（米）
    g_state.delta_left_m   = (float)tick_left  * g_tick_to_meter_left;
    g_state.delta_right_m  = (float)tick_right * g_tick_to_meter_right;

    // 累计里程
    g_state.odom_left_m   += g_state.delta_left_m;
    g_state.odom_right_m  += g_state.delta_right_m;

    // 计算速度 m/s
    g_state.speed_left_mps  = g_state.delta_left_m  / g_sample_dt_s;
    g_state.speed_right_mps = g_state.delta_right_m / g_sample_dt_s;

    // 平均速度
    g_state.speed_average_mps = 0.5f * (g_state.speed_left_mps + g_state.speed_right_mps);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取编码器状态结构体
//  @param      无
//-------------------------------------------------------------------------------------------------------------------
const EncoderLayerState* encoder_layer_get_state(void)
{
    return &g_state;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取原始计数值
//-------------------------------------------------------------------------------------------------------------------
int32 encoder_layer_get_raw_count_left(void)
{
    return -encoder_get_count(ENCODER_LEFT_ID);
}

int32 encoder_layer_get_raw_count_right(void)
{
    return encoder_get_count(ENCODER_RIGHT_ID);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      清空原始计数器
//-------------------------------------------------------------------------------------------------------------------
void encoder_layer_clear_raw_counts(void)
{
    encoder_clear_count(ENCODER_LEFT_ID);
    encoder_clear_count(ENCODER_RIGHT_ID);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      清空累计里程
//-------------------------------------------------------------------------------------------------------------------
void encoder_layer_clear_odom(void)
{
    g_state.odom_left_m  = 0.0f;
    g_state.odom_right_m = 0.0f;
}
