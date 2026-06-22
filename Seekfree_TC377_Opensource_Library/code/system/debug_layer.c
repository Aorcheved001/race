/*
 * debug_layer.c
 *
 *  Created on: 2026年4月27日
 *      Author: 调试层抽象
 */

#include "debug_layer.h"
#include "rtk.h"

// -------------------------------------------------------------------------------------------------------------------
// 内部变量定义（为了解耦，把原本在 interrupt.c 里的相关变量搬到这里）
// -------------------------------------------------------------------------------------------------------------------
static float s_roll_zero_offset_deg = 0.0f;
static float s_pitch_zero_offset_deg = 0.0f;
static float s_roll_zero_sum_deg = 0.0f;
static float s_pitch_zero_sum_deg = 0.0f;
static uint16 s_attitude_zero_sample_count = 0u;
static uint8 s_attitude_zero_ready = 0u;

// -------------------------------------------------------------------------------------------------------------------
// 时间计数相关变量（用于检测MCU卡死）
// -------------------------------------------------------------------------------------------------------------------
static volatile uint32 s_debug_tick_ms = 0u;      // 1ms中断维护的系统时间戳
static volatile uint32 s_debug_drop_count = 0u;   // 丢包计数（队列满时丢弃）
static volatile uint32 s_debug_max_queue_depth = 0u;  // 队列最大深度
static volatile uint16 s_debug_pending4_max = 0u;   // 4ms任务最大积压数

#define DEBUG_TX_QUEUE_LEN      8u
#define DEBUG_TX_MSG_MAX_LEN    160u
typedef struct
{
    char msg[DEBUG_TX_MSG_MAX_LEN];
    uint16 len;
} DebugTxMsg;

static DebugTxMsg s_debug_tx_queue[DEBUG_TX_QUEUE_LEN];
static uint8 s_debug_tx_head = 0u;
static uint8 s_debug_tx_tail = 0u;
static uint8 s_debug_tx_count = 0u;

// -------------------------------------------------------------------------------------------------------------------
// 时间计数接口实现
// -------------------------------------------------------------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
////  @brief      1ms中断回调（在Interrupt_1ms中调用）
////  @param      void
////  @return     void
////  @note       每毫秒调用一次，维护系统时间戳
////-------------------------------------------------------------------------------------------------------------------
void debug_layer_on_1ms_tick(void)
{
    s_debug_tick_ms++;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取当前系统时间戳（毫秒）
////  @param      void
////  @return     uint32_t 当前时间戳
////  @note       用于检测MCU卡死：如果时间戳跳跃过大，说明发生了卡死
////-------------------------------------------------------------------------------------------------------------------
uint32 debug_layer_get_tick_ms(void)
{
    return s_debug_tick_ms;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取丢包计数
////  @param      void
////  @return     uint32_t 丢包数量
////  @note       队列满时丢弃的消息数
////-------------------------------------------------------------------------------------------------------------------
uint32 debug_layer_get_drop_count(void)
{
    return s_debug_drop_count;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取队列最大深度
////  @param      void
////  @return     uint32_t 队列历史最大深度
////  @note       用于监控队列使用情况
////-------------------------------------------------------------------------------------------------------------------
uint32 debug_layer_get_max_queue_depth(void)
{
    return s_debug_max_queue_depth;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取4ms任务最大积压数
////  @param      void
////  @return     uint16 最大积压数
////  @note       用于检测主循环是否处理不过来
////-------------------------------------------------------------------------------------------------------------------
uint16 debug_layer_get_pending4_max(void)
{
    return s_debug_pending4_max;
}


// -------------------------------------------------------------------------------------------------------------------
// 无线调试发送队列：任务层只入队，40ms统一flush，避免零散发送阻塞主循环
// -------------------------------------------------------------------------------------------------------------------
uint8 debug_wireless_queue_push(const char *msg, uint16 len)
{
    uint16 copy_len;

    if((msg == NULL) || (len == 0u))
    {
        return 0u;
    }

    if(s_debug_tx_count >= DEBUG_TX_QUEUE_LEN)
    {
        s_debug_drop_count++;
        return 0u;
    }

    copy_len = len;
    if(copy_len >= DEBUG_TX_MSG_MAX_LEN)
    {
        copy_len = DEBUG_TX_MSG_MAX_LEN - 1u;
    }

    memcpy(s_debug_tx_queue[s_debug_tx_tail].msg, msg, copy_len);
    s_debug_tx_queue[s_debug_tx_tail].msg[copy_len] = '\0';
    s_debug_tx_queue[s_debug_tx_tail].len = copy_len;
    s_debug_tx_tail = (uint8)((s_debug_tx_tail + 1u) % DEBUG_TX_QUEUE_LEN);
    s_debug_tx_count++;

    if((uint32)s_debug_tx_count > s_debug_max_queue_depth)
    {
        s_debug_max_queue_depth = (uint32)s_debug_tx_count;
    }

    return 1u;
}

void debug_wireless_queue_flush(void)
{
    while(s_debug_tx_count > 0u)
    {
        wireless_uart_send_buffer((const uint8 *)s_debug_tx_queue[s_debug_tx_head].msg,
                                  (uint32)s_debug_tx_queue[s_debug_tx_head].len);
        s_debug_tx_head = (uint8)((s_debug_tx_head + 1u) % DEBUG_TX_QUEUE_LEN);
        s_debug_tx_count--;
    }
}

void debug_layer_record_pending4(uint16 pending4)
{
    if(pending4 > s_debug_pending4_max)
    {
        s_debug_pending4_max = pending4;
    }
}
// -------------------------------------------------------------------------------------------------------------------
// 内部工具函数
// -------------------------------------------------------------------------------------------------------------------
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

// -------------------------------------------------------------------------------------------------------------------
// 具体的调试输出函数实现
// -------------------------------------------------------------------------------------------------------------------

void debug_send_position_to_host(void)
{
    const INS_State* state = Ins_get_state();
    char send_buf[64];
    if(state != NULL)
        sprintf(send_buf, "(%.2f,%.2f,%.3f)\r\n", INS.cod_RealTime.x, INS.cod_RealTime.y, state->yaw);
    else
        sprintf(send_buf, "(%.2f,%.2f,0.000)\r\n", INS.cod_RealTime.x, INS.cod_RealTime.y);
    wireless_uart_send_string(send_buf);
}

void debug_send_pos_to_host(void)
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
    sprintf(send_buf, "imu_yaw:%.2f,%.2f,%.2f\r\n", yaw_gyro, yaw_mag_rel, yaw_ekf);
    wireless_uart_send_string(send_buf);

    // sprintf(send_buf, "mag_yaw:%.2f\r\n", yaw_mag_rel);
    // wireless_uart_send_string(send_buf);
    //
    // sprintf(send_buf, "fin_yaw:%.2f\r\n", yaw_ekf);
    // wireless_uart_send_string(send_buf);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      发送姿态和三轴磁力计数据到上位机
////  @param      void
////  @return     void
////  @note       输出格式为 imu_att:roll,pitch,yaw_gyro,yaw_mag_raw,yaw_mag_rel,yaw_ekf
////-------------------------------------------------------------------------------------------------------------------
void debug_send_attitude_to_host(void)
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
    // 统一使用 INS 层的映射关系（与 map_body_attitude_for_mag_compensation 一致）
    // 车体 roll = -pitch (pitch 正向时车体向左滚转)
    // 车体 pitch = roll (roll 正向时车体抬头)
    body_roll_rad = -pitch_rad;
    body_pitch_rad = roll_rad;

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

// -------------------------------------------------------------------------------------------------------------------
//  @brief      统一的 40ms 调试输出任务（把零散的发送任务收拢到这里）
// -------------------------------------------------------------------------------------------------------------------
void debug_layer_output_40ms(void)
{
    static uint16 s_debug_seq = 0u;
    uint32 tick_ms = debug_layer_get_tick_ms();
    uint32 q_drop = debug_layer_get_drop_count();
    uint32 q_depth = debug_layer_get_max_queue_depth();
    uint16 pending4 = debug_layer_get_pending4_max();
    uint32 ring_count = encoder_ring_count();

    s_debug_seq++;

    // INS: seq,tick_ms,x,y,yaw_gyro,yaw_mag,yaw_ekf,ring_count,pending4,q_drop,q_depth
    {
        const INS_State* state = Ins_get_state();
        float yaw_gyro = 0.0f;
        float yaw_mag_raw = 0.0f;
        float yaw_mag_rel = 0.0f;
        float yaw_ekf = 0.0f;
        static char ins_buf[DEBUG_TX_MSG_MAX_LEN];

        if(state != NULL)
        {
            int ins_len;
            Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);
            yaw_gyro *= INS_RAD2DEG;
            yaw_mag_rel *= INS_RAD2DEG;
            yaw_ekf *= INS_RAD2DEG;
            yaw_gyro = -yaw_gyro;
            yaw_mag_rel = -yaw_mag_rel;
            yaw_ekf = -yaw_ekf;

            ins_len = snprintf(ins_buf, sizeof(ins_buf),
                               "I:%u,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%u,%lu,%lu\r\n",
                               s_debug_seq,
                               (unsigned long)tick_ms,
                               (double)state->x,
                               (double)state->y,
                               (double)yaw_gyro,
                               (double)yaw_mag_rel,
                               (double)yaw_ekf,
                               (unsigned long)ring_count,
                               (unsigned)pending4,
                               (unsigned long)q_drop,
                               (unsigned long)q_depth);
            if(ins_len > 0)
            {
                debug_wireless_queue_push(ins_buf, (uint16)ins_len);
            }
        }
    }

    // RTK: seq,tick_ms,x_ins,y_ins,yaw_ins,valid,x_enu,y_enu,yaw_tn,ant_raw,q_drop,ring_count,pending4
    {
        const rtk_state_t *rtk = rtk_get_state();
        if(rtk != NULL)
        {
            uint8 valid_flags = 0u;
            float yaw_ins_deg = rtk->yaw_ins * RTK_RAD_TO_DEG;
            float yaw_tn_deg = rtk->yaw_true_north * RTK_RAD_TO_DEG;
            static char rtk_buf[DEBUG_TX_MSG_MAX_LEN];
            int rtk_len;

            if(rtk->pos_valid) valid_flags |= 1u;
            if(rtk->yaw_valid) valid_flags |= 2u;
            if(rtk->is_aligned) valid_flags |= 4u;

            rtk_len = snprintf(rtk_buf, sizeof(rtk_buf),
                               "R:%u,%lu,%.4f,%.4f,%.2f,%u,%.4f,%.4f,%.4f,%.1f,%lu,%lu,%u\r\n",
                               s_debug_seq,
                               (unsigned long)tick_ms,
                               (double)rtk->x_ins,
                               (double)rtk->y_ins,
                               (double)yaw_ins_deg,
                               (unsigned)valid_flags,
                               (double)rtk->x_enu,
                               (double)rtk->y_enu,
                               (double)yaw_tn_deg,
                               (double)rtk->antenna_direction_deg,
                               (unsigned long)q_drop,
                               (unsigned long)ring_count,
                               (unsigned)pending4);
            if(rtk_len > 0)
            {
                debug_wireless_queue_push(rtk_buf, (uint16)rtk_len);
            }
        }
    }

    // GNSS: sats,fix_quality,pos_valid,origin_sample_pct
    {
        static uint8 gnss_status_counter = 0u;
        gnss_status_counter++;
        if(gnss_status_counter >= 25u)
        {
            const rtk_state_t *rtk = rtk_get_state();
            gnss_status_counter = 0u;
            if(rtk != NULL)
            {
                static char gnss_buf[48];
                uint8 sample_pct = rtk_get_origin_sample_progress();
                int gnss_len = snprintf(gnss_buf, sizeof(gnss_buf), "G:%u,%u,%u,%u\r\n",
                                        (unsigned)rtk->num_sats,
                                        (unsigned)rtk->fix_state,
                                        (unsigned)(rtk->pos_valid ? 1u : 0u),
                                        (unsigned)sample_pct);
                if(gnss_len > 0)
                {
                    debug_wireless_queue_push(gnss_buf, (uint16)gnss_len);
                }
            }
        }
    }

    debug_wireless_queue_flush();
}
