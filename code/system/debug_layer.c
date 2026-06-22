/*
 * debug_layer.c
 *
 *  Created on: 2026年4月27日
 *      Author: 调试层抽象
 */

#include "debug_layer.h"

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
    printf("(%.2f,%.2f)\r\n", INS.cod_RealTime.x, INS.cod_RealTime.y);
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
    // 需要输出什么调试信息，在这里把注释解开即可，不影响 interrupt.c
    
    // debug_send_position_to_host();
    // imu_mag_send_raw_data_to_pc();
    // debug_send_pos_to_host();
    // debug_send_attitude_to_host();

    // 测试编码器数据发送
    encoder_send_speed_to_host();

    // 实时发送转向角度，用于调试零位和最大转角
    steering_send_angle_to_host();
}