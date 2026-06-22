/*
 * VOFA.c
 *
 * VOFA+ 上位机协�?实现
 * �?�? JustFloat 协�??发�? + 上位机实时调参（速度�?/位置�? Kp/Ki/Kd�?
 *
 * JustFloat 协�??帧格式：
 * [4字节 float][4字节 float]...[0x00 0x00 0x80 0x7f]  帧尾固定�?
 *
 * VOFA+ 调参指令格式示例�?
 * P1=0.5!   �? 速度�? Kp
 * K1=0.1!   �? 速度�? Ki
 * D1=0.05!  �? 速度�? Kd
 * P3=1.0!   �? 位置�? Kp
 * K3=0.0!   �? 位置�? Ki
 * D3=0.0!   �? 位置�? Kd
 *
 * 创建日期: 2026-03-15
 * 作�?: DreamerDay
 */

//------------------------------------------- 头文件包�? ------------------------------------------------------------
#include "VOFA.h"
#include "zf_device_wireless_uart.h"
#include "PID.h"
#include <math.h>
#include <encoder.h>

//------------------------------------------- 内部变量定义 ------------------------------------------------------------
uint8 vofa_uart_rx_buf[64];      // 串口接收缓冲�?
uint8 vofa_fifo_out_buf[64];     // FIFO 读出缓冲�?
uint8 vofa_get_data = 0;         // 单字节接收缓�?
uint32 vofa_fifo_data_count = 0; // FIFO �?当前数据�?
fifo_struct vofa_data_fifo;      // FIFO 结构�?

//------------------------------------------- 滑动平均滤波 ------------------------------------------------------------
#define VOFA_SMOOTH_SIZE    (8)      // 滑动平均窗口大小�?8�?采样点）

static float s_speed_left_buf[VOFA_SMOOTH_SIZE] = {0};
static float s_speed_right_buf[VOFA_SMOOTH_SIZE] = {0};
static float s_speed_target_buf[VOFA_SMOOTH_SIZE] = {0};
static uint8 s_smooth_index = 0;
static uint8 s_smooth_count = 0;

//------------------------------------------- 内部函数实现 ------------------------------------------------------------

/**
 * @brief  float �? 4字节（小�?模式�?
 * @param  val 要转换的�?点数
 * @param  buf 存放�?换后4字节的数�?
 */
static void vofa_float_to_bytes(float val, uint8 buf[4])
{
    uint8 *p = (uint8 *)&val;
    buf[0] = p[0];
    buf[1] = p[1];
    buf[2] = p[2];
    buf[3] = p[3];
}

/**
 * @brief  通过无线串口发送一�? float（JustFloat 协�??�?
 * @param  val 要发送的�?点数
 */
static void vofa_send_float(float val)
{
    uint8 byte[4];
    vofa_float_to_bytes(val, byte);
    wireless_uart_send_buffer(byte, 4);
}

/**
 * @brief  解析 VOFA+ 调参指令（格式：key=value!�?
 * @return 解析成功返回对应�?点值，失败返回 0.0f
 */
static float vofa_parse_value(void)
{
    uint8 start_idx = 0;
    uint8 end_idx = 0;
    uint8 valid = 0;

    for (uint8 i = 0; i < 64; i++)
    {
        if (vofa_fifo_out_buf[i] == '=')
        {
            start_idx = i + 1;
            valid |= 0x01;
        }
        if (vofa_fifo_out_buf[i] == '!')
        {
            end_idx = i - 1;
            valid |= 0x02;
            break;
        }
    }
    if (valid != 0x03) return 0.0f;

    uint8 cursor = start_idx;
    uint8 minus = 0;
    if (cursor <= end_idx && vofa_fifo_out_buf[cursor] == '-')
    {
        minus = 1;
        cursor++;
    }

    float result = 0.0f;
    uint8 dec_pos = 0;
    uint8 has_dec = 0;

    for (uint8 i = cursor; i <= end_idx; i++)
    {
        if (vofa_fifo_out_buf[i] == '.')
        {
            has_dec = 1;
            dec_pos = i - cursor;
            continue;
        }

        uint8 digit = vofa_fifo_out_buf[i] - '0';
        if (digit > 9) return 0.0f;

        if (has_dec)
            result += digit * powf(0.1f, (float)(i - cursor - dec_pos));
        else
            result = result * 10.0f + (float)digit;
    }

    return minus ? -result : result;
}

//------------------------------------------- 外部接口函数 ------------------------------------------------------------

/**
 * @brief  VOFA+ 初�?�化
 * @note   必须�? wireless_uart_init 之后调用
 */
void vofa_init(void)
{
    fifo_init(&vofa_data_fifo, FIFO_DATA_8BIT, vofa_uart_rx_buf, 64);
    set_wireless_type(WIRELESS_UART, vofa_uart_rx_handler);
}

/**
 * @brief  计算滑动平均�?
 * @param  buf 缓冲区指�?
 * @param  count 有效数据�?�?
 * @return 平均�?
 */
static float vofa_calc_average(const float *buf, uint8 count)
{
    if (count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint8 i = 0; i < count; i++)
    {
        sum += buf[i];
    }
    return sum / (float)count;
}

/**
 * @brief  VOFA+ 数据发送（推荐�? 4ms 调用一次）
 * @param  enc 编码器状态指针，用于获取实时速度
 * @note   4通道内�?�：
 *         通道0：目标速度（m/s�?
 *         通道1：左�?实际速度（m/s�?- 滑动平均滤波�?
 *         通道2：右�?实际速度（m/s�?- 滑动平均滤波�?
 *         通道3：左�? PWM 输出
 */
void vofa_update(const EncoderLayerState *enc)
{
#if VOFA_ENABLE
    // 存入滑动平均缓冲�?
    s_speed_left_buf[s_smooth_index] = enc->speed_left_mps;
    s_speed_right_buf[s_smooth_index] = enc->speed_right_mps;
    s_speed_target_buf[s_smooth_index] = (float)Yao.Target_Speed_L;   // Ŀ���ٶȣ�����/4ms��

    s_smooth_index++;
    if (s_smooth_index >= VOFA_SMOOTH_SIZE) s_smooth_index = 0;
    if (s_smooth_count < VOFA_SMOOTH_SIZE) s_smooth_count++;

    // 计算滑动平均�?
    float speed_left_avg = vofa_calc_average(s_speed_left_buf, s_smooth_count);
    float speed_right_avg = vofa_calc_average(s_speed_right_buf, s_smooth_count);
    float speed_target_avg = vofa_calc_average(s_speed_target_buf, s_smooth_count);

    // 发送平滑后的数�?
//    vofa_send_float(speed_target_avg);                  // 通道0：目标速度
//    vofa_send_float(g_state.speed_left_mps);            // 通道1：左�?实际速度（平滑后�?
//    vofa_send_float(g_state.speed_right_mps);           // 通道2：右�?实际速度（平滑后�?
//    vofa_send_float(g_wheel_pid.out_left_pwm);          // 通道3：左�? PWM�?0~60�?
//    vofa_send_float(g_wheel_pid.out_right_pwm);         // 通道4：右�? PWM�?0~60�?
//    vofa_send_float(g_wheel_pid.pid_spd_left.SumError); // 通道5：左�?�?计积�?
//    vofa_send_float(g_wheel_pid.pid_spd_right.SumError);// 通道6：右�?�?计积�?
    vofa_send_float(Yao_pid.vRB_pid.iError);
    vofa_send_float(Yao.OutP_RB);
    vofa_send_float(g_state.tick_left_4ms);                     // ���� 4ms ����
    vofa_send_float(g_state.tick_right_4ms);                    // ���� 4ms ����
    vofa_send_float(Yao.Target_Speed_L);





    // JustFloat 协�??固定帧尾
    uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f};
    wireless_uart_send_buffer(tail, 4);
#endif
}

/**
 * @brief  VOFA+ 调参处理函数（建�?在主�?�?�?周期调用�?
 * @note   �?持实时修改速度�?和位�?�?�? PID 参数
 */
void vofa_pid_adjust(void)
{
    vofa_fifo_data_count = fifo_used(&vofa_data_fifo);
    if (vofa_fifo_data_count == 0) return;

    fifo_read_buffer(&vofa_data_fifo, vofa_fifo_out_buf, &vofa_fifo_data_count, FIFO_READ_AND_CLEAN);

    float val = vofa_parse_value();

    // P1/K1/D1 �� �ٶȻ� param_vLB/vRB[0]/[1]/[2]��������ͬ���޸ģ�
    // P3/K3/D3 �� λ�û����¼ܹ�����λ�û�������ͨ�����޲�����
    switch (vofa_fifo_out_buf[0])
    {
        case 'P':
            if (vofa_fifo_out_buf[1] == '1')
            {
                param_vLB[KP] = val;   // �ٶȻ� Kp����
                param_vRB[KP] = val;   // �ٶȻ� Kp���ң�
            }
            break;

        case 'K':
            if (vofa_fifo_out_buf[1] == '1')
            {
                param_vLB[KI] = val;   // �ٶȻ� Ki����
                param_vRB[KI] = val;   // �ٶȻ� Ki���ң�
            }
            break;

        case 'D':
            if (vofa_fifo_out_buf[1] == '1')
            {
                param_vLB[KD] = val;   // �ٶȻ� Kd����
                param_vRB[KD] = val;   // �ٶȻ� Kd���ң�
            }
            break;

        default:
            break;
    }
}

/**
 * @brief  无线串口接收�?�?处理函数
 * @note   需要在 isr.c �?注册到�?�应 UART �?�?
 */
void vofa_uart_rx_handler(void)
{
    uart_query_byte(VOFA_UART_INDEX, &vofa_get_data);
    fifo_write_buffer(&vofa_data_fifo, &vofa_get_data, 1);
}
