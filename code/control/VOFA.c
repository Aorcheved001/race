/*
 * vofa.c
 *
 *  VOFA+ 上位机驱动
 *  支持 JustFloat 协议发送 + 上位机指令在线调节速度环/位置环 Kp/Ki/Kd
 *
 *  JustFloat 协议格式：
 *    [4字节 float][4字节 float][...][0x00 0x00 0x80 0x7f]  帧尾为固定值
 *
 *  VOFA+ 指令格式：
 *    P1=xx!   -> 左轮速度环 Kp    P2=xx!   -> 右轮速度环 Kp
 *    K1=xx!   -> 左轮速度环 Ki    K2=xx!   -> 右轮速度环 Ki
 *    D1=xx!   -> 左轮速度环 Kd    D2=xx!   -> 右轮速度环 Kd
 *    P3=xx!   -> 位置环 Kp        K3=xx!   -> 位置环 Ki        D3=xx!   -> 位置环 Kd
 *
 *  Created on: 2026-03-15
 *      Author: DreamerDay
 */

//-------------------------------------------头文件包含------------------------------------------------------------
#include "VOFA.h"

//-------------------------------------------全局变量定义------------------------------------------------------------
uint8  vofa_uart_rx_buf[64];                                            // 串口接收缓冲区
uint8  vofa_fifo_out_buf[64];                                           // FIFO 读出缓冲区
uint8  vofa_get_data = 0;                                                // 串口单字节接收
uint32 vofa_fifo_data_count = 0;                                         // FIFO 数据计数
fifo_struct vofa_data_fifo;                                             // FIFO 结构体

//-------------------------------------------内部函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     将 float 转换为 4 字节小端数据
//  参数说明     val     待转换浮点数
//  参数说明     buf     输出 4 字节数组
//  返回值       void
//  使用示例     vofa_float_to_bytes(1.0f, buf);
//  备注信息     JustFloat 协议使用小端
//-------------------------------------------------------------------------------------------------------------------
static void vofa_float_to_bytes(float val, uint8 buf[4])
{
    uint8 *p = (uint8 *)&val;
    buf[0] = p[0];
    buf[1] = p[1];
    buf[2] = p[2];
    buf[3] = p[3];
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     通过串口发送一个 float（JustFloat 协议）
//  参数说明     val     要发送的浮点数
//  返回值       void
//  使用示例     vofa_send_float(speed);
//-------------------------------------------------------------------------------------------------------------------
static void vofa_send_float(float val)
{
    uint8 byte[4];
    vofa_float_to_bytes(val, byte);
    wireless_uart_send_buffer(byte, 4);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     解析 VOFA+ 下发的指令，格式 =数值!
//  参数说明     void
//  返回值       解析出 float 值，错误返回 0.0f
//  使用示例     float val = vofa_parse_value();
//-------------------------------------------------------------------------------------------------------------------
static float vofa_parse_value(void)
{
    uint8 start_idx = 0;
    uint8 end_idx   = 0;
    uint8 valid     = 0;

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
    uint8 minus  = 0;
    if (cursor <= end_idx && vofa_fifo_out_buf[cursor] == '-')
    {
        minus  = 1;
        cursor++;
    }

    float result    = 0.0f;
    uint8 dec_pos   = 0;
    uint8 has_dec   = 0;

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

//-------------------------------------------外部函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     VOFA+ 初始化（必须先初始化 wireless_uart）
//  参数说明     void
//  返回值       void
//  使用示例     vofa_init();
//  备注信息     绑定 UART2 中断回调，使用 vofa 专用 FIFO
//-------------------------------------------------------------------------------------------------------------------
void vofa_init(void)
{
    fifo_init(&vofa_data_fifo, FIFO_DATA_8BIT, vofa_uart_rx_buf, 64);
    set_wireless_type(WIRELESS_UART, vofa_uart_rx_handler);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     发送一帧 JustFloat 数据，建议每 4ms 调用一次
//  参数说明     enc     编码器状态指针，用于获取实时速度
//  返回值       void
//  使用示例     vofa_update(enc);
//  备注信息     4 通道：目标速度 | 左轮速度 | 右轮速度 | 左轮PWM(归一化)
//-------------------------------------------------------------------------------------------------------------------
void vofa_update(const EncoderLayerState *enc)
{
#if VOFA_ENABLE
    vofa_send_float(g_wheel_pid.cmd_speed_left_mps);         // 通道0：目标速度 m/s
    vofa_send_float(enc->speed_left_mps);              // 通道1：左轮实时速度 m/s
<<<<<<< HEAD
    vofa_send_float(-(enc->speed_right_mps));             // 通道2：右轮实时速度 m/s origin_pidout
    vofa_send_float((g_wheel_pid.out_left_pwm)); // 通道3：左轮PWM归一化
//    vofa_send_float(origin_error_right); // 通道3：左轮PWM归一化
    vofa_send_float(g_wheel_pid.out_right_pwm);

//    vofa_send_float(g_wheel_pid.cmd_speed_left_mps);         // 通道0：目标速度 tick/s
//    vofa_send_float(tick_left_pid);              // 通道1：左轮实时速度 tick/ms
//    vofa_send_float(tick_right_pid);             // 通道2：右轮实时速度 tick/ms
//    vofa_send_float((g_wheel_pid.out_left_pwm / 5000.0f)); // 通道3：左轮PWM归一化
=======
    vofa_send_float(enc->speed_right_mps);             // 通道2：右轮实时速度 m/s
    vofa_send_float(g_wheel_pid.out_left_pwm / 5000.0f); // 通道3：左轮PWM归一化
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6

    // JustFloat 固定帧尾
    uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f};
    wireless_uart_send_buffer(tail, 4);
#endif
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     处理 VOFA+ 下发 PID 参数调节指令
//  参数说明     void
//  返回值       void
//  使用示例     vofa_pid_adjust();
//  备注信息     放在循环中轮询
//               指令格式:
//               P1=xx! - 左轮速度环 Kp    P2=xx! - 右轮速度环 Kp
//               K1=xx! - 左轮速度环 Ki    K2=xx! - 右轮速度环 Ki
//               D1=xx! - 左轮速度环 Kd    D2=xx! - 右轮速度环 Kd
//               P3=xx! - 位置环 Kp        K3=xx! - 位置环 Ki        D3=xx! - 位置环 Kd
//-------------------------------------------------------------------------------------------------------------------
void vofa_pid_adjust(void)
{
    vofa_fifo_data_count = fifo_used(&vofa_data_fifo);
    if (vofa_fifo_data_count == 0) return;

    fifo_read_buffer(&vofa_data_fifo, vofa_fifo_out_buf, &vofa_fifo_data_count, FIFO_READ_AND_CLEAN);

    float val = vofa_parse_value();

    // P1/K1/D1 -> 左轮速度环   P2/K2/D2 -> 右轮速度环
    // P3/K3/D3 -> 位置环
    switch (vofa_fifo_out_buf[0])
    {
        case 'P':
            if (vofa_fifo_out_buf[1] == '1')
                g_wheel_pid.speed_param_left[0]  = val;     // 左轮速度环 Kp
            else if (vofa_fifo_out_buf[1] == '2')
                g_wheel_pid.speed_param_right[0] = val;     // 右轮速度环 Kp
            else if (vofa_fifo_out_buf[1] == '3')
                g_wheel_pid.position_param[0] = val;        // 位置环 Kp
            break;

        case 'K':
            if (vofa_fifo_out_buf[1] == '1')
                g_wheel_pid.speed_param_left[1]  = val;     // 左轮速度环 Ki
            else if (vofa_fifo_out_buf[1] == '2')
                g_wheel_pid.speed_param_right[1] = val;     // 右轮速度环 Ki
            else if (vofa_fifo_out_buf[1] == '3')
                g_wheel_pid.position_param[1] = val;        // 位置环 Ki
            break;

        case 'D':
            if (vofa_fifo_out_buf[1] == '1')
                g_wheel_pid.speed_param_left[2]  = val;     // 左轮速度环 Kd
            else if (vofa_fifo_out_buf[1] == '2')
                g_wheel_pid.speed_param_right[2] = val;     // 右轮速度环 Kd
            else if (vofa_fifo_out_buf[1] == '3')
                g_wheel_pid.position_param[2] = val;        // 位置环 Kd
            break;

        default:
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数功能     UART2 接收中断回调函数，在 isr.c 中自动调用
//  参数说明     void
//  返回值       void
//  使用示例     vofa_uart_rx_handler();
//  备注信息     将收到的字节写入 FIFO
//-------------------------------------------------------------------------------------------------------------------
void vofa_uart_rx_handler(void)
{
    uart_query_byte(VOFA_UART_INDEX, &vofa_get_data);
    fifo_write_buffer(&vofa_data_fifo, &vofa_get_data, 1);
}
