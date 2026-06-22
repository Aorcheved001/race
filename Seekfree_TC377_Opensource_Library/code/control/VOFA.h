/*
 * VOFA.h
 *
 *  VOFA+ 调试协议
 *  支持 JustFloat 协议发送 + 上位机在线调参（速度环/位置环 Kp/Ki/Kd）
 *
 *  使用方法：
 *    1. 在 wireless_uart_init() 之后调用 vofa_init()
 *    3. 在主循环中调用 vofa_pid_adjust() 处理上位机 PID 参数
 *    2. 在 4ms 任务中调用 vofa_update() 发送数据
 *    4. VOFA+ 协议选 JustFloat，发送格式 P1=0.5! K1=0.1! D1=0.05! 等
 */

#ifndef CODE_CONTROL_VOFA_H_
#define CODE_CONTROL_VOFA_H_

//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------宏定义----------------------------------------------------------------

#define VOFA_ENABLE             (1)                                    // 0=关闭  1=开启
#define VOFA_UART_INDEX         (WIRELESS_UART_INDEX)                  // 串口通道
#define VOFA_UART_BAUDRATE      (WIRELESS_UART_BUAD_RATE)              // 波特率
#define VOFA_UART_TX_PIN        (WIRELESS_UART_TX_PIN)                 // TX 引脚
#define VOFA_UART_RX_PIN        (WIRELESS_UART_RX_PIN)                 // RX 引脚

//-------------------------------------------内部变量声明------------------------------------------------------------
extern uint8  vofa_uart_rx_buf[64];                                     // 串口接收缓冲区
extern uint8  vofa_fifo_out_buf[64];                                   // FIFO 输出缓冲区
extern uint8  vofa_get_data;                                           // 单字节接收变量
extern uint32 vofa_fifo_data_count;                                    // FIFO 数据个数
extern fifo_struct vofa_data_fifo;                                     // FIFO 结构体

//-------------------------------------------函数声明---------------------------------------------------------------
void vofa_init(void);                                                   // 初始化（调用前先初始化 wireless_uart）
void vofa_update(const EncoderLayerState *enc);                        // 发送数据（每 4ms 调用一次）
void vofa_pid_adjust(void);                                             // 处理 VOFA+ 上位机 PID 指令（主循环调用）
void vofa_uart_rx_handler(void);                                        // UART 接收中断处理函数

#endif /* CODE_CONTROL_VOFA_H_ */
