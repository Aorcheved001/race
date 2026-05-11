/*
 * vofa.h
 *
 *  VOFA+ 调参驱动
 *  支持 JustFloat 波形发送 + 上位机在线调参（速度环/位置环 Kp/Ki/Kd）
 *
 *  使用方法：
 *    1. 在 wireless_uart_init() 之后调用 vofa_init()
 *    2. 在 4ms 任务中调用 vofa_update() 发送波形
 *    3. 在主循环中调用 vofa_pid_adjust() 解析并下发 PID 参数
 *    4. VOFA+ 协议选 JustFloat，发送格式 P1=0.5! K1=0.1! D1=0.05! 等
 */

#ifndef CODE_DEBUG_VOFA_H_
#define CODE_DEBUG_VOFA_H_

//-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------宏定义区------------------------------------------------------------

#define VOFA_ENABLE             (1)                                    // 0=关闭  1=开启
#define VOFA_UART_INDEX         (WIRELESS_UART_INDEX)                  // 串口通道
#define VOFA_UART_BAUDRATE      (WIRELESS_UART_BUAD_RATE)              // 波特率
#define VOFA_UART_TX_PIN        (WIRELESS_UART_TX_PIN)                 // TX 引脚
#define VOFA_UART_RX_PIN        (WIRELESS_UART_RX_PIN)                 // RX 引脚

//-------------------------------------------内部变量声明区--------------------------------------------------------
extern uint8  vofa_uart_rx_buf[64];                                     // 串口接收缓冲区
extern uint8  vofa_fifo_out_buf[64];                                   // FIFO 输出缓冲区
extern uint8  vofa_get_data;                                           // 单字节接收变量
extern uint32 vofa_fifo_data_count;                                    // FIFO 数据个数
extern fifo_struct vofa_data_fifo;                                     // FIFO 结构体

//-------------------------------------------函数声明区------------------------------------------------------------
void vofa_init(void);                                                   // 初始化（调用前需先完成 wireless_uart_init）
void vofa_update(const EncoderLayerState *enc);                        // 发送波形（每 4ms 调用一次）
void vofa_pid_adjust(void);                                             // 解析 VOFA+ 下发的 PID 指令（主循环调用）
void vofa_uart_rx_handler(void);                                        // UART2 接收中断处理函数

#endif /* CODE_DEBUG_VOFA_H_ */
