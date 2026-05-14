/*
 * vofa.h
 *
<<<<<<< HEAD
 * VOFA+ 调试参数工具
 * 支持 JustFloat 波形发送 + 上位机在线调参（支持修改 Kp/Ki/Kd 等）
 *
 * 使用方法：
 *   1. 在 wireless_uart_init() 函数中调用 vofa_init()
 *   2. 在 4ms 定时中断中调用 vofa_update() 发送波形
 *   3. 在主循环中调用 vofa_pid_adjust() 解析并更新 PID 参数
 *   4. VOFA+ 上位机指令格式（JustFloat 模式）：
 *      P1=xx!  - 左轮速度环 Kp
 *      P2=xx!  - 右轮速度环 Kp
 *      K1=xx!  - 左轮速度环 Ki
 *      K2=xx!  - 右轮速度环 Ki
 *      D1=xx!  - 左轮速度环 Kd
 *      D2=xx!  - 右轮速度环 Kd
 *      P3=xx!  - 位置环 Kp
 *      K3=xx!  - 位置环 Ki
 *      D3=xx!  - 位置环 Kd
 *      ...（可自行扩展）
=======
 * VOFA+ ??????
 * ?? JustFloat ???? + ???????????? Kp/Ki/Kd ??
 *
 * ?????
 *   1. ? wireless_uart_init() ????? vofa_init()
 *   2. ? 4ms ??????? vofa_update() ????
 *   3. ??????? vofa_pid_adjust() ????? PID ??
 *   4. VOFA+ ????????JustFloat ????
 *      P1=xx!  - ????? Kp
 *      P2=xx!  - ????? Kp
 *      K1=xx!  - ????? Ki
 *      K2=xx!  - ????? Ki
 *      D1=xx!  - ????? Kd
 *      D2=xx!  - ????? Kd
 *      P3=xx!  - ??? Kp
 *      K3=xx!  - ??? Ki
 *      D3=xx!  - ??? Kd
 *      ...???????
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
 */

#ifndef CODE_DEBUG_VOFA_H_
#define CODE_DEBUG_VOFA_H_

<<<<<<< HEAD
//------------------------------------------- 头文件引用 ------------------------------------------------------------

#include "zf_common_headfile.h"

//------------------------------------------- 宏定义 -----------------------------------------------------------------

#define VOFA_ENABLE             (1)                                    // 0=关闭  1=开启

#define VOFA_UART_INDEX         (WIRELESS_UART_INDEX)                  // 使用的串口编号
#define VOFA_UART_BAUDRATE      (WIRELESS_UART_BUAD_RATE)              // 波特率
#define VOFA_UART_TX_PIN        (WIRELESS_UART_TX_PIN)                 // TX 引脚
#define VOFA_UART_RX_PIN        (WIRELESS_UART_RX_PIN)                 // RX 引脚

//------------------------------------------- 外部变量声明 ----------------------------------------------------------

extern uint8  vofa_uart_rx_buf[64];         // UART 接收缓冲区
extern uint8  vofa_fifo_out_buf[64];        // FIFO 输出缓冲区
extern uint8  vofa_get_data;                // 接收到完整数据标志
extern uint32 vofa_fifo_data_count;         // FIFO 中数据个数
extern fifo_struct vofa_data_fifo;          // FIFO 结构体

//------------------------------------------- 函数声明 ---------------------------------------------------------------

void vofa_init(void);                                                   // VOFA 初始化（必须在 wireless_uart_init 之后调用）

void vofa_update(const EncoderLayerState *enc);                        // 发送波形数据（推荐在 4ms 定时器中调用）

void vofa_pid_adjust(void);                                             // 解析 VOFA+ 下发的 PID 调参指令（在主循环中调用）

void vofa_uart_rx_handler(void);                                        // UART 接收中断处理函数

#endif /* CODE_DEBUG_VOFA_H_ */
=======
//------------------------------------------- ????? ------------------------------------------------------------

#include "zf_common_headfile.h"

//------------------------------------------- ??? -----------------------------------------------------------------

#define VOFA_ENABLE             (1)                                    // 0=??  1=??

#define VOFA_UART_INDEX         (WIRELESS_UART_INDEX)                  // ???????
#define VOFA_UART_BAUDRATE      (WIRELESS_UART_BUAD_RATE)              // ???
#define VOFA_UART_TX_PIN        (WIRELESS_UART_TX_PIN)                 // TX ??
#define VOFA_UART_RX_PIN        (WIRELESS_UART_RX_PIN)                 // RX ??

//------------------------------------------- ?????? ----------------------------------------------------------

extern uint8  vofa_uart_rx_buf[64];         // UART ?????
extern uint8  vofa_fifo_out_buf[64];        // FIFO ?????
extern uint8  vofa_get_data;                // ?????????
extern uint32 vofa_fifo_data_count;         // FIFO ?????
extern fifo_struct vofa_data_fifo;          // FIFO ???

//------------------------------------------- ???? ---------------------------------------------------------------

void vofa_init(void);                                                   // VOFA ??????? wireless_uart_init ?????

void vofa_update(const EncoderLayerState *enc);                        // ?????????? 4ms ???????

void vofa_pid_adjust(void);                                             // ?? VOFA+ ??? PID ?????????????

void vofa_uart_rx_handler(void);                                        // UART ????????

#endif /* CODE_DEBUG_VOFA_H_ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
