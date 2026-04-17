
#include "zf_common_headfile.h"

//fifo_used(&uart_data_fifo)   读取数据数量的函数
//rx接收
#define UART_INDEX              (UART_4   )
#define UART_BAUDRATE           (115200)
#define UART_TX_PIN             (UART4_TX_P00_9  )//PA3  2.2发送
#define UART_RX_PIN             (UART4_RX_P00_12  )//PA2

uint8 uart1_speech_get_data[64];                    // 串口接收数据缓冲区,显示和暂存
uint8 speech_fifo_get_data[8];                     //最终采用的数组

uint8  get_data = 0;
uint32 speech_fifo_data_count = 0;                  // fifo 数据个数
uint8  speech_index = 0;                            // 当前写入位置
uint8 speaker_state = 0;
uint8 copy_speech = 1;

fifo_struct uart1_speech_data_fifo;


//清空则清空串口中所有数据
//头帧必须是 “语音助手”
//尾帧必须是  “结束”
void speech_uart_rx_interrupt_handler (void)
{

//    get_data = uart_read_byte(UART_INDEX);                                    // 接收数据 while 等待式 不建议在中断使用
//    uart_query_byte(UART_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
//    fifo_write_buffer(&uart1_speech_data_fifo, &get_data, 1);                   // 将数据写入 fifo 中
    if(uart_query_byte(UART_INDEX, &get_data))
    {
        if(get_data == CMD_END)
        {
            speaker_state = 0;
            uart1_speech_get_data[9] = CMD_END;
        }
        if(get_data == CMD_CLEAR)
        {
            memset(uart1_speech_get_data, 0, sizeof(uart1_speech_get_data));
            speech_index = 0;
            speaker_state = 0;
            copy_speech == 1;
        }
        // 如果收到 0xFE，就清空数组并从头开始
        if(speaker_state == 1)
        {

            // 正常数据存入数组
            if(speech_index < sizeof(uart1_speech_get_data))
            {
                uart1_speech_get_data[speech_index] = get_data;
                speech_index++;
            }
            else
            {
                // 数组满了，防止越界，清空
                speech_index = 0;
                memset(uart1_speech_get_data, 0, sizeof(uart1_speech_get_data));
            }
        }

        if(get_data == CMD_WAKEUP)
        {
            speaker_state = 1;
            uart1_speech_get_data[0] = CMD_WAKEUP;
            speech_index = 1;
        }

    }
}


//IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
//{
//    interrupt_global_enable(0);                     // 开启中断嵌套
//
//    uart_rx_interrupt_handler();                    // 串口接收处理
//}
//

void speech_uart_init(void)
{

    fifo_init(&uart1_speech_data_fifo, FIFO_DATA_8BIT, uart1_speech_get_data, 64);              // 初始化 fifo 挂载缓冲区

    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
    uart_rx_interrupt(UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断

    //uart_tx_interrupt(UART_4, 1);
}

void deal_speech_data(void)
{
    ips200_show_int(25,16*14,111,4);
    if(uart1_speech_get_data[0] == CMD_WAKEUP && uart1_speech_get_data[9] == CMD_END)
    {
        ips200_show_int(25,16*15,111,4);
        for(int i = 0; i < 8; i++)
            {
               speech_fifo_get_data[i] = uart1_speech_get_data[i+1];
            }
    }

}
