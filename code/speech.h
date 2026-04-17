/*
 * speech.h
 *
 *  qinease
 *
 */

#ifndef CODE_SPEECH_H_
#define CODE_SPEECH_H_

#include "zf_common_headfile.h"

extern uint8  get_data;                                // 接收数据变量
extern uint8 speech_fifo_get_data[8];                 // fifo 输出读出缓冲区
extern uint8 uart1_speech_get_data[64];                // 串口接收数据缓冲区,显示和暂存

//语音助手           0xFF
//打开左转向灯        0xA0
//打开右转向灯        0xA1
// 语音命令宏定义
#define CMD_WAKEUP           0xFF  // 语音唤醒
#define CMD_LEFT_TURN        0xA0  // 打开左转向灯
#define CMD_RIGHT_TURN       0xA1  // 打开右转向灯
#define CMD_HIGH_BEAM        0xA2  // 打开远光灯
#define CMD_LOW_BEAM         0xA3  // 打开近光灯
#define CMD_FOG_LAMP         0xA4  // 打开雾灯
#define CMD_HAZARD_LAMP      0xA5  // 打开双闪灯
#define CMD_INTERIOR_LAMP    0xA6  // 打开车内照明灯
#define CMD_WIPER            0xA7  // 打开雨刮器
#define CMD_HORN_1S          0xB0  // 鸣笛一秒钟
#define CMD_HORN_2S          0xB1  // 鸣笛两秒钟
#define CMD_HORN_3S          0xB2  // 鸣笛三秒钟
#define CMD_HORN_2BEEP       0xB3  // 鸣笛两声
#define CMD_HORN_3BEEP       0xB4  // 鸣笛三声
#define CMD_HORN_4BEEP       0xB5  // 鸣笛四声
#define CMD_HORN_LONG_SHORT  0xB6  // 长短鸣笛
#define CMD_HORN_URGENT      0xB7  // 急促鸣笛
#define CMD_HORN_ALARM       0xB8  // 警报鸣笛
#define CMD_PASS_LEFT        0xC0  // 通过门洞一左则
#define CMD_PASS_1           0xC1  // 通过门洞一
#define CMD_PASS_2           0xC2  // 通过门洞二
#define CMD_PASS_3           0xC3  // 通过门洞三
#define CMD_PASS_RIGHT       0xC4  // 通过门洞三右侧
#define CMD_RETURN_RIGHT     0xD0  // 门洞一右侧返回
#define CMD_RETURN_1         0xD1  // 门洞一返回
#define CMD_RETURN_2         0xD2  // 门洞二返回
#define CMD_RETURN_3         0xD3  // 门洞三返回
#define CMD_RETURN_LEFT      0xD4  // 门洞三左侧返回
#define CMD_FORWARD_10M      0xE0  // 前行十米
#define CMD_BACKWARD_10M     0xE1  // 后退十米
#define CMD_SNAKE_FORWARD    0xE2  // 蛇形前进十米
#define CMD_SNAKE_BACKWARD   0xE3  // 蛇形后退十米
#define CMD_COUNTERCLOCKWISE 0xE4  // 逆时针转一圈
#define CMD_CLOCKWISE        0xE5  // 顺时针转一圈
#define CMD_TURN_LEFT        0xE6  // 左转
#define CMD_TURN_RIGHT       0xE7  // 右转
#define CMD_END              0xFE  // 结束
#define CMD_CLEAR            0xFD  // 清空

void speech_uart_init(void);
void speech_uart_rx_interrupt_handler (void);
void deal_speech_data(void);
extern uint8  speech_index;
extern uint8 speaker_state;
#endif /* CODE_INIT_H_ */
