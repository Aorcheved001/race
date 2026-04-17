#ifndef USER_INTERRUPT_SCHEDULER_H_
#define USER_INTERRUPT_SCHEDULER_H_
 //-------------------------------------------头文件声明区------------------------------------------------------------

#include "zf_common_headfile.h"

 //-------------------------------------------变量声明区------------------------------------------------------------

extern volatile uint8 g_key_scan_flag;

 //-------------------------------------------变量声明区------------------------------------------------------------

void Interrupt_1ms(void);                       // 1ms 中断服务函数

void Interrupt_2ms(void);                       // 2ms 中断服务函数

void Interrupt_4ms(void);                       // 4ms 中断服务函数

void Interrupt_8ms(void);                       // 8ms 中断服务函数

void Interrupt_16ms(void);                      // 16ms 中断服务函数

void Interrupt_40ms(void);                      // 40ms 中断服务函数

void InterruptTasks_Poll(void);                 // 中断任务轮询函数

#endif /* USER_INTERRUPT_SCHEDULER_H_ */
