/*
 * yaokong.h
 *
 * Created on: 2024年9月7日
 * Author: LateRain
 */

#ifndef CODE_CONTROL_YAOKONG_H_
#define CODE_CONTROL_YAOKONG_H_

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------全局变量区------------------------------------------------------------
extern bool  flag_stop;                                                   // 控制标志位，1表示停止，0表示运行
extern int16 speed_yk;                                                    // 遥控速度指令，单位：脉冲/4ms
extern uint8 yaokong_active;                                              // 遥控器激活标志，1表示有输入

 //-------------------------------------------函数声明区------------------------------------------------------------
void  yaokong_init(int16 max_pulse);                                      // 初始化遥控，设置最大脉冲目标
void  yaokong_set_control_enabled(uint8 enabled);                         // 设置遥控是否允许直接下发执行器指令
uint8 yaokong_data_deal(void);                                            // 处理遥控数据，返回处理状态

#endif /* CODE_CONTROL_YAOKONG_H_ */
