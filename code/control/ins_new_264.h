/*
 * ins_new_264.h
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

#ifndef CODE_INS_NEW_264_H_
#define CODE_INS_NEW_264_H_

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------宏定义区------------------------------------------------------------

 //-------------------------------------------结构体定义区------------------------------------------------------------
typedef struct {
    float x;                                                              // X坐标
    float y;                                                              // Y坐标
} INS_Point;

typedef enum {
    INS_SUB_MODE_SAVE = 0,                                                // 存点模式
    INS_SUB_MODE_FOLLOW = 1                                               // 循迹模式
} INS_SubMode_t;

typedef struct {
    INS_Point       cod_RealTime;                                         // 实时坐标
    float           Dis_ins;                                              // 到目标点距离
    float           Yaw_ins;                                              // 航向角（角度）
    uint8           ins_active;                                           // INS激活标志
    INS_SubMode_t   sub_mode;                                             // 当前子模式
} INS_DataStruct;

 //-------------------------------------------全局变量声明区------------------------------------------------------------
extern INS_DataStruct INS;                                                // INS数据结构体

 //-------------------------------------------函数声明区------------------------------------------------------------
void INS_init(void);                                                      // INS模块初始化
void INS_NavigationTask(void);                                            // INS导航任务，处理按键和状态切换
void INS_Display(void);                                                   // INS显示任务，刷新屏幕

#endif /* CODE_INS_NEW_264_H_ */
