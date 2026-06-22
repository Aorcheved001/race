/*
 * subject_001.h
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

#ifndef CODE_SUBJECT_001_H_
#define CODE_SUBJECT_001_H_

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
void subject1_init(void);                                                 // 科目一初始化
void subject1_update(void);                                               // 科目一周期更新（8ms）
void subject1_display(void);                                              // 科目一显示刷新（40ms）

#endif /* CODE_SUBJECT_001_H_ */
