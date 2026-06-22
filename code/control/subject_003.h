/*
 * subject_003.h
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 * Description: 科目三状态机 —— 两阶段迷宫任务
 */

#ifndef CODE_SUBJECT_003_H_
#define CODE_SUBJECT_003_H_

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------状态枚举定义------------------------------------------------------------

typedef enum {
    STATE_IDLE = 0,           // 待机状态：系统初始化完成，等待开始指令
    STATE_DRIVE,              // 人工驾驶：车手遥控驾驶车模，同时录制轨迹
    STATE_PARK,               // 到达停车区：第一阶段完成，等待人工归位车头
    STATE_ADJUST,             // 车头归位确认：人工调整车头朝向后确认
    STATE_FOLLOW,             // 自动循迹：第二阶段，自动沿记录路径返回发车区
    STATE_ARRIVE,             // 到达发车区：第二阶段完成
    STATE_ERROR               // 异常状态：循迹异常等
} Subject3_State_t;

 //-------------------------------------------外部变量声明------------------------------------------------------------

extern volatile uint8_t subject3_error_flag;    // 循迹异常标志位（外部可置1，subject3_update()开头统一检查）

 //-------------------------------------------函数声明区------------------------------------------------------------

/**
 * @brief 初始化科目三状态机
 * @note  清零所有状态和标志位，在系统初始化时调用一次
 */
void subject3_init(void);

/**
 * @brief 科目三状态机周期更新（8ms调用）
 * @note  开头检查 subject3_error_flag，若为1则切换到 STATE_ERROR
 *        然后根据当前状态分发到对应 handler
 */
void subject3_update(void);

/**
 * @brief 获取当前状态机状态（用于调试显示）
 * @return 当前状态枚举值
 */
Subject3_State_t subject3_get_state(void);

#endif /* CODE_SUBJECT_003_H_ */
