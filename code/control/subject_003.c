/*
 * subject_003.c
 *
 * Created on: 2025年5月12日
 * Author: Daydreamer
 * Description: 科目三状态机实现 —— 两阶段迷宫任务
 *              第一阶段：人工驾驶 + 轨迹录制（DRIVE → PARK）
 *              第二阶段：车头归位 + 自动循迹返回（ADJUST → FOLLOW → ARRIVE）
 */

 //-------------------------------------------头文件引用------------------------------------------------------------
#include "subject_003.h"
#include "zf_common_headfile.h"
#include "Ins.h"
#include "encoder.h"
#include "track.h"
#include "PID.h"
#include "steering_control.h"
#include "Motor.h"
#include "menu.h"

 //-------------------------------------------全局变量定义------------------------------------------------------------

volatile uint8_t subject3_error_flag = 0;       // 循迹异常标志位（外部可置1）

 //-------------------------------------------内部变量定义------------------------------------------------------------

static Subject3_State_t s_state = STATE_IDLE;   // 当前状态

static float s_park_yaw = 0.0f;                 // 停车时航向角（旋转前），用于yaw偏移补偿
static float s_yaw_offset = 0.0f;               // yaw偏移量（旋转后 - 旋转前），循迹时补偿用

 //-------------------------------------------内部函数前向声明------------------------------------------------------------

static void state_idle_handler(void);
static void state_drive_handler(void);
static void state_park_handler(void);
static void state_adjust_handler(void);
static void state_follow_handler(void);
static void state_arrive_handler(void);
static void state_error_handler(void);

 //-------------------------------------------状态转移辅助函数------------------------------------------------------------

/**
 * @brief 切换状态并执行进入/退出动作
 */
static void transition_to(Subject3_State_t new_state)
{
    /* ====== 退出当前状态的清理动作 ====== */
    switch (s_state)
    {
        case STATE_DRIVE:       
            // 退出人工驾驶：停止录制
            track_stop_save();
            break;

        case STATE_FOLLOW:
            // 退出自动循迹：停止循迹，PID速度归零，转向回中
            track_stop_follow();
            wheel_pid_set_target_speed(0.0f);
            steering_set_target(0.0f);
            break;

        default:
            break;
    }

    s_state = new_state;

    /* ====== 进入新状态的初始化动作 ====== */
    const INS_State* ins_state = NULL;

    switch (new_state)
    {
        case STATE_IDLE:
            // 待机：初始化轨迹模块和INS核心
            track_init();
            Ins_reset(0.0f, 0.0f, 0.0f);
            encoder_layer_clear_odom();
            encoder_layer_clear_raw_counts();
            wheel_pid_enable(0, 0, 0, 0);  // 关闭PID输出
            break;

        case STATE_DRIVE:
            // 人工驾驶：清空旧轨迹，开始录制
            track_flash_clear_all();
            track_start_save();
            break;

        case STATE_PARK:
            // 到达停车区：记录旋转前航向
            ins_state = Ins_get_state();
            if (ins_state != NULL)
            {
                s_park_yaw = ins_state->yaw;
            }
            break;

        case STATE_ADJUST:
            // 车头归位确认：记录旋转后航向，计算yaw偏移
            ins_state = Ins_get_state();
            if (ins_state != NULL)
            {
                s_yaw_offset = ins_state->yaw - s_park_yaw;
            }
            break;

        case STATE_FOLLOW:
            // 自动循迹：设置yaw偏移补偿，开始反向循迹，使能PID
            track_set_yaw_offset(s_yaw_offset);
            track_start_follow_reverse();
            wheel_pid_enable(1, 1, 1, 0);  // 使能PID（位置+速度），禁用转向环
            break;

        case STATE_ARRIVE:
            // 到达发车区：停止循迹，停车
            track_stop_follow();
            wheel_pid_set_target_speed(0.0f);
            steering_set_target(0.0f);
            break;

        case STATE_ERROR:
            // 异常状态：PID速度归零，转向回中，停止录制和循迹
            wheel_pid_set_target_speed(0.0f);
            steering_set_target(0.0f);
            track_stop_save();
            track_stop_follow();
            break;

        default:
            break;
    }
}

 //-------------------------------------------各状态处理函数------------------------------------------------------------

/**
 * @brief IDLE 待机状态处理
 *        周期执行：显示待机界面，检测K4长按启动
 */
static void state_idle_handler(void)
{
    // 周期执行：等待K4长按启动

    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        transition_to(STATE_DRIVE);
    }
}

/**
 * @brief DRIVE 人工驾驶 + 轨迹录制处理
 *        周期执行：遥控器控制，轨迹由4ms中断自动录制
 */
static void state_drive_handler(void)
{
    // K1短按 → 到达停车区
    if (key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        transition_to(STATE_PARK);
        return;
    }

    // K4长按 → 紧急复位
    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        transition_to(STATE_IDLE);
        return;
    }
}

/**
 * @brief PARK 到达停车区处理
 *        周期执行：等待人工归位车头，显示状态
 */
static void state_park_handler(void)
{
    // K1短按 → 确认车头归位
    if (key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        transition_to(STATE_ADJUST);
        return;
    }

    // K4长按 → 紧急复位
    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        transition_to(STATE_IDLE);
        return;
    }
}

/**
 * @brief ADJUST 车头归位确认处理
 *        周期执行：显示航向角、yaw偏移量
 */
static void state_adjust_handler(void)
{
    // K1短按 → 开始自动循迹
    if (key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        transition_to(STATE_FOLLOW);
        return;
    }

    // K4长按 → 紧急复位
    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        transition_to(STATE_IDLE);
        return;
    }
}

/**
 * @brief FOLLOW 自动循迹处理
 *        周期执行：循迹逻辑由4ms中断中的track_proc()自动执行
 *        此处仅检测退出条件
 */
static void state_follow_handler(void)
{
    // K4长按 → 紧急复位
    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        transition_to(STATE_IDLE);
        return;
    }

    // 检测循迹是否到达终点（反向遍历时 follow_abs_index 递减到 1 后停止）
    // track_follow_reverse_flag 由 track_stop_follow() 清零
    if (track_follow_reverse_flag == 0)
    {
        // 循迹已自动停止（到达终点）
        transition_to(STATE_ARRIVE);
        return;
    }
}

/**
 * @brief ARRIVE 到达发车区处理
 *        周期执行：显示任务完成信息
 */
static void state_arrive_handler(void)
{
    // K4长按 → 回到待机
    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        transition_to(STATE_IDLE);
        return;
    }
}

/**
 * @brief ERROR 异常状态处理
 *        持续保持停车，等待人工干预
 */
static void state_error_handler(void)
{
    // 持续输出停车指令（防止PID积分漂移导致意外运动）
    wheel_pid_set_target_speed(0.0f);
    steering_set_target(0.0f);

    // K4长按 → 回到待机
    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        transition_to(STATE_IDLE);
        return;
    }
}

 //-------------------------------------------外部接口实现------------------------------------------------------------

/**
 * @brief 初始化科目三状态机
 */
void subject3_init(void)
{
    s_state = STATE_IDLE;
    subject3_error_flag = 0;
    s_park_yaw = 0.0f;
    s_yaw_offset = 0.0f;
}

/**
 * @brief 科目三状态机周期更新（8ms调用）
 *        1. 检查异常标志位
 *        2. 扫描按键
 *        3. 分发到对应状态 handler
 */
void subject3_update(void)
{
    /* ====== 全局异常检查 ====== */
    if (subject3_error_flag && s_state != STATE_IDLE && s_state != STATE_ERROR)
    {
        transition_to(STATE_ERROR);
        subject3_error_flag = 0;  // 清零标志位
        return;                   // 本周期不再执行后续逻辑
    }

    /* ====== 按状态分发 ====== */
    // 注意：key_scanner() 由 1ms 周期独立调用，此处仅调用 key_detect() 读取状态
    switch (s_state)
    {
        case STATE_IDLE:    state_idle_handler();    break;
        case STATE_DRIVE:   state_drive_handler();   break;
        case STATE_PARK:    state_park_handler();    break;
        case STATE_ADJUST:  state_adjust_handler();  break;
        case STATE_FOLLOW:  state_follow_handler();  break;
        case STATE_ARRIVE:  state_arrive_handler();  break;
        case STATE_ERROR:   state_error_handler();   break;
        default:            s_state = STATE_IDLE;    break;
    }
}

/**
 * @brief 获取当前状态机状态
 */
Subject3_State_t subject3_get_state(void)
{
    return s_state;
}

