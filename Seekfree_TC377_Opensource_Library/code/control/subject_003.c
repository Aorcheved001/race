/*
 * subject_003.c
 *
 * Created on: 2025年5月12日
 * Author: Daydreamer
 * Description: 科目三状态机实现 —— 两阶段迷宫任务
 *              第一阶段：人工驾驶 + 轨迹录制（DRIVE）
 *              第二阶段：遥控调头 + 自动循迹返回（CHANGE → FOLLOW → ARRIVE）
 */

 //-------------------------------------------头文件引用------------------------------------------------------------
#include "subject_003.h"
#include "zf_common_headfile.h"
#include "Ins.h"
#include "mag_yaw_lut.h"
#include "encoder.h"
#include "track.h"
#include "PID.h"
#include "steering_control.h"
#include "MOTOR.h"
#include "menu.h"
#include "zf_device_wireless_uart.h"
#include <stdio.h>

 //-------------------------------------------全局变量定义------------------------------------------------------------

volatile uint8_t subject3_error_flag = 0;       // 循迹异常标志位（外部可置1）

 //-------------------------------------------内部变量定义------------------------------------------------------------

static Subject3_State_t s_state = STATE_IDLE;   // 当前状态
static uint8_t s_active = 0;                    // 科目三激活标志（K4长按后置1，回到IDLE清零）

static float s_park_yaw = 0.0f;                 // 停车时航向角（旋转前），用于yaw偏移补偿
static float s_yaw_offset = 0.0f;               // yaw偏移量（旋转后 - 旋转前），循迹时补偿用
static uint8_t s_change_k1_calib = 0u;          // CHANGE阶段K1校准状态: 0=等待K1, 1=校准中, 2=完成待跳转

// YAW_CORRECT 阶段变量
static float s_yaw_correct_start_yaw = 0.0f;    // 矫正阶段开始时的 gyro yaw
static float s_yaw_correct_target = 0.0f;       // 矫正目标航向（来自磁力计/RTK）
static uint8_t s_yaw_correct_converge_count = 0u; // 连续收敛帧计数
static uint8_t s_yaw_correct_active = 0u;       // 矫正融合是否激活
static uint16_t s_yaw_correct_frame_count = 0u; // YAW_CORRECT阶段累计帧数
static float s_yaw_correct_err_sum = 0.0f;     // yaw误差累计
static float s_yaw_correct_err_sq_sum = 0.0f;  // yaw误差平方累计

 //-------------------------------------------内部函数前向声明------------------------------------------------------------

static void state_idle_handler(void);
static void state_drive_handler(void);
static void state_change_handler(void);
static void state_yaw_correct_handler(void);
static void state_follow_handler(void);
static void state_arrive_handler(void);
static void state_error_handler(void);
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

        case STATE_YAW_CORRECT:
            Ins_set_mag_fusion_enabled(0u);
            break;

        case STATE_FOLLOW:
            // 退出自动循迹：停止循迹，PID速度归零，转向回中
            track_stop_follow();
            wheel_pid_set_target_speed(0);
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
            // 待机：初始化轨迹模块和INS核心，等待K4激活显示
            track_init();
            Ins_set_mag_fusion_enabled(0u);
            Ins_reset(0.0f, 0.0f, 0.0f);
            encoder_layer_clear_odom();
            encoder_layer_clear_raw_counts();
            Yao.flag_motor_start_user = 0;  // 关闭电机输出
            break;

        case STATE_DRIVE:
            // 人工驾驶：清空旧轨迹，上电自动校准零偏，开始录制
            Ins_set_mag_fusion_enabled(0u);
            track_flash_clear_all();
            Ins_start_bias_calibration();  // 上电自动采集静止零偏（需车停稳2秒）
            s_change_k1_calib = 0u;        // 复位CHANGE校准标志
            track_start_save();
            break;

        case STATE_CHANGE:
            // 遥控调头：记录park_yaw，等待K1触发校准
            Ins_set_mag_fusion_enabled(0u);
            s_change_k1_calib = 0u;        // 等待用户调好方向后按K1
            ins_state = Ins_get_state();
            if (ins_state != NULL)
            {
                s_park_yaw = ins_state->yaw;
            }
            break;

        case STATE_YAW_CORRECT:
            // 航向矫正：车辆静止，用LUT修正后的磁航向纠正陀螺漂移
            s_yaw_correct_start_yaw = Ins_get_state()->yaw;
            s_yaw_correct_target = Ins_get_mag_yaw_corr_rel();  // LUT修正后的磁航向
            s_yaw_correct_converge_count = 0u;
            s_yaw_correct_frame_count = 0u;
            s_yaw_correct_err_sum = 0.0f;
            s_yaw_correct_err_sq_sum = 0.0f;
            s_yaw_correct_active = 1u;
            Ins_set_mag_fusion_enabled(1u);
            break;

        case STATE_FOLLOW:
            Ins_set_mag_fusion_enabled(0u);
            // 自动循迹：计算yaw偏移，开始反向循迹
            // 注意：不重置INS坐标！车的当前位置已在轨迹终点附近
            ins_state = Ins_get_state();
            if (ins_state != NULL)
            {
                s_yaw_offset = ins_state->yaw - s_park_yaw;
            }
            track_set_yaw_offset(s_yaw_offset);
            track_start_follow_reverse();
            Yao.flag_motor_start_user = 1;
            break;

        case STATE_ARRIVE:
            // 到达发车区：停止循迹，停车
            track_stop_follow();
            wheel_pid_set_target_speed(0);
            steering_set_target(0.0f);
            break;

        case STATE_ERROR:
            // 异常状态：PID速度归零，转向回中，停止录制和循迹
            wheel_pid_set_target_speed(0);
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
    if (s_active == 0)
    {
        // 未激活：等待K4长按激活科目三
        if (key_detect(KEY_4, KEY_LONG_PRESS))
        {
            s_active = 1;  // 激活显示，但仍在IDLE
        }
    }
    else
    {
        // 已激活：等待K1短按开始录制
        if (key_detect(KEY_1, KEY_SHORT_PRESS))
        {
            transition_to(STATE_DRIVE);
            return;
        }
        // K4长按 → 退出科目三
        if (key_detect(KEY_4, KEY_LONG_PRESS))
        {
            s_active = 0;
        }
    }
}

/**
 * @brief DRIVE 人工驾驶 + 轨迹录制处理
 *        周期执行：遥控器控制，轨迹由4ms中断自动录制
 */
static void state_drive_handler(void)
{
    // K1短按 → 进入遥控调头阶段
    if (key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        transition_to(STATE_CHANGE);
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
 * @brief CHANGE 遥控调头处理
 *        周期执行：遥控器调转车头方向，INS持续运行，坐标实时发送
 */
static void state_change_handler(void)
{
    // 推进静止零偏校准（仅当校准已激活时）
    if (s_change_k1_calib == 1u)
    {
        const INS_State* ins_state = Ins_get_state();
        uint8_t is_stationary = 0u;
        if (ins_state != NULL)
        {
            const EncoderLayerState* enc = encoder_layer_get_state();
            if (enc != NULL && fabsf(enc->speed_average_mps) < 0.05f
                && fabsf(imu660.data_Ripen.gyro_z) < 0.05f)
            {
                is_stationary = 1u;
            }
        }
        if (Ins_update_bias_calibration(0.008f, is_stationary))
        {
            // 校准完成！跳转YAW_CORRECT（而非直接FOLLOW）
            s_change_k1_calib = 2u;
            transition_to(STATE_YAW_CORRECT);
            return;
        }
    }

    // K1短按 → 调好方向后触发零偏校准（车停稳2秒后自动跳转FOLLOW）
    if (key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        if (s_change_k1_calib == 0u)
        {
            // 首次K1：开始零偏校准
            Ins_start_bias_calibration();
            s_change_k1_calib = 1u;
            // 不跳转，等待校准完成
        }
        // 校准中/已完成时不响应K1
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
 * @brief YAW_CORRECT 航向矫正处理
 *        车辆静止，使用LUT修正后的磁航向纠正陀螺漂移
 *        收敛逻辑：当 gyro_yaw 与 mag_yaw_corr_rel 差值连续 N 帧 < 阈值时，
 *        调用 Ins_sync_yaw() 同步所有 yaw 状态，然后跳转 FOLLOW
 */
static void state_yaw_correct_handler(void)
{
    // 获取当前融合航向和LUT修正后的磁航向
    float yaw_fused = Ins_get_state()->yaw;
    float mag_yaw_corr = Ins_get_mag_yaw_corr_rel();

    // 计算航向差（弧度）
    float yaw_error = mag_yaw_corr - yaw_fused;
    while (yaw_error > 3.14159265f)  yaw_error -= 2.0f * 3.14159265f;
    while (yaw_error < -3.14159265f) yaw_error += 2.0f * 3.14159265f;

    // subject3_update 约8ms调用：250帧约2s；125帧约1s。
    const uint16_t MIN_STABLE_FRAMES = 250u;
    const uint8_t CONVERGE_COUNT_REQUIRED = 125u;
    const float CONVERGE_THRESHOLD_RAD = 3.0f * 0.017453292519943f;
    const float ERR_STD_THRESHOLD_RAD = 1.5f * 0.017453292519943f;
    const float PULL_RATE = 0.02f;

    s_yaw_correct_frame_count++;
    s_yaw_correct_err_sum += yaw_error;
    s_yaw_correct_err_sq_sum += yaw_error * yaw_error;

    {
        float adjusted_yaw = yaw_fused + PULL_RATE * yaw_error;
        while (adjusted_yaw > 3.14159265f)  adjusted_yaw -= 2.0f * 3.14159265f;
        while (adjusted_yaw < -3.14159265f) adjusted_yaw += 2.0f * 3.14159265f;
        Ins_sync_yaw(adjusted_yaw);
    }

    if (fabsf(yaw_error) < CONVERGE_THRESHOLD_RAD)
    {
        if (s_yaw_correct_converge_count < 255u)
        {
            s_yaw_correct_converge_count++;
        }
    }
    else
    {
        s_yaw_correct_converge_count = 0u;
    }

    if (s_yaw_correct_frame_count >= MIN_STABLE_FRAMES &&
        s_yaw_correct_converge_count >= CONVERGE_COUNT_REQUIRED)
    {
        float mean = s_yaw_correct_err_sum / (float)s_yaw_correct_frame_count;
        float var = s_yaw_correct_err_sq_sum / (float)s_yaw_correct_frame_count - mean * mean;
        float std = 0.0f;
        if (var > 0.0f)
        {
            std = sqrtf(var);
        }

        if (std < ERR_STD_THRESHOLD_RAD)
        {
            // 收敛完成：同步到当前融合航向，再关闭磁融合进入FOLLOW。
            Ins_sync_yaw(Ins_get_state()->yaw);
            Ins_set_mag_fusion_enabled(0u);
            transition_to(STATE_FOLLOW);
            return;
        }
    }

    // K4长按 → 紧急复位（放弃矫正）
    if (key_detect(KEY_4, KEY_LONG_PRESS))
    {
        Ins_set_mag_fusion_enabled(0u);
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
    wheel_pid_set_target_speed(0);
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
    s_active = 0;           // 上电后不显示，K4长按激活
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
        case STATE_IDLE:        state_idle_handler();        break;
        case STATE_DRIVE:       state_drive_handler();       break;
        case STATE_CHANGE:      state_change_handler();      break;
        case STATE_YAW_CORRECT: state_yaw_correct_handler(); break;
        case STATE_FOLLOW:      state_follow_handler();      break;
        case STATE_ARRIVE:      state_arrive_handler();      break;
        case STATE_ERROR:       state_error_handler();       break;
        default:                s_state = STATE_IDLE;        break;
    }
}

//-------------------------------------------状态名查找表------------------------------------------------------------

static const char* s_state_names[] =
{
    "IDLE ",    // STATE_IDLE
    "DRIVE",    // STATE_DRIVE
    "CHANG",    // STATE_CHANGE
    "YAWCR",    // STATE_YAW_CORRECT
    "FOLLO",    // STATE_FOLLOW
    "ARRIV",    // STATE_ARRIVE
    "ERROR"     // STATE_ERROR
};

 //-------------------------------------------显示函数实现------------------------------------------------------------

/**
 * @brief 科目三显示函数
 * @note  40ms周期调用，刷新屏幕显示
 *        显示当前状态、INS信息、航向角、编码器速度等
 */
void subject3_display(void)
{
    if (s_active == 0)  // 未激活时不清屏（让科目一正常显示）
        return;

    const INS_State* ins_state = Ins_get_state();
    const EncoderLayerState* enc = encoder_layer_get_state();

    ips200_show_string(0, 0, "Subject3 ");

    // GPS 状态
    const gnss_state_t* gnss = gnss_get_state();
    if (gnss != NULL && gnss->fix_type > 0)
        ips200_show_string(100, 0, "GPS:OK");
    else
        ips200_show_string(100, 0, "GPS:NO");

    // 当前状态名（标题栏右侧）
    if (s_state <= STATE_ERROR)
        ips200_show_string(152, 0, s_state_names[s_state]);

    // ---- 第2行：多层 yaw ----
    float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
    Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

    ips200_show_string(0, 16, "Gyro:");
    ips200_show_float(40, 16, yaw_gyro * 57.29578f, 3, 1);

    ips200_show_string(0, 32, "M_Raw:");
    ips200_show_float(48, 32, yaw_mag_raw * 57.29578f, 3, 1);
    ips200_show_string(105, 32, "M_Rel:");
    ips200_show_float(155, 32, yaw_mag_rel * 57.29578f, 3, 1);

    ips200_show_string(0, 48, "Fin Yaw:");
    ips200_show_float(80, 48, yaw_ekf * 57.29578f, 3, 2);
    ips200_show_string(140, 48, "deg");

    // CHANGE状态提示
    if (s_state == STATE_CHANGE)
        ips200_show_string(140, 16, "CHANG");

    // ---- 第6~7行：按状态显示不同信息 ----
    switch (s_state)
    {
        case STATE_IDLE:
            ips200_show_string(0, 64, "Ready to record        ");
            ips200_show_string(0, 80, "                              ");
            ips200_show_string(0, 96, "                              ");
            ips200_show_string(0, 112, "K1:Record  K4:Exit          ");
            break;

        case STATE_DRIVE:
            if (ins_state != NULL)
            {
                ips200_show_string(0, 64, "X:");
                ips200_show_float(16, 64, ins_state->x, 3, 2);
                ips200_show_string(80, 64, "Y:");
                ips200_show_float(96, 64, ins_state->y, 3, 2);
            }
            ips200_show_string(0, 80, "Pts:");
            ips200_show_uint(32, 80, track_total_points, 4);
            if (enc != NULL)
            {
                ips200_show_string(80, 80, "Spd:");
                ips200_show_float(112, 80, enc->speed_average_mps, 1, 2);
                ips200_show_string(152, 80, "m/s");
            }
            ips200_show_string(0, 112, "K1:Park    K4:Reset         ");
            break;

        case STATE_CHANGE:
            if (ins_state != NULL)
            {
                float cur_yaw_deg = ins_state->yaw * 57.29578f;
                ips200_show_string(0, 64, "Yaw:");
                ips200_show_float(32, 64, cur_yaw_deg, 3, 2);
                ips200_show_string(95, 64, "deg");
                ips200_show_string(0, 80, "X:");
                ips200_show_float(16, 80, ins_state->x, 3, 2);
                ips200_show_string(80, 80, "Y:");
                ips200_show_float(96, 80, ins_state->y, 3, 2);
            }
            ips200_show_string(0, 96, "RC: Rotate car head     ");
            ips200_show_string(0, 112, "K1:Follow  K4:Reset         ");
            break;

        case STATE_YAW_CORRECT:
            if (ins_state != NULL)
            {
                float cur_yaw_deg = ins_state->yaw * 57.29578f;
                float corr_deg = Ins_get_mag_yaw_corr_rel() * 57.29578f;
                ips200_show_string(0, 64, "Gyro:");
                ips200_show_float(40, 64, cur_yaw_deg, 3, 2);
                ips200_show_string(95, 64, "deg");
                ips200_show_string(0, 80, "MagC:");
                ips200_show_float(40, 80, corr_deg, 3, 2);
                ips200_show_string(95, 80, "deg");
                ips200_show_string(0, 96, "Conv:");
                ips200_show_uint(40, 96, s_yaw_correct_converge_count, 3);
                ips200_show_string(60, 96, "/125");
            }
            ips200_show_string(0, 112, "Correcting...  K4:Reset     ");
            break;

        case STATE_FOLLOW:
            if (ins_state != NULL)
            {
                ips200_show_string(0, 64, "Tgt X:");
                ips200_show_float(48, 64, Ins_date_377.x, 3, 2);
                ips200_show_string(110, 64, "Y:");
                ips200_show_float(128, 64, Ins_date_377.y, 3, 2);
            }
            ips200_show_string(0, 80, "Idx:");
            ips200_show_uint(32, 80, track_get_follow_index(), 4);
            ips200_show_string(72, 80, "/");
            ips200_show_uint(80, 80, track_total_points, 4);
            if (enc != NULL)
            {
                ips200_show_string(0, 96, "Spd:");
                ips200_show_float(32, 96, enc->speed_average_mps, 1, 2);
                ips200_show_string(72, 96, "m/s");
            }
            ips200_show_string(0, 112, "K4:Reset                     ");
            break;

        case STATE_ARRIVE:
            ips200_show_string(0, 64, "Mission Complete!      ");
            ips200_show_string(0, 80, "                              ");
            ips200_show_string(0, 96, "                              ");
            ips200_show_string(0, 112, "K4:IDLE                      ");
            break;

        case STATE_ERROR:
            ips200_show_string(0, 64, "!!! TRACK ERROR !!!    ");
            ips200_show_string(0, 80, "                              ");
            ips200_show_string(0, 96, "                              ");
            ips200_show_string(0, 112, "K4:Reset                     ");
            break;

        default:
            break;
    }

    // 磁力计校准覆盖显示
    if (imu_mag_calib_is_active())
    {
        ips200_show_string(150, 96, "CAL");
        ips200_show_string(0, 80, "Mag Samples:");
        ips200_show_uint(88, 80, imu_mag_calib_get_sample_count(), 3);
        ips200_show_string(120, 80, "/500");
    }
}

/**
 * @brief 获取当前状态机状态
 */
Subject3_State_t subject3_get_state(void)
{
    return s_state;
}

/**
 * @brief 查询当前是否处于遥控调头阶段
 * @return 1=CHANGE状态，0=其他
 */
uint8_t subject3_is_in_change(void)
{
    return (s_state == STATE_CHANGE || s_state == STATE_YAW_CORRECT) ? 1u : 0u;
}

