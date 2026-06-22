/*
 * steering_control.c
 *
 *  Created on: 2026年4月13日
 *      Author: A
 *  Description: 转向控制 —— 绝对编码器 + PD + 陀螺仪前馈
 *               HOME 为固定机械原位，上电不变
 *               左负右正，PID 按 (目标 - 当前) 误差驱动 PWM
 */

#include "zf_common_headfile.h"
#include "steering_control.h"
#include "encoder.h"

//-------------------------------------------内部变量------------------------------------------------------------
static float g_target_angle = 0.0f;             // 目标角度（度）
static float g_last_error  = 0.0f;             // 上一次误差，用于微分
static SteeringPidDebug g_steer_debug = {0};   // 转向PID调试快照

// 差速转向助力变量（陀螺仪角速率闭环）
static float g_diff_delta = 0.0f;                 // 差速偏移量（脉冲/4ms，浮点），左轮+此值，右轮-此值
static float g_diff_last_error = 0.0f;          // 差速PID上次误差（角速率误差，rad/s）
static float g_diff_integral = 0.0f;            // 差速PID积分累计
static uint8  g_diff_enabled = 1;                // 差速助力开关（默认启用）
static float g_diff_speed_dir = 1.0f;           // 行驶方向：1.0=前进，-1.0=倒车
static float g_diff_curvature_ff = 0.0f;        // 路径曲率前馈（1/m），由track模块设置
static float g_diff_ff_delta = 0.0f;            // 前馈差速偏移量（脉冲/4ms）

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取当前转向角度（相对机械原位）
//  @return     角度（度），0=原位，左负右正
//  @note       中位=710，左转极限=1667，右转极限=3656（跨零点）
//              左转：raw增大 (710→1667)，diff>0，角度为负
//              右转：raw减小 (710→0→4096→3656)，diff<0，角度为正
//-------------------------------------------------------------------------------------------------------------------
float steering_get_current_angle(void)
{
    int16 raw = absolute_encoder_get_location();
    int32 diff = (int32)raw - (int32)STEER_HOME_RAW;

    // 跨越零点：超过半圈视为越界，反向修补一圈
    if(diff > (STEER_ENCODER_RES / 2))
        diff -= STEER_ENCODER_RES;
    else if(diff < -(STEER_ENCODER_RES / 2))
        diff += STEER_ENCODER_RES;

    // 不对称映射（使用预计算的差值宏）：
    // 左转：diff > 0，最大 diff = 957，对应 -STEER_MAX_ANGLE
    // 右转：diff < 0，最大 |diff| = 1150，对应 +STEER_MAX_ANGLE
    float angle;
    if(diff >= 0)
    {
        // 左转（负角度）
        angle = -(float)diff * (STEER_MAX_ANGLE / (float)STEER_LEFT_MAX_DIFF);
    }
    else
    {
        // 右转（正角度）
        angle = -(float)diff * (STEER_MAX_ANGLE / (float)STEER_RIGHT_MAX_DIFF);
    }

    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取当前转向编码器原始值和差值
//  @param      raw   输出：编码器原始值
//  @param      diff  输出：相对原位的差值（左正右负，跨零点已处理）
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void steering_get_raw_and_diff(int16 *raw, int32 *diff)
{
    *raw = absolute_encoder_get_location();
    int32 d = (int32)(*raw) - (int32)STEER_HOME_RAW;

    if(d > (STEER_ENCODER_RES / 2))
        d -= STEER_ENCODER_RES;
    else if(d < -(STEER_ENCODER_RES / 2))
        d += STEER_ENCODER_RES;

    *diff = d;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      转向模块初始化
//-------------------------------------------------------------------------------------------------------------------
void steering_init(void)
{
    absolute_encoder_init();
    system_delay_ms(10);

    // DRV8701 驱动初始化：方向引脚 GPIO + PWM 引脚
    gpio_init(STEER_DIR_PIN, GPO, 0, GPO_PUSH_PULL);  // 方向引脚，默认低电平（左转）
    pwm_init(STEER_PWM_PIN, 10000, 0);                // PWM引脚，10kHz，占空比0

    g_target_angle = 0.0f;
    g_last_error  = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置目标角度
//  @param      angle_deg   目标角度（度），左负右正
//-------------------------------------------------------------------------------------------------------------------
void steering_set_target(float angle_deg)
{
    g_target_angle = func_limit(angle_deg, STEER_MAX_ANGLE);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      转向控制（每个控制周期调用一次）
//  @note       PD + 陀螺仪前馈，误差 = 目标 - 当前
//-------------------------------------------------------------------------------------------------------------------
void steering_control(void)
{
    // 1. 获取当前角度
    float current = steering_get_current_angle();

    // 2. 误差 = 目标 - 当前
    float error = g_target_angle - current;

    // 3. PD + 陀螺仪前馈
    float derivative = (error - g_last_error) / STEER_CONTROL_DT;
    float output = STEER_KP * error
                 + STEER_KD * derivative
                 + STEER_GKD * imu660.data_Ripen.gyro_z;

    // 4. 限幅
    if(fabsf(error) <= STEER_ERROR_DEADBAND_DEG)
    {
        output = 0.0f;
    }
    else
    {
        output = func_limit(output, (float)STEER_PWM_MAX);
        if(fabsf(output) < (float)STEER_PWM_MIN_EFFECTIVE)
        {
            output = (output >= 0.0f) ? (float)STEER_PWM_MIN_EFFECTIVE : -(float)STEER_PWM_MIN_EFFECTIVE;
        }
    }

    g_steer_debug.target_deg = g_target_angle;
    g_steer_debug.current_deg = current;
    g_steer_debug.error_deg = error;
    g_steer_debug.derivative_deg_s = derivative;
    g_steer_debug.output_pwm = output;
    steering_get_raw_and_diff(&g_steer_debug.raw, &g_steer_debug.diff);

    // 5. DRV8701 驱动：DIR + PWM
    // output > 0: 需要右转（正方向） → DIR高，PWM输出
    // output < 0: 需要左转（负方向） → DIR低，PWM输出
    // output == 0: 停止 → PWM归零
    int16 abs_duty = (int16)fabsf(output);
    if(output >= 0.0f)
    {
        gpio_low(STEER_DIR_PIN);   // 右转（正方向）
    }
    else
    {
        gpio_high(STEER_DIR_PIN);    // 左转（负方向）
    }
    pwm_set_duty(STEER_PWM_PIN, (uint16)abs_duty);

    // 6. 保存误差
    g_last_error = error;

    // 7. 差速转向助力计算（陀螺仪角速率闭环）
    //    输入：目标角速率 - 实际角速率（陀螺仪测量，取反后右转为正）
    //    目标角速率 = v * tan(δ) / L （自行车模型/单轨模型，右转为正）
    //    实际角速率 = -imu660.gyro_z（取反：IMU右手定则逆时针为正→车辆右转为正）
    //    输出：差速脉冲偏移，正值=左轮加速右轮减速（辅助右转）
    //    原理：当实际转向角速率跟不上目标时，后轮差速产生额外偏航力矩
    //    优势：稳态仍有输出（转弯中目标ω持续存在），真正缩小转弯半径
    if(g_diff_enabled)
    {
        // ===== 曲率前馈计算 =====
        // 前馈原理：PP算法计算出路径曲率κ后，直接预加差速偏移
        // 差速约定：diff>0=辅助右转（左轮加速），diff<0=辅助左转（右轮加速）
        // κ>0(左转) → 需要辅助左转 → 前馈应为负值 → 右轮加速
        // κ<0(右转) → 需要辅助右转 → 前馈应为正值 → 左轮加速
        // 因此前馈取反：ff = -KFF * κ * v * speed_dir
        // speed_dir修正：倒车时差速方向需要反转
        {
            float v_mps_ff = g_state.speed_average_mps;
            if(fabsf(v_mps_ff) < 0.05f)
                v_mps_ff = g_diff_speed_dir * fabsf(v_mps_ff);
            g_diff_ff_delta = -DIFF_STEER_FF_GAIN * g_diff_curvature_ff * v_mps_ff * g_diff_speed_dir;
            // 前馈限幅
            if(g_diff_ff_delta > DIFF_STEER_FF_MAX)
                g_diff_ff_delta = DIFF_STEER_FF_MAX;
            if(g_diff_ff_delta < -DIFF_STEER_FF_MAX)
                g_diff_ff_delta = -DIFF_STEER_FF_MAX;
        }

        // ===== PID反馈计算 =====
        // 计算目标角速率（自行车模型/单轨模型）
        float steer_rad = g_target_angle * 0.017453292519943295f;  // 度→弧度
        float v_mps = g_state.speed_average_mps;                   // 编码器平均速度（m/s）
        // 当速度接近0时，编码器方向不可靠，用行驶方向标志修正
        if(fabsf(v_mps) < 0.05f)
            v_mps = g_diff_speed_dir * fabsf(v_mps);
        float omega_target = v_mps * tanf(steer_rad) / INS_WHEELBASE_M;
        g_steer_debug.omega_target_rad_s = omega_target;

        // 实际角速率（陀螺仪直接测量）
        // 注意：IMU963RA的gyro_z遵循右手定则（逆时针=正），
        //       但车辆坐标系约定右转为正，因此取反
        float omega_actual = -imu660.data_Ripen.gyro_z;
        g_steer_debug.omega_actual_rad_s = omega_actual;

        // 角速率误差：正值=需要更多右转，负值=需要更多左转
        float omega_error = omega_target - omega_actual;

        // 低速增强：低速时转向需求更大，增益适当放大
        float gain_mult = 1.0f;
        if(fabsf(v_mps) < DIFF_STEER_LOW_SPEED_THRESH && fabsf(v_mps) > 0.01f)
        {
            gain_mult = DIFF_STEER_LOW_SPEED_GAIN;
        }

        // 死区判断：角速率误差小于阈值时不启用PID反馈（前馈仍生效！）
        float pid_delta;
        if(fabsf(omega_error) < DIFF_STEER_DEADZONE_RAD_S)
        {
            pid_delta = 0.0f;
            g_diff_integral = 0.0f;
            g_diff_last_error = 0.0f;
        }
        else
        {
            // PID计算
            float derivative = (omega_error - g_diff_last_error) / STEER_CONTROL_DT;
            g_diff_integral += omega_error * STEER_CONTROL_DT;

            // 积分限幅
            float int_limit = (float)DIFF_STEER_MAX_DELTA / (DIFF_STEER_KI + 0.001f);
            if(g_diff_integral > int_limit)  g_diff_integral = int_limit;
            if(g_diff_integral < -int_limit) g_diff_integral = -int_limit;

            float diff_output = gain_mult * (DIFF_STEER_KP * omega_error
                              + DIFF_STEER_KI * g_diff_integral
                              + DIFF_STEER_KD * derivative);

            // PID限幅（浮点，不取整）
            if(diff_output > DIFF_STEER_MAX_DELTA)
                diff_output = DIFF_STEER_MAX_DELTA;
            if(diff_output < -DIFF_STEER_MAX_DELTA)
                diff_output = -DIFF_STEER_MAX_DELTA;

            pid_delta = diff_output;
            g_diff_last_error = omega_error;
        }

        // ===== 合成：前馈 + PID反馈 =====
        g_diff_delta = g_diff_ff_delta + pid_delta;
        g_steer_debug.diff_delta = g_diff_delta;
        g_steer_debug.diff_ff_delta = g_diff_ff_delta;
        // 总限幅：前馈+PID之和不超过 MAX_DELTA + FF_MAX
        float total_max = DIFF_STEER_MAX_DELTA + DIFF_STEER_FF_MAX;
        if(g_diff_delta > total_max)
            g_diff_delta = total_max;
        if(g_diff_delta < -total_max)
            g_diff_delta = -total_max;
    }
    else
    {
        g_diff_delta = 0.0f;
        g_diff_ff_delta = 0.0f;
        g_steer_debug.diff_delta = 0.0f;
        g_steer_debug.diff_ff_delta = 0.0f;
        g_steer_debug.omega_target_rad_s = 0.0f;
        g_steer_debug.omega_actual_rad_s = -imu660.data_Ripen.gyro_z;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取转向PID调试快照
//  @param      out  输出调试数据
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void steering_get_pid_debug(SteeringPidDebug *out)
{
    if(out == NULL)
    {
        return;
    }
    *out = g_steer_debug;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      发送当前转向角度、目标角度与原始编码器值到上位机
//-------------------------------------------------------------------------------------------------------------------
void steering_send_angle_to_host(void)
{
    int16 raw = absolute_encoder_get_location();
    printf("steer:%.2f,target:%.2f,raw:%d\r\n", steering_get_current_angle(), g_target_angle, raw);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取当前差速偏移量
//  @return     差速脉冲偏移（脉冲/4ms，浮点），正值=左轮加速右轮减速（辅助右转），负值反之
//-------------------------------------------------------------------------------------------------------------------
float steering_get_diff_delta(void)
{
    return g_diff_delta;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置行驶方向（供差速模块判断倒车）
//  @param      dir  1.0=前进，-1.0=倒车
//-------------------------------------------------------------------------------------------------------------------
void steering_diff_set_speed_dir(float dir)
{
    g_diff_speed_dir = (dir >= 0.0f) ? 1.0f : -1.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      启用/禁用差速助力
//  @param      enable  1=启用，0=禁用
//-------------------------------------------------------------------------------------------------------------------
void steering_diff_assist_enable(uint8 enable)
{
    g_diff_enabled = enable;
    if(!enable)
    {
        g_diff_delta = 0.0f;
        g_diff_integral = 0.0f;
        g_diff_last_error = 0.0f;
        g_diff_curvature_ff = 0.0f;
        g_diff_ff_delta = 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置路径曲率前馈（由track模块在PP计算后调用）
//  @param      kappa   路径曲率（1/m），正值=左转曲率，负值=右转曲率
//  @note       曲率前馈在弯道入口即预加差速，消除转向延迟导致的切角
//-------------------------------------------------------------------------------------------------------------------
void steering_diff_set_curvature_ff(float kappa)
{
    g_diff_curvature_ff = kappa;
}
