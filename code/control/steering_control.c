/*
 * steering_control.c
 *
 *  Created on: 2026年4月13日
 *      Author: ABCDE   FGHIJKL
 */

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------内部全局变量------------------------------------------------------------
static int16  g_zero_raw = 0;                  // 零位原始值（上电时校准）
static float  g_target_angle = 0.0f;           // 目标角度（度）
static float  g_last_error = 0.0f;             // 上一次误差
static uint8  g_output_enabled = 1u;           // 输出使能标志
static float  g_max_angle = STEER_MAX_ANGLE;   // 当前角度限制（度），可运行时修改

 //-------------------------------------------内部函数声明------------------------------------------------------------
static int16 steering_read_zero(void);         // 采集转向零位

 ////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      采集转向零位（多次读取取平均）
 ////  @param      无
 ////  @return     int16          零位原始值
 ////  @note       初始化时调用，采集 STEER_ZERO_SAMPLES 次取平均以减小噪声
 ////-------------------------------------------------------------------------------------------------------------------
static int16 steering_read_zero(void)
{
    int32 sum = 0;
    const uint8 samples = STEER_ZERO_SAMPLES;

    for(uint8 i = 0; i < samples; i++)
    {
        sum += absolute_encoder_get_location();
        system_delay_ms(2);
    }

    return (int16)(sum / samples);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      转向模块初始化
 ////  @param      无
 ////  @return     void
 ////  @note       初始化绝对编码器/PWM/GPIO，并采集零位
 ////-------------------------------------------------------------------------------------------------------------------
void steering_init(void)
{
    // 1. 初始化绝对值编码器（SPI）
    absolute_encoder_init();
    system_delay_ms(10);

    // 2. 采集转向零位（上电瞬时零位）
    g_zero_raw = steering_read_zero();

    // 3. 初始化转向电机 PWM（17kHz，与原有逻辑一致）
    pwm_init(STEER_MOTOR_PWM_PIN, 17000, 0);

    // 4. 初始化转向电机方向引脚（默认正向）
    gpio_init(STEER_MOTOR_DIR_PIN, GPO, 1, GPO_PUSH_PULL);

    // 5. 初始化 PD 状态
    g_target_angle = 0.0f;
    g_last_error = 0.0f;
    g_output_enabled = 1u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      设置转向目标角度
 ////  @param      angle_deg       目标角度（度），正值为右转，负值为左转
 ////  @return     void
 ////  @note       内部自动限幅到 STEER_MAX_ANGLE
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_target(float angle_deg)
{
    g_target_angle = func_limit(angle_deg, g_max_angle);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      转向控制函数（4ms 周期调用）
 ////  @param      无
 ////  @return     void
 ////  @note       PD 闭环 + 陀螺仪前馈控制，输出 PWM 驱动转向电机
 ////-------------------------------------------------------------------------------------------------------------------
void steering_control(void)
{
    // 1. 读取绝对值编码器当前值
    int16 raw = absolute_encoder_get_location();

    // 2. 转换为角度（度），以 g_zero_raw 为 0 度参考
    float current_angle = (float)(raw - g_zero_raw) * (360.0f / 4096.0f) * STEER_GEAR_RATIO;

    // 3. 计算误差
    float error = g_target_angle - current_angle;

    // 4. 解绕圈处理：防止角度跨越 +-180° 边界导致误差跳变
    if(error > 180.0f)
    {
        error -= 360.0f;
    }
    else if(error < -180.0f)
    {
        error += 360.0f;
    }

    // 5. PD + 陀螺仪前馈控制
    float derivative = (error - g_last_error) / 0.004f;   // dt = 4ms
    float output = KP * error
                 + KD * derivative
                 + GKD * imu660.data_Ripen.gyro_z;        // gyro_z 单位: rad/s

    // 6. 输出限幅
    output = func_limit(output, (float)STEER_PWM_MAX);

    // 7. 驱动转向电机
    if(g_output_enabled)
    {
        int16 abs_duty = (int16)fabsf(output);
        uint8 dir = (output >= 0.0f) ? 1 : 0;

        pwm_set_duty(STEER_MOTOR_PWM_PIN, (uint16)abs_duty);
        gpio_set_level(STEER_MOTOR_DIR_PIN, dir);
    }
    else
    {
        // 输出关闭时：PWM = 0，电机不动
        pwm_set_duty(STEER_MOTOR_PWM_PIN, 0);
    }

    // 8. 保存当前误差供下次微分计算
    g_last_error = error;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      获取当前转向角度
 ////  @param      无
 ////  @return     float    当前角度（度），0 度 = 零位方向
 ////-------------------------------------------------------------------------------------------------------------------
float steering_get_current_angle(void)
{
    int16 raw = absolute_encoder_get_location();
    float angle = (float)(raw - g_zero_raw) * (360.0f / 4096.0f) * STEER_GEAR_RATIO;

    // 归一化到 +-180 度
    if(angle > 180.0f)  angle -= 360.0f;
    if(angle < -180.0f) angle += 360.0f;

    return angle;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      以当前车轮方向为零位（运行时标定）
 ////  @param      无
 ////  @return     void
 ////  @note       调用前确保车轮处于正前方
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_zero_at_current(void)
{
    g_zero_raw = steering_read_zero();
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      手动设置零位偏移量
 ////  @param      raw_offset    零位偏移（原始值），正值 = 零位正向偏移
 ////  @return     void
 ////  @note       用于精确微调零位，例如通过串口/菜单输入偏移值
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_zero_offset(int16 raw_offset)
{
    g_zero_raw = steering_read_zero() + raw_offset;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      使能/关闭转向输出
 ////  @param      enable    1=使能（PD闭环驱动电机），0=关闭（PWM归零，电机停转）
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_output_enabled(uint8 enable)
{
    g_output_enabled = enable ? 1u : 0u;
    if(!g_output_enabled)
    {
        pwm_set_duty(STEER_MOTOR_PWM_PIN, 0);
    }
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      设置转向角度限制
 ////  @param      max_angle_deg   最大转向角度（度），常用值：30.0（默认）/ 45.0（掉头）
 ////  @return     void
 ////  @note       仅修改 steering_set_target() 的限幅值，不影响已设置的 target
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_max_angle(float max_angle_deg)
{
    if(max_angle_deg > 0.0f && max_angle_deg < 180.0f)
    {
        g_max_angle = max_angle_deg;
    }
}


