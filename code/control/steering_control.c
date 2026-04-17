/*
 * steering_control.c
 *
 *  Created on: 2026��4��12��
 *      Author: A
 */

 //-------------------------------------------ͷ�ļ���------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------�ڲ�ȫ�ֱ���------------------------------------------------------------
static int16  g_zero_raw = 0;                  // ��λԭʼֵ���ϵ�ʱУ׼��
static float  g_target_angle = 0.0f;           // Ŀ��Ƕȣ��ȣ�
static float  g_last_error = 0.0f;             // ��һ�����
static uint8  g_output_enabled = 1u;           // ���ʹ�ܱ�־
static float  g_max_angle = STEER_MAX_ANGLE;   // 大齿轮角度限制（度），可运行时修改

// 齿轮比：编码器读小齿轮(30齿)，需要换算为大齿轮(56齿)的角度
// 大齿轮角度 = 小齿轮角度 × (30/56)
#define STEER_GEAR_RATIO        (30.0f / 56.0f)   // ��ǰ�Ƕ����ƣ��ȣ���������ʱ�޸�

 //-------------------------------------------�ڲ���������------------------------------------------------------------
static int16 steering_read_zero(void);         // �ɼ�ת����λ

 ////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      �ɼ�ת����λ����ζ�ȡȡƽ����
 ////  @param      ��
 ////  @return     int16          ��λԭʼֵ
 ////  @note       ��ʼ��ʱ���ã��ɼ� STEER_ZERO_SAMPLES ��ȡƽ���Լ�С����
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
 ////  @brief      ת��ģ���ʼ��
 ////  @param      ��
 ////  @return     void
 ////  @note       ��ʼ�����Ա�����/PWM/GPIO�����ɼ���λ
 ////-------------------------------------------------------------------------------------------------------------------
void steering_init(void)
{
    // 1. ��ʼ������ֵ��������SPI��
    absolute_encoder_init();
    system_delay_ms(10);

    // 2. �ɼ�ת����λ���ϵ�˲ʱ��λ��
    g_zero_raw = steering_read_zero();

    // 3. ��ʼ��ת���� PWM��17kHz����ԭ���߼�һ�£�
    pwm_init(STEER_MOTOR_PWM_PIN, 17000, 0);

    // 4. ��ʼ��ת�����������ţ�Ĭ������
    gpio_init(STEER_MOTOR_DIR_PIN, GPO, 1, GPO_PUSH_PULL);

    // 5. ��ʼ�� PD ״̬
    g_target_angle = 0.0f;
    g_last_error = 0.0f;
    g_output_enabled = 1u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ����ת��Ŀ��Ƕ�
 ////  @param      angle_deg       Ŀ��Ƕȣ��ȣ�����ֵΪ��ת����ֵΪ��ת
 ////  @return     void
 ////  @note       �ڲ��Զ��޷��� STEER_MAX_ANGLE
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_target(float angle_deg)
{
    g_target_angle = func_limit(angle_deg, g_max_angle);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ת����ƺ�����4ms ���ڵ��ã�
 ////  @param      ��
 ////  @return     void
 ////  @note       PD �ջ� + ������ǰ�����ƣ���� PWM ����ת����
 ////-------------------------------------------------------------------------------------------------------------------
void steering_control(void)
{
    // 1. ��ȡ����ֵ��������ǰֵ
    int16 raw = absolute_encoder_get_location();

    // 2. ת��Ϊ�Ƕȣ��ȣ����� g_zero_raw Ϊ 0 �Ȳο�
    float current_angle = (float)(raw - g_zero_raw) * (360.0f / 4096.0f) * STEER_GEAR_RATIO;

    // 3. �������
    float error = g_target_angle - current_angle;

    // 4. ����Ȧ��������ֹ�Ƕȿ�Խ +-180�� �߽絼���������
    if(error > 180.0f)
    {
        error -= 360.0f;
    }
    else if(error < -180.0f)
    {
        error += 360.0f;
    }

    // 5. PD + ������ǰ������
    float derivative = (error - g_last_error) / 0.004f;   // dt = 4ms
    float output = KP * error
                 + KD * derivative
                 + GKD * imu660.data_Ripen.gyro_z;        // gyro_z ��λ: rad/s

    // 6. ����޷�
    output = func_limit(output, (float)STEER_PWM_MAX);

    // 7. ����ת����
    if(g_output_enabled)
    {
        int16 abs_duty = (int16)fabsf(output);
        uint8 dir = (output >= 0.0f) ? 1 : 0;

        pwm_set_duty(STEER_MOTOR_PWM_PIN, (uint16)abs_duty);
        gpio_set_level(STEER_MOTOR_DIR_PIN, dir);
    }
    else
    {
        // ����ر�ʱ��PWM = 0���������
        pwm_set_duty(STEER_MOTOR_PWM_PIN, 0);
    }

    // 8. ���浱ǰ���´�΢�ּ���
    g_last_error = error;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ��ȡ��ǰת��Ƕ�
 ////  @param      ��
 ////  @return     float    ��ǰ�Ƕȣ��ȣ���0 �� = ��λ����
 ////-------------------------------------------------------------------------------------------------------------------
float steering_get_current_angle(void)
{
    int16 raw = absolute_encoder_get_location();
    float angle = (float)(raw - g_zero_raw) * (360.0f / 4096.0f) * STEER_GEAR_RATIO;

    // ��һ���� +-180 ��
    if(angle > 180.0f)  angle -= 360.0f;
    if(angle < -180.0f) angle += 360.0f;

    return angle;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      �Ե�ǰ���ַ���Ϊ��λ������ʱ�궨��
 ////  @param      ��
 ////  @return     void
 ////  @note       ����ǰȷ�����ִ�����ǰ��
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_zero_at_current(void)
{
    g_zero_raw = steering_read_zero();
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      �ֶ�������λƫ����
 ////  @param      raw_offset    ��λƫ�ƣ�ԭʼֵ������ֵ = ��λ����ƫ��
 ////  @return     void
 ////  @note       ���ھ�ȷ΢����λ������ͨ������/�˵�����ƫ��ֵ
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_zero_offset(int16 raw_offset)
{
    g_zero_raw = steering_read_zero() + raw_offset;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ʹ��/�ر�ת�����
 ////  @param      enable    1=ʹ�ܣ�PD�ջ������������0=�رգ�PWM���㣬���ͣת��
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
 ////  @brief      ����ת��Ƕ�����
 ////  @param      max_angle_deg   ���ת��Ƕȣ��ȣ�������ֵ��30.0��Ĭ�ϣ�/ 45.0����ͷ��
 ////  @return     void
 ////  @note       ���޸� steering_set_target() ���޷�ֵ����Ӱ�������õ� target
 ////-------------------------------------------------------------------------------------------------------------------
void steering_set_max_angle(float max_angle_deg)
{
    if(max_angle_deg > 0.0f && max_angle_deg < 180.0f)
    {
        g_max_angle = max_angle_deg;
    }
}


