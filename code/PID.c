/*
 * PID.c
 *
 *  Created on: 2026-03-24
 *      Author: Daydreamer
 */
//-------------------------------------------ͷ�ļ�������------------------------------------------------------------
#include "zf_common_headfile.h"
#include "PID.h"
#include "motor.h"

//-------------------------------------------����������------------------------------------------------------------
WHEEL_PID_LAYER g_wheel_pid;                                         // �����ٶ�PIDȫ�ֶ���

//-------------------------------------------�ڲ�����������------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//  �������     �ٶȻ�PID����
//  ����˵��     pid_info      PID ״̬�ṹ��
//  ����˵��     pid_param     �������� [Kp, Ki, Kd, I_limit]
//  ����˵��     target        Ŀ���ٶ�
//  ����˵��     current       ��ǰ�ٶ�
//  ����˵��     dt_s          �������ڣ�s��
//  ���ز���     float         PWM���
//-------------------------------------------------------------------------------------------------------------------
static float pid_speed_step(PID_INFO *pid_info,
                            const float *pid_param,
                            float target,
                            float current,
                            float dt_s)
{
    if (!pid_info || !pid_param) return 0.0f;
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    pid_info->iError = target - current;                             // ��ǰ��� e(k)

    pid_info->SumError += pid_info->iError * dt_s;                   // �������ۼ�
    if (pid_param[3] > 0.0f)
    {
        pid_info->SumError = func_limit(pid_info->SumError, pid_param[3]); // �����޷�
    }

    float diff = (pid_info->iError - pid_info->LastError) / dt_s;    // ΢����

    float out = pid_param[0] * pid_info->iError                      // P
              + pid_param[1] * pid_info->SumError                    // I
              + pid_param[2] * diff;                                 // D

    pid_info->LastError = pid_info->iError;                          // ������������´�΢��
    return out;
}

//-------------------------------------------���⺯��������------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//  �������     ���� PID �����ṹ��
//  ����˵��     pid_info  PID ״̬�ṹ��ָ��
//-------------------------------------------------------------------------------------------------------------------
void pid_para_init(PID_INFO *pid_info)
{
    if (!pid_info) return;
    pid_info->iError = 0.0f;
    pid_info->LastError = 0.0f;
    pid_info->SumError = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     �����ٶ�PID��ʼ��
//  ����˵��     ��
//  ��ע��Ϣ     ���ٶȻ��������������
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_init(void)
{
    memset(&g_wheel_pid, 0, sizeof(g_wheel_pid));

    pid_para_init(&g_wheel_pid.pid_spd);                             // �ٶȻ�״̬����

    g_wheel_pid.speed_param[0] = 1000.0f;                             // �ٶȻ� Kp�����PWM��
    g_wheel_pid.speed_param[1] = 40.0f;                              // �ٶȻ� Ki
    g_wheel_pid.speed_param[2] = 0.0f;                               // �ٶȻ� Kd
    g_wheel_pid.speed_param[3] = 2.0f;                               // �ٶȻ������޷���m/s��s��

    g_wheel_pid.target_speed_mps = 0.0f;                             // Ŀ���ٶ�
    g_wheel_pid.speed_limit = 3.0f;                                  // �ٶ��޷� 2 m/s
    g_wheel_pid.out_pwm = 0.0f;                                      // ���PWM

    g_wheel_pid.enable_output = 0u;                                  // Ĭ�ϲ��·�������
    g_wheel_pid.initialized = 1u;                                    // ��ʼ�����
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ���� PID ���ʹ��
//  ����˵��     enable_output 1=���PWM����� 0=�����
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_enable(uint8 enable_output)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    if (enable_output && !g_wheel_pid.enable_output)
    {
        pid_para_init(&g_wheel_pid.pid_spd);
    }

    g_wheel_pid.enable_output = enable_output ? 1u : 0u;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ���ú�������λ��Ŀ��
//  ����˵��     target_pos_m Ŀ��λ�ã�m��
//  ��ע��Ϣ     �����ӿڼ����ԣ�ʵ�ʲ�ʹ��λ�ÿ���
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_target_pos(float target_pos_m)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    (void)target_pos_m;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ���ú��������ٶ�Ŀ��
//  ����˵��     target_speed_mps Ŀ���ٶ�
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_target_speed(float target_speed_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    g_wheel_pid.target_speed_mps = func_limit(target_speed_mps, g_wheel_pid.speed_limit);
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ����λ�û�����
//  ����˵��     kp, ki, kd, i_limit
//  ��ע��Ϣ     �����ӿڼ����ԣ�ʵ�ʲ�ʹ��λ�ÿ���
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    (void)kp; (void)ki; (void)kd; (void)i_limit;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     �����ٶȻ�����
//  ����˵��     kp, ki, kd, i_limit
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    g_wheel_pid.speed_param[0] = kp;
    g_wheel_pid.speed_param[1] = ki;
    g_wheel_pid.speed_param[2] = kd;
    g_wheel_pid.speed_param[3] = i_limit;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     �ٶ�PID���ڸ���
//  ����˵��     enc   ��������״̬
//  ����˵��     dt_s  ����ʱ��
//  ��ע��Ϣ     ���ٶȻ�������������ơ����������Pure Pursuit����
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_update(const EncoderLayerState *enc, float dt_s)
{
    if (!enc) return;
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    float current_speed_mps = enc->speed_average_mps;

    float spd_out = pid_speed_step(&g_wheel_pid.pid_spd,
                                   g_wheel_pid.speed_param,
                                   g_wheel_pid.target_speed_mps,
                                   current_speed_mps,
                                   dt_s);

    spd_out = func_limit(spd_out, 10000.0f);
    g_wheel_pid.out_pwm = spd_out;

    if (g_wheel_pid.enable_output)
    {
        int16 spd = (int16)spd_out;
        MotorDir dir;
        uint8 pct;

        if(spd > 0)
        {
            dir = MOTOR_DIR_FORWARD;
            pct = (uint32)spd * 100U / PWM_DUTY_MAX;
        }
        else if(spd < 0)
        {
            dir = MOTOR_DIR_REVERSE;
            pct = (uint32)(-spd) * 100U / PWM_DUTY_MAX;
        }
        else
        {
            dir = MOTOR_DIR_BRAKE;
            pct = 0;
        }

        motor_control(motor_LB, dir, pct);
        motor_control(motor_RB, dir, pct);
    }
}
