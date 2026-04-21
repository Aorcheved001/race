/*
 * yaokong.c
 */

 //-------------------------------------------ͷ�ļ���------------------------------------------------------------
#include "zf_common_headfile.h"
#include "motor.h"
#include "PID.h"
#include "steering_control.h"

 //-------------------------------------------ȫ�ֱ�����------------------------------------------------------------
bool flag_stop = 0;                                                       // ���Ʊ�־λ��1��ʾֹͣ��0��ʾ����
float speed_yk = 0.8f;                                                    // ң���ٶ�ָ���λ��m/s
uint8 yaokong_active = 0;                                                 // ң���������־��1��ʾ������

 //-------------------------------------------�ڲ�������------------------------------------------------------------
static uint8 s_control_enabled = 1u;                                      // ����ʹ�ܱ�־��1��ʾ��������

 //-------------------------------------------����������------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ��ʼ��ң��
 ////  @param      speed_mps       ��ʼ�ٶȣ�m/s��
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void yaokong_init(float speed_mps)
{
    speed_yk = fabsf(speed_mps);                                          // �����ٶȾ���ֵ
    flag_stop = 0;                                                        // ����ֹͣ��־
    s_control_enabled = 1u;                                               // ʹ�ܿ���
    wheel_pid_enable(1);                                                  // ʹ������PID
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ����ң���Ƿ�����ֱ���·�ִ����ָ��
 ////  @param      enabled         1��ʾ������0��ʾ��ֹ
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void yaokong_set_control_enabled(uint8 enabled)
{
    s_control_enabled = enabled ? 1u : 0u;                                // ���ÿ���ʹ�ܱ�־
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ����ң������
 ////  @param      void
 ////  @return     uint8           1��ʾ�����ɹ���0��ʾ������
 ////  @note       ����ң�������ݲ����Ƶ���Ͷ��
 ////-------------------------------------------------------------------------------------------------------------------
uint8 yaokong_data_deal(void)
{
    if (lora3a22_state_flag != 1u)
    {
        return 0u;
    }

    if (lora3a22_finsh_flag != 1u)
    {
        return 0u;
    }

    flag_stop = (lora3a22_uart_transfer.key[0] == 1u) ? 1u : 0u;          // ����ֹͣ����

    float target_speed = 0.0f;
    uint8 has_input = 0u;

    if (!flag_stop)
    {
        if (lora3a22_uart_transfer.joystick[2] > 300)
        {
            target_speed = speed_yk;                                      // ǰ��
            has_input = 1u;
        }
        else if (lora3a22_uart_transfer.joystick[2] < -300)
        {
            target_speed = -speed_yk;                                     // ����
            has_input = 1u;
        }
    }

    float steer_offset_deg = 0.0f;
    int16 steer_raw = lora3a22_uart_transfer.joystick[3];
    if ((steer_raw > 300) || (steer_raw < -300))
    {
        steer_offset_deg = 0.008f * (float)steer_raw;                     // ����ת��ƫ��
        steer_offset_deg = func_limit(steer_offset_deg, 35.0f);           // �޷�ת���
        has_input = 1u;
    }

    if (lora3a22_uart_transfer.key[1] == 1u)
    {
        steer_offset_deg = 0.0f;                                          // ����1����ʱ����
    }

    if (s_control_enabled)
    {
        wheel_pid_set_target_speed(target_speed);                         // ����Ŀ���ٶ�
        steering_set_target(steer_offset_deg);                        // ���ö���Ƕ�
    }

    yaokong_active = has_input;                                           // ���¼����־
    lora3a22_finsh_flag = 0u;                                             // ���������ɱ�־
    return 1u;
}
