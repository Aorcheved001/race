/*
 * init_all.cd
 */
 //-------------------------------------------ͷ�ļ���------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------����������------------------------------------------------------------
void system_init_all(void)
{

    imu963ra_init();                                                // ��ʼ��IMU963RA
    imu_bias_init(&imu_date);                                       // ��ʼ��IMUƫ��
    imu_gyro_z_autocalib(&imu_date, 1000);                           // �Զ�У׼������Z��ƫ��
    imu_mag_bias_load(&imu_date);                                    // ���ش����Ʊ궨����

    ips200_init(IPS200_TYPE_SPI);                                   // ��ʼ��IPS200
    gnss_init(TAU1201);                                             // ��ʼ��GNSS
    key_init(8);                                                    // ��ʼ������
    encoder_init();                                               // ��ʼ��������
    encoder_layer_set_model(-0.00002204f, -0.00002204f, 0.004f); // ���ñ�����ģ�Ͳ���

    wireless_uart_init();
    motor_init();
    steering_init();
    yaokong_init(0.8f);

    Ins_init();                                                     // ��ʼ��INS
    INS_init();                                                     // Ins״̬����ʼ��

    pit_ms_init(CCU60_CH0, 1);                                      // ��ʼ��1ms��ʱ��

    wheel_pid_init();                                               // ��ʼ����PID
    wheel_pid_set_target_speed(0.0f);                               // ����Ŀ���ٶ�Ϊ0
    wheel_pid_enable(1);                                            // ʹ����PID

    track_init();                                                   // ��ʼ���켣
}
