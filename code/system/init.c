/*
 * Init.c
 *
 *  Created on: 2024??12??3??
 *      Author: 21912
 */
#include "zf_common_headfile.h"
#include "task.h"


void Init_State(void)
{
    project_task.task1_state = OFF;
    for(int i = 0; i < task2_state_num; i++)
    {
    project_task.task2_state[i] = 0;
    }
    project_task.task3_state = OFF;
    project_task.task4_state = OFF;
    project_task.drive_state = OFF;
}
void Init_All(void)
{
    //屏幕初始化
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_clear();
    ips200_show_string(0, 0*16, "System Init...");

    dot_matrix_screen_init();       // 点阵屏幕初始化
    dot_matrix_screen_set_brightness(5000);
    Init_State();

    // GPS初始化
//    gnss_init(TAU1201);
//    INS_GPS_Fusion_Init(); // 初始化GPS融合参数
//    ips200_show_string(0, 1*16, "GPS: OK");
//    imu963ra_init();
    // IMU963RA初始化
//    if(imu963ra_init() == 0)
//        ips200_show_string(0, 2*16, "IMU: OK");
//    else
//        ips200_show_string(0, 2*16, "IMU: ERROR!");
//    //卡尔曼滤波初始化
//    imu963ra_kalman_filter_init(&imu, 1, 1, 0.002f);
//
//    imu963ra_menc15a_kalman_filter_init(&vel_kf, 1e-9, 1e-4, 3e-5, 0.01f);
//    imu963ra_menc15a_kalman_filter_init(&vel_kf_x, 1e-6f, 1e-3f, 1e-2f, 0.1f);
//    imu963ra_menc15a_kalman_filter_init(&vel_kf_y, 1e-6f, 1e-3f, 1e-2f, 0.1f);



//    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P00_7, TIM2_ENCODER_CH2_P00_8);
//    ips200_show_string(0, 3*16, "Encoder: OK");

    // 中断5ms
    pit_ms_init(CCU60_CH0, 2);

    // 按键扫描10ms
    key_init(10);
        ips200_show_string(0, 1*16, "GPS: OK");
    system_delay_ms(1000); //延迟
    ips200_show_string(0, 2*16, "GPS: OK");
    ips200_clear();
}

