/*
 * init_all.c
 *
 *  系统初始化模块
 *  集中初始化所有硬件和软件模块
 */

//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------函数定义------------------------------------------------------------
void system_init_all(void)
{
    /* IMU 初始化 */
//    imu963ra_init();                                                /* 初始化IMU963RA */
//    imu_bias_init(&imu_date);                                       /* 初始化IMU偏差 */
//    imu_gyro_z_autocalib(&imu_date, 1000);                          /* 自动校准陀螺Z轴漂移 */
//    imu_mag_bias_load(&imu_date);                                   /* 加载磁力计校准参数 */

    /* 外设初始化 */
    ips200_init(IPS200_TYPE_SPI);                                   /* 初始化IPS200屏幕 */
//    gnss_init(TAU1201);                                             /* 初始化GNSS */
//    key_init(8);                                                    /* 初始化按键 */
    encoder_init();                                                 /* 初始化编码器 */
    encoder_layer_set_model(0.00036816f, -0.00036816f, 0.001f);    /* 设置编码器模型参数 */

    /* 通信与驱动初始化 */
    wireless_uart_init();                                           /* 初始化无线串口 */
    vofa_init();                                                    /* 初始化VOFA+调试模块 */
    motor_init();                                                   /* 初始化电机 */
//    steering_init();                                                /* 初始化转向舵机 */
//    yaokong_init(0.8f);                                             /* 初始化遥控器 */

    /* INS 系统初始化 */
//    Ins_init();                                                     /* 初始化INS */
    pit_ms_init(CCU60_CH0, 1);                                      /* 初始化1ms定时器 */

//    /* PID 控制初始化 */
    wheel_pid_init();                                               /* 初始化车轮PID */
//    wheel_pid_set_speed_target(4.0f, 0.0f);
    wheel_pid_set_target_speed(4.0f);                               /* 设置目标速度为0 */
    wheel_pid_enable(1, 1, 0, 0);                                   /* 使能输出、速度环，关闭位置环，使能角速度环 */

    /* 跟踪追踪初始化 */
//    track_init();                                                   /* 初始化循迹 */
}
