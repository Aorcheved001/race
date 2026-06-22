/*
 * init_all.c
 */
 //-------------------------------------------头文件引用------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------函数定义------------------------------------------------------------
void system_init_all(void)
{

   imu963ra_init();                                                // 初始化IMU963RA
   imu_bias_init(&imu_date);                                       // 初始化IMU偏置
   imu_gyro_z_autocalib(&imu_date, 1000);                           // 自动校准陀螺仪Z轴偏置
   imu_mag_bias_load(&imu_date);                                    // 加载磁力计标定参数

    ips200_init(IPS200_TYPE_SPI);                                   // 初始化IPS200
    gnss_init(TAU1201);                                             // 初始化GNSS
    key_init(8);                                                    // 初始化按键
    encoder_init();                                                 // 初始化编码器
    encoder_layer_set_model(0.00036816f, -0.00036816f, 0.001f);     // 设置编码器模型参数

 // wireless_uart_init();
    motor_init();
    steering_init();                                                // 初始化转向电机
    yaokong_init(0.8f);                                             // 初始化遥控控制逻辑

    Ins_init();                                                     // 初始化INS核心模块
    subject1_init();                                                // 初始化科目一状态机
//  subject3_init();                                              // 初始化科目三状态机（默认关闭）
    pit_ms_init(CCU60_CH0, 1);                                      // 初始化1ms定时器

    wheel_pid_init();                                               // 初始化车轮PID
    wheel_pid_set_target_speed(0.0f);                               // 设置目标速度为0
    wheel_pid_enable(1, 1, 0, 0);                                   // 使能车轮PID

    track_init();                                                   // 初始化轨迹
}
