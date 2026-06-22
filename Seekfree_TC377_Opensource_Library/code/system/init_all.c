/*
 * init_all.c
 */
 //-------------------------------------------头文件引用------------------------------------------------------------
#include "zf_common_headfile.h"
#include "rtk.h"

 //-------------------------------------------函数定义------------------------------------------------------------
void system_init_all(void)
{
    printf("1");
   imu963ra_init();                                                // 初始化IMU963RA
   imu_bias_init(&imu_date);                                       // 初始化IMU偏置
   imu_gyro_z_autocalib(&imu_date, 3500);                             // 自动校准陀螺仪Z轴偏置
   imu_mag_bias_load(&imu_date);                                    // 加载磁力计标定参数

    ips200_init(IPS200_TYPE_SPI);                                   // 初始化IPS200
//  gnss_init(TAU1201);                                             // 初始化GNSS
    key_init(8);                                                    // 初始化按键
//    encoder_init();                                                 // 初始化编码器
//    encoder_layer_set_model(0.00036816f, 0.00036816f, 0.004f);     // 设置编码器模型参数（4ms 周期）
    debug_init();
    wireless_uart_init();

    Motor_Init();
    steering_init();                                                // 初始化转向电机
    yaokong_init(15);    //.                                         // 初始化遥控控制逻辑
    Ins_init();                                                     // 初始化INS核心模块
    rtk_init();                                                    // 初始化RTK模块（GN43RFA定位+双天线测向）
    subject1_init();                                                // 初始化科目一状态机
//  subject3_init();                                              // 初始化科目三状态机（默认关闭）
    pit_ms_init(CCU60_CH0, 1);                                      // 初始化1ms定时器

    pid_para_init(&Yao_pid.vLB_pid);                                // 初始化左轮 PID 参数
    pid_para_init(&Yao_pid.vRB_pid);                                // 初始化右轮 PID 参数
    wheel_pid_set_target_speed(0);                               // 设置目标速度为0（脉冲/4ms）
    Yao.flag_motor_start_user = 0;                                  // 初始化电机禁止启动

    lora3a22_init();                                                //.   lora3a22_uart_callback() isr.c
//    pedal_init();

    track_init();                                                   // 初始化轨迹

    printf("2");

}
