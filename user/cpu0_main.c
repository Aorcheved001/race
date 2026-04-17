/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
int a = 0;

// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    //Init_All();
    pit_ms_init(CCU60_CH0, 2);

    // 按键扫描10ms
    key_init(10);
    //ips200_set_font(IPS200_8X16_FONT);
    speech_uart_init();
    motor_init();
    //pwm_init(MOTOR1_PWM_PIN1, 10000, 0);                                    // 初始化 MOTOR2 的 PWM 通道1，频率 17kHz，初始占空比 0
    //pwm_init(MOTOR1_PWM_PIN2, 10000, 0);
    //gpio_init(P33_9, GPO, 1, GPO_PUSH_PULL);
    ips200_init(IPS200_TYPE_SPI);
   // absolute_encoder_init();
    //gpio_set_level(P33_9,1);
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
//        motor_control(motor_LB, MOTOR_DIR_FORWARD, 80);
//        motor_control(motor_RB, MOTOR_DIR_FORWARD, 80);
//        ips200_show_int (0, 0, a, 1);
//        uart_write_byte(UART_4, 0xA5);
//        if(key_detect(KEY_1, KEY_SHORT_PRESS))
//        {
//            a = 1;
//            ips200_show_int (0, 0, a, 1);
//        }
//
//        if(key_detect(KEY_2, KEY_SHORT_PRESS))//20.7
//        {
//            a = 0;
//            ips200_show_int (0, 0, a, 1);
//        }
//
//        if(key_detect(KEY_3, KEY_SHORT_PRESS))//20.7
//        {
//            a = 3;
//            ips200_show_int (0, 0, a, 1);
//        }
//
//        if(key_detect(KEY_4, KEY_SHORT_PRESS))//20.7
//        {
//            a = 4;
//            ips200_show_int (0, 0, a, 1);
//        }
//        if(a == 1)
//        {
//        //pwm_set_duty (MOTOR1_PWM_PIN2, 5000);
//            pwm_set_duty(MOTOR1_PWM_PIN1, 5000);      // P33.9 S1 PWM
//            pwm_set_duty(MOTOR1_PWM_PIN2, 0);
//        }
//        //motor_control(motor_LB, MOTOR_DIR_BRAKE, 10);
//        //pwm_set_duty(MOTOR1_PWM_PIN1, 5000);
//        //pwm_set_duty(MOTOR1_PWM_PIN2, 0);
//        //pwm_set_duty (MOTOR1_PWM_PIN2, 5000);
//        //motor_control(motor_LB, 5000);
//
//        if(a == 0)
//            motor_control(motor_LB, MOTOR_DIR_FORWARD, 0);
//
//        if(a == 3)
//            motor_control(motor_LB, MOTOR_DIR_FORWARD, 30);
//
//        if(a == 4)
//            motor_control(motor_LB, MOTOR_DIR_REVERSE, 30);
        // 此处编写需要循环执行的代码
        InterruptTasks_Poll();

//        menu_first();
//        //task2();
//        task();
        //dot_matrix_screen_set_brightness(5000);
        //dot_matrix_screen_show_string("120");   //120双闪灯
        //dot_matrix_screen_show_string("134");   //134左转向灯
        //dot_matrix_screen_show_string("530");   //530右转向灯
        //dot_matrix_screen_show_string("678");   //678近光灯
        //dot_matrix_screen_show_string("679");     //679远光灯
        //dot_matrix_screen_show_string("67A");     //67A雾灯


        // 此处编写需要循环执行的代码


        6666;
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************

