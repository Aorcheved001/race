/*
 * speaker.c
 *
 *  Created on: 06-三月-26
 *      Author: qinease
 */

//

#include "task.h"

volatile uint8 Led_time = 0;
uint8 cmd_led[3];
volatile uint8 transimit_cmd_led = 0;
volatile uint8 Led_time_state = 0;

void task1(void)
{

}
//dot_matrix_screen_set_brightness(5000);
//dot_matrix_screen_show_string("120");   //120双闪灯
//dot_matrix_screen_show_string("134");   //134左转向灯
//dot_matrix_screen_show_string("530");   //530右转向灯
//dot_matrix_screen_show_string("678");   //678近光灯
//dot_matrix_screen_show_string("679");     //679远光灯
//dot_matrix_screen_show_string("67A");     //67A雾灯

void task2_LED(void)
{
    int j =0;
    for(int i = 1; i <= 8; i++)
    {
        switch(uart1_speech_get_data[i])
        {
            case CMD_LEFT_TURN:
                cmd_led[j] = CMD_LEFT_TURN;
                j++;
            break;

            case CMD_RIGHT_TURN:
                cmd_led[j] = CMD_RIGHT_TURN;
                j++;
            break;
            case CMD_HIGH_BEAM:
                cmd_led[j] = CMD_HIGH_BEAM;
                j++;
            break;
            case CMD_LOW_BEAM:
                cmd_led[j] = CMD_LOW_BEAM;
                j++;
            break;
            case CMD_FOG_LAMP:
                cmd_led[j] = CMD_FOG_LAMP;
                j++;
            break;
            case CMD_HAZARD_LAMP:
                cmd_led[j] = CMD_HAZARD_LAMP;
                j++;
            break;
            case CMD_INTERIOR_LAMP:
                // 打开车内照明灯
                cmd_led[j] = CMD_INTERIOR_LAMP;
                j++;
            break;
            case CMD_WIPER:
                // 打开雨刮器
                cmd_led[j] = CMD_WIPER;
                j++;
            break;
        }
    }

    Led_time_state = 1; //Led_time_state = 1时Led_time开始计时
    if(Led_time < 50)
    {
        transimit_cmd_led = cmd_led[0];
        switch(cmd_led[0])
        {
            case CMD_LEFT_TURN:
                dot_matrix_screen_show_string("134");   //134左转向灯
            break;

            case CMD_RIGHT_TURN:
                dot_matrix_screen_show_string("530");   //530右转向灯
            break;
            case CMD_HIGH_BEAM:
                dot_matrix_screen_show_string("679");     //679远光灯
            break;
            case CMD_LOW_BEAM:
                dot_matrix_screen_show_string("678");   //678近光灯
            break;
            case CMD_FOG_LAMP:
                dot_matrix_screen_show_string("67A");     //67A雾灯
            break;
            case CMD_HAZARD_LAMP:
                dot_matrix_screen_show_string("120");   //120双闪灯
            break;
            case CMD_INTERIOR_LAMP:
                // 打开车内照明灯
            break;
            case CMD_WIPER:
                // 打开雨刮器
            break;
        }
    }

    if(Led_time >= 50)
    {
        transimit_cmd_led = cmd_led[1];
        switch(cmd_led[1])
        {
            case CMD_LEFT_TURN:
                dot_matrix_screen_show_string("134");   //134左转向灯
            break;

            case CMD_RIGHT_TURN:
                dot_matrix_screen_show_string("530");   //530右转向灯
            break;

            case CMD_HIGH_BEAM:
                dot_matrix_screen_show_string("679");     //679远光灯
            break;

            case CMD_LOW_BEAM:
                dot_matrix_screen_show_string("678");   //678近光灯
            break;

            case CMD_FOG_LAMP:
                dot_matrix_screen_show_string("67A");     //67A雾灯
            break;
            case CMD_HAZARD_LAMP:
                dot_matrix_screen_show_string("120");   //120双闪灯
            break;

            case CMD_INTERIOR_LAMP:
                // 打开车内照明灯
            break;

            case CMD_WIPER:
                // 打开雨刮器
            break;
        }
    }
}
void task2_GPS(void)
{

}

void task2_Speaker(void)
{

    if(project_task.task2_state[speech_ok] == ON)
    {
        switch(uart1_speech_get_data[1])
        {
            case CMD_HORN_1S:
                Speaker_Horn_Start(HORN_1S);
                break;
            case CMD_HORN_2S:
                Speaker_Horn_Start(HORN_2S);
                break;
            case CMD_HORN_3S:
                Speaker_Horn_Start(HORN_3S);
                break;
            case CMD_HORN_2BEEP:
                Speaker_Horn_Start(HORN_2BEEP);
                break;
            case CMD_HORN_3BEEP:
                Speaker_Horn_Start(HORN_3BEEP);
                break;
            case CMD_HORN_4BEEP:
                Speaker_Horn_Start(HORN_4BEEP);
                break;
            case CMD_HORN_LONG_SHORT:
                Speaker_Horn_Start(HORN_LONG_SHORT);
                break;
            case CMD_HORN_URGENT:
                Speaker_Horn_Start(HORN_URGENT);
                break;
            case CMD_HORN_ALARM:
                Speaker_Horn_Start(HORN_ALARM);
                break;
        }
    }
}

uint8 code_flag = 1;
uint8 area_state = START_Area;
void task2(void)
{




    if(project_task.task2_state[speech_ok] == ON
    && project_task.task2_state[gps_num_state] == ON
    && project_task.task2_state[go_num_state] == ON)
    {
        switch(area_state)
        {
            //发车区灯光任务
            case START_Area:
                while(Led_time <= 100)    task2_LED();
                area_state = GO_Area;
            break;

            //行进区通过门洞任务
            case GO_Area:


                area_state = ACTION_Area;
                break;
            //动作区车模运动任务
            case ACTION_Area:

                area_state = STOP_Area;
                break;
            //停车区鸣笛任务
            case STOP_Area:
                task2_Speaker();
                break;
        }

    }

}
void task(void)
{
    if(project_task.task2_state[start_num_state] == ON)
    {
        task2();
    }
}

