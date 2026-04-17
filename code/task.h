/*
 * speaker.h
 *
 *  Created on: 06-三月-26
 *      Author: LENOVO
 */

#ifndef CODE_TASK_H_
#define CODE_TASK_H_

#include "zf_common_headfile.h"

#define OFF                         0
#define ON                          1

#define START_Area                  0
#define GO_Area                     1
#define ACTION_Area                 2
#define STOP_Area                   3

#define task2_state_num             6

#define start_num_state             0
#define gps_num_state               1
#define speech_num_state            2   //对应start speech
#define speech_disp_num_state       3   //对应display
#define speech_ok                   4   //对应make sure
#define go_num_state                5

extern volatile uint8 Led_time;
extern volatile uint8 Led_time_state;
extern volatile uint8 transimit_cmd_led;


//task2_state  ：有GPS存点,灯光秀,语音,鸣笛,go运动
//task2_state[0]:start标志位
//task2_state[1]:gps存点 标志位
//task2_state[2]：语音标志位
typedef struct
{
     uint8 task1_state;
     uint8 task2_state[task2_state_num];
     uint8 task3_state;
     uint8 task4_state;
     uint8 drive_state;
}task_state;

extern task_state project_task;

void task(void);

#endif /* CODE_TASK_H_ */


