#include "zf_common_headfile.h"

#ifndef _All_Ctrl_h
#define _All_Ctrl_h

typedef struct
{
    //电机输出
    float OutP_LB;
    float OutP_RB;

    //常用
    int16 Target_Speed_L;
    int16 Target_Speed_R;

    bool flag_motor_start_user;

} CONTRO_CENTER;

extern CONTRO_CENTER Yao;

void task_1ms(void);
void task_2ms(void);
void task_4ms(void);
void task_8ms(void);
void task_10ms(void);
void task_16ms(void);
void task_20ms(void);
void task_40ms(void);


#endif
