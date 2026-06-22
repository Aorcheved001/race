#include "zf_common_headfile.h"

#ifndef _FLY_MOTOR_PID_h
#define _FLY_MOTOR_PID_h

#define KP 0
#define KI 1
#define KD 2
/*****************---------结构体---------*****************/
typedef struct
{
    float iError;                 // 误差
    float LastError;              // 上次误差
    float PrevError;              // 上上次误差
    float LastData;               // 上次数据
    float iErrorHistory[5];       // 历史误差
    float SumError;               // 累计误差
} PID_INFO;

typedef struct
{
    PID_INFO vLB_pid;                   // 左轮速度环
    PID_INFO vRB_pid;                   // 右轮速度环

} PID_ERECT;

extern PID_ERECT Yao_pid;                 //PID

/*****************---------结构体---------*****************/


/*****************---------PID参数---------*****************/
extern float erect__speed[3];                 //速度策略

extern float erect_yawan[4];                  //开环压弯


extern float erect_car_speed[4];              // 小车真实速度环

extern float erect_dynamic_balance[4];        // 动态机械零点环

// 串级PID左右平衡
extern float param_vLB[4];
extern float param_vRB[4];

/*****************---------PID参数---------*****************/


/*****************---------函数---------*****************/

float PID_Position( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint );
float PID_Increase_vLB( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
float PID_Increase_vRB( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint );
// 参数初始化
void pid_para_init(PID_INFO *pid_info);

// 目标速度设置（脉冲/4ms，int16 直接设脉冲值）
void wheel_pid_set_target_speed(int16 target);

/*****************---------函数---------*****************/

#endif

