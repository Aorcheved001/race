#include "zf_common_headfile.h"
#include "control.h"
#include "MOTOR.h"
#include "PID.h"
#include "encoder.h"
#include "Ins.h"
#include "steering_control.h"
#include "track.h"
#include "yaokong.h"
#include "subject_001.h"

CONTRO_CENTER Yao;

// INS 输入缓存（原 interrupt.c 中定义）
static INS_Input s_ins_input = {0};

// 磁航向测量辅助函数（从旧 interrupt.c 迁移）
static float normalize_angle_rad_local(float angle)
{
    while (angle > INS_PI) angle -= 2.0f * INS_PI;
    while (angle < -INS_PI) angle += 2.0f * INS_PI;
    return angle;
}

static uint8 get_mag_yaw_measurement(float *yaw_rad)
{
    float mag_x = imu660.data_Ripen.mag_x;
    float mag_y = imu660.data_Ripen.mag_y;

    if(yaw_rad == NULL)
    {
        return 0u;
    }

    if((fabsf(mag_x) + fabsf(mag_y)) < 1e-4f)
    {
        return 0u;
    }

    *yaw_rad = normalize_angle_rad_local(atan2f(mag_y, mag_x));
    return 1u;
}

/*****************---------中断执行函数---------*****************/
void task_1ms(void)
{
    // 时间戳由 CCU60_CH0 -> Interrupt_1ms() 维护，避免双计数
}

void task_2ms(void)
{
//    key_scanner();
}

void task_4ms(void)
{
    // ---- 1. 编码器层更新（内部已完成 读硬件+立即清零，原子操作） ----
    encoder_layer_update();

    // ---- 2. 将 4ms 脉冲值拷贝到新字段（供新 PID 读取） ----
    g_state.tick_left_4ms  = g_state.tick_left;
    g_state.tick_right_4ms = g_state.tick_right;

    // ---- 3. 电机输出 ----
    if (Yao.flag_motor_start_user == 1)
    {
        // PID 闭环（使用 4ms 脉冲值）
        Yao.OutP_LB += PID_Increase_vLB(&Yao_pid.vLB_pid, param_vLB,
                                         g_state.tick_left_4ms, Yao.Target_Speed_L);
        Yao.OutP_RB += PID_Increase_vRB(&Yao_pid.vRB_pid, param_vRB,
                                         g_state.tick_right_4ms, Yao.Target_Speed_R);

        // PID 输出限幅
        if (Yao.OutP_LB >  param_vLB[3]) Yao.OutP_LB =  param_vLB[3];
        if (Yao.OutP_LB < -param_vLB[3]) Yao.OutP_LB = -param_vLB[3];
        if (Yao.OutP_RB >  param_vRB[3]) Yao.OutP_RB =  param_vRB[3];
        if (Yao.OutP_RB < -param_vRB[3]) Yao.OutP_RB = -param_vRB[3];

        Motor_Ctrl(MOTOR_L, (int32)Yao.OutP_LB);
        Motor_Ctrl(MOTOR_R, (int32)Yao.OutP_RB);
    }
    else
    {
        Yao.OutP_LB = 0;
        Yao.OutP_RB = 0;
        Motor_Ctrl(MOTOR_L, 0);
        Motor_Ctrl(MOTOR_R, 0);
    }



}

void task_8ms(void)
{

}

void task_10ms(void)
{


}

void task_16ms(void)
{



}

void task_20ms(void)
{


}



void task_40ms(void)
{
}
