#include "zf_common_headfile.h"
#include "steering_control.h"

PID_ERECT Yao_pid;

//左右平衡 PID 参数: {Kp, Ki, Kd, OUT_LIMIT}
float param_vLB[4]    = { 0  ,    20    ,  0 , 4000 };
float param_vRB[4]    = { 0  ,    20    ,  0 , 4000 };




//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置目标速度（单向差速 + 倒车支持）
//  @param      target  目标脉冲值（脉冲/4ms），直接设置 Yao.Target_Speed_L/R
//  @note       单向差速策略：转弯时只加速外侧轮，不减速内侧轮
//              - 正向右转：左轮+delta（加速），右轮保持target（不减速）
//              - 正向左转：右轮+delta（加速），左轮保持target（不减速）
//              - 倒车时差速方向自动反转（倒车右转=后轮视角左转）
//              优势：内侧轮保持足够驱动力，不易打滑
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_set_target_speed(int16 target)
{
    // 获取差速偏移：正值=辅助右转（左轮加速，右轮减速），负值=辅助左转
    float diff = steering_get_diff_delta();

    float target_l = (float)target;
    float target_r = (float)target;

    // 判断行驶方向
    if(target >= 0)
    {
        // ===== 正向行驶 =====
        if(diff > 0)
        {
            // 辅助右转：只加速左轮（外侧），不减速右轮（内侧）
            target_l = target + diff;
            // 右轮保持target不变
        }
        else if(diff < 0)
        {
            // 辅助左转：只加速右轮（外侧），不减速左轮（内侧）
            target_r = target + (-diff);  // -diff为正值
            // 左轮保持target不变
        }
        // diff==0: 两侧都保持target

        // 安全保护：确保不降到0以下（不反转）
        if(target_l < 0) target_l = 0;
        if(target_r < 0) target_r = 0;
    }
    else
    {
        // ===== 倒车行驶（target < 0）=====
        // 倒车时差速方向反转：
        //   正向右转(diff>0) → 倒车时车尾右摆 → 需要右轮加速（倒车外侧）
        //   正向左转(diff<0) → 倒车时车尾左摆 → 需要左轮加速（倒车外侧）
        if(diff > 0)
        {
            // 倒车右转（车尾右摆）：只加速右轮（倒车外侧），不减速左轮
            target_r = target - diff;  // target为负，-diff为负，更负=倒车更快
            // 左轮保持target不变
        }
        else if(diff < 0)
        {
            // 倒车左转（车尾左摆）：只加速左轮（倒车外侧），不减速右轮
            target_l = target + diff;  // target为负，diff为负，更负=倒车更快
            // 右轮保持target不变
        }

        // 安全保护：确保不升到0以上（不反转）
        if(target_l > 0) target_l = 0;
        if(target_r > 0) target_r = 0;
    }

    Yao.Target_Speed_L = (int16)target_l;
    Yao.Target_Speed_R = (int16)target_r;
}





/*****************---------外-速度---------*****************/
float PID_Position( PID_INFO *pid_info , float * PID_Parm , float NowPoint , float SetPoint )
{
    float V_out;
    float a = 0.5;

    // 1.计算速度偏差
    pid_info->iError = ( NowPoint - SetPoint );

    // 2.对速度偏差进行低通滤波
    pid_info->iError = (1-a) * pid_info->iError + a * pid_info->LastError;  // 使得波形更加平滑，滤除高频干扰，防止速度突变。
    pid_info->SumError += pid_info->iError;

    // 3.积分限幅
    if( PID_Parm[3] )
    {
        if (pid_info->SumError >  PID_Parm[3]) pid_info->SumError =  PID_Parm[3];
        if (pid_info->SumError < -PID_Parm[3]) pid_info->SumError = -PID_Parm[3];
    }

    // 4.计算输出
    V_out = PID_Parm[KP] * pid_info->iError +
            PID_Parm[KI] * pid_info->SumError +
            PID_Parm[KD] * ( pid_info->iError - pid_info->LastError );
    pid_info->LastError = pid_info->iError;

    return V_out;
}


/*****************---------速度环PID---------*****************/
float PID_Increase_vLB( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint )
{
    float Increase;

    pid_info->iError = SetPoint - NowPoint;

    Increase = PID_Parm[KP] * ( pid_info->iError - pid_info->LastError ) +
               PID_Parm[KI] * pid_info->iError +
               PID_Parm[KD] * ( pid_info->iError - 2*pid_info->LastError + pid_info->PrevError );

    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = pid_info->iError;
    pid_info->LastData = NowPoint;

    return Increase;
}
float PID_Increase_vRB( PID_INFO *pid_info , float *PID_Parm , float NowPoint , float SetPoint )
{
    float Increase;

    pid_info->iError = SetPoint - NowPoint;

    Increase = PID_Parm[KP] * ( pid_info->iError - pid_info->LastError ) +
               PID_Parm[KI] * pid_info->iError +
               PID_Parm[KD] * ( pid_info->iError - 2*pid_info->LastError + pid_info->PrevError );

    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = pid_info->iError;
    pid_info->LastData = NowPoint;

    return Increase;
}

/*****************---------PID参数初始化---------*****************/
void pid_para_init( PID_INFO *pid_info )
{
    pid_info->iError = 0;
    pid_info->SumError = 0;
    pid_info->PrevError = 0;
    pid_info->LastError = 0;
    pid_info->LastData = 0;
}
