/*
 * subiect_001.c
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

 //-------------------------------------------头文件引用------------------------------------------------------------
#include "subject_001.h"
#include "track.h"
#include "Ins.h"
#include "PID.h"
#include "steering_control.h"
#include "encoder.h"
#include "menu.h"

 //-------------------------------------------全局变量定义------------------------------------------------------------
INS_DataStruct INS = {0};

 //-------------------------------------------内部函数声明------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      刷新实时状态
 ////  @param      ins_state       INS状态指针
 ////  @return     void
 ////  @note       更新坐标、航向角，循迹时更新到目标点距离
////-------------------------------------------------------------------------------------------------------------------
static void INS_RefreshRealtimeState(const INS_State* ins_state)
{
    if(ins_state == NULL)
    {
        return;
    }

    INS.cod_RealTime.x = ins_state->x;                                   // 更新实时坐标x
    INS.cod_RealTime.y = ins_state->y;                                   // 更新实时坐标y
    INS.Yaw_ins = ins_state->yaw * 57.29578f;                            // 更新航向角（弧度转角度）

    if(track_follow_flag == 1)
    {
        float dx = ins_state->x - Ins_date_377.x;
        float dy = ins_state->y - Ins_date_377.y;
        INS.Dis_ins = sqrtf(dx * dx + dy * dy);                          // 计算到目标点欧氏距离
    }
    else
    {
        INS.Dis_ins = 0.0f;
    }
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      停止所有轨迹动作
 ////  @param      void
 ////  @return     void
 ////  @note       同时停止录制和循迹
////-------------------------------------------------------------------------------------------------------------------
static void INS_StopTrackActions(void)
{
    track_stop_save();
    track_stop_follow();
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      判断轨迹是否正在运行
 ////  @param      void
 ////  @return     uint8           1-运行中 0-未运行
 ////  @note       检查录制和循迹标志
////-------------------------------------------------------------------------------------------------------------------
static uint8 INS_IsTrackRunning(void)
{
    return (track_save_flag == 1 || track_follow_flag == 1) ? 1 : 0;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      处理K4长按事件
 ////  @param      void
 ////  @return     void
 ////  @note       未激活时进入INS模式，激活状态时退出INS模式
////-------------------------------------------------------------------------------------------------------------------
static void INS_HandleKey4LongPress(void)
{
    if(INS.ins_active == 0)
    {
        INS.ins_active = 1;
        INS.sub_mode = INS_SUB_MODE_SAVE;
    }
    else
    {
        INS.ins_active = 0;
        INS_StopTrackActions();
        track_init();
    }

    ips200_clear();
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      处理K1短按事件
 ////  @param      void
 ////  @return     void
 ////  @note       未运行时开始录制/循迹，运行中则停止
////-------------------------------------------------------------------------------------------------------------------
static void INS_HandleKey1ShortPress(void)
{
    if(!INS_IsTrackRunning())
    {
        if(INS.sub_mode == INS_SUB_MODE_SAVE)
        {
            track_start_save();
        }
        else
        {
            track_start_follow();
        }
    }
    else
    {
        INS_StopTrackActions();
    }

    ips200_clear();
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      处理K2短按事件
 ////  @param      void
 ////  @return     void
 ////  @note       切换子模式（录制/循迹），运行中不进行切换
////-------------------------------------------------------------------------------------------------------------------
static void INS_HandleKey2ShortPress(void)
{
    if(INS_IsTrackRunning())
    {
        return;
    }

    INS.sub_mode = (INS.sub_mode == INS_SUB_MODE_SAVE) ? INS_SUB_MODE_FOLLOW : INS_SUB_MODE_SAVE;
    ips200_clear();
}

static void INS_HandleKey2LongPress(void)
{
    if(INS_IsTrackRunning())
    {
        return;
    }

    ips200_clear();
    ips200_show_string(0, 80, "Offline Mag Calib");
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      处理K3短按事件
 ////  @param      void
 ////  @return     void
 ////  @note       录制模式且正在录制时，停止录制
////-------------------------------------------------------------------------------------------------------------------
static void INS_HandleKey3ShortPress(void)
{
    if(INS.sub_mode == INS_SUB_MODE_SAVE && track_save_flag == 1)
    {
        track_stop_save();
        ips200_clear();
    }
}

 //-------------------------------------------功能函数------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      INS模块初始化
 ////  @param      void
 ////  @return     void
////-------------------------------------------------------------------------------------------------------------------
void subject1_init(void)
{
    INS.ins_active = 0;                                                  // INS激活标志清零
    INS.sub_mode = INS_SUB_MODE_SAVE;                                    // 默认子模式为录制模式
    INS.cod_RealTime.x = 0.0f;                                           // 实时坐标x清零
    INS.cod_RealTime.y = 0.0f;                                           // 实时坐标y清零
    INS.Dis_ins = 0.0f;                                                  // 距离清零
    INS.Yaw_ins = 0.0f;                                                  // 航向角清零
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      INS导航任务
 ////  @param      void
 ////  @return     void
 ////  @note       8ms周期调用，处理按键和模式切换
////-------------------------------------------------------------------------------------------------------------------
void subject1_update(void)
{
    const INS_State* ins_state = Ins_get_state();                        // 获取INS状态
    INS_RefreshRealtimeState(ins_state);

    if(key_detect(KEY_4, KEY_LONG_PRESS))
    {
        printf("1 ");
        INS_HandleKey4LongPress();
        return;
    }

    if(INS.ins_active == 0)
    {
        return;
    }

    if(key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        INS_HandleKey1ShortPress();
    }

    if(key_detect(KEY_2, KEY_SHORT_PRESS))
    {
        INS_HandleKey2ShortPress();
    }

    if(key_detect(KEY_2, KEY_LONG_PRESS))
    {
        INS_HandleKey2LongPress();
    }

    if(key_detect(KEY_3, KEY_SHORT_PRESS))
    {
        INS_HandleKey3ShortPress();
    }
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      INS显示函数
 ////  @param      void
 ////  @return     void
 ////  @note       40ms周期调用，刷新屏幕显示
////-------------------------------------------------------------------------------------------------------------------
void subject1_display(void)
{
    if(INS.ins_active == 0)                                              // INS未激活时不显示
    {
        return;
    }

    const INS_State* ins_state = Ins_get_state();                        // 获取INS状态
    const EncoderLayerState* enc = encoder_layer_get_state();            // 获取编码器状态

    ips200_show_string(0, 0, "INS Control");
    if(gnss_get_state()->fix_type > 0) ips200_show_string(100, 0, "GPS:OK");
    else                               ips200_show_string(100, 0, "GPS:NO");

    ips200_show_string(0, 16, "Mode:");
    if(INS.sub_mode == INS_SUB_MODE_SAVE)
        ips200_show_string(48, 16, "SAVE   ");
    else
        ips200_show_string(48, 16, "FOLLOW ");

    // 显示各层yaw值
    float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
    Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

    ips200_show_string(0, 32, "Gyro:");
    ips200_show_float(40, 32, yaw_gyro * 57.29578f, 3, 1);


    ips200_show_string(0, 48, "M_Raw:");
    ips200_show_float(48, 48, yaw_mag_raw * 57.29578f, 3, 1);
    ips200_show_string(105, 48, "M_Rel:");
    ips200_show_float(155, 48, yaw_mag_rel * 57.29578f, 3, 1);

    ips200_show_string(0, 64, "Fin Yaw:");
    ips200_show_float(80, 64, yaw_ekf * 57.29578f, 3, 2);
    ips200_show_string(140, 64, "deg");

    // 显示模式信息
    if(INS.sub_mode == INS_SUB_MODE_SAVE)
    {
        ips200_show_string(0, 80, "Cur X:");
        ips200_show_float(48, 80, INS.cod_RealTime.x, 3, 2);
        ips200_show_string(110, 80, "Y:");
        ips200_show_float(128, 80, INS.cod_RealTime.y, 3, 2);

        ips200_show_string(0, 96, "Points:");
        ips200_show_uint(56, 96, track_total_points, 4);

        ips200_show_string(120, 96, "Spd:");
        ips200_show_float(152, 96, enc->speed_average_mps, 1, 2);
        ips200_show_string(192, 96, "m/s");
    }
    else
    {
        if(track_follow_flag == 1)                                       // 正在循迹
        {
            ips200_show_string(0, 80, "Tgt X:");
            ips200_show_float(48, 80, Ins_date_377.x, 3, 2);
            ips200_show_string(110, 80, "Y:");
            ips200_show_float(128, 80, Ins_date_377.y, 3, 2);

            ips200_show_string(0, 96, "Dis:");
            ips200_show_float(32, 96, INS.Dis_ins, 3, 2);
            ips200_show_string(100, 96, "m");

            ips200_show_string(120, 96, "Spd:");
            ips200_show_float(152, 96, enc->speed_average_mps, 1, 2);
            ips200_show_string(192, 96, "m/s");
        }
        else                                                             // 循迹停止
        {
            ips200_show_string(0, 80, "Track Ready!     ");
            ips200_show_string(0, 96, "Points:");
            ips200_show_uint(56, 96, track_total_points, 4);

            ips200_show_string(120, 96, "Spd:");
            ips200_show_float(152, 96, enc->speed_average_mps, 1, 2);
            ips200_show_string(192, 96, "m/s");
        }
    }

    if(INS.sub_mode == INS_SUB_MODE_SAVE)
    {
        if(track_save_flag == 1)
            ips200_show_string(0, 112, "K1:Stop K3:End K4:Ex");
        else
            ips200_show_string(0, 112, "K1:Run K2:M/C K4:Ex");
    }
    else
    {
        if(track_follow_flag == 1)
            ips200_show_string(0, 112, "K1:Stop K2:--  K4:Ex");
        else
            ips200_show_string(0, 112, "K1:Run K2:M/C K4:Ex");
    }

    if(imu_mag_calib_is_active())
    {
        ips200_show_string(150, 96, "CAL");
        ips200_show_string(0, 80, "Mag Samples:");
        ips200_show_uint(88, 80, imu_mag_calib_get_sample_count(), 3);
        ips200_show_string(120, 80, "/500");
    }
}
