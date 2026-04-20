/*
 * ins_new_264.c
 *
<<<<<<< HEAD
 * Created on: 2024��6��6��
 * Author: LateRain
 * Modified: 2025��11��22��
 */

 //-------------------------------------------ͷ�ļ���------------------------------------------------------------
=======
 * Created on: 2024??6??6??
 * Author: LateRain
 * Modified: 2025??11??22??
 */

 //-------------------------------------------??????------------------------------------------------------------
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
#include "ins_new_264.h"
#include "track.h"
#include "Ins.h"
#include "PID.h"
#include "servo.h"
#include "encoder.h"
#include "menu.h"

<<<<<<< HEAD
 //-------------------------------------------ȫ�ֱ�����------------------------------------------------------------
INS_DataStruct INS = {0};

 //-------------------------------------------�ڲ�����������------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ˢ��ʵʱ״̬
 ////  @param      ins_state       INS״ָ̬��
 ////  @return     void
 ////  @note       �������ꡢ����ǣ�ѭ��ʱ���㵽Ŀ������
=======
 //-------------------------------------------????????------------------------------------------------------------
INS_DataStruct INS = {0};

 //-------------------------------------------?????????????------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ???????
 ////  @param      ins_state       INS?????
 ////  @return     void
 ////  @note       ?????????????????????????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////-------------------------------------------------------------------------------------------------------------------
static void INS_RefreshRealtimeState(const INS_State* ins_state)
{
    if(ins_state == NULL)
    {
        return;
    }

<<<<<<< HEAD
    INS.cod_RealTime.x = ins_state->x;                                   // ����ʵʱ����x
    INS.cod_RealTime.y = ins_state->y;                                   // ����ʵʱ����y
    INS.Yaw_ins = ins_state->yaw * 57.29578f;                            // ���º���ǣ�����ת�Ƕȣ�
=======
    INS.cod_RealTime.x = ins_state->x;                                   // ??????????x
    INS.cod_RealTime.y = ins_state->y;                                   // ??????????y
    INS.Yaw_ins = ins_state->yaw * 57.29578f;                            // ??????????????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

    if(track_follow_flag == 1)
    {
        float dx = ins_state->x - Ins_date_377.x;
        float dy = ins_state->y - Ins_date_377.y;
<<<<<<< HEAD
        INS.Dis_ins = sqrtf(dx * dx + dy * dy);                          // ���㵽Ŀ����ŷ�Ͼ���
=======
        INS.Dis_ins = sqrtf(dx * dx + dy * dy);                          // ???????????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    }
    else
    {
        INS.Dis_ins = 0.0f;
    }
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ֹͣ���й켣����
 ////  @param      void
 ////  @return     void
 ////  @note       ͬʱֹͣ����ѭ��
=======
 ////  @brief      ?????��?????
 ////  @param      void
 ////  @return     void
 ////  @note       ???????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////-------------------------------------------------------------------------------------------------------------------
static void INS_StopTrackActions(void)
{
    track_stop_save();
    track_stop_follow();
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      �жϹ켣�Ƿ���������
 ////  @param      void
 ////  @return     uint8           1-������ 0-δ����
 ////  @note       ������ѭ����־
=======
 ////  @brief      ?��?????????????
 ////  @param      void
 ////  @return     uint8           1-?????? 0-��????
 ////  @note       ????????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////-------------------------------------------------------------------------------------------------------------------
static uint8 INS_IsTrackRunning(void)
{
    return (track_save_flag == 1 || track_follow_flag == 1) ? 1 : 0;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ����K4�����¼�
 ////  @param      void
 ////  @return     void
 ////  @note       δ����ʱ����INSģʽ���Ѽ���ʱ�˳�INSģʽ
=======
 ////  @brief      ????K4???????
 ////  @param      void
 ////  @return     void
 ////  @note       ��?????????INS?????????????INS??
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
<<<<<<< HEAD
 ////  @brief      ����K1�̰��¼�
 ////  @param      void
 ////  @return     void
 ////  @note       δ����ʱ��ʼ���/ѭ������������ֹͣ
=======
 ////  @brief      ????K1??????
 ////  @param      void
 ////  @return     void
 ////  @note       ��???????????/???????????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
<<<<<<< HEAD
 ////  @brief      ����K2�̰��¼�
 ////  @param      void
 ////  @return     void
 ////  @note       �л���ģʽ�����/ѭ�����������в������л�
=======
 ////  @brief      ????K2??????
 ////  @param      void
 ////  @return     void
 ////  @note       ?��??????????/????????????��??????��?
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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

    if(imu_mag_calib_is_active())
    {
<<<<<<< HEAD
        imu_mag_calib_finish(&imu_date);
    }
    else
    {
        imu_mag_calib_start();
=======
        // 完成椭球校准并保存到Flash
        if(imu_mag_calib_ellipsoid_finish(&imu_date))
        {
            ips200_show_string(0, 80, "Calib OK! Saved");
        }
        else
        {
            ips200_show_string(0, 80, "Calib Failed");
        }
    }
    else
    {
        // 启动椭球校准
        imu_mag_calib_ellipsoid_start();
        ips200_show_string(0, 80, "Calib Started");
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    }

    ips200_clear();
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ����K3�̰��¼�
 ////  @param      void
 ////  @return     void
 ////  @note       �����ģʽ����Ч��ֹͣ���
=======
 ////  @brief      ????K3??????
 ////  @param      void
 ////  @return     void
 ////  @note       ???????????��???????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////-------------------------------------------------------------------------------------------------------------------
static void INS_HandleKey3ShortPress(void)
{
    if(INS.sub_mode == INS_SUB_MODE_SAVE && track_save_flag == 1)
    {
        track_stop_save();
        ips200_clear();
    }
}

<<<<<<< HEAD
 //-------------------------------------------����������------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      INSģ���ʼ��
=======
 //-------------------------------------------??????????------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      INS???????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void INS_init(void)
{
<<<<<<< HEAD
    INS.ins_active = 0;                                                  // INS�����־����
    INS.sub_mode = INS_SUB_MODE_SAVE;                                    // Ĭ����ģʽΪ���ģʽ
    INS.cod_RealTime.x = 0.0f;                                           // ʵʱ����x����
    INS.cod_RealTime.y = 0.0f;                                           // ʵʱ����y����
    INS.Dis_ins = 0.0f;                                                  // ��������
    INS.Yaw_ins = 0.0f;                                                  // ���������
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      INS��������
 ////  @param      void
 ////  @return     void
 ////  @note       ���ڵ��ã��������������״̬�л�
 ////-------------------------------------------------------------------------------------------------------------------
void INS_NavigationTask(void)
{
    const INS_State* ins_state = Ins_get_state();                        // ��ȡINS״̬
=======
    INS.ins_active = 0;                                                  // INS??????????
    INS.sub_mode = INS_SUB_MODE_SAVE;                                    // ?????????????
    INS.cod_RealTime.x = 0.0f;                                           // ??????x????
    INS.cod_RealTime.y = 0.0f;                                           // ??????y????
    INS.Dis_ins = 0.0f;                                                  // ????????
    INS.Yaw_ins = 0.0f;                                                  // ?????????
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      INS????????
 ////  @param      void
 ////  @return     void
 ////  @note       ????????????????????????��?
 ////-------------------------------------------------------------------------------------------------------------------
void INS_NavigationTask(void)
{
    const INS_State* ins_state = Ins_get_state();                        // ???INS??
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    INS_RefreshRealtimeState(ins_state);

    if(key_detect(KEY_4, KEY_LONG_PRESS))
    {
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
<<<<<<< HEAD
 ////  @brief      INS��ʾ����
 ////  @param      void
 ////  @return     void
 ////  @note       ���ڵ��ã�ˢ����Ļ��ʾ
 ////-------------------------------------------------------------------------------------------------------------------
void INS_Display(void)
{
    if(INS.ins_active == 0)                                              // INSδ����ʱ����ʾ
=======
 ////  @brief      INS???????
 ////  @param      void
 ////  @return     void
 ////  @note       ?????????????????
 ////-------------------------------------------------------------------------------------------------------------------
void INS_Display(void)
{
    if(INS.ins_active == 0)                                              // INS��??????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    {
        return;
    }

<<<<<<< HEAD
    const INS_State* ins_state = Ins_get_state();                        // ��ȡINS״̬
    const EncoderLayerState* enc = encoder_layer_get_state();            // ��ȡ������״̬
=======
    const INS_State* ins_state = Ins_get_state();                        // ???INS??
    const EncoderLayerState* enc = encoder_layer_get_state();            // ???????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

    ips200_show_string(0, 0, "INS Control");
    if(gnss_get_state()->fix_type > 0) ips200_show_string(100, 0, "GPS:OK");
    else                               ips200_show_string(100, 0, "GPS:NO");

    ips200_show_string(0, 16, "Mode:");
    if(INS.sub_mode == INS_SUB_MODE_SAVE)
        ips200_show_string(48, 16, "SAVE   ");
    else
        ips200_show_string(48, 16, "FOLLOW ");

<<<<<<< HEAD
    // ��ʾ����yawֵ
=======
    // ???????yaw?
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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

<<<<<<< HEAD
    // ��ʾ������Ϣ
=======
    // ??????????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
<<<<<<< HEAD
        if(track_follow_flag == 1)                                       // ����ѭ��
=======
        if(track_follow_flag == 1)                                       // ???????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
<<<<<<< HEAD
        else                                                             // ѭ������
=======
        else                                                             // ???????
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
