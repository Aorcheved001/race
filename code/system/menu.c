/*
 * menu.c
<<<<<<< HEAD
 * ï¿½Ëµï¿½ï¿½ï¿½Ê¾ÏµÍ³
 *
 * Created on: 2024ï¿½ï¿½6ï¿½ï¿½6ï¿½ï¿½
 * Author: LateRain
 * Modified: 2025ï¿½ï¿½11ï¿½ï¿½22ï¿½ï¿½
 */

 //-------------------------------------------Í·ï¿½Ä¼ï¿½ï¿½ï¿½------------------------------------------------------------
#include "zf_common_headfile.h"
#include "steering_test.h"

 //-------------------------------------------È«ï¿½Ö±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½-----------------------------------------------------------
static int8 page = 1;                                                     // ï¿½ï¿½Ç°ï¿½ï¿½Ê¾Ò³ï¿½ï¿½

#define MAX_PAGE 9                                                        // ï¿½ï¿½ï¿½Ò³ï¿½ï¿½ï¿½ï¿½

 //-------------------------------------------ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½------------------------------------------------------------
////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ï¿½ï¿½â°´ï¿½ï¿½×´Ì¬
 ////  @param      key_n       ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 ////  @param      state       ï¿½ï¿½ï¿½ï¿½×´Ì¬ (ï¿½Ì°ï¿½/ï¿½ï¿½ï¿½ï¿½)
 ////  @return     int8        1ï¿½ï¿½Ê¾ï¿½ï¿½âµ½ï¿½ï¿½Ó¦×´Ì¬ï¿½ï¿½0ï¿½ï¿½Ê¾Î´ï¿½ï¿½âµ½
 ////  @note       ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä¶Ì°ï¿½ï¿½Í³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 ////-------------------------------------------------------------------------------------------------------------------
int key_detect(key_index_enum key_n, key_state_enum state)
{
    static uint8 long_press_flag[KEY_NUMBER] = {0};                       // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¾ï¿½ï¿½ï¿½ï¿½

    key_state_enum current_state = key_get_state(key_n);                  // ï¿½ï¿½È¡ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½×´Ì¬
=======
 * ²Ëµ¥ÏÔÊ¾ÏµÍ³
 *
 * Created on: 2024Äê6ÔÂ6ÈÕ
 * Author: LateRain
 * Modified: 2025Äê11ÔÂ22ÈÕ
 */

 //-------------------------------------------Í·ÎÄ¼þÇø------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------È«¾Ö±äÁ¿¶¨ÒåÇø-----------------------------------------------------------
static int8 page = 1;                                                     // µ±Ç°ÏÔÊ¾Ò³Ãæ

#define MAX_PAGE 8                                                        // ×î´óÒ³ÃæÊý

 //-------------------------------------------º¯ÊýÉùÃ÷Çø------------------------------------------------------------
////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ¼ì²â°´¼ü×´Ì¬
 ////  @param      key_n       °´¼üË÷Òý
 ////  @param      state       °´¼ü×´Ì¬ (¶Ì°´/³¤°´)
 ////  @return     int8        1±íÊ¾¼ì²âµ½¶ÔÓ¦×´Ì¬£¬0±íÊ¾Î´¼ì²âµ½
 ////  @note       ´¦Àí°´¼üµÄ¶Ì°´ºÍ³¤°´¼ì²â
 ////-------------------------------------------------------------------------------------------------------------------
int key_detect(key_index_enum key_n, key_state_enum state)
{
    static uint8 long_press_flag[KEY_NUMBER] = {0};                       // ³¤°´±êÖ¾Êý×é

    key_state_enum current_state = key_get_state(key_n);                  // »ñÈ¡µ±Ç°°´¼ü×´Ì¬
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

    if(state == KEY_LONG_PRESS)
    {
        if(current_state == KEY_LONG_PRESS)
        {
            if(long_press_flag[key_n] == 0)
            {
                long_press_flag[key_n] = 1;
                key_clear_state(key_n);
                return 1;
            }
        }
        else if(current_state == KEY_RELEASE)
        {
            long_press_flag[key_n] = 0;
        }
    }
    else if(state == KEY_SHORT_PRESS)
    {
        if(current_state == KEY_SHORT_PRESS)
        {
            key_clear_state(key_n);
            return 1;
        }
    }

    return 0;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ï¿½Ëµï¿½ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 ////  @param      void
 ////  @return     void
 ////  @note       ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½É¨ï¿½ï¿½ï¿½Ò³ï¿½ï¿½ï¿½ï¿½Ê¾
=======
 ////  @brief      ²Ëµ¥ÏÔÊ¾Ö÷º¯Êý
 ////  @param      void
 ////  @return     void
 ////  @note       ´¦Àí°´¼üÉ¨ÃèºÍÒ³ÃæÏÔÊ¾
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////-------------------------------------------------------------------------------------------------------------------
void menu(void)
{
    if(g_key_scan_flag)
    {
        g_key_scan_flag = 0;
        key_scanner();
    }

<<<<<<< HEAD
    const INS_State* ins_state = Ins_get_state();                         // ï¿½ï¿½È¡INS×´Ì¬
=======
    const INS_State* ins_state = Ins_get_state();                         // »ñÈ¡INS×´Ì¬
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

    if(key_detect(KEY_3, KEY_SHORT_PRESS))
    {
        ips200_clear();
<<<<<<< HEAD
        if(++page > MAX_PAGE) page = 0;                                   // ï¿½ï¿½Ò»Ò³
=======
        if(++page > MAX_PAGE) page = 0;                                   // ÏÂÒ»Ò³
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    }
    if(key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        ips200_clear();
<<<<<<< HEAD
        if(--page < 0) page = MAX_PAGE;                                   // ï¿½ï¿½Ò»Ò³
=======
        if(--page < 0) page = MAX_PAGE;                                   // ÉÏÒ»Ò³
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    }

    switch(page)
    {
        case 0:
            ips200_show_string(20, 0 * 16, "--- System Home ---");
            ips200_show_string(20, 1 * 16, "Core: TC377 Aurix");
            ips200_show_string(20, 2 * 16, "Menu: Minimal v1.0");
            ips200_show_string(20, 4 * 16, "Press K1/K3 to Flip");
            break;

        case 1:
        {
            const gnss_state_t* gps_ptr = gnss_get_state();
            ips200_show_string(20, 0 * 16, "--- GPS Status ---");
            ips200_show_string(20, 1 * 16, "Status:");
            if(gps_ptr->fix_type > 0)
                ips200_show_string(100, 1 * 16, "FIXED    ");
            else
                ips200_show_string(100, 1 * 16, "SEARCHING");

            ips200_show_string(20, 2 * 16, "Sats  :");
            ips200_show_uint(100, 2 * 16, gps_ptr->num_sats, 2);

            ips200_show_string(20, 3 * 16, "Height:");
            ips200_show_float(100, 3 * 16, gps_ptr->alt_m, 3, 2);
            ips200_show_string(160, 3 * 16, "m");

            ips200_show_string(20, 5 * 16, "Date  :");
            ips200_show_string(100, 5 * 16, "----/--/--");
            ips200_show_uint(100, 5 * 16, gnss.time.year, 4);
            ips200_show_string(140, 5 * 16, "/");
            ips200_show_uint(150, 5 * 16, gnss.time.month, 2);
            ips200_show_string(170, 5 * 16, "/");
            ips200_show_uint(180, 5 * 16, gnss.time.day, 2);

            ips200_show_string(20, 6 * 16, "Time  :");
            ips200_show_string(100, 6 * 16, "--:--:--");
            ips200_show_uint(100, 6 * 16, gnss.time.hour, 2);
            ips200_show_string(120, 6 * 16, ":");
            ips200_show_uint(130, 6 * 16, gnss.time.minute, 2);
            ips200_show_string(150, 6 * 16, ":");
            ips200_show_uint(160, 6 * 16, gnss.time.second, 2);
            break;
        }

        case 2:
            ips200_show_string(20, 0 * 16, "--- GPS Position ---");
            ips200_show_string(20, 1 * 16, "Lat :");
            ips200_show_float(80, 1 * 16, (float)gnss_get_state()->lat_deg, 3, 6);
            ips200_show_string(20, 2 * 16, "Lon :");
            ips200_show_float(80, 2 * 16, (float)gnss_get_state()->lon_deg, 3, 6);
            ips200_show_string(20, 4 * 16, "--- GPS Motion ---");
            ips200_show_string(20, 5 * 16, "Speed:");
            ips200_show_float(100, 5 * 16, gnss.speed, 3, 2);
            ips200_show_string(160, 5 * 16, "km/h");
            ips200_show_string(20, 6 * 16, "Dir  :");
            ips200_show_float(100, 6 * 16, gnss.direction, 3, 1);
            ips200_show_string(160, 6 * 16, "deg");
            break;

        case 3:
            ips200_show_string(20, 0 * 16, "--- IMU Accel ---");
            ips200_show_string(20, 1 * 16, "Acc_X:");
            ips200_show_float(100, 1 * 16, imu660.data_Ripen.acc_x, 3, 3);
            ips200_show_string(20, 2 * 16, "Acc_Y:");
            ips200_show_float(100, 2 * 16, imu660.data_Ripen.acc_y, 3, 3);
            ips200_show_string(20, 3 * 16, "Acc_Z:");
            ips200_show_float(100, 3 * 16, imu660.data_Ripen.acc_z, 3, 3);
            ips200_show_string(20, 5 * 16, "Unit : m/s2");
            break;

        case 4:
            ips200_show_string(20, 0 * 16, "--- IMU Gyro ---");
            ips200_show_string(20, 1 * 16, "Gyr_X:");
            ips200_show_float(100, 1 * 16, imu660.data_Ripen.gyro_x, 3, 3);
            ips200_show_string(20, 2 * 16, "Gyr_Y:");
            ips200_show_float(100, 2 * 16, imu660.data_Ripen.gyro_y, 3, 3);
            ips200_show_string(20, 3 * 16, "Gyr_Z:");
            ips200_show_float(100, 3 * 16, imu660.data_Ripen.gyro_z, 3, 3);
            ips200_show_string(20, 5 * 16, "Unit : rad/s");
            break;

        case 5:
            ips200_show_string(20, 0 * 16, "--- IMU Magnet ---");
            ips200_show_string(20, 1 * 16, "Mag_X:");
            ips200_show_float(100, 1 * 16, imu660.data_Ripen.mag_x, 3, 3);
            ips200_show_string(20, 2 * 16, "Mag_Y:");
            ips200_show_float(100, 2 * 16, imu660.data_Ripen.mag_y, 3, 3);
            ips200_show_string(20, 3 * 16, "Mag_Z:");
            ips200_show_float(100, 3 * 16, imu660.data_Ripen.mag_z, 3, 3);
            break;

        case 6:
        {
            const EncoderLayerState* enc = encoder_layer_get_state();
            ips200_show_string(20, 0 * 16, "--- Encoder ---");
            ips200_show_string(20, 1 * 16, "Speed L:");
            ips200_show_float(100, 1 * 16, enc->speed_left_mps, 3, 2);
            ips200_show_string(20, 2 * 16, "Speed R:");
            ips200_show_float(100, 2 * 16, enc->speed_right_mps, 3, 2);
            ips200_show_string(20, 3 * 16, "Avg    :");
            ips200_show_float(100, 3 * 16, enc->speed_average_mps, 3, 2);
            ips200_show_string(20, 4 * 16, "Raw L :");
            ips200_show_int(100, 4 * 16, (int32)encoder_layer_get_raw_count_left(), 6);
            ips200_show_string(20, 5 * 16, "Raw R :");
            ips200_show_int(100, 5 * 16, (int32)encoder_layer_get_raw_count_right(), 6);
            ips200_show_string(20, 6 * 16, "TickLR :");
            ips200_show_int(100, 6 * 16, (int32)enc->tick_left, 6);
            ips200_show_int(150, 6 * 16, (int32)enc->tick_right, 6);
            break;
        }

        case 7:
            ips200_show_string(20, 0 * 16, "--- INS Entry ---");
            ips200_show_string(20, 2 * 16, "Use K4 Long Press");
            ips200_show_string(20, 3 * 16, "to enter INS UI");
            ips200_show_string(20, 5 * 16, "K1/K3 keep paging");
            break;

        case 8:
            ips200_show_string(20, 0 * 16, "--- INS State ---");
            ips200_show_string(20, 1 * 16, "X   :");
            ips200_show_float(80, 1 * 16, ins_state->x, 3, 3);
            ips200_show_string(160, 1 * 16, "m");
            ips200_show_string(20, 2 * 16, "Y   :");
            ips200_show_float(80, 2 * 16, ins_state->y, 3, 3);
            ips200_show_string(160, 2 * 16, "m");
            ips200_show_string(20, 3 * 16, "Yaw :");
            ips200_show_float(80, 3 * 16, ins_state->yaw * 57.29578f, 3, 2);
            ips200_show_string(160, 3 * 16, "deg");
            ips200_show_string(20, 5 * 16, "EKF: 3-State");
            ips200_show_string(20, 6 * 16, "v3.0 Active");
            break;

<<<<<<< HEAD
        case 9:
            steering_test_page();
            break;

=======
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
        default:
            page = 0;
            break;
    }
}
