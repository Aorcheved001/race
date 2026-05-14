/*
 * menu.c
 * 菜单显示系统
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------全局变量定义区-----------------------------------------------------------
static int8 page = 1;                                                     // 当前显示页面

#define MAX_PAGE 8                                                        // 最大页面数

 //-------------------------------------------函数声明区------------------------------------------------------------
////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      检测按键状态
 ////  @param      key_n       按键索引
 ////  @param      state       按键状态 (短按/长按)
 ////  @return     int8        1表示检测到对应状态，0表示未检测到
 ////  @note       处理按键的短按和长按检测
 ////-------------------------------------------------------------------------------------------------------------------
int key_detect(key_index_enum key_n, key_state_enum state)
{
    static uint8 long_press_flag[KEY_NUMBER] = {0};                       // 长按标志数组

    key_state_enum current_state = key_get_state(key_n);                  // 获取当前按键状态

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
 ////  @brief      菜单显示主函数
 ////  @param      void
 ////  @return     void
 ////  @note       处理按键扫描和页面显示
 ////-------------------------------------------------------------------------------------------------------------------
void menu(void)
{
    if(g_key_scan_flag)
    {
        g_key_scan_flag = 0;
        key_scanner();
    }

    const INS_State* ins_state = Ins_get_state();                         // 获取INS状态

    if(key_detect(KEY_3, KEY_SHORT_PRESS))
    {
        ips200_clear();
        if(++page > MAX_PAGE) page = 0;                                   // 下一页
    }
    if(key_detect(KEY_1, KEY_SHORT_PRESS))
    {
        ips200_clear();
        if(--page < 0) page = MAX_PAGE;                                   // 上一页
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

        default:
            page = 0;
            break;
    }
}
