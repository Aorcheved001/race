///*
// * menu.c
// *
// *  Created on: 2026-03-15
// *      Author: Daydreamer
// */
//
////-------------------------------------------头文件声明区------------------------------------------------------------
//#include "zf_common_headfile.h"
//
////-------------------------------------------宏定义区------------------------------------------------------------
//#define MAX_PAGE 8               // 菜单页范围 0-8
//static int8 page = 1;            // 当前菜单页
//
////-------------------------------------------结构体定义区------------------------------------------------------------
//INS_Demo_t INS = { .Mode = -1 }; // INS 演示状态，-1 表示仅菜单模式
//
//////-------------------------------------------------------------------------------------------------------------------
// ////  @brief      按键检测函数
// ////  @param      key_n - 按键编号 (KEY_1 ~ KEY_4)
// ////  @param      state - 按键状态 (短按/长按)
// ////-------------------------------------------------------------------------------------------------------------------
//int key_detect(key_index_enum key_n, key_state_enum state)
//{
//    static uint8 long_press_flag[KEY_NUMBER] = {0};
//
//    key_state_enum current_state = key_get_state(key_n);
//
//    if(state == KEY_LONG_PRESS)
//    {
//        if(current_state == KEY_LONG_PRESS)
//        {
//            if(long_press_flag[key_n] == 0)
//            {
//                long_press_flag[key_n] = 1;
//                key_clear_state(key_n);
//                return 1;
//            }
//        }
//        else if(current_state == KEY_RELEASE)
//        {
//            long_press_flag[key_n] = 0;
//        }
//    }
//    else if(state == KEY_SHORT_PRESS)
//    {
//        if(current_state == KEY_SHORT_PRESS)
//        {
//            key_clear_state(key_n);
//            return 1;
//        }
//    }
//
//    return 0;
//}
//
//////-------------------------------------------------------------------------------------------------------------------
// ////  @brief      菜单任务
// ////  @param      无
// ////-------------------------------------------------------------------------------------------------------------------
//void menu(void)
//{
//    if(g_key_scan_flag)
//    {
//        g_key_scan_flag = 0;
//        key_scanner();
//    }
//
//    // INS 子状态机运行时，强制停留在 case7，屏蔽菜单翻页干扰
//    if ((INS.Mode != -1) && (page != 7))
//    {
//        page = 7;
//        ips200_clear();
//    }
//
//    // 菜单翻页只在纯菜单模式下有效
//    if(INS.Mode == -1)
//    {
//        if(key_detect(KEY_3, KEY_SHORT_PRESS))
//        {
//            ips200_clear();
//            if(++page > MAX_PAGE) page = 0;
//        }
//        if(key_detect(KEY_1, KEY_SHORT_PRESS))
//        {
//            ips200_clear();
//            if(--page < 0) page = MAX_PAGE;
//        }
//    }
//
//    switch(page)
//    {
//        case 0:
//            ips200_show_string(20, 0*16, "--- System Home ---");
//            ips200_show_string(20, 1*16, "Core: TC377 Aurix");
//            ips200_show_string(20, 2*16, "Menu:v1.0");
//            ips200_show_string(20, 4*16, "Press K1/K3 to Flip");
//            break;
//
//        case 1:
//            ips200_show_string(20, 0*16, "--- GPS Status ---");
//            ips200_show_string(20, 1*16, "Status:");
//            const gnss_state_t* gps_ptr = gnss_get_state();
//            if(gps_ptr->fix_type > 0)
//                ips200_show_string(100, 1*16, "FIXED    ");
//            else
//                ips200_show_string(100, 1*16, "SEARCHING");
//
//            ips200_show_string(20, 2*16, "Sats  :");
//            ips200_show_uint(100, 2*16, gps_ptr->num_sats, 2);
//
//            ips200_show_string(20, 3*16, "Height:");
//            ips200_show_float(100, 3*16, gps_ptr->alt_m, 3, 2);
//            ips200_show_string(160, 3*16, "m");
//
//            ips200_show_string(20, 5*16, "Date  :");
//            ips200_show_string(100, 5*16, "----/--/--");
//            ips200_show_uint(100, 5*16, gnss.time.year, 4);
//            ips200_show_string(140, 5*16, "/");
//            ips200_show_uint(150, 5*16, gnss.time.month, 2);
//            ips200_show_string(170, 5*16, "/");
//            ips200_show_uint(180, 5*16, gnss.time.day, 2);
//
//            ips200_show_string(20, 6*16, "Time  :");
//            ips200_show_string(100, 6*16, "--:--:--");
//            ips200_show_uint(100, 6*16, gnss.time.hour, 2);
//            ips200_show_string(120, 6*16, ":");
//            ips200_show_uint(130, 6*16, gnss.time.minute, 2);
//            ips200_show_string(150, 6*16, ":");
//            ips200_show_uint(160, 6*16, gnss.time.second, 2);
//            break;
//
//        case 2:
//            ips200_show_string(20, 0*16, "--- GPS Position ---");
//            ips200_show_string(20, 1*16, "Lat :");
//            ips200_show_float(80, 1*16, (float)gnss_get_state()->lat_deg, 3, 6);
//            ips200_show_string(20, 2*16, "Lon :");
//            ips200_show_float(80, 2*16, (float)gnss_get_state()->lon_deg, 3, 6);
//
//            ips200_show_string(20, 4*16, "--- GPS Motion ---");
//            ips200_show_string(20, 5*16, "Speed:");
//            ips200_show_float(100, 5*16, gnss.speed, 3, 2);
//            ips200_show_string(160, 5*16, "km/h");
//            ips200_show_string(20, 6*16, "Dir  :");
//            ips200_show_float(100, 6*16, gnss.direction, 3, 1);
//            ips200_show_string(160, 6*16, "deg");
//            break;
//
//        case 3:
//            ips200_show_string(20, 0*16, "--- IMU Accel ---");
//            ips200_show_string(20, 1*16, "Acc_X:");
//            ips200_show_float(100, 1*16, imu660.data_Ripen.acc_x, 3, 3);
//            ips200_show_string(20, 2*16, "Acc_Y:");
//            ips200_show_float(100, 2*16, imu660.data_Ripen.acc_y, 3, 3);
//            ips200_show_string(20, 3*16, "Acc_Z:");
//            ips200_show_float(100, 3*16, imu660.data_Ripen.acc_z, 3, 3);
//            ips200_show_string(20, 5*16, "Unit : m/s2");
//            break;
//
//        case 4:
//            ips200_show_string(20, 0*16, "--- IMU Gyro ---");
//            ips200_show_string(20, 1*16, "Gyr_X:");
//            ips200_show_float(100, 1*16, imu660.data_Ripen.gyro_x, 3, 3);
//            ips200_show_string(20, 2*16, "Gyr_Y:");
//            ips200_show_float(100, 2*16, imu660.data_Ripen.gyro_y, 3, 3);
//            ips200_show_string(20, 3*16, "Gyr_Z:");
//            ips200_show_float(100, 3*16, imu660.data_Ripen.gyro_z, 3, 3);
//            ips200_show_string(20, 5*16, "Unit : rad/s");
//            break;
//
//        case 5:
//            ips200_show_string(20, 0*16, "--- IMU Magnet ---");
//            ips200_show_string(20, 1*16, "Mag_X:");
//            break;
//
//        case 6:
//        {
//            const EncoderLayerState* enc = encoder_layer_get_state();
//            ips200_show_string(20, 0*16, "--- Encoder ---");
//            ips200_show_string(20, 1*16, "Speed L:");
//            ips200_show_float(100, 1*16, enc->speed_left_mps, 3, 2);
//            ips200_show_string(20, 2*16, "Speed R:");
//            ips200_show_float(100, 2*16, enc->speed_right_mps, 3, 2);
//            ips200_show_string(20, 3*16, "Avg    :");
//            ips200_show_float(100, 3*16, enc->speed_average_mps, 3, 2);
//            ips200_show_string(20, 4*16, "Raw L :");
//            ips200_show_int(100, 4*16, (int32)encoder_layer_get_raw_count_left(), 6);
//            ips200_show_string(20, 5*16, "Raw R :");
//            ips200_show_int(100, 5*16, (int32)encoder_layer_get_raw_count_right(), 6);
//            ips200_show_string(20, 6*16, "TickLR :");
//            ips200_show_int(100, 6*16, (int32)enc->tick_left, 6);
//            ips200_show_int(150, 6*16, (int32)enc->tick_right, 6);
//            break;
//        }
//
//        case 7:
//        {
//            // ---- 第0行：标题 + GPS 状态（每帧刷新） ----
//            ips200_show_string(0,   0*16, "INS Demo Mode  ");
//            if(gnss_get_state()->fix_type > 0)
//                ips200_show_string(150, 0*16, "GPS:OK");
//            else
//                ips200_show_string(150, 0*16, "GPS:NO");
//
//            // ---- KEY4 长按：切换菜单 <-> INS 状态机 ----
//            if(key_detect(KEY_4, KEY_LONG_PRESS))
//            {
//                if (INS.Mode == -1)
//                {
//                    // 进入 INS 子状态机
//                    INS.Mode = 0;
//                    encoder_layer_clear_odom();           // 清零里程，INS 从原点出发
//                    wheel_pid_set_target_speed(0.0f);     // 先清速度目标，再使能
//                    wheel_pid_enable(1);                  // INS/PID 接管
//                }
//                else
//                {
//                    // 退出 INS 子状态机
//                    INS.Mode = -1;
//                    wheel_pid_set_target_speed(0.0f);
//                    wheel_pid_enable(0);                  // 释放 INS/PID 输出
//                    motor_set_drive_lr(0, 0);             // 立即停车，防止惯性滑行
//                }
//
//                // 清理按键状态，避免双状态机串键
//                key_clear_state(KEY_1);
//                key_clear_state(KEY_2);
//                key_clear_state(KEY_3);
//                key_clear_state(KEY_4);
//                ips200_clear();
//                break;  // 本帧跳过后续显示，等下一帧刷新避免残影
//            }
//
//            if (INS.Mode != -1)
//            {
//                // ---- INS 运行态 ----
//                ips200_show_string(0,   1*16, "[RUN] INS Active  ");
//
//                // 第2行：X / Y
//                ips200_show_string(0,   2*16, "Cur X:");
//                ips200_show_float(20,   2*16, g_ins_state.x, 4, 2);
//                ips200_show_string(110, 2*16, "Cur y:");
//                ips200_show_float(130,  2*16, g_ins_state.y, 4, 2);
//
//                // 第3行：Yaw
//                ips200_show_string(0,   3*16, "Yaw:");
//                ips200_show_float(40,   3*16, g_ins_state.yaw * 57.29578f, 4, 2);
//                ips200_show_string(130, 3*16, "deg");
//
//                // 第4行：当前速度
//                ips200_show_string(0,   4*16, "Spd:");
//                ips200_show_float(40,   4*16, g_ins_state.v, 2, 3);
//                ips200_show_string(110, 4*16, "m/s");
//
//                // 第5行：已记录点数
//                ips200_show_string(0,   5*16, "Pts:");
//                ips200_show_int(40,     5*16, Ins_date_377.point, 4);
//
//                // 第6行：操作提示
//                ips200_show_string(0,   6*16, "K4 Long = Exit    ");
//            }
//            else
//            {
//                // ---- 待机态 ----
//                ips200_show_string(0,   1*16, "Hold K4 to Enter  ");
//
//                // 第2行：覆盖 INS 运行态的 X/Y 残影
//                ips200_show_string(0,   2*16, "                  ");
//
//                // 第3行：实时 Yaw，方便对齐方向后再启动
//                ips200_show_string(0,   3*16, "Yaw:");
//                ips200_show_float(40,   3*16, g_ins_state.yaw * 57.29578f, 4, 2);
//                ips200_show_string(130, 3*16, "deg");
//
//                // 第4-6行：空行，覆盖运行态残影
//                ips200_show_string(0,   4*16, "                  ");
//                ips200_show_string(0,   5*16, "                  ");
//                ips200_show_string(0,   6*16, "                  ");
//            }
//            break;
//        }
//
//        case 8:
//            ips200_show_string(20, 0*16, "--- INS Params ---");
//            ips200_show_string(20, 1*16, "kv:");
//            ips200_show_float(60, 1*16, g_ins_state.kv, 3, 4);
//            ips200_show_string(20, 2*16, "ks:");
//            ips200_show_float(60, 2*16, g_ins_state.ks, 3, 4);
//            ips200_show_string(20, 3*16, "bs:");
//            ips200_show_float(60, 3*16, g_ins_state.bs, 3, 4);
//            ips200_show_string(20, 4*16, "bl:");
//            ips200_show_float(60, 4*16, g_ins_state.bl, 3, 4);
//            ips200_show_string(20, 5*16, "X :");
//            ips200_show_float(60, 5*16, g_ins_state.x, 3, 3);
//            ips200_show_string(20, 6*16, "Y :");
//            ips200_show_float(60, 6*16, g_ins_state.y, 3, 3);
//            break;
//
//        default:
//            page = 0;
//            break;
//    }
//}
