/*
 * steering_test.c
 * 前轮编码器纯测试页面 - 在 IPS200 屏幕上显示编码器数据
 *
 * 仅测试编码器，不控制电机
 *
 * 按键操作：
 *   K2 短按 → 将当前前轮位置设为零位（摆正后按）
 */

#include "zf_common_headfile.h"
#include "steering_control.h"

extern volatile uint8 g_key_scan_flag;
extern void key_scanner(void);
extern int key_detect(key_index_enum key_n, key_state_enum state);

//-------------------------------------------------------------------------------------------------------------------
// 编码器测试页面 - 在 40ms 任务中调用
//-------------------------------------------------------------------------------------------------------------------
void steering_test_page(void)
{
    if(g_key_scan_flag)
    {
        g_key_scan_flag = 0;
        key_scanner();
    }

    /* 按键操作：K2 标定零位 */
    if(key_detect(KEY_2, KEY_SHORT_PRESS))
    {
        steering_set_zero_at_current();
    }

    /* 读取编码器原始值 */
    int16 raw = absolute_encoder_get_location();

    /* 计算当前角度 */
    float cur_angle = steering_get_current_angle();

    /* 显示 */
    ips200_clear();

    ips200_show_string(10, 0 * 16, "=== Encoder Test ===");

    /* 编码器原始值 0~4095 */
    ips200_show_string(10, 1 * 16, "Raw:");
    ips200_show_int(80, 1 * 16, raw, 5);

    /* 当前角度（度） */
    ips200_show_string(10, 2 * 16, "Angle:");
    ips200_show_float(90, 2 * 16, cur_angle, 3, 1);
    ips200_show_string(150, 2 * 16, "deg");

    /* 操作提示 */
    ips200_show_string(10, 5 * 16, "K2: Set Zero");
}
