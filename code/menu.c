/*
 * menu.c
 *
 * 自定义两级菜单：
 * 顶层：task1 / task2 / task3 / task4 / driver
 * 每个元素有子菜单，确认键修改标志位。
 *
 * 按键映射（来自 zf_device_key.h 中 KEY_LIST）：
 * KEY_1 -> P20_6   上
 * KEY_2 -> P20_7   下
 * KEY_3 -> P11_2   确认（短按修改标志位，长按返回上一级）
 */

#include "menu.h"
#include "task.h"

/*--------------------------------宏定义-------------------------------*/
#define  MAX_PAGE               5      //总页数
#define  FIRST_CURSOR           5
#define  TASK1_CURSOR           5      //task1界面的最大行数

#define  TASK2_CURSOR           5      //task2界面的最大行数
#define  TASK2_GPS_CURSOR       4      //task2子菜单GPS
#define  TASK2_SPEECH_CURSOR    3      //task2子菜单speech

#define  TASK3_CURSOR           5      //task3界面的最大行数
#define  TASK4_CURSOR           5      //task4界面的最大行数
#define  DRIVE_CURSOR           5      //驾驶界面的最大行数


#define OFF 0
#define ON  1
#define Font_gap   16
/*-----------------------------参数设置--------------------------------*/
uint8 t1cursor = 1;

char   t2cursor = 1;
char   t2cursor_gps = 1;
char   t2cursor_speech = 1;

int8 t3cursor = 1;
int8 t4cursor = 1;
uint8 dr_cursor = 1;
uint8 menu_first_cursor = 1;

uint8 first_key = 0;      //切换界面标志位

task_state project_task;
 // 按键检测逻辑：支持短按、长按一次触发
 int key_detect(key_index_enum key_n, key_state_enum state)
 {
     static uint8 long_press_flag[KEY_NUMBER] = {0};

     key_state_enum current_state = key_get_state(key_n);

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

/*============================== Task 菜单数据结构 ==============================*/

typedef struct
{
    uint8 started;   // 是否已开始
    uint8 job;       // 具体任务选项编号
    int16 param;     // 参数（示例用，可根据需要替换为 float 等）
} task_config_t;

// 4 个 task，对应 4 页
static task_config_t g_tasks[4] =
{
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
};

static const char *const g_task_titles[4] =
{
    "task1",
    "task2",
    "task3",
    "task4",
};

// 当前正在运行的 task 索引；-1 表示没有 task 运行
static int8 g_active_task = 0;

/*============================== 绘制函数 ==============================*/

/*============================== 菜单主逻辑（放在主循环中调用） ==============================*/

void firstdisplay(void)
{
    ips200_show_string(100, 0, "MENU");

    for(int i = 1; i <= FIRST_CURSOR;i++)
    {
    ips200_show_string(0, Font_gap*i, (menu_first_cursor == i) ? ">>>" : "   ");
    }
    ips200_show_string(25, Font_gap, "task1");
    ips200_show_string(25, Font_gap*2, "task2");
    ips200_show_string(25, Font_gap*3, "task3");
    ips200_show_string(25, Font_gap*4, "task4");
    ips200_show_string(25, Font_gap*5, "drive");
}

void task1display(void)
{

}

void task2display(void)
{
    ips200_show_string(100, 0, "TASK2");
    for(int i = 0; i <= TASK2_CURSOR;i++)
    {
    ips200_show_string(0, Font_gap*i, (t2cursor == i) ? ">>>" : "   ");
    }

    if(project_task.task2_state[start_num_state] == ON)
    {
        ips200_show_string(200, Font_gap, "ON ");
    }
    if(project_task.task2_state[start_num_state] == OFF)
    {
        ips200_show_string(200, Font_gap, "OFF");
    }
    ips200_show_string(25, 16, "Start:");
    /**********************************************************/

    if(project_task.task2_state[gps_num_state] == ON)
    {
        ips200_show_string(200, Font_gap*2, "ON ");
    }
    if(project_task.task2_state[gps_num_state] == OFF)
    {
        ips200_show_string(200, Font_gap*2, "OFF");
    }
    ips200_show_string(25, Font_gap*2, "SAVE DOT");
    /**********************************************************/

    if(project_task.task2_state[speech_ok] == ON)
    {
        ips200_show_string(200, Font_gap*3, "OK ");
    }
    if(project_task.task2_state[speech_ok] == OFF)
    {
        ips200_show_string(200, Font_gap*3, "OFF");
    }
    ips200_show_string(25, Font_gap*3, "Speech Recognition");
    /**********************************************************/
    if(project_task.task2_state[go_num_state] == ON)
    {
        ips200_show_string(200, Font_gap*4, "ON ");
    }
    if(project_task.task2_state[go_num_state] == OFF)
    {
        ips200_show_string(200, Font_gap*4, "OFF");
    }
    ips200_show_string(25, Font_gap*4, "GO");
//    if(speech_fifo_get_data[0] == 0xFF)
//    //ips200_clear();
//    ips200_show_int(25,16*5,t2cursor,1);
//    ips200_show_int(25,16*6,g_active_task,1);
//    ips200_show_int(25,16*7,project_task.task2_state[start_num_state],1);
}
//按键一存点，存在flash里
//
void task2_GPS_display(void)
{
    ips200_show_string(90, 0, "TASK2 GPS");

    for(int i = 0; i <= TASK2_GPS_CURSOR;i++)
    {
    ips200_show_string(0, Font_gap*i, (t2cursor_gps == i) ? ">>>" : "   ");
    }
    ips200_show_string(25, Font_gap*1, "GPS point");
    if(project_task.task2_state[gps_num_state] == ON)
    {
        ips200_show_string(200, Font_gap*1, "ON ");
    }
    if(project_task.task2_state[gps_num_state] == OFF)
    {
        ips200_show_string(200, Font_gap*1, "OFF");
    }
    ips200_show_string(50, Font_gap*2, "SAVE");
    ips200_show_string(100, Font_gap*2, "FINISH");
    ips200_show_string(25, Font_gap*7, "point number:");

}

void show_yuyin(uint8 shuju , uint8 hang)
{
    switch(shuju)
    {
        //ips200_clear();
        case CMD_WAKEUP:
                                ips200_show_string(25, Font_gap*hang, "WAKEUP          ");
            // 语音回复 "我在呢"
            break;
        case CMD_LEFT_TURN:     ips200_show_string(25, Font_gap*hang, "LEFT_TURN       ");
            // 打开左转向灯
            break;
        case CMD_RIGHT_TURN:    ips200_show_string(25, Font_gap*hang, "RIGHT_TURN      ");
            // 打开右转向灯
            break;
        case CMD_HIGH_BEAM:     ips200_show_string(25, Font_gap*hang, "HIGH_BEAM       ");
            // 打开远光灯
            break;
        case CMD_LOW_BEAM:      ips200_show_string(25, Font_gap*hang, "LOW_BEAM        ");
            // 打开近光灯
            break;
        case CMD_FOG_LAMP:      ips200_show_string(25, Font_gap*hang, "FOG_LAMP        ");
            // 打开雾灯
            break;
        case CMD_HAZARD_LAMP:   ips200_show_string(25, Font_gap*hang, "HAZARD_LAMP     ");
            // 打开双闪灯
            break;
        case CMD_INTERIOR_LAMP: ips200_show_string(25, Font_gap*hang, "INTERIOR_LAMP   ");
            // 打开车内照明灯
            break;
        case CMD_WIPER:         ips200_show_string(25, Font_gap*hang, "WIPER           ");
            // 打开雨刮器
            break;
        case CMD_HORN_1S:       ips200_show_string(25, Font_gap*hang, "HORN_1S         ");
            // 鸣笛一秒钟
            break;
        case CMD_HORN_2S:       ips200_show_string(25, Font_gap*hang, "HORN_2S         ");
            // 鸣笛两秒钟
            break;
        case CMD_HORN_3S:       ips200_show_string(25, Font_gap*hang, "HORN_3S         ");
            // 鸣笛三秒钟
            break;
        case CMD_HORN_2BEEP:    ips200_show_string(25, Font_gap*hang, "HORN_2BEEP      ");
            // 鸣笛两声
            break;
        case CMD_HORN_3BEEP:    ips200_show_string(25, Font_gap*hang, "HORN_3BEEP      ");
            // 鸣笛三声
            break;
        case CMD_HORN_4BEEP:    ips200_show_string(25, Font_gap*hang, "HORN_4BEEP      ");
            // 鸣笛四声
            break;
       case CMD_HORN_LONG_SHORT:ips200_show_string(25, Font_gap*hang, "HORN_LONG_SHORT ");
            // 长短鸣笛
            break;
        case CMD_HORN_URGENT:   ips200_show_string(25, Font_gap*hang, "HORN_URGENT     ");
            // 急促鸣笛
            break;
        case CMD_HORN_ALARM:    ips200_show_string(25, Font_gap*hang, "HORN_ALARM      ");
            // 警报鸣笛
            break;
        case CMD_PASS_LEFT:     ips200_show_string(25, Font_gap*hang, "PASS_LEFT       ");
            // 通过门洞一左则
            break;
        case CMD_PASS_1:        ips200_show_string(25, Font_gap*hang, "PASS_1          ");
            // 通过门洞一
            break;
        case CMD_PASS_2:        ips200_show_string(25, Font_gap*hang, "PASS_2          ");
            // 通过门洞二
            break;
        case CMD_PASS_3:        ips200_show_string(25, Font_gap*hang, "PASS_3          ");
            // 通过门洞三
            break;
        case CMD_PASS_RIGHT:    ips200_show_string(25, Font_gap*hang, "PASS_RIGHT      ");
            // 通过门洞三右侧
            break;
        case CMD_RETURN_RIGHT:  ips200_show_string(25, Font_gap*hang, "RETURN_RIGHT    ");
            // 门洞一右侧返回
            break;
        case CMD_RETURN_1:      ips200_show_string(25, Font_gap*hang, "RETURN_1        ");
            // 门洞一返回
            break;
        case CMD_RETURN_2:      ips200_show_string(25, Font_gap*hang, "RETURN_2        ");
            // 门洞二返回
            break;
        case CMD_RETURN_3:      ips200_show_string(25, Font_gap*hang, "RETURN_3        ");
            // 门洞三返回
            break;
        case CMD_RETURN_LEFT:   ips200_show_string(25, Font_gap*hang, "RETURN_LEFT     ");
            // 门洞三左侧返回
            break;
        case CMD_FORWARD_10M:   ips200_show_string(25, Font_gap*hang, "FORWARD_10M     ");
            // 前行十米
            break;
        case CMD_BACKWARD_10M:  ips200_show_string(25, Font_gap*hang, "BACKWARD_10M    ");
            // 后退十米
            break;
        case CMD_SNAKE_FORWARD: ips200_show_string(25, Font_gap*hang, "SNAKE_FORWAR    ");
            // 蛇形前进十米
            break;
        case CMD_SNAKE_BACKWARD:ips200_show_string(25, Font_gap*hang, "SNAKE_BACKWARD  ");
            // 蛇形后退十米
            break;
      case CMD_COUNTERCLOCKWISE:ips200_show_string(25, Font_gap*hang, "COUNTERCLOCKWISE");
            // 逆时针转一圈
            break;
        case CMD_CLOCKWISE:     ips200_show_string(25, Font_gap*hang, "CLOCKWISE       ");
            // 顺时针转一圈
            break;
        case CMD_TURN_LEFT:     ips200_show_string(25, Font_gap*hang, "TURN_LEFT       ");
            // 左转
            break;
        case CMD_TURN_RIGHT:    ips200_show_string(25, Font_gap*hang, "TURN_RIGHT      ");
            // 右转
            break;
        case CMD_END:           ips200_show_string(25, Font_gap*hang, "END             ");
            // 结束
            break;
        case CMD_CLEAR:         ips200_show_string(25, Font_gap*hang, "CLEAR           ");
       // 清空
            break;
        default:
            // 未知命令，可不做处理
            break;
    }
}

void task2_speech_display(void)
{
    ips200_show_string(90, 0, "TASK2 SPEECH");

    for(int i = 0; i <= TASK2_SPEECH_CURSOR;i++)
    {
    ips200_show_string(0, Font_gap*i, (t2cursor_speech == i) ? ">>>" : "   ");
    }

    ips200_show_string(25, Font_gap*1, "start speech");
    ips200_show_string(25, Font_gap*2, "display");
    ips200_show_string(25, Font_gap*3, "make sure");

    if(project_task.task2_state[speech_num_state] == ON)
    {
        ips200_show_string(200, Font_gap*1,"ON ");
    }
    if(project_task.task2_state[speech_num_state] == OFF)
    {
        ips200_show_string(200, Font_gap*1, "OFF");
    }

    if(project_task.task2_state[speech_disp_num_state] == ON)
    {
        for(int i = 1; i < 9; i++ )
        {
            show_yuyin(uart1_speech_get_data[i],i+4);
            //show_yuyin(speech_fifo_get_data[i],i+4);
        }
    }

    if(project_task.task2_state[speech_num_state] == ON)
    {
        show_yuyin(get_data,19);

    }


    ips200_show_int (100, Font_gap*2, speech_index, 1);
    ips200_show_int (150, Font_gap*2, speaker_state, 1);

}


void task2_judge_state(void)
{

}
void menu_first(void)
{
    // 读取按键事件：上 (P20_6)、下 (P20_7)、确认 (P11_2)
    uint8 up_short      = key_detect(KEY_2, KEY_SHORT_PRESS);
    uint8 down_short    = key_detect(KEY_1, KEY_SHORT_PRESS);
    uint8 up_long       = key_detect(KEY_2, KEY_LONG_PRESS);
    uint8 down_long     = key_detect(KEY_1, KEY_LONG_PRESS);
    uint8 confirm_short = key_detect(KEY_4, KEY_SHORT_PRESS);


    switch(first_key)
    {
        case 0:
            if(up_short)
            {
                if(--menu_first_cursor == 0)
                {
                    menu_first_cursor = FIRST_CURSOR;
                }
            }
            if(down_short)
            {
                menu_first_cursor++;
                if(menu_first_cursor > FIRST_CURSOR)
                {
                    menu_first_cursor = 1;
                }
            }

            firstdisplay();

            if(confirm_short)
            {
                switch(menu_first_cursor)
                {
                    case 1: first_key = 1; break;
                    case 2: first_key = 2; break;
                    case 3: first_key = 3; break;
                    case 4: first_key = 4; break;
                    case 5: first_key = 5; break;
                }
                ips200_clear();
            }

            break;
        case 1:
            break;
//*************************************科目二菜单*******************************************//
        case 2:
            if(up_short)
            {
                if(--t2cursor < 0)
                {
                    t2cursor = TASK2_CURSOR;
                }
            }
            if(down_short)
            {
                t2cursor++;
                if(t2cursor > TASK2_CURSOR)
                {
                    t2cursor = 1;
                }
            }

            task2display();

            if(confirm_short)
            {
                switch(t2cursor)
                {
                    //标题处确认为返回主菜单
                    case 0:
                        if(g_active_task == 0)//当确认科目后不能返回主菜单
                            first_key = 0;
                        break;
                    //start处确认，是确认所选科目，确认之后才能进行存点，语音等操作
                    case 1:
                        if(project_task.task2_state[start_num_state] == ON)
                        {
                            project_task.task2_state[start_num_state] = OFF;
                            project_task.task2_state[gps_num_state] = OFF;
                            project_task.task2_state[speech_num_state] = OFF;
                            project_task.task2_state[go_num_state] = OFF;
                            g_active_task = 0;
                        }
                        else
                        {
                            project_task.task2_state[start_num_state] = ON;
                            g_active_task = 1;
                        }
                        break;
                    //GPS存点
                    case 2:
                        if(project_task.task2_state[start_num_state] == ON)
                            first_key = 21;
                        break;
                    case 3:
                        if(project_task.task2_state[start_num_state] == ON)
                        {
                            first_key = 22;
                            project_task.task2_state[speech_ok] = OFF;
                        }
                        break;
                    case 4: first_key = 4; break;
                    case 5: first_key = 5; break;
                }
                ips200_clear();
            }
            break;

        //********************task2的子菜单，GPS存点*********************//
        case 21:
            if(up_short)
            {
                if(--t2cursor_gps < 0)
                {
                    t2cursor_gps = TASK2_GPS_CURSOR;
                }
            }
            if(down_short)
            {
                t2cursor_gps++;
                if(t2cursor_gps > TASK2_GPS_CURSOR)
                {
                    t2cursor_gps = 1;
                }
            }
            task2_GPS_display();

            if(confirm_short)
            {
                switch(t2cursor_gps)
                {
                    case 0:
                        first_key = 2;
                        break;
                    case 1:
                        project_task.task2_state[gps_num_state] = OFF;
                        break;
                }
                ips200_clear();
            }


            break;
            //********************task2的子菜单，语音识别*********************//
        case 22:
            if(up_short)
            {
                if(--t2cursor_speech < 0)
                {
                    t2cursor_speech = TASK2_SPEECH_CURSOR;
                }
            }
            if(down_short)
            {
                t2cursor_speech++;
                if(t2cursor_speech > TASK2_SPEECH_CURSOR)
                {
                    t2cursor_speech = 1;
                }
            }

            task2_speech_display();
            /*      0：返回
                    1：打开或者关闭串口接收中断
                    2：显示
                    3：确认
            */
            if(confirm_short)
            {
                switch(t2cursor_speech)
                {
                    case 0:
                        if(project_task.task2_state[speech_num_state] == OFF)
                        first_key = 2; //退回菜单2
                        break;
                    case 1:
                        if(project_task.task2_state[speech_num_state] ==  ON)
                        project_task.task2_state[speech_num_state] = OFF;
                        else project_task.task2_state[speech_num_state] =  ON;
                        break;
                    case 2:
                        if(project_task.task2_state[speech_num_state] == OFF)
                        {
                            if(project_task.task2_state[speech_disp_num_state] == ON)
                            {
                                project_task.task2_state[speech_disp_num_state] = OFF;
                            }

                            if(project_task.task2_state[speech_disp_num_state] == OFF)
                            {
                                project_task.task2_state[speech_disp_num_state] = ON;
                            }
                        }
                        break;
                    case 3:
                        if(project_task.task2_state[speech_num_state] == OFF)
                        {
                            if(project_task.task2_state[speech_ok] == ON)
                            {
                                project_task.task2_state[speech_ok] = OFF;
                            }

                            if(project_task.task2_state[speech_ok] == OFF)
                            {
                                project_task.task2_state[speech_ok] = ON;
                                first_key = 2;                              //退回菜单2
                            }


                        }
                        break;

                }
                ips200_clear();
            }
            break;
    }
}

