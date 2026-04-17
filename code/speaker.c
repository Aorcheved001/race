/*
 * speaker.c
 *
 *  Created on: 06-三月-26
 *      Author: LENOVO
 */
#include "speaker.h"


// 音乐播放控制变量
static const MusicNote *current_melody = NULL;
static uint16 melody_len = 0;
static uint16 note_index = 0;
static uint32 next_note_time = 0;
static uint8 is_playing = 0;

// 鸣笛控制变量
static Horn_State g_horn_state = {HORN_STATE_IDLE, HORN_1S, 0, 0, 0};


//-------------------------------------------------------------------------------------------------------------------
//  @brief      喇叭初始化    在Init.c中调用即可
//  @param      Speaker_Mode      即Speaker_D_Mode以及Speaker_AB_Mode
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Init(Speaker_Mode_Enum Speaker_Mode){

    // 初始化SD引脚为输出模式，默认输出低电平（打开）
    gpio_init(SD, GPO, 0, GPO_PUSH_PULL);

    // 初始化MODE引脚为输出模式
    gpio_init(MODE, GPO, 0, GPO_PUSH_PULL);

    gpio_set_level(SD,0);       //芯片段使能   低电平打开状态  高电平关闭状态

    if(Speaker_Mode==Speaker_D_Mode){
        gpio_set_level(MODE,1); //芯片模式切换，高电平状态下，芯片处于D类模式
    }
    else{
        gpio_set_level(MODE,0); //低电平状态下芯片处于AB类模式
    }

    // 初始化PWM引脚，频率5000Hz，初始占空比0（静音）
    pwm_init(ATOM0_CH7_P02_7, 5000, 0);

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      喇叭播放默认声音
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Play(){
    pwm_set_duty(ATOM0_CH7_P02_7, 5);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      喇叭停止播放
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Stop(){
    pwm_set_duty(ATOM0_CH7_P02_7, 0);
    is_playing = 0; // 停止当前旋律
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      喇叭播放特定频率音调
//  @param      frequency 频率
//  @param      duration_ms 持续时间(0为一直播放)
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_PlayTone(uint16 frequency, uint16 duration_ms)
{
    if(frequency > 0)
    {
        // 重新初始化PWM以改变频率
        // 注意：pwm_init会重新配置GTM通道
//        pwm_init(ATOM0_CH7_P02_7, frequency, 5000); // 50% 占空比
        pwm_init(ATOM0_CH7_P02_7, frequency, 5); // 2% 占空比
    }
    else
    {
        Speaker_Stop();
    }

    if(duration_ms > 0)
    {
        system_delay_ms(duration_ms);
        Speaker_Stop();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      简单的蜂鸣提示（阻塞）
//  @param      ms 持续时间
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Beep(uint16 ms)
{
    Speaker_PlayTone(4000, ms);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开始播放旋律（非阻塞）
//  @param      melody 音符数组
//  @param      length 数组长度
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_PlayMelody(const MusicNote *melody, uint16 length)
{
    current_melody = melody;
    melody_len = length;
    note_index = 0;
    is_playing = 1;
    next_note_time = 0; // 立即开始
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      鸣笛1秒钟
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_1s(void)
{
    Speaker_PlayTone(1000, 1000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      鸣笛两秒钟
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_2s(void)
{
    Speaker_PlayTone(1000, 2000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      鸣笛三秒钟
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_3s(void)
{
    Speaker_PlayTone(1000, 3000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      鸣笛两声（时长1秒，间隔1秒）
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_2beep(void)
{
    Speaker_PlayTone(1000, 1000);
    system_delay_ms(1000);
    Speaker_PlayTone(1000, 1000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      鸣笛三声（时长1秒，间隔1秒）
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_3beep(void)
{
    Speaker_PlayTone(1000, 1000);
    system_delay_ms(1000);
    Speaker_PlayTone(1000, 1000);
    system_delay_ms(1000);
    Speaker_PlayTone(1000, 1000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      鸣笛四声（时长1秒，间隔1秒）
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_4beep(void)
{
    Speaker_PlayTone(1000, 1000);
    system_delay_ms(1000);
    Speaker_PlayTone(1000, 1000);
    system_delay_ms(1000);
    Speaker_PlayTone(1000, 1000);
    system_delay_ms(1000);
    Speaker_PlayTone(1000, 1000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      长短鸣笛（鸣笛1秒，间隔1秒，鸣笛3秒）
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_LongShort(void)
{
    Speaker_PlayTone(1000, 1000);
    system_delay_ms(1000);
    Speaker_PlayTone(1000, 3000);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      急促鸣笛（鸣笛0.5秒，间隔0.5秒，6次）
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_Urgent(void)
{
    for(uint8 i = 0; i < 6; i++)
    {
        Speaker_PlayTone(1000, 500);
        system_delay_ms(500);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      警报鸣笛（500Hz和1000Hz交替，每次1秒，6次）
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_Alarm(void)
{
    for(uint8 i = 0; i < 6; i++)
    {
        Speaker_PlayTone(500, 1000);
        Speaker_PlayTone(1000, 1000);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开始非阻塞式鸣笛
//  @param      mode 鸣笛模式
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_Start(Horn_Mode_Enum mode)
{
    g_horn_state.state = HORN_STATE_PLAYING;
    g_horn_state.mode = mode;
    g_horn_state.next_time = system_getval_ms();
    g_horn_state.step = 0;
    g_horn_state.repeat = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      停止鸣笛
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Horn_Stop(void)
{
    g_horn_state.state = HORN_STATE_IDLE;
    Speaker_Stop();
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      检查鸣笛是否正在进行
//  @param
//  @return     1: 正在鸣笛，0: 未鸣笛
//-------------------------------------------------------------------------------------------------------------------
uint8 Speaker_Horn_IsPlaying(void)
{
    return (g_horn_state.state != HORN_STATE_IDLE);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      鸣笛状态机处理
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
static void Speaker_Horn_Task(void)
{
    if(g_horn_state.state == HORN_STATE_IDLE)
    {
        return;
    }

    uint32 current_time = system_getval_ms();

    if(current_time < g_horn_state.next_time)
    {
        return;
    }

    switch(g_horn_state.mode)
    {
        case HORN_1S:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.state = HORN_STATE_IDLE;
                    break;
            }
            break;

        case HORN_2S:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 2000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.state = HORN_STATE_IDLE;
                    break;
            }
            break;

        case HORN_3S:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 3000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.state = HORN_STATE_IDLE;
                    break;
            }
            break;

        case HORN_2BEEP:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 2;
                    break;
                case 2:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 3;
                    break;
                case 3:
                    Speaker_Stop();
                    g_horn_state.state = HORN_STATE_IDLE;
                    break;
            }
            break;

        case HORN_3BEEP:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 2;
                    break;
                case 2:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 3;
                    break;
                case 3:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 4;
                    break;
                case 4:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 5;
                    break;
                case 5:
                    Speaker_Stop();
                    g_horn_state.state = HORN_STATE_IDLE;
                    break;
            }
            break;

        case HORN_4BEEP:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 2;
                    break;
                case 2:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 3;
                    break;
                case 3:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 4;
                    break;
                case 4:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 5;
                    break;
                case 5:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 6;
                    break;
                case 6:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 7;
                    break;
                case 7:
                    Speaker_Stop();
                    g_horn_state.state = HORN_STATE_IDLE;
                    break;
            }
            break;

        case HORN_LONG_SHORT:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 2;
                    break;
                case 2:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 3000;
                    g_horn_state.step = 3;
                    break;
                case 3:
                    Speaker_Stop();
                    g_horn_state.state = HORN_STATE_IDLE;
                    break;
            }
            break;

        case HORN_URGENT:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 500;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_Stop();
                    g_horn_state.next_time = current_time + 500;
                    g_horn_state.step = 0;
                    g_horn_state.repeat++;
                    if(g_horn_state.repeat >= 6)
                    {
                        g_horn_state.state = HORN_STATE_IDLE;
                    }
                    break;
            }
            break;

        case HORN_ALARM:
            switch(g_horn_state.step)
            {
                case 0:
                    Speaker_PlayTone(500, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 1;
                    break;
                case 1:
                    Speaker_PlayTone(1000, 0);
                    g_horn_state.next_time = current_time + 1000;
                    g_horn_state.step = 0;
                    g_horn_state.repeat++;
                    if(g_horn_state.repeat >= 6)
                    {
                        g_horn_state.state = HORN_STATE_IDLE;
                    }
                    break;
            }
            break;

        default:
            g_horn_state.state = HORN_STATE_IDLE;
            break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      喇叭后台任务，需在主循环中调用
//  @param
//  @return     none
//-------------------------------------------------------------------------------------------------------------------
void Speaker_Task(void)
{
    // 处理音乐播放
    if(is_playing && current_melody != NULL)
    {
        uint32 current_time = system_getval_ms();

        if(current_time >= next_note_time)
        {
            if(note_index < melody_len)
            {
                // 获取当前音符
                MusicNote note = current_melody[note_index];

                // 播放音符
                if(note.frequency > 0)
                {
//                    pwm_init(ATOM0_CH7_P02_7, note.frequency, 5000);
                    pwm_init(ATOM0_CH7_P02_7, note.frequency, 5);
                }
                else
                {
                    pwm_set_duty(ATOM0_CH7_P02_7, 0); // 休止符
                }

                // 设置下一次切换时间
                next_note_time = current_time + note.duration;

                // 移动到下一个音符
                note_index++;
            }
            else
            {
                // 播放结束
                Speaker_Stop();
            }
        }
    }

    // 处理鸣笛任务
    Speaker_Horn_Task();
}
