/*
 * menu.h
 *
 *  简洁版菜单框架
 *  适用于 TC377 惯性导航项目
 */

#ifndef CODE_SPEAKER_H_
#define CODE_SPEAKER_H_

#include "zf_common_headfile.h"

#define SD P02_5
#define MODE P02_8
#define IN P02_7




// 音符频率定义 (Hz)
// 音符频率定义 (Hz)
#define NOTE_NONE 0
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

// 音乐结构体
typedef struct {
    uint16 frequency; // 音符频率 (Hz)
    uint16 duration;  // 持续时间 (ms)
} MusicNote;

typedef enum
{
    Speaker_D_Mode,                                                 //功放芯片D模式
    Speaker_AB_Mode,                                                  //功放芯片AB模式

}Speaker_Mode_Enum;


void Speaker_Init(Speaker_Mode_Enum Speaker_Mode);

void Speaker_Play(void);

typedef enum
{
    HORN_1S         = CMD_HORN_1S,
    HORN_2S         = CMD_HORN_2S,
    HORN_3S         = CMD_HORN_3S,
    HORN_2BEEP      = CMD_HORN_2BEEP,
    HORN_3BEEP      = CMD_HORN_3BEEP,
    HORN_4BEEP      = CMD_HORN_4BEEP,
    HORN_LONG_SHORT = CMD_HORN_LONG_SHORT,
    HORN_URGENT     = CMD_HORN_URGENT,
    HORN_ALARM      = CMD_HORN_ALARM
} Horn_Mode_Enum;

typedef enum
{
    HORN_STATE_IDLE = 0,
    HORN_STATE_PLAYING,
    HORN_STATE_PAUSING
} Horn_State_Enum;

typedef struct
{
    Horn_State_Enum state;
    Horn_Mode_Enum mode;
    uint32 next_time;
    uint8 step;
    uint8 repeat;
} Horn_State;

void Speaker_Horn_1s(void);
void Speaker_Horn_2s(void);
void Speaker_Horn_3s(void);
void Speaker_Horn_2beep(void);
void Speaker_Horn_3beep(void);
void Speaker_Horn_4beep(void);
void Speaker_Horn_LongShort(void);
void Speaker_Horn_Urgent(void);
void Speaker_Horn_Alarm(void);

// 非阻塞式鸣笛函数
void Speaker_Horn_Start(Horn_Mode_Enum mode);
void Speaker_Horn_Stop(void);
uint8 Speaker_Horn_IsPlaying(void);

#endif /* CODE_SPEAKER_H_ */

