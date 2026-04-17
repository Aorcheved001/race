/*
 * encoder.h
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */
#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

//-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"


 //-------------------------------------------宏定义区------------------------------------------------------------
#define ENCODER_LEFT_ID     TIM5_ENCODER                                // 左轮编码器使用的定时器
#define ENCODER_LEFT_PIN_A  TIM5_ENCODER_CH1_P10_3                      // 左轮编码器引脚 A
#define ENCODER_LEFT_PIN_B  TIM5_ENCODER_CH2_P10_1                      // 左轮编码器引脚 B

#define ENCODER_RIGHT_ID     TIM6_ENCODER                               // 右轮编码器使用的定时器
#define ENCODER_RIGHT_PIN_A  TIM6_ENCODER_CH1_P20_3                     // 右轮编码器引脚 A
#define ENCODER_RIGHT_PIN_B  TIM6_ENCODER_CH2_P20_0                     // 右轮编码器引脚 B


 //-------------------------------------------结构体区------------------------------------------------------------
typedef struct
{
    int16 tick_left;                        // 左轮当前周期 tick 增量
    int16 tick_right;                       // 右轮当前周期 tick 增量
    float delta_left_m;                     // 左轮当前周期位移增量（m）
    float delta_right_m;                    // 右轮当前周期位移增量（m）
    float odom_left_m;                      // 自系统启动以来的左轮累计里程（m）
    float odom_right_m;                     // 自系统启动以来的右轮累计里程（m）
    float speed_left_mps;                   // 左轮当前周期瞬时速度（m/s）
    float speed_right_mps;                  // 右轮当前周期瞬时速度（m/s）
    float speed_average_mps;                // 左右轮速度算术平均（m/s）
} EncoderLayerState;


 //-------------------------------------------函数声明区------------------------------------------------------------
void encoder_init(void);              // 编码器层初始化

void encoder_layer_set_model(float tick_to_meter_left, float tick_to_meter_right, float sample_dt_s);           // 设置编码器模型参数

void encoder_layer_update(void);            // 周期更新（一次采样与计算）

const EncoderLayerState* encoder_layer_get_state(void);                                                         // 获取当前周期计算出的编码器层状态（只读指针）

void encoder_layer_clear_odom(void);        // 清零累计里程（不影响当前周期增量与速度）

int32 encoder_layer_get_raw_count_left(void);// 获取当前硬件计数值（不清零）

int32 encoder_layer_get_raw_count_right(void);  // 获取当前硬件计数值（不清零）

void  encoder_layer_clear_raw_counts(void);  // 清零左右硬件计数

#endif /* CODE_ENCODER_H_ */
