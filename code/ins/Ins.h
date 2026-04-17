/*
 * Ins.h
 * 三层INS惯性导航系统
 */

#ifndef CODE_INS_INS_H_
#define CODE_INS_INS_H_

 //-------------------------------------------头文件区------------------------------------------------------------
#include "stdint.h"

 //-------------------------------------------宏定义区------------------------------------------------------------
#define INS_STATE_DIM        3                                          // 状态向量维度
#define INS_INPUT_DIM        2                                          // 输入向量维度 (速度, 角速度)
#define INS_WHEELBASE_M      0.2107f                                    // 轴距 (米)
#define INS_PI               3.14159265358979f                          // 圆周率
#define INS_DEG2RAD          0.017453292519943f                         // 角度转弧度系数
#define INS_RAD2DEG          57.2957795130823f                          // 弧度转角度系数

 //-------------------------------------------结构体区------------------------------------------------------------
typedef struct
{
    float x;                                                             // X坐标 (米)
    float y;                                                             // Y坐标 (米)
    float yaw;                                                           // 航向角 (弧度)
} INS_State;

typedef struct
{
    float v_mps;                                                         // 线速度 (米/秒)
    float omega_rad_s;                                                   // 角速度 (弧度/秒)
    float gyro_z_rad_s;                                                  // 陀螺仪Z轴角速度 (弧度/秒)
    float mag_yaw_rad;                                                   // 磁力计航向角 (弧度)
    uint8_t mag_valid;                                                   // 磁力计数据有效性标志
} INS_Input;

typedef struct
{
    float kalman_6axis_q;                                                // 六轴卡尔曼系统噪声协方差
    float kalman_6axis_r;                                                // 六轴卡尔曼测量噪声协方差
    float kalman_6axis_T;                                                // 六轴卡尔曼离散时间
    float mag_alpha;                                                     // 磁力计互补滤波系数
    float Q_yaw;                                                         // yaw过程噪声参数（预留）
    float R_mag;                                                         // 磁力计测量噪声参数（预留）
    float wheelbase;                                                     // 轴距
    float tick_to_meter_left;                                            // 左轮编码器系数
    float tick_to_meter_right;                                           // 右轮编码器系数
    float zupt_speed_threshold;                                          // 零速速度阈值
    float zupt_gyro_threshold;                                           // 零速陀螺仪阈值
} INS_Config;

 //-------------------------------------------函数声明区------------------------------------------------------------
void Ins_init(void);                                                     // 初始化INS系统

void Ins_reset(float x, float y, float theta);                           // 重置INS状态

void Ins_set_config(const INS_Config *config);                           // 设置INS配置参数

void Ins_update(const INS_Input *input, float dt_s);                     // 更新INS状态

const INS_State* Ins_get_state(void);                                    // 获取当前INS状态
void Ins_get_yaw_layers(float *yaw_gyro, float *yaw_stable, float *yaw_final); // 获取三层yaw值

#endif /* CODE_INS_INS_H_ */
