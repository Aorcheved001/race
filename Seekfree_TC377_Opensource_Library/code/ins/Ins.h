/*
 * Ins.h
 * 惯性导航系统头文件，包含状态定义、配置参数和EKF结构
 */

#ifndef CODE_INS_INS_H_
#define CODE_INS_INS_H_

 //-------------------------------------------头文件包含------------------------------------------------------------
#include <stdint.h>
#include "vehicle_config.h"  // 引入统一车辆参数配置

 //-------------------------------------------宏定义----------------------------------------------------------------
#define INS_STATE_DIM        3                                          // 状态维度
#define INS_INPUT_DIM        2                                          // 输入维度 (轮速, 角速度)
// INS_WHEELBASE_M 已在 vehicle_config.h 中定义
#define INS_PI               3.14159265358979f                          // 圆周率
#define INS_DEG2RAD          0.017453292519943f                         // 度转弧度系数
#define INS_RAD2DEG          57.2957795130823f                          // 弧度转度系数

 //-------------------------------------------类型定义--------------------------------------------------------------
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
    uint8_t mag_valid;                                                   // 磁力计数据有效标志
    float delta_left_m;                                                  // 左轮位移增量 (米)，由编码器模块计算
    float delta_right_m;                                                 // 右轮位移增量 (米)，由编码器模块计算
} INS_Input;

typedef struct
{
    float kalman_6axis_q;                                                // 卡尔曼过程噪声参数
    float kalman_6axis_r;                                                // 卡尔曼测量噪声参数
    float kalman_6axis_T;                                                // 卡尔曼采样周期
    float mag_alpha;                                                     // 磁力计融合低通滤波系数
    float Q_yaw;                                                         // yaw过程噪声协方差(对应EKF的N_psi)
    float R_mag;                                                         // 磁力计测量噪声协方差(对应EKF的R)
    float wheelbase;                                                     // 轴距
    float tick_to_meter_left;                                            // 左轮脉冲转米系数
    float tick_to_meter_right;                                           // 右轮脉冲转米系数
    float zupt_speed_threshold;                                          // 零速检测阈值
    float zupt_gyro_threshold;                                           // 零速陀螺仪阈值
} INS_Config;

typedef struct {
    float x[2];          // [yaw, bias]
    float P[2][2];       // 误差协方差矩阵
    float N_psi;         // yaw 过程噪声功率谱密度 PSD (rad^2/s)，控制航向漂移速度
    float N_b;           // bias 过程噪声功率谱密度 PSD ((rad/s)^2/s)，控制零偏漂移速度
    float N_b_frozen;    // 冻结时的 N_b，用于静止时保持 bias
    float R;             // 测量噪声方差 (rad^2)
    float yaw_predict;   // Predict 步骤后的 yaw，用于 get_yaw_layers 输出
} YawEKF2State;

 //-------------------------------------------函数声明---------------------------------------------------------------
void Ins_init(void);                                                     // 初始化INS模块

void Ins_reset(float x, float y, float theta);                           // 重置INS状态

void Ins_set_config(const INS_Config *config);                           // 配置INS参数

void Ins_update(const INS_Input *input, float dt_s);                     // 更新INS状态

const INS_State* Ins_get_state(void);                                    // 获取当前INS状态
void Ins_get_attitude(float *roll, float *pitch, float *yaw);            // 获取姿态角(欧拉角)
void Ins_get_yaw_layers(float *yaw_gyro, float *yaw_mag_raw, float *yaw_mag_rel, float *yaw_ekf); // 获取多层yaw值
void Ins_get_yaw_debug_layers(float *yaw_gyro, float *mag_yaw_raw, float *mag_yaw_rel,
                               float *mag_yaw_corr_rel, float *yaw_fused);                         // 获取含LUT修正的调试yaw
void Ins_get_gyro_yaw_compare(float *yaw_z_bias, float *yaw_rp_bias, float *yaw_model); // 获取Z轴/rp补偿/车辆模型积分yaw
void Ins_get_mag_vector(float *mag_x, float *mag_y, float *mag_z);       // 获取磁力计原始数据(机体坐标系)

// 磁航向 LUT 修正后的数据获取
float Ins_get_mag_yaw_corr_rel(void);                                    // 获取LUT修正后的相对磁航向(弧度)
void Ins_set_mag_fusion_enabled(uint8_t enable);                         // 设置磁航向是否参与最终yaw融合
uint8_t Ins_is_mag_fusion_enabled(void);                                  // 查询磁航向融合开关

// yaw 同步接口（YAW_CORRECT 阶段矫正完成后调用）
void Ins_sync_yaw(float yaw_rad);                                        // 同步所有yaw状态到指定值

// 静止零偏校准（放弃磁力计，纯陀螺yaw方案）
void Ins_start_bias_calibration(void);                                   // 开始静止零偏校准（记录当前yaw，开始计时）
uint8_t Ins_update_bias_calibration(float dt_s, uint8_t is_stationary);  // 更新零偏校准（返回1=校准完成）
float Ins_get_calibrated_bias(void);                                     // 获取校准后的零偏值(rad/s)
void Ins_set_fixed_bias(float bias_rad_s);                               // 手动设置固定零偏
void Ins_set_position_freeze(uint8_t freeze);                            // 冻结/解冻位置积分（抬车时用）
uint8_t Ins_is_bias_calibrated(void);                                    // 查询零偏是否已校准

// 测试函数
void Ins_test_relative_mag(void);

#endif /* CODE_INS_INS_H_ */