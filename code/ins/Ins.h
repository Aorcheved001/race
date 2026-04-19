/*
 * Ins.h
 * 惯性导航系统头文件，包含状态定义、配置参数和EKF结构
 */

#ifndef CODE_INS_INS_H_
#define CODE_INS_INS_H_

 //-------------------------------------------头文件包含------------------------------------------------------------
#include <stdint.h>

 //-------------------------------------------宏定义----------------------------------------------------------------
#define INS_STATE_DIM        3                                          // 状态维度
#define INS_INPUT_DIM        2                                          // 输入维度 (轮速, 角速度)
#define INS_WHEELBASE_M      0.2107f                                    // 轴距 (米)
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
void Ins_get_mag_vector(float *mag_x, float *mag_y, float *mag_z);       // 获取磁力计原始数据(机体坐标系)

// 测试函数
void Ins_test_relative_mag(void);

#endif /* CODE_INS_INS_H_ */