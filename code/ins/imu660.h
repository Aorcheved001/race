/*
 * imu660.h
 * IMU数据处理模块
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

#ifndef CODE_BALANCE_IMU660_H_
#define CODE_BALANCE_IMU660_H_

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------宏定义区------------------------------------------------------------
#define M_PI 3.1415                                                        // 圆周率

 //-------------------------------------------结构体定义区------------------------------------------------------------
typedef struct
{
    float gyro_x;                                                          // X轴角速度
    float gyro_y;                                                          // Y轴角速度
    float gyro_z;                                                          // Z轴角速度
    float acc_x;                                                           // X轴加速度
    float acc_y;                                                           // Y轴加速度
    float acc_z;                                                           // Z轴加速度
    float gyro_x_bias;                                                     // X轴角速度零偏
    float gyro_y_bias;                                                     // Y轴角速度零偏
    float gyro_z_bias;                                                     // Z轴角速度零偏
    float acc_x_bias;                                                      // X轴加速度零偏
    float acc_y_bias;                                                      // Y轴加速度零偏
    float acc_z_bias;                                                      // Z轴加速度零偏
    float mag_x_bias;                                                      // X轴磁力计零偏
    float mag_y_bias;                                                      // Y轴磁力计零偏
    float mag_z_bias;                                                      // Z轴磁力计零偏
    float mag_x_scale;                                                     // X轴磁力计缩放
    float mag_y_scale;                                                     // Y轴磁力计缩放
    float mag_z_scale;                                                     // Z轴磁力计缩放
    float mag_x;                                                           // X轴磁力计
    float mag_y;                                                           // Y轴磁力计
    float mag_z;                                                           // Z轴磁力计
} imu_param;

typedef struct
{
    float pitch;                                                           // 俯仰角（弧度）
    float roll;                                                            // 横滚角（弧度）
    float yaw;                                                             // 航向角（弧度）
} euler_param;

typedef struct
{
    euler_param offset_angle;                                              // 偏移角度
    imu_param data_Raw;                                                    // 原始数据
    imu_param data_Ripen;                                                  // 处理后数据
    euler_param eulerAngle;                                                // 欧拉角
} imu660_struct;

 //-------------------------------------------全局变量声明区------------------------------------------------------------
extern imu_param imu_date;                                                 // IMU数据
extern imu660_struct imu660;                                               // IMU660结构体

 //-------------------------------------------椭球拟合宏定义区------------------------------------------------------------
#define MAG_CALIB_MAX_SAMPLES       500    // 磁力计校准最大采样点数
#define MAG_CALIB_MIN_SAMPLES       100    // 磁力计校准最小采样点数

 //-------------------------------------------函数声明区------------------------------------------------------------
void date_handle(imu_param *data_src);                                     // 处理IMU数据
void imu_bias_init(imu_param *data_src);                                   // 初始化IMU零偏
uint8 imu_mag_bias_load(imu_param *data_src);                              // 从Flash加载磁力计零偏
void imu_mag_bias_save(const imu_param *data_src);                         // 保存磁力计零偏到Flash
void imu_mag_calib_start(void);                                            // 开始磁力计校准
uint8 imu_mag_calib_finish(imu_param *data_src);                           // 完成磁力计校准
uint8 imu_mag_calib_is_active(void);                                       // 检查磁力计校准状态
void static_imu_test(imu_param *data_src);                                 // 静态IMU测试
void imu_gyro_z_autocalib(imu_param *data_src, unsigned int samples);      // 自动校准Z轴角速度

// 椭球拟合磁力计校准函数
void imu_mag_calib_ellipsoid_start(void);                                   // 开始椭球拟合校准
uint8 imu_mag_calib_ellipsoid_update(float mx, float my, float mz);       // 更新椭球拟合数据
uint8 imu_mag_calib_ellipsoid_finish(imu_param *data_src);                // 完成椭球拟合并计算参数
uint32 imu_mag_calib_get_sample_count(void);                               // 获取当前采样点数

void imu_mag_send_raw_data_to_pc(void);

#endif
