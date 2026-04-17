/*
 * gnss.h
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */
#ifndef CODE_GNSS_H_
#define CODE_GNSS_H_

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------宏定义区------------------------------------------------------------



 //-------------------------------------------结构体区------------------------------------------------------------

typedef struct
{
    float   lat_deg;        // 纬度 (deg, WGS84)
    float   lon_deg;        // 经度 (deg, WGS84)
    float   alt_m;          // 海拔高度 (m)

    uint8_t fix_type;       // 定位类型 0=无效 1=单点 2=差分/增强
    uint8_t num_sats;       // 可用卫星数量
    float   hdop;           // 水平精度因子

    float   x_m;            // 相对参考原点的 X 坐标 (m)
    float   y_m;            // 相对参考原点的 Y 坐标 (m)

    uint8_t pos_valid;      // 位置是否有效（已建原点/解算可用）
} gnss_state_t;             // GNSS 状态数据结构（原始解 + 局部坐标）


typedef struct
{
    float   lat_deg;        // 纬度 (deg)
    float   lon_deg;        // 经度 (deg)
    float   alt_m;          // 海拔高度 (m)
    uint8_t is_set;         // 原点是否已设置
} gnss_origin_t;            // GNSS 参考原点（用于局部坐标转换）

typedef struct {
    float lat_deg;                                              // 样本纬度（度）
    float lon_deg;                                              // 样本经度（度）
    float alt_m;                                                // 样本高度（米）
} gnss_sample_t;


 //-------------------------------------------函数声明区------------------------------------------------------------
void gps_init(void);                                             // GNSS 初始化（设置默认参数）

void gnss_set_origin_deg(float lat_deg, float lon_deg, float alt_m);  // 设置 GNSS 参考原点（经纬高）

void gnss_update_solution_deg(float lat_deg,                     //经度
                              float lon_deg,                     //纬度
                              float alt_m,                       //海拔
                              uint8_t fix_type,                  //定位类型 0=无效 1=单点 2=差分/增强
                              uint8_t num_sats,                  //可用卫星数量
                              float hdop);                       // 更新 GNSS 解并计算局部坐标

const gnss_state_t* gnss_get_state(void);                        // 获取当前 GNSS 状态结构体（只读）

uint8_t gnss_get_position_xy(float *x_m, float *y_m);            // 获取当前位置局部坐标（m）

#endif /* CODE_GNSS_H_ */
