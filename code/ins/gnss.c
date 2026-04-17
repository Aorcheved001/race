/*
 * gnss.c
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */

#include "zf_common_headfile.h"
 //-------------------------------------------宏定义区------------------------------------------------------------
#define GNSS_WGS84_RE           6378137.0f                      // 地球半径 (m)
#define GNSS_DEG_TO_RAD         0.017453292519943295f           // 角度转弧度系数

#define GNSS_ORIGIN_SAMPLE_COUNT 50                             // 采集 50 个点用于计算原点
#define GNSS_OUTLIER_THRESHOLD_M 1.5f                           // 剔除距离均值大于 1.5 米的离群点

 //-------------------------------------------内部结构体区------------------------------------------------------------
static gnss_state_t  s_gnss_state  = {0};                       // 本地 GNSS 状态缓存
static gnss_origin_t s_gnss_origin = {0};                       // 本地 GNSS 原点缓存

static gnss_sample_t s_origin_samples[GNSS_ORIGIN_SAMPLE_COUNT];// 原点采样缓存数组
static uint16_t      s_sample_count = 0;                        // 已采样点数统计


 //-------------------------------------------函数定义区------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
//  @brief      判断定位是否可信
//  @param      fix_type    定位类型
//  @param      num_sats    卫星数
//  @param      hdop        HDOP 值用于判断当前收到gnss的值是否可信
//  @return     1 表示解可用，0 表示解无效
//--------------------------------------------------------------------------------------------------
static uint8_t gnss_is_solution_acceptable(uint8_t fix_type,
                                           uint8_t num_sats,
                                           float   hdop)
{
    if (fix_type == 0u)                 // 定位类型为 0 视为无效解
    {
        return 0u;
    }

    if (num_sats < 4u)                  // 卫星数小于 4 视为无效解
    {
        return 0u;
    }

    if (hdop <= 0.0f || hdop > 1.2f)    // [BugFix] hdop<=0 为无效/未上报值，hdop>1.2 精度不足，均拒绝
    {
        return 0u;
    }

    return 1u;                          // 其他情况视为有效解
}

//--------------------------------------------------------------------------------------------------
//  @brief      经纬度转换为局部平面坐标
//  @param      lat_deg     纬度（度）
//  @param      lon_deg     经度（度）
//  @param      x_m         输出的 x 坐标（米）
//  @param      y_m         输出的 y 坐标（米）
//  @note       适用于小范围近似，将原点设为首次有效定位点
//              x = (lon - lon0) * cos(lat0) * Re
//              y = (lat - lat0) * Re
//--------------------------------------------------------------------------------------------------
static void gnss_latlon_to_local_xy(float  lat_deg,
                                    float  lon_deg,
                                    float *x_m,
                                    float *y_m)
{
    if ((x_m == NULL) || (y_m == NULL))                     // 输入参数为空指针，直接返回
    {
        return;
    }

    if (s_gnss_origin.is_set == 0u)                        // 原点未设置，直接返回
    {
        *x_m = 0.0f;                                        // 原点未设置，x 坐标设为 0
        *y_m = 0.0f;                                        // 原点未设置，y 坐标设为 0
        return;
    }

    float lat0_deg = s_gnss_origin.lat_deg;                 // 原点纬度（度）
    float lon0_deg = s_gnss_origin.lon_deg;                 // 原点经度（度）

    float d_lat_deg = lat_deg - lat0_deg;                   // 纬度差（度）
    float d_lon_deg = lon_deg - lon0_deg;                   // 经度差（度）

    float lat0_rad  = lat0_deg * GNSS_DEG_TO_RAD;          // 原点纬度（弧度）

    float cos_lat0  =  cosf(lat0_rad);                     // 原点纬度的余弦值
    float d_lat_rad = d_lat_deg * GNSS_DEG_TO_RAD;        // 纬度差（弧度）
    float d_lon_rad = d_lon_deg * GNSS_DEG_TO_RAD;        // 经度差（弧度）

    float r_earth   = GNSS_WGS84_RE;                      // 地球半径（米）

    *x_m = d_lon_rad * cos_lat0 * r_earth;                // 计算 x 坐标（米）
    *y_m = d_lat_rad * r_earth;                           // 计算 y 坐标（米）
}

//--------------------------------------------------------------------------------------------------
//  @brief      剔除离群点并计算平滑原点
//  @param      无
//  @note       内部函数，对采集的 GNSS_ORIGIN_SAMPLE_COUNT 个点进行两步法均值滤波，
//              丢弃偏离初次均值超过 GNSS_OUTLIER_THRESHOLD_M 米的噪点。
//--------------------------------------------------------------------------------------------------
static void calculate_and_set_origin(void)
{
    float sum_lat = 0.0f, sum_lon = 0.0f, sum_alt = 0.0f;

    // 1. 求初步均值，作为剔除离群点的基准参考点
    for (int i = 0; i < GNSS_ORIGIN_SAMPLE_COUNT; i++)
    {
        sum_lat += s_origin_samples[i].lat_deg;                 // 累加纬度
        sum_lon += s_origin_samples[i].lon_deg;                 // 累加经度
        sum_alt += s_origin_samples[i].alt_m;                   // 累加高度
    }
    float mean_lat = sum_lat / GNSS_ORIGIN_SAMPLE_COUNT;        // 计算纬度初次均值
    float mean_lon = sum_lon / GNSS_ORIGIN_SAMPLE_COUNT;        // 计算经度初次均值
    float mean_alt = sum_alt / GNSS_ORIGIN_SAMPLE_COUNT;        // 计算高度初次均值

    // 2. 剔除离群点并重新求均值
    float final_sum_lat = 0.0f, final_sum_lon = 0.0f, final_sum_alt = 0.0f;
    int valid_count = 0;                                        // 有效非离群点计数

    float lat0_rad = mean_lat * GNSS_DEG_TO_RAD;                // 基准纬度转弧度
    float cos_lat0 = cosf(lat0_rad);                            // 基准纬度的余弦值，用于经度距离补偿

    for (int i = 0; i < GNSS_ORIGIN_SAMPLE_COUNT; i++)
    {
        float d_lat_deg = s_origin_samples[i].lat_deg - mean_lat;
        float d_lon_deg = s_origin_samples[i].lon_deg - mean_lon;

        // 近似计算样本点到均值点的平面距离（米）
        float dx = d_lon_deg * GNSS_DEG_TO_RAD * cos_lat0 * GNSS_WGS84_RE;
        float dy = d_lat_deg * GNSS_DEG_TO_RAD * GNSS_WGS84_RE;
        float dist = sqrtf(dx * dx + dy * dy);

        // 只有在阈值范围内的点才参与最终计算，剔除明显跳点
        if (dist <= GNSS_OUTLIER_THRESHOLD_M)
        {
            final_sum_lat += s_origin_samples[i].lat_deg;       // 累加有效纬度
            final_sum_lon += s_origin_samples[i].lon_deg;       // 累加有效经度
            final_sum_alt += s_origin_samples[i].alt_m;         // 累加有效高度
            valid_count++;
        }
    }

    // 3. 设置最终原点
    if (valid_count > 0)
    {
        gnss_set_origin_deg(final_sum_lat / valid_count,        // 取有效点均值设为原点
                            final_sum_lon / valid_count,
                            final_sum_alt / valid_count);
    }
    else
    {
        // 极端情况：全部是离群点（散布太大），降级使用初次均值
        gnss_set_origin_deg(mean_lat, mean_lon, mean_alt);
    }
}

//--------------------------------------------------------------------------------------------------
//  @brief      GPS 初始化
//  @param      无
//--------------------------------------------------------------------------------------------------
void gps_init(void)
{
    gnss_init(TAU1201);                                             // 初始化 GNSS 模块（底层驱动由库提供）

    memset(&s_gnss_state, 0, sizeof(s_gnss_state));                 // 清零状态缓存
    memset(&s_gnss_origin, 0, sizeof(s_gnss_origin));               // 清零原点缓存
    s_sample_count = 0;                                             // 清零原点采样计数
}

//--------------------------------------------------------------------------------------------------
//  @brief      设置 GNSS 参考原点（经纬高）
//  @param      lat_deg     纬度（度）
//  @param      lon_deg     经度（度）
//  @param      alt_m       高度（米）
//--------------------------------------------------------------------------------------------------
void gnss_set_origin_deg(float lat_deg, float lon_deg, float alt_m)
{
    s_gnss_origin.lat_deg = lat_deg;                        // 设置纬度（度）
    s_gnss_origin.lon_deg = lon_deg;                        // 设置经度（度）
    s_gnss_origin.alt_m   = alt_m;                          // 设置高度（米）
    s_gnss_origin.is_set  = 1u;                             // 标记原点已设置
}

//--------------------------------------------------------------------------------------------------
//  @brief      更新 GNSS 解并刷新局部坐标
//  @param      lat_deg     纬度（度）
//  @param      lon_deg     经度（度）
//  @param      alt_m       高度（米）
//  @param      fix_type    定位类型
//  @param      num_sats    卫星数
//  @param      hdop        HDOP 值
//--------------------------------------------------------------------------------------------------
void gnss_update_solution_deg(float lat_deg,
                              float lon_deg,
                              float alt_m,
                              uint8_t fix_type,
                              uint8_t num_sats,
                              float hdop)
{
    s_gnss_state.lat_deg  = lat_deg;                        // 更新纬度（度）
    s_gnss_state.lon_deg  = lon_deg;                        // 更新经度（度）
    s_gnss_state.alt_m    = alt_m;                          // 更新高度（米）
    s_gnss_state.fix_type = fix_type;                       // 更新定位类型
    s_gnss_state.num_sats = num_sats;                       // 更新卫星数
    s_gnss_state.hdop     = hdop;                           // 更新 HDOP 值

    // 若原点未设置，收集一定数量的有效点进行平均和剔除离群点
    if (s_gnss_origin.is_set == 0u)                        // 原点未设置
    {
        if (gnss_is_solution_acceptable(fix_type, num_sats, hdop)) // 检查解是否可接受
        {
            if (s_sample_count < GNSS_ORIGIN_SAMPLE_COUNT)
            {
                s_origin_samples[s_sample_count].lat_deg = lat_deg; // 存入纬度缓存
                s_origin_samples[s_sample_count].lon_deg = lon_deg; // 存入经度缓存
                s_origin_samples[s_sample_count].alt_m   = alt_m;   // 存入高度缓存
                s_sample_count++;

                if (s_sample_count >= GNSS_ORIGIN_SAMPLE_COUNT)
                {
                    calculate_and_set_origin();                     // 样本收集完毕，计算平滑原点
                }
            }
        }
    }

    // 原点有效时计算局部坐标
    if (s_gnss_origin.is_set != 0u)                        // 原点已设置
    {
        // 综合原点状态、解类型、卫星数和 HDOP 判断位置有效性
        uint8_t is_valid = (uint8_t)gnss_is_solution_acceptable(fix_type, num_sats, hdop);
           s_gnss_state.pos_valid = is_valid;
           if (s_gnss_state.pos_valid)
           {
               gnss_latlon_to_local_xy(lat_deg, lon_deg,
                                       &s_gnss_state.x_m,
                                       &s_gnss_state.y_m);   // 计算局部坐标
        }
    }


}

//--------------------------------------------------------------------------------------------------
//  @brief      获取 GNSS 状态
//  @param      无
//  @return     指向 gnss_state_t 结构体的指针
//--------------------------------------------------------------------------------------------------
const gnss_state_t* gnss_get_state(void)
{
    return &s_gnss_state;
}

//--------------------------------------------------------------------------------------------------
//  @brief      获取当前位置局部坐标
//  @param      x_m         指向 x 坐标的指针（米）
//  @param      y_m         指向 y 坐标的指针（米）
//  @return     位置有效性（1：有效，0：无效）
//--------------------------------------------------------------------------------------------------
uint8_t gnss_get_position_xy(float *x_m, float *y_m)
{
    if (s_gnss_state.pos_valid == 0u)               // 位置无效
    {
        // 无效时输出 0，避免上层使用脏数据
        if (x_m != NULL)                // x 坐标指针非空
        {
            *x_m = 0.0f;                // 输出 0 表示无效
        }
        if (y_m != NULL)                // y 坐标指针非空
        {
            *y_m = 0.0f;                // 输出 0 表示无效
        }
        return 0u;
    }

    if (x_m != NULL)                     // x 坐标指针非空
    {
        *x_m = s_gnss_state.x_m;        // 输出 x 坐标
    }
    if (y_m != NULL)                     // y 坐标指针非空
    {
        *y_m = s_gnss_state.y_m;        // 输出 y 坐标
    }

    return 1u;
}


