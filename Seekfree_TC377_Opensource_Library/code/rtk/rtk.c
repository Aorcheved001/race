/*
 * rtk.c
 *
 *  Created on: 2026-06-17
 *      Author: RTK坐标转换层
 *
 *  功能：将GN43RFA RTK模块输出的经纬度转换为局部平面坐标(ENU)，
 *        并提供双天线真北航向(antenna_direction)用于坐标系对齐。
 *
 *  数据流：
 *    1. gnss_uart_callback() 接收NMEA数据 → gnss_flag=1
 *    2. rtk_update() 在主循环中调用 → gnss_data_parse() 解析
 *    3. 从 gnss 全局结构体读取 lat/lon/antenna_direction
 *    4. 经纬度 → ENU局部坐标 (x_enu, y_enu)
 *    5. antenna_direction → yaw_true_north (rad)
 */

#include "zf_common_headfile.h"
#include "rtk.h"
#include "zf_device_gnss.h"

 //-------------------------------------------内部变量区------------------------------------------------------------

static rtk_state_t   s_rtk_state   = {0};                       // RTK状态缓存
static rtk_origin_t  s_rtk_origin  = {0};                       // RTK原点缓存

static rtk_sample_t  s_origin_samples[RTK_ORIGIN_SAMPLE_COUNT]; // 原点采样缓存
static uint16_t      s_sample_count = 0;                        // 已采样点数

// INS对齐参考（对齐时刻的RTK状态快照）
static float    s_align_x_enu_0 = 0.0f;                         // 对齐时刻RTK ENU x
static float    s_align_y_enu_0 = 0.0f;                         // 对齐时刻RTK ENU y
static float    s_align_yaw_0   = 0.0f;                         // 对齐时刻RTK真北航向(rad)
static uint8_t  s_is_aligned    = 0u;                           // 是否已完成INS对齐

// 天线安装偏角（运行时可配置，覆盖编译时默认值）
static float    s_mount_offset_deg = RTK_ANTENNA_MOUNT_OFFSET_DEG;  // 天线安装偏角（度）

 //-------------------------------------------内部函数区------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//  @brief      角度归一化到 [-π, π]
//--------------------------------------------------------------------------------------------------
static float rtk_normalize_angle(float angle)
{
    while (angle > RTK_PI)  angle -= 2.0f * RTK_PI;
    while (angle < -RTK_PI) angle += 2.0f * RTK_PI;
    return angle;
}

//--------------------------------------------------------------------------------------------------
//  @brief      判断GNSS定位是否可信
//  @param      fix_quality     GGA fix_quality (0=无效 1=单点 2=差分 4=RTK固定 5=RTK浮点)
//  @param      num_sats        卫星数
//  @return     1=可信 0=不可信
//--------------------------------------------------------------------------------------------------
static uint8_t rtk_is_solution_acceptable(uint8_t fix_quality, uint8_t num_sats)
{
    if (fix_quality == 0u)               // GGA定位无效
    {
        return 0u;
    }

    if (num_sats < RTK_MIN_SATS_FOR_POS) // 卫星数不足
    {
        return 0u;
    }

#if RTK_REQUIRE_RTK_FIX
    if ((fix_quality != 4u) && (fix_quality != 5u))
    {
        return 0u;
    }
#endif
    return 1u;
}

//--------------------------------------------------------------------------------------------------
//  @brief      高精度整数经纬度转换为ENU局部平面坐标
//  @param      lat_deg_e9  纬度，单位1e-9度
//  @param      lon_deg_e9  经度，单位1e-9度
//  @param      x_enu       输出的东向坐标（米）
//  @param      y_enu       输出的北向坐标（米）
//  @note       ENU投影：x=东，y=北
//              x = (lon - lon0) * cos(lat0) * Re
//              y = (lat - lat0) * Re
//--------------------------------------------------------------------------------------------------
static void rtk_latlon_e9_to_enu(int64_t lat_deg_e9,
                                 int64_t lon_deg_e9,
                                 float  *x_enu,
                                 float  *y_enu)
{
    if ((x_enu == NULL) || (y_enu == NULL))
    {
        return;
    }

    if (s_rtk_origin.is_set == 0u)
    {
        *x_enu = 0.0f;
        *y_enu = 0.0f;
        return;
    }

    int64_t d_lat_e9 = lat_deg_e9 - s_rtk_origin.lat_deg_e9;
    int64_t d_lon_e9 = lon_deg_e9 - s_rtk_origin.lon_deg_e9;

    double lat0_deg  = (double)s_rtk_origin.lat_deg_e9 * 1.0e-9;
    double lat0_rad  = lat0_deg * RTK_DEG_TO_RAD;
    double cos_lat0  = cos(lat0_rad);

    double d_lat_rad = ((double)d_lat_e9 * 1.0e-9) * RTK_DEG_TO_RAD;
    double d_lon_rad = ((double)d_lon_e9 * 1.0e-9) * RTK_DEG_TO_RAD;

    *x_enu = (float)(d_lon_rad * cos_lat0 * RTK_WGS84_RE);
    *y_enu = (float)(d_lat_rad * RTK_WGS84_RE);
}

//--------------------------------------------------------------------------------------------------
//  @brief      剔除离群点并计算平滑原点
//  @note       两步法均值滤波，丢弃偏离初次均值超过阈值的噪点
//--------------------------------------------------------------------------------------------------
static void calculate_and_set_origin(void)
{
    int64_t sum_lat_e9 = 0;
    int64_t sum_lon_e9 = 0;
    float sum_alt = 0.0f;

    for (int i = 0; i < RTK_ORIGIN_SAMPLE_COUNT; i++)
    {
        sum_lat_e9 += s_origin_samples[i].lat_deg_e9;
        sum_lon_e9 += s_origin_samples[i].lon_deg_e9;
        sum_alt += s_origin_samples[i].alt_m;
    }

    int64_t mean_lat_e9 = sum_lat_e9 / RTK_ORIGIN_SAMPLE_COUNT;
    int64_t mean_lon_e9 = sum_lon_e9 / RTK_ORIGIN_SAMPLE_COUNT;
    float mean_alt = sum_alt / RTK_ORIGIN_SAMPLE_COUNT;

    int64_t final_sum_lat_e9 = 0;
    int64_t final_sum_lon_e9 = 0;
    float final_sum_alt = 0.0f;
    int valid_count = 0;

    double lat0_rad = ((double)mean_lat_e9 * 1.0e-9) * RTK_DEG_TO_RAD;
    double cos_lat0 = cos(lat0_rad);

    for (int i = 0; i < RTK_ORIGIN_SAMPLE_COUNT; i++)
    {
        int64_t d_lat_e9 = s_origin_samples[i].lat_deg_e9 - mean_lat_e9;
        int64_t d_lon_e9 = s_origin_samples[i].lon_deg_e9 - mean_lon_e9;
        double dx = ((double)d_lon_e9 * 1.0e-9) * RTK_DEG_TO_RAD * cos_lat0 * RTK_WGS84_RE;
        double dy = ((double)d_lat_e9 * 1.0e-9) * RTK_DEG_TO_RAD * RTK_WGS84_RE;
        double dist = sqrt(dx * dx + dy * dy);

        if (dist <= RTK_OUTLIER_THRESHOLD_M)
        {
            final_sum_lat_e9 += s_origin_samples[i].lat_deg_e9;
            final_sum_lon_e9 += s_origin_samples[i].lon_deg_e9;
            final_sum_alt += s_origin_samples[i].alt_m;
            valid_count++;
        }
    }

    if (valid_count > 0)
    {
        rtk_set_origin_e9(final_sum_lat_e9 / valid_count,
                          final_sum_lon_e9 / valid_count,
                          final_sum_alt / valid_count);
    }
    else
    {
        rtk_set_origin_e9(mean_lat_e9, mean_lon_e9, mean_alt);
    }
}

//--------------------------------------------------------------------------------------------------
//  @brief      将antenna_direction(0~360°, 真北=0, 顺时针)转换为yaw(rad)
//  @param      antenna_dir_deg    天线方向角(度)
//  @return     yaw(rad)，0=北，顺时针为正，范围[-π, π]
//  @note       antenna_direction: 0=北, 90=东, 180=南, 270=西
//              转换为数学角度: 北=0, 东=π/2, 南=±π, 西=-π/2
//
//              GN43RFA双天线基线方向修正（逐飞文档: antenna_direction = MAIN→AUX）:
//              当前安装: MAIN天线在车头，AUX天线在车尾。
//              MAIN→AUX = 车头→车尾 = 车辆航向 + 180°。
//              因此: 车辆航向 = antenna_direction - 180° = antenna_direction + 180° (归一化)。
//--------------------------------------------------------------------------------------------------
static float antenna_direction_to_yaw_rad(float antenna_dir_deg)
{
    // antenna_direction: 0~360, 真北=0, 顺时针 (MAIN→AUX基线方向)
    // 车辆航向 = antenna_direction + 180°
    //   +180°: MAIN→AUX指向车尾，取反得车头方向
    //   安装偏角不在这里混入，统一由 rtk_get_yaw_ref_deg() 扣除。
    float dir = antenna_dir_deg + 180.0f;
    // 归一化到 [0, 360)
    while (dir >=  360.0f) dir -= 360.0f;
    while (dir <     0.0f) dir += 360.0f;
    // 转换到 [-180, 180] 范围
    if (dir > 180.0f)
    {
        dir -= 360.0f;
    }
    return dir * RTK_DEG_TO_RAD;
}

 //-------------------------------------------外部函数区------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//  @brief      初始化RTK模块
//--------------------------------------------------------------------------------------------------
void rtk_init(void)
{
    gnss_init(GN43RFA);                                            // 初始化GN43RFA RTK模块

    memset(&s_rtk_state, 0, sizeof(s_rtk_state));
    memset(&s_rtk_origin, 0, sizeof(s_rtk_origin));
    s_sample_count = 0;
    s_align_x_enu_0 = 0.0f;
    s_align_y_enu_0 = 0.0f;
    s_align_yaw_0   = 0.0f;
    s_is_aligned    = 0u;
}

//--------------------------------------------------------------------------------------------------
//  @brief      更新RTK状态（在主循环中调用）
//  @note       检查gnss_flag，如果有新数据则解析并更新坐标
//--------------------------------------------------------------------------------------------------
void rtk_update(void)
{
    if (gnss_flag == 0)                     // 没有新数据
    {
        return;
    }
    gnss_flag = 0;                          // 清除标志

    // 解析GNSS数据
    uint8 parse_result = gnss_data_parse();
    if (parse_result != 0)                  // 解析失败
    {
        return;
    }

    // 从逐飞库的全局结构体读取数据
    // gnss.latitude: 纬度(度), double类型, 格式: dd.mmmmmm
    // gnss.longitude: 经度(度), double类型, 格式: ddd.mmmmmm
    // gnss.state: 1=定位有效
    // gnss.satellite_used: 卫星数
    // gnss.antenna_direction: 双天线真北航向(0~360°)
    // gnss.antenna_direction_state: 1=有效

    // lat/lon保留给状态显示；RTK ENU使用deg_e9整数差分，避免小位移量化
    double lat_deg  = gnss.latitude;
    double lon_deg  = gnss.longitude;
    int64_t lat_deg_e9 = gnss.latitude_deg_e9;
    int64_t lon_deg_e9 = gnss.longitude_deg_e9;
    float  alt_m    = gnss.height;
    uint8_t state  = gnss.state;         // RMC有效标志 (0/1)
    uint8_t sats   = gnss.satellite_used;
    uint8_t fix_q  = gnss.fix_quality;   // GGA定位质量 0=无效 1=单点 2=差分 4=RTK固定 5=RTK浮点

    // 更新RTK状态中的原始数据
    s_rtk_state.lat_deg   = lat_deg;
    s_rtk_state.lon_deg   = lon_deg;
    s_rtk_state.alt_m     = alt_m;
    s_rtk_state.fix_state = fix_q;       // 使用 GGA fix_quality 而非 RMC state
    s_rtk_state.num_sats  = sats;

    // 更新双天线真北航向
    if (gnss.antenna_direction_state == 1u)
    {
        s_rtk_state.yaw_true_north = antenna_direction_to_yaw_rad(gnss.antenna_direction);
        s_rtk_state.yaw_valid = 1u;
    }
    else
    {
        s_rtk_state.yaw_valid = 0u;
    }

    // 若原点未设置，收集有效点进行平均
    if (s_rtk_origin.is_set == 0u)
    {
        if (rtk_is_solution_acceptable(fix_q, sats) && (gnss.latlon_high_precision_valid != 0u))
        {
            if (s_sample_count < RTK_ORIGIN_SAMPLE_COUNT)
            {
                s_origin_samples[s_sample_count].lat_deg_e9 = lat_deg_e9;
                s_origin_samples[s_sample_count].lon_deg_e9 = lon_deg_e9;
                s_origin_samples[s_sample_count].alt_m   = alt_m;
                s_sample_count++;

                if (s_sample_count >= RTK_ORIGIN_SAMPLE_COUNT)
                {
                    calculate_and_set_origin();
                }
            }
        }
    }

    // 原点有效时计算ENU坐标
    if (s_rtk_origin.is_set != 0u)
    {
        uint8_t is_valid = (uint8_t)(rtk_is_solution_acceptable(fix_q, sats) && (gnss.latlon_high_precision_valid != 0u));
        s_rtk_state.pos_valid = is_valid;

        if (is_valid)
        {
            rtk_latlon_e9_to_enu(lat_deg_e9, lon_deg_e9,
                                  &s_rtk_state.x_enu,
                                  &s_rtk_state.y_enu);
        }
    }

    // ---- INS坐标系对齐 ----
    // 自动对齐：当RTK位置有效且航向有效时，记录对齐参考
    // 假设此时INS近似在(0,0,0)（上电后车辆静止等待RTK定位）
    if (s_is_aligned == 0u &&
        s_rtk_state.pos_valid != 0u &&
        s_rtk_state.yaw_valid != 0u)
    {
        s_align_x_enu_0 = s_rtk_state.x_enu;
        s_align_y_enu_0 = s_rtk_state.y_enu;
        s_align_yaw_0   = s_rtk_state.yaw_true_north;
        s_is_aligned    = 1u;
    }

    // 计算INS对齐坐标
    // ENU → INS 旋转推导（INS: x=前 y=左, yaw逆时针为正；ENU: x=东 y=北）：
    //   车体前向 f = (sinθ?, cosθ?)  在ENU中的分量
    //   车体左向 l = (-cosθ?, sinθ?) 在ENU中的分量
    //   x_ins = d·f =  sinθ?·dx + cosθ?·dy
    //   y_ins = d·l = -cosθ?·dx + sinθ?·dy
    //   yaw_ins = 对齐航向 - 当前航向（CCW正，与INS一致）
    // 其中 θ? = 对齐时刻的真北航向，dx/dy = 相对对齐点的ENU位移
    if (s_is_aligned != 0u && s_rtk_state.pos_valid != 0u)
    {
        float dx    = s_rtk_state.x_enu - s_align_x_enu_0;
        float dy    = s_rtk_state.y_enu - s_align_y_enu_0;
        float sin_t = sinf(s_align_yaw_0);
        float cos_t = cosf(s_align_yaw_0);

        s_rtk_state.x_ins   = sin_t * dx + cos_t * dy;
        s_rtk_state.y_ins   = -cos_t * dx + sin_t * dy;
        s_rtk_state.yaw_ins = rtk_normalize_angle(s_align_yaw_0 - s_rtk_state.yaw_true_north);
        s_rtk_state.is_aligned = 1u;
    }
    else
    {
        s_rtk_state.x_ins     = 0.0f;
        s_rtk_state.y_ins     = 0.0f;
        s_rtk_state.yaw_ins   = 0.0f;
        s_rtk_state.is_aligned = 0u;
    }
}


//--------------------------------------------------------------------------------------------------
//  @brief      重置RTK到INS坐标系的对齐参考
//  @note       下一次位置和航向均有效时，使用当前RTK点作为新的INS原点/零航向
//--------------------------------------------------------------------------------------------------
void rtk_reset_alignment(void)
{
    s_align_x_enu_0 = 0.0f;
    s_align_y_enu_0 = 0.0f;
    s_align_yaw_0   = 0.0f;
    s_is_aligned    = 0u;

    s_rtk_state.x_ins      = 0.0f;
    s_rtk_state.y_ins      = 0.0f;
    s_rtk_state.yaw_ins    = 0.0f;
    s_rtk_state.is_aligned = 0u;
}
//--------------------------------------------------------------------------------------------------
//  @brief      获取RTK状态
//--------------------------------------------------------------------------------------------------
const rtk_state_t* rtk_get_state(void)
{
    return &s_rtk_state;
}

//--------------------------------------------------------------------------------------------------
//  @brief      获取ENU坐标
//--------------------------------------------------------------------------------------------------
uint8_t rtk_get_position_xy(float *x_enu, float *y_enu)
{
    if (s_rtk_state.pos_valid == 0u)
    {
        if (x_enu != NULL) *x_enu = 0.0f;
        if (y_enu != NULL) *y_enu = 0.0f;
        return 0u;
    }

    if (x_enu != NULL) *x_enu = s_rtk_state.x_enu;
    if (y_enu != NULL) *y_enu = s_rtk_state.y_enu;
    return 1u;
}

//--------------------------------------------------------------------------------------------------
//  @brief      获取真北航向
//--------------------------------------------------------------------------------------------------
uint8_t rtk_get_yaw_true_north(float *yaw_rad)
{
    if (yaw_rad == NULL) return 0u;

    if (s_rtk_state.yaw_valid == 0u)
    {
        *yaw_rad = 0.0f;
        return 0u;
    }

    *yaw_rad = s_rtk_state.yaw_true_north;
    return 1u;
}

//--------------------------------------------------------------------------------------------------
//  @brief      手动设置原点
//--------------------------------------------------------------------------------------------------
void rtk_set_origin_e9(int64_t lat_deg_e9, int64_t lon_deg_e9, float alt_m)
{
    s_rtk_origin.lat_deg_e9 = lat_deg_e9;
    s_rtk_origin.lon_deg_e9 = lon_deg_e9;
    s_rtk_origin.alt_m      = alt_m;
    s_rtk_origin.is_set     = 1u;
}

//--------------------------------------------------------------------------------------------------
//  @brief      原点是否已设置
//--------------------------------------------------------------------------------------------------
uint8_t rtk_is_origin_set(void)
{
    return s_rtk_origin.is_set;
}

//--------------------------------------------------------------------------------------------------
//  @brief      获取原点采样进度
//  @return     0~100, 100表示原点已设置
//--------------------------------------------------------------------------------------------------
uint8_t rtk_get_origin_sample_progress(void)
{
    if (s_rtk_origin.is_set != 0u)
    {
        return 100u;
    }
    return (uint8_t)((uint32_t)s_sample_count * 100u / RTK_ORIGIN_SAMPLE_COUNT);
}

//--------------------------------------------------------------------------------------------------
//  @brief      设置天线安装偏角（运行时覆盖编译时默认值）
//  @param      offset_deg  偏角（度），正=顺时针
//--------------------------------------------------------------------------------------------------
void rtk_set_mount_offset_deg(float offset_deg)
{
    s_mount_offset_deg = offset_deg;
}

//--------------------------------------------------------------------------------------------------
//  @brief      获取当前天线安装偏角（度）
//  @return     偏角（度）
//--------------------------------------------------------------------------------------------------
float rtk_get_mount_offset_deg(void)
{
    return s_mount_offset_deg;
}

//--------------------------------------------------------------------------------------------------
//  @brief      获取参考航向（度）= yaw_ins(度) - mount_offset(度)
//  @return     参考航向（度），归一化到 [-180, 180]
//  @note       用于 yawrtk 帧中的 rtk_yaw_ref 字段
//--------------------------------------------------------------------------------------------------
float rtk_get_yaw_ref_deg(void)
{
    float yaw_ins_deg = s_rtk_state.yaw_ins * RTK_RAD_TO_DEG;
    float ref = yaw_ins_deg - s_mount_offset_deg;
    // 归一化到 [-180, 180]
    while (ref >  180.0f) ref -= 360.0f;
    while (ref < -180.0f) ref += 360.0f;
    return ref;
}
