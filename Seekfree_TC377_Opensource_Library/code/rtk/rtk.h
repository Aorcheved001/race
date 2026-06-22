/*
 * rtk.h
 *
 *  Created on: 2026-06-17
 *      Author: RTK坐标转换层
 *
 *  功能：将GN43RFA RTK模块输出的经纬度转换为局部平面坐标(ENU)，
 *        并对齐到INS相对坐标系，提供双天线真北航向用于坐标系旋转。
 *
 *  坐标系说明：
 *    ENU坐标系：x=东，y=北，yaw=0指向真北(顺时针为正)
 *    INS坐标系：x=前进，y=左，yaw=0指向启动方向(逆时针为正)
 *
 *  对齐变换（ENU → INS）：
 *    设对齐时刻 RTK 真北航向为 θ?（antenna_direction 转弧度）
 *    dx = x_enu - x_enu?,  dy = y_enu - y_enu?
 *    x_ins =  sin(θ?)·dx + cos(θ?)·dy
 *    y_ins = -cos(θ?)·dx + sin(θ?)·dy
 *    yaw_ins = θ? - yaw_true_north
 *
 *  验证：
 *    车朝北(θ?=0)：前进→dy↑→x_ins↑?  右移→dx↑→y_ins↓?
 *    车朝东(θ?=π/2)：前进→dx↑→x_ins↑?  左移→dy↑→y_ins↑?
 */

#ifndef CODE_RTK_RTK_H_
#define CODE_RTK_RTK_H_

#include "zf_common_headfile.h"

 //-------------------------------------------宏定义区------------------------------------------------------------

#define RTK_WGS84_RE            6378137.0f                      // WGS84地球长半轴 (m)
#define RTK_DEG_TO_RAD          0.017453292519943295f           // 角度转弧度系数
#define RTK_RAD_TO_DEG          57.295779513082323f             // 弧度转角度系数
#define RTK_PI                  3.14159265358979323846f         // 圆周率

#define RTK_ORIGIN_SAMPLE_COUNT 50                              // 采集50个点用于计算原点
#define RTK_OUTLIER_THRESHOLD_M 1.5f                            // 剔除距离均值大于1.5米的离群点
#define RTK_MIN_SATS_FOR_ORIGIN 6                               // 计算原点时最少需要的卫星数
#define RTK_MIN_SATS_FOR_POS    6                               // 定位最少需要的卫星数
#define RTK_REQUIRE_RTK_FIX     1                               // 1=仅RTK Float/Fixed可用于轨迹参考，0=允许单点/DGPS

// 天线安装偏角标定 (度)
// MAIN天线在车头，AUX天线在车尾，基准修正 = 180° (MAIN→AUX基线指向车尾)
// 若天线不完全平行于车体纵轴，标定此偏角:
//   车辆航向 = antenna_direction + 180° + ANTENNA_MOUNT_OFFSET
// 标定方法: 车直线前进时，RTK航向与INS航向的差值即为偏角
#define RTK_ANTENNA_MOUNT_OFFSET_DEG    0.0f                    // 天线安装偏角 (度)，正=顺时针

 //-------------------------------------------结构体区------------------------------------------------------------

typedef struct
{
    double  lat_deg;        // 纬度 (deg, WGS84) - 必须用double，float精度不够导致ENU差值抵消
    double  lon_deg;        // 经度 (deg, WGS84) - 必须用double，float精度不够导致ENU差值抵消
    float   alt_m;          // 海拔高度 (m)

    // ENU局部坐标（相对原点，y=真北）
    float   x_enu;          // 东向位移 (m)
    float   y_enu;          // 北向位移 (m)

    // INS对齐坐标（RTK坐标旋转到INS相对坐标系）
    float   x_ins;          // INS坐标系下的x (m)，x=前进方向
    float   y_ins;          // INS坐标系下的y (m)，y=左侧方向
    float   yaw_ins;        // INS坐标系下的yaw (rad)，0=启动方向，CCW为正
    uint8_t is_aligned;     // 是否已完成INS对齐

    // 双天线真北航向
    float   yaw_true_north; // 真北航向 (rad)，0=北，顺时针为正，范围[-π, π]
    uint8_t yaw_valid;      // 真北航向是否有效

    // 定位状态
    uint8_t fix_state;      // 0=无效 1=单点 2=差分/RTK
    uint8_t num_sats;       // 可用卫星数量
    uint8_t pos_valid;      // 位置是否有效（原点已建 + 解算可用）
} rtk_state_t;

typedef struct
{
    int64_t lat_deg_e9;     // 原点纬度，单位1e-9度，避免RTK小位移量化
    int64_t lon_deg_e9;     // 原点经度，单位1e-9度，避免RTK小位移量化
    float   alt_m;          // 原点高度 (m)
    uint8_t is_set;         // 原点是否已设置
} rtk_origin_t;

typedef struct
{
    int64_t lat_deg_e9;       // 样本纬度，单位1e-9度
    int64_t lon_deg_e9;       // 样本经度，单位1e-9度
    float   alt_m;            // 样本高度（米）
} rtk_sample_t;

 //-------------------------------------------函数声明区------------------------------------------------------------

void        rtk_init(void);                                     // 初始化RTK模块（GN43RFA）
void        rtk_update(void);                                   // 在主循环中调用，解析GNSS数据并更新状态
void        rtk_reset_alignment(void);                          // 重置ENU到INS的对齐参考点，下一帧有效RTK重新对齐

const rtk_state_t* rtk_get_state(void);                         // 获取RTK状态（只读）
uint8_t     rtk_get_position_xy(float *x_enu, float *y_enu);   // 获取ENU坐标
uint8_t     rtk_get_yaw_true_north(float *yaw_rad);             // 获取真北航向

void        rtk_set_origin_e9(int64_t lat_deg_e9, int64_t lon_deg_e9, float alt_m); // 手动设置高精度整数原点
uint8_t     rtk_is_origin_set(void);                            // 原点是否已设置
uint8_t     rtk_get_origin_sample_progress(void);              // 原点采样进度(0~100)

// RTK 安装偏角运行时管理
void        rtk_set_mount_offset_deg(float offset_deg);        // 设置天线安装偏角（度），覆盖编译时默认值
float       rtk_get_mount_offset_deg(void);                    // 获取当前天线安装偏角（度）
float       rtk_get_yaw_ref_deg(void);                         // 获取参考航向（度）= yaw_ins - mount_offset

#endif /* CODE_RTK_RTK_H_ */
