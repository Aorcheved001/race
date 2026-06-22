/*
 * mag_yaw_lut.c
 * 磁航向查表补偿模块实现
 *
 * LUT 修正公式：mag_yaw_corr = normalize(mag_yaw + lut[mag_yaw])
 * 查表方式：线性插值，环形边界处理
 */

#include "zf_common_headfile.h"
#include "mag_yaw_lut.h"
#include "Ins.h"

 //-------------------------------------------模块静态变量------------------------------------------------------------
static float  s_lut[MAG_YAW_LUT_SIZE] = {0.0f};    // LUT 数据（度制修正量）
static uint8_t s_lut_loaded  = 0u;                   // LUT 是否已加载
static uint8_t s_lut_enabled = 0u;                   // LUT 是否已启用

 //-------------------------------------------内部工具函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      将角度归一化到 [-180, 180]（度制）
//  @param      angle_deg   输入角度（度）
//  @return     归一化后的角度（度）
//-------------------------------------------------------------------------------------------------------------------
static float normalize_angle_deg(float angle_deg)
{
    while(angle_deg > 180.0f)  angle_deg -= 360.0f;
    while(angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

 //-------------------------------------------对外接口------------------------------------------------------------

void mag_yaw_lut_init(void)
{
    uint16_t i;
    for(i = 0; i < MAG_YAW_LUT_SIZE; i++)
    {
        s_lut[i] = 0.0f;
    }
    s_lut_loaded  = 0u;
    s_lut_enabled = 0u;
}

void mag_yaw_lut_load(const float lut_deg[MAG_YAW_LUT_SIZE])
{
    uint16_t i;
    if(lut_deg == NULL)
    {
        return;
    }
    for(i = 0; i < MAG_YAW_LUT_SIZE; i++)
    {
        s_lut[i] = lut_deg[i];
    }
    s_lut_loaded = 1u;
}

void mag_yaw_lut_set_enabled(uint8_t enable)
{
    // 只有加载了 LUT 数据后才能启用
    if(enable != 0u && s_lut_loaded != 0u)
    {
        s_lut_enabled = 1u;
    }
    else
    {
        s_lut_enabled = 0u;
    }
}

uint8_t mag_yaw_lut_is_enabled(void)
{
    return s_lut_enabled;
}

float mag_yaw_lut_apply_deg(float yaw_deg)
{
    float yaw_norm;
    float yaw_pos;       // [0, 360)
    float idx_f;
    int   idx0;
    int   idx1;
    float frac;
    float corr0, corr1;
    float corr;

    // LUT 未启用或未加载，直接返回
    if(s_lut_enabled == 0u)
    {
        return normalize_angle_deg(yaw_deg);
    }

    // 归一化到 [0, 360) 用于查表
    yaw_norm = normalize_angle_deg(yaw_deg);
    yaw_pos  = yaw_norm;
    if(yaw_pos < 0.0f)
    {
        yaw_pos += 360.0f;
    }

    // 线性插值查表
    idx_f = yaw_pos;                    // 0 ~ 359.999...
    idx0  = (int)idx_f;                 // 整数部分
    frac  = idx_f - (float)idx0;        // 小数部分

    // 环形边界：359° 的下一个是 0°
    idx1 = idx0 + 1;
    if(idx1 >= MAG_YAW_LUT_SIZE)
    {
        idx1 = 0;
    }

    // 安全检查
    if(idx0 < 0)  idx0 = 0;
    if(idx0 >= MAG_YAW_LUT_SIZE) idx0 = MAG_YAW_LUT_SIZE - 1;

    corr0 = s_lut[idx0];
    corr1 = s_lut[idx1];
    corr  = corr0 * (1.0f - frac) + corr1 * frac;

    // 应用修正：mag_yaw_corr = normalize(mag_yaw + lut[mag_yaw])
    return normalize_angle_deg(yaw_deg + corr);
}

float mag_yaw_lut_apply_rad(float yaw_rad)
{
    float yaw_deg   = yaw_rad * INS_RAD2DEG;
    float corr_deg  = mag_yaw_lut_apply_deg(yaw_deg);
    return corr_deg * INS_DEG2RAD;
}

const float* mag_yaw_lut_get_data(void)
{
    if(s_lut_loaded != 0u)
    {
        return s_lut;
    }
    return NULL;
}
