/*
 * mag_yaw_lut.h
 * 磁航向查表补偿模块头文件
 *
 * 功能：对磁力计原始航向进行非线性误差修正
 * 流程：mag_yaw_raw -> LUT修正 -> mag_yaw_corr_raw
 *
 * LUT 存储"需要加到磁 yaw 上的修正量"：
 *   mag_yaw_corr = normalize(mag_yaw + lut[mag_yaw])
 *
 * 应用位置：在磁航向原始计算之后、减初始磁航向之前
 */

#ifndef CODE_INS_MAG_YAW_LUT_H_
#define CODE_INS_MAG_YAW_LUT_H_

#include <stdint.h>

 //-------------------------------------------宏定义----------------------------------------------------------------
#define MAG_YAW_LUT_SIZE    360       // LUT 表项数，每度一项

 //-------------------------------------------函数声明---------------------------------------------------------------

/**
 * @brief  初始化磁航向 LUT 模块
 * @note   默认 LUT 全零（无修正），LUT 默认禁用
 */
void mag_yaw_lut_init(void);

/**
 * @brief  加载 LUT 数据
 * @param  lut_deg  指向 360 项 float 数组，每项为该角度的修正量（度）
 * @note   会将数据拷贝到内部缓冲区，调用者无需保持数组生命周期
 *         加载后 LUT 仍处于禁用状态，需调用 mag_yaw_lut_enable() 启用
 */
void mag_yaw_lut_load(const float lut_deg[MAG_YAW_LUT_SIZE]);

/**
 * @brief  启用/禁用 LUT 修正
 * @param  enable  1=启用，0=禁用
 * @note   禁用时 apply 函数直接返回输入值
 */
void mag_yaw_lut_set_enabled(uint8_t enable);

/**
 * @brief  查询 LUT 是否已启用
 * @return 1=已启用，0=已禁用
 */
uint8_t mag_yaw_lut_is_enabled(void);

/**
 * @brief  对磁航向应用 LUT 修正（度制）
 * @param  yaw_deg  输入磁航向（度），范围不限，函数内部归一化
 * @return 修正后的磁航向（度），范围 [-180, 180]
 * @note   使用线性插值，处理环形边界（359° 与 0° 相邻）
 *         LUT 禁用时直接返回输入值
 */
float mag_yaw_lut_apply_deg(float yaw_deg);

/**
 * @brief  对磁航向应用 LUT 修正（弧度制）
 * @param  yaw_rad  输入磁航向（弧度）
 * @return 修正后的磁航向（弧度），范围 [-PI, PI]
 */
float mag_yaw_lut_apply_rad(float yaw_rad);

/**
 * @brief  获取内部 LUT 数据指针（只读）
 * @return 指向内部 360 项 float 数组的指针，或 NULL（未加载时）
 */
const float* mag_yaw_lut_get_data(void);

#endif /* CODE_INS_MAG_YAW_LUT_H_ */
