/*
 * vehicle_config.h
 *
 * 车辆参数统一配置文件
 * 所有车辆相关的物理参数集中定义，避免重复定义和维护困难
 *
 * Created on: 2026-04-23
 *      Author: System
 */

#ifndef CODE_VEHICLE_CONFIG_H_
#define CODE_VEHICLE_CONFIG_H_

//==============================================================================
//                              车辆几何参数
//==============================================================================

/**
 * 车辆轴距（前后轮距离）
 * 用途：Pure Pursuit 转向计算、车辆运动模型
 * 说明：前轮转向轴中心到后轮轴中心的距离
 */
#define VEHICLE_WHEELBASE_M         0.78f     // 单位：米

/**
 * 车辆轮距（左右轮距离）
 * 用途：差速转向角速度估算、后轮差速控制
 * 说明：后轮左右轮中心之间的距离，用于差速辅助转向
 */
#define VEHICLE_TRACK_WIDTH_M       0.59f       // 单位：米（验证阶段暂用0.2，后续根据实测修改）

/**
 * 车轮直径
 * 用途：编码器脉冲转距离计算
 */
#define VEHICLE_WHEEL_DIAMETER_M    0.24f     // 单位：米

/**
 * 车轮周长
 * 用途：里程计计算
 */
#define VEHICLE_WHEEL_CIRCUMFERENCE_M  (VEHICLE_WHEEL_DIAMETER_M * 3.14159265358979f)

//==============================================================================
//                              转向系统参数
//==============================================================================

/**
 * 最大转向角（度）—— 物理极限
 * 用途：编码器映射、转向目标入口限幅、遥控映射
 * 说明：对应转向机构的物理极限行程（实测左右最大 25°）
 */
#define VEHICLE_MAX_STEER_ANGLE_DEG  25.0f

/**
 * 循迹最大转向角（度）—— 算法限幅（含差速助力等效扩展）
 * 用途：Pure Pursuit 输出限幅
 * 说明：配合后轮差速助力，PP可输出超过机械极限的角度
 *       PP输出30° → 前轮实际25°（机械极限）→ 误差5° → 差速满输出2脉冲
 *       等效转向能力扩展至约40°+，最小转弯半径从1.67m降至约1.2m
 *       不建议超过30°：差速已饱和(MAX_DELTA=2)，再大只增加转向电机负担
 */
#define VEHICLE_PP_MAX_STEER_DEG     30.0f

/**
 * 转向角死区（度）
 * 用途：小角度滤波，避免频繁调整
 */
#define VEHICLE_STEER_DEADZONE_DEG   1.0f

//==============================================================================
//                              运动约束参数
//==============================================================================



//==============================================================================
//                              编码器参数
//==============================================================================

/**
 * 编码器每转脉冲数
 * 用途：编码器脉冲转距离计算
 */
#define ENCODER_PULSES_PER_REVOLUTION  2048

/**
 * 编码器脉冲转距离系数（默认值）
 * 公式：PI * WHEEL_DIAMETER / PULSES_PER_REVOLUTION
 */
#define ENCODER_TICK_TO_METER_DEFAULT  (VEHICLE_WHEEL_CIRCUMFERENCE_M / ENCODER_PULSES_PER_REVOLUTION)

//==============================================================================
//                              兼容性宏定义
//==============================================================================

/* 为兼容旧代码，提供以下别名宏 */

/* 轴距别名（用于Pure Pursuit） */
#ifndef TRACK_WHEELBASE
#define TRACK_WHEELBASE              VEHICLE_WHEELBASE_M
#endif

/* 轴距别名（用于INS） */
#ifndef INS_WHEELBASE_M
#define INS_WHEELBASE_M              VEHICLE_WHEELBASE_M
#endif

/* INS车辆yaw模型修正系数
 * 说明：保留真实轴距不变，仅补偿 v*tan(steer)/wheelbase 与实车yaw响应的比例误差。
 * steer左负右正，左/右分开标定。
 */
#ifndef INS_STEER_MODEL_LEFT_SCALE
#define INS_STEER_MODEL_LEFT_SCALE   1.0000f
#endif

#ifndef INS_STEER_MODEL_RIGHT_SCALE
#define INS_STEER_MODEL_RIGHT_SCALE  1.0000f
#endif

#ifndef INS_STEER_MODEL_DEADBAND_DEG
#define INS_STEER_MODEL_DEADBAND_DEG 0.50f
#endif

/* 轮距别名（用于差速转向角速度估算） */
#ifndef COMPAT_WHEELBASE_M
#define COMPAT_WHEELBASE_M           VEHICLE_TRACK_WIDTH_M
#endif

#endif /* CODE_VEHICLE_CONFIG_H_ */
