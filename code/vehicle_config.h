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
#define VEHICLE_WHEELBASE_M         0.777f    // 单位：米

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
 * 说明：对应转向机构的物理极限行程（编码器 raw 2139→3200 左 / 2139→875 右）
 */
#define VEHICLE_MAX_STEER_ANGLE_DEG  45.0f

/**
 * 循迹最大转向角（度）—— 算法保守限幅
 * 用途：Pure Pursuit 输出限幅
 * 说明：循迹模式下的保守上限，避免算法输出过大导致车辆失控
 */
#define VEHICLE_PP_MAX_STEER_DEG     40.0f

/**
 * 转向角死区（度）
 * 用途：小角度滤波，避免频繁调整
 */
#define VEHICLE_STEER_DEADZONE_DEG   2.0f

//==============================================================================
//                              运动约束参数
//==============================================================================

/**
 * 最大线速度（m/s）
 * 用途：速度限幅
 */
#define VEHICLE_MAX_SPEED_MPS        2.5f

/**
 * 最大角速度（rad/s）
 * 用途：角速度限幅
 */
#define VEHICLE_MAX_YAW_RATE_RPS     5.0f

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

/* 轮距别名（用于差速转向角速度估算） */
#ifndef COMPAT_WHEELBASE_M
#define COMPAT_WHEELBASE_M           VEHICLE_TRACK_WIDTH_M
#endif

#endif /* CODE_VEHICLE_CONFIG_H_ */
