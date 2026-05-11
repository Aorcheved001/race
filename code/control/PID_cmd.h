/*
 * PID_cmd.h
 *
 *  PID 指令解析与调试系统
 *  支持实时参数调整、结构化数据输出、自动测试
 *
 *  Created on: 2026-04-27
 *      Author: Auto-Generated
 */

#ifndef CODE_CONTROL_PID_CMD_H_
#define CODE_CONTROL_PID_CMD_H_

//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------宏定义-----------------------------------------------------------------

/* 指令缓冲区大小 */
#define PID_CMD_BUFFER_SIZE         128

/* 遥测默认周期 (ms) */
#define PID_CMD_TELEMETRY_DEFAULT_MS    50

/* 遥测缓冲区大小 */
#define PID_CMD_TELEMETRY_BUF_SIZE      256

/* 测试模式枚举 */
typedef enum
{
    PID_TEST_MODE_NONE = 0,         // 无测试
    PID_TEST_MODE_STEP,             // 阶跃响应测试
} pid_test_mode_enum;

//-------------------------------------------结构体定义-------------------------------------------------------------

/**
 * PID指令系统状态结构体
 * 管理指令解析、遥测输出、测试模式等状态
 */
typedef struct
{
    /*--------- 指令解析 --------*/
    char cmd_buffer[PID_CMD_BUFFER_SIZE];       // 指令接收缓冲区
    uint16 cmd_index;                           // 缓冲区指针
    uint8 cmd_ready;                            // 指令就绪标志

    /*--------- 遥测输出 --------*/
    uint16 telemetry_rate_ms;                   // 遥测输出周期 (ms)
    uint16 telemetry_counter;                   // 遥测计时器
    uint8 telemetry_enabled;                    // 遥测使能标志

    /*--------- 测试模式 --------*/
    pid_test_mode_enum test_mode;               // 当前测试模式
    uint32 test_start_time_ms;                  // 测试开始时间
    uint32 test_duration_ms;                    // 测试持续时间
    float test_target_speed;                    // 测试目标速度

    /*--------- 时间戳 --------*/
    uint32 timestamp_ms;                        // 系统时间戳 (ms)

} PID_CMD_STATE;

//-------------------------------------------外部变量声明-----------------------------------------------------------
extern PID_CMD_STATE g_pid_cmd;

//-------------------------------------------函数声明--------------------------------------------------------------

/* 初始化与处理 */
void wheel_pid_cmd_init(void);                                  // 初始化指令系统
void wheel_pid_cmd_process_byte(uint8 byte);                    // 处理单个接收字节
void wheel_pid_cmd_poll(void);                                  // 轮询处理（在定时任务中调用）

/* 遥测输出 */
void wheel_pid_cmd_send_telemetry(void);                        // 发送遥测数据

/* 测试模式 */
void wheel_pid_cmd_start_step_test(float speed, uint32 duration_ms);   // 开始阶跃测试
void wheel_pid_cmd_stop_test(void);                             // 停止测试

/* 指令执行 */
void wheel_pid_cmd_execute(const char *cmd);                    // 执行指令字符串

/* 辅助函数 */
void wheel_pid_cmd_send_string(const char *str);                // 发送字符串
void wheel_pid_cmd_send_status(void);                           // 发送状态信息

#endif /* CODE_CONTROL_PID_CMD_H_ */
