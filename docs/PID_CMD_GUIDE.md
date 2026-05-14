# PID 串口命令与遥测系统

> 版本: 1.0.0  
> 创建日期: 2026-04-27  
> 适用平台: TC377 + ZF框架

---

## 一、概述

本文档描述了 PID 串口命令解析与结构化遥测系统的实现方案。通过串口命令可以实时修改 PID 参数、触发测试、获取状态，无需重新编译烧录。

### 功能特性

- ✅ 实时修改速度环/位置环/角速度环 PID 参数
- ✅ 结构化遥测数据输出（CSV格式）
- ✅ 阶跃响应测试模式
- ✅ 可配置遥测输出频率
- ✅ 查询当前参数和状态

---

## 二、PID_cmd.h

```c
/*
 * PID_cmd.h
 *
 *  PID 串口命令解析与遥测系统
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

/* 命令缓冲区大小 */
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
 * PID命令系统状态结构体
 * 管理命令解析、遥测输出、测试模式等状态
 */
typedef struct
{
    /*--------- 命令解析 --------*/
    char cmd_buffer[PID_CMD_BUFFER_SIZE];       // 命令接收缓冲区
    uint16 cmd_index;                           // 缓冲区索引
    uint8 cmd_ready;                            // 命令就绪标志

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
void wheel_pid_cmd_init(void);                                  // 初始化命令系统
void wheel_pid_cmd_process_byte(uint8 byte);                    // 处理单个接收字节
void wheel_pid_cmd_poll(void);                                  // 轮询处理（在定时任务中调用）

/* 遥测输出 */
void wheel_pid_cmd_send_telemetry(void);                        // 发送遥测数据

/* 测试模式 */
void wheel_pid_cmd_start_step_test(float speed, uint32 duration_ms);   // 开始阶跃测试
void wheel_pid_cmd_stop_test(void);                             // 停止测试

/* 命令执行 */
void wheel_pid_cmd_execute(const char *cmd);                    // 执行命令字符串

/* 辅助函数 */
void wheel_pid_cmd_send_string(const char *str);                // 发送字符串
void wheel_pid_cmd_send_status(void);                           // 发送状态信息

#endif /* CODE_CONTROL_PID_CMD_H_ */
```

---

## 三、PID_cmd.c

```c
/*
 * PID_cmd.c
 *
 *  PID 串口命令解析与遥测系统实现
 *  支持实时参数调整、结构化数据输出、自动测试
 *
 *  Created on: 2026-04-27
 *      Author: Auto-Generated
 */

//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"
#include "PID_cmd.h"
#include "PID.h"
#include "encoder.h"
#include "steering_control.h"

//-------------------------------------------全局变量--------------------------------------------------------------

/* PID命令系统全局状态 */
PID_CMD_STATE g_pid_cmd;

/* 保存测试前的遥测设置，用于恢复 */
static uint16 s_saved_telemetry_rate_ms = 50;
static uint8 s_saved_telemetry_enabled = 0u;

//-------------------------------------------内部函数声明------------------------------------------------------------

static void cmd_send_help(void);
static void cmd_send_status(void);
static void cmd_set_speed_param(int argc, char *argv[]);
static void cmd_set_position_param(int argc, char *argv[]);
static void cmd_set_yaw_param(int argc, char *argv[]);
static void cmd_set_speed_target(int argc, char *argv[]);
static void cmd_set_yaw_target(int argc, char *argv[]);
static void cmd_set_telemetry_rate(int argc, char *argv[]);
static void cmd_start_step_test(int argc, char *argv[]);
static void cmd_stop_test(void);
static void cmd_enable_loops(int argc, char *argv[]);
static float parse_float(const char *str, float default_val);
static int parse_int(const char *str, int default_val);
static int strcmp_nocase(const char *s1, const char *s2);

//-------------------------------------------辅助函数实现-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_send_string
// 功能说明     通过无线串口发送字符串（自动添加 \r\n）
// 参数说明     str         - 要发送的字符串
// 返回参数     void
// 使用示例     wheel_pid_cmd_send_string("OK");
// 备注信息     封装 wireless_uart_send_string，统一输出接口
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_send_string(const char *str)
{
    if (str == NULL) return;
    
    wireless_uart_send_string(str);
    wireless_uart_send_string("\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     parse_float
// 功能说明     安全解析浮点数
// 参数说明     str         - 字符串
// 参数说明     default_val - 解析失败时的默认值
// 返回参数     float       - 解析结果
//-------------------------------------------------------------------------------------------------------------------
static float parse_float(const char *str, float default_val)
{
    if (str == NULL || str[0] == '\0') return default_val;
    
    float result = default_val;
    int sign = 1;
    int i = 0;
    float decimal = 0.1f;
    
    // 跳过前导空格
    while (str[i] == ' ') i++;
    
    // 处理符号
    if (str[i] == '-') { sign = -1; i++; }
    else if (str[i] == '+') { i++; }
    
    // 整数部分
    result = 0.0f;
    while (str[i] >= '0' && str[i] <= '9')
    {
        result = result * 10.0f + (str[i] - '0');
        i++;
    }
    
    // 小数部分
    if (str[i] == '.')
    {
        i++;
        float frac = 0.0f;
        float div = 10.0f;
        while (str[i] >= '0' && str[i] <= '9')
        {
            frac += (str[i] - '0') / div;
            div *= 10.0f;
            i++;
        }
        result += frac;
    }
    
    return result * sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     parse_int
// 功能说明     安全解析整数
// 参数说明     str         - 字符串
// 参数说明     default_val - 解析失败时的默认值
// 返回参数     int         - 解析结果
//-------------------------------------------------------------------------------------------------------------------
static int parse_int(const char *str, int default_val)
{
    if (str == NULL || str[0] == '\0') return default_val;
    
    int result = 0;
    int sign = 1;
    int i = 0;
    
    while (str[i] == ' ') i++;
    
    if (str[i] == '-') { sign = -1; i++; }
    else if (str[i] == '+') { i++; }
    
    while (str[i] >= '0' && str[i] <= '9')
    {
        result = result * 10 + (str[i] - '0');
        i++;
    }
    
    return result * sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     strcmp_nocase
// 功能说明     不区分大小写的字符串比较
// 参数说明     s1, s2      - 待比较字符串
// 返回参数     int         - 0表示相等
//-------------------------------------------------------------------------------------------------------------------
static int strcmp_nocase(const char *s1, const char *s2)
{
    if (s1 == NULL || s2 == NULL) return -1;
    
    while (*s1 && *s2)
    {
        char c1 = *s1;
        char c2 = *s2;
        
        // 转小写
        if (c1 >= 'A' && c1 <= 'Z') c1 += 32;
        if (c2 >= 'A' && c2 <= 'Z') c2 += 32;
        
        if (c1 != c2) return c1 - c2;
        
        s1++;
        s2++;
    }
    
    return *s1 - *s2;
}

//-------------------------------------------初始化函数--------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_init
// 功能说明     初始化 PID 命令系统
// 参数说明     void
// 返回参数     void
// 使用示例     在 wheel_pid_init() 中调用
// 备注信息     初始化命令缓冲区、遥测参数、测试状态
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_init(void)
{
    memset(&g_pid_cmd, 0, sizeof(g_pid_cmd));
    
    g_pid_cmd.telemetry_rate_ms = PID_CMD_TELEMETRY_DEFAULT_MS;
    g_pid_cmd.telemetry_enabled = 0u;           // 默认关闭遥测
    g_pid_cmd.test_mode = PID_TEST_MODE_NONE;
    g_pid_cmd.cmd_ready = 0u;
    g_pid_cmd.cmd_index = 0u;
    g_pid_cmd.timestamp_ms = 0u;
}

//-------------------------------------------命令解析函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_process_byte
// 功能说明     处理单个接收字节（在串口回调中调用）
// 参数说明     byte        - 接收到的字节
// 返回参数     void
// 使用示例     wheel_pid_cmd_process_byte(data);
// 备注信息     累积字节直到收到完整命令（以 \r 或 \n 结尾）
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_process_byte(uint8 byte)
{
    // 忽略空字节
    if (byte == 0) return;
    
    // 检测命令结束符
    if (byte == '\r' || byte == '\n')
    {
        // 如果缓冲区有内容，标记命令就绪
        if (g_pid_cmd.cmd_index > 0)
        {
            g_pid_cmd.cmd_buffer[g_pid_cmd.cmd_index] = '\0';
            g_pid_cmd.cmd_ready = 1u;
        }
        return;
    }
    
    // 检测缓冲区溢出
    if (g_pid_cmd.cmd_index >= PID_CMD_BUFFER_SIZE - 1)
    {
        g_pid_cmd.cmd_index = 0u;   // 丢弃溢出的命令
        return;
    }
    
    // 累积字节
    g_pid_cmd.cmd_buffer[g_pid_cmd.cmd_index++] = (char)byte;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_poll
// 功能说明     轮询处理命令和遥测（在定时任务中调用）
// 参数说明     void
// 返回参数     void
// 使用示例     在 run_4ms_tasks() 中调用
// 备注信息     处理待执行命令、更新遥测输出、管理测试模式
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_poll(void)
{
    // 更新时间戳
    g_pid_cmd.timestamp_ms += 4u;   // 假设每4ms调用一次
    
    // 处理待执行命令
    if (g_pid_cmd.cmd_ready)
    {
        wheel_pid_cmd_execute(g_pid_cmd.cmd_buffer);
        g_pid_cmd.cmd_ready = 0u;
        g_pid_cmd.cmd_index = 0u;
    }
    
    // 遥测输出
    if (g_pid_cmd.telemetry_enabled)
    {
        g_pid_cmd.telemetry_counter += 4u;
        if (g_pid_cmd.telemetry_counter >= g_pid_cmd.telemetry_rate_ms)
        {
            g_pid_cmd.telemetry_counter = 0u;
            wheel_pid_cmd_send_telemetry();
        }
    }
    
    // 测试模式管理
    if (g_pid_cmd.test_mode == PID_TEST_MODE_STEP)
    {
        uint32 elapsed = g_pid_cmd.timestamp_ms - g_pid_cmd.test_start_time_ms;
        if (elapsed >= g_pid_cmd.test_duration_ms)
        {
            wheel_pid_cmd_stop_test();
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_execute
// 功能说明     执行命令字符串
// 参数说明     cmd         - 命令字符串
// 返回参数     void
// 使用示例     wheel_pid_cmd_execute("SET_SPEED_PARAM 0 50 0 100");
// 备注信息     解析命令类型和参数，调用对应的处理函数
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_execute(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') return;
    
    // 参数解析（最多8个参数）
    char *argv[8];
    int argc = 0;
    char buffer[PID_CMD_BUFFER_SIZE];
    
    // 复制命令到缓冲区（避免修改原字符串）
    uint16 i = 0;
    while (cmd[i] && i < PID_CMD_BUFFER_SIZE - 1)
    {
        buffer[i] = cmd[i];
        i++;
    }
    buffer[i] = '\0';
    
    // 分割参数
    argv[argc] = buffer;
    for (i = 0; buffer[i] && argc < 8; i++)
    {
        if (buffer[i] == ' ' || buffer[i] == '\t')
        {
            buffer[i] = '\0';
            if (buffer[i + 1] != '\0')
            {
                argv[++argc] = &buffer[i + 1];
            }
        }
    }
    argc++;
    
    // 命令匹配与执行
    if (strcmp_nocase(argv[0], "HELP") == 0)
    {
        cmd_send_help();
    }
    else if (strcmp_nocase(argv[0], "GET_STATUS") == 0)
    {
        cmd_send_status();
    }
    else if (strcmp_nocase(argv[0], "SET_SPEED_PARAM") == 0)
    {
        cmd_set_speed_param(argc, argv);
    }
    else if (strcmp_nocase(argv[0], "SET_POSITION_PARAM") == 0)
    {
        cmd_set_position_param(argc, argv);
    }
    else if (strcmp_nocase(argv[0], "SET_YAW_PARAM") == 0)
    {
        cmd_set_yaw_param(argc, argv);
    }
    else if (strcmp_nocase(argv[0], "SET_SPEED_TARGET") == 0)
    {
        cmd_set_speed_target(argc, argv);
    }
    else if (strcmp_nocase(argv[0], "SET_YAW_TARGET") == 0)
    {
        cmd_set_yaw_target(argc, argv);
    }
    else if (strcmp_nocase(argv[0], "SET_TELEMETRY_RATE") == 0)
    {
        cmd_set_telemetry_rate(argc, argv);
    }
    else if (strcmp_nocase(argv[0], "START_STEP_TEST") == 0)
    {
        cmd_start_step_test(argc, argv);
    }
    else if (strcmp_nocase(argv[0], "STOP_TEST") == 0)
    {
        cmd_stop_test();
    }
    else if (strcmp_nocase(argv[0], "ENABLE_LOOPS") == 0)
    {
        cmd_enable_loops(argc, argv);
    }
    else
    {
        wheel_pid_cmd_send_string("ERR:Unknown command. Send HELP for list.");
    }
}

//-------------------------------------------命令处理函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_send_help
// 功能说明     发送帮助信息
//-------------------------------------------------------------------------------------------------------------------
static void cmd_send_help(void)
{
    wheel_pid_cmd_send_string("=== PID Command Help ===");
    wheel_pid_cmd_send_string("SET_SPEED_PARAM kp ki kd i_limit");
    wheel_pid_cmd_send_string("SET_POSITION_PARAM kp ki kd i_limit");
    wheel_pid_cmd_send_string("SET_YAW_PARAM kp ki kd i_limit");
    wheel_pid_cmd_send_string("SET_SPEED_TARGET left_mps right_mps");
    wheel_pid_cmd_send_string("SET_YAW_TARGET yaw_rps");
    wheel_pid_cmd_send_string("SET_TELEMETRY_RATE ms (0=off)");
    wheel_pid_cmd_send_string("START_STEP_TEST speed_mps duration_ms");
    wheel_pid_cmd_send_string("STOP_TEST");
    wheel_pid_cmd_send_string("ENABLE_LOOPS out spd pos yaw");
    wheel_pid_cmd_send_string("GET_STATUS");
    wheel_pid_cmd_send_string("HELP");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_send_status
// 功能说明     发送当前状态信息
//-------------------------------------------------------------------------------------------------------------------
static void cmd_send_status(void)
{
    char buf[128];
    
    wheel_pid_cmd_send_string("=== PID Status ===");
    
    // 速度环参数
    sprintf(buf, "SPEED_PARAM: Kp=%.2f Ki=%.2f Kd=%.2f I_limit=%.2f",
            g_wheel_pid.speed_param[0], g_wheel_pid.speed_param[1],
            g_wheel_pid.speed_param[2], g_wheel_pid.speed_param[3]);
    wheel_pid_cmd_send_string(buf);
    
    // 位置环参数
    sprintf(buf, "POSITION_PARAM: Kp=%.2f Ki=%.2f Kd=%.2f I_limit=%.2f",
            g_wheel_pid.position_param[0], g_wheel_pid.position_param[1],
            g_wheel_pid.position_param[2], g_wheel_pid.position_param[3]);
    wheel_pid_cmd_send_string(buf);
    
    // 角速度环参数
    sprintf(buf, "YAW_PARAM: Kp=%.2f Ki=%.2f Kd=%.2f I_limit=%.2f",
            g_wheel_pid.yaw_rate_param[0], g_wheel_pid.yaw_rate_param[1],
            g_wheel_pid.yaw_rate_param[2], g_wheel_pid.yaw_rate_param[3]);
    wheel_pid_cmd_send_string(buf);
    
    // 使能状态
    sprintf(buf, "ENABLE: out=%d speed=%d pos=%d yaw=%d",
            g_wheel_pid.enable_output, g_wheel_pid.enable_speed_loop,
            g_wheel_pid.enable_position_loop, g_wheel_pid.enable_yaw_rate_loop);
    wheel_pid_cmd_send_string(buf);
    
    // 目标值
    sprintf(buf, "TARGET: speed_L=%.2f speed_R=%.2f yaw=%.2f",
            g_wheel_pid.cmd_speed_left_mps, g_wheel_pid.cmd_speed_right_mps,
            g_wheel_pid.cmd_yaw_rate_rps);
    wheel_pid_cmd_send_string(buf);
    
    // 遥测状态
    sprintf(buf, "TELEMETRY: rate=%dms enabled=%d",
            g_pid_cmd.telemetry_rate_ms, g_pid_cmd.telemetry_enabled);
    wheel_pid_cmd_send_string(buf);
    
    // 测试状态
    sprintf(buf, "TEST_MODE: %d", g_pid_cmd.test_mode);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_set_speed_param
// 功能说明     设置速度环 PID 参数
//-------------------------------------------------------------------------------------------------------------------
static void cmd_set_speed_param(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_SPEED_PARAM kp ki kd i_limit");
        return;
    }
    
    float kp = parse_float(argv[1], 0.0f);
    float ki = parse_float(argv[2], 0.0f);
    float kd = parse_float(argv[3], 0.0f);
    float i_limit = parse_float(argv[4], 100.0f);
    
    wheel_pid_set_speed_param(kp, ki, kd, i_limit);
    
    char buf[64];
    sprintf(buf, "OK:SPEED_PARAM kp=%.2f ki=%.2f kd=%.2f i_limit=%.2f", kp, ki, kd, i_limit);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_set_position_param
// 功能说明     设置位置环 PID 参数
//-------------------------------------------------------------------------------------------------------------------
static void cmd_set_position_param(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_POSITION_PARAM kp ki kd i_limit");
        return;
    }
    
    float kp = parse_float(argv[1], 0.0f);
    float ki = parse_float(argv[2], 0.0f);
    float kd = parse_float(argv[3], 0.0f);
    float i_limit = parse_float(argv[4], 2.5f);
    
    wheel_pid_set_position_param(kp, ki, kd, i_limit);
    
    char buf[64];
    sprintf(buf, "OK:POSITION_PARAM kp=%.2f ki=%.2f kd=%.2f i_limit=%.2f", kp, ki, kd, i_limit);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_set_yaw_param
// 功能说明     设置角速度环 PID 参数
//-------------------------------------------------------------------------------------------------------------------
static void cmd_set_yaw_param(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_YAW_PARAM kp ki kd i_limit");
        return;
    }
    
    float kp = parse_float(argv[1], 0.0f);
    float ki = parse_float(argv[2], 0.0f);
    float kd = parse_float(argv[3], 0.0f);
    float i_limit = parse_float(argv[4], 100.0f);
    
    wheel_pid_set_yaw_rate_param(kp, ki, kd, i_limit);
    
    char buf[64];
    sprintf(buf, "OK:YAW_PARAM kp=%.2f ki=%.2f kd=%.2f i_limit=%.2f", kp, ki, kd, i_limit);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_set_speed_target
// 功能说明     设置速度目标
//-------------------------------------------------------------------------------------------------------------------
static void cmd_set_speed_target(int argc, char *argv[])
{
    if (argc < 3)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_SPEED_TARGET left_mps right_mps");
        return;
    }
    
    float left = parse_float(argv[1], 0.0f);
    float right = parse_float(argv[2], 0.0f);
    
    wheel_pid_set_speed_target(left, right);
    
    char buf[64];
    sprintf(buf, "OK:SPEED_TARGET L=%.2f R=%.2f", left, right);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_set_yaw_target
// 功能说明     设置角速度目标
//-------------------------------------------------------------------------------------------------------------------
static void cmd_set_yaw_target(int argc, char *argv[])
{
    if (argc < 2)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_YAW_TARGET yaw_rps");
        return;
    }
    
    float yaw = parse_float(argv[1], 0.0f);
    
    wheel_pid_set_yaw_target(yaw);
    
    char buf[64];
    sprintf(buf, "OK:YAW_TARGET %.2f rad/s", yaw);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_set_telemetry_rate
// 功能说明     设置遥测输出周期
//-------------------------------------------------------------------------------------------------------------------
static void cmd_set_telemetry_rate(int argc, char *argv[])
{
    if (argc < 2)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_TELEMETRY_RATE ms (0=off)");
        return;
    }
    
    int rate_ms = parse_int(argv[1], 50);
    
    if (rate_ms <= 0)
    {
        g_pid_cmd.telemetry_enabled = 0u;
        g_pid_cmd.telemetry_rate_ms = 0;
        wheel_pid_cmd_send_string("OK:TELEMETRY OFF");
    }
    else
    {
        g_pid_cmd.telemetry_rate_ms = (uint16)rate_ms;
        g_pid_cmd.telemetry_enabled = 1u;
        g_pid_cmd.telemetry_counter = 0u;
        
        char buf[32];
        sprintf(buf, "OK:TELEMETRY_RATE %dms", rate_ms);
        wheel_pid_cmd_send_string(buf);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_start_step_test
// 功能说明     开始阶跃响应测试
//-------------------------------------------------------------------------------------------------------------------
static void cmd_start_step_test(int argc, char *argv[])
{
    if (argc < 3)
    {
        wheel_pid_cmd_send_string("ERR:Usage: START_STEP_TEST speed_mps duration_ms");
        return;
    }
    
    float speed = parse_float(argv[1], 1.0f);
    int duration = parse_int(argv[2], 3000);
    
    wheel_pid_cmd_start_step_test(speed, (uint32)duration);
    
    char buf[64];
    sprintf(buf, "OK:STEP_TEST speed=%.2f duration=%dms", speed, duration);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_stop_test
// 功能说明     停止测试
//-------------------------------------------------------------------------------------------------------------------
static void cmd_stop_test(void)
{
    wheel_pid_cmd_stop_test();
    wheel_pid_cmd_send_string("OK:TEST_STOPPED");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     cmd_enable_loops
// 功能说明     设置各环使能状态
//-------------------------------------------------------------------------------------------------------------------
static void cmd_enable_loops(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: ENABLE_LOOPS output speed pos yaw");
        return;
    }
    
    int out = parse_int(argv[1], 0);
    int spd = parse_int(argv[2], 0);
    int pos = parse_int(argv[3], 0);
    int yaw = parse_int(argv[4], 0);
    
    wheel_pid_enable((uint8)out, (uint8)spd, (uint8)pos, (uint8)yaw);
    
    char buf[64];
    sprintf(buf, "OK:ENABLE out=%d spd=%d pos=%d yaw=%d", out, spd, pos, yaw);
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------遥测函数----------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_send_telemetry
// 功能说明     发送遥测数据行（CSV格式）
// 参数说明     void
// 返回参数     void
// 使用示例     在遥测定时器中调用
// 备注信息     输出格式: TELEMETRY,timestamp_ms,speed_cmd_l,speed_cmd_r,speed_act_l,speed_act_r,...
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_send_telemetry(void)
{
    const EncoderLayerState *enc = encoder_layer_get_state();
    if (enc == NULL) return;
    
    char buf[PID_CMD_TELEMETRY_BUF_SIZE];
    
    // 计算误差
    float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
    float err_spd_r = g_wheel_pid.cmd_speed_right_mps - enc->speed_right_mps;
    float err_yaw = g_wheel_pid.cmd_yaw_rate_rps - g_wheel_pid.ref_yaw_rate_rps;
    
    // 格式化遥测数据
    // TELEMETRY,timestamp_ms,speed_cmd_l,speed_cmd_r,speed_act_l,speed_act_r,
    // yaw_cmd,yaw_act,out_pwm_l,out_pwm_r,out_steer,error_spd_l,error_spd_r,error_yaw,pos_l,pos_r
    sprintf(buf,
            "TELEMETRY,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f",
            (unsigned long)g_pid_cmd.timestamp_ms,
            g_wheel_pid.cmd_speed_left_mps,
            g_wheel_pid.cmd_speed_right_mps,
            enc->speed_left_mps,
            enc->speed_right_mps,
            g_wheel_pid.cmd_yaw_rate_rps,
            g_wheel_pid.ref_yaw_rate_rps,
            g_wheel_pid.out_left_pwm,
            g_wheel_pid.out_right_pwm,
            g_wheel_pid.out_steer_pwm,
            err_spd_l,
            err_spd_r,
            err_yaw,
            enc->odom_left_m,
            enc->odom_right_m);
    
    wheel_pid_cmd_send_string(buf);
}

//-------------------------------------------测试模式函数------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_start_step_test
// 功能说明     开始阶跃响应测试
// 参数说明     speed       - 目标速度 (m/s)
// 参数说明     duration_ms - 测试持续时间 (ms)
// 返回参数     void
// 使用示例     wheel_pid_cmd_start_step_test(1.0f, 3000);
// 备注信息     设置目标速度并启动遥测，测试结束后自动停止并恢复遥测设置
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_start_step_test(float speed, uint32 duration_ms)
{
    // 保存当前遥测设置
    s_saved_telemetry_rate_ms = g_pid_cmd.telemetry_rate_ms;
    s_saved_telemetry_enabled = g_pid_cmd.telemetry_enabled;

    // 重置编码器里程
    encoder_layer_clear_odom();

    // 设置目标速度（左右相同）
    wheel_pid_set_speed_target(speed, speed);

    // 确保速度环使能
    wheel_pid_enable(1, 1, 0, 0);

    // 启动遥测（测试时使用更高频率）
    g_pid_cmd.telemetry_enabled = 1u;
    g_pid_cmd.telemetry_rate_ms = 20;
    g_pid_cmd.telemetry_counter = 0u;

    // 设置测试状态
    g_pid_cmd.test_mode = PID_TEST_MODE_STEP;
    g_pid_cmd.test_start_time_ms = g_pid_cmd.timestamp_ms;
    g_pid_cmd.test_duration_ms = duration_ms;
    g_pid_cmd.test_target_speed = speed;

    // 发送测试开始标记
    wheel_pid_cmd_send_string("TEST_STEP_START");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     wheel_pid_cmd_stop_test
// 功能说明     停止当前测试
// 参数说明     void
// 返回参数     void
// 使用示例     wheel_pid_cmd_stop_test();
// 备注信息     停止电机、恢复遥测设置、发送测试结束标记
//-------------------------------------------------------------------------------------------------------------------
void wheel_pid_cmd_stop_test(void)
{
    // 停止电机
    wheel_pid_set_speed_target(0.0f, 0.0f);

    // 恢复遥测设置
    g_pid_cmd.telemetry_rate_ms = s_saved_telemetry_rate_ms;
    g_pid_cmd.telemetry_enabled = s_saved_telemetry_enabled;
    g_pid_cmd.test_mode = PID_TEST_MODE_NONE;

    // 发送测试结束标记
    wheel_pid_cmd_send_string("TEST_STEP_END");
}
```

---

## 四、集成步骤

### 步骤 1：添加源文件

将 `PID_cmd.h` 和 `PID_cmd.c` 放入 `code/control/` 目录下。

### 步骤 2：修改 PID.c

在 `wheel_pid_init()` 函数中添加命令系统初始化：

```c
// 在 PID.c 开头添加头文件包含
#include "PID_cmd.h"

// 在 wheel_pid_init() 函数末尾添加：
void wheel_pid_init(void)
{
    // ... 现有初始化代码 ...
    
    // 初始化命令系统
    wheel_pid_cmd_init();
}
```

### 步骤 3：修改 interrupt.c

在 `run_4ms_tasks()` 中添加命令轮询：

```c
// 在 interrupt.c 开头添加头文件包含
#include "PID_cmd.h"

// 在 run_4ms_tasks() 函数中添加：
static void run_4ms_tasks(void)
{
    // ... 现有代码 ...
    
    // 轮询处理串口命令和遥测
    wheel_pid_cmd_poll();
}
```

### 步骤 4：修改 wireless_uart_callback（可选方案A）

如果希望在中断中立即处理命令，修改 `zf_device_wireless_uart.c`：

```c
// 在 zf_device_wireless_uart.c 开头添加
#include "PID_cmd.h"

// 修改 wireless_uart_callback 函数：
void wireless_uart_callback(void)
{
    uart_query_byte(WIRELESS_UART_INDEX, &wireless_uart_data);
    fifo_write_buffer(&wireless_uart_fifo, &wireless_uart_data, 1);
    
    // 添加：处理命令字节
    wheel_pid_cmd_process_byte(wireless_uart_data);
    
    // ... 现有的自动波特率检测代码 ...
}
```

### 步骤 5：或者使用轮询方式（可选方案B）

如果不想修改库文件，可以在定时任务中从 FIFO 读取：

```c
// 在 run_4ms_tasks() 中添加：
static void run_4ms_tasks(void)
{
    // ... 现有代码 ...
    
    // 从 FIFO 读取并处理命令
    uint8 rx_byte;
    while (wireless_uart_read_buffer(&rx_byte, 1) > 0)
    {
        wheel_pid_cmd_process_byte(rx_byte);
    }
    
    // 轮询处理命令和遥测
    wheel_pid_cmd_poll();
}
```

### 步骤 6：在 ADS 中添加编译

1. 在 ADS 项目中刷新文件系统
2. 确保 `PID_cmd.c` 被编译器识别
3. 构建项目 (Ctrl+B)

### 步骤 7：烧录固件

#### 7.1 安全烧录流程

```
1. 【硬件准备】
   ├── 车轮悬空（架在支架上）
   └── 电机电源断开（只给主控板供电）

2. 【连接调试器】
   └── 确保 COM33 (DAS JDS) 已连接

3. 【同步源码】（如需要）
   └── python tools\sync.py --force

4. 【构建项目】
   └── 在 ADS IDE 中按 Ctrl+B

5. 【烧录固件】
   └── tools\headless_build.bat --flash-only -no-start

6. 【检查状态】
   ├── 串口监控 (COM24) 准备好
   └── 确认程序在 HALT 状态

7. 【上电运行】
   ├── 给电机供电
   └── 按复位键 或 调试器启动程序

8. 【观察测试】
   └── 通过串口/无线模块观察运行状态
```

#### 7.2 烧录命令说明

| 命令 | 作用 |
|------|------|
| `tools\headless_build.bat --flash-only` | 烧录后自动启动运行 |
| `tools\headless_build.bat --flash-only -no-start` | **安全模式**：烧录后保持 HALT 状态 |
| `tools\headless_build.bat --sync --flash-only` | 先同步源码再烧录 |
| `tools\headless_build.bat -f` | 构建 + 烧录（完整流程） |

#### 7.3 安全原则

| 顺序 | 操作 | 原因 |
|------|------|------|
| ✅ 先 | 车轮悬空 | 防止意外跑动 |
| ✅ 先 | 断开电机电源 | 烧录时电机不动 |
| ✅ 后 | 给电机供电 | 程序确认正常后 |
| ✅ 后 | 复位/启动 | 最后一步才运行 |

---

## 五、测试命令示例

### 基础命令测试

```
HELP
```
预期响应：显示所有可用命令

```
GET_STATUS
```
预期响应：显示当前 PID 参数、使能状态、目标值

### 参数调整测试

```
SET_SPEED_PARAM 0 50 0 100
```
设置速度环参数：Kp=0, Ki=50, Kd=0, I_limit=100

```
SET_SPEED_TARGET 0.5 0.5
```
设置目标速度：左轮0.5m/s，右轮0.5m/s

```
ENABLE_LOOPS 1 1 0 0
```
使能输出和速度环，关闭位置环和角速度环

### 遥测测试

```
SET_TELEMETRY_RATE 50
```
开启遥测，每50ms输出一行

```
SET_TELEMETRY_RATE 0
```
关闭遥测

### 阶跃测试

```
START_STEP_TEST 1.0 3000
```
执行阶跃测试：目标速度1.0m/s，持续3秒

```
STOP_TEST
```
手动停止测试

---

## 六、遥测数据格式

### CSV 字段说明

| 序号 | 字段 | 单位 | 说明 |
|------|------|------|------|
| 1 | TELEMETRY | - | 数据标识符 |
| 2 | timestamp_ms | ms | 系统时间戳 |
| 3 | speed_cmd_l | m/s | 左轮目标速度 |
| 4 | speed_cmd_r | m/s | 右轮目标速度 |
| 5 | speed_act_l | m/s | 左轮实际速度 |
| 6 | speed_act_r | m/s | 右轮实际速度 |
| 7 | yaw_cmd | rad/s | 目标角速度 |
| 8 | yaw_act | rad/s | 实际角速度参考值 |
| 9 | out_pwm_l | % | 左轮PWM输出 |
| 10 | out_pwm_r | % | 右轮PWM输出 |
| 11 | out_steer | - | 转向PWM输出 |
| 12 | error_spd_l | m/s | 左轮速度误差 |
| 13 | error_spd_r | m/s | 右轮速度误差 |
| 14 | error_yaw | rad/s | 角速度误差 |
| 15 | pos_l | m | 左轮累计里程 |
| 16 | pos_r | m | 右轮累计里程 |

### 示例数据行

```
TELEMETRY,1000,0.500,0.500,0.482,0.491,0.000,0.045,45.2,47.8,0.0,0.018,0.009,-0.045,1.234,1.256
```

---

## 七、Python 自动调参脚本示例

```python
import serial
import time

class PIDTuner:
    def __init__(self, port='COM24', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.5)
    
    def send_cmd(self, cmd):
        self.ser.write((cmd + '\r\n').encode())
        time.sleep(0.1)
        while self.ser.in_waiting:
            print(self.ser.readline().decode().strip())
    
    def set_speed_param(self, kp, ki, kd, i_limit):
        self.send_cmd(f'SET_SPEED_PARAM {kp} {ki} {kd} {i_limit}')
    
    def start_step_test(self, speed, duration_ms):
        self.send_cmd(f'START_STEP_TEST {speed} {duration_ms}')
    
    def collect_telemetry(self, duration_s):
        data = []
        start = time.time()
        while time.time() - start < duration_s:
            line = self.ser.readline().decode().strip()
            if line.startswith('TELEMETRY,'):
                data.append(line)
        return data

# 使用示例
tuner = PIDTuner('COM24')
tuner.set_speed_param(0, 50, 0, 100)
tuner.start_step_test(1.0, 3000)
data = tuner.collect_telemetry(3.5)
print(f'Collected {len(data)} samples')
```

---

## 八、注意事项

1. **编码问题**：C源文件使用 GB2312 编码，本文档中的中文注释需要确保编码正确
2. **串口冲突**：遥控器数据也使用 UART2，命令解析会与遥控数据混合，建议在非遥控操作时使用
3. **测试安全**：阶跃测试时确保车轮架空或车辆在安全区域
4. **遥测频率**：不要设置过高的遥测频率（建议 >= 20ms），避免串口缓冲区溢出

---

## 九、版本历史

| 版本  | 日期       | 修改内容           |
|-------|------------|--------------------|
| 1.0.1 | 2026-04-28 | 添加烧录操作步骤   |
| 1.0.0 | 2026-04-27 | 初始版本           |
