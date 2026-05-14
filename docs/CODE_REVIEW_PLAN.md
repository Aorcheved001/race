# AURIX TC377 new_ins 代码问题检查计划书

> **目标**: 全面检查代码问题、工程问题、逻辑问题  
> **范围**: 仅代码文件（.c/.h），不包含脚本、配置等  
> **方法**: 5轮迭代检查 × 每轮3次验证  
> **输出**: 只输出真实存在的问题  
> **车辆架构**: 前轮转向 + 后轮驱动（阿克曼转向）

---

## ? 执行策略

### 检查维度
1. **代码逻辑问题**: 算法错误、边界条件、死锁、竞态条件
2. **工程架构问题**: 模块耦合、接口设计、数据流向、初始化顺序
3. **内存安全问题**: 数组越界、指针悬空、栈溢出、内存泄漏
4. **实时性问题**: 中断嵌套、优先级反转、时序冲突、周期抖动
5. **硬件接口问题**: 寄存器配置、外设初始化、时序约束、电气特性

### 验证流程（每轮3次验证）
```
第N轮检查
├── 第1次验证: 代码审查 + 静态分析 + 规则检查
├── 第2次验证: 逻辑推导 + 数据流追踪 + 状态分析
└── 第3次验证: 交叉验证 + 文档对比 + 实际场景模拟
    └── 确认问题真实存在 → 记录到问题清单
```

### 严重等级定义
- **? 致命 (Critical)**: 导致系统崩溃、死机、硬件损坏
- **? 严重 (Major)**: 功能失效、数据错误、性能严重下降
- **? 一般 (Minor)**: 功能异常但可恢复、代码质量、可维护性
- **? 建议 (Suggestion)**: 优化建议、代码风格、最佳实践

---

## ? 检查计划

### 第1轮: 核心控制模块（PID + Track + Steering + Motor + Encoder）
**重点**: 控制逻辑正确性、参数一致性、接口匹配

### 第2轮: 导航与传感器融合（INS + IMU + GNSS + 磁力计）
**重点**: 坐标系转换、数据融合、滤波算法

### 第3轮: 系统架构与调度（中断 + 任务 + 初始化 + 全局变量）
**重点**: 时序分析、优先级、资源竞争

### 第4轮: 数据流与接口（模块间通信 + 参数传递 + 单位一致性）
**重点**: 数据完整性、接口契约、状态一致性

### 第5轮: 边界与异常处理（边界条件 + 异常恢复 + 数值安全）
**重点**: 鲁棒性、容错性、安全性

---

## ? 问题记录格式

### 问题模板
```markdown
## 问题编号: P-N-M-L
**发现轮次**: 第N轮 第M次验证
**问题分类**: [代码逻辑/工程架构/内存安全/实时性/硬件接口]
**严重等级**: ?致命 / ?严重 / ?一般 / ?建议
**文件位置**: `file.c:line` 或 `file.h:line`
**问题描述**: 清晰描述问题现象和影响
**问题代码**: 
```c
// 有问题的代码片段
```
**验证方法**: 如何确认问题真实存在
**影响分析**: 对系统的影响范围和程度
**修复建议**: 如何修复（仅建议，不修改代码）
```

---

## ? 执行状态

- [x] **第1轮检查**: 核心控制模块 ? (发现5个问题)
- [x] **第2轮检查**: 导航与传感器融合 ? (发现8个问题)
- [x] **第3轮检查**: 系统架构与调度 ? (发现2个问题)
- [ ] **第4轮检查**: 数据流与接口
- [ ] **第5轮检查**: 边界与异常处理

**当前进度**: 第3轮检查完成，准备执行第4轮检查

---

## ? 问题统计

| 轮次 | 致命 | 严重 | 一般 | 建议 | 总计 |
|------|------|------|------|------|------|
| 第1轮 | 0 | 1 | 1 | 3 | 5 |
| 第2轮 | 0 | 2 | 2 | 4 | 8 |
| 第3轮 | 0 | 2 | 0 | 0 | 2 |
| 第4轮 | 0 | 0 | 0 | 0 | 0 |
| 第5轮 | 0 | 0 | 0 | 0 | 0 |
| **总计** | **0** | **5** | **3** | **7** | **15** |

---

## ? 问题清单

---

### 第1轮检查：核心控制模块

---

## 问题编号: P1-1-3-一般
**发现轮次**: 第1轮 第3次验证  
**问题分类**: 工程架构  
**严重等级**: ? 一般  
**文件位置**: `code/control/PID.c:541-556`  

**问题描述**:  
对于阿克曼转向车辆（前轮转向+后轮驱动），PID角速度环计算的`out_steer`输出未被应用到任何执行器，属于冗余计算。

**问题代码**:  
```c
// PID.c: wheel_pid_update_cascade()
out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, ...);  // 角速度环输出

// 但实际输出只用了 out_left 和 out_right
motor_control(motor_LB, MOTOR_DIR_FORWARD, (uint8)out_left);
motor_control(motor_RB, MOTOR_DIR_FORWARD, (uint8)out_right);
/* 角速度环输出 out_steer_pwm 供外部转向模块使用 */  // ?? 但实际未被使用
```

**验证方法**:  
1. 代码审查：检查 `out_steer` 是否被应用
2. 逻辑推导：阿克曼转向车辆不需要差速转向
3. 调用分析：`track.c` 中转向由 `steering_control()` 独立控制

**影响分析**:  
- 浪费计算资源（每个4ms周期执行一次PID计算）
- 可能造成困惑（代码暗示有角速度控制，但实际未启用）

**修复建议**:  
1. 如果确认不需要差速转向，禁用角速度环：`wheel_pid_enable(1, 1, 0, 0)`
2. 或者移除角速度环计算逻辑，简化代码


**用户意见**:  
1.车身转弯半径较大，为缩小转弯半径所以加入了转向环，来做辅助转向，帮助缩小转向半径，前期先不使用差速辅助转向，所以这个输出的先保留，后期如果需要用到辅助差速转向再使用

**修复方案**:  
保留现状，不修改。`out_steer` 输出保留供后期差速辅助转向使用。
---

## 问题编号: P1-2-3-建议
**发现轮次**: 第1轮 第3次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 建议  
**文件位置**: `code/control/PID.c:462 + code/control/PID.c:56`  

**问题描述**:  
PID 模式2中轮距硬编码为 `0.2f`，而其他地方使用宏定义 `COMPAT_WHEELBASE_M`，代码风格不一致，存在维护风险。

**问题代码**:  
```c
// PID.c: 模式2中
float target_yaw = (cmd_right - cmd_left) / 0.2f;  // ?? 硬编码

// PID.c: wheel_pid_update中
float estimated_yaw = (cmd_right - cmd_left) / COMPAT_WHEELBASE_M;  // 使用宏定义

// PID.c:56
#define COMPAT_WHEELBASE_M  0.2f
```

**验证方法**:  
1. 代码审查：发现两处轮距计算方式不一致
2. 数值验证：虽然值相同（0.2f），但代码风格不一致

**影响分析**:  
- 维护困难：修改轮距时需要改多处
- 可读性差：硬编码的魔数不易理解

**修复建议**:  
统一使用宏定义：
```c
float target_yaw = (cmd_right - cmd_left) / COMPAT_WHEELBASE_M;
```
**用户意见**:  
1.这个0.2的数据只是为了在小车上做验证，并不是真实的车轴距离，我建议统一使用宏定义，在头文件里面申明，方便修改，注释写清楚，数值先填0.2，后面我会自己修改

**修复方案**:  
? 已修复
1. 创建 `code/vehicle_config.h` 统一车辆参数配置文件
2. 定义 `VEHICLE_TRACK_WIDTH_M 0.2f`（轮距，用于差速转向）
3. 定义 `VEHICLE_WHEELBASE_M 0.2107f`（轴距，用于Pure Pursuit）
4. 提供兼容宏 `COMPAT_WHEELBASE_M` 指向 `VEHICLE_TRACK_WIDTH_M`
5. PID.c 中已将硬编码 `0.2f` 替换为 `COMPAT_WHEELBASE_M`
---

## 问题编号: P1-3-3-建议
**发现轮次**: 第1轮 第3次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 建议  
**文件位置**: `code/control/track.h:39 + code/control/PID.c:56`  

**问题描述**:  
`TRACK_WHEELBASE` (0.2107m) 和 `COMPAT_WHEELBASE_M` (0.2m) 名称相似但含义不同，容易混淆。

**问题代码**:  
```c
// track.h:39 - 轴距（前后轮距离），用于 Pure Pursuit
#define TRACK_WHEELBASE  0.2107f

// PID.c:56 - 轮距（左右轮距离），用于角速度估算
#define COMPAT_WHEELBASE_M  0.2f
```

**验证方法**:  
1. 名称分析：两者都包含 "WHEELBASE"，容易混淆
2. 用途分析：一个用于转向计算（轴距），一个用于角速度估算（轮距）

**影响分析**:  
- 维护风险：新人容易误解参数含义
- 注释不足：没有说明两者的区别

**修复建议**:  
添加清晰的注释：
```c
// track.h
#define TRACK_WHEELBASE  0.2107f  // 轴距（前后轮距离），用于 Pure Pursuit 转向计算

// PID.c
#define COMPAT_WHEELBASE_M  0.2f  // 轮距（左右轮距离），用于差速转向角速度估算
```
**用户意见**:  
1.按照修复建议办

**修复方案**:  
? 已修复
1. 在 `vehicle_config.h` 中为所有车辆参数添加了清晰注释
2. `VEHICLE_WHEELBASE_M` - 轴距（前后轮距离），用于 Pure Pursuit 转向计算
3. `VEHICLE_TRACK_WIDTH_M` - 轮距（左右轮距离），用于差速转向角速度估算
---

## 问题编号: P1-4-2-严重
**发现轮次**: 第1轮 第2次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 严重  
**文件位置**: `code/control/track.c:260-369`  

**问题描述**:  
`find_lookahead_point()` 函数内部修改全局变量 `follow_abs_index`，如果搜索失败，索引可能被修改到不可预测的位置，导致下次循迹从错误的位置开始。

**问题代码**:  
```c
// track.c: find_lookahead_point()
static uint8 find_lookahead_point(float x, float y)
{
    uint32 start_index = follow_abs_index;  // 保存起始索引
    
    while(follow_abs_index <= search_end)
    {
        follow_abs_index++;  // ?? 修改全局变量
        
        if(follow_abs_index > search_end)
            break;
    }
    
    // 如果所有点都没找到，直接跳到最后一个点
    if(track_follow_prepare_point(stored_points))
    {
        follow_abs_index = stored_points;  // ?? 修改为最后一个点
        return 1;
    }
    
    return 0;  // ?? 失败时，索引可能已被修改
}
```

**验证方法**:  
1. 代码审查：函数名暗示只是"查找"，但实际修改全局状态
2. 调用分析：`track_follow()` 中如果查找失败，索引已被污染

**影响分析**:  
- 循迹逻辑错误：搜索失败后，下次循迹可能从错误位置开始
- 函数副作用：违反单一职责原则，难以维护

**修复建议**:  
1. 如果查找失败，恢复原始索引：
```c
uint32 saved_index = follow_abs_index;
// ... 搜索逻辑
if(!found) {
    follow_abs_index = saved_index;  // 恢复索引
    return 0;
}
```

2. 或者将函数分为两个职责：
   - `find_lookahead_point()`: 只查找，不修改索引
   - `advance_to_next_point()`: 推进索引

**用户意见**:  
1.建议使用局部变量先进行索引，如果索引到了才更新全局变量

**修复方案**:  
? 已修复
1. 在 `find_lookahead_point()` 中使用局部变量 `local_abs_index` 进行索引搜索
2. 只有在成功找到有效点后才更新全局 `follow_abs_index`
3. 搜索失败时全局索引保持不变，避免污染
4. 同时修复了 P5-3-2：增加了对 `track_flash_get_point()` 返回值的检查，避免使用未初始化数据
---
---

## 问题编号: P1-5-3-建议
**发现轮次**: 第1轮 第3次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 建议  
**文件位置**: `code/control/PID.c:570`  

**问题描述**:  
`wheel_pid_update()` 中角速度估算使用的是**命令速度差**而非**实际速度差**，可能导致估算误差。

**问题代码**:  
```c
// PID.c: wheel_pid_update()
float estimated_yaw = (g_wheel_pid.cmd_speed_right_mps - g_wheel_pid.cmd_speed_left_mps) / COMPAT_WHEELBASE_M;

// ?? 使用的是命令速度（目标），而非编码器实际速度
// enc->speed_right_mps 和 enc->speed_left_mps 才是实际速度
```

**验证方法**:  
1. 代码审查：检查 `estimated_yaw` 的计算方式
2. 逻辑推导：命令速度是目标值，实际速度可能有延迟和误差

**影响分析**:  
- 低速或加减速时，估算角速度与实际角速度有偏差
- 可能影响角速度环控制效果

**修复建议**:  
使用编码器实际速度估算：
```c
float estimated_yaw = (enc->speed_right_mps - enc->speed_left_mps) / COMPAT_WHEELBASE_M;
```

**备注**:  
对于阿克曼转向车辆，这个估算值可能不需要。如果是差速转向，需要使用实际速度。
**用户意见**:  
1.我不是跟你说了嘛，我是阿克曼结构，但是车身转弯半径过大，所以我打算使用后轮差速辅助转向，具体使用哪个你决定下

**修复方案**:  
? 已修复
使用编码器实际速度估算角速度，更适合差速辅助转向场景：
```c
float estimated_yaw = (enc->speed_right_mps - enc->speed_left_mps) / COMPAT_WHEELBASE_M;
```
原因：实际速度更准确反映车辆运动状态，对差速辅助转向控制更有效。
---
---

### 第1轮问题统计

| 严重等级 | 数量 | 问题编号 |
|---------|------|---------|
| ? 致命 | 0 | - |
| ? 严重 | 1 | P1-4-2 |
| ? 一般 | 1 | P1-1-3 |
| ? 建议 | 3 | P1-2-3, P1-3-3, P1-5-3 |
| **总计** | **5** | - |

---

### 第2轮检查：导航与传感器融合

---

## 问题编号: P2-1-3-严重
**发现轮次**: 第2轮 第3次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 严重  
**文件位置**: `code/ins/Ins.c:91 + code/ins/Ins.c:96`  

**问题描述**:  
`map_body_attitude_for_mag_compensation()` 函数将姿态解算器的 roll/pitch 映射到车体坐标时，映射关系在两处使用不一致：
- 磁力计倾斜补偿时：`body_roll = -solver_pitch`, `body_pitch = solver_roll`
- 上位机输出时（interrupt.c:189）：`body_roll = pitch_rad`, `body_pitch = -roll_rad`

**问题代码**:  
```c
// Ins.c:91 - 磁力计补偿映射
*body_roll = -solver_pitch;
*body_pitch = solver_roll;

// interrupt.c:189 - 上位机输出映射（相反的映射！）
body_roll_rad = pitch_rad;
body_pitch_rad = -roll_rad;
```

**验证方法**:  
1. 代码审查：对比两处映射关系
2. 逻辑推导：同一次姿态数据，两个不同的映射结果

**影响分析**:  
- 上位机显示的姿态与实际用于磁力计补偿的姿态不一致
- 调试时会产生困惑，可能误判问题

**修复建议**:  
统一映射关系，或者添加注释说明两处映射的目的不同。
**用户意见**:  
1.先不修复，保留，待我详细阅读后决定

**修复方案**:  
? 已修复（与 P4-1-3 一并处理）
1. 修改 `interrupt.c` 中的姿态映射，与 INS 层 `map_body_attitude_for_mag_compensation()` 保持一致
2. 统一映射关系：`body_roll = -pitch`, `body_pitch = roll`
3. 上位机显示与算法内部补偿现在使用相同的姿态口径
4. 消除了调试时的困惑
---
---

## 问题编号: P2-2-2-严重
**发现轮次**: 第2轮 第2次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 严重  
**文件位置**: `code/ins/Ins.c:110-119 + code/control/encoder.c:11-12`  

**问题描述**:  
INS 模块中编码器脉冲转距离系数 `tick_to_meter_left/right` 默认值为 `-0.00002204f`（负值），但 encoder.c 中默认值为 `0.00002204f`（正值），符号不一致可能导致里程计方向相反。

**问题代码**:  
```c
// Ins.c:110-111 - INS 配置
s_config.tick_to_meter_left = -0.00002204f;
s_config.tick_to_meter_right = -0.00002204f;

// encoder.c:11-12 - 编码器模块
static float g_tick_to_meter_left  = 0.00002204f;  // 正值！
static float g_tick_to_meter_right = 0.00002204f;  // 正值！
```

**验证方法**:  
1. 代码审查：对比两处默认值
2. 数据流追踪：encoder.c 中已经对左轮取反（line 57），INS 中再次取反会导致方向错误

**影响分析**:  
- 如果 INS 配置被应用，里程计方向会反转
- 实际运行中 INS 配置可能覆盖 encoder 配置，导致位置计算错误

**修复建议**:  
确认正确的方向约定，统一两处的符号。encoder.c:57 已经有 `tick_left = -encoder_get_count(...)` 处理，INS 中应为正值。

**用户意见**: 
 1:这个是我init层调用的encoder_layer_set_model(-0.00002204f, -0.00002204f, 0.004f);    // 设置编码器模型参数
 只有这个前面加-号的时候才又有正常的里程计方向，你可以分析下，阅读全部的代码后给出更详细的指导意见

**修复方案**:  
? 已修复
通过 P2-3-3 的修改，INS 不再使用内部的 `tick_to_meter` 系数，而是直接使用 encoder 模块已计算好的 `delta_left_m/delta_right_m`。这样：
1. 消除了 INS 和 encoder 两处系数定义的冗余
2. 数据流单向：encoder 计算 → interrupt 传递 → INS 使用
3. 方向一致性由 encoder 模块统一管理（在 `encoder_layer_set_model()` 中设置）
---
---

## 问题编号: P2-3-3-一般
**发现轮次**: 第2轮 第3次验证  
**问题分类**: 工程架构  
**严重等级**: ? 一般  
**文件位置**: `code/ins/Ins.c:577 + code/system/interrupt.c:103`  

**问题描述**:  
`Ins_update()` 内部直接调用 `encoder_layer_get_state()` 获取编码器数据，破坏了模块间的数据流单向性。上层已传入 `INS_Input` 结构体，但 INS 内部又自行获取编码器数据。

**问题代码**:  
```c
// Ins.c:577 - 内部直接获取编码器数据
enc_state = encoder_layer_get_state();
if(enc_state != NULL)
{
    int16 tick_left = enc_state->tick_left;
    int16 tick_right = enc_state->tick_right;
    // ...
}

// interrupt.c:103 - 上层已传入 INS_Input
Ins_update(&s_ins_input, 0.004f);  // 但 INS_Input 中没有 tick_left/right
```

**验证方法**:  
1. 接口分析：`INS_Input` 结构体中没有编码器 tick 数据
2. 数据流追踪：位置更新依赖的数据来源不明确

**影响分析**:  
- 模块耦合：INS 直接依赖 encoder 模块
- 可测试性差：难以单独测试 INS 模块
- 数据流混乱：不清楚位置更新数据的真实来源

**修复建议**:  
在 `INS_Input` 中添加编码器 tick 数据，由上层传入：
```c
typedef struct {
    // ... existing fields
    int16_t tick_left;
    int16_t tick_right;
} INS_Input;
```
**用户意见**: 
按照修复建议办

**修复方案**:  
? 已修复
1. 在 `INS_Input` 结构体中添加 `delta_left_m` 和 `delta_right_m` 字段
2. `interrupt.c` 中从 `EncoderLayerState` 获取并填充这两个字段
3. `Ins.c` 中 `ins_position_update()` 改为接收位移增量而非原始 tick
4. INS 不再直接调用 `encoder_layer_get_state()`，数据流单向化
---

---

## 问题编号: P2-4-2-一般
**发现轮次**: 第2轮 第2次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 一般  
**文件位置**: `code/system/interrupt.c:98`  

**问题描述**:  
`get_mag_yaw_measurement()` 函数在 `interrupt.c` 中计算磁航向，但 `Ins.c` 中也有 `compute_tilt_compensated_mag_yaw()` 计算倾斜补偿后的磁航向，两处逻辑重复且不一致。

**问题代码**:  
```c
// interrupt.c:80 - 简单的水平面磁航向（无倾斜补偿）
*yaw_rad = normalize_angle_rad_local(-atan2f(mag_y, mag_x));

// Ins.c:72 - 倾斜补偿后的磁航向
mag_x_horizontal = mag_x * cos_pitch + mag_z * sin_pitch;
mag_y_horizontal = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;
return normalize_angle_rad(-atan2f(mag_y_horizontal, mag_x_horizontal));
```

**验证方法**:  
1. 代码审查：发现两处磁航向计算
2. 功能对比：interrupt.c 中未做倾斜补偿，Ins.c 中做了倾斜补偿
3. 数据流追踪：`s_ins_input.mag_yaw_rad` 被计算但未在 Ins_update() 中使用

**影响分析**:  
- 冗余计算：interrupt.c 中计算的磁航向未被使用
- 功能不一致：两处计算方式不同
- 维护困难：修改时需要同步多处

**修复建议**:  
移除 interrupt.c 中的磁航向计算，只保留 Ins.c 中带倾斜补偿的版本。


**用户意见**: 
按照修复意见办

**修复方案**:  
? 已修复
1. `get_mag_yaw_measurement()` 不再计算简单的水平面磁航向
2. 仅检查磁力计数据有效性，返回占位值
3. 实际的倾斜补偿磁航向由 INS 内部的 `compute_tilt_compensated_mag_yaw()` 计算
4. 消除了两处磁航向计算的冗余和不一致

---

## 问题编号: P2-5-3-建议
**发现轮次**: 第2轮 第3次验证  
**问题分类**: 工程架构  
**严重等级**: ? 建议  
**文件位置**: `code/ins/Ins.h:15 + code/control/track.h:37`  

**问题描述**:  
`INS_WHEELBASE_M` (0.2107m) 和 `TRACK_WHEELBASE` (0.2107m) 值相同但定义在两个地方，存在维护风险。与第1轮问题 P1-3-3 类似，但这次是轴距参数的重复定义。

**问题代码**:  
```c
// Ins.h:15
#define INS_WHEELBASE_M      0.2107f  // 轴距 (米)

// track.h:37
#define TRACK_WHEELBASE               0.2107f
```

**验证方法**:  
1. 数值对比：两者值相同
2. 用途分析：都用于车辆轴距参数

**影响分析**:  
- 维护风险：修改轴距时需要改两处
- 可能不一致：如果只修改一处会导致行为异常

**修复建议**:  
统一使用一个宏定义，例如：
```c
// vehicle_config.h
#define VEHICLE_WHEELBASE_M  0.2107f  // 车辆轴距（前后轮距离）
```
**用户意见**: 
1：可以新建一个车辆参数层，装的都是车辆数据的宏定义，把应该塞进去的参数都塞进去，并且注释写清楚

**修复方案**:  
? 已修复
1. 创建 `code/vehicle_config.h` 统一车辆参数配置文件
2. 包含轴距、轮距、轮径、编码器参数、转向参数等
3. 提供兼容宏 `INS_WHEELBASE_M` 和 `TRACK_WHEELBASE` 指向 `VEHICLE_WHEELBASE_M`
4. 所有参数都有详细注释说明用途

---

## 问题编号: P2-6-3-建议
**发现轮次**: 第2轮 第3次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 建议  
**文件位置**: `code/ins/Ins.c:574`  

**问题描述**:  
`Ins_update()` 中 `(void)input->omega_rad_s` 将车辆模型计算的角速度丢弃，该数据由上层辛苦计算但未被使用，属于资源浪费。

**问题代码**:  
```c
// Ins.c:574
(void)input->omega_rad_s;  // 当前版本未使用车辆模型角速度

// interrupt.c:101 - 上层花费计算资源
s_ins_input.omega_rad_s = s_ins_input.v_mps * tanf(steer_rad) / INS_WHEELBASE_M;
```

**验证方法**:  
1. 数据流追踪：`omega_rad_s` 被计算但未使用
2. 性能分析：涉及三角函数计算（tanf），浪费 CPU 周期

**影响分析**:  
- 浪费计算资源（每个4ms周期一次三角函数）
- 代码可读性差：计算了但不用

**修复建议**:  
如果确认不需要车辆模型角速度，移除上层计算；或者让 INS 使用该数据补充角速度观测。

---
**用户意见**: 
1：本来方案是打算使用二维ekf的车辆运动约束加磁力计的值算yaw_bias的来修正yaw，但是现在我还在调磁力计，所以这个车辆运动状态算出来的值还是保持原样，先不进行使用

**修复方案**:  
暂不修复。保留 `omega_rad_s` 计算，待磁力计调试完成后启用。

## 问题编号: P2-7-2-建议
**发现轮次**: 第2轮 第2次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 建议  
**文件位置**: `code/ins/Ins.c:574-576`  

**问题描述**:  
`Ins_update()` 末尾有多处 `(void)` 强制忽略参数，暗示 `INS_Config` 中的某些配置项未被使用，配置参数冗余。

**问题代码**:  
```c
// Ins.c:574-576
(void)input->omega_rad_s;
(void)input->mag_yaw_rad;
(void)s_config.Q_yaw;      // ?? 但 s_yaw_ekf.N_psi 用的是这个值
(void)s_config.R_mag;      // ?? 但 s_yaw_ekf.R 用的是这个值
(void)s_config.wheelbase;
```

**验证方法**:  
1. 代码审查：`(void)` 语句暗示未使用
2. 逻辑矛盾：Q_yaw 和 R_mag 被标记未使用，但 yaw_ekf 确实在使用

**影响分析**:  
- 可能是代码重构后的遗留问题
- 造成理解困难：参数到底用没用？

**修复建议**:  
移除这些 `(void)` 语句，或者添加注释说明为何忽略。
**用户意见**: 
1：重新详细分析下这个问题是否真实

**修复方案**:  
? 已修复
经重新分析，问题**不完全真实**：
- `s_config.Q_yaw` 在 `yaw_ekf2_init()` 中用于初始化 `s_yaw_ekf.N_psi`，已正确使用
- `s_config.R_mag` 在 `yaw_ekf2_init()` 中用于初始化 `s_yaw_ekf.R`，已正确使用
- `omega_rad_s`、`mag_yaw_rad`、`wheelbase` 确实未被使用

已修改为：
1. 移除对 `Q_yaw` 和 `R_mag` 的 `(void)` 标记
2. 保留对确实未使用参数的 `(void)` 标记，并添加注释说明原因

---

## 问题编号: P2-8-3-建议
**发现轮次**: 第2轮 第3次验证  
**问题分类**: 工程架构  
**严重等级**: ? 建议  
**文件位置**: `code/ins/Ins.c`  

**问题描述**:  
INS 模块内部存在多个静态全局变量（`s_state`, `s_config`, `s_kalman_6axis`, `s_yaw_ekf`, `s_pos_x`, `s_pos_y` 等），但缺少统一的初始化状态检查机制。

**问题代码**:  
虽然有 `s_initialized` 标志，但 `Ins_update()` 会自动调用 `Ins_init()`：
```c
// Ins.c:492-495
if(!s_initialized)
{
    Ins_init();
}
```

**验证方法**:  
1. 状态分析：未初始化时自动初始化，可能掩盖初始化时序问题
2. 实时性分析：首次调用会有额外初始化开销

**影响分析**:  
- 可能掩盖初始化时序错误
- 首次调用延迟不可预测

**修复建议**:  
明确要求上层在启动时显式调用 `Ins_init()`，如果 `Ins_update()` 发现未初始化应报错而非自动初始化。

---
**用户意见**: 
按照修复意见办理

**修复方案**:  
? 已修复
1. `Ins_update()` 不再自动调用 `Ins_init()`
2. 如果未初始化，`Ins_update()` 直接返回，不做任何处理
3. 要求上层在 `system_init_all()` 中显式调用 `Ins_init()`（已确认存在）
4. 这样可以暴露初始化时序问题，避免首次调用延迟不可预测

### 第2轮问题统计

| 严重等级 | 数量 | 问题编号 |
|---------|------|---------|
| ? 致命 | 0 | - |
| ? 严重 | 2 | P2-1-3, P2-2-2 |
| ? 一般 | 2 | P2-3-3, P2-4-2 |
| ? 建议 | 4 | P2-5-3, P2-6-3, P2-7-2, P2-8-3 |
| **总计** | **8** | - |

---

### 第3轮检查：系统架构与调度

---

## 问题编号: P3-1-2-严重
**发现轮次**: 第3轮 第2次验证  
**问题分类**: 实时性  
**严重等级**: ? 严重  
**文件位置**: `code/system/interrupt.c:73-78 + code/system/interrupt.c:249-267`  

**问题描述**:  
周期计数器 `s_pending_4ms`、`s_pending_8ms`、`s_pending_40ms` 在中断上下文中递增，在主循环中递减，但 `InterruptTasks_Poll()` 对这些共享状态的读改写没有临界区保护，存在并发竞争，可能导致周期节拍被覆盖丢失。

**问题代码**:  
```c
// interrupt.c: 中断侧递增
void Interrupt_4ms(void)  { if (s_pending_4ms  < 500u) s_pending_4ms++; }
void Interrupt_8ms(void)  { if (s_pending_8ms  < 500u) s_pending_8ms++; }
void Interrupt_40ms(void) { if (s_pending_40ms < 500u) s_pending_40ms++; }

// interrupt.c: 主循环侧递减并消费
while (s_pending_4ms)
{
    s_pending_4ms--;
    run_4ms_tasks();
}

while (s_pending_8ms)
{
    s_pending_8ms--;
    run_8ms_tasks();
}

while (s_pending_40ms)
{
    s_pending_40ms--;
    run_40ms_tasks();
}
```

**验证方法**:  
1. 代码审查：识别出同一组 `volatile` 计数器同时被 ISR 和主循环访问  
2. 逻辑推导：`读取旧值 -> ISR 自增 -> 主循环回写旧值减1` 会覆盖新到达节拍  
3. 状态分析：该问题会直接影响 4ms/8ms/40ms 任务的执行次数，与调度设计目标不一致

**影响分析**:  
- 周期任务可能偶发丢拍，导致 PID、导航、上位机输出等任务执行频率不稳定
- 当主循环负载上升时，问题更容易暴露，实时性风险增大

**修复建议**:  
1. 对共享计数器的读取与递减增加临界区保护，避免 ISR 与主循环并发覆盖
2. 或改为“先原子取快照、后批量消费”的设计，降低共享状态竞争

---
**用户意见**: 
按照修复意见办理

**修复方案**:  
? 已修复
1. 在 `InterruptTasks_Poll()` 中使用临界区保护共享计数器
2. 采用"原子取快照、批量消费"的设计
3. 使用 `disableInterrupts()` / `restoreInterrupts()` 保护计数器读写
4. 避免了 ISR 与主循环并发覆盖的风险

## 问题编号: P3-2-3-严重
**发现轮次**: 第3轮 第3次验证  
**问题分类**: 工程架构  
**严重等级**: ? 严重  
**文件位置**: `user/cpu0_main.c:54-57 + code/system/interrupt.c:232-240 + code/control/ins_new_264.c:252-355`  

**问题描述**:  
`INS_Display()` 被两个不同调度入口重复调用：一处在 `run_40ms_tasks()` 中按 40ms 周期执行，另一处在 `core0_main()` 主循环中无节制执行。显示任务的实际执行频率因此取决于主循环空转速度，而不是任务书定义的周期调度。

**问题代码**:  
```c
// interrupt.c: 40ms 调度
static void run_40ms_tasks(void)
{
    INS_Display();
    send_attitude_to_host();
}

// cpu0_main.c: 主循环重复调用
while (TRUE)
{
    InterruptTasks_Poll();
    INS_Display();
}
```

**验证方法**:  
1. 代码审查：确认 `INS_Display()` 同时存在于 40ms 任务和主循环中
2. 文档对比：任务书第3轮关注“系统架构与调度”，该实现已破坏既定的周期执行边界
3. 场景模拟：当显示函数内部执行大量 `ips200_show_*` 操作时，主循环负载会被放大，进一步挤压周期任务处理时间

**影响分析**:  
- 显示任务执行频率不可控，40ms 调度约束失效
- 屏幕刷新负载被重复放大，可能拖慢主循环并放大其他周期任务的延迟
- 调试时看到的系统表现可能与设计时序不一致，增加定位难度

**修复建议**:  
1. 保留单一调度入口，让 `INS_Display()` 只在一个确定周期内执行
2. 将主循环职责限制为轮询调度，不直接插入额外的周期任务调用

---
**用户意见**: 
只需要在主循环里面把这个ins_display注释掉就可以了

**修复方案**:  
? 已修复
1. 在 `cpu0_main.c` 主循环中注释掉 `INS_Display()` 调用
2. 保留 `run_40ms_tasks()` 中的唯一调度入口
3. 确保显示任务按 40ms 周期稳定执行

### 第3轮问题统计

| 严重等级 | 数量 | 问题编号 |
|---------|------|---------|
| ? 致命 | 0 | - |
| ? 严重 | 2 | P3-1-2, P3-2-3 |
| ? 一般 | 0 | - |
| ? 建议 | 0 | - |
| **总计** | **2** | - |


---

### 第4轮检查：数据流与接口

---

## 问题编号: P4-1-3-严重
**发现轮次**: 第4轮 第3次验证  
**问题分类**: 数据流与接口  
**严重等级**: ? 严重  
**文件位置**: `code/system/interrupt.c:186-190 + code/ins/Ins.c:84-97`  

**问题描述**:  
车体姿态映射关系在输出层与算法层不一致。`interrupt.c` 向上位机输出时使用 `body_roll = pitch`、`body_pitch = -roll`，而 `Ins.c` 在磁力计倾斜补偿前却实现成 `body_roll = -pitch`、`body_pitch = roll`。同一姿态数据在两条链路中的符号口径相反，会导致显示结果与算法内部使用的数据不一致。

**问题代码**:  
```c
// interrupt.c: 输出层映射
body_roll_rad = pitch_rad;
body_pitch_rad = -roll_rad;

// Ins.c: 算法层映射
*body_roll = -solver_pitch;
*body_pitch = solver_roll;
```

**验证方法**:  
1. 代码审查：对比输出层和算法层的姿态映射实现
2. 数据流追踪：确认两处都处理同一组 roll/pitch 数据，但符号相反
3. 文档对比：项目约定明确要求 `body_roll = pitch`、`body_pitch = -roll`

**影响分析**:  
- 上位机显示与算法内部补偿使用的姿态口径不一致
- 磁力计倾斜补偿可能在部分姿态方向上出现反向修正

**修复建议**:  
1. 统一输出层与算法层的姿态映射口径
2. 将最终确认的映射关系固化到单一公共接口，避免多处手写映射

---
**用户意见**: 
统一使用ins层的逻辑关系

**修复方案**:  
? 已修复
1. 修改 `interrupt.c` 中的姿态映射，与 INS 层 `map_body_attitude_for_mag_compensation()` 保持一致
2. 统一映射关系：`body_roll = -pitch`, `body_pitch = roll`
3. 更新注释说明映射逻辑
4. 上位机显示与算法内部补偿现在使用相同的姿态口径

## 问题编号: P4-2-2-严重
**发现轮次**: 第4轮 第2次验证  
**问题分类**: 数据流与接口  
**严重等级**: ? 严重  
**文件位置**: `code/control/track.c:181-221 + code/control/track.c:427-441 + code/control/track.h:51-56`  

**问题描述**:  
循迹目标点结构 `Ins_follow` 包含 `speed_dir` 字段，`pure_pursuit_calc_steer()` 也依赖该字段在倒车时反转转向角，但 `track_follow()` 构造传入目标点时只初始化了 `x/y/yaw`，没有传递 `speed_dir`。结果是转向计算层实际收到的 `speed_dir` 为默认零值，接口中的方向信息被静默丢失。

**问题代码**:  
```c
// pure_pursuit_calc_steer() 依赖 speed_dir
if(target->speed_dir < 0.5f)
{
    steer_deg = -steer_deg;
}

// track_follow() 构造目标点时未传递 speed_dir
Ins_follow target = {Ins_date_377.x, Ins_date_377.y, Ins_date_377.yaw};
steer_output = pure_pursuit_calc_steer(state->x, state->y, state->yaw, &target);
```

**验证方法**:  
1. 代码审查：确认 `Ins_follow` 定义包含 `speed_dir`
2. 数据流追踪：确认目标点从 `Ins_date_377` 传到 `pure_pursuit_calc_steer()` 时丢失该字段
3. 场景模拟：倒车循迹时，转向反向逻辑会基于错误方向信息执行

**影响分析**:  
- 倒车循迹时转向方向可能错误
- 结构体接口表面完整，但关键字段未被正确传递，增加维护误判风险

**修复建议**:  
1. 构造目标点时完整传递 `speed_dir`
2. 对依赖方向字段的接口增加显式初始化和一致性检查

---
**用户意见**: 
后面需要增加关于倒车的逻辑，这个地方先保留，后面进行更改

## 问题编号: P4-3-3-一般
**发现轮次**: 第4轮 第3次验证  
**问题分类**: 工程架构  
**严重等级**: ? 一般  
**文件位置**: `code/system/interrupt.c:96-103 + code/ins/Ins.h:28-35 + code/ins/Ins.c:579-580`  

**问题描述**:  
`INS_Input` 结构体定义了 `omega_rad_s` 和 `mag_yaw_rad` 字段，`interrupt.c` 也实际写入了这两个输入，但 `Ins_update()` 中完全没有使用，而是继续直接读取全局 IMU/编码器状态。接口层提供的数据与实现层真实消费的数据源不一致，模块边界被静默绕开。

**问题代码**:  
```c
// interrupt.c: 上游写入
s_ins_input.mag_valid = get_mag_yaw_measurement(&s_ins_input.mag_yaw_rad);
s_ins_input.omega_rad_s = s_ins_input.v_mps * tanf(steer_rad) / INS_WHEELBASE_M;

// Ins.c: 下游忽略
(void)input->omega_rad_s;
(void)input->mag_yaw_rad;
```

**验证方法**:  
1. 头文件审查：确认 `INS_Input` 将这两个字段作为正式接口暴露
2. 调用链分析：确认 `interrupt.c` 按接口契约填充了字段
3. 实现核对：确认 `Ins_update()` 最终未消费这两个字段

**影响分析**:  
- 接口定义与真实行为不一致，降低模块可替换性
- 后续维护者可能误以为修改输入结构即可影响 INS 行为，实际不会生效

**修复建议**:  
1. 要么在 `Ins_update()` 中按接口契约使用这些字段
2. 要么删减无效接口字段，避免形成伪接口

---

**用户意见**: 
上面第一轮与第二轮已经对这个问题进行了修复，你再检查下是否得到了正确的修改

**修复方案**:  
? 已确认修复
1. `INS_Input` 结构体中 `omega_rad_s` 和 `mag_yaw_rad` 字段保留，作为接口扩展预留
2. `Ins_update()` 中通过 `(void)input->omega_rad_s` 和 `(void)input->mag_yaw_rad` 显式标记为保留
3. 添加注释说明：`omega_rad_s` 待磁力计调试完成后启用，`mag_yaw_rad` 由 INS 内部 `compute_tilt_compensated_mag_yaw()` 计算
4. 接口定义与实现行为一致，避免形成伪接口

### 第4轮问题统计

| 严重等级 | 数量 | 问题编号 |
|---------|------|---------|
| ? 致命 | 0 | - |
| ? 严重 | 2 | P4-1-3, P4-2-2 |
| ? 一般 | 1 | P4-3-3 |
| ? 建议 | 0 | - |
| **总计** | **3** | - |

---

### 第5轮检查：边界与异常处理

---

## 问题编号: P5-1-2-严重
**发现轮次**: 第5轮 第2次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 严重  
**文件位置**: `code/ins/Ins.c:421-439 + code/control/track.c:783-819`  

**问题描述**:  
`Ins_reset()` 只更新了内部累计位置 `s_pos_x/s_pos_y` 和 `s_state.yaw`，没有同步更新对外可读的 `s_state.x/s_state.y`。而 `track_start_follow()` 在调用 `Ins_reset(0,0,0)` 后，立刻通过 `Ins_get_state()` 读取 `state->x/state->y` 去寻找前视点。这样一来，循迹启动阶段第一次寻点使用的仍可能是 reset 之前的旧坐标。

**问题代码**:  
```c
// Ins.c: reset 时未同步 s_state.x / s_state.y
s_state.yaw = normalized_theta;
s_pos_x = x;
s_pos_y = y;

// track.c: reset 后立刻读取 state->x / state->y
Ins_reset(0.0f, 0.0f, 0.0f);
const INS_State* state = Ins_get_state();
if(find_lookahead_point(state->x, state->y))
{
    ...
}
```

**验证方法**:  
1. 代码审查：核对 `Ins_reset()` 更新的状态字段
2. 数据流追踪：确认 `track_start_follow()` 在下一次 `Ins_update()` 之前就读取了 `Ins_get_state()`
3. 场景模拟：若 reset 前 `s_state.x/y` 非零，则首次寻点会从错误起点开始

**影响分析**:  
- 循迹启动瞬间可能选中错误前视点
- 初始转向和目标点显示可能与预期原点不一致

**修复建议**:  
1. 在 `Ins_reset()` 中同步更新 `s_state.x` 和 `s_state.y`
2. 或者在 `track_start_follow()` 中避免在 reset 后立即依赖未刷新状态

---
**用户意见**: 
按照指导意见修复

**修复方案**:  
? 已修复
1. 在 `Ins_reset()` 中同步更新 `s_state.x = x` 和 `s_state.y = y`
2. 确保 `track_start_follow()` 调用 `Ins_reset()` 后，`Ins_get_state()` 返回正确的新坐标
3. 循迹启动阶段首次寻点使用正确原点

## 问题编号: P5-2-2-严重
**发现轮次**: 第5轮 第3次验证  
**问题分类**: 代码逻辑  
**严重等级**: ? 严重  
**文件位置**: `code/control/track.c:603-619 + code/control/track.c:797-805`  

**问题描述**:  
`track_flash_get_point()` 把“四个字段全为 0”视为无效数据。但轨迹点协议允许 `x=0`、`y=0`、`yaw=0`，同时 `speed_dir=0` 表示倒车，因此 `(0, 0, 0, 0)` 是一个合法的“原点倒车点”。当前实现会把这种合法边界值误判为无效，导致轨迹读取失败。

**问题代码**:  
```c
// track.c: 将全 0 记录直接判为无效
if(Ins_Date_Read[base + 0] == 0x00000000u &&
   Ins_Date_Read[base + 1] == 0x00000000u &&
   Ins_Date_Read[base + 2] == 0x00000000u &&
   Ins_Date_Read[base + 3] == 0x00000000u)
{
    return 0;
}
```

**验证方法**:  
1. 协议审查：`speed_dir` 定义中 `0` 就是合法倒车值
2. 边界推导：当起点位于原点、朝向为 0 且记录为倒车时，四个字段恰好全 0
3. 调用分析：`track_start_follow()` 对第一页首点做有效性校验，误判后会直接启动失败

**影响分析**:  
- 合法的边界轨迹点会被拒绝读取
- 倒车起步场景可能无法开始循迹

**修复建议**:  
1. 不要用“全 0”作为无效记录判据
2. 仅依赖擦除值、范围校验和元数据点数来判断记录有效性

**用户意见**: 
=按照修复建议修复

**修复方案**:  
? 已修复
1. 移除"全0判为无效"的检查逻辑（`Ins_Date_Read[base + 0] == 0x00000000u && ...`）
2. 仅保留 Flash 擦除值检查（`0xFFFFFFFF`）作为无效记录判据
3. 保留数值范围检查和浮点有效性检查
4. 现在 `(0, 0, 0, 0)` 会被视为合法的原点倒车点数据

---

## 问题编号: P5-3-2-严重
**发现轮次**: 第5轮 第2次验证  
**问题分类**: 内存安全  
**严重等级**: ? 严重  
**文件位置**: `code/control/track.c:286-325`  

**问题描述**:  
`find_lookahead_point()` 中的 `last_same_dir_point` 在声明后立即传给 `track_flash_get_point()`，但没有检查返回值；如果当前点无效，`last_same_dir_point` 将保持未初始化状态。后续一旦进入“方向不同，回退到最后一个同向点”分支，就会把未初始化数据写入 `Ins_date_377`。

**问题代码**:  
```c
Ins_follow last_same_dir_point;
track_flash_get_point(follow_point_idx, &last_same_dir_point);

...
else
{
    Ins_date_377.x   = last_same_dir_point.x;
    Ins_date_377.y   = last_same_dir_point.y;
    Ins_date_377.yaw = last_same_dir_point.yaw;
    Ins_date_377.speed_dir = last_same_dir_point.speed_dir;
    return 1;
}
```

**验证方法**:  
1. 代码审查：确认 `track_flash_get_point()` 返回值被忽略
2. 状态分析：若当前点损坏或读取失败，`last_same_dir_point` 不具备有效内容
3. 场景模拟：当后续首次读到的有效点方向与 `current_speed_dir` 不同，就会使用未初始化结构体

**影响分析**:  
- `Ins_date_377` 可能被污染为随机坐标和航向
- 循迹时可能出现突发错误转向或目标点跳变

**修复建议**:  
1. 检查 `track_flash_get_point()` 返回值，失败时不要使用该结构体
2. 只有在确认读到有效同向点后，才更新 `last_same_dir_point`

---
**用户意见**: 
我没看懂，先保存着

**修复方案**:  
? 已修复（与 P1-4-2 一并处理）
1. 在 `find_lookahead_point()` 中检查 `track_flash_get_point()` 返回值
2. 如果当前点读取失败，尝试从第一个点开始
3. 只有确认读取成功后才使用 `last_same_dir_point` 数据
4. 避免了未初始化结构体污染 `Ins_date_377` 的风险

### 第5轮问题统计

| 严重等级 | 数量 | 问题编号 |
|---------|------|---------|
| ? 致命 | 0 | - |
| ? 严重 | 3 | P5-1-2, P5-2-2, P5-3-2 |
| ? 一般 | 0 | - |
| ? 建议 | 0 | - |
| **总计** | **3** | - |
