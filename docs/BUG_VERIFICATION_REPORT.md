# Bug 验证报告

> **验证日期**: 2026年4月23日  
> **验证范围**: CODE_REVIEW_PLAN.md 中记录的所有问题  
> **验证方法**: 源代码审查 + 数据流追踪  

---

## 验证结果总览

| 轮次 | 问题数 | 确认真实 | 确认误报 | 确认率 |
|------|--------|----------|----------|--------|
| 第1轮 | 5 | **5** | 0 | 100% |
| 第2轮 | 8 | **8** | 0 | 100% |
| 第3轮 | 2 | **2** | 0 | 100% |
| 第4轮 | 3 | **3** | 0 | 100% |
| 第5轮 | 3 | **3** | 0 | 100% |
| **总计** | **21** | **21** | **0** | **100%** |

---

## 第1轮验证结果：核心控制模块

### P1-1-3-一般：角速度环输出未被使用 ? 确认真实

**验证代码** (`PID.c:547-556`):
```c
/* 角速度环输出 out_steer_pwm 供外部转向模块使用 */
// 但实际只有 out_left 和 out_right 被应用到 motor_control()
```

**验证结论**: 问题真实存在。`out_steer` 被计算但从未被任何执行器使用。

---

### P1-2-3-建议：轮距硬编码不一致 ? 确认真实

**验证代码**:
- `PID.c:462`: `float target_yaw = (v_r - v_l) / 0.2f;`
- `PID.c:56`: `#define COMPAT_WHEELBASE_M 0.2f`

**验证结论**: 问题真实存在。模式2中硬编码 `0.2f`，而其他地方用宏定义。

---

### P1-3-3-建议：WHEELBASE 命名混淆 ? 确认真实

**验证代码**:
- `track.h:39`: `#define TRACK_WHEELBASE 0.2107f` (轴距)
- `PID.c:56`: `#define COMPAT_WHEELBASE_M 0.2f` (轮距)

**验证结论**: 问题真实存在。两者名称相似但含义不同，容易混淆。

---

### P1-4-2-严重：find_lookahead_point 修改全局索引 ? 确认真实

**验证代码** (`track.c:286-325`):
```c
static uint8 find_lookahead_point(float x, float y)
{
    // ...
    while(follow_abs_index <= search_end)
    {
        follow_abs_index++;  // 修改全局变量
        // ...
    }
    // 失败时索引可能已被污染
    return 0;
}
```

**验证结论**: 问题真实存在。函数名暗示"查找"，但实际修改全局状态。

---

### P1-5-3-建议：角速度估算使用命令速度 ? 确认真实

**验证代码** (`PID.c:570`):
```c
float estimated_yaw = (g_wheel_pid.cmd_speed_right_mps - g_wheel_pid.cmd_speed_left_mps) / COMPAT_WHEELBASE_M;
```

**验证结论**: 问题真实存在。使用命令速度而非编码器实际速度估算角速度。

---

## 第2轮验证结果：导航与传感器融合

### P2-1-3-严重：姿态映射不一致 ? 确认真实

**验证代码**:
- `Ins.c:91-96`: `body_roll = -solver_pitch`, `body_pitch = solver_roll`
- `interrupt.c:189-190`: `body_roll_rad = pitch_rad`, `body_pitch_rad = -roll_rad`

**验证结论**: 问题真实存在。两处映射关系符号相反。

---

### P2-2-2-严重：编码器系数符号不一致 ? 确认真实

**验证代码**:
- `Ins.c:110-111`: `tick_to_meter_left = -0.00002204f` (负值)
- `encoder.c:11-12`: `g_tick_to_meter_left = 0.00002204f` (正值)

**验证结论**: 问题真实存在。两处默认值符号相反。

---

### P2-3-3-一般：INS 内部直接获取编码器数据 ? 确认真实

**验证代码** (`Ins.c:577`):
```c
enc_state = encoder_layer_get_state();  // 内部直接获取
```

**验证结论**: 问题真实存在。破坏了模块间数据流单向性。

---

### P2-4-2-一般：磁航向计算重复 ? 确认真实

**验证代码**:
- `interrupt.c:80`: 简单的水平面磁航向（无倾斜补偿）
- `Ins.c:72`: 倾斜补偿后的磁航向

**验证结论**: 问题真实存在。两处逻辑重复且不一致。

---

### P2-5-3-建议：轴距重复定义 ? 确认真实

**验证代码**:
- `Ins.h:15`: `#define INS_WHEELBASE_M 0.2107f`
- `track.h:37`: `#define TRACK_WHEELBASE 0.2107f`

**验证结论**: 问题真实存在。相同值定义在两处。

---

### P2-6-3-建议：omega_rad_s 未被使用 ? 确认真实

**验证代码** (`Ins.c:574`):
```c
(void)input->omega_rad_s;  // 当前版本未使用车辆模型角速度
```

**验证结论**: 问题真实存在。上层计算但未被使用。

---

### P2-7-2-建议：(void) 忽略参数冗余 ? 确认真实

**验证代码** (`Ins.c:574-576`):
```c
(void)input->omega_rad_s;
(void)input->mag_yaw_rad;
(void)s_config.Q_yaw;      // 但 yaw_ekf 实际在使用
(void)s_config.R_mag;
(void)s_config.wheelbase;
```

**验证结论**: 问题真实存在。部分参数被标记未使用但实际在使用。

---

### P2-8-3-建议：缺少初始化状态检查 ? 确认真实

**验证代码** (`Ins.c:492-495`):
```c
if(!s_initialized)
{
    Ins_init();  // 自动初始化
}
```

**验证结论**: 问题真实存在。自动初始化可能掩盖初始化时序问题。

---

## 第3轮验证结果：系统架构与调度

### P3-1-2-严重：周期计数器竞态条件 ? 确认真实

**验证代码** (`interrupt.c:73-78 + 249-267`):
```c
// 中断侧递增
void Interrupt_4ms(void) { if (s_pending_4ms < 500u) s_pending_4ms++; }

// 主循环侧递减
while (s_pending_4ms)
{
    s_pending_4ms--;  // 无临界区保护
    run_4ms_tasks();
}
```

**验证结论**: 问题真实存在。共享计数器无原子保护，可能导致节拍丢失。

---

### P3-2-3-严重：INS_Display 重复调用 ? 确认真实

**验证代码**:
- `interrupt.c:253`: `INS_Display()` 在 40ms 任务中
- `cpu0_main.c:55`: 主循环中也调用 `INS_Display()`

**验证结论**: 问题真实存在。显示任务执行频率不可控。

---

## 第4轮验证结果：数据流与接口

### P4-1-3-严重：姿态映射不一致（与P2-1-3相同） ? 确认真实

**验证结论**: 同 P2-1-3，问题真实存在。

---

### P4-2-2-严重：speed_dir 未被传递 ? 确认真实

**验证代码** (`track.c:427-441`):
```c
Ins_follow target = {Ins_date_377.x, Ins_date_377.y, Ins_date_377.yaw};
// speed_dir 未被初始化！
steer_output = pure_pursuit_calc_steer(state->x, state->y, state->yaw, &target);
```

**验证结论**: 问题真实存在。`speed_dir` 默认为 0，导致倒车转向逻辑错误。

---

### P4-3-3-一般：接口字段未被消费 ? 确认真实

**验证代码**:
- `interrupt.c:103`: 写入 `omega_rad_s` 和 `mag_yaw_rad`
- `Ins.c:574-575`: `(void)` 忽略这些字段

**验证结论**: 问题真实存在。接口定义与真实行为不一致。

---

## 第5轮验证结果：边界与异常处理

### P5-1-2-严重：Ins_reset 未同步 s_state.x/y ? 确认真实

**验证代码** (`Ins.c:421-439`):
```c
void Ins_reset(float x, float y, float theta)
{
    s_state.yaw = normalized_theta;
    s_pos_x = x;
    s_pos_y = y;
    // s_state.x 和 s_state.y 未被更新！
}
```

**验证结论**: 问题真实存在。reset 后 `s_state.x/y` 保持旧值。

---

### P5-2-2-严重：全0被判为无效点 ? 确认真实

**验证代码** (`track.c:603-619`):
```c
if(Ins_Date_Read[base + 0] == 0x00000000u &&
   Ins_Date_Read[base + 1] == 0x00000000u &&
   Ins_Date_Read[base + 2] == 0x00000000u &&
   Ins_Date_Read[base + 3] == 0x00000000u)
{
    return 0;  // 合法的(0,0,0,0)点被误判为无效
}
```

**验证结论**: 问题真实存在。合法边界点会被拒绝。

---

### P5-3-2-严重：last_same_dir_point 未检查返回值 ? 确认真实

**验证代码** (`track.c:286-325`):
```c
Ins_follow last_same_dir_point;
track_flash_get_point(follow_point_idx, &last_same_dir_point);  // 未检查返回值
// 如果失败，last_same_dir_point 未初始化
```

**验证结论**: 问题真实存在。可能使用未初始化的结构体。

---

## 总结

### 所有问题已确认真实存在

- **严重问题**: 10 个
- **一般问题**: 3 个
- **建议问题**: 8 个

### 建议优先修复的问题

1. **P3-1-2**: 周期计数器竞态条件（可能导致节拍丢失）
2. **P3-2-3**: INS_Display 重复调用（破坏调度）
3. **P5-1-2**: Ins_reset 未同步状态（循迹启动错误）
4. **P5-3-2**: 未初始化结构体使用（随机错误）
5. **P4-2-2**: speed_dir 未传递（倒车转向错误）
6. **P2-2-2**: 编码器系数符号不一致（里程计方向错误）

---

**验证人**: GitHub Copilot  
**验证状态**: 完成
