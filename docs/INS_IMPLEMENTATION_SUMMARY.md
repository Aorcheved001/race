# INS 惯性导航系统实现总结

> 项目: TC377TP 智能车 | 版本: v1.0 | 日期: 2026-04-11

---

## 一、整体架构

系统位于 `code/ins/` 目录，采用**三层分层融合架构**：

| 层级 | 模块 | 功能 | 代码位置 |
|------|------|------|----------|
| 第一层 | 6轴卡尔曼滤波 | 姿态估计 (Roll/Pitch/Yaw) | `Ins.c:L134-L205` |
| 第二层 | 2状态 Yaw EKF | 航向融合 (陀螺仪 + 磁力计) | `Ins.c:L244-L311` |
| 第三层 | 差速航位推算 | 位置估计 (编码器 + Yaw) | `Ins.c:L320-L327` |

**GNSS 模块**独立运行，提供局部平面坐标参考，但**未直接参与 INS 融合**。

---

## 二、传感器融合算法

### 2.1 第一层：6轴对角卡尔曼滤波

**输入**：陀螺仪三轴角速度 + 加速度计三轴加速度

**核心设计**：三个状态轴（roll/pitch/yaw）完全独立，各自运行一维 KF。

**预测步骤**：
```c
// 欧拉角微分方程：机体角速度 → 欧拉角变化率
roll_rate  = gyro_x + sin(roll)*tan(pitch)*gyro_y + cos(roll)*tan(pitch)*gyro_z
pitch_rate = cos(roll)*gyro_y - sin(roll)*gyro_z
yaw_rate   = sin(roll)/cos(pitch)*gyro_y + cos(roll)/cos(pitch)*gyro_z

Xk_[i] = Xk[i] + T * Uk[i]     // 欧拉角积分
Pk_[i] = Pk[i] + Q[i]          // 协方差增长
```

**观测模型**（加速度计推算 Roll/Pitch）：
```c
Zk[0] = atan2(acc_y, acc_z)                              // 加速度计roll
Zk[1] = -atan2(acc_x, sqrt(acc_y^2 + acc_z^2))           // 加速度计pitch
Zk[2] = 0.0f                                             // yaw无加速度计观测
```

**更新步骤**：
```c
K[i] = Pk_[i] / (Pk_[i] + R[i])
Xk[i] = (1-K[i]) * Xk_[i] + K[i] * Zk[i]
Pk[i] = (1-K[i]) * Pk_[i]
```

**关键设计**：`K[2] = 0`（Yaw 轴不使用加速度计观测，完全依赖陀螺仪积分）

**参数**：
| 参数 | 值 | 说明 |
|------|-----|------|
| Q | 0.001 | 过程噪声协方差 |
| R | 0.1 | 观测噪声协方差 |
| T | 0.004s | 离散时间（4ms周期） |

### 2.2 第二层：2状态 Yaw EKF

**状态向量**：`x[0] = yaw`（航向角），`x[1] = gyro_bias`（陀螺仪Z轴零偏）

**预测**：
```c
yaw += (gyro_z - bias) * dt
```

过程噪声矩阵（连续时间离散化）：
```
q00 = N_psi * dt + Nb * dt^3 / 3
q01 = -Nb * dt^2 / 2
q11 = Nb * dt
```

**自适应噪声**：
| 状态 | N_b | 说明 |
|------|-----|------|
| 静止 | 5e-7 | 允许bias快速收敛 |
| 运动 | 5e-9 | 冻结bias防止漂移 |

**更新**（磁力计观测）：
- 新息限幅：`max_innovation = 0.052 rad`（3度），抑制磁力计毛刺
- 卡尔曼增益限幅：`|K1| <= 0.15`，防止bias修正过快
- Bias 限幅：`|bias| <= 10 deg/s`，防飞车保护
- 协方差下限：`P[i][i] >= 1e-8`，防止数值退化

### 2.3 第三层：差速航位推算

```c
dist_left  = tick_left  * tick_to_meter_left     // 左轮脉冲转米
dist_right = tick_right * tick_to_meter_right    // 右轮脉冲转米
dist_center = 0.5f * (dist_left + dist_right)    // 中心行进距离
s_pos_x += dist_center * cosf(yaw)               // X轴积分
s_pos_y += dist_center * sinf(yaw)               // Y轴积分
```

**编码器参数**：
| 参数 | 值 |
|------|-----|
| tick_to_meter_left | -0.00002204 m/脉冲 |
| tick_to_meter_right | -0.00002204 m/脉冲 |
| wheelbase（轴距） | 0.2107 m |

---

## 三、坐标系

| 坐标系 | 说明 |
|--------|------|
| **IMU机体坐标系 (Body)** | X/Y/Z 三轴对应传感器物理安装方向 |
| **局部导航坐标系 (Local ENU)** | 位置 x/y 为局部平面坐标（米），原点由 GNSS 首次有效定位点确定 |
| **航向角约定** | yaw 范围 `[-pi, pi]`，磁力计航向 `yaw_mag = -atan2(mag_y, mag_x)` |

**GNSS 经纬度转局部坐标**：
```c
x = (lon - lon0) * cos(lat0) * Re    // Re = 6378137.0m (WGS84)
y = (lat - lat0) * Re
```

---

## 四、关键数据结构

### INS_State -- 导航输出
```c
typedef struct {
    float x;      // X坐标 (米)
    float y;      // Y坐标 (米)
    float yaw;    // 航向角 (弧度)
} INS_State;
```

### INS_Input -- 导航输入
```c
typedef struct {
    float v_mps;           // 线速度 (m/s)
    float omega_rad_s;     // 角速度 (rad/s)
    float gyro_z_rad_s;    // 陀螺仪Z轴 (rad/s)
    float mag_yaw_rad;     // 磁力计航向 (rad)
    uint8_t mag_valid;     // 磁力计有效标志
} INS_Input;
```

### YawEKF2State -- Yaw EKF内部状态
```c
typedef struct {
    float x[2];            // [yaw, bias]
    float P[2][2];         // 2x2 协方差矩阵
    float N_psi;           // yaw角度随机游走 PSD
    float N_b;             // bias过程噪声 (运动时)
    float N_b_frozen;      // bias过程噪声 (静止时)
    float R;               // 磁力计观测噪声
    float yaw_predict;     // 预测值（调试用）
} YawEKF2State;
```

### imu_param -- IMU完整参数
```c
typedef struct {
    float gyro_x/y/z, acc_x/y/z, mag_x/y/z;
    float gyro_x/y/z_bias, acc_x/y/z_bias;
    float mag_x/y/z_bias;           // 硬铁偏置
    float mag_soft_iron[9];         // 3x3 软铁补偿矩阵
} imu_param;
```

---

## 五、磁力计校准系统

### 5.1 校准流程
1. **硬铁补偿**：减去三轴偏置 `mag_x/y/z_bias`
2. **软铁补偿**：乘以 3x3 软铁矩阵 `mag_soft_iron[9]`
3. **椭球拟合**：最小二乘法拟合椭球方程（9参数）
4. **参数持久化**：保存到 Flash 第63页（魔数 `0x4D414742` + 版本号校验）

### 5.2 校准参数导出格式
```c
#define MAG_HARD_IRON_X  32.83f
#define MAG_HARD_IRON_Y  -970.03f
#define MAG_HARD_IRON_Z  -866.07f

float mag_soft_iron[9] = {
    0.996321f, 0.056600f, 0.026942f,
    0.056600f, 0.903278f, 0.099891f,
    0.026942f, 0.099891f, 1.159703f
};
```

---

## 六、滤波方法汇总

| 滤波器 | 位置 | 作用 |
|--------|------|------|
| IMU原始数据低通 | `imu660.c:L560` | alpha = 0.8，平滑传感器噪声 |
| 磁力计航向低通 | `Ins.c:L502` | alpha = 0.3，一阶IIR |
| 6轴对角卡尔曼 | `Ins.c:L134` | Roll/Pitch/Yaw 姿态估计 |
| 2状态 Yaw EKF | `Ins.c:L244` | 航向融合 + 陀螺仪零偏估计 |

---

## 七、零速检测 (ZUPT)

```c
if (|v_mps| < 0.05 m/s  &&  |gyro_z| < 0.05 rad/s) → 判定为静止
```

静止时将编码器脉冲强制置零，防止静止漂移。

---

## 八、数据流

```
[IMU963RA传感器]
    │
    ▼
date_handle() ──→ 偏置补偿 + 低通滤波(alpha=0.8) + 磁力计软铁校正
    │
    ▼
imu660.data_Ripen ──→ 成熟的加速度/角速度/磁力计数据
    │
    ├──→ ins_kalman_6axis_update() ──→ Roll, Pitch, Yaw_gyro
    │         │
    │         └──→ 重力补偿 → 线性加速度
    │
    ├──→ yaw_ekf2_predict(gyro_z, dt, is_stationary)
    │         │
    │         └──→ 预测 yaw 和 bias
    │
    └──→ (若 mag_valid)
              │
              ├──→ 计算相对磁航向 (带初始捕获对齐)
              ├──→ 一阶低通滤波 (alpha=0.3)
              └──→ yaw_ekf2_update(mag_yaw_filtered)
                        │
                        └──→ 最终融合 Yaw
                              │
                              ▼
                    ins_position_update(yaw, encoder_ticks)
                              │
                              ▼
                    s_state = {x, y, yaw}  ← 最终输出
```

---

## 九、系统评价

### 优点
- 分层架构清晰，各层职责分明
- Yaw EKF 设计精良：自适应噪声、新息限幅、bias限幅、协方差下限等工程保护措施完善
- 磁力计校准系统完整（椭球拟合 + Flash持久化）
- 相对磁航向设计避免了上电初始朝向跳变问题
- ZUPT 零速检测有效抑制静止漂移

### 局限
- GNSS 未与 INS 融合，仅作为独立位置参考
- 6轴卡尔曼采用对角简化形式，未建模轴间耦合
- 位置估计完全依赖编码器航位推算，无外部位置观测校正
- 使用欧拉角而非四元数，存在万向节死锁风险（对地面小车场景影响有限）
