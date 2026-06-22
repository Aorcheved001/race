# 新INS惯性导航系统设计方案（二维平面版）

## 一、系统架构概述

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    二维平面INS惯性导航系统架构                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐                │
│   │   陀螺仪      │    │   磁力计      │    │   编码器      │                │
│   │   (GYRO)      │    │   (MAG)       │    │   (ODO)       │                │
│   │   加速度计    │    │               │    │               │                │
│   │   (ACC)       │    │               │    │               │                │
│   └──────┬───────┘    └──────┬───────┘    └──────┬───────┘                │
│          │                     │                     │                      │
│          ▼                     │                     │                      │
│   ┌─────────────────────┐      │                     │                      │
│   │  第一层：六轴卡尔曼  │      │                     │                      │
│   │  陀螺仪+加速度计    │      │                     │                      │
│   │  输出: yaw_gyro     │      │                     │                      │
│   │  (短期高精度，会漂移)│      │                     │                      │
│   └──────────┬──────────┘      │                     │                      │
│              │                  │                     │                      │
│              ▼                  ▼                     │                      │
│   ┌─────────────────────────────────────┐            │                      │
│   │      第二层：磁力计yaw校正          │            │                      │
│   │   yaw_gyro + 磁力计 → yaw_stable   │            │                      │
│   │   (长期无漂移，稳定航向)            │            │                      │
│   └──────────────────┬──────────────────┘            │                      │
│                      │                                │                      │
│                      ▼                                ▼                      │
│              ┌──────────────────────────────────────────────┐              │
│              │           第三层：位置推算                   │              │
│              │   yaw_stable + 编码器 → x, y                │              │
│              └──────────────────────────────────────────────┘              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 二、三阶段详细设计

### 2.1 第一层：六轴卡尔曼滤波（陀螺仪+加速度计）

**目标**：融合陀螺仪和加速度计，得到初步yaw角

**输入**：
- `gyro_x, gyro_y, gyro_z` - 陀螺仪三轴角速度
- `acc_x, acc_y, acc_z` - 加速度计三轴加速度

**输出**：
- `roll, pitch` - 横滚角、俯仰角（用于重力补偿）
- `yaw_gyro` - 航向角（会漂移）

**核心代码**：

```c
// imu963ra结构体
typedef struct {
    // 欧拉角
    float roll;
    float pitch;
    float yaw;
    
    // 角速度
    float gx, gy, gz;
    
    // 加速度
    float ax, ay, az;
    
    // 磁力计
    float mx, my, mz;
    
    // 卡尔曼滤波变量
    float Xk_[3];      // 先验估计
    float Xk[3];       // 后验估计
    float Uk[3];       // 系统输入
    float Zk[3];       // 测量状态
    float Pk[3];       // 后验估计误差协方差
    float Pk_[3];      // 先验估计误差协方差
    float K[3];        // 卡尔曼增益
    float Q[3];        // 系统噪声协方差
    float R[3];        // 测量噪声协方差
    float T;           // 离散时间
    
    // 重力补偿后的线性加速度
    float ax_linear;
    float ay_linear;
    float az_linear;
    
    float resultant_acceleration;
} imu963ra_struct;

// 初始化
void imu963ra_kalman_filter_init(imu963ra_struct *imu, float q, float r, float T);

// 更新（周期调用）
void imu963ra_kalman_filter_update(imu963ra_struct *imu);
```

**算法说明**：
1. 陀螺仪积分得到姿态角预测
2. 加速度计计算roll和pitch测量值
3. 卡尔曼滤波融合，输出稳定的roll和pitch
4. yaw仅由陀螺仪积分得到，会漂移

---

### 2.2 第二层：磁力计yaw校正

**目标**：使用磁力计约束yaw长期漂移

**输入**：
- `yaw_gyro` - 六轴卡尔曼输出的yaw（短期精度高）
- `mag_x, mag_y` - 磁力计X, Y轴数据

**输出**：
- `yaw_stable` - 融合后稳定yaw

**算法选择**：

#### 方案一：互补滤波（推荐，简单高效）

```c
// 1. 磁力计计算yaw
float yaw_mag = atan2f(mag_y, mag_x);

// 2. 互补滤波融合
float alpha = 0.98f;  // 信任陀螺仪程度
float yaw_diff = normalize_angle(yaw_mag - yaw_gyro);
float yaw_stable = yaw_gyro + (1.0f - alpha) * yaw_diff;
yaw_stable = normalize_angle(yaw_stable);
```

#### 方案二：卡尔曼滤波（更精确）

```c
// 状态: yaw
// 预测: yaw_pred = yaw_gyro
// 更新: yaw = yaw_pred + K * (yaw_mag - yaw_pred)
// K = P / (P + R_mag)
```

---

### 2.3 第三层：位置推算

**目标**：结合稳定yaw和编码器，推算x, y位置

**输入**：
- `yaw_stable` - 稳定航向角
- `tick_left, tick_right` - 左右轮编码器脉冲

**输出**：
- `x, y` - 车辆位置

**算法**：

```c
// 1. 计算左右轮位移
float dist_left = tick_left * tick_to_meter_left;
float dist_right = tick_right * tick_to_meter_right;

// 2. 计算中心位移
float dist = (dist_left + dist_right) / 2.0f;

// 3. 更新位置
x += dist * cosf(yaw_stable);
y += dist * sinf(yaw_stable);
```

---

## 三、数据流图

```
传感器数据
├── GYRO (x,y,z) ──────────────────┐
├── ACC (x,y,z) ───────────────────┤
│                                  ▼
│                    ┌─────────────────────┐
│                    │  第一层：六轴卡尔曼  │
│                    │  陀螺仪+加速度计    │
│                    │                     │
│                    │  输出: roll, pitch  │
│                    │        yaw_gyro     │
│                    └──────────┬──────────┘
│                               │
│                               ▼
├── MAG (x,y) ──────────────────┤
│                               ▼
│                    ┌─────────────────────┐
│                    │  第二层：磁力计校正  │
│                    │  互补滤波/卡尔曼    │
│                    │                     │
│                    │  输出: yaw_stable   │
│                    └──────────┬──────────┘
│                               │
│                               ▼
├── ENCODER (L,R) ──────────────┤
│                               ▼
│                    ┌─────────────────────┐
│                    │  第三层：位置推算    │
│                    │  运动学模型         │
│                    │                     │
│                    │  输出: x, y         │
│                    └─────────────────────┘
```

---

## 四、文件结构设计

```
code/ins/
├── ins_main.h              // INS主模块头文件（接口与原系统一致）
├── ins_main.c              // INS主模块实现
├── ins_kalman_6axis.h      // 六轴卡尔曼滤波头文件
├── ins_kalman_6axis.c      // 六轴卡尔曼滤波实现
├── ins_mag_correct.h       // 磁力计校正模块头文件
├── ins_mag_correct.c       // 磁力计校正模块实现
├── ins_position.h          // 位置推算模块头文件
├── ins_position.c          // 位置推算模块实现
├── imu660.h                // IMU驱动头文件
└── imu660.c                // IMU驱动实现
```

---

## 五、核心结构体设计

### 5.1 六轴卡尔曼滤波结构体

```c
// ins_kalman_6axis.h

typedef struct {
    // 欧拉角（弧度）
    float roll;
    float pitch;
    float yaw_gyro;          // 六轴融合后的yaw（会漂移）
    
    // 角速度（rad/s）
    float gx, gy, gz;
    
    // 加速度（m/s?）
    float ax, ay, az;
    
    // 磁力计
    float mx, my, mz;
    
    // 卡尔曼滤波变量
    float Xk_[3];            // 先验估计
    float Xk[3];             // 后验估计
    float Uk[3];             // 系统输入
    float Zk[3];             // 测量状态
    float Pk[3];             // 后验估计误差协方差
    float Pk_[3];            // 先验估计误差协方差
    float K[3];              // 卡尔曼增益
    float Q[3];              // 系统噪声协方差
    float R[3];              // 测量噪声协方差
    float T;                 // 离散时间
    
    // 重力补偿后的线性加速度
    float ax_linear;
    float ay_linear;
    float az_linear;
    
    float resultant_acceleration;
} INS_Kalman6Axis;
```

### 5.2 INS主数据结构（与原系统接口一致）

```c
// ins_main.h

// 状态结构（与原系统一致）
typedef struct {
    float x;                   // X坐标（米）
    float y;                   // Y坐标（米）
    float yaw;                 // 航向角（弧度）- 稳定yaw
} INS_State;

// 输入结构（与原系统一致）
typedef struct {
    float v_mps;               // 线速度（米/秒）
    float omega_rad_s;         // 角速度（弧度/秒）
    float gyro_z_rad_s;        // 陀螺仪Z轴角速度（弧度/秒）
    float mag_yaw_rad;         // 磁力计航向角（弧度）
    uint8_t mag_valid;         // 磁力计数据有效性标志
} INS_Input;

// 配置结构
typedef struct {
    // 六轴卡尔曼参数
    float kalman_6axis_q;      // 系统噪声协方差
    float kalman_6axis_r;      // 测量噪声协方差
    float kalman_6axis_T;      // 离散时间
    
    // 磁力计校正参数
    float mag_alpha;           // 互补滤波系数 (0.95~0.99)
    float Q_yaw;               // yaw过程噪声
    float R_mag;               // 磁力计测量噪声
    
    // 位置推算参数
    float wheelbase;           // 轴距
    float tick_to_meter_left;  // 左轮编码器系数
    float tick_to_meter_right; // 右轮编码器系数
    
    // 零速修正参数
    float zupt_speed_threshold; // 零速速度阈值
    float zupt_gyro_threshold;  // 零速陀螺仪阈值
} INS_Config;
```

---

## 六、API接口设计（与原系统一致）

```c
// ins_main.h

//-------------------------------------------函数声明区------------------------------------------------------------
void Ins_init(void);                                                     // 初始化INS系统

void Ins_reset(float x, float y, float theta);                           // 重置INS状态

void Ins_set_config(const INS_Config *config);                           // 设置INS配置参数

void Ins_update(const INS_Input *input, float dt_s);                     // 更新INS状态

const INS_State* Ins_get_state(void);                                    // 获取当前INS状态
```

---

## 七、核心算法实现

### 7.1 六轴卡尔曼滤波

```c
// ins_kalman_6axis.c

static INS_Kalman6Axis s_kalman_6axis = {0};

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      六轴卡尔曼滤波初始化
 ////  @param      q             系统噪声协方差
 ////  @param      r             测量噪声协方差
 ////  @param      T             离散时间
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void INS_Kalman6Axis_Init(float q, float r, float T)
{
    s_kalman_6axis.roll = 0.0f;
    s_kalman_6axis.pitch = 0.0f;
    s_kalman_6axis.yaw_gyro = 0.0f;
    
    for(int i = 0; i < 3; i++)
    {
        s_kalman_6axis.Xk_[i] = 0.0f;
        s_kalman_6axis.Xk[i] = 0.0f;
        s_kalman_6axis.Uk[i] = 0.0f;
        s_kalman_6axis.Zk[i] = 0.0f;
        s_kalman_6axis.Pk[i] = 1.0f;
        s_kalman_6axis.Pk_[i] = 0.0f;
        s_kalman_6axis.K[i] = 0.0f;
        s_kalman_6axis.Q[i] = q;
        s_kalman_6axis.R[i] = r;
    }
    
    s_kalman_6axis.T = T;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      六轴卡尔曼滤波更新
 ////  @param      gyro_x, gyro_y, gyro_z   陀螺仪三轴角速度（rad/s）
 ////  @param      acc_x, acc_y, acc_z      加速度计三轴加速度（m/s?）
 ////  @return     void
 ////  @note       输出roll, pitch, yaw_gyro
 ////-------------------------------------------------------------------------------------------------------------------
void INS_Kalman6Axis_Update(float gyro_x, float gyro_y, float gyro_z, 
                            float acc_x, float acc_y, float acc_z)
{
    s_kalman_6axis.gx = gyro_x;
    s_kalman_6axis.gy = gyro_y;
    s_kalman_6axis.gz = gyro_z;
    s_kalman_6axis.ax = acc_x;
    s_kalman_6axis.ay = acc_y;
    s_kalman_6axis.az = acc_z;
    
    // 计算合加速度
    s_kalman_6axis.resultant_acceleration = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
    
    // step1 - 系统输入（姿态角速度）
    s_kalman_6axis.Uk[0] = gyro_x + sinf(s_kalman_6axis.Xk[0]) * tanf(s_kalman_6axis.Xk[1]) * gyro_y 
                          + cosf(s_kalman_6axis.Xk[0]) * tanf(s_kalman_6axis.Xk[1]) * gyro_z;
    s_kalman_6axis.Uk[1] = cosf(s_kalman_6axis.Xk[0]) * gyro_y - sinf(s_kalman_6axis.Xk[0]) * gyro_z;
    s_kalman_6axis.Uk[2] = sinf(s_kalman_6axis.Xk[0]) * gyro_y / cosf(s_kalman_6axis.Xk[1]) 
                          + cosf(s_kalman_6axis.Xk[0]) * gyro_z / cosf(s_kalman_6axis.Xk[1]);
    
    // step2 - 先验估计
    s_kalman_6axis.Xk_[0] = s_kalman_6axis.Xk[0] + s_kalman_6axis.T * s_kalman_6axis.Uk[0];
    s_kalman_6axis.Xk_[1] = s_kalman_6axis.Xk[1] + s_kalman_6axis.T * s_kalman_6axis.Uk[1];
    s_kalman_6axis.Xk_[2] = s_kalman_6axis.Xk[2] + s_kalman_6axis.T * s_kalman_6axis.Uk[2];
    
    // step3 - 先验估计误差协方差
    s_kalman_6axis.Pk_[0] = s_kalman_6axis.Pk[0] + s_kalman_6axis.Q[0];
    s_kalman_6axis.Pk_[1] = s_kalman_6axis.Pk[1] + s_kalman_6axis.Q[1];
    s_kalman_6axis.Pk_[2] = s_kalman_6axis.Pk[2] + s_kalman_6axis.Q[2];
    
    // step4 - 卡尔曼增益
    s_kalman_6axis.K[0] = s_kalman_6axis.Pk_[0] / (s_kalman_6axis.Pk_[0] + s_kalman_6axis.R[0]);
    s_kalman_6axis.K[1] = s_kalman_6axis.Pk_[1] / (s_kalman_6axis.Pk_[1] + s_kalman_6axis.R[1]);
    s_kalman_6axis.K[2] = 0.0f;  // yaw无测量，不更新
    
    // step5 - 测量数据（从加速度计计算roll和pitch）
    s_kalman_6axis.Zk[0] = atan2f(acc_y, acc_z);
    s_kalman_6axis.Zk[1] = -atan2f(acc_x, sqrtf(acc_y*acc_y + acc_z*acc_z));
    s_kalman_6axis.Zk[2] = 0.0f;
    
    // step6 - 后验估计
    s_kalman_6axis.Xk[0] = (1.0f - s_kalman_6axis.K[0]) * s_kalman_6axis.Xk_[0] + s_kalman_6axis.K[0] * s_kalman_6axis.Zk[0];
    s_kalman_6axis.Xk[1] = (1.0f - s_kalman_6axis.K[1]) * s_kalman_6axis.Xk_[1] + s_kalman_6axis.K[1] * s_kalman_6axis.Zk[1];
    s_kalman_6axis.Xk[2] = s_kalman_6axis.Xk_[2];  // yaw仅积分
    
    // step7 - 后验估计误差协方差
    s_kalman_6axis.Pk[0] = (1.0f - s_kalman_6axis.K[0]) * s_kalman_6axis.Pk_[0];
    s_kalman_6axis.Pk[1] = (1.0f - s_kalman_6axis.K[1]) * s_kalman_6axis.Pk_[1];
    s_kalman_6axis.Pk[2] = s_kalman_6axis.Pk_[2];
    
    // 输出欧拉角
    s_kalman_6axis.roll = s_kalman_6axis.Xk[0];
    s_kalman_6axis.pitch = s_kalman_6axis.Xk[1];
    s_kalman_6axis.yaw_gyro = s_kalman_6axis.Xk[2];
    
    // 重力补偿
    float sin_roll = sinf(s_kalman_6axis.roll);
    float cos_roll = cosf(s_kalman_6axis.roll);
    float sin_pitch = sinf(s_kalman_6axis.pitch);
    float cos_pitch = cosf(s_kalman_6axis.pitch);
    
    float gx_body = -9.80665f * sin_pitch;
    float gy_body = 9.80665f * sin_roll * cos_pitch;
    float gz_body = 9.80665f * cos_roll * cos_pitch;
    
    s_kalman_6axis.ax_linear = acc_x - gx_body;
    s_kalman_6axis.ay_linear = acc_y - gy_body;
    s_kalman_6axis.az_linear = acc_z - gz_body;
}

float INS_Kalman6Axis_GetYawGyro(void)
{
    return s_kalman_6axis.yaw_gyro;
}

float INS_Kalman6Axis_GetRoll(void)
{
    return s_kalman_6axis.roll;
}

float INS_Kalman6Axis_GetPitch(void)
{
    return s_kalman_6axis.pitch;
}
```

### 7.2 磁力计yaw校正

```c
// ins_mag_correct.c

static float s_yaw_stable = 0.0f;
static float s_mag_alpha = 0.98f;

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      磁力计yaw校正初始化
 ////  @param      alpha         互补滤波系数
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void INS_MagCorrect_Init(float alpha)
{
    s_mag_alpha = alpha;
    s_yaw_stable = 0.0f;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      磁力计yaw校正更新
 ////  @param      yaw_gyro      六轴卡尔曼输出的yaw（弧度）
 ////  @param      mag_x         磁力计X轴
 ////  @param      mag_y         磁力计Y轴
 ////  @return     float         校正后的稳定yaw（弧度）
 ////-------------------------------------------------------------------------------------------------------------------
float INS_MagCorrect_Update(float yaw_gyro, float mag_x, float mag_y)
{
    // 1. 磁力计计算yaw
    float yaw_mag = atan2f(mag_y, mag_x);
    
    // 2. 角度差归一化
    float yaw_diff = yaw_mag - yaw_gyro;
    while(yaw_diff > 3.14159265f) yaw_diff -= 2.0f * 3.14159265f;
    while(yaw_diff < -3.14159265f) yaw_diff += 2.0f * 3.14159265f;
    
    // 3. 互补滤波融合
    s_yaw_stable = yaw_gyro + (1.0f - s_mag_alpha) * yaw_diff;
    
    // 4. 角度归一化
    while(s_yaw_stable > 3.14159265f) s_yaw_stable -= 2.0f * 3.14159265f;
    while(s_yaw_stable < -3.14159265f) s_yaw_stable += 2.0f * 3.14159265f;
    
    return s_yaw_stable;
}

float INS_MagCorrect_GetYawStable(void)
{
    return s_yaw_stable;
}
```

### 7.3 位置推算

```c
// ins_position.c

static float s_x = 0.0f;
static float s_y = 0.0f;
static float s_tick_to_meter_left = 0.0001f;
static float s_tick_to_meter_right = 0.0001f;

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      位置推算初始化
 ////  @param      tick_to_meter_left   左轮编码器系数
 ////  @param      tick_to_meter_right  右轮编码器系数
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void INS_Position_Init(float tick_to_meter_left, float tick_to_meter_right)
{
    s_tick_to_meter_left = tick_to_meter_left;
    s_tick_to_meter_right = tick_to_meter_right;
    s_x = 0.0f;
    s_y = 0.0f;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      位置推算重置
 ////  @param      x             初始X坐标
 ////  @param      y             初始Y坐标
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void INS_Position_Reset(float x, float y)
{
    s_x = x;
    s_y = y;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      位置推算更新
 ////  @param      yaw           航向角（弧度）
 ////  @param      tick_left     左轮编码器脉冲
 ////  @param      tick_right    右轮编码器脉冲
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void INS_Position_Update(float yaw, int16_t tick_left, int16_t tick_right)
{
    // 1. 计算左右轮位移
    float dist_left = tick_left * s_tick_to_meter_left;
    float dist_right = tick_right * s_tick_to_meter_right;
    
    // 2. 计算中心位移
    float dist = (dist_left + dist_right) / 2.0f;
    
    // 3. 更新位置
    s_x += dist * cosf(yaw);
    s_y += dist * sinf(yaw);
}

void INS_Position_GetPosition(float *x, float *y)
{
    *x = s_x;
    *y = s_y;
}
```

### 7.4 INS主模块（整合各模块）

```c
// ins_main.c

#include "ins_main.h"
#include "ins_kalman_6axis.h"
#include "ins_mag_correct.h"
#include "ins_position.h"
#include "imu660.h"

 //-------------------------------------------内部变量区------------------------------------------------------------
static INS_State s_state = {0};
static INS_Config s_config = {0};
static uint8_t s_initialized = 0;

 //-------------------------------------------函数定义区------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      初始化INS系统
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_init(void)
{
    // 初始化六轴卡尔曼
    INS_Kalman6Axis_Init(s_config.kalman_6axis_q, s_config.kalman_6axis_r, s_config.kalman_6axis_T);
    
    // 初始化磁力计校正
    INS_MagCorrect_Init(s_config.mag_alpha);
    
    // 初始化位置推算
    INS_Position_Init(s_config.tick_to_meter_left, s_config.tick_to_meter_right);
    
    // 初始化状态
    s_state.x = 0.0f;
    s_state.y = 0.0f;
    s_state.yaw = 0.0f;
    
    s_initialized = 1;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      重置INS状态
 ////  @param      x             初始X坐标
 ////  @param      y             初始Y坐标
 ////  @param      theta         初始航向角（弧度）
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_reset(float x, float y, float theta)
{
    s_state.x = x;
    s_state.y = y;
    s_state.yaw = theta;
    
    INS_Position_Reset(x, y);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      设置INS配置参数
 ////  @param      config        配置参数指针
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_set_config(const INS_Config *config)
{
    if(config == NULL) return;
    
    s_config = *config;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      更新INS状态
 ////  @param      input         输入数据指针
 ////  @param      dt_s          时间步长（秒）
 ////  @return     void
 ////  @note       周期调用（如4ms）
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_update(const INS_Input *input, float dt_s)
{
    if(!s_initialized) Ins_init();
    if(input == NULL || dt_s <= 0.0f) return;
    
    // 第一层：六轴卡尔曼滤波
    INS_Kalman6Axis_Update(input->gyro_z_rad_s, 0.0f, 0.0f, 
                           imu660.data_Ripen.acc_x, 
                           imu660.data_Ripen.acc_y, 
                           imu660.data_Ripen.acc_z);
    float yaw_gyro = INS_Kalman6Axis_GetYawGyro();
    
    // 第二层：磁力计yaw校正
    float yaw_stable = yaw_gyro;
    if(input->mag_valid)
    {
        yaw_stable = INS_MagCorrect_Update(yaw_gyro, 
                                           imu660.data_Ripen.mag_x, 
                                           imu660.data_Ripen.mag_y);
    }
    s_state.yaw = yaw_stable;
    
    // 第三层：位置推算（在interrupt.c中通过编码器更新）
    // 这里只更新yaw，位置由外部调用INS_Position_Update
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      获取当前INS状态
 ////  @param      void
 ////  @return     INS_State*    状态指针
 ////-------------------------------------------------------------------------------------------------------------------
const INS_State* Ins_get_state(void)
{
    return &s_state;
}
```

---

## 八、参数调优指南

### 8.1 六轴卡尔曼参数

| 参数 | 默认值 | 作用 | 调整建议 |
|------|--------|------|----------|
| kalman_6axis_q | 0.001 | 系统噪声协方差 | 越小越信任模型 |
| kalman_6axis_r | 0.1 | 测量噪声协方差 | 越小越信任加速度计 |
| kalman_6axis_T | 0.004 | 离散时间 | 根据采样周期设置 |

### 8.2 磁力计校正参数

| 参数 | 默认值 | 作用 | 调整建议 |
|------|--------|------|----------|
| mag_alpha | 0.98 | 互补滤波系数 | 越大越信任陀螺仪(0.95~0.99) |

### 8.3 位置推算参数

| 参数 | 默认值 | 作用 | 调整建议 |
|------|--------|------|----------|
| tick_to_meter | - | 编码器系数 | 根据编码器参数计算 |

---

## 九、与原系统接口对比

| 原系统接口 | 新系统接口 | 说明 |
|------------|------------|------|
| `Ins_init()` | `Ins_init()` | 一致 |
| `Ins_reset(x, y, theta)` | `Ins_reset(x, y, theta)` | 一致 |
| `Ins_set_config(config)` | `Ins_set_config(config)` | 一致 |
| `Ins_update(input, dt)` | `Ins_update(input, dt)` | 一致 |
| `Ins_get_state()` | `Ins_get_state()` | 一致 |
| `INS_State` | `INS_State` | 一致 |
| `INS_Input` | `INS_Input` | 一致 |
| `INS_Config` | `INS_Config` | 扩展（增加六轴卡尔曼参数） |

---

## 十、实现计划

### 阶段一：六轴卡尔曼模块（1天）

1. 实现 `ins_kalman_6axis.c/h`
2. 移植用户提供的卡尔曼滤波代码
3. 单元测试

### 阶段二：磁力计校正模块（0.5天）

1. 实现 `ins_mag_correct.c/h`
2. 实现互补滤波融合
3. 单元测试

### 阶段三：位置推算模块（0.5天）

1. 实现 `ins_position.c/h`
2. 单元测试

### 阶段四：整合测试（1天）

1. 实现 `ins_main.c/h`
2. 整合各模块
3. 替换原Ins.c/h
4. 整体测试
5. 参数调优

---

## 十一、预期效果

| 指标 | 目标值 |
|------|--------|
| yaw漂移 | < 1°/分钟 |
| 位置精度 | 10米轨迹误差 < 0.3米 |
| 响应速度 | 收敛时间 < 1秒 |
| 实时性 | 4ms周期更新 |
| 接口兼容 | 100%兼容原系统 |
