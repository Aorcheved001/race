/*
 * Ins.c
 * INS 核心状态估计与航向融合
 */

#include "zf_common_headfile.h"
#include "mag_yaw_lut.h"

//-------------------------------------------内部状态结构------------------------------------------------------------
typedef struct
{
    float roll;                                                          // 六轴姿态解算 roll
    float pitch;                                                         // 六轴姿态解算 pitch
    float yaw_gyro;                                                      // 纯陀螺积分得到的 yaw
    float Xk_[3];                                                        // 预测状态
    float Xk[3];                                                         // 当前状态
    float Uk[3];                                                         // 状态输入
    float Zk[3];                                                         // 测量量
    float Pk[3];                                                         // 当前协方差
    float Pk_[3];                                                        // 预测协方差
    float K[3];                                                          // 卡尔曼增益
    float Q[3];                                                          // 过程噪声
    float R[3];                                                          // 测量噪声
    float T;                                                             // 采样周期
    float ax_linear;                                                     // 去重力后的 X 向线加速度
    float ay_linear;                                                     // 去重力后的 Y 向线加速度
    float az_linear;                                                     // 去重力后的 Z 向线加速度
} INS_Kalman6Axis;

//-------------------------------------------模块静态变量------------------------------------------------------------
static INS_State s_state = {0};                                          // INS 当前状态
static INS_Config s_config = {0};                                        // INS 配置参数
static INS_Kalman6Axis s_kalman_6axis = {0};                             // 六轴姿态解算器
static YawEKF2State s_yaw_ekf = {0};                                     // 航向 EKF 状态（保留结构，不再用于最终 yaw）
static float s_pos_x = 0.0f;                                             // 累计位置 X
static float s_pos_y = 0.0f;                                             // 累计位置 Y
static uint8_t s_initialized = 0u;                                       // 初始化标志

// 磁航向相对零点（保留变量，磁力计已弃用）
static float s_initial_mag_yaw = 0.0f;                                   // 初始磁航向零点
static uint8_t s_mag_initial_yaw_captured = 0u;                          // 初始磁航向是否已记录
static float s_mag_yaw_raw = 0.0f;                                       // 倾斜补偿后的原始磁航向
static float s_mag_yaw_rel = 0.0f;                                       // 相对初始方向的磁航向
static float s_mag_yaw_rel_filtered = 0.0f;                              // 相对磁航向低通输出

// 磁航向 LUT 修正层
static float s_mag_yaw_corr_raw = 0.0f;                                  // LUT修正后的原始磁航向(弧度)
static float s_mag_yaw_corr_rel = 0.0f;                                  // LUT修正后的相对磁航向(弧度)
static float s_mag_yaw_corr_rel_filtered = 0.0f;                         // LUT修正后相对磁航向低通输出
static float s_initial_mag_yaw_corr = 0.0f;                              // LUT修正后的初始磁航向零点
static uint8_t s_mag_yaw_corr_initial_captured = 0u;                     // LUT修正后初始磁航向是否已记录
static uint8_t s_mag_fusion_enabled = 0u;                                // 磁航向是否参与最终yaw融合，默认关闭

// 静止零偏校准（放弃磁力计方案）
static uint8_t s_bias_calib_active = 0u;                                 // 零偏校准进行中标志
static float s_bias_calib_start_yaw = 0.0f;                              // 校准时记录的起始 yaw（6轴陀螺积分）
static float s_bias_calib_elapsed = 0.0f;                                // 校准累计静止时长
static float s_calibrated_bias_rad_s = 0.0f;                             // 校准得到的零偏 (rad/s)
static float s_bias_corrected_yaw = 0.0f;                                // 零偏修正后的 yaw（最终使用）
static float s_rp_bias_corrected_yaw = 0.0f;                             // roll/pitch补偿角速度积分后的 yaw（仅调试对比）
static float s_model_yaw = 0.0f;                                        // 车辆运动模型角速度积分 yaw（仅调试对比）
static float s_last_rp_yaw_rate = 0.0f;                                  // 最近一次 roll/pitch 补偿后的 yaw rate
static uint8_t s_position_frozen = 0u;                                   // 位置冻结标志（抬车时禁止编码器积分）

//-------------------------------------------内部工具函数------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
////  @brief      将角度归一化到 [-PI, PI]
////  @param      angle       输入角度
////  @return     归一化后的角度
 ////-------------------------------------------------------------------------------------------------------------------
static float normalize_angle_rad(float angle)
{
    while(angle > INS_PI) angle -= 2.0f * INS_PI;                        // 大于 PI 时减去 2PI
    while(angle < -INS_PI) angle += 2.0f * INS_PI;                       // 小于 -PI 时加上 2PI
    return angle;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      计算倾斜补偿后的磁航向
////  @param      mag_x       磁力计 X 轴
////  @param      mag_y       磁力计 Y 轴
////  @param      mag_z       磁力计 Z 轴
////  @param      roll        参与补偿的 roll
////  @param      pitch       参与补偿的 pitch
////  @return     float       倾斜补偿后的磁航向
////  @note       使用当前姿态把磁场投影回水平面后再计算 yaw
 ////-------------------------------------------------------------------------------------------------------------------
static float compute_tilt_compensated_mag_yaw(float mag_x, float mag_y, float mag_z, float roll, float pitch)
{
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);
    float mag_x_horizontal = 0.0f;
    float mag_y_horizontal = 0.0f;

    mag_x_horizontal = mag_x * cos_pitch + mag_z * sin_pitch;
    mag_y_horizontal = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;

    return normalize_angle_rad(-atan2f(mag_y_horizontal, mag_x_horizontal));
}

static void map_body_attitude_for_mag_compensation(float solver_roll,
                                                   float solver_pitch,
                                                   float *body_roll,
                                                   float *body_pitch)
{
    if(body_roll != NULL)
    {
        *body_roll = -solver_pitch;
    }

    if(body_pitch != NULL)
    {
        *body_pitch = solver_roll;
    }
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      初始化 INS 配置参数
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void Init_config(void)
{
    s_config.kalman_6axis_q = 0.001f;                                    // 六轴姿态过程噪声 (P7-3: 保持不变，陀螺预测噪声合理)
    s_config.kalman_6axis_r = 0.5f;                                       // 六轴姿态测量噪声基准值 (P7-4: 动态R方案, 基准R=0.5保证静态性能, 转弯时自动放大)
    s_config.kalman_6axis_T = 0.004f;                                    // 4ms 采样周期

    // Round4 调参：旋转时跳过磁力计更新，只在静止/低速时用磁力计修正漂移
    s_config.Q_yaw = 1e-6f;                                              // EKF yaw 过程噪声
    s_config.R_mag = 1e-2f;                                              // EKF 磁观测噪声
    s_config.mag_alpha = 0.3f;                                           // 磁力计低通滤波系数

    s_config.wheelbase = INS_WHEELBASE_M;                                // 轴距
    s_config.tick_to_meter_left = 0.00036816f;                           // 左轮编码器脉冲转距离 [P7-2-1] 与encoder层同步
    s_config.tick_to_meter_right = -0.00036816f;                         // 右轮编码器脉冲转距离 [P7-2-1] 与encoder层同步
    s_config.zupt_speed_threshold = 0.05f;                               // 静止速度阈值
    s_config.zupt_gyro_threshold = 0.05f;                                // 静止角速度阈值
    s_config.mag_alpha = 0.3f;                                           // [P7-2-2] 磁航向低通滤波系数
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      初始化六轴姿态解算器
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_init(void)
{
    memset(&s_kalman_6axis, 0, sizeof(s_kalman_6axis));                  // 清空解算状态
    s_kalman_6axis.Q[0] = s_config.kalman_6axis_q;
    s_kalman_6axis.Q[1] = s_config.kalman_6axis_q;
    s_kalman_6axis.Q[2] = s_config.kalman_6axis_q;
    s_kalman_6axis.R[0] = s_config.kalman_6axis_r;
    s_kalman_6axis.R[1] = s_config.kalman_6axis_r;
    s_kalman_6axis.R[2] = s_config.kalman_6axis_r;
    s_kalman_6axis.Pk[0] = 1.0f;
    s_kalman_6axis.Pk[1] = 1.0f;
    s_kalman_6axis.Pk[2] = 1.0f;
    s_kalman_6axis.T = s_config.kalman_6axis_T;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      重置六轴姿态解算器
////  @param      roll        初始 roll
////  @param      pitch       初始 pitch
////  @param      yaw         初始 yaw
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_reset(float roll, float pitch, float yaw)
{
    ins_kalman_6axis_init();                                             // 重新初始化滤波器
    s_kalman_6axis.roll = roll;
    s_kalman_6axis.pitch = pitch;
    s_kalman_6axis.yaw_gyro = normalize_angle_rad(yaw);
    s_kalman_6axis.Xk[0] = roll;
    s_kalman_6axis.Xk[1] = pitch;
    s_kalman_6axis.Xk[2] = s_kalman_6axis.yaw_gyro;
    s_kalman_6axis.Xk_[0] = roll;
    s_kalman_6axis.Xk_[1] = pitch;
    s_kalman_6axis.Xk_[2] = s_kalman_6axis.yaw_gyro;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      更新六轴姿态解算器
////  @param      gyro_x      陀螺仪 X 轴角速度(rad/s)
////  @param      gyro_y      陀螺仪 Y 轴角速度(rad/s)
////  @param      gyro_z      陀螺仪 Z 轴角速度(rad/s)
////  @param      acc_x       加速度计 X 轴
////  @param      acc_y       加速度计 Y 轴
////  @param      acc_z       加速度计 Z 轴
////  @return     float       更新后的纯陀螺 yaw
 ////-------------------------------------------------------------------------------------------------------------------
static float ins_kalman_6axis_update(float gyro_x, float gyro_y, float gyro_z,
                                     float acc_x, float acc_y, float acc_z)
{
    float roll = s_kalman_6axis.Xk[0];
    float pitch = s_kalman_6axis.Xk[1];
    float cos_pitch = cosf(pitch);

    if(fabsf(cos_pitch) < 1e-6f)
    {
            cos_pitch = (cos_pitch >= 0.0f) ? 1e-6f : -1e-6f;                // 避免除零
    }

    s_kalman_6axis.Uk[0] = gyro_x + sinf(roll) * tanf(pitch) * gyro_y +
                           cosf(roll) * tanf(pitch) * gyro_z;
    s_kalman_6axis.Uk[1] = cosf(roll) * gyro_y - sinf(roll) * gyro_z;
    s_kalman_6axis.Uk[2] = sinf(roll) * gyro_y / cos_pitch +
                           cosf(roll) * gyro_z / cos_pitch;
    s_last_rp_yaw_rate = s_kalman_6axis.Uk[2];

    s_kalman_6axis.Xk_[0] = s_kalman_6axis.Xk[0] + s_kalman_6axis.T * s_kalman_6axis.Uk[0];
    s_kalman_6axis.Xk_[1] = s_kalman_6axis.Xk[1] + s_kalman_6axis.T * s_kalman_6axis.Uk[1];
    s_kalman_6axis.Xk_[2] = s_kalman_6axis.Xk[2] + s_kalman_6axis.T * s_kalman_6axis.Uk[2];

    s_kalman_6axis.Pk_[0] = s_kalman_6axis.Pk[0] + s_kalman_6axis.Q[0];
    s_kalman_6axis.Pk_[1] = s_kalman_6axis.Pk[1] + s_kalman_6axis.Q[1];
    s_kalman_6axis.Pk_[2] = s_kalman_6axis.Pk[2] + s_kalman_6axis.Q[2];

    s_kalman_6axis.K[0] = s_kalman_6axis.Pk_[0] / (s_kalman_6axis.Pk_[0] + s_kalman_6axis.R[0]);
    s_kalman_6axis.K[1] = s_kalman_6axis.Pk_[1] / (s_kalman_6axis.Pk_[1] + s_kalman_6axis.R[1]);
    s_kalman_6axis.K[2] = 0.0f;                                          // yaw 不使用加速度观测修正

    {
        float acc_yz_norm = sqrtf(acc_y * acc_y + acc_z * acc_z);
        if(acc_yz_norm < 1e-6f)
        {
            acc_yz_norm = 1e-6f;
        }

        s_kalman_6axis.Zk[0] = atan2f(acc_y, acc_z);                     // 加速度测得的 roll
        s_kalman_6axis.Zk[1] = -atan2f(acc_x, acc_yz_norm);              // 加速度测得的 pitch
        s_kalman_6axis.Zk[2] = 0.0f;
    }

    s_kalman_6axis.Xk[0] = (1.0f - s_kalman_6axis.K[0]) * s_kalman_6axis.Xk_[0] +
                           s_kalman_6axis.K[0] * s_kalman_6axis.Zk[0];
    s_kalman_6axis.Xk[1] = (1.0f - s_kalman_6axis.K[1]) * s_kalman_6axis.Xk_[1] +
                           s_kalman_6axis.K[1] * s_kalman_6axis.Zk[1];
    s_kalman_6axis.Xk[2] = normalize_angle_rad(s_kalman_6axis.Xk_[2]);   // yaw 单独归一化

    s_kalman_6axis.Pk[0] = (1.0f - s_kalman_6axis.K[0]) * s_kalman_6axis.Pk_[0];
    s_kalman_6axis.Pk[1] = (1.0f - s_kalman_6axis.K[1]) * s_kalman_6axis.Pk_[1];
    s_kalman_6axis.Pk[2] = s_kalman_6axis.Pk_[2];

    s_kalman_6axis.roll = s_kalman_6axis.Xk[0];
    s_kalman_6axis.pitch = s_kalman_6axis.Xk[1];
    s_kalman_6axis.yaw_gyro = s_kalman_6axis.Xk[2];

    {
        float sin_roll = sinf(s_kalman_6axis.roll);
        float cos_roll = cosf(s_kalman_6axis.roll);
        float sin_pitch = sinf(s_kalman_6axis.pitch);
        float cos_pitch_now = cosf(s_kalman_6axis.pitch);
        float gravity_x = -9.80665f * sin_pitch;                         // 当前姿态下重力在 X 轴分量
        float gravity_y = 9.80665f * sin_roll * cos_pitch_now;
        float gravity_z = 9.80665f * cos_roll * cos_pitch_now;

        s_kalman_6axis.ax_linear = acc_x - gravity_x;
        s_kalman_6axis.ay_linear = acc_y - gravity_y;
        s_kalman_6axis.az_linear = acc_z - gravity_z;
    }

    return s_kalman_6axis.yaw_gyro;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      初始化航向 EKF
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_init(void)
{
    memset(&s_yaw_ekf, 0, sizeof(YawEKF2State));
    s_yaw_ekf.N_psi = s_config.Q_yaw;
    // 无磁力计模式：静止时 bias 过程噪声需要更大，因为没有磁力计绝对参考
    // 只能靠零速检测来修正 bias（静止时角速度应为0，非零部分就是 bias）
    s_yaw_ekf.N_b = 1e-5f;                // 静止时 bias 过程噪声（增大，让零速修正更积极）
    s_yaw_ekf.N_b_frozen = 1e-9f;         // 运动时 bias 过程噪声（保持很小，避免运动中 bias 漂移）
    s_yaw_ekf.R = s_config.R_mag;
    s_yaw_ekf.P[0][0] = 0.01f;            // 初始 yaw 方差，允许较快吸收首批磁观测
    s_yaw_ekf.P[1][1] = 0.001f;           // 初始 bias 方差
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      重置航向 EKF
////  @param      initial_yaw 初始航向
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_reset(float initial_yaw)
{
    yaw_ekf2_init();
    s_yaw_ekf.x[0] = normalize_angle_rad(initial_yaw);
    s_yaw_ekf.x[1] = 0.0f;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      航向 EKF 预测
////  @param      gyro_z      陀螺仪 Z 轴角速度
////  @param      dt          采样周期
////  @param      is_stationary 是否静止
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_predict(float gyro_z, float dt, uint8_t is_stationary)
{
    s_yaw_ekf.x[0] += (gyro_z - s_yaw_ekf.x[1]) * dt;
    s_yaw_ekf.x[0] = normalize_angle_rad(s_yaw_ekf.x[0]);
    s_yaw_ekf.yaw_predict = s_yaw_ekf.x[0];

    // 静止时放大 N_b，让滤波器更容易收敛陀螺 bias
    float Nb = is_stationary ? s_yaw_ekf.N_b : s_yaw_ekf.N_b_frozen;

    // 按连续白噪声模型离散化过程噪声 Q
    float q00 = s_yaw_ekf.N_psi * dt + Nb * dt * dt * dt / 3.0f;
    float q01 = -Nb * dt * dt / 2.0f;
    float q11 = Nb * dt;

    float P00 = s_yaw_ekf.P[0][0];
    float P01 = s_yaw_ekf.P[0][1];
    float P11 = s_yaw_ekf.P[1][1];

    s_yaw_ekf.P[0][0] = P00 - 2.0f * dt * P01 + dt * dt * P11 + q00;
    s_yaw_ekf.P[0][1] = P01 - dt * P11 + q01;
    s_yaw_ekf.P[1][0] = s_yaw_ekf.P[0][1]; // 保持协方差对称
    s_yaw_ekf.P[1][1] = P11 + q11;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      航向 EKF 观测更新
////  @param      mag_yaw     磁航向观测值
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_update(float mag_yaw)
{
    float y_tilde = normalize_angle_rad(mag_yaw - s_yaw_ekf.x[0]);
    float S = s_yaw_ekf.P[0][0] + s_yaw_ekf.R;

    float K0 = s_yaw_ekf.P[0][0] / S;
    float K1 = s_yaw_ekf.P[1][0] / S;

    // Round2 调参：放宽创新量限制，允许旋转时EKF跟踪磁力计
    // Round1的3°太保守，旋转时磁力计滞后导致y_tilde被截断→bias正反馈→EKF丢失90°
    float max_innovation = 0.175f;    // 10 degrees (was 3°/0.052rad)
    if (y_tilde > max_innovation) y_tilde = max_innovation;
    if (y_tilde < -max_innovation) y_tilde = -max_innovation;

    // Round2 调参：大幅收紧K1限幅，防止旋转时bias被磁力计滞后污染
    // K1<0且y_tilde<0时K1*y_tilde>0使bias增大→EKF减速→正反馈丢失航向
    if (K1 > 0.03f) K1 = 0.03f;
    if (K1 < -0.03f) K1 = -0.03f;

    s_yaw_ekf.x[0] = normalize_angle_rad(s_yaw_ekf.x[0] + K0 * y_tilde);
    s_yaw_ekf.x[1] += K1 * y_tilde;

    // 对 bias 做限幅，避免滤波器发散到不合理范围（这里限制为 10 deg/s）
    float max_bias = 10.0f * INS_PI / 180.0f;
    if (s_yaw_ekf.x[1] > max_bias) s_yaw_ekf.x[1] = max_bias;
    if (s_yaw_ekf.x[1] < -max_bias) s_yaw_ekf.x[1] = -max_bias;

    // 先保存旧的协方差交叉项
    float p01_old = s_yaw_ekf.P[0][1];

    s_yaw_ekf.P[0][0] = (1.0f - K0) * s_yaw_ekf.P[0][0];
    s_yaw_ekf.P[0][1] = (1.0f - K0) * p01_old;
    s_yaw_ekf.P[1][0] = s_yaw_ekf.P[0][1];                 // 保持对称
    s_yaw_ekf.P[1][1] = s_yaw_ekf.P[1][1] - K1 * p01_old;  // 更新 P11

    // 防止数值误差导致协方差退化成负数
    if (s_yaw_ekf.P[0][0] < 1e-8f) s_yaw_ekf.P[0][0] = 1e-8f;
    if (s_yaw_ekf.P[1][1] < 1e-8f) s_yaw_ekf.P[1][1] = 1e-8f;
    s_yaw_ekf.P[0][1] = s_yaw_ekf.P[1][0];  // 保持对称
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      根据编码器位移更新位置
////  @param      yaw           当前航向
////  @param      delta_left    左轮位移增量 (米)
////  @param      delta_right   右轮位移增量 (米)
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_position_update(float yaw, float delta_left, float delta_right)
{
    float dist_center = 0.5f * (delta_left + delta_right);               // 车体中心位移
    s_pos_x += dist_center * cosf(yaw);                                  // 更新 X 位置
    s_pos_y += dist_center * sinf(yaw);                                  // 更新 Y 位置
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      判断当前是否静止
////  @param      input       INS 输入数据
////  @return     uint8_t     1-静止 0-运动
 ////-------------------------------------------------------------------------------------------------------------------
static uint8_t ins_is_stationary(const INS_Input *input)
{
    if(input == NULL)
    {
        return 1u;
    }

    if(fabsf(input->v_mps) < s_config.zupt_speed_threshold &&
       fabsf(input->gyro_z_rad_s) < s_config.zupt_gyro_threshold)
    {
        return 1u;
    }

    return 0u;
}

//-------------------------------------------对外接口------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
////  @brief      初始化 INS 模块
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_init(void)
{
    Init_config();                                                       // 初始化默认配置
    ins_kalman_6axis_init();                                             // 初始化六轴姿态解算器
    yaw_ekf2_init();                                                     // 初始化航向 EKF（保留，不再用于最终 yaw）
    s_state.x = 0.0f;
    s_state.y = 0.0f;
    s_state.yaw = 0.0f;
    s_pos_x = 0.0f;
    s_pos_y = 0.0f;

    s_mag_initial_yaw_captured = 0u;                                     // 清除磁航向零点记录
    s_mag_yaw_corr_initial_captured = 0u;                                // 清除LUT修正磁航向零点记录

    // 零偏校准状态复位
    s_bias_calib_active = 0u;
    s_bias_calib_elapsed = 0.0f;
    s_calibrated_bias_rad_s = 0.0f;
    s_bias_corrected_yaw = 0.0f;
    s_rp_bias_corrected_yaw = 0.0f;
    s_last_rp_yaw_rate = 0.0f;
    s_model_yaw = 0.0f;
    s_position_frozen = 0u;

    s_initialized = 1u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      重置 INS 状态
////  @param      x           初始 X 位置
////  @param      y           初始 Y 位置
////  @param      theta       初始航向
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_reset(float x, float y, float theta)
{
    if(!s_initialized)
    {
        Ins_init();
    }

    float normalized_theta = normalize_angle_rad(theta);
    s_state.yaw = normalized_theta;
    s_state.x = x;                                                      // 同步更新 s_state.x
    s_state.y = y;                                                      // 同步更新 s_state.y
    s_pos_x = x;
    s_pos_y = y;

    // 每次 reset 都重新捕获磁航向零点
    // 这样相对磁航向会以当前朝向为新的参考方向
    s_mag_initial_yaw_captured = 0u;

    // 零偏校准状态复位
    s_bias_calib_active = 0u;
    s_bias_calib_elapsed = 0.0f;
    s_calibrated_bias_rad_s = 0.0f;
    s_bias_corrected_yaw = normalized_theta;                             // 同步补偿 yaw
    s_rp_bias_corrected_yaw = normalized_theta;
    s_model_yaw = normalized_theta;
    s_position_frozen = 0u;

    ins_kalman_6axis_reset(0.0f, 0.0f, normalized_theta);
    yaw_ekf2_reset(normalized_theta);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      更新 INS 配置参数
////  @param      config      新的配置参数
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_set_config(const INS_Config *config)
{
    if(config == NULL)
    {
        return;
    }

    if(config->kalman_6axis_q > 0.0f)
        s_config.kalman_6axis_q = config->kalman_6axis_q;
    if(config->kalman_6axis_r > 0.0f)
        s_config.kalman_6axis_r = config->kalman_6axis_r;
    if(config->kalman_6axis_T > 0.0f)
        s_config.kalman_6axis_T = config->kalman_6axis_T;
    if(config->Q_yaw > 0.0f)
    {
        s_config.Q_yaw = config->Q_yaw;
        s_yaw_ekf.N_psi = config->Q_yaw;
    }
    if(config->R_mag > 0.0f)
    {
        s_config.R_mag = config->R_mag;
        s_yaw_ekf.R = config->R_mag;
    }
    if(config->wheelbase > 0.0f)
        s_config.wheelbase = config->wheelbase;
    if(config->tick_to_meter_left != 0.0f)
        s_config.tick_to_meter_left = config->tick_to_meter_left;
    if(config->tick_to_meter_right != 0.0f)
        s_config.tick_to_meter_right = config->tick_to_meter_right;
    if(config->zupt_speed_threshold > 0.0f)
        s_config.zupt_speed_threshold = config->zupt_speed_threshold;
    if(config->zupt_gyro_threshold > 0.0f)
        s_config.zupt_gyro_threshold = config->zupt_gyro_threshold;

    if(s_initialized)
    {
        ins_kalman_6axis_reset(s_kalman_6axis.roll, s_kalman_6axis.pitch, s_state.yaw);
    }
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      更新 INS 状态
////  @param      input       INS 输入数据
////  @param      dt_s        当前采样周期
 ////  @return     void
////  @note       调用前必须先调用 Ins_init() 初始化
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_update(const INS_Input *input, float dt_s)
{
    float yaw_gyro = 0.0f;
    uint8_t mag_ok = 0u;
    uint8_t is_stationary = 0;

    // 要求上层显式调用 Ins_init()，不自动初始化
    // 这样可以暴露初始化时序问题
    if(!s_initialized)
    {
        return;  // 未初始化，直接返回
    }

    if(input == NULL || dt_s <= 0.0f)
    {
        return;
    }

    s_kalman_6axis.T = dt_s;                                             // 使用实时采样周期更新六轴解算器

    yaw_gyro = ins_kalman_6axis_update(imu660.data_Ripen.gyro_x,
                                       imu660.data_Ripen.gyro_y,
                                       input->gyro_z_rad_s,
                                       imu660.data_Ripen.acc_x,
                                       imu660.data_Ripen.acc_y,
                                       imu660.data_Ripen.acc_z);
    (void)yaw_gyro;

    is_stationary = ins_is_stationary(input);
    yaw_ekf2_predict(input->gyro_z_rad_s - s_calibrated_bias_rad_s, dt_s, is_stationary);

    // 零偏修正陀螺层：作为 yaw_gyro 输出与 EKF 预测输入。
    {
        float gyro_z_compensated = input->gyro_z_rad_s - s_calibrated_bias_rad_s;
        s_bias_corrected_yaw += gyro_z_compensated * dt_s;
        s_bias_corrected_yaw = normalize_angle_rad(s_bias_corrected_yaw);

        s_rp_bias_corrected_yaw += (s_last_rp_yaw_rate - s_calibrated_bias_rad_s) * dt_s;
        s_rp_bias_corrected_yaw = normalize_angle_rad(s_rp_bias_corrected_yaw);

        s_model_yaw -= input->omega_rad_s * dt_s;                        // 取反: steer左负右正, gyro_z左正右负
        s_model_yaw = normalize_angle_rad(s_model_yaw);
    }

    // 磁力计层：计算相对启动方向的磁航向，用作 yaw EKF 的绝对观测。
    if(input->mag_valid != 0u)
    {
        float mag_norm = fabsf(imu660.data_Ripen.mag_x) +
                         fabsf(imu660.data_Ripen.mag_y) +
                         fabsf(imu660.data_Ripen.mag_z);
        if(mag_norm > 1e-4f)
        {
            float body_roll = 0.0f;
            float body_pitch = 0.0f;

            map_body_attitude_for_mag_compensation(s_kalman_6axis.roll,
                                                   s_kalman_6axis.pitch,
                                                   &body_roll,
                                                   &body_pitch);

            s_mag_yaw_raw = compute_tilt_compensated_mag_yaw(imu660.data_Ripen.mag_x,
                                                             imu660.data_Ripen.mag_y,
                                                             imu660.data_Ripen.mag_z,
                                                             body_roll,
                                                             body_pitch);

            if(s_mag_initial_yaw_captured == 0u)
            {
                s_initial_mag_yaw = s_mag_yaw_raw;
                s_mag_yaw_rel = 0.0f;
                s_mag_yaw_rel_filtered = 0.0f;
                s_mag_initial_yaw_captured = 1u;
            }
            else
            {
                s_mag_yaw_rel = normalize_angle_rad(s_mag_yaw_raw - s_initial_mag_yaw);
                {
                    float diff = normalize_angle_rad(s_mag_yaw_rel - s_mag_yaw_rel_filtered);
                    s_mag_yaw_rel_filtered = normalize_angle_rad(s_mag_yaw_rel_filtered +
                                            s_config.mag_alpha * diff);
                }
                mag_ok = 1u;
            }

            // LUT 修正层：mag_yaw_raw -> LUT -> mag_yaw_corr_raw -> mag_yaw_corr_rel
            {
                s_mag_yaw_corr_raw = mag_yaw_lut_apply_rad(s_mag_yaw_raw);

                if(s_mag_yaw_corr_initial_captured == 0u)
                {
                    s_initial_mag_yaw_corr = s_mag_yaw_corr_raw;
                    s_mag_yaw_corr_rel = 0.0f;
                    s_mag_yaw_corr_rel_filtered = 0.0f;
                    s_mag_yaw_corr_initial_captured = 1u;
                }
                else
                {
                    s_mag_yaw_corr_rel = normalize_angle_rad(s_mag_yaw_corr_raw - s_initial_mag_yaw_corr);
                    {
                        float diff_corr = normalize_angle_rad(s_mag_yaw_corr_rel - s_mag_yaw_corr_rel_filtered);
                        s_mag_yaw_corr_rel_filtered = normalize_angle_rad(s_mag_yaw_corr_rel_filtered +
                                                s_config.mag_alpha * diff_corr);
                    }
                }
            }
        }
    }

    if((mag_ok != 0u) && (s_mag_fusion_enabled != 0u))
    {
        yaw_ekf2_update(s_mag_yaw_corr_rel_filtered);
        s_state.yaw = s_yaw_ekf.x[0];
    }
    else
    {
        s_yaw_ekf.x[0] = s_bias_corrected_yaw;
        s_yaw_ekf.yaw_predict = s_bias_corrected_yaw;
        s_state.yaw = s_bias_corrected_yaw;
    }

    // 使用输入结构体中的编码器位移增量（由上层传入，数据流单向）
    float delta_left = input->delta_left_m;
    float delta_right = input->delta_right_m;

    if(is_stationary)
    {
        delta_left = 0.0f;
        delta_right = 0.0f;
    }

    // 位置冻结检查：抬车/CHANGE 阶段禁止编码器积分，避免虚假位移
    if (!s_position_frozen)
    {
        ins_position_update(s_state.yaw, delta_left, delta_right);
    }
    s_state.x = s_pos_x;
    s_state.y = s_pos_y;

    // 参数使用说明：
    // - omega_rad_s: 当前版本未使用车辆模型角速度（待调试完成后启用）
    // - mag_yaw_rad: 当前版本未使用，磁航向由 INS 内部 compute_tilt_compensated_mag_yaw() 计算
    // - Q_yaw: 在 yaw_ekf2_init() 中初始化 s_yaw_ekf.N_psi，已正确使用
    // - R_mag: 在 yaw_ekf2_init() 中初始化 s_yaw_ekf.R，已正确使用
    // - wheelbase: 当前版本未直接使用（通过 INS_WHEELBASE_M 宏使用）
    (void)input->omega_rad_s;
    (void)input->mag_yaw_rad;
    (void)s_config.wheelbase;
}
////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取当前 INS 状态
 ////  @param      void
 ////  @return     const INS_State*
 ////-------------------------------------------------------------------------------------------------------------------
const INS_State* Ins_get_state(void)
{
    return &s_state;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取当前姿态角
//  @param      roll         输出当前 roll 指针
//  @param      pitch        输出当前 pitch 指针
//  @param      yaw          输出当前 yaw 指针
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_attitude(float *roll, float *pitch, float *yaw)
{
    if(roll != NULL)
    {
        *roll = s_kalman_6axis.roll;
    }

    if(pitch != NULL)
    {
        *pitch = s_kalman_6axis.pitch;
    }

    if(yaw != NULL)
    {
        *yaw = s_state.yaw;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取各层 yaw 数据
//  @param      yaw_gyro     纯陀螺积分得到的 yaw
//  @param      yaw_mag_raw  倾斜补偿后的原始磁航向
//  @param      yaw_mag_rel  低通后的相对磁航向
//  @param      yaw_ekf      EKF 融合后的最终 yaw
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_yaw_layers(float *yaw_gyro, float *yaw_mag_raw, float *yaw_mag_rel, float *yaw_ekf)
{
    if(yaw_gyro != NULL)
    {
        // 返回零偏修正后的原始 Z 陀螺积分 yaw，便于和磁航向/EKF 对比
        *yaw_gyro = s_bias_corrected_yaw;
    }
    if(yaw_mag_raw != NULL)
    {
        *yaw_mag_raw = s_mag_yaw_raw;
    }
    if(yaw_mag_rel != NULL)
    {
        // 对外统一输出低通后的相对磁航向，避免原始磁角抖动过大
        *yaw_mag_rel = s_mag_yaw_rel_filtered;
    }
    if(yaw_ekf != NULL)
    {
        *yaw_ekf = s_state.yaw;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取当前用于姿态补偿的磁力计三轴
//  @param      mag_x        X 轴磁力计
//  @param      mag_y        Y 轴磁力计
//  @param      mag_z        Z 轴磁力计
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_mag_vector(float *mag_x, float *mag_y, float *mag_z)
{
    if(mag_x != NULL)
    {
        *mag_x = imu660.data_Ripen.mag_x;
    }

    if(mag_y != NULL)
    {
        *mag_y = imu660.data_Ripen.mag_y;
    }

    if(mag_z != NULL)
    {
        *mag_z = imu660.data_Ripen.mag_z;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取含 LUT 修正的调试 yaw 数据
//  @param      yaw_gyro         零偏修正后的陀螺积分 yaw
//  @param      mag_yaw_raw      倾斜补偿后的原始磁航向
//  @param      mag_yaw_rel      低通后的相对磁航向（未LUT修正）
//  @param      mag_yaw_corr_rel LUT修正后的相对磁航向（低通后）
//  @param      yaw_fused        EKF 融合后的最终 yaw
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_yaw_debug_layers(float *yaw_gyro, float *mag_yaw_raw, float *mag_yaw_rel,
                               float *mag_yaw_corr_rel, float *yaw_fused)
{
    if(yaw_gyro != NULL)
    {
        *yaw_gyro = s_bias_corrected_yaw;
    }
    if(mag_yaw_raw != NULL)
    {
        *mag_yaw_raw = s_mag_yaw_raw;
    }
    if(mag_yaw_rel != NULL)
    {
        *mag_yaw_rel = s_mag_yaw_rel_filtered;
    }
    if(mag_yaw_corr_rel != NULL)
    {
        *mag_yaw_corr_rel = s_mag_yaw_corr_rel_filtered;
    }
    if(yaw_fused != NULL)
    {
        *yaw_fused = s_state.yaw;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取两路纯陀螺 yaw 对比量
//  @param      yaw_z_bias       Z轴陀螺零偏修正后积分 yaw（当前主链路）
//  @param      yaw_rp_bias      roll/pitch补偿角速度再零偏修正后积分 yaw（调试链路）
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_gyro_yaw_compare(float *yaw_z_bias, float *yaw_rp_bias, float *yaw_model)
{
    if(yaw_z_bias != NULL)
    {
        *yaw_z_bias = s_bias_corrected_yaw;
    }
    if(yaw_rp_bias != NULL)
    {
        *yaw_rp_bias = s_rp_bias_corrected_yaw;
    }
    if(yaw_model != NULL)
    {
        *yaw_model = s_model_yaw;
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取 LUT 修正后的相对磁航向
//  @return     LUT修正后的相对磁航向（弧度），低通后
//-------------------------------------------------------------------------------------------------------------------
float Ins_get_mag_yaw_corr_rel(void)
{
    return s_mag_yaw_corr_rel_filtered;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      设置磁航向是否参与最终 yaw 融合
//  @param      enable  0=gyro-only, 非0=允许使用LUT后磁航向更新yaw EKF
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_set_mag_fusion_enabled(uint8_t enable)
{
    s_mag_fusion_enabled = (enable != 0u) ? 1u : 0u;
    if(s_mag_fusion_enabled == 0u)
    {
        s_yaw_ekf.x[0] = s_bias_corrected_yaw;
        s_yaw_ekf.yaw_predict = s_bias_corrected_yaw;
        s_state.yaw = s_bias_corrected_yaw;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      查询磁航向是否参与最终 yaw 融合
//  @return     1=启用, 0=关闭
//-------------------------------------------------------------------------------------------------------------------
uint8_t Ins_is_mag_fusion_enabled(void)
{
    return s_mag_fusion_enabled;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      同步所有 yaw 状态到指定值
//  @param      yaw_rad  目标航向（弧度）
//  @note       YAW_CORRECT 阶段矫正完成后调用，避免回跳
//              必须同步：s_bias_corrected_yaw, s_yaw_ekf, s_kalman_6axis, s_state.yaw
//-------------------------------------------------------------------------------------------------------------------
void Ins_sync_yaw(float yaw_rad)
{
    float normalized = normalize_angle_rad(yaw_rad);

    // 同步零偏修正后的陀螺积分 yaw
    s_bias_corrected_yaw = normalized;
    s_rp_bias_corrected_yaw = normalized;
    s_model_yaw = normalized;

    // 同步航向 EKF 状态
    s_yaw_ekf.x[0] = normalized;
    s_yaw_ekf.x[1] = s_calibrated_bias_rad_s;  // 保持当前 bias 估计
    s_yaw_ekf.yaw_predict = normalized;

    // 同步六轴卡尔曼 yaw
    s_kalman_6axis.yaw_gyro = normalized;
    s_kalman_6axis.Xk[2]  = normalized;
    s_kalman_6axis.Xk_[2] = normalized;

    // 同步最终输出
    s_state.yaw = normalized;
}
//-------------------------------------------静止零偏校准（放弃磁力计方案）----------------------------------------

////-------------------------------------------------------------------------------------------------------------------
////  @brief      开始静止零偏校准
////  @param      void
////  @return     void
////  @note       记录当前零偏修正 yaw，开始累积静止时长
////              调用后需在每帧调用 Ins_update_bias_calibration() 推进校准
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_start_bias_calibration(void)
{
    s_bias_calib_active = 1u;
    s_bias_calib_start_yaw = s_bias_corrected_yaw;
    s_bias_calib_elapsed = 0.0f;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      更新静止零偏校准
////  @param      dt_s             本周期时长（秒）
////  @param      is_stationary    是否静止（1=静止）
////  @return     uint8_t          1=校准完成，0=校准中/未激活
////  @note       仅累积静止时长，一旦检测到运动则重置计时器
////              满 2 秒后计算偏航漂移率并冻结为 s_calibrated_bias_rad_s
 ////-------------------------------------------------------------------------------------------------------------------
uint8_t Ins_update_bias_calibration(float dt_s, uint8_t is_stationary)
{
    if (!s_bias_calib_active)
    {
        return 0u;
    }

    if (!is_stationary)
    {
        // 车动了：重置计时器，重新开始
        s_bias_calib_elapsed = 0.0f;
        s_bias_calib_start_yaw = s_bias_corrected_yaw;
        return 0u;
    }

    s_bias_calib_elapsed += dt_s;

    if (s_bias_calib_elapsed >= 2.0f)  // 2 秒静止校准
    {
        // 计算漂移率 = (当前yaw - 起始yaw) / 时间
        float drift_rad = normalize_angle_rad(s_bias_corrected_yaw - s_bias_calib_start_yaw);
        s_calibrated_bias_rad_s = drift_rad / s_bias_calib_elapsed;

        // 零偏修正的 yaw 保持不变（已经在正确值上）

        s_bias_calib_active = 0u;
        return 1u;
    }

    return 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取校准后的零偏值
////  @param      void
////  @return     float       零偏 (rad/s)
 ////-------------------------------------------------------------------------------------------------------------------
float Ins_get_calibrated_bias(void)
{
    return s_calibrated_bias_rad_s;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      手动设置固定零偏（绕过校准流程）
////  @param      bias_rad_s  零偏值 (rad/s)
////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_set_fixed_bias(float bias_rad_s)
{
    s_calibrated_bias_rad_s = bias_rad_s;
    // s_bias_corrected_yaw 保持当前值，不重置
    s_bias_calib_active = 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      冻结/解冻位置积分
////  @param      freeze  1=冻结（抬车/CHANGE阶段禁用编码器积分），0=解冻
////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_set_position_freeze(uint8_t freeze)
{
    s_position_frozen = (freeze != 0u) ? 1u : 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      查询零偏是否已校准
////  @param      void
////  @return     uint8_t  1=已校准（s_calibrated_bias_rad_s 有效），0=未校准
 ////-------------------------------------------------------------------------------------------------------------------
uint8_t Ins_is_bias_calibrated(void)
{
    return (s_bias_calib_active == 0u) ? 1u : 0u;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      测试相对磁航向流程
//  @param      void
//  @return     void
//  @note       依次模拟 0、90、180、270 度输入，检查相对磁航向与 EKF 输出是否正确
//-------------------------------------------------------------------------------------------------------------------
void Ins_test_relative_mag(void)
{
    float test_angles_deg[] = {0.0f, 90.0f, 180.0f, 270.0f};
    INS_Input dummy_input = {0};
    dummy_input.mag_valid = 1;
    float dt = 0.004f; // 4ms

    printf("\n=== 开始测试相对磁航向 ===\n");
    for(int i = 0; i < 4; i++)
    {
        float init_angle_rad = test_angles_deg[i] * INS_PI / 180.0f;

        // 重新初始化模块
        Ins_init();

        // 构造水平面上的理想磁场方向
        // mag_yaw = atan2(y, x)，因此 y = sin(angle)，x = cos(angle)
        imu660.data_Ripen.mag_x = cosf(init_angle_rad);
        imu660.data_Ripen.mag_y = sinf(init_angle_rad);

        // 构造静止状态输入
        imu660.data_Ripen.gyro_x = 0;
        imu660.data_Ripen.gyro_y = 0;
        dummy_input.gyro_z_rad_s = 0;
        imu660.data_Ripen.acc_x = 0;
        imu660.data_Ripen.acc_y = 0;
        imu660.data_Ripen.acc_z = 9.8f;

        dummy_input.v_mps = 0;
        dummy_input.omega_rad_s = 0;

        // 连续更新足够长时间，让 EKF 和低通滤波收敛
        for(int step = 0; step < 500; step++) // 约 2 秒
        {
            Ins_update(&dummy_input, dt);
        }

        float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
        Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

        float err_deg = fabsf(yaw_ekf * 180.0f / INS_PI);
        if(err_deg > 180.0f) err_deg = 360.0f - err_deg; // 取最小夹角误差

        printf("测试角度 %d: 期望相对航向 = %.1f deg\n", i+1, test_angles_deg[i]);
        printf("  原始磁航向 = %.2f deg\n", yaw_mag_raw * 180.0f / INS_PI);
        printf("  相对磁航向 = %.2f deg\n", yaw_mag_rel * 180.0f / INS_PI);
        printf("  EKF 输出 Yaw = %.2f deg (误差: %.2f deg)\n", yaw_ekf * 180.0f / INS_PI, err_deg);
        if(err_deg < 0.5f) {
            printf("  -> [通过] 误差小于 0.5 度\n");
        } else {
            printf("  -> [失败] 误差过大\n");
        }
    }
    printf("=== 相对磁航向测试结束 ===\n\n");
}
