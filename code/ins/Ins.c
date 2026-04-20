/*
 * Ins.c
<<<<<<< HEAD
 * INS????????
=======
 * INS 核心状态估计与航向融合
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 */

#include "zf_common_headfile.h"

<<<<<<< HEAD
 //-------------------------------------------??????------------------------------------------------------------
typedef struct
{
    float roll;                                                          // ???????????
    float pitch;                                                         // ????????????
    float yaw_gyro;                                                      // ???????yaw???????
    float Xk_[3];                                                        // ???????
    float Xk[3];                                                         // ???????
    float Uk[3];                                                         // ??????
    float Zk[3];                                                         // ??????
    float Pk[3];                                                         // ?????????????
    float Pk_[3];                                                        // ?????????????
    float K[3];                                                          // ??????????
    float Q[3];                                                          // ????????????
    float R[3];                                                          // ??????????????
    float T;                                                             // ??????
    float ax_linear;                                                     // ???????????X????????
    float ay_linear;                                                     // ???????????Y????????
    float az_linear;                                                     // ???????????Z????????
} INS_Kalman6Axis;

 //-------------------------------------------?????????------------------------------------------------------------
static INS_State s_state = {0};                                          // INS??
static INS_Config s_config = {0};                                        // INS????????
static INS_Kalman6Axis s_kalman_6axis = {0};                             // ??????????
static YawEKF2State s_yaw_ekf = {0};                                     // ????EKF??
static float s_pos_x = 0.0f;                                             // ????????X
static float s_pos_y = 0.0f;                                             // ????????Y
static uint8_t s_initialized = 0u;                                       // ????????

// 相对磁航向所需变量
static float s_initial_mag_yaw = 0.0f;                                   // 初始磁力计绝对航向
static uint8_t s_mag_initial_yaw_captured = 0u;                          // 是否已捕获初始磁航向
static float s_mag_yaw_raw = 0.0f;                                       // 原始绝对磁航向
static float s_mag_yaw_rel = 0.0f;                                       // 计算出的相对磁航向
static float s_mag_yaw_rel_filtered = 0.0f;                              // 低通滤波后的相对磁航向

 //-------------------------------------------?????????------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ????????? [-??, ??]
 ////  @param      angle       ??????
 ////  @return     ??????????
 ////-------------------------------------------------------------------------------------------------------------------
static float normalize_angle_rad(float angle)
{
    while(angle > INS_PI) angle -= 2.0f * INS_PI;                        // ????????2??
    while(angle < -INS_PI) angle += 2.0f * INS_PI;                       // ????-?????2??
=======
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
static YawEKF2State s_yaw_ekf = {0};                                     // 航向 EKF 状态
static float s_pos_x = 0.0f;                                             // 累计位置 X
static float s_pos_y = 0.0f;                                             // 累计位置 Y
static uint8_t s_initialized = 0u;                                       // 初始化标志

// 磁航向相对零点
static float s_initial_mag_yaw = 0.0f;                                   // 初始磁航向零点
static uint8_t s_mag_initial_yaw_captured = 0u;                          // 初始磁航向是否已记录
static float s_mag_yaw_raw = 0.0f;                                       // 倾斜补偿后的原始磁航向
static float s_mag_yaw_rel = 0.0f;                                       // 相对初始方向的磁航向
static float s_mag_yaw_rel_filtered = 0.0f;                              // 相对磁航向低通输出

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
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    return angle;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ????????????????
=======
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
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void Init_config(void)
{
<<<<<<< HEAD
    s_config.kalman_6axis_q = 0.001f;                                    // ??????????????
    s_config.kalman_6axis_r = 0.1f;                                      // ????????????????
    s_config.kalman_6axis_T = 0.004f;                                    // 4ms????????

    // Round2 tuning: 略微提高过程噪声，允许旋转扰动后bias更快重收敛
    s_config.Q_yaw = 5e-5f;                                              // EKF yaw过程噪声方差(N_psi) [R1:2e-5]

    // Round1 tuning: 略微增大观测噪声，抑制磁力计毛刺(293度跳变)
    s_config.R_mag = 5e-4f;                                              // EKF 磁力计观测噪声方差 [was 0.001]

    s_config.wheelbase = INS_WHEELBASE_M;                                // ???
    s_config.tick_to_meter_left = -0.00002204f;                          // ????????????
    s_config.tick_to_meter_right = -0.00002204f;                         // ????????????
    s_config.zupt_speed_threshold = 0.05f;                               // ??????????
    s_config.zupt_gyro_threshold = 0.05f;                                // ?????????????
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ?????????????
=======
    s_config.kalman_6axis_q = 0.001f;                                    // 六轴姿态过程噪声
    s_config.kalman_6axis_r = 0.1f;                                      // 六轴姿态测量噪声
    s_config.kalman_6axis_T = 0.004f;                                    // 4ms 采样周期

    // Round2 调参：适度提高 yaw 过程噪声，让静止时更容易吸收陀螺零偏
    s_config.Q_yaw = 5e-5f;                                              // EKF yaw 过程噪声 N_psi [R1:2e-5]

    // Round1 调参：适度提高磁观测信任度，改善相对航向收敛速度
    s_config.R_mag = 5e-4f;                                              // EKF 磁观测噪声 [was 0.001]

    s_config.wheelbase = INS_WHEELBASE_M;                                // 轴距
    s_config.tick_to_meter_left = -0.00002204f;                          // 左轮编码器脉冲转距离
    s_config.tick_to_meter_right = -0.00002204f;                         // 右轮编码器脉冲转距离
    s_config.zupt_speed_threshold = 0.05f;                               // 静止速度阈值
    s_config.zupt_gyro_threshold = 0.05f;                                // 静止角速度阈值
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      初始化六轴姿态解算器
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_init(void)
{
<<<<<<< HEAD
    memset(&s_kalman_6axis, 0, sizeof(s_kalman_6axis));                  // ???????????
=======
    memset(&s_kalman_6axis, 0, sizeof(s_kalman_6axis));                  // 清空解算状态
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
<<<<<<< HEAD
 ////  @brief      ????????????
 ////  @param      roll        ???roll
 ////  @param      pitch       ???pitch
 ////  @param      yaw         ???yaw
=======
////  @brief      重置六轴姿态解算器
////  @param      roll        初始 roll
////  @param      pitch       初始 pitch
////  @param      yaw         初始 yaw
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_reset(float roll, float pitch, float yaw)
{
<<<<<<< HEAD
    ins_kalman_6axis_init();                                             // ?????????
=======
    ins_kalman_6axis_init();                                             // 重新初始化滤波器
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
<<<<<<< HEAD
 ////  @brief      ????????????
 ////  @param      gyro_x      ??????X???????rad/s??
 ////  @param      gyro_y      ??????Y???????rad/s??
 ////  @param      gyro_z      ??????Z???????rad/s??
 ////  @param      acc_x       ??????X??????
 ////  @param      acc_y       ??????Y??????
 ////  @param      acc_z       ??????Z??????
 ////  @return     float       ???????yaw
=======
////  @brief      更新六轴姿态解算器
////  @param      gyro_x      陀螺仪 X 轴角速度(rad/s)
////  @param      gyro_y      陀螺仪 Y 轴角速度(rad/s)
////  @param      gyro_z      陀螺仪 Z 轴角速度(rad/s)
////  @param      acc_x       加速度计 X 轴
////  @param      acc_y       加速度计 Y 轴
////  @param      acc_z       加速度计 Z 轴
////  @return     float       更新后的纯陀螺 yaw
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////-------------------------------------------------------------------------------------------------------------------
static float ins_kalman_6axis_update(float gyro_x, float gyro_y, float gyro_z,
                                     float acc_x, float acc_y, float acc_z)
{
    float roll = s_kalman_6axis.Xk[0];
    float pitch = s_kalman_6axis.Xk[1];
    float cos_pitch = cosf(pitch);

    if(fabsf(cos_pitch) < 1e-6f)
    {
<<<<<<< HEAD
        cos_pitch = (cos_pitch >= 0.0f) ? 1e-6f : -1e-6f;                // ???????
=======
            cos_pitch = (cos_pitch >= 0.0f) ? 1e-6f : -1e-6f;                // 避免除零
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    }

    s_kalman_6axis.Uk[0] = gyro_x + sinf(roll) * tanf(pitch) * gyro_y +
                           cosf(roll) * tanf(pitch) * gyro_z;
    s_kalman_6axis.Uk[1] = cosf(roll) * gyro_y - sinf(roll) * gyro_z;
    s_kalman_6axis.Uk[2] = sinf(roll) * gyro_y / cos_pitch +
                           cosf(roll) * gyro_z / cos_pitch;

    s_kalman_6axis.Xk_[0] = s_kalman_6axis.Xk[0] + s_kalman_6axis.T * s_kalman_6axis.Uk[0];
    s_kalman_6axis.Xk_[1] = s_kalman_6axis.Xk[1] + s_kalman_6axis.T * s_kalman_6axis.Uk[1];
    s_kalman_6axis.Xk_[2] = s_kalman_6axis.Xk[2] + s_kalman_6axis.T * s_kalman_6axis.Uk[2];

    s_kalman_6axis.Pk_[0] = s_kalman_6axis.Pk[0] + s_kalman_6axis.Q[0];
    s_kalman_6axis.Pk_[1] = s_kalman_6axis.Pk[1] + s_kalman_6axis.Q[1];
    s_kalman_6axis.Pk_[2] = s_kalman_6axis.Pk[2] + s_kalman_6axis.Q[2];

    s_kalman_6axis.K[0] = s_kalman_6axis.Pk_[0] / (s_kalman_6axis.Pk_[0] + s_kalman_6axis.R[0]);
    s_kalman_6axis.K[1] = s_kalman_6axis.Pk_[1] / (s_kalman_6axis.Pk_[1] + s_kalman_6axis.R[1]);
<<<<<<< HEAD
    s_kalman_6axis.K[2] = 0.0f;                                          // yaw?????????
=======
    s_kalman_6axis.K[2] = 0.0f;                                          // yaw 不使用加速度观测修正
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

    {
        float acc_yz_norm = sqrtf(acc_y * acc_y + acc_z * acc_z);
        if(acc_yz_norm < 1e-6f)
        {
            acc_yz_norm = 1e-6f;
        }

<<<<<<< HEAD
        s_kalman_6axis.Zk[0] = atan2f(acc_y, acc_z);                     // ????????roll
        s_kalman_6axis.Zk[1] = -atan2f(acc_x, acc_yz_norm);              // ????????pitch
=======
        s_kalman_6axis.Zk[0] = atan2f(acc_y, acc_z);                     // 加速度测得的 roll
        s_kalman_6axis.Zk[1] = -atan2f(acc_x, acc_yz_norm);              // 加速度测得的 pitch
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
        s_kalman_6axis.Zk[2] = 0.0f;
    }

    s_kalman_6axis.Xk[0] = (1.0f - s_kalman_6axis.K[0]) * s_kalman_6axis.Xk_[0] +
                           s_kalman_6axis.K[0] * s_kalman_6axis.Zk[0];
    s_kalman_6axis.Xk[1] = (1.0f - s_kalman_6axis.K[1]) * s_kalman_6axis.Xk_[1] +
                           s_kalman_6axis.K[1] * s_kalman_6axis.Zk[1];
<<<<<<< HEAD
    s_kalman_6axis.Xk[2] = normalize_angle_rad(s_kalman_6axis.Xk_[2]);   // yaw??????
=======
    s_kalman_6axis.Xk[2] = normalize_angle_rad(s_kalman_6axis.Xk_[2]);   // yaw 单独归一化
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

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
<<<<<<< HEAD
        float gravity_x = -9.80665f * sin_pitch;                         // ?????????????
=======
        float gravity_x = -9.80665f * sin_pitch;                         // 当前姿态下重力在 X 轴分量
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
        float gravity_y = 9.80665f * sin_roll * cos_pitch_now;
        float gravity_z = 9.80665f * cos_roll * cos_pitch_now;

        s_kalman_6axis.ax_linear = acc_x - gravity_x;
        s_kalman_6axis.ay_linear = acc_y - gravity_y;
        s_kalman_6axis.az_linear = acc_z - gravity_z;
    }

    return s_kalman_6axis.yaw_gyro;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ???EKF?????????
=======
////  @brief      初始化航向 EKF
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_init(void)
{
    memset(&s_yaw_ekf, 0, sizeof(YawEKF2State));
    s_yaw_ekf.N_psi = s_config.Q_yaw;
<<<<<<< HEAD
    // Round2 tuning: 大幅提高静止bias噪声，解决旋转后漂移问题
    // 旋转扰动了内部bias估计，停稳后需要快速重收敛
    s_yaw_ekf.N_b = 5e-7f;                // 动态时的bias过程噪声方差
    s_yaw_ekf.N_b_frozen = 5e-9f;       // 静止时的bias过程噪声方差 [R1:1e-10, orig:1e-12]
    s_yaw_ekf.R = s_config.R_mag;
    s_yaw_ekf.P[0][0] = 0.01f;           // 初始角度协方差不能太大，否则会被一开始的错误观测带偏
    s_yaw_ekf.P[1][1] = 0.001f;          // 初始零偏协方差
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ???EKF?????????
 ////  @param      initial_yaw ????????
=======
    // Round2 调参：静止时允许 bias 慢速收敛，运动时尽量冻结 bias
    // 这样能抑制静态漂移，同时避免转动过程把真实角速度误吸收到 bias
    s_yaw_ekf.N_b = 5e-7f;                // 静止时 bias 过程噪声
    s_yaw_ekf.N_b_frozen = 5e-9f;         // 运动时 bias 过程噪声 [R1:1e-10, orig:1e-12]
    s_yaw_ekf.R = s_config.R_mag;
    s_yaw_ekf.P[0][0] = 0.01f;            // 初始 yaw 方差，允许较快吸收首批磁观测
    s_yaw_ekf.P[1][1] = 0.001f;           // 初始 bias 方差
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      重置航向 EKF
////  @param      initial_yaw 初始航向
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_reset(float initial_yaw)
{
    yaw_ekf2_init();
    s_yaw_ekf.x[0] = normalize_angle_rad(initial_yaw);
    s_yaw_ekf.x[1] = 0.0f;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ???EKF????????
 ////  @param      gyro_z      ?????Z??????
 ////  @param      dt          ?????
 ////  @param      is_stationary ????
=======
////  @brief      航向 EKF 预测
////  @param      gyro_z      陀螺仪 Z 轴角速度
////  @param      dt          采样周期
////  @param      is_stationary 是否静止
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_predict(float gyro_z, float dt, uint8_t is_stationary)
{
    s_yaw_ekf.x[0] += (gyro_z - s_yaw_ekf.x[1]) * dt;
    s_yaw_ekf.x[0] = normalize_angle_rad(s_yaw_ekf.x[0]);
    s_yaw_ekf.yaw_predict = s_yaw_ekf.x[0];

<<<<<<< HEAD
    // ???????????? N_b (????????bias)
    float Nb = is_stationary ? s_yaw_ekf.N_b : s_yaw_ekf.N_b_frozen;

    // ????????????????????? Q?????????
=======
    // 静止时放大 N_b，让滤波器更容易收敛陀螺 bias
    float Nb = is_stationary ? s_yaw_ekf.N_b : s_yaw_ekf.N_b_frozen;

    // 按连续白噪声模型离散化过程噪声 Q
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    float q00 = s_yaw_ekf.N_psi * dt + Nb * dt * dt * dt / 3.0f;
    float q01 = -Nb * dt * dt / 2.0f;
    float q11 = Nb * dt;

    float P00 = s_yaw_ekf.P[0][0];
    float P01 = s_yaw_ekf.P[0][1];
    float P11 = s_yaw_ekf.P[1][1];

    s_yaw_ekf.P[0][0] = P00 - 2.0f * dt * P01 + dt * dt * P11 + q00;
    s_yaw_ekf.P[0][1] = P01 - dt * P11 + q01;
<<<<<<< HEAD
    s_yaw_ekf.P[1][0] = s_yaw_ekf.P[0][1]; // ????????
=======
    s_yaw_ekf.P[1][0] = s_yaw_ekf.P[0][1]; // 保持协方差对称
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    s_yaw_ekf.P[1][1] = P11 + q11;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ???EKF?????????
 ////  @param      mag_yaw     ?????????????
=======
////  @brief      航向 EKF 观测更新
////  @param      mag_yaw     磁航向观测值
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_update(float mag_yaw)
{
    float y_tilde = normalize_angle_rad(mag_yaw - s_yaw_ekf.x[0]);
    float S = s_yaw_ekf.P[0][0] + s_yaw_ekf.R;

    float K0 = s_yaw_ekf.P[0][0] / S;
    float K1 = s_yaw_ekf.P[1][0] / S;

<<<<<<< HEAD
    // Round1 tuning: 收紧异常残差抑制，磁力计毛刺不应超过3度 [was 10度/0.174rad]
=======
    // Round1 调参：限制单次创新量，避免磁干扰导致跳变 [was 10deg/0.174rad]
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    float max_innovation = 0.052f;    // 3 degrees
    if (y_tilde > max_innovation) y_tilde = max_innovation;
    if (y_tilde < -max_innovation) y_tilde = -max_innovation;

<<<<<<< HEAD
    // Round1 tuning: 放宽K1限制，允许更快的bias修正速度 [was 0.05]
    // 静止时bias应该快速收敛到真实零偏值
=======
    // Round1 调参：限制 K1，避免一次磁观测把 bias 拉偏 [was 0.05]
    // 这样可以减少短时磁干扰带来的 bias 污染
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    if (K1 > 0.15f) K1 = 0.15f;
    if (K1 < -0.15f) K1 = -0.15f;

    s_yaw_ekf.x[0] = normalize_angle_rad(s_yaw_ekf.x[0] + K0 * y_tilde);
    s_yaw_ekf.x[1] += K1 * y_tilde;

<<<<<<< HEAD
    // 限制零偏绝对值大小 (防飞车保护：假设陀螺仪零偏不可能超过 10 deg/s)
=======
    // 对 bias 做限幅，避免滤波器发散到不合理范围（这里限制为 10 deg/s）
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    float max_bias = 10.0f * INS_PI / 180.0f;
    if (s_yaw_ekf.x[1] > max_bias) s_yaw_ekf.x[1] = max_bias;
    if (s_yaw_ekf.x[1] < -max_bias) s_yaw_ekf.x[1] = -max_bias;

<<<<<<< HEAD
    // ????????????????????????
=======
    // 先保存旧的协方差交叉项
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    float p01_old = s_yaw_ekf.P[0][1];

    s_yaw_ekf.P[0][0] = (1.0f - K0) * s_yaw_ekf.P[0][0];
    s_yaw_ekf.P[0][1] = (1.0f - K0) * p01_old;
<<<<<<< HEAD
    s_yaw_ekf.P[1][0] = s_yaw_ekf.P[0][1];                 // ?????
    s_yaw_ekf.P[1][1] = s_yaw_ekf.P[1][1] - K1 * p01_old;  // ??????

    // 7.2 P ?????????????
    if (s_yaw_ekf.P[0][0] < 1e-8f) s_yaw_ekf.P[0][0] = 1e-8f;
    if (s_yaw_ekf.P[1][1] < 1e-8f) s_yaw_ekf.P[1][1] = 1e-8f;
    s_yaw_ekf.P[0][1] = s_yaw_ekf.P[1][0];  // ?????
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ???????????
 ////  @param      yaw         ?????
 ////  @param      tick_left   ?????????????
 ////  @param      tick_right  ?????????????
=======
    s_yaw_ekf.P[1][0] = s_yaw_ekf.P[0][1];                 // 保持对称
    s_yaw_ekf.P[1][1] = s_yaw_ekf.P[1][1] - K1 * p01_old;  // 更新 P11

    // 防止数值误差导致协方差退化成负数
    if (s_yaw_ekf.P[0][0] < 1e-8f) s_yaw_ekf.P[0][0] = 1e-8f;
    if (s_yaw_ekf.P[1][1] < 1e-8f) s_yaw_ekf.P[1][1] = 1e-8f;
    s_yaw_ekf.P[0][1] = s_yaw_ekf.P[1][0];  // 保持对称
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      根据编码器更新位置
////  @param      yaw         当前航向
////  @param      tick_left   左轮编码器增量
////  @param      tick_right  右轮编码器增量
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_position_update(float yaw, int16_t tick_left, int16_t tick_right)
{
<<<<<<< HEAD
    float dist_left = (float)tick_left * s_config.tick_to_meter_left;    // ????????
    float dist_right = (float)tick_right * s_config.tick_to_meter_right; // ????????
    float dist_center = 0.5f * (dist_left + dist_right);                 // ????????
    s_pos_x += dist_center * cosf(yaw);                                  // ????X????
    s_pos_y += dist_center * sinf(yaw);                                  // ????Y????
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ?????????????????
 ////  @param      input       INS???????
 ////  @return     uint8_t     1-??? 0-???
=======
    float dist_left = (float)tick_left * s_config.tick_to_meter_left;    // 左轮位移
    float dist_right = (float)tick_right * s_config.tick_to_meter_right; // 右轮位移
    float dist_center = 0.5f * (dist_left + dist_right);                 // 车体中心位移
    s_pos_x += dist_center * cosf(yaw);                                  // 更新 X 位置
    s_pos_y += dist_center * sinf(yaw);                                  // 更新 Y 位置
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      判断当前是否静止
////  @param      input       INS 输入数据
////  @return     uint8_t     1-静止 0-运动
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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

<<<<<<< HEAD
 //-------------------------------------------??????????------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ?????INS??
=======
//-------------------------------------------对外接口------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
////  @brief      初始化 INS 模块
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_init(void)
{
<<<<<<< HEAD
    Init_config();                                                       // ????????????
    ins_kalman_6axis_init();                                             // ?????????????
    yaw_ekf2_init();                                                     // ????????EKF
=======
    Init_config();                                                       // 初始化默认配置
    ins_kalman_6axis_init();                                             // 初始化六轴姿态解算器
    yaw_ekf2_init();                                                     // 初始化航向 EKF
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    s_state.x = 0.0f;
    s_state.y = 0.0f;
    s_state.yaw = 0.0f;
    s_pos_x = 0.0f;
    s_pos_y = 0.0f;

<<<<<<< HEAD
    s_mag_initial_yaw_captured = 0u;                                     // 复位磁力计初始朝向捕获标志
=======
    s_mag_initial_yaw_captured = 0u;                                     // 清除磁航向零点记录
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

    s_initialized = 1u;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ????INS??
 ////  @param      x           ???X????
 ////  @param      y           ???Y????
 ////  @param      theta       ????????
=======
////  @brief      重置 INS 状态
////  @param      x           初始 X 位置
////  @param      y           初始 Y 位置
////  @param      theta       初始航向
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
    s_pos_x = x;
    s_pos_y = y;

<<<<<<< HEAD
    // 当重置位置和角度时，复位捕获标志
    // 这样下次收到有效的磁力计数据时，会自动与当前重置的角度对齐
=======
    // 每次 reset 都重新捕获磁航向零点
    // 这样相对磁航向会以当前朝向为新的参考方向
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    s_mag_initial_yaw_captured = 0u;

    ins_kalman_6axis_reset(0.0f, 0.0f, normalized_theta);
    yaw_ekf2_reset(normalized_theta);
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ????INS????????
 ////  @param      config      ???????????
=======
////  @brief      更新 INS 配置参数
////  @param      config      新的配置参数
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
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
<<<<<<< HEAD
 ////  @brief      ????INS??
 ////  @param      input       INS????????
 ////  @param      dt_s        ?????????
=======
////  @brief      更新 INS 状态
////  @param      input       INS 输入数据
////  @param      dt_s        当前采样周期
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_update(const INS_Input *input, float dt_s)
{
    float yaw_gyro = 0.0f;
    const EncoderLayerState* enc_state = NULL;
    uint8_t is_stationary = 0;

    if(!s_initialized)
    {
        Ins_init();
    }

    if(input == NULL || dt_s <= 0.0f)
    {
        return;
    }

<<<<<<< HEAD
    s_kalman_6axis.T = dt_s;                                             // ??????????????????????
=======
    s_kalman_6axis.T = dt_s;                                             // 使用实时采样周期更新六轴解算器
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e

    yaw_gyro = ins_kalman_6axis_update(imu660.data_Ripen.gyro_x,
                                       imu660.data_Ripen.gyro_y,
                                       input->gyro_z_rad_s,
                                       imu660.data_Ripen.acc_x,
                                       imu660.data_Ripen.acc_y,
                                       imu660.data_Ripen.acc_z);

    is_stationary = ins_is_stationary(input);
    yaw_ekf2_predict(s_kalman_6axis.Uk[2], dt_s, is_stationary);

    if(input->mag_valid)
    {
<<<<<<< HEAD
        // 注意：根据硬件贴片方向，磁力计的偏航角计算可能与陀螺仪积分的旋向相反。
        // 如果陀螺仪左转是正角，而磁力计左转读数变小，则需要在这里加上负号统一坐标系。
        s_mag_yaw_raw = normalize_angle_rad(-atan2f(imu660.data_Ripen.mag_y, imu660.data_Ripen.mag_x));

        // 捕获上电（或reset后）的初始磁力计航向
        if (!s_mag_initial_yaw_captured)
        {
            // 我们希望此时的相对磁航向 (mag_yaw_rel) 完全等于当前 EKF 已经积分出的航向 (s_yaw_ekf.x[0])
            // 这样就不会因为在没有磁力计数据期间的旋转而产生跳变
=======
        float body_roll_for_mag = 0.0f;
        float body_pitch_for_mag = 0.0f;

        map_body_attitude_for_mag_compensation(s_kalman_6axis.roll,
                                               s_kalman_6axis.pitch,
                                               &body_roll_for_mag,
                                               &body_pitch_for_mag);

        // 先将解算姿态映射到车身坐标，再参与磁航向倾斜补偿
        s_mag_yaw_raw = compute_tilt_compensated_mag_yaw(imu660.data_Ripen.mag_x,
                                                         imu660.data_Ripen.mag_y,
                                                         imu660.data_Ripen.mag_z,
                                                         body_roll_for_mag,
                                                         body_pitch_for_mag);

        // 首次进入或 reset 后重新记录当前磁航向零点
        if (!s_mag_initial_yaw_captured)
        {
            // 让相对磁航向在初始化瞬间与 EKF 当前航向保持一致
            // 这样切入磁观测时不会因为零点不一致而产生突跳
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
            // mag_yaw_rel = raw - initial = current_ekf_yaw
            // => initial = raw - current_ekf_yaw
            s_initial_mag_yaw = normalize_angle_rad(s_mag_yaw_raw - s_yaw_ekf.x[0]);
            s_mag_initial_yaw_captured = 1u;
<<<<<<< HEAD
            s_mag_yaw_rel_filtered = s_yaw_ekf.x[0]; // 同步初始化滤波器历史值
        }

        // 计算相对磁航向
        s_mag_yaw_rel = normalize_angle_rad(s_mag_yaw_raw - s_initial_mag_yaw);

        // 简单的一阶低通滤波 (Alpha = 0.3) 平滑磁力计高频噪声
        // 将Alpha从0.1提高到0.3，减少滤波延迟，提升系统响应速度
        float diff = normalize_angle_rad(s_mag_yaw_rel - s_mag_yaw_rel_filtered);
        s_mag_yaw_rel_filtered = normalize_angle_rad(s_mag_yaw_rel_filtered + 0.3f * diff);

        // EKF 观测更新，使用低通滤波后的相对磁航向
        yaw_ekf2_update(s_mag_yaw_rel_filtered);
    }

    s_state.yaw = normalize_angle_rad(s_yaw_ekf.x[0]);                   // ?????EKF???yaw
=======
            s_mag_yaw_rel_filtered = s_yaw_ekf.x[0]; // 低通输出与 EKF 航向对齐
        }

        // 计算相对初始方向的磁航向
        s_mag_yaw_rel = normalize_angle_rad(s_mag_yaw_raw - s_initial_mag_yaw);

        // 对相对磁航向做一次低通滤波 (Alpha = 0.3)，兼顾响应和稳定性
        // Alpha 从 0.1 提到 0.3 后，转向响应更快，抖动仍可接受
        float diff = normalize_angle_rad(s_mag_yaw_rel - s_mag_yaw_rel_filtered);
        s_mag_yaw_rel_filtered = normalize_angle_rad(s_mag_yaw_rel_filtered + 0.3f * diff);

        // 用滤波后的相对磁航向修正 EKF
        yaw_ekf2_update(s_mag_yaw_rel_filtered);
    }

    s_state.yaw = normalize_angle_rad(s_yaw_ekf.x[0]);                   // 以 EKF 输出作为最终 yaw
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    enc_state = encoder_layer_get_state();
    if(enc_state != NULL)
    {
        int16 tick_left = enc_state->tick_left;
        int16 tick_right = enc_state->tick_right;

        if(is_stationary)
        {
            tick_left = 0;
            tick_right = 0;
        }

        ins_position_update(s_state.yaw, tick_left, tick_right);
        s_state.x = s_pos_x;
        s_state.y = s_pos_y;
    }

<<<<<<< HEAD
    (void)input->omega_rad_s;                                            // ????????????
=======
    (void)input->omega_rad_s;                                            // 当前版本未使用车辆模型角速度
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    (void)input->mag_yaw_rad;
    (void)s_config.Q_yaw;
    (void)s_config.R_mag;
    (void)s_config.wheelbase;
}

////-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
 ////  @brief      ??????INS??
=======
////  @brief      获取当前 INS 状态
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
 ////  @param      void
 ////  @return     const INS_State*
 ////-------------------------------------------------------------------------------------------------------------------
const INS_State* Ins_get_state(void)
{
    return &s_state;
}

//-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
//  @brief      获取各层yaw值
//  @param      yaw_gyro     陀螺仪预测yaw（输出）
//  @param      yaw_mag_raw  原始绝对磁航向（输出）
//  @param      yaw_mag_rel  计算出的相对磁航向（输出）
//  @param      yaw_ekf      EKF最终融合yaw（输出）
=======
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
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_yaw_layers(float *yaw_gyro, float *yaw_mag_raw, float *yaw_mag_rel, float *yaw_ekf)
{
    if(yaw_gyro != NULL)
    {
<<<<<<< HEAD
        // 返回6轴卡尔曼算出来的陀螺仪积分yaw，而不是EKF里的预测值
=======
        // 返回六轴姿态解算器内部维护的纯陀螺 yaw，便于和 EKF 对比
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
        *yaw_gyro = s_kalman_6axis.yaw_gyro;
    }
    if(yaw_mag_raw != NULL)
    {
        *yaw_mag_raw = s_mag_yaw_raw;
    }
    if(yaw_mag_rel != NULL)
    {
<<<<<<< HEAD
        // 返回滤波后的磁力计相对航向，方便上位机观察平滑效果
=======
        // 对外统一输出低通后的相对磁航向，避免原始磁角抖动过大
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
        *yaw_mag_rel = s_mag_yaw_rel_filtered;
    }
    if(yaw_ekf != NULL)
    {
        *yaw_ekf = s_state.yaw;
    }
}

//-------------------------------------------------------------------------------------------------------------------
<<<<<<< HEAD
//  @brief      相对磁航向单元测试与回归测试
//  @param      void
//  @return     void
//  @note       验证四个初始朝向（0, 90, 180, 270度），确认上电后的磁航向和EKF是否稳定在 0度
=======
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
//  @brief      测试相对磁航向流程
//  @param      void
//  @return     void
//  @note       依次模拟 0、90、180、270 度输入，检查相对磁航向与 EKF 输出是否正确
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
//-------------------------------------------------------------------------------------------------------------------
void Ins_test_relative_mag(void)
{
    float test_angles_deg[] = {0.0f, 90.0f, 180.0f, 270.0f};
    INS_Input dummy_input = {0};
    dummy_input.mag_valid = 1;
    float dt = 0.004f; // 4ms

<<<<<<< HEAD
    printf("\n=== 相对磁航向 单元测试开始 ===\n");
=======
    printf("\n=== 开始测试相对磁航向 ===\n");
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
    for(int i = 0; i < 4; i++)
    {
        float init_angle_rad = test_angles_deg[i] * INS_PI / 180.0f;

<<<<<<< HEAD
        // 模拟上电
        Ins_init();

        // 我们模拟磁力计在当前方向上的绝对输出
        // mag_yaw = atan2(y, x), 所以 y = sin(angle), x = cos(angle)
        imu660.data_Ripen.mag_x = cosf(init_angle_rad);
        imu660.data_Ripen.mag_y = sinf(init_angle_rad);

        // 陀螺仪和加速度计处于静止状态
=======
        // 重新初始化模块
        Ins_init();

        // 构造水平面上的理想磁场方向
        // mag_yaw = atan2(y, x)，因此 y = sin(angle)，x = cos(angle)
        imu660.data_Ripen.mag_x = cosf(init_angle_rad);
        imu660.data_Ripen.mag_y = sinf(init_angle_rad);

        // 构造静止状态输入
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
        imu660.data_Ripen.gyro_x = 0;
        imu660.data_Ripen.gyro_y = 0;
        dummy_input.gyro_z_rad_s = 0;
        imu660.data_Ripen.acc_x = 0;
        imu660.data_Ripen.acc_y = 0;
        imu660.data_Ripen.acc_z = 9.8f;

        dummy_input.v_mps = 0;
        dummy_input.omega_rad_s = 0;

<<<<<<< HEAD
        // 连续运行EKF一段时间，让它收敛
        for(int step = 0; step < 500; step++) // 运行2秒
=======
        // 连续更新足够长时间，让 EKF 和低通滤波收敛
        for(int step = 0; step < 500; step++) // 约 2 秒
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
        {
            Ins_update(&dummy_input, dt);
        }

        float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
        Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

        float err_deg = fabsf(yaw_ekf * 180.0f / INS_PI);
<<<<<<< HEAD
        if(err_deg > 180.0f) err_deg = 360.0f - err_deg; // 角度误差

        printf("测试用例 %d: 初始物理朝向 = %.1f deg\n", i+1, test_angles_deg[i]);
        printf("  原始磁航向 = %.2f deg\n", yaw_mag_raw * 180.0f / INS_PI);
        printf("  相对磁航向 = %.2f deg\n", yaw_mag_rel * 180.0f / INS_PI);
        printf("  EKF最终Yaw = %.2f deg (误差: %.2f deg)\n", yaw_ekf * 180.0f / INS_PI, err_deg);

        if(err_deg < 0.5f) {
            printf("  -> [通过] 误差小于0.5度\n");
        } else {
            printf("  -> [失败] 误差过大!\n");
        }
    }
    printf("=== 相对磁航向 单元测试结束 ===\n\n");
=======
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
>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
}
