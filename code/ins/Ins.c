/*
 * Ins.c
 * INS单文件实现
 */

#include "zf_common_headfile.h"
 //-------------------------------------------结构体区------------------------------------------------------------
typedef struct
{
    float roll;                                                          // 横滚角（弧度）
    float pitch;                                                         // 俯仰角（弧度）
    float yaw_gyro;                                                      // 陀螺积分yaw（弧度）
    float Xk_[3];                                                        // 先验估计
    float Xk[3];                                                         // 后验估计
    float Uk[3];                                                         // 系统输入
    float Zk[3];                                                         // 测量状态
    float Pk[3];                                                         // 后验误差协方差
    float Pk_[3];                                                        // 先验误差协方差
    float K[3];                                                          // 卡尔曼增益
    float Q[3];                                                          // 系统噪声协方差
    float R[3];                                                          // 测量噪声协方差
    float T;                                                             // 离散时间
    float ax_linear;                                                     // 重力补偿后的X轴线加速度
    float ay_linear;                                                     // 重力补偿后的Y轴线加速度
    float az_linear;                                                     // 重力补偿后的Z轴线加速度
} INS_Kalman6Axis;

 //-------------------------------------------内部变量区------------------------------------------------------------
static INS_State s_state = {0};                                          // INS状态
static INS_Config s_config = {0};                                        // INS配置参数
static INS_Kalman6Axis s_kalman_6axis = {0};                             // 六轴卡尔曼状态
static float s_yaw_stable = 0.0f;                                        // 稳定yaw
static float s_pos_x = 0.0f;                                             // 位置推算X
static float s_pos_y = 0.0f;                                             // 位置推算Y
static uint8_t s_initialized = 0u;                                       // 初始化标志

 //-------------------------------------------内部函数区------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      角度归一化到 [-π, π]
 ////  @param      angle       输入角度
 ////  @return     归一化后的角度
 ////-------------------------------------------------------------------------------------------------------------------
static float normalize_angle_rad(float angle)
{
    while(angle > INS_PI) angle -= 2.0f * INS_PI;                        // 大于π则减2π
    while(angle < -INS_PI) angle += 2.0f * INS_PI;                       // 小于-π则加2π
    return angle;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      初始化默认配置参数
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void Init_config(void)
{
    s_config.kalman_6axis_q = 0.001f;                                    // 六轴卡尔曼系统噪声
    s_config.kalman_6axis_r = 0.1f;                                      // 六轴卡尔曼测量噪声
    s_config.kalman_6axis_T = 0.004f;                                    // 4ms更新周期
    s_config.mag_alpha = 0.98f;                                          // 磁力计互补滤波系数
    s_config.Q_yaw = 0.001f;                                             // yaw过程噪声预留
    s_config.R_mag = 0.05f;                                              // 磁力计测量噪声预留
    s_config.wheelbase = INS_WHEELBASE_M;                                // 轴距
    s_config.tick_to_meter_left = -0.00002204f;                          // 左轮编码器系数
    s_config.tick_to_meter_right = -0.00002204f;                         // 右轮编码器系数
    s_config.zupt_speed_threshold = 0.05f;                               // 零速速度阈值
    s_config.zupt_gyro_threshold = 0.05f;                                // 零速陀螺仪阈值
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      六轴卡尔曼初始化
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_init(void)
{
    memset(&s_kalman_6axis, 0, sizeof(s_kalman_6axis));                  // 清零滤波器状态
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
 ////  @brief      六轴卡尔曼重置
 ////  @param      roll        初始roll
 ////  @param      pitch       初始pitch
 ////  @param      yaw         初始yaw
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_reset(float roll, float pitch, float yaw)
{
    ins_kalman_6axis_init();                                             // 清空滤波历史
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
 ////  @brief      六轴卡尔曼更新
 ////  @param      gyro_x      陀螺仪X轴角速度（rad/s）
 ////  @param      gyro_y      陀螺仪Y轴角速度（rad/s）
 ////  @param      gyro_z      陀螺仪Z轴角速度（rad/s）
 ////  @param      acc_x       加速度计X轴加速度
 ////  @param      acc_y       加速度计Y轴加速度
 ////  @param      acc_z       加速度计Z轴加速度
 ////  @return     float       陀螺积分yaw
 ////-------------------------------------------------------------------------------------------------------------------
static float ins_kalman_6axis_update(float gyro_x, float gyro_y, float gyro_z,
                                     float acc_x, float acc_y, float acc_z)
{
    float roll = s_kalman_6axis.Xk[0];
    float pitch = s_kalman_6axis.Xk[1];
    float cos_pitch = cosf(pitch);

    if(fabsf(cos_pitch) < 1e-6f)
    {
        cos_pitch = (cos_pitch >= 0.0f) ? 1e-6f : -1e-6f;                // 防止除零
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
    s_kalman_6axis.K[2] = 0.0f;                                          // yaw无测量更新

    {
        float acc_yz_norm = sqrtf(acc_y * acc_y + acc_z * acc_z);
        if(acc_yz_norm < 1e-6f)
        {
            acc_yz_norm = 1e-6f;
        }

        s_kalman_6axis.Zk[0] = atan2f(acc_y, acc_z);                     // 加速度测量roll
        s_kalman_6axis.Zk[1] = -atan2f(acc_x, acc_yz_norm);              // 加速度测量pitch
        s_kalman_6axis.Zk[2] = 0.0f;
    }

    s_kalman_6axis.Xk[0] = (1.0f - s_kalman_6axis.K[0]) * s_kalman_6axis.Xk_[0] +
                           s_kalman_6axis.K[0] * s_kalman_6axis.Zk[0];
    s_kalman_6axis.Xk[1] = (1.0f - s_kalman_6axis.K[1]) * s_kalman_6axis.Xk_[1] +
                           s_kalman_6axis.K[1] * s_kalman_6axis.Zk[1];
    s_kalman_6axis.Xk[2] = normalize_angle_rad(s_kalman_6axis.Xk_[2]);   // yaw仅积分

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
        float gravity_x = -9.80665f * sin_pitch;                         // 机体系重力分量
        float gravity_y = 9.80665f * sin_roll * cos_pitch_now;
        float gravity_z = 9.80665f * cos_roll * cos_pitch_now;

        s_kalman_6axis.ax_linear = acc_x - gravity_x;
        s_kalman_6axis.ay_linear = acc_y - gravity_y;
        s_kalman_6axis.az_linear = acc_z - gravity_z;
    }

    return s_kalman_6axis.yaw_gyro;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      磁力计yaw校正
 ////  @param      yaw_gyro    六轴卡尔曼输出yaw
 ////  @param      mag_x       磁力计X轴
 ////  @param      mag_y       磁力计Y轴
 ////  @return     float       校正后的yaw
 ////-------------------------------------------------------------------------------------------------------------------
static float ins_mag_correct_update(float yaw_gyro, float mag_x, float mag_y)
{
    float yaw_mag = atan2f(mag_y, mag_x);                                // 磁力计yaw
    float yaw_diff = normalize_angle_rad(yaw_mag - yaw_gyro);            // 角度差归一化
    s_yaw_stable = yaw_gyro + (1.0f - s_config.mag_alpha) * yaw_diff;    // 互补滤波融合
    s_yaw_stable = normalize_angle_rad(s_yaw_stable);
    return s_yaw_stable;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      位置推算更新
 ////  @param      yaw         航向角
 ////  @param      tick_left   左轮编码器脉冲
 ////  @param      tick_right  右轮编码器脉冲
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_position_update(float yaw, int16_t tick_left, int16_t tick_right)
{
    float dist_left = (float)tick_left * s_config.tick_to_meter_left;    // 左轮位移
    float dist_right = (float)tick_right * s_config.tick_to_meter_right; // 右轮位移
    float dist_center = 0.5f * (dist_left + dist_right);                 // 中心位移
    s_pos_x += dist_center * cosf(yaw);                                  // 更新X坐标
    s_pos_y += dist_center * sinf(yaw);                                  // 更新Y坐标
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      判断当前是否处于零速状态
 ////  @param      input       INS输入指针
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

 //-------------------------------------------函数定义区------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      初始化INS系统
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_init(void)
{
    Init_config();                                                       // 初始化默认配置
    ins_kalman_6axis_init();                                             // 初始化六轴卡尔曼
    s_state.x = 0.0f;
    s_state.y = 0.0f;
    s_state.yaw = 0.0f;
    s_yaw_stable = 0.0f;
    s_pos_x = 0.0f;
    s_pos_y = 0.0f;
    s_initialized = 1u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      重置INS状态
 ////  @param      x           初始X坐标
 ////  @param      y           初始Y坐标
 ////  @param      theta       初始航向角
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
      s_yaw_stable = normalized_theta;
      s_pos_x = x;
      s_pos_y = y;
      ins_kalman_6axis_reset(0.0f, 0.0f, normalized_theta);
//    s_state.x = x;
//    s_state.y = y;
//    s_state.yaw = normalize_angle_rad(theta);
//    s_yaw_stable = s_state.yaw;
//    s_pos_x = x;
//    s_pos_y = y;
//    ins_kalman_6axis_reset(0.0f, 0.0f, s_state.yaw);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      设置INS配置参数
 ////  @param      config      配置参数指针
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
    if(config->mag_alpha >= 0.0f && config->mag_alpha <= 1.0f)
        s_config.mag_alpha = config->mag_alpha;
    if(config->Q_yaw > 0.0f)
        s_config.Q_yaw = config->Q_yaw;
    if(config->R_mag > 0.0f)
        s_config.R_mag = config->R_mag;
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
 ////  @brief      更新INS状态
 ////  @param      input       INS输入数据
 ////  @param      dt_s        时间步长（秒）
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_update(const INS_Input *input, float dt_s)
{
    float yaw_gyro = 0.0f;
    float yaw_stable = 0.0f;
    const EncoderLayerState* enc_state = NULL;

    if(!s_initialized)
    {
        Ins_init();
    }

    if(input == NULL || dt_s <= 0.0f)
    {
        return;
    }

    s_kalman_6axis.T = dt_s;                                             // 跟随实际周期修正采样时间

    yaw_gyro = ins_kalman_6axis_update(imu660.data_Ripen.gyro_x,
                                       imu660.data_Ripen.gyro_y,
                                       input->gyro_z_rad_s,
                                       imu660.data_Ripen.acc_x,
                                       imu660.data_Ripen.acc_y,
                                       imu660.data_Ripen.acc_z);

    if(input->mag_valid)
    {
        yaw_stable = ins_mag_correct_update(yaw_gyro,
                                            imu660.data_Ripen.mag_x,
                                            imu660.data_Ripen.mag_y);
    }
    else
    {
        s_yaw_stable = normalize_angle_rad(yaw_gyro);                    // 磁力计无效时直接使用积分yaw
        yaw_stable = s_yaw_stable;
    }

    s_state.yaw = normalize_angle_rad(yaw_stable);
//    s_state.yaw = normalize_angle_rad(yaw_gyro);                         // 始终使用陀螺仪积分yaw


    enc_state = encoder_layer_get_state();
    if(enc_state != NULL)
    {
        int16 tick_left = enc_state->tick_left;
        int16 tick_right = enc_state->tick_right;

        if(ins_is_stationary(input))
        {
            tick_left = 0;
            tick_right = 0;
        }

        ins_position_update(s_state.yaw, tick_left, tick_right);
        s_state.x = s_pos_x;
        s_state.y = s_pos_y;
    }

    (void)input->omega_rad_s;                                            // 保留原接口输入
    (void)input->mag_yaw_rad;
    (void)s_config.Q_yaw;
    (void)s_config.R_mag;
    (void)s_config.wheelbase;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      获取当前INS状态
 ////  @param      void
 ////  @return     const INS_State*
 ////-------------------------------------------------------------------------------------------------------------------
const INS_State* Ins_get_state(void)
{
    return &s_state;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取三层yaw值
//  @param      yaw_gyro     陀螺仪积分yaw（输出）
//  @param      yaw_stable   磁力计校正yaw（输出）
//  @param      yaw_final    最终yaw（输出）
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_yaw_layers(float *yaw_gyro, float *yaw_stable, float *yaw_final)
{
    if(yaw_gyro != NULL)
    {
        *yaw_gyro = s_kalman_6axis.yaw_gyro;
    }
    if(yaw_stable != NULL)
    {
        *yaw_stable = s_yaw_stable;
    }
    if(yaw_final != NULL)
    {
        *yaw_final = s_state.yaw;
    }
}
