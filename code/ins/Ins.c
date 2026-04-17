/*
 * Ins.c
 * INS????????
 */

#include "zf_common_headfile.h"

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
    return angle;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ????????????????
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void Init_config(void)
{
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
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_init(void)
{
    memset(&s_kalman_6axis, 0, sizeof(s_kalman_6axis));                  // ???????????
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
 ////  @brief      ????????????
 ////  @param      roll        ???roll
 ////  @param      pitch       ???pitch
 ////  @param      yaw         ???yaw
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_kalman_6axis_reset(float roll, float pitch, float yaw)
{
    ins_kalman_6axis_init();                                             // ?????????
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
 ////  @brief      ????????????
 ////  @param      gyro_x      ??????X???????rad/s??
 ////  @param      gyro_y      ??????Y???????rad/s??
 ////  @param      gyro_z      ??????Z???????rad/s??
 ////  @param      acc_x       ??????X??????
 ////  @param      acc_y       ??????Y??????
 ////  @param      acc_z       ??????Z??????
 ////  @return     float       ???????yaw
 ////-------------------------------------------------------------------------------------------------------------------
static float ins_kalman_6axis_update(float gyro_x, float gyro_y, float gyro_z,
                                     float acc_x, float acc_y, float acc_z)
{
    float roll = s_kalman_6axis.Xk[0];
    float pitch = s_kalman_6axis.Xk[1];
    float cos_pitch = cosf(pitch);

    if(fabsf(cos_pitch) < 1e-6f)
    {
        cos_pitch = (cos_pitch >= 0.0f) ? 1e-6f : -1e-6f;                // ???????
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
    s_kalman_6axis.K[2] = 0.0f;                                          // yaw?????????

    {
        float acc_yz_norm = sqrtf(acc_y * acc_y + acc_z * acc_z);
        if(acc_yz_norm < 1e-6f)
        {
            acc_yz_norm = 1e-6f;
        }

        s_kalman_6axis.Zk[0] = atan2f(acc_y, acc_z);                     // ????????roll
        s_kalman_6axis.Zk[1] = -atan2f(acc_x, acc_yz_norm);              // ????????pitch
        s_kalman_6axis.Zk[2] = 0.0f;
    }

    s_kalman_6axis.Xk[0] = (1.0f - s_kalman_6axis.K[0]) * s_kalman_6axis.Xk_[0] +
                           s_kalman_6axis.K[0] * s_kalman_6axis.Zk[0];
    s_kalman_6axis.Xk[1] = (1.0f - s_kalman_6axis.K[1]) * s_kalman_6axis.Xk_[1] +
                           s_kalman_6axis.K[1] * s_kalman_6axis.Zk[1];
    s_kalman_6axis.Xk[2] = normalize_angle_rad(s_kalman_6axis.Xk_[2]);   // yaw??????

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
        float gravity_x = -9.80665f * sin_pitch;                         // ?????????????
        float gravity_y = 9.80665f * sin_roll * cos_pitch_now;
        float gravity_z = 9.80665f * cos_roll * cos_pitch_now;

        s_kalman_6axis.ax_linear = acc_x - gravity_x;
        s_kalman_6axis.ay_linear = acc_y - gravity_y;
        s_kalman_6axis.az_linear = acc_z - gravity_z;
    }

    return s_kalman_6axis.yaw_gyro;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ???EKF?????????
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_init(void)
{
    memset(&s_yaw_ekf, 0, sizeof(YawEKF2State));
    s_yaw_ekf.N_psi = s_config.Q_yaw;
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
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_reset(float initial_yaw)
{
    yaw_ekf2_init();
    s_yaw_ekf.x[0] = normalize_angle_rad(initial_yaw);
    s_yaw_ekf.x[1] = 0.0f;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ???EKF????????
 ////  @param      gyro_z      ?????Z??????
 ////  @param      dt          ?????
 ////  @param      is_stationary ????
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_predict(float gyro_z, float dt, uint8_t is_stationary)
{
    s_yaw_ekf.x[0] += (gyro_z - s_yaw_ekf.x[1]) * dt;
    s_yaw_ekf.x[0] = normalize_angle_rad(s_yaw_ekf.x[0]);
    s_yaw_ekf.yaw_predict = s_yaw_ekf.x[0];

    // ???????????? N_b (????????bias)
    float Nb = is_stationary ? s_yaw_ekf.N_b : s_yaw_ekf.N_b_frozen;

    // ????????????????????? Q?????????
    float q00 = s_yaw_ekf.N_psi * dt + Nb * dt * dt * dt / 3.0f;
    float q01 = -Nb * dt * dt / 2.0f;
    float q11 = Nb * dt;

    float P00 = s_yaw_ekf.P[0][0];
    float P01 = s_yaw_ekf.P[0][1];
    float P11 = s_yaw_ekf.P[1][1];

    s_yaw_ekf.P[0][0] = P00 - 2.0f * dt * P01 + dt * dt * P11 + q00;
    s_yaw_ekf.P[0][1] = P01 - dt * P11 + q01;
    s_yaw_ekf.P[1][0] = s_yaw_ekf.P[0][1]; // ????????
    s_yaw_ekf.P[1][1] = P11 + q11;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ???EKF?????????
 ////  @param      mag_yaw     ?????????????
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void yaw_ekf2_update(float mag_yaw)
{
    float y_tilde = normalize_angle_rad(mag_yaw - s_yaw_ekf.x[0]);
    float S = s_yaw_ekf.P[0][0] + s_yaw_ekf.R;

    float K0 = s_yaw_ekf.P[0][0] / S;
    float K1 = s_yaw_ekf.P[1][0] / S;

    // Round1 tuning: 收紧异常残差抑制，磁力计毛刺不应超过3度 [was 10度/0.174rad]
    float max_innovation = 0.052f;    // 3 degrees
    if (y_tilde > max_innovation) y_tilde = max_innovation;
    if (y_tilde < -max_innovation) y_tilde = -max_innovation;

    // Round1 tuning: 放宽K1限制，允许更快的bias修正速度 [was 0.05]
    // 静止时bias应该快速收敛到真实零偏值
    if (K1 > 0.15f) K1 = 0.15f;
    if (K1 < -0.15f) K1 = -0.15f;

    s_yaw_ekf.x[0] = normalize_angle_rad(s_yaw_ekf.x[0] + K0 * y_tilde);
    s_yaw_ekf.x[1] += K1 * y_tilde;

    // 限制零偏绝对值大小 (防飞车保护：假设陀螺仪零偏不可能超过 10 deg/s)
    float max_bias = 10.0f * INS_PI / 180.0f;
    if (s_yaw_ekf.x[1] > max_bias) s_yaw_ekf.x[1] = max_bias;
    if (s_yaw_ekf.x[1] < -max_bias) s_yaw_ekf.x[1] = -max_bias;

    // ????????????????????????
    float p01_old = s_yaw_ekf.P[0][1];

    s_yaw_ekf.P[0][0] = (1.0f - K0) * s_yaw_ekf.P[0][0];
    s_yaw_ekf.P[0][1] = (1.0f - K0) * p01_old;
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
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
static void ins_position_update(float yaw, int16_t tick_left, int16_t tick_right)
{
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

 //-------------------------------------------??????????------------------------------------------------------------

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ?????INS??
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void Ins_init(void)
{
    Init_config();                                                       // ????????????
    ins_kalman_6axis_init();                                             // ?????????????
    yaw_ekf2_init();                                                     // ????????EKF
    s_state.x = 0.0f;
    s_state.y = 0.0f;
    s_state.yaw = 0.0f;
    s_pos_x = 0.0f;
    s_pos_y = 0.0f;

    s_mag_initial_yaw_captured = 0u;                                     // 复位磁力计初始朝向捕获标志

    s_initialized = 1u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ????INS??
 ////  @param      x           ???X????
 ////  @param      y           ???Y????
 ////  @param      theta       ????????
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

    // 当重置位置和角度时，复位捕获标志
    // 这样下次收到有效的磁力计数据时，会自动与当前重置的角度对齐
    s_mag_initial_yaw_captured = 0u;

    ins_kalman_6axis_reset(0.0f, 0.0f, normalized_theta);
    yaw_ekf2_reset(normalized_theta);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ????INS????????
 ////  @param      config      ???????????
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
 ////  @brief      ????INS??
 ////  @param      input       INS????????
 ////  @param      dt_s        ?????????
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

    s_kalman_6axis.T = dt_s;                                             // ??????????????????????

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
        // 注意：根据硬件贴片方向，磁力计的偏航角计算可能与陀螺仪积分的旋向相反。
        // 如果陀螺仪左转是正角，而磁力计左转读数变小，则需要在这里加上负号统一坐标系。
        s_mag_yaw_raw = normalize_angle_rad(-atan2f(imu660.data_Ripen.mag_y, imu660.data_Ripen.mag_x));

        // 捕获上电（或reset后）的初始磁力计航向
        if (!s_mag_initial_yaw_captured)
        {
            // 我们希望此时的相对磁航向 (mag_yaw_rel) 完全等于当前 EKF 已经积分出的航向 (s_yaw_ekf.x[0])
            // 这样就不会因为在没有磁力计数据期间的旋转而产生跳变
            // mag_yaw_rel = raw - initial = current_ekf_yaw
            // => initial = raw - current_ekf_yaw
            s_initial_mag_yaw = normalize_angle_rad(s_mag_yaw_raw - s_yaw_ekf.x[0]);
            s_mag_initial_yaw_captured = 1u;
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

    (void)input->omega_rad_s;                                            // ????????????
    (void)input->mag_yaw_rad;
    (void)s_config.Q_yaw;
    (void)s_config.R_mag;
    (void)s_config.wheelbase;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      ??????INS??
 ////  @param      void
 ////  @return     const INS_State*
 ////-------------------------------------------------------------------------------------------------------------------
const INS_State* Ins_get_state(void)
{
    return &s_state;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取各层yaw值
//  @param      yaw_gyro     陀螺仪预测yaw（输出）
//  @param      yaw_mag_raw  原始绝对磁航向（输出）
//  @param      yaw_mag_rel  计算出的相对磁航向（输出）
//  @param      yaw_ekf      EKF最终融合yaw（输出）
//  @return     void
//-------------------------------------------------------------------------------------------------------------------
void Ins_get_yaw_layers(float *yaw_gyro, float *yaw_mag_raw, float *yaw_mag_rel, float *yaw_ekf)
{
    if(yaw_gyro != NULL)
    {
        // 返回6轴卡尔曼算出来的陀螺仪积分yaw，而不是EKF里的预测值
        *yaw_gyro = s_kalman_6axis.yaw_gyro;
    }
    if(yaw_mag_raw != NULL)
    {
        *yaw_mag_raw = s_mag_yaw_raw;
    }
    if(yaw_mag_rel != NULL)
    {
        // 返回滤波后的磁力计相对航向，方便上位机观察平滑效果
        *yaw_mag_rel = s_mag_yaw_rel_filtered;
    }
    if(yaw_ekf != NULL)
    {
        *yaw_ekf = s_state.yaw;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      相对磁航向单元测试与回归测试
//  @param      void
//  @return     void
//  @note       验证四个初始朝向（0, 90, 180, 270度），确认上电后的磁航向和EKF是否稳定在 0度
//-------------------------------------------------------------------------------------------------------------------
void Ins_test_relative_mag(void)
{
    float test_angles_deg[] = {0.0f, 90.0f, 180.0f, 270.0f};
    INS_Input dummy_input = {0};
    dummy_input.mag_valid = 1;
    float dt = 0.004f; // 4ms

    printf("\n=== 相对磁航向 单元测试开始 ===\n");
    for(int i = 0; i < 4; i++)
    {
        float init_angle_rad = test_angles_deg[i] * INS_PI / 180.0f;

        // 模拟上电
        Ins_init();

        // 我们模拟磁力计在当前方向上的绝对输出
        // mag_yaw = atan2(y, x), 所以 y = sin(angle), x = cos(angle)
        imu660.data_Ripen.mag_x = cosf(init_angle_rad);
        imu660.data_Ripen.mag_y = sinf(init_angle_rad);

        // 陀螺仪和加速度计处于静止状态
        imu660.data_Ripen.gyro_x = 0;
        imu660.data_Ripen.gyro_y = 0;
        dummy_input.gyro_z_rad_s = 0;
        imu660.data_Ripen.acc_x = 0;
        imu660.data_Ripen.acc_y = 0;
        imu660.data_Ripen.acc_z = 9.8f;

        dummy_input.v_mps = 0;
        dummy_input.omega_rad_s = 0;

        // 连续运行EKF一段时间，让它收敛
        for(int step = 0; step < 500; step++) // 运行2秒
        {
            Ins_update(&dummy_input, dt);
        }

        float yaw_gyro, yaw_mag_raw, yaw_mag_rel, yaw_ekf;
        Ins_get_yaw_layers(&yaw_gyro, &yaw_mag_raw, &yaw_mag_rel, &yaw_ekf);

        float err_deg = fabsf(yaw_ekf * 180.0f / INS_PI);
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
}
