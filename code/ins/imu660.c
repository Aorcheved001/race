/*
 * imu660.c
 * IMU传感器驱动实现
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

 //-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------全局变量定义区-----------------------------------------------------------
imu_param imu_date = {0};                                                  // IMU数据
imu660_struct imu660 = {0};                                                // IMU660结构体

 //-------------------------------------------宏定义区------------------------------------------------------------
#define alpha 0.8f                                                         // 低通滤波器系数
#define imu963ra_gyro_transition_local(gyro_value) ((float)(gyro_value) / imu963ra_transition_factor[1])   // 陀螺仪数据转换宏
#define IMU_MAG_BIAS_FLASH_PAGE      63u                                   // 磁力计零偏存储Flash页号
#define IMU_MAG_BIAS_MAGIC           0x4D414742u                           // 魔数"MAGB"
#define IMU_MAG_BIAS_VERSION         2u                                    // 版本号2（支持缩放）

 //-------------------------------------------内部变量区-简单标定------------------------------------------------------------
static uint8 s_mag_calib_active = 0u;                                      // 磁力计校准激活标志
static uint32 s_mag_calib_samples = 0u;                                    // 磁力计校准采样数
static float s_mag_min_x = 0.0f;                                           // 磁力计X轴最小值
static float s_mag_min_y = 0.0f;                                           // 磁力计Y轴最小值
static float s_mag_min_z = 0.0f;                                           // 磁力计Z轴最小值
static float s_mag_max_x = 0.0f;                                           // 磁力计X轴最大值
static float s_mag_max_y = 0.0f;                                           // 磁力计Y轴最大值
static float s_mag_max_z = 0.0f;                                           // 磁力计Z轴最大值

 //-------------------------------------------内部变量区-椭球拟合标定------------------------------------------------------------
static uint8 s_mag_ellipsoid_active = 0u;                                 // 椭球拟合校准激活标志
static float s_mag_samples[MAG_CALIB_MAX_SAMPLES][3];                     // 采样数据缓冲区
static uint32 s_mag_sample_count = 0u;                                     // 当前采样数
static uint32 s_mag_sample_timer = 0u;                                     // 采样计时器

 //-------------------------------------------函数声明区------------------------------------------------------------
static void imu_mag_calib_update(imu_param *data_src);                     // 磁力计校准更新函数

 //-------------------------------------------极简线性代数库------------------------------------------------------------
typedef struct {
    float data[9];
} Mat3x3;

typedef struct {
    float data[3];
} Vec3;

static void mat3x3_zero(Mat3x3* m) {
    for(int i = 0; i < 9; i++) m->data[i] = 0.0f;
}

static void mat3x3_add(Mat3x3* a, Mat3x3* b, Mat3x3* out) {
    for(int i = 0; i < 9; i++) out->data[i] = a->data[i] + b->data[i];
}

static void mat3x3_scale(Mat3x3* m, float s, Mat3x3* out) {
    for(int i = 0; i < 9; i++) out->data[i] = m->data[i] * s;
}

static float mat3x3_det(Mat3x3* m) {
    return m->data[0] * (m->data[4] * m->data[8] - m->data[5] * m->data[7]) -
           m->data[1] * (m->data[3] * m->data[8] - m->data[5] * m->data[6]) +
           m->data[2] * (m->data[3] * m->data[7] - m->data[4] * m->data[6]);
}

static uint8 mat3x3_inv(Mat3x3* m, Mat3x3* out) {
    float det = mat3x3_det(m);
    if(fabsf(det) < 1e-10f) return 0;

    float inv_det = 1.0f / det;
    out->data[0] = (m->data[4] * m->data[8] - m->data[5] * m->data[7]) * inv_det;
    out->data[1] = (m->data[2] * m->data[7] - m->data[1] * m->data[8]) * inv_det;
    out->data[2] = (m->data[1] * m->data[5] - m->data[2] * m->data[4]) * inv_det;
    out->data[3] = (m->data[5] * m->data[6] - m->data[3] * m->data[8]) * inv_det;
    out->data[4] = (m->data[0] * m->data[8] - m->data[2] * m->data[6]) * inv_det;
    out->data[5] = (m->data[2] * m->data[3] - m->data[0] * m->data[5]) * inv_det;
    out->data[6] = (m->data[3] * m->data[7] - m->data[4] * m->data[6]) * inv_det;
    out->data[7] = (m->data[1] * m->data[6] - m->data[0] * m->data[7]) * inv_det;
    out->data[8] = (m->data[0] * m->data[4] - m->data[1] * m->data[3]) * inv_det;
    return 1;
}

static void vec3_zero(Vec3* v) {
    v->data[0] = v->data[1] = v->data[2] = 0.0f;
}

static void vec3_mul_mat3x3(Vec3* v, Mat3x3* m, Vec3* out) {
    out->data[0] = v->data[0] * m->data[0] + v->data[1] * m->data[3] + v->data[2] * m->data[6];
    out->data[1] = v->data[0] * m->data[1] + v->data[1] * m->data[4] + v->data[2] * m->data[7];
    out->data[2] = v->data[0] * m->data[2] + v->data[1] * m->data[5] + v->data[2] * m->data[8];
}

static void vec3_sub(Vec3* a, Vec3* b, Vec3* out) {
    out->data[0] = a->data[0] - b->data[0];
    out->data[1] = a->data[1] - b->data[1];
    out->data[2] = a->data[2] - b->data[2];
}

static float vec3_dot(Vec3* a, Vec3* b) {
    return a->data[0] * b->data[0] + a->data[1] * b->data[1] + a->data[2] * b->data[2];
}

 //-------------------------------------------椭球拟合算法------------------------------------------------------------
static uint8 solve_ellipsoid(float samples[][3], uint32_t n, Vec3* bias, Mat3x3* scale) {
    if(n < MAG_CALIB_MIN_SAMPLES) return 0;

    // 为了实现正确的硬铁和软铁校准，这里改为简化且更鲁棒的最小二乘法：
    // 假设椭球轴与坐标轴平行，求出 X, Y, Z 的极值，以此估算零偏和缩放因子
    float min_x = samples[0][0], max_x = samples[0][0];
    float min_y = samples[0][1], max_y = samples[0][1];
    float min_z = samples[0][2], max_z = samples[0][2];

    for(uint32_t i = 1; i < n; i++) {
        if(samples[i][0] < min_x) min_x = samples[i][0];
        if(samples[i][0] > max_x) max_x = samples[i][0];
        if(samples[i][1] < min_y) min_y = samples[i][1];
        if(samples[i][1] > max_y) max_y = samples[i][1];
        if(samples[i][2] < min_z) min_z = samples[i][2];
        if(samples[i][2] > max_z) max_z = samples[i][2];
    }

    // 1. 计算硬铁干扰 (Hard Iron) - 也就是椭球中心的偏移 (Bias)
    bias->data[0] = (max_x + min_x) / 2.0f;
    bias->data[1] = (max_y + min_y) / 2.0f;
    bias->data[2] = (max_z + min_z) / 2.0f;

    // 2. 计算软铁干扰 (Soft Iron) - 也就是三轴的缩放比例 (Scale)
    float delta_x = (max_x - min_x) / 2.0f;
    float delta_y = (max_y - min_y) / 2.0f;
    float delta_z = (max_z - min_z) / 2.0f;

    // 计算平均半径
    float avg_delta = (delta_x + delta_y + delta_z) / 3.0f;

    // 为了防止除以零或者太小的值导致 scale 爆炸
    if(delta_x < 1.0f) delta_x = 1.0f;
    if(delta_y < 1.0f) delta_y = 1.0f;
    if(delta_z < 1.0f) delta_z = 1.0f;

    mat3x3_zero(scale);
    scale->data[0] = avg_delta / delta_x;
    scale->data[4] = avg_delta / delta_y;
    scale->data[8] = avg_delta / delta_z;

    return 1;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      检查浮点数是否有效（非NaN）
 ////  @param      value          待检查的浮点数
 ////  @return     uint8          1表示有效，0表示NaN
 ////-------------------------------------------------------------------------------------------------------------------
static uint8 imu_float_is_valid_local(float value)
{
    return (value == value) ? 1u : 0u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      保存磁力计零偏到Flash
 ////  @param      data_src       IMU参数指针
 ////  @return     void
 ////  @note       先擦除第63页，再将磁力计零偏保存到Flash
 ////-------------------------------------------------------------------------------------------------------------------
void imu_mag_bias_save(const imu_param *data_src)
{
    uint32 flash_buf[EEPROM_PAGE_LENGTH];
    flash_data_union cvt;

    if(data_src == NULL)
    {
        return;
    }

    flash_erase_page(0, IMU_MAG_BIAS_FLASH_PAGE);                           // 擦除第63页Flash

    memset(flash_buf, 0xFF, sizeof(flash_buf));
    flash_buf[0] = IMU_MAG_BIAS_MAGIC;                                     // 写入魔数
    flash_buf[1] = IMU_MAG_BIAS_VERSION;                                   // 写入版本号2

    cvt.float_type = data_src->mag_x_bias;
    flash_buf[2] = cvt.uint32_type;                                        // 写入X轴零偏
    cvt.float_type = data_src->mag_y_bias;
    flash_buf[3] = cvt.uint32_type;                                        // 写入Y轴零偏
    cvt.float_type = data_src->mag_z_bias;
    flash_buf[4] = cvt.uint32_type;                                        // 写入Z轴零偏

    cvt.float_type = data_src->mag_x_scale;
    flash_buf[5] = cvt.uint32_type;                                        // 写入X轴缩放
    cvt.float_type = data_src->mag_y_scale;
    flash_buf[6] = cvt.uint32_type;                                        // 写入Y轴缩放
    cvt.float_type = data_src->mag_z_scale;
    flash_buf[7] = cvt.uint32_type;                                        // 写入Z轴缩放

    flash_write_page(0, IMU_MAG_BIAS_FLASH_PAGE, flash_buf, EEPROM_PAGE_LENGTH);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      从Flash加载磁力计零偏
 ////  @param      data_src       IMU参数指针
 ////  @return     uint8          1表示成功，0表示失败
 ////  @note       从Flash第63页读取磁力计零偏
 ////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_bias_load(imu_param *data_src)
{
    uint32 flash_buf[EEPROM_PAGE_LENGTH];
    flash_data_union cvt;
    float mag_x_bias, mag_y_bias, mag_z_bias;
    float mag_x_scale, mag_y_scale, mag_z_scale;

    if(data_src == NULL)
    {
        return 0u;
    }

    flash_read_page(0, IMU_MAG_BIAS_FLASH_PAGE, flash_buf, EEPROM_PAGE_LENGTH);
    if((flash_buf[0] != IMU_MAG_BIAS_MAGIC))
    {
        return 0u;                                                         // 魔数不匹配
    }

    cvt.uint32_type = flash_buf[2];
    mag_x_bias = cvt.float_type;                                           // 读取X轴零偏
    cvt.uint32_type = flash_buf[3];
    mag_y_bias = cvt.float_type;                                           // 读取Y轴零偏
    cvt.uint32_type = flash_buf[4];
    mag_z_bias = cvt.float_type;                                           // 读取Z轴零偏

    if(flash_buf[1] >= 2u)
    {
        cvt.uint32_type = flash_buf[5];
        mag_x_scale = cvt.float_type;                                       // 读取X轴缩放
        cvt.uint32_type = flash_buf[6];
        mag_y_scale = cvt.float_type;                                       // 读取Y轴缩放
        cvt.uint32_type = flash_buf[7];
        mag_z_scale = cvt.float_type;                                       // 读取Z轴缩放
    }
    else
    {
        mag_x_scale = 1.0f;
        mag_y_scale = 1.0f;
        mag_z_scale = 1.0f;
    }

    if(!imu_float_is_valid_local(mag_x_bias) || !imu_float_is_valid_local(mag_y_bias) || !imu_float_is_valid_local(mag_z_bias) ||
       !imu_float_is_valid_local(mag_x_scale) || !imu_float_is_valid_local(mag_y_scale) || !imu_float_is_valid_local(mag_z_scale))
    {
        return 0u;                                                         // 数据无效（包含NaN）
    }

    data_src->mag_x_bias = mag_x_bias;
    data_src->mag_y_bias = mag_y_bias;
    data_src->mag_z_bias = mag_z_bias;
    data_src->mag_x_scale = mag_x_scale;
    data_src->mag_y_scale = mag_y_scale;
    data_src->mag_z_scale = mag_z_scale;
    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      更新磁力计校准数据（简单方法）
 ////  @param      data_src       IMU参数指针
 ////  @return     void
 ////  @note       收集磁力计数据用于校准
 ////-------------------------------------------------------------------------------------------------------------------
static void imu_mag_calib_update(imu_param *data_src)
{
    if((data_src == NULL) || !s_mag_calib_active)
    {
        return;
    }

    float mag_x = (float)imu963ra_mag_x;
    float mag_y = (float)imu963ra_mag_y;
    float mag_z = (float)imu963ra_mag_z;

    if(s_mag_calib_samples == 0u)
    {
        s_mag_min_x = s_mag_max_x = mag_x;
        s_mag_min_y = s_mag_max_y = mag_y;
        s_mag_min_z = s_mag_max_z = mag_z;
    }
    else
    {
        if(mag_x < s_mag_min_x) s_mag_min_x = mag_x;
        if(mag_x > s_mag_max_x) s_mag_max_x = mag_x;
        if(mag_y < s_mag_min_y) s_mag_min_y = mag_y;
        if(mag_y > s_mag_max_y) s_mag_max_y = mag_y;
        if(mag_z < s_mag_min_z) s_mag_min_z = mag_z;
        if(mag_z > s_mag_max_z) s_mag_max_z = mag_z;
    }

    s_mag_calib_samples++;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      处理IMU数据
 ////  @param      data_src       IMU参数指针
 ////  @return     void
 ////  @note       获取并处理IMU传感器数据，包括滤波和单位转换
 ////-------------------------------------------------------------------------------------------------------------------
void date_handle(imu_param *data_src)
{
    imu963ra_get_mag();                                                    // 获取磁力计数据
    imu963ra_get_gyro();                                                   // 获取陀螺仪数据
    imu963ra_get_acc();                                                    // 获取加速度计数据

    imu_mag_calib_update(data_src);                                        // 更新磁力计校准

    if(s_mag_ellipsoid_active && s_mag_sample_count < MAG_CALIB_MAX_SAMPLES)
    {
        s_mag_sample_timer++;
        // 控制采样速度：7.5秒采集500个点，约每15ms采样一次
        if(s_mag_sample_timer >= 15u)
        {
            s_mag_samples[s_mag_sample_count][0] = (float)imu963ra_mag_x;
            s_mag_samples[s_mag_sample_count][1] = (float)imu963ra_mag_y;
            s_mag_samples[s_mag_sample_count][2] = (float)imu963ra_mag_z;
            s_mag_sample_count++;
            s_mag_sample_timer = 0u;
        }
    }

    imu660.data_Raw.acc_x = (float)imu963ra_acc_x;                         // 原始加速度X
    imu660.data_Raw.acc_y = (float)imu963ra_acc_y;                         // 原始加速度Y
    imu660.data_Raw.acc_z = (float)imu963ra_acc_z;                         // 原始加速度Z
    imu660.data_Raw.gyro_x = (float)imu963ra_gyro_x;                       // 原始角速度X
    imu660.data_Raw.gyro_y = (float)imu963ra_gyro_y;                       // 原始角速度Y
    imu660.data_Raw.gyro_z = (float)imu963ra_gyro_z;                       // 原始角速度Z
    imu660.data_Raw.mag_x = (float)imu963ra_mag_x;                         // 原始磁力计X
    imu660.data_Raw.mag_y = (float)imu963ra_mag_y;                         // 原始磁力计Y
    imu660.data_Raw.mag_z = (float)imu963ra_mag_z;                         // 原始磁力计Z

    float acc_x_offset = imu963ra_acc_x - data_src->acc_x_bias;            // 加速度X偏移
    float acc_y_offset = imu963ra_acc_y - data_src->acc_y_bias;            // 加速度Y偏移
    float acc_z_offset = imu963ra_acc_z + data_src->acc_z_bias;            // 加速度Z偏移
    float gyro_x_offset = imu963ra_gyro_x - data_src->gyro_x_bias;         // 角速度X偏移
    float gyro_y_offset = imu963ra_gyro_y - data_src->gyro_y_bias;         // 角速度Y偏移
    float gyro_z_offset = imu963ra_gyro_z - data_src->gyro_z_bias;         // 角速度Z偏移

    imu660.data_Ripen.acc_x = (imu963ra_acc_transition(acc_x_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_x * (1.0f - alpha);    // 加速度X滤波
    imu660.data_Ripen.acc_y = (imu963ra_acc_transition(acc_y_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_y * (1.0f - alpha);    // 加速度Y滤波
    imu660.data_Ripen.acc_z = (imu963ra_acc_transition(acc_z_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_z * (1.0f - alpha);    // 加速度Z滤波
    imu660.data_Ripen.gyro_x = (imu963ra_gyro_transition_local(gyro_x_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_x * (1.0f - alpha);   // 角速度X滤波
    imu660.data_Ripen.gyro_y = (imu963ra_gyro_transition_local(gyro_y_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_y * (1.0f - alpha);   // 角速度Y滤波
    imu660.data_Ripen.gyro_z = (imu963ra_gyro_transition_local(gyro_z_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_z * (1.0f - alpha);   // 角速度Z滤波

    float mx = ((float)imu963ra_mag_x - data_src->mag_x_bias) * data_src->mag_x_scale;
    float my = ((float)imu963ra_mag_y - data_src->mag_y_bias) * data_src->mag_y_scale;
    float mz = ((float)imu963ra_mag_z - data_src->mag_z_bias) * data_src->mag_z_scale;

    // 注意：如果已经做了椭球拟合标定，mx/my/mz 已经被 scale 归一化到 1.0 附近
    // 这里不再使用 imu963ra_mag_transition (除以 3000) 进行转换，否则会导致数据极小被判定为无效
    imu660.data_Ripen.mag_x = mx;
    imu660.data_Ripen.mag_y = my;
    imu660.data_Ripen.mag_z = mz;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      自动校准陀螺仪Z轴
 ////  @param      data_src       IMU参数指针
 ////  @param      samples        采样次数
 ////  @return     void
 ////  @note       计算陀螺仪Z轴偏置
 ////-------------------------------------------------------------------------------------------------------------------
void imu_gyro_z_autocalib(imu_param *data_src, unsigned int samples)
{
    if (!data_src || samples == 0) return;

    float sum_z = 0.0f;
    for (unsigned int i = 0; i < samples; i++)
    {
        imu963ra_get_gyro();
        sum_z += (float)imu963ra_gyro_z;
        system_delay_ms(2);
    }

    data_src->gyro_z_bias = sum_z / (float)samples;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      初始化IMU偏置
 ////  @param      data_src       IMU参数指针
 ////  @return     void
 ////  @note       初始化所有传感器偏置为0
 ////-------------------------------------------------------------------------------------------------------------------
void imu_bias_init(imu_param *data_src)
{
    data_src->gyro_x_bias = 0.0f;                                          // X轴角速度零偏清零
    data_src->gyro_y_bias = 0.0f;                                          // Y轴角速度零偏清零
    data_src->gyro_z_bias = 0.0f;                                          // Z轴角速度零偏清零
    data_src->acc_x_bias = 0.0f;                                           // X轴加速度零偏清零
    data_src->acc_y_bias = 0.0f;                                           // Y轴加速度零偏清零
    data_src->acc_z_bias = 0.0f;                                           // Z轴加速度零偏清零
    data_src->mag_x_bias = 0.0f;                                           // X轴磁力计零偏清零
    data_src->mag_y_bias = 0.0f;                                           // Y轴磁力计零偏清零
    data_src->mag_z_bias = 0.0f;                                           // Z轴磁力计零偏清零
    data_src->mag_x_scale = 1.0f;                                           // X轴磁力计缩放初始化为1
    data_src->mag_y_scale = 1.0f;                                           // Y轴磁力计缩放初始化为1
    data_src->mag_z_scale = 1.0f;                                           // Z轴磁力计缩放初始化为1
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      开始磁力计校准（椭球拟合法）
////  @param      void
////  @return     void
////  @note       启动磁力计校准过程
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_calib_start(void)
{
    s_mag_ellipsoid_active = 1u;
    s_mag_sample_count = 0u;
    s_mag_sample_timer = 0u;
    s_mag_calib_active = 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      完成磁力计校准（椭球拟合法）
////  @param      data_src       IMU参数指针
////  @return     uint8          1表示成功，0表示失败
////  @note       计算并保存磁力计偏置和缩放到Flash
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_finish(imu_param *data_src)
{
    if((data_src == NULL) || !s_mag_ellipsoid_active)
    {
        return 0u;
    }

    s_mag_ellipsoid_active = 0u;

    if(s_mag_sample_count < MAG_CALIB_MIN_SAMPLES)
    {
        return 0u;
    }

    Vec3 bias;
    Mat3x3 scale;
    vec3_zero(&bias);
    mat3x3_zero(&scale);

    if(!solve_ellipsoid(s_mag_samples, s_mag_sample_count, &bias, &scale))
    {
        return 0u;
    }

    data_src->mag_x_bias = bias.data[0];
    data_src->mag_y_bias = bias.data[1];
    data_src->mag_z_bias = bias.data[2];
    data_src->mag_x_scale = scale.data[0];
    data_src->mag_y_scale = scale.data[4];
    data_src->mag_z_scale = scale.data[8];

    imu_mag_bias_save(data_src);
    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      检查磁力计校准是否激活
////  @param      void
////  @return     uint8          1表示激活，0表示未激活
////  @note       返回磁力计校准状态
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_is_active(void)
{
    return s_mag_ellipsoid_active;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      开始椭球拟合磁力计校准
 ////  @param      void
 ////  @return     void
 ////-------------------------------------------------------------------------------------------------------------------
void imu_mag_calib_ellipsoid_start(void)
{
    s_mag_ellipsoid_active = 1u;
    s_mag_sample_count = 0u;
    s_mag_sample_timer = 0u;
    s_mag_calib_active = 0u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      更新椭球拟合数据
 ////  @param      mx, my, mz    磁力计数据
 ////  @return     uint8          1表示成功，0表示缓冲区满
 ////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_ellipsoid_update(float mx, float my, float mz)
{
    if(!s_mag_ellipsoid_active || s_mag_sample_count >= MAG_CALIB_MAX_SAMPLES)
    {
        return 0u;
    }

    s_mag_samples[s_mag_sample_count][0] = mx;
    s_mag_samples[s_mag_sample_count][1] = my;
    s_mag_samples[s_mag_sample_count][2] = mz;
    s_mag_sample_count++;

    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      完成椭球拟合并计算参数
 ////  @param      data_src       IMU参数指针
 ////  @return     uint8          1表示成功，0表示失败
 ////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_ellipsoid_finish(imu_param *data_src)
{
    if((data_src == NULL) || !s_mag_ellipsoid_active)
    {
        return 0u;
    }

    s_mag_ellipsoid_active = 0u;

    if(s_mag_sample_count < MAG_CALIB_MIN_SAMPLES)
    {
        return 0u;
    }

    Vec3 bias;
    Mat3x3 scale;
    vec3_zero(&bias);
    mat3x3_zero(&scale);

    if(!solve_ellipsoid(s_mag_samples, s_mag_sample_count, &bias, &scale))
    {
        return 0u;
    }

    data_src->mag_x_bias = bias.data[0];
    data_src->mag_y_bias = bias.data[1];
    data_src->mag_z_bias = bias.data[2];
    data_src->mag_x_scale = scale.data[0];
    data_src->mag_y_scale = scale.data[4];
    data_src->mag_z_scale = scale.data[8];

    imu_mag_bias_save(data_src);
    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取当前采样点数
////  @param      void
////  @return     uint32         当前采样数
////-------------------------------------------------------------------------------------------------------------------
uint32 imu_mag_calib_get_sample_count(void)
{
    return s_mag_sample_count;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      向上位机发送磁力计原始数据用于绘图
////  @param      void
////  @return     void
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_send_raw_data_to_pc(void)
{
    // 直接发送原始数据（未减去零偏）
    // 使用 sprintf 将数据格式化到字符串，然后通过 wireless_uart_send_string 发送
    char send_buffer[64];
    sprintf(send_buffer, "%.2f,%.2f,%.2f\r\n", imu660.data_Raw.mag_x, imu660.data_Raw.mag_y, imu660.data_Raw.mag_z);
    wireless_uart_send_string(send_buffer);
}

////-------------------------------------------------------------------------------------------------------------------
 ////  @brief      静态IMU测试
 ////  @param      data_src       IMU参数指针
 ////  @return     void
 ////  @note       计算加速度计偏置
 ////-------------------------------------------------------------------------------------------------------------------
void static_imu_test(imu_param *data_src)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    unsigned int count = 0;
    const unsigned int max_samples = 20000;

    printf("Starting Calibration... Please keep the car still!\n");
    system_delay_ms(500);

    for(count = 0; count < max_samples; count++)
    {
        date_handle(data_src);
        sum_x += imu660.data_Raw.acc_x;                                    // 累加X轴加速度
        sum_y += imu660.data_Raw.acc_y;                                    // 累加Y轴加速度
        sum_z += imu660.data_Raw.acc_z;                                    // 累加Z轴加速度
        system_delay_ms(5);
    }

    data_src->acc_x_bias = sum_x / (float)count;                           // 计算X轴加速度零偏
    data_src->acc_y_bias = sum_y / (float)count;                           // 计算Y轴加速度零偏
    data_src->acc_z_bias = sum_z / (float)count;                           // 计算Z轴加速度零偏

    printf("Accel biases calculated:\nX: %f\nY: %f\nZ: %f\n",
           data_src->acc_x_bias, data_src->acc_y_bias, data_src->acc_z_bias);
}
