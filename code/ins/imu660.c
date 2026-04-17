/*
 * imu660.c
 * IMU传感器驱动处理文件
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

//-------------------------------------------头文件包含------------------------------------------------------------
#include "zf_common_headfile.h"
#include "calibration_params.h"

//-------------------------------------------全局变量定义-----------------------------------------------------------
imu_param imu_date = {0};                                                  // IMU数据结构体
imu660_struct imu660 = {0};                                                // IMU660数据结构体

//-------------------------------------------宏定义------------------------------------------------------------
#define alpha 0.8f                                                         // 互补滤波系数
#define imu963ra_gyro_transition_local(gyro_value) ((float)(gyro_value) / imu963ra_transition_factor[1])   // 陀螺仪数据转换
#define IMU_MAG_BIAS_FLASH_PAGE      63u                                   // 磁力计校准参数存储Flash页
#define IMU_MAG_BIAS_MAGIC           0x4D414742u                           // 校准参数标识"MAGB"
#define IMU_MAG_BIAS_VERSION         3u                                    // 校准参数版本3(支持3x3软铁矩阵)

//-------------------------------------------静态变量-简易校准------------------------------------------------------------
static uint8 s_mag_calib_active = 0u;                                      // 磁力计简易校准使能标志
static uint32 s_mag_calib_samples = 0u;                                    // 磁力计简易校准采样数
static float s_mag_min_x = 0.0f;                                           // 磁力计X轴最小值
static float s_mag_min_y = 0.0f;                                           // 磁力计Y轴最小值
static float s_mag_min_z = 0.0f;                                           // 磁力计Z轴最小值
static float s_mag_max_x = 0.0f;                                           // 磁力计X轴最大值
static float s_mag_max_y = 0.0f;                                           // 磁力计Y轴最大值
static float s_mag_max_z = 0.0f;                                           // 磁力计Z轴最大值

//-------------------------------------------静态变量-椭球拟合校准------------------------------------------------------------
static uint8 s_mag_ellipsoid_active = 0u;                                 // 磁力计椭球拟合校准使能
static float s_mag_samples[MAG_CALIB_MAX_SAMPLES][3];                     // 磁力计校准采样数据缓存
static uint32 s_mag_sample_count = 0u;                                     // 磁力计校准采样计数
static uint32 s_mag_sample_timer = 0u;                                     // 磁力计采样定时计数器

//-------------------------------------------函数声明------------------------------------------------------------
static void imu_mag_calib_update(imu_param *data_src);                     // 磁力计简易校准更新

//-------------------------------------------矩阵向量运算函数------------------------------------------------------------
typedef struct {
    float data[9];
} Mat3x3;

typedef struct {
    float data[3];
} Vec3;

// 3x3矩阵清零
static void mat3x3_zero(Mat3x3* m) {
    for(int i = 0; i < 9; i++) m->data[i] = 0.0f;
}

// 3x3矩阵加法
static void mat3x3_add(Mat3x3* a, Mat3x3* b, Mat3x3* out) {
    for(int i = 0; i < 9; i++) out->data[i] = a->data[i] + b->data[i];
}

// 3x3矩阵数乘
static void mat3x3_scale(Mat3x3* m, float s, Mat3x3* out) {
    for(int i = 0; i < 9; i++) out->data[i] = m->data[i] * s;
}

// 3x3矩阵行列式计算
static float mat3x3_det(Mat3x3* m) {
    return m->data[0] * (m->data[4] * m->data[8] - m->data[5] * m->data[7]) -
           m->data[1] * (m->data[3] * m->data[8] - m->data[5] * m->data[6]) +
           m->data[2] * (m->data[3] * m->data[7] - m->data[4] * m->data[6]);
}

// 3x3矩阵求逆
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

// 3维向量清零
static void vec3_zero(Vec3* v) {
    v->data[0] = v->data[1] = v->data[2] = 0.0f;
}

// 3维向量乘3x3矩阵
static void vec3_mul_mat3x3(Vec3* v, Mat3x3* m, Vec3* out) {
    out->data[0] = v->data[0] * m->data[0] + v->data[1] * m->data[3] + v->data[2] * m->data[6];
    out->data[1] = v->data[0] * m->data[1] + v->data[1] * m->data[4] + v->data[2] * m->data[7];
    out->data[2] = v->data[0] * m->data[2] + v->data[1] * m->data[5] + v->data[2] * m->data[8];
}

// 3维向量减法
static void vec3_sub(Vec3* a, Vec3* b, Vec3* out) {
    out->data[0] = a->data[0] - b->data[0];
    out->data[1] = a->data[1] - b->data[1];
    out->data[2] = a->data[2] - b->data[2];
}

// 3维向量点积
static float vec3_dot(Vec3* a, Vec3* b) {
    return a->data[0] * b->data[0] + a->data[1] * b->data[1] + a->data[2] * b->data[2];
}

//-------------------------------------------椭球拟合核心算法------------------------------------------------------------
// Cholesky分解: A = L * L^T (9x9对称正定矩阵分解), L为下三角矩阵
static uint8 cholesky_9x9(float A[9][9], float L[9][9]) {
    int i, j, k;
    for(i = 0; i < 9; i++) for(j = 0; j < 9; j++) L[i][j] = 0.0f;
    for(i = 0; i < 9; i++) {
        float sum = 0.0f;
        for(k = 0; k < i; k++) sum += L[i][k] * L[i][k];
        float val = A[i][i] - sum;
        if(val <= 1e-20f) return 0;
        L[i][i] = sqrtf(val);
        for(j = i + 1; j < 9; j++) {
            sum = 0.0f;
            for(k = 0; k < i; k++) sum += L[j][k] * L[i][k];
            L[j][i] = (A[i][j] - sum) / L[i][i];
        }
    }
    return 1;
}

// 前向替换: 求解 L * x = b (L为下三角矩阵, 输出解x)
static void fwd_sub_9(const float L[9][9], float x[9]) {
    int i, j;
    for(i = 0; i < 9; i++) {
        float sum = x[i];
        for(j = 0; j < i; j++) sum -= L[i][j] * x[j];
        x[i] = sum / L[i][i];
    }
}

// 后向替换: 求解 L^T * x = b (L^T为上三角矩阵, 输出解x)
static void bwd_sub_9(const float L[9][9], float x[9]) {
    int i, j;
    for(i = 8; i >= 0; i--) {
        float sum = x[i];
        for(j = i + 1; j < 9; j++) sum -= L[j][i] * x[j];
        x[i] = sum / L[i][i];
    }
}

// 雅可比迭代: 求解9x9实对称矩阵特征值与特征向量
// 输入:A(对称矩阵), 输出:A(对角化矩阵), V(列=特征向量)
static void jacobi_9x9(float A[9][9], float V[9][9]) {
    int i, j, p, q, iter;
    for(i = 0; i < 9; i++) for(j = 0; j < 9; j++) V[i][j] = (i == j) ? 1.0f : 0.0f;
    for(iter = 0; iter < 500; iter++) {
        for(p = 0; p < 8; p++) {
            for(q = p + 1; q < 9; q++) {
                if(fabsf(A[p][q]) < 1e-18f) continue;
                float theta;
                if(fabsf(A[p][p] - A[q][q]) < 1e-15f)
                    theta = (A[p][q] > 0.0f) ? 0.785398163f : -0.785398163f;
                else
                    theta = 0.5f * atan2f(2.0f * A[p][q], A[p][p] - A[q][q]);
                float ct = cosf(theta), st = sinf(theta);
                float App =  ct*ct*A[p][p] - 2*ct*st*A[p][q] + st*st*A[q][q];
                float Aqq = st*st*A[p][p] + 2*ct*st*A[p][q] + ct*ct*A[q][q];
                float Ap_old[9], Aq_old[9];
                for(i = 0; i < 9; i++) { Ap_old[i] = A[p][i]; Aq_old[i] = A[q][i]; }
                A[p][p] = App; A[q][q] = Aqq; A[p][q] = 0.0f; A[q][p] = 0.0f;
                for(i = 0; i < 9; i++) {
                    if(i != p && i != q) {
                        A[p][i] =  ct * Ap_old[i] - st * Aq_old[i]; A[i][p] = A[p][i];
                        A[q][i] =  st * Ap_old[i] + ct * Aq_old[i]; A[i][q] = A[q][i];
                    }
                }
                for(i = 0; i < 9; i++) {
                    float vp = V[i][p], vq = V[i][q];
                    V[i][p] =  ct * vp - st * vq;
                    V[i][q] =  st * vp + ct * vq;
                }
            }
        }
        // 判断收敛
        float max_off = 0.0f;
        for(i = 0; i < 8; i++) for(j = i + 1; j < 9; j++) {
            float v = fabsf(A[i][j]); if(v > max_off) max_off = v;
        }
        if(max_off < 1e-14f) break;
    }
}

// 磁力计椭球拟合校准: 求解硬铁偏移 + 软铁校正矩阵
// 约束条件: 4*a*c - b^2 > 0 (保证拟合结果为椭球/球体)
// 求解原理: S*v = lambda*C*v -> Cholesky分解 -> Jacobi迭代
static uint8 solve_ellipsoid(float samples[][3], uint32_t n, Vec3* bias, Mat3x3* scale) {
    if(n < MAG_CALIB_MIN_SAMPLES) return 0;

    int i, j, k;

    // ---- 步骤1: 构建Gram矩阵 S = D^T * D 和向量 b = D^T * 1 ----
    // 数据向量: [x^2, y^2, z^2, 2xy, 2xz, 2yz, 2x, 2y, 2z]
    float S[9][9];
    float b_vec[9];

    for(i = 0; i < 9; i++) { b_vec[i] = 0.0f; for(j = 0; j < 9; j++) S[i][j] = 0.0f; }

    for(k = 0; k < (int)n; k++) {
        float x = samples[k][0], y = samples[k][1], z = samples[k][2];
        float d[9];
        d[0]=x*x; d[1]=y*y; d[2]=z*z;
        d[3]=2.0f*x*y; d[4]=2.0f*x*z; d[5]=2.0f*y*z;
        d[6]=2.0f*x; d[7]=2.0f*y; d[8]=2.0f*z;
        for(i = 0; i < 9; i++) {
            b_vec[i] += d[i];
            for(j = i; j < 9; j++) {
                float v = d[i] * d[j];
                S[i][j] += v;
                if(i != j) S[j][i] += v;
            }
        }
    }

    // ---- 步骤2: Cholesky分解 S = L * L^T ----
    float L[9][9];
    if(!cholesky_9x9(S, L)) return 0;

    // ---- 步骤3: 构建约束矩阵 C ----
    // 约束: 4*v[0]*v[2] - v[3]^2 > 0  -->  确保为椭球
    float C[9][9];
    for(i = 0; i < 9; i++) for(j = 0; j < 9; j++) C[i][j] = 0.0f;
    C[0][2] = 2.0f;  C[2][0] = 2.0f;
    C[1][1] = -1.0f;

    // ---- 步骤4: 求解 L * Y = C (列向量求解, 并行计算) ----
    float Y[9][9];
    float tmp[9];
    for(j = 0; j < 9; j++) {
        for(i = 0; i < 9; i++) tmp[i] = C[i][j];
        fwd_sub_9(L, tmp);
        for(i = 0; i < 9; i++) Y[i][j] = tmp[i];
    }

    // ---- 步骤5: 计算 A = L^{-1} * C * L^{-T} = Y * L^{-T} (对称化) ----
    float A[9][9];
    for(j = 0; j < 9; j++) {
        for(i = 0; i < 9; i++) tmp[i] = Y[j][i];  // Y转置的第j行
        bwd_sub_9(L, tmp);
        for(i = 0; i < 9; i++) A[i][j] = tmp[i];
    }

    // 强制对称(消除计算误差)
    for(i = 0; i < 9; i++) for(j = i+1; j < 9; j++) {
        float avg = 0.5f * (A[i][j] + A[j][i]);
        A[i][j] = avg; A[j][i] = avg;
    }

    // ---- 步骤6: Jacobi迭代求解A的特征值与特征向量 ----
    float V[9][9];
    jacobi_9x9(A, V);

    // ---- 步骤7: 回代得到原始问题解 v = L^{-T} * w ----
    float v_raw[9][9];
    for(j = 0; j < 9; j++) {
        for(i = 0; i < 9; i++) tmp[i] = V[i][j];
        bwd_sub_9(L, tmp);
        for(i = 0; i < 9; i++) v_raw[i][j] = tmp[i];
    }

    // ---- 步骤8: 选择满足椭球约束 4ac-b^2>0 的最优解 ----
    int best_idx = -1;
    float best_lambda = -1e30f;
    for(j = 0; j < 9; j++) {
        float con = 4.0f * v_raw[0][j] * v_raw[2][j] - v_raw[3][j] * v_raw[3][j];
        if(con > 0.0f && A[j][j] > best_lambda) {
            best_lambda = A[j][j];
            best_idx = j;
        }
    }

    // 获取最优解向量
    float v[9];
    if(best_idx >= 0) {
        for(i = 0; i < 9; i++) v[i] = v_raw[i][best_idx];
    } else {
        // 备用方案: 直接求解线性方程 S * v = b_vec
        float aug[9][10];
        for(i = 0; i < 9; i++) {
            for(j = 0; j < 9; j++) aug[i][j] = S[i][j];
            aug[i][9] = b_vec[i];
        }
        for(i = 0; i < 9; i++) {
            int mr = i; float mv = fabsf(aug[i][i]);
            for(k = i+1; k < 9; k++) if(fabsf(aug[k][i]) > mv) { mv = fabsf(aug[k][i]); mr = k; }
            if(mv < 1e-15f) return 0;
            if(mr != i) for(j = 0; j < 10; j++) { float t=aug[i][j]; aug[i][j]=aug[mr][j]; aug[mr][j]=t; }
            for(k = i+1; k < 9; k++) { float f = aug[k][i]/aug[i][i]; for(j = i; j < 10; j++) aug[k][j] -= f*aug[i][j]; }
        }
        for(i = 8; i >= 0; i--) {
            v[i] = aug[i][9];
            for(j = i+1; j < 9; j++) v[i] -= aug[i][j]*v[j];
            v[i] /= aug[i][i];
        }
    }

    // ---- 步骤9: 计算硬铁偏移量 ----
    // 3x3二次型矩阵 Q = [[a, d, e], [d, b, f], [e, f, c]]
    float Q[3][3];
    Q[0][0]=v[0]; Q[0][1]=v[3]; Q[0][2]=v[4];
    Q[1][0]=v[3]; Q[1][1]=v[1]; Q[1][2]=v[5];
    Q[2][0]=v[4]; Q[2][1]=v[5]; Q[2][2]=v[2];

    // 一次项向量 u = [g, h, i]
    float u[3];
    u[0]=v[6]; u[1]=v[7]; u[2]=v[8];

    // 硬铁偏移(椭球中心): c = -Q^{-1} * u
    float detQ = Q[0][0]*(Q[1][1]*Q[2][2]-Q[1][2]*Q[2][1])
               - Q[0][1]*(Q[1][0]*Q[2][2]-Q[1][2]*Q[2][0])
               + Q[0][2]*(Q[1][0]*Q[2][1]-Q[1][1]*Q[2][0]);
    if(fabsf(detQ) < 1e-15f) return 0;

    float invQ[3][3];
    invQ[0][0]=(Q[1][1]*Q[2][2]-Q[1][2]*Q[2][1])/detQ;
    invQ[0][1]=(Q[0][2]*Q[2][1]-Q[0][1]*Q[2][2])/detQ;
    invQ[0][2]=(Q[0][1]*Q[1][2]-Q[0][2]*Q[1][1])/detQ;
    invQ[1][0]=(Q[1][2]*Q[2][0]-Q[1][0]*Q[2][2])/detQ;
    invQ[1][1]=(Q[0][0]*Q[2][2]-Q[0][2]*Q[2][0])/detQ;
    invQ[1][2]=(Q[0][2]*Q[1][0]-Q[0][0]*Q[1][2])/detQ;
    invQ[2][0]=(Q[1][0]*Q[2][1]-Q[1][1]*Q[2][0])/detQ;
    invQ[2][1]=(Q[0][1]*Q[2][0]-Q[0][0]*Q[2][1])/detQ;
    invQ[2][2]=(Q[0][0]*Q[1][1]-Q[0][1]*Q[1][0])/detQ;

    bias->data[0] = -(invQ[0][0]*u[0] + invQ[0][1]*u[1] + invQ[0][2]*u[2]);
    bias->data[1] = -(invQ[1][0]*u[0] + invQ[1][1]*u[1] + invQ[1][2]*u[2]);
    bias->data[2] = -(invQ[2][0]*u[0] + invQ[2][1]*u[1] + invQ[2][2]*u[2]);

    // ---- 步骤10: 计算软铁校正矩阵 ----
    float sf = 1.0f + (bias->data[0]*u[0] + bias->data[1]*u[1] + bias->data[2]*u[2]);
    if(sf <= 0.0f) return 0;

    // 归一化矩阵 Qn = Q / sf
    float Qn[3][3];
    for(i = 0; i < 3; i++) for(j = 0; j < 3; j++) Qn[i][j] = Q[i][j] / sf;

    // 雅可比迭代求特征值(3x3)
    float A3[3][3], V3[3][3];
    for(i = 0; i < 3; i++) for(j = 0; j < 3; j++) { A3[i][j]=Qn[i][j]; V3[i][j]=(i==j)?1.0f:0.0f; }

    for(int iter = 0; iter < 50; iter++) {
        float mx = 0.0f; int p=0, q2=1;
        for(i = 0; i < 3; i++) for(j = i+1; j < 3; j++) {
            if(fabsf(A3[i][j]) > mx) { mx = fabsf(A3[i][j]); p = i; q2 = j; }
        }
        if(mx < 1e-12f) break;
        float th;
        if(fabsf(A3[p][p]-A3[q2][q2]) < 1e-12f)
            th = (A3[p][q2]>0.0f)?0.785398f:-0.785398f;
        else
            th = 0.5f * atan2f(2.0f*A3[p][q2], A3[p][p]-A3[q2][q2]);
        float ct=cosf(th), st=sinf(th);
        float App2=ct*ct*A3[p][p]-2*ct*st*A3[p][q2]+st*st*A3[q2][q2];
        float Aqq2=st*st*A3[p][p]+2*ct*st*A3[p][q2]+ct*ct*A3[q2][q2];
        float Ap3[3], Aq3[3];
        for(i=0;i<3;i++){Ap3[i]=A3[p][i];Aq3[i]=A3[q2][i];}
        A3[p][p]=App2;A3[q2][q2]=Aqq2;A3[p][q2]=0.0f;A3[q2][p]=0.0f;
        for(i=0;i<3;i++) if(i!=p&&i!=q2){
            A3[p][i]=ct*Ap3[i]-st*Aq3[i];A3[i][p]=A3[p][i];
            A3[q2][i]=st*Ap3[i]+ct*Aq3[i];A3[i][q2]=A3[q2][i];
        }
        for(i=0;i<3;i++){float vp=V3[i][p],vq=V3[i][q2];V3[i][p]=ct*vp-st*vq;V3[i][q2]=st*vp+ct*vq;}
    }

    float eigs[3];
    eigs[0]=A3[0][0]; eigs[1]=A3[1][1]; eigs[2]=A3[2][2];

    // 校验: 特征值必须为正(保证椭球有效)
    if(eigs[0]<=0.0f || eigs[1]<=0.0f || eigs[2]<=0.0f) return 0;

    // 软铁矩阵: S = V * diag(1/sqrt(lambda)) * V^T
    float sqrt_inv[3];
    sqrt_inv[0]=1.0f/sqrtf(eigs[0]); sqrt_inv[1]=1.0f/sqrtf(eigs[1]); sqrt_inv[2]=1.0f/sqrtf(eigs[2]);

    float Sf[3][3];
    for(i=0;i<3;i++) for(j=0;j<3;j++) {
        Sf[i][j]=V3[i][0]*sqrt_inv[0]*V3[j][0]+V3[i][1]*sqrt_inv[1]*V3[j][1]+V3[i][2]*sqrt_inv[2]*V3[j][2];
    }

    scale->data[0]=Sf[0][0]; scale->data[1]=Sf[0][1]; scale->data[2]=Sf[0][2];
    scale->data[3]=Sf[1][0]; scale->data[4]=Sf[1][1]; scale->data[5]=Sf[1][2];
    scale->data[6]=Sf[2][0]; scale->data[7]=Sf[2][1]; scale->data[8]=Sf[2][2];

    return 1;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      判断浮点数是否有效(排除NaN)
////  @param      value          待判断浮点数
////  @return     uint8          1=有效,0=无效NaN
////-------------------------------------------------------------------------------------------------------------------
static uint8 imu_float_is_valid_local(float value)
{
    return (value == value) ? 1u : 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      保存磁力计校准参数到Flash
////  @param      data_src       IMU参数结构体指针
////  @return     void
////  @note       存储在第63页,避免占用程序Flash空间
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_bias_save(const imu_param *data_src)
{
    uint32 flash_buf[EEPROM_PAGE_LENGTH];
    flash_data_union cvt;

    if(data_src == NULL) return;

    flash_erase_page(0, IMU_MAG_BIAS_FLASH_PAGE);                           // 擦除第63页Flash

    memset(flash_buf, 0xFF, sizeof(flash_buf));
    flash_buf[0] = IMU_MAG_BIAS_MAGIC;                                     // 写入参数标识
    flash_buf[1] = IMU_MAG_BIAS_VERSION;                                   // 写入参数版本

    cvt.float_type = data_src->mag_x_bias;
    flash_buf[2] = cvt.uint32_type;                                        // 存储磁力计X轴偏移
    cvt.float_type = data_src->mag_y_bias;
    flash_buf[3] = cvt.uint32_type;                                        // 存储磁力计Y轴偏移
    cvt.float_type = data_src->mag_z_bias;
    flash_buf[4] = cvt.uint32_type;                                        // 存储磁力计Z轴偏移

    // 存储软铁3x3校正矩阵(9个float, 从flash_buf[5]开始)
    for(int i = 0; i < 9; i++) {
        cvt.float_type = data_src->mag_soft_iron[i];
        flash_buf[5 + i] = cvt.uint32_type;
    }

    flash_write_page(0, IMU_MAG_BIAS_FLASH_PAGE, flash_buf, EEPROM_PAGE_LENGTH);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      从Flash加载磁力计校准参数
////  @param      data_src       IMU参数结构体指针
////  @return     uint8          1=加载成功,0=加载失败
////  @note       读取Flash第63页存储的校准参数
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_bias_load(imu_param *data_src)
{
#if defined(MAG_HARD_IRON_X) && defined(MAG_SOFT_IRON_XX)
    // 已定义宏固定校准参数,不使用Flash存储的校准参数
    return 0u;
#else
    uint32 flash_buf[EEPROM_PAGE_LENGTH];
    flash_data_union cvt;
    float mag_x_bias, mag_y_bias, mag_z_bias;
    float soft_iron[9];

    if(data_src == NULL) return 0u;

    flash_read_page(0, IMU_MAG_BIAS_FLASH_PAGE, flash_buf, EEPROM_PAGE_LENGTH);
    if(flash_buf[0] != IMU_MAG_BIAS_MAGIC) return 0u;                      // 参数标识校验失败

    cvt.uint32_type = flash_buf[2];
    mag_x_bias = cvt.float_type;                                           // 读取X轴偏移
    cvt.uint32_type = flash_buf[3];
    mag_y_bias = cvt.float_type;                                           // 读取Y轴偏移
    cvt.uint32_type = flash_buf[4];
    mag_z_bias = cvt.float_type;                                           // 读取Z轴偏移

    // 根据版本号读取软铁校正参数
    if(flash_buf[1] >= 3u) {
        // v3版本: 完整3x3软铁矩阵
        for(int i = 0; i < 9; i++) { cvt.uint32_type = flash_buf[5+i]; soft_iron[i] = cvt.float_type; }
    } else if(flash_buf[1] == 2u) {
        // v2版本: 仅对角线参数,补全单位矩阵
        cvt.uint32_type = flash_buf[5]; soft_iron[0] = cvt.float_type;
        cvt.uint32_type = flash_buf[6]; soft_iron[4] = cvt.float_type;
        cvt.uint32_type = flash_buf[7]; soft_iron[8] = cvt.float_type;
        soft_iron[1]=0.0f;soft_iron[2]=0.0f;soft_iron[3]=0.0f;
        soft_iron[5]=0.0f;soft_iron[6]=0.0f;soft_iron[7]=0.0f;
    } else {
        // v1及以下版本: 默认单位矩阵
        soft_iron[0]=1.0f;soft_iron[1]=0.0f;soft_iron[2]=0.0f;
        soft_iron[3]=0.0f;soft_iron[4]=1.0f;soft_iron[5]=0.0f;
        soft_iron[6]=0.0f;soft_iron[7]=0.0f;soft_iron[8]=1.0f;
    }

    // 校验数据有效性
    if(!imu_float_is_valid_local(mag_x_bias)||!imu_float_is_valid_local(mag_y_bias)||!imu_float_is_valid_local(mag_z_bias))
        return 0u;
    for(int i = 0; i < 9; i++) { if(!imu_float_is_valid_local(soft_iron[i])) return 0u; }

    data_src->mag_x_bias = mag_x_bias;
    data_src->mag_y_bias = mag_y_bias;
    data_src->mag_z_bias = mag_z_bias;
    for(int i = 0; i < 9; i++) data_src->mag_soft_iron[i] = soft_iron[i];

    return 1u;
#endif
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      磁力计简易校准数据更新(极值法)
////  @param      data_src       IMU参数结构体指针
////  @return     void
////  @note       实时更新磁力计三轴最大最小值
////-------------------------------------------------------------------------------------------------------------------
static void imu_mag_calib_update(imu_param *data_src)
{
    if((data_src == NULL) || !s_mag_calib_active) return;

    float mag_x = (float)imu963ra_mag_x;
    float mag_y = (float)imu963ra_mag_y;
    float mag_z = (float)imu963ra_mag_z;

    if(s_mag_calib_samples == 0u) {
        s_mag_min_x = s_mag_max_x = mag_x;
        s_mag_min_y = s_mag_max_y = mag_y;
        s_mag_min_z = s_mag_max_z = mag_z;
    } else {
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
////  @brief      IMU数据处理函数
////  @param      data_src       IMU参数结构体指针
////  @return     void
////  @note       读取原始数据,完成校准、滤波、单位转换
////-------------------------------------------------------------------------------------------------------------------
void date_handle(imu_param *data_src)
{
    imu963ra_get_mag();                                                    // 获取磁力计原始数据
    imu963ra_get_gyro();                                                   // 获取陀螺仪原始数据
    imu963ra_get_acc();                                                    // 获取加速度计原始数据

    imu_mag_calib_update(data_src);                                        // 简易校准数据更新

    // 椭球拟合校准采样
    if(s_mag_ellipsoid_active && s_mag_sample_count < MAG_CALIB_MAX_SAMPLES) {
        s_mag_sample_timer++;
        // 定时采样, 每15ms采集一次数据
        if(s_mag_sample_timer >= 15u) {
            s_mag_samples[s_mag_sample_count][0] = (float)imu963ra_mag_x;
            s_mag_samples[s_mag_sample_count][1] = (float)imu963ra_mag_y;
            s_mag_samples[s_mag_sample_count][2] = (float)imu963ra_mag_z;
            s_mag_sample_count++;
            s_mag_sample_timer = 0u;
        }
    }

    // 存储原始数据
    imu660.data_Raw.acc_x = (float)imu963ra_acc_x;
    imu660.data_Raw.acc_y = (float)imu963ra_acc_y;
    imu660.data_Raw.acc_z = (float)imu963ra_acc_z;
    imu660.data_Raw.gyro_x = (float)imu963ra_gyro_x;
    imu660.data_Raw.gyro_y = (float)imu963ra_gyro_y;
    imu660.data_Raw.gyro_z = (float)imu963ra_gyro_z;
    imu660.data_Raw.mag_x = (float)imu963ra_mag_x;
    imu660.data_Raw.mag_y = (float)imu963ra_mag_y;
    imu660.data_Raw.mag_z = (float)imu963ra_mag_z;

    // 去除零偏
    float acc_x_offset = imu963ra_acc_x - data_src->acc_x_bias;
    float acc_y_offset = imu963ra_acc_y - data_src->acc_y_bias;
    float acc_z_offset = imu963ra_acc_z + data_src->acc_z_bias;
    float gyro_x_offset = imu963ra_gyro_x - data_src->gyro_x_bias;
    float gyro_y_offset = imu963ra_gyro_y - data_src->gyro_y_bias;
    float gyro_z_offset = imu963ra_gyro_z - data_src->gyro_z_bias;

    // 互补滤波+单位转换
    imu660.data_Ripen.acc_x = (imu963ra_acc_transition(acc_x_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_x * (1.0f - alpha);
    imu660.data_Ripen.acc_y = (imu963ra_acc_transition(acc_y_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_y * (1.0f - alpha);
    imu660.data_Ripen.acc_z = (imu963ra_acc_transition(acc_z_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_z * (1.0f - alpha);
    imu660.data_Ripen.gyro_x = (imu963ra_gyro_transition_local(gyro_x_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_x * (1.0f - alpha);
    imu660.data_Ripen.gyro_y = (imu963ra_gyro_transition_local(gyro_y_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_y * (1.0f - alpha);
    imu660.data_Ripen.gyro_z = (imu963ra_gyro_transition_local(gyro_z_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_z * (1.0f - alpha);

    // 磁力计校准: 硬铁补偿 + 软铁3x3矩阵校正
    float mx_raw = (float)imu963ra_mag_x - data_src->mag_x_bias;
    float my_raw = (float)imu963ra_mag_y - data_src->mag_y_bias;
    float mz_raw = (float)imu963ra_mag_z - data_src->mag_z_bias;

    float mx = mx_raw * data_src->mag_soft_iron[0] + my_raw * data_src->mag_soft_iron[1] + mz_raw * data_src->mag_soft_iron[2];
    float my = mx_raw * data_src->mag_soft_iron[3] + my_raw * data_src->mag_soft_iron[4] + mz_raw * data_src->mag_soft_iron[5];
    float mz = mx_raw * data_src->mag_soft_iron[6] + my_raw * data_src->mag_soft_iron[7] + mz_raw * data_src->mag_soft_iron[8];

    imu660.data_Ripen.mag_x = mx;
    imu660.data_Ripen.mag_y = my;
    imu660.data_Ripen.mag_z = mz;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      陀螺仪Z轴自动校准
////  @param      data_src       IMU参数结构体指针
////  @param      samples        采样次数
////  @return     void
////  @note       静止状态下计算Z轴零偏
////-------------------------------------------------------------------------------------------------------------------
void imu_gyro_z_autocalib(imu_param *data_src, unsigned int samples)
{
    if (!data_src || samples == 0) return;

    float sum_z = 0.0f;
    for (unsigned int i = 0; i < samples; i++) {
        imu963ra_get_gyro();
        sum_z += (float)imu963ra_gyro_z;
        system_delay_ms(2);
    }
    data_src->gyro_z_bias = sum_z / (float)samples;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      IMU零偏初始化
////  @param      data_src       IMU参数结构体指针
////  @return     void
////  @note       开发阶段优先加载PC端生成的 calibration_params.h 参数
////-------------------------------------------------------------------------------------------------------------------
void imu_bias_init(imu_param *data_src)
{
    data_src->gyro_x_bias = 0.0f;
    data_src->gyro_y_bias = 0.0f;
    data_src->gyro_z_bias = 0.0f;
    data_src->acc_x_bias  = 0.0f;
    data_src->acc_y_bias  = 0.0f;
    data_src->acc_z_bias  = 0.0f;

#if defined(MAG_HARD_IRON_X) && defined(MAG_SOFT_IRON_XX)
    data_src->mag_x_bias  = MAG_HARD_IRON_X;
    data_src->mag_y_bias  = MAG_HARD_IRON_Y;
    data_src->mag_z_bias  = MAG_HARD_IRON_Z;

    data_src->mag_soft_iron[0] = MAG_SOFT_IRON_XX;
    data_src->mag_soft_iron[1] = MAG_SOFT_IRON_XY;
    data_src->mag_soft_iron[2] = MAG_SOFT_IRON_XZ;
    data_src->mag_soft_iron[3] = MAG_SOFT_IRON_YX;
    data_src->mag_soft_iron[4] = MAG_SOFT_IRON_YY;
    data_src->mag_soft_iron[5] = MAG_SOFT_IRON_YZ;
    data_src->mag_soft_iron[6] = MAG_SOFT_IRON_ZX;
    data_src->mag_soft_iron[7] = MAG_SOFT_IRON_ZY;
    data_src->mag_soft_iron[8] = MAG_SOFT_IRON_ZZ;
#else
    data_src->mag_x_bias  = 0.0f;
    data_src->mag_y_bias  = 0.0f;
    data_src->mag_z_bias  = 0.0f;

    // 未生成PC端校准参数时，退化为单位矩阵
    data_src->mag_soft_iron[0]=1.0f;data_src->mag_soft_iron[1]=0.0f;data_src->mag_soft_iron[2]=0.0f;
    data_src->mag_soft_iron[3]=0.0f;data_src->mag_soft_iron[4]=1.0f;data_src->mag_soft_iron[5]=0.0f;
    data_src->mag_soft_iron[6]=0.0f;data_src->mag_soft_iron[7]=0.0f;data_src->mag_soft_iron[8]=1.0f;
#endif
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      启动磁力计校准(椭球拟合模式)
////  @param      void
////  @return     void
////  @note       清空参数,开启采样标志
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_calib_start(void)
{
    // 重置校准参数
    imu_date.mag_x_bias = 0.0f;
    imu_date.mag_y_bias = 0.0f;
    imu_date.mag_z_bias = 0.0f;
    imu_date.mag_soft_iron[0]=1.0f;imu_date.mag_soft_iron[1]=0.0f;imu_date.mag_soft_iron[2]=0.0f;
    imu_date.mag_soft_iron[3]=0.0f;imu_date.mag_soft_iron[4]=1.0f;imu_date.mag_soft_iron[5]=0.0f;
    imu_date.mag_soft_iron[6]=0.0f;imu_date.mag_soft_iron[7]=0.0f;imu_date.mag_soft_iron[8]=1.0f;

    s_mag_ellipsoid_active = 1u;
    s_mag_sample_count = 0u;
    s_mag_sample_timer = 0u;
    s_mag_calib_active = 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      结束磁力计校准(椭球拟合)
////  @param      data_src       IMU参数结构体指针
////  @return     uint8          1=校准成功,0=校准失败
////  @note       计算校准参数并保存到Flash
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_finish(imu_param *data_src)
{
    if((data_src == NULL) || !s_mag_ellipsoid_active) return 0u;

    s_mag_ellipsoid_active = 0u;
    if(s_mag_sample_count < MAG_CALIB_MIN_SAMPLES) return 0u;

    Vec3 bias; Mat3x3 scale;
    vec3_zero(&bias); mat3x3_zero(&scale);

    if(!solve_ellipsoid(s_mag_samples, s_mag_sample_count, &bias, &scale)) return 0u;

    data_src->mag_x_bias = bias.data[0];
    data_src->mag_y_bias = bias.data[1];
    data_src->mag_z_bias = bias.data[2];
    for(int i = 0; i < 9; i++) data_src->mag_soft_iron[i] = scale.data[i];

    imu_mag_bias_save(data_src);
    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      查询磁力计校准状态
////  @param      void
////  @return     uint8          1=校准中,0=未校准
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_is_active(void)
{
    return s_mag_ellipsoid_active;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      启动磁力计椭球拟合校准
////  @param      void
////  @return     void
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_calib_ellipsoid_start(void)
{
    // 重置校准参数
    imu_date.mag_x_bias = 0.0f;
    imu_date.mag_y_bias = 0.0f;
    imu_date.mag_z_bias = 0.0f;
    imu_date.mag_soft_iron[0]=1.0f;imu_date.mag_soft_iron[1]=0.0f;imu_date.mag_soft_iron[2]=0.0f;
    imu_date.mag_soft_iron[3]=0.0f;imu_date.mag_soft_iron[4]=1.0f;imu_date.mag_soft_iron[5]=0.0f;
    imu_date.mag_soft_iron[6]=0.0f;imu_date.mag_soft_iron[7]=0.0f;imu_date.mag_soft_iron[8]=1.0f;

    s_mag_ellipsoid_active = 1u;
    s_mag_sample_count = 0u;
    s_mag_sample_timer = 0u;
    s_mag_calib_active = 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      手动添加磁力计校准采样数据
////  @param      mx, my, mz     磁力计三轴原始数据
////  @return     uint8          1=添加成功,0=校准未开启/缓存满
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_ellipsoid_update(float mx, float my, float mz)
{
    if(!s_mag_ellipsoid_active || s_mag_sample_count >= MAG_CALIB_MAX_SAMPLES) return 0u;

    s_mag_samples[s_mag_sample_count][0] = mx;
    s_mag_samples[s_mag_sample_count][1] = my;
    s_mag_samples[s_mag_sample_count][2] = mz;
    s_mag_sample_count++;
    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      手动完成磁力计椭球校准
////  @param      data_src       IMU参数结构体指针
////  @return     uint8          1=校准成功,0=校准失败
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_ellipsoid_finish(imu_param *data_src)
{
    if((data_src == NULL) || !s_mag_ellipsoid_active) return 0u;

    s_mag_ellipsoid_active = 0u;
    if(s_mag_sample_count < MAG_CALIB_MIN_SAMPLES) return 0u;

    Vec3 bias; Mat3x3 scale;
    vec3_zero(&bias); mat3x3_zero(&scale);

    if(!solve_ellipsoid(s_mag_samples, s_mag_sample_count, &bias, &scale)) return 0u;

    data_src->mag_x_bias = bias.data[0];
    data_src->mag_y_bias = bias.data[1];
    data_src->mag_z_bias = bias.data[2];
    for(int i = 0; i < 9; i++) data_src->mag_soft_iron[i] = scale.data[i];

    imu_mag_bias_save(data_src);
    return 1u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      获取磁力计校准采样数量
////  @param      void
////  @return     uint32         当前采样点数
////-------------------------------------------------------------------------------------------------------------------
uint32 imu_mag_calib_get_sample_count(void)
{
    return s_mag_sample_count;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      发送磁力计原始数据到PC(用于上位机校准)
////  @param      void
////  @return     void
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_send_raw_data_to_pc(void)
{
    char send_buffer[64];
    sprintf(send_buffer, "%.2f,%.2f,%.2f\r\n", imu660.data_Raw.mag_x, imu660.data_Raw.mag_y, imu660.data_Raw.mag_z);
    wireless_uart_send_string(send_buffer);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      加速度计静态校准测试
////  @param      data_src       IMU参数结构体指针
////  @return     void
////  @note       静止状态下采集数据计算加速度计零偏
////-------------------------------------------------------------------------------------------------------------------
void static_imu_test(imu_param *data_src)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    unsigned int count = 0;
    const unsigned int max_samples = 20000;

    printf("Starting Calibration... Please keep the car still!\n");
    system_delay_ms(500);

    for(count = 0; count < max_samples; count++) {
        date_handle(data_src);
        sum_x += imu660.data_Raw.acc_x;
        sum_y += imu660.data_Raw.acc_y;
        sum_z += imu660.data_Raw.acc_z;
        system_delay_ms(5);
    }

    data_src->acc_x_bias = sum_x / (float)count;
    data_src->acc_y_bias = sum_y / (float)count;
    data_src->acc_z_bias = sum_z / (float)count;

    printf("Accel biases calculated:\nX: %f\nY: %f\nZ: %f\n",
           data_src->acc_x_bias, data_src->acc_y_bias, data_src->acc_z_bias);
}
