/*
 * imu660.c
 * IMU?????????????????
 *
 * Created on: 2024??6??6??
 * Author: LateRain
 * Modified: 2025??11??22??
 */

//-------------------------------------------????????------------------------------------------------------------
#include "zf_common_headfile.h"
#include "calibration_params.h"

//-------------------------------------------??????????-----------------------------------------------------------
imu_param imu_date = {0};                                                  // IMU???????
imu660_struct imu660 = {0};                                                // IMU660???????

//-------------------------------------------????------------------------------------------------------------
#define alpha 0.8f                                                         // ??????????
#define imu963ra_gyro_transition_local(gyro_value) ((float)(gyro_value) / imu963ra_transition_factor[1])   // ?????????????
#define IMU_MAG_BIAS_FLASH_PAGE      63u                                   // ??????§µ??????›¥Flash?
#define IMU_MAG_BIAS_MAGIC           0x4D414742u                           // §µ????????"MAGB"
#define IMU_MAG_BIAS_VERSION         3u                                    // §µ??????·Ú3(???3x3????????)

//-------------------------------------------???????-????§µ?------------------------------------------------------------
static uint8 s_mag_calib_active = 0u;                                      // ?????????§µ??????
static uint32 s_mag_calib_samples = 0u;                                    // ?????????§µ???????
static float s_mag_min_x = 0.0f;                                           // ??????X????§³?
static float s_mag_min_y = 0.0f;                                           // ??????Y????§³?
static float s_mag_min_z = 0.0f;                                           // ??????Z????§³?
static float s_mag_max_x = 0.0f;                                           // ??????X??????
static float s_mag_max_y = 0.0f;                                           // ??????Y??????
static float s_mag_max_z = 0.0f;                                           // ??????Z??????

//-------------------------------------------???????-???????§µ?------------------------------------------------------------
static uint8 s_mag_ellipsoid_active = 0u;                                 // ?????????????§µ????
static float s_mag_samples[MAG_CALIB_MAX_SAMPLES][3];                     // ??????§µ????????????
static uint32 s_mag_sample_count = 0u;                                     // ??????§µ?????????
static uint32 s_mag_sample_timer = 0u;                                     // ??????????????????

//-------------------------------------------????????------------------------------------------------------------
static void imu_mag_calib_update(imu_param *data_src);                     // ?????????§µ?????

//-------------------------------------------??????????????------------------------------------------------------------
typedef struct {
    float data[9];
} Mat3x3;

typedef struct {
    float data[3];
} Vec3;

// 3x3????????
static void mat3x3_zero(Mat3x3* m) {
    for(int i = 0; i < 9; i++) m->data[i] = 0.0f;
}

// 3x3??????
static void mat3x3_add(Mat3x3* a, Mat3x3* b, Mat3x3* out) {
    for(int i = 0; i < 9; i++) out->data[i] = a->data[i] + b->data[i];
}

// 3x3????????
static void mat3x3_scale(Mat3x3* m, float s, Mat3x3* out) {
    for(int i = 0; i < 9; i++) out->data[i] = m->data[i] * s;
}

// 3x3?????????????
static float mat3x3_det(Mat3x3* m) {
    return m->data[0] * (m->data[4] * m->data[8] - m->data[5] * m->data[7]) -
           m->data[1] * (m->data[3] * m->data[8] - m->data[5] * m->data[6]) +
           m->data[2] * (m->data[3] * m->data[7] - m->data[4] * m->data[6]);
}

// 3x3????????
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

// 3?????????
static void vec3_zero(Vec3* v) {
    v->data[0] = v->data[1] = v->data[2] = 0.0f;
}

// 3???????3x3????
static void vec3_mul_mat3x3(Vec3* v, Mat3x3* m, Vec3* out) {
    out->data[0] = v->data[0] * m->data[0] + v->data[1] * m->data[3] + v->data[2] * m->data[6];
    out->data[1] = v->data[0] * m->data[1] + v->data[1] * m->data[4] + v->data[2] * m->data[7];
    out->data[2] = v->data[0] * m->data[2] + v->data[1] * m->data[5] + v->data[2] * m->data[8];
}

// 3?????????
static void vec3_sub(Vec3* a, Vec3* b, Vec3* out) {
    out->data[0] = a->data[0] - b->data[0];
    out->data[1] = a->data[1] - b->data[1];
    out->data[2] = a->data[2] - b->data[2];
}

// 3????????
static float vec3_dot(Vec3* a, Vec3* b) {
    return a->data[0] * b->data[0] + a->data[1] * b->data[1] + a->data[2] * b->data[2];
}

//-------------------------------------------????????????------------------------------------------------------------
// Cholesky???: A = L * L^T (9x9?????????????), L??????????
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

// ????I: ??? L * x = b (L??????????, ?????x)
static void fwd_sub_9(const float L[9][9], float x[9]) {
    int i, j;
    for(i = 0; i < 9; i++) {
        float sum = x[i];
        for(j = 0; j < i; j++) sum -= L[i][j] * x[j];
        x[i] = sum / L[i][i];
    }
}

// ?????I: ??? L^T * x = b (L^T??????????, ?????x)
static void bwd_sub_9(const float L[9][9], float x[9]) {
    int i, j;
    for(i = 8; i >= 0; i--) {
        float sum = x[i];
        for(j = i + 1; j < 9; j++) sum -= L[j][i] * x[j];
        x[i] = sum / L[i][i];
    }
}

// ???????: ???9x9??????????????????????
// ????:A(??????), ???:A(????????), V(??=????????)
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
        // ?§Ø?????
        float max_off = 0.0f;
        for(i = 0; i < 8; i++) for(j = i + 1; j < 9; j++) {
            float v = fabsf(A[i][j]); if(v > max_off) max_off = v;
        }
        if(max_off < 1e-14f) break;
    }
}

// ?????????????§µ?: ????????? + ????§µ??????
// ???????: 4*a*c - b^2 > 0 (?????????????/????)
// ??????: S*v = lambda*C*v -> Cholesky??? -> Jacobi????
static uint8 solve_ellipsoid(float samples[][3], uint32_t n, Vec3* bias, Mat3x3* scale) {
    if(n < MAG_CALIB_MIN_SAMPLES) return 0;

    int i, j, k;

    // ---- ????1: ????Gram???? S = D^T * D ?????? b = D^T * 1 ----
    // ????????: [x^2, y^2, z^2, 2xy, 2xz, 2yz, 2x, 2y, 2z]
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

    // ---- ????2: Cholesky??? S = L * L^T ----
    float L[9][9];
    if(!cholesky_9x9(S, L)) return 0;

    // ---- ????3: ??????????? C ----
    // ???: 4*v[0]*v[2] - v[3]^2 > 0  -->  ????????
    float C[9][9];
    for(i = 0; i < 9; i++) for(j = 0; j < 9; j++) C[i][j] = 0.0f;
    C[0][2] = 2.0f;  C[2][0] = 2.0f;
    C[1][1] = -1.0f;

    // ---- ????4: ??? L * Y = C (?????????, ???§Þ???) ----
    float Y[9][9];
    float tmp[9];
    for(j = 0; j < 9; j++) {
        for(i = 0; i < 9; i++) tmp[i] = C[i][j];
        fwd_sub_9(L, tmp);
        for(i = 0; i < 9; i++) Y[i][j] = tmp[i];
    }

    // ---- ????5: ???? A = L^{-1} * C * L^{-T} = Y * L^{-T} (????) ----
    float A[9][9];
    for(j = 0; j < 9; j++) {
        for(i = 0; i < 9; i++) tmp[i] = Y[j][i];  // Y?????j??
        bwd_sub_9(L, tmp);
        for(i = 0; i < 9; i++) A[i][j] = tmp[i];
    }

    // ?????(???????????)
    for(i = 0; i < 9; i++) for(j = i+1; j < 9; j++) {
        float avg = 0.5f * (A[i][j] + A[j][i]);
        A[i][j] = avg; A[j][i] = avg;
    }

    // ---- ????6: Jacobi???????A????????????????? ----
    float V[9][9];
    jacobi_9x9(A, V);

    // ---- ????7: ????????????? v = L^{-T} * w ----
    float v_raw[9][9];
    for(j = 0; j < 9; j++) {
        for(i = 0; i < 9; i++) tmp[i] = V[i][j];
        bwd_sub_9(L, tmp);
        for(i = 0; i < 9; i++) v_raw[i][j] = tmp[i];
    }

    // ---- ????8: ?????????????? 4ac-b^2>0 ??????? ----
    int best_idx = -1;
    float best_lambda = -1e30f;
    for(j = 0; j < 9; j++) {
        float con = 4.0f * v_raw[0][j] * v_raw[2][j] - v_raw[3][j] * v_raw[3][j];
        if(con > 0.0f && A[j][j] > best_lambda) {
            best_lambda = A[j][j];
            best_idx = j;
        }
    }

    // ????????????
    float v[9];
    if(best_idx >= 0) {
        for(i = 0; i < 9; i++) v[i] = v_raw[i][best_idx];
    } else {
        // ???¡Â???: ????????????? S * v = b_vec
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

    // ---- ????9: ???????????? ----
    // 3x3????????? Q = [[a, d, e], [d, b, f], [e, f, c]]
    float Q[3][3];
    Q[0][0]=v[0]; Q[0][1]=v[3]; Q[0][2]=v[4];
    Q[1][0]=v[3]; Q[1][1]=v[1]; Q[1][2]=v[5];
    Q[2][0]=v[4]; Q[2][1]=v[5]; Q[2][2]=v[2];

    // ????????? u = [g, h, i]
    float u[3];
    u[0]=v[6]; u[1]=v[7]; u[2]=v[8];

    // ??????(????????): c = -Q^{-1} * u
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

    // ---- ????10: ????????§µ?????? ----
    float sf = 1.0f + (bias->data[0]*u[0] + bias->data[1]*u[1] + bias->data[2]*u[2]);
    if(sf <= 0.0f) return 0;

    // ????????? Qn = Q / sf
    float Qn[3][3];
    for(i = 0; i < 3; i++) for(j = 0; j < 3; j++) Qn[i][j] = Q[i][j] / sf;

    // ??????????????(3x3)
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

    // §µ??: ????????????(?????????§¹)
    if(eigs[0]<=0.0f || eigs[1]<=0.0f || eigs[2]<=0.0f) return 0;

    // ????????: S = V * diag(1/sqrt(lambda)) * V^T
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
////  @brief      ?§Ø???????????§¹(???NaN)
////  @param      value          ???§Ø??????
////  @return     uint8          1=??§¹,0=??§¹NaN
////-------------------------------------------------------------------------------------------------------------------
static uint8 imu_float_is_valid_local(float value)
{
    return (value == value) ? 1u : 0u;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      ?????????§µ???????Flash
////  @param      data_src       IMU???????????
////  @return     void
////  @note       ?›¥???63?,??????¨®???Flash???
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_bias_save(const imu_param *data_src)
{
    uint32 flash_buf[EEPROM_PAGE_LENGTH];
    flash_data_union cvt;

    if(data_src == NULL) return;

    flash_erase_page(0, IMU_MAG_BIAS_FLASH_PAGE);                           // ??????63?Flash

    memset(flash_buf, 0xFF, sizeof(flash_buf));
    flash_buf[0] = IMU_MAG_BIAS_MAGIC;                                     // §Õ????????
    flash_buf[1] = IMU_MAG_BIAS_VERSION;                                   // §Õ??????·Ú

    cvt.float_type = data_src->mag_x_bias;
    flash_buf[2] = cvt.uint32_type;                                        // ?›¥??????X?????
    cvt.float_type = data_src->mag_y_bias;
    flash_buf[3] = cvt.uint32_type;                                        // ?›¥??????Y?????
    cvt.float_type = data_src->mag_z_bias;
    flash_buf[4] = cvt.uint32_type;                                        // ?›¥??????Z?????

    // ?›¥????3x3§µ??????(9??float, ??flash_buf[5]???)
    for(int i = 0; i < 9; i++) {
        cvt.float_type = data_src->mag_soft_iron[i];
        flash_buf[5 + i] = cvt.uint32_type;
    }

    flash_write_page(0, IMU_MAG_BIAS_FLASH_PAGE, flash_buf, EEPROM_PAGE_LENGTH);
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      ??Flash?????????§µ?????
////  @param      data_src       IMU???????????
////  @return     uint8          1=??????,0=???????
////  @note       ???Flash??63??›¥??§µ?????
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_bias_load(imu_param *data_src)
{
#if defined(MAG_HARD_IRON_X) && defined(MAG_SOFT_IRON_XX)
    // ????????§µ?????,?????Flash?›¥??§µ?????
    return 0u;
#else
    uint32 flash_buf[EEPROM_PAGE_LENGTH];
    flash_data_union cvt;
    float mag_x_bias, mag_y_bias, mag_z_bias;
    float soft_iron[9];

    if(data_src == NULL) return 0u;

    flash_read_page(0, IMU_MAG_BIAS_FLASH_PAGE, flash_buf, EEPROM_PAGE_LENGTH);
    if(flash_buf[0] != IMU_MAG_BIAS_MAGIC) return 0u;                      // ???????§µ?????

    cvt.uint32_type = flash_buf[2];
    mag_x_bias = cvt.float_type;                                           // ???X?????
    cvt.uint32_type = flash_buf[3];
    mag_y_bias = cvt.float_type;                                           // ???Y?????
    cvt.uint32_type = flash_buf[4];
    mag_z_bias = cvt.float_type;                                           // ???Z?????

    // ????·Ú????????§µ??????
    if(flash_buf[1] >= 3u) {
        // v3?·Ú: ????3x3????????
        for(int i = 0; i < 9; i++) { cvt.uint32_type = flash_buf[5+i]; soft_iron[i] = cvt.float_type; }
    } else if(flash_buf[1] == 2u) {
        // v2?·Ú: ??????????,?????¦Ë????
        cvt.uint32_type = flash_buf[5]; soft_iron[0] = cvt.float_type;
        cvt.uint32_type = flash_buf[6]; soft_iron[4] = cvt.float_type;
        cvt.uint32_type = flash_buf[7]; soft_iron[8] = cvt.float_type;
        soft_iron[1]=0.0f;soft_iron[2]=0.0f;soft_iron[3]=0.0f;
        soft_iron[5]=0.0f;soft_iron[6]=0.0f;soft_iron[7]=0.0f;
    } else {
        // v1?????¡ã·Ú: ????¦Ë????
        soft_iron[0]=1.0f;soft_iron[1]=0.0f;soft_iron[2]=0.0f;
        soft_iron[3]=0.0f;soft_iron[4]=1.0f;soft_iron[5]=0.0f;
        soft_iron[6]=0.0f;soft_iron[7]=0.0f;soft_iron[8]=1.0f;
    }

    // §µ????????§¹??
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
////  @brief      ?????????§µ????????(?????)
////  @param      data_src       IMU???????????
////  @return     void
////  @note       ????????????????????§³?
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
////  @brief      IMU???????????
////  @param      data_src       IMU???????????
////  @return     void
////  @note       ?????????,???§µ??????????¦Ë???
////-------------------------------------------------------------------------------------------------------------------
void date_handle(imu_param *data_src)
{
    imu963ra_get_mag();                                                    // ???????????????
    imu963ra_get_gyro();                                                   // ???????????????
    imu963ra_get_acc();                                                    // ???????????????

    imu_mag_calib_update(data_src);                                        // ????§µ????????

    // ???????§µ?????
    if(s_mag_ellipsoid_active && s_mag_sample_count < MAG_CALIB_MAX_SAMPLES) {
        s_mag_sample_timer++;
        // ???????, ?15ms??????????
        if(s_mag_sample_timer >= 15u) {
            s_mag_samples[s_mag_sample_count][0] = (float)imu963ra_mag_x;
            s_mag_samples[s_mag_sample_count][1] = (float)imu963ra_mag_y;
            s_mag_samples[s_mag_sample_count][2] = (float)imu963ra_mag_z;
            s_mag_sample_count++;
            s_mag_sample_timer = 0u;
        }
    }

    // ?›¥??????
    imu660.data_Raw.acc_x = (float)imu963ra_acc_x;
    imu660.data_Raw.acc_y = (float)imu963ra_acc_y;
    imu660.data_Raw.acc_z = (float)imu963ra_acc_z;
    imu660.data_Raw.gyro_x = (float)imu963ra_gyro_x;
    imu660.data_Raw.gyro_y = (float)imu963ra_gyro_y;
    imu660.data_Raw.gyro_z = (float)imu963ra_gyro_z;
    imu660.data_Raw.mag_x = (float)imu963ra_mag_x;
    imu660.data_Raw.mag_y = (float)imu963ra_mag_y;
    imu660.data_Raw.mag_z = (float)imu963ra_mag_z;

    // ??????
    float acc_x_offset = imu963ra_acc_x - data_src->acc_x_bias;
    float acc_y_offset = imu963ra_acc_y - data_src->acc_y_bias;
    float acc_z_offset = imu963ra_acc_z + data_src->acc_z_bias;
    float gyro_x_offset = imu963ra_gyro_x - data_src->gyro_x_bias;
    float gyro_y_offset = imu963ra_gyro_y - data_src->gyro_y_bias;
    float gyro_z_offset = imu963ra_gyro_z - data_src->gyro_z_bias;

    // ???????+??¦Ë???
    imu660.data_Ripen.acc_x = (imu963ra_acc_transition(acc_x_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_x * (1.0f - alpha);
    imu660.data_Ripen.acc_y = (imu963ra_acc_transition(acc_y_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_y * (1.0f - alpha);
    imu660.data_Ripen.acc_z = (imu963ra_acc_transition(acc_z_offset) * 9.79f) * alpha + imu660.data_Ripen.acc_z * (1.0f - alpha);
    imu660.data_Ripen.gyro_x = (imu963ra_gyro_transition_local(gyro_x_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_x * (1.0f - alpha);
    imu660.data_Ripen.gyro_y = (imu963ra_gyro_transition_local(gyro_y_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_y * (1.0f - alpha);
    imu660.data_Ripen.gyro_z = (imu963ra_gyro_transition_local(gyro_z_offset) * M_PI / 180.0f) * alpha + imu660.data_Ripen.gyro_z * (1.0f - alpha);

    // ??????§µ?: ??????? + ????3x3????§µ??
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
////  @brief      ??????Z?????§µ?
////  @param      data_src       IMU???????????
////  @param      samples        ????????
////  @return     void
////  @note       ??????????Z?????
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
////  @brief      IMU????????
////  @param      data_src       IMU???????????
////  @return     void
////  @note       ??????????????PC??????? calibration_params.h ????
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

    // ¦Ä????PC??§µ??????????????¦Ë????
    data_src->mag_soft_iron[0]=1.0f;data_src->mag_soft_iron[1]=0.0f;data_src->mag_soft_iron[2]=0.0f;
    data_src->mag_soft_iron[3]=0.0f;data_src->mag_soft_iron[4]=1.0f;data_src->mag_soft_iron[5]=0.0f;
    data_src->mag_soft_iron[6]=0.0f;data_src->mag_soft_iron[7]=0.0f;data_src->mag_soft_iron[8]=1.0f;
#endif
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      ??????????§µ?(?????????)
////  @param      void
////  @return     void
////  @note       ??????,???????????
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_calib_start(void)
{
    // ????§µ?????
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
////  @brief      ??????????§µ?(???????)
////  @param      data_src       IMU???????????
////  @return     uint8          1=§µ????,0=§µ????
////  @note       ????§µ??????????›ÔFlash
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
////  @brief      ?????????§µ???
////  @param      void
////  @return     uint8          1=§µ???,0=¦Ä§µ?
////-------------------------------------------------------------------------------------------------------------------
uint8 imu_mag_calib_is_active(void)
{
    return s_mag_ellipsoid_active;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      ?????????????????§µ?
////  @param      void
////  @return     void
////-------------------------------------------------------------------------------------------------------------------
void imu_mag_calib_ellipsoid_start(void)
{
    // ????§µ?????
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
////  @brief      ????????????§µ?????????
////  @param      mx, my, mz     ????????????????
////  @return     uint8          1=??????,0=§µ?¦Ä????/??????
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
////  @brief      ???????????????§µ?
////  @param      data_src       IMU???????????
////  @return     uint8          1=§µ????,0=§µ????
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
////  @brief      ?????????§µ?????????
////  @param      void
////  @return     uint32         ???????????
////-------------------------------------------------------------------------------------------------------------------
uint32 imu_mag_calib_get_sample_count(void)
{
    return s_mag_sample_count;
}

////-------------------------------------------------------------------------------------------------------------------
////  @brief      ????????????????PC(??????¦Ë??§µ?)
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
////  @brief      ????????§µ?????
////  @param      data_src       IMU???????????
////  @return     void
////  @note       ????????????????????????
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
