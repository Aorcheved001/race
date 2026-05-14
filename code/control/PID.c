/*
 * PID.c
 *
 * ??? PID ???
 * ?????? + ??? + ????????????
 *
 * ????
 * +------------------------------------------------------------------------------+
 * |                          PID ???????                                  |
 * +------------------------------------------------------------------------------+
 * |                                                                              |
 * |  +----------------+   +----------------+   +----------------+   +--------+  |
 * |  |   ???         |   |   ???       |   |   ??????  |   | ??PWM|  |
 * |  | (??)          |   | (??)         |   | (??)        |   |        |  |
 * |  |                |   |               |   |              |   |        |  |
 * |  | ??: ????    |   | ??: ????   |   | ??: ?????  |   |        |  |
 * |  | - ????       |   | - ????      |   | - ?????    |   |        |  |
 * |  | = ??          |   | = ??         |   | = ??        |   |        |  |
 * |  |                |   |               |   |              |   |        |  |
 * |  | ??: ????    |   | ??: PWM      |   | ??: ??PWM  |   | ??PWM|  |
 * |  +----------------+   +----------------+   +----------------+   +--------+  |
 * |        |                   |                   |                   |        |
 * |        v                   v                   v                   v        |
 * |  +--------------------------------------------------------------+            |
 * |  |  PID??: out = Kp*e + Ki*?(e*dt) + Kd*de/dt               |            |
 * |  +--------------------------------------------------------------+            |
 * |                                                                              |
 * +------------------------------------------------------------------------------+
 *
 * ??????
 *    ???? + ???? = ????
 *    ???? - ???? = ?????
 *
 * Created on: 2026-03-15
 *     Author: Daydreamer
 */

#include "zf_common_headfile.h"

//------------------------------------------- ??? ------------------------------------------------------------

/* PID ?? PWM ???????? Motor.c ????? */
#define WHEEL_PID_PWM_PERCENT_MAX    60.0f
#define WHEEL_PID_DEADZONE  4.0f     // ?????????PWM=5?    5.0f
#define WHEEL_PID_MIN_START_PWM   10.0f     // ????PWM???????

/* ???????? */
#include "vehicle_config.h"


//------------------------------------------- ???? ----------------------------------------------------------

/* ?? PID ?????? */
WHEEL_PID_LAYER g_wheel_pid;

/* ?????????? (m/s) */
static float s_speed_ref_abs_max_mps = 1.0f;

/* ??????????? (rad/s) */
static float s_yaw_rate_ref_abs_max_rps = 5.0f;

int16 origin_pidout = 0;

int16 origin_pid_sum_error = 0;
int16 origin_error_right = 0;
int16 origin_error_left = 0;
//------------------------------------------- ?????? ------------------------------------------------------

static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s);


//------------------------------------------- PID ???? ------------------------------------------------------

// ??? PID ????
static void pid_para_init(PID_INFO *pid_info)
{
    if (!pid_info) return;

    pid_info->iError    = 0.0f;
    pid_info->LastError = 0.0f;
    pid_info->PrevError = 0.0f;
    pid_info->SumError  = 0.0f;
    pid_info->LastData  = 0.0f;
}

// ????? PID ??
static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s)
{
    if (!pid_info || !param) return 0.0f;
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    /* ???? */
    pid_info->iError = error;

    /* ???????? */
    pid_info->SumError += error * dt_s;
    origin_pid_sum_error = pid_info->SumError;

    if (param[PID_PARAM_I_LIMIT] > 0.0f)
    {
        pid_info->SumError = func_limit(pid_info->SumError, param[PID_PARAM_I_LIMIT]);
    }

    /* ??? */
    float diff = (error - pid_info->LastError) / dt_s;

    /* PID ?? */
    float out = param[PID_PARAM_KP] * error
              + param[PID_PARAM_KI] * pid_info->SumError
              + param[PID_PARAM_KD] * diff;

    origin_pidout = out;


    /* ?????? */
    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = error;

    return out;
}


//------------------------------------------- ????? -------------------------------------------------------

// PID ??????
void wheel_pid_init(void)
{
    /* ??????? */
    memset(&g_wheel_pid, 0, sizeof(g_wheel_pid));

    /* ????? PID ?? */
    pid_para_init(&g_wheel_pid.pid_pos_left);
    pid_para_init(&g_wheel_pid.pid_pos_right);
    pid_para_init(&g_wheel_pid.pid_spd_left);
    pid_para_init(&g_wheel_pid.pid_spd_right);
    pid_para_init(&g_wheel_pid.pid_yaw_rate);

    /* ?? PID ?????????????? */

    // ?????
    g_wheel_pid.position_param[PID_PARAM_KP]      = 10.0f;
    g_wheel_pid.position_param[PID_PARAM_KI]      = 0.0f;
    g_wheel_pid.position_param[PID_PARAM_KD]      = 0.4f;
    g_wheel_pid.position_param[PID_PARAM_I_LIMIT] = 2.5f;

    // ?????????
    g_wheel_pid.speed_param_left[PID_PARAM_KP]       = 10.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_KI]       = 1.8f;
    g_wheel_pid.speed_param_left[PID_PARAM_KD]       = 0.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_I_LIMIT]  = 200.0f;

    // ?????????
    g_wheel_pid.speed_param_right[PID_PARAM_KP]      = 10.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_KI]      = 1.8f;
    g_wheel_pid.speed_param_right[PID_PARAM_KD]      = 0.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_I_LIMIT] = 200.0f;

    // ????????
    g_wheel_pid.yaw_rate_param[PID_PARAM_KP]      = 2.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KI]      = 0.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KD]      = 0.1f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_I_LIMIT] = 100.0f;

    /* ?????? */
    g_wheel_pid.cmd_speed_left_mps  = 0.0f;
    g_wheel_pid.cmd_speed_right_mps = 0.0f;
    g_wheel_pid.ref_speed_left_mps  = 0.0f;
    g_wheel_pid.ref_speed_right_mps = 0.0f;
    g_wheel_pid.cmd_yaw_rate_rps    = 0.0f;
    g_wheel_pid.ref_yaw_rate_rps    = 0.0f;

    /* ??????? */
    g_wheel_pid.enable_output        = 0u;
    g_wheel_pid.enable_speed_loop    = 0u;
    g_wheel_pid.enable_position_loop = 0u;
    g_wheel_pid.enable_yaw_rate_loop = 0u;

    g_wheel_pid.initialized = 1u;

    wheel_pid_cmd_init();
}

// PID ??????
void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,
                      uint8 enable_position_loop, uint8 enable_yaw_rate_loop)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    g_wheel_pid.enable_output        = enable_output ? 1u : 0u;
    g_wheel_pid.enable_speed_loop    = enable_speed_loop ? 1u : 0u;
    g_wheel_pid.enable_position_loop = enable_position_loop ? 1u : 0u;
    g_wheel_pid.enable_yaw_rate_loop = enable_yaw_rate_loop ? 1u : 0u;
}

//------------------------------------------- ????? -------------------------------------------------------

// ????????? (m/s)
void wheel_pid_set_speed_target(float left_mps, float right_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_speed_left_mps  = left_mps;
    g_wheel_pid.cmd_speed_right_mps = right_mps;
}

// ??????????????????
void wheel_pid_set_target_speed(float target_speed_mps)
{
    wheel_pid_set_speed_target(target_speed_mps, target_speed_mps);
}

// ????????? (m)
void wheel_pid_set_position_target(float left_m, float right_m)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.target_pos_left_m  = left_m;
    g_wheel_pid.target_pos_right_m = right_m;
}

// ????????? (rad/s)
void wheel_pid_set_yaw_target(float yaw_rate_rps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_yaw_rate_rps = yaw_rate_rps;
}

//------------------------------------------- ?????? -----------------------------------------------------

void wheel_pid_set_speed_param_left(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.speed_param_left[PID_PARAM_KP] = kp;
    g_wheel_pid.speed_param_left[PID_PARAM_KI] = ki;
    g_wheel_pid.speed_param_left[PID_PARAM_KD] = kd;
    g_wheel_pid.speed_param_left[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_speed_param_right(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.speed_param_right[PID_PARAM_KP] = kp;
    g_wheel_pid.speed_param_right[PID_PARAM_KI] = ki;
    g_wheel_pid.speed_param_right[PID_PARAM_KD] = kd;
    g_wheel_pid.speed_param_right[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit)
{
    wheel_pid_set_speed_param_left(kp, ki, kd, i_limit);
    wheel_pid_set_speed_param_right(kp, ki, kd, i_limit);
}

void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.position_param[PID_PARAM_KP] = kp;
    g_wheel_pid.position_param[PID_PARAM_KI] = ki;
    g_wheel_pid.position_param[PID_PARAM_KD] = kd;
    g_wheel_pid.position_param[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.yaw_rate_param[PID_PARAM_KP] = kp;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KI] = ki;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KD] = kd;
    g_wheel_pid.yaw_rate_param[PID_PARAM_I_LIMIT] = i_limit;
}

void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (abs_max_mps > 1e-6f)
        s_speed_ref_abs_max_mps = abs_max_mps;
}

void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (abs_max_rps > 1e-6f)
        s_yaw_rate_ref_abs_max_rps = abs_max_rps;
}

//------------------------------------------- ??????? ---------------------------------------------------

// ?? PID ????????????
static void wheel_pid_update_cascade(const EncoderLayerState *enc, float yaw_rps, float dt_s)
{
    if (!enc) return;
    if (!g_wheel_pid.initialized) wheel_pid_init();
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    const uint8 pos_on = g_wheel_pid.enable_position_loop;
    const uint8 spd_on = g_wheel_pid.enable_speed_loop;
    const uint8 yaw_on = g_wheel_pid.enable_yaw_rate_loop;

    float out_left = 0.0f;
    float out_right = 0.0f;
    float out_steer = 0.0f;

    // ???????????????????
    if (pos_on)
    {
        g_wheel_pid.target_pos_left_m  += g_wheel_pid.cmd_speed_left_mps * dt_s;
        g_wheel_pid.target_pos_right_m += g_wheel_pid.cmd_speed_right_mps * dt_s;
    }

    /* ======================== ??1??? + ?? + ?? ======================== */
    if (pos_on && spd_on && yaw_on)
    {
        // ???
        float err_pos_l = g_wheel_pid.target_pos_left_m - enc->odom_left_m;
        float err_pos_r = g_wheel_pid.target_pos_right_m - enc->odom_right_m;

        float v_l = pid_step(&g_wheel_pid.pid_pos_left, g_wheel_pid.position_param, err_pos_l, dt_s);
        float v_r = pid_step(&g_wheel_pid.pid_pos_right, g_wheel_pid.position_param, err_pos_r, dt_s);

        // ???????
        v_l = func_limit_ab(v_l, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);
        v_r = func_limit_ab(v_r, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);

        g_wheel_pid.ref_speed_left_mps = v_l;
        g_wheel_pid.ref_speed_right_mps = v_r;

        // ???
        float err_spd_l = v_l - enc->speed_left_mps;
        float err_spd_r = v_r - enc->speed_right_mps;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

        // ??????
//        float target_yaw = (v_r - v_l) / COMPAT_WHEELBASE_M;
//        g_wheel_pid.ref_yaw_rate_rps = target_yaw;
//
//        float err_yaw = target_yaw - yaw_rps;
//        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);

        out_steer = 0;
    }
    /* ======================== ??2??? + ?? ======================== */
    else if (!pos_on && spd_on && yaw_on)
    {
        g_wheel_pid.ref_speed_left_mps  = g_wheel_pid.cmd_speed_left_mps;
        g_wheel_pid.ref_speed_right_mps = g_wheel_pid.cmd_speed_right_mps;

        float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
        float err_spd_r = g_wheel_pid.cmd_speed_right_mps - enc->speed_right_mps;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

        float target_yaw = (g_wheel_pid.cmd_speed_right_mps - g_wheel_pid.cmd_speed_left_mps) / COMPAT_WHEELBASE_M;
        g_wheel_pid.ref_yaw_rate_rps = target_yaw;

        float err_yaw = target_yaw - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /* ======================== ??3????? ======================== */
    else if (!pos_on && spd_on && !yaw_on)
    {
        g_wheel_pid.ref_speed_left_mps  = g_wheel_pid.cmd_speed_left_mps;
        g_wheel_pid.ref_speed_right_mps = g_wheel_pid.cmd_speed_right_mps;

        float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
        //float err_spd_l = enc->speed_left_mps - g_wheel_pid.cmd_speed_left_mps;     // actual - target
        float err_spd_r = g_wheel_pid.cmd_speed_right_mps + enc->speed_right_mps;

        origin_error_right = err_spd_r;
        origin_error_left  = err_spd_l;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

        out_steer = 0.0f;

    }
    /* ======================== ??4???????? ======================== */
    else if (!pos_on && !spd_on && yaw_on)
    {
        out_left = 0.0f;
        out_right = 0.0f;
        g_wheel_pid.ref_yaw_rate_rps = g_wheel_pid.cmd_yaw_rate_rps;

        float err_yaw = g_wheel_pid.cmd_yaw_rate_rps - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /* ======================== ??5????? ======================== */
    else
    {
        g_wheel_pid.ref_speed_left_mps = 0.0f;
        g_wheel_pid.ref_speed_right_mps = 0.0f;
        g_wheel_pid.ref_yaw_rate_rps = 0.0f;
        out_left = out_right = out_steer = 0.0f;
    }

    /* PWM ?? */
    out_left  = func_limit(out_left,  WHEEL_PID_PWM_PERCENT_MAX);
    out_right = func_limit(out_right, WHEEL_PID_PWM_PERCENT_MAX);
    out_steer = func_limit(out_steer, STEER_PWM_ABS_MAX);

//    /* ???? */
//    if (fabs(out_left) < WHEEL_PID_DEADZONE)
//        out_left = 0.0f;
//
//    if (fabs(out_right) < WHEEL_PID_DEADZONE)
//        out_right = 0.0f;

//    // ??
//    if (out_left > 0 && out_left < WHEEL_PID_MIN_START_PWM)
//        out_left = WHEEL_PID_MIN_START_PWM;
//    else if (out_left < 0 && out_left > -WHEEL_PID_MIN_START_PWM)
//        out_left = -WHEEL_PID_MIN_START_PWM;
//
//    // ??
//    if (out_right > 0 && out_right < WHEEL_PID_MIN_START_PWM)
//        out_right = WHEEL_PID_MIN_START_PWM;
//    else if (out_right < 0 && out_right > -WHEEL_PID_MIN_START_PWM)
//        out_right = -WHEEL_PID_MIN_START_PWM;

    /* ????? */
    g_wheel_pid.out_left_pwm  = out_left;
    g_wheel_pid.out_right_pwm = out_right;
    g_wheel_pid.out_steer_pwm = out_steer;

    /* ?????? */
    if (g_wheel_pid.enable_output)
    {
//----------------------------??pid--------------------------------------------------------//
        if (out_left > 0)
            motor_control(motor_LB, MOTOR_DIR_FORWARD, (uint8)out_left);
        else if (out_left < 0)
            motor_control(motor_LB, MOTOR_DIR_REVERSE, (uint8)(-out_left));
        else
            motor_control(motor_LB, MOTOR_DIR_BRAKE, 0);

        if (out_right > 0)
            motor_control(motor_RB, MOTOR_DIR_FORWARD, (uint8)out_right);
        else if (out_right < 0)
            motor_control(motor_RB, MOTOR_DIR_REVERSE, (uint8)(-out_right));
        else
            motor_control(motor_RB, MOTOR_DIR_BRAKE, 0);

//----------------------------??-------------------------------------------------------------//
//        if (out_left > 0)
//            motor_control(motor_LB, MOTOR_DIR_FORWARD, 50);
//        else if (out_left < 0)
//            motor_control(motor_LB, MOTOR_DIR_REVERSE, 50);
//        else
//            motor_control(motor_LB, MOTOR_DIR_BRAKE, 0);
//
//        if (out_right > 0)
//            motor_control(motor_RB, MOTOR_DIR_FORWARD,50);
//        else if (out_right < 0)
//            motor_control(motor_RB, MOTOR_DIR_REVERSE, 50);
//        else
//            motor_control(motor_RB, MOTOR_DIR_BRAKE, 0);

        /* ?? PWM ???????? g_wheel_pid.out_steer_pwm */
    }
}

// ???????????????
void wheel_pid_update(const EncoderLayerState *enc, float dt_s)
{
    /* ?????????????? */
    float estimated_yaw = (enc->speed_right_mps - enc->speed_left_mps) / COMPAT_WHEELBASE_M;
    wheel_pid_update_cascade(enc, estimated_yaw, dt_s);
}