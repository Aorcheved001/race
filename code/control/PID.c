/*
 * PID.c
 *
<<<<<<< HEAD
 * ¼¶ÁªÊ½ PID ¿ØÖÆÆ÷
 * Ö§³Ö£ºÎ»ÖÃ»· + ËÙ¶È»· + Æ«º½½ÇËÙ¶È»·£¨×ªÍä¿ØÖÆ£©
 *
 * ¿ØÖÆ¿òÍ¼
 * +------------------------------------------------------------------------------+
 * |                          PID ¼¶Áª¿ØÖÆ½á¹¹Í¼                                  |
 * +------------------------------------------------------------------------------+
 * |                                                                              |
 * |  +----------------+   +----------------+   +----------------+   +--------+  |
 * |  |   Î»ÖÃ»·         |   |   ËÙ¶È»·       |   |   Æ«º½½ÇËÙ¶È»·  |   | Êä³öPWM|  |
 * |  | (Íâ»·)          |   | (ÖÐ»·)         |   | (ÄÚ»·)        |   |        |  |
 * |  |                |   |               |   |              |   |        |  |
 * |  | ÊäÈë: Ä¿±êÎ»ÖÃ    |   | ÊäÈë: Ä¿±êËÙ¶È   |   | ÊäÈë: Ä¿±ê½ÇËÙ  |   |        |  |
 * |  | - µ±Ç°Î»ÖÃ       |   | - µ±Ç°ËÙ¶È      |   | - µ±Ç°½ÇËÙ¶È    |   |        |  |
 * |  | = Îó²î          |   | = Îó²î         |   | = Îó²î        |   |        |  |
 * |  |                |   |               |   |              |   |        |  |
 * |  | Êä³ö: Ä¿±êËÙ¶È    |   | Êä³ö: PWM      |   | Êä³ö: ×ªÏòPWM  |   | ×îÖÕPWM|  |
=======
 * çº§è”å¼ PID æŽ§åˆ¶å™¨
 * æ”¯æŒï¼šä½ç½®çŽ¯ + é€Ÿåº¦çŽ¯ + åèˆªè§’é€Ÿåº¦çŽ¯ï¼ˆè½¬å¼¯æŽ§åˆ¶ï¼‰
 *
 * æŽ§åˆ¶æ¡†å›¾
 * +------------------------------------------------------------------------------+
 * |                          PID çº§è”æŽ§åˆ¶ç»“æž„å›¾                                  |
 * +------------------------------------------------------------------------------+
 * |                                                                              |
 * |  +----------------+   +----------------+   +----------------+   +--------+  |
 * |  |   ä½ç½®çŽ¯       |   |   é€Ÿåº¦çŽ¯       |   |   åèˆªè§’é€Ÿåº¦çŽ¯  |   | è¾“å‡ºPWM|  |
 * |  | (å¤–çŽ¯)         |   | (ä¸­çŽ¯)         |   | (å†…çŽ¯)         |   |        |  |
 * |  |                |   |                |   |                |   |        |  |
 * |  | è¾“å…¥: ç›®æ ‡ä½ç½® |   | è¾“å…¥: ç›®æ ‡é€Ÿåº¦ |   | è¾“å…¥: ç›®æ ‡è§’é€Ÿ |   |        |  |
 * |  | - å½“å‰ä½ç½®     |   | - å½“å‰é€Ÿåº¦     |   | - å½“å‰è§’é€Ÿåº¦   |   |        |  |
 * |  | = è¯¯å·®         |   | = è¯¯å·®         |   | = è¯¯å·®         |   |        |  |
 * |  |                |   |                |   |                |   |        |  |
 * |  | è¾“å‡º: ç›®æ ‡é€Ÿåº¦ |   | è¾“å‡º: PWM      |   | è¾“å‡º: è½¬å‘PWM  |   | æœ€ç»ˆPWM|  |
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
 * |  +----------------+   +----------------+   +----------------+   +--------+  |
 * |        |                   |                   |                   |        |
 * |        v                   v                   v                   v        |
 * |  +--------------------------------------------------------------+            |
<<<<<<< HEAD
 * |  |  PID¹«Ê½: out = Kp*e + Ki*¦²(e*dt) + Kd*de/dt               |            |
=======
 * |  |  PIDå…¬å¼: out = Kp*e + Ki*Î£(e*dt) + Kd*de/dt               |            |
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
 * |  +--------------------------------------------------------------+            |
 * |                                                                              |
 * +------------------------------------------------------------------------------+
 *
<<<<<<< HEAD
 * ²îËÙ×ªÏòÔ­Àí
 *    ÓÒÂÖËÙ¶È + ×óÂÖËÙ¶È = Ö±ÏßËÙ¶È
 *    ÓÒÂÖËÙ¶È - ×óÂÖËÙ¶È = ×ªÏò½ÇËÙ¶È
=======
 * å·®é€Ÿè½¬å‘åŽŸç†
 *    å³è½®é€Ÿåº¦ + å·¦è½®é€Ÿåº¦ = ç›´çº¿é€Ÿåº¦
 *    å³è½®é€Ÿåº¦ - å·¦è½®é€Ÿåº¦ = è½¬å‘è§’é€Ÿåº¦
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
 *
 * Created on: 2026-03-15
 *     Author: Daydreamer
 */

#include "zf_common_headfile.h"

<<<<<<< HEAD
//------------------------------------------- ºê¶¨Òå ------------------------------------------------------------

/* PID Êä³ö PWM °Ù·Ö±È×î´óÖµ£¨Óë Motor.c ½Ó¿ÚÆ¥Åä£© */
#define WHEEL_PID_PWM_PERCENT_MAX    60.0f
#define WHEEL_PID_DEADZONE  4.0f     // ËÀÇøãÐÖµ£¨×î´óËÀÇøPWM=5£©    5.0f
#define WHEEL_PID_MIN_START_PWM   10.0f     // ×îÐ¡Æô¶¯PWMÖµ£¨¹Ø¼ü²ÎÊý£©

/* °üº¬³µÁ¾ÅäÖÃ²ÎÊý */
#include "vehicle_config.h"


//------------------------------------------- È«¾Ö±äÁ¿ ----------------------------------------------------------

/* È«¾Ö PID ¿ØÖÆÆ÷½á¹¹Ìå */
WHEEL_PID_LAYER g_wheel_pid;

/* ËÙ¶È»·²Î¿¼ËÙ¶ÈÏÞ·ùÖµ (m/s) */
static float s_speed_ref_abs_max_mps = 1.0f;

/* Æ«º½½ÇËÙ¶È»·²Î¿¼ÏÞ·ùÖµ (rad/s) */
=======
//------------------------------------------- å®å®šä¹‰ ------------------------------------------------------------

/* PID è¾“å‡º PWM ç™¾åˆ†æ¯”æœ€å¤§å€¼ï¼ˆä¸Ž Motor.c æŽ¥å£åŒ¹é…ï¼‰ */
#define WHEEL_PID_PWM_PERCENT_MAX    100.0f

/* åŒ…å«è½¦è¾†é…ç½®å‚æ•° */
#include "vehicle_config.h"


//------------------------------------------- å…¨å±€å˜é‡ ----------------------------------------------------------

/* å…¨å±€ PID æŽ§åˆ¶å™¨ç»“æž„ä½“ */
WHEEL_PID_LAYER g_wheel_pid;

/* é€Ÿåº¦çŽ¯å‚è€ƒé€Ÿåº¦é™å¹…å€¼ (m/s) */
static float s_speed_ref_abs_max_mps = 2.5f;

/* åèˆªè§’é€Ÿåº¦çŽ¯å‚è€ƒé™å¹…å€¼ (rad/s) */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
static float s_yaw_rate_ref_abs_max_rps = 5.0f;

int16 origin_pidout = 0;

<<<<<<< HEAD
int16 origin_pid_sum_error = 0;
int16 origin_error_right = 0;
int16 origin_error_left = 0;
//------------------------------------------- ÄÚ²¿º¯ÊýÉùÃ÷ ------------------------------------------------------
=======
//------------------------------------------- å†…éƒ¨å‡½æ•°å£°æ˜Ž ------------------------------------------------------
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6

static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s);


<<<<<<< HEAD
//------------------------------------------- PID ºËÐÄËã·¨ ------------------------------------------------------

// ³õÊ¼»¯ PID ×´Ì¬±äÁ¿
=======
//------------------------------------------- PID æ ¸å¿ƒç®—æ³• ------------------------------------------------------

// åˆå§‹åŒ– PID çŠ¶æ€å˜é‡
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
static void pid_para_init(PID_INFO *pid_info)
{
    if (!pid_info) return;

    pid_info->iError    = 0.0f;
    pid_info->LastError = 0.0f;
    pid_info->PrevError = 0.0f;
    pid_info->SumError  = 0.0f;
    pid_info->LastData  = 0.0f;
}

<<<<<<< HEAD
// ±ê×¼Î»ÖÃÊ½ PID ¼ÆËã
=======
// æ ‡å‡†ä½ç½®å¼ PID è®¡ç®—
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
static float pid_step(PID_INFO *pid_info, const float *param, float error, float dt_s)
{
    if (!pid_info || !param) return 0.0f;
    if (dt_s <= 1e-6f) dt_s = 0.004f;

    /* è®°å½•å½“å‰è¯¯å·® */
    pid_info->iError = error;

<<<<<<< HEAD
    /* »ý·ÖÏî£¨´øÏÞ·ù£© */
=======
    /* ç§¯åˆ†é¡¹ï¼ˆå¸¦é™å¹…ï¼‰ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    pid_info->SumError += error * dt_s;
    origin_pid_sum_error = pid_info->SumError;

    if (param[PID_PARAM_I_LIMIT] > 0.0f)
    {
        pid_info->SumError = func_limit(pid_info->SumError, param[PID_PARAM_I_LIMIT]);
    }

<<<<<<< HEAD
    /* Î¢·ÖÏî */
    float diff = (error - pid_info->LastError) / dt_s;

    /* PID Êä³ö */
=======
    /* å¾®åˆ†é¡¹ */
    float diff = (error - pid_info->LastError) / dt_s;

    /* PID è¾“å‡º */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    float out = param[PID_PARAM_KP] * error
              + param[PID_PARAM_KI] * pid_info->SumError
              + param[PID_PARAM_KD] * diff;

<<<<<<< HEAD
    origin_pidout = out;


    /* ¸üÐÂÀúÊ·Îó²î */
=======
    /* æ›´æ–°åŽ†å²è¯¯å·® */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    pid_info->PrevError = pid_info->LastError;
    pid_info->LastError = error;

    return out;
}


<<<<<<< HEAD
//------------------------------------------- ³õÊ¼»¯º¯Êý -------------------------------------------------------

// PID ¿ØÖÆÆ÷³õÊ¼»¯
void wheel_pid_init(void)
{
    /* Çå¿ÕÈ«¾Ö½á¹¹Ìå */
    memset(&g_wheel_pid, 0, sizeof(g_wheel_pid));

    /* ³õÊ¼»¯¸÷»· PID ×´Ì¬ */
=======
//------------------------------------------- åˆå§‹åŒ–å‡½æ•° -------------------------------------------------------

// PID æŽ§åˆ¶å™¨åˆå§‹åŒ–
void wheel_pid_init(void)
{
    /* æ¸…ç©ºå…¨å±€ç»“æž„ä½“ */
    memset(&g_wheel_pid, 0, sizeof(g_wheel_pid));

    /* åˆå§‹åŒ–å„çŽ¯ PID çŠ¶æ€ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    pid_para_init(&g_wheel_pid.pid_pos_left);
    pid_para_init(&g_wheel_pid.pid_pos_right);
    pid_para_init(&g_wheel_pid.pid_spd_left);
    pid_para_init(&g_wheel_pid.pid_spd_right);
    pid_para_init(&g_wheel_pid.pid_yaw_rate);

<<<<<<< HEAD
    /* Ä¬ÈÏ PID ²ÎÊý£¨½¨Òé¸ù¾ÝÊµ¼Ê³µÁ¾µ÷ÊÔ£© */

    // Î»ÖÃ»·²ÎÊý
=======
    /* é»˜è®¤ PID å‚æ•°ï¼ˆå»ºè®®æ ¹æ®å®žé™…è½¦è¾†è°ƒè¯•ï¼‰ */

    // ä½ç½®çŽ¯å‚æ•°
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_wheel_pid.position_param[PID_PARAM_KP]      = 10.0f;
    g_wheel_pid.position_param[PID_PARAM_KI]      = 0.0f;
    g_wheel_pid.position_param[PID_PARAM_KD]      = 0.4f;
    g_wheel_pid.position_param[PID_PARAM_I_LIMIT] = 2.5f;

<<<<<<< HEAD
    // ËÙ¶È»·²ÎÊý£¨×óÂÖ£©
    g_wheel_pid.speed_param_left[PID_PARAM_KP]       = 10.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_KI]       = 1.8f;
    g_wheel_pid.speed_param_left[PID_PARAM_KD]       = 0.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_I_LIMIT]  = 200.0f;

    // ËÙ¶È»·²ÎÊý£¨ÓÒÂÖ£©
    g_wheel_pid.speed_param_right[PID_PARAM_KP]      = 10.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_KI]      = 1.8f;
    g_wheel_pid.speed_param_right[PID_PARAM_KD]      = 0.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_I_LIMIT] = 200.0f;

    // Æ«º½½ÇËÙ¶È»·²ÎÊý
=======
    // é€Ÿåº¦çŽ¯å‚æ•°ï¼ˆå·¦è½®ï¼‰
    g_wheel_pid.speed_param_left[PID_PARAM_KP]       = 42.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_KI]       = 0.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_KD]       = 0.0f;
    g_wheel_pid.speed_param_left[PID_PARAM_I_LIMIT]  = 100.0f;

    // é€Ÿåº¦çŽ¯å‚æ•°ï¼ˆå³è½®ï¼‰
    g_wheel_pid.speed_param_right[PID_PARAM_KP]      = 42.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_KI]      = 0.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_KD]      = 0.0f;
    g_wheel_pid.speed_param_right[PID_PARAM_I_LIMIT] = 100.0f;

    // åèˆªè§’é€Ÿåº¦çŽ¯å‚æ•°
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_wheel_pid.yaw_rate_param[PID_PARAM_KP]      = 2.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KI]      = 0.0f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_KD]      = 0.1f;
    g_wheel_pid.yaw_rate_param[PID_PARAM_I_LIMIT] = 100.0f;

<<<<<<< HEAD
    /* ³õÊ¼»¯Ä¿±êÖµ */
=======
    /* åˆå§‹åŒ–ç›®æ ‡å€¼ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_wheel_pid.cmd_speed_left_mps  = 0.0f;
    g_wheel_pid.cmd_speed_right_mps = 0.0f;
    g_wheel_pid.ref_speed_left_mps  = 0.0f;
    g_wheel_pid.ref_speed_right_mps = 0.0f;
    g_wheel_pid.cmd_yaw_rate_rps    = 0.0f;
    g_wheel_pid.ref_yaw_rate_rps    = 0.0f;

<<<<<<< HEAD
    /* Ä¬ÈÏ¹Ø±ÕËùÓÐ»· */
=======
    /* é»˜è®¤å…³é—­æ‰€æœ‰çŽ¯ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_wheel_pid.enable_output        = 0u;
    g_wheel_pid.enable_speed_loop    = 0u;
    g_wheel_pid.enable_position_loop = 0u;
    g_wheel_pid.enable_yaw_rate_loop = 0u;

    g_wheel_pid.initialized = 1u;

    wheel_pid_cmd_init();
}

<<<<<<< HEAD
// PID ¸÷»·Ê¹ÄÜ¿ØÖÆ
=======
// PID å„çŽ¯ä½¿èƒ½æŽ§åˆ¶
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,
                      uint8 enable_position_loop, uint8 enable_yaw_rate_loop)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();

    g_wheel_pid.enable_output        = enable_output ? 1u : 0u;
    g_wheel_pid.enable_speed_loop    = enable_speed_loop ? 1u : 0u;
    g_wheel_pid.enable_position_loop = enable_position_loop ? 1u : 0u;
    g_wheel_pid.enable_yaw_rate_loop = enable_yaw_rate_loop ? 1u : 0u;
}

<<<<<<< HEAD
//------------------------------------------- ÉèÖÃÄ¿±êÖµ -------------------------------------------------------

// ÉèÖÃ×óÓÒÂÖÄ¿±êËÙ¶È (m/s)
=======
//------------------------------------------- è®¾ç½®ç›®æ ‡å€¼ -------------------------------------------------------

// è®¾ç½®å·¦å³è½®ç›®æ ‡é€Ÿåº¦ (m/s)
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_set_speed_target(float left_mps, float right_mps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_speed_left_mps  = left_mps;
    g_wheel_pid.cmd_speed_right_mps = right_mps;
}

<<<<<<< HEAD
// ÉèÖÃÏàÍ¬Ä¿±êËÙ¶È£¨Ö±ÏßÐÐÊ»¿ì½Ý½Ó¿Ú£©
=======
// è®¾ç½®ç›¸åŒç›®æ ‡é€Ÿåº¦ï¼ˆç›´çº¿è¡Œé©¶å¿«æ·æŽ¥å£ï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_set_target_speed(float target_speed_mps)
{
    wheel_pid_set_speed_target(target_speed_mps, target_speed_mps);
}

<<<<<<< HEAD
// ÉèÖÃ×óÓÒÂÖÄ¿±êÎ»ÖÃ (m)
=======
// è®¾ç½®å·¦å³è½®ç›®æ ‡ä½ç½® (m)
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_set_position_target(float left_m, float right_m)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.target_pos_left_m  = left_m;
    g_wheel_pid.target_pos_right_m = right_m;
}

<<<<<<< HEAD
// ÉèÖÃÄ¿±êÆ«º½½ÇËÙ¶È (rad/s)
=======
// è®¾ç½®ç›®æ ‡åèˆªè§’é€Ÿåº¦ (rad/s)
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_set_yaw_target(float yaw_rate_rps)
{
    if (!g_wheel_pid.initialized) wheel_pid_init();
    g_wheel_pid.cmd_yaw_rate_rps = yaw_rate_rps;
}

<<<<<<< HEAD
//------------------------------------------- ²ÎÊýÕû¶¨½Ó¿Ú -----------------------------------------------------
=======
//------------------------------------------- å‚æ•°æ•´å®šæŽ¥å£ -----------------------------------------------------
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6

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

<<<<<<< HEAD
//------------------------------------------- Ö÷¿ØÖÆ¸üÐÂº¯Êý ---------------------------------------------------

// ¼¶Áª PID ¸üÐÂºËÐÄº¯Êý£¨ÄÚ²¿Ê¹ÓÃ£©
=======
//------------------------------------------- ä¸»æŽ§åˆ¶æ›´æ–°å‡½æ•° ---------------------------------------------------

// çº§è” PID æ›´æ–°æ ¸å¿ƒå‡½æ•°ï¼ˆå†…éƒ¨ä½¿ç”¨ï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
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

<<<<<<< HEAD
    // Î»ÖÃ»·Ê¹ÄÜÊ±£¬ÀÛ¼ÓÎ»ÖÃÄ¿±ê£¨¹ì¼£¸ú×Ù£©
=======
    // ä½ç½®çŽ¯ä½¿èƒ½æ—¶ï¼Œç´¯åŠ ä½ç½®ç›®æ ‡ï¼ˆè½¨è¿¹è·Ÿè¸ªï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (pos_on)
    {
        g_wheel_pid.target_pos_left_m  += g_wheel_pid.cmd_speed_left_mps * dt_s;
        g_wheel_pid.target_pos_right_m += g_wheel_pid.cmd_speed_right_mps * dt_s;
    }

<<<<<<< HEAD
    /* ======================== Ä£Ê½1£ºÎ»ÖÃ + ËÙ¶È  (Æ«º½½ÇËÙ¶È) ======================== */
    if (pos_on && spd_on && yaw_on)
    {
        // Î»ÖÃ»·
=======
    /* ======================== æ¨¡å¼1ï¼šä½ç½® + é€Ÿåº¦ + åèˆªè§’é€Ÿåº¦ ======================== */
    if (pos_on && spd_on && yaw_on)
    {
        // ä½ç½®çŽ¯
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
        float err_pos_l = g_wheel_pid.target_pos_left_m - enc->odom_left_m;
        float err_pos_r = g_wheel_pid.target_pos_right_m - enc->odom_right_m;

        float v_l = pid_step(&g_wheel_pid.pid_pos_left, g_wheel_pid.position_param, err_pos_l, dt_s);
        float v_r = pid_step(&g_wheel_pid.pid_pos_right, g_wheel_pid.position_param, err_pos_r, dt_s);

<<<<<<< HEAD
        // ÏÞ·ùËÙ¶È²Î¿¼Öµ
=======
        // é™å¹…é€Ÿåº¦å‚è€ƒå€¼
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
        v_l = func_limit_ab(v_l, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);
        v_r = func_limit_ab(v_r, -s_speed_ref_abs_max_mps, s_speed_ref_abs_max_mps);

        g_wheel_pid.ref_speed_left_mps = v_l;
        g_wheel_pid.ref_speed_right_mps = v_r;

<<<<<<< HEAD
        // ËÙ¶È»·
=======
        // é€Ÿåº¦çŽ¯
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
        float err_spd_l = v_l - enc->speed_left_mps;
        float err_spd_r = v_r - enc->speed_right_mps;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

<<<<<<< HEAD
        // Æ«º½½ÇËÙ¶È»·
//        float target_yaw = (v_r - v_l) / COMPAT_WHEELBASE_M;
//        g_wheel_pid.ref_yaw_rate_rps = target_yaw;
//
//        float err_yaw = target_yaw - yaw_rps;
//        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);

        out_steer = 0;
    }
    /* ======================== Ä£Ê½2£ºËÙ¶È + Æ«º½½ÇËÙ¶È ======================== */
=======
        // åèˆªè§’é€Ÿåº¦çŽ¯
        float target_yaw = (v_r - v_l) / COMPAT_WHEELBASE_M;
        g_wheel_pid.ref_yaw_rate_rps = target_yaw;

        float err_yaw = target_yaw - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
    /* ======================== æ¨¡å¼2ï¼šé€Ÿåº¦ + åèˆªè§’é€Ÿåº¦ ======================== */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
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
<<<<<<< HEAD
    /* ======================== Ä£Ê½3£º½öËÙ¶È»· ======================== */
=======
    /* ======================== æ¨¡å¼3ï¼šä»…é€Ÿåº¦çŽ¯ ======================== */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    else if (!pos_on && spd_on && !yaw_on)
    {
        g_wheel_pid.ref_speed_left_mps  = g_wheel_pid.cmd_speed_left_mps;
        g_wheel_pid.ref_speed_right_mps = g_wheel_pid.cmd_speed_right_mps;

        float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
        //float err_spd_l = enc->speed_left_mps - g_wheel_pid.cmd_speed_left_mps;     // actual - target
        float err_spd_r = g_wheel_pid.cmd_speed_right_mps + enc->speed_right_mps;

<<<<<<< HEAD
        origin_error_right = err_spd_r;
        origin_error_left  = err_spd_l;

        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);

=======
        out_left  = pid_step(&g_wheel_pid.pid_spd_left,  g_wheel_pid.speed_param_left,  err_spd_l, dt_s);
        out_right = pid_step(&g_wheel_pid.pid_spd_right, g_wheel_pid.speed_param_right, err_spd_r, dt_s);
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
        out_steer = 0.0f;

    }
<<<<<<< HEAD
    /* ======================== Ä£Ê½4£º½öÆ«º½½ÇËÙ¶È»· ======================== */
=======
    /* ======================== æ¨¡å¼4ï¼šä»…åèˆªè§’é€Ÿåº¦çŽ¯ ======================== */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    else if (!pos_on && !spd_on && yaw_on)
    {
        out_left = 0.0f;
        out_right = 0.0f;
        g_wheel_pid.ref_yaw_rate_rps = g_wheel_pid.cmd_yaw_rate_rps;

        float err_yaw = g_wheel_pid.cmd_yaw_rate_rps - yaw_rps;
        out_steer = pid_step(&g_wheel_pid.pid_yaw_rate, g_wheel_pid.yaw_rate_param, err_yaw, dt_s);
    }
<<<<<<< HEAD
    /* ======================== Ä£Ê½5£ºÈ«²¿¹Ø±Õ ======================== */
=======
    /* ======================== æ¨¡å¼5ï¼šå…¨éƒ¨å…³é—­ ======================== */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    else
    {
        g_wheel_pid.ref_speed_left_mps = 0.0f;
        g_wheel_pid.ref_speed_right_mps = 0.0f;
        g_wheel_pid.ref_yaw_rate_rps = 0.0f;
        out_left = out_right = out_steer = 0.0f;
    }

<<<<<<< HEAD
    /* PWM ÏÞ·ù */
=======
    /* PWM é™å¹… */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    out_left  = func_limit(out_left,  WHEEL_PID_PWM_PERCENT_MAX);
    out_right = func_limit(out_right, WHEEL_PID_PWM_PERCENT_MAX);
    out_steer = func_limit(out_steer, STEER_PWM_ABS_MAX);

<<<<<<< HEAD
//    /* ËÀÇø¿ØÖÆ */
//    if (fabs(out_left) < WHEEL_PID_DEADZONE)
//        out_left = 0.0f;
//
//    if (fabs(out_right) < WHEEL_PID_DEADZONE)
//        out_right = 0.0f;

//    // ×óÂÖ
//    if (out_left > 0 && out_left < WHEEL_PID_MIN_START_PWM)
//        out_left = WHEEL_PID_MIN_START_PWM;
//    else if (out_left < 0 && out_left > -WHEEL_PID_MIN_START_PWM)
//        out_left = -WHEEL_PID_MIN_START_PWM;
//
//    // ÓÒÂÖ
//    if (out_right > 0 && out_right < WHEEL_PID_MIN_START_PWM)
//        out_right = WHEEL_PID_MIN_START_PWM;
//    else if (out_right < 0 && out_right > -WHEEL_PID_MIN_START_PWM)
//        out_right = -WHEEL_PID_MIN_START_PWM;

    /* ±£´æÊä³öÖµ */
=======
    /* ä¿å­˜è¾“å‡ºå€¼ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_wheel_pid.out_left_pwm  = out_left;
    g_wheel_pid.out_right_pwm = out_right;
    g_wheel_pid.out_steer_pwm = out_steer;

<<<<<<< HEAD
    /* Ö´ÐÐµç»ú¿ØÖÆ */
    if (g_wheel_pid.enable_output)
    {
//----------------------------Õý³£pid--------------------------------------------------------//
=======
    /* æ‰§è¡Œç”µæœºæŽ§åˆ¶ */
    if (g_wheel_pid.enable_output)
    {
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
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

<<<<<<< HEAD
//----------------------------²âÊÔ-------------------------------------------------------------//
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

        /* ×ªÏò PWM ¿ÉÓÉÍâ²¿Ä£¿éÊ¹ÓÃ g_wheel_pid.out_steer_pwm */
    }
}

// Íâ²¿µ÷ÓÃ½Ó¿Ú£¨ÍÆ¼öÊ¹ÓÃ´Ëº¯Êý£©
void wheel_pid_update(const EncoderLayerState *enc, float dt_s)
{
    /* Ê¹ÓÃ±àÂëÆ÷ËÙ¶È¹ÀËãÆ«º½½ÇËÙ¶È */
=======
        /* è½¬å‘ PWM å¯ç”±å¤–éƒ¨æ¨¡å—ä½¿ç”¨ g_wheel_pid.out_steer_pwm */
    }
}

// å¤–éƒ¨è°ƒç”¨æŽ¥å£ï¼ˆæŽ¨èä½¿ç”¨æ­¤å‡½æ•°ï¼‰
void wheel_pid_update(const EncoderLayerState *enc, float dt_s)
{
    /* ä½¿ç”¨ç¼–ç å™¨é€Ÿåº¦ä¼°ç®—åèˆªè§’é€Ÿåº¦ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    float estimated_yaw = (enc->speed_right_mps - enc->speed_left_mps) / COMPAT_WHEELBASE_M;
    wheel_pid_update_cascade(enc, estimated_yaw, dt_s);
}