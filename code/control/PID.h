/*
 * PID.h
 *
<<<<<<< HEAD
 * ¼¶ÁªÊ½ PID ¿ØÖÆÆ÷
 * Ö§³Ö£ºÎ»ÖÃ»· + ËÙ¶È»· + Æ«º½½ÇËÙ¶È»·
 *
 * ÕûÌåËµÃ÷
 * +------------------------------------------------------------------+
 * |                    ²îËÙµ×ÅÌ PID ¿ØÖÆ¼Ü¹¹                         |
 * +------------------------------------------------------------------+
 * |                                                                  |
 * |   Íâ»·(Î»ÖÃ»·)   -->   ÖÐ»·(ËÙ¶È»·)   -->   ÄÚ»·(Æ«º½½ÇËÙ¶È»·)   -->   Êä³öPWM   |
 * |   target_pos          ref_speed          ref_yaw_rate           out_steer_pwm   |
 * |   (m)                 (m/s)              (rad/s)                ×ªÏòPWM         |
 * |                                                                  |
 * |   ÊäÈë£º              ÊäÈë£º             ÊäÈë£º                                |
 * |   ÀÛ¼ÓÎ»ÖÃÖ¸Áî        ËÙ¶ÈÄ¿±êÖµ         Æ«º½½ÇËÙ¶ÈÖ¸Áî / ×ªÏòÖ¸Áî            |
 * |                                                                  |
 * |   ²ÎÊý£º              ²ÎÊý£º             ²ÎÊý£º                                |
 * |   position_param      speed_param        yaw_rate_param                       |
 * |                                                                  |
 * +------------------------------------------------------------------+
 * |   ËÙ¶È»·Ö±½ÓÊä³ö×óÓÒÂÖ PWM                                        |
 * |   Æ«º½½ÇËÙ¶ÈÆ«²î = Ä¿±ê×ªÍä½ÇËÙ¶È - Êµ¼Ê×ªÍä½ÇËÙ¶È                 |
 * +------------------------------------------------------------------+
 *
 * Ê¹ÓÃ·½·¨£º
 *   1. wheel_pid_init()              - ³õÊ¼»¯£¨mainÖÐµ÷ÓÃÒ»´Î£©
 *   2. wheel_pid_enable(...)         - Ê¹ÄÜ¸÷¿ØÖÆ»·
 *   3. wheel_pid_set_speed_target()  - ÉèÖÃËÙ¶ÈÄ¿±ê£¨m/s£©
 *   4. wheel_pid_set_yaw_target()    - ÉèÖÃÆ«º½½ÇËÙ¶ÈÄ¿±ê£¨rad/s£©
 *   5. wheel_pid_update(enc, dt)     - ÖÜÆÚµ÷ÓÃ£¨ÍÆ¼ö4ms£©
 *
 * ¼æÈÝ¾É½Ó¿Ú£º
 *   - wheel_pid_enable(1,1,0,1)      Ê¹ÄÜ¶ÔÓ¦»·
 *   - wheel_pid_set_target_speed(v)  ÉèÖÃÏàÍ¬×óÓÒÂÖËÙ¶È
 *   - wheel_pid_update(enc, dt)      ÄÚ²¿×Ô¶¯¹ÀËãÆ«º½½ÇËÙ¶È
=======
 * çº§è”å¼ PID æŽ§åˆ¶å™¨
 * æ”¯æŒï¼šä½ç½®çŽ¯ + é€Ÿåº¦çŽ¯ + åèˆªè§’é€Ÿåº¦çŽ¯
 *
 * æ•´ä½“è¯´æ˜Ž
 * +------------------------------------------------------------------+
 * |                    å·®é€Ÿåº•ç›˜ PID æŽ§åˆ¶æž¶æž„                         |
 * +------------------------------------------------------------------+
 * |                                                                  |
 * |   å¤–çŽ¯(ä½ç½®çŽ¯)   -->   ä¸­çŽ¯(é€Ÿåº¦çŽ¯)   -->   å†…çŽ¯(åèˆªè§’é€Ÿåº¦çŽ¯)   -->   è¾“å‡ºPWM   |
 * |   target_pos          ref_speed          ref_yaw_rate           out_steer_pwm   |
 * |   (m)                 (m/s)              (rad/s)                è½¬å‘PWM         |
 * |                                                                  |
 * |   è¾“å…¥ï¼š              è¾“å…¥ï¼š             è¾“å…¥ï¼š                                |
 * |   ç´¯åŠ ä½ç½®æŒ‡ä»¤        é€Ÿåº¦ç›®æ ‡å€¼         åèˆªè§’é€Ÿåº¦æŒ‡ä»¤ / è½¬å‘æŒ‡ä»¤            |
 * |                                                                  |
 * |   å‚æ•°ï¼š              å‚æ•°ï¼š             å‚æ•°ï¼š                                |
 * |   position_param      speed_param        yaw_rate_param                       |
 * |                                                                  |
 * +------------------------------------------------------------------+
 * |   é€Ÿåº¦çŽ¯ç›´æŽ¥è¾“å‡ºå·¦å³è½® PWM                                        |
 * |   åèˆªè§’é€Ÿåº¦åå·® = ç›®æ ‡è½¬å¼¯è§’é€Ÿåº¦ - å®žé™…è½¬å¼¯è§’é€Ÿåº¦                 |
 * +------------------------------------------------------------------+
 *
 * ä½¿ç”¨æ–¹æ³•ï¼š
 *   1. wheel_pid_init()              - åˆå§‹åŒ–ï¼ˆmainä¸­è°ƒç”¨ä¸€æ¬¡ï¼‰
 *   2. wheel_pid_enable(...)         - ä½¿èƒ½å„æŽ§åˆ¶çŽ¯
 *   3. wheel_pid_set_speed_target()  - è®¾ç½®é€Ÿåº¦ç›®æ ‡ï¼ˆm/sï¼‰
 *   4. wheel_pid_set_yaw_target()    - è®¾ç½®åèˆªè§’é€Ÿåº¦ç›®æ ‡ï¼ˆrad/sï¼‰
 *   5. wheel_pid_update(enc, dt)     - å‘¨æœŸè°ƒç”¨ï¼ˆæŽ¨è4msï¼‰
 *
 * å…¼å®¹æ—§æŽ¥å£ï¼š
 *   - wheel_pid_enable(1,1,0,1)      ä½¿èƒ½å¯¹åº”çŽ¯
 *   - wheel_pid_set_target_speed(v)  è®¾ç½®ç›¸åŒå·¦å³è½®é€Ÿåº¦
 *   - wheel_pid_update(enc, dt)      å†…éƒ¨è‡ªåŠ¨ä¼°ç®—åèˆªè§’é€Ÿåº¦
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
 */

#ifndef CODE_CONTROL_PID_H_
#define CODE_CONTROL_PID_H_

<<<<<<< HEAD
//------------------------------------------- Í·ÎÄ¼þÒýÓÃ ------------------------------------------------------------
=======
//------------------------------------------- å¤´æ–‡ä»¶å¼•ç”¨ ------------------------------------------------------------
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6

#include "zf_common_headfile.h"
#include "vehicle_config.h"

<<<<<<< HEAD
//------------------------------------------- ºê¶¨Òå -----------------------------------------------------------------

/* PID ²ÎÊýÊý×éË÷Òý */
#define PID_PARAM_KP         0       /* ±ÈÀýÏµÊý */
#define PID_PARAM_KI         1       /* »ý·ÖÏµÊý */
#define PID_PARAM_KD         2       /* Î¢·ÖÏµÊý */
#define PID_PARAM_I_LIMIT    3       /* »ý·ÖÏÞ·ùÖµ */

/* PWM ºÍ²Î¿¼ÖµÏÞ·ù */
#define WHEEL_PID_PWM_ABS_MAX       2000.0f   /* PWM¾ø¶Ô×î´óÖµ£¨Ô­Ê¼·¶Î§£© */
#define SPEED_REF_ABS_MAX_MPS       2.5f      /* ËÙ¶È²Î¿¼ÏÞ·ù (m/s) */
#define YAW_RATE_REF_ABS_MAX_RPS    5.0f      /* Æ«º½½ÇËÙ¶È²Î¿¼ÏÞ·ù (rad/s) */
#define STEER_PWM_ABS_MAX           200.0f    /* ×ªÏòPWMÏÞ·ùÖµ */

#define WHEEL_PID_PWM_LIMIT_PERCENT 70.0f     /* PWMÊä³ö°Ù·Ö±ÈÏÞ·ù£¨Ä¬ÈÏ70%£© */

//------------------------------------------- ½á¹¹Ìå¶¨Òå -------------------------------------------------------------

/**
 * PID ÄÚ²¿×´Ì¬½á¹¹Ìå£¨Î»ÖÃÊ½PID£©
 */
typedef struct
{
    float iError;        /* µ±Ç°Îó²î */
    float LastError;     /* ÉÏÒ»´ÎÎó²î */
    float PrevError;     /* ÉÏÉÏÒ»´ÎÎó²î£¨ÓÃÓÚÎ¢·Ö£© */
    float SumError;      /* Îó²î»ý·ÖÖµ */
    float LastData;      /* ÉÏÒ»´ÎÊä³öÖµ */
} PID_INFO;

/**
 * ²îËÙµ×ÅÌ¼¶Áª PID ×Ü¿ØÖÆÆ÷½á¹¹Ìå
 */
typedef struct
{
    /*--------- PID ÄÚ²¿×´Ì¬ --------*/
    PID_INFO pid_pos_left;           /* ×óÂÖÎ»ÖÃ»· PID ×´Ì¬ */
    PID_INFO pid_pos_right;          /* ÓÒÂÖÎ»ÖÃ»· PID ×´Ì¬ */
    PID_INFO pid_spd_left;           /* ×óÂÖËÙ¶È»· PID ×´Ì¬ */
    PID_INFO pid_spd_right;          /* ÓÒÂÖËÙ¶È»· PID ×´Ì¬ */
    PID_INFO pid_yaw_rate;           /* Æ«º½½ÇËÙ¶È»· PID ×´Ì¬ */

    /*--------- PID ²ÎÊý --------*/
    float position_param[4];         /* Î»ÖÃ»· PID ²ÎÊý [Kp, Ki, Kd, I_limit] */
    float speed_param_left[4];       /* ×óÂÖËÙ¶È»· PID ²ÎÊý */
    float speed_param_right[4];      /* ÓÒÂÖËÙ¶È»· PID ²ÎÊý */
    float yaw_rate_param[4];         /* Æ«º½½ÇËÙ¶È»· PID ²ÎÊý */

    /*--------- Ä¿±êÓë²Î¿¼Öµ --------*/
    float cmd_speed_left_mps;        /* Ö¸ÁîËÙ¶È - ×óÂÖ (m/s) */
    float cmd_speed_right_mps;       /* Ö¸ÁîËÙ¶È - ÓÒÂÖ (m/s) */
    float target_pos_left_m;         /* Ä¿±êÎ»ÖÃ - ×óÂÖ (m) */
    float target_pos_right_m;        /* Ä¿±êÎ»ÖÃ - ÓÒÂÖ (m) */
    float ref_speed_left_mps;        /* ²Î¿¼ËÙ¶È - ×óÂÖ (m/s) */
    float ref_speed_right_mps;       /* ²Î¿¼ËÙ¶È - ÓÒÂÖ (m/s) */
    float cmd_yaw_rate_rps;          /* Ö¸ÁîÆ«º½½ÇËÙ¶È (rad/s) */
    float ref_yaw_rate_rps;          /* ²Î¿¼Æ«º½½ÇËÙ¶È (rad/s) */

    /*--------- PID Êä³ö --------*/
    float out_left_pwm;              /* ×óÂÖ PWM Êä³öÖµ */
    float out_right_pwm;             /* ÓÒÂÖ PWM Êä³öÖµ */
    float out_steer_pwm;             /* ×ªÏò PWM Êä³öÖµ */

    /*--------- Ê¹ÄÜ±êÖ¾ --------*/
    uint8 enable_output;             /* ×ÜÊä³öÊ¹ÄÜ */
    uint8 enable_speed_loop;         /* ËÙ¶È»·Ê¹ÄÜ */
    uint8 enable_position_loop;      /* Î»ÖÃ»·Ê¹ÄÜ */
    uint8 enable_yaw_rate_loop;      /* Æ«º½½ÇËÙ¶È»·Ê¹ÄÜ */
    uint8 initialized;               /* ³õÊ¼»¯Íê³É±êÖ¾ */
} WHEEL_PID_LAYER;


//------------------------------------------- Íâ²¿±äÁ¿ÉùÃ÷ -----------------------------------------------------------

extern WHEEL_PID_LAYER g_wheel_pid;
extern int16 origin_pidout;
extern int16 origin_pid_sum_error;
extern int16 origin_error_right;
extern int16 origin_error_left;


//------------------------------------------- º¯ÊýÉùÃ÷ ---------------------------------------------------------------

void wheel_pid_init(void);                                                    /* ³õÊ¼»¯ PID ¿ØÖÆÆ÷ */

void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,           /* Ê¹ÄÜ¿ØÖÆ»· */
                      uint8 enable_position_loop, uint8 enable_yaw_rate_loop);

void wheel_pid_set_speed_target(float left_mps, float right_mps);             /* ÉèÖÃ×óÓÒÂÖÄ¿±êËÙ¶È (m/s) */

void wheel_pid_set_target_speed(float target_speed_mps);                      /* ÉèÖÃÏàÍ¬Ä¿±êËÙ¶È£¨Ö±Ïß¿ì½Ý½Ó¿Ú£© */

void wheel_pid_set_position_target(float left_m, float right_m);              /* ÉèÖÃ×óÓÒÂÖÄ¿±êÎ»ÖÃ (m) */

void wheel_pid_set_yaw_target(float yaw_rate_rps);                            /* ÉèÖÃÄ¿±êÆ«º½½ÇËÙ¶È (rad/s) */

void wheel_pid_set_speed_param_left(float kp, float ki, float kd, float i_limit);   /* ÉèÖÃ×óÂÖËÙ¶È»·²ÎÊý */

void wheel_pid_set_speed_param_right(float kp, float ki, float kd, float i_limit);  /* ÉèÖÃÓÒÂÖËÙ¶È»·²ÎÊý */

void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit);        /* Í¬Ê±ÉèÖÃ×óÓÒÂÖËÙ¶È»·²ÎÊý */

void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit);     /* ÉèÖÃÎ»ÖÃ»·²ÎÊý */

void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit);     /* ÉèÖÃÆ«º½½ÇËÙ¶È»·²ÎÊý */

void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps);                    /* ÉèÖÃËÙ¶È²Î¿¼ÏÞ·ù (m/s) */

void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps);                 /* ÉèÖÃÆ«º½½ÇËÙ¶È²Î¿¼ÏÞ·ù (rad/s) */

void wheel_pid_update(const EncoderLayerState *enc, float dt_s);              /* Ö÷¸üÐÂº¯Êý£¨ÍÆ¼öµ÷ÓÃ£© */

#endif /* CODE_CONTROL_PID_H_ */
=======
//------------------------------------------- å®å®šä¹‰ -----------------------------------------------------------------

/* PID å‚æ•°æ•°ç»„ç´¢å¼• */
#define PID_PARAM_KP         0       /* æ¯”ä¾‹ç³»æ•° */
#define PID_PARAM_KI         1       /* ç§¯åˆ†ç³»æ•° */
#define PID_PARAM_KD         2       /* å¾®åˆ†ç³»æ•° */
#define PID_PARAM_I_LIMIT    3       /* ç§¯åˆ†é™å¹…å€¼ */

/* PWM å’Œå‚è€ƒå€¼é™å¹… */
#define WHEEL_PID_PWM_ABS_MAX       2000.0f   /* PWMç»å¯¹æœ€å¤§å€¼ï¼ˆåŽŸå§‹èŒƒå›´ï¼‰ */
#define SPEED_REF_ABS_MAX_MPS       2.5f      /* é€Ÿåº¦å‚è€ƒé™å¹… (m/s) */
#define YAW_RATE_REF_ABS_MAX_RPS    5.0f      /* åèˆªè§’é€Ÿåº¦å‚è€ƒé™å¹… (rad/s) */
#define STEER_PWM_ABS_MAX           200.0f    /* è½¬å‘PWMé™å¹…å€¼ */

#define WHEEL_PID_PWM_LIMIT_PERCENT 70.0f     /* PWMè¾“å‡ºç™¾åˆ†æ¯”é™å¹…ï¼ˆé»˜è®¤70%ï¼‰ */

//------------------------------------------- ç»“æž„ä½“å®šä¹‰ -------------------------------------------------------------

/**
 * PID å†…éƒ¨çŠ¶æ€ç»“æž„ä½“ï¼ˆä½ç½®å¼PIDï¼‰
 */
typedef struct
{
    float iError;        /* å½“å‰è¯¯å·® */
    float LastError;     /* ä¸Šä¸€æ¬¡è¯¯å·® */
    float PrevError;     /* ä¸Šä¸Šä¸€æ¬¡è¯¯å·®ï¼ˆç”¨äºŽå¾®åˆ†ï¼‰ */
    float SumError;      /* è¯¯å·®ç§¯åˆ†å€¼ */
    float LastData;      /* ä¸Šä¸€æ¬¡è¾“å‡ºå€¼ */
} PID_INFO;

/**
 * å·®é€Ÿåº•ç›˜çº§è” PID æ€»æŽ§åˆ¶å™¨ç»“æž„ä½“
 */
typedef struct
{
    /*--------- PID å†…éƒ¨çŠ¶æ€ --------*/
    PID_INFO pid_pos_left;           /* å·¦è½®ä½ç½®çŽ¯ PID çŠ¶æ€ */
    PID_INFO pid_pos_right;          /* å³è½®ä½ç½®çŽ¯ PID çŠ¶æ€ */
    PID_INFO pid_spd_left;           /* å·¦è½®é€Ÿåº¦çŽ¯ PID çŠ¶æ€ */
    PID_INFO pid_spd_right;          /* å³è½®é€Ÿåº¦çŽ¯ PID çŠ¶æ€ */
    PID_INFO pid_yaw_rate;           /* åèˆªè§’é€Ÿåº¦çŽ¯ PID çŠ¶æ€ */

    /*--------- PID å‚æ•° --------*/
    float position_param[4];         /* ä½ç½®çŽ¯ PID å‚æ•° [Kp, Ki, Kd, I_limit] */
    float speed_param_left[4];       /* å·¦è½®é€Ÿåº¦çŽ¯ PID å‚æ•° */
    float speed_param_right[4];      /* å³è½®é€Ÿåº¦çŽ¯ PID å‚æ•° */
    float yaw_rate_param[4];         /* åèˆªè§’é€Ÿåº¦çŽ¯ PID å‚æ•° */

    /*--------- ç›®æ ‡ä¸Žå‚è€ƒå€¼ --------*/
    float cmd_speed_left_mps;        /* æŒ‡ä»¤é€Ÿåº¦ - å·¦è½® (m/s) */
    float cmd_speed_right_mps;       /* æŒ‡ä»¤é€Ÿåº¦ - å³è½® (m/s) */
    float target_pos_left_m;         /* ç›®æ ‡ä½ç½® - å·¦è½® (m) */
    float target_pos_right_m;        /* ç›®æ ‡ä½ç½® - å³è½® (m) */
    float ref_speed_left_mps;        /* å‚è€ƒé€Ÿåº¦ - å·¦è½® (m/s) */
    float ref_speed_right_mps;       /* å‚è€ƒé€Ÿåº¦ - å³è½® (m/s) */
    float cmd_yaw_rate_rps;          /* æŒ‡ä»¤åèˆªè§’é€Ÿåº¦ (rad/s) */
    float ref_yaw_rate_rps;          /* å‚è€ƒåèˆªè§’é€Ÿåº¦ (rad/s) */

    /*--------- PID è¾“å‡º --------*/
    float out_left_pwm;              /* å·¦è½® PWM è¾“å‡ºå€¼ */
    float out_right_pwm;             /* å³è½® PWM è¾“å‡ºå€¼ */
    float out_steer_pwm;             /* è½¬å‘ PWM è¾“å‡ºå€¼ */

    /*--------- ä½¿èƒ½æ ‡å¿— --------*/
    uint8 enable_output;             /* æ€»è¾“å‡ºä½¿èƒ½ */
    uint8 enable_speed_loop;         /* é€Ÿåº¦çŽ¯ä½¿èƒ½ */
    uint8 enable_position_loop;      /* ä½ç½®çŽ¯ä½¿èƒ½ */
    uint8 enable_yaw_rate_loop;      /* åèˆªè§’é€Ÿåº¦çŽ¯ä½¿èƒ½ */
    uint8 initialized;               /* åˆå§‹åŒ–å®Œæˆæ ‡å¿— */
} WHEEL_PID_LAYER;


//------------------------------------------- å¤–éƒ¨å˜é‡å£°æ˜Ž -----------------------------------------------------------

extern WHEEL_PID_LAYER g_wheel_pid;


//------------------------------------------- å‡½æ•°å£°æ˜Ž ---------------------------------------------------------------

void wheel_pid_init(void);                                                    /* åˆå§‹åŒ– PID æŽ§åˆ¶å™¨ */

void wheel_pid_enable(uint8 enable_output, uint8 enable_speed_loop,           /* ä½¿èƒ½æŽ§åˆ¶çŽ¯ */
                      uint8 enable_position_loop, uint8 enable_yaw_rate_loop);

void wheel_pid_set_speed_target(float left_mps, float right_mps);             /* è®¾ç½®å·¦å³è½®ç›®æ ‡é€Ÿåº¦ (m/s) */

void wheel_pid_set_target_speed(float target_speed_mps);                      /* è®¾ç½®ç›¸åŒç›®æ ‡é€Ÿåº¦ï¼ˆç›´çº¿å¿«æ·æŽ¥å£ï¼‰ */

void wheel_pid_set_position_target(float left_m, float right_m);              /* è®¾ç½®å·¦å³è½®ç›®æ ‡ä½ç½® (m) */

void wheel_pid_set_yaw_target(float yaw_rate_rps);                            /* è®¾ç½®ç›®æ ‡åèˆªè§’é€Ÿåº¦ (rad/s) */

void wheel_pid_set_speed_param_left(float kp, float ki, float kd, float i_limit);   /* è®¾ç½®å·¦è½®é€Ÿåº¦çŽ¯å‚æ•° */

void wheel_pid_set_speed_param_right(float kp, float ki, float kd, float i_limit);  /* è®¾ç½®å³è½®é€Ÿåº¦çŽ¯å‚æ•° */

void wheel_pid_set_speed_param(float kp, float ki, float kd, float i_limit);        /* åŒæ—¶è®¾ç½®å·¦å³è½®é€Ÿåº¦çŽ¯å‚æ•° */

void wheel_pid_set_position_param(float kp, float ki, float kd, float i_limit);     /* è®¾ç½®ä½ç½®çŽ¯å‚æ•° */

void wheel_pid_set_yaw_rate_param(float kp, float ki, float kd, float i_limit);     /* è®¾ç½®åèˆªè§’é€Ÿåº¦çŽ¯å‚æ•° */

void wheel_pid_set_speed_ref_limit_mps(float abs_max_mps);                    /* è®¾ç½®é€Ÿåº¦å‚è€ƒé™å¹… (m/s) */

void wheel_pid_set_yaw_rate_ref_limit_rps(float abs_max_rps);                 /* è®¾ç½®åèˆªè§’é€Ÿåº¦å‚è€ƒé™å¹… (rad/s) */

void wheel_pid_update(const EncoderLayerState *enc, float dt_s);              /* ä¸»æ›´æ–°å‡½æ•°ï¼ˆæŽ¨èè°ƒç”¨ï¼‰ */

#endif /* CODE_CONTROL_PID_H_ */
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
