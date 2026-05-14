/*
 * PID_cmd.c
 *
<<<<<<< HEAD
 * PID ÃüÁîÐÐµ÷ÊÔÏµÍ³
 * Ö§³ÖÊµÊ±²ÎÊýÐÞ¸Ä¡¢Ä¿±êÉèÖÃ¡¢Ò£²â¡¢×Ô¶¯²âÊÔµÈ¹¦ÄÜ
=======
 * PID å‘½ä»¤è¡Œè°ƒè¯•ç³»ç»Ÿ
 * æ”¯æŒå®žæ—¶å‚æ•°ä¿®æ”¹ã€ç›®æ ‡è®¾ç½®ã€é¥æµ‹ã€è‡ªåŠ¨æµ‹è¯•ç­‰åŠŸèƒ½
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
 *
 * Created on: 2026-04-27
 *     Author: Auto-Generated
 */

#include "zf_common_headfile.h"

<<<<<<< HEAD
//------------------------------------------- È«¾Ö±äÁ¿ --------------------------------------------------------------

/* PID ÃüÁîÏµÍ³È«¾Ö×´Ì¬ */
PID_CMD_STATE g_pid_cmd;

/* ±£´æÖ®Ç°µÄÒ£²âÉèÖÃ£¬ÓÃÓÚ²âÊÔ½áÊøºó»Ö¸´ */
static uint16 s_saved_telemetry_rate_ms = 50;
static uint8  s_saved_telemetry_enabled = 0u;

//------------------------------------------- ÄÚ²¿º¯ÊýÉùÃ÷ ----------------------------------------------------------
=======
//------------------------------------------- å…¨å±€å˜é‡ --------------------------------------------------------------

/* PID å‘½ä»¤ç³»ç»Ÿå…¨å±€çŠ¶æ€ */
PID_CMD_STATE g_pid_cmd;

/* ä¿å­˜ä¹‹å‰çš„é¥æµ‹è®¾ç½®ï¼Œç”¨äºŽæµ‹è¯•ç»“æŸåŽæ¢å¤ */
static uint16 s_saved_telemetry_rate_ms = 50;
static uint8  s_saved_telemetry_enabled = 0u;

//------------------------------------------- å†…éƒ¨å‡½æ•°å£°æ˜Ž ----------------------------------------------------------
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6

static void cmd_send_help(void);
static void cmd_send_status(void);
static void cmd_set_speed_param(int argc, char *argv[]);
static void cmd_set_position_param(int argc, char *argv[]);
static void cmd_set_yaw_param(int argc, char *argv[]);
static void cmd_set_speed_target(int argc, char *argv[]);
static void cmd_set_yaw_target(int argc, char *argv[]);
static void cmd_set_telemetry_rate(int argc, char *argv[]);
static void cmd_start_step_test(int argc, char *argv[]);
static void cmd_stop_test(void);
static void cmd_enable_loops(int argc, char *argv[]);

static float parse_float(const char *str, float default_val);
static int   parse_int(const char *str, int default_val);
static int   strcmp_nocase(const char *s1, const char *s2);

<<<<<<< HEAD
//------------------------------------------- ¹¤¾ßº¯Êý --------------------------------------------------------------

// Í¨¹ýÎÞÏß´®¿Ú·¢ËÍ×Ö·û´®£¨×Ô¶¯×·¼Ó \r\n£©
=======
//------------------------------------------- å·¥å…·å‡½æ•° --------------------------------------------------------------

// é€šè¿‡æ— çº¿ä¸²å£å‘é€å­—ç¬¦ä¸²ï¼ˆè‡ªåŠ¨è¿½åŠ  \r\nï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_cmd_send_string(const char *str)
{
    if (str == NULL) return;

    wireless_uart_send_string(str);
    wireless_uart_send_string("\r\n");
}

<<<<<<< HEAD
// °²È«×Ö·û´®×ª float
=======
// å®‰å…¨å­—ç¬¦ä¸²è½¬ float
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
static float parse_float(const char *str, float default_val)
{
    if (str == NULL || str[0] == '\0') return default_val;

    float result = default_val;
    int sign = 1;
    int i = 0;

<<<<<<< HEAD
    // Ìø¹ýÇ°µ¼¿Õ¸ñ
    while (str[i] == ' ') i++;

    // ´¦Àí·ûºÅ
    if (str[i] == '-') { sign = -1; i++; }
    else if (str[i] == '+') { i++; }

    // ÕûÊý²¿·Ö
=======
    // è·³è¿‡å‰å¯¼ç©ºæ ¼
    while (str[i] == ' ') i++;

    // å¤„ç†ç¬¦å·
    if (str[i] == '-') { sign = -1; i++; }
    else if (str[i] == '+') { i++; }

    // æ•´æ•°éƒ¨åˆ†
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    result = 0.0f;
    while (str[i] >= '0' && str[i] <= '9')
    {
        result = result * 10.0f + (str[i] - '0');
        i++;
    }

<<<<<<< HEAD
    // Ð¡Êý²¿·Ö
=======
    // å°æ•°éƒ¨åˆ†
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (str[i] == '.')
    {
        i++;
        float frac = 0.0f;
        float div = 10.0f;
        while (str[i] >= '0' && str[i] <= '9')
        {
            frac += (str[i] - '0') / div;
            div *= 10.0f;
            i++;
        }
        result += frac;
    }

    return result * sign;
}

<<<<<<< HEAD
// °²È«×Ö·û´®×ª int
=======
// å®‰å…¨å­—ç¬¦ä¸²è½¬ int
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
static int parse_int(const char *str, int default_val)
{
    if (str == NULL || str[0] == '\0') return default_val;

    int result = 0;
    int sign = 1;
    int i = 0;

    while (str[i] == ' ') i++;

    if (str[i] == '-') { sign = -1; i++; }
    else if (str[i] == '+') { i++; }

    while (str[i] >= '0' && str[i] <= '9')
    {
        result = result * 10 + (str[i] - '0');
        i++;
    }

    return result * sign;
}

<<<<<<< HEAD
// ²»Çø·Ö´óÐ¡Ð´×Ö·û´®±È½Ï
=======
// ä¸åŒºåˆ†å¤§å°å†™å­—ç¬¦ä¸²æ¯”è¾ƒ
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
static int strcmp_nocase(const char *s1, const char *s2)
{
    if (s1 == NULL || s2 == NULL) return -1;

    while (*s1 && *s2)
    {
        char c1 = *s1;
        char c2 = *s2;

        if (c1 >= 'A' && c1 <= 'Z') c1 += 32;
        if (c2 >= 'A' && c2 <= 'Z') c2 += 32;

        if (c1 != c2) return c1 - c2;

        s1++;
        s2++;
    }

    return *s1 - *s2;
}

<<<<<<< HEAD
//------------------------------------------- ³õÊ¼»¯º¯Êý -----------------------------------------------------------

// PID ÃüÁîÏµÍ³³õÊ¼»¯£¨ÔÚ wheel_pid_init() ÖÐ±»µ÷ÓÃ£©
=======
//------------------------------------------- åˆå§‹åŒ–å‡½æ•° -----------------------------------------------------------

// PID å‘½ä»¤ç³»ç»Ÿåˆå§‹åŒ–ï¼ˆåœ¨ wheel_pid_init() ä¸­è¢«è°ƒç”¨ï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_cmd_init(void)
{
    memset(&g_pid_cmd, 0, sizeof(g_pid_cmd));

    g_pid_cmd.telemetry_rate_ms = PID_CMD_TELEMETRY_DEFAULT_MS;
<<<<<<< HEAD
    g_pid_cmd.telemetry_enabled = 0u;           // Ä¬ÈÏ¹Ø±ÕÒ£²â
=======
    g_pid_cmd.telemetry_enabled = 0u;           // é»˜è®¤å…³é—­é¥æµ‹
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_pid_cmd.test_mode         = PID_TEST_MODE_NONE;
    g_pid_cmd.cmd_ready         = 0u;
    g_pid_cmd.cmd_index         = 0u;
    g_pid_cmd.timestamp_ms      = 0u;
}

<<<<<<< HEAD
//------------------------------------------- ÃüÁî½ÓÊÕÓë´¦Àí -------------------------------------------------------

// ´¦Àí´®¿Ú½ÓÊÕµ½µÄµ¥¸ö×Ö½Ú£¨ÔÚ´®¿ÚÖÐ¶Ï»ò²éÑ¯ÖÐµ÷ÓÃ£©
=======
//------------------------------------------- å‘½ä»¤æŽ¥æ”¶ä¸Žå¤„ç† -------------------------------------------------------

// å¤„ç†ä¸²å£æŽ¥æ”¶åˆ°çš„å•ä¸ªå­—èŠ‚ï¼ˆåœ¨ä¸²å£ä¸­æ–­æˆ–æŸ¥è¯¢ä¸­è°ƒç”¨ï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_cmd_process_byte(uint8 byte)
{
    if (byte == 0) return;

<<<<<<< HEAD
    // ¼ì²âÃüÁî½áÊø·û
=======
    // æ£€æµ‹å‘½ä»¤ç»“æŸç¬¦
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (byte == '\r' || byte == '\n')
    {
        if (g_pid_cmd.cmd_index > 0)
        {
            g_pid_cmd.cmd_buffer[g_pid_cmd.cmd_index] = '\0';
            g_pid_cmd.cmd_ready = 1u;
        }
        return;
    }

<<<<<<< HEAD
    // ·ÀÖ¹»º³åÇøÒç³ö
=======
    // é˜²æ­¢ç¼“å†²åŒºæº¢å‡º
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (g_pid_cmd.cmd_index >= PID_CMD_BUFFER_SIZE - 1)
    {
        g_pid_cmd.cmd_index = 0u;
        return;
    }

    g_pid_cmd.cmd_buffer[g_pid_cmd.cmd_index++] = (char)byte;
}

<<<<<<< HEAD
// ÃüÁîÂÖÑ¯´¦Àí£¨½¨ÒéÔÚ 4ms ¶¨Ê±ÈÎÎñÖÐµ÷ÓÃ£©
void wheel_pid_cmd_poll(void)
{
    g_pid_cmd.timestamp_ms += 4u;   // Ã¿µ÷ÓÃÒ»´ÎÀÛ¼Ó4ms

    // Ö´ÐÐ´ý´¦ÀíµÄÃüÁî
=======
// å‘½ä»¤è½®è¯¢å¤„ç†ï¼ˆå»ºè®®åœ¨ 4ms å®šæ—¶ä»»åŠ¡ä¸­è°ƒç”¨ï¼‰
void wheel_pid_cmd_poll(void)
{
    g_pid_cmd.timestamp_ms += 4u;   // æ¯è°ƒç”¨ä¸€æ¬¡ç´¯åŠ 4ms

    // æ‰§è¡Œå¾…å¤„ç†çš„å‘½ä»¤
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (g_pid_cmd.cmd_ready)
    {
        wheel_pid_cmd_execute(g_pid_cmd.cmd_buffer);
        g_pid_cmd.cmd_ready = 0u;
        g_pid_cmd.cmd_index = 0u;
    }

<<<<<<< HEAD
    // Ò£²â·¢ËÍ
=======
    // é¥æµ‹å‘é€
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (g_pid_cmd.telemetry_enabled)
    {
        g_pid_cmd.telemetry_counter += 4u;
        if (g_pid_cmd.telemetry_counter >= g_pid_cmd.telemetry_rate_ms)
        {
            g_pid_cmd.telemetry_counter = 0u;
            wheel_pid_cmd_send_telemetry();
        }
    }

<<<<<<< HEAD
    // ×Ô¶¯²âÊÔ³¬Ê±´¦Àí
=======
    // è‡ªåŠ¨æµ‹è¯•è¶…æ—¶å¤„ç†
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (g_pid_cmd.test_mode == PID_TEST_MODE_STEP)
    {
        uint32 elapsed = g_pid_cmd.timestamp_ms - g_pid_cmd.test_start_time_ms;
        if (elapsed >= g_pid_cmd.test_duration_ms)
        {
            wheel_pid_cmd_stop_test();
        }
    }
}

<<<<<<< HEAD
// Ö´ÐÐÍêÕûÃüÁî×Ö·û´®
=======
// æ‰§è¡Œå®Œæ•´å‘½ä»¤å­—ç¬¦ä¸²
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_cmd_execute(const char *cmd)
{
    if (cmd == NULL || cmd[0] == '\0') return;

    char *argv[8];
    int argc = 0;
    char buffer[PID_CMD_BUFFER_SIZE];

<<<<<<< HEAD
    // ¸´ÖÆÃüÁîµ½»º³åÇø
=======
    // å¤åˆ¶å‘½ä»¤åˆ°ç¼“å†²åŒº
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    uint16 i = 0;
    while (cmd[i] && i < PID_CMD_BUFFER_SIZE - 1)
    {
        buffer[i] = cmd[i];
        i++;
    }
    buffer[i] = '\0';

<<<<<<< HEAD
    // ·Ö¸î²ÎÊý
=======
    // åˆ†å‰²å‚æ•°
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    argv[argc] = buffer;
    for (i = 0; buffer[i] && argc < 8; i++)
    {
        if (buffer[i] == ' ' || buffer[i] == '\t')
        {
            buffer[i] = '\0';
            if (buffer[i + 1] != '\0')
            {
                argv[++argc] = &buffer[i + 1];
            }
        }
    }
    argc++;

<<<<<<< HEAD
    // ÃüÁîÆ¥ÅäÓë·Ö·¢
=======
    // å‘½ä»¤åŒ¹é…ä¸Žåˆ†å‘
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    if (strcmp_nocase(argv[0], "HELP") == 0)
        cmd_send_help();
    else if (strcmp_nocase(argv[0], "GET_STATUS") == 0)
        cmd_send_status();
    else if (strcmp_nocase(argv[0], "SET_SPEED_PARAM") == 0)
        cmd_set_speed_param(argc, argv);
    else if (strcmp_nocase(argv[0], "SET_POSITION_PARAM") == 0)
        cmd_set_position_param(argc, argv);
    else if (strcmp_nocase(argv[0], "SET_YAW_PARAM") == 0)
        cmd_set_yaw_param(argc, argv);
    else if (strcmp_nocase(argv[0], "SET_SPEED_TARGET") == 0)
        cmd_set_speed_target(argc, argv);
    else if (strcmp_nocase(argv[0], "SET_YAW_TARGET") == 0)
        cmd_set_yaw_target(argc, argv);
    else if (strcmp_nocase(argv[0], "SET_TELEMETRY_RATE") == 0)
        cmd_set_telemetry_rate(argc, argv);
    else if (strcmp_nocase(argv[0], "START_STEP_TEST") == 0)
        cmd_start_step_test(argc, argv);
    else if (strcmp_nocase(argv[0], "STOP_TEST") == 0)
        cmd_stop_test();
    else if (strcmp_nocase(argv[0], "ENABLE_LOOPS") == 0)
        cmd_enable_loops(argc, argv);
    else
        wheel_pid_cmd_send_string("ERR:Unknown command. Send HELP for list.");
}

<<<<<<< HEAD
//------------------------------------------- ÃüÁî´¦Àíº¯Êý ---------------------------------------------------------
=======
//------------------------------------------- å‘½ä»¤å¤„ç†å‡½æ•° ---------------------------------------------------------
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6

static void cmd_send_help(void)
{
    wheel_pid_cmd_send_string("=== PID Command Help ===");
    wheel_pid_cmd_send_string("SET_SPEED_PARAM kp ki kd i_limit");
    wheel_pid_cmd_send_string("SET_POSITION_PARAM kp ki kd i_limit");
    wheel_pid_cmd_send_string("SET_YAW_PARAM kp ki kd i_limit");
    wheel_pid_cmd_send_string("SET_SPEED_TARGET left_mps right_mps");
    wheel_pid_cmd_send_string("SET_YAW_TARGET yaw_rps");
    wheel_pid_cmd_send_string("SET_TELEMETRY_RATE ms (0=off)");
    wheel_pid_cmd_send_string("START_STEP_TEST speed_mps duration_ms");
    wheel_pid_cmd_send_string("STOP_TEST");
    wheel_pid_cmd_send_string("ENABLE_LOOPS out spd pos yaw");
    wheel_pid_cmd_send_string("GET_STATUS");
    wheel_pid_cmd_send_string("HELP");
}

static void cmd_send_status(void)
{
    char buf[128];
    wheel_pid_cmd_send_string("=== PID Status ===");

    sprintf(buf, "SPEED_PARAM_L: Kp=%.2f Ki=%.2f Kd=%.2f I_limit=%.2f",
            g_wheel_pid.speed_param_left[0], g_wheel_pid.speed_param_left[1],
            g_wheel_pid.speed_param_left[2], g_wheel_pid.speed_param_left[3]);
    wheel_pid_cmd_send_string(buf);

    sprintf(buf, "SPEED_PARAM_R: Kp=%.2f Ki=%.2f Kd=%.2f I_limit=%.2f",
            g_wheel_pid.speed_param_right[0], g_wheel_pid.speed_param_right[1],
            g_wheel_pid.speed_param_right[2], g_wheel_pid.speed_param_right[3]);
    wheel_pid_cmd_send_string(buf);

    sprintf(buf, "POSITION_PARAM: Kp=%.2f Ki=%.2f Kd=%.2f I_limit=%.2f",
            g_wheel_pid.position_param[0], g_wheel_pid.position_param[1],
            g_wheel_pid.position_param[2], g_wheel_pid.position_param[3]);
    wheel_pid_cmd_send_string(buf);

    sprintf(buf, "YAW_PARAM: Kp=%.2f Ki=%.2f Kd=%.2f I_limit=%.2f",
            g_wheel_pid.yaw_rate_param[0], g_wheel_pid.yaw_rate_param[1],
            g_wheel_pid.yaw_rate_param[2], g_wheel_pid.yaw_rate_param[3]);
    wheel_pid_cmd_send_string(buf);

    sprintf(buf, "ENABLE: out=%d speed=%d pos=%d yaw=%d",
            g_wheel_pid.enable_output, g_wheel_pid.enable_speed_loop,
            g_wheel_pid.enable_position_loop, g_wheel_pid.enable_yaw_rate_loop);
    wheel_pid_cmd_send_string(buf);

    sprintf(buf, "TARGET: speed_L=%.2f speed_R=%.2f yaw=%.2f",
            g_wheel_pid.cmd_speed_left_mps, g_wheel_pid.cmd_speed_right_mps,
            g_wheel_pid.cmd_yaw_rate_rps);
    wheel_pid_cmd_send_string(buf);

    sprintf(buf, "TELEMETRY: rate=%dms enabled=%d",
            g_pid_cmd.telemetry_rate_ms, g_pid_cmd.telemetry_enabled);
    wheel_pid_cmd_send_string(buf);

    sprintf(buf, "TEST_MODE: %d", g_pid_cmd.test_mode);
    wheel_pid_cmd_send_string(buf);
}

<<<<<<< HEAD
// ...£¨ºóÐøµÄ cmd_set_xxx º¯ÊýÓëÔ­À´Âß¼­ÍêÈ«Ò»ÖÂ£¬½ö×¢ÊÍ¸ÄÎªÖÐÎÄ£©
=======
// ...ï¼ˆåŽç»­çš„ cmd_set_xxx å‡½æ•°ä¸ŽåŽŸæ¥é€»è¾‘å®Œå…¨ä¸€è‡´ï¼Œä»…æ³¨é‡Šæ”¹ä¸ºä¸­æ–‡ï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6

static void cmd_set_speed_param(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_SPEED_PARAM kp ki kd i_limit");
        return;
    }

    float kp = parse_float(argv[1], 0.0f);
    float ki = parse_float(argv[2], 0.0f);
    float kd = parse_float(argv[3], 0.0f);
    float i_limit = parse_float(argv[4], 100.0f);

    wheel_pid_set_speed_param(kp, ki, kd, i_limit);

    char buf[64];
    sprintf(buf, "OK:SPEED_PARAM kp=%.2f ki=%.2f kd=%.2f i_limit=%.2f", kp, ki, kd, i_limit);
    wheel_pid_cmd_send_string(buf);
}

static void cmd_set_position_param(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_POSITION_PARAM kp ki kd i_limit");
        return;
    }

    float kp = parse_float(argv[1], 0.0f);
    float ki = parse_float(argv[2], 0.0f);
    float kd = parse_float(argv[3], 0.0f);
    float i_limit = parse_float(argv[4], 2.5f);

    wheel_pid_set_position_param(kp, ki, kd, i_limit);

    char buf[64];
    sprintf(buf, "OK:POSITION_PARAM kp=%.2f ki=%.2f kd=%.2f i_limit=%.2f", kp, ki, kd, i_limit);
    wheel_pid_cmd_send_string(buf);
}

static void cmd_set_yaw_param(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_YAW_PARAM kp ki kd i_limit");
        return;
    }

    float kp = parse_float(argv[1], 0.0f);
    float ki = parse_float(argv[2], 0.0f);
    float kd = parse_float(argv[3], 0.0f);
    float i_limit = parse_float(argv[4], 100.0f);

    wheel_pid_set_yaw_rate_param(kp, ki, kd, i_limit);

    char buf[64];
    sprintf(buf, "OK:YAW_PARAM kp=%.2f ki=%.2f kd=%.2f i_limit=%.2f", kp, ki, kd, i_limit);
    wheel_pid_cmd_send_string(buf);
}

static void cmd_set_speed_target(int argc, char *argv[])
{
    if (argc < 3)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_SPEED_TARGET left_mps right_mps");
        return;
    }

    float left  = parse_float(argv[1], 0.0f);
    float right = parse_float(argv[2], 0.0f);

    wheel_pid_set_speed_target(left, right);

    char buf[64];
    sprintf(buf, "OK:SPEED_TARGET L=%.2f R=%.2f", left, right);
    wheel_pid_cmd_send_string(buf);
}

static void cmd_set_yaw_target(int argc, char *argv[])
{
    if (argc < 2)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_YAW_TARGET yaw_rps");
        return;
    }

    float yaw = parse_float(argv[1], 0.0f);

    wheel_pid_set_yaw_target(yaw);

    char buf[64];
    sprintf(buf, "OK:YAW_TARGET %.2f rad/s", yaw);
    wheel_pid_cmd_send_string(buf);
}

static void cmd_set_telemetry_rate(int argc, char *argv[])
{
    if (argc < 2)
    {
        wheel_pid_cmd_send_string("ERR:Usage: SET_TELEMETRY_RATE ms (0=off)");
        return;
    }

    int rate_ms = parse_int(argv[1], 50);

    if (rate_ms <= 0)
    {
        g_pid_cmd.telemetry_enabled = 0u;
        g_pid_cmd.telemetry_rate_ms = 0;
        wheel_pid_cmd_send_string("OK:TELEMETRY OFF");
    }
    else
    {
        g_pid_cmd.telemetry_rate_ms = (uint16)rate_ms;
        g_pid_cmd.telemetry_enabled = 1u;
        g_pid_cmd.telemetry_counter = 0u;

        char buf[32];
        sprintf(buf, "OK:TELEMETRY_RATE %dms", rate_ms);
        wheel_pid_cmd_send_string(buf);
    }
}

static void cmd_start_step_test(int argc, char *argv[])
{
    if (argc < 3)
    {
        wheel_pid_cmd_send_string("ERR:Usage: START_STEP_TEST speed_mps duration_ms");
        return;
    }

    float speed = parse_float(argv[1], 1.0f);
    int duration = parse_int(argv[2], 3000);

    wheel_pid_cmd_start_step_test(speed, (uint32)duration);

    char buf[64];
    sprintf(buf, "OK:STEP_TEST speed=%.2f duration=%dms", speed, duration);
    wheel_pid_cmd_send_string(buf);
}

static void cmd_stop_test(void)
{
    wheel_pid_cmd_stop_test();
    wheel_pid_cmd_send_string("OK:TEST_STOPPED");
}

static void cmd_enable_loops(int argc, char *argv[])
{
    if (argc < 5)
    {
        wheel_pid_cmd_send_string("ERR:Usage: ENABLE_LOOPS out spd pos yaw");
        return;
    }

    int out = parse_int(argv[1], 0);
    int spd = parse_int(argv[2], 0);
    int pos = parse_int(argv[3], 0);
    int yaw = parse_int(argv[4], 0);

    wheel_pid_enable((uint8)out, (uint8)spd, (uint8)pos, (uint8)yaw);

    char buf[64];
    sprintf(buf, "OK:ENABLE out=%d spd=%d pos=%d yaw=%d", out, spd, pos, yaw);
    wheel_pid_cmd_send_string(buf);
}

<<<<<<< HEAD
//------------------------------------------- Ò£²âÓë²âÊÔº¯Êý ---------------------------------------------------------

// ·¢ËÍÒ£²âÊý¾Ý£¨CSV¸ñÊ½£©
=======
//------------------------------------------- é¥æµ‹ä¸Žæµ‹è¯•å‡½æ•° ---------------------------------------------------------

// å‘é€é¥æµ‹æ•°æ®ï¼ˆCSVæ ¼å¼ï¼‰
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_cmd_send_telemetry(void)
{
    const EncoderLayerState *enc = encoder_layer_get_state();
    if (enc == NULL) return;

    char buf[PID_CMD_TELEMETRY_BUF_SIZE];

    float err_spd_l = g_wheel_pid.cmd_speed_left_mps - enc->speed_left_mps;
    float err_spd_r = g_wheel_pid.cmd_speed_right_mps - enc->speed_right_mps;
    float err_yaw   = g_wheel_pid.cmd_yaw_rate_rps - g_wheel_pid.ref_yaw_rate_rps;

    sprintf(buf,
            "TELEMETRY,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f",
            (unsigned long)g_pid_cmd.timestamp_ms,
            g_wheel_pid.cmd_speed_left_mps, g_wheel_pid.cmd_speed_right_mps,
            enc->speed_left_mps, enc->speed_right_mps,
            g_wheel_pid.cmd_yaw_rate_rps, g_wheel_pid.ref_yaw_rate_rps,
            g_wheel_pid.out_left_pwm, g_wheel_pid.out_right_pwm, g_wheel_pid.out_steer_pwm,
            err_spd_l, err_spd_r, err_yaw,
            enc->odom_left_m, enc->odom_right_m);

    wheel_pid_cmd_send_string(buf);
}

<<<<<<< HEAD
// ¿ªÊ¼½×Ô¾ÏìÓ¦²âÊÔ
void wheel_pid_cmd_start_step_test(float speed, uint32 duration_ms)
{
    // ±£´æµ±Ç°Ò£²âÉèÖÃ
=======
// å¼€å§‹é˜¶è·ƒå“åº”æµ‹è¯•
void wheel_pid_cmd_start_step_test(float speed, uint32 duration_ms)
{
    // ä¿å­˜å½“å‰é¥æµ‹è®¾ç½®
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    s_saved_telemetry_rate_ms = g_pid_cmd.telemetry_rate_ms;
    s_saved_telemetry_enabled = g_pid_cmd.telemetry_enabled;

    encoder_layer_clear_odom();

    wheel_pid_set_speed_target(speed, speed);
<<<<<<< HEAD
    wheel_pid_enable(1, 1, 0, 0);        // ½ö¿ªÆôËÙ¶È»·

    // Ìá¸ßÒ£²âÆµÂÊ
=======
    wheel_pid_enable(1, 1, 0, 0);        // ä»…å¼€å¯é€Ÿåº¦çŽ¯

    // æé«˜é¥æµ‹é¢‘çŽ‡
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_pid_cmd.telemetry_enabled = 1u;
    g_pid_cmd.telemetry_rate_ms = 20;
    g_pid_cmd.telemetry_counter = 0u;

    g_pid_cmd.test_mode = PID_TEST_MODE_STEP;
    g_pid_cmd.test_start_time_ms = g_pid_cmd.timestamp_ms;
    g_pid_cmd.test_duration_ms = duration_ms;
    g_pid_cmd.test_target_speed = speed;

    wheel_pid_cmd_send_string("TEST_STEP_START");
}

<<<<<<< HEAD
// Í£Ö¹µ±Ç°²âÊÔ
=======
// åœæ­¢å½“å‰æµ‹è¯•
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
void wheel_pid_cmd_stop_test(void)
{
    wheel_pid_set_speed_target(0.0f, 0.0f);

<<<<<<< HEAD
    // »Ö¸´Ö®Ç°Ò£²âÉèÖÃ
=======
    // æ¢å¤ä¹‹å‰é¥æµ‹è®¾ç½®
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
    g_pid_cmd.telemetry_rate_ms = s_saved_telemetry_rate_ms;
    g_pid_cmd.telemetry_enabled = s_saved_telemetry_enabled;
    g_pid_cmd.test_mode = PID_TEST_MODE_NONE;

    wheel_pid_cmd_send_string("TEST_STEP_END");
<<<<<<< HEAD
}
=======
}
>>>>>>> d71cca4cef4b2eec4917dccbf69700830f5cd7e6
