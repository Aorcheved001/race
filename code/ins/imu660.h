/*
<<<<<<< HEAD
 * imu660.h
 * IMUæ•°æ®å¤„ç†æ¨¡å—
 *
 * Created on: 2024å¹´6æœˆ6æ—¥
 * Author: LateRain
 * Modified: 2025å¹´11æœˆ22æ—¥
 */

#ifndef CODE_BALANCE_IMU660_H_
#define CODE_BALANCE_IMU660_H_

 //-------------------------------------------å¤´æ–‡ä»¶åŒº------------------------------------------------------------
#include "zf_common_headfile.h"

 //-------------------------------------------å®å®šä¹‰åŒº------------------------------------------------------------
#define M_PI 3.1415                                                        // åœ†å‘¨çŽ‡

 //-------------------------------------------ç»“æž„ä½“å®šä¹‰åŒº------------------------------------------------------------
typedef struct
{
    float gyro_x;                                                          // Xè½´è§’é€Ÿåº¦
    float gyro_y;                                                          // Yè½´è§’é€Ÿåº¦
    float gyro_z;                                                          // Zè½´è§’é€Ÿåº¦
    float acc_x;                                                           // Xè½´åŠ é€Ÿåº¦
    float acc_y;                                                           // Yè½´åŠ é€Ÿåº¦
    float acc_z;                                                           // Zè½´åŠ é€Ÿåº¦
    float gyro_x_bias;                                                     // Xè½´è§’é€Ÿåº¦é›¶å
    float gyro_y_bias;                                                     // Yè½´è§’é€Ÿåº¦é›¶å
    float gyro_z_bias;                                                     // Zè½´è§’é€Ÿåº¦é›¶å
    float acc_x_bias;                                                      // Xè½´åŠ é€Ÿåº¦é›¶å
    float acc_y_bias;                                                      // Yè½´åŠ é€Ÿåº¦é›¶å
    float acc_z_bias;                                                      // Zè½´åŠ é€Ÿåº¦é›¶å
    float mag_x_bias;                                                      // Xè½´ç£åŠ›è®¡é›¶å
    float mag_y_bias;                                                      // Yè½´ç£åŠ›è®¡é›¶å
    float mag_z_bias;                                                      // Zè½´ç£åŠ›è®¡é›¶å
    float mag_x_scale;                                                     // Xè½´ç£åŠ›è®¡ç¼©æ”¾
    float mag_y_scale;                                                     // Yè½´ç£åŠ›è®¡ç¼©æ”¾
    float mag_z_scale;                                                     // Zè½´ç£åŠ›è®¡ç¼©æ”¾
    float mag_x;                                                           // Xè½´ç£åŠ›è®¡
    float mag_y;                                                           // Yè½´ç£åŠ›è®¡
    float mag_z;                                                           // Zè½´ç£åŠ›è®¡
} imu_param;

typedef struct
{
    float pitch;                                                           // ä¿¯ä»°è§’ï¼ˆå¼§åº¦ï¼‰
    float roll;                                                            // æ¨ªæ»šè§’ï¼ˆå¼§åº¦ï¼‰
    float yaw;                                                             // èˆªå‘è§’ï¼ˆå¼§åº¦ï¼‰
} euler_param;

typedef struct
{
    euler_param offset_angle;                                              // åç§»è§’åº¦
    imu_param data_Raw;                                                    // åŽŸå§‹æ•°æ®
    imu_param data_Ripen;                                                  // å¤„ç†åŽæ•°æ®
    euler_param eulerAngle;                                                // æ¬§æ‹‰è§’
} imu660_struct;

 //-------------------------------------------å…¨å±€å˜é‡å£°æ˜ŽåŒº------------------------------------------------------------
extern imu_param imu_date;                                                 // IMUæ•°æ®
extern imu660_struct imu660;                                               // IMU660ç»“æž„ä½“

 //-------------------------------------------æ¤­çƒæ‹Ÿåˆå®å®šä¹‰åŒº------------------------------------------------------------
#define MAG_CALIB_MAX_SAMPLES       500    // ç£åŠ›è®¡æ ¡å‡†æœ€å¤§é‡‡æ ·ç‚¹æ•°
#define MAG_CALIB_MIN_SAMPLES       100    // ç£åŠ›è®¡æ ¡å‡†æœ€å°é‡‡æ ·ç‚¹æ•°

 //-------------------------------------------å‡½æ•°å£°æ˜ŽåŒº------------------------------------------------------------
void date_handle(imu_param *data_src);                                     // å¤„ç†IMUæ•°æ®
void imu_bias_init(imu_param *data_src);                                   // åˆå§‹åŒ–IMUé›¶å
uint8 imu_mag_bias_load(imu_param *data_src);                              // ä»ŽFlashåŠ è½½ç£åŠ›è®¡é›¶å
void imu_mag_bias_save(const imu_param *data_src);                         // ä¿å­˜ç£åŠ›è®¡é›¶ååˆ°Flash
void imu_mag_calib_start(void);                                            // å¼€å§‹ç£åŠ›è®¡æ ¡å‡†
uint8 imu_mag_calib_finish(imu_param *data_src);                           // å®Œæˆç£åŠ›è®¡æ ¡å‡†
uint8 imu_mag_calib_is_active(void);                                       // æ£€æŸ¥ç£åŠ›è®¡æ ¡å‡†çŠ¶æ€
void static_imu_test(imu_param *data_src);                                 // é™æ€IMUæµ‹è¯•
void imu_gyro_z_autocalib(imu_param *data_src, unsigned int samples);      // è‡ªåŠ¨æ ¡å‡†Zè½´è§’é€Ÿåº¦

// æ¤­çƒæ‹Ÿåˆç£åŠ›è®¡æ ¡å‡†å‡½æ•°
void imu_mag_calib_ellipsoid_start(void);                                   // å¼€å§‹æ¤­çƒæ‹Ÿåˆæ ¡å‡†
uint8 imu_mag_calib_ellipsoid_update(float mx, float my, float mz);       // æ›´æ–°æ¤­çƒæ‹Ÿåˆæ•°æ®
uint8 imu_mag_calib_ellipsoid_finish(imu_param *data_src);                // å®Œæˆæ¤­çƒæ‹Ÿåˆå¹¶è®¡ç®—å‚æ•°
uint32 imu_mag_calib_get_sample_count(void);                               // èŽ·å–å½“å‰é‡‡æ ·ç‚¹æ•°

void imu_mag_send_raw_data_to_pc(void);

#endif
=======

 * imu660.h

 * IMUÊý¾Ý´¦ÀíÄ£¿é

 *

 * Created on: 2024Äê6ÔÂ6ÈÕ

 * Author: LateRain

 * Modified: 2025Äê11ÔÂ22ÈÕ

 */



#ifndef CODE_BALANCE_IMU660_H_

#define CODE_BALANCE_IMU660_H_



 //-------------------------------------------Í·ÎÄ¼þÇø------------------------------------------------------------

#include "zf_common_headfile.h"



 //-------------------------------------------ºê¶¨ÒåÇø------------------------------------------------------------

#define M_PI 3.1415                                                        // Ô²ÖÜÂÊ



 //-------------------------------------------½á¹¹Ìå¶¨ÒåÇø------------------------------------------------------------

typedef struct

{

    float gyro_x;                                                          // XÖá½ÇËÙ¶È

    float gyro_y;                                                          // YÖá½ÇËÙ¶È

    float gyro_z;                                                          // ZÖá½ÇËÙ¶È

    float acc_x;                                                           // XÖá¼ÓËÙ¶È

    float acc_y;                                                           // YÖá¼ÓËÙ¶È

    float acc_z;                                                           // ZÖá¼ÓËÙ¶È

    float gyro_x_bias;                                                     // XÖá½ÇËÙ¶ÈÁãÆ«

    float gyro_y_bias;                                                     // YÖá½ÇËÙ¶ÈÁãÆ«

    float gyro_z_bias;                                                     // ZÖá½ÇËÙ¶ÈÁãÆ«

    float acc_x_bias;                                                      // XÖá¼ÓËÙ¶ÈÁãÆ«

    float acc_y_bias;                                                      // YÖá¼ÓËÙ¶ÈÁãÆ«

    float acc_z_bias;                                                      // ZÖá¼ÓËÙ¶ÈÁãÆ«

    float mag_x_bias;                                                      // XÖá´ÅÁ¦¼ÆÁãÆ«£¨Ó²Ìú£©

    float mag_y_bias;                                                      // YÖá´ÅÁ¦¼ÆÁãÆ«£¨Ó²Ìú£©

    float mag_z_bias;                                                      // ZÖá´ÅÁ¦¼ÆÁãÆ«£¨Ó²Ìú£©

    float mag_soft_iron[9];                                                // 3x3ÈíÌú²¹³¥¾ØÕó£¨ÐÐÓÅÏÈ£©

    float mag_x;                                                           // XÖá´ÅÁ¦¼Æ

    float mag_y;                                                           // YÖá´ÅÁ¦¼Æ

    float mag_z;                                                           // ZÖá´ÅÁ¦¼Æ

} imu_param;



typedef struct

{

    float pitch;                                                           // ¸©Ñö½Ç£¨»¡¶È£©

    float roll;                                                            // ºá¹ö½Ç£¨»¡¶È£©

    float yaw;                                                             // º½Ïò½Ç£¨»¡¶È£©

} euler_param;



typedef struct

{

    euler_param offset_angle;                                              // Æ«ÒÆ½Ç¶È

    imu_param data_Raw;                                                    // Ô­Ê¼Êý¾Ý

    imu_param data_Ripen;                                                  // ´¦ÀíºóÊý¾Ý

    euler_param eulerAngle;                                                // Å·À­½Ç

} imu660_struct;



 //-------------------------------------------È«¾Ö±äÁ¿ÉùÃ÷Çø------------------------------------------------------------

extern imu_param imu_date;                                                 // IMUÊý¾Ý

extern imu660_struct imu660;                                               // IMU660½á¹¹Ìå



 //-------------------------------------------ÍÖÇòÄâºÏºê¶¨ÒåÇø------------------------------------------------------------

#define MAG_CALIB_MAX_SAMPLES       500    // ´ÅÁ¦¼ÆÐ£×¼×î´ó²ÉÑùµãÊý

#define MAG_CALIB_MIN_SAMPLES       100    // ´ÅÁ¦¼ÆÐ£×¼×îÐ¡²ÉÑùµãÊý



 //-------------------------------------------º¯ÊýÉùÃ÷Çø------------------------------------------------------------

void date_handle(imu_param *data_src);                                     // ´¦ÀíIMUÊý¾Ý

void imu_bias_init(imu_param *data_src);                                   // ³õÊ¼»¯IMUÁãÆ«

uint8 imu_mag_bias_load(imu_param *data_src);                              // ´ÓFlash¼ÓÔØ´ÅÁ¦¼ÆÁãÆ«

void imu_mag_bias_save(const imu_param *data_src);                         // ±£´æ´ÅÁ¦¼ÆÁãÆ«µ½Flash

void imu_mag_calib_start(void);                                            // ¿ªÊ¼´ÅÁ¦¼ÆÐ£×¼

uint8 imu_mag_calib_finish(imu_param *data_src);                           // Íê³É´ÅÁ¦¼ÆÐ£×¼

uint8 imu_mag_calib_is_active(void);                                       // ¼ì²é´ÅÁ¦¼ÆÐ£×¼×´Ì¬

void static_imu_test(imu_param *data_src);                                 // ¾²Ì¬IMU²âÊÔ

void imu_gyro_z_autocalib(imu_param *data_src, unsigned int samples);      // ×Ô¶¯Ð£×¼ZÖá½ÇËÙ¶È



// ÍÖÇòÄâºÏ´ÅÁ¦¼ÆÐ£×¼º¯Êý

void imu_mag_calib_ellipsoid_start(void);                                   // ¿ªÊ¼ÍÖÇòÄâºÏÐ£×¼

uint8 imu_mag_calib_ellipsoid_update(float mx, float my, float mz);       // ¸üÐÂÍÖÇòÄâºÏÊý¾Ý

uint8 imu_mag_calib_ellipsoid_finish(imu_param *data_src);                // Íê³ÉÍÖÇòÄâºÏ²¢¼ÆËã²ÎÊý

uint32 imu_mag_calib_get_sample_count(void);                               // »ñÈ¡µ±Ç°²ÉÑùµãÊý



void imu_mag_send_raw_data_to_pc(void);



#endif

>>>>>>> 82b88aed84cc13edbe61c5ab32284ccfbdcd9e9e
