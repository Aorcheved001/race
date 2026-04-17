/*********************************************************************************************************************
* TC377 Opensourec Library ����TC377 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC377 ��Դ���һ����
*
* TC377 ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_absolute_encoder
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC377TP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-11-03       pudding           first version
* 2023-04-25       pudding           ��������ע��˵��
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   SCLK               �鿴 zf_device_absolute_encoder.h �� ABSOLUTE_ENCODER_SCLK_PIN �궨��
*                   MOSI               �鿴 zf_device_absolute_encoder.h �� ABSOLUTE_ENCODER_MOSI_PIN �궨��
*                   MISO               �鿴 zf_device_absolute_encoder.h �� ABSOLUTE_ENCODER_MISO_PIN �궨��
*                   CS                 �鿴 zf_device_absolute_encoder.h �� ABSOLUTE_ENCODER_CS_PIN �궨��
*                   VCC                3.3V��Դ
*                   GND                ��Դ��
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_absolute_encoder_h_
#define _zf_device_absolute_encoder_h_

#include "zf_common_typedef.h"

//=================================================���� �Ƕȴ����� ��������================================================
#define ABSOLUTE_ENCODER_USE_SOFT_SPI       (1)                                 // Ĭ��ʹ��Ӳ�� SPI ��ʽ����
#if ABSOLUTE_ENCODER_USE_SOFT_SPI                                               // ������ ��ɫ�����Ĳ�����ȷ�� ��ɫ�ҵľ���û���õ�
//====================================================���� SPI ����====================================================
#define ABSOLUTE_ENCODER_SOFT_SPI_DELAY     (1)                                 // ���� SPI ��ʱ����ʱ���� ��ֵԽС SPI ͨ������Խ��
#define ABSOLUTE_ENCODER_SCLK_PIN           (P20_7)                            // ���� SPI SCK ����
#define ABSOLUTE_ENCODER_MOSI_PIN           (P20_6)                            // ���� SPI MOSI ����
#define ABSOLUTE_ENCODER_MISO_PIN           (P20_5)                            // ���� SPI MISO ����
//====================================================���� SPI ����====================================================
#else
//====================================================Ӳ�� SPI ����====================================================
#define ABSOLUTE_ENCODER_SPI_SPEED          (10*1000*1000)                      // Ӳ�� SPI ����
#define ABSOLUTE_ENCODER_SPI                (SPI_0)                             // Ӳ�� SPI ��
#define ABSOLUTE_ENCODER_SCLK_PIN           (SPI0_SCLK_P20_11)                  // Ӳ�� SPI SCK ����
#define ABSOLUTE_ENCODER_MOSI_PIN           (SPI0_MOSI_P20_14)                  // Ӳ�� SPI MOSI ����
#define ABSOLUTE_ENCODER_MISO_PIN           (SPI0_MISO_P20_12)                  // Ӳ�� SPI MISO ����
//====================================================Ӳ�� SPI ����====================================================
#endif

#define ABSOLUTE_ENCODER_CS_PIN             (P20_4)                            // CS���Ŷ���
#define ABSOLUTE_ENCODER_CSN(x)             ((x) ? (gpio_high(ABSOLUTE_ENCODER_CS_PIN)): (gpio_low(ABSOLUTE_ENCODER_CS_PIN)))

#define ABSOLUTE_ENCODER_TIMEOUT_COUNT      (100)                               // �Լ쳬ʱʱ��
#define ABSOLUTE_ENCODER_DEFAULT_ZERO       (0)

#define ABSOLUTE_ENCODER_SPI_W              (0x80)
#define ABSOLUTE_ENCODER_SPI_R              (0x40)

#define ABSOLUTE_ENCODER_ZERO_L_REG         (0x00)
#define ABSOLUTE_ENCODER_ZERO_H_REG         (0x01)
#define ABSOLUTE_ENCODER_DIR_REG            (0X09)
//=================================================���� �Ƕȴ����� ��������================================================


//=================================================���� �Ƕȴ����� ��������================================================
int16   absolute_encoder_get_location       (void);                             // ����ֵ��������ȡ��ǰ�Ƕ�ֵ
int16   absolute_encoder_get_offset         (void);                             // ����ֵ��������ȡ����ϴ�λ�õ�ƫ��ֵ
uint8   absolute_encoder_init               (void);                             // ����ֵ��������ʼ��
//=================================================���� �Ƕȴ����� ��������================================================
#endif

