/*
 * track.c
 *
 * Created on: 2024��6��6��
 * Author: LateRain
 * Modified: 2025��11��22��
 */

//-------------------------------------------ͷ�ļ���------------------------------------------------------------
#include "track.h"
#include "Ins.h"
#include "zf_driver_flash.h"
#include "zf_device_ips200.h"
#include "steering_control.h"

//-------------------------------------------�ṹ�����������------------------------------------------------------------
Ins_Date Ins_date_377 = {0};

//-------------------------------------------ȫ�ֱ���������-----------------------------------------------------------
uint8 track_save_flag = 0;
uint8 track_follow_flag = 0;
uint32 track_total_points = 0;

//-------------------------------------------�ڲ�����������-----------------------------------------------------------
static uint32 track_flash_cur_write_page = Track_Flash_Page_Begin;
// ��ǰ��ѯд���ҳ��

static float dist_acc_m = 0.0f;
// �ڲ�����̴������̱���

static int16_t flash_point_index = 0;
// ��ǰ����flash_union_buffer�ĵڼ���Ԫ�أ���float�ƣ�

static uint32 Ins_Date_Read[510] = {0};
// ��ȡ��������uint32����flash_read_page�ӿ�һ�£�

static uint16 follow_page = Track_Flash_Page_Begin;
// ѭ��ʱ��ǰ��ȡ��ҳ��

static uint16 follow_point_idx = 1;
// ѭ��ʱ��ǰ��ȡ�ĵ�����

static uint32 follow_abs_index = 1;
// ѭ��ʱ��ǰ��ľ���������1-track_total_points��

static float steer_output = 0.0f;
// ת�����

static float steer_output_filtered = 0.0f;
// ת���˲����

static float current_speed_dir = 1.0f; // ��ǰ��ʻ����1��ʾǰ����0��ʾ����

#define STEER_FILTER_ALPHA    0.85f
// ת��һ�׵�ͨ�˲�ϵ����Խ��Խ����ԭʼֵ��

#define TRACK_META_PAGE                0u
#define TRACK_META_MAGIC               0x5452434Bu
#define TRACK_META_VERSION             1u
#define TRACK_MAX_COORD_ABS            1000000.0f
#define TRACK_MAX_YAW_ABS_RAD          3.5f

//-------------------------------------------�ڲ�����������-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��ȡ�Ѵ洢�ĵ����������ޱ�����
//  ����˵��      void
//  ���ز���      uint32           ʵ�ʿ��õĵ���
//  ��ע��Ϣ      ��ֹtrack_total_points���
//-------------------------------------------------------------------------------------------------------------------
static uint32 track_get_stored_point_count(void)
{
    const uint32 max_points = (uint32)Track_Flash_Page_Max * (uint32)Track_Flash_Point_Page_Max;
    return (track_total_points > max_points) ? max_points : track_total_points;
}

static uint8 track_float_is_valid(float value, float abs_limit)
{
    if(value != value)
    {
        return 0;
    }

    return (fabsf(value) <= abs_limit) ? 1u : 0u;
}

static void track_meta_save(void)
{
    uint32 meta_buf[EEPROM_PAGE_LENGTH];
    memset(meta_buf, 0xFF, sizeof(meta_buf));

    meta_buf[0] = TRACK_META_MAGIC;
    meta_buf[1] = TRACK_META_VERSION;
    meta_buf[2] = track_get_stored_point_count();
    meta_buf[3] = track_flash_cur_write_page;
    meta_buf[4] = (uint32)((flash_point_index < 0) ? 0 : flash_point_index);

    flash_write_page(0, TRACK_META_PAGE, meta_buf, EEPROM_PAGE_LENGTH);
}

static void track_meta_clear(void)
{
    flash_erase_page(0, TRACK_META_PAGE);
}

static void track_meta_load(void)
{
    uint32 meta_buf[EEPROM_PAGE_LENGTH];
    const uint32 max_points = (uint32)Track_Flash_Page_Max * (uint32)Track_Flash_Point_Page_Max;

    flash_read_page(0, TRACK_META_PAGE, meta_buf, EEPROM_PAGE_LENGTH);

    if(meta_buf[0] != TRACK_META_MAGIC || meta_buf[1] != TRACK_META_VERSION)
    {
        return;
    }

    if(meta_buf[2] > max_points)
    {
        return;
    }

    if(meta_buf[3] < Track_Flash_Page_Begin || meta_buf[3] >= (Track_Flash_Page_Begin + Track_Flash_Page_Max))
    {
        return;
    }

    if(meta_buf[4] > Track_Flash_Page_Float_Num || (meta_buf[4] % Track_Flash_Float_Num) != 0u)
    {
        return;
    }

    track_total_points = meta_buf[2];
    track_flash_cur_write_page = meta_buf[3];
    flash_point_index = (int16_t)meta_buf[4];
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��flash_union_buffer��ҳд��ָ��flashҳ
//  ����˵��      page_num         Ŀ��ҳ��
//  ���ز���      void
//  ʹ��ʾ��      track_flash_cache_flush(1);
//  ��ע��Ϣ      ֱ�ӽ�flash_union_bufferд�룬����ǰȷ������������buffer
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_cache_flush(uint32 page_num)
{
    if((page_num < Track_Flash_Page_Begin) || (page_num >= (Track_Flash_Page_Begin + Track_Flash_Page_Max)))
    {
        return;
    }

    // �Ȳ���ҳ����д������
    flash_erase_page(0, page_num);
    flash_write_page_from_buffer(0, page_num);              // ֱ�ӽ�flash_union_bufferд��ָ��ҳ
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ҳ����ѯ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_flash_add();
//  ��ע��Ϣ      �����һҳ��ص���ʼҳ
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_add(void)
{
    track_flash_cur_write_page++;                            // ��ѯ��

    if(track_flash_cur_write_page >= (Track_Flash_Page_Begin + Track_Flash_Page_Max))
    {
        track_flash_cur_write_page = Track_Flash_Page_Begin; // �����һҳ��ص���ʼҳ
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     Pure Pursuit����ת���
//  ����˵��      x, y, yaw       ��ǰλ�úͺ����
//  ����˵��      target          Ŀ���
//  ���ز���      float           ת��ǣ��ȣ�
//  ��ע��Ϣ      ����Pure Pursuit�㷨����ת���
//-------------------------------------------------------------------------------------------------------------------
static float pure_pursuit_calc_steer(float x, float y, float yaw, Ins_follow* target)
{
    // 1. ����ȫ��ƫ��
    float dx = target->x - x;
    float dy = target->y - y;

    // 2. ת������������ϵ
    float x_local = dx * cosf(yaw) + dy * sinf(yaw);
    float y_local = -dx * sinf(yaw) + dy * cosf(yaw);

    // 3. ����ǰ�Ӿ��� (Ld)
    float ld = sqrtf(x_local * x_local + y_local * y_local);

    // ��ֹ�����㣬�ҷ�ֹ����������µľ���ת��
    if(ld < 0.1f) return 0.0f;

    // 4. �������� (k = 2 * y / Ld^2)
    float curvature = 2.0f * y_local / (ld * ld);

    // 5. ����ת���
    float steer_rad = atanf(curvature * TRACK_WHEELBASE);
    float steer_deg = steer_rad * 57.2957795130823f; // ת�Ƕ�

    // ����ʱ��תת����
    if(target->speed_dir < 0.5f)
    {
        steer_deg = -steer_deg;
    }

    // 6. ������������
    // ����Ƕ��� -2 �� 2 ��֮�䣬��Ϊ��ֱ�У���������
    if (steer_deg > -2.0f && steer_deg < 2.0f)
    {
        steer_deg = 0.0f;
    }

    // 7. �޷�
    if(steer_deg > 20.0f) steer_deg = 20.0f;
    if(steer_deg < -20.0f) steer_deg = -20.0f;

    return steer_deg;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ׼��ָ�������ĵ�����
//  ����˵��      abs_index       ���Ե�������1��ʼ��
//  ���ز���      uint8           1-�ɹ� 0-ʧ��
//  ��ע��Ϣ      �Զ��л�ҳ����������
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_follow_prepare_point(uint32 abs_index)
{
    uint32 stored_points = track_get_stored_point_count();
    if(abs_index < 1 || abs_index > stored_points)
    {
        return 0;
    }

    uint32 page_offset = (abs_index - 1U) / Track_Flash_Point_Page_Max;
    uint32 target_page = Track_Flash_Page_Begin + page_offset;
    uint32 point_in_page = ((abs_index - 1U) % Track_Flash_Point_Page_Max) + 1U;

    if(target_page != follow_page)
    {
        track_flash_read_page(target_page);
        follow_page = (uint16)target_page;
    }

    follow_point_idx = (uint16)point_in_page;
    follow_abs_index = abs_index;

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     Ѱ��ǰ�ӵ�
//  ����˵��      x, y            ��ǰλ��
//  ���ز���      uint8           1-�ҵ� 0-δ�ҵ�
//  ��ע��Ϣ      �ӵ�ǰ�㿪ʼ��ǰѰ�Ҿ������ǰ�Ӿ���ĵ㣬�����Ƶ����
//-------------------------------------------------------------------------------------------------------------------
static uint8 find_lookahead_point(float x, float y)
{
    uint32 stored_points = track_get_stored_point_count();
    if(stored_points == 0)
    {
        return 0;
    }

    if(follow_abs_index < 1 || follow_abs_index > stored_points)
    {
        follow_abs_index = 1;
    }

    uint32 search_end = stored_points;
    uint32 start_index = follow_abs_index;

    // �ȼ�鵱ǰ�����Ƿ���Ч
    if(!track_follow_prepare_point(follow_abs_index))
    {
        follow_abs_index = 1;
        if(!track_follow_prepare_point(follow_abs_index))
        {
            return 0;
        }
    }

    // �ӵ�ǰ������ʼ����
    uint32 last_same_dir_index = follow_abs_index;
    Ins_follow last_same_dir_point;
    track_flash_get_point(follow_point_idx, &last_same_dir_point);

    while(follow_abs_index <= search_end)
    {
        Ins_follow pt;
        if(track_flash_get_point(follow_point_idx, &pt))
        {
            // ����Ƿ��뵱ǰ������ͬ
            if((pt.speed_dir >= 0.5f && current_speed_dir >= 0.5f) ||
               (pt.speed_dir < 0.5f && current_speed_dir < 0.5f))
            {
                // ������ͬ���������һ��ͬ�����
                last_same_dir_index = follow_abs_index;
                last_same_dir_point = pt;

                // �������Ƿ��㹻
                float dx = pt.x - x;
                float dy = pt.y - y;
                float dist = sqrtf(dx * dx + dy * dy);

                if(dist >= TRACK_LOOKAHEAD_DISTANCE)
                {
                    Ins_date_377.x   = pt.x;
                    Ins_date_377.y   = pt.y;
                    Ins_date_377.yaw = pt.yaw;
                    Ins_date_377.speed_dir = pt.speed_dir;

                    return 1;
                }
            }
            else
            {
                // ����ͬ��ʹ�����һ��ͬ����ĵ�
                Ins_date_377.x   = last_same_dir_point.x;
                Ins_date_377.y   = last_same_dir_point.y;
                Ins_date_377.yaw = last_same_dir_point.yaw;
                Ins_date_377.speed_dir = last_same_dir_point.speed_dir;

                // �������������һ��ͬ�����
                follow_abs_index = last_same_dir_index;

                return 1;
            }
        }

        follow_abs_index++;
        if(follow_abs_index > search_end)
        {
            break;
        }

        if(!track_follow_prepare_point(follow_abs_index))
        {
            continue;
        }
    }

    // ���������е㶼û�ҵ���ʹ�����һ����
    if(track_follow_prepare_point(stored_points))
    {
        Ins_follow pt;
        if(track_flash_get_point(follow_point_idx, &pt))
        {
            Ins_date_377.x   = pt.x;
            Ins_date_377.y   = pt.y;
            Ins_date_377.yaw = pt.yaw;
            Ins_date_377.speed_dir = pt.speed_dir;
            follow_abs_index = stored_points;

            return 1;
        }
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ѭ������
//  ����˵��      void
//  ���ز���      void
//  ��ע��Ϣ      ִ��Pure Pursuitѭ���㷨������ʧ��ʱ������һ֡ת��ǲ�����
//-------------------------------------------------------------------------------------------------------------------
static void track_follow(void)
{
    const INS_State* state = Ins_get_state();

    if(state == NULL)
    {
        track_stop_follow();
        return;
    }

    if(!find_lookahead_point(state->x, state->y))
    {
        steering_set_target(steer_output_filtered);
        wheel_pid_set_target_speed(TRACK_FOLLOW_SPEED * 0.5f);
        return;
    }

    // ���㵽ǰ�ӵ�ľ���
    float dx = Ins_date_377.x - state->x;
    float dy = Ins_date_377.y - state->y;
    float dist_to_lookahead = sqrtf(dx * dx + dy * dy);

    // ��ȡ�ܵ���
    uint32 stored_points = track_get_stored_point_count();

    // ����Ѿ��ӽ�ǰ�ӵ㣬��ǰ�ƽ�����
    if(dist_to_lookahead < 0.05f)
    {
        if(follow_abs_index < stored_points)
        {
            // �����һ�����Ƿ���ͬ
            uint32 next_index = follow_abs_index + 1;
            if(track_follow_prepare_point(next_index))
            {
                Ins_follow next_point;
                if(track_flash_get_point(follow_point_idx, &next_point))
                {
                    // �����һ���㷽��ͬ���л�����
                    if((next_point.speed_dir >= 0.5f && current_speed_dir < 0.5f) ||
                       (next_point.speed_dir < 0.5f && current_speed_dir >= 0.5f))
                    {
                        current_speed_dir = next_point.speed_dir;
                    }
                }
            }

            follow_abs_index++;
        }
        else if(follow_abs_index == stored_points)
        {
            // �������յ㣬ֹͣѭ��
            track_stop_follow();
            return;
        }
    }

    Ins_follow target = {Ins_date_377.x, Ins_date_377.y, Ins_date_377.yaw};
    steer_output = pure_pursuit_calc_steer(state->x, state->y, state->yaw, &target);

    steer_output_filtered = STEER_FILTER_ALPHA * steer_output +
                            (1.0f - STEER_FILTER_ALPHA) * steer_output_filtered;

    // ����·������ٶȷ���������ʻ����
    float target_speed = TRACK_FOLLOW_SPEED;
    if(Ins_date_377.speed_dir < 0.5f) // speed_dir < 0.5��ʾ����
    {
        target_speed = -TRACK_FOLLOW_SPEED;
    }

    steering_set_target(steer_output_filtered);
    wheel_pid_set_target_speed(target_speed);
}

//-------------------------------------------����������-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��ʼ�����ģ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_init();
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void track_init(void)
{
    track_flash_cur_write_page = Track_Flash_Page_Begin;

    dist_acc_m = 0.0f;                                        // ����ۼ�����
    flash_point_index = 0;                                    // д����������

    flash_buffer_clear();                                     // ���flash_union_buffer��ȫ����0xFF����ֹ�����ݣ�

    memset(Ins_Date_Read, 0, sizeof(Ins_Date_Read));          // ��ն�ȡ������

    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    follow_abs_index = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    current_speed_dir = 1.0f; // ��ʼ����Ϊǰ��

    track_save_flag = 0;
    track_follow_flag = 0;
    track_total_points = 0;

    track_meta_load();
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ���չ̶���̴���һ����
//  ʹ��ʾ��      track_flash_push_point();
//  @note        ����1��ʾ�ɹ�д�룬0��ʾ������δ���������
//               ������һҳ���Զ���ҳдflash����ѯ����һҳ
//-------------------------------------------------------------------------------------------------------------------
uint8 track_flash_push_point(void)
{
    const EncoderLayerState* state = encoder_layer_get_state();

    if(state != NULL)
    {
        float ds = 0.5f * (state->delta_right_m + state->delta_left_m);
        dist_acc_m += fabsf(ds);                              // �ۼ���ʻ���
    }

    // û���̶���̣���д��
    if(dist_acc_m < TRACK_SAMPLE_STEP)                        // ����ۼ���̲���һ����������
    {
        return 0;
    }
    dist_acc_m -= TRACK_SAMPLE_STEP;                          // ������������������С

    // Խ�籣����������һҳ����д��flash����ҳ���
    if(flash_point_index + (int16_t)Track_Flash_Float_Num > (int16_t)Track_Flash_Page_Float_Num)
    {
        track_flash_cache_flush(track_flash_cur_write_page);
        track_meta_save();
        track_flash_add();
        flash_point_index = 0;
        flash_buffer_clear();
    }

    // ��x, y, yaw, speed_dirд��flash_union_buffer
    const INS_State* ins_state = Ins_get_state();
    const EncoderLayerState* enc_state = encoder_layer_get_state();
    flash_data_union tmp;

    // �洢λ�úͺ����
    tmp.float_type = ins_state->x;
    flash_union_buffer[flash_point_index] = tmp;              // ��xд��flash_union_buffer
    tmp.float_type = ins_state->y;
    flash_union_buffer[flash_point_index + 1] = tmp;          // ��yд��flash_union_buffer
    tmp.float_type = ins_state->yaw;
    flash_union_buffer[flash_point_index + 2] = tmp;          // ��yawд��flash_union_buffer

    // �洢�ٶȷ���1��ʾǰ����0��ʾ����
    float speed_dir = 1.0f; // Ĭ��ǰ��
    if(enc_state != NULL && enc_state->speed_average_mps < -0.05f) // �ٶ�Ϊ����ʾ����
    {
        speed_dir = 0.0f;
    }
    tmp.float_type = speed_dir;
    flash_union_buffer[flash_point_index + 3] = tmp;          // ��speed_dirд��flash_union_buffer

    flash_point_index += 4;
    if(track_total_points < ((uint32)Track_Flash_Page_Max * (uint32)Track_Flash_Point_Page_Max))
    {
        track_total_points++;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ����ǰflash_union_bufferд��ָ��ҳ������ӿڣ�
//  ����˵��      page_num         Ŀ��ҳ��
//  ���ز���      void
//  ʹ��ʾ��      track_flash_write_cache(1);
//  ��ע��Ϣ      ���ı���ѯҳ�ţ�ִֻ��һ��ָ��ҳд��
//-------------------------------------------------------------------------------------------------------------------
void track_flash_write_cache(uint32 page_num)
{
    track_flash_cache_flush(page_num);
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��ָ��ҳ��ȡһ��ҳԭʼ���ݵ���ȡ������ Ins_Date_Read
//  ����˵��      page_num         Ŀ��ҳ��
//  ���ز���      void
//  ʹ��ʾ��      track_flash_read_page(1);
//  ��ע��Ϣ      ��ȡ��ɺ���� track_flash_get_point(index, &out) ����Ž��������
//-------------------------------------------------------------------------------------------------------------------
void track_flash_read_page(uint32 page_num)
{
    if((page_num < Track_Flash_Page_Begin) || (page_num >= (Track_Flash_Page_Begin + Track_Flash_Page_Max)))
    {
        return;
    }

    flash_read_page(0, page_num, Ins_Date_Read, 510);         // ����ҳ���ݶ����ȡ������
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     �Ӷ�ȡ�������н���ָ����ŵĵ�
//  ����˵��      point_index      �ڼ����㣬1-170
//  ����˵��      out              ����ṹ��ָ��
//  ���ز���      uint8            1-�ɹ� 0-Խ��
//  ʹ��ʾ��      Ins_follow follow; track_flash_get_point(5, &follow);
//  ��ע��Ϣ      ���ȵ���track_flash_read_page��ȡ��Ӧҳ
//-------------------------------------------------------------------------------------------------------------------
uint8 track_flash_get_point(uint32 point_index, Ins_follow* out)
{
    uint32 base = 0;
    flash_data_union data_temp;                               // ��ʱ��������������ת��

    if(out == NULL || point_index < 1 || point_index > Track_Flash_Point_Page_Max)
    {
        return 0;
    }

    base = (point_index - 1) * Track_Flash_Float_Num;          // ��point_index�������ʼ��ַΪ(point_index-1)*4

    data_temp.uint32_type = Ins_Date_Read[base + 0];          // ��x��uint32ת��Ϊfloat
    out->x = data_temp.float_type;                            // ��x��ֵ��out->x

    data_temp.uint32_type = Ins_Date_Read[base + 1];          // ��y��uint32ת��Ϊfloat
    out->y = data_temp.float_type;                            // ��y��ֵ��out->y

    data_temp.uint32_type = Ins_Date_Read[base + 2];          // ��yaw��uint32ת��Ϊfloat
    out->yaw = data_temp.float_type;                          // ��yaw��ֵ��out->yaw

    data_temp.uint32_type = Ins_Date_Read[base + 3];          // ��speed_dir��uint32ת��Ϊfloat
    out->speed_dir = data_temp.float_type;                    // ��speed_dir��ֵ��out->speed_dir

    // ����Ƿ�ΪFlash����ֵ
    if(Ins_Date_Read[base + 0] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 1] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 2] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 3] == 0xFFFFFFFFu)
    {
        return 0;
    }

    // ����Ƿ�Ϊ0ֵ��������δ��ʼ����
    if(Ins_Date_Read[base + 0] == 0x00000000u &&
       Ins_Date_Read[base + 1] == 0x00000000u &&
       Ins_Date_Read[base + 2] == 0x00000000u &&
       Ins_Date_Read[base + 3] == 0x00000000u)
    {
        return 0;
    }

    // ��鸡������Ч��
    if(!track_float_is_valid(out->x, TRACK_MAX_COORD_ABS) ||
       !track_float_is_valid(out->y, TRACK_MAX_COORD_ABS) ||
       !track_float_is_valid(out->yaw, TRACK_MAX_YAW_ABS_RAD) ||
       !track_float_is_valid(out->speed_dir, 1.0f))
    {
        return 0;
    }

    // ��������Ƿ��������ֹ�������ֵ��
    if(fabsf(out->x) > 1000.0f || fabsf(out->y) > 1000.0f)
    {
        return 0;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ������д��ҳ
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_flash_clear_all();
//  ��ע��Ϣ      ��������õ�ȫ�����ҳ
//-------------------------------------------------------------------------------------------------------------------
void track_flash_clear_all(void)
{
    uint32 page = 0;

    track_meta_clear();
    for(page = 0; page < Track_Flash_Page_Max; page++)
    {
        flash_erase_page(0, Track_Flash_Page_Begin + page);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��ȡ��ǰ��ѯдҳ
//  ����˵��      void
//  ���ز���      uint32           ��ǰ��ѯдҳ��
//  ʹ��ʾ��      track_flash_get_cur_write_page();
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint32 track_flash_get_cur_write_page(void)
{
    return track_flash_cur_write_page;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��ȡ��ǰ��д��buffer��float������ÿ3��floatΪһ���㣩
//  ����˵��      void
//  ���ز���      int16_t          ��ǰflash_point_indexֵ
//  ʹ��ʾ��      track_flash_get_point_index();
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
int16_t track_flash_get_point_index(void)
{
    return flash_point_index;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ������¼������ǰδ��ҳ�Ļ���ǿ��д��Flash
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_flash_finish();
//  ��ע��Ϣ      ֹͣ��¼ʱ������ã�����������ɵ�ᶪʧ
//-------------------------------------------------------------------------------------------------------------------
void track_flash_finish(void)
{
    if(flash_point_index > 0)
    {
        track_flash_cache_flush(track_flash_cur_write_page);
    }

    track_meta_save();
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ������й켣����
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_clear();
//  ��ע��Ϣ      �������д��ҳ������״̬
//-------------------------------------------------------------------------------------------------------------------
void track_clear(void)
{
    track_flash_clear_all();

    track_save_flag = 0;
    track_follow_flag = 0;
    track_flash_cur_write_page = Track_Flash_Page_Begin;
    flash_point_index = 0;
    dist_acc_m = 0.0f;
    track_total_points = 0;
    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    follow_abs_index = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    flash_buffer_clear();
    track_meta_clear();

    wheel_pid_set_target_speed(0.0f);
    steering_set_target(0.0f);
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��ʼ���
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_start_save();
//  ��ע��Ϣ      ����״̬����ʼ���
//-------------------------------------------------------------------------------------------------------------------
void track_start_save(void)
{
    track_flash_cur_write_page = Track_Flash_Page_Begin;
    flash_point_index = 0;
    dist_acc_m = 0.0f;
    track_total_points = 0;
    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    follow_abs_index = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    flash_buffer_clear();
    track_follow_flag = 0;
    track_save_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ֹͣ���
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_stop_save();
//  ��ע��Ϣ      ֹͣ��㲢������д��Flash
//-------------------------------------------------------------------------------------------------------------------
void track_stop_save(void)
{
    if(track_save_flag == 1)
    {
        track_save_flag = 0;
        track_flash_finish();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ��ʼѭ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_start_follow();
//  ��ע��Ϣ      �ӵ�һҳ��ʼѭ��
//-------------------------------------------------------------------------------------------------------------------
void track_start_follow(void)
{
    if(track_get_stored_point_count() == 0)
    {
        track_follow_flag = 0;
        wheel_pid_set_target_speed(0.0f);
        steering_set_target(0.0f);
        return;
    }

    // ����INSλ�ã�ȷ����ԭ�㿪ʼ
    Ins_reset(0.0f, 0.0f, 0.0f);

    // ����ѭ��״̬
    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    follow_abs_index = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    current_speed_dir = 1.0f; // ��ʼ����Ϊǰ��

    // ȷ��Flash���ݶ�ȡ���
    track_flash_read_page(follow_page);

    // ��֤��һҳ�����Ƿ���Ч
    Ins_follow first_point;
    if(!track_flash_get_point(1, &first_point))
    {
        track_follow_flag = 0;
        wheel_pid_set_target_speed(0.0f);
        steering_set_target(0.0f);
        return;
    }

    // ��ʼ��Ins_date_377��speed_dir�ֶ�
    Ins_date_377.speed_dir = first_point.speed_dir;
    current_speed_dir = first_point.speed_dir;

    // ����ִ��һ��Ѱ�㣬ȷ��Tag X�ڿ�ʼ�ƶ�ǰ�͸���
    const INS_State* state = Ins_get_state();
    if(state != NULL)
    {
        // ��������3��Ѱ�㣬ȷ���ҵ���Ч��
        for(int i = 0; i < 3; i++)
        {
            if(find_lookahead_point(state->x, state->y))
            {
                break;
            }
            // �����ӳٺ�����
            for(int j = 0; j < 1000; j++)
            {
                __asm__ volatile ("nop");
            }
        }
    }

    // ȷ������ص�����λ��
    steering_set_target(0.0f);

    // ���ó�ʼ�ٶ�
    wheel_pid_set_target_speed(TRACK_FOLLOW_SPEED);

    // �л���ѭ��ģʽ
    track_save_flag = 0;
    track_follow_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     �����д洢�Ĺ켣�㷢�͵���λ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_send_all_points_to_host();
//  ��ע��Ϣ      ���͸�ʽ: TrackPoint:index,x,y,yaw\r\n
//-------------------------------------------------------------------------------------------------------------------
void track_send_all_points_to_host(void)
{
    uint32 stored_points = track_get_stored_point_count();
    if(stored_points == 0)
    {
        printf("TrackInfo:NoData\r\n");
        return;
    }

    printf("TrackInfo:Start,TotalPoints=%lu\r\n", (unsigned long)stored_points);

    uint32 i;
    for(i = 1; i <= stored_points; i++)
    {
        if(track_follow_prepare_point(i))
        {
            Ins_follow pt;
            if(track_flash_get_point(follow_point_idx, &pt))
            {
                printf("TrackPoint:%lu,%.4f,%.4f,%.4f,%.0f\r\n",
                       (unsigned long)i, pt.x, pt.y, pt.yaw, pt.speed_dir);
            }
        }
    }

    printf("TrackInfo:End\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ֹͣѭ��
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_stop_follow();
//  ��ע��Ϣ      ֹͣѭ����ͣ��
//-------------------------------------------------------------------------------------------------------------------
void track_stop_follow(void)
{
    track_follow_flag = 0;
    wheel_pid_set_target_speed(0.0f);
    steering_set_target(0.0f);
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     �켣ģ��������
//  ����˵��      void
//  ���ز���      void
//  ʹ��ʾ��      track_proc();
//  ��ע��Ϣ      ���ڵ��ã�ִ�д���ѭ��
//-------------------------------------------------------------------------------------------------------------------
void track_proc(void)
{
    if(track_save_flag == 1)
    {
        track_flash_push_point();
    }
    else if(track_follow_flag == 1)
    {
        track_follow();
    }
}
