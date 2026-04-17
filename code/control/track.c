/*
 * track.c
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

//-------------------------------------------头文件区------------------------------------------------------------
#include "track.h"
#include "Ins.h"
#include "zf_driver_flash.h"
#include "zf_device_ips200.h"

//-------------------------------------------结构体变量定义区------------------------------------------------------------
Ins_Date Ins_date_377 = {0};

//-------------------------------------------全局变量定义区-----------------------------------------------------------
uint8 track_save_flag = 0;
uint8 track_follow_flag = 0;
uint32 track_total_points = 0;

//-------------------------------------------内部变量定义区-----------------------------------------------------------
static uint32 track_flash_cur_write_page = Track_Flash_Page_Begin;
// 当前轮询写入的页号

static float dist_acc_m = 0.0f;
// 内部计里程存点存放里程变量

static int16_t flash_point_index = 0;
// 当前存入flash_union_buffer的第几个元素（以float计）

static uint32 Ins_Date_Read[510] = {0};
// 读取缓冲区（uint32，与flash_read_page接口一致）

static uint16 follow_page = Track_Flash_Page_Begin;
// 循迹时当前读取的页号

static uint16 follow_point_idx = 1;
// 循迹时当前读取的点索引

static uint32 follow_abs_index = 1;
// 循迹时当前点的绝对索引（1-track_total_points）

static float steer_output = 0.0f;
// 转向输出

static float steer_output_filtered = 0.0f;
// 转向滤波输出

static float current_speed_dir = 1.0f; // 当前行驶方向：1表示前进，0表示倒车

#define STEER_FILTER_ALPHA    0.85f
// 转向一阶低通滤波系数（越大越跟随原始值）

#define TRACK_META_PAGE                0u
#define TRACK_META_MAGIC               0x5452434Bu
#define TRACK_META_VERSION             1u
#define TRACK_MAX_COORD_ABS            1000000.0f
#define TRACK_MAX_YAW_ABS_RAD          3.5f

//-------------------------------------------内部辅助函数区-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获取已存储的点数（带上限保护）
//  参数说明      void
//  返回参数      uint32           实际可用的点数
//  备注信息      防止track_total_points溢出
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
//  函数简介     将flash_union_buffer整页写入指定flash页
//  参数说明      page_num         目标页号
//  返回参数      void
//  使用示例      track_flash_cache_flush(1);
//  备注信息      直接将flash_union_buffer写入，调用前确保数据已填满buffer
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_cache_flush(uint32 page_num)
{
    if((page_num < Track_Flash_Page_Begin) || (page_num >= (Track_Flash_Page_Begin + Track_Flash_Page_Max)))
    {
        return;
    }

    // 先擦除页，再写入数据
    flash_erase_page(0, page_num);
    flash_write_page_from_buffer(0, page_num);              // 直接将flash_union_buffer写入指定页
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     页号轮询加
//  参数说明      void
//  返回参数      void
//  使用示例      track_flash_add();
//  备注信息      到最后一页后回到起始页
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_add(void)
{
    track_flash_cur_write_page++;                            // 轮询加

    if(track_flash_cur_write_page >= (Track_Flash_Page_Begin + Track_Flash_Page_Max))
    {
        track_flash_cur_write_page = Track_Flash_Page_Begin; // 到最后一页后回到起始页
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     Pure Pursuit计算转向角
//  参数说明      x, y, yaw       当前位置和航向角
//  参数说明      target          目标点
//  返回参数      float           转向角（度）
//  备注信息      基于Pure Pursuit算法计算转向角
//-------------------------------------------------------------------------------------------------------------------
static float pure_pursuit_calc_steer(float x, float y, float yaw, Ins_follow* target)
{
    // 1. 计算全局偏差
    float dx = target->x - x;
    float dy = target->y - y;

    // 2. 转换到车身坐标系
    float x_local = dx * cosf(yaw) + dy * sinf(yaw);
    float y_local = -dx * sinf(yaw) + dy * cosf(yaw);

    // 3. 计算前视距离 (Ld)
    float ld = sqrtf(x_local * x_local + y_local * y_local);

    // 防止除以零，且防止距离过近导致的剧烈转向
    if(ld < 0.1f) return 0.0f;

    // 4. 计算曲率 (k = 2 * y / Ld^2)
    float curvature = 2.0f * y_local / (ld * ld);

    // 5. 计算转向角
    float steer_rad = atanf(curvature * TRACK_WHEELBASE);
    float steer_deg = steer_rad * 57.2957795130823f; // 转角度

    // 倒车时反转转向方向
    if(target->speed_dir < 0.5f)
    {
        steer_deg = -steer_deg;
    }

    // 6. 增加死区控制
    // 如果角度在 -2 到 2 度之间，认为是直行，消除抖动
    if (steer_deg > -2.0f && steer_deg < 2.0f)
    {
        steer_deg = 0.0f;
    }

    // 7. 限幅
    if(steer_deg > 20.0f) steer_deg = 20.0f;
    if(steer_deg < -20.0f) steer_deg = -20.0f;

    return steer_deg;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     准备指定索引的点数据
//  参数说明      abs_index       绝对点索引（1开始）
//  返回参数      uint8           1-成功 0-失败
//  备注信息      自动切换页并更新索引
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
//  函数简介     寻找前视点
//  参数说明      x, y            当前位置
//  返回参数      uint8           1-找到 0-未找到
//  备注信息      从当前点开始向前寻找距离大于前视距离的点，不回绕到起点
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

    // 先检查当前索引是否有效
    if(!track_follow_prepare_point(follow_abs_index))
    {
        follow_abs_index = 1;
        if(!track_follow_prepare_point(follow_abs_index))
        {
            return 0;
        }
    }

    // 从当前索引开始搜索
    uint32 last_same_dir_index = follow_abs_index;
    Ins_follow last_same_dir_point;
    track_flash_get_point(follow_point_idx, &last_same_dir_point);

    while(follow_abs_index <= search_end)
    {
        Ins_follow pt;
        if(track_flash_get_point(follow_point_idx, &pt))
        {
            // 检查是否与当前方向相同
            if((pt.speed_dir >= 0.5f && current_speed_dir >= 0.5f) ||
               (pt.speed_dir < 0.5f && current_speed_dir < 0.5f))
            {
                // 方向相同，更新最后一个同方向点
                last_same_dir_index = follow_abs_index;
                last_same_dir_point = pt;

                // 检查距离是否足够
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
                // 方向不同，使用最后一个同方向的点
                Ins_date_377.x   = last_same_dir_point.x;
                Ins_date_377.y   = last_same_dir_point.y;
                Ins_date_377.yaw = last_same_dir_point.yaw;
                Ins_date_377.speed_dir = last_same_dir_point.speed_dir;

                // 更新索引到最后一个同方向点
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

    // 搜索完所有点都没找到，使用最后一个点
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
//  函数简介     循迹处理
//  参数说明      void
//  返回参数      void
//  备注信息      执行Pure Pursuit循迹算法，查找失败时保持上一帧转向角并降速
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
        servo_set_angle(90.0f + steer_output_filtered);
        wheel_pid_set_target_speed(TRACK_FOLLOW_SPEED * 0.5f);
        return;
    }

    // 计算到前视点的距离
    float dx = Ins_date_377.x - state->x;
    float dy = Ins_date_377.y - state->y;
    float dist_to_lookahead = sqrtf(dx * dx + dy * dy);

    // 获取总点数
    uint32 stored_points = track_get_stored_point_count();

    // 如果已经接近前视点，向前推进索引
    if(dist_to_lookahead < 0.05f)
    {
        if(follow_abs_index < stored_points)
        {
            // 检查下一个点是否方向不同
            uint32 next_index = follow_abs_index + 1;
            if(track_follow_prepare_point(next_index))
            {
                Ins_follow next_point;
                if(track_flash_get_point(follow_point_idx, &next_point))
                {
                    // 如果下一个点方向不同，切换方向
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
            // 到达最终点，停止循迹
            track_stop_follow();
            return;
        }
    }

    Ins_follow target = {Ins_date_377.x, Ins_date_377.y, Ins_date_377.yaw};
    steer_output = pure_pursuit_calc_steer(state->x, state->y, state->yaw, &target);

    steer_output_filtered = STEER_FILTER_ALPHA * steer_output +
                            (1.0f - STEER_FILTER_ALPHA) * steer_output_filtered;

    // 根据路径点的速度方向设置行驶方向
    float target_speed = TRACK_FOLLOW_SPEED;
    if(Ins_date_377.speed_dir < 0.5f) // speed_dir < 0.5表示倒车
    {
        target_speed = -TRACK_FOLLOW_SPEED;
    }

    servo_set_angle(90.0f + steer_output_filtered);
    wheel_pid_set_target_speed(target_speed);
}

//-------------------------------------------函数定义区-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     初始化存点模块
//  参数说明      void
//  返回参数      void
//  使用示例      track_init();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void track_init(void)
{
    track_flash_cur_write_page = Track_Flash_Page_Begin;

    dist_acc_m = 0.0f;                                        // 里程累计清零
    flash_point_index = 0;                                    // 写入索引清零

    flash_buffer_clear();                                     // 清空flash_union_buffer（全部置0xFF，防止脏数据）

    memset(Ins_Date_Read, 0, sizeof(Ins_Date_Read));          // 清空读取缓冲区

    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    follow_abs_index = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    current_speed_dir = 1.0f; // 初始方向为前进

    track_save_flag = 0;
    track_follow_flag = 0;
    track_total_points = 0;

    track_meta_load();
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     按照固定里程存入一个点
//  使用示例      track_flash_push_point();
//  @note        返回1表示成功写入，0表示本周期未到采样里程
//               缓存满一页后自动整页写flash并轮询到下一页
//-------------------------------------------------------------------------------------------------------------------
uint8 track_flash_push_point(void)
{
    const EncoderLayerState* state = encoder_layer_get_state();

    if(state != NULL)
    {
        float ds = 0.5f * (state->delta_right_m + state->delta_left_m);
        dist_acc_m += fabsf(ds);                              // 累计行驶里程
    }

    // 没到固定里程，不写点
    if(dist_acc_m < TRACK_SAMPLE_STEP)                        // 如果累计里程不足一个采样步长
    {
        return 0;
    }
    dist_acc_m -= TRACK_SAMPLE_STEP;                          // 保留余量，长期误差更小

    // 越界保护：缓存满一页，先写入flash再切页清空
    if(flash_point_index + (int16_t)Track_Flash_Float_Num > (int16_t)Track_Flash_Page_Float_Num)
    {
        track_flash_cache_flush(track_flash_cur_write_page);
        track_meta_save();
        track_flash_add();
        flash_point_index = 0;
        flash_buffer_clear();
    }

    // 将x, y, yaw, speed_dir写入flash_union_buffer
    const INS_State* ins_state = Ins_get_state();
    const EncoderLayerState* enc_state = encoder_layer_get_state();
    flash_data_union tmp;

    // 存储位置和航向角
    tmp.float_type = ins_state->x;
    flash_union_buffer[flash_point_index] = tmp;              // 将x写入flash_union_buffer
    tmp.float_type = ins_state->y;
    flash_union_buffer[flash_point_index + 1] = tmp;          // 将y写入flash_union_buffer
    tmp.float_type = ins_state->yaw;
    flash_union_buffer[flash_point_index + 2] = tmp;          // 将yaw写入flash_union_buffer

    // 存储速度方向：1表示前进，0表示倒车
    float speed_dir = 1.0f; // 默认前进
    if(enc_state != NULL && enc_state->speed_average_mps < -0.05f) // 速度为负表示倒车
    {
        speed_dir = 0.0f;
    }
    tmp.float_type = speed_dir;
    flash_union_buffer[flash_point_index + 3] = tmp;          // 将speed_dir写入flash_union_buffer

    flash_point_index += 4;
    if(track_total_points < ((uint32)Track_Flash_Page_Max * (uint32)Track_Flash_Point_Page_Max))
    {
        track_total_points++;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     将当前flash_union_buffer写入指定页（对外接口）
//  参数说明      page_num         目标页号
//  返回参数      void
//  使用示例      track_flash_write_cache(1);
//  备注信息      不改变轮询页号，只执行一次指定页写入
//-------------------------------------------------------------------------------------------------------------------
void track_flash_write_cache(uint32 page_num)
{
    track_flash_cache_flush(page_num);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     从指定页读取一整页原始数据到读取缓冲区 Ins_Date_Read
//  参数说明      page_num         目标页号
//  返回参数      void
//  使用示例      track_flash_read_page(1);
//  备注信息      读取完成后调用 track_flash_get_point(index, &out) 按序号解析任意点
//-------------------------------------------------------------------------------------------------------------------
void track_flash_read_page(uint32 page_num)
{
    if((page_num < Track_Flash_Page_Begin) || (page_num >= (Track_Flash_Page_Begin + Track_Flash_Page_Max)))
    {
        return;
    }

    flash_read_page(0, page_num, Ins_Date_Read, 510);         // 将整页数据读入读取缓冲区
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     从读取缓冲区中解析指定序号的点
//  参数说明      point_index      第几个点，1-170
//  参数说明      out              输出结构体指针
//  返回参数      uint8            1-成功 0-越界
//  使用示例      Ins_follow follow; track_flash_get_point(5, &follow);
//  备注信息      需先调用track_flash_read_page读取对应页
//-------------------------------------------------------------------------------------------------------------------
uint8 track_flash_get_point(uint32 point_index, Ins_follow* out)
{
    uint32 base = 0;
    flash_data_union data_temp;                               // 临时变量，用于类型转换

    if(out == NULL || point_index < 1 || point_index > Track_Flash_Point_Page_Max)
    {
        return 0;
    }

    base = (point_index - 1) * Track_Flash_Float_Num;          // 第point_index个点的起始地址为(point_index-1)*4

    data_temp.uint32_type = Ins_Date_Read[base + 0];          // 将x从uint32转换为float
    out->x = data_temp.float_type;                            // 将x赋值给out->x

    data_temp.uint32_type = Ins_Date_Read[base + 1];          // 将y从uint32转换为float
    out->y = data_temp.float_type;                            // 将y赋值给out->y

    data_temp.uint32_type = Ins_Date_Read[base + 2];          // 将yaw从uint32转换为float
    out->yaw = data_temp.float_type;                          // 将yaw赋值给out->yaw

    data_temp.uint32_type = Ins_Date_Read[base + 3];          // 将speed_dir从uint32转换为float
    out->speed_dir = data_temp.float_type;                    // 将speed_dir赋值给out->speed_dir

    // 检查是否为Flash擦除值
    if(Ins_Date_Read[base + 0] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 1] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 2] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 3] == 0xFFFFFFFFu)
    {
        return 0;
    }

    // 检查是否为0值（可能是未初始化）
    if(Ins_Date_Read[base + 0] == 0x00000000u &&
       Ins_Date_Read[base + 1] == 0x00000000u &&
       Ins_Date_Read[base + 2] == 0x00000000u &&
       Ins_Date_Read[base + 3] == 0x00000000u)
    {
        return 0;
    }

    // 检查浮点数有效性
    if(!track_float_is_valid(out->x, TRACK_MAX_COORD_ABS) ||
       !track_float_is_valid(out->y, TRACK_MAX_COORD_ABS) ||
       !track_float_is_valid(out->yaw, TRACK_MAX_YAW_ABS_RAD) ||
       !track_float_is_valid(out->speed_dir, 1.0f))
    {
        return 0;
    }

    // 检查坐标是否合理（防止过大的数值）
    if(fabsf(out->x) > 1000.0f || fabsf(out->y) > 1000.0f)
    {
        return 0;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     清空所有存点页
//  参数说明      void
//  返回参数      void
//  使用示例      track_flash_clear_all();
//  备注信息      会擦除配置的全部存点页
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
//  函数简介     获取当前轮询写页
//  参数说明      void
//  返回参数      uint32           当前轮询写页号
//  使用示例      track_flash_get_cur_write_page();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 track_flash_get_cur_write_page(void)
{
    return track_flash_cur_write_page;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获取当前已写入buffer的float数量（每3个float为一个点）
//  参数说明      void
//  返回参数      int16_t          当前flash_point_index值
//  使用示例      track_flash_get_point_index();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
int16_t track_flash_get_point_index(void)
{
    return flash_point_index;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     结束记录，将当前未满页的缓存强制写入Flash
//  参数说明      void
//  返回参数      void
//  使用示例      track_flash_finish();
//  备注信息      停止记录时必须调用，否则最后若干点会丢失
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
//  函数简介     清除所有轨迹数据
//  参数说明      void
//  返回参数      void
//  使用示例      track_clear();
//  备注信息      擦除所有存点页并重置状态
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
    servo_set_angle(90.0f);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     开始存点
//  参数说明      void
//  返回参数      void
//  使用示例      track_start_save();
//  备注信息      重置状态并开始存点
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
//  函数简介     停止存点
//  参数说明      void
//  返回参数      void
//  使用示例      track_stop_save();
//  备注信息      停止存点并将缓存写入Flash
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
//  函数简介     开始循迹
//  参数说明      void
//  返回参数      void
//  使用示例      track_start_follow();
//  备注信息      从第一页开始循迹
//-------------------------------------------------------------------------------------------------------------------
void track_start_follow(void)
{
    if(track_get_stored_point_count() == 0)
    {
        track_follow_flag = 0;
        wheel_pid_set_target_speed(0.0f);
        servo_set_angle(90.0f);
        return;
    }

    // 重置INS位置，确保从原点开始
    Ins_reset(0.0f, 0.0f, 0.0f);

    // 重置循迹状态
    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    follow_abs_index = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    current_speed_dir = 1.0f; // 初始方向为前进

    // 确保Flash数据读取完成
    track_flash_read_page(follow_page);

    // 验证第一页数据是否有效
    Ins_follow first_point;
    if(!track_flash_get_point(1, &first_point))
    {
        track_follow_flag = 0;
        wheel_pid_set_target_speed(0.0f);
        servo_set_angle(90.0f);
        return;
    }

    // 初始化Ins_date_377的speed_dir字段
    Ins_date_377.speed_dir = first_point.speed_dir;
    current_speed_dir = first_point.speed_dir;

    // 立即执行一次寻点，确保Tag X在开始移动前就更新
    const INS_State* state = Ins_get_state();
    if(state != NULL)
    {
        // 连续尝试3次寻点，确保找到有效点
        for(int i = 0; i < 3; i++)
        {
            if(find_lookahead_point(state->x, state->y))
            {
                break;
            }
            // 短暂延迟后重试
            for(int j = 0; j < 1000; j++)
            {
                __asm__ volatile ("nop");
            }
        }
    }

    // 确保舵机回到中心位置
    servo_set_angle(90.0f);

    // 设置初始速度
    wheel_pid_set_target_speed(TRACK_FOLLOW_SPEED);

    // 切换到循迹模式
    track_save_flag = 0;
    track_follow_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     将所有存储的轨迹点发送到上位机
//  参数说明      void
//  返回参数      void
//  使用示例      track_send_all_points_to_host();
//  备注信息      发送格式: TrackPoint:index,x,y,yaw\r\n
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
//  函数简介     停止循迹
//  参数说明      void
//  返回参数      void
//  使用示例      track_stop_follow();
//  备注信息      停止循迹并停车
//-------------------------------------------------------------------------------------------------------------------
void track_stop_follow(void)
{
    track_follow_flag = 0;
    wheel_pid_set_target_speed(0.0f);
    servo_set_angle(90.0f);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     轨迹模块任务处理
//  参数说明      void
//  返回参数      void
//  使用示例      track_proc();
//  备注信息      周期调用，执行存点或循迹
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
