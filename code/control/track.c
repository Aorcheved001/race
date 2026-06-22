/*
 * track.c
 *
 *  Created on: 2024年6月6日
 *      Author: LateRain
 *  Modified: 2025年11月22日
 */

//-------------------------------------------头文件引用------------------------------------------------------------
#include "track.h"
#include "Ins.h"
#include "zf_driver_flash.h"
#include "zf_device_ips200.h"
#include "steering_control.h"

//-------------------------------------------结构体变量定义------------------------------------------------------------
Ins_Date Ins_date_377 = {0};

//-------------------------------------------全局变量定义-----------------------------------------------------------
uint8 track_save_flag = 0;
uint8 track_follow_flag = 0;
uint8 track_follow_reverse_flag = 0;     // 反向循迹标志
uint32 track_total_points = 0;

//-------------------------------------------内部变量定义-----------------------------------------------------------
static uint32 track_flash_cur_write_page = Track_Flash_Page_Begin;
// 当前轮询写入的页号

static float dist_acc_m = 0.0f;
// 内部里程累计距离缓存

static int16_t flash_point_index = 0;
// 当前在flash_union_buffer的第几个元素（按float计）

static uint32 Ins_Date_Read[510] = {0};
// 读取数据缓存数组（uint32，和flash_read_page接口一致）

static uint16 follow_page = Track_Flash_Page_Begin;
// 循迹时当前读取的页号

static uint16 follow_point_idx = 1;
// 循迹时当前读取的点序号

static uint32 follow_abs_index = 1;
// 循迹时当前的绝对点序号（1-track_total_points）

static float steer_output = 0.0f;
// 转向输出

static float steer_output_filtered = 0.0f;
// 转向滤波输出

static float current_speed_dir = 1.0f; // 当前行驶方向，1表示前进，0表示后退

static float s_yaw_offset = 0.0f;    // yaw偏移量（旋转后-旋转前），循迹时补偿180°旋转用

#define STEER_FILTER_ALPHA    0.85f
// 转向一阶低通滤波系数，越大越接近原始值

#define TRACK_META_PAGE                0u
#define TRACK_META_MAGIC               0x5452434Bu
#define TRACK_META_VERSION             1u
#define TRACK_MAX_COORD_ABS            1000000.0f
#define TRACK_MAX_YAW_ABS_RAD          3.5f

//-------------------------------------------内部函数声明-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  函数名     获取已存储的点数（带边界保护）
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
//  函数名     从flash_union_buffer把页写入指定flash页
//  参数说明      page_num         目标页号
//  返回参数      void
//  使用示例      track_flash_cache_flush(1);
//  备注信息      直接将flash_union_buffer写入，调用前确保已填充好buffer
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
//  函数名     页号轮询加
//  参数说明      void
//  返回参数      void
//  使用示例      track_flash_add();
//  备注信息      超过最后一页回到起始页
//-------------------------------------------------------------------------------------------------------------------
static void track_flash_add(void)
{
    track_flash_cur_write_page++;                            // 轮询加

    if(track_flash_cur_write_page >= (Track_Flash_Page_Begin + Track_Flash_Page_Max))
    {
        track_flash_cur_write_page = Track_Flash_Page_Begin; // 超过最后一页回到起始页
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     Pure Pursuit计算转向角
//  参数说明      x, y, yaw       当前位置和航向角
//  参数说明      target          目标点
//  返回参数      float           转向角（度）
//  备注信息      基于Pure Pursuit算法计算转向角
//-------------------------------------------------------------------------------------------------------------------
static float pure_pursuit_calc_steer(float x, float y, float yaw, Ins_follow* target)
{
    // 1. 计算全局偏移量
    float dx = target->x - x;
    float dy = target->y - y;

    // 2. 转换到车辆坐标系
    float x_local = dx * cosf(yaw) + dy * sinf(yaw);
    float y_local = -dx * sinf(yaw) + dy * cosf(yaw);

    // 3. 计算前视距离 (Ld)
    float ld = sqrtf(x_local * x_local + y_local * y_local);

    // 防止除零，并防止近距离导致的巨大转向
    if(ld < 0.1f) return 0.0f;

    // 4. 计算曲率 (k = 2 * y / Ld^2)
    float curvature = 2.0f * y_local / (ld * ld);

    // 5. 计算转向角
    float steer_rad = atanf(curvature * TRACK_WHEELBASE);
    float steer_deg = steer_rad * 57.2957795130823f; // 转角度

    // 倒车时转向取反
    if(target->speed_dir < 0.5f)
    {
        steer_deg = -steer_deg;
    }

    // 6. 小角度死区处理
    // 当角度在 -2 到 2 度之间，认为是直行，不做处理
    if (steer_deg > -2.0f && steer_deg < 2.0f)
    {
        steer_deg = 0.0f;
    }

    // 7. 限幅
    if(steer_deg > VEHICLE_PP_MAX_STEER_DEG) steer_deg = VEHICLE_PP_MAX_STEER_DEG;
    if(steer_deg < -VEHICLE_PP_MAX_STEER_DEG) steer_deg = -VEHICLE_PP_MAX_STEER_DEG;

    return steer_deg;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     准备指定序号的点数据
//  参数说明      abs_index       绝对点序号（从1开始）
//  返回参数      uint8           1-成功 0-失败
//  备注信息      自动切换页并加载数据
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
//  函数名     寻找前视点
//  参数说明      x, y            当前位置
//  返回参数      uint8           1-找到 0-未找到
//  备注信息      从当前点开始向前寻找距离大于前视距离的点，并更新索引
//-------------------------------------------------------------------------------------------------------------------
static uint8 find_lookahead_point(float x, float y)
{
    uint32 stored_points = track_get_stored_point_count();
    if(stored_points == 0)
    {
        return 0;
    }

    // 使用局部变量进行索引搜索，避免污染全局索引
    uint32 local_abs_index = follow_abs_index;

    if(local_abs_index < 1 || local_abs_index > stored_points)
    {
        local_abs_index = 1;
    }

    uint32 search_end = stored_points;
    uint32 saved_start_index = local_abs_index;  // 保存起始索引，用于失败时恢复

    // 先检查当前点是否有效
    if(!track_follow_prepare_point(local_abs_index))
    {
        local_abs_index = 1;
        if(!track_follow_prepare_point(local_abs_index))
        {
            return 0;  // 失败时全局索引不变
        }
    }

    // 从当前点开始搜索
    uint32 last_same_dir_index = local_abs_index;
    Ins_follow last_same_dir_point;
    
    // 检查返回值，避免使用未初始化数据
    if(!track_flash_get_point(follow_point_idx, &last_same_dir_point))
    {
        // 当前点读取失败，尝试从第一个点开始
        local_abs_index = 1;
        if(!track_follow_prepare_point(local_abs_index) || 
           !track_flash_get_point(follow_point_idx, &last_same_dir_point))
        {
            return 0;  // 失败时全局索引不变
        }
    }

    while(local_abs_index <= search_end)
    {
        Ins_follow pt;
        if(track_flash_get_point(follow_point_idx, &pt))
        {
            // 检查是否与当前方向相同
            if((pt.speed_dir >= 0.5f && current_speed_dir >= 0.5f) ||
               (pt.speed_dir < 0.5f && current_speed_dir < 0.5f))
            {
                // 方向相同，记录最后一个同向点
                last_same_dir_index = local_abs_index;
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

                    // 找到有效点，更新全局索引
                    follow_abs_index = local_abs_index;
                    return 1;
                }
            }
            else
            {
                // 方向不同，使用最后一个同向的点
                Ins_date_377.x   = last_same_dir_point.x;
                Ins_date_377.y   = last_same_dir_point.y;
                Ins_date_377.yaw = last_same_dir_point.yaw;
                Ins_date_377.speed_dir = last_same_dir_point.speed_dir;

                // 更新全局索引为最后一个同向点
                follow_abs_index = last_same_dir_index;
                return 1;
            }
        }

        local_abs_index++;
        if(local_abs_index > search_end)
        {
            break;
        }

        if(!track_follow_prepare_point(local_abs_index))
        {
            continue;
        }
    }

    // 如果所有点都没找到，使用最后一个点
    if(track_follow_prepare_point(stored_points))
    {
        Ins_follow pt;
        if(track_flash_get_point(follow_point_idx, &pt))
        {
            Ins_date_377.x   = pt.x;
            Ins_date_377.y   = pt.y;
            Ins_date_377.yaw = pt.yaw;
            Ins_date_377.speed_dir = pt.speed_dir;

            // 更新全局索引到最后一个点
            follow_abs_index = stored_points;
            return 1;
        }
    }

    // 所有查找都失败，全局索引保持不变
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     循迹任务
//  参数说明      void
//  返回参数      void
//  备注信息      执行Pure Pursuit循迹算法，丢失时保持上一帧转向角不突变
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
            // 检查下一个点是否同向
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
            // 已经到终点，停止循迹
            track_stop_follow();
            return;
        }
    }

    // [P7-4-2] 补全 speed_dir 字段，传递给 pure_pursuit_calc_steer
    Ins_follow target = {Ins_date_377.x, Ins_date_377.y, Ins_date_377.yaw, Ins_date_377.speed_dir};
    steer_output = pure_pursuit_calc_steer(state->x, state->y, state->yaw, &target);

    steer_output_filtered = STEER_FILTER_ALPHA * steer_output +
                            (1.0f - STEER_FILTER_ALPHA) * steer_output_filtered;

    // 根据路径方向，速度方向与行驶方向一致
    float target_speed = TRACK_FOLLOW_SPEED;
    if(Ins_date_377.speed_dir < 0.5f) // speed_dir < 0.5表示倒车
    {
        target_speed = -TRACK_FOLLOW_SPEED;
    }

    steering_set_target(steer_output_filtered);
    wheel_pid_set_target_speed(target_speed);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     设置yaw偏移量
//  参数说明      offset_rad      yaw偏移量（弧度），旋转后航向-旋转前航向
//  返回参数      void
//  使用示例      track_set_yaw_offset(3.14f);
//  备注信息      用于停车区180°旋转后的yaw补偿，需在反向循迹前调用
//-------------------------------------------------------------------------------------------------------------------
void track_set_yaw_offset(float offset_rad)
{
    s_yaw_offset = offset_rad;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     反向寻找前视点
//  参数说明      x, y            当前位置
//  返回参数      uint8           1-找到 0-未找到
//  备注信息      从当前索引往递减方向搜索轨迹点，找到距离大于前视距离的点
//-------------------------------------------------------------------------------------------------------------------
static uint8 find_lookahead_point_reverse(float x, float y)
{
    uint32 stored_points = track_get_stored_point_count();
    if(stored_points == 0)
    {
        return 0;
    }

    // 使用局部变量进行索搜索，避免污染全局索引
    uint32 local_abs_index = follow_abs_index;

    if(local_abs_index < 1 || local_abs_index > stored_points)
    {
        local_abs_index = stored_points;
    }

    // 反向搜索：从 local_abs_index 递减到 1
    uint32 search_end = 1;

    // 先检查当前点是否有效
    if(!track_follow_prepare_point(local_abs_index))
    {
        local_abs_index = stored_points;
        if(!track_follow_prepare_point(local_abs_index))
        {
            return 0;
        }
    }

    // 从当前点开始反向搜索
    uint32 last_same_dir_index = local_abs_index;
    Ins_follow last_same_dir_point;

    if(!track_flash_get_point(follow_point_idx, &last_same_dir_point))
    {
        // 当前点读取失败，尝试从最后一个点开始
        local_abs_index = stored_points;
        if(!track_follow_prepare_point(local_abs_index) ||
           !track_flash_get_point(follow_point_idx, &last_same_dir_point))
        {
            return 0;
        }
    }

    while(local_abs_index >= search_end)
    {
        Ins_follow pt;
        if(track_flash_get_point(follow_point_idx, &pt))
        {
            // 检查是否与当前方向相同
            if((pt.speed_dir >= 0.5f && current_speed_dir >= 0.5f) ||
               (pt.speed_dir < 0.5f && current_speed_dir < 0.5f))
            {
                // 方向相同，记录最后一个同向点（反向遍历时是索引最小的）
                last_same_dir_index = local_abs_index;
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

                    // 找到有效点，更新全局索引
                    follow_abs_index = local_abs_index;
                    return 1;
                }
            }
            else
            {
                // 方向不同，使用最后一个同向的点
                Ins_date_377.x   = last_same_dir_point.x;
                Ins_date_377.y   = last_same_dir_point.y;
                Ins_date_377.yaw = last_same_dir_point.yaw;
                Ins_date_377.speed_dir = last_same_dir_point.speed_dir;

                // 更新全局索引为最后一个同向点
                follow_abs_index = last_same_dir_index;
                return 1;
            }
        }

        if(local_abs_index <= search_end)
        {
            break;
        }
        local_abs_index--;

        if(!track_follow_prepare_point(local_abs_index))
        {
            continue;
        }
    }

    // 如果所有点都没找到，使用第一个点
    if(track_follow_prepare_point(1))
    {
        Ins_follow pt;
        if(track_flash_get_point(follow_point_idx, &pt))
        {
            Ins_date_377.x   = pt.x;
            Ins_date_377.y   = pt.y;
            Ins_date_377.yaw = pt.yaw;
            Ins_date_377.speed_dir = pt.speed_dir;

            // 更新全局索引到第一个点
            follow_abs_index = 1;
            return 1;
        }
    }

    // 所有查找都失败，全局索引保持不变
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     反向循迹任务
//  参数说明      void
//  返回参数      void
//  备注信息      执行Pure Pursuit反向循迹算法，使用yaw偏移补偿
//-------------------------------------------------------------------------------------------------------------------
static void track_follow_reverse(void)
{
    const INS_State* state = Ins_get_state();

    if(state == NULL)
    {
        track_stop_follow();
        return;
    }

    // yaw偏移补偿：还原到录制时的坐标系
    // [P7-4-3] 注: x/y 为全局坐标，无需偏航补偿；yaw 需要补偿以匹配录制坐标系
    float yaw_compensated = state->yaw - s_yaw_offset;

    if(!find_lookahead_point_reverse(state->x, state->y))
    {
        steering_set_target(steer_output_filtered);
        wheel_pid_set_target_speed(TRACK_FOLLOW_SPEED * 0.5f);
        return;
    }

    // 计算到前视点的距离
    float dx = Ins_date_377.x - state->x;
    float dy = Ins_date_377.y - state->y;
    float dist_to_lookahead = sqrtf(dx * dx + dy * dy);

    // 如果已经接近前视点，向后推进索引（递减）
    if(dist_to_lookahead < 0.05f)
    {
        if(follow_abs_index > 1)
        {
            follow_abs_index--;
            // 检查下一个点是否同向
            if(track_follow_prepare_point(follow_abs_index))
            {
                Ins_follow next_point;
                if(track_flash_get_point(follow_point_idx, &next_point))
                {
                    if((next_point.speed_dir >= 0.5f && current_speed_dir < 0.5f) ||
                       (next_point.speed_dir < 0.5f && current_speed_dir >= 0.5f))
                    {
                        current_speed_dir = next_point.speed_dir;
                    }
                }
            }
        }
        else
        {
            // 已经到终点（第一个点），停止循迹
            track_stop_follow();
            return;
        }
    }

    // [P7-4-2] 补全 speed_dir 字段，传递给 pure_pursuit_calc_steer
    Ins_follow target = {Ins_date_377.x, Ins_date_377.y, Ins_date_377.yaw, Ins_date_377.speed_dir};
    // 使用补偿后的yaw计算转向角
    steer_output = pure_pursuit_calc_steer(state->x, state->y, yaw_compensated, &target);

    steer_output_filtered = STEER_FILTER_ALPHA * steer_output +
                            (1.0f - STEER_FILTER_ALPHA) * steer_output_filtered;

    // 根据路径方向，速度方向与行驶方向一致
    float target_speed = TRACK_FOLLOW_SPEED;
    if(Ins_date_377.speed_dir < 0.5f) // speed_dir < 0.5表示倒车
    {
        target_speed = -TRACK_FOLLOW_SPEED;
    }

    steering_set_target(steer_output_filtered);
    wheel_pid_set_target_speed(target_speed);
}

//-------------------------------------------外部函数定义-----------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  函数名     初始化轨迹模块
//  参数说明      void
//  返回参数      void
//  使用示例      track_init();
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void track_init(void)
{
    track_flash_cur_write_page = Track_Flash_Page_Begin;

    dist_acc_m = 0.0f;                                        // 重置累计里程
    flash_point_index = 0;                                    // 写入索引清零

    flash_buffer_clear();                                     // 将flash_union_buffer全部置0xFF（防止错误数据）

    memset(Ins_Date_Read, 0, sizeof(Ins_Date_Read));          // 清空读取缓存区

    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    follow_abs_index = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    current_speed_dir = 1.0f; // 初始化为前进

    track_save_flag = 0;
    track_follow_flag = 0;
    track_follow_reverse_flag = 0;
    track_total_points = 0;

    track_meta_load();
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     按固定里程存储一个点
//  使用示例      track_flash_push_point();
//  @note        返回1表示成功写入，0表示距离未达标或未开始
//               超过一页会自动换页写flash并轮询到下一页
//-------------------------------------------------------------------------------------------------------------------
uint8 track_flash_push_point(void)
{
    const EncoderLayerState* state = encoder_layer_get_state();

    if(state != NULL)
    {
        float ds = 0.5f * (state->delta_right_m + state->delta_left_m);
        dist_acc_m += fabsf(ds);                              // 累计行驶里程
    }

    // 没达到固定里程，不写入
    if(dist_acc_m < TRACK_SAMPLE_STEP)                        // 如果累计里程不足一个采样间距
    {
        return 0;
    }
    dist_acc_m -= TRACK_SAMPLE_STEP;                          // 减去一个采样间距，保持余量

    // 越界保护，超过一页先写入flash，再轮询页
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

    // 存储速度方向，1表示前进，0表示后退
    float speed_dir = 1.0f; // 默认前进
    if(enc_state != NULL && enc_state->speed_average_mps < -0.05f) // 速度为负表示后退
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
//  函数名     将当前flash_union_buffer写入指定页（外部接口）
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
//  函数名     从指定页读取一页原始数据到读取缓存区 Ins_Date_Read
//  参数说明      page_num         目标页号
//  返回参数      void
//  使用示例      track_flash_read_page(1);
//  备注信息      读取完成后可调用 track_flash_get_point(index, &out) 来解析得到的数据
//-------------------------------------------------------------------------------------------------------------------
void track_flash_read_page(uint32 page_num)
{
    if((page_num < Track_Flash_Page_Begin) || (page_num >= (Track_Flash_Page_Begin + Track_Flash_Page_Max)))
    {
        return;
    }

    flash_read_page(0, page_num, Ins_Date_Read, 510);         // 将页数据全部读到缓存区
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     从读取缓存区中解析指定序号的点
//  参数说明      point_index      第几个点，1-170
//  参数说明      out              输出结构体指针
//  返回参数      uint8            1-成功 0-越界
//  使用示例      Ins_follow follow; track_flash_get_point(5, &follow);
//  备注信息      需先调用track_flash_read_page读取对应页
//-------------------------------------------------------------------------------------------------------------------
uint8 track_flash_get_point(uint32 point_index, Ins_follow* out)
{
    uint32 base = 0;
    flash_data_union data_temp;                               // 临时变量，用于数据类型转换

    if(out == NULL || point_index < 1 || point_index > Track_Flash_Point_Page_Max)
    {
        return 0;
    }

    base = (point_index - 1) * Track_Flash_Float_Num;          // 第point_index点的起始地址为(point_index-1)*4

    data_temp.uint32_type = Ins_Date_Read[base + 0];          // 将x从uint32转换为float
    out->x = data_temp.float_type;                            // 将x赋值给out->x

    data_temp.uint32_type = Ins_Date_Read[base + 1];          // 将y从uint32转换为float
    out->y = data_temp.float_type;                            // 将y赋值给out->y

    data_temp.uint32_type = Ins_Date_Read[base + 2];          // 将yaw从uint32转换为float
    out->yaw = data_temp.float_type;                          // 将yaw赋值给out->yaw

    data_temp.uint32_type = Ins_Date_Read[base + 3];          // 将speed_dir从uint32转换为float
    out->speed_dir = data_temp.float_type;                    // 将speed_dir赋值给out->speed_dir

    // 检查是否为Flash擦除值（Flash擦除后为0xFFFFFFFF）
    if(Ins_Date_Read[base + 0] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 1] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 2] == 0xFFFFFFFFu ||
       Ins_Date_Read[base + 3] == 0xFFFFFFFFu)
    {
        return 0;
    }

    // 注意：不检查全0值，因为 (0, 0, 0, 0) 是合法的原点倒车点
    // speed_dir=0 表示倒车，x=0, y=0, yaw=0 表示原点朝向0度

    // 检查浮点数有效性
    if(!track_float_is_valid(out->x, TRACK_MAX_COORD_ABS) ||
       !track_float_is_valid(out->y, TRACK_MAX_COORD_ABS) ||
       !track_float_is_valid(out->yaw, TRACK_MAX_YAW_ABS_RAD) ||
       !track_float_is_valid(out->speed_dir, 1.0f))
    {
        return 0;
    }

    // 检查数值是否过大，防止错误数据
    if(fabsf(out->x) > 1000.0f || fabsf(out->y) > 1000.0f)
    {
        return 0;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     清除所有数据页
//  参数说明      void
//  返回参数      void
//  使用示例      track_flash_clear_all();
//  备注信息      清除所有用到的数据页
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
//  函数名     获取当前轮询写页
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
//  函数名     获取当前已写入buffer的float数量（每4个float为一个点）
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
//  函数名     停止记录时将当前未写页的缓存强制写入Flash
//  参数说明      void
//  返回参数      void
//  使用示例      track_flash_finish();
//  备注信息      停止记录时调用，否则最后一段会丢失
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
//  函数名     清除所有轨迹数据
//  参数说明      void
//  返回参数      void
//  使用示例      track_clear();
//  备注信息      清除所有数据页并重置状态
//-------------------------------------------------------------------------------------------------------------------
void track_clear(void)
{
    track_flash_clear_all();

    track_save_flag = 0;
    track_follow_flag = 0;
    track_follow_reverse_flag = 0;
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
//  函数名     开始记录
//  参数说明      void
//  返回参数      void
//  使用示例      track_start_save();
//  备注信息      重置状态并开始记录
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
    track_follow_reverse_flag = 0;
    track_save_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     停止记录
//  参数说明      void
//  返回参数      void
//  使用示例      track_stop_save();
//  备注信息      停止记录并把数据写入Flash
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
//  函数名     开始循迹
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
        steering_set_target(0.0f);
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
    current_speed_dir = 1.0f; // 初始化为前进

    // 确保Flash数据读取完成
    track_flash_read_page(follow_page);

    // 验证第一页数据是否有效
    Ins_follow first_point;
    if(!track_flash_get_point(1, &first_point))
    {
        track_follow_flag = 0;
        wheel_pid_set_target_speed(0.0f);
        steering_set_target(0.0f);
        return;
    }

    // 初始化Ins_date_377的speed_dir字段
    Ins_date_377.speed_dir = first_point.speed_dir;
    current_speed_dir = first_point.speed_dir;

    // 尝试执行一次寻点，确保Tag X在开始移动前就更新
    const INS_State* state = Ins_get_state();
    if(state != NULL)
    {
        // 尝试最多3次寻点，确保找到有效点
        for(int i = 0; i < 3; i++)
        {
            if(find_lookahead_point(state->x, state->y))
            {
                break;
            }
            // 简单的延迟等待
            for(int j = 0; j < 1000; j++)
            {
                __asm__ volatile ("nop");
            }
        }
    }

    // 确保转向回到中位
    steering_set_target(0.0f);

    // 设置初始速度
    wheel_pid_set_target_speed(TRACK_FOLLOW_SPEED);

    // 切换到循迹模式
    track_save_flag = 0;
    track_follow_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     将已存储的轨迹点发送到上位机
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
//  函数名     停止循迹
//  参数说明      void
//  返回参数      void
//  使用示例      track_stop_follow();
//  备注信息      停止循迹并停车
//-------------------------------------------------------------------------------------------------------------------
void track_stop_follow(void)
{
    track_follow_flag = 0;
    track_follow_reverse_flag = 0;
    wheel_pid_set_target_speed(0.0f);
    steering_set_target(0.0f);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     开始反向循迹
//  参数说明      void
//  返回参数      void
//  使用示例      track_start_follow_reverse();
//  备注信息      从最后一个点开始递减遍历，需先调用track_set_yaw_offset设置偏移量
//-------------------------------------------------------------------------------------------------------------------
void track_start_follow_reverse(void)
{
    if(track_get_stored_point_count() == 0)
    {
        track_follow_reverse_flag = 0;
        wheel_pid_set_target_speed(0.0f);
        steering_set_target(0.0f);
        return;
    }

    // 注意：反向循迹不重置INS位置，车已在停车区，坐标系保持不变

    // 重置循迹状态，从最后一个点开始
    follow_abs_index = track_get_stored_point_count();
    follow_page = Track_Flash_Page_Begin;
    follow_point_idx = 1;
    steer_output = 0.0f;
    steer_output_filtered = 0.0f;
    current_speed_dir = 1.0f; // 默认前进方向

    // 加载最后一页数据到缓存
    uint32 last_page_offset = (follow_abs_index - 1U) / Track_Flash_Point_Page_Max;
    uint32 last_page = Track_Flash_Page_Begin + last_page_offset;
    track_flash_read_page(last_page);
    follow_page = (uint16)last_page;

    // 验证最后一个点数据是否有效
    uint32 last_point_in_page = ((follow_abs_index - 1U) % Track_Flash_Point_Page_Max) + 1U;
    Ins_follow last_point;
    if(!track_flash_get_point(last_point_in_page, &last_point))
    {
        track_follow_reverse_flag = 0;
        wheel_pid_set_target_speed(0.0f);
        steering_set_target(0.0f);
        return;
    }

    follow_point_idx = (uint16)last_point_in_page;
    Ins_date_377.speed_dir = last_point.speed_dir;
    current_speed_dir = last_point.speed_dir;

    // 尝试执行一次寻点，确保前视点在开始移动前就更新
    const INS_State* state = Ins_get_state();
    if(state != NULL)
    {
        float yaw_compensated = state->yaw - s_yaw_offset;
        for(int i = 0; i < 3; i++)
        {
            if(find_lookahead_point_reverse(state->x, state->y))
            {
                break;
            }
        }
    }

    // 确保转向回到中位
    steering_set_target(0.0f);

    // 设置初始速度
    wheel_pid_set_target_speed(TRACK_FOLLOW_SPEED);

    // 切换到反向循迹模式
    track_save_flag = 0;
    track_follow_flag = 0;
    track_follow_reverse_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数名     轨迹模块主处理函数
//  参数说明      void
//  返回参数      void
//  使用示例      track_proc();
//  备注信息      周期调用，执行记录或循迹
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
    else if(track_follow_reverse_flag == 1)
    {
        track_follow_reverse();
    }
}
