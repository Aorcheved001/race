/*
 * track.h
 *
 * Created on: 2024年6月6日
 * Author: LateRain
 * Modified: 2025年11月22日
 */

#ifndef CODE_TRACK_H_
#define CODE_TRACK_H_

//-------------------------------------------头文件区------------------------------------------------------------
#include "zf_common_headfile.h"

//-------------------------------------------宏定义区------------------------------------------------------------
#define Track_Flash_Page_Max          24
// 用于存点的flash页数

#define Track_Flash_Page_Begin        1
// 存点开始的页号

#define Track_Flash_Float_Num         4
// 每个点存4个float数据：x, y, yaw, speed_dir（1表示前进，0表示倒车）

#define Track_Flash_Page_Float_Num    510
// 每页实际可用的float数量（每页存170个点×3个float=510，读取缓冲区也为510）

#define Track_Flash_Point_Page_Max    (Track_Flash_Page_Float_Num / Track_Flash_Float_Num)
// 每页最多可以存多少个点（127个，510/4=127.5，取整为127）

#define TRACK_SAMPLE_STEP             0.1f
// 采样步长（米），每10cm存一个点

#define TRACK_LOOKAHEAD_DISTANCE      0.4f
// Pure Pursuit前视距离（米），建议为采样步长的3-5倍

#define TRACK_WHEELBASE               0.2107f
// 车辆轴距（米）

#define TRACK_FOLLOW_SPEED            0.35f
// 循迹速度（米/秒）

//-------------------------------------------结构体区------------------------------------------------------------
typedef struct {
    float x;
    float y;
    float yaw;
    float speed_dir; // 1表示前进，0表示倒车
} Ins_Date;

typedef struct {
    float x;
    float y;
    float yaw;
    float speed_dir; // 1表示前进，0表示倒车
} Ins_follow;

//-------------------------------------------函数声明区------------------------------------------------------------
void track_init(void);                                                     // 初始化存点模块（清零索引/里程/buffer）
uint8 track_flash_push_point(void);                                        // 按固定里程调用一次，写入缓存；满页自动flush
void track_flash_write_cache(uint32 page_num);                             // 将flash_union_buffer写入指定页
void track_flash_read_page(uint32 page_num);                               // 从指定页读取到读取缓冲区
uint8 track_flash_get_point(uint32 point_index, Ins_follow* out);         // 从读取缓冲区解析指定序号的点
void track_flash_clear_all(void);                                          // 清空所有存点页
uint32 track_flash_get_cur_write_page(void);                               // 获取当前轮询写页
int16_t track_flash_get_point_index(void);                                 // 获取当前已写入buffer的float数量
void track_flash_finish(void);                                             // 结束记录时调用，强制将末页缓存写入Flash

void track_clear(void);                                                    // 清除所有轨迹数据
void track_start_save(void);                                               // 开始存点
void track_stop_save(void);                                                // 停止存点
void track_start_follow(void);                                             // 开始循迹
void track_stop_follow(void);                                              // 停止循迹
void track_proc(void);                                                     // 周期调用，执行存点或循迹
void track_send_all_points_to_host(void);                                  // 将所有存储的轨迹点发送到上位机

//-------------------------------------------向外声明区------------------------------------------------------------
extern Ins_Date Ins_date_377;                                              // 最近一次读取解析到的点
extern uint8 track_save_flag;                                              // 存点标志
extern uint8 track_follow_flag;                                            // 循迹标志
extern uint32 track_total_points;                                          // 已存储的总点数

#endif /* CODE_TRACK_H_ */
