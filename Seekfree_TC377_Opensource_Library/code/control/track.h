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
#include "vehicle_config.h"  // 引入统一车辆参数配置

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
// [P7-4-5] 注: 510 不是 4 的倍数，整数除法截断后每页浪费 2 个 float（8 字节）

#define TRACK_SAMPLE_STEP             0.10f
// 采样步长（米），每10cm存一个点

#define TRACK_LOOKAHEAD_DISTANCE      0.50f
// Pure Pursuit前视距离（米），0.50m大幅降低转向饱和率
// 原0.15m过小：1cm偏差即导致转向饱和(22°)，正反馈发散
// 0.35m测试：76.4%转向饱和，RMS横向偏差19.6cm，严重不足
// 0.50m下：5cm偏差→约15°（可调制），10cm偏差→22°（临界），系统稳定

#define TRACK_LD_SPEED_GAIN           0.30f
// 速度自适应前瞻增益（秒），Ld = Ld_base + gain * v
// 0.30s × 0.36m/s ≈ 0.11m，实际Ld ≈ 0.61m
// 高速时自动增大前瞻距离，防止转向过激

// TRACK_WHEELBASE 已在 vehicle_config.h 中定义
// 车辆轴距（前后轮距离），用于 Pure Pursuit 转向计算

#define TRACK_FOLLOW_SPEED            4
// 循迹速度（脉冲/4ms），4脉冲≈0.36m/s，降速提高跟踪精度

//-------------------------------------------结构体区------------------------------------------------------------
typedef struct {
    float x;
    float y;
    float yaw;
    float speed_dir; // 1表示前进，0表示倒车
} TrackPoint;

// 兼容旧名称
typedef TrackPoint Ins_Date;
typedef TrackPoint Ins_follow;

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
void track_start_follow_reverse(void);                                     // 开始反向循迹（从最后一个点递减遍历）
void track_stop_follow(void);                                              // 停止循迹
void track_set_yaw_offset(float offset_rad);                               // 设置yaw偏移量（循迹时补偿180°旋转）
void track_proc(void);                                                     // 周期调用，执行存点或循迹
void track_send_all_points_to_host(void);                                  // 将所有存储的轨迹点发送到上位机
uint32 track_get_follow_index(void);
//-------------------------------------------向外声明区------------------------------------------------------------
extern Ins_Date Ins_date_377;                                              // 最近一次读取解析到的点
extern uint8 track_save_flag;                                              // 存点标志
extern uint8 track_follow_flag;                                            // 循迹标志
extern uint8 track_follow_reverse_flag;                                    // 反向循迹标志
extern uint32 track_total_points;                                          // 已存储的总点数

#endif /* CODE_TRACK_H_ */
