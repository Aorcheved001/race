/*
 * debug_layer.h
 *
 *  Created on: 2026年4月27日
 *      Author: 调试层抽象
 */

#ifndef CODE_SYSTEM_DEBUG_LAYER_H_
#define CODE_SYSTEM_DEBUG_LAYER_H_

#include "zf_common_headfile.h"

// -------------------------------------------------------------------------------------------------------------------
// 调试层输出接口
// -------------------------------------------------------------------------------------------------------------------

// 周期调用的调试输出统一入口（例如在 40ms 任务中调用）
void debug_layer_output_40ms(void);
void debug_layer_on_1ms_tick(void);
uint32 debug_layer_get_tick_ms(void);
uint32 debug_layer_get_drop_count(void);
uint32 debug_layer_get_max_queue_depth(void);
uint16 debug_layer_get_pending4_max(void);
void debug_layer_record_pending4(uint16 pending4);

// -------------------------------------------------------------------------------------------------------------------
// 无线发送队列接口（4ms/8ms任务调用push入队，40ms任务调用flush统一发送）
// -------------------------------------------------------------------------------------------------------------------
uint8 debug_wireless_queue_push(const char *msg, uint16 len);     // 入队（不阻塞，队列满时丢弃）
void    debug_wireless_queue_flush(void);                              // 刷新队列（40ms统一发送）

// -------------------------------------------------------------------------------------------------------------------
// 内部具体调试功能（如果需要单独调用也可暴露）
// -------------------------------------------------------------------------------------------------------------------
void debug_send_position_to_host(void);
void debug_send_pos_to_host(void);
void debug_send_attitude_to_host(void);
// debug_send_encoder_sign_to_host 已删除 [P7-4-1]

#endif /* CODE_SYSTEM_DEBUG_LAYER_H_ */