/*
 * menu.h
 *
 *  简洁版菜单框架
 *  适用于 TC377 惯性导航项目
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#include "zf_common_headfile.h"

// 菜单显示与逻辑处理
void menu(void);
void menumy(void);
void menu_first(void);

// 按键检测逻辑（内部使用）
int key_detect(key_index_enum key_n, key_state_enum state);

#endif /* CODE_MENU_H_ */
