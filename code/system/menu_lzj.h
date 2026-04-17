/*
 * menu.c
 *
 *  Created on: 2026-03-15
 *      Author: Daydreamer
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_
//-------------------------------------------头文件申明区------------------------------------------------------------
#include "zf_common_headfile.h"


//-------------------------------------------结构体定义区------------------------------------------------------------

typedef struct {
    float x;
    float y;
} INS_Point;

typedef struct {
    int Mode;               // 当前惯导状态
    INS_Point cod_RealTime; // 当前实时位置
    INS_Point cod_Saved[10];// 保存的位置
    INS_Point cod_Target[10];// 目标位置
    int PointCount;         // 目标位置数量
    int CurrentTarget;      // 当前目标位置索引
    float Dis_ins;          // 实时距离（m）
} INS_Demo_t;               // 惯导菜单结构体

//-------------------------------------------向外声明区------------------------------------------------------------
extern INS_Demo_t INS;      //向外声明

//-------------------------------------------函数声明区------------------------------------------------------------
void menu(void);            // 惯导菜单函数

int key_detect(key_index_enum key_n, key_state_enum state);             // 按键检测函数

#endif /* CODE_MENU_H_ */
