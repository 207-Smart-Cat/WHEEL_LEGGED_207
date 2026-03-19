#ifndef _SCREEN_DISPLAY_H_
#define _SCREEN_DISPLAY_H_

#include "zf_common_headfile.h"

// ** 宏定义区域 **
#define RGB565_SKYBLUE 0x87CE
#define IPS200_TYPE (IPS200_TYPE_SPI)

// ** 全局变量区域 **
extern uint8 IPS200_flag; //  屏幕显示flag（PIT中断置位）

// 函数声明
void screen_display_init(void);
void screen_display_process(void);

#endif