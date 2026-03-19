#ifndef _VOFA_PROTOCOL_H
#define _VOFA_PROTOCOL_H

#include "zf_common_headfile.h"

// 1. 把 FIFO 结构体暴露给 isr.c，让中断服务员能把数据放进去
extern fifo_struct uart_data_fifo; 

// 2. VOFA 协议解析核心大统领
void VOFA_Protocol_Parse(uint8 *rx_buffer, uint32 data_length);

// 3. ================= 新增：封装好的串口调参接口 =================
void VOFA_UART_Init(void);      // 放 main 的初始化里
void VOFA_UART_Process(void);   // 放 main 的 while(1) 循环里

#endif