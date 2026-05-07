#ifndef _VOFA_PROTOCOL_H
#define _VOFA_PROTOCOL_H

#include "zf_common_headfile.h"



// 1. 把 FIFO 结构体暴露给 isr.c，让中断服务员能把数据放进去
extern fifo_struct uart_data_fifo;


typedef enum {
    VOFA_PARAM_RX_SRC_UART = 0,
    VOFA_PARAM_RX_SRC_WIFI,
    VOFA_PARAM_RX_SRC_SCREEN,
    VOFA_PARAM_RX_SRC_FLASH,
    VOFA_PARAM_RX_SRC_DEFAULT
} vofa_param_rx_source_t;

void VOFA_Set_Param_Rx_Source(vofa_param_rx_source_t source);
void VOFA_Set_Param_Log_Detail(uint8 enable);
uint8 VOFA_Get_Param_Log_Detail(void);
void VOFA_Log_Param_Update(uint8 param_id, const char *name, float value, const char *cmd);
void VOFA_Log_Param_Bulk(const char *message, const char *cmd, uint16 count);
void VOFA_Log_Param_Command(const char *message, const char *cmd);
// 2. VOFA 协议解析核心大统领
void VOFA_Protocol_Parse(uint8 *rx_buffer, uint32 data_length);

// 3. ================= 新增：封装好的串口调参接口 =================
void VOFA_UART_Init(void);      // 放 main 的初始化里
void VOFA_UART_Process(void);   // 放 main 的 while(1) 循环里

void VOFA_Save_Params_To_Flash(void);
void VOFA_Load_Params_From_Flash(void);
uint8 VOFA_Send_Params_To_Wifi(const float *params);
void VOFA_Upload_Params_To_UI(void); // 用于上电时同步电脑界面的数据


#endif


