#ifndef LD2450_H_
#define LD2450_H_
#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>

/* 单个目标信息结构体 */
typedef struct {
    int16_t x;          // X坐标 (mm)，正数代表右侧，负数代表左侧
    int16_t y;          // Y坐标 (mm)，距离雷达的直线距离
    int16_t speed;      // 速度 (cm/s)，正数代表远离，负数代表靠近
    uint16_t res;       // 距离分辨率 (mm)
    bool is_valid;      // 目标是否有效 (是否存在)
} LD2450_Target_t;

/* 雷达完整数据帧结构体 (最多支持3个目标) */
typedef struct {
    LD2450_Target_t target[3]; 
} LD2450_Data_t;

/* 模块对外接口 */
void LD2450_Init(void);
void LD2450_UART_Callback(uint8_t rx_data);    // 需放置在 FSP 的串口接收回调中
bool LD2450_Get_Data(LD2450_Data_t *out_data); // 在主循环中非阻塞调用以获取最新数据

#endif /* LD2450_H_ */

