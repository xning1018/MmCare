#include "ld2450.h"
#include <string.h>
#include "hal_data.h"
#define FRAME_LEN 30

static uint8_t rx_buf[FRAME_LEN];
static uint8_t rx_idx = 0;
static bool new_frame_ready = false;
static LD2450_Data_t radar_data_parsed;

void LD2450_Init(void) {
    memset(rx_buf, 0, sizeof(rx_buf));
    rx_idx = 0;
    new_frame_ready = false;
    memset(&radar_data_parsed, 0, sizeof(radar_data_parsed));
}

/* 核心逻辑：剥离 16 位数据的符号位，还原真实的负数或正数 
 * 官方协议：最高位为1代表正数，最高位为0代表负数
 */
static int16_t parse_signed_value(uint8_t low_byte, uint8_t high_byte) {
    uint16_t raw = (uint16_t)(low_byte | (high_byte << 8));
    if (raw & 0x8000) {
        return -(int16_t)(raw & 0x7FFF); // 最高位为1，是负数
    } else {
        return (int16_t)raw;             // 最高位为0，是正数
    }
}

/* 严格状态机，逐字节接收，自动抛弃错位和乱码数据 */
void LD2450_UART_Callback(uint8_t rx_data) {
    switch (rx_idx) {
        case 0: 
            if (rx_data == 0xAA) rx_buf[rx_idx++] = rx_data; 
            break;
        case 1: 
            if (rx_data == 0xFF) rx_buf[rx_idx++] = rx_data; else rx_idx = 0; 
            break;
        case 2: 
            if (rx_data == 0x03) rx_buf[rx_idx++] = rx_data; else rx_idx = 0; 
            break;
        case 3: 
            if (rx_data == 0x00) rx_buf[rx_idx++] = rx_data; else rx_idx = 0; 
            break;
        default:
            rx_buf[rx_idx++] = rx_data;
            if (rx_idx >= FRAME_LEN) {
                // 校验帧尾
                if (rx_buf[28] == 0x55 && rx_buf[29] == 0xCC) {
                    /* 提取 3 个目标的数据 */
                    for (int i = 0; i < 3; i++) {
                   uint8_t offset = (uint8_t)(4 + (i * 8));
                        
                        radar_data_parsed.target[i].x = parse_signed_value(rx_buf[offset], rx_buf[offset+1]);
                        radar_data_parsed.target[i].y = parse_signed_value(rx_buf[offset+2], rx_buf[offset+3]);
                        radar_data_parsed.target[i].speed = parse_signed_value(rx_buf[offset+4], rx_buf[offset+5]);
                        radar_data_parsed.target[i].res = (uint16_t)(rx_buf[offset+6] | (rx_buf[offset+7] << 8));
                        
                        // 判定目标是否存在：官方协议指出当坐标和速度全为0时，目标不存在
                        if (radar_data_parsed.target[i].x == 0 && 
                            radar_data_parsed.target[i].y == 0 && 
                            radar_data_parsed.target[i].speed == 0) {
                            radar_data_parsed.target[i].is_valid = false;
                        } else {
                            radar_data_parsed.target[i].is_valid = true;
                        }
                    }
                    new_frame_ready = true;
                }
                rx_idx = 0; // 一帧结束（无论对错），强制重置游标准备接下一帧
            }
            break;
    }
}

/* 主循环安全拉取数据接口 */
bool LD2450_Get_Data(LD2450_Data_t *out_data) {
    if (new_frame_ready) {
        // 短暂关闭中断，防止主循环读取结构体时被 UART 中断打断造成数据撕裂
        __disable_irq(); 
        memcpy(out_data, &radar_data_parsed, sizeof(LD2450_Data_t));
        new_frame_ready = false;
        __enable_irq();
        return true;
    }
    return false;
}
