#include "ld2450.h"
#include "hal_data.h"
#include <string.h>

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

/* 剥离 16 位数据的符号位，还原真实的负数或正数
 * 官方协议：最高位为1代表正数，最高位为0代表负数
 */
static int16_t parse_signed_value(uint8_t low_byte, uint8_t high_byte) {
  uint16_t raw = (uint16_t)(low_byte | (high_byte << 8));
  if (raw & 0x8000) {
    return (int16_t)(raw & 0x7FFF); // 最高位为1，是正数
  } else {
    return -(int16_t)raw; // 最高位为0，是负数
  }
}

extern volatile uint32_t g_system_tick;
static uint32_t last_rx_tick = 0;

void LD2450_ResetState(void) {
  rx_idx = 0;
}

/* 严格状态机，逐字节接收，带帧间超时自愈和连续帧头容错 */
void LD2450_UART_Callback(uint8_t rx_data) {
  uint32_t current_tick = g_system_tick;

  /* 帧间超时自愈：LD2450整帧30字节在1.2ms内发完，帧间隔约98ms。
   * 若距离上一字节超过20ms，说明处于帧间隔期或上一帧残缺，强制重置游标 */
  if (rx_idx > 0 && (current_tick - last_rx_tick > 20)) {
    rx_idx = 0;
  }
  last_rx_tick = current_tick;

  switch (rx_idx) {
  case 0:
    if (rx_data == 0xAA) {
      rx_buf[0] = rx_data;
      rx_idx = 1;
    }
    break;
  case 1:
    if (rx_data == 0xFF) {
      rx_buf[1] = rx_data;
      rx_idx = 2;
    } else if (rx_data == 0xAA) {
      rx_idx = 1; // 连续 0xAA 容错
    } else {
      rx_idx = 0;
    }
    break;
  case 2:
    if (rx_data == 0x03) {
      rx_buf[2] = rx_data;
      rx_idx = 3;
    } else if (rx_data == 0xAA) {
      rx_idx = 1;
    } else {
      rx_idx = 0;
    }
    break;
  case 3:
    if (rx_data == 0x00) {
      rx_buf[3] = rx_data;
      rx_idx = 4;
    } else if (rx_data == 0xAA) {
      rx_idx = 1;
    } else {
      rx_idx = 0;
    }
    break;
  default:
    rx_buf[rx_idx++] = rx_data;
    if (rx_idx >= FRAME_LEN) {
      // 校验帧尾 0x55 0xCC
      if (rx_buf[28] == 0x55 && rx_buf[29] == 0xCC) {
        /* 提取 3 个目标的数据 */
        for (int i = 0; i < 3; i++) {
          uint8_t offset = (uint8_t)(4 + (i * 8));

          radar_data_parsed.target[i].x =
              parse_signed_value(rx_buf[offset], rx_buf[offset + 1]);
          radar_data_parsed.target[i].y =
              parse_signed_value(rx_buf[offset + 2], rx_buf[offset + 3]);
          radar_data_parsed.target[i].speed =
              parse_signed_value(rx_buf[offset + 4], rx_buf[offset + 5]);
          radar_data_parsed.target[i].res =
              (uint16_t)(rx_buf[offset + 6] | (rx_buf[offset + 7] << 8));

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
