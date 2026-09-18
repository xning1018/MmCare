#ifndef RADAR_UART_H_
#define RADAR_UART_H_

#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>

/* ================================================================
 * TinyFrame 协议
 * ================================================================ */
#define TF_SOF 0x01u

/* LD6002C 消息类型 */
#define TF_TYPE_FALL_STATUS 0x0E02u /* 跌倒状态: 1字节, 0=正常, 1=跌倒 */
#define TF_TYPE_FALL_HEIGHT 0x0E0Eu /* 目标高度 */
#define TF_TYPE_PRESENCE 0x0F09u    /* 有无人: 1字节, 0=无人, 1=有人 */
#define TF_TYPE_PARAMS 0x0E06u      /* 参数回传: 28字节 */
#define TF_TYPE_POINT_CLOUD 0x0A08u /* 3D点云: 包含精确的目标垂直距离(Y轴) */
#define TF_TYPE_FW_VER 0xFFFFu      /* 固件版本 */

/* ================================================================
 * 安装高度配置（单位: 米）
 * ================================================================ */
#define RADAR_MOUNT_HEIGHT 2.2f
#define RADAR_HEIGHT_RAW_MAX 5.0f

/* 点云数据极大，至少需要256字节的 Payload 空间，否则合法帧会被状态机丢弃 */
#define RADAR_RX_BUF_SIZE 2048u
#define RADAR_MAX_PAYLOAD 1024u

/* 解析状态机 */
typedef enum {
  TF_STATE_SOF = 0,
  TF_STATE_ID_H,
  TF_STATE_ID_L,
  TF_STATE_LEN_H,
  TF_STATE_LEN_L,
  TF_STATE_TYPE_H,
  TF_STATE_TYPE_L,
  TF_STATE_HEAD_CKSUM,
  TF_STATE_DATA,
  TF_STATE_DATA_CKSUM
} tf_parse_state_t;

typedef struct {
  uint16_t type;
  uint16_t len;
  uint8_t data[RADAR_MAX_PAYLOAD];
  bool valid;
} tf_frame_t;

/* 雷达业务数据 */
typedef struct {
  float height_radar_m;
  float height_floor_m;
  uint8_t is_fall;
  uint8_t is_human;
  uint32_t frame_count;
  bool fall_updated;
  bool presence_updated;
  bool height_updated;

  uint16_t last_type;
  uint16_t last_height_type; /* 最后收到的高度消息类型 */
  uint16_t last_height_len;  /* 最后收到的高度消息长度 */
  uint16_t unknown_types[4];
  uint8_t unknown_idx;

  /* 记录 0x0E02 帧的实际长度和前8字节原始数据 */
  uint16_t last_0e02_len;
  uint8_t last_0e02_raw[8];
} radar_data_t;

void Radar_Init(void);
void Radar_PostInit(void);
void Radar_Process(void);
radar_data_t *Radar_GetData(void);
void radar_uart_callback(uart_callback_args_t *p_args);
void Radar_SendFrame(uint16_t type, const uint8_t *p_data, uint16_t len);
void Radar_SendMountHeight(float height_m);
void Radar_SendInit(float high, float threshold, uint32_t sensitivity,
                    float rect_xl, float rect_xr, float rect_zf, float rect_zb);

#endif /* RADAR_UART_H_ */