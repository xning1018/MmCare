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
 * 安装高度与角度配置（壁装向下俯角倾斜模式，抗浴帘微动）
 * ================================================================ */
#define RADAR_MOUNT_HEIGHT          2.10f   /* 顶部壁装挂高 2.1 米 */
#define RADAR_TILT_SIN              0.5736f /* 俯仰角约 35 度 sin(35°) (覆盖 30°~45°) */
#define RADAR_TILT_COS              0.8192f /* 俯仰角约 35 度 cos(35°) */
#define RADAR_CURTAIN_CUTOFF_M      0.40f   /* 浴帘屏蔽盲区门限: 0.40m 以内近场彻底滤除 */
#define RADAR_HEIGHT_RAW_MAX        5.0f

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

/* 雷达业务数据 (含 LD6002C 完整 3D 点云多参数特征) */
typedef struct {
  float height_radar_m;   /* 质心距雷达垂直距离 (m) */
  float height_floor_m;   /* 质心离地高度 H (m) */
  float speed_m_s;        /* 3D 点云质心移动速度 V (m/s) */
  float pc_avg_x_m;       /* 3D 点云质心横向坐标 X (m) */
  float pc_avg_z_m;       /* 3D 点云质心前向距离 Z (m) */
  float pc_v_drop_ms;     /* 质心垂直向地面跌落速度 -dH/dt (m/s, 正值表示向下坠落) */
  float pc_peak_drop_ms;  /* 跌落窗口内最大垂直坠落速度峰值 (m/s) */
  float pc_acc_3d_ms2;    /* 3D 点云运动加速度 (m/s^2) */
  float pc_span_vert_m;   /* 3D 点云垂直高度跨度 (m) */
  float pc_span_horiz_m;  /* 3D 点云水平展开跨度 (m) */
  uint16_t pc_point_cnt;  /* 当前有效 3D 点云数量 */
  uint32_t last_pc_tick;  /* 最近一次有效 3D 点云更新的系统时基 (ms) */

  uint8_t is_fall;        /* 0x0E02 原生跌倒状态位 */
  uint8_t is_human;       /* 0x0F09 有人状态位 */
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

#endif /* RADAR_UART_H_ */