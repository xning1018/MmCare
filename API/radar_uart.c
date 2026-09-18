#include "radar_uart.h"
#include <string.h>

static volatile uint8_t s_rx_buf[RADAR_RX_BUF_SIZE];
static volatile uint32_t s_rx_write = 0;
static volatile uint32_t s_rx_read = 0;

static tf_parse_state_t s_state = TF_STATE_SOF;
static tf_frame_t s_frame;
static uint16_t s_data_idx = 0;

static uint8_t s_head_xor = 0;
static uint8_t s_data_xor = 0;

static radar_data_t s_radar_data;

void radar_uart_callback(uart_callback_args_t *p_args) {
  if (UART_EVENT_RX_CHAR == p_args->event) {
    uint32_t next = (s_rx_write + 1u) % RADAR_RX_BUF_SIZE;
    if (next != s_rx_read) {
      s_rx_buf[s_rx_write] = (uint8_t)(p_args->data & 0xFFu);
      s_rx_write = next;
    }
  }
}

static void _process_frame(const tf_frame_t *f) {
  if (!f->valid)
    return;
  s_radar_data.frame_count++;
  s_radar_data.last_type = f->type;

  switch (f->type) {
  /* 0x0E02: 跌倒状态（当前固件仅1字节，未来固件可能附带高度） */
  case TF_TYPE_FALL_STATUS: {
    if (f->len >= 1u) {
      s_radar_data.is_fall = (f->data[0] == 1u) ? 1u : 0u;
      s_radar_data.fall_updated = true;
    }

    /* 兼容：若未来固件在 0x0E02 帧中附带高度数据（len >= 5），自动解析 */
    if (f->len >= 5u) {
      float h_radar = 0.0f;
      memcpy(&h_radar, &f->data[1], 4);

      if (h_radar < -0.001f || h_radar > 10.0f) {
        uint32_t val_int = 0;
        memcpy(&val_int, &f->data[1], 4);
        if (val_int > 0 && val_int < 10000) {
          h_radar = (float)val_int / 1000.0f;
        } else {
          h_radar = 0.0f;
        }
      }

      if (h_radar < 0.0f)
        h_radar = 0.0f;
      if (h_radar > RADAR_HEIGHT_RAW_MAX)
        h_radar = RADAR_HEIGHT_RAW_MAX;

      s_radar_data.height_radar_m = h_radar;
      s_radar_data.height_floor_m = RADAR_MOUNT_HEIGHT - h_radar;
      if (s_radar_data.height_floor_m < 0.0f)
        s_radar_data.height_floor_m = 0.0f;
      s_radar_data.height_updated = true;
    }
    break;
  }

  /* 由 0x0F09 独立负责有无人状态 */
  case TF_TYPE_PRESENCE: {
    if (f->len >= 1u) {
      s_radar_data.is_human = (f->data[0] > 0u) ? 1u : 0u;
      s_radar_data.presence_updated = true;
    }
    break;
  }

  /* 兼容性处理：若雷达确实发了 0x0E0E，尝试解析 */
  case TF_TYPE_FALL_HEIGHT: {
    s_radar_data.last_height_type = f->type;
    s_radar_data.last_height_len = f->len;

    if (f->len == 4u) {
      float h_radar = 0.0f;
      memcpy(&h_radar, f->data, 4);

      uint32_t val_int = 0;
      memcpy(&val_int, f->data, 4);

      if (h_radar > -0.001f && h_radar < 0.001f && val_int > 0 &&
          val_int < 10000) {
        h_radar = (float)val_int / 1000.0f;
      }

      if (h_radar < 0.0f)
        h_radar = 0.0f;
      if (h_radar > RADAR_HEIGHT_RAW_MAX)
        h_radar = RADAR_HEIGHT_RAW_MAX;

      s_radar_data.height_radar_m = h_radar;
      s_radar_data.height_floor_m = RADAR_MOUNT_HEIGHT - h_radar;
      if (s_radar_data.height_floor_m < 0.0f)
        s_radar_data.height_floor_m = 0.0f;
      s_radar_data.height_updated = true;
    }
    break;
  }

  /* 修正3：点云坐标解析。这是获取连续距离变化的最客观来源 */
  case TF_TYPE_POINT_CLOUD: {
    s_radar_data.last_height_type = 0x0A08u;
    s_radar_data.last_height_len = f->len;
    s_radar_data.height_updated = true; /* 只要收到点云，说明 User log 已开启 */
    if (f->len >= 4u) {
      int32_t target_num = 0;
      memcpy(&target_num, f->data, 4);

      if (target_num > 0) {
        /* 结构: num(4) + [id(4) + x(4) + y(4) + z(4) + speed(4)]... */
        /* 限制目标解析个数，防止 target_num 字段损坏引起越界 */
        uint32_t max_targets_in_frame = (f->len - 4u) / 20u;
        if ((uint32_t)target_num > max_targets_in_frame) {
          target_num = (int32_t)max_targets_in_frame;
        }

        if (target_num > 0) {
          float min_y_point = 999.0f;
          bool found_valid = false;

          /* 遍历所有目标点，找出 Y
           * 轴坐标最小（即离雷达最近、离地面最高，如人的头部）的目标 */
          for (int32_t i = 0; i < target_num; i++) {
            float y_temp = 0.0f;
            /* 每个目标占 20 字节。第 i 个目标的 Y 坐标在偏移 4 + i*20 + 8 (id
             * 4B + x 4B) = 12 + i*20 处 */
            memcpy(&y_temp, &f->data[12 + i * 20], 4);

            if (y_temp < 0.0f)
              y_temp = -y_temp;
            if (y_temp > RADAR_HEIGHT_RAW_MAX)
              y_temp = RADAR_HEIGHT_RAW_MAX;

            if (y_temp < min_y_point) {
              min_y_point = y_temp;
              found_valid = true;
            }
          }

          if (found_valid) {
            s_radar_data.height_radar_m = min_y_point;
            s_radar_data.height_floor_m = RADAR_MOUNT_HEIGHT - min_y_point;
            if (s_radar_data.height_floor_m < 0.0f)
              s_radar_data.height_floor_m = 0.0f;
          }
        }
      } else {
        /* 无目标点时，若有人则保持上次高度，否则重置为0（人消失了） */
        if (s_radar_data.is_human == 0u) {
          s_radar_data.height_radar_m = 0.0f;
          s_radar_data.height_floor_m = 0.0f;
        }
        /* 有人但无目标点时保持上次值，不更新 */
      }
    }
    break;
  }

  default: {
    s_radar_data.unknown_types[s_radar_data.unknown_idx % 4u] = f->type;
    s_radar_data.unknown_idx++;
    break;
  }
  }
}

static void _feed_byte(uint8_t byte) {
  switch (s_state) {
  case TF_STATE_SOF:
    if (byte == TF_SOF) {
      memset(&s_frame, 0, sizeof(s_frame));
      s_data_idx = 0;
      s_head_xor = byte;
      s_state = TF_STATE_ID_H;
    }
    break;

  case TF_STATE_ID_H:
    s_head_xor ^= byte;
    s_state = TF_STATE_ID_L;
    break;

  case TF_STATE_ID_L:
    s_head_xor ^= byte;
    s_state = TF_STATE_LEN_H;
    break;

  case TF_STATE_LEN_H:
    s_head_xor ^= byte;
    s_frame.len = (uint16_t)(byte << 8u);
    s_state = TF_STATE_LEN_L;
    break;

  case TF_STATE_LEN_L:
    s_head_xor ^= byte;
    s_frame.len |= (uint16_t)byte;
    if (s_frame.len > RADAR_MAX_PAYLOAD)
      s_state = TF_STATE_SOF;
    else
      s_state = TF_STATE_TYPE_H;
    break;

  case TF_STATE_TYPE_H:
    s_head_xor ^= byte;
    s_frame.type = (uint16_t)(byte << 8u);
    s_state = TF_STATE_TYPE_L;
    break;

  case TF_STATE_TYPE_L:
    s_head_xor ^= byte;
    s_frame.type |= (uint16_t)byte;
    s_state = TF_STATE_HEAD_CKSUM;
    break;

  case TF_STATE_HEAD_CKSUM: {
    uint8_t expected = (uint8_t)~s_head_xor;
    if (byte != expected) {
      s_state = TF_STATE_SOF;
      break;
    }
    s_data_xor = 0;
    if (s_frame.len > 0u) {
      s_data_idx = 0;
      s_state = TF_STATE_DATA;
    } else {
      s_state = TF_STATE_DATA_CKSUM;
    }
    break;
  }

  case TF_STATE_DATA: {
    s_data_xor ^= byte;
    if (s_data_idx < RADAR_MAX_PAYLOAD)
      s_frame.data[s_data_idx] = byte;
    s_data_idx++;
    if (s_data_idx >= s_frame.len)
      s_state = TF_STATE_DATA_CKSUM;
    break;
  }

  case TF_STATE_DATA_CKSUM: {
    uint8_t expected = (uint8_t)~s_data_xor;
    if (byte != expected) {
      s_state = TF_STATE_SOF;
      break;
    }
    s_frame.valid = true;
    _process_frame(&s_frame);
    s_state = TF_STATE_SOF;
    break;
  }

  default:
    s_state = TF_STATE_SOF;
    break;
  }
}

void Radar_Init(void) {
  memset(&s_radar_data, 0, sizeof(s_radar_data));
  s_radar_data.height_radar_m = 0.0f;
  s_radar_data.height_floor_m = 0.0f;
  s_state = TF_STATE_SOF;
  s_rx_write = 0;
  s_rx_read = 0;

  R_SCI_UART_Open(&g_uart4_ctrl, &g_uart4_cfg);
}

void Radar_PostInit(void) {
  /* 打开 User log 信息 (包含点云 0x0A08) */
  uint32_t log_enable = 1u;
  Radar_SendFrame(0x010Eu, (uint8_t *)&log_enable, 4u);
}

void Radar_SendFrame(uint16_t type, const uint8_t *p_data, uint16_t len) {
  /* 同步扩大发送缓冲池，防溢出。长度为最大 Payload + 头尾开销 (16字节) */
  static uint8_t buf[RADAR_MAX_PAYLOAD + 16u];
  uint16_t pos = 0u;
  uint8_t hxor = 0u;
  uint8_t dxor = 0u;
  uint16_t i;

  if (len > RADAR_MAX_PAYLOAD)
    return;

  buf[pos] = TF_SOF;
  hxor ^= buf[pos];
  pos++;
  buf[pos] = 0x00u;
  hxor ^= buf[pos];
  pos++;
  buf[pos] = 0x00u;
  hxor ^= buf[pos];
  pos++;
  buf[pos] = (uint8_t)(len >> 8u);
  hxor ^= buf[pos];
  pos++;
  buf[pos] = (uint8_t)(len & 0xFFu);
  hxor ^= buf[pos];
  pos++;
  buf[pos] = (uint8_t)(type >> 8u);
  hxor ^= buf[pos];
  pos++;
  buf[pos] = (uint8_t)(type & 0xFFu);
  hxor ^= buf[pos];
  pos++;
  buf[pos] = (uint8_t)(~hxor);
  pos++;

  for (i = 0u; i < len; i++) {
    buf[pos] = p_data[i];
    dxor ^= p_data[i];
    pos++;
  }
  buf[pos] = (uint8_t)(~dxor);
  pos++;

  R_SCI_UART_Write(&g_uart4_ctrl, buf, pos);
}

void Radar_SendInit(float high, float threshold, uint32_t sensitivity,
                    float rect_xl, float rect_xr, float rect_zf,
                    float rect_zb) {
  (void)high;
  (void)threshold;
  (void)sensitivity;
  (void)rect_xl;
  (void)rect_xr;
  (void)rect_zf;
  (void)rect_zb;
  Radar_SendFrame(0x2110u, NULL, 0u);
}

void Radar_SendMountHeight(float height_m) {
  uint8_t data[4u];
  memcpy(data, &height_m, sizeof(float));
  Radar_SendFrame(0x0E04u, data, 4u);
}

void Radar_Process(void) {
  uint32_t write_snapshot = s_rx_write;
  while (s_rx_read != write_snapshot) {
    uint8_t byte = s_rx_buf[s_rx_read];
    s_rx_read = (s_rx_read + 1u) % RADAR_RX_BUF_SIZE;
    _feed_byte(byte);
  }
}

radar_data_t *Radar_GetData(void) { return &s_radar_data; }