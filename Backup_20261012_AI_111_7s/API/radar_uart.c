#include "radar_uart.h"
#include <math.h>
#include <string.h>

extern volatile uint32_t g_system_tick;
static uint32_t s_last_pc_valid_tick = 0;
static float s_prev_pc_x = 0.0f;
static float s_prev_pc_y = 0.0f;
static float s_prev_pc_z = 0.0f;
static bool s_has_prev_pc = false;
static float s_adaptive_mount_h = RADAR_MOUNT_HEIGHT; /* 自适应安装挂高基线 (无需死板限定 2.2m) */

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
    uint32_t now_tick = g_system_tick;

    if (f->len >= 4u) {
      int32_t target_num = 0;
      memcpy(&target_num, f->data, 4);

      if (target_num > 0 && f->len >= 20u) {
        /* 兼容 20字节/点 [id(4)+x(4)+y(4)+z(4)+speed(4)] 与 16字节/点 [id(4)+x(4)+y(4)+z(4)] */
        uint32_t stride = 20u;
        if ((f->len - 4u) % 20u != 0u && (f->len - 4u) % 16u == 0u) {
          stride = 16u;
        }
        uint32_t max_targets_in_frame = (f->len - 4u) / stride;
        if ((uint32_t)target_num > max_targets_in_frame) {
          target_num = (int32_t)max_targets_in_frame;
        }

        if (target_num > 0) {
          float sum_x = 0.0f;
          float sum_y = 0.0f;
          float sum_z = 0.0f;
          float max_speed = 0.0f;
          float min_x = 99.0f, max_x = -99.0f;
          float min_y = 99.0f, max_y = -99.0f;
          float min_z = 99.0f, max_z = -99.0f;
          int32_t valid_cnt = 0;

          /* 遍历所有目标点，计算人体点云质心坐标 (avg_x, avg_y, avg_z) 与空间包络跨度 */
          for (int32_t i = 0; i < target_num; i++) {
            uint32_t base_off = 4u + (uint32_t)i * stride;
            float x_temp = 0.0f;
            float y_temp = 0.0f;
            float z_temp = 0.0f;
            float s_temp = 0.0f;
            memcpy(&x_temp, &f->data[base_off + 4u], 4);
            memcpy(&y_temp, &f->data[base_off + 8u], 4);
            memcpy(&z_temp, &f->data[base_off + 12u], 4);

            if (isnan(x_temp) || isnan(y_temp) || isnan(z_temp))
              continue;
            if (x_temp < -10.0f || x_temp > 10.0f || z_temp < -10.0f || z_temp > 10.0f)
              continue;
            if (y_temp < 0.0f)
              y_temp = -y_temp;
            if (y_temp > RADAR_HEIGHT_RAW_MAX)
              y_temp = RADAR_HEIGHT_RAW_MAX;

            sum_x += x_temp;
            sum_y += y_temp;
            sum_z += z_temp;
            if (x_temp < min_x) min_x = x_temp;
            if (x_temp > max_x) max_x = x_temp;
            if (y_temp < min_y) min_y = y_temp;
            if (y_temp > max_y) max_y = y_temp;
            if (z_temp < min_z) min_z = z_temp;
            if (z_temp > max_z) max_z = z_temp;
            valid_cnt++;

            if (stride >= 20u) {
              memcpy(&s_temp, &f->data[base_off + 16u], 4);
              s_temp = fabsf(s_temp);
              if (!isnan(s_temp) && s_temp <= 9.99f && s_temp > max_speed) {
                max_speed = s_temp;
              }
            }
          }

          if (valid_cnt > 0) {
            float avg_x = sum_x / (float)valid_cnt;
            float avg_y = sum_y / (float)valid_cnt;
            float avg_z = sum_z / (float)valid_cnt;

            /* 在线自适应安装高度包络：若实际测量距离超过默认挂高，自动向上扩展挂高基准 */
            if (avg_y + 0.15f > s_adaptive_mount_h && avg_y < 4.5f) {
              s_adaptive_mount_h = avg_y + 0.15f;
            }
            s_radar_data.height_radar_m = avg_y;
            s_radar_data.height_floor_m = s_adaptive_mount_h - avg_y;
            if (s_radar_data.height_floor_m < 0.0f)
              s_radar_data.height_floor_m = 0.0f;

            s_radar_data.pc_avg_x_m = avg_x;
            s_radar_data.pc_avg_z_m = avg_z;
            s_radar_data.pc_point_cnt = (uint16_t)valid_cnt;
            s_radar_data.pc_span_vert_m = (max_y >= min_y) ? (max_y - min_y) : 0.0f;
            float sx = (max_x >= min_x) ? (max_x - min_x) : 0.0f;
            float sz = (max_z >= min_z) ? (max_z - min_z) : 0.0f;
            s_radar_data.pc_span_horiz_m = sqrtf(sx * sx + sz * sz);
            s_radar_data.last_pc_tick = now_tick;

            float prev_speed = s_radar_data.speed_m_s;
            float calc_speed = max_speed;

            /* 若 LD6002C 固件的 speed 字段固定填 0，则直接由 LD6002C 3D 点云质心位移计算瞬时移动速度 (m/s) */
            if (calc_speed < 0.01f) {
              if (s_has_prev_pc) {
                uint32_t dt_ms = now_tick - s_last_pc_valid_tick;
                float dx = avg_x - s_prev_pc_x;
                float dy = avg_y - s_prev_pc_y;
                float dz = avg_z - s_prev_pc_z;
                float dist_3d = sqrtf(dx * dx + dy * dy + dz * dz);

                if (dist_3d >= 0.002f && dt_ms > 0u) {
                  uint32_t eff_dt = (dt_ms > 1000u) ? 500u : ((dt_ms < 50u) ? 50u : dt_ms);
                  calc_speed = (dist_3d * 1000.0f) / (float)eff_dt;
                  if (calc_speed > 9.99f)
                    calc_speed = 9.99f;
                  /* 计算垂直向下跌落速度 pc_v_drop_ms (avg_y 增大代表离地高度 H 减小即向下坠落) */
                  s_radar_data.pc_v_drop_ms = (dy * 1000.0f) / (float)eff_dt;
                  s_radar_data.pc_acc_3d_ms2 = ((calc_speed - prev_speed) * 1000.0f) / (float)eff_dt;
                  s_prev_pc_x = avg_x;
                  s_prev_pc_y = avg_y;
                  s_prev_pc_z = avg_z;
                  s_last_pc_valid_tick = now_tick;
                  s_radar_data.speed_m_s = calc_speed;
                } else if (dt_ms >= 1000u) {
                  /* 超过 1000ms 坐标无位移，速度归零 */
                  s_last_pc_valid_tick = now_tick;
                  s_radar_data.speed_m_s = 0.0f;
                  s_radar_data.pc_v_drop_ms = 0.0f;
                  s_radar_data.pc_acc_3d_ms2 = 0.0f;
                }
              } else {
                s_prev_pc_x = avg_x;
                s_prev_pc_y = avg_y;
                s_prev_pc_z = avg_z;
                s_has_prev_pc = true;
                s_last_pc_valid_tick = now_tick;
              }
            } else {
              /* 硬件原生 speed 字段非 0，直接采用并计算垂直坠落速度与加速度 */
              if (s_has_prev_pc) {
                uint32_t dt_ms = now_tick - s_last_pc_valid_tick;
                uint32_t eff_dt = (dt_ms > 1000u) ? 500u : ((dt_ms < 50u) ? 50u : dt_ms);
                s_radar_data.pc_v_drop_ms = ((avg_y - s_prev_pc_y) * 1000.0f) / (float)eff_dt;
                s_radar_data.pc_acc_3d_ms2 = ((calc_speed - prev_speed) * 1000.0f) / (float)eff_dt;
              }
              s_prev_pc_x = avg_x;
              s_prev_pc_y = avg_y;
              s_prev_pc_z = avg_z;
              s_has_prev_pc = true;
              s_last_pc_valid_tick = now_tick;
              s_radar_data.speed_m_s = calc_speed;
            }
          }
        }
      } else {
        /* 关键修复：LD6002C 在有效点云帧之间会夹杂大量 target_num=0 的空帧。
         * 不能一收到空帧就立刻把 speed_m_s 清零（否则 500ms 刷新的 OLED 永远只看到 0.00）。
         * 只有当超过 1000ms 没有收到任何有效点云位移，或判定无人时，才将速度归零。 */
        if (s_radar_data.is_human == 0u) {
          s_radar_data.height_radar_m = 0.0f;
          s_radar_data.height_floor_m = 0.0f;
          s_radar_data.speed_m_s = 0.0f;
          s_radar_data.pc_v_drop_ms = 0.0f;
          s_radar_data.pc_acc_3d_ms2 = 0.0f;
          s_has_prev_pc = false;
        } else if (now_tick - s_last_pc_valid_tick > 1000u) {
          s_radar_data.speed_m_s = 0.0f;
          s_radar_data.pc_v_drop_ms = 0.0f;
          s_radar_data.pc_acc_3d_ms2 = 0.0f;
        }
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