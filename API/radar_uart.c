#include "radar_uart.h"
#include <math.h>
#include <string.h>

extern volatile uint32_t g_system_tick;
static uint32_t s_last_pc_valid_tick = 0;
static float s_prev_pc_x = 0.0f;
static float s_prev_pc_y = 0.0f;
static float s_prev_pc_z = 0.0f;
static float s_prev_floor_h = 1.4f;
static float s_smooth_floor_h = 1.50f;
static bool s_smooth_h_inited = false;
static bool s_has_prev_pc = false;

static volatile uint8_t s_rx_buf[RADAR_RX_BUF_SIZE];
static volatile uint32_t s_rx_write = 0;
static volatile uint32_t s_rx_read = 0;

static tf_parse_state_t s_state = TF_STATE_SOF;
static tf_frame_t s_frame;
static uint16_t s_data_idx = 0;

static uint8_t s_head_xor = 0;
static uint8_t s_data_xor = 0;

static radar_data_t s_radar_data;
volatile bool g_uart4_tx_complete = false;

void radar_uart_callback(uart_callback_args_t *p_args) {
  if (UART_EVENT_RX_CHAR == p_args->event) {
    uint32_t next = (s_rx_write + 1u) % RADAR_RX_BUF_SIZE;
    if (next != s_rx_read) {
      s_rx_buf[s_rx_write] = (uint8_t)(p_args->data & 0xFFu);
      s_rx_write = next;
    }
  } else if (UART_EVENT_TX_COMPLETE == p_args->event) {
    g_uart4_tx_complete = true;
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
      if (s_radar_data.is_human == 0u) {
        s_radar_data.height_updated = false; // 无人时强制清空高度有效标志，杜绝假低位
        s_radar_data.height_radar_m = 0.0f;
        s_radar_data.height_floor_m = 0.0f;
        s_radar_data.speed_m_s = 0.0f;
        s_radar_data.pc_v_drop_ms = 0.0f;
        s_radar_data.pc_acc_3d_ms2 = 0.0f;
        s_smooth_h_inited = false;
        s_smooth_floor_h = 1.50f;
        s_has_prev_pc = false;
      }
    }
    break;
  }

  /* 兼容性处理：若雷达确实发了 0x0E0E，直接跳过 */
  case TF_TYPE_FALL_HEIGHT:
    break;

  /* 修正3：点云坐标解析。这是获取连续距离变化的最客观来源 */
  case TF_TYPE_POINT_CLOUD: {
    s_radar_data.last_height_type = 0x0A08u;
    s_radar_data.last_height_len = f->len;
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
          float sum_floor_h = 0.0f;
          float max_floor_h = 0.0f;
          float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
          float min_x = 99.0f, max_x = -99.0f;
          float min_y = 99.0f, max_y = -99.0f;
          float min_z = 99.0f, max_z = -99.0f;
          int32_t valid_cnt = 0;

          /* 遍历目标点：执行浴帘微动近场滤除与壁装俯仰角投影变换 */
          for (int32_t i = 0; i < target_num; i++) {
            uint32_t base_off = 4u + (uint32_t)i * stride;
            float x_temp = 0.0f, y_temp = 0.0f, z_temp = 0.0f;
            uint32_t off = (stride >= 20u) ? 4u : 0u;
            memcpy(&x_temp, &f->data[base_off + off], 4);
            memcpy(&y_temp, &f->data[base_off + off + 4u], 4);
            memcpy(&z_temp, &f->data[base_off + off + 8u], 4);

            if (isnan(x_temp) || isnan(y_temp) || isnan(z_temp))
              continue;
            if (x_temp < -8.0f || x_temp > 8.0f || z_temp < -8.0f || z_temp > 8.0f)
              continue;
            if (y_temp < 0.0f)
              y_temp = -y_temp;
            if (y_temp > RADAR_HEIGHT_RAW_MAX)
              y_temp = RADAR_HEIGHT_RAW_MAX;

            /* 【关键抗浴帘滤波】浴帘微动杂波近场过滤门限：
             * 凡径向距离 < 0.40m 或法向距离 < 0.40m 的点，属于极近场与贴墙杂波，跳过！ */
            float r_point = sqrtf(x_temp * x_temp + y_temp * y_temp + z_temp * z_temp);
            if (r_point < RADAR_CURTAIN_CUTOFF_M || y_temp < 0.40f)
              continue;

            /* 【壁装倾角 35° 真实离地高度几何投影】 */
            float delta_h = y_temp * RADAR_TILT_SIN - z_temp * RADAR_TILT_COS;
            float floor_h = RADAR_MOUNT_HEIGHT - delta_h;
            /* 凡高于 1.95m 的点必为天花板、花洒横杆虚点，低于 0.10m 为地面噪波，坚决滤除！ */
            if (floor_h > 1.95f || floor_h < 0.10f)
              continue;

            sum_floor_h += floor_h;
            if (floor_h > max_floor_h) {
              max_floor_h = floor_h;
            }
            valid_cnt++;

            sum_x += x_temp; sum_y += y_temp; sum_z += z_temp;
            if (x_temp < min_x) min_x = x_temp;
            if (x_temp > max_x) max_x = x_temp;
            if (y_temp < min_y) min_y = y_temp;
            if (y_temp > max_y) max_y = y_temp;
            if (z_temp < min_z) min_z = z_temp;
            if (z_temp > max_z) max_z = z_temp;
          }

          /* 只有有效人体点数 >= 3 个点，才确认为真实在场人体！ */
          if (valid_cnt >= 3) {
            float avg_x = sum_x / (float)valid_cnt;
            float avg_y = sum_y / (float)valid_cnt;
            float avg_z = sum_z / (float)valid_cnt;

            /* 【纯客观雷达真实物理质心高度】
             * 严格依据雷达壁装高度与俯仰角的真实空间物理几何投影
             * 不加任何主观融合公式或人工区间门限，100% 以雷达真实探测到的物理点云质心为准 */
            float raw_h = sum_floor_h / (float)valid_cnt;
            if (raw_h > 2.20f) raw_h = 2.20f;
            if (raw_h < 0.05f) raw_h = 0.05f;

            if (!s_smooth_h_inited) {
              s_smooth_floor_h = raw_h;
              s_smooth_h_inited = true;
            } else {
              /* 轻量低通平滑：消除高频电子相位热噪声，确保读数稳定不乱晃，同时动作发生时极速响应 */
              float dh = (raw_h > s_smooth_floor_h) ? (raw_h - s_smooth_floor_h) : (s_smooth_floor_h - raw_h);
              if (dh < 0.02f) {
                // 原地静止极微晃动时锁定防抖
                s_smooth_floor_h = 0.05f * raw_h + 0.95f * s_smooth_floor_h;
              } else if (dh >= 0.06f) {
                // 动作产生时 65% 高响应跟进，无迟滞
                s_smooth_floor_h = 0.65f * raw_h + 0.35f * s_smooth_floor_h;
              } else {
                s_smooth_floor_h = 0.25f * raw_h + 0.75f * s_smooth_floor_h;
              }
            }

            s_radar_data.height_radar_m = avg_y;
            s_radar_data.height_floor_m = s_smooth_floor_h;
            s_radar_data.height_updated = true;

            s_radar_data.pc_avg_x_m = avg_x;
            s_radar_data.pc_avg_z_m = avg_z;
            s_radar_data.pc_point_cnt = (uint16_t)valid_cnt;
            s_radar_data.pc_span_vert_m = (max_y >= min_y) ? (max_y - min_y) : 0.0f;
            float sx = (max_x >= min_x) ? (max_x - min_x) : 0.0f;
            float sz = (max_z >= min_z) ? (max_z - min_z) : 0.0f;
            s_radar_data.pc_span_horiz_m = sqrtf(sx * sx + sz * sz);
            s_radar_data.last_pc_tick = now_tick;

            float prev_speed = s_radar_data.speed_m_s;
            float calc_speed = 0.0f;

            /* 由平滑后特征高度与质心位移计算移动速度与真实垂直坠落速度 (m/s) */
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

                /* 垂直坠落速度基于平滑后高度计算，杜绝单帧白噪声假脉冲 */
                float dh = s_prev_floor_h - s_smooth_floor_h;
                float v_drop = (dh * 1000.0f) / (float)eff_dt;
                s_radar_data.pc_v_drop_ms = (v_drop > 0.0f) ? v_drop : 0.0f;
                if (s_radar_data.pc_v_drop_ms > s_radar_data.pc_peak_drop_ms) {
                  s_radar_data.pc_peak_drop_ms = s_radar_data.pc_v_drop_ms;
                }
                s_radar_data.pc_acc_3d_ms2 = ((calc_speed - prev_speed) * 1000.0f) / (float)eff_dt;
                s_prev_pc_x = avg_x;
                s_prev_pc_y = avg_y;
                s_prev_pc_z = avg_z;
                s_prev_floor_h = s_smooth_floor_h;
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
              s_prev_floor_h = s_smooth_floor_h;
              s_has_prev_pc = true;
              s_last_pc_valid_tick = now_tick;
              s_radar_data.speed_m_s = 0.0f;
              s_radar_data.pc_v_drop_ms = 0.0f;
            }
          } else {
            /* 有效点数过少 (1~2个孤立离散点为浴帘微动或空气杂波)，不采信为有效人体点云 */
            if (now_tick - s_last_pc_valid_tick > 1000u) {
              s_radar_data.height_updated = false;
              s_radar_data.speed_m_s = 0.0f;
              s_radar_data.pc_v_drop_ms = 0.0f;
              s_radar_data.pc_point_cnt = 0;
            }
          }
        }
      } else {
        /* 超过 1500ms 无有效点云或判定无人，彻底清除高度有效性与瞬时速度，杜绝假低位 */
        if (s_radar_data.is_human == 0u || (now_tick - s_last_pc_valid_tick > 1500u)) {
          s_radar_data.height_updated = false;
          s_radar_data.height_radar_m = 0.0f;
          s_radar_data.height_floor_m = 0.0f;
          s_smooth_h_inited = false;
          s_smooth_floor_h = 1.50f;
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
  /* 1. 打开 User log 信息 (包含 3D 点云 0x0A08，官方示例: 01 00 00 00 04 01 0E F5 01 00 00 00 FE) */
  uint32_t log_enable = 1u;
  Radar_SendFrame(0x010Eu, (uint8_t *)&log_enable, 4u);
  R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);
  /* 2. 同步下发雷达标准安装挂高 2.2 米 (0x0E04) */
  Radar_SendMountHeight(RADAR_MOUNT_HEIGHT);
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

  g_uart4_tx_complete = false;
  fsp_err_t err = R_SCI_UART_Write(&g_uart4_ctrl, buf, pos);
  if (FSP_SUCCESS == err) {
    uint32_t timeout = 0;
    while (!g_uart4_tx_complete && timeout < 50) {
      R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
      timeout++;
    }
  }
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

  /* 垂直坠落速度峰值衰减：每 200ms 衰减 15% */
  static uint32_t s_last_decay_tick = 0;
  if (g_system_tick - s_last_decay_tick >= 200u) {
    s_last_decay_tick = g_system_tick;
    s_radar_data.pc_peak_drop_ms *= 0.85f;
    if (s_radar_data.pc_peak_drop_ms < 0.05f) {
      s_radar_data.pc_peak_drop_ms = 0.0f;
    }
  }

  /* 自动保活与重试：若尚未确认收到 0x0A08 3D点云，每隔 2.5 秒重发一次开启指令，确保 100% 激活点云 */
  static uint32_t s_last_enable_tick = 0;
  if (!s_radar_data.height_updated && (g_system_tick - s_last_enable_tick >= 2500u)) {
    s_last_enable_tick = g_system_tick;
    Radar_PostInit();
  }
}

radar_data_t *Radar_GetData(void) { return &s_radar_data; }