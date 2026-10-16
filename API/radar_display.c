/* ================================================================
 * radar_display.c
 * LD6002C OLED 显示（与 fall_alarm 报警状态对齐）
 * ================================================================ */
#if 0

#include "radar_display.h"
#include <string.h>

#define LINE_WIDTH 16u

static uint32_t s_last_frm_shown = 0u;
static uint32_t s_refresh_timer = 0u; /* 300ms 全屏刷新计时器 */
static bool s_last_link_ok = true;
static bool s_last_alarm_active = false;
static uint8_t s_last_raw_fall = 0u;

static void _f2s(char *dst, float val, uint8_t decimals) {
  if (val < 0.0f) {
    val = 0.0f;
  }
  if (val > 99.9f) {
    dst[0] = '>';
    dst[1] = '9';
    dst[2] = '9';
    dst[3] = '\0';
    return;
  }

  uint32_t mul = 1u;
  for (uint8_t i = 0; i < decimals; i++) {
    mul *= 10u;
  }

  uint32_t int_part = (uint32_t)val;
  uint32_t frac_part = (uint32_t)((val - (float)int_part) * (float)mul + 0.5f);
  if (frac_part >= mul) {
    int_part++;
    frac_part = 0u;
  }

  uint8_t pos = 0;
  if (int_part >= 10u) {
    dst[pos++] = (char)('0' + (int_part / 10u));
  }
  dst[pos++] = (char)('0' + (int_part % 10u));
  dst[pos++] = '.';
  for (int8_t d = (int8_t)(decimals - 1u); d >= 0; d--) {
    uint32_t p = 1u;
    for (int8_t i = 0; i < d; i++) {
      p *= 10u;
    }
    dst[pos++] = (char)('0' + ((frac_part / p) % 10u));
  }
  dst[pos] = '\0';
}

static void _u2s5(char *dst, uint32_t val) {
  char tmp[6];
  uint8_t i = 0;

  if (val == 0u) {
    tmp[i++] = '0';
  } else {
    while ((val > 0u) && (i < 5u)) {
      tmp[i++] = (char)('0' + (val % 10u));
      val /= 10u;
    }
  }

  uint8_t len = i;
  uint8_t pad = (5u > len) ? (5u - len) : 0u;
  uint8_t pos = 0;

  for (uint8_t p = 0; p < pad; p++) {
    dst[pos++] = ' ';
  }
  for (int8_t j = (int8_t)(len - 1); j >= 0; j--) {
    dst[pos++] = tmp[j];
  }
  dst[pos] = '\0';
}

/* 将字符串右填充空格至 16 列后写入 OLED */
static void _show_line(uint8_t line, const char *text) {
  char line_buf[LINE_WIDTH + 1u];
  uint8_t i = 0;

  while ((text[i] != '\0') && (i < LINE_WIDTH)) {
    line_buf[i] = text[i];
    i++;
  }
  while (i < LINE_WIDTH) {
    line_buf[i++] = ' ';
  }
  line_buf[LINE_WIDTH] = '\0';
  OLED_ShowString(line, 1, line_buf);
}

static void _draw_link_line(bool link_ok) {
  if (link_ok) {
    _show_line(1, "LD:OK           ");
  } else {
    _show_line(1, "LD:--           ");
  }
}

static void _draw_fall_alarm_line(uint8_t raw_fall, bool alarm_active) {
  if (alarm_active) {
    _show_line(2, "Raw:1  Alm:ON   ");
  } else if (raw_fall != 0u) {
    _show_line(2, "Raw:1  Alm:--   ");
  } else {
    _show_line(2, "Raw:0  Alm:--   ");
  }
}

/* 在 8 列内输出 "Gnd:X.Xm" 或 "Gnd: --- " */
static void _format_gnd_field(char *dst8, float height_floor_m) {
  char num[6];
  uint8_t i = 0;
  uint8_t k = 0;

  dst8[0] = 'G';
  dst8[1] = 'n';
  dst8[2] = 'd';
  dst8[3] = ':';
  _f2s(num, height_floor_m, 1u);
  i = 4;
  while ((num[k] != '\0') && (i < 7u)) {
    dst8[i++] = num[k++];
  }
  dst8[i++] = 'm';
  while (i < 8u) {
    dst8[i++] = ' ';
  }
}

/* 第3行整行：Hum(8列) + Gnd(8列) */
static void _draw_hum_gnd_line(radar_data_t *data) {
  char line_buf[LINE_WIDTH + 1u];

  if (data->is_human != 0u) {
    memcpy(line_buf, "Hum:yes ", 8);
  } else {
    memcpy(line_buf, "Hum:no  ", 8);
  }

  /* 雷达固件不输出距离数据时显示 '---'，有数据时正常显示 */
  if (data->height_updated) {
    _format_gnd_field(line_buf + 8, data->height_floor_m);
  } else {
    memcpy(line_buf + 8, "Gnd: ---", 8);
  }

  line_buf[LINE_WIDTH] = '\0';
  OLED_ShowString(3, 1, line_buf);
}

static void _show_hex4(char *dst, uint16_t val) {
  const char hex[] = "0123456789ABCDEF";
  dst[0] = hex[(val >> 12) & 0xFu];
  dst[1] = hex[(val >> 8) & 0xFu];
  dst[2] = hex[(val >> 4) & 0xFu];
  dst[3] = hex[(val >> 0) & 0xFu];
}

static void _draw_debug_line(radar_data_t *data) {
  char buf[LINE_WIDTH + 1u];
  char tmp[8];

  /*
   * 第4行诊断: Fr:xxxxx T:xxxx
   * Fr = 帧计数
   * T  = 最后收到的消息类型
   */
  _u2s5(tmp, data->frame_count % 100000u);
  buf[0] = 'F';
  buf[1] = 'r';
  buf[2] = ':';
  buf[3] = tmp[0];
  buf[4] = tmp[1];
  buf[5] = tmp[2];
  buf[6] = tmp[3];
  buf[7] = tmp[4];
  buf[8] = ' ';
  buf[9] = 'T';
  buf[10] = ':';

  _show_hex4(tmp, data->last_type);
  buf[11] = tmp[0];
  buf[12] = tmp[1];
  buf[13] = tmp[2];
  buf[14] = tmp[3];
  buf[15] = ' ';

  buf[LINE_WIDTH] = '\0';
  OLED_ShowString(4, 1, buf);
}

void RadarDisplay_Init(void) {
  s_last_frm_shown = 0u;
  s_last_link_ok = true;
  s_last_alarm_active = false;
  s_last_raw_fall = 0u;

  OLED_Clear();
  _draw_link_line(true);
  _draw_fall_alarm_line(0u, false);
  _show_line(3, "Hum:--  Gnd: --- ");
  _show_line(4, "Frm:     0 R:--m ");
}

void RadarDisplay_Update(radar_data_t *data, const fall_alarm_status_t *alarm) {
  bool link_ok = (alarm != NULL) ? alarm->radar_link_ok : true;
  bool alarm_active = (alarm != NULL) ? alarm->active : false;

  /* L1: 链路状态，变化立即刷新 */
  if (link_ok != s_last_link_ok) {
    s_last_link_ok = link_ok;
    _draw_link_line(link_ok);
  }

  /* L2: 跌倒状态，变化立即刷新 */
  if (data->fall_updated || (s_last_alarm_active != alarm_active) ||
      (s_last_raw_fall != data->is_fall)) {
    data->fall_updated = false;
    s_last_raw_fall = data->is_fall;
    s_last_alarm_active = alarm_active;
    _draw_fall_alarm_line(data->is_fall, alarm_active);
  }

  /* 每 300ms 全屏刷新 L3 + L4，确保即使数据为 0 也能看到当前状态 */
  s_refresh_timer += 10u;
  if (s_refresh_timer >= 300u) {
    s_refresh_timer = 0u;

    /* L3: 有人状态 + 离地高度 */
    _draw_hum_gnd_line(data);

    /* L4: 帧计数 + TYPE 诊断 */
    _draw_debug_line(data);
  }
}

#endif
