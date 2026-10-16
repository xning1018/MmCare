#ifndef RADAR_DISPLAY_H_
#define RADAR_DISPLAY_H_

#include "fall_alarm.h"
#include "radar_uart.h"
#include "oled.h"

/* OLED 128x64，8x16 字体，4 行 x 16 列
 *   第1行: 标题 + 雷达链路  "LD6002C Lk:OK  "
 *   第2行: 雷达原始 + 报警  "Raw:1  Alm:ON   "
 *   第3行: 有人 + 离地高度    "Hum:yes Gnd:1.5m"
 *   第4行: 帧计数 + 距雷达    "Frm:00123 R:1.5m"
 */
#define RADAR_DISPLAY_FRM_STEP  50u

void RadarDisplay_Init(void);
void RadarDisplay_Update(radar_data_t *data, const fall_alarm_status_t *alarm);
#endif /* RADAR_DISPLAY_H_ */
