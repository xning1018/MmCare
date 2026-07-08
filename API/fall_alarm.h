#ifndef FALL_ALARM_H_
#define FALL_ALARM_H_

#include <stdbool.h>
#include <stdint.h>
#include "radar_uart.h"

/* MY1680 SD 卡：文件夹 55，曲目 001/002 对应参数 1、2 */
#define MY1680_ALERT_FOLDER    55u
#define MY1680_ALERT_TRACK_1   1u
#define MY1680_ALERT_TRACK_2   2u
#define MY1680_DEFAULT_VOLUME  25u

/* 跌倒确认帧数；雷达 2s 无新帧视为链路异常 */
#define FALL_CONFIRM_TICKS       3u
#define RADAR_STALE_TIMEOUT_MS   2000u

typedef struct
{
    bool active;
    bool rising_edge;
    bool falling_edge;
    bool radar_link_ok;
} fall_alarm_status_t;

void FallAlarm_Init(void);
fall_alarm_status_t FallAlarm_Update(const radar_data_t *p_data, uint32_t tick_ms);
void FallAlarm_VoiceStart(void);
void FallAlarm_VoiceTick(void);
void FallAlarm_VoiceStop(void);

#endif /* FALL_ALARM_H_ */

