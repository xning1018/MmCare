/* ================================================================
 * fall_alarm.c
 * 跌倒确认（有人 + 去抖 + 雷达链路）与非阻塞 MY1680 语音状态机
 * ================================================================ */

#include "fall_alarm.h"
#include "my1680.h"

typedef enum
{
    VOICE_IDLE = 0,
    VOICE_PLAY1,
    VOICE_WAIT_BUSY_ACTIVE1, /* 缓冲态：等待指令发送后硬件 BUSY 引脚置为忙碌 */
    VOICE_WAIT1,
    VOICE_PLAY2,
    VOICE_WAIT_BUSY_ACTIVE2, /* 缓冲态：等待指令发送后硬件 BUSY 引脚置为忙碌 */
    VOICE_WAIT2,
    VOICE_DONE
} voice_state_t;

static voice_state_t s_voice_state       = VOICE_IDLE;
static bool          s_alarm_active      = false;
static uint8_t       s_fall_confirm      = 0u;
static uint32_t      s_last_frame_count  = 0u;
static bool          s_have_last_frame  = false;
static uint32_t      s_stale_ms          = 0u;
static bool          s_radar_link_ok     = true;

void FallAlarm_Init(void)
{
    s_voice_state       = VOICE_IDLE;
    s_alarm_active      = false;
    s_fall_confirm      = 0u;
    s_last_frame_count  = 0u;
    s_have_last_frame   = false;
    s_stale_ms          = 0u;
    s_radar_link_ok     = true;
}

static bool _raw_fall_condition(const radar_data_t *p_data)
{
    /* 站立与下蹲一票否决：质心高度 > 1.0m，或者高度在下蹲区间(0.65~1.05m)且下坠停止，坚决不触发跌倒！ */
    if (p_data->height_updated && (p_data->height_floor_m > 1.00f ||
        (p_data->height_floor_m >= 0.65f && p_data->height_floor_m <= 1.05f && p_data->pc_v_drop_ms < 0.40f)))
    {
        return false;
    }
    return (p_data->is_fall == 1u) && (p_data->is_human == 1u) && s_radar_link_ok;
}

fall_alarm_status_t FallAlarm_Update(const radar_data_t *p_data, uint32_t tick_ms)
{
    fall_alarm_status_t status = { false, false, false, s_radar_link_ok };
    bool is_new_frame = false;

    if (p_data == NULL)
    {
        if (s_stale_ms > (UINT32_MAX - tick_ms))
        {
            s_stale_ms = RADAR_STALE_TIMEOUT_MS;
        }
        else
        {
            s_stale_ms += tick_ms;
        }

        if (s_stale_ms >= RADAR_STALE_TIMEOUT_MS)
        {
            s_radar_link_ok = false;
        }

        status.radar_link_ok = s_radar_link_ok;
        bool new_active = (s_fall_confirm >= FALL_CONFIRM_TICKS);
        status.rising_edge  = new_active && !s_alarm_active;
        status.falling_edge = !new_active && s_alarm_active;
        status.active       = new_active;
        s_alarm_active      = new_active;
        return status;
    }

    /* 1. 检查帧更新与链路超时 */
    if (!s_have_last_frame || (p_data->frame_count != s_last_frame_count))
    {
        s_last_frame_count = p_data->frame_count;
        s_have_last_frame  = true;
        s_stale_ms         = 0u;
        s_radar_link_ok    = true;
        is_new_frame       = true;
    }
    else
    {
        /* 在累加前检查溢出，要求传入的 tick_ms 必须为增量时间 */
        if (s_stale_ms > (UINT32_MAX - tick_ms))
        {
            s_stale_ms = RADAR_STALE_TIMEOUT_MS; // 直接置为超时
        }
        else
        {
            s_stale_ms += tick_ms;
        }

        if (s_stale_ms >= RADAR_STALE_TIMEOUT_MS)
        {
            s_radar_link_ok = false;
        }
    }

    status.radar_link_ok = s_radar_link_ok;

    /* 2. 跌倒确认逻辑（仅在接收到新帧，或链路异常时执行，防止单帧死循环累加） */
    if (is_new_frame || !s_radar_link_ok)
    {
        if (_raw_fall_condition(p_data))
        {
            if (s_fall_confirm < FALL_CONFIRM_TICKS)
            {
                s_fall_confirm++;
            }
        }
        else
        {
            s_fall_confirm = 0u;
        }
    }

    /* 3. 边沿与状态更新 */
    bool new_active = (s_fall_confirm >= FALL_CONFIRM_TICKS);

    status.rising_edge  = new_active && !s_alarm_active;
    status.falling_edge = !new_active && s_alarm_active;
    status.active       = new_active;
    s_alarm_active      = new_active;

    return status;
}

void FallAlarm_VoiceStart(void)
{
    if ((VOICE_IDLE == s_voice_state) || (VOICE_DONE == s_voice_state))
    {
        s_voice_state = VOICE_PLAY1;
    }
}

void FallAlarm_VoiceTick(void)
{
    switch (s_voice_state)
    {
        case VOICE_PLAY1:
            if (MY1680_PlayMusic_SubDir(MY1680_ALERT_FOLDER, MY1680_ALERT_TRACK_1) != 0u)
            {
                s_voice_state = VOICE_WAIT_BUSY_ACTIVE1;
            }
            break;

        case VOICE_WAIT_BUSY_ACTIVE1:
            /* 假设 1 代表忙碌。等待芯片响应指令并开始播放 */
            if (MY1680_GetBusyStatus() == 1u)
            {
                s_voice_state = VOICE_WAIT1;
            }
            break;

        case VOICE_WAIT1:
            /* 假设 0 代表空闲。等待播放完毕 */
            if (MY1680_GetBusyStatus() == 0u)
            {
                s_voice_state = VOICE_PLAY2;
            }
            break;

        case VOICE_PLAY2:
            if (MY1680_PlayMusic_SubDir(MY1680_ALERT_FOLDER, MY1680_ALERT_TRACK_2) != 0u)
            {
                s_voice_state = VOICE_WAIT_BUSY_ACTIVE2;
            }
            break;

        case VOICE_WAIT_BUSY_ACTIVE2:
            if (MY1680_GetBusyStatus() == 1u)
            {
                s_voice_state = VOICE_WAIT2;
            }
            break;

        case VOICE_WAIT2:
            if (MY1680_GetBusyStatus() == 0u)
            {
                s_voice_state = VOICE_DONE;
            }
            break;

        case VOICE_IDLE:
        case VOICE_DONE:
        default:
            break;
    }
}

void FallAlarm_VoiceStop(void)
{
    MY1680_Stop();
    s_voice_state = VOICE_IDLE;
}

