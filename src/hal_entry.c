/* hal_entry.c */
#include "buzzer.h"
#include "fall_alarm.h" // 引入跌倒逻辑头文件
#include "hal_data.h"
#include "ld2450.h" // 引入 LD2450 雷达头文件
#include "led.h"
#include "light_sensor.h"
#include "ml307r.h"
#include "my1680.h"
#include "ntc_temp.h" // 引入温度传感器头文件
#include "oled.h"
#include "radar_uart.h" // 引入雷达处理头文件 (LD6002C)
#include <stdio.h>
#include <string.h>


FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

// 低功耗模式与精确系统时基
volatile uint32_t g_system_tick = 0; // 绝对系统时间，由 AGT 定时器维护

void system_tick_callback(timer_callback_args_t *p_args) {
  if (TIMER_EVENT_CYCLE_END == p_args->event) {
    g_system_tick += 10;
  }
}

// 系统后台维护任务（供 HTTP 阻塞期间调用）
static int history_x = 0;

static int history_y = 0;
static int anomaly_flag = 0;
static int upload_x = 0;
static int upload_y = 0;
static int upload_v = 0;
static int is_falling = 0;
static int is_night_light = 0;
static uint8_t cur_light_status = 0;
static uint32_t last_night_light_tick = 0;
static uint32_t last_buzzer_tick = 0;
static uint32_t last_heartbeat_tick = 0;
static uint32_t last_valid_target_tick = 0;
static uint32_t last_fall_tick = 0;
static int should_upload = 0;
static bool is_first_frame = true;
static int g_4g_ok = 0;
static int g_system_ready = 0;

void System_Yield(void) {
  uint32_t current_tick = g_system_tick;

  Radar_Process();

  LD2450_Data_t last_ld2450_data;
  if (LD2450_Get_Data(&last_ld2450_data)) {
    int best_idx = -1;
    for (int i = 0; i < 3; i++) {
      if (last_ld2450_data.target[i].is_valid) {
        best_idx = i;
        break;
      }
    }

    if (best_idx != -1) {
      int cur_x = last_ld2450_data.target[best_idx].x;
      int cur_y = last_ld2450_data.target[best_idx].y;
      int cur_v = last_ld2450_data.target[best_idx].speed;

      if (cur_v > 40 || cur_v < -40 ||
          (!is_first_frame &&
           (cur_x - history_x > 400 || cur_x - history_x < -400)) ||
          (!is_first_frame &&
           (cur_y - history_y > 400 || cur_y - history_y < -400))) {
        anomaly_flag = 1;
      }

      if (!is_first_frame) {
        int abs_dx =
            (cur_x - history_x > 0) ? (cur_x - history_x) : (history_x - cur_x);
        int abs_dy =
            (cur_y - history_y > 0) ? (cur_y - history_y) : (history_y - cur_y);
        int abs_v = (cur_v > 0) ? cur_v : -cur_v;

        if (abs_dx > 50 || abs_dy > 50 || abs_v > 50) {
          should_upload = 1;
        }
      }

      is_first_frame = false;
      history_x = cur_x;
      history_y = cur_y;

      upload_x = cur_x;
      upload_y = cur_y;
      upload_v = cur_v;

      last_valid_target_tick = current_tick;
    }
  }

  if (current_tick - last_valid_target_tick > 3000) {
    history_x = 0;
    history_y = 0;
    upload_x = 0;
    upload_y = 0;
    upload_v = 0;
    is_first_frame = true;
  }

  if (current_tick - last_night_light_tick >= 500) {
    last_night_light_tick = current_tick;
    cur_light_status = LightSensor_GetStatus();
  }

  is_night_light = 0;
  if (cur_light_status == 1) {
    radar_data_t *rt_radar = Radar_GetData();
    if (rt_radar->is_human == 1 &&
        (current_tick - last_valid_target_tick < 3000)) {
      is_night_light = 1;
    }
  }

  if (is_falling == 1 || is_falling == 11) {
    if (current_tick - last_buzzer_tick >= 500) {
      last_buzzer_tick = current_tick;
      Buzzer_Toggle();
    }
  } else {
    Buzzer_Off();
    last_buzzer_tick = current_tick;
  }

  if (is_falling == 1 || is_falling == 11) {
    LED_Off();
    LED202_On();
    LED604_Off();
  } else if (is_night_light) {
    LED_Off();
    LED202_Off();
    LED604_On();
  } else {
    LED_On();
    LED202_Off();
    LED604_Off();
  }

  radar_data_t *p_radar = Radar_GetData();
  static fall_alarm_status_t alarm = {0};
  if (current_tick - last_fall_tick >= 500) {
    last_fall_tick = current_tick;
    uint32_t delta_time = current_tick - last_heartbeat_tick;
    if (delta_time == 0)
      delta_time = 500;
    alarm = FallAlarm_Update(p_radar, delta_time);
    last_heartbeat_tick = current_tick;
  }

  int fall_val = alarm.active ? 1 : 0;
  if (fall_val == 1 && anomaly_flag == 1) {
    fall_val = 11;
  }

  static int prev_fall_val = 0;
  if (fall_val != prev_fall_val && fall_val != 0) {
    should_upload = 1;
    MY1680_SetVol(30);
    MY1680_PlayMusic_SubDir(55, 1);
  }
  prev_fall_val = fall_val;
  is_falling = fall_val;

  if (g_system_ready) {
    static uint32_t last_oled_tick = 0;
    if (current_tick - last_oled_tick >= 500) {
      last_oled_tick = current_tick;
      char oled_buf[20];

      float current_temp = NTC_GetTemperature();
      sprintf(oled_buf, "4G:%s T:%4.1fC", g_4g_ok ? "OK" : "NO", current_temp);
      for (int i = strlen(oled_buf); i < 16; i++)
        oled_buf[i] = ' ';
      oled_buf[16] = '\0';
      OLED_ShowString(1, 1, oled_buf);

      sprintf(oled_buf, "HUM:%s FAL:%s", p_radar->is_human ? "Y" : "N",
              is_falling ? "Y" : "N");
      for (int i = strlen(oled_buf); i < 16; i++)
        oled_buf[i] = ' ';
      oled_buf[16] = '\0';
      OLED_ShowString(2, 1, oled_buf);

      sprintf(oled_buf, "X:%-5d Y:%-5d", upload_x, upload_y);
      for (int i = strlen(oled_buf); i < 16; i++)
        oled_buf[i] = ' ';
      oled_buf[16] = '\0';
      OLED_ShowString(3, 1, oled_buf);

      sprintf(oled_buf, "V:%-13d", upload_v);
      for (int i = strlen(oled_buf); i < 16; i++)
        oled_buf[i] = ' ';
      oled_buf[16] = '\0';
      OLED_ShowString(4, 1, oled_buf);
    }
  }
}

void hal_entry(void) {
  /* === 1. 本地外设与传感器初始化 === */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(1, 1, "System Init...");

  NTC_Init(); // 初始化 NTC 温度 ADC

  Buzzer_Init();
  MY1680_Init();
  LightSensor_Init(); // 【新增】光敏传感器初始化
  LED_Init();         // 初始化灯带管脚

  // P606 夜灯引脚已废弃，直接使用灯带白灯控制

  // 开机蜂鸣器 bibi 两声
  Buzzer_On();
  R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
  Buzzer_Off();
  R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
  Buzzer_On();
  R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
  Buzzer_Off();

  // (开机语音已移除，改为跌倒时播报)

  Radar_Init();     // 初始化雷达串口
  FallAlarm_Init(); // 初始化跌倒报警状态机
  LD2450_Init();    // 初始化 LD2450 雷达
  R_SCI_UART_Open(&g_uart_3_ctrl,
                  &g_uart_3_cfg); // 必须打开对应串口，否则收不到雷达数据

  /* === 2. 网络模组初始化 === */
  R_AGT_Open(&g_timer0_ctrl, &g_timer0_cfg);
  R_AGT_Start(&g_timer0_ctrl); // 必须在网络初始化之前开启，否则 System_Yield
                               // 获取不到滴答

  ML307R_Init();
  System_Yield();
  R_BSP_SoftwareDelay(2000, BSP_DELAY_UNITS_MILLISECONDS);
  System_Yield();

  Radar_PostInit(); // 向 LD6002C 发送开启 User log
                    // 的指令，否则高度和点云数据将永远是 0

  OLED_ShowString(1, 1, "WAITING AT... ");
  while (!ML307R_SendCmd("AT\r\n", "OK", 1000)) {
    R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
    System_Yield();
  }

  OLED_ShowString(1, 1, "CHECK SIM...  ");
  while (!ML307R_SendCmd("AT+CPIN?\r\n", "READY", 1000)) {
    R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
    System_Yield();
  }

  OLED_ShowString(1, 1, "SEARCH NET... ");
  while (1) {
    if (ML307R_SendCmd("AT+CEREG?\r\n", "0,1", 2000))
      break;
    if (ML307R_SendCmd("AT+CEREG?\r\n", "0,5", 2000))
      break;
    R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS);
    System_Yield();
  }
  OLED_ShowString(1, 1, "BASE NET:  OK ");

  OLED_ShowString(2, 1, "ACTING PDP... ");
  while (!ML307R_InitNetwork()) {
    R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS);
    System_Yield();
  }
  OLED_ShowString(2, 1, "DIAL IP:   OK ");

  /* 指向搭建好的轻量服务器 IP 和端口 */
  const char *http_url = "182.254.135.92:8080/data";
  char json_buf[128];

  g_4g_ok = 1;
  g_system_ready = 1;
  OLED_Clear();

  R_LPM_Open(&g_lpm0_ctrl, &g_lpm0_cfg);

  while (1) {
    System_Yield(); // 每帧后台维护雷达和蜂鸣器

    static uint32_t last_upload_tick = 0;

    if (should_upload) {
      should_upload = 0;

      // 添加上传冷却机制，防止雷达连续检测到运动时疯狂发起 HTTP POST
      // 耗尽基站/模组资源
      if (g_system_tick - last_upload_tick >= 2000) {
        last_upload_tick = g_system_tick;
        anomaly_flag = 0;

        float current_temp = NTC_GetTemperature();
        radar_data_t *snap =
            Radar_GetData(); // 只取一次快照，保证同一帧数据一致
        int temp_int = (int)(current_temp * 10);
        int dist_int = (int)(snap->height_floor_m * 100);

        sprintf(json_buf,
                "{\"device_id\":101,\"temp\":%d,\"human\":%d,\"fall\":%d,"
                "\"dist\":%d,\"x\":%d,\"y\":%d,\"v\":%d}",
                temp_int, snap->is_human, is_falling, dist_int, upload_x,
                upload_y, upload_v);

        if (ML307R_HttpPost(http_url, json_buf)) {
          g_4g_ok = 1;
        } else {
          g_4g_ok = 0;
        }
      }
    }

    R_LPM_LowPowerModeEnter(&g_lpm0_ctrl);
  }
}

void audio_timer_callback(timer_callback_args_t *p_args) { (void)p_args; }
void wifi_uart_callback(uart_callback_args_t *p_args) { (void)p_args; }

// 连接 FSP 的 UART3 回调与 LD2450 接收函数
void user_uart_callback(uart_callback_args_t *p_args) {
  if (UART_EVENT_RX_CHAR == p_args->event) {
    LD2450_UART_Callback((uint8_t)p_args->data);
  }
}

void R_BSP_WarmStart(bsp_warm_start_event_t event) {
  if (BSP_WARM_START_RESET == event) {
#if BSP_FEATURE_FLASH_LP_VERSION != 0
    R_FACI_LP->DFLCTL = 1U;
#endif
  }
  if (BSP_WARM_START_POST_C == event) {
    R_IOPORT_Open(&g_ioport_ctrl, g_ioport.p_cfg);
  }
}
