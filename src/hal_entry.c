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

// ==========================================
// 模式切换宏定义
// ==========================================
// 1 = 数据采集模式 (通过 printf 打印 CSV 格式数据用于训练)
// 0 = 正常运行模式 (原有的 4G 上传及业务逻辑)
#define DATA_COLLECT_MODE 0
// ==========================================

#if defined(__ARMCC_VERSION)
/* 安全重定向 fputc，彻底防止未开启半主机调试器时的 HardFault 宕机 */
int fputc(int ch, FILE *f) {
  (void)f;
  return ch;
}
#endif

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
volatile uint32_t g_last_ld2450_uart_tick = 0; // LD2450 串口通信保活心跳

void System_Yield(void) {
  static volatile bool s_in_yield = false;
  if (s_in_yield) {
    return;
  }
  s_in_yield = true;

  uint32_t current_tick = g_system_tick;

  /* LD2450 串口通信看门狗与自愈机制：
   * LD2450 正常每 100ms 发送一帧数据。
   * 若系统就绪后超过 1500ms 没有任何字节进入中断，仅安全重置软件协议状态机，绝不切断硬件总线时钟！ */
  if (g_system_ready) {
    static uint32_t last_uart_watchdog_tick = 0;
    if ((current_tick - g_last_ld2450_uart_tick > 1500) &&
        (current_tick - last_uart_watchdog_tick > 1500)) {
      last_uart_watchdog_tick = current_tick;
      g_last_ld2450_uart_tick = current_tick;
      LD2450_Init();
    }
  }

  Radar_Process();






/* 若一直没有收到高度数据，说明 User log
   * 没有成功开启（可能雷达启动慢漏掉了指令），周期性重发指令 */
  radar_data_t *p_radar = Radar_GetData();
  static uint32_t last_log_retry_tick = 0;
  if (!p_radar->height_updated && (current_tick - last_log_retry_tick > 2000)) {
    last_log_retry_tick = current_tick;
    Radar_PostInit();
  }

  static int empty_target_count = 0;
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
      empty_target_count = 0;
      int cur_x = last_ld2450_data.target[best_idx].x;
      int cur_y = last_ld2450_data.target[best_idx].y;
      int cur_v = last_ld2450_data.target[best_idx].speed;

      if (cur_v > 200 || cur_v < -200 ||
          (!is_first_frame &&
           (cur_x - history_x > 1100 || cur_x - history_x < -1100)) ||
          (!is_first_frame &&
           (cur_y - history_y > 8000 || cur_y - history_y < -8000))) {
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

        /* 计算横向速度 (X轴方向)，单位转换为 cm/s (X单位为mm, dt为ms) */
        uint32_t dt = current_tick - last_valid_target_tick;
        if (dt > 0) {
          upload_v = ((cur_x - history_x) * 100) / (int)dt;
        }
      } else {
        upload_v = 0;
      }

      is_first_frame = false;
      history_x = cur_x;
      history_y = cur_y;

      upload_x = cur_x;
      upload_y = cur_y;

      last_valid_target_tick = current_tick;

#if DATA_COLLECT_MODE
      // 数据采集模式：打印 CSV 格式
      // 格式：Timestamp(ms), X(mm), Y(mm), Velocity(cm/s)
      printf("%u,%d,%d,%d\r\n", current_tick, cur_x, cur_y, upload_v);
#endif
    } else {
      /* 收到帧但三个人形目标均不存在 */
      empty_target_count++;
      if (empty_target_count >= 5) { // 连续 5 帧(约500ms)判定无人，快速归零
        history_x = 0;
        history_y = 0;
        upload_x = 0;
        upload_y = 0;
        upload_v = 0;
        is_first_frame = true;
      }
    }
  }

  if (current_tick - last_valid_target_tick > 1500) {
    history_x = 0;
    history_y = 0;
    upload_x = 0;
    upload_y = 0;
    upload_v = 0;
    is_first_frame = true;
  }

  /* 双雷达融合大招：LD6002C 判定静态很准但变 Y 慢，LD2450 侦测运动瞬间响应。
     只要 LD2450 看到人在动（2秒内），强制认为有人，解决很难变回 Y 的问题！ */
  if (current_tick - last_valid_target_tick < 2000) {
    p_radar->is_human = 1;
  }

  if (current_tick - last_night_light_tick >= 500) {
    last_night_light_tick = current_tick;
    cur_light_status = LightSensor_GetStatus();
  }

  is_night_light = 0;
  radar_data_t *rt_radar = Radar_GetData();
  
  /* 正宗智能夜灯逻辑：
     必须【同时满足】环境暗(1) 并且 雷达检测到人(1)，才开启210灯。
     如果天亮了(0)，或者雷达判定无人了(0)，都会自动关灯。*/
  if (cur_light_status == 1 && rt_radar->is_human == 1) {
    is_night_light = 1;
  }

  static int buzzer_toggle_count = 0;
  static int alarm_playing = 0;

  if ((is_falling == 1 || is_falling == 11 || is_falling == 111) && !alarm_playing) {
    alarm_playing = 1;
    buzzer_toggle_count = 0;
    last_buzzer_tick = current_tick;
  }

  if (alarm_playing) {
    if (current_tick - last_buzzer_tick >= 500) {
      last_buzzer_tick = current_tick;
      if (buzzer_toggle_count < 18) {
        Buzzer_Toggle();
        buzzer_toggle_count++;
      } else {
        Buzzer_Off();
        buzzer_toggle_count = 0;
        alarm_playing = 0;
        
        // 报警 9 秒（18次翻转）后自动停止，并强制清除底层雷达跌倒状态，防止一直反复报警
        p_radar->is_fall = 0;
        is_falling = 0;
        anomaly_flag = 0;
      }
    }
  } else {
    Buzzer_Off();
    buzzer_toggle_count = 0;
  }

  if (alarm_playing) {
    if (buzzer_toggle_count < 18) {
      if (buzzer_toggle_count % 2 == 0) {
        // 灯光全灭 (闪烁暗状态)
        LED_Off();
        LED202_Off();
        LED604_Off();
        LED210_Off();
      } else {
        // 跌倒红灯亮起 (闪烁亮状态)
        LED_Off();
        LED202_On();
        LED604_On();
        LED210_Off();
      }
    } else {
      LED_Off();
      LED202_On();
      LED604_On();
      LED210_Off();
    }
  } else if (is_night_light) {
    LED_Off();
    LED202_Off();
    LED604_On();
    LED210_On();
  } else {
    LED_On();
    LED202_Off();
    LED604_Off();
    LED210_Off();
  }

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

  static uint32_t fall_111_start_tick = 0;
  static int fall_111_waiting = 0;

  if (fall_val == 1 && anomaly_flag == 1) {
    fall_val = 11;
    fall_111_waiting = 0;
  } else if (fall_val == 0 && anomaly_flag == 1) {
    if (fall_111_waiting == 0) {
      fall_111_waiting = 1;
      fall_111_start_tick = current_tick;
    }
  } else if (anomaly_flag == 0) {
    if (fall_111_waiting == 2) {
      fall_111_waiting = 0;
    }
  }

  if (fall_111_waiting == 1) {
    if (current_tick - fall_111_start_tick >= 1800) {
      fall_111_waiting = 2;
    } else {
      fall_val = 0;
    }
  }

  if (fall_111_waiting == 2) {
    fall_val = 111;
  }

  static int prev_fall_val = 0;
  if (fall_val != prev_fall_val && fall_val != 0) {
    should_upload = 1;
    MY1680_SetVol(30);
    MY1680_PlayMusic_SubDir(55, 1);
  }
  prev_fall_val = fall_val;
  is_falling = fall_val;

  /* 始终刷新 OLED 显示（即使未联网也要显示参数与报警） */
  static uint32_t last_oled_tick = 0;
  if (current_tick - last_oled_tick >= 500) {
    last_oled_tick = current_tick;
    char oled_buf[20];

    float current_temp = NTC_GetTemperature();
    sprintf(oled_buf, "4G:%-6sT:%4.1fC", g_4g_ok ? "OK" : g_4g_status_tag, current_temp);
    for (int i = (int)strlen(oled_buf); i < 16; i++)
      oled_buf[i] = ' ';
    oled_buf[16] = '\0';
    OLED_ShowString(1, 1, oled_buf);

    sprintf(oled_buf, "HUM:%s FAL:%s", p_radar->is_human ? "Y" : "N",
            is_falling ? "Y" : "N");
    for (int i = (int)strlen(oled_buf); i < 16; i++)
      oled_buf[i] = ' ';
    oled_buf[16] = '\0';
    OLED_ShowString(2, 1, oled_buf);

    sprintf(oled_buf, "X:%-5d Y:%-5d", upload_x, upload_y);
    for (int i = (int)strlen(oled_buf); i < 16; i++)
      oled_buf[i] = ' ';
    oled_buf[16] = '\0';
    OLED_ShowString(3, 1, oled_buf);

    // 第 4 行：若未连上4G，优先显示故障诊断信息；就绪后显示雷达高度参数
    if (g_4g_status != ML307R_STAT_NET_OK) {
      sprintf(oled_buf, "%-16s", g_4g_diag_info);
    } else {
      sprintf(oled_buf, "T%04X L%d D%d", p_radar->last_height_type,
              p_radar->last_height_len, (int)(p_radar->height_floor_m * 100));
    }
    for (int i = (int)strlen(oled_buf); i < 16; i++)
      oled_buf[i] = ' ';
    oled_buf[16] = '\0';
    OLED_ShowString(4, 1, oled_buf);
  }

  s_in_yield = false;
}

void hal_entry(void) {
  /* === 1. 本地外设与传感器初始化 === */
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(1, 1, "System Init...");

  NTC_Init(); // 初始化 NTC 温度 ADC

  Buzzer_Init();
  MY1680_Init();
  LightSensor_Init(); // 光敏传感器初始化
  LED_Init();         // 初始化灯带管脚

  // 开机蜂鸣器 bibi 两声
  Buzzer_On();
  R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
  Buzzer_Off();
  R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
  Buzzer_On();
  R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
  Buzzer_Off();

  Radar_Init();     // 初始化雷达串口
  FallAlarm_Init(); // 初始化跌倒报警状态机
  LD2450_Init();    // 初始化 LD2450 雷达
  R_SCI_UART_Open(&g_uart_3_ctrl,
                  &g_uart_3_cfg); // 必须打开对应串口，否则收不到雷达数据
  g_last_ld2450_uart_tick = g_system_tick;

  /* === 2. 网络模组快速探针 (绝不阻塞本地监控) === */
  R_AGT_Open(&g_timer0_ctrl, &g_timer0_cfg);
  R_AGT_Start(&g_timer0_ctrl); // 开启系统滴答

  ML307R_Init();
  Radar_PostInit(); // 向 LD6002C 发送开启 User log 指令

  /* 快速非阻塞探测 4G 模组状态 (单次探测，绝不长时间卡死) */
  if (ML307R_CheckNetwork()) {
    g_4g_ok = 1;
  } else {
    g_4g_ok = 0;
  }

  g_system_ready = 1;

  /* === 3. 立即呈现全场景实时监控界面 === */
  OLED_Clear();
  System_Yield(); // 立即执行首轮传感器采集并刷新 OLED 4行监控参数

  /* 指向搭建好的轻量服务器 IP 和端口（对齐 MMCare 2.0.0 规范） */
  const char *http_url = "182.254.135.92:8080/data?api_key=mmcare-demo-key-2026";
  char json_buf[192];

  while (1) {
    System_Yield(); // 每帧后台维护雷达和蜂鸣器

    static uint32_t last_upload_tick = 0;
    static uint32_t last_net_retry_tick = 0;
    static int http_fail_count = 0;       // 连续失败计数器
    static int fall_already_reported = 0; // 跌倒事件单次上报锁，防止9秒内重复调用HTTP

    uint32_t current_tick = g_system_tick;

    // 跌倒结束后恢复上报锁
    if (is_falling == 0) {
      fall_already_reported = 0;
    }

    // 判断是否需要上传数据：
    // 1. 跌倒报警触发且尚未向云端上报过：立即发起上报
    // 2. 正常无报警状态：每 2000ms 定时上报一次心跳数据（加上 4G 通信耗时约 3s，整网刷新周期精准为 5 秒）
    bool need_upload = false;
    if (is_falling != 0) {
      if (!fall_already_reported) {
        need_upload = true;
        fall_already_reported = 1;
      }
    } else if (current_tick - last_upload_tick >= 2500) {
      need_upload = true;
    }

    if (need_upload) {
      should_upload = 0;

      // 核心控制：仅在联网状态下 (g_4g_ok == 1) 发起 HTTP POST 上传！
      // 断网时 (g_4g_ok == 0) 直接跳过 HTTP 阻塞，本地声光/语音报警立即响应，不受任何网络超时卡顿影响！
      if (g_4g_ok) {
        float current_temp = NTC_GetTemperature();
        radar_data_t *snap = Radar_GetData(); // 只取一次快照，保证同一帧数据一致
        int temp_scaled = (int)(current_temp * 10.0f);
        int is_human = snap->is_human ? 1 : 0;
        int is_fall = (is_falling != 0) ? 1 : 0;
        int dist_cm = (int)upload_y;
        if (dist_cm < 0) dist_cm = 0;

        sprintf(json_buf,
                "{\"device_id\":101,"
                "\"temp\":%d,"
                "\"human\":%d,"
                "\"fall\":%d,"
                "\"dist\":%d,"
                "\"x\":%d,"
                "\"y\":%d,"
                "\"v\":%d}",
                temp_scaled, is_human, is_fall, dist_cm, (int)upload_x, (int)upload_y, (int)upload_v);

        if (ML307R_HttpPost(http_url, json_buf)) {
          g_4g_ok = 1;
          http_fail_count = 0; // 成功一次，立刻清零
        } else {
          http_fail_count++;
          /* 连续失败 3 次才判定为断网 */
          if (http_fail_count >= 3) {
            g_4g_ok = 0;
            last_net_retry_tick = current_tick;
          }
        }
        last_upload_tick = g_system_tick; // 完成上传后打卡，精准控制下一次间隔
      } else {
        last_upload_tick = current_tick;
      }
    }

    // 断网或未就绪状态下 (g_4g_ok == 0)，后台每隔 8 秒轻量化自动探测入网，确保主循环雷达流畅
    if (!g_4g_ok && (current_tick - last_net_retry_tick >= 8000)) {
      last_net_retry_tick = current_tick;
      if (ML307R_CheckNetwork()) {
        g_4g_ok = 1;
        http_fail_count = 0;
      } else {
        g_4g_ok = 0;
      }
    }

    // 轻量让出 CPU，消除阻塞延时与低功耗模式对 256000 高速串口 FIFO 的溢出冲击
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
  }
}

void audio_timer_callback(timer_callback_args_t *p_args) { (void)p_args; }
void wifi_uart_callback(uart_callback_args_t *p_args) { (void)p_args; }

// 连接 FSP 的 UART3 回调与 LD2450 接收函数
void user_uart_callback(uart_callback_args_t *p_args) {
  if (UART_EVENT_RX_CHAR == p_args->event) {
    g_last_ld2450_uart_tick = g_system_tick;
    LD2450_UART_Callback((uint8_t)p_args->data);
  } else if (p_args->event & (UART_EVENT_ERR_OVERFLOW | UART_EVENT_ERR_FRAMING | UART_EVENT_ERR_PARITY)) {
    /* 硬件 FIFO 溢出 / 帧错误自愈：
     * FSP 的 sci_uart_eri_isr 已在进入回调前清除了错误标志。
     * 此处仅需重置 LD2450 接收协议状态机，避免残帧影响后续解析即可，绝不强关硬件 RE 或读写保留位 */
    LD2450_ResetState();
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
