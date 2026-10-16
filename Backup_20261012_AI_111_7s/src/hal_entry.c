/* hal_entry.c */
#include "ai_fall_engine.h" // 引入端侧 PINN-CfC 液态神经网络与 1024 粒子滤波跌倒引擎
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
static uint32_t g_ready_tick = 0;
volatile uint32_t g_last_ld2450_uart_tick = 0; // LD2450 串口通信保活心跳

void System_Yield(void) {
  static volatile bool s_in_yield = false;
  if (s_in_yield) {
    return;
  }
  s_in_yield = true;

  uint32_t current_tick = g_system_tick;
  bool warmup_done = (g_system_ready != 0) && ((current_tick - g_ready_tick) >= 5000u);

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
  if (!warmup_done) {
    /* 开机/复位冷启动预热期内，自动清零雷达上电瞬态残余的跌倒标志 */
    p_radar->is_fall = 0;
  }
  static uint32_t last_log_retry_tick = 0;
  if (!p_radar->height_updated && (current_tick - last_log_retry_tick > 2000)) {
    last_log_retry_tick = current_tick;
    Radar_PostInit();
  }

  static int empty_target_count = 0;
  LD2450_Data_t last_ld2450_data;
  if (LD2450_Get_Data(&last_ld2450_data)) {
    /* 将 LD2450 三目标包与 LD6002C 多参数结构体同时送入端侧双通道 AI 跌倒引擎 */
    (void)AI_FallEngine_Step(&last_ld2450_data, p_radar, current_tick);
    const AI_FallState_t *p_ai_frame = AI_FallEngine_GetState();

    int best_idx = -1;
    for (int i = 0; i < 3; i++) {
      if (last_ld2450_data.target[i].is_valid) {
        best_idx = i;
        break;
      }
    }

    if (best_idx != -1) {
      empty_target_count = 0;
      /* 融合 1024 粒子滤波后验质心与瞬时观测，兼顾多径虚影抑制与低延迟响应 */
      int raw_x = last_ld2450_data.target[best_idx].x;
      int raw_y = last_ld2450_data.target[best_idx].y;
      int cur_x = (!is_first_frame && p_ai_frame->filtered_r_m > 0.15f)
                      ? ((raw_x * 6 + (int)p_ai_frame->filtered_x_mm * 4) / 10)
                      : raw_x;
      int cur_y = (!is_first_frame && p_ai_frame->filtered_r_m > 0.15f)
                      ? ((raw_y * 6 + (int)p_ai_frame->filtered_y_mm * 4) / 10)
                      : raw_y;
      int cur_v = last_ld2450_data.target[best_idx].speed;

      /* 若已过开机预热期，且 PINN-CfC 判定为跌倒流形或出现超高速失衡冲击且非正常慢走/快走姿态，标记异常候选 */
      if (warmup_done &&
          (p_ai_frame->ld2450_ai_fall_triggered ||
           ((cur_v > 210 || cur_v < -210 ||
             (!is_first_frame &&
              (cur_x - history_x > 1200 || cur_x - history_x < -1200))) &&
            p_ai_frame->posture_class != AI_POSTURE_SLOW_WALK &&
            p_ai_frame->posture_class != AI_POSTURE_FAST_TURN))) {
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

        /* 优先采用径向多普勒速度与横向位移速度融合值，并融合 LD6002C 的 3D 速度 (单位: cm/s) */
        uint32_t dt = current_tick - last_valid_target_tick;
        if (dt > 0) {
          int vx_cms = ((cur_x - history_x) * 100) / (int)dt;
          int v60_cms = (int)(p_radar->speed_m_s * 100.0f);
          int best_v = (abs_v > (vx_cms > 0 ? vx_cms : -vx_cms)) ? cur_v : vx_cms;
          if (v60_cms > (best_v > 0 ? best_v : -best_v)) {
            best_v = v60_cms;
          }
          upload_v = best_v;
        }
      } else {
        upload_v = cur_v;
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
    /* 若 LD2450 暂时无动态目标，但 LD6002C 实时 3D 点云仍在跟踪人体，自动用 LD6002C 的参数互补填充上传 */
    if (p_radar->is_human && p_radar->height_updated &&
        (current_tick - p_radar->last_pc_tick < 1500u)) {
      upload_x = (int)(p_radar->pc_avg_x_m * 1000.0f);
      upload_y = (int)((p_radar->pc_avg_z_m > 0.1f ? p_radar->pc_avg_z_m : p_radar->height_radar_m) * 1000.0f);
      upload_v = (int)(p_radar->speed_m_s * 100.0f);
    } else {
      history_x = 0;
      history_y = 0;
      upload_x = 0;
      upload_y = 0;
      upload_v = 0;
      is_first_frame = true;
    }
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
        
        // 报警 9 秒（18次翻转）后自动停止，并强制清除底层雷达跌倒状态与 AI 锁存，立即上报恢复正常状态
        p_radar->is_fall = 0;
        is_falling = 0;
        anomaly_flag = 0;
        AI_FallEngine_ResetAlarm();
        should_upload = 1;
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

  /* 异构双雷达独立多参数 AI 并行决策合成：
   * 通道 A (LD2450)：1024 粒子滤波 + 64点 STFT + PINN-CfC 液态神经网络识别跌倒 -> 111
   * 通道 B (LD6002C)：三维点云多参数引擎（离地高度H下降、垂直坠落速率-dH/dt、三维速度V冲击、
   *                   三维加速度、点云卧姿宽高比、低位静止确认，兼容 0x0E02）识别跌倒 -> 1
   * 并行触发规则：任意一路识别出跌倒立即报警 (111 或 1)；两路同时识别出跌倒立即报警 (11)！ */
  (void)AI_FallEngine_Step(NULL, p_radar, current_tick);
  const AI_FallState_t *p_ai_state = AI_FallEngine_GetState();

  static uint32_t fall_111_start_tick = 0;
  static int fall_111_waiting = 0;
  int fall_val = 0;

  if (!warmup_done) {
    /* 开机/复位后前 5 秒为传感器与粒子滤波热机稳定期，强制清零一切跌倒误触发状态 */
    anomaly_flag = 0;
    fall_111_waiting = 0;
    p_radar->is_fall = 0;
    AI_FallEngine_ResetAlarm();
    fall_val = 0;
  } else {
    fall_val = p_ai_state->combined_fall_code;
    if (alarm.active && fall_val == 0) {
      fall_val = 1;
    } else if (alarm.active && fall_val == 111) {
      fall_val = 11;
    }

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
        /* 1.8 秒触地静止确认：若此时 PINN-CfC 识别到人在持续正常慢走(Class 0)，则拦截误报 */
        if (p_ai_state->posture_class == AI_POSTURE_SLOW_WALK &&
            (p_ai_state->filtered_v_ms > 0.35f || p_ai_state->filtered_v_ms < -0.35f)) {
          anomaly_flag = 0;
          fall_111_waiting = 0;
          fall_val = 0;
        } else {
          fall_111_waiting = 2;
        }
      } else {
        fall_val = 0;
      }
    }

    if (fall_111_waiting == 2 && fall_val == 0) {
      fall_val = 111;
    }
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

    // 第 4 行：若未连上4G，优先显示故障诊断信息；就绪后显示 LD6002C 离地高度与移动速度
    if (g_4g_status != ML307R_STAT_NET_OK) {
      sprintf(oled_buf, "%-16s", g_4g_diag_info);
    } else {
      sprintf(oled_buf, "H:%4.2fm V:%4.2fm", p_radar->height_floor_m,
              p_radar->speed_m_s);
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

  Radar_Init();         // 初始化雷达串口
  FallAlarm_Init();     // 初始化跌倒报警状态机
  LD2450_Init();        // 初始化 LD2450 雷达
  AI_FallEngine_Init(); // 初始化端侧 PINN-CfC 液态神经网络与 1024 粒子滤波引擎
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

  /* 进入正式监控前再次清空开机初始化期间积累的瞬态标志，并开启 5 秒热机静默保护 */
  anomaly_flag = 0;
  is_falling = 0;
  is_first_frame = true;
  AI_FallEngine_ResetAlarm();
  FallAlarm_Init();
  g_ready_tick = g_system_tick;
  g_system_ready = 1;

  /* === 3. 立即呈现全场景实时监控界面 === */
  OLED_Clear();
  System_Yield(); // 立即执行首轮传感器采集并刷新 OLED 4行监控参数

  /* 指向搭建好的轻量服务器 IP 和端口（上报完整跌倒触发码 1 / 11 / 111） */
  const char *http_url = "182.254.135.92:8080/api/data";
  char json_buf[224];

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
    } else if (should_upload && (current_tick - last_upload_tick >= 800)) {
      need_upload = true;
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
        int is_human = snap->is_human ? 1 : 0;
        int is_fall = (is_falling != 0) ? 1 : 0;
        const char *pose_str = is_fall ? "falling" : (is_human ? "standing" : "away");
        int dist_cm = (int)upload_y;
        if (dist_cm < 0) dist_cm = 0;

        sprintf(json_buf,
                "{\"device_id\":\"MM001\","
                "\"temperature\":%.1f,"
                "\"humidity\":%d,"
                "\"distance\":%.2f,"
                "\"motion_speed\":%d,"
                "\"person\":%d,"
                "\"pose\":\"%s\","
                "\"fall\":%d,"
                "\"fall_confidence\":%d}",
                current_temp, is_falling, (float)dist_cm * 0.01f, (int)upload_v,
                is_human, pose_str, is_fall, is_falling);

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

    // 断网或未就绪状态下 (g_4g_ok == 0)，后台每隔 4 秒轻量化自动探测入网，确保主循环雷达流畅
    if (!g_4g_ok && (current_tick - last_net_retry_tick >= 4000)) {
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
