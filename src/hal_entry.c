/* hal_entry.c */
#include "ai_fall_engine.h" // 引入端侧 PINN-CfC 液态神经网络与 1024 粒子滤波跌倒引擎
#include "ai_weights.h"     // 引入模型权重与就绪标识 AI_MODEL_READY
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
static uint32_t fall_111_start_tick = 0;
static int fall_111_waiting = 0;
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
static int s_locked_target_idx = -1;
static int s_locked_last_x = 0;
static int s_locked_last_y = 0;
static int g_4g_ok = 0;
static int g_system_ready = 0;
static uint32_t g_ready_tick = 0;
volatile uint32_t g_last_ld2450_uart_tick = 0; // LD2450 串口通信保活心跳

static void oled_show_row_padded(uint8_t row, const char *text) {
  char buf[17];
  memset(buf, ' ', 16);
  buf[16] = '\0';
  size_t len = strlen(text);
  if (len > 16) len = 16;
  memcpy(buf, text, len);
  OLED_ShowString(row, 1, buf);
}

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
  static LD2450_Data_t s_last_ld2450_data = {0};
  static bool s_has_ld2450_data = false;
  if (LD2450_Get_Data(&s_last_ld2450_data)) {
    s_has_ld2450_data = true;
    /* 将 LD2450 三目标包与 LD6002C 多参数结构体同时送入端侧双通道 AI 跌倒引擎 */
    (void)AI_FallEngine_Step(&s_last_ld2450_data, p_radar, current_tick);
    const AI_FallState_t *p_ai_frame = AI_FallEngine_GetState();

    /* 3D 空间 ROI 围栏与多目标智能锁定跟踪 (Target Lock):
     * 1. 空间 ROI 围栏: Y(350~4200mm), X(-2200~+2200mm)，超出范围的目标一律视为旁人/门外干扰丢弃！
     * 2. 连续性锁定: 优先跟踪已锁定的主目标 ID，防止旁边有人晃动或走动时抢占目标焦点！
     * 3. 最近邻绑定: 若上一帧目标丢失，在 ROI 内挑选距垫子中心/雷达最近的目标锁定！ */
    int best_idx = -1;

    /* 优先检验之前锁定的目标是否仍在 ROI 围栏内且位移连续 (800mm 门限防止跳移至旁人) */
    if (s_locked_target_idx >= 0 && s_locked_target_idx < 3) {
      if (s_last_ld2450_data.target[s_locked_target_idx].is_valid) {
        int tx = s_last_ld2450_data.target[s_locked_target_idx].x;
        int ty = s_last_ld2450_data.target[s_locked_target_idx].y;
        if (ty >= 850 && ty <= 4200 && tx >= -2200 && tx <= 2200) {
          int dx = tx - s_locked_last_x;
          int dy = ty - s_locked_last_y;
          if (is_first_frame || (dx * dx + dy * dy < 800 * 800)) {
            best_idx = s_locked_target_idx;
          }
        }
      }
    }

    /* 若锁定目标暂缺或丢失，在 ROI 候选目标中选取最佳目标 (优先距核心监测区最近) */
    if (best_idx == -1) {
      int min_dist_sq = 0x7FFFFFFF;
      for (int i = 0; i < 3; i++) {
        if (s_last_ld2450_data.target[i].is_valid) {
          int tx = s_last_ld2450_data.target[i].x;
          int ty = s_last_ld2450_data.target[i].y;
          if (ty >= 850 && ty <= 4200 && tx >= -2200 && tx <= 2200) {
            int d_core_sq = tx * tx + (ty - 1800) * (ty - 1800);
            if (d_core_sq < min_dist_sq) {
              min_dist_sq = d_core_sq;
              best_idx = i;
            }
          }
        }
      }
    }

    if (best_idx != -1) {
      /* 融合 1024 粒子滤波后验质心与瞬时观测，兼顾多径虚影抑制与低延迟响应 */
      int raw_x = s_last_ld2450_data.target[best_idx].x;
      int raw_y = s_last_ld2450_data.target[best_idx].y;
      int cur_x = (!is_first_frame && p_ai_frame->filtered_r_m > 0.15f)
                      ? ((raw_x * 6 + (int)p_ai_frame->filtered_x_mm * 4) / 10)
                      : raw_x;
      int cur_y = (!is_first_frame && p_ai_frame->filtered_r_m > 0.15f)
                      ? ((raw_y * 6 + (int)p_ai_frame->filtered_y_mm * 4) / 10)
                      : raw_y;

      /* 零速死区：消除静止时天线热噪声，速度绝对值 >= 12cm/s 才放通 */
      int raw_v = s_last_ld2450_data.target[best_idx].speed;
      int abs_raw_v = (raw_v > 0) ? raw_v : -raw_v;
      int cur_v = (abs_raw_v >= 12) ? raw_v : 0;

      /* 【关键抗环境杂波：静态虚影超时释放】
       * 隔着浴帘测数据时，微动浴帘或花洒可能反弹固定虚像 (速度=0, 坐标长期不动)。
       * 若绝对速度连续 2.5 秒 < 10cm/s 且 LD6002C 无运动点云，判定为静态杂波，释放锁定！ */
      static uint32_t s_static_target_tick = 0;
      int dx = cur_x - history_x; if (dx < 0) dx = -dx;
      int dy = cur_y - history_y; if (dy < 0) dy = -dy;
      if (abs_raw_v < 10 && dx < 40 && dy < 40 && (!p_radar->height_updated || p_radar->speed_m_s < 0.10f)) {
        if (s_static_target_tick == 0) s_static_target_tick = current_tick;
        else if (current_tick - s_static_target_tick > 2500u) {
          best_idx = -1; // 超时判定为环境静止物体，释放锁定！
          s_locked_target_idx = -1;
        }
      } else {
        s_static_target_tick = 0;
      }

      if (best_idx != -1) {
        s_locked_target_idx = best_idx;
        s_locked_last_x = s_last_ld2450_data.target[best_idx].x;
        s_locked_last_y = s_last_ld2450_data.target[best_idx].y;
        empty_target_count = 0;

        /* 纯 AI 异常冲击感知：完全依端侧 AI 神经网络输出为准 */
        if (warmup_done && (p_ai_frame->ld2450_ai_fall_triggered || p_ai_frame->ld6002c_fall_triggered)) {
          anomaly_flag = 1;
        }

        if (!is_first_frame) {
          int abs_dx = (cur_x - history_x > 0) ? (cur_x - history_x) : (history_x - cur_x);
          int abs_dy = (cur_y - history_y > 0) ? (cur_y - history_y) : (history_y - cur_y);
          int abs_v = (cur_v > 0) ? cur_v : -cur_v;

          if (abs_dx > 35 || abs_dy > 35 || abs_v > 15) {
            should_upload = 1;
            history_x = cur_x;
            history_y = cur_y;
            upload_x = cur_x;
            upload_y = cur_y;
          } else {
            // 人在原地静止微动时，死区防抖，锁定显示坐标，彻底不晃
            upload_x = history_x;
            upload_y = history_y;
          }

          int v60_cms = (int)(p_radar->speed_m_s * 100.0f);
          if (v60_cms < 10) v60_cms = 0;
          upload_v = (abs_v >= v60_cms) ? cur_v : v60_cms;
        } else {
          upload_v = cur_v;
          history_x = cur_x;
          history_y = cur_y;
          upload_x = cur_x;
          upload_y = cur_y;
        }

        is_first_frame = false;
        last_valid_target_tick = current_tick;
      }
    }

    if (best_idx == -1) {
      /* 收到帧但无任何有效 ROI 围栏内目标 (旁边站立人员或门外路人已被硬过滤) */
      empty_target_count++;
      if (empty_target_count >= 10) { // 连续 10 帧(约1.0s)判定 LD2450 核心区无目标
        s_locked_target_idx = -1;
        history_x = 0;
        history_y = 0;
        upload_x = 0;
        upload_y = 0;
        upload_v = 0;
        is_first_frame = true;
      }
    }
  }

  /* 双雷达有人/无人严格融合决策：
   * 1) LD2450 在 1500ms 内在淋浴区 (Y >= 850mm) 有真实有效目标；
   * 2) 或 LD6002C 在 1500ms 内检测到人体存在且有效点云簇 >= 3 点 (彻底滤除浴帘飘动微噪点)。
   * 若双方均超过 1500ms 无真实有效人体，坚决判定为【无人】！ */
  bool ld2450_active = (current_tick - last_valid_target_tick <= 1500u);
  bool ld6002c_active = (p_radar->is_human != 0u && (current_tick - p_radar->last_pc_tick <= 1500u) && (p_radar->pc_point_cnt >= 3u));

  if (ld2450_active || ld6002c_active) {
    p_radar->is_human = 1;
    /* 若 LD2450 暂时静止无微动，用 LD6002C 质心坐标补充上传位置 */
    if (!ld2450_active && ld6002c_active && p_radar->height_updated) {
      upload_x = (int)(p_radar->pc_avg_x_m * 1000.0f);
      upload_y = (int)(p_radar->height_radar_m * 1000.0f);
      upload_v = (int)(p_radar->speed_m_s * 100.0f);
    }
  } else {
    /* 彻底无人：彻底归零，绝不残留假高度、假速度，绝不触发报警！ */
    p_radar->is_human = 0;
    p_radar->height_updated = false;
    p_radar->height_radar_m = 0.0f;
    p_radar->height_floor_m = 0.0f;
    p_radar->speed_m_s = 0.0f;
    p_radar->pc_v_drop_ms = 0.0f;
    p_radar->pc_peak_drop_ms = 0.0f;
    upload_x = 0;
    upload_y = 0;
    upload_v = 0;
    history_x = 0;
    history_y = 0;
    s_locked_target_idx = -1;
    s_has_ld2450_data = false;
    anomaly_flag = 0;
    fall_111_waiting = 0;
    p_radar->is_fall = 0;
    is_first_frame = true;
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

  /* 获取端侧神经网络推理状态：采集前权重处于基线待命中，采集训练后真实输出 1, 11, 111 */
  const AI_FallState_t *p_ai_state = AI_FallEngine_GetState();
  int fall_val = 0;

#if defined(AI_MODEL_READY) && (AI_MODEL_READY == 1u)
  if (!warmup_done || !p_radar->is_human) {
    fall_val = 0;
    is_falling = 0;
  } else {
    // 真实数据采集并训练后，由端侧 AI 神经网络决策跌倒触发码
    fall_val = p_ai_state->combined_fall_code;
    is_falling = fall_val;
  }
#else
  /* 采集前未训练阶段：彻底关闭报警，保持无跌倒状态 */
  fall_val = 0;
  is_falling = 0;
#endif

  /* 采集训练后：一旦真机判定跌倒，立即声光报警并在 10 秒后自动复位 */
  if ((is_falling == 1 || is_falling == 11 || is_falling == 111) && !alarm_playing) {
    alarm_playing = 1;
    buzzer_toggle_count = 0;
    last_buzzer_tick = current_tick;
    MY1680_SetVol(30);
    MY1680_PlayMusic_SubDir(55, 1); // 播放语音求助警报
  }

  if (alarm_playing) {
    if (current_tick - last_buzzer_tick >= 500) {
      last_buzzer_tick = current_tick;
      if (buzzer_toggle_count < 20) {
        Buzzer_Toggle();
        buzzer_toggle_count++;
      } else {
        Buzzer_Off();
        buzzer_toggle_count = 0;
        alarm_playing = 0;
        AI_FallEngine_ResetAlarm();
      }
    }
  } else {
    Buzzer_Off();
    buzzer_toggle_count = 0;
  }

  /* LED 灯带：报警时警示闪烁，夜间智能夜灯，日常常亮白灯 */
  if (alarm_playing) {
    LED_Off();
    LED210_Off();
    if ((buzzer_toggle_count % 2) != 0) {
      LED202_On();
      LED604_On();
    } else {
      LED202_Off();
      LED604_Off();
    }
  } else if (is_night_light) {
    LED_Off();
    LED202_Off();
    LED604_On();
    LED210_On();
  } else {
    /* 平常状态下：LED灯带常亮白灯 */
    LED_White_On();
    LED210_Off();
  }

  /* 始终刷新 OLED 显示（采集前状态明确显示为待等） */
  static uint32_t last_oled_tick = 0;
  if (current_tick - last_oled_tick >= 400) {
    last_oled_tick = current_tick;
    char oled_buf[24];

    float current_temp = NTC_GetTemperature();
    int temp_i = (int)current_temp;
    int temp_d = (int)((current_temp - (float)temp_i) * 10.0f);
    if (temp_d < 0) temp_d = -temp_d;

    sprintf(oled_buf, "4G:%-6sT:%2d.%1dC", g_4g_ok ? "OK" : g_4g_status_tag, temp_i, temp_d);
    oled_show_row_padded(1, oled_buf);

    int h_i = (int)p_radar->height_floor_m;
    int h_d = (int)((p_radar->height_floor_m - (float)h_i) * 100.0f);
    if (h_d < 0) h_d = -h_d;
    if (is_falling != 0) {
      sprintf(oled_buf, "HUM:Y FAL:!%d!", is_falling);
    } else {
      sprintf(oled_buf, "H:%d.%02dm F:WAIT", h_i, h_d);
    }
    oled_show_row_padded(2, oled_buf);

    sprintf(oled_buf, "X:%-5d Y:%-5d", upload_x, upload_y);
    oled_show_row_padded(3, oled_buf);

    // 第 4 行：若未连上4G，优先显示故障诊断信息；就绪后显示稳定距离与移动速度
    if (g_4g_status != ML307R_STAT_NET_OK && !g_4g_ok) {
      oled_show_row_padded(4, g_4g_diag_info);
    } else {
      int dist_mm = (upload_y >= 0) ? upload_y : -upload_y;
      int dis_i = dist_mm / 1000;
      int dis_d = (dist_mm % 1000) / 10;
      int v_i = (int)p_radar->speed_m_s;
      int v_d = (int)((p_radar->speed_m_s - (float)v_i) * 100.0f);
      if (v_d < 0) v_d = -v_d;
      sprintf(oled_buf, "Dist:%d.%02dm V:%d.%02d", dis_i, dis_d, v_i, v_d);
      oled_show_row_padded(4, oled_buf);
    }
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
  char json_buf[320];

  while (1) {
    System_Yield(); // 每帧后台维护雷达和蜂鸣器

    static uint32_t last_upload_tick = 0;
    static uint32_t last_net_retry_tick = 0;
    static int http_fail_count = 0;       // 连续失败计数器
    // 姿态/跌倒/在场状态突变检测器
    static int s_last_reported_fall = 0;
    static int s_last_reported_hum = 0;
    static int s_last_reported_human = 0;
    static float s_last_reported_dist_m = 0.0f;
    static float s_last_reported_h_m = 0.0f;

    uint32_t current_tick = g_system_tick;

    int current_is_fall = (is_falling != 0) ? 1 : 0;
    radar_data_t *p_cur_radar = Radar_GetData();
    int current_is_human = (p_cur_radar && p_cur_radar->is_human) ? 1 : 0;
    float cur_h_m = (p_cur_radar && p_cur_radar->height_updated) ? p_cur_radar->height_floor_m : 0.0f;
    int current_hum = is_falling;
    if (!current_is_fall && current_is_human) {
      /* 纯 AI 姿态输出：直接取端侧 PINN-CfC 神经网络分类输出 */
      AI_PostureClass_t ai_cls = AI_FallEngine_GetState()->posture_class;
      current_hum = (ai_cls == AI_POSTURE_BEND_SQUAT) ? 23 : 21;
    }

    /* 状态突变判定：跌倒触发/解除、有人/无人切换、姿态在站立与弯腰下蹲间切换 */
    bool state_changed = (current_is_fall != s_last_reported_fall) ||
                         (current_is_human != s_last_reported_human) ||
                         (current_hum != s_last_reported_hum);

    float cur_dist_m = (float)abs((int)upload_y) / 1000.0f;

    /* 动态位移与高度变动检测（有人在场走动、站立/下蹲过渡时快速插队） */
    float diff_dist = cur_dist_m - s_last_reported_dist_m;
    if (diff_dist < 0.0f) diff_dist = -diff_dist;
    float diff_h = cur_h_m - s_last_reported_h_m;
    if (diff_h < 0.0f) diff_h = -diff_h;
    bool motion_changed = current_is_human && (diff_dist >= 0.25f || diff_h >= 0.15f);

    // 判断是否需要上传数据（兼顾极速响应与 4G Socket 硬件防阻塞）：
    bool need_upload = false;
    uint32_t elapsed_upload = current_tick - last_upload_tick;

    if (elapsed_upload >= 300u) {
      if (state_changed || should_upload) {
        need_upload = true; // 状态突变或雷达位移显著：300ms 极速插队上报！
      } else if (is_falling != 0 && elapsed_upload >= 500u) {
        need_upload = true; // 跌倒报警持续中：500ms 高频刷新！
      } else if (motion_changed && elapsed_upload >= 500u) {
        need_upload = true; // 人体动态位移：500ms 丝滑跟随！
      } else if (current_is_human && elapsed_upload >= 700u) {
        need_upload = true; // 有人在场平稳心跳：700ms 高速流畅刷新！
      } else if (elapsed_upload >= 2000u) {
        need_upload = true; // 彻底无人时：2.0秒保活心跳
      }
    }

    if (need_upload) {
      should_upload = 0;
      s_last_reported_fall = 0;
      s_last_reported_human = current_is_human;
      s_last_reported_hum = current_hum;
      s_last_reported_dist_m = cur_dist_m;
      s_last_reported_h_m = cur_h_m;

      /* 自动自愈：若当前 4G 标记为断开，每隔 2 秒尝试快速重置 Socket 并探测入网，决不允许永久停发！ */
      if (!g_4g_ok && (current_tick - last_net_retry_tick >= 2000u)) {
        last_net_retry_tick = current_tick;
        ML307R_ResetSocket();
        if (ML307R_CheckNetwork()) {
          g_4g_ok = 1;
          http_fail_count = 0;
        }
      }

      if (g_4g_ok) {
        float current_temp = NTC_GetTemperature();
        radar_data_t *snap = Radar_GetData(); // 只取一次快照，保证同一帧数据一致
        int is_human = snap->is_human ? 1 : 0;
        int dist_mm = (upload_y >= 0) ? upload_y : -upload_y;

        int j_temp_i = (int)current_temp;
        int j_temp_d = (int)((current_temp - (float)j_temp_i) * 10.0f);
        if (j_temp_d < 0) j_temp_d = -j_temp_d;

        int dis_i = dist_mm / 1000;
        int dis_d = (dist_mm % 1000) / 10;

        int h_i = (int)snap->height_floor_m;
        int h_d = (int)((snap->height_floor_m - (float)h_i) * 100.0f);
        if (h_d < 0) h_d = -h_d;

        int v60_i = (int)snap->speed_m_s;
        int v60_d = (int)((snap->speed_m_s - (float)v60_i) * 100.0f);
        if (v60_d < 0) v60_d = -v60_d;

        float vd = snap->pc_v_drop_ms;
        const char *vd_sign = (vd < 0.0f) ? "-" : "";
        float vd_abs = (vd < 0.0f) ? -vd : vd;
        int vd_i = (int)vd_abs;
        int vd_d = (int)((vd_abs - (float)vd_i) * 100.0f);

#if defined(AI_MODEL_READY) && (AI_MODEL_READY == 1u)
        const char *p_pose = is_falling ? "falling" : (is_human ? ((AI_FallEngine_GetState()->posture_class == AI_POSTURE_BEND_SQUAT) ? "bending" : "standing") : "away");
#else
        const char *p_pose = is_falling ? "falling" : (is_human ? "none" : "away");
#endif
        int p_len = sprintf(json_buf,
                "{\"device_id\":\"MM001\",\"temperature\":%d.%d,\"humidity\":%d,"
                "\"distance\":%d.%02d,\"motion_speed\":%d,\"person\":%s,\"pose\":\"%s\",",
                j_temp_i, j_temp_d, is_human ? (is_falling ? is_falling : 0) : 0, dis_i, dis_d, (int)upload_v,
                is_human ? "true" : "false", p_pose);
        sprintf(json_buf + p_len,
                "\"fall\":%s,\"fall_confidence\":%d,\"h_m\":%d.%02d,"
                "\"v60_ms\":%d.%02d,\"v_drop_ms\":%s%d.%02d}",
                is_falling ? "true" : "false", is_falling,
                h_i, h_d, v60_i, v60_d, vd_sign, vd_i, vd_d);

        if (ML307R_HttpPost(http_url, json_buf)) {
          g_4g_ok = 1;
          http_fail_count = 0; // 成功一次，立刻清零
        } else {
          http_fail_count++;
          if (http_fail_count >= 2) {
            ML307R_ResetSocket();
          }
          /* 连续失败 4 次才临时标记为未就绪，并触发自愈重试 */
          if (http_fail_count >= 4) {
            g_4g_ok = 0;
            last_net_retry_tick = current_tick;
          }
        }
        last_upload_tick = g_system_tick; // 完成上传后打卡，精准控制下一次间隔
      } else {
        last_upload_tick = current_tick;
      }
    }

    // 后台轻量化自动探测入网，确保主循环雷达流畅
    if (!g_4g_ok && (current_tick - last_net_retry_tick >= 2500)) {
      last_net_retry_tick = current_tick;
      ML307R_ResetSocket();
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
