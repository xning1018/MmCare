/* ml307r.c */
#include "ml307r.h"
#include "led.h"
#include "oled.h"
#include <stdio.h>
#include <string.h>

#define RX_BUF_SIZE 1024
volatile char g_rx_buf[RX_BUF_SIZE];
volatile uint16_t g_rx_idx = 0;
volatile bool g_uart9_tx_complete = false;
volatile uint32_t g_uart9_rx_total_count = 0;

volatile ml307r_status_t g_4g_status = ML307R_STAT_INIT;
char g_4g_diag_info[20] = "WAIT...";
char g_4g_status_tag[8] = "WAIT";

int16_t g_http_stage = 0;
char g_http_last_resp[32] = {0};

extern void System_Yield(void);
extern volatile uint32_t g_system_tick;

/* 线程安全中断回调 */
void user_uart9_callback(uart_callback_args_t *p_args) {
  if (p_args->event & UART_EVENT_RX_CHAR) {
    g_uart9_rx_total_count++;
    if (g_rx_idx < (RX_BUF_SIZE - 2)) {
      g_rx_buf[g_rx_idx] = (char)p_args->data;
      g_rx_buf[g_rx_idx + 1] = '\0';
      g_rx_idx++;
    }
  } else if (p_args->event & UART_EVENT_TX_COMPLETE) {
    g_uart9_tx_complete = true;
  }
}

void ML307R_Init(void) { R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg); }

bool ML307R_SendCmd(const char *cmd, const char *expected_reply,
                    uint32_t timeout_ms) {
  g_rx_idx = 0;
  g_rx_buf[0] = '\0';
  g_uart9_tx_complete = false;

  if (strlen(cmd) > 0) {
    if (R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)cmd, strlen(cmd)) !=
        FSP_SUCCESS)
      return false;

    uint32_t tx_timeout = 0;
    while (!g_uart9_tx_complete && tx_timeout < 500) {
      R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
      tx_timeout++;
    }
  }

  uint32_t elapsed = 0;
  while (elapsed < timeout_ms) {
    if (strstr((const char *)g_rx_buf, expected_reply) != NULL) {
      R_BSP_SoftwareDelay(20, BSP_DELAY_UNITS_MILLISECONDS);
      return true;
    }
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    System_Yield();
    elapsed += 10;
  }
  return false;
}

bool ML307R_InitNetwork(void) {
  return ML307R_CheckNetwork();
}

bool ML307R_CheckNetwork(void) {
  // 1. 发送测试指令 AT (尝试2次，每次400ms)
  bool at_success = false;
  for (int retry = 0; retry < 2; retry++) {
    if (ML307R_SendCmd("AT\r\n", "OK", 400)) {
      at_success = true;
      break;
    }
  }

  // 如果完全没收到任何字节
  if (!at_success && g_uart9_rx_total_count == 0) {
    g_4g_status = ML307R_STAT_NO_RX;
    strcpy(g_4g_status_tag, "NO_RX");
    strcpy(g_4g_diag_info, "Check TX/RX Pin");
    return false;
  }

  // 如果有字节但没 OK（说明波特率或模组正启动）
  if (!at_success) {
    g_4g_status = ML307R_STAT_NO_AT;
    strcpy(g_4g_status_tag, "NO_AT");
    strcpy(g_4g_diag_info, "AT Resp Error");
    return false;
  }

  // 关闭回显，避免污染接收缓冲
  ML307R_SendCmd("ATE0\r\n", "OK", 300);

  // 2. 检查 SIM 卡就绪
  if (!ML307R_SendCmd("AT+CPIN?\r\n", "READY", 600)) {
    g_4g_status = ML307R_STAT_NO_SIM;
    strcpy(g_4g_status_tag, "NO_SIM");
    strcpy(g_4g_diag_info, "Check SIM Card");
    return false;
  }

  // 3. 检查基站注册状态 (CEREG)
  bool cereg_ok = (ML307R_SendCmd("AT+CEREG?\r\n", ",1", 600) ||
                   ML307R_SendCmd("AT+CEREG?\r\n", ",5", 600));
  if (!cereg_ok) {
    g_4g_status = ML307R_STAT_SEARCH;
    strcpy(g_4g_status_tag, "SEAR");
    strcpy(g_4g_diag_info, "Searching 4G...");
    return false;
  }

  // 4. 检查与激活 PDP 网络
  if (ML307R_SendCmd("AT+MIPCALL?\r\n", "+MIPCALL: 1", 1000)) {
    g_4g_status = ML307R_STAT_NET_OK;
    strcpy(g_4g_status_tag, "OK");
    strcpy(g_4g_diag_info, "4G Net Ready");
    return true;
  }

  // 未激活，尝试激活
  ML307R_SendCmd("AT+MIPCALL=1\r\n", "OK", 3000);
  R_BSP_SoftwareDelay(300, BSP_DELAY_UNITS_MILLISECONDS);

  if (ML307R_SendCmd("AT+MIPCALL?\r\n", "+MIPCALL: 1", 1500)) {
    g_4g_status = ML307R_STAT_NET_OK;
    strcpy(g_4g_status_tag, "OK");
    strcpy(g_4g_diag_info, "4G Net Ready");
    return true;
  }

  g_4g_status = ML307R_STAT_ACT_ERR;
  strcpy(g_4g_status_tag, "ACT_E");
  strcpy(g_4g_diag_info, "PDP Act Failed");
  return false;
}

/**
 * @brief  基于纯底层 TCP Socket 透传重构的 HTTP POST 函数
 * @param  url: 格式必须为 "IP:端口/路径" 例如 "123.45.67.89:8080/post"
 * @param  json_payload: 需要发送的JSON字符串
 */
/**
 * @brief  基于底层 TCP/SSL 通道重构的 HTTPS POST 函数
 * @param  url: 格式必须为 "域名:端口/路径"
 * @param  json_payload: 需要发送的JSON字符串
 */
bool ML307R_HttpPost(const char *url, const char *json_payload) {
  /* 腾讯云域名极长，host 数组必须扩容到 128 字节，否则必定爆内存死机 */
  static char cmd_buf[256];
  static char http_buf[512]; // 稍微放大，防止超长域名拼接时溢出

  static char host[128];
  static char path[128];
  memset(host, 0, sizeof(host));
  memset(path, 0, sizeof(path));
  uint16_t port = 80;

  memset(g_http_last_resp, 0, sizeof(g_http_last_resp));

  // 【解析 URL】提取 Host, Port 和 Path
  const char *p_port = strchr(url, ':');
  const char *p_path = strchr(url, '/');

  if (p_port && p_path && (p_port < p_path)) {
    size_t host_len = (size_t)(p_port - url);
    if (host_len >= sizeof(host))
      host_len = sizeof(host) - 1;
    strncpy(host, url, host_len);
    host[host_len] = '\0';

    port = (uint16_t)atoi(p_port + 1);
    strcpy(path, p_path);
  } else if (p_path) {
    size_t host_len = (size_t)(p_path - url);
    if (host_len >= sizeof(host))
      host_len = sizeof(host) - 1;
    strncpy(host, url, host_len);
    host[host_len] = '\0';
    strcpy(path, p_path);
  } else {
    strcpy(g_http_last_resp, "URL PARSE ERR");
    return false;
  }

  // 【STAGE 1】彻底闭合 0 号 Socket 连接（防止底层状态挂起导致 55 错误）
  g_http_stage = 1;
  // OLED_ShowString(3, 1, "STAGE: 1 (CLOSE)");

  // 快速闭合连接，超时设为 150ms 避免因未连接而空等 1 秒
  ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 150);
  R_BSP_SoftwareDelay(20, BSP_DELAY_UNITS_MILLISECONDS);

  // 【STAGE 2】发起标准 TCP 连接
  g_http_stage = 2;
  // OLED_ShowString(3, 1, "STAGE: 2 (CONN) ");

  sprintf(cmd_buf, "AT+MIPOPEN=0,\"TCP\",\"%s\",%d\r\n", host, port);

  // 等待建立连接（常规在 1.5~3 秒内完成）
  if (!ML307R_SendCmd(cmd_buf, "+MIPOPEN: 0,0", 5000)) {
    // === 【防错重试机制】 ===
    if (strstr((char *)g_rx_buf, "55") != NULL) {
      ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 300);
      R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);
      if (ML307R_SendCmd(cmd_buf, "+MIPOPEN: 0,0", 4000)) {
        goto __conn_ok;
      }
    }

    char *err_p = strstr((char *)g_rx_buf, "+MIPOPEN:");
    if (err_p)
      strncpy(g_http_last_resp, err_p, 15);
    else
      strcpy(g_http_last_resp, "CONN TIMEOUT");
    return false;
  }

__conn_ok:
  // 【STAGE 3】按标准 HTTP 规范拼接文本报文
  g_http_stage = 3;
  uint16_t body_len = (uint16_t)strlen(json_payload);

  int http_len = sprintf(
      http_buf,
      "POST %s HTTP/1.1\r\n"
      "Host: %s:%d\r\n"
      "X-API-Key: mmcare-demo-key-2026\r\n"
      "Accept: application/json\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: %d\r\n"
      "Connection: close\r\n"
      "\r\n"
      "%s",
      path, host, port, body_len, json_payload);

  // 【STAGE 4】申请定长发送
  g_http_stage = 4;
  sprintf(cmd_buf, "AT+MIPSEND=0,%d\r\n", http_len);

  g_rx_idx = 0;
  g_rx_buf[0] = '\0';
  g_uart9_tx_complete = false;
  if (R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)cmd_buf, strlen(cmd_buf)) !=
      FSP_SUCCESS)
    return false;
  uint32_t tx_timeout = 0;
  while (!g_uart9_tx_complete && tx_timeout < 500) {
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
    tx_timeout++;
  }

  uint32_t t_start = g_system_tick;
  bool ready = false;
  while (1) {
    if (strstr((const char *)g_rx_buf, ">") != NULL) {
      ready = true;
      break;
    }
    if (g_system_tick - t_start >= 1500)
      break;
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    System_Yield();
  }
  if (!ready) {
    strcpy(g_http_last_resp, "NO > PROMPT");
    goto __exit;
  }

  // 【STAGE 5】往 '>' 后面灌入完整的 HTTP 文本数据，并直达监听 200 OK
  g_http_stage = 5;
  g_rx_idx = 0;
  g_rx_buf[0] = '\0';
  g_uart9_tx_complete = false;
  if (R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)http_buf,
                       (uint32_t)http_len) != FSP_SUCCESS)
    return false;
  tx_timeout = 0;
  while (!g_uart9_tx_complete && tx_timeout < 500) {
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
    tx_timeout++;
  }

  // 【STAGE 6】等待服务器返回 HTTP 状态码 200 OK
  g_http_stage = 6;
  t_start = g_system_tick;
  while (1) {
    if (strstr((const char *)g_rx_buf, "200") != NULL ||
        strstr((const char *)g_rx_buf, "201") != NULL) {
      g_http_stage = 7;
      strcpy(g_http_last_resp, "200 OK");

      if (strstr((const char *)g_rx_buf, "LED_ON") != NULL) {
        LED_On();
      } else if (strstr((const char *)g_rx_buf, "LED_OFF") != NULL) {
        LED_Off();
      }

      ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 200);
      g_4g_status = ML307R_STAT_NET_OK;
      strcpy(g_4g_status_tag, "OK");
      strcpy(g_4g_diag_info, "POST 200 OK");
      return true;
    }
    if (g_system_tick - t_start >= 4000)
      break;
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    System_Yield();
  }

  char *resp_p = strstr((char *)g_rx_buf, "HTTP/");
  if (resp_p)
    strncpy(g_http_last_resp, resp_p, 15);
  else
    strcpy(g_http_last_resp, "NO 200 OK RESP");

__exit:
  ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 500);
  g_4g_status = ML307R_STAT_POST_ERR;
  strcpy(g_4g_status_tag, "FAIL");
  strncpy(g_4g_diag_info, g_http_last_resp, 16);
  g_4g_diag_info[16] = '\0';
  return false;
}