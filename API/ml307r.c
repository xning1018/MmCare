/* ml307r.c */
#include "ml307r.h"
#include "oled.h"
#include <string.h>
#include <stdio.h>
#include "led.h"
#define RX_BUF_SIZE 1024
volatile char g_rx_buf[RX_BUF_SIZE];
volatile uint16_t g_rx_idx = 0;
volatile bool g_uart9_tx_complete = false;

int16_t g_http_stage = 0;
char g_http_last_resp[32] = {0};

extern void System_Yield(void);

/* 线程安全中断回调（保持原样不变） */
void user_uart9_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR)
    {
        if (g_rx_idx < (RX_BUF_SIZE - 2))
        {
            g_rx_buf[g_rx_idx] = (char)p_args->data;
            g_rx_buf[g_rx_idx + 1] = '\0'; 
            g_rx_idx++;
        }
    }
    else if (p_args->event == UART_EVENT_TX_COMPLETE)
    {
        g_uart9_tx_complete = true;
    }
}

void ML307R_Init(void)
{
    R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
}

bool ML307R_SendCmd(const char *cmd, const char *expected_reply, uint32_t timeout_ms)
{
    g_rx_idx = 0;
    g_rx_buf[0] = '\0'; 
    g_uart9_tx_complete = false;

    if (strlen(cmd) > 0)
    {
        if (R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)cmd, strlen(cmd)) != FSP_SUCCESS) return false;
        
        uint32_t tx_timeout = 0;
        while (!g_uart9_tx_complete && tx_timeout < 500)
        {
            R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
            System_Yield();
            tx_timeout++;
        }
    }

    uint32_t timer = 0;
    while (timer < timeout_ms)
    {
        if (strstr((const char *)g_rx_buf, expected_reply) != NULL) return true;
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
        System_Yield();
        timer += 10;
    }
    return false;
}

bool ML307R_InitNetwork(void)
{
    if (ML307R_SendCmd("AT+MIPCALL?\r\n", "+MIPCALL: 1", 2000)) return true;
    if (ML307R_SendCmd("AT+MIPCALL=1\r\n", "OK", 4000))
    {
        R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS);
        if (ML307R_SendCmd("AT+MIPCALL?\r\n", "+MIPCALL: 1", 2000)) return true;
    }
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
bool ML307R_HttpPost(const char *url, const char *json_payload)
{
    /* 【关键修复】腾讯云域名极长，host 数组必须扩容到 128 字节，否则必定爆内存死机 */
    static char cmd_buf[256]; 
    static char http_buf[512]; // 稍微放大，防止超长域名拼接时溢出
    
    char host[128] = {0};      // 彻底解决内存溢出引发的 STAGE 0 错误
    char path[64] = {0};
    uint16_t port = 80; 

    memset(g_http_last_resp, 0, sizeof(g_http_last_resp));

    // 【解析 URL】提取 Host, Port 和 Path
    const char *p_port = strchr(url, ':');
    const char *p_path = strchr(url, '/');
    
    if (p_port && p_path && (p_port < p_path))
    {
        size_t host_len = (size_t)(p_port - url);
        if (host_len >= sizeof(host)) host_len = sizeof(host) - 1;
        strncpy(host, url, host_len);
        host[host_len] = '\0'; 
        
        port = (uint16_t)atoi(p_port + 1);
        strcpy(path, p_path);
    }
    else if (p_path)
    {
        size_t host_len = (size_t)(p_path - url);
        if (host_len >= sizeof(host)) host_len = sizeof(host) - 1;
        strncpy(host, url, host_len);
        host[host_len] = '\0';
        strcpy(path, p_path);
    }
    else
    {
        strcpy(g_http_last_resp, "URL PARSE ERR");
        return false;
    }

    // 【STAGE 1】彻底闭合 0 号 Socket 连接（防止底层状态挂起导致 55 错误）
    g_http_stage = 1;
    OLED_ShowString(3, 1, "STAGE: 1 (CLOSE)");
    
    // 连续发送两次断开，第一步异步清理，第二步强制复位
    ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 1000); // 增加超时到 1 秒
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS); // 人为挂起 100ms 缓冲时间

    // 【STAGE 2】发起标准 TCP 连接
    g_http_stage = 2;
    OLED_ShowString(3, 1, "STAGE: 2 (CONN) ");
    
    sprintf(cmd_buf, "AT+MIPOPEN=0,\"TCP\",\"%s\",%d\r\n", host, port);
    
    // 增大建立连接的等待超时时间到 6000ms（6秒），给基站充足的咬合时间
    if (!ML307R_SendCmd(cmd_buf, "+MIPOPEN: 0,0", 6000)) 
    {
        // === 【新增防错重试机制】 ===
        // 如果依然不幸返回了 55 报错，说明通道死锁，立刻在函数内部强行闭合并二次重试连接！
        if (strstr((char *)g_rx_buf, "55") != NULL)
        {
            OLED_ShowString(4, 1, "RETRY CONN...  ");
            ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 1000);
            R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
            if (ML307R_SendCmd(cmd_buf, "+MIPOPEN: 0,0", 5000))
            {
                goto __conn_ok; // 重试成功，直接跳跃到后续流程
            }
        }

        char *err_p = strstr((char *)g_rx_buf, "+MIPOPEN:");
        if(err_p) strncpy(g_http_last_resp, err_p, 15);
        else strcpy(g_http_last_resp, "CONN TIMEOUT");
        return false;
    }

__conn_ok:
    // 【STAGE 3】纯手工按标准 HTTP 规范拼接文本报文
    g_http_stage = 3;
    OLED_ShowString(3, 1, "STAGE: 3 (PACK) ");
    uint16_t body_len = (uint16_t)strlen(json_payload);
    
    int http_len = sprintf(http_buf,
        "POST %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Accept: application/json\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n" // 告诉服务器发完数据就断开，防止它挂起通道
        "\r\n"
        "%s",
        path, host, port, body_len, json_payload);

    // 【STAGE 4】申请定长发送
    g_http_stage = 4;
    OLED_ShowString(3, 1, "STAGE: 4 (SEND) ");
    sprintf(cmd_buf, "AT+MIPSEND=0,%d\r\n", http_len);
    
    g_rx_idx = 0; g_rx_buf[0] = '\0'; g_uart9_tx_complete = false;
    if (R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)cmd_buf, strlen(cmd_buf)) != FSP_SUCCESS) return false;
    uint32_t tx_timeout = 0;
    while (!g_uart9_tx_complete && tx_timeout < 500) { R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS); System_Yield(); tx_timeout++; }

    uint32_t t = 0;
    bool ready = false;
    while (t < 2000)
    {
        if (strstr((const char *)g_rx_buf, ">") != NULL) { ready = true; break; }
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
        System_Yield();
        t += 10;
    }
    if (!ready)
    {
        strcpy(g_http_last_resp, "NO > PROMPT");
        goto __exit;
    }

    // 【STAGE 5】往 '>' 后面灌入完整的 HTTP 文本数据
    g_http_stage = 5;
    OLED_ShowString(3, 1, "STAGE: 5 (DATA) ");
    g_rx_idx = 0; g_rx_buf[0] = '\0'; g_uart9_tx_complete = false;
    if (R_SCI_UART_Write(&g_uart9_ctrl, (uint8_t *)http_buf, (uint32_t)http_len) != FSP_SUCCESS) return false;
    tx_timeout = 0;
    while (!g_uart9_tx_complete && tx_timeout < 500) { R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS); System_Yield(); tx_timeout++; }
    
    if (!ML307R_SendCmd("", "OK", 3000))
    {
        strcpy(g_http_last_resp, "SEND DATA ERR");
        goto __exit;
    }

    // 【STAGE 6】死等服务器返回 HTTP 状态码 200 OK
    g_http_stage = 6;
    OLED_ShowString(3, 1, "STAGE: 6 (WAIT) ");
    t = 0;
    while (t < 6000)
    {
        // 某些 HTTPS 关口返回的可能是 201 Created 或 200
        if (strstr((const char *)g_rx_buf, "200") != NULL || strstr((const char *)g_rx_buf, "201") != NULL)
        {
            g_http_stage = 7;
            strcpy(g_http_last_resp, "200 OK");
            
            // ---- 【反向指令解析核心区】 ----
            // 当数据上传成功后，立刻在这里检查腾讯云有没有下发控制 LED 的 JSON 串
            if (strstr((const char *)g_rx_buf, "LED_ON") != NULL)
            {
                LED_On(); // 开启你 led.c 里的 405 灯
            }
            else if (strstr((const char *)g_rx_buf, "LED_OFF") != NULL)
            {
                LED_Off();
            }
            // -------------------------------

            ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 500); 
            return true;
        }
        R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
        System_Yield();
        t += 10;
    }

    char *resp_p = strstr((char *)g_rx_buf, "HTTP/");
    if(resp_p) strncpy(g_http_last_resp, resp_p, 15);
    else strcpy(g_http_last_resp, "NO 200 OK RESP");

__exit:
    ML307R_SendCmd("AT+MIPCLOSE=0\r\n", "OK", 500);
    return false;
}