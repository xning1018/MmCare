/* ml307r.h */
#ifndef ML307R_H
#define ML307R_H

#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>
extern int g_http_id;
extern int16_t g_http_stage;
extern char g_http_last_resp[32];

typedef enum {
    ML307R_STAT_INIT = 0,    // 开机初始化
    ML307R_STAT_NO_RX,       // 串口完全无接收（硬件接线/引脚接反/供电）
    ML307R_STAT_NO_AT,       // 串口有收到字节但AT无OK应答
    ML307R_STAT_NO_SIM,      // 未检测到有效SIM卡 (CPIN不是READY)
    ML307R_STAT_SEARCH,      // 正在搜网 (CEREG 不是 1 或 5)
    ML307R_STAT_ACT_ERR,     // 基站已注册，但MIPCALL激活失败
    ML307R_STAT_NET_OK,      // 4G网络已就绪，已分配IP
    ML307R_STAT_POSTING,     // 正在向云端POST数据
    ML307R_STAT_POST_ERR,    // 云端POST返回错误或超时
} ml307r_status_t;

extern volatile ml307r_status_t g_4g_status;
extern char g_4g_diag_info[20];
extern char g_4g_status_tag[8];
extern volatile uint32_t g_uart9_rx_total_count;

void ML307R_Init(void);
bool ML307R_SendCmd(const char *cmd, const char *expected_reply,
                    uint32_t timeout_ms);
bool ML307R_InitNetwork(void);
bool ML307R_CheckNetwork(void);

/* 银尔达官方流 HTTP POST  */
bool ML307R_HttpPost(const char *url, const char *json_payload);
void ML307R_ResetSocket(void);

#endif /* ML307R_H */
