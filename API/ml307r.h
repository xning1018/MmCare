/* ml307r.h */
#ifndef ML307R_H
#define ML307R_H

#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>
extern int g_http_id;
extern int16_t g_http_stage;
extern char g_http_last_resp[32];

void ML307R_Init(void);
bool ML307R_SendCmd(const char *cmd, const char *expected_reply, uint32_t timeout_ms);
bool ML307R_InitNetwork(void);

/* 银尔达官方流 HTTP POST (双参数版) */
bool ML307R_HttpPost(const char *url, const char *json_payload);

#endif /* ML307R_H */

