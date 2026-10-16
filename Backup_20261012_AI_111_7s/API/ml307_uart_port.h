#ifndef __ML307_UART_PORT_H__
#define __ML307_UART_PORT_H__

#include <stdint.h>

void ml307_uart_send(const char *data, uint16_t len);
int  ml307_uart_read_line(char *buf, int maxlen, int timeout_ms);
int  ml307_wait_prompt(char ch, int timeout_ms);

#endif

