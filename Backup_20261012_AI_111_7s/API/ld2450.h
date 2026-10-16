#ifndef LD2450_H_
#define LD2450_H_
#include "hal_data.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int16_t x;
  int16_t y;
  int16_t speed;
  uint16_t res;
  bool is_valid;
} LD2450_Target_t;

typedef struct {
  LD2450_Target_t target[3];
} LD2450_Data_t;

void LD2450_Init(void);
void LD2450_ResetState(void);
void LD2450_UART_Callback(uint8_t rx_data);
bool LD2450_Get_Data(LD2450_Data_t *out_data);

#endif /* LD2450_H_ */
