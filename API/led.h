#ifndef LED_H_
#define LED_H_

#include "hal_data.h"

#define LED_PIN_405 BSP_IO_PORT_04_PIN_05
#define LED_PIN_202 BSP_IO_PORT_02_PIN_02
#define LED_PIN_604 BSP_IO_PORT_06_PIN_04
#define LED_PIN_210 BSP_IO_PORT_02_PIN_10

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

void LED202_On(void);
void LED202_Off(void);
void LED202_Toggle(void);

void LED604_On(void);
void LED604_Off(void);
void LED604_Toggle(void);

void LED210_On(void);
void LED210_Off(void);
void LED210_Toggle(void);

void LED_White_On(void);
void LED_White_Off(void);

#endif
