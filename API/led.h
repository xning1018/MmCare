#ifndef LED_H_
#define LED_H_

#include "hal_data.h"

/* ??? P405?P202?P604 ?? */
#define LED_PIN_405 BSP_IO_PORT_04_PIN_05
#define LED_PIN_202 BSP_IO_PORT_02_PIN_02
#define LED_PIN_604 BSP_IO_PORT_06_PIN_04

/* 405 LED ?? */
void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

/* 202 LED ?? */
void LED202_On(void);
void LED202_Off(void);
void LED202_Toggle(void);

/* 604 LED ?? */
void LED604_On(void);
void LED604_Off(void);
void LED604_Toggle(void);

#endif /* LED_H_ */
