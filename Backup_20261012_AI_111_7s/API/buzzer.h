#ifndef BUZZER_H_
#define BUZZER_H_

#include "hal_data.h"

/* 宏定义 P415 引脚 */
#define BUZZER_PIN    BSP_IO_PORT_04_PIN_15

/* 蜂鸣器驱动功能函数声明 */
void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);

#endif /* BUZZER_H_ */