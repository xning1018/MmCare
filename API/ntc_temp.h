
/* ================================================================
 * ntc_temp.h
 * NTC 热敏电阻温度采集模块
 * ================================================================ */
#ifndef NTC_TEMP_H
#define NTC_TEMP_H

#include "hal_data.h"

/* 模块客观物理参数定义 */
#define NTC_B_VALUE     3950.0f   // B值
#define NTC_T0          298.15f   // 25摄氏度对应的开尔文温度
#define NTC_R0          10000.0f  // 25摄氏度时的标称阻值
#define FIXED_RESISTOR  10000.0f  // 模块串联分压电阻

/* 函数声明 */
void NTC_Init(void);
float NTC_GetTemperature(void);

#endif /* NTC_TEMP_H */

