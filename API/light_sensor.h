#ifndef LIGHT_SENSOR_H_
#define LIGHT_SENSOR_H_

#include "hal_data.h"

// 光敏传感器 D0 接口连接的引脚
#define LIGHT_SENSOR_PIN BSP_IO_PORT_00_PIN_10

void LightSensor_Init(void);
uint8_t LightSensor_GetStatus(void);

#endif /* LIGHT_SENSOR_H_ */
