#include "light_sensor.h"

void LightSensor_Init(void)
{
    // 配置引脚为输入引脚
    R_IOPORT_PinCfg(g_ioport.p_ctrl, LIGHT_SENSOR_PIN, IOPORT_CFG_PORT_DIRECTION_INPUT);
}

// 返回: 0 - 有光, 1 - 无光
uint8_t LightSensor_GetStatus(void)
{
    bsp_io_level_t level;
    R_IOPORT_PinRead(g_ioport.p_ctrl, LIGHT_SENSOR_PIN, &level);
    
    // 模块高电平表示无光 (暗)，低电平表示有光 (亮)
    if (level == BSP_IO_LEVEL_HIGH) {
        return 1;
    } else {
        return 0;
    }
}
