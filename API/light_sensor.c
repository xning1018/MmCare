#include "light_sensor.h"

// 保存当前光线状态：0 为有光，1 为无光（暗）
volatile uint8_t g_light_status = 0; 

void LightSensor_Init(void)
{
    // 打开并使能外部中断 (FSP配置的 g_light_sensor_irq)
    R_ICU_ExternalIrqOpen(&g_light_sensor_irq_ctrl, &g_light_sensor_irq_cfg);
    R_ICU_ExternalIrqEnable(&g_light_sensor_irq_ctrl);

    // 初始化时主动读取一次引脚电平，确定初始状态
    bsp_io_level_t level;
    R_IOPORT_PinRead(g_ioport.p_ctrl, LIGHT_SENSOR_PIN, &level);
    if (level == BSP_IO_LEVEL_HIGH) {
        g_light_status = 1; // 模块高电平表示无光 (暗)
    } else {
        g_light_status = 0; // 模块低电平表示有光 (亮)
    }
}

// 返回: 0 - 有光, 1 - 无光
uint8_t LightSensor_GetStatus(void)
{
    // 直接返回中断维护的全局状态
    return g_light_status;
}

// FSP 中断回调函数，当光线阈值跳变时触发
void light_sensor_callback(external_irq_callback_args_t *p_args)
{
    if (p_args->channel == g_light_sensor_irq_cfg.channel)
    {
        // 触发中断后，读取当前电平状态更新标志位
        bsp_io_level_t level;
        R_IOPORT_PinRead(g_ioport.p_ctrl, LIGHT_SENSOR_PIN, &level);
        
        if (level == BSP_IO_LEVEL_HIGH) {
            g_light_status = 1; // 变暗
        } else {
            g_light_status = 0; // 变亮
        }
    }
}
