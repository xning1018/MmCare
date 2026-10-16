#include "buzzer.h"

/* 静态全局变量，用于记录当前蜂鸣器状态
 * 修正为低电平触发逻辑：
 * 1 (HIGH) : 默认高电平，不响 (静音)
 * 0 (LOW)  : 低电平，响 (鸣叫)
 */
static bsp_io_level_t g_buzzer_status = BSP_IO_LEVEL_HIGH;

/**
 * @brief  蜂鸣器初始化函数
 */
void Buzzer_Init(void)
{
    /* 初始化为高电平，保持静音 */
    g_buzzer_status = BSP_IO_LEVEL_HIGH;
    R_IOPORT_PinWrite(&g_ioport_ctrl, BUZZER_PIN, g_buzzer_status);
}

/**
 * @brief  开启蜂鸣器（鸣叫）
 */
void Buzzer_On(void)
{
    /* 输出低电平触发鸣叫 */
    g_buzzer_status = BSP_IO_LEVEL_LOW;
    R_IOPORT_PinWrite(&g_ioport_ctrl, BUZZER_PIN, g_buzzer_status);
}

/**
 * @brief  关闭蜂鸣器（静音）
 */
void Buzzer_Off(void)
{
    /* 输出高电平关闭鸣叫 */
    g_buzzer_status = BSP_IO_LEVEL_HIGH;
    R_IOPORT_PinWrite(&g_ioport_ctrl, BUZZER_PIN, g_buzzer_status);
}

/**
 * @brief  翻转蜂鸣器状态
 */
void Buzzer_Toggle(void)
{
    if (g_buzzer_status == BSP_IO_LEVEL_HIGH)
    {
        g_buzzer_status = BSP_IO_LEVEL_LOW;  // 从静音切换为鸣叫
    }
    else
    {
        g_buzzer_status = BSP_IO_LEVEL_HIGH; // 从鸣叫切换为静音
    }
    R_IOPORT_PinWrite(&g_ioport_ctrl, BUZZER_PIN, g_buzzer_status);
}



