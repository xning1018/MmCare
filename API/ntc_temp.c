/* ================================================================
 * ntc_temp.c
 * NTC 热敏电阻温度采集模块实现
 * ================================================================ */
#include "ntc_temp.h"
#include <math.h>

/**
 * @brief 初始化 ADC 模块
 */
void NTC_Init(void)
{
    fsp_err_t err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
    if (FSP_SUCCESS == err)
    {
        R_ADC_ScanCfg(&g_adc0_ctrl, &g_adc0_channel_cfg);
    }
}

/**
 * @brief 触发转换并获取摄氏度温度
 * @return float 摄氏度
 */
float NTC_GetTemperature(void)
{
    uint16_t adc_raw = 0;
    adc_status_t adc_status; // FSP 规范：定义状态结构体
    
    // 启动单次扫描
    R_ADC_ScanStart(&g_adc0_ctrl);
    
    // 等待转换完成 (遵循最新 FSP API 规范的安全轮询机制)
    do {
        R_ADC_StatusGet(&g_adc0_ctrl, &adc_status);
    } while (ADC_STATE_SCAN_IN_PROGRESS == adc_status.state);
    
    // 读取通道 3 的数据 (需确保 FSP 中配置的是 Channel 3)
    R_ADC_Read(&g_adc0_ctrl, ADC_CHANNEL_3, &adc_raw);
    
    if (adc_raw == 0 || adc_raw >= 4095) {
        return -99.9f; // 传感器断线或短路标志
    }

    // 根据分压拓扑计算 NTC 阻值
    float r_ntc = FIXED_RESISTOR * ((float)adc_raw / (4095.0f - (float)adc_raw));

    // 计算开尔文温度并转换为摄氏度
    // 核心修改：使用 logf() 替代 log()，匹配单精度 float 并消除编译警告
    float temp_k = 1.0f / ((1.0f / NTC_T0) + (1.0f / NTC_B_VALUE) * logf(r_ntc / NTC_R0));
    
    return temp_k - 273.15f;
}

