#include "my1680.h"

// 用于标记 UART 数据是否发送完成
volatile bool g_my1680_tx_complete = false;

/*
 * UART 回调函数 (FSP 触发)
 * 必须在 FSP Configurator 中将 UART 的 Callback 命名为 my1680_uart_callback
 */
void my1680_uart_callback(uart_callback_args_t * p_args)
{
    if (UART_EVENT_TX_COMPLETE == p_args->event)
    {
        g_my1680_tx_complete = true;
    }
}

/*
 * 底层串口发送字符串
 */
static void UART_SendStr(uint8_t *pstr, uint32_t lenth)
{
    g_my1680_tx_complete = false;
    
    // 调用 FSP API 发送数据
    fsp_err_t err = R_SCI_UART_Write(g_uart_my1680.p_ctrl, pstr, lenth);
    if (FSP_SUCCESS == err)
    {
        // 阻塞等待传输完成 (在中断中会置位 g_my1680_tx_complete)
        while (!g_my1680_tx_complete)
        {
            ; // 等待
        }
    }
}

/*
 * 获取 BUSY 引脚状态 (P413)
 * 返回值: 1 -- 模块繁忙 (播放中), 0 -- 空闲 (暂停或停止)
 */
uint8_t MY1680_GetBusyStatus(void)
{
    bsp_io_level_t level;
    R_IOPORT_PinRead(g_ioport.p_ctrl, MY1680_BUSY_PIN, &level);
    return (level == BSP_IO_LEVEL_HIGH) ? 1 : 0;
}

/*
 * 模块初始化
 */
void MY1680_Init(void)
{
    // 打开 UART 设备
    R_SCI_UART_Open(g_uart_my1680.p_ctrl, g_uart_my1680.p_cfg);
    
    // 模块上电需要 300ms 以上初始化时间
    R_BSP_SoftwareDelay(350, BSP_DELAY_UNITS_MILLISECONDS);
    
    // 停止当前所有的播放
    MY1680_Stop();
}

/*
 * 发送固定长度的数据
 */
void MY1680_SendData(uint8_t *pdata, uint32_t lenth)
{
    UART_SendStr(pdata, lenth);
    
    // 说明书要求连续发送两条命令之间间隔在 20MS 以上
    R_BSP_SoftwareDelay(25, BSP_DELAY_UNITS_MILLISECONDS); 
}

/*
 * 异或校验计算
 */
uint8_t Voice_XorCheck(uint8_t *pdata, uint8_t lenth)
{
    uint8_t r_value = *pdata;
    uint8_t i = 0;
    for(i = 1; i < lenth; i++)
    {
        r_value ^= pdata[i];
    }
    return r_value;
}

/*
 * 发送带参数的协议命令
 */
void Voice_SendCmd(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg_lenth)
{
    uint8_t cmd_buff[] = {0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF}; 
    uint8_t total_lenth = 0; 
    
    cmd_buff[total_lenth++] = 0x7E; 
    cmd_buff[total_lenth++] = 1 + 1 + arg_lenth + 1; // 长度 = 数据长度(1) + 命令(1) + 参数长度 + 校验(1)
    cmd_buff[total_lenth++] = cmd;  
    
    // 参数填充
    switch(arg_lenth)
    {
        case 0: break; 
        case 1: 
            cmd_buff[total_lenth++] = arg1;
            break; 
        case 2: 
            cmd_buff[total_lenth++] = arg1; 
            cmd_buff[total_lenth++] = arg2; 
            break; 
    }
    // 计算校验码
    cmd_buff[total_lenth++] = Voice_XorCheck(&cmd_buff[1], 2 + arg_lenth); 
    // 帧结束
    cmd_buff[total_lenth++] = 0xEF; 
    
    // 发送
    MY1680_SendData(cmd_buff, total_lenth);
}

/*
 * 播放根目录下某个音频文件
 */
uint8_t MY1680_PlayMusic_RootDir(uint16_t filename)
{
    if(MY1680_GetBusyStatus() == 1) return 0; // 繁忙检测
    uint8_t high_byte = (uint8_t)(filename / 100);
    uint8_t low_byte = (uint8_t)(filename % 100);
    Voice_SendCmd(0x41, high_byte, low_byte, 2);
    return 1;
}

/*
 * 播放子目录下某个音频文件
 */
uint8_t MY1680_PlayMusic_SubDir(uint8_t dirname, uint8_t filename)
{
    if(MY1680_GetBusyStatus() == 1) return 0; // 繁忙检测
    Voice_SendCmd(0x42, dirname, filename, 2);
    return 1;
}

/*
 * 设置喇叭音量
 */
void MY1680_SetVol(uint8_t vol)
{
    Voice_SendCmd(0x31, vol, 0, 1);
}

/*
 * 启动和暂停
 */
void MY1680_PauseAndStart(void)
{
    Voice_SendCmd(0x1C, 0, 0, 0);
}

/*
 * 停止播放
 */
void MY1680_Stop(void)
{
    Voice_SendCmd(0x1E, 0, 0, 0);
}

/*
 * 插播根目录下某个音频文件
 */
void MY1680_PlayMusic_RootDir_Insert(uint16_t filename)
{
    uint8_t high_byte = (uint8_t)(filename / 100);
    uint8_t low_byte = (uint8_t)(filename % 100);
    Voice_SendCmd(0x43, high_byte, low_byte, 2);
}

/*
 * 插播子目录下某个音频文件
 */
void MY1680_PlayMusic_SubDir_Insert(uint8_t dirname, uint8_t filename)
{
    Voice_SendCmd(0x44, dirname, filename, 2);
}

/*
 * 播报数字
 */
void MY1680_PlayNum(void *pdata, uint8_t data_type)
{
    int data1 = 0;   
    float data2 = 0; 
    uint8_t num_bit[8] = {'-', 0, 0, 0, 0, 0, 0, 0}; 
    
    // 1.数据转化
    switch(data_type)
    {
        case MY1680_Int:  data1 = *(int *)pdata; break;
        case MY1680_Float:data2 = *(float *)pdata; break;
    }
    
    // 2.处理数据
    if(data_type == MY1680_Float)
    {
        data1 = (int)(data2 * 100);
        num_bit[6] = (uint8_t)(data1 % 100 / 10);
        num_bit[7] = (uint8_t)(data1 % 10);
        data1 = data1 / 100;
    }
    
    num_bit[4] = (uint8_t)(data1 % 10);
    num_bit[3] = (uint8_t)(data1 % 100 / 10);
    num_bit[2] = (uint8_t)(data1 % 1000 / 100);
    num_bit[1] = (uint8_t)(data1 / 1000);
    
    if(data1 < 0) num_bit[0] = '-';
    else num_bit[0] = '+';

    // 3.语音播报
    // 千位
    if(num_bit[1] != 0)
    {
        while(MY1680_PlayMusic_SubDir(0, num_bit[1]) == 0);
        while(MY1680_PlayMusic_SubDir(0, 21) == 0); 
    }
    if(num_bit[1] != 0 && num_bit[2] == 0 && num_bit[3] == 0 && num_bit[4] == 0) goto FLOAT_NUM;
    
    // 百位
    if(num_bit[2] != 0)
    {
        while(MY1680_PlayMusic_SubDir(0, num_bit[2]) == 0);
        while(MY1680_PlayMusic_SubDir(0, 20) == 0); 
    }
    if(num_bit[2] != 0 && num_bit[3] == 0 && num_bit[4] == 0) goto FLOAT_NUM;
    if(num_bit[1] != 0 && num_bit[2] == 0 && num_bit[3] != 0)
    {
        while(MY1680_PlayMusic_SubDir(0, 0) == 0); 
    }
    
    // 十位
    if(num_bit[3] != 0)
    {
        while(MY1680_PlayMusic_SubDir(0, num_bit[3]) == 0);
        while(MY1680_PlayMusic_SubDir(0, 10) == 0); 
    }
    if(num_bit[3] != 0 && num_bit[4] == 0) goto FLOAT_NUM;
    if(num_bit[1] != 0 && num_bit[2] == 0 && num_bit[3] == 0)
    {
        while(MY1680_PlayMusic_SubDir(0, 0) == 0);
    }
    
    // 个位
    if(num_bit[4] != 0)
        while(MY1680_PlayMusic_SubDir(0, num_bit[4]) == 0);
    else
        while(MY1680_PlayMusic_SubDir(0, 0) == 0);

FLOAT_NUM:
    // 小数部分
    if(data_type == MY1680_Float)
    {
        while(MY1680_PlayMusic_SubDir(0, 22) == 0); 
        while(MY1680_PlayMusic_SubDir(0, num_bit[6]) == 0);
        if(num_bit[7] != 0)
            while(MY1680_PlayMusic_SubDir(0, num_bit[7]) == 0);
    }
}




