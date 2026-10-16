#include "oled.h"
#include "OLED_Font.h" 
#include <string.h>

static volatile bool g_i2c_tx_complete = false;

void oled_i2c_callback(i2c_master_callback_args_t *p_args)
{
    if (I2C_MASTER_EVENT_TX_COMPLETE == p_args->event) g_i2c_tx_complete = true;
}

static void OLED_I2C_WriteBytes(uint8_t *p_data, uint32_t len)
{
    g_i2c_tx_complete = false;
    fsp_err_t err = R_IIC_MASTER_Write(&g_i2c_master0_ctrl, p_data, len, false);
    if (FSP_SUCCESS == err) {
        uint32_t timeout = 50000;
        while (!g_i2c_tx_complete && --timeout);
    }
}

void OLED_WriteCommand(uint8_t Command)
{
    uint8_t buffer[2];
    buffer[0] = 0x00; 
    buffer[1] = Command;
    OLED_I2C_WriteBytes(buffer, 2);
}

void OLED_WriteData(uint8_t Data)
{
    uint8_t buffer[2];
    buffer[0] = 0x40; 
    buffer[1] = Data;
    OLED_I2C_WriteBytes(buffer, 2);
}

void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);					
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	
    OLED_WriteCommand(0x00 | (X & 0x0F));			
}

void OLED_Clear(void)
{  
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for(i = 0; i < 128; i++) OLED_WriteData(0x00);
    }
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{     	
    uint8_t i;
    if (Char < ' ' || Char > '~') return; 

    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i]);			
    
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	
    for (i = 0; i < 8; i++) OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    if (String == NULL)
    {
        return;
    }

    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) OLED_ShowChar(Line, Column + i, String[i]);
}

uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)							
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

#if 0
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 =(uint32_t) Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 =(uint32_t) (-Number);
    }
    for (i = 0; i < Length; i++)							
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)							
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10) OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        else OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)							
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
}

void OLED_ShowChineseString(uint8_t Line, uint8_t Column, char *ChineseString)
{
    uint32_t i = 0;
    uint8_t current_column = Column;
    uint32_t font_len = sizeof(OLED_CH) / sizeof(ChineseCell_t);

    while (ChineseString[i] != '\0')
    {
        if ((uint8_t)ChineseString[i] >= 0x80)
        {
            if (ChineseString[i + 1] == '\0')
            {
                /* 非法结尾，显示空白占位并退出 */
                OLED_SetCursor((Line - 1) * 2, (current_column - 1) * 8);
                for (uint32_t j = 0; j < 16; j++) OLED_WriteData(0x00);
                OLED_SetCursor((Line - 1) * 2 + 1, (current_column - 1) * 8);
                for (uint32_t j = 0; j < 16; j++) OLED_WriteData(0x00);
                break;
            }

            bool found = false;
            for (uint32_t k = 0; k < font_len; k++)
            {
                if ((uint8_t)OLED_CH[k].Index[0] == (uint8_t)ChineseString[i] &&
                    (uint8_t)OLED_CH[k].Index[1] == (uint8_t)ChineseString[i + 1])
                {
                    found = true;

                    /* 列行式数据：前16字节直接写Page0（上半），后16字节直接写Page1（下半）*/
                    OLED_SetCursor((Line - 1) * 2, (current_column - 1) * 8);
                    for (uint32_t c = 0; c < 16; c++)
                        OLED_WriteData(OLED_CH[k].Msk[c]);

                    OLED_SetCursor((Line - 1) * 2 + 1, (current_column - 1) * 8);
                    for (uint32_t c = 0; c < 16; c++)
                        OLED_WriteData(OLED_CH[k].Msk[16 + c]);

                    break;
                }
            }

            if (!found)
            {
                /* 找不到字符，显示空白占位 */
                OLED_SetCursor((Line - 1) * 2, (current_column - 1) * 8);
                for (uint32_t j = 0; j < 16; j++) OLED_WriteData(0x00);
                OLED_SetCursor((Line - 1) * 2 + 1, (current_column - 1) * 8);
                for (uint32_t j = 0; j < 16; j++) OLED_WriteData(0x00);
            }

            current_column += 2;
            i += 2;
        }
        else
        {
            /* ASCII字符，走原来的英文显示逻辑 */
            OLED_ShowChar(Line, current_column, ChineseString[i]);
            current_column += 1;
            i += 1;
        }
    }
}
#endif

void OLED_Init(void)
{
    R_IIC_MASTER_Open(&g_i2c_master0_ctrl, &g_i2c_master0_cfg);
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
    
    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x30);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    
    OLED_WriteCommand(0xAF);
    OLED_Clear();
}

