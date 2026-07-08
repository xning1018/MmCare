#ifndef _MY_1680_H_
#define _MY_1680_H_

#include "hal_data.h" 

/* ================= 硬件配置宏 ================= */
// 更新为 P413 作为 BUSY 状态检测引脚
#define MY1680_BUSY_PIN     BSP_IO_PORT_04_PIN_13

// 引用 FSP 中配置的 UART 实例 (Channel 2)
extern const uart_instance_t g_uart_my1680;

/* MY1680 忙等待超时次数，防止硬件故障死锁 */
#define MY1680_BUSY_TIMEOUT  100000u

/********************** MY1680 命令宏定义 *******************************/
#define CMD_PLAY            0x11            //播放
#define CMD_STOP            0x12            //暂停
#define CMD_NEXT            0x13            //下一曲
#define CMD_PREV            0x14            //上一曲
#define CMD_VOICEADD        0x15            //音量加
#define CMD_VOICESUB        0x16            //音量减
#define CMD_RESET           0x19            //复位
#define CMD_SPEED           0x1A            //快进
#define CMD_SLOW            0x1B            //快退
#define CMD_STARTorSTOP     0x1C            //播放/暂停
#define CMD_END             0x1E            //停止
#define CMD_ROOTCHO_MUSIC   0x41            //根目录选择歌曲
#define CMD_CHOOSE_MUSIC    0x42            //任意目录选择歌曲

void MY1680_Init(void);

// 模组底层发送函数
void MY1680_SendData(uint8_t *pdata, uint32_t lenth);
uint8_t Voice_XorCheck(uint8_t *pdata, uint8_t lenth);
void Voice_SendCmd(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg_lenth);
uint8_t MY1680_GetBusyStatus(void);

// 上层封装函数
uint8_t MY1680_PlayMusic_RootDir(uint16_t filename);
uint8_t MY1680_PlayMusic_SubDir(uint8_t dirname, uint8_t filename);
void MY1680_SetVol(uint8_t vol);
void MY1680_PauseAndStart(void);
void MY1680_Stop(void);
void MY1680_PlayMusic_RootDir_Insert(uint16_t filename);
void MY1680_PlayMusic_SubDir_Insert(uint8_t dirname, uint8_t filename);
void my1680_uart_callback(uart_callback_args_t * p_args);
// 应用层数字播报函数
/* MY1680 播放类型枚举 */
#define MY1680_Int    0u
#define MY1680_Float  1u
void MY1680_PlayNum(void *pdata, uint8_t data_type);

#endif /* _MY_1680_H_ */

