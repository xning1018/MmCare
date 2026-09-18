#include "led.h"

static bsp_io_level_t g_led_405_status = BSP_IO_LEVEL_LOW;
static bsp_io_level_t g_led_202_status = BSP_IO_LEVEL_LOW;
static bsp_io_level_t g_led_604_status = BSP_IO_LEVEL_LOW;
static bsp_io_level_t g_led_210_status = BSP_IO_LEVEL_LOW;

static void LED_Write(bsp_io_port_pin_t pin, bsp_io_level_t level) {
  R_IOPORT_PinWrite(&g_ioport_ctrl, pin, level);
}

void LED_Init(void) {
  // 共阳极灯带：高电平熄灭，低电平点亮。
  g_led_405_status = BSP_IO_LEVEL_HIGH;
  g_led_202_status = BSP_IO_LEVEL_HIGH;
  g_led_604_status = BSP_IO_LEVEL_HIGH;
  // 210 端口配置为 Initial Low，推断为高电平点亮
  g_led_210_status = BSP_IO_LEVEL_LOW;
  
  LED_Write(LED_PIN_405, g_led_405_status);
  LED_Write(LED_PIN_202, g_led_202_status);
  LED_Write(LED_PIN_604, g_led_604_status);
  LED_Write(LED_PIN_210, g_led_210_status);
}

void LED_On(void) {
  g_led_405_status = BSP_IO_LEVEL_LOW; // 低电平点亮
  LED_Write(LED_PIN_405, g_led_405_status);
}

void LED_Off(void) {
  g_led_405_status = BSP_IO_LEVEL_HIGH; // 高电平熄灭
  LED_Write(LED_PIN_405, g_led_405_status);
}

void LED_Toggle(void) {
  g_led_405_status = (g_led_405_status == BSP_IO_LEVEL_LOW) ? BSP_IO_LEVEL_HIGH
                                                            : BSP_IO_LEVEL_LOW;
  LED_Write(LED_PIN_405, g_led_405_status);
}

void LED202_On(void) {
  g_led_202_status = BSP_IO_LEVEL_LOW; // 低电平点亮
  LED_Write(LED_PIN_202, g_led_202_status);
}

void LED202_Off(void) {
  g_led_202_status = BSP_IO_LEVEL_HIGH; // 高电平熄灭
  LED_Write(LED_PIN_202, g_led_202_status);
}

void LED202_Toggle(void) {
  g_led_202_status = (g_led_202_status == BSP_IO_LEVEL_LOW) ? BSP_IO_LEVEL_HIGH
                                                            : BSP_IO_LEVEL_LOW;
  LED_Write(LED_PIN_202, g_led_202_status);
}

void LED604_On(void) {
  g_led_604_status = BSP_IO_LEVEL_LOW; // 低电平点亮
  LED_Write(LED_PIN_604, g_led_604_status);
}

void LED604_Off(void) {
  g_led_604_status = BSP_IO_LEVEL_HIGH; // 高电平熄灭
  LED_Write(LED_PIN_604, g_led_604_status);
}

void LED604_Toggle(void) {
  g_led_604_status = (g_led_604_status == BSP_IO_LEVEL_LOW) ? BSP_IO_LEVEL_HIGH
                                                            : BSP_IO_LEVEL_LOW;
  LED_Write(LED_PIN_604, g_led_604_status);
}

void LED210_On(void) {
  g_led_210_status = BSP_IO_LEVEL_HIGH; // 高电平点亮
  LED_Write(LED_PIN_210, g_led_210_status);
}

void LED210_Off(void) {
  g_led_210_status = BSP_IO_LEVEL_LOW; // 低电平熄灭
  LED_Write(LED_PIN_210, g_led_210_status);
}

void LED210_Toggle(void) {
  g_led_210_status = (g_led_210_status == BSP_IO_LEVEL_HIGH) ? BSP_IO_LEVEL_LOW
                                                             : BSP_IO_LEVEL_HIGH;
  LED_Write(LED_PIN_210, g_led_210_status);
}
