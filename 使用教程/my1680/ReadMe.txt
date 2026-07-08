my1680 

喇叭黑上红下 
busy  413   :只需配置input模式
5V
GND

配置UART

点击SCi2
Asynchronous UART 
然后锁定引脚

进入 Stacks 标签页，点击 New Stack -> Connectivity -> UART (r_sci_uart)。
Name: g_uart_my1680
Channel: 2 
Baud Rate: 9600
Callback: my1680_uart_callback      需要手打间隔少一个下划线

RX  --p302
TX  --p301   




  


