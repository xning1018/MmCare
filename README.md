# Mmcare - 智能康养监护终端 (Intelligent Healthcare Monitoring Terminal)

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Platform: Renesas RA](https://img.shields.io/badge/Platform-Renesas_RA-red.svg)]()
[![Build: FSP](https://img.shields.io/badge/Build-e²_studio-green.svg)]()

Mmcare 是一款面向智慧养老、病房监护等场景打造的高可靠性、无感知、全隐私保护的物联网监控终端设备。本项目基于瑞萨（Renesas）RA 系列 MCU 开发，采用前后台非阻塞架构，集成了创新的**双毫米波雷达交叉验证算法**，彻底解决传统单一雷达跌倒误报率高的痛点。

## ✨ 核心特性 (Features)

*   🎯 **双雷达交叉验证算法**：融合“主跌倒雷达”与“LD2450 轨迹雷达”。通过捕捉瞬间速度突变（`|V|>40`）与空间位置跳变，并采用锁存标志（`anomaly_flag`）对齐时间轴，实现极高准确率的真实跌倒确认。
*   📡 **4G 蜂窝全场景上云**：集成 ML307R 4G 模组，自动 AT 拨号建连。以标准 JSON 格式通过 HTTP POST 实时上报状态（设备ID、温度、跌倒、XY坐标、速度等），内置 2000ms 防洪泛冷却机制。
*   🔊 **多模态声光预警**：集成 OLED 状态屏、高亮 LED 及 MY1680 语音芯片，支持诸如 405/202 相位交替闪烁与定制语音播报的本地闭环响应。
*   🌡️ **多维环境感知**：板载 NTC 热敏电阻（基于 B 值对数精确解算）与环境光传感器，全天候感知起居环境。
*   🔋 **高效裸机低功耗架构**：摒弃臃肿的 RTOS，采用 `System_Yield` 时间轮片非阻塞调度。空闲时自动进入 MCU 低功耗休眠模式（LPM）。
*   🔌 **面向工业量产**：支持“去杜邦线”的高集成度 PCB 底板化拓展，为未来增加后备 UPS 电池及 RS485 有线组网打下物理基础。

## 🛠️ 硬件依赖 (Hardware)

*   **MCU 主控**: 瑞萨 RA 系列开发板 (如 RA4M2 / RA6M5)
*   **雷达 1**: 毫米波跌倒检测雷达 (UART)
*   **雷达 2**: HLK-LD2450 轨迹追踪雷达 (UART)
*   **通信模组**: 中移物联 ML307R 4G Cat.1 模块
*   **传感器**: 10K NTC 热敏电阻、光敏电阻
*   **交互外设**: 0.96寸 I2C OLED、MY1680 语音播报模块、无源蜂鸣器、LED

## 📁 核心代码结构 (Architecture)

```text
Mmcare/
├── src/
│   ├── hal_entry.c        # 主循环，状态机，4G 上传逻辑，低功耗管理
│   ├── ld2450.c/h         # 轨迹雷达串口解析及目标追踪
│   ├── fall_alarm.c/h     # 主跌倒雷达处理及双雷达交叉验证逻辑
│   ├── ml307r.c/h         # 4G 模组 AT 指令驱动与 HTTP/TCP 协议栈
│   ├── my1680.c/h         # 语音播报芯片驱动
│   ├── led.c/h, buzzer.c  # 声光警报控制
│   └── oled.c/h           # OLED 屏幕驱动及 UI 渲染
├── API/
│   └── ntc_temp.c/h       # NTC ADC 采样与温度物理公式换算
└── README.md
```

## 🚀 快速上手 (Getting Started)

1. **环境准备**：安装 Renesas e² studio 与 FSP (Flexible Software Package)。
2. **硬件连接**：根据代码中的引脚配置（通过 FSP Configuration），连接各外设模块的 UART、ADC 和 GPIO。
3. **参数配置**：
   * 在 `src/hal_entry.c` 中修改服务器地址：`const char *http_url = "你的服务器IP:端口/data";`
4. **编译下载**：在 e² studio 中点击 Build Project，连接 J-Link 或 DAP 下载器，将固件烧录至 MCU。
5. **运行**：设备上电后，OLED 将显示初始化状态，4G 拨号成功后显示 `DIAL IP: OK`。随后雷达开始工作。

## ⚠️ 开发状态与免责声明 (Disclaimer)

> [!WARNING]
> **当前工程仍处于原型开发与概念验证（PoC）阶段，尚未达到彻底完善的商业量产标准。**
> 
> *   **潜在漏洞**：代码中可能存在未处理的边界条件、偶发的网络断线重连失败或串口数据丢包等缺陷。
> *   **硬件可靠性**：目前物理连接主要依赖杜邦线，存在信号串扰和接触不良的隐患，强烈建议依照前文规划设计专用的 PCB 扩展底板。
> *   **算法鲁棒性**：双雷达交叉验证算法在极端遮挡或复杂多径环境下的参数仍需持续优化。
> 
> 本项目开源的主要目的是分享架构思路与技术方案。**请勿直接将当前版本应用于对生命安全负责的严肃医疗或商业级看护场景。** 仍有许多工作需要完善，欢迎各位同仁提交 Issue 探讨或发起 PR 共同建设！

## 📜 开源协议 (License)

本项目采用 [MIT License](LICENSE) 开源协议。欢迎极客、开发者及医疗康养行业的伙伴复用代码或共同探讨硬件方案。
