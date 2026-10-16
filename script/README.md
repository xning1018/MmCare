# Renesas FSP 分散加载脚本目录 (Linker Scatter Script)

> **注意：请勿删除或移动本目录！**

本目录是 **瑞萨 FSP (Flexible Software Package)** 与 **Keil MDK (Arm Compiler 6)** 协同工作的核心系统目录：

- `fsp.scat`：MCU 内存布局与 Flash 分散加载脚本（Scatter File）。
  在 Keil 工程文件 `Mmcare.uvprojx` 中通过 `<ScatterFile>.\script\fsp.scat</ScatterFile>` 显式指定并依赖。
- `ac6/fsp_keep.via`：编译器符号保活规则参数文件。

> **说明**：
> - 嵌入式系统自动化脚本、数据采集与 AI 训练工具位于 **`scripts/`**（复数）目录下。
> - 本目录 **`script/`**（单数）为瑞萨官方固定的链接器工具链目录，属于底层构建规范的一部分。
