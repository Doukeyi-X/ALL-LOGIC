# ALL LOGIC

[English](#english) · [中文](#中文)

---

## 中文

ALL LOGIC 是一款**非官方**的多厂商逻辑分析仪上位机。

我们的工作**不是从零写一套新软件**，而是在 DreamSourceLab 已经开源的 **[DSView](https://github.com/DreamSourceLab/DSView)** 代码上做**二次开发**。DSView 本身又基于 [sigrok](https://sigrok.org) 的 PulseView。原作者的版权声明仍保留在对应源文件中。

**ALL LOGIC 不是 DreamSourceLab 官方产品，也没有获得任何硬件厂商的授权或背书。**  
设备列表里出现的「DSLogic」等名称，只是仪器自己上报的产品名，不代表官方支持。

### 我们改了什么

在 DSView 原有开源代码之上，主要做了这些二次开发：

- 更换软件名称与图标（ALL LOGIC）
- 增加其他厂商逻辑分析仪的社区驱动，便于互操作
- 增加本机 MCP 接口，方便 AI 客户端控制采集与解码
- 关于页、许可证说明改为明确标注「二次修改 / 非官方」

本仓库**只发布上位机源码**。设备固件工程、安装包、本地校准工具不放在 git 里。Windows 安装包和绿色版见 [Releases](https://github.com/Doukeyi-X/ALL-LOGIC/releases)。

### 目录说明

| 路径 | 内容 |
|------|------|
| `DSView/` | 图形界面（基于原 DSView / PulseView） |
| `libsigrok4DSL/` | 设备层与社区驱动 |
| `libsigrokdecode4DSL/` | 协议解码器 |
| `lang/` | 界面语言 |
| `tools/package_windows.ps1` | Windows 打包脚本 |
| `COPYING` | GNU GPLv3 全文 |
| `NOTICE.txt` | 二次开发与商标说明 |

### 许可证

整套程序按 **GNU GPLv3 或更高版本** 发布。详见 `COPYING` 和 `NOTICE.txt`。

你可以自由使用、修改、再分发；若对外提供二进制，必须同时提供对应源码，并保留本许可证与原作者版权。

### 上游项目

- DSView：https://github.com/DreamSourceLab/DSView
- sigrok / PulseView：https://sigrok.org

---

## English

ALL LOGIC is an **unofficial** multi-vendor logic analyzer host.

This project is **not a from-scratch rewrite**. It is **secondary development** on the already open-sourced **[DSView](https://github.com/DreamSourceLab/DSView)** code from DreamSourceLab. DSView itself is based on [sigrok](https://sigrok.org) PulseView. Original copyright notices remain in the corresponding source files.

**ALL LOGIC is not an official DreamSourceLab product and is not licensed or endorsed by any hardware vendor.**  
Names such as “DSLogic” in the device list are product names reported by the instrument. They do not mean official vendor support.

### What we changed

On top of the original DSView sources we mainly:

- Rebranded the host as ALL LOGIC (name and icon)
- Added community drivers for additional logic analyzers (interoperability)
- Added a local MCP interface so AI clients can control capture and decode
- Replaced the About / legal text so the secondary-modification status is explicit

This repository publishes **host source only**. Device firmware trees, installers, and local calibration tools are not in git. Windows setup and portable zip are in [Releases](https://github.com/Doukeyi-X/ALL-LOGIC/releases).

### Layout

| Path | Contents |
|------|----------|
| `DSView/` | GUI (based on original DSView / PulseView) |
| `libsigrok4DSL/` | Device layer and community drivers |
| `libsigrokdecode4DSL/` | Protocol decoders |
| `lang/` | UI language files |
| `tools/package_windows.ps1` | Windows packaging script |
| `COPYING` | Full GNU GPLv3 text |
| `NOTICE.txt` | Secondary-development and trademark notes |

### License

The program as a whole is licensed under the **GNU GPLv3 or later**. See `COPYING` and `NOTICE.txt`.

You may use, modify, and redistribute it. If you distribute binaries, you must also provide the corresponding source and keep this license and the original copyrights.

### Upstream

- DSView: https://github.com/DreamSourceLab/DSView
- sigrok / PulseView: https://sigrok.org
