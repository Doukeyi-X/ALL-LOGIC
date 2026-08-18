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

### 已支持的设备（社区驱动）

下列设备为二次开发中自行接入，**不是**各厂商官方上位机：

1. **CH32H417 逻辑分析仪** — [立创开源硬件平台项目](https://oshwhub.com/q2h2/project_bszkxrnf)
2. **SLogic16U3（5 Gbps 版本）** — [Sipeed 介绍页](https://wiki.sipeed.com/hardware/zh/logic_analyzer/slogic16u3/Introduction.html)
3. **PXLogic32U3（5 Gbps 版本）** — [MarryChip](https://marrychip.com/)

原 DSView 已支持的 DreamSourceLab 仪器，在二次开发中仍然可用。

### 给个 Star，也欢迎提需求

如果这个项目对你有用，请点一下右上角的 **Star**，方便更多人找到它，也是对我们继续改下去的最大鼓励。

遇到崩溃、采不到数、驱动识别不对，或你手头有想接入的逻辑分析仪，请开 Issue：

https://github.com/Doukeyi-X/ALL-LOGIC/issues

Issue 里尽量写清：设备型号、系统版本、复现步骤，或附上硬件开源/购买链接。我们会按社区互操作的方式评估能否加上。

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

### Contributors

| | |
|---|---|
| [Doukeyi-X](https://github.com/Doukeyi-X) | ALL LOGIC 二次开发 |
| [DreamSourceLab](https://github.com/DreamSourceLab)（梦源） | 原作 DSView |

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

### Supported devices (community drivers)

These were added in this secondary development. They are **not** official vendor hosts:

1. **CH32H417 logic analyzer** — [OSHWHUB project](https://oshwhub.com/q2h2/project_bszkxrnf)
2. **SLogic16U3 (5 Gbps)** — [Sipeed introduction](https://wiki.sipeed.com/hardware/zh/logic_analyzer/slogic16u3/Introduction.html)
3. **PXLogic32U3 (5 Gbps)** — [MarryChip](https://marrychip.com/)

DreamSourceLab instruments already supported by upstream DSView still work.

### Star, bugs, and new hardware

If ALL LOGIC helps you, please **Star** the repo. It makes the project easier to find and keeps the work going.

Found a crash, a device that will not enumerate, or a logic analyzer you want added? Open an issue:

https://github.com/Doukeyi-X/ALL-LOGIC/issues

Please include the model, OS, steps to reproduce, or a hardware / shop link. We will evaluate new analyzers as community interoperability work.

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

### Contributors

| | |
|---|---|
| [Doukeyi-X](https://github.com/Doukeyi-X) | ALL LOGIC secondary development |
| [DreamSourceLab](https://github.com/DreamSourceLab) | Original DSView |

### Upstream

- DSView: https://github.com/DreamSourceLab/DSView
- sigrok / PulseView: https://sigrok.org
