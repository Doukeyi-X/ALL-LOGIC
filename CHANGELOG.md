# Changelog

ALL LOGIC 二次开发记录。上游 DSView 历史见 `DSView/NEWS`。

---

## 1.4.0 — 2026-08-20

### 新增设备支持

- **ATK-Logic**（正点原子 DL16 等）— USB `1A86:FFCC`，按 [官方上位机](https://github.com/alientek-openedv/atk-logic) 协议移植的社区驱动。支持识别、滚动 / 重复 / 缓冲采集（不含 PWM 设置界面）。
- **nanoDLA / Cypress FX2（fx2lafw）** — USB `1D50:608C`（8 通道）/ `1D50:608D`（16 通道）。

原 DSView 已支持的 DreamSourceLab 仪器，以及此前接入的 CH32H417、SLogic16U3、PXLogic32U3，仍然可用。

### ATK-Logic

- 流式采样率按官方规则 `Hz × 通道数 ≤ 320e6`（16 通道上限 20 MHz）。缓冲模式 USB2 DL16 仍可到 250 MHz。
- USB2 设备不再误报「低速 / 请改插 USB3」。
- 滚动、重复、缓冲采集对齐官方组帧与解析；滚动窗口不再 16 MSa 就停。
- 修复高速采集一段时间后卡住、数据不再上传（Windows 上异步 IN 过程中同步 OUT 会死锁）。
- 不再给滞后通道补零，避免时间轴错位。

### nanoDLA / FX2

- 已在运行 fx2lafw 时不再重复下载 RAM 固件（与 PulseView 一致）；仅真正的裸 FX2 才会上传。
- 不再匹配 Cypress 引导 `04B4:8613`，避免插入 DSLogic 时先显示成 nano。

### 其它

- Windows 热插拔白名单补上 nanoDLA 与 ATK-Logic，插上即可出现在设备列表。
- Windows 安装包增加可选组件：**将 `.dsl` 会话文件关联到 ALL LOGIC**（默认勾选，卸载时若仍指向本程序则清除）。

### 下载

- `ALLLOGIC-1.4.0-win64-setup.exe` — Windows 安装包
- `ALLLOGIC-1.4.0-win64.zip` — 绿色版（解压即用）

见 [GitHub Releases](https://github.com/Doukeyi-X/ALL-LOGIC/releases)。CH32H417 固件与第一次使用步骤见 [OpenSourceLogic-CH32H417](https://github.com/Doukeyi-X/OpenSourceLogic-CH32H417)。

非官方社区上位机，与正点原子、Muse Lab、梦源等厂商无隶属关系。

---

## English

### 1.4.0 — 2026-08-20

**Added**

- ATK-Logic (Alientek DL16 and similar), USB `1A86:FFCC`. Community driver ported from the [official host](https://github.com/alientek-openedv/atk-logic). Identify plus stream / repeat / buffer capture (no PWM UI).
- nanoDLA / Cypress FX2 via fx2lafw, USB `1D50:608C` (8ch) / `1D50:608D` (16ch).
- Optional installer component to associate `.dsl` session files with ALL LOGIC.

**ATK-Logic**

- Live/stream rate cap matches the official host: `Hz × channels ≤ 320e6` (20 MHz at 16 channels). USB2 DL16 buffer still goes to 250 MHz.
- USB2-only devices no longer warn about “low speed / replug USB3”.
- Stream, repeat, and buffer capture follow official framing; rolling no longer stops at 16 MSa.
- Fixed stalls after a while at high rates (sync USB OUT while async IN is in flight deadlocks WinUSB).
- Do not zero-pad lagging channels (that shifted the timeline).

**nanoDLA / FX2**

- Skip RAM firmware upload when fx2lafw is already running (PulseView behavior).
- Do not claim Cypress boot `04B4:8613` (DSLogic FX2 boot).

**Other**

- Hotplug whitelist includes nanoDLA and ATK-Logic.

See [GitHub Releases](https://github.com/Doukeyi-X/ALL-LOGIC/releases). CH32H417 firmware and first-time steps: [OpenSourceLogic-CH32H417](https://github.com/Doukeyi-X/OpenSourceLogic-CH32H417).

Not affiliated with Alientek, Muse Lab, DreamSourceLab, or any other hardware vendor.
