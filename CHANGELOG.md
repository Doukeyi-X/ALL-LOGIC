# Changelog

ALL LOGIC 二次开发记录。上游 DSView 历史见 `DSView/NEWS`。

---

## Unreleased

### 新增设备支持

- **Sipeed SLogic Combo 8 / SLogic32U3** — USB `359F:0300`、`359F:3032`；按 `slogic-dev` 协议支持型号对应的通道、采样率、端点和数据打包。同族的 SLogic16U3（`359F:3031`）一并纳入这个统一型号表。

### SLogic32U3

- 修复 32 通道 @200MHz 采集时 USB 吞吐跟不上、波形约 3 秒才走 1 秒，以及停止 / 再次开始卡死的问题：USB 传输管线稳定在约 790 MB/s，200 MSa（1 秒深度）一整轮约 1.2 秒完成。
- 主机侧位转置改为多线程（默认取一半核心，上限 8），并在传输已按 64 样本组对齐时直接从 USB 缓冲区转换，省掉每包一次的全量拷贝。转置结果与改前逐字节一致（同一图案导出 CSV 校验）。
- 修复采集期间 MCP 请求超时（客户端提前断开）导致的空指针崩溃：连接在请求处理中退出时延迟回收，避免写入已析构的 socket。
- `configure` 支持 `pattern`（Normal / USB connection test / Emulation），`get_status` 会回报当前图案。
- 采集线程不再空转：会话的 "freewheel" 循环（无 poll fd 时驱动只靠它取队列）以前不带任何等待，采集期间会烧满一个核并拖慢界面刷新；现在按驱动请求的 1 ms 间隔补睡空闲时间，回调本身忙碌时不补睡。界面线程响应从约 200 ms 降到平均约 5 ms。
- 修复设备没有开始出数据时永久挂死：吞吐/停滞判断原先只在收到数据（`len > 0`）时生效，参考驱动则对每个传输都判断，因此固件漏掉 RUN 命令后本机会静默重提交 URB、永远停在「采集中」，只能手动停止。现在与参考实现一致，连续多次无数据会中止采集。
- 该情形还会自动补救一次：若一条传输都没收到数据，驱动在采集线程里重发一次 RUN（不在 USB 回调里发控制请求，避免 U3 固件返回 BUSY）。实测 3 次真实失败里这次重发都未能救回，但采集会在 0.2 秒内干净结束，紧接着再按 Start 即可正常满速采集——所以它是兜底尝试，不是保证。
- 采集启动路径不做引擎复位：`acq start` 只写 `CTRL=STOP`/`RUN` 以及通道掩码、采样率、vref、图案四个配置块，**不会**写 `CTRL=RST`——复位会连同用户设定的图案与 vref 一起清掉。全驱动只有两处复位：打开设备（选中设备）时，以及把图案切回 Normal 时（有意清掉测试图案）。`-l4` 现在会把每次寄存器读写打进日志（`ctrl wr addr=0x0004 data=...`），可逐条核对启动序列；`tools/test_config_persistence.py` 会只配置一次图案 + 非默认 vref，随后连续 Start/Stop 多轮，每轮导出 CSV 校验图案仍在（14 个静默通道、整数倍翻转结构）。
- 长采集不再被误判为 stall：主机侧转置 + 写快照约 670 MB/s，低于设备的 790 MB/s，采到 3 秒左右设备端会出现反压、传输变慢。参考实现把「传输比预期慢」直接判为致命，本驱动照搬后 4 秒采集会在 621 MSa 提前结束（同一请求 sigrok-cli 能跑完，代价是 7.44 s）。现在区分两种情况：**一个字节都没来过**（固件吞掉 RUN）沿用原来的连续判定 + 重发一次 RUN，保证不会出现「永远卡在采集中」；**数据已在流动**则只有整整 1 秒完全没有字节才算真死，慢只是按主机节奏排空（`-l4` 有 `USB link slower than the analyzer` 提示）。实测 1 s / 2 s / 4 s @200MHz/32ch 分别 100% / 100% / 100% 满额（4 s 用时 4.85 s，参考 7.44 s）。
- 每次采集结束会打印一行实测吞吐：`stream done: 800.0 MB in 1.009 s (793 MB/s), 200.000 MSa, host tail 204 ms`，把「设备是否满速」「主机转置落后多少」变成一眼可见的数字。
- 与官方 `sigrok-cli`（1.1.0 AppImage）对照：同样 32ch@200MHz 1s 背靠背采集，参考实现 30 次里 15 次只写出 12 字节的 `FRAME-BEGIN`（0 个样本），加 1 秒 / 5 秒间隔仍各只有 6/12 成功；本驱动同样条件 48/48 满额（另一轮 47/48），1 s 数据约 1.27–1.35 s 真实时间完成。

### 其它

- `cmake --install --prefix <dir>` 现在能装全：安装规则改用相对路径，不再因为写入 `/usr` 而中断，`share/DSView` 与解码器都会落到指定前缀。

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

### Unreleased

**Added**

- Sipeed SLogic Combo 8 / SLogic32U3, USB `359F:0300` and `359F:3032`, using the model-specific `slogic-dev` endpoints, rate tables, and packed sample formats. The related SLogic16U3 (`359F:3031`) now shares the same model table.

**SLogic32U3**

- 32ch @200MHz capture no longer falls behind the analyzer: the USB pipeline sustains about 790 MB/s, so a 200 MSa (1 s) buffer completes in roughly 1.2 s, and stop / restart stay responsive.
- Host-side wire-format transpose is now multithreaded (half the cores, capped at 8) and converts straight out of the USB buffer when a transfer is already 64-sample aligned, removing a full 800 MB copy per capture. Output is byte-identical to the previous implementation (verified by exporting CSV for the same device pattern).
- Fixed a use-after-free that crashed the app when an MCP request timed out mid-capture: sockets that disconnect while handling a request are now released after the handler returns.
- `configure` accepts `pattern` (Normal / USB connection test / Emulation) and `get_status` reports it.
- The collect thread no longer spins: the session "freewheel" loop (the only way a driver without a poll fd gets drained) ran with no wait at all, burning a full core for the whole capture and starving the UI. It is now paced by the interval the driver asked for, and only pads idle time, so a busy callback still runs unthrottled. UI response time during a 32ch@200MHz capture dropped from roughly 200 ms to about 5 ms on average.
- Fixed a permanent hang when the analyzer never starts streaming: the throughput/stall guard only ran when a transfer carried data (`len > 0`), while the reference driver evaluates it for every transfer. A firmware that dropped the RUN command therefore sat in "capturing" forever, resubmitting empty URBs, with no way out but a manual stop. The guard now matches the reference and aborts after repeated empty transfers.
- That case now gets one best-effort retry: when not a single byte arrived, the driver re-issues RUN once from the collect thread (not from the USB callback, where U3 firmware answers BUSY). In the three real failures observed the retry did not rescue the start, but the capture ends cleanly within 0.2 s and the following Start streams at full speed, so it is a fallback rather than a guarantee.
- Capture start never resets the engine: `acq start` writes only `CTRL=STOP`/`RUN` plus the channel mask, samplerate, vref and pattern configuration blocks, and never `CTRL=RST` - a reset would take the user's pattern generator and vref settings with it. The driver resets in exactly two places: when the device is opened (selected), and when the pattern is switched back to Normal (deliberately clearing the test pattern). `-l4` now traces every register access (`ctrl wr addr=0x0004 data=...`) so the start sequence can be audited line by line, and `tools/test_config_persistence.py` configures pattern + a non-default vref once, then runs several Start/Stop cycles and re-checks the pattern in each exported CSV (14 quiet channels, integer-ratio transitions).
- Long captures are no longer misread as stalls: the host converts and appends at roughly 670 MB/s while the analyzer produces 790 MB/s, so after about three seconds the device throttles itself and every transfer looks slow. The reference treats a slow transfer as fatal, and copying that aborted a 4 s capture at 621 MSa (sigrok-cli finishes the same request, taking 7.44 s to do it). The driver now separates the two cases: while **not a single byte has arrived** (firmware swallowed RUN) the original consecutive-failure rule plus the one-shot RUN retry still guarantee the capture cannot hang in "capturing"; once **data is flowing**, only a full second of complete silence is fatal and a slow link is simply drained at the host's pace (`-l4` prints `USB link slower than the analyzer`). Measured: 1 s / 2 s / 4 s @200MHz/32ch all complete at 100%, with the 4 s run taking 4.85 s against the reference's 7.44 s.
- Every capture now logs its measured throughput, e.g. `stream done: 800.0 MB in 1.009 s (793 MB/s), 200.000 MSa, host tail 204 ms`, so "did the device run at full speed" and "how far behind is the host transpose" are single visible numbers.
- Compared against the official `sigrok-cli` (1.1.0 AppImage) under the same 32ch@200MHz 1 s back-to-back load: the reference produced only the 12-byte `FRAME-BEGIN` file (0 samples) in 15 of 30 runs, and still only 6 of 12 runs with a 1 s or 5 s gap in between; this driver completed 48 of 48 (and 47 of 48 in a second run), with 1 s of samples taking 1.27-1.35 s of wall clock.

**Other**

- `cmake --install --prefix <dir>` no longer aborts partway through: install rules use relative paths, so `share/DSView`, decoders and everything else land under the requested prefix.

---

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
