/*
 * CH32 固件升级 — APP 跳转 + IAP CDC 虚拟串口
 *
 * APP USB2 (1A86:5537) / APP USB3 (1A86:5538): EP1 bulk 0xAE → IAP
 * IAP CDC (1A86:5539): 虚拟串口升级
 *
 * 串口帧（主机→设备）:
 *   AA 55 | Cmd | Len | data[Len] | sumL | sumH | 55 AA
 *   sum = Cmd + Len + sum(data)
 * 设备应答（END 无应答）:
 *   AA 55 | 00 | status | 55 AA     status 0=成功 1=失败
 *
 * Cmd: 0x81 擦除, 0x80 编程, 0x82 校验, 0x83 结束
 */

#include "firmwareupgradedlg.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QFile>
#include <QApplication>
#include <QThread>
#include <QCloseEvent>
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <mmsystem.h>
#include <setupapi.h>
#include <initguid.h>
#include <devguid.h>
#pragma comment(lib, "winmm.lib")
#endif

#include "../ui/msgbox.h"
#include "../log.h"
#include <libsigrok.h>

#define CH32_APP_VID        0x1A86
#define CH32_APP_PID_USB2   0x5537
#define CH32_APP_PID_USB3   0x5538   /* = USB2 PID + 1 */
#define CH32_IAP_VID        0x1A86
#define CH32_IAP_PID        0x5539

#define CMD_SET_IAP         0xAE
#define CMD_IAP_PROM        0x80
#define CMD_IAP_ERASE       0x81
#define CMD_IAP_VERIFY      0x82
#define CMD_IAP_END         0x83

/*
 * 帧: AA 55 Cmd Len data[Len] sumL sumH 55 AA
 * 与 IAP 固件 USBD_DATA_SIZE 对齐：Len 最大 120
 * 流水线深度：连续发 N 包再收 N 个 ACK
 */
/* 实测: pipe>1 末尾 ACK 易丢；稳定优先用 pipe=1 + 120B */
#define IAP_CHUNK           120
#define IAP_PIPE_DEPTH      1
#define HEAD1               0xAA
#define HEAD2               0x55

namespace pv {
namespace dialogs {

#ifdef _WIN32
static QString find_com_port_by_vid_pid(uint16_t vid, uint16_t pid)
{
	HDEVINFO devs = SetupDiGetClassDevsA(&GUID_DEVCLASS_PORTS, NULL, NULL,
					     DIGCF_PRESENT);
	if (devs == INVALID_HANDLE_VALUE)
		return QString();

	char want[64];
	snprintf(want, sizeof(want), "VID_%04X&PID_%04X", vid, pid);

	SP_DEVINFO_DATA info;
	info.cbSize = sizeof(info);
	QString found;

	for (DWORD i = 0; SetupDiEnumDeviceInfo(devs, i, &info); i++) {
		char hw[512] = {0};
		if (!SetupDiGetDeviceRegistryPropertyA(
			    devs, &info, SPDRP_HARDWAREID, NULL,
			    (PBYTE)hw, sizeof(hw), NULL))
			continue;

		QString hws = QString::fromLocal8Bit(hw).toUpper();
		if (!hws.contains(QString::fromLatin1(want).toUpper()))
			continue;

		HKEY key = SetupDiOpenDevRegKey(
			devs, &info, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
		if (key == INVALID_HANDLE_VALUE)
			continue;

		char port[64] = {0};
		DWORD type = 0, sz = sizeof(port);
		if (RegQueryValueExA(key, "PortName", NULL, &type,
				     (LPBYTE)port, &sz) == ERROR_SUCCESS) {
			found = QString::fromLocal8Bit(port);
		}
		RegCloseKey(key);
		if (!found.isEmpty())
			break;
	}

	SetupDiDestroyDeviceInfoList(devs);
	return found;
}

static HANDLE open_com(const QString &portName)
{
	QString path = portName;
	if (!path.startsWith("\\\\.\\"))
		path = "\\\\.\\" + path;

	HANDLE h = CreateFileA(path.toLocal8Bit().constData(),
			       GENERIC_READ | GENERIC_WRITE,
			       0, NULL, OPEN_EXISTING, 0, NULL);
	if (h == INVALID_HANDLE_VALUE)
		return INVALID_HANDLE_VALUE;

	/* 部分 CDC 驱动 GetCommState 会失败，忽略并继续用默认参数 */
	DCB dcb;
	memset(&dcb, 0, sizeof(dcb));
	dcb.DCBlength = sizeof(dcb);
	if (GetCommState(h, &dcb)) {
		dcb.BaudRate = CBR_115200;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		dcb.fBinary = TRUE;
		/* 避免 DTR 翻转导致 CDC 复位 */
		dcb.fDtrControl = DTR_CONTROL_ENABLE;
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
		dcb.fOutxCtsFlow = FALSE;
		dcb.fOutxDsrFlow = FALSE;
		dcb.fOutX = FALSE;
		dcb.fInX = FALSE;
		SetCommState(h, &dcb);
	}

	/*
	 * 非阻塞读：有数据立刻返回，无数据立即返回 0 字节。
	 * 应答轮询在用户态 tight-loop，避免每包卡在 ReadFile 超时上。
	 */
	COMMTIMEOUTS to;
	to.ReadIntervalTimeout = MAXDWORD;
	to.ReadTotalTimeoutMultiplier = 0;
	to.ReadTotalTimeoutConstant = 0;
	to.WriteTotalTimeoutMultiplier = 0;
	to.WriteTotalTimeoutConstant = 500;
	SetCommTimeouts(h, &to);

	/* 放大驱动缓冲，利于流水线 */
	SetupComm(h, 64 * 1024, 64 * 1024);

	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	return h;
}

static bool com_write(HANDLE h, const uint8_t *data, int len)
{
	if (h == INVALID_HANDLE_VALUE || !data || len <= 0)
		return false;
	const uint8_t *p = data;
	int left = len;
	while (left > 0) {
		DWORD wr = 0;
		if (!WriteFile(h, p, (DWORD)left, &wr, NULL))
			return false;
		if (wr == 0)
			return false;
		p += wr;
		left -= (int)wr;
	}
	return true;
}
#else
static QString find_com_port_by_vid_pid(uint16_t, uint16_t) { return QString(); }
#endif

static int open_app_device(libusb_context *ctx, libusb_device_handle **out,
			  QString *errDetail)
{
	libusb_device **list = NULL;
	ssize_t n = libusb_get_device_list(ctx, &list);
	int r = LIBUSB_ERROR_NO_DEVICE;
	*out = NULL;
	if (errDetail)
		errDetail->clear();

	for (ssize_t i = 0; i < n; i++) {
		struct libusb_device_descriptor d;
		if (libusb_get_device_descriptor(list[i], &d) != 0)
			continue;
		if (d.idVendor != CH32_APP_VID ||
		    (d.idProduct != CH32_APP_PID_USB2 &&
		     d.idProduct != CH32_APP_PID_USB3))
			continue;
		r = libusb_open(list[i], out);
		if (r != 0) {
			if (errDetail)
				*errDetail = QString::fromUtf8("libusb_open 失败: %1")
						 .arg(libusb_error_name(r));
			break;
		}
		if (libusb_kernel_driver_active(*out, 0) == 1)
			libusb_detach_kernel_driver(*out, 0);
		r = libusb_claim_interface(*out, 0);
		if (r != 0) {
			if (errDetail)
				*errDetail = QString::fromUtf8(
					"claim 接口失败: %1（设备可能被 DSView 占用，"
					"请先停止采集；程序会尝试自动释放）")
						 .arg(libusb_error_name(r));
			libusb_close(*out);
			*out = NULL;
		}
		break;
	}
	if (r == LIBUSB_ERROR_NO_DEVICE && errDetail && errDetail->isEmpty())
		*errDetail = QString::fromUtf8("总线未发现 APP (1A86:5537/5538)");
	libusb_free_device_list(list, 1);
	return r;
}

/* also detect IAP by USB PID even before COM is ready */
static int count_usb_pid(libusb_context *ctx, uint16_t vid, uint16_t pid)
{
	libusb_device **list = NULL;
	ssize_t n = libusb_get_device_list(ctx, &list);
	int c = 0;
	for (ssize_t i = 0; i < n; i++) {
		struct libusb_device_descriptor d;
		if (libusb_get_device_descriptor(list[i], &d) != 0)
			continue;
		if (d.idVendor == vid && d.idProduct == pid)
			c++;
	}
	libusb_free_device_list(list, 1);
	return c;
}

/* 栈上组帧，避免每包 QByteArray 堆分配。返回帧长，buf 至少 8+len */
static int build_frame(uint8_t *buf, uint8_t cmd, const uint8_t *data, uint8_t len)
{
	buf[0] = HEAD1;
	buf[1] = HEAD2;
	buf[2] = cmd;
	buf[3] = len;
	uint16_t sum = (uint16_t)cmd + (uint16_t)len;
	for (uint8_t i = 0; i < len; i++) {
		buf[4 + i] = data[i];
		sum = (uint16_t)(sum + data[i]);
	}
	buf[4 + len] = (uint8_t)(sum & 0xFF);
	buf[5 + len] = (uint8_t)(sum >> 8);
	buf[6 + len] = HEAD2;
	buf[7 + len] = HEAD1;
	return 8 + len;
}

#ifdef _WIN32
/* 非阻塞读累计，解析应答 AA 55 00 st 55 AA；成功返回 1，超时 0，错误 -1 */
static int wait_reply_nb(HANDLE h, int timeout_ms, uint8_t *acc, int *ptotal)
{
	DWORD start = GetTickCount();
	int total = ptotal ? *ptotal : 0;

	while ((int)(GetTickCount() - start) < timeout_ms) {
		DWORD rd = 0;
		if (total >= 64)
			total = 0;
		if (!ReadFile(h, acc + total, (DWORD)(64 - total), &rd, NULL))
			return -1;
		if (rd > 0) {
			total += (int)rd;
			for (int i = 0; i + 5 < total; i++) {
				if (acc[i] == HEAD1 && acc[i + 1] == HEAD2 &&
				    acc[i + 4] == HEAD2 && acc[i + 5] == HEAD1) {
					int ok = (acc[i + 3] == 0);
					/* 丢掉已消费字节 */
					int used = i + 6;
					if (used < total)
						memmove(acc, acc + used, total - used);
					total -= used;
					if (ptotal)
						*ptotal = total;
					return ok ? 1 : -1;
				}
			}
			if (total > 16) {
				memmove(acc, acc + total - 16, 16);
				total = 16;
			}
		}
		/* 无 Sleep：依赖非阻塞 ReadFile + 忙等，CDC 应答通常 <1ms */
	}
	if (ptotal)
		*ptotal = total;
	return 0;
}

static bool send_cmd(HANDLE h, uint8_t cmd, const uint8_t *data, uint8_t len,
		     bool expect_reply, int timeout_ms = 1000)
{
	uint8_t frame[8 + 256];
	int flen = build_frame(frame, cmd, data, len);
	if (!com_write(h, frame, flen))
		return false;
	if (!expect_reply)
		return true;
	uint8_t acc[64];
	int total = 0;
	return wait_reply_nb(h, timeout_ms, acc, &total) == 1;
}

/*
 * 流水线发送：连续写 depth 帧，再按序收 ACK。
 * chunk_size: 新 IAP 用 240，旧 IAP 仅支持 <=60。
 */
static bool send_stream_prom(HANDLE h, const uint8_t *image, size_t image_size,
			     int chunk_size, int pipe_depth,
			     FirmwareUpgradeWorker *self)
{
	uint8_t frame[8 + 256];
	uint8_t acc[64];
	int acc_total = 0;
	size_t off = 0;
	int last_pct = -1;
	int pending = 0;

	if (chunk_size < 1)
		chunk_size = 60;
	if (chunk_size > 248)
		chunk_size = 248;
	if (pipe_depth < 1)
		pipe_depth = 1;

	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

	while (off < image_size || pending > 0) {
		while (pending < pipe_depth && off < image_size) {
			size_t chunk = image_size - off;
			if (chunk > (size_t)chunk_size)
				chunk = (size_t)chunk_size;
			int flen = build_frame(frame, CMD_IAP_PROM,
					       &image[off], (uint8_t)chunk);
			if (!com_write(h, frame, flen))
				return false;
			off += chunk;
			pending++;
		}

		/* 页擦写时设备会停顿；收尾阶段 pending 可能较久 */
		int wait_ms = (off >= image_size) ? 5000 : 2000;
		int r = wait_reply_nb(h, wait_ms, acc, &acc_total);
		if (r != 1)
			return false;
		pending--;

		int pct = 22 + (int)(off * 70 / image_size);
		if (pct > 92)
			pct = 92;
		if (pct != last_pct && self) {
			last_pct = pct;
			self->progress(pct, QString::fromUtf8(
				"正在编程... %1 / %2 字节")
				.arg(off).arg(image_size));
		}
	}
	return true;
}

/* END 后设备会立刻复位，写/关串口失败属正常 */
static void send_end_and_close(HANDLE *ph)
{
	if (!ph || *ph == INVALID_HANDLE_VALUE)
		return;
	HANDLE h = *ph;
	*ph = INVALID_HANDLE_VALUE;

	uint8_t frame[16];
	int flen = build_frame(frame, CMD_IAP_END, NULL, 0);
	com_write(h, frame, flen);
	Sleep(30);
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	CloseHandle(h);
	Sleep(100);
}
#endif

/* Wait until COM appears; optionally report USB PID presence via progress. */
static QString wait_iap_com_ex(int timeout_ms, libusb_context *ctx,
			       FirmwareUpgradeWorker *self)
{
	int waited = 0;
	bool saw_pid = false;
	while (waited < timeout_ms) {
		if (ctx && count_usb_pid(ctx, CH32_IAP_VID, CH32_IAP_PID) > 0) {
			if (!saw_pid) {
				saw_pid = true;
				if (self)
					self->progress(
						80,
						QString::fromUtf8(
							"已检测到 IAP 设备 (1A86:5539)，等待系统分配 COM..."));
			}
		}
		QString p = find_com_port_by_vid_pid(CH32_IAP_VID, CH32_IAP_PID);
		if (!p.isEmpty())
			return p;
		QThread::msleep(250);
		waited += 250;
	}
	return QString();
}

FirmwareUpgradeWorker::FirmwareUpgradeWorker(Mode mode, const QString &binPath,
					     QObject *parent)
	: QObject(parent), _mode(mode), _binPath(binPath)
{
}

void FirmwareUpgradeWorker::run()
{
	if (_mode == ModeEnterIap)
		runEnterIap();
	else
		runUpgrade();
}

void FirmwareUpgradeWorker::runEnterIap()
{
#ifndef _WIN32
	emit finished(false, QString::fromUtf8("当前仅支持 Windows。"));
	return;
#else
	libusb_context *ctx = NULL;
	libusb_device_handle *dev = NULL;
	uint8_t pkt[64];
	QString err;

	/* 若已在 IAP，直接报告串口 */
	emit progress(10, QString::fromUtf8("检查是否已在 IAP 模式..."));
	{
		QString already = find_com_port_by_vid_pid(CH32_IAP_VID, CH32_IAP_PID);
		if (!already.isEmpty()) {
			emit progress(100, QString::fromUtf8("设备已在 IAP 模式。"));
			emit finished(true, QString::fromUtf8(
				"设备已处于 IAP 模式，串口：%1。\n"
				"可直接选择固件并开始升级。").arg(already));
			return;
		}
	}

	if (libusb_init(&ctx) != 0) {
		emit finished(false, QString::fromUtf8("libusb 初始化失败。"));
		return;
	}

	emit progress(20, QString::fromUtf8("正在查找 APP 设备 (1A86:5537/5538)..."));
	if (open_app_device(ctx, &dev, &err) != 0 || dev == NULL) {
		/* retry once after short delay (caller should have released DSView handle) */
		QThread::msleep(300);
		err.clear();
		if (open_app_device(ctx, &dev, &err) != 0 || dev == NULL) {
			libusb_exit(ctx);
			emit finished(false, QString::fromUtf8(
				"无法打开 APP 设备 (1A86:5537/5538)。\n%1\n"
				"建议：关闭占用该设备的程序，确认 WinUSB 驱动，"
				"或运行 tools/test_enter_iap.exe 诊断。")
					 .arg(err));
			return;
		}
	}

	emit progress(50, QString::fromUtf8("正在发送进入 IAP 命令 (0xAE)..."));
	memset(pkt, 0, sizeof(pkt));
	pkt[0] = CMD_SET_IAP;
	pkt[1] = 0;
	{
		int xfer = 0;
		int r = libusb_bulk_transfer(dev, 0x01, pkt, 2, &xfer, 3000);
		if (r != 0) {
			/* SuperSpeed APP 有时需要凑满包 */
			memset(pkt, 0, sizeof(pkt));
			pkt[0] = CMD_SET_IAP;
			pkt[1] = 0;
			xfer = 0;
			r = libusb_bulk_transfer(dev, 0x01, pkt, 32, &xfer, 3000);
		}
		libusb_release_interface(dev, 0);
		libusb_close(dev);
		dev = NULL;
		if (r != 0) {
			libusb_exit(ctx);
			emit finished(false, QString::fromUtf8(
				"发送 0xAE 失败：%1").arg(libusb_error_name(r)));
			return;
		}
	}

	/* 实测约 2s 后出现 5539；先等再扫 COM */
	emit progress(70, QString::fromUtf8("已发送 0xAE，等待设备复位（约 2 秒）..."));
	QThread::msleep(2000);
	QString port = wait_iap_com_ex(18000, ctx, this);
	libusb_exit(ctx);
	ctx = NULL;

	if (port.isEmpty()) {
		emit finished(false, QString::fromUtf8(
			"已发送进入 IAP 命令，但未分配 CDC 串口 (1A86:5539)。\n"
			"请打开设备管理器查看是否有 1A86:5539 或未知设备。\n"
			"可运行 tools\\test_enter_iap.exe 做完整诊断。"));
		return;
	}

	emit progress(100, QString::fromUtf8("已进入 IAP 模式。"));
	emit finished(true, QString::fromUtf8(
		"已成功进入 IAP 模式，串口：%1。\n"
		"请选择固件文件后点击「开始升级」。").arg(port));
#endif
}

void FirmwareUpgradeWorker::runUpgrade()
{
#ifndef _WIN32
	emit finished(false, QString::fromUtf8("当前仅支持 Windows 下的 CDC 串口升级。"));
	return;
#else
	libusb_context *ctx = NULL;
	libusb_device_handle *dev = NULL;
	QFile f(_binPath);
	std::vector<uint8_t> image;
	HANDLE com = INVALID_HANDLE_VALUE;
	uint8_t pkt[64];
	size_t off;

	emit progress(0, QString::fromUtf8("正在读取固件文件..."));

	if (!f.open(QIODevice::ReadOnly)) {
		emit finished(false, QString::fromUtf8("无法打开文件：%1").arg(_binPath));
		return;
	}
	QByteArray ba = f.readAll();
	f.close();
	if (ba.size() < 256) {
		emit finished(false, QString::fromUtf8("固件文件过小，请确认是否为有效的 APP .bin。"));
		return;
	}
	image.assign((const uint8_t *)ba.constData(),
		     (const uint8_t *)ba.constData() + ba.size());

	/* 1) 若 APP 在线则跳转 IAP */
	if (libusb_init(&ctx) == 0) {
		emit progress(3, QString::fromUtf8("正在查找 APP 设备 (1A86:5537/5538)..."));
		QString oerr;
		if (open_app_device(ctx, &dev, &oerr) == 0 && dev) {
			emit progress(6, QString::fromUtf8("正在发送进入 IAP 命令 (0xAE)..."));
			memset(pkt, 0, sizeof(pkt));
			pkt[0] = CMD_SET_IAP;
			pkt[1] = 0;
			int xfer = 0;
			int r = libusb_bulk_transfer(dev, 0x01, pkt, 2, &xfer, 3000);
			if (r != 0) {
				memset(pkt, 0, sizeof(pkt));
				pkt[0] = CMD_SET_IAP;
				pkt[1] = 0;
				xfer = 0;
				libusb_bulk_transfer(dev, 0x01, pkt, 32, &xfer, 3000);
			}
			libusb_release_interface(dev, 0);
			libusb_close(dev);
			dev = NULL;
			emit progress(10, QString::fromUtf8("等待设备进入 IAP 并重新枚举 CDC..."));
			QThread::msleep(2000);
		} else {
			emit progress(10, QString::fromUtf8("未找到 APP，尝试直接连接 IAP 串口..."));
		}
		/* keep ctx for wait_iap_com_ex */
	}

	/* 2) 打开 CDC COM */
	emit progress(12, QString::fromUtf8("正在查找 IAP 串口 (1A86:5539)..."));
	QString port = wait_iap_com_ex(18000, ctx, this);
	if (ctx) {
		libusb_exit(ctx);
		ctx = NULL;
	}
	if (port.isEmpty()) {
		emit finished(false, QString::fromUtf8(
			"未找到 IAP CDC 串口 (VID 1A86 / PID 5539)。\n"
			"可先点「进入 IAP」，或确认 IAP 固件已烧录。"));
		return;
	}

	emit progress(15, QString::fromUtf8("正在打开串口 %1 ...").arg(port));
	com = open_com(port);
	if (com == INVALID_HANDLE_VALUE) {
		emit finished(false, QString::fromUtf8("无法打开串口 %1，请确认未被其他程序占用。").arg(port));
		return;
	}

	/* 提高系统定时器精度，缩短忙等粒度 */
	timeBeginPeriod(1);

	/* 3) 擦除（仅此步可能较慢） */
	emit progress(18, QString::fromUtf8("正在擦除 Flash..."));
	if (!send_cmd(com, CMD_IAP_ERASE, NULL, 0, true, 5000)) {
		timeEndPeriod(1);
		CloseHandle(com);
		emit finished(false, QString::fromUtf8("擦除失败：串口无应答（%1）。").arg(port));
		return;
	}

	/* 4) 编程：120B 一问一答（自测 ~0.7s / 130KB）；失败再回退 60B */
	emit progress(22, QString::fromUtf8("正在编程..."));
	bool ok = send_stream_prom(com, image.data(), image.size(),
				   IAP_CHUNK, IAP_PIPE_DEPTH, this);
	if (!ok) {
		emit progress(22, QString::fromUtf8(
			"120 字节模式失败，回退 60 字节..."));
		PurgeComm(com, PURGE_RXCLEAR | PURGE_TXCLEAR);
		if (!send_cmd(com, CMD_IAP_ERASE, NULL, 0, true, 5000)) {
			timeEndPeriod(1);
			CloseHandle(com);
			emit finished(false, QString::fromUtf8("兼容模式重新擦除失败。"));
			return;
		}
		ok = send_stream_prom(com, image.data(), image.size(),
				      60, 1, this);
	}
	if (!ok) {
		timeEndPeriod(1);
		CloseHandle(com);
		emit finished(false, QString::fromUtf8(
			"编程失败。可运行 tools\\test_fw_upgrade.exe 诊断。"));
		return;
	}

	/* 5) 结束：跳过 VERIFY 整片回读（可省约一半时间） */
	emit progress(97, QString::fromUtf8("正在结束并重启设备..."));
	send_end_and_close(&com);
	timeEndPeriod(1);

	emit progress(100, QString::fromUtf8("升级完成，设备将重启进入 APP。"));
	emit finished(true, QString::fromUtf8("固件升级成功（串口 %1，已跳过校验）。").arg(port));
	(void)off;
#endif
}

FirmwareUpgradeDlg::FirmwareUpgradeDlg(QWidget *parent)
	: DSDialog(parent, true /* hasClose */),
	  _thread(NULL), _worker(NULL), _busy(false)
{
	setTitle(QString::fromUtf8("固件升级"));
	setMinimumWidth(520);

	_pathEdit = new QLineEdit(this);
	_browseBtn = new QPushButton(QString::fromUtf8("浏览..."), this);
	_enterIapBtn = new QPushButton(QString::fromUtf8("进入 IAP"), this);
	_startBtn = new QPushButton(QString::fromUtf8("开始升级"), this);
	_closeBtn = new QPushButton(QString::fromUtf8("关闭"), this);
	_progress = new QProgressBar(this);
	_progress->setRange(0, 100);
	_progress->setValue(0);
	_status = new QLabel(
		QString::fromUtf8(
			"1. 可先点「进入 IAP」切换到 IAP 串口 (1A86:5539)\n"
			"2. 选择 APP 固件 .bin\n"
			"3. 点「开始升级」（高速：大包 + 流水线，跳过校验）\n"
			"提示：APP USB2=5537 / USB3=5538，IAP=5539"),
		this);
	_status->setWordWrap(true);

	QHBoxLayout *row = new QHBoxLayout();
	row->addWidget(_pathEdit, 1);
	row->addWidget(_browseBtn);

	QHBoxLayout *btnRow = new QHBoxLayout();
	btnRow->addWidget(_enterIapBtn);
	btnRow->addStretch(1);
	btnRow->addWidget(_startBtn);
	btnRow->addWidget(_closeBtn);

	QVBoxLayout *lay = new QVBoxLayout();
	lay->addLayout(row);
	lay->addWidget(_progress);
	lay->addWidget(_status);
	lay->addLayout(btnRow);
	layout()->addLayout(lay);

	connect(_browseBtn, SIGNAL(clicked()), this, SLOT(onBrowse()));
	connect(_enterIapBtn, SIGNAL(clicked()), this, SLOT(onEnterIap()));
	connect(_startBtn, SIGNAL(clicked()), this, SLOT(onStart()));
	connect(_closeBtn, SIGNAL(clicked()), this, SLOT(onClose()));
}

FirmwareUpgradeDlg::~FirmwareUpgradeDlg()
{
	if (_thread) {
		_thread->quit();
		_thread->wait(3000);
		delete _thread;
		_thread = NULL;
	}
}

bool FirmwareUpgradeDlg::isBusy() const
{
	return _busy;
}

void FirmwareUpgradeDlg::closeEvent(QCloseEvent *event)
{
	if (_busy) {
		MsgBox::Show(QString::fromUtf8("提示"),
			     QString::fromUtf8("正在升级中，请等待完成后再关闭。"),
			     this);
		event->ignore();
		return;
	}
	/* 允许标题栏关闭按钮 / Alt+F4 正常关闭 */
	event->accept();
	QDialog::closeEvent(event);
}

void FirmwareUpgradeDlg::onClose()
{
	if (_busy) {
		MsgBox::Show(QString::fromUtf8("提示"),
			     QString::fromUtf8("正在升级中，请等待完成后再关闭。"),
			     this);
		return;
	}
	reject();
}

void FirmwareUpgradeDlg::onBrowse()
{
	QString path = QFileDialog::getOpenFileName(
		this,
		QString::fromUtf8("选择固件文件"),
		QString(),
		QString::fromUtf8("二进制固件 (*.bin);;所有文件 (*.*)"));
	if (!path.isEmpty())
		_pathEdit->setText(path);
}

void FirmwareUpgradeDlg::setBusyUi(bool busy)
{
	_busy = busy;
	_startBtn->setEnabled(!busy);
	_browseBtn->setEnabled(!busy);
	_enterIapBtn->setEnabled(!busy);
	/* 升级中禁止关闭，避免中断 */
	_closeBtn->setEnabled(!busy);
}

void FirmwareUpgradeDlg::startWorker(FirmwareUpgradeWorker::Mode mode,
				     const QString &binPath)
{
	if (_thread || _busy) {
		MsgBox::Show(QString::fromUtf8("提示"),
			     QString::fromUtf8("操作正在进行中，请稍候。"),
			     this);
		return;
	}

	/* 释放 DSView 对 APP 的 USB claim，否则无法发送 0xAE */
	ds_close_actived_device();
	QThread::msleep(200);

	setBusyUi(true);
	_progress->setValue(0);
	_status->setText(mode == FirmwareUpgradeWorker::ModeEnterIap
			 ? QString::fromUtf8("正在进入 IAP...")
			 : QString::fromUtf8("正在启动升级..."));

	_thread = new QThread(this);
	_worker = new FirmwareUpgradeWorker(mode, binPath);
	_worker->moveToThread(_thread);
	connect(_thread, SIGNAL(started()), _worker, SLOT(run()));
	connect(_worker, SIGNAL(progress(int, QString)),
		this, SLOT(onProgress(int, QString)));
	connect(_worker, SIGNAL(finished(bool, QString)),
		this, SLOT(onFinished(bool, QString)));
	connect(_worker, SIGNAL(finished(bool, QString)),
		_thread, SLOT(quit()));
	connect(_thread, SIGNAL(finished()), _worker, SLOT(deleteLater()));
	_thread->start();
}

void FirmwareUpgradeDlg::onEnterIap()
{
	startWorker(FirmwareUpgradeWorker::ModeEnterIap, QString());
}

void FirmwareUpgradeDlg::onStart()
{
	QString path = _pathEdit->text().trimmed();
	if (path.isEmpty() || !QFile::exists(path)) {
		MsgBox::Show(QString::fromUtf8("提示"),
			     QString::fromUtf8("请先选择有效的 .bin 固件文件。"),
			     this);
		return;
	}
	startWorker(FirmwareUpgradeWorker::ModeUpgrade, path);
}

void FirmwareUpgradeDlg::onProgress(int percent, QString msg)
{
	_progress->setValue(percent);
	_status->setText(msg);
}

void FirmwareUpgradeDlg::onFinished(bool ok, QString msg)
{
	_status->setText(msg);
	setBusyUi(false);

	if (_thread) {
		_thread->wait(1000);
		_thread->deleteLater();
		_thread = NULL;
		_worker = NULL;
	}

	/* 成功只更新状态栏；失败再弹框 */
	if (!ok)
		MsgBox::Show(QString::fromUtf8("操作失败"), msg, this);
}

} // namespace dialogs
} // namespace pv
