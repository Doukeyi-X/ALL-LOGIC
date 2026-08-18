/*
 * PXLogic USB logic analyzer — DSView / libsigrok4DSL
 *
 * Wire protocol (PXView / CH569W + FPGA):
 *   VID/PID  1A86:5237 (old) or 16C0:05DC (new)
 *   EP 0x01/0x81  16-byte register mailbox  magic 0xfefe0000/0001
 *   EP 0x03       firmware / FPGA bitstream (4KiB aligned)
 *   EP 0x82       sample stream (LA_CROSS_DATA)
 *   EP0 vendor IN 0xB0  trigger status (ctl_data)
 */

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "pxlogic.h"

#undef LOG_PREFIX
#define LOG_PREFIX "pxlogic: "

static const uint64_t samplerates[] = {
	SR_KHZ(2), SR_KHZ(5), SR_KHZ(10), SR_KHZ(20), SR_KHZ(40),
	SR_KHZ(50), SR_KHZ(100), SR_KHZ(200), SR_KHZ(400), SR_KHZ(500),
	SR_MHZ(1), SR_MHZ(2), SR_MHZ(4), SR_MHZ(5), SR_MHZ(10),
	SR_MHZ(20), SR_MHZ(25), SR_MHZ(50), SR_MHZ(100), SR_MHZ(125),
	SR_MHZ(200), SR_MHZ(250), SR_MHZ(400), SR_MHZ(500), SR_MHZ(800),
	SR_GHZ(1),
};

static const struct PX_channels channel_modes[] = {
	{ BUFFER_LOGIC250x32, LOGIC, SR_CHANNEL_LOGIC, 0, 32, SR_MHZ(250), SR_KHZ(2), SR_MHZ(250),
	  "Use 32 Channels (Max 250MHz)" },
	{ BUFFER_LOGIC250x16, LOGIC, SR_CHANNEL_LOGIC, 0, 16, SR_MHZ(250), SR_KHZ(2), SR_MHZ(250),
	  "Use 16 Channels (Max 250MHz)" },
	{ BUFFER_LOGIC500x16, LOGIC, SR_CHANNEL_LOGIC, 0, 16, SR_MHZ(500), SR_KHZ(2), SR_MHZ(500),
	  "Use 16 Channels (Max 500MHz)" },
	{ BUFFER_LOGIC1000x8, LOGIC, SR_CHANNEL_LOGIC, 0, 8, SR_GHZ(1), SR_KHZ(2), SR_GHZ(1),
	  "Use 8 Channels (Max 1000MHz)" },
	{ STREAM_LOGIC50x32, LOGIC, SR_CHANNEL_LOGIC, 1, 32, SR_MHZ(50), SR_KHZ(2), SR_MHZ(50),
	  "Use 32 Channels (Max50MHz)" },
	{ STREAM_LOGIC125x16, LOGIC, SR_CHANNEL_LOGIC, 1, 16, SR_MHZ(125), SR_KHZ(2), SR_MHZ(125),
	  "Use 16 Channels (Max 125MHz)" },
	{ STREAM_LOGIC250x8, LOGIC, SR_CHANNEL_LOGIC, 1, 8, SR_MHZ(250), SR_KHZ(2), SR_MHZ(250),
	  "Use 8 Channels (Max 250MHz)" },
	{ STREAM_LOGIC500x4, LOGIC, SR_CHANNEL_LOGIC, 1, 4, SR_MHZ(500), SR_KHZ(2), SR_MHZ(500),
	  "Use 4 Channels (Max 500MHz)" },
	{ STREAM_LOGIC1000x2, LOGIC, SR_CHANNEL_LOGIC, 1, 2, SR_MHZ(1000), SR_KHZ(2), SR_MHZ(1000),
	  "Use 2 Channels (Max 1000MHz)" },
	{ STREAM_LOGIC200x1, LOGIC, SR_CHANNEL_LOGIC, 1, 1, SR_MHZ(200), SR_KHZ(2), SR_MHZ(200),
	  "Use 1 Channels (Max200MHz)" },
	{ STREAM_LOGIC100x2, LOGIC, SR_CHANNEL_LOGIC, 1, 2, SR_MHZ(100), SR_KHZ(2), SR_MHZ(100),
	  "Use 2 Channels (Max100MHz)" },
	{ STREAM_LOGIC50x4, LOGIC, SR_CHANNEL_LOGIC, 1, 4, SR_MHZ(50), SR_KHZ(2), SR_MHZ(50),
	  "Use 4 Channels (Max50MHz)" },
	{ STREAM_LOGIC25x8, LOGIC, SR_CHANNEL_LOGIC, 1, 8, SR_MHZ(25), SR_KHZ(2), SR_MHZ(25),
	  "Use 8 Channels (Max25MHz)" },
	{ STREAM_LOGIC10x16, LOGIC, SR_CHANNEL_LOGIC, 1, 16, SR_MHZ(10), SR_KHZ(2), SR_MHZ(10),
	  "Use 16 Channels (Max10MHz)" },
	{ STREAM_LOGIC5x32, LOGIC, SR_CHANNEL_LOGIC, 1, 32, SR_MHZ(5), SR_KHZ(2), SR_MHZ(5),
	  "Use 32 Channels (Max5MHz)" },
};

#define CAP_BUF32   (1u << BUFFER_LOGIC250x32)
#define CAP_BUF16   ((1u << BUFFER_LOGIC250x16) | (1u << BUFFER_LOGIC500x16) | (1u << BUFFER_LOGIC1000x8))
#define CAP_STR_U3  ((1u << STREAM_LOGIC50x32) | (1u << STREAM_LOGIC125x16) | \
		     (1u << STREAM_LOGIC250x8) | (1u << STREAM_LOGIC500x4) | \
		     (1u << STREAM_LOGIC1000x2))
#define CAP_STR_U2  ((1u << STREAM_LOGIC200x1) | (1u << STREAM_LOGIC100x2) | \
		     (1u << STREAM_LOGIC50x4) | (1u << STREAM_LOGIC25x8) | \
		     (1u << STREAM_LOGIC10x16) | (1u << STREAM_LOGIC5x32))

static const struct PX_profile supported_PX[] = {
	{ 0x1A86, 0x5237, LIBUSB_SPEED_SUPER, 0, "PX_Tool", "PX-Logic U3 channel 32",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    CAP_BUF32 | (1u << BUFFER_LOGIC500x16) | (1u << BUFFER_LOGIC1000x8) | CAP_STR_U3,
	    SR_Gn(4), 0, BUFFER_LOGIC250x32, SR_NS(500) } },
	{ 0x1A86, 0x5237, LIBUSB_SPEED_HIGH, 0, "PX_Tool", "PX-Logic U2 channel 32",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    CAP_BUF32 | (1u << BUFFER_LOGIC500x16) | (1u << BUFFER_LOGIC1000x8) | CAP_STR_U2,
	    SR_Gn(4), 0, BUFFER_LOGIC500x16, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_SUPER, 0, "PX_Tool", "PX-Logic U3 channel 32",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    CAP_BUF32 | (1u << BUFFER_LOGIC500x16) | (1u << BUFFER_LOGIC1000x8) | CAP_STR_U3,
	    SR_Gn(4), 0, BUFFER_LOGIC250x32, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_HIGH, 0, "PX_Tool", "PX-Logic U2 channel 32",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    CAP_BUF32 | (1u << BUFFER_LOGIC500x16) | (1u << BUFFER_LOGIC1000x8) | CAP_STR_U2,
	    SR_Gn(4), 0, BUFFER_LOGIC500x16, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_SUPER, 1, "PX_Tool", "PX-Logic U3 channel 16 Pro",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    (1u << BUFFER_LOGIC500x16) | (1u << BUFFER_LOGIC1000x8) |
	    (1u << STREAM_LOGIC125x16) | (1u << STREAM_LOGIC250x8) |
	    (1u << STREAM_LOGIC500x4) | (1u << STREAM_LOGIC1000x2),
	    SR_Gn(4), 0, BUFFER_LOGIC500x16, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_HIGH, 1, "PX_Tool", "PX-Logic U2 channel 16 Pro",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    (1u << BUFFER_LOGIC500x16) | (1u << BUFFER_LOGIC1000x8) | CAP_STR_U2,
	    SR_Gn(4), 0, BUFFER_LOGIC500x16, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_SUPER, 2, "PX_Tool", "PX-Logic U3 channel 16 Plus",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    (1u << BUFFER_LOGIC500x16) | (1u << STREAM_LOGIC125x16) |
	    (1u << STREAM_LOGIC250x8) | (1u << STREAM_LOGIC500x4),
	    SR_Gn(2), 0, BUFFER_LOGIC500x16, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_HIGH, 2, "PX_Tool", "PX-Logic U2 channel 16 Plus",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    (1u << BUFFER_LOGIC500x16) | CAP_STR_U2,
	    SR_Gn(2), 0, BUFFER_LOGIC500x16, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_SUPER, 3, "PX_Tool", "PX-Logic U3 channel 16 Base",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    (1u << BUFFER_LOGIC250x16) | (1u << STREAM_LOGIC125x16) | (1u << STREAM_LOGIC250x8),
	    SR_Gn(1), 0, BUFFER_LOGIC250x16, SR_NS(500) } },
	{ 0x16C0, 0x05DC, LIBUSB_SPEED_HIGH, 3, "PX_Tool", "PX-Logic U2 channel 16 Base",
	  "SCI_LOGIC.bin", PXLOGIC_FIRMWARE_VER, "hspi_ddr.bin", "hspi_ddr_RST.bin",
	  { CAPS_MODE_LOGIC, CAPS_FEATURE_USB30 | CAPS_FEATURE_BUF,
	    (1u << BUFFER_LOGIC250x16) | CAP_STR_U2,
	    SR_Gn(1), 0, BUFFER_LOGIC250x16, SR_NS(500) } },
	{ 0, 0, 0, 0, NULL, NULL, NULL, 0, NULL, NULL, { 0, 0, 0, 0, 0, 0, 0 } },
};

static const struct sr_list_item filter_list[] = {
	{ 0, "None" },
	{ 1, "1 Sample Clock" },
	{ -1, NULL },
};

static const struct sr_list_item opmode_list[] = {
	{ LO_OP_BUFFER, "Buffer Mode" },
	{ LO_OP_STREAM, "Stream Mode" },
	{ -1, NULL },
};

static struct sr_list_item channel_mode_ui[PX_CHMODE_COUNT + 1];

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_FILTER,
	SR_CONF_VTH,
	SR_CONF_MAX_HEIGHT,
	SR_CONF_CLOCK_EDGE,
};

static const int32_t sessions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_CHANNEL_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
	SR_CONF_VTH,
	SR_CONF_FILTER,
	SR_CONF_CLOCK_EDGE,
	SR_CONF_MAX_HEIGHT,
};

static const char *probe_names[] = {
	"0", "1", "2", "3", "4", "5", "6", "7",
	"8", "9", "10", "11", "12", "13", "14", "15",
	"16", "17", "18", "19", "20", "21", "22", "23",
	"24", "25", "26", "27", "28", "29", "30", "31",
	NULL,
};

SR_PRIV struct sr_dev_driver pxlogic_driver_info;
static struct sr_dev_driver *di = &pxlogic_driver_info;

static void receive_transfer(struct libusb_transfer *transfer);
static void finish_acquisition(struct PX_context *devc);

static int ch_mode_allowed(const struct PX_context *devc, int id)
{
	if (!devc || !devc->profile)
		return 0;
	if (id < 0 || id >= PX_CHMODE_COUNT)
		return 0;
	if (!(devc->profile->dev_caps.channels & (1ull << id)))
		return 0;
	if (devc->op_mode == LO_OP_STREAM)
		return channel_modes[id].stream ? 1 : 0;
	return channel_modes[id].stream ? 0 : 1;
}

static void rebuild_channel_mode_ui(const struct PX_context *devc)
{
	int i, n = 0;

	for (i = 0; i < PX_CHMODE_COUNT; i++) {
		if (!ch_mode_allowed(devc, i))
			continue;
		channel_mode_ui[n].id = i;
		channel_mode_ui[n].name = channel_modes[i].descr;
		n++;
	}
	channel_mode_ui[n].id = -1;
	channel_mode_ui[n].name = NULL;
}

static void adjust_samplerate(struct PX_context *devc)
{
	int max_i = (int)ARRAY_SIZE(samplerates) - 1;
	int min_i = 0;

	while (max_i > 0 && samplerates[max_i] > channel_modes[devc->ch_mode].max_samplerate)
		max_i--;
	while (min_i < max_i && samplerates[min_i] < channel_modes[devc->ch_mode].min_samplerate)
		min_i++;
	devc->samplerates_max_index = (uint16_t)max_i;
	devc->samplerates_min_index = (uint16_t)min_i;
	if (devc->cur_samplerate > samplerates[max_i])
		devc->cur_samplerate = samplerates[max_i];
	if (devc->cur_samplerate < samplerates[min_i])
		devc->cur_samplerate = samplerates[min_i];
}

static int setup_probes(struct sr_dev_inst *sdi, int num_probes)
{
	int j;
	struct sr_channel *probe;

	sr_dev_probes_free(sdi);
	sdi->channels = NULL;
	for (j = 0; j < num_probes; j++) {
		probe = sr_channel_new((uint16_t)j, SR_CHANNEL_LOGIC, TRUE, probe_names[j]);
		if (!probe)
			return SR_ERR;
		sdi->channels = g_slist_append(sdi->channels, probe);
	}
	return SR_OK;
}

static unsigned int en_ch_num(const struct sr_dev_inst *sdi)
{
	GSList *l;
	unsigned int n = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *p = l->data;
		if (p->enabled)
			n++;
	}
	return n ? n : 1;
}

static unsigned int en_ch_num_mask(const struct sr_dev_inst *sdi)
{
	GSList *l;
	unsigned int mask = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *p = l->data;
		if (p->enabled && p->index < 32)
			mask |= (1u << p->index);
	}
	return mask ? mask : 0xffffffffu;
}

static uint64_t px_channel_depth(const struct sr_dev_inst *sdi)
{
	struct PX_context *devc = sdi->priv;
	unsigned int n = en_ch_num(sdi);

	return (devc->profile->dev_caps.hw_depth / n) & ~PXLOGIC_SAMPLES_ALIGN;
}

static uint64_t align_4k(uint64_t v)
{
	return (v % 4096) ? ((v / 4096 + 1) * 4096) : v;
}

static int load_res_file(const char *name, unsigned char **out, size_t *out_len)
{
	char path[1024];
	FILE *fp;
	long sz;
	unsigned char *buf;

	snprintf(path, sizeof(path), "%s/%s", DS_RES_PATH, name);
	fp = fopen(path, "rb");
	if (!fp) {
		sr_err("firmware not found: %s", path);
		return SR_ERR;
	}
	if (fseek(fp, 0, SEEK_END) != 0 || (sz = ftell(fp)) <= 0) {
		fclose(fp);
		return SR_ERR;
	}
	rewind(fp);
	buf = g_try_malloc((gsize)sz);
	if (!buf) {
		fclose(fp);
		return SR_ERR_MALLOC;
	}
	if (fread(buf, 1, (size_t)sz, fp) != (size_t)sz) {
		g_free(buf);
		fclose(fp);
		return SR_ERR;
	}
	fclose(fp);
	*out = buf;
	*out_len = (size_t)sz;
	return SR_OK;
}

/* mode 0: CH569 app flash  mode 1: FPGA bitstream (device mode 4) */
static int firmware_config(libusb_device_handle *usbdevh, const char *name, int mode)
{
	unsigned char *filebuf = NULL;
	unsigned char *buf = NULL;
	size_t filesize = 0;
	unsigned int base_addr;
	int length, ret;

	if (load_res_file(name, &filebuf, &filesize) != SR_OK)
		return SR_ERR;

	sr_info("load %s (%zu bytes) mode=%d", name, filesize, mode);
	if (mode == 0) {
		base_addr = 48 * 1024;
		length = 48 * 1024;
		buf = g_try_malloc((gsize)length * 3);
		if (!buf) {
			g_free(filebuf);
			return SR_ERR_MALLOC;
		}
		memset(buf, 0xff, (size_t)length);
		memcpy(buf, filebuf, filesize < (size_t)length ? filesize : (size_t)length);
		memcpy(buf + length, buf, (size_t)length);
		memcpy(buf + length * 2, buf, (size_t)length);
		length *= 3;
		libusb_clear_halt(usbdevh, 0x03);
		ret = (int)usb_wr_data_update(usbdevh, base_addr, length, 0, buf, 0);
	} else {
		base_addr = 0;
		length = (int)filesize;
		libusb_clear_halt(usbdevh, 0x03);
		ret = (int)usb_wr_data_update(usbdevh, base_addr, length, 4, filebuf, 0);
	}
	g_free(filebuf);
	g_free(buf);
	if (ret != 0) {
		sr_err("firmware_config write failed (%d)", ret);
		return SR_ERR;
	}
	return SR_OK;
}

static void apply_ch_mode(struct sr_dev_inst *sdi, int id)
{
	struct PX_context *devc = sdi->priv;

	if (!ch_mode_allowed(devc, id)) {
		int i;
		for (i = 0; i < PX_CHMODE_COUNT; i++) {
			if (ch_mode_allowed(devc, i)) {
				id = i;
				break;
			}
		}
	}
	devc->ch_mode = (enum PX_CHANNEL_ID)id;
	devc->stream = channel_modes[id].stream;
	devc->ch_num = channel_modes[id].num;
	devc->cur_samplerate = channel_modes[id].default_samplerate;
	adjust_samplerate(devc);
	setup_probes(sdi, channel_modes[id].num);
}

static void set_trigger(const struct sr_dev_inst *sdi)
{
	struct PX_context *devc = sdi->priv;
	uint32_t tmp;
	int i;

	devc->ch_en = en_ch_num_mask(sdi);
	devc->trig_zero = 0;
	devc->trig_one = 0;
	devc->trig_rise = 0;
	devc->trig_fall = 0;

	if (trigger && trigger->trigger_en) {
		for (i = 0; i < 32; i++) {
			char c = trigger->trigger0[TriggerStages][i];
			uint32_t bit = 1u << i;

			if (c == '0')
				devc->trig_zero |= bit;
			else if (c == '1')
				devc->trig_one |= bit;
			else if (c == 'R')
				devc->trig_rise |= bit;
			else if (c == 'F')
				devc->trig_fall |= bit;
			else if (c == 'C') {
				devc->trig_rise |= bit;
				devc->trig_fall |= bit;
			}
		}
	}

	tmp = (uint32_t)(devc->capture_ratio / 100.0 * devc->limit_samples);
	if (tmp < PXLOGIC_ATOMIC_SAMPLES)
		tmp = PXLOGIC_ATOMIC_SAMPLES;
	if (devc->stream)
		tmp = min(tmp, (uint32_t)(px_channel_depth(sdi) * 10 / 100));
	else
		tmp = min(tmp, (uint32_t)(px_channel_depth(sdi) * PXLOGIC_TRIG_MAX_PERCENT / 100));
	devc->trigger_pos_set = tmp;
}

static int gpio_from_rate(uint64_t rate, unsigned int *mode, unsigned int *div)
{
	*div = 0;
	if (rate == SR_GHZ(1))
		*mode = 0;
	else if (rate == SR_MHZ(500))
		*mode = 1;
	else if (rate == SR_MHZ(250))
		*mode = 2;
	else if (rate == SR_MHZ(125))
		*mode = 3;
	else if (rate == SR_MHZ(800))
		*mode = 0 + 4;
	else if (rate == SR_MHZ(400))
		*mode = 1 + 4;
	else if (rate == SR_MHZ(200))
		*mode = 2 + 4;
	else if (rate == SR_MHZ(100))
		*mode = 3 + 4;
	else {
		*mode = 3 + 4;
		if (rate == SR_MHZ(50))
			*div = 1;
		else if (rate == SR_MHZ(25))
			*div = 3;
		else if (rate == SR_MHZ(20))
			*div = 4;
		else if (rate == SR_MHZ(10))
			*div = 9;
		else if (rate == SR_MHZ(5))
			*div = 19;
		else if (rate == SR_MHZ(4))
			*div = 24;
		else if (rate == SR_MHZ(2))
			*div = 49;
		else if (rate == SR_MHZ(1))
			*div = 99;
		else if (rate == SR_KHZ(500))
			*div = 199;
		else if (rate == SR_KHZ(400))
			*div = 249;
		else if (rate == SR_KHZ(200))
			*div = 499;
		else if (rate == SR_KHZ(100))
			*div = 999;
		else if (rate == SR_KHZ(50))
			*div = 1999;
		else if (rate == SR_KHZ(40))
			*div = 2499;
		else if (rate == SR_KHZ(20))
			*div = 4999;
		else if (rate == SR_KHZ(10))
			*div = 9999;
		else if (rate == SR_KHZ(5))
			*div = 19999;
		else if (rate == SR_KHZ(2))
			*div = 49999;
		else
			*div = 0;
	}
	return 0;
}

/* -------------------- transfers -------------------- */

static void cancel_transfers(struct PX_context *devc)
{
	unsigned int i;

	if (!devc || !devc->transfers)
		return;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void finish_acquisition(struct PX_context *devc)
{
	struct sr_datafeed_packet packet;

	if (!devc || devc->status == PX_ST_FINISH)
		return;
	devc->stop = 1;
	devc->abort = 1;
	devc->status = PX_ST_FINISH;

	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_END;
	packet.status = SR_PKT_OK;
	ds_data_forward(devc->sdi, &packet);

	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	devc->submitted_transfers = 0;
	sr_info("finish, samples=%" PRIu64, devc->samples_counter);
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct PX_context *devc = transfer->user_data;
	unsigned int i;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);
	if (!devc->transfers)
		return;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}
	if (devc->submitted_transfers > 0)
		devc->submitted_transfers--;
	if (devc->submitted_transfers == 0)
		finish_acquisition(devc);
}

static void receive_transfer(struct libusb_transfer *transfer)
{
	struct PX_context *devc = transfer->user_data;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	uint64_t sending_now;
	int r;

	if (devc->acq_aborted || devc->abort) {
		free_transfer(transfer);
		return;
	}

	if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE ||
	    transfer->status == LIBUSB_TRANSFER_STALL) {
		devc->acq_aborted = 1;
		devc->abort = 1;
		free_transfer(transfer);
		return;
	}

	if (transfer->actual_length > 0 &&
	    transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		sending_now = (uint64_t)transfer->actual_length;
		if (devc->op_mode == LO_OP_BUFFER ||
		    (devc->op_mode == LO_OP_STREAM && !devc->is_loop)) {
			uint64_t add = (sending_now * 8) / (uint64_t)devc->ch_num;
			if (devc->limit_samples &&
			    devc->samples_counter + add >= devc->limit_samples) {
				sending_now = (devc->limit_samples - devc->samples_counter) *
					      (uint64_t)devc->ch_num / 8;
				devc->samples_counter = devc->limit_samples;
			} else {
				devc->samples_counter += add;
			}
		} else {
			devc->samples_counter += (sending_now * 8) / (uint64_t)devc->ch_num;
		}

		if (sending_now > 0 && !devc->stop) {
			memset(&packet, 0, sizeof(packet));
			memset(&logic, 0, sizeof(logic));
			packet.type = SR_DF_LOGIC;
			packet.status = SR_PKT_OK;
			packet.payload = &logic;
			logic.length = sending_now;
			logic.unitsize = (uint16_t)(devc->ch_num / 8);
			if (logic.unitsize == 0)
				logic.unitsize = 1;
			logic.format = LA_CROSS_DATA;
			logic.data = transfer->buffer;
			ds_data_forward(devc->sdi, &packet);
		}
	}

	if (devc->limit_samples && devc->samples_counter >= devc->limit_samples &&
	    !(devc->op_mode == LO_OP_STREAM && devc->is_loop)) {
		devc->stop = 1;
		devc->acq_aborted = 1;
		devc->abort = 1;
		free_transfer(transfer);
		return;
	}

	if (!devc->stop && !devc->abort) {
		r = libusb_submit_transfer(transfer);
		if (r != 0) {
			sr_err("resubmit failed: %s", libusb_error_name(r));
			free_transfer(transfer);
		}
	} else {
		free_transfer(transfer);
	}
}

static int start_transfers(struct sr_dev_inst *sdi)
{
	struct PX_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct libusb_transfer *xfer;
	unsigned char *buf;
	unsigned int i, ch_num, ch_en, gpio_mode, gpio_div, stream_mask;
	uint64_t usb_buff_max, samples_ch_1s;
	uint32_t pwm_max;
	int rc;

	devc->acq_aborted = 0;
	devc->stop = 0;
	devc->abort = 0;
	devc->status = PX_ST_DATA;
	memset(&devc->cmd_data, 0, sizeof(devc->cmd_data));

	ch_num = en_ch_num(sdi);
	ch_en = en_ch_num_mask(sdi);
	devc->ch_num = (int)ch_num;
	set_trigger(sdi);
	stream_mask = (devc->op_mode == LO_OP_STREAM) ? (1u << 1) : 0;

	samples_ch_1s = align_4k(devc->cur_samplerate / 100 / 8);
	if (devc->usb_speed == LIBUSB_SPEED_SUPER)
		usb_buff_max = 4ull * 1024 * 1024;
	else
		usb_buff_max = align_4k(480000000ull / 100 / 8);
	if (samples_ch_1s * ch_num > usb_buff_max)
		devc->block_size = (uint32_t)((usb_buff_max / ch_num / 4096) * 4096 * ch_num);
	else
		devc->block_size = (uint32_t)(samples_ch_1s * ch_num);
	if (devc->block_size < 4096)
		devc->block_size = 4096;

	devc->limit_samples2Byte = devc->limit_samples * ch_num / 8 + devc->block_size;

	devc->num_transfers = 4;
	devc->submitted_transfers = 0;
	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * devc->num_transfers);
	if (!devc->transfers)
		return SR_ERR_MALLOC;

	usb_wr_reg(usb->devhdl, 8192 + (11 << 2), 0);
	libusb_clear_halt(usb->devhdl, 0x82);

	pwm_max = 120000000 / 10000;
	usb_wr_reg(usb->devhdl, 2 << 1, pwm_max);
	usb_wr_reg(usb->devhdl, 2 << 2,
		   (uint32_t)(devc->vth * (100.0 / 200.0) / 3.334 * pwm_max));
	usb_wr_reg(usb->devhdl, 4 << 2, 0);
	usb_wr_reg(usb->devhdl, 0 << 2, 5 | stream_mask);
	usb_wr_reg(usb->devhdl, 0 << 2, 5 | stream_mask | (1u << 4));
	usb_wr_reg(usb->devhdl, 0 << 2, 5 | stream_mask);
	usb_wr_reg(usb->devhdl, 8 << 2, 0xffffffff);
	usb_wr_reg(usb->devhdl, 7 << 2, devc->block_size);
	usb_wr_reg(usb->devhdl, 8192 + (2 << 2), devc->block_size);
	usb_wr_reg(usb->devhdl, 8192 + (9 << 2), (uint32_t)devc->limit_samples2Byte);
	usb_wr_reg(usb->devhdl, 8192 + (10 << 2), (uint32_t)(devc->limit_samples2Byte >> 32));

	gpio_from_rate(devc->cur_samplerate, &gpio_mode, &gpio_div);
	usb_wr_reg(usb->devhdl, 15 << 2, devc->ext_trig_mode);
	usb_wr_reg(usb->devhdl, 22 << 2, (uint32_t)devc->trig_out_en);
	usb_wr_reg(usb->devhdl, 5 << 2, gpio_mode | ((unsigned)devc->clock_edge << 3));
	usb_wr_reg(usb->devhdl, 6 << 2, gpio_div);
	usb_wr_reg(usb->devhdl, 8192 + (19 << 2), ch_num);
	usb_wr_reg(usb->devhdl, 8192 + (20 << 2), devc->trigger_pos_set);
	usb_wr_reg(usb->devhdl, 8192 + (11 << 2), 0);
	usb_wr_reg(usb->devhdl, 4 << 2, ch_en);
	usb_wr_reg(usb->devhdl, 0 << 2, 0 | stream_mask | ((unsigned)devc->filter << 3));
	usb_wr_reg(usb->devhdl, 9 << 2, devc->trig_zero);
	usb_wr_reg(usb->devhdl, 10 << 2, devc->trig_one);
	usb_wr_reg(usb->devhdl, 11 << 2, devc->trig_rise);
	usb_wr_reg(usb->devhdl, 12 << 2, devc->trig_fall);
	rc = (int)usb_wr_reg(usb->devhdl, 8 << 2, 0x0);
	(void)rc;

	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc(devc->block_size);
		if (!buf)
			return SR_ERR_MALLOC;
		xfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(xfer, usb->devhdl, 0x82, buf,
					  (int)devc->block_size, receive_transfer,
					  devc, 0);
		if (libusb_submit_transfer(xfer) != 0) {
			sr_err("submit transfer %u failed", i);
			g_free(buf);
			libusb_free_transfer(xfer);
			return SR_ERR;
		}
		devc->transfers[i] = xfer;
		devc->submitted_transfers++;
	}
	return SR_OK;
}

static int receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
	struct PX_context *devc = sdi->priv;
	struct drv_context *drvc = di->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct timeval tv = { 0, 0 };
	int completed = 0;
	struct sr_datafeed_packet packet;

	(void)fd;
	(void)revents;

	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx, &tv, &completed);

	if (devc->samples_counter == 0 && usb && usb->devhdl &&
	    devc->cmd_data.trig_out_validset == 0) {
		if (command_ctl_rddata(usb->devhdl, &devc->cmd_data) == SR_OK &&
		    devc->cmd_data.trig_out_validset) {
			devc->trigger_pos_set = devc->cmd_data.real_pos;
			if (devc->trig_one | devc->trig_zero |
			    devc->trig_fall | devc->trig_rise) {
				memset(&packet, 0, sizeof(packet));
				packet.type = SR_DF_TRIGGER;
				packet.status = SR_PKT_OK;
				ds_data_forward(sdi, &packet);
			}
		}
	}

	if (devc->status == PX_ST_FINISH) {
		if (devc->freewheel)
			sr_session_source_remove(-1);
		return TRUE;
	}
	return TRUE;
}

/* -------------------- driver API -------------------- */

static int hw_init(struct sr_context *sr_ctx)
{
	return std_hw_init(sr_ctx, di, LOG_PREFIX);
}

static int hw_cleanup(void)
{
	safe_free(di->priv);
	return SR_OK;
}

static GSList *hw_scan(GSList *options)
{
	struct drv_context *drvc;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	struct sr_dev_inst *sdi;
	struct PX_context *devc;
	struct sr_usb_dev_inst *usb_info;
	GSList *devices = NULL;
	int i, j;
	enum libusb_speed spd;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		const struct PX_profile *prof = NULL;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		spd = libusb_get_device_speed(devlist[i]);
		if (spd != LIBUSB_SPEED_HIGH && spd != LIBUSB_SPEED_SUPER &&
		    spd != LIBUSB_SPEED_SUPER_PLUS)
			continue;
		for (j = 0; supported_PX[j].vid; j++) {
			if (des.idVendor == supported_PX[j].vid &&
			    des.idProduct == supported_PX[j].pid &&
			    supported_PX[j].usb_speed ==
				    ((spd == LIBUSB_SPEED_SUPER_PLUS) ? LIBUSB_SPEED_SUPER : spd)) {
				prof = &supported_PX[j];
				break;
			}
		}
		if (!prof)
			continue;
		if (sr_usb_device_is_exists(devlist[i]))
			continue;

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc)
			break;
		devc->usb_dev = libusb_ref_device(devlist[i]);
		devc->profile = prof;
		devc->usb_speed = spd;
		devc->op_mode = LO_OP_BUFFER;
		devc->ch_mode = (enum PX_CHANNEL_ID)prof->dev_caps.default_channelmode;
		devc->stream = channel_modes[devc->ch_mode].stream;
		devc->ch_num = channel_modes[devc->ch_mode].num;
		devc->cur_samplerate = channel_modes[devc->ch_mode].default_samplerate;
		devc->limit_samples = SR_Mn(1);
		devc->vth = 1.6;
		devc->filter = 0;
		devc->clock_edge = FALSE;
		devc->capture_ratio = 10;
		devc->max_height = 1;
		devc->pwm0_freq_set = 1000;
		devc->pwm0_duty_set = 500;
		devc->pwm1_freq_set = 1000;
		devc->pwm1_duty_set = 500;
		adjust_samplerate(devc);

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE, prof->vendor, prof->model, NULL);
		if (!sdi) {
			libusb_unref_device(devc->usb_dev);
			g_free(devc);
			break;
		}
		sdi->driver = di;
		sdi->dev_type = DEV_TYPE_USB;
		sdi->priv = devc;
		devc->sdi = sdi;
		sdi->handle = (ds_device_handle)devc->usb_dev;
		usb_info = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
					       libusb_get_device_address(devlist[i]));
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		}
		setup_probes(sdi, devc->ch_num);
		sr_info("Found %s %04x:%04x speed=%d",
			prof->model, des.idVendor, des.idProduct, (int)spd);
		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 0);
	return devices;
}

static const GSList *hw_dev_mode_list(const struct sr_dev_inst *sdi)
{
	static GSList *l;

	(void)sdi;
	if (!l)
		l = g_slist_append(NULL, (gpointer)&sr_mode_list[0]);
	return l;
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	struct PX_context *devc = sdi->priv;
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint32_t reg, lm;
	int r;

	if (sdi->status == SR_ST_ACTIVE)
		return SR_OK;
	if (!devc->usb_dev)
		return SR_ERR;

	r = libusb_open(devc->usb_dev, &usb->devhdl);
	if (r != 0) {
		sr_err("open failed: %s", libusb_error_name(r));
		if (r == LIBUSB_ERROR_NOT_SUPPORTED)
			ds_set_last_error(SR_ERR_DEVICE_NO_DRIVER);
		else
			ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	if (libusb_kernel_driver_active(usb->devhdl, 0) == 1)
		libusb_detach_kernel_driver(usb->devhdl, 0);
	if (libusb_claim_interface(usb->devhdl, USB_INTERFACE_C) < 0) {
		sr_err("claim iface C failed");
		libusb_close(usb->devhdl);
		usb->devhdl = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	if (libusb_claim_interface(usb->devhdl, USB_INTERFACE_D) < 0)
		sr_warn("claim iface D failed (continuing)");

	devc->usb_speed = libusb_get_device_speed(devc->usb_dev);
	lm = 0;
	if (usb_rd_reg(usb->devhdl, 8192 + 22 * 4, &lm) == 0 &&
	    lm != devc->profile->logic_mode) {
		int k;
		for (k = 0; supported_PX[k].vid; k++) {
			if (supported_PX[k].vid == devc->profile->vid &&
			    supported_PX[k].pid == devc->profile->pid &&
			    supported_PX[k].usb_speed == devc->profile->usb_speed &&
			    supported_PX[k].logic_mode == lm) {
				devc->profile = &supported_PX[k];
				g_free(sdi->name);
				sdi->name = g_strdup(devc->profile->model);
				sr_info("profile -> %s (logic_mode=%u)",
					devc->profile->model, lm);
				break;
			}
		}
	}

	reg = 0;
	if (usb_rd_reg(usb->devhdl, 8192 + 13 * 4, &reg) == 0) {
		sr_info("fw ver on device=0x%x expect=0x%x",
			reg, devc->profile->firmware_version);
		if (reg != devc->profile->firmware_version) {
			if (firmware_config(usb->devhdl, devc->profile->firmware, 0) != SR_OK) {
				sdi->status = SR_ST_INITIALIZING;
				return SR_ERR;
			}
			usb_wr_reg(usb->devhdl, 8192 + 12 * 4, 0);
			sdi->status = SR_ST_INITIALIZING;
			sr_info("firmware uploaded, waiting re-enumeration");
			return SR_ERR_DEVICE_CLOSED;
		}
	}

	if (firmware_config(usb->devhdl, devc->profile->fpga_rst_bit, 1) != SR_OK)
		sr_warn("FPGA rst bitstream failed");
	if (firmware_config(usb->devhdl, devc->profile->fpga_bit, 1) != SR_OK) {
		sr_err("FPGA bitstream failed");
		return SR_ERR;
	}

	sdi->status = SR_ST_ACTIVE;
	sr_info("open ok: %s", sdi->name ? sdi->name : "PXLogic");
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;

	if (!sdi || !sdi->conn)
		return SR_ERR;
	usb = sdi->conn;
	if (usb->devhdl) {
		libusb_release_interface(usb->devhdl, USB_INTERFACE_C);
		libusb_release_interface(usb->devhdl, USB_INTERFACE_D);
		libusb_close(usb->devhdl);
		usb->devhdl = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct PX_context *devc;

	if (!sdi)
		return SR_ERR;
	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		g_free(devc);
		sdi->priv = NULL;
	}
	if (sdi->conn) {
		sr_usb_dev_inst_free(sdi->conn);
		sdi->conn = NULL;
	}
	sr_dev_inst_free(sdi);
	return SR_OK;
}

SR_PRIV void pxlogic_on_usb_reconnected(struct sr_dev_inst *sdi,
					struct libusb_device *new_dev)
{
	struct PX_context *devc;

	if (!sdi || !sdi->priv || !new_dev)
		return;
	if (!sdi->driver || !sdi->driver->name ||
	    strcmp(sdi->driver->name, "pxlogic") != 0)
		return;
	devc = sdi->priv;
	if (devc->usb_dev)
		libusb_unref_device(devc->usb_dev);
	devc->usb_dev = libusb_ref_device(new_dev);
	devc->sdi = sdi;
	sdi->handle = (ds_device_handle)devc->usb_dev;
	devc->usb_speed = libusb_get_device_speed(new_dev);
	sr_info("PXLogic reconnected speed=%d", (int)devc->usb_speed);
}

static int hw_dev_status_get(const struct sr_dev_inst *sdi,
			     struct sr_status *status, gboolean prg)
{
	struct PX_context *devc;

	(void)prg;
	if (!sdi || !sdi->priv || !status)
		return SR_ERR;
	devc = sdi->priv;
	memset(status, 0, sizeof(*status));
	status->trig_hit = (uint8_t)devc->cmd_data.trig_out_validset;
	status->vlen = devc->block_size;
	status->captured_cnt0 = (uint8_t)devc->samples_counter;
	status->captured_cnt1 = (uint8_t)(devc->samples_counter >> 8);
	status->captured_cnt2 = (uint8_t)(devc->samples_counter >> 16);
	status->captured_cnt3 = (uint8_t)(devc->samples_counter >> 24);
	status->stream_mode = devc->stream;
	return SR_OK;
}

static int config_get(int id, GVariant **data, const struct sr_dev_inst *sdi,
		      const struct sr_channel *ch,
		      const struct sr_channel_group *cg)
{
	struct PX_context *devc;

	(void)ch;
	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_DEVICE_MODE:
		*data = g_variant_new_int16(sdi->mode);
		break;
	case SR_CONF_UNIT_BITS:
		*data = g_variant_new_byte(1);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_string(maxHeights[devc->max_height]);
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		*data = g_variant_new_byte((uint8_t)devc->max_height);
		break;
	case SR_CONF_HW_DEPTH:
		*data = g_variant_new_uint64(px_channel_depth(sdi));
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16(channel_modes[devc->ch_mode].num);
		break;
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int16((int16_t)en_ch_num(sdi));
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(devc->stream);
		break;
	case SR_CONF_VTH:
		*data = g_variant_new_double(devc->vth);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16((int16_t)devc->op_mode);
		break;
	case SR_CONF_CHANNEL_MODE:
		*data = g_variant_new_int16((int16_t)devc->ch_mode);
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_int16((int16_t)devc->filter);
		break;
	case SR_CONF_CLOCK_EDGE:
		*data = g_variant_new_boolean(devc->clock_edge);
		break;
	case SR_CONF_LOOP_MODE:
		*data = g_variant_new_boolean(devc->is_loop != 0);
		break;
	case SR_CONF_USB_SPEED:
		*data = g_variant_new_int32((int32_t)devc->usb_speed);
		break;
	case SR_CONF_USB30_SUPPORT:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_RLE:
	case SR_CONF_RLE_SUPPORT:
		*data = g_variant_new_boolean(FALSE);
		break;
	case SR_CONF_TRIGGER_POS:
		*data = g_variant_new_uint64(devc->trigger_pos_set);
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_set(int id, GVariant *data, struct sr_dev_inst *sdi,
		      struct sr_channel *ch,
		      struct sr_channel_group *cg)
{
	struct PX_context *devc;
	const char *stropt;
	unsigned int i;

	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_SAMPLERATE: {
		uint64_t want = g_variant_get_uint64(data);
		uint64_t best = samplerates[devc->samplerates_min_index];
		uint64_t best_err = UINT64_MAX;

		for (i = devc->samplerates_min_index; i <= devc->samplerates_max_index; i++) {
			uint64_t err = (samplerates[i] > want) ?
				(samplerates[i] - want) : (want - samplerates[i]);
			if (err < best_err) {
				best_err = err;
				best = samplerates[i];
			}
		}
		devc->cur_samplerate = best;
		break;
	}
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_CAPTURE_RATIO:
		devc->capture_ratio = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_EN:
		if (ch)
			ch->enabled = g_variant_get_boolean(data);
		break;
	case SR_CONF_MAX_HEIGHT:
		stropt = g_variant_get_string(data, NULL);
		for (i = 0; i < ARRAY_SIZE(maxHeights); i++) {
			if (!strcmp(stropt, maxHeights[i])) {
				devc->max_height = (int)i;
				break;
			}
		}
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		devc->max_height = g_variant_get_byte(data);
		break;
	case SR_CONF_VTH: {
		double v = g_variant_get_double(data);
		if (v < 0.0)
			v = 0.0;
		if (v > 5.0)
			v = 5.0;
		devc->vth = v;
		break;
	}
	case SR_CONF_FILTER:
		devc->filter = g_variant_get_int16(data) ? 1 : 0;
		break;
	case SR_CONF_CLOCK_EDGE:
		devc->clock_edge = g_variant_get_boolean(data);
		break;
	case SR_CONF_CHANNEL_MODE: {
		int nv = g_variant_get_int16(data);
		if (nv < 0 || nv >= PX_CHMODE_COUNT)
			return SR_ERR;
		apply_ch_mode(sdi, nv);
		break;
	}
	case SR_CONF_OPERATION_MODE: {
		int nv = g_variant_get_int16(data);
		if (nv != LO_OP_BUFFER && nv != LO_OP_STREAM)
			return SR_ERR;
		devc->op_mode = (uint16_t)nv;
		devc->stream = (nv == LO_OP_STREAM);
		apply_ch_mode(sdi, devc->ch_mode);
		break;
	}
	case SR_CONF_LOOP_MODE:
		devc->is_loop = g_variant_get_boolean(data) ? 1 : 0;
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_list(int key, GVariant **data, const struct sr_dev_inst *sdi,
		       const struct sr_channel_group *cg)
{
	GVariant *gvar;
	GVariantBuilder gvb;
	static uint64_t rate_buf[32];
	int rate_n = 0;
	unsigned int i;
	const struct PX_context *devc =
		(sdi && sdi->priv) ? (const struct PX_context *)sdi->priv : NULL;

	(void)cg;

	switch (key) {
	case SR_CONF_DEVICE_OPTIONS:
		*data = g_variant_new_from_data(
			G_VARIANT_TYPE("ai"), hwoptions,
			ARRAY_SIZE(hwoptions) * sizeof(int32_t), TRUE, NULL, NULL);
		break;
	case SR_CONF_DEVICE_SESSIONS:
		*data = g_variant_new_from_data(
			G_VARIANT_TYPE("ai"), sessions,
			ARRAY_SIZE(sessions) * sizeof(int32_t), TRUE, NULL, NULL);
		break;
	case SR_CONF_SAMPLERATE:
		if (devc) {
			for (i = devc->samplerates_min_index;
			     i <= devc->samplerates_max_index && rate_n < 31; i++)
				rate_buf[rate_n++] = samplerates[i];
		} else {
			for (i = 0; i < ARRAY_SIZE(samplerates) && rate_n < 31; i++)
				rate_buf[rate_n++] = samplerates[i];
		}
		rate_buf[rate_n] = 0;
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_from_data(G_VARIANT_TYPE("at"), rate_buf,
					       (gsize)rate_n * sizeof(uint64_t),
					       TRUE, NULL, NULL);
		g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_strv(maxHeights, ARRAY_SIZE(maxHeights));
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_uint64((uint64_t)&filter_list);
		break;
	case SR_CONF_CHANNEL_MODE:
		rebuild_channel_mode_ui(devc);
		*data = g_variant_new_uint64((uint64_t)&channel_mode_ui);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_uint64((uint64_t)&opmode_list);
		break;
	default:
		return SR_ERR_ARG;
	}
	return SR_OK;
}

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
	struct PX_context *devc = sdi->priv;
	struct drv_context *drvc = di->priv;
	const struct libusb_pollfd **lupfd;
	unsigned int i;
	int ret;

	(void)cb_data;
	if (sdi->status != SR_ST_ACTIVE) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	devc->sdi = sdi;
	devc->samples_counter = 0;
	devc->stop = 0;
	devc->abort = 0;
	devc->acq_aborted = 0;
	devc->freewheel = 0;
	memset(&devc->cmd_data, 0, sizeof(devc->cmd_data));

	sr_info("acq start %s nch=%d rate=%" PRIu64 " limit=%" PRIu64 " stream=%d",
		devc->op_mode == LO_OP_STREAM ? "STREAM" : "BUFFER",
		devc->ch_num, devc->cur_samplerate, devc->limit_samples,
		devc->stream);

	if ((ret = start_transfers(sdi)) != SR_OK)
		return ret;

	lupfd = libusb_get_pollfds(drvc->sr_ctx->libusb_ctx);
	if (!lupfd) {
		devc->freewheel = 1;
		sr_session_source_add(-1, 0, 0, receive_data, sdi);
	} else {
		for (i = 0; lupfd[i]; i++)
			sr_source_add(lupfd[i]->fd, lupfd[i]->events, 20, receive_data, sdi);
		g_free((void *)lupfd);
	}

	std_session_send_df_header(sdi, LOG_PREFIX);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct PX_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	devc->stop = 1;
	devc->acq_aborted = 1;
	if (!devc->abort) {
		devc->abort = 1;
		cancel_transfers(devc);
		if (devc->submitted_transfers <= 0)
			finish_acquisition(devc);
	}
	return SR_OK;
}

SR_PRIV struct sr_dev_driver pxlogic_driver_info = {
	.name = "pxlogic",
	.longname = "PXLogic USB Logic Analyzer",
	.api_version = 1,
	.driver_type = DRIVER_TYPE_HARDWARE,
	.init = hw_init,
	.cleanup = hw_cleanup,
	.scan = hw_scan,
	.dev_mode_list = hw_dev_mode_list,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = hw_dev_open,
	.dev_close = hw_dev_close,
	.dev_destroy = hw_dev_destroy,
	.dev_status_get = hw_dev_status_get,
	.dev_acquisition_start = hw_dev_acquisition_start,
	.dev_acquisition_stop = hw_dev_acquisition_stop,
	.priv = NULL,
};
