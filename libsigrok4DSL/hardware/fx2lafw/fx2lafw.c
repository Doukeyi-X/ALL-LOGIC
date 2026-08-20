/*
 * Cypress FX2 / sigrok fx2lafw driver for DSView / libsigrok4DSL (ALL LOGIC)
 *
 * nanoDLA (Muse Lab, https://github.com/wuxx/nanoDLA) enumerates as
 * 1D50:608C (C0 EEPROM) and uses PulseView's fx2lafw 8ch firmware.
 *
 * Matches PulseView fx2lafw devices that already enumerate as:
 *   1D50:608C  8ch  (nanoDLA and clones)
 *   1D50:608D  16ch
 * Do not match 04B4:8613 — that is also DSLogic FX2 boot.
 *
 * Protocol from libsigrok src/hardware/fx2lafw (GPLv3+).
 * Firmware: sigrok-firmware-fx2lafw (GPLv2+ / LGPLv2.1+), in DSView/res.
 */

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <glib.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "fx2lafw.h"

#undef LOG_PREFIX
#define LOG_PREFIX "fx2lafw: "

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1
#define FX2_EP_IN		(2 | LIBUSB_ENDPOINT_IN)

#define CMD_GET_FW_VERSION	0xb0
#define CMD_START		0xb1
#define CMD_GET_REVID_VERSION	0xb2

#define CMD_START_FLAGS_WIDE_POS	5
#define CMD_START_FLAGS_CLK_SRC_POS	6
#define CMD_START_FLAGS_SAMPLE_8BIT	(0 << CMD_START_FLAGS_WIDE_POS)
#define CMD_START_FLAGS_SAMPLE_16BIT	(1 << CMD_START_FLAGS_WIDE_POS)
#define CMD_START_FLAGS_CLK_30MHZ	(0 << CMD_START_FLAGS_CLK_SRC_POS)
#define CMD_START_FLAGS_CLK_48MHZ	(1 << CMD_START_FLAGS_CLK_SRC_POS)

#define FX2LAFW_REQUIRED_VERSION_MAJOR	1
#define MAX_SAMPLE_DELAY		(6 * 256)
#define MAX_RENUM_DELAY_MS		3000
#define MAX_EMPTY_TRANSFERS		32

#define NUM_TRANSFERS			8
#define XFER_TIMEOUT_MS			500

#define CROSS_SCALE			64
#define CROSS_PLANE_BYTES		8
#define MAX_CH				16
#define DEFAULT_SAMPLES			SR_Mn(1)
#define HW_DEPTH			SR_Mn(16)

#define DEV_CAPS_16BIT			(1u << 0)

enum {
	ST_IDLE = 0,
	ST_START,
	ST_DATA,
	ST_STOP,
	ST_FINISH,
};

#pragma pack(push, 1)
struct version_info {
	uint8_t major;
	uint8_t minor;
};

struct cmd_start_acquisition {
	uint8_t flags;
	uint8_t sample_delay_h;
	uint8_t sample_delay_l;
};
#pragma pack(pop)

struct fx2_profile {
	uint16_t vid;
	uint16_t pid;
	const char *vendor;
	const char *model;
	const char *firmware;
	uint32_t caps;
};

static const struct fx2_profile profiles[] = {
	{ 0x1d50, 0x608c, "Muse Lab", "nanoDLA",
	  "fx2lafw-sigrok-fx2-8ch.fw", 0 },
	{ 0x1d50, 0x608d, "sigrok", "FX2 LA (16ch)",
	  "fx2lafw-sigrok-fx2-16ch.fw", DEV_CAPS_16BIT },
	{ 0, 0, NULL, NULL, NULL, 0 },
};

static const uint64_t samplerates[] = {
	SR_KHZ(20), SR_KHZ(25), SR_KHZ(50), SR_KHZ(100),
	SR_KHZ(200), SR_KHZ(250), SR_KHZ(500),
	SR_MHZ(1), SR_MHZ(2), SR_MHZ(3), SR_MHZ(4),
	SR_MHZ(6), SR_MHZ(8), SR_MHZ(12), SR_MHZ(16), SR_MHZ(24),
};

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
};

static const struct sr_list_item opmode_list[] = {
	{ LO_OP_STREAM, "Stream Mode" },
	{ -1, NULL },
};

struct fx2_context {
	struct libusb_device *usb_dev;
	struct libusb_device_handle *devhdl;
	struct sr_context *sr_ctx;
	const struct sr_dev_inst *sdi;
	const struct fx2_profile *profile;

	int num_channels;
	int sample_wide; /* 1 = 16-bit samples */
	uint64_t samplerate;
	uint64_t limit_samples;
	uint64_t num_samples;
	int max_height;
	int op_mode;
	int is_loop;

	int64_t fw_updated;
	int fw_loaded;

	int status;
	int abort;
	int submitted_transfers;
	int num_transfers;
	int empty_transfer_count;
	struct libusb_transfer **transfers;
	int freewheel;

	uint8_t *raw_pending;
	int raw_pending_len;
	int raw_pending_cap;
	uint8_t stream_res[2];
	int stream_res_len;

	uint8_t *cross_buf;
	int cross_cap;
	uint8_t en_bits[MAX_CH];
	int en_count;
};

SR_PRIV struct sr_dev_driver fx2lafw_driver_info;
static struct sr_dev_driver *di = &fx2lafw_driver_info;

/* -------------------- helpers -------------------- */

static const struct fx2_profile *fx2_find_profile(uint16_t vid, uint16_t pid)
{
	int i;
	for (i = 0; profiles[i].vid; i++) {
		if (profiles[i].vid == vid && profiles[i].pid == pid)
			return &profiles[i];
	}
	return NULL;
}

static char *fx2_fw_path(const char *name)
{
	if (!name || DS_RES_PATH[0] == '\0')
		return NULL;
	return g_build_filename(DS_RES_PATH, name, NULL);
}

static int fx2_get_fw_version(libusb_device_handle *hdl, struct version_info *vi)
{
	int ret = libusb_control_transfer(
		hdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
		CMD_GET_FW_VERSION, 0, 0,
		(unsigned char *)vi, sizeof(*vi), 1000);
	return (ret == (int)sizeof(*vi)) ? SR_OK : SR_ERR;
}

/*
 * PulseView: RAM firmware is running iff manufacturer/product are
 * "sigrok"/"fx2lafw". Do not probe GET_FW_VERSION here — on a C0
 * stub it stalls for the control timeout and slows every scan.
 */
static int fx2_has_running_fw(libusb_device *dev)
{
	libusb_device_handle *hdl = NULL;
	struct libusb_device_descriptor des;
	unsigned char man[64], prod[64];
	int ok = 0;

	if (!dev || libusb_get_device_descriptor(dev, &des) != 0)
		return 0;
	if (libusb_open(dev, &hdl) != 0)
		return 0;

	man[0] = prod[0] = 0;
	if (des.iManufacturer)
		libusb_get_string_descriptor_ascii(hdl, des.iManufacturer,
						   man, sizeof(man));
	if (des.iProduct)
		libusb_get_string_descriptor_ascii(hdl, des.iProduct,
						   prod, sizeof(prod));
	libusb_close(hdl);

	if (g_ascii_strcasecmp((char *)man, "sigrok") == 0 &&
	    strstr((char *)prod, "fx2lafw") != NULL)
		ok = 1;
	return ok;
}

/* After RAM upload the FX2 drops off the bus and comes back.
 * Must pump libusb events — otherwise Windows never sees the re-enum. */
static libusb_device *fx2_wait_live(struct sr_context *sr_ctx,
				    const struct fx2_profile *want)
{
	libusb_device **devlist;
	libusb_device *found = NULL;
	int t, i;

	for (t = 0; t < 25 && !found; t++) {
		struct timeval tv = { 0, 100000 };
		libusb_handle_events_timeout_completed(sr_ctx->libusb_ctx,
						       &tv, NULL);
		if (libusb_get_device_list(sr_ctx->libusb_ctx, &devlist) < 0)
			continue;
		for (i = 0; devlist[i]; i++) {
			struct libusb_device_descriptor des;
			const struct fx2_profile *p;

			if (libusb_get_device_descriptor(devlist[i], &des) != 0)
				continue;
			p = fx2_find_profile(des.idVendor, des.idProduct);
			if (!p)
				continue;
			if (want && p != want)
				continue;
			if (sr_usb_device_is_exists(devlist[i]) ||
			    sr_usb_driver_vidpid_is_listed("fx2lafw",
							   des.idVendor,
							   des.idProduct))
				continue;
			if (!fx2_has_running_fw(devlist[i]))
				continue;
			found = libusb_ref_device(devlist[i]);
			break;
		}
		libusb_free_device_list(devlist, 1);
	}
	return found;
}

static uint64_t fx2_max_rate(const struct fx2_context *devc)
{
	return devc->sample_wide ? SR_MHZ(12) : SR_MHZ(24);
}

static void fx2_clamp_rate(struct fx2_context *devc)
{
	uint64_t maxr = fx2_max_rate(devc);
	int i, best = 7; /* 1 MHz */
	uint64_t dbest = UINT64_MAX;

	if (devc->samplerate > maxr)
		devc->samplerate = maxr;
	for (i = 0; i < (int)ARRAY_SIZE(samplerates); i++) {
		uint64_t r = samplerates[i];
		uint64_t d;
		if (r > maxr)
			continue;
		d = r > devc->samplerate ? r - devc->samplerate
					 : devc->samplerate - r;
		if (d < dbest) {
			dbest = d;
			best = i;
		}
	}
	devc->samplerate = samplerates[best];
}

static void fx2_refresh_enabled(struct fx2_context *devc)
{
	GSList *l;
	int n = 0;

	for (l = devc->sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled)
			continue;
		if (ch->index >= MAX_CH)
			continue;
		devc->en_bits[n++] = (uint8_t)ch->index;
	}
	if (n == 0) {
		for (n = 0; n < devc->num_channels; n++)
			devc->en_bits[n] = (uint8_t)n;
	}
	devc->en_count = n;
}

static int fx2_ensure(uint8_t **buf, int *cap, int need)
{
	if (need <= *cap)
		return SR_OK;
	{
		uint8_t *p = g_try_realloc(*buf, (gsize)need);
		if (!p)
			return SR_ERR_MALLOC;
		*buf = p;
		*cap = need;
	}
	return SR_OK;
}

static uint16_t fx2_load_sample(const uint8_t *p, int sample_bytes)
{
	if (sample_bytes == 1)
		return p[0];
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int fx2_convert_push(struct fx2_context *devc, const uint8_t *raw, int raw_len)
{
	const int sample_bytes = devc->sample_wide ? 2 : 1;
	const int en = devc->en_count;
	const int raw_group = CROSS_SCALE * sample_bytes;
	const int cross_group = CROSS_PLANE_BYTES * en;
	int groups, g, c, i, consumed, left, out_len;
	uint8_t *out;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	const uint8_t *src;
	int src_len;

	if (en < 1 || raw_len <= 0)
		return SR_OK;

	/* stitch leftover half-sample */
	if (devc->stream_res_len && sample_bytes == 2) {
		uint8_t tmp[4];
		int use;
		tmp[0] = devc->stream_res[0];
		tmp[1] = raw[0];
		use = 1;
		devc->stream_res_len = 0;
		if (fx2_convert_push(devc, tmp, 2) != SR_OK)
			return SR_ERR;
		raw += use;
		raw_len -= use;
		if (raw_len <= 0)
			return SR_OK;
	}

	if (fx2_ensure(&devc->raw_pending, &devc->raw_pending_cap,
		       devc->raw_pending_len + raw_len) != SR_OK)
		return SR_ERR_MALLOC;
	memcpy(devc->raw_pending + devc->raw_pending_len, raw, raw_len);
	devc->raw_pending_len += raw_len;

	src = devc->raw_pending;
	src_len = (devc->raw_pending_len / sample_bytes) * sample_bytes;
	if (devc->raw_pending_len > src_len && sample_bytes == 2) {
		devc->stream_res[0] = devc->raw_pending[src_len];
		devc->stream_res_len = 1;
		devc->raw_pending_len = src_len;
	}

	groups = src_len / raw_group;
	if (groups <= 0)
		return SR_OK;

	if (devc->limit_samples && !devc->is_loop) {
		uint64_t room = (devc->num_samples < devc->limit_samples)
			? (devc->limit_samples - devc->num_samples) : 0;
		int maxg = (int)(room / CROSS_SCALE);
		if (maxg <= 0) {
			devc->num_samples = devc->limit_samples;
			return SR_OK;
		}
		if (groups > maxg)
			groups = maxg;
	}

	out_len = groups * cross_group;
	if (fx2_ensure(&devc->cross_buf, &devc->cross_cap, out_len) != SR_OK)
		return SR_ERR_MALLOC;

	out = devc->cross_buf;
	for (g = 0; g < groups; g++) {
		const uint8_t *blk = src + g * raw_group;
		for (c = 0; c < en; c++) {
			uint64_t plane = 0;
			uint8_t bit = devc->en_bits[c];
			for (i = 0; i < CROSS_SCALE; i++) {
				uint16_t s = fx2_load_sample(blk + i * sample_bytes,
							     sample_bytes);
				if ((s >> bit) & 1u)
					plane |= (1ULL << i);
			}
			out[0] = (uint8_t)plane;
			out[1] = (uint8_t)(plane >> 8);
			out[2] = (uint8_t)(plane >> 16);
			out[3] = (uint8_t)(plane >> 24);
			out[4] = (uint8_t)(plane >> 32);
			out[5] = (uint8_t)(plane >> 40);
			out[6] = (uint8_t)(plane >> 48);
			out[7] = (uint8_t)(plane >> 56);
			out += CROSS_PLANE_BYTES;
		}
	}

	packet.type = SR_DF_LOGIC;
	packet.status = SR_PKT_OK;
	packet.payload = &logic;
	logic.length = (uint64_t)out_len;
	logic.format = LA_CROSS_DATA;
	logic.data_error = 0;
	logic.data = devc->cross_buf;
	ds_data_forward(devc->sdi, &packet);

	devc->num_samples += (uint64_t)groups * CROSS_SCALE;

	consumed = groups * raw_group;
	left = devc->raw_pending_len - consumed;
	if (left > 0)
		memmove(devc->raw_pending, devc->raw_pending + consumed, left);
	devc->raw_pending_len = left;
	return SR_OK;
}

static int fx2_cmd_start(struct fx2_context *devc)
{
	struct cmd_start_acquisition cmd;
	int delay = 0;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	if ((SR_MHZ(48) % devc->samplerate) == 0) {
		cmd.flags = CMD_START_FLAGS_CLK_48MHZ;
		delay = (int)(SR_MHZ(48) / devc->samplerate) - 1;
		if (delay > MAX_SAMPLE_DELAY)
			delay = 0;
	}
	if (delay == 0 && (SR_MHZ(30) % devc->samplerate) == 0) {
		cmd.flags = CMD_START_FLAGS_CLK_30MHZ;
		delay = (int)(SR_MHZ(30) / devc->samplerate) - 1;
	}
	if (delay < 0 || delay > MAX_SAMPLE_DELAY) {
		sr_err("cannot sample at %" PRIu64 " Hz", devc->samplerate);
		return SR_ERR;
	}
	cmd.sample_delay_h = (uint8_t)((delay >> 8) & 0xff);
	cmd.sample_delay_l = (uint8_t)(delay & 0xff);
	cmd.flags |= devc->sample_wide ? CMD_START_FLAGS_SAMPLE_16BIT
				       : CMD_START_FLAGS_SAMPLE_8BIT;

	ret = libusb_control_transfer(
		devc->devhdl, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
		CMD_START, 0, 0, (unsigned char *)&cmd, sizeof(cmd), 200);
	if (ret < 0) {
		sr_err("START failed: %s", libusb_error_name(ret));
		return SR_ERR;
	}
	sr_info("START rate=%" PRIu64 " delay=%d flags=0x%02x wide=%d",
		devc->samplerate, delay, cmd.flags, devc->sample_wide);
	return SR_OK;
}

/* -------------------- transfers -------------------- */

static void finish_acquisition(struct fx2_context *devc)
{
	struct sr_datafeed_packet packet;

	if (!devc || devc->status == ST_FINISH)
		return;
	devc->abort = 1;
	sr_info("SR_DF_END samples=%" PRIu64, devc->num_samples);
	packet.type = SR_DF_END;
	packet.status = SR_PKT_OK;
	packet.payload = NULL;
	ds_data_forward(devc->sdi, &packet);
	if (devc->transfers) {
		g_free(devc->transfers);
		devc->transfers = NULL;
	}
	devc->num_transfers = 0;
	devc->submitted_transfers = 0;
	devc->status = ST_FINISH;
}

static void fx2_cancel_transfers(struct fx2_context *devc)
{
	int i;
	if (!devc || !devc->transfers)
		return;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void fx2_request_stop(struct fx2_context *devc)
{
	if (!devc || devc->abort)
		return;
	devc->abort = 1;
	devc->status = ST_STOP;
	fx2_cancel_transfers(devc);
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct fx2_context *devc = transfer->user_data;
	int i;

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
	devc->submitted_transfers--;
	if (devc->submitted_transfers <= 0)
		finish_acquisition(devc);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct fx2_context *devc = transfer->user_data;
	int r, err = 0;

	if (devc->status == ST_START)
		devc->status = ST_DATA;
	if (devc->abort)
		devc->status = ST_STOP;

	if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
		fx2_request_stop(devc);
		free_transfer(transfer);
		return;
	}
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED &&
	    transfer->status != LIBUSB_TRANSFER_TIMED_OUT)
		err = 1;

	if (devc->status == ST_DATA && transfer->actual_length > 0 && !err) {
		devc->empty_transfer_count = 0;
		if (fx2_convert_push(devc, transfer->buffer,
				     transfer->actual_length) != SR_OK)
			fx2_request_stop(devc);
		if (!devc->is_loop && devc->limit_samples &&
		    devc->num_samples >= devc->limit_samples)
			fx2_request_stop(devc);
	} else if (devc->status == ST_DATA) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS)
			fx2_request_stop(devc);
	}

	if (devc->status == ST_DATA && !devc->abort) {
		r = libusb_submit_transfer(transfer);
		if (r != 0) {
			sr_err("resubmit failed: %s", libusb_error_name(r));
			free_transfer(transfer);
		}
	} else {
		free_transfer(transfer);
	}
}

static int fx2_xfer_size(const struct fx2_context *devc)
{
	size_t s;

	/* ~10ms of samples, multiple of 512 (same idea as PulseView). */
	s = (size_t)(devc->samplerate / 100);
	if (s < 2048)
		s = 2048;
	if (s > 256 * 1024)
		s = 256 * 1024;
	return (int)((s + 511) & ~(size_t)511);
}

static int start_transfers(struct fx2_context *devc)
{
	int i;
	int xfer_size;
	unsigned char *buf;
	struct libusb_transfer *xfer;

	xfer_size = fx2_xfer_size(devc);
	devc->num_transfers = NUM_TRANSFERS;
	devc->submitted_transfers = 0;
	devc->empty_transfer_count = 0;
	devc->transfers = g_try_malloc0(sizeof(struct libusb_transfer *) *
					devc->num_transfers);
	if (!devc->transfers)
		return SR_ERR_MALLOC;
	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc((gsize)xfer_size);
		if (!buf)
			return SR_ERR_MALLOC;
		xfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(xfer, devc->devhdl, FX2_EP_IN,
					  buf, xfer_size,
					  receive_transfer, devc, XFER_TIMEOUT_MS);
		if (libusb_submit_transfer(xfer) != 0) {
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
	struct fx2_context *devc = sdi->priv;
	struct drv_context *drvc = di->priv;
	struct timeval tv = { 0, 0 };
	int completed = 0;

	(void)fd;
	(void)revents;
	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx,
					       &tv, &completed);
	if (devc->status == ST_FINISH) {
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

static void setup_channels(struct sr_dev_inst *sdi, int count)
{
	int i;
	char name[16];

	while (sdi->channels) {
		struct sr_channel *ch = sdi->channels->data;
		sdi->channels = g_slist_delete_link(sdi->channels, sdi->channels);
		g_free(ch->name);
		g_free(ch);
	}
	for (i = 0; i < count; i++) {
		snprintf(name, sizeof(name), "%d", i);
		sdi->channels = g_slist_append(
			sdi->channels,
			sr_channel_new(i, SR_CHANNEL_LOGIC, TRUE, name));
	}
}

static int fx2_try_upload(libusb_device *dev, const struct fx2_profile *prof)
{
	char *path;
	int ret;

	path = fx2_fw_path(prof->firmware);
	if (!path) {
		sr_err("firmware dir not set");
		return SR_ERR;
	}
	sr_info("uploading %s", path);
	ret = ezusb_upload_firmware(dev, USB_CONFIGURATION, path);
	g_free(path);
	return ret;
}

static GSList *hw_scan(GSList *options)
{
	struct drv_context *drvc;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	GSList *devices = NULL;
	static int64_t last_upload;
	int i;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		const struct fx2_profile *prof;
		struct fx2_context *devc;
		struct sr_dev_inst *sdi;
		struct sr_usb_dev_inst *usb_info;
		libusb_device *live;
		uint8_t bus, address;
		int has_fw;
		int64_t now;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		prof = fx2_find_profile(des.idVendor, des.idProduct);
		if (!prof)
			continue;
		if (sr_usb_device_is_exists(devlist[i]) ||
		    sr_usb_driver_vidpid_is_listed("fx2lafw",
						   des.idVendor, des.idProduct))
			continue;

		has_fw = fx2_has_running_fw(devlist[i]);
		live = NULL;

		/*
		 * PulseView: skip RAM upload when strings already say
		 * sigrok/fx2lafw. nanoDLA C0 still needs one upload per
		 * power-on; do not add the stub until firmware is live.
		 */
		if (!has_fw) {
			now = g_get_monotonic_time();
			if (last_upload && now - last_upload < G_TIME_SPAN_SECOND * 5) {
				sr_info("skip re-upload %04x:%04x (waiting for FX2 re-enum)",
					des.idVendor, des.idProduct);
				continue;
			}
			if (fx2_try_upload(devlist[i], prof) != SR_OK) {
				sr_err("firmware upload failed for %04x:%04x",
				       des.idVendor, des.idProduct);
				continue;
			}
			last_upload = g_get_monotonic_time();
			live = fx2_wait_live(drvc->sr_ctx, prof);
			if (!live) {
				sr_err("FX2 did not come back after firmware upload");
				continue;
			}
			if (libusb_get_device_descriptor(live, &des) == 0) {
				const struct fx2_profile *p2 =
					fx2_find_profile(des.idVendor, des.idProduct);
				if (p2)
					prof = p2;
			}
			has_fw = 1;
		}

		if (sr_usb_device_is_exists(live ? live : devlist[i])) {
			if (live)
				libusb_unref_device(live);
			continue;
		}

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc) {
			if (live)
				libusb_unref_device(live);
			break;
		}
		devc->usb_dev = live ? live : libusb_ref_device(devlist[i]);
		bus = libusb_get_bus_number(devc->usb_dev);
		address = libusb_get_device_address(devc->usb_dev);
		devc->sr_ctx = drvc->sr_ctx;
		devc->profile = prof;
		devc->num_channels = (prof->caps & DEV_CAPS_16BIT) ? 16 : 8;
		devc->sample_wide = (prof->caps & DEV_CAPS_16BIT) ? 1 : 0;
		devc->samplerate = SR_MHZ(1);
		devc->limit_samples = DEFAULT_SAMPLES;
		devc->max_height = 1;
		devc->op_mode = LO_OP_STREAM;
		devc->fw_loaded = has_fw;

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE,
				      prof->vendor, prof->model, "fx2lafw");
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
		usb_info = sr_usb_dev_inst_new(bus, address);
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		}
		setup_channels(sdi, devc->num_channels);
		sr_info("Found %s %04x:%04x fw=ok bus=%u addr=%u",
			prof->model, des.idVendor, des.idProduct, bus, address);
		devices = g_slist_append(devices, sdi);
	}
	libusb_free_device_list(devlist, 0);
	return devices;
}

static const GSList *hw_dev_mode_list(const struct sr_dev_inst *sdi)
{
	GSList *l = NULL;
	(void)sdi;
	l = g_slist_append(l, (gpointer)&sr_mode_list[0]);
	return l;
}

static int fx2_open_handle(struct fx2_context *devc)
{
	int r;
	struct version_info vi;

	r = libusb_open(devc->usb_dev, &devc->devhdl);
	if (r != 0) {
		sr_err("open failed: %s", libusb_error_name(r));
		return SR_ERR;
	}
	libusb_set_auto_detach_kernel_driver(devc->devhdl, 1);
	if (libusb_kernel_driver_active(devc->devhdl, USB_INTERFACE) == 1)
		libusb_detach_kernel_driver(devc->devhdl, USB_INTERFACE);
	r = libusb_set_configuration(devc->devhdl, USB_CONFIGURATION);
	if (r != 0 && r != LIBUSB_ERROR_BUSY)
		sr_warn("set_configuration: %s", libusb_error_name(r));
	r = libusb_claim_interface(devc->devhdl, USB_INTERFACE);
	if (r != 0) {
		sr_err("claim failed: %s", libusb_error_name(r));
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}
	if (fx2_get_fw_version(devc->devhdl, &vi) != SR_OK ||
	    vi.major != FX2LAFW_REQUIRED_VERSION_MAJOR) {
		sr_err("fx2lafw not running (need %d.x)",
		       FX2LAFW_REQUIRED_VERSION_MAJOR);
		libusb_release_interface(devc->devhdl, USB_INTERFACE);
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
		return SR_ERR;
	}
	sr_info("firmware %u.%u", vi.major, vi.minor);
	return SR_OK;
}

static int fx2_rediscover(struct fx2_context *devc)
{
	struct drv_context *drvc = di->priv;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	int i, found = 0;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return SR_ERR;
	for (i = 0; devlist[i]; i++) {
		const struct fx2_profile *p;
		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		p = fx2_find_profile(des.idVendor, des.idProduct);
		if (!p)
			continue;
		/* after C0 RAM upload, nanoDLA stays 1D50:608C / 608D */
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		devc->usb_dev = libusb_ref_device(devlist[i]);
		devc->profile = p;
		devc->num_channels = (p->caps & DEV_CAPS_16BIT) ? 16 : 8;
		devc->sample_wide = (p->caps & DEV_CAPS_16BIT) ? 1 : 0;
		found = 1;
		break;
	}
	libusb_free_device_list(devlist, 1);
	return found ? SR_OK : SR_ERR;
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	struct fx2_context *devc = sdi->priv;
	int ret;

	if (sdi->status == SR_ST_ACTIVE)
		return SR_OK;

	if (devc->fw_updated > 0) {
		int64_t waited = 0;
		sr_info("waiting for FX2 re-enumeration");
		g_usleep(300 * 1000);
		while (waited < MAX_RENUM_DELAY_MS) {
			if (fx2_rediscover(devc) == SR_OK &&
			    fx2_open_handle(devc) == SR_OK) {
				sdi->handle = (ds_device_handle)devc->usb_dev;
				if (sdi->conn) {
					struct sr_usb_dev_inst *u = sdi->conn;
					u->usb_dev = devc->usb_dev;
					u->bus = libusb_get_bus_number(devc->usb_dev);
					u->address = libusb_get_device_address(devc->usb_dev);
				}
				break;
			}
			if (devc->devhdl) {
				libusb_close(devc->devhdl);
				devc->devhdl = NULL;
			}
			g_usleep(100 * 1000);
			waited += 100;
		}
		if (!devc->devhdl) {
			sr_err("device did not come back after firmware upload");
			ds_set_last_error(SR_ERR_DEVICE_USB_IO_ERROR);
			return SR_ERR;
		}
		devc->fw_updated = 0;
	} else {
		ret = fx2_open_handle(devc);
		if (ret != SR_OK) {
			ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
			return ret;
		}
	}

	devc->sdi = sdi;
	fx2_clamp_rate(devc);
	sdi->status = SR_ST_ACTIVE;
	sr_info("open ok %s ch=%d",
		devc->profile ? devc->profile->model : "fx2",
		devc->num_channels);
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct fx2_context *devc;
	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	if (devc->devhdl) {
		libusb_release_interface(devc->devhdl, USB_INTERFACE);
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct fx2_context *devc;
	if (!sdi)
		return SR_ERR;
	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		g_free(devc->raw_pending);
		g_free(devc->cross_buf);
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

SR_PRIV void fx2lafw_on_usb_reconnected(struct sr_dev_inst *sdi,
					struct libusb_device *new_dev)
{
	struct fx2_context *devc;
	struct libusb_device_descriptor des;
	const struct fx2_profile *p;

	if (!sdi || !sdi->priv || !new_dev)
		return;
	if (!sdi->driver || !sdi->driver->name ||
	    strcmp(sdi->driver->name, "fx2lafw") != 0)
		return;
	devc = sdi->priv;
	if (libusb_get_device_descriptor(new_dev, &des) == 0) {
		p = fx2_find_profile(des.idVendor, des.idProduct);
		if (p)
			devc->profile = p;
	}
	if (devc->usb_dev)
		libusb_unref_device(devc->usb_dev);
	devc->usb_dev = libusb_ref_device(new_dev);
	devc->sdi = sdi;
	sdi->handle = (ds_device_handle)devc->usb_dev;
	sr_info("reconnected %04x:%04x", des.idVendor, des.idProduct);
}

static int hw_dev_status_get(const struct sr_dev_inst *sdi,
			     struct sr_status *status, gboolean prg)
{
	(void)sdi;
	(void)prg;
	if (!status)
		return SR_ERR;
	memset(status, 0, sizeof(*status));
	return SR_OK;
}

static int config_get(int id, GVariant **data, const struct sr_dev_inst *sdi,
		      const struct sr_channel *ch,
		      const struct sr_channel_group *cg)
{
	struct fx2_context *devc;
	(void)ch;
	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
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
		*data = g_variant_new_byte(devc->max_height);
		break;
	case SR_CONF_HW_DEPTH:
		*data = g_variant_new_uint64(HW_DEPTH);
		break;
	case SR_CONF_TOTAL_CH_NUM:
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int16((int16_t)devc->num_channels);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16(devc->op_mode);
		break;
	case SR_CONF_LOOP_MODE:
		*data = g_variant_new_boolean(devc->is_loop != 0);
		break;
	case SR_CONF_USB30_SUPPORT:
		*data = g_variant_new_boolean(FALSE);
		break;
	case SR_CONF_RLE:
	case SR_CONF_RLE_SUPPORT:
		*data = g_variant_new_boolean(FALSE);
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
	struct fx2_context *devc;
	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_DEVICE_MODE:
		if (g_variant_get_int16(data) != LOGIC)
			return SR_ERR_ARG;
		sdi->mode = LOGIC;
		break;
	case SR_CONF_SAMPLERATE:
		devc->samplerate = g_variant_get_uint64(data);
		fx2_clamp_rate(devc);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_EN:
		if (ch)
			ch->enabled = g_variant_get_boolean(data);
		break;
	case SR_CONF_MAX_HEIGHT: {
		const char *s = g_variant_get_string(data, NULL);
		int i;
		for (i = 0; i < (int)ARRAY_SIZE(maxHeights); i++) {
			if (s && strcmp(s, maxHeights[i]) == 0) {
				devc->max_height = i;
				break;
			}
		}
		break;
	}
	case SR_CONF_MAX_HEIGHT_VALUE:
		devc->max_height = g_variant_get_byte(data);
		if (devc->max_height > 4)
			devc->max_height = 4;
		break;
	case SR_CONF_OPERATION_MODE:
		devc->op_mode = LO_OP_STREAM;
		break;
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
	static uint64_t rate_buf[20];
	int n = 0, i;
	struct fx2_context *devc =
		(sdi && sdi->priv) ? (struct fx2_context *)sdi->priv : NULL;
	uint64_t maxr = SR_MHZ(24);

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
		if (devc)
			maxr = fx2_max_rate(devc);
		for (i = 0; i < (int)ARRAY_SIZE(samplerates); i++) {
			if (samplerates[i] <= maxr)
				rate_buf[n++] = samplerates[i];
		}
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_from_data(
			G_VARIANT_TYPE("at"), rate_buf,
			(gsize)n * sizeof(uint64_t), TRUE, NULL, NULL);
		g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_strv(maxHeights, ARRAY_SIZE(maxHeights));
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
	struct fx2_context *devc = sdi->priv;
	const struct libusb_pollfd **lupfd;
	struct drv_context *drvc = di->priv;
	unsigned int i;
	int ret;

	(void)cb_data;
	if (sdi->status != SR_ST_ACTIVE) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	devc->sdi = sdi;
	devc->num_samples = 0;
	devc->abort = 0;
	devc->status = ST_START;
	devc->freewheel = 0;
	devc->raw_pending_len = 0;
	devc->stream_res_len = 0;
	fx2_refresh_enabled(devc);
	fx2_clamp_rate(devc);

	sr_info("acq start ch=%d wide=%d rate=%" PRIu64 " limit=%" PRIu64,
		devc->en_count, devc->sample_wide,
		devc->samplerate, devc->limit_samples);

	if ((ret = start_transfers(devc)) != SR_OK)
		return ret;

	lupfd = libusb_get_pollfds(drvc->sr_ctx->libusb_ctx);
	if (!lupfd) {
		devc->freewheel = 1;
		sr_session_source_add(-1, 0, 0, receive_data, sdi);
	} else {
		for (i = 0; lupfd[i]; i++)
			sr_source_add(lupfd[i]->fd, lupfd[i]->events,
				      20, receive_data, sdi);
		g_free((void *)lupfd);
	}

	if ((ret = fx2_cmd_start(devc)) != SR_OK) {
		devc->abort = 1;
		fx2_cancel_transfers(devc);
		return ret;
	}
	std_session_send_df_header(sdi, LOG_PREFIX);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct fx2_context *devc;
	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	if (!devc->abort) {
		devc->abort = 1;
		devc->status = ST_STOP;
		fx2_cancel_transfers(devc);
		if (devc->submitted_transfers <= 0)
			finish_acquisition(devc);
	}
	return SR_OK;
}

SR_PRIV struct sr_dev_driver fx2lafw_driver_info = {
	.name = "fx2lafw",
	.longname = "nanoDLA (fx2lafw)",
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
