/*
 * Sipeed SLogic16U3 driver for DSView / libsigrok4DSL
 *
 * USB protocol (Sipeed libsigrok slogic-dev):
 *   VID 0x359F  PID 0x3031
 *   EP0 vendor control: 32-bit register read/write
 *     bRequest 0x00 REG_READ  / 0x01 REG_WRITE
 *     wValue   register address (byte), 4 bytes per transfer
 *   Registers:
 *     0x0004 R32_CTRL  bit0=RUN  bit1=RST
 *     0x000C R32_AUX   mailbox header + payload at 0x0010
 *   AUX commands: 1=channel mask  2=samplerate  3=vref  5=test mode
 *   EP 0x82 Bulk IN: packed sample stream (no header)
 *
 * DSView expects LA_CROSS_DATA (64-sample channel planes).
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "slogic16u3.h"

#undef LOG_PREFIX
#define LOG_PREFIX "slogic16u3: "

#define SLOGIC_VID               0x359F
#define SLOGIC_PID_16U3          0x3031

#define SLOGIC_EP_IN             0x82

#define SLOGIC_REQ_REG_READ      0x00
#define SLOGIC_REQ_REG_WRITE     0x01

#define SLOGIC_R32_CTRL          0x0004
#define SLOGIC_R32_AUX           0x000C

#define SLOGIC_CTRL_STOP         0x00000000u
#define SLOGIC_CTRL_RUN          0x00000001u
#define SLOGIC_CTRL_RST          0x00000002u

#define SLOGIC_AUX_CMD_CHANNEL   0x00000001u
#define SLOGIC_AUX_CMD_RATE      0x00000002u
#define SLOGIC_AUX_CMD_VREF      0x00000003u

#define SLOGIC_CTRL_TIMEOUT_MS   500
#define SLOGIC_BULK_TIMEOUT_MS   500
#define SLOGIC_NUM_TRANSFERS     12
#define SLOGIC_TRANSFER_SIZE     (256 * 1024)
#define SLOGIC_DROP_FIRST_BYTES  4

#define SLOGIC_DEFAULT_SAMPLES   SR_Mn(1)
#define SLOGIC_HW_DEPTH          SR_Mn(128)
#define SLOGIC_MAX_PHYS_CH       16

#define SLOGIC_CROSS_SCALE       64
#define SLOGIC_CROSS_PLANE_BYTES 8

#define SLOGIC_VTH_MIN           0.0
#define SLOGIC_VTH_MAX           5.0
#define SLOGIC_VTH_DEFAULT       1.7

enum {
	SLOGIC_ST_IDLE = 0,
	SLOGIC_ST_START,
	SLOGIC_ST_DATA,
	SLOGIC_ST_STOP,
	SLOGIC_ST_FINISH,
};

enum {
	SLOGIC_CHMODE_16 = 0,
	SLOGIC_CHMODE_8 = 1,
	SLOGIC_CHMODE_4 = 2,
};

static const uint64_t slogic_ui_rates[] = {
	SR_MHZ(800),
	SR_MHZ(400),
	SR_MHZ(200),
	SR_MHZ(160),
	SR_MHZ(100),
	SR_MHZ(80),
	SR_MHZ(50),
	SR_MHZ(40),
	SR_MHZ(32),
	SR_MHZ(25),
	SR_MHZ(20),
	SR_MHZ(16),
	SR_MHZ(10),
	SR_MHZ(8),
	SR_MHZ(5),
};

static const struct sr_list_item filter_list[] = {
	{ SR_FILTER_NONE, "None" },
	{ SR_FILTER_1T, "1 Sample Clock" },
	{ -1, NULL },
};

static const struct sr_list_item opmode_list[] = {
	{ LO_OP_STREAM, "Stream Mode" },
	{ -1, NULL },
};

static struct sr_list_item channel_mode_list[] = {
	{ SLOGIC_CHMODE_16, "Use Channels 0~15 (Max 200MHz)" },
	{ SLOGIC_CHMODE_8, "Use Channels 0~7 (Max 400MHz)" },
	{ SLOGIC_CHMODE_4, "Use Channels 0~3 (Max 800MHz)" },
	{ -1, NULL },
};

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_FILTER,
	SR_CONF_VTH,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_CHANNEL_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
	SR_CONF_VTH,
	SR_CONF_FILTER,
};

static const char *probe_names[] = {
	"0", "1", "2", "3", "4", "5", "6", "7",
	"8", "9", "10", "11", "12", "13", "14", "15",
	NULL,
};

struct slogic_context {
	struct libusb_device *usb_dev;
	struct libusb_device_handle *devhdl;
	struct sr_context *sr_ctx;
	const struct sr_dev_inst *sdi;

	int channel_count;
	int ch_mode;
	int filter;
	int max_height;
	int op_mode;
	int is_loop;
	double vth;

	uint64_t samplerate;
	uint64_t limit_samples;
	uint64_t num_samples;
	uint64_t num_bytes;

	enum libusb_speed usb_speed;

	uint16_t filt_prev_in;
	uint16_t filt_prev_out;
	int filt_have_prev;

	int status;
	int abort;
	int fw_streaming;
	int submitted_transfers;
	int num_transfers;
	struct libusb_transfer **transfers;
	int freewheel;

	int drop_left;

	uint16_t *raw_pending;
	int raw_pending_len;
	int raw_pending_cap;

	uint8_t stream_res[2];
	int stream_res_len;

	uint8_t *cross_buf;
	int cross_cap;

	uint8_t en_bits[SLOGIC_MAX_PHYS_CH];
	int en_count;
};

SR_PRIV struct sr_dev_driver slogic16u3_driver_info;
static struct sr_dev_driver *di = &slogic16u3_driver_info;

/* -------------------- USB / speed -------------------- */

static int slogic_is_usb3(const struct slogic_context *devc)
{
	return devc && (devc->usb_speed == LIBUSB_SPEED_SUPER ||
			devc->usb_speed == LIBUSB_SPEED_SUPER_PLUS);
}

static const char *slogic_speed_name(enum libusb_speed sp)
{
	switch (sp) {
	case LIBUSB_SPEED_LOW: return "LOW";
	case LIBUSB_SPEED_FULL: return "FULL";
	case LIBUSB_SPEED_HIGH: return "HIGH(USB2.0)";
	case LIBUSB_SPEED_SUPER: return "SUPER(USB3.0)";
	case LIBUSB_SPEED_SUPER_PLUS: return "SUPER+(USB3.x)";
	default: return "UNKNOWN";
	}
}

static int slogic_mode_channels(int mode)
{
	if (mode == SLOGIC_CHMODE_4)
		return 4;
	if (mode == SLOGIC_CHMODE_8)
		return 8;
	return 16;
}

/*
 * USB3: Sipeed spec 800/400/200 (Windows libusb typically half).
 * USB2 HS: ~40 MB/s → 80/40/20.
 */
static uint64_t slogic_link_max_rate(const struct slogic_context *devc)
{
	int nch = devc ? devc->channel_count : 16;
	int usb3 = slogic_is_usb3(devc);

#ifdef _WIN32
	if (usb3) {
		if (nch <= 4)
			return SR_MHZ(400);
		if (nch <= 8)
			return SR_MHZ(200);
		return SR_MHZ(100);
	}
#else
	if (usb3) {
		if (nch <= 4)
			return SR_MHZ(800);
		if (nch <= 8)
			return SR_MHZ(400);
		return SR_MHZ(200);
	}
#endif
	if (nch <= 4)
		return SR_MHZ(80);
	if (nch <= 8)
		return SR_MHZ(40);
	return SR_MHZ(20);
}

static void slogic_apply_model_name(struct sr_dev_inst *sdi,
				    enum libusb_speed sp)
{
	const char *model;

	if (!sdi)
		return;
	if (sp == LIBUSB_SPEED_SUPER || sp == LIBUSB_SPEED_SUPER_PLUS)
		model = "SLogic16U3 USB3.0";
	else if (sp == LIBUSB_SPEED_HIGH)
		model = "SLogic16U3 USB2.0";
	else
		model = "SLogic16U3";
	g_free(sdi->name);
	sdi->name = g_strdup(model);
}

static void slogic_update_usb_speed(struct slogic_context *devc)
{
	enum libusb_speed sp = LIBUSB_SPEED_UNKNOWN;

	if (!devc || !devc->usb_dev)
		return;
	sp = libusb_get_device_speed(devc->usb_dev);
	devc->usb_speed = sp;
	if (devc->sdi)
		slogic_apply_model_name((struct sr_dev_inst *)devc->sdi, sp);
}

static uint64_t slogic_pick_rate(uint64_t want, uint64_t link_max)
{
	unsigned int i;
	uint64_t best = SR_MHZ(5);
	uint64_t best_err = UINT64_MAX;

	if (want == 0)
		want = SR_MHZ(100);
	if (want > link_max)
		want = link_max;

	for (i = 0; i < ARRAY_SIZE(slogic_ui_rates); i++) {
		uint64_t r = slogic_ui_rates[i];
		uint64_t err;

		if (r > link_max)
			continue;
		err = (r > want) ? (r - want) : (want - r);
		if (err < best_err) {
			best_err = err;
			best = r;
		}
	}
	return best;
}

static void slogic_map_samplerate(struct slogic_context *devc, uint64_t want)
{
	devc->samplerate = slogic_pick_rate(want, slogic_link_max_rate(devc));
}

static int slogic_rate_list_add(uint64_t *out, int n, int cap, uint64_t r)
{
	int i, j;

	if (n >= cap)
		return n;
	for (i = 0; i < n; i++) {
		if (out[i] == r)
			return n;
		if (out[i] < r)
			break;
	}
	for (j = n; j > i; j--)
		out[j] = out[j - 1];
	out[i] = r;
	return n + 1;
}

static void slogic_build_rate_list(const struct slogic_context *devc,
				   uint64_t *out, int *out_n)
{
	uint64_t link_max = slogic_link_max_rate(devc);
	unsigned int i;
	int n = 0;
	const int cap = 32;

	for (i = 0; i < ARRAY_SIZE(slogic_ui_rates); i++) {
		if (slogic_ui_rates[i] <= link_max)
			n = slogic_rate_list_add(out, n, cap, slogic_ui_rates[i]);
	}
	out[n] = 0;
	*out_n = n;
}

/* -------------------- register / AUX -------------------- */

static int slogic_ctrl_xfer(struct slogic_context *devc, int is_read,
			    uint16_t addr, uint8_t *data, size_t len)
{
	int ret;
	size_t i;
	uint8_t bm;

	if (!devc || !devc->devhdl || !data)
		return SR_ERR_ARG;

	len = (len + 3) & ~(size_t)3;
	bm = (uint8_t)(LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE |
		       (is_read ? LIBUSB_ENDPOINT_IN : LIBUSB_ENDPOINT_OUT));

	for (i = 0; i < len; i += 4) {
		ret = libusb_control_transfer(
			devc->devhdl, bm,
			is_read ? SLOGIC_REQ_REG_READ : SLOGIC_REQ_REG_WRITE,
			(uint16_t)(addr + i), 0, data + i, 4,
			SLOGIC_CTRL_TIMEOUT_MS);
		if (ret < 0) {
			sr_err("ctrl %s addr=0x%04x failed: %s",
			       is_read ? "read" : "write",
			       (unsigned)(addr + i), libusb_error_name(ret));
			return SR_ERR;
		}
	}
	return SR_OK;
}

static int slogic_wr32(struct slogic_context *devc, uint16_t addr, uint32_t v)
{
	uint8_t b[4];

	b[0] = (uint8_t)(v);
	b[1] = (uint8_t)(v >> 8);
	b[2] = (uint8_t)(v >> 16);
	b[3] = (uint8_t)(v >> 24);
	return slogic_ctrl_xfer(devc, 0, addr, b, 4);
}

static int slogic_rd32(struct slogic_context *devc, uint16_t addr, uint32_t *out)
{
	uint8_t b[4];
	int ret;

	ret = slogic_ctrl_xfer(devc, 1, addr, b, 4);
	if (ret != SR_OK)
		return ret;
	*out = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
	       ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
	return SR_OK;
}

static int slogic_aux_transact(struct slogic_context *devc, uint32_t cmd,
			       uint8_t *payload, size_t payload_cap,
			       size_t *payload_len)
{
	uint32_t h;
	int retry;
	size_t n;

	if (slogic_wr32(devc, SLOGIC_R32_AUX, cmd) != SR_OK)
		return SR_ERR;

	for (retry = 0; retry < 8; retry++) {
		if (slogic_rd32(devc, SLOGIC_R32_AUX, &h) != SR_OK)
			return SR_ERR;
		if ((h >> 16) & 1u)
			break;
	}
	if (!((h >> 16) & 1u)) {
		sr_err("AUX cmd %u timeout (hdr=0x%08x)", cmd, h);
		return SR_ERR;
	}

	n = (size_t)((h & 0xffffu) >> 9);
	if (n > payload_cap)
		n = payload_cap;
	if (n & 3)
		n = (n + 3) & ~(size_t)3;
	if (n == 0)
		n = 4;

	memset(payload, 0, payload_cap);
	if (slogic_ctrl_xfer(devc, 1, SLOGIC_R32_AUX + 4, payload, n) != SR_OK)
		return SR_ERR;
	if (payload_len)
		*payload_len = n;
	return SR_OK;
}

static int slogic_aux_write_payload(struct slogic_context *devc,
				    uint8_t *payload, size_t n)
{
	if (n & 3)
		n = (n + 3) & ~(size_t)3;
	return slogic_ctrl_xfer(devc, 0, SLOGIC_R32_AUX + 4, payload, n);
}

static int slogic_reset(struct slogic_context *devc)
{
	int ret;

	ret = slogic_wr32(devc, SLOGIC_R32_CTRL, SLOGIC_CTRL_RST);
	if (ret != SR_OK)
		return ret;
	return slogic_wr32(devc, SLOGIC_R32_CTRL, SLOGIC_CTRL_STOP);
}

static uint32_t slogic_vth_to_dac(double v)
{
	if (v < 0.0)
		v = 0.0;
	if (v > 6.0)
		v = 6.0;
	/* Official: dac = V / 3.33 / 2 * 1024  (~10-bit, 1.6 V ref) */
	return (uint32_t)(v / 3.33 / 2.0 * 1024.0 + 0.5);
}

static int slogic_apply_channel(struct slogic_context *devc)
{
	uint8_t pay[16];
	size_t n = 0;
	uint32_t mask = (devc->channel_count >= 32)
		? 0xffffffffu
		: ((1u << devc->channel_count) - 1u);

	if (slogic_aux_transact(devc, SLOGIC_AUX_CMD_CHANNEL, pay,
				sizeof(pay), &n) != SR_OK)
		return SR_ERR;
	pay[0] = (uint8_t)(mask);
	pay[1] = (uint8_t)(mask >> 8);
	pay[2] = (uint8_t)(mask >> 16);
	pay[3] = (uint8_t)(mask >> 24);
	if (n < 4)
		n = 4;
	if (slogic_aux_write_payload(devc, pay, n) != SR_OK)
		return SR_ERR;
	sr_info("AUX channel mask=0x%08x nch=%d", mask, devc->channel_count);
	return SR_OK;
}

static int slogic_apply_samplerate(struct slogic_context *devc)
{
	uint8_t pay[16];
	size_t n = 0;
	int tries;

	if (slogic_aux_transact(devc, SLOGIC_AUX_CMD_RATE, pay,
				sizeof(pay), &n) != SR_OK)
		return SR_ERR;
	if (n < 8)
		n = 8;

	for (tries = 0; tries < 2; tries++) {
		uint16_t idx, base_mhz;
		uint64_t base, want;
		uint32_t divm1;

		if (slogic_ctrl_xfer(devc, 1, SLOGIC_R32_AUX + 4, pay, n) != SR_OK)
			return SR_ERR;
		idx = (uint16_t)(pay[0] | (pay[1] << 8));
		base_mhz = (uint16_t)(pay[2] | (pay[3] << 8));
		base = (uint64_t)base_mhz * SR_MHZ(1);
		want = devc->samplerate;
		if (base == 0) {
			sr_err("AUX samplerate base=0");
			return SR_ERR;
		}
		if (base % want != 0) {
			idx++;
			pay[0] = (uint8_t)idx;
			pay[1] = (uint8_t)(idx >> 8);
			if (slogic_aux_write_payload(devc, pay, 4) != SR_OK)
				return SR_ERR;
			continue;
		}
		divm1 = (uint32_t)(base / want - 1);
		pay[4] = (uint8_t)(divm1);
		pay[5] = (uint8_t)(divm1 >> 8);
		pay[6] = (uint8_t)(divm1 >> 16);
		pay[7] = (uint8_t)(divm1 >> 24);
		if (slogic_aux_write_payload(devc, pay, n) != SR_OK)
			return SR_ERR;
		sr_info("AUX rate want=%" PRIu64 " base=%u MHz div=%u idx=%u",
			want, base_mhz, divm1 + 1, idx);
		return SR_OK;
	}
	sr_err("failed to map samplerate %" PRIu64, devc->samplerate);
	return SR_ERR;
}

static int slogic_apply_vth(struct slogic_context *devc)
{
	uint8_t pay[16];
	size_t n = 0;
	uint32_t dac = slogic_vth_to_dac(devc->vth);

	if (slogic_aux_transact(devc, SLOGIC_AUX_CMD_VREF, pay,
				sizeof(pay), &n) != SR_OK)
		return SR_ERR;
	pay[0] = (uint8_t)(dac);
	pay[1] = (uint8_t)(dac >> 8);
	pay[2] = (uint8_t)(dac >> 16);
	pay[3] = (uint8_t)(dac >> 24);
	if (n < 4)
		n = 4;
	if (slogic_aux_write_payload(devc, pay, n) != SR_OK)
		return SR_ERR;
	sr_info("AUX vth=%.2f V dac=%u", devc->vth, dac);
	return SR_OK;
}

static int slogic_hw_start(struct slogic_context *devc)
{
	return slogic_wr32(devc, SLOGIC_R32_CTRL, SLOGIC_CTRL_RUN);
}

static void slogic_hw_stop(struct slogic_context *devc)
{
	if (!devc || !devc->fw_streaming || !devc->devhdl)
		return;
	if (slogic_wr32(devc, SLOGIC_R32_CTRL, SLOGIC_CTRL_STOP) != SR_OK) {
		sr_err("HW stop failed, will retry");
		return;
	}
	devc->fw_streaming = 0;
	sr_info("HW stop sent");
}

static void slogic_drain_ep(struct slogic_context *devc)
{
	uint8_t *tmp;
	int xfer;
	int r;
	int loops = 0;

	if (!devc || !devc->devhdl)
		return;
	tmp = g_try_malloc(64 * 1024);
	if (!tmp)
		return;
	do {
		xfer = 0;
		r = libusb_bulk_transfer(devc->devhdl, SLOGIC_EP_IN, tmp,
					 64 * 1024, &xfer, 50);
		loops++;
	} while (r == 0 && xfer > 0 && loops < 32);
	g_free(tmp);
}

/* -------------------- probes / convert -------------------- */

static void slogic_apply_channel_mode(struct sr_dev_inst *sdi)
{
	struct slogic_context *devc = sdi->priv;
	GSList *l;
	int n = slogic_mode_channels(devc->ch_mode);

	devc->channel_count = n;
	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		ch->enabled = (ch->index < n) ? TRUE : FALSE;
	}
}

static void slogic_refresh_enabled_bits(struct slogic_context *devc)
{
	GSList *l;
	int n = 0;
	int max_bit = devc->channel_count;

	for (l = devc->sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (!ch->enabled)
			continue;
		if (ch->index >= max_bit)
			continue;
		if (n >= SLOGIC_MAX_PHYS_CH)
			break;
		devc->en_bits[n++] = (uint8_t)ch->index;
	}
	if (n == 0) {
		int i;
		for (i = 0; i < max_bit; i++)
			devc->en_bits[i] = (uint8_t)i;
		n = max_bit;
	}
	devc->en_count = n;
}

static int slogic_ensure_cross_cap(struct slogic_context *devc, int need)
{
	uint8_t *p;

	if (need <= devc->cross_cap)
		return SR_OK;
	need = (need + 4095) & ~4095;
	p = g_try_realloc(devc->cross_buf, need);
	if (!p)
		return SR_ERR_MALLOC;
	devc->cross_buf = p;
	devc->cross_cap = need;
	return SR_OK;
}

static int slogic_ensure_pending_cap(struct slogic_context *devc, int need)
{
	uint16_t *p;

	if (need <= devc->raw_pending_cap)
		return SR_OK;
	need = (need + 1023) & ~1023;
	p = g_try_realloc(devc->raw_pending, (gsize)need * sizeof(uint16_t));
	if (!p)
		return SR_ERR_MALLOC;
	devc->raw_pending = p;
	devc->raw_pending_cap = need;
	return SR_OK;
}

static uint16_t slogic_apply_1t_filter(struct slogic_context *devc, uint16_t in)
{
	uint16_t outv;

	if (devc->filter != SR_FILTER_1T)
		return in;
	if (!devc->filt_have_prev) {
		devc->filt_prev_in = in;
		devc->filt_prev_out = in;
		devc->filt_have_prev = 1;
		return in;
	}
	outv = (in == devc->filt_prev_in) ? in : devc->filt_prev_out;
	devc->filt_prev_in = in;
	devc->filt_prev_out = outv;
	return outv;
}

static int slogic_push_sample(struct slogic_context *devc, uint16_t s)
{
	if (slogic_ensure_pending_cap(devc, devc->raw_pending_len + 1) != SR_OK)
		return SR_ERR_MALLOC;
	devc->raw_pending[devc->raw_pending_len++] =
		slogic_apply_1t_filter(devc, s);
	return SR_OK;
}

/*
 * Unpack SLogic wire format into uint16 parallel samples (bit i = Di).
 *   16ch: LE u16 per sample
 *    8ch: 1 byte per sample
 *    4ch: 2 samples/byte, low nibble first
 */
static int slogic_unpack_append(struct slogic_context *devc,
				const uint8_t *raw, int raw_len)
{
	int nch = devc->channel_count;
	const uint8_t *p = raw;
	int left = raw_len;

	if (!raw || raw_len <= 0)
		return SR_OK;

	if (nch >= 16) {
		if (devc->stream_res_len == 1 && left > 0) {
			uint16_t s = (uint16_t)devc->stream_res[0] |
				     ((uint16_t)p[0] << 8);
			if (slogic_push_sample(devc, s) != SR_OK)
				return SR_ERR_MALLOC;
			p++;
			left--;
			devc->stream_res_len = 0;
		}
		while (left >= 2) {
			uint16_t s = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
			if (slogic_push_sample(devc, s) != SR_OK)
				return SR_ERR_MALLOC;
			p += 2;
			left -= 2;
		}
		if (left == 1) {
			devc->stream_res[0] = p[0];
			devc->stream_res_len = 1;
		}
		return SR_OK;
	}

	if (nch <= 4) {
		while (left > 0) {
			uint8_t b = *p++;
			left--;
			if (slogic_push_sample(devc, (uint16_t)(b & 0x0f)) != SR_OK)
				return SR_ERR_MALLOC;
			if (slogic_push_sample(devc, (uint16_t)((b >> 4) & 0x0f)) != SR_OK)
				return SR_ERR_MALLOC;
		}
		return SR_OK;
	}

	/* 8ch */
	while (left > 0) {
		if (slogic_push_sample(devc, (uint16_t)*p++) != SR_OK)
			return SR_ERR_MALLOC;
		left--;
	}
	return SR_OK;
}

static int slogic_convert_push(struct slogic_context *devc,
			       const uint8_t *raw, int raw_len)
{
	const int en = devc->en_count;
	const int cross_group = SLOGIC_CROSS_PLANE_BYTES * en;
	int groups, g, c, i, consumed, left;
	uint8_t *out;
	int out_len;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	uint64_t samples_out;

	if (en < 1 || raw_len <= 0)
		return SR_OK;
	if (slogic_unpack_append(devc, raw, raw_len) != SR_OK)
		return SR_ERR_MALLOC;

	groups = devc->raw_pending_len / SLOGIC_CROSS_SCALE;
	if (groups <= 0)
		return SR_OK;

	if (devc->limit_samples && !devc->is_loop) {
		uint64_t room = (devc->num_samples < devc->limit_samples)
			? (devc->limit_samples - devc->num_samples) : 0;
		int max_groups = (int)(room / SLOGIC_CROSS_SCALE);
		if (max_groups <= 0) {
			devc->num_samples = devc->limit_samples;
			return SR_OK;
		}
		if (groups > max_groups)
			groups = max_groups;
	}

	out_len = groups * cross_group;
	if (slogic_ensure_cross_cap(devc, out_len) != SR_OK)
		return SR_ERR_MALLOC;

	out = devc->cross_buf;
	for (g = 0; g < groups; g++) {
		const uint16_t *blk = devc->raw_pending + g * SLOGIC_CROSS_SCALE;
		for (c = 0; c < en; c++) {
			uint64_t plane = 0;
			uint8_t bit = devc->en_bits[c];
			for (i = 0; i < SLOGIC_CROSS_SCALE; i++) {
				if ((blk[i] >> bit) & 1u)
					plane |= (1ULL << i);
			}
			out[0] = (uint8_t)(plane);
			out[1] = (uint8_t)(plane >> 8);
			out[2] = (uint8_t)(plane >> 16);
			out[3] = (uint8_t)(plane >> 24);
			out[4] = (uint8_t)(plane >> 32);
			out[5] = (uint8_t)(plane >> 40);
			out[6] = (uint8_t)(plane >> 48);
			out[7] = (uint8_t)(plane >> 56);
			out += SLOGIC_CROSS_PLANE_BYTES;
		}
	}

	samples_out = (uint64_t)groups * SLOGIC_CROSS_SCALE;

	memset(&packet, 0, sizeof(packet));
	memset(&logic, 0, sizeof(logic));
	packet.type = SR_DF_LOGIC;
	packet.status = SR_PKT_OK;
	packet.payload = &logic;
	logic.length = (uint64_t)out_len;
	logic.format = LA_CROSS_DATA;
	logic.data = devc->cross_buf;
	ds_data_forward(devc->sdi, &packet);

	devc->num_samples += samples_out;
	devc->num_bytes += (uint64_t)out_len;

	consumed = groups * SLOGIC_CROSS_SCALE;
	left = devc->raw_pending_len - consumed;
	if (left > 0)
		memmove(devc->raw_pending, devc->raw_pending + consumed,
			(size_t)left * sizeof(uint16_t));
	devc->raw_pending_len = left;
	return SR_OK;
}

/* -------------------- transfers -------------------- */

static void slogic_cancel_transfers(struct slogic_context *devc)
{
	unsigned int i;

	if (!devc || !devc->transfers)
		return;
	for (i = 0; i < (unsigned)devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void finish_acquisition(struct slogic_context *devc)
{
	struct sr_datafeed_packet packet;

	if (!devc || devc->status == SLOGIC_ST_FINISH)
		return;

	devc->raw_pending_len = 0;
	devc->abort = 1;
	slogic_hw_stop(devc);

	sr_info("send SR_DF_END, samples=%" PRIu64, devc->num_samples);
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
	devc->status = SLOGIC_ST_FINISH;
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct slogic_context *devc = transfer->user_data;
	unsigned int i;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	if (!devc->transfers)
		return;
	for (i = 0; i < (unsigned)devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}
	devc->submitted_transfers--;
	if (devc->submitted_transfers <= 0)
		finish_acquisition(devc);
}

static void slogic_request_acq_stop(struct slogic_context *devc)
{
	if (!devc || devc->abort)
		return;
	devc->abort = 1;
	devc->status = SLOGIC_ST_STOP;
	slogic_cancel_transfers(devc);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct slogic_context *devc = transfer->user_data;
	const uint8_t *buf;
	int len;
	int r;

	if (devc->status == SLOGIC_ST_START)
		devc->status = SLOGIC_ST_DATA;
	if (devc->abort)
		devc->status = SLOGIC_ST_STOP;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED &&
	    transfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
		if (!devc->abort)
			sr_err("transfer status %d", transfer->status);
		devc->status = SLOGIC_ST_STOP;
	}

	if (devc->status == SLOGIC_ST_DATA && transfer->actual_length > 0) {
		buf = transfer->buffer;
		len = transfer->actual_length;
		if (devc->drop_left > 0) {
			int n = (len < devc->drop_left) ? len : devc->drop_left;
			buf += n;
			len -= n;
			devc->drop_left -= n;
		}
		if (len > 0 &&
		    slogic_convert_push(devc, buf, len) != SR_OK) {
			sr_err("format convert failed");
			slogic_request_acq_stop(devc);
		}
		if (!devc->is_loop &&
		    devc->limit_samples &&
		    devc->num_samples >= devc->limit_samples)
			slogic_request_acq_stop(devc);
	}

	if (devc->status == SLOGIC_ST_DATA && !devc->abort) {
		r = libusb_submit_transfer(transfer);
		if (r != 0) {
			sr_err("resubmit failed: %s", libusb_error_name(r));
			free_transfer(transfer);
		}
	} else {
		free_transfer(transfer);
	}
}

static int start_transfers(struct slogic_context *devc)
{
	int i;
	unsigned char *buf;
	struct libusb_transfer *xfer;

	devc->num_transfers = SLOGIC_NUM_TRANSFERS;
	devc->submitted_transfers = 0;
	devc->transfers = g_try_malloc0(sizeof(struct libusb_transfer *) *
					devc->num_transfers);
	if (!devc->transfers)
		return SR_ERR_MALLOC;

	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc0(SLOGIC_TRANSFER_SIZE);
		if (!buf)
			return SR_ERR_MALLOC;
		xfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(xfer, devc->devhdl, SLOGIC_EP_IN,
					  buf, SLOGIC_TRANSFER_SIZE,
					  receive_transfer, devc,
					  SLOGIC_BULK_TIMEOUT_MS);
		if (libusb_submit_transfer(xfer) != 0) {
			sr_err("submit transfer %d failed", i);
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
	struct slogic_context *devc = sdi->priv;
	struct drv_context *drvc = di->priv;
	struct timeval tv = { 0, 0 };
	int completed = 0;

	(void)fd;
	(void)revents;

	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx,
					       &tv, &completed);

	if (devc->fw_streaming && devc->abort)
		slogic_hw_stop(devc);

	if (devc->status == SLOGIC_ST_FINISH) {
		slogic_hw_stop(devc);
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
	char name[8];

	while (sdi->channels) {
		struct sr_channel *ch = sdi->channels->data;
		sdi->channels = g_slist_delete_link(sdi->channels, sdi->channels);
		g_free(ch->name);
		g_free(ch);
	}
	for (i = 0; i < count; i++) {
		const char *n = probe_names[i] ? probe_names[i] : name;
		if (!probe_names[i])
			snprintf(name, sizeof(name), "%d", i);
		sdi->channels = g_slist_append(
			sdi->channels,
			sr_channel_new(i, SR_CHANNEL_LOGIC, TRUE, n));
	}
}

static GSList *hw_scan(GSList *options)
{
	struct drv_context *drvc;
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	struct sr_dev_inst *sdi;
	struct slogic_context *devc;
	GSList *devices = NULL;
	int i;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		uint8_t bus, address;
		struct sr_usb_dev_inst *usb_info;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		if (des.idVendor != SLOGIC_VID || des.idProduct != SLOGIC_PID_16U3)
			continue;
		if (sr_usb_device_is_exists(devlist[i]))
			continue;

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc)
			break;

		bus = libusb_get_bus_number(devlist[i]);
		address = libusb_get_device_address(devlist[i]);

		devc->usb_dev = libusb_ref_device(devlist[i]);
		devc->sr_ctx = drvc->sr_ctx;
		devc->ch_mode = SLOGIC_CHMODE_16;
		devc->channel_count = 16;
		devc->filter = SR_FILTER_NONE;
		devc->limit_samples = SLOGIC_DEFAULT_SAMPLES;
		devc->max_height = 1;
		devc->vth = SLOGIC_VTH_DEFAULT;
		devc->op_mode = LO_OP_STREAM;
		devc->is_loop = 0;
		devc->usb_speed = LIBUSB_SPEED_UNKNOWN;

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE,
				      "Sipeed", "SLogic16U3", NULL);
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
		slogic_update_usb_speed(devc);
		slogic_map_samplerate(devc, SR_MHZ(100));

		usb_info = sr_usb_dev_inst_new(bus, address);
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		} else {
			sdi->conn = NULL;
		}
		setup_channels(sdi, 16);
		slogic_apply_channel_mode(sdi);

		sr_info("Found SLogic16U3 359f:%04x bus=%u addr=%u speed=%s name=%s",
			des.idProduct, bus, address,
			slogic_speed_name(devc->usb_speed),
			sdi->name ? sdi->name : "?");
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
	struct slogic_context *devc = sdi->priv;
	int r;

	if (sdi->status == SR_ST_ACTIVE)
		return SR_OK;

	r = libusb_open(devc->usb_dev, &devc->devhdl);
	if (r != 0) {
		sr_err("open failed: %s", libusb_error_name(r));
		if (r == LIBUSB_ERROR_NOT_SUPPORTED)
			ds_set_last_error(SR_ERR_DEVICE_NO_DRIVER);
		else
			ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}

	if (libusb_kernel_driver_active(devc->devhdl, 0) == 1)
		libusb_detach_kernel_driver(devc->devhdl, 0);

	r = libusb_claim_interface(devc->devhdl, 0);
	if (r != 0) {
		sr_err("claim failed: %s", libusb_error_name(r));
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
		ds_set_last_error(SR_ERR_DEVICE_IS_EXCLUSIVE);
		return SR_ERR;
	}

	devc->sdi = sdi;
	slogic_update_usb_speed(devc);
	slogic_map_samplerate(devc, devc->samplerate ? devc->samplerate
						     : SR_MHZ(100));
	if (slogic_reset(devc) != SR_OK)
		sr_warn("device reset failed (continuing)");

	sdi->status = SR_ST_ACTIVE;
	sr_info("open: name=\"%s\" speed=%s",
		sdi->name ? sdi->name : "?",
		slogic_speed_name(devc->usb_speed));
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct slogic_context *devc;

	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	if (devc->devhdl) {
		if (sdi->status == SR_ST_ACTIVE) {
			if (!devc->fw_streaming)
				devc->fw_streaming = 1;
			slogic_hw_stop(devc);
		}
		libusb_release_interface(devc->devhdl, 0);
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct slogic_context *devc;

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

SR_PRIV void slogic16u3_on_usb_reconnected(struct sr_dev_inst *sdi,
					   struct libusb_device *new_dev)
{
	struct slogic_context *devc;

	if (!sdi || !sdi->priv || !new_dev)
		return;
	if (!sdi->driver || !sdi->driver->name ||
	    strcmp(sdi->driver->name, "slogic-16u3") != 0)
		return;

	devc = sdi->priv;
	if (devc->usb_dev)
		libusb_unref_device(devc->usb_dev);
	devc->usb_dev = libusb_ref_device(new_dev);
	devc->sdi = sdi;
	sdi->handle = (ds_device_handle)devc->usb_dev;
	slogic_update_usb_speed(devc);
	slogic_map_samplerate(devc, devc->samplerate ? devc->samplerate
						     : SR_MHZ(100));
	sr_info("reconnected: name=\"%s\" speed=%s max=%" PRIu64,
		sdi->name ? sdi->name : "?",
		slogic_speed_name(devc->usb_speed),
		slogic_link_max_rate(devc));
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
	struct slogic_context *devc;

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
		*data = g_variant_new_uint64(SLOGIC_HW_DEPTH);
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16(16);
		break;
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int16(devc->channel_count);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(TRUE);
		break;
	case SR_CONF_VTH:
		*data = g_variant_new_double(devc->vth);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16(devc->op_mode);
		break;
	case SR_CONF_CHANNEL_MODE:
		*data = g_variant_new_int16(devc->ch_mode);
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_int16(devc->filter);
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
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_set(int id, GVariant *data, struct sr_dev_inst *sdi,
		      struct sr_channel *ch,
		      struct sr_channel_group *cg)
{
	struct slogic_context *devc;
	const char *stropt;
	unsigned int i;

	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_SAMPLERATE:
		slogic_map_samplerate(devc, g_variant_get_uint64(data));
		sr_info("samplerate=%" PRIu64, devc->samplerate);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
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
		if (v < SLOGIC_VTH_MIN)
			v = SLOGIC_VTH_MIN;
		if (v > SLOGIC_VTH_MAX)
			v = SLOGIC_VTH_MAX;
		devc->vth = v;
		sr_info("VTH stored: %.2f V", devc->vth);
		break;
	}
	case SR_CONF_FILTER: {
		int nv = g_variant_get_int16(data);
		if (nv != SR_FILTER_NONE && nv != SR_FILTER_1T)
			return SR_ERR;
		devc->filter = nv;
		break;
	}
	case SR_CONF_CHANNEL_MODE: {
		int nv = g_variant_get_int16(data);
		if (nv != SLOGIC_CHMODE_4 && nv != SLOGIC_CHMODE_8 &&
		    nv != SLOGIC_CHMODE_16)
			return SR_ERR;
		devc->ch_mode = nv;
		slogic_apply_channel_mode(sdi);
		slogic_map_samplerate(devc, devc->samplerate);
		sr_info("channel mode -> %d ch, rate=%" PRIu64,
			devc->channel_count, devc->samplerate);
		break;
	}
	case SR_CONF_OPERATION_MODE: {
		int nv = g_variant_get_int16(data);
		if (nv != LO_OP_STREAM)
			return SR_ERR;
		devc->op_mode = LO_OP_STREAM;
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
	case SR_CONF_SAMPLERATE: {
		static uint64_t rate_buf[32];
		int rate_n = 0;
		const struct slogic_context *devc =
			(sdi && sdi->priv) ? (const struct slogic_context *)sdi->priv
					   : NULL;

		if (devc)
			slogic_build_rate_list(devc, rate_buf, &rate_n);
		else {
			unsigned int i;
			for (i = 0; i < ARRAY_SIZE(slogic_ui_rates); i++)
				if (slogic_ui_rates[i] <= SR_MHZ(200))
					rate_buf[rate_n++] = slogic_ui_rates[i];
			rate_buf[rate_n] = 0;
		}
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_from_data(
			G_VARIANT_TYPE("at"), rate_buf,
			(gsize)rate_n * sizeof(uint64_t),
			TRUE, NULL, NULL);
		g_variant_builder_add(&gvb, "{sv}", "samplerates", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	}
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_strv(maxHeights, ARRAY_SIZE(maxHeights));
		break;
	case SR_CONF_FILTER:
		*data = g_variant_new_uint64((uint64_t)&filter_list);
		break;
	case SR_CONF_CHANNEL_MODE:
		*data = g_variant_new_uint64((uint64_t)&channel_mode_list);
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
	struct slogic_context *devc = sdi->priv;
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
	devc->num_bytes = 0;
	devc->abort = 0;
	devc->fw_streaming = 0;
	devc->status = SLOGIC_ST_START;
	devc->freewheel = 0;
	devc->raw_pending_len = 0;
	devc->filt_have_prev = 0;
	devc->stream_res_len = 0;
	devc->drop_left = SLOGIC_DROP_FIRST_BYTES;

	slogic_apply_channel_mode(sdi);
	slogic_refresh_enabled_bits(devc);
	slogic_map_samplerate(devc, devc->samplerate);

	sr_info("acq start: nch=%d en=%d rate=%" PRIu64 " loop=%d limit=%" PRIu64
		" speed=%s",
		devc->channel_count, devc->en_count, devc->samplerate,
		devc->is_loop, devc->limit_samples,
		slogic_speed_name(devc->usb_speed));

	/* Stop leftover capture and drain EP before arming new URBs. */
	slogic_wr32(devc, SLOGIC_R32_CTRL, SLOGIC_CTRL_STOP);
	slogic_drain_ep(devc);

	if ((ret = start_transfers(devc)) != SR_OK) {
		sr_err("start_transfers failed");
		return ret;
	}

	if ((ret = slogic_apply_channel(devc)) != SR_OK ||
	    (ret = slogic_apply_samplerate(devc)) != SR_OK ||
	    (ret = slogic_apply_vth(devc)) != SR_OK) {
		devc->abort = 1;
		slogic_cancel_transfers(devc);
		return ret;
	}

	lupfd = libusb_get_pollfds(drvc->sr_ctx->libusb_ctx);
	if (!lupfd) {
		devc->freewheel = 1;
		sr_session_source_add(-1, 0, 0, receive_data, sdi);
	} else {
		for (i = 0; lupfd[i]; i++) {
			sr_source_add(lupfd[i]->fd, lupfd[i]->events,
				      20, receive_data, sdi);
		}
		g_free((void *)lupfd);
	}

	if ((ret = slogic_hw_start(devc)) != SR_OK) {
		devc->abort = 1;
		devc->fw_streaming = 0;
		slogic_cancel_transfers(devc);
		return ret;
	}
	devc->fw_streaming = 1;

	std_session_send_df_header(sdi, LOG_PREFIX);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct slogic_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	slogic_hw_stop(devc);
	if (!devc->abort) {
		devc->abort = 1;
		devc->status = SLOGIC_ST_STOP;
		slogic_cancel_transfers(devc);
		if (devc->submitted_transfers <= 0)
			finish_acquisition(devc);
	}
	return SR_OK;
}

SR_PRIV struct sr_dev_driver slogic16u3_driver_info = {
	.name = "slogic-16u3",
	.longname = "Sipeed SLogic16U3 Logic Analyzer",
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
