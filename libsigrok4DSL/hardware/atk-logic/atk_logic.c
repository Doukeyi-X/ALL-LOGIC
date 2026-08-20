/*
 * Alientek ATK-Logic driver for DSView / libsigrok4DSL (ALL LOGIC)
 *
 * USB protocol ported from https://github.com/alientek-openedv/atk-logic
 * (same DSView/PulseView lineage). VID 1A86 PID FFCC.
 *   EP 0x02 OUT  MCU 512-byte / FPGA 2048-byte interleaved frames
 *   EP 0x81 IN   replies + capture stream
 *
 * FPGA command (then convert_to_device, pad to 2048):
 *   8 zero bytes | 0x0A | payload | 0x0B | CRC32(payload)
 *   payload = { code, orig_len+1, data... }
 *   0x10 GetDeviceData   0x11 ParameterSetting
 *   0x12 SimpleTrigger   0x15 Stop   0x18 Exit
 *
 * MCU command (raw 512-byte bulk, no convert):
 *   0x0A 0x81 GetMCUVersion
 *   0x0A 0x87 SetResetState
 *
 * Incoming capture (after convert_to_pc) is framed:
 *   0x0A | order | u16 len | data[len] | 0x00 | 0x0B
 *   order 1 = per-channel bitstream (byte0 = channel id)
 *   order 6 = transfer complete
 *
 * Not an official Alientek host.
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "atk_logic.h"

#undef LOG_PREFIX
#define LOG_PREFIX "atk-logic: "

#define ATK_VID                  0x1A86
#define ATK_PID                  0xFFCC
#define ATK_EP_OUT               0x02
#define ATK_EP_IN                0x81

#define ATK_CMD_GET_DEV          0x10
#define ATK_CMD_PARAM            0x11
#define ATK_CMD_TRIGGER          0x12
#define ATK_CMD_STOP             0x15
#define ATK_CMD_EXIT             0x18

#define ATK_MCU_GET_VER          0x81
#define ATK_MCU_RESET            0x87

#define ATK_XFER_TIMEOUT_MS      1200	/* official ThreadRead */
#define ATK_CMD_TIMEOUT_MS       100	/* official SendToDevice */
#define ATK_NUM_TRANSFERS        16
#define ATK_TRANSFER_SIZE        (16 * 1024)	/* official READ_TRANSFER_BUFFER_SIZE */
#define ATK_FRAME                2048
#define ATK_MAX_CH               16
#define ATK_CROSS_SCALE          64
#define ATK_CROSS_PLANE_BYTES    8
#define ATK_DEFAULT_SAMPLES      SR_Mn(1)
#define ATK_HW_DEPTH             SR_Mn(16)
#define ATK_RLE_BUF              (512 * 1024)

enum {
	ATK_ST_IDLE = 0,
	ATK_ST_START,
	ATK_ST_DATA,
	ATK_ST_STOP,
	ATK_ST_FINISH,
};

/* Official combo box (index+1 is sent as the rate byte). */
static const uint64_t atk_rate_table[] = {
	SR_MHZ(1), SR_MHZ(2), SR_MHZ(4), SR_MHZ(5),
	SR_MHZ(10), SR_MHZ(20), SR_MHZ(25), SR_MHZ(40),
	SR_MHZ(50), SR_MHZ(100), SR_MHZ(200), SR_MHZ(250),
	SR_MHZ(500), SR_GHZ(1),
};

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };
static const char *probe_names[] = {
	"0", "1", "2", "3", "4", "5", "6", "7",
	"8", "9", "10", "11", "12", "13", "14", "15",
	NULL,
};

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_VTH,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
	SR_CONF_VTH,
};

static const struct sr_list_item opmode_list[] = {
	{ LO_OP_BUFFER, "Buffer Mode" },
	{ LO_OP_STREAM, "Stream Mode" },
	{ -1, NULL },
};

static const uint32_t crc_table[256] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};

struct atk_context {
	struct libusb_device *usb_dev;
	struct libusb_device_handle *devhdl;
	struct sr_context *sr_ctx;
	const struct sr_dev_inst *sdi;

	enum libusb_speed usb_speed;
	int device_level; /* MCU buf[8]: 1 = 1 GHz class */
	int usb3;         /* 1 if FPGA reported USB 3 */
	char fpga_name[64];

	uint64_t samplerate;
	uint8_t rate_index; /* 0-based into atk_rate_table */
	uint64_t limit_samples;
	uint64_t num_samples;
	double vth;
	int max_height;
	int op_mode; /* LO_OP_BUFFER / LO_OP_STREAM */
	int is_loop;
	int use_rle;

	int status;
	int abort;
	int fw_streaming;
	int got_end;
	int hold_emit; /* buffer mode: accumulate until order 6 */
	int pending_rearm;
	int pending_stop;
	int trig_hit;
	uint64_t hw_progress;
	uint64_t trig_depth;
	uint64_t skip_bits[ATK_MAX_CH];
	int submitted_transfers;
	int num_transfers;
	struct libusb_transfer **transfers;
	struct libusb_transfer *out_xfer;
	uint8_t *out_buf;
	int out_busy;
	int freewheel;

	uint8_t *parse_res;
	int parse_res_len;
	int parse_res_cap;
	uint8_t *rle_buf;
	uint8_t usb_rem[ATK_FRAME];
	int usb_rem_len;
	uint8_t ch_seen[ATK_MAX_CH];

	GByteArray *ch_fifo[ATK_MAX_CH];
	uint8_t en_ch[ATK_MAX_CH];
	int en_count;

	uint8_t *cross_buf;
	int cross_cap;
};

SR_PRIV struct sr_dev_driver atk_logic_driver_info;
static struct sr_dev_driver *di = &atk_logic_driver_info;

/* -------------------- wire helpers -------------------- */

static uint32_t atk_crc32(const uint8_t *buf, int n)
{
	uint32_t crc = 0;
	int i;

	if (n < 1)
		return 0xffffffffu;
	for (i = 0; i < n; i++)
		crc = crc_table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
	return crc ^ 0xffffffffu;
}

static void atk_convert_to_device(const uint8_t *s, uint8_t *d, unsigned len)
{
	const uint16_t *src = (const uint16_t *)s;
	uint16_t *d0, *d1, *d2, *d3;
	unsigned i, j, k;

	for (i = 0; i < len; i += ATK_FRAME) {
		d0 = (uint16_t *)(d + 0);
		d1 = (uint16_t *)(d + 512);
		d2 = (uint16_t *)(d + 512 * 2);
		d3 = (uint16_t *)(d + 512 * 3);
		for (j = 0, k = 0; j < 256; j++, k += 4) {
			d0[j] = src[k];
			d1[j] = src[k + 1];
			d2[j] = src[k + 2];
			d3[j] = src[k + 3];
		}
		src += 1024;
		d += ATK_FRAME;
	}
}

static void atk_convert_to_pc(const uint8_t *s, uint8_t *d, unsigned len)
{
	const uint16_t *src0 = (const uint16_t *)(s + 0);
	const uint16_t *src1 = (const uint16_t *)(s + 512);
	const uint16_t *src2 = (const uint16_t *)(s + 512 * 2);
	const uint16_t *src3 = (const uint16_t *)(s + 512 * 3);
	uint16_t *dst = (uint16_t *)d;
	unsigned i, j, k;

	for (i = 0; i < len; i += ATK_FRAME) {
		for (j = 0, k = 0; j < 256; j++, k += 4) {
			dst[k] = src0[j];
			dst[k + 1] = src1[j];
			dst[k + 2] = src2[j];
			dst[k + 3] = src3[j];
		}
		dst += 1024;
		src0 += 1024;
		src1 += 1024;
		src2 += 1024;
		src3 += 1024;
	}
}

static int atk_bulk_out(struct atk_context *devc, uint8_t *buf, int len, int timeout)
{
	int xfer = 0;
	int r = libusb_bulk_transfer(devc->devhdl, ATK_EP_OUT, buf, len,
				     &xfer, timeout);
	if (r != 0 && r != LIBUSB_ERROR_TIMEOUT) {
		sr_err("bulk OUT failed: %s", libusb_error_name(r));
		return SR_ERR;
	}
	return SR_OK;
}

static int atk_bulk_in(struct atk_context *devc, uint8_t *buf, int len,
		       int *got, int timeout)
{
	int xfer = 0;
	int r = libusb_bulk_transfer(devc->devhdl, ATK_EP_IN, buf, len,
				     &xfer, timeout);
	*got = xfer;
	if (r == 0 || (r == LIBUSB_ERROR_TIMEOUT && xfer > 0))
		return SR_OK;
	if (r == LIBUSB_ERROR_TIMEOUT)
		return SR_ERR;
	sr_dbg("bulk IN: %s", libusb_error_name(r));
	return SR_ERR;
}

static void atk_drain_in(struct atk_context *devc)
{
	uint8_t buf[ATK_TRANSFER_SIZE];
	int got, n = 0;

	while (n++ < 128) {
		if (atk_bulk_in(devc, buf, sizeof(buf), &got, 30) != SR_OK || got <= 0)
			break;
	}
}

static int atk_mcu_cmd(struct atk_context *devc, uint8_t code, const void *data, int dlen)
{
	uint8_t buf[512];

	memset(buf, 0, sizeof(buf));
	buf[0] = 0x0a;
	buf[1] = code;
	if (data && dlen > 0 && dlen < 510)
		memcpy(buf + 2, data, dlen);
	return atk_bulk_out(devc, buf, sizeof(buf), ATK_CMD_TIMEOUT_MS);
}

static void LIBUSB_CALL atk_out_cb(struct libusb_transfer *t)
{
	struct atk_context *devc = t->user_data;

	if (devc)
		devc->out_busy = 0;
	if (t->status != LIBUSB_TRANSFER_COMPLETED &&
	    t->status != LIBUSB_TRANSFER_TIMED_OUT)
		sr_err("FPGA OUT status %d", t->status);
}

static int atk_out_submit(struct atk_context *devc, uint8_t *frame)
{
	int r;

	if (devc->out_busy)
		return SR_ERR;
	if (!devc->out_buf)
		devc->out_buf = g_try_malloc(ATK_FRAME);
	if (!devc->out_xfer)
		devc->out_xfer = libusb_alloc_transfer(0);
	if (!devc->out_buf || !devc->out_xfer)
		return SR_ERR_MALLOC;
	memcpy(devc->out_buf, frame, ATK_FRAME);
	libusb_fill_bulk_transfer(devc->out_xfer, devc->devhdl, ATK_EP_OUT,
				  devc->out_buf, ATK_FRAME, atk_out_cb, devc,
				  ATK_CMD_TIMEOUT_MS);
	devc->out_busy = 1;
	r = libusb_submit_transfer(devc->out_xfer);
	if (r != 0) {
		devc->out_busy = 0;
		sr_err("FPGA OUT submit: %s", libusb_error_name(r));
		return SR_ERR;
	}
	return SR_OK;
}

static int atk_fpga_cmd(struct atk_context *devc, uint8_t code,
			const uint8_t *data, int dlen)
{
	uint8_t payload[256];
	uint8_t raw[ATK_FRAME];
	uint8_t conv[ATK_FRAME];
	int plen;
	uint32_t crc;

	if (dlen < 0 || dlen > 250)
		return SR_ERR_ARG;
	payload[0] = code;
	payload[1] = (uint8_t)(dlen + 1);
	if (dlen > 0)
		memcpy(payload + 2, data, dlen);
	plen = dlen + 2;
	crc = atk_crc32(payload, plen);

	memset(raw, 0, sizeof(raw));
	raw[8] = 0x0a;
	memcpy(raw + 9, payload, plen);
	raw[9 + plen] = 0x0b;
	memcpy(raw + 10 + plen, &crc, 4);
	atk_convert_to_device(raw, conv, ATK_FRAME);
	/*
	 * Sync bulk OUT while async IN is in flight deadlocks WinUSB
	 * once the pipe is busy (4ch 50 MHz is enough). Official host
	 * submits OUT asynchronously too.
	 */
	if (devc->submitted_transfers > 0)
		return atk_out_submit(devc, conv);
	return atk_bulk_out(devc, conv, ATK_FRAME, ATK_CMD_TIMEOUT_MS);
}

static int atk_set_reset(struct atk_context *devc, uint8_t state)
{
	return atk_mcu_cmd(devc, ATK_MCU_RESET, &state, 1);
}

static void atk_stop_and_drain(struct atk_context *devc)
{
	atk_fpga_cmd(devc, ATK_CMD_STOP, NULL, 0);
	g_usleep(40000);
	atk_drain_in(devc);
}

/* -------------------- rates / channels -------------------- */

static int atk_en_count(const struct sr_dev_inst *sdi)
{
	GSList *l;
	int n = 0;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type == SR_CHANNEL_LOGIC && ch->enabled)
			n++;
	}
	return n > 0 ? n : 16;
}

static uint64_t atk_max_rate(const struct atk_context *devc, int ench)
{
	int usb3 = devc->usb3 &&
		   (devc->usb_speed == LIBUSB_SPEED_SUPER ||
		    devc->usb_speed == LIBUSB_SPEED_SUPER_PLUS);
	int i;
	uint64_t cap;

	if (ench < 1)
		ench = 16;

	/*
	 * Official live/stream: hz * nch <= 320e6 (USB 2.0 HS bulk).
	 * 16ch → 20 MHz, 12ch → 25 MHz, 8ch → 40 MHz, 3ch → 100 MHz.
	 * Buffer on USB2 DL16: 250 MHz. Plus on USB3: 1 GHz / 500 MHz.
	 */
	if (devc->op_mode == LO_OP_STREAM || devc->is_loop) {
		cap = 320000000ull / (uint64_t)ench;
		for (i = (int)ARRAY_SIZE(atk_rate_table) - 1; i >= 0; i--) {
			if (atk_rate_table[i] <= cap)
				return atk_rate_table[i];
		}
		return SR_MHZ(1);
	}
	if (devc->device_level == 1 && usb3) {
		if (ench <= 8)
			return SR_GHZ(1);
		return SR_MHZ(500);
	}
	return SR_MHZ(250);
}

static int atk_rate_to_index(uint64_t hz)
{
	int i, best = 0;
	uint64_t dbest = UINT64_MAX;

	for (i = 0; i < (int)ARRAY_SIZE(atk_rate_table); i++) {
		uint64_t d = hz > atk_rate_table[i] ? hz - atk_rate_table[i]
						    : atk_rate_table[i] - hz;
		if (d < dbest) {
			dbest = d;
			best = i;
		}
	}
	return best;
}

static void atk_clamp_rate(struct atk_context *devc)
{
	int ench;
	uint64_t maxr;

	if (!devc->sdi)
		return;
	ench = atk_en_count(devc->sdi);
	maxr = atk_max_rate(devc, ench);
	if (devc->samplerate > maxr)
		devc->samplerate = maxr;
	devc->rate_index = (uint8_t)atk_rate_to_index(devc->samplerate);
	devc->samplerate = atk_rate_table[devc->rate_index];
}

static void atk_refresh_enabled(struct atk_context *devc)
{
	GSList *l;
	int n = 0;

	memset(devc->en_ch, 0, sizeof(devc->en_ch));
	for (l = devc->sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC || !ch->enabled)
			continue;
		if (ch->index >= ATK_MAX_CH)
			continue;
		devc->en_ch[n++] = (uint8_t)ch->index;
	}
	if (n == 0) {
		for (n = 0; n < ATK_MAX_CH; n++)
			devc->en_ch[n] = (uint8_t)n;
	}
	devc->en_count = n;
}

static int atk_ensure_cross(struct atk_context *devc, int need)
{
	if (need <= devc->cross_cap)
		return SR_OK;
	{
		uint8_t *p = g_try_realloc(devc->cross_buf, (gsize)need);
		if (!p)
			return SR_ERR_MALLOC;
		devc->cross_buf = p;
		devc->cross_cap = need;
	}
	return SR_OK;
}

/* -------------------- parse + LA_CROSS -------------------- */

static int atk_emit_ready_groups(struct atk_context *devc)
{
	int groups, g, c, minb;
	uint8_t *out;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	if (devc->en_count < 1)
		return 0;

	/*
	 * Sync on channels that have actually sent data. Never-seen
	 * lanes get zeros (idle). Do NOT pad a lagging seen lane —
	 * that shifts its timeline and produces garbage waveforms.
	 */
	minb = INT_MAX;
	for (c = 0; c < devc->en_count; c++) {
		int ch = devc->en_ch[c];
		GByteArray *a;
		int nb;
		if (!devc->ch_seen[ch])
			continue;
		a = devc->ch_fifo[ch];
		nb = a ? (int)a->len : 0;
		if (nb < minb)
			minb = nb;
	}
	if (minb == INT_MAX)
		return 0;
	groups = minb / ATK_CROSS_PLANE_BYTES;
	if (groups <= 0)
		return 0;

	if (devc->limit_samples && !devc->is_loop) {
		uint64_t room = (devc->num_samples < devc->limit_samples)
			? (devc->limit_samples - devc->num_samples) : 0;
		int maxg = (int)(room / ATK_CROSS_SCALE);
		if (maxg <= 0) {
			devc->num_samples = devc->limit_samples;
			return 0;
		}
		if (groups > maxg)
			groups = maxg;
	}

	if (atk_ensure_cross(devc, groups * ATK_CROSS_PLANE_BYTES * devc->en_count) != SR_OK)
		return -1;

	out = devc->cross_buf;
	for (g = 0; g < groups; g++) {
		for (c = 0; c < devc->en_count; c++) {
			int ch = devc->en_ch[c];
			GByteArray *a = devc->ch_fifo[ch];
			if (!devc->ch_seen[ch] || !a || a->len < ATK_CROSS_PLANE_BYTES)
				memset(out, 0, ATK_CROSS_PLANE_BYTES);
			else {
				memcpy(out, a->data, ATK_CROSS_PLANE_BYTES);
				g_byte_array_remove_range(a, 0, ATK_CROSS_PLANE_BYTES);
			}
			out += ATK_CROSS_PLANE_BYTES;
		}
	}

	packet.type = SR_DF_LOGIC;
	packet.status = SR_PKT_OK;
	packet.payload = &logic;
	logic.length = (uint64_t)groups * ATK_CROSS_PLANE_BYTES * devc->en_count;
	logic.format = LA_CROSS_DATA;
	logic.data_error = 0;
	logic.data = devc->cross_buf;
	ds_data_forward(devc->sdi, &packet);

	devc->num_samples += (uint64_t)groups * ATK_CROSS_SCALE;
	return groups;
}

static void atk_feed_channel(struct atk_context *devc, int ch,
			     const uint8_t *data, int len)
{
	if (ch < 0 || ch >= ATK_MAX_CH || len <= 0)
		return;
	devc->ch_seen[ch] = 1;
	if (!devc->ch_fifo[ch])
		devc->ch_fifo[ch] = g_byte_array_new();
	g_byte_array_append(devc->ch_fifo[ch], data, (guint)len);
}

static int atk_have_ch_data(const struct atk_context *devc)
{
	int i;

	for (i = 0; i < ATK_MAX_CH; i++) {
		if (devc->ch_seen[i])
			return 1;
	}
	return 0;
}

static int atk_rle_expand(struct atk_context *devc, const uint8_t *src, int slen)
{
	int i, o = 0;

	if (!devc->rle_buf) {
		devc->rle_buf = g_try_malloc(ATK_RLE_BUF);
		if (!devc->rle_buf)
			return 0;
	}
	for (i = 0; i + 1 < slen; i += 2) {
		int n = src[i];
		uint8_t v = src[i + 1];
		if (n <= 0)
			continue;
		if (o + n > ATK_RLE_BUF)
			n = ATK_RLE_BUF - o;
		if (n <= 0)
			break;
		memset(devc->rle_buf + o, v, (size_t)n);
		o += n;
	}
	return o;
}

static void atk_apply_crop(struct atk_context *devc)
{
	int c;

	for (c = 0; c < ATK_MAX_CH; c++) {
		GByteArray *a = devc->ch_fifo[c];
		uint64_t nbytes;
		if (!a || a->len == 0)
			continue;
		nbytes = devc->skip_bits[c] / 8;
		if (nbytes == 0)
			continue;
		if (nbytes >= a->len)
			g_byte_array_set_size(a, 0);
		else
			g_byte_array_remove_range(a, 0, (guint)nbytes);
	}
}

static void atk_flush_held(struct atk_context *devc)
{
	int guard = 0;

	atk_apply_crop(devc);
	devc->hold_emit = 0;
	while (guard++ < 1 << 20) {
		int n = atk_emit_ready_groups(devc);
		if (n <= 0)
			break;
	}
}

static uint64_t atk_le5(const uint8_t *p)
{
	return (uint64_t)p[0] | ((uint64_t)p[1] << 8) |
	       ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24) |
	       ((uint64_t)p[4] << 32);
}

static void atk_parse_packets(struct atk_context *devc, const uint8_t *buf, int len)
{
	int i = 0;

	while (i + 6 <= len) {
		uint16_t dlen;
		int pkt;
		const uint8_t *d;

		if (buf[i] != 0x0a || buf[i + 1] < 1 || buf[i + 1] > 6) {
			i++;
			continue;
		}
		dlen = (uint16_t)buf[i + 2] | ((uint16_t)buf[i + 3] << 8);
		pkt = (int)dlen + 6;
		if (i + pkt > len)
			break;
		if (buf[i + 4 + dlen] != 0x00 || buf[i + 5 + dlen] != 0x0b) {
			i++;
			continue;
		}
		d = buf + i + 4;

		switch (buf[i + 1]) {
		case 1:
			if (dlen >= 3) {
				int ch = d[0];
				const uint8_t *pay = d + 2;
				int plen = dlen - 2;
				if (devc->use_rle) {
					int n = atk_rle_expand(devc, pay, plen);
					if (n > 0)
						atk_feed_channel(devc, ch,
								 devc->rle_buf, n);
				} else {
					atk_feed_channel(devc, ch, pay, plen);
				}
			}
			break;
		case 3:
			if (dlen >= 7) {
				uint64_t vern = atk_le5(d + 2);
				int ch, off = 7;
				sr_info("trigger offset %" PRIu64, vern);
				devc->trig_hit = 1;
				for (ch = 0; ch < ATK_MAX_CH && off + 5 <= dlen; ch++, off += 5) {
					uint64_t nbytes = atk_le5(d + off);
					int64_t skip;
					if (nbytes == 0) {
						devc->skip_bits[ch] = 0;
						continue;
					}
					skip = (int64_t)nbytes * 8 -
					       (int64_t)devc->trig_depth -
					       (int64_t)vern;
					devc->skip_bits[ch] = skip > 0 ? (uint64_t)skip : 0;
				}
			}
			break;
		case 4:
			if (dlen >= 3)
				sr_info("ack cmd=0x%02x st=%u", d[2],
					dlen >= 4 ? d[3] : 0);
			break;
		case 5:
			if (dlen >= 7) {
				devc->hw_progress = atk_le5(d + 2);
				if (devc->trig_depth &&
				    devc->hw_progress > devc->trig_depth)
					devc->trig_hit = 1;
			}
			break;
		case 6:
			/* Leftover STOP / padding can look like order 6
			 * before any channel bytes. Ignore those. */
			if (!atk_have_ch_data(devc))
				break;
			sr_info("device end-of-transfer samples=%" PRIu64, devc->num_samples);
			if (dlen >= 3 && d[2] == 1) {
				sr_warn("stream overflow");
				devc->got_end = 1;
				if (devc->hold_emit)
					atk_flush_held(devc);
				break;
			}
			if (devc->is_loop) {
				devc->pending_rearm = 1;
				break;
			}
			devc->got_end = 1;
			if (devc->hold_emit)
				atk_flush_held(devc);
			break;
		default:
			break;
		}
		i += pkt;
	}

	if (i < len) {
		int left = len - i;
		if (left > devc->parse_res_cap) {
			uint8_t *p = g_try_realloc(devc->parse_res, (gsize)left);
			if (!p)
				return;
			devc->parse_res = p;
			devc->parse_res_cap = left;
		}
		memcpy(devc->parse_res, buf + i, left);
		devc->parse_res_len = left;
	} else {
		devc->parse_res_len = 0;
	}

	if (!devc->hold_emit)
		atk_emit_ready_groups(devc);
}

static void atk_handle_raw(struct atk_context *devc, const uint8_t *raw, int raw_len)
{
	uint8_t *conv;
	int n, total;
	uint8_t *work;

	if (raw_len < ATK_FRAME)
		return;
	n = (raw_len / ATK_FRAME) * ATK_FRAME;
	conv = g_try_malloc((gsize)n);
	if (!conv)
		return;
	atk_convert_to_pc(raw, conv, (unsigned)n);

	total = devc->parse_res_len + n;
	work = g_try_malloc((gsize)total);
	if (!work) {
		g_free(conv);
		return;
	}
	if (devc->parse_res_len)
		memcpy(work, devc->parse_res, devc->parse_res_len);
	memcpy(work + devc->parse_res_len, conv, n);
	devc->parse_res_len = 0;
	g_free(conv);

	atk_parse_packets(devc, work, total);
	g_free(work);
}

/* -------------------- start / stop wire -------------------- */

static void atk_put_le(uint8_t *p, uint64_t v, int n)
{
	int i;
	for (i = 0; i < n; i++)
		p[i] = (uint8_t)((v >> (8 * i)) & 0xff);
}

static int atk_is_buffer(const struct atk_context *devc)
{
	return devc->op_mode != LO_OP_STREAM;
}

static uint8_t atk_trig_bits(int ch, int enabled, int instantly, int odd)
{
	uint8_t en, rise, fall, high;
	char c = 'X';

	if (odd) {
		en = 8;
		rise = 1;
		fall = 2;
		high = 4;
	} else {
		en = 128;
		rise = 16;
		fall = 32;
		high = 64;
	}
	if (!enabled)
		return 0;
	if (!instantly && trigger && trigger->trigger_en)
		c = trigger->trigger0[TriggerStages][ch];
	switch (c) {
	case 'R':
		return (uint8_t)(en | rise);
	case 'F':
		return (uint8_t)(en | fall);
	case '1':
		return (uint8_t)(en | high);
	case '0':
		return en; /* low */
	case 'C':
		return (uint8_t)(en | rise | fall);
	default:
		return (uint8_t)(en | rise | fall | high);
	}
}

static int atk_want_instant(const struct atk_context *devc)
{
	int i, ch;

	if (!trigger || !trigger->trigger_en)
		return 1;
	for (i = 0; i < devc->en_count; i++) {
		ch = devc->en_ch[i];
		if (ch < 0 || ch >= MaxTriggerProbes)
			continue;
		if (trigger->trigger0[TriggerStages][ch] != 'X')
			return 0;
	}
	return 1;
}

static int atk_send_param(struct atk_context *devc)
{
	uint8_t p[16];
	uint64_t depth, trig;
	int tbyte, pos;
	double v;

	memset(p, 0, sizeof(p));
	/* bit7 = buffer, bit6 = RLE. Loop/rolling is always live stream. */
	if (atk_is_buffer(devc) && !devc->is_loop)
		p[0] |= 0x80;
	if (devc->use_rle && atk_is_buffer(devc) && !devc->is_loop)
		p[0] |= 0x40;
	v = devc->vth;
	tbyte = (int)floor(fabs(v) * 10.0 + 0.5);
	if (tbyte > 127)
		tbyte = 127;
	if (v < 0)
		tbyte |= 0x80;
	p[1] = (uint8_t)tbyte;
	p[2] = (uint8_t)(devc->rate_index + 1);
	depth = devc->limit_samples ? devc->limit_samples : ATK_DEFAULT_SAMPLES;
	/* Rolling: FPGA stops at depth and sends order 6. 16 MSa is only
	 * 0.32 s at 50 MHz — UI shows ~320 ms / 3.2e8 ns then we used to
	 * deadlock rearming. Official live depth is 40-bit; run until STOP. */
	if (devc->is_loop)
		depth = (1ull << 40) - 1;
	pos = trigger ? (int)ds_trigger_get_pos() : 0;
	if (pos < 0)
		pos = 0;
	if (pos > 100)
		pos = 100;
	trig = depth * (uint64_t)pos / 100;
	devc->trig_depth = trig;
	atk_put_le(p + 3, depth, 5);
	atk_put_le(p + 8, trig, 5);
	sr_info("param flags=0x%02x vth=%.2f rate_idx=%u depth=%" PRIu64
		" trig=%" PRIu64 " rle=%d",
		p[0], devc->vth, p[2], depth, trig, devc->use_rle);
	return atk_fpga_cmd(devc, ATK_CMD_PARAM, p, 13);
}

static int atk_send_trigger(struct atk_context *devc)
{
	uint8_t t[16];
	int i, pair, instantly;
	int enabled[ATK_MAX_CH];

	memset(enabled, 0, sizeof(enabled));
	for (i = 0; i < devc->en_count; i++)
		enabled[devc->en_ch[i]] = 1;

	instantly = devc->is_loop ? 1 : atk_want_instant(devc);
	memset(t, 0, sizeof(t));
	for (pair = 0; pair < 8; pair++) {
		int even = pair * 2;
		int odd = even + 1;
		t[pair] = (uint8_t)(atk_trig_bits(even, enabled[even], instantly, 0) |
				    atk_trig_bits(odd, enabled[odd], instantly, 1));
	}
	t[8] = instantly ? 1 : 0;
	sr_info("trigger instantly=%d bytes=%02x %02x %02x %02x %02x %02x %02x %02x",
		instantly, t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]);
	return atk_fpga_cmd(devc, ATK_CMD_TRIGGER, t, 9);
}

static int atk_hw_stop(struct atk_context *devc)
{
	if (!devc || !devc->fw_streaming)
		return SR_OK;
	if (atk_fpga_cmd(devc, ATK_CMD_STOP, NULL, 0) != SR_OK)
		return SR_ERR;
	devc->fw_streaming = 0;
	sr_info("stop sent");
	return SR_OK;
}

/* -------------------- USB transfers -------------------- */

static void finish_acquisition(struct atk_context *devc)
{
	struct sr_datafeed_packet packet;

	if (!devc || devc->status == ATK_ST_FINISH)
		return;
	devc->abort = 1;
	devc->pending_stop = 1;
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
	devc->status = ATK_ST_FINISH;
}

static void atk_cancel_transfers(struct atk_context *devc)
{
	int i;
	if (!devc || !devc->transfers)
		return;
	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void atk_request_stop(struct atk_context *devc)
{
	if (!devc || devc->abort)
		return;
	devc->abort = 1;
	devc->status = ATK_ST_STOP;
	atk_cancel_transfers(devc);
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct atk_context *devc = transfer->user_data;
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
	struct atk_context *devc = transfer->user_data;
	int r;

	if (devc->status == ATK_ST_START)
		devc->status = ATK_ST_DATA;
	if (devc->abort)
		devc->status = ATK_ST_STOP;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED &&
	    transfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
		if (!devc->abort)
			sr_err("transfer status %d", transfer->status);
		devc->status = ATK_ST_STOP;
	}

	if (devc->status == ATK_ST_DATA && transfer->actual_length > 0) {
		atk_handle_raw(devc, transfer->buffer, transfer->actual_length);
		/* Never USB OUT from this IN callback — it deadlocks
		 * WinUSB/libusb after a while. Rearm/stop run in receive_data. */
		if (!devc->is_loop && devc->got_end)
			atk_request_stop(devc);
		else if (!devc->hold_emit && !devc->is_loop &&
			 devc->limit_samples &&
			 devc->num_samples >= devc->limit_samples)
			atk_request_stop(devc);
	}

	if (devc->status == ATK_ST_DATA && !devc->abort) {
		r = libusb_submit_transfer(transfer);
		if (r != 0) {
			sr_err("resubmit failed: %s", libusb_error_name(r));
			free_transfer(transfer);
		}
	} else {
		free_transfer(transfer);
	}
}

static int start_transfers(struct atk_context *devc)
{
	int i;
	unsigned char *buf;
	struct libusb_transfer *xfer;

	devc->num_transfers = ATK_NUM_TRANSFERS;
	devc->submitted_transfers = 0;
	devc->transfers = g_try_malloc0(sizeof(struct libusb_transfer *) *
					devc->num_transfers);
	if (!devc->transfers)
		return SR_ERR_MALLOC;
	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc0(ATK_TRANSFER_SIZE);
		if (!buf)
			return SR_ERR_MALLOC;
		xfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(xfer, devc->devhdl, ATK_EP_IN,
					  buf, ATK_TRANSFER_SIZE,
					  receive_transfer, devc,
					  ATK_XFER_TIMEOUT_MS);
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
	struct atk_context *devc = sdi->priv;
	struct drv_context *drvc = di->priv;
	struct timeval tv = { 0, 0 };
	int completed = 0;

	(void)fd;
	(void)revents;
	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx,
					       &tv, &completed);
	if (devc->pending_rearm && !devc->abort && devc->fw_streaming) {
		if (atk_send_trigger(devc) == SR_OK)
			devc->pending_rearm = 0;
	}
	if (devc->pending_stop || (devc->fw_streaming && devc->abort)) {
		if (atk_hw_stop(devc) == SR_OK)
			devc->pending_stop = 0;
	}
	if (devc->status == ATK_ST_FINISH) {
		atk_hw_stop(devc);
		if (devc->freewheel)
			sr_session_source_remove(-1);
		return TRUE;
	}
	return TRUE;
}

/* -------------------- identify -------------------- */

static void atk_try_identify(struct atk_context *devc)
{
	uint8_t buf[ATK_FRAME];
	int got, tries;

	atk_stop_and_drain(devc);
	if (atk_mcu_cmd(devc, ATK_MCU_GET_VER, NULL, 0) != SR_OK)
		return;

	for (tries = 0; tries < 20; tries++) {
		int k;
		if (atk_bulk_in(devc, buf, 512, &got, 80) != SR_OK)
			continue;
		for (k = 0; k + 9 <= got; k++) {
			if (buf[k] == 0x0a && buf[k + 1] == 0x81 &&
			    buf[k + 2] == 0x01 && buf[k + 3] == 0x61) {
				devc->device_level = buf[k + 8];
				sr_info("MCU app level=%d ver=%u.%u dev=%u",
					devc->device_level, buf[k + 4],
					buf[k + 5], buf[k + 6]);
				tries = 20;
				break;
			}
		}
	}

	atk_set_reset(devc, 0);
	g_usleep(20000);
	atk_set_reset(devc, 1);
	g_usleep(30000);
	atk_drain_in(devc);

	if (atk_fpga_cmd(devc, ATK_CMD_GET_DEV, NULL, 0) != SR_OK)
		return;
	g_usleep(30000);

	for (tries = 0; tries < 8; tries++) {
		uint8_t conv[ATK_FRAME];
		int n, i;

		if (atk_bulk_in(devc, buf, sizeof(buf), &got, 80) != SR_OK)
			continue;
		if (got < ATK_FRAME)
			continue;
		n = (got / ATK_FRAME) * ATK_FRAME;
		atk_convert_to_pc(buf, conv, (unsigned)n);
		i = 0;
		while (i + 6 <= n) {
			uint16_t dlen;
			int pkt;
			if (conv[i] != 0x0a || conv[i + 1] != 2) {
				i++;
				continue;
			}
			dlen = (uint16_t)conv[i + 2] | ((uint16_t)conv[i + 3] << 8);
			pkt = dlen + 6;
			if (i + pkt > n)
				break;
			if (conv[i + 4 + dlen] != 0x00 || conv[i + 5 + dlen] != 0x0b) {
				i++;
				continue;
			}
			/* data: [?, ?, status, usb, ...] name at +9 */
			if (dlen >= 10) {
				const uint8_t *d = conv + i + 4;
				if (d[2] == 1) {
					devc->usb3 = (d[3] == 3);
					{
						int nl = dlen - 9;
						if (nl > (int)sizeof(devc->fpga_name) - 1)
							nl = (int)sizeof(devc->fpga_name) - 1;
						memcpy(devc->fpga_name, d + 9, nl);
						devc->fpga_name[nl] = 0;
					}
					sr_info("FPGA usb=%s name=\"%s\"",
						devc->usb3 ? "3.0" : "2.0",
						devc->fpga_name);
				}
			}
			break;
		}
		if (devc->fpga_name[0])
			break;
	}
	atk_set_reset(devc, 0);
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

static void setup_channels(struct sr_dev_inst *sdi)
{
	int i;
	char name[8];

	while (sdi->channels) {
		struct sr_channel *ch = sdi->channels->data;
		sdi->channels = g_slist_delete_link(sdi->channels, sdi->channels);
		g_free(ch->name);
		g_free(ch);
	}
	for (i = 0; i < ATK_MAX_CH; i++) {
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
	struct atk_context *devc;
	GSList *devices = NULL;
	int i;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	sr_info("Scan ATK-Logic device...");
	if (libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist) < 0 ||
	    !devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		struct sr_usb_dev_inst *usb_info;
		uint8_t bus, address;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		if (des.idVendor != ATK_VID || des.idProduct != ATK_PID)
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
		devc->limit_samples = ATK_DEFAULT_SAMPLES;
		devc->max_height = 1;
		devc->vth = 3.3;
		devc->op_mode = LO_OP_STREAM; /* official default isBuffer=false */
		devc->samplerate = SR_MHZ(1);
		devc->rate_index = 0;
		devc->usb_speed = LIBUSB_SPEED_UNKNOWN;

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE,
				      "Alientek", "ATK-Logic", "1.0");
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
		setup_channels(sdi);
		sr_info("Found ATK-Logic 1A86:FFCC bus=%u addr=%u", bus, address);
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

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	struct atk_context *devc = sdi->priv;
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

	devc->usb_speed = libusb_get_device_speed(devc->usb_dev);
	devc->sdi = sdi;
	atk_try_identify(devc);
	{
		uint8_t pwm_off;
		pwm_off = 0x20;
		atk_fpga_cmd(devc, 0x17, &pwm_off, 1);
		pwm_off = 0x10;
		atk_fpga_cmd(devc, 0x17, &pwm_off, 1);
	}
	atk_fpga_cmd(devc, ATK_CMD_STOP, NULL, 0);
	atk_set_reset(devc, 0);
	if (devc->fpga_name[0]) {
		char shown[80];
		snprintf(shown, sizeof(shown), "ATK-Logic %s", devc->fpga_name);
		g_free(sdi->name);
		sdi->name = g_strdup(shown);
	}
	atk_clamp_rate(devc);
	sdi->status = SR_ST_ACTIVE;
	sr_info("open ok speed=%d usb3=%d level=%d name=\"%s\"",
		(int)devc->usb_speed, devc->usb3, devc->device_level,
		devc->fpga_name);
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct atk_context *devc;

	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	if (devc->devhdl) {
		if (sdi->status == SR_ST_ACTIVE) {
			int n;
			struct timeval tv = { 0, 20000 };

			devc->pending_stop = 1;
			atk_request_stop(devc);
			atk_hw_stop(devc);
			for (n = 0; n < 50 &&
			     (devc->submitted_transfers > 0 || devc->out_busy);
			     n++) {
				if (devc->sr_ctx && devc->sr_ctx->libusb_ctx)
					libusb_handle_events_timeout(
						devc->sr_ctx->libusb_ctx, &tv);
			}
			atk_fpga_cmd(devc, ATK_CMD_EXIT, NULL, 0);
		}
		if (devc->out_xfer && !devc->out_busy) {
			libusb_free_transfer(devc->out_xfer);
			devc->out_xfer = NULL;
		}
		g_free(devc->out_buf);
		devc->out_buf = NULL;
		libusb_release_interface(devc->devhdl, 0);
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int hw_dev_destroy(struct sr_dev_inst *sdi)
{
	struct atk_context *devc;
	int i;

	if (!sdi)
		return SR_ERR;
	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		for (i = 0; i < ATK_MAX_CH; i++) {
			if (devc->ch_fifo[i])
				g_byte_array_free(devc->ch_fifo[i], TRUE);
		}
		g_free(devc->parse_res);
		g_free(devc->rle_buf);
		g_free(devc->cross_buf);
		if (devc->out_xfer && !devc->out_busy) {
			libusb_free_transfer(devc->out_xfer);
			devc->out_xfer = NULL;
		}
		g_free(devc->out_buf);
		devc->out_buf = NULL;
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

SR_PRIV void atk_logic_on_usb_reconnected(struct sr_dev_inst *sdi,
					  struct libusb_device *new_dev)
{
	struct atk_context *devc;

	if (!sdi || !sdi->priv || !new_dev)
		return;
	if (!sdi->driver || !sdi->driver->name ||
	    strcmp(sdi->driver->name, "atk-logic") != 0)
		return;
	devc = sdi->priv;
	if (devc->usb_dev)
		libusb_unref_device(devc->usb_dev);
	devc->usb_dev = libusb_ref_device(new_dev);
	devc->sdi = sdi;
	sdi->handle = (ds_device_handle)devc->usb_dev;
	devc->usb_speed = libusb_get_device_speed(new_dev);
	atk_clamp_rate(devc);
	sr_info("reconnected speed=%d", (int)devc->usb_speed);
}

static int hw_dev_status_get(const struct sr_dev_inst *sdi,
			     struct sr_status *status, gboolean prg)
{
	struct atk_context *devc;
	uint64_t n;

	(void)prg;
	if (!status)
		return SR_ERR;
	memset(status, 0, sizeof(*status));
	if (!sdi || !sdi->priv)
		return SR_OK;
	devc = sdi->priv;
	n = devc->hw_progress ? devc->hw_progress : devc->num_samples;
	status->captured_cnt0 = (uint8_t)(n);
	status->captured_cnt1 = (uint8_t)(n >> 8);
	status->captured_cnt2 = (uint8_t)(n >> 16);
	status->captured_cnt3 = (uint8_t)(n >> 24);
	status->trig_hit = (uint8_t)((n >> 32) << 2);
	status->stream_mode = (devc->op_mode == LO_OP_STREAM);
	return SR_OK;
}

static int config_get(int id, GVariant **data, const struct sr_dev_inst *sdi,
		      const struct sr_channel *ch,
		      const struct sr_channel_group *cg)
{
	struct atk_context *devc;

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
		*data = g_variant_new_uint64(ATK_HW_DEPTH);
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16(16);
		break;
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int16(16);
		break;
	case SR_CONF_STREAM:
		*data = g_variant_new_boolean(
			devc->op_mode == LO_OP_STREAM || devc->is_loop);
		break;
	case SR_CONF_VTH:
		*data = g_variant_new_double(devc->vth);
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_int16(devc->op_mode);
		break;
	case SR_CONF_LOOP_MODE:
		*data = g_variant_new_boolean(devc->is_loop != 0);
		break;
	case SR_CONF_USB_SPEED:
		*data = g_variant_new_int32((int32_t)devc->usb_speed);
		break;
	case SR_CONF_USB30_SUPPORT:
		/* DL16 is USB 2.0 only. Do not warn to replug into USB 3. */
		*data = g_variant_new_boolean(devc->usb3 != 0);
		break;
	case SR_CONF_RLE:
		*data = g_variant_new_boolean(devc->use_rle != 0);
		break;
	case SR_CONF_RLE_SUPPORT:
		*data = g_variant_new_boolean(TRUE);
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
	struct atk_context *devc;

	(void)ch;
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
		atk_clamp_rate(devc);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_EN:
		if (ch)
			ch->enabled = g_variant_get_boolean(data);
		atk_clamp_rate(devc);
		break;
	case SR_CONF_VTH:
		devc->vth = g_variant_get_double(data);
		if (devc->vth > 5.0)
			devc->vth = 5.0;
		if (devc->vth < -5.0)
			devc->vth = -5.0;
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
		devc->op_mode = g_variant_get_int16(data);
		atk_clamp_rate(devc);
		break;
	case SR_CONF_LOOP_MODE:
		devc->is_loop = g_variant_get_boolean(data) ? 1 : 0;
		if (devc->is_loop)
			devc->op_mode = LO_OP_STREAM;
		atk_clamp_rate(devc);
		break;
	case SR_CONF_RLE:
		devc->use_rle = g_variant_get_boolean(data) ? 1 : 0;
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
	static uint64_t rate_buf[16];
	int n = 0, i;
	struct atk_context *devc =
		(sdi && sdi->priv) ? (struct atk_context *)sdi->priv : NULL;
	uint64_t maxr = SR_MHZ(250);

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
			atk_clamp_rate(devc);
			maxr = atk_max_rate(devc, atk_en_count(sdi));
		}
		for (i = 0; i < (int)ARRAY_SIZE(atk_rate_table); i++) {
			if (atk_rate_table[i] <= maxr)
				rate_buf[n++] = atk_rate_table[i];
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
	struct atk_context *devc = sdi->priv;
	const struct libusb_pollfd **lupfd;
	struct drv_context *drvc = di->priv;
	unsigned int i;
	int ret, c;

	(void)cb_data;
	if (sdi->status != SR_ST_ACTIVE) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	devc->sdi = sdi;
	devc->num_samples = 0;
	devc->abort = 0;
	devc->got_end = 0;
	devc->fw_streaming = 0;
	devc->status = ATK_ST_START;
	devc->freewheel = 0;
	devc->parse_res_len = 0;
	devc->trig_hit = 0;
	devc->hw_progress = 0;
	devc->pending_rearm = 0;
	devc->pending_stop = 0;
	devc->usb_rem_len = 0;
	if (devc->is_loop)
		devc->op_mode = LO_OP_STREAM;
	devc->hold_emit = atk_is_buffer(devc) && !devc->is_loop;
	memset(devc->skip_bits, 0, sizeof(devc->skip_bits));
	memset(devc->ch_seen, 0, sizeof(devc->ch_seen));
	atk_refresh_enabled(devc);
	atk_clamp_rate(devc);
	for (c = 0; c < ATK_MAX_CH; c++) {
		if (devc->ch_fifo[c])
			g_byte_array_set_size(devc->ch_fifo[c], 0);
	}

	atk_set_reset(devc, 1);
	g_usleep(20000);
	atk_stop_and_drain(devc);

	sr_info("acq start en=%d rate=%" PRIu64 " limit=%" PRIu64
		" buf=%d rle=%d hold=%d",
		devc->en_count, devc->samplerate, devc->limit_samples,
		atk_is_buffer(devc), devc->use_rle, devc->hold_emit);

	if ((ret = atk_send_param(devc)) != SR_OK)
		return ret;
	g_usleep(30000);
	/* Official: ParameterSetting, wait, SimpleTrigger, THEN USB IN. */
	if ((ret = atk_send_trigger(devc)) != SR_OK)
		return ret;
	devc->fw_streaming = 1;

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

	std_session_send_df_header(sdi, LOG_PREFIX);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct atk_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;
	devc = sdi->priv;
	devc->pending_stop = 1;
	if (devc->hold_emit)
		atk_flush_held(devc);
	if (!devc->abort)
		atk_request_stop(devc);
	return SR_OK;
}

SR_PRIV struct sr_dev_driver atk_logic_driver_info = {
	.name = "atk-logic",
	.longname = "Alientek ATK-Logic",
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
