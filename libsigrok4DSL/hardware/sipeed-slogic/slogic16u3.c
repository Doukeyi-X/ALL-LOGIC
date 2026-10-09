/*
 * Sipeed SLogic driver for DSView / libsigrok4DSL
 *
 * USB protocol (Sipeed libsigrok slogic-dev):
 *   VID 0x359F  PID 0x0300 (SLogic Combo 8),
 *                 0x3031 (SLogic16U3), or
 *                 0x3032 (SLogic32U3)
 *   EP0 vendor control: 32-bit register read/write
 *     bRequest 0x00 REG_READ  / 0x01 REG_WRITE
 *     wValue   register address (byte), 4 bytes per transfer
 *   Registers:
 *     0x0004 R32_CTRL  bit0=RUN  bit1=RST
 *     0x000C R32_AUX   mailbox header + payload at 0x0010
 *   AUX commands: 1=channel mask  2=samplerate  3=vref  5=test mode
 *   Bulk IN: packed sample stream (no header).  Combo 8 uses EP 0x81;
 *   both U3 models use EP 0x82.
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
#include "slogic/slogic.h"

#undef LOG_PREFIX
#define LOG_PREFIX "slogic16u3: "


#define SLOGIC_EP_COMBO8_IN      0x81
#define SLOGIC_EP_U3_IN           0x82

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
#define SLOGIC_AUX_CMD_TEST      0x00000005u

#define SLOGIC_COMBO8_CMD_START  0xb1
#define SLOGIC_CTRL_TIMEOUT_MS   500
#define SLOGIC_MAX_TRANSFERS     16
#define SLOGIC_TRANSFER_SIZE_MIN (32 * 1024)
#define SLOGIC_TRANSFER_SIZE_MAX (3 * 1024 * 1024)
#define SLOGIC_TRANSFER_ALIGN    (32 * 1024)
#define SLOGIC_TRANSFER_TOLERANCE 0.30

/* Upper bound on helper threads used to convert the wire format. */
#define SLOGIC_MAX_CONV_THREADS 8
/* The U3 firmware occasionally misses the RUN command and then never streams
 * anything; re-issue it this many times before giving up on the capture. */
#define SLOGIC_MAX_START_RETRIES 1
#define SLOGIC_DROP_FIRST_BYTES  4
/* Once data is flowing, a transfer that arrives late only means the host is
 * draining slower than the analyzer produces (USB backpressure), which is
 * normal for long captures.  Only treat the link as dead when nothing at all
 * arrives for this long. */
#define SLOGIC_STREAM_IDLE_US    1000000

#define SLOGIC_DEFAULT_SAMPLES   SR_Mn(1)
#define SLOGIC_HW_DEPTH          SR_Mn(128)
#define SLOGIC_MAX_PHYS_CH       32

#define SLOGIC_CROSS_SCALE       64
#define SLOGIC_CROSS_PLANE_BYTES 8

/* The application is often built as a debug binary. Keep the hot bit
 * transpose path optimized in that configuration as well; otherwise the
 * USB producer can outrun the converter at 800 MB/s. */
#if defined(__GNUC__) && !defined(__clang__)
#define SLOGIC_HOT __attribute__((optimize("O3"), hot))
#define SLOGIC_ALWAYS_INLINE __attribute__((always_inline))
#else
#define SLOGIC_HOT
#define SLOGIC_ALWAYS_INLINE
#endif

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
	/* Keep the historical 16U3 IDs (0=16, 1=8, 2=4). */
	SLOGIC_CHMODE_16 = 0,
	SLOGIC_CHMODE_8 = 1,
	SLOGIC_CHMODE_4 = 2,
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

static const struct sr_list_item channel_mode_16u3_list[] = {
	{ SLOGIC_CHMODE_16, "Use Channels 0~15 (Max 200MHz)" },
	{ SLOGIC_CHMODE_8, "Use Channels 0~7 (Max 400MHz)" },
	{ SLOGIC_CHMODE_4, "Use Channels 0~3 (Max 800MHz)" },
	{ -1, NULL },
};

static const struct sr_list_item channel_mode_combo8_list[] = {
	{ 0, "Use Channels 0~7 (Max 40MHz)" },
	{ 1, "Use Channels 0~3 (Max 80MHz)" },
	{ 2, "Use Channels 0~1 (Max 160MHz)" },
	{ -1, NULL },
};

static const struct sr_list_item channel_mode_32u3_list[] = {
	{ 0, "Use Channels 0~31 (Max 200MHz)" },
	{ 1, "Use Channels 0~15 (Max 400MHz)" },
	{ 2, "Use Channels 0~7 (Max 800MHz)" },
	{ 3, "Use Channels 0~3 (Max 1400MHz)" },
	{ -1, NULL },
};

static const char *pattern_modes[] = {
	"Normal",
	"USB connection test",
	"Emulation",
};

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_PATTERN_MODE,
	SR_CONF_FILTER,
	SR_CONF_VTH,
	SR_CONF_MAX_HEIGHT,
};

static const int32_t sessions[] = {
	SR_CONF_MAX_HEIGHT,
	SR_CONF_OPERATION_MODE,
	SR_CONF_PATTERN_MODE,
	SR_CONF_CHANNEL_MODE,
	SR_CONF_SAMPLERATE,
	SR_CONF_LIMIT_SAMPLES,
	SR_CONF_VTH,
	SR_CONF_FILTER,
};

static const char *probe_names[] = {
	"0", "1", "2", "3", "4", "5", "6", "7",
	"8", "9", "10", "11", "12", "13", "14", "15",
	"16", "17", "18", "19", "20", "21", "22", "23",
	"24", "25", "26", "27", "28", "29", "30", "31",
	NULL,
};


struct slogic_context {
	const struct slogic_model *model;
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
	int pattern_mode;
	double vth;

	uint64_t samplerate;
	uint64_t limit_samples;
	uint64_t num_samples;
	uint64_t num_bytes;

	enum libusb_speed usb_speed;

	uint32_t filt_prev_in;
	uint32_t filt_prev_out;
	int filt_have_prev;

	volatile gint status;
	volatile gint abort;
	volatile gint fw_streaming;
	volatile gint callbacks_active;
	int submitted_transfers;
	int num_transfers;
	struct libusb_transfer **transfers;
	int freewheel;
	GThread *libusb_event_thread;
	volatile gint libusb_event_thread_run;
	GMutex transfer_lock;
	GAsyncQueue *raw_queue;
	int source_added;
	int end_sent;
	int user_stop;
	volatile gint discard_queue;
	int hw_stop_attempts;
	uint64_t samples_need_bytes;
	uint64_t raw_received_bytes;
	uint64_t transfer_size;
	uint64_t transfer_duration_ms;
	uint64_t expected_rate_bytes;
	uint64_t transfers_completed;
	uint64_t transfers_received_bytes;
	uint64_t transfers_received_bytes_latest;
	int64_t transfers_time_start;
	int64_t transfers_time_latest;
	int64_t transfers_time_data;
	unsigned int timeout_count;
	unsigned int timeout_count_limit;
	int slow_warned;
	int silent_abort;
	volatile gint restart_pending;
	volatile gint start_retries;

	int drop_left;

	uint32_t *raw_pending;
	int raw_pending_len;
	int raw_pending_cap;

	uint8_t stream_res[4];
	int stream_res_len;

	uint8_t *cross_buf;
	int cross_cap;
	int cross_len;
	uint8_t *fast_pending;
	size_t fast_pending_len;
	size_t fast_pending_cap;

	uint8_t en_bits[SLOGIC_MAX_PHYS_CH];
	int en_count;

	/* Parallel LA_CROSS_DATA conversion.  The host has to bit-transpose the
	 * interleaved wire format into 64-sample channel planes, which costs more
	 * CPU time than the USB link needs to deliver the data.  A private pool
	 * splits each USB packet into contiguous group ranges so the output stays
	 * byte-for-byte identical to the serial conversion. */
	GThreadPool *conv_pool;
	int conv_threads;
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


static const struct sr_list_item *slogic_channel_mode_list(
		const struct slogic_model *model)
{
	if (!model)
		return channel_mode_16u3_list;
	if (model->proto == SLOGIC_PROTO_COMBO8)
		return channel_mode_combo8_list;
	if (model->physical_channels == 32)
		return channel_mode_32u3_list;
	return channel_mode_16u3_list;
}

static int slogic_mode_channels(const struct slogic_context *devc, int mode)
{
	if (!devc || !devc->model || mode < 0 ||
		(size_t)mode >= devc->model->limit_count)
		return 16;
	return (devc->model->physical_channels >> (mode));
}

static uint64_t slogic_link_max_rate(const struct slogic_context *devc)
{
	const struct slogic_model *model = devc ? devc->model : NULL;
	int mode = devc ? devc->ch_mode : 0;
	uint64_t limit;

	if (!model)
		model = slogic_model_for_pid(SLOGIC_PID_16U3);
	if (mode < 0 || (size_t)mode >= model->limit_count)
		mode = 0;

	/* The reference driver publishes one limit for each channel mode. */
	limit = slogic_max_rate(model, model->physical_channels >> (mode));
	/* U3 analyzers also enumerate through USB high-speed.  Keep the rate
	 * within the practical USB2 payload limit when the link is known. */
	if (devc && model->proto == SLOGIC_PROTO_U3 &&
		devc->usb_speed != LIBUSB_SPEED_UNKNOWN && !slogic_is_usb3(devc)) {
		int nch = (model->physical_channels >> (mode));
		uint64_t usb2_limit = SR_MHZ(320) / (uint64_t)nch;
		if (limit > usb2_limit)
			limit = usb2_limit;
	}
	return limit;
}

static void slogic_apply_model_name(struct sr_dev_inst *sdi,
					    const struct slogic_model *model,
					    enum libusb_speed sp)
{
	char *name;

	if (!sdi || !model)
		return;
	if (model->proto == SLOGIC_PROTO_COMBO8 || sp == LIBUSB_SPEED_UNKNOWN)
		name = g_strdup(model->name);
	else if (sp == LIBUSB_SPEED_SUPER || sp == LIBUSB_SPEED_SUPER_PLUS)
		name = g_strdup_printf("%s USB3.0", model->name);
	else if (sp == LIBUSB_SPEED_HIGH)
		name = g_strdup_printf("%s USB2.0", model->name);
	else
		name = g_strdup(model->name);
	g_free(sdi->name);
	sdi->name = name;
}

static void slogic_update_usb_speed(struct slogic_context *devc)
{
	enum libusb_speed sp = LIBUSB_SPEED_UNKNOWN;

	if (!devc || !devc->usb_dev)
		return;
	sp = libusb_get_device_speed(devc->usb_dev);
	devc->usb_speed = sp;
	if (devc->sdi)
		slogic_apply_model_name((struct sr_dev_inst *)devc->sdi,
					devc->model, sp);
}

static uint64_t slogic_pick_rate(const struct slogic_context *devc,
					 uint64_t want)
{
	unsigned int i;
	const struct slogic_model *model = devc && devc->model ?
		devc->model : slogic_model_for_pid(SLOGIC_PID_16U3);
	uint64_t link_max = slogic_link_max_rate(devc);
	uint64_t best = 0;
	uint64_t best_err = UINT64_MAX;

	if (want == 0)
		want = link_max;
	if (want > link_max)
		want = link_max;

	for (i = 0; i < model->rate_count; i++) {
		uint64_t r = model->rates[i];
		uint64_t err;

		if (r > link_max)
			continue;
		err = (r > want) ? (r - want) : (want - r);
		if (err < best_err) {
			best_err = err;
			best = r;
		}
	}
	if (!best)
		best = model->rates[0];
	return best;
}

static void slogic_map_samplerate(struct slogic_context *devc, uint64_t want)
{
	devc->samplerate = slogic_pick_rate(devc, want);
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
	/* The caller must size `out` for the whole list (32 entries max) plus
	 * the 0 terminator written below. */
	uint64_t link_max = slogic_link_max_rate(devc);
	const struct slogic_model *model = devc && devc->model ?
		devc->model : slogic_model_for_pid(SLOGIC_PID_16U3);
	unsigned int i;
	int n = 0;
	const int cap = 32;

	for (i = 0; i < model->rate_count; i++) {
		if (model->rates[i] <= link_max)
			n = slogic_rate_list_add(out, n, cap, model->rates[i]);
	}
	out[n] = 0;
	*out_n = n;
}

/* -------------------- register / AUX -------------------- */

/* Combo 8 has a small command protocol rather than the U3 register map. */
/* -------------------- shared-core transport shim (P2) --------------------
 * The register/AUX protocol lives once in the vendored slogic/ core; this
 * adapter only provides the USB control-transfer transport and maps the
 * DSView-side config into a slogic_config, then calls slogic_reset/configure/
 * run/stop. */
static int al_ctrl_write(void *ctx, uint8_t b_request, uint16_t w_value,
			 uint16_t w_index, const uint8_t *data,
			 uint16_t len, unsigned timeout_ms)
{
	struct slogic_context *devc = ctx;

	return libusb_control_transfer(devc->devhdl,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, b_request,
		w_value, w_index, (unsigned char *)data, len, (int)timeout_ms);
}

static int al_ctrl_read(void *ctx, uint8_t b_request, uint16_t w_value,
			uint16_t w_index, uint8_t *data, uint16_t len,
			unsigned timeout_ms)
{
	struct slogic_context *devc = ctx;

	return libusb_control_transfer(devc->devhdl,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN, b_request,
		w_value, w_index, data, len, (int)timeout_ms);
}

static void al_transport(struct slogic_context *devc, slogic_transport *t)
{
	t->ctx = devc;
	t->control_write = al_ctrl_write;
	t->control_read = al_ctrl_read;
}

static void al_config(struct slogic_context *devc, slogic_config *c)
{
	*c = (slogic_config){
		.channel_count = devc->channel_count,
		.samplerate_hz = devc->samplerate,
		.threshold_v = devc->vth,
		.pattern_mode = devc->pattern_mode,
	};
}

static int slogic_hw_reset(struct slogic_context *devc)
{
	slogic_transport t;

	if (!devc || !devc->model)
		return SR_ERR_ARG;
	al_transport(devc, &t);
	return slogic_reset(devc->model, &t) == SLOGIC_OK ? SR_OK : SR_ERR;
}

static void slogic_drain_ep(struct slogic_context *devc);

static int slogic_hw_start(struct slogic_context *devc)
{
	slogic_transport t;
	slogic_config c;

	if (!devc || !devc->model)
		return SR_ERR_ARG;
	al_transport(devc, &t);
	al_config(devc, &c);
	/* U3 writes CTRL=RUN; Combo 8 carries rate + channel count in its start
	 * command (the core handles both from the config). */
	return slogic_run(devc->model, &t, &c) == SLOGIC_OK ? SR_OK : SR_ERR;
}

static void slogic_hw_stop(struct slogic_context *devc)
{
	int ret;

	if (!devc || !g_atomic_int_get(&devc->fw_streaming))
		return;
	if (!devc->devhdl) {
		g_atomic_int_set(&devc->fw_streaming, 0);
		return;
	}
	if (devc->model && devc->model->proto == SLOGIC_PROTO_COMBO8) {
		/* CMD_STOP is unreliable in current Combo8 firmware; draining EP1
		 * is the upstream driver's documented stop sequence. */
		slogic_drain_ep(devc);
		g_atomic_int_set(&devc->fw_streaming, 0);
		return;
	}
	{
		slogic_transport t;
		al_transport(devc, &t);
		ret = slogic_stop(devc->model, &t) == SLOGIC_OK ? SR_OK : SR_ERR;
	}
	if (ret != SR_OK) {
		devc->hw_stop_attempts++;
		sr_err("HW stop failed, will retry (%d/10)",
			devc->hw_stop_attempts);
		if (devc->hw_stop_attempts >= 10) {
			sr_err("HW stop unavailable; completing host session");
			g_atomic_int_set(&devc->fw_streaming, 0);
		}
	} else {
		devc->hw_stop_attempts = 0;
		g_atomic_int_set(&devc->fw_streaming, 0);
		sr_info("HW stop sent");
	}
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
		r = libusb_bulk_transfer(devc->devhdl,
				 devc->model ? devc->model->ep_in : SLOGIC_EP_U3_IN, tmp,
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
	int n = slogic_mode_channels(devc, devc->ch_mode);

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
	uint32_t *p;

	if (need <= devc->raw_pending_cap)
		return SR_OK;
	need = (need + 1023) & ~1023;
	p = g_try_realloc(devc->raw_pending, (gsize)need * sizeof(uint32_t));
	if (!p)
		return SR_ERR_MALLOC;
	devc->raw_pending = p;
	devc->raw_pending_cap = need;
	return SR_OK;
}

static uint32_t slogic_apply_1t_filter(struct slogic_context *devc, uint32_t in)
{
	uint32_t outv;

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

static int slogic_push_sample(struct slogic_context *devc, uint32_t s)
{
	if (slogic_ensure_pending_cap(devc, devc->raw_pending_len + 1) != SR_OK)
		return SR_ERR_MALLOC;
	devc->raw_pending[devc->raw_pending_len++] =
		slogic_apply_1t_filter(devc, s);
	return SR_OK;
}

/*
 * Unpack the wire stream into parallel samples (bit i = Di).
 * Samples below eight channels are bit-packed (2-channel: four samples per
 * byte, 4-channel: two samples per byte).  Eight, sixteen and thirty-two
 * channel modes use one, two and four little-endian bytes per sample.
 */
static int slogic_unpack_append(struct slogic_context *devc,
				const uint8_t *raw, int raw_len)
{
	int nch = devc->channel_count;
	const uint8_t *p = raw;
	int left = raw_len;
	int bytes_per_sample;
	uint32_t mask;

	if (!raw || raw_len <= 0)
		return SR_OK;

	if (nch < 1)
		return SR_ERR_ARG;

	if (nch < 8) {
		const int samples_per_byte = 8 / nch;
		mask = (UINT32_C(1) << nch) - 1u;
		while (left-- > 0) {
			uint8_t b = *p++;
			int i;
			for (i = 0; i < samples_per_byte; i++) {
				if (slogic_push_sample(devc,
						((uint32_t)b >> (i * nch)) & mask) != SR_OK)
					return SR_ERR_MALLOC;
			}
		}
		return SR_OK;
	}

	bytes_per_sample = (nch + 7) / 8;
	if (bytes_per_sample > (int)sizeof(devc->stream_res))
		return SR_ERR_ARG;
	if (devc->stream_res_len >= bytes_per_sample)
		devc->stream_res_len = 0;
	/* Complete a sample split over two USB transfers. */
	while (devc->stream_res_len > 0 &&
	       devc->stream_res_len < bytes_per_sample && left > 0) {
		devc->stream_res[devc->stream_res_len++] = *p++;
		left--;
		if (devc->stream_res_len == bytes_per_sample) {
			uint32_t s = 0;
			int i;
			for (i = 0; i < bytes_per_sample; i++)
				s |= (uint32_t)devc->stream_res[i] << (8 * i);
			if (slogic_push_sample(devc, s) != SR_OK)
				return SR_ERR_MALLOC;
			devc->stream_res_len = 0;
		}
	}
	while (left >= bytes_per_sample) {
		uint32_t s = 0;
		int i;
		for (i = 0; i < bytes_per_sample; i++)
			s |= (uint32_t)p[i] << (8 * i);
		if (slogic_push_sample(devc, s) != SR_OK)
			return SR_ERR_MALLOC;
		p += bytes_per_sample;
		left -= bytes_per_sample;
	}
	if (left > 0) {
		memcpy(devc->stream_res, p, (size_t)left);
		devc->stream_res_len = left;
	}
	return SR_OK;
}

static int slogic_ensure_fast_cap(struct slogic_context *devc, size_t need)
{
	uint8_t *p;

	if (need <= devc->fast_pending_cap)
		return SR_OK;
	/* Keep the pending area bounded to a few USB packets.  Normally only
	 * the final incomplete 64-sample group remains here. */
	if (need < 4096)
		need = 4096;
	need = (need + SLOGIC_TRANSFER_ALIGN - 1) &
		~(size_t)(SLOGIC_TRANSFER_ALIGN - 1);
	p = g_try_realloc(devc->fast_pending, need);
	if (!p)
		return SR_ERR_MALLOC;
	devc->fast_pending = p;
	devc->fast_pending_cap = need;
	return SR_OK;
}

/* Transpose eight bytes (eight consecutive samples of one byte lane). */
static inline SLOGIC_HOT SLOGIC_ALWAYS_INLINE uint64_t
slogic_transpose8x8(const uint8_t *in)
{
	uint64_t x, t;

	memcpy(&x, in, sizeof(x));
	t = (x ^ (x >> 7)) & UINT64_C(0x00aa00aa00aa00aa);
	x ^= t ^ (t << 7);
	t = (x ^ (x >> 14)) & UINT64_C(0x0000cccc0000cccc);
	x ^= t ^ (t << 14);
	t = (x ^ (x >> 28)) & UINT64_C(0x00000000f0f0f0f0);
	x ^= t ^ (t << 28);
	return x;
}

static void slogic_forward_cross(struct slogic_context *devc, int out_len)
{
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;

	if (out_len <= 0)
		return;
	memset(&packet, 0, sizeof(packet));
	memset(&logic, 0, sizeof(logic));
	packet.type = SR_DF_LOGIC;
	packet.status = SR_PKT_OK;
	packet.payload = &logic;
	logic.length = (uint64_t)out_len;
	logic.format = LA_CROSS_DATA;
	logic.unitsize = (uint16_t)((devc->en_count + 7) / 8);
	if (logic.unitsize == 0)
		logic.unitsize = 1;
	logic.data = devc->cross_buf;
	ds_data_forward(devc->sdi, &packet);
	devc->num_bytes += (uint64_t)out_len;
}

/* Convert `groups` complete 64-sample groups starting at src into the channel
 * planes at dst.  Groups are independent and each one writes exactly
 * SLOGIC_CROSS_PLANE_BYTES * en bytes at a fixed offset, so a group range can
 * be converted on its own (see the worker pool below). */
static SLOGIC_HOT void slogic_convert_groups(const struct slogic_context *devc,
					     const uint8_t *src, uint8_t *dst,
					     size_t groups)
{
	const int bytes_per_sample = (devc->channel_count + 7) / 8;
	const size_t group_raw = (size_t)SLOGIC_CROSS_SCALE * bytes_per_sample;
	const size_t cross_group = (size_t)SLOGIC_CROSS_PLANE_BYTES *
				   (size_t)devc->en_count;
	size_t g, block, c;

	/* Channels that are not enabled leave their plane byte untouched. */
	memset(dst, 0, groups * cross_group);
	for (g = 0; g < groups; g++) {
		const uint8_t *gsrc = src + g * group_raw;
		uint8_t *gdst = dst + g * cross_group;

		for (block = 0; block < SLOGIC_CROSS_SCALE / 8; block++) {
			const uint8_t *samples =
				gsrc + block * (size_t)bytes_per_sample * 8;
			uint64_t transposed[4] = { 0, 0, 0, 0 };
			uint8_t lane_samples[8];
			int lane;

			for (lane = 0; lane < bytes_per_sample; lane++) {
				int sample;
				for (sample = 0; sample < 8; sample++)
					lane_samples[sample] =
						samples[sample * bytes_per_sample + lane];
				transposed[lane] = slogic_transpose8x8(
					lane_samples);
			}
			for (c = 0; c < (size_t)devc->en_count; c++) {
				int bit = devc->en_bits[c];
				int lane_bit;

				lane = bit / 8;
				if (lane >= bytes_per_sample)
					continue;
				lane_bit = bit % 8;
				gdst[c * SLOGIC_CROSS_PLANE_BYTES + block] =
					(uint8_t)(transposed[lane] >>
						  (lane_bit * 8));
			}
		}
	}
}

struct slogic_conv_batch {
	GMutex lock;
	GCond cond;
	gint left;
};

struct slogic_conv_chunk {
	const struct slogic_context *devc;
	const uint8_t *src;
	uint8_t *dst;
	size_t groups;
	struct slogic_conv_batch *batch;
};

static void slogic_conv_chunk_run(struct slogic_conv_chunk *chunk)
{
	slogic_convert_groups(chunk->devc, chunk->src, chunk->dst,
			      chunk->groups);
	g_mutex_lock(&chunk->batch->lock);
	if (--chunk->batch->left == 0)
		g_cond_signal(&chunk->batch->cond);
	g_mutex_unlock(&chunk->batch->lock);
}

static void slogic_conv_chunk_worker(gpointer data, gpointer user_data)
{
	(void)user_data;
	slogic_conv_chunk_run((struct slogic_conv_chunk *)data);
}

static SLOGIC_HOT int slogic_convert_fast(struct slogic_context *devc,
			       const uint8_t *raw, int raw_len)
{
	const int nch = devc->channel_count;
	const int en = devc->en_count;
	const int bytes_per_sample = (nch + 7) / 8;
	const size_t group_raw = (size_t)SLOGIC_CROSS_SCALE * bytes_per_sample;
	const int cross_group = SLOGIC_CROSS_PLANE_BYTES * en;
	const uint8_t *src;
	size_t groups, consumed;
	size_t raw_groups = 0;
	uint8_t *out;
	int out_len;

	if (en < 1 || raw_len <= 0 || bytes_per_sample < 1 || bytes_per_sample > 4)
		return SR_OK;
	/* Nothing staged and a whole number of 64-sample groups in this
	 * transfer (the normal case): convert straight out of the USB buffer.
	 * Staging a copy of it would add a full extra pass over 800 MB. */
	if (devc->fast_pending_len == 0 &&
	    ((size_t)raw_len % group_raw) == 0) {
		src = raw;
		groups = (size_t)raw_len / group_raw;
		raw_groups = groups;
	} else {
		if (slogic_ensure_fast_cap(devc, devc->fast_pending_len +
				(size_t)raw_len) != SR_OK)
			return SR_ERR_MALLOC;
		memcpy(devc->fast_pending + devc->fast_pending_len, raw,
			(size_t)raw_len);
		devc->fast_pending_len += (size_t)raw_len;
		src = devc->fast_pending;
		groups = devc->fast_pending_len / group_raw;
	}
	if (devc->limit_samples && !devc->is_loop) {
		uint64_t room = devc->num_samples < devc->limit_samples
			? devc->limit_samples - devc->num_samples : 0;
		size_t max_groups = (size_t)(room / SLOGIC_CROSS_SCALE);
		if (groups > max_groups)
			groups = max_groups;
	}
	if (groups == 0)
		return SR_OK;

	out_len = (int)(groups * (size_t)cross_group);
	if (out_len <= 0 || slogic_ensure_cross_cap(devc, out_len) != SR_OK)
		return SR_ERR_MALLOC;
	out = devc->cross_buf;
	if (devc->conv_pool && devc->conv_threads > 1 &&
	    groups >= (size_t)devc->conv_threads * 2) {
		const size_t n = (size_t)devc->conv_threads;
		struct slogic_conv_batch batch;
		struct slogic_conv_chunk *chunks =
			g_new0(struct slogic_conv_chunk, n);
		size_t base = 0;
		size_t i;

		g_mutex_init(&batch.lock);
		g_cond_init(&batch.cond);
		batch.left = (gint)n;
		for (i = 0; i < n; i++) {
			size_t count = groups / n + (i < groups % n ? 1 : 0);

			chunks[i].devc = devc;
			chunks[i].src = src + base * group_raw;
			chunks[i].dst = out + base * (size_t)cross_group;
			chunks[i].groups = count;
			chunks[i].batch = &batch;
			base += count;
			/* Fall back to running the chunk here if the pool refuses
			 * the job, otherwise the batch would never complete. */
			if (!g_thread_pool_push(devc->conv_pool, &chunks[i], NULL))
				slogic_conv_chunk_run(&chunks[i]);
		}
		g_mutex_lock(&batch.lock);
		while (batch.left > 0)
			g_cond_wait(&batch.cond, &batch.lock);
		g_mutex_unlock(&batch.lock);
		g_cond_clear(&batch.cond);
		g_mutex_clear(&batch.lock);
		g_free(chunks);
	} else {
		slogic_convert_groups(devc, src, out, groups);
	}
	slogic_forward_cross(devc, out_len);
	devc->num_samples += (uint64_t)groups * SLOGIC_CROSS_SCALE;
	consumed = groups * group_raw;
	if (raw_groups != 0) {
		/* Fast path: keep only the clamped tail for the next transfer. */
		size_t left = (size_t)raw_len - consumed;

		devc->fast_pending_len = 0;
		if (left > 0) {
			if (slogic_ensure_fast_cap(devc, left) != SR_OK)
				return SR_ERR_MALLOC;
			memcpy(devc->fast_pending, raw + consumed, left);
			devc->fast_pending_len = left;
		}
	} else {
		if (consumed < devc->fast_pending_len)
			memmove(devc->fast_pending, devc->fast_pending + consumed,
				devc->fast_pending_len - consumed);
		devc->fast_pending_len -= consumed;
	}
	return SR_OK;
}

static int slogic_convert_slow(struct slogic_context *devc,
			       const uint8_t *raw, int raw_len)
{
	const int en = devc->en_count;
	const int cross_group = SLOGIC_CROSS_PLANE_BYTES * en;
	int groups, g, c, i, consumed, left;
	uint8_t *out;
	int out_len;
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
		const uint32_t *blk = devc->raw_pending + g * SLOGIC_CROSS_SCALE;
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

	slogic_forward_cross(devc, out_len);

	devc->num_samples += samples_out;

	consumed = groups * SLOGIC_CROSS_SCALE;
	left = devc->raw_pending_len - consumed;
	if (left > 0)
		memmove(devc->raw_pending, devc->raw_pending + consumed,
			(size_t)left * sizeof(uint32_t));
	devc->raw_pending_len = left;
	return SR_OK;
}

static int slogic_convert_push(struct slogic_context *devc,
			       const uint8_t *raw, int raw_len)
{
	if (devc->channel_count >= 8 && devc->filter == SR_FILTER_NONE)
		return slogic_convert_fast(devc, raw, raw_len);
	return slogic_convert_slow(devc, raw, raw_len);
}

/* -------------------- transfers -------------------- */

static void slogic_cancel_transfers(struct slogic_context *devc)
{
	unsigned int i;

	if (!devc)
		return;
	g_mutex_lock(&devc->transfer_lock);
	for (i = 0; i < (unsigned)devc->num_transfers &&
		    i < SLOGIC_MAX_TRANSFERS; i++) {
		if (devc->transfers && devc->transfers[i])
			/* Keep the pointer alive until libusb has accepted the cancel. */
			libusb_cancel_transfer(devc->transfers[i]);
	}
	g_mutex_unlock(&devc->transfer_lock);
}

static int slogic_submitted_count(struct slogic_context *devc)
{
	int n;

	g_mutex_lock(&devc->transfer_lock);
	n = devc->submitted_transfers;
	g_mutex_unlock(&devc->transfer_lock);
	return n;
}

static int slogic_active_callbacks(struct slogic_context *devc)
{
	return g_atomic_int_get(&devc->callbacks_active);
}

static void slogic_clear_raw_queue(struct slogic_context *devc)
{
	GByteArray *array;

	if (!devc || !devc->raw_queue)
		return;
	while ((array = g_async_queue_try_pop(devc->raw_queue)) != NULL)
		g_byte_array_unref(array);
}

/* Remove a completed transfer from the live set. The array itself remains
 * allocated until the session callback has observed that all callbacks have
 * returned. */
static void slogic_transfer_done(struct slogic_context *devc,
				 struct libusb_transfer *transfer)
{
	unsigned int i;

	g_mutex_lock(&devc->transfer_lock);
	for (i = 0; i < (unsigned)devc->num_transfers; i++) {
		if (devc->transfers && devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			if (devc->submitted_transfers > 0)
				devc->submitted_transfers--;
			break;
		}
	}
	g_mutex_unlock(&devc->transfer_lock);

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);
}

static void slogic_request_acq_stop(struct slogic_context *devc,
					gboolean discard)
{
	if (!devc)
		return;
	g_atomic_int_set(&devc->abort, 1);
	devc->status = SLOGIC_ST_STOP;
	if (discard)
		g_atomic_int_set(&devc->discard_queue, 1);
	slogic_cancel_transfers(devc);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct slogic_context *devc = transfer ? transfer->user_data : NULL;
	uint8_t *owned = NULL;
	int len = 0;
	int ret;
	int64_t now;
	int64_t delta;
	gboolean resubmit = FALSE;
	gboolean queued = FALSE;
	gboolean reached_limit = FALSE;

	if (!devc || !transfer)
		return;
	g_atomic_int_inc(&devc->callbacks_active);

	now = g_get_monotonic_time();
	delta = devc->transfers_time_latest ?
		now - devc->transfers_time_latest : 0;
	devc->transfers_completed++;
	devc->transfers_time_latest = now;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED ||
	    transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
		len = transfer->actual_length;
		if (len > 0) {
			if (devc->drop_left > 0) {
				int drop = len < devc->drop_left ? len : devc->drop_left;
				memmove(transfer->buffer, transfer->buffer + drop,
					(size_t)(len - drop));
				len -= drop;
				devc->drop_left -= drop;
			}
			if (!devc->is_loop && devc->samples_need_bytes > 0) {
				uint64_t room = devc->samples_need_bytes >
					devc->raw_received_bytes ?
					devc->samples_need_bytes -
					devc->raw_received_bytes : 0;
				if ((uint64_t)len > room)
					len = (int)room;
			}
			if (len > 0) {
				devc->raw_received_bytes += (uint64_t)len;
				devc->transfers_received_bytes_latest = (uint64_t)len;
				devc->transfers_received_bytes += (uint64_t)len;
				if (devc->raw_queue &&
				    !g_atomic_int_get(&devc->discard_queue)) {
					owned = transfer->buffer;
					transfer->buffer = NULL;
					GByteArray *array =
						g_byte_array_new_take(owned, (gsize)len);
					if (array) {
						g_async_queue_push(devc->raw_queue, array);
						queued = TRUE;
					} else {
						g_free(owned);
						owned = NULL;
						g_atomic_int_set(&devc->abort, 1);
						g_atomic_int_set(&devc->discard_queue, 1);
					}
				}
			}
			if (!devc->is_loop && devc->samples_need_bytes > 0 &&
			    devc->raw_received_bytes >= devc->samples_need_bytes) {
				reached_limit = TRUE;
				g_atomic_int_set(&devc->abort, 1);
				devc->status = SLOGIC_ST_STOP;
			}
		}
		/*
		 * Throughput guard. The reference driver evaluates this for every
		 * transfer, including the ones that complete with zero bytes.
		 * Keeping it inside the "len > 0" branch above meant a device that
		 * never started streaming just timed out silently and was
		 * resubmitted forever, so the capture hung in "capturing" with no
		 * data and no way out but a manual stop.
		 */
		if (!g_atomic_int_get(&devc->abort) &&
		    devc->transfers_completed > 1 && delta > 0) {
			double expected_us =
				(double)devc->transfer_size * 1000000.0 /
				(double)devc->expected_rate_bytes;
			double actual_rate = (double)len * 1000000.0 /
				(double)delta;
			if ((double)delta > (SLOGIC_TRANSFER_TOLERANCE + 1.0) *
				    expected_us ||
			    actual_rate < (1.0 - SLOGIC_TRANSFER_TOLERANCE) *
				    (double)devc->expected_rate_bytes)
				devc->timeout_count++;
			else
				devc->timeout_count = 0;
			if (len > 0)
				devc->transfers_time_data = now;

			/*
			 * The reference driver treats "transfer was slower than
			 * expected" as a fatal stall on every transfer.  That
			 * is right while no data has arrived at all (the RUN
			 * command was swallowed), but wrong once the stream is
			 * running: 32ch@200MHz makes the host the bottleneck,
			 * the analyzer throttles itself, and later transfers
			 * legitimately look slow.  Aborting there truncated
			 * long captures (4 s of samples ended after 621 MSa,
			 * while sigrok-cli finished the same request).  So
			 * once bytes have started flowing, only complete
			 * silence for SLOGIC_STREAM_IDLE_US is fatal;
			 * slow-but-alive just gets one warning.
			 */
			if (devc->raw_received_bytes > 0) {
				int64_t idle_us = devc->transfers_time_data ?
					now - devc->transfers_time_data :
					now - devc->transfers_time_start;

				if (idle_us > SLOGIC_STREAM_IDLE_US) {
					devc->timeout_count =
						devc->timeout_count_limit + 1;
					devc->silent_abort = 1;
				} else {
					devc->timeout_count = 0;
				}
				if (!devc->slow_warned &&
				    (double)delta >
					    (SLOGIC_TRANSFER_TOLERANCE + 1.0) *
						    expected_us) {
					devc->slow_warned = 1;
					sr_dbg("USB link slower than the "
						"analyzer (%.0f MB/s): draining "
						"at the host's pace",
						actual_rate / 1000000.0);
				}
			}
			if (devc->timeout_count >= devc->timeout_count_limit) {
				if (devc->raw_received_bytes == 0 &&
				    g_atomic_int_get(&devc->start_retries) <
					    SLOGIC_MAX_START_RETRIES &&
				    !g_atomic_int_get(&devc->restart_pending)) {
					/* The firmware sometimes swallows the RUN
					 * command and then never streams. Ask the
					 * collect thread to re-arm instead of
					 * handing the user an empty capture. */
					sr_warn("USB stream did not start after "
						"%" PRIu64 " transfers, re-arming "
						"hardware",
						(uint64_t)devc->timeout_count);
					devc->timeout_count = 0;
					devc->transfers_completed = 0;
					devc->transfers_time_latest = 0;
					g_atomic_int_set(&devc->restart_pending, 1);
				} else {
					if (devc->raw_received_bytes == 0)
						sr_err("USB stream never started: no data in "
							"%" PRIu64 " consecutive transfers, "
							"aborting capture",
							(uint64_t)devc->timeout_count);
					else if (devc->silent_abort)
						sr_err("USB stream went silent mid-capture: "
							"no data for more than %d ms, "
							"aborting after %" PRIu64 " samples",
							(int)(SLOGIC_STREAM_IDLE_US / 1000),
							devc->num_samples);
					else
						sr_err("USB transfer stalled: %.2f MB/s, "
							"%" PRIu64 " consecutive slow transfers",
							actual_rate / 1000000.0,
							(uint64_t)devc->timeout_count);
					g_atomic_int_set(&devc->abort, 1);
					devc->status = SLOGIC_ST_STOP;
					g_atomic_int_set(&devc->discard_queue, 1);
				}
			}
		}
	} else {
		gboolean expected_cancel = g_atomic_int_get(&devc->abort);

		if (!expected_cancel) {
			sr_err("bulk transfer status %d (%s)", transfer->status,
				libusb_error_name(transfer->status));
			g_atomic_int_set(&devc->discard_queue, 1);
		}
		g_atomic_int_set(&devc->abort, 1);
		devc->status = SLOGIC_ST_STOP;
	}

	/* USB callbacks only move completed buffers to the queue. All expensive
	 * conversion and UI delivery happens in receive_data(). */
	if (!g_atomic_int_get(&devc->abort) && !reached_limit) {
		if (!transfer->buffer)
			transfer->buffer = g_try_malloc((gsize)devc->transfer_size);
		if (transfer->buffer) {
			transfer->actual_length = 0;
			transfer->length = (int)devc->transfer_size;
			transfer->timeout = (unsigned int)(
				(SLOGIC_TRANSFER_TOLERANCE + 1.0) *
				devc->transfer_duration_ms * 4.0);
			if (transfer->timeout < 10)
				transfer->timeout = 10;
			ret = libusb_submit_transfer(transfer);
			if (ret == 0)
				resubmit = TRUE;
			else {
				sr_err("resubmit failed: %s", libusb_error_name(ret));
				g_atomic_int_set(&devc->abort, 1);
				devc->status = SLOGIC_ST_STOP;
				g_atomic_int_set(&devc->discard_queue, 1);
			}
		} else {
			sr_err("unable to allocate replacement USB buffer");
			g_atomic_int_set(&devc->abort, 1);
			devc->status = SLOGIC_ST_STOP;
			g_atomic_int_set(&devc->discard_queue, 1);
		}
	}

	if (!resubmit) {
		if (owned && !queued)
			g_free(owned);
		slogic_transfer_done(devc, transfer);
	}
	g_atomic_int_dec_and_test(&devc->callbacks_active);
}

static uint64_t slogic_wire_bytes_per_second(const struct slogic_context *devc)
{
	uint64_t channels = devc->channel_count > 0 ?
		(uint64_t)devc->channel_count : 1;

	return (devc->samplerate * channels + 7) / 8;
}

static uint64_t slogic_pick_transfer_size(struct slogic_context *devc)
{
	uint64_t rate = slogic_wire_bytes_per_second(devc);
	uint64_t size = (rate * 4 + 999) / 1000; /* approximately 4 ms */

	if (size < SLOGIC_TRANSFER_SIZE_MIN)
		size = SLOGIC_TRANSFER_SIZE_MIN;
	if (size > SLOGIC_TRANSFER_SIZE_MAX)
		size = SLOGIC_TRANSFER_SIZE_MAX;
	size = (size + SLOGIC_TRANSFER_ALIGN - 1) &
		~(uint64_t)(SLOGIC_TRANSFER_ALIGN - 1);
	if (size > SLOGIC_TRANSFER_SIZE_MAX)
		size = SLOGIC_TRANSFER_SIZE_MAX;
	return size;
}

static void slogic_update_transfer_timing(struct slogic_context *devc)
{
	devc->transfer_duration_ms =
		(devc->transfer_size * 1000 + devc->expected_rate_bytes - 1) /
		devc->expected_rate_bytes;
	if (devc->transfer_duration_ms == 0)
		devc->transfer_duration_ms = 1;
}

static int start_transfers(struct slogic_context *devc)
{
	uint64_t need = devc->samples_need_bytes;
	uint64_t nneed;
	int i;

	devc->transfer_size = slogic_pick_transfer_size(devc);
	devc->expected_rate_bytes = slogic_wire_bytes_per_second(devc);
	if (devc->expected_rate_bytes == 0)
		devc->expected_rate_bytes = 1;
	slogic_update_transfer_timing(devc);

	devc->raw_queue = g_async_queue_new();
	if (!devc->raw_queue)
		return SR_ERR_MALLOC;
	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) *
					SLOGIC_MAX_TRANSFERS);
	if (!devc->transfers)
		return SR_ERR_MALLOC;
	devc->num_transfers = SLOGIC_MAX_TRANSFERS;
	devc->submitted_transfers = 0;
	devc->transfers_completed = 0;
	devc->transfers_received_bytes = 0;
	devc->transfers_time_start = g_get_monotonic_time();
	devc->transfers_time_latest = devc->transfers_time_start;
	devc->transfers_time_data = 0;
	devc->slow_warned = 0;
	devc->silent_abort = 0;
	devc->timeout_count = 0;

	if (need && !devc->is_loop) {
		nneed = (need + devc->transfer_size - 1) / devc->transfer_size;
		if (nneed < 4)
			nneed = 4;
		if (nneed > SLOGIC_MAX_TRANSFERS)
			nneed = SLOGIC_MAX_TRANSFERS;
	} else {
		nneed = SLOGIC_MAX_TRANSFERS;
	}
	devc->timeout_count_limit = (unsigned int)nneed;

	for (i = 0; i < (int)nneed; i++) {
		struct libusb_transfer *xfer;
		unsigned char *buf;
		int ret;
		unsigned int timeout;

		buf = g_try_malloc((gsize)devc->transfer_size);
		if (!buf) {
			if (i == 0 &&
			    devc->transfer_size > SLOGIC_TRANSFER_SIZE_MIN) {
				devc->transfer_size >>= 1;
				devc->transfer_size &= ~(uint64_t)(SLOGIC_TRANSFER_ALIGN - 1);
				slogic_update_transfer_timing(devc);
				i--;
				continue;
			}
			break;
		}
		xfer = libusb_alloc_transfer(0);
		if (!xfer) {
			g_free(buf);
			break;
		}
		timeout = (unsigned int)((SLOGIC_TRANSFER_TOLERANCE + 1.0) *
			devc->transfer_duration_ms * (i + 2));
		if (timeout < 10)
			timeout = 10;
		libusb_fill_bulk_transfer(xfer, devc->devhdl,
			devc->model ? devc->model->ep_in : SLOGIC_EP_U3_IN,
			buf, (int)devc->transfer_size, receive_transfer, devc,
			timeout);
		g_mutex_lock(&devc->transfer_lock);
		devc->transfers[i] = xfer;
		devc->submitted_transfers++;
		g_mutex_unlock(&devc->transfer_lock);
		ret = libusb_submit_transfer(xfer);
		if (ret != 0) {
			if (ret == LIBUSB_ERROR_NO_MEM && i > 0)
				sr_dbg("USB kernel accepted %u large transfers", i);
			else if (ret == LIBUSB_ERROR_NO_MEM &&
				 devc->transfer_size > SLOGIC_TRANSFER_SIZE_MIN) {
				g_mutex_lock(&devc->transfer_lock);
				devc->transfers[i] = NULL;
				if (devc->submitted_transfers > 0)
					devc->submitted_transfers--;
				g_mutex_unlock(&devc->transfer_lock);
				g_free(xfer->buffer);
				xfer->buffer = NULL;
				libusb_free_transfer(xfer);
				devc->transfer_size >>= 1;
				devc->transfer_size &= ~(uint64_t)(SLOGIC_TRANSFER_ALIGN - 1);
				slogic_update_transfer_timing(devc);
				i--;
				continue;
			} else
				sr_err("submit transfer %d failed: %s", i,
					libusb_error_name(ret));
			g_mutex_lock(&devc->transfer_lock);
			devc->transfers[i] = NULL;
			if (devc->submitted_transfers > 0)
				devc->submitted_transfers--;
			g_mutex_unlock(&devc->transfer_lock);
			g_free(xfer->buffer);
			xfer->buffer = NULL;
			libusb_free_transfer(xfer);
			break;
		}
	}

	if (slogic_submitted_count(devc) == 0)
		return SR_ERR;
	devc->timeout_count_limit =
		(unsigned int)slogic_submitted_count(devc);
	sr_info("USB pipeline: %d transfers x %" PRIu64
		" bytes (%.2f ms each)",
		slogic_submitted_count(devc), devc->transfer_size,
		(double)devc->transfer_duration_ms);
	return SR_OK;
}

static void slogic_abort_startup(struct slogic_context *devc)
{
	int wait_ms = 0;

	if (!devc)
		return;
	g_atomic_int_set(&devc->abort, 1);
	g_atomic_int_set(&devc->discard_queue, 1);
	slogic_cancel_transfers(devc);
	while ((slogic_submitted_count(devc) > 0 ||
		slogic_active_callbacks(devc) > 0) && wait_ms++ < 2000)
		g_usleep(1000);
	slogic_clear_raw_queue(devc);
	if (devc->raw_queue) {
		g_async_queue_unref(devc->raw_queue);
		devc->raw_queue = NULL;
	}
	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	devc->submitted_transfers = 0;
}

static void slogic_finish_session(struct slogic_context *devc)
{
	struct sr_datafeed_packet packet;

	if (devc->end_sent)
		return;
	devc->raw_pending_len = 0;
	devc->fast_pending_len = 0;

	/*
	 * Report what the link actually delivered.  The analyzer streams a
	 * 32ch@200MHz capture at about 800 MB/s, so this line is the quickest
	 * way to tell "the device kept up" (stream window ~= the requested
	 * duration) from "the device stalled and the host drained a partial
	 * buffer".
	 */
	if (devc->transfers_time_start && devc->transfers_time_latest) {
		double stream_s =
			(double)(devc->transfers_time_latest -
				 devc->transfers_time_start) / 1e6;
		double total_s =
			(double)(g_get_monotonic_time() -
				 devc->transfers_time_start) / 1e6;
		double mb = (double)devc->transfers_received_bytes / 1e6;

		sr_info("stream done: %.1f MB in %.3f s (%.0f MB/s), "
			"%.3f MSa, host tail %.0f ms",
			mb, stream_s, stream_s > 0.0 ? mb / stream_s : 0.0,
			(double)devc->num_samples / 1e6,
			(total_s - stream_s) * 1000.0);
	}

	sr_info("send SR_DF_END, samples=%" PRIu64, devc->num_samples);
	memset(&packet, 0, sizeof(packet));
	packet.type = SR_DF_END;
	packet.status = SR_PKT_OK;
	ds_data_forward(devc->sdi, &packet);
	devc->end_sent = 1;

	if (devc->raw_queue) {
		slogic_clear_raw_queue(devc);
		g_async_queue_unref(devc->raw_queue);
		devc->raw_queue = NULL;
	}
	g_free(devc->transfers);
	devc->transfers = NULL;
	devc->num_transfers = 0;
	devc->submitted_transfers = 0;
	devc->status = SLOGIC_ST_FINISH;
	if (devc->source_added) {
		sr_session_source_remove(-1);
		devc->source_added = 0;
	}
}

static int receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
	struct slogic_context *devc = sdi->priv;
	GByteArray *array;
	int n;

	(void)fd;
	(void)revents;

	if (g_atomic_int_get(&devc->abort)) {
		devc->status = SLOGIC_ST_STOP;
		slogic_cancel_transfers(devc);
		/* Firmware STOP is deliberately deferred until every bulk URB has
		 * returned. Some U3 firmware reports BUSY when a control transfer
		 * is issued from inside (or alongside) an active bulk callback. */
		if (slogic_submitted_count(devc) == 0 &&
		    slogic_active_callbacks(devc) == 0 &&
		    g_atomic_int_get(&devc->fw_streaming))
			slogic_hw_stop(devc);
		if (g_atomic_int_get(&devc->discard_queue))
			slogic_clear_raw_queue(devc);
	}

	/*
	 * A control transfer must not be issued from inside a bulk callback
	 * (some U3 firmware answers BUSY then), so the re-arm requested by
	 * receive_transfer() is executed here, on the collect thread.
	 */
	if (!g_atomic_int_get(&devc->abort) &&
	    g_atomic_int_get(&devc->restart_pending)) {
		g_atomic_int_set(&devc->restart_pending, 0);
		g_atomic_int_inc(&devc->start_retries);
		if (slogic_hw_start(devc) != SR_OK) {
			sr_err("hardware re-arm failed, ending capture");
			g_atomic_int_set(&devc->abort, 1);
			devc->status = SLOGIC_ST_STOP;
			g_atomic_int_set(&devc->discard_queue, 1);
		} else {
			sr_warn("hardware re-armed (start retry %d)",
				g_atomic_int_get(&devc->start_retries));
		}
	}

	/* Convert a bounded number of packets per iteration. This keeps the UI
	 * event loop responsive while the USB thread continues filling the queue. */
	n = g_atomic_int_get(&devc->discard_queue) ? 0 : 2;
	while (n-- > 0 && devc->raw_queue &&
		   (array = g_async_queue_try_pop(devc->raw_queue)) != NULL) {
		if (slogic_convert_push(devc, array->data, (int)array->len) != SR_OK) {
			sr_err("format convert failed");
			g_atomic_int_set(&devc->discard_queue, 1);
			g_atomic_int_set(&devc->abort, 1);
		}
		g_byte_array_unref(array);
	}

	if (g_atomic_int_get(&devc->abort) &&
			slogic_submitted_count(devc) == 0 &&
			slogic_active_callbacks(devc) == 0 &&
			(!devc->raw_queue ||
			 g_async_queue_length(devc->raw_queue) == 0)) {
		if (g_atomic_int_get(&devc->fw_streaming))
			return TRUE; /* retry hardware STOP on the next tick */
		slogic_finish_session(devc);
		return FALSE;
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

	if (count > SLOGIC_MAX_PHYS_CH)
		count = SLOGIC_MAX_PHYS_CH;

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
		const struct slogic_model *model;

		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		if (des.idVendor != SLOGIC_VID)
			continue;
		model = slogic_model_for_pid(des.idProduct);
		if (!model)
			continue;
		if (sr_usb_device_is_exists(devlist[i]))
			continue;

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc)
			break;
		g_mutex_init(&devc->transfer_lock);

		bus = libusb_get_bus_number(devlist[i]);
		address = libusb_get_device_address(devlist[i]);

		devc->usb_dev = libusb_ref_device(devlist[i]);
		devc->sr_ctx = drvc->sr_ctx;
		devc->model = model;
		devc->ch_mode = 0;
		devc->channel_count = (model->physical_channels >> (0));
		devc->filter = SR_FILTER_NONE;
		devc->limit_samples = SLOGIC_DEFAULT_SAMPLES;
		devc->max_height = 1;
		devc->vth = SLOGIC_VTH_DEFAULT;
		devc->op_mode = LO_OP_STREAM;
		devc->is_loop = 0;
		devc->pattern_mode = 0;
		devc->usb_speed = LIBUSB_SPEED_UNKNOWN;

		sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE,
				      "Sipeed", model->name, NULL);
		if (!sdi) {
			libusb_unref_device(devc->usb_dev);
			g_mutex_clear(&devc->transfer_lock);
			g_free(devc);
			break;
		}
		sdi->driver = di;
		sdi->dev_type = DEV_TYPE_USB;
		sdi->priv = devc;
		devc->sdi = sdi;
		sdi->handle = (ds_device_handle)devc->usb_dev;
		slogic_update_usb_speed(devc);
		slogic_map_samplerate(devc, slogic_max_rate(model, model->physical_channels >> (0)));

		usb_info = sr_usb_dev_inst_new(bus, address);
		if (usb_info) {
			usb_info->usb_dev = devc->usb_dev;
			sdi->conn = usb_info;
		} else {
			sdi->conn = NULL;
		}
		setup_channels(sdi, model->physical_channels);
		slogic_apply_channel_mode(sdi);

		sr_info("Found %s 359f:%04x bus=%u addr=%u speed=%s name=%s",
			model->name,
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

static gpointer slogic_libusb_event_thread(gpointer user_data)
{
	struct slogic_context *devc = user_data;
	struct timeval tv = { 1, 0 };

	while (g_atomic_int_get(&devc->libusb_event_thread_run)) {
		if (!devc->sr_ctx || !devc->sr_ctx->libusb_ctx)
			break;
		libusb_handle_events_timeout_completed(
			devc->sr_ctx->libusb_ctx, &tv, NULL);
	}
	return NULL;
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

	g_atomic_int_set(&devc->libusb_event_thread_run, 1);
	devc->libusb_event_thread = g_thread_new("slogic-libusb-events",
		slogic_libusb_event_thread, devc);
	if (!devc->libusb_event_thread) {
		g_atomic_int_set(&devc->libusb_event_thread_run, 0);
		libusb_release_interface(devc->devhdl, 0);
		libusb_close(devc->devhdl);
		devc->devhdl = NULL;
		return SR_ERR_MALLOC;
	}

	/*
	 * The wire format has to be transposed into LA_CROSS_DATA on the host,
	 * which costs more than the USB link needs to deliver the samples.  Use
	 * a few helpers so a 32ch@200MHz stream stays ahead of real time.
	 *
	 * Half of the cores are left to the rest of the process (UI repaint,
	 * decode, other applications); the pool only has to convert faster than
	 * the ~790 MB/s the analyzer delivers, and measurements showed the
	 * remaining serial snapshot append - not the transpose - dominates the
	 * tail once this many helpers are running.
	 */
	if (!devc->conv_pool) {
		int cores = g_get_num_processors();
		int threads = cores / 2;

		if (threads < 1)
			threads = 1;
		if (threads > SLOGIC_MAX_CONV_THREADS)
			threads = SLOGIC_MAX_CONV_THREADS;
		devc->conv_pool = g_thread_pool_new(slogic_conv_chunk_worker,
						    NULL, threads, TRUE, NULL);
		devc->conv_threads = devc->conv_pool ? threads : 1;
	}

	devc->sdi = sdi;
	slogic_update_usb_speed(devc);
	slogic_map_samplerate(devc, devc->samplerate ? devc->samplerate
						     : SR_MHZ(100));
	if (slogic_hw_reset(devc) != SR_OK)
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
	int wait_ms = 0;

	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	if (devc->devhdl) {
		if (devc->source_added || slogic_submitted_count(devc) > 0) {
			g_atomic_int_set(&devc->discard_queue, 1);
			slogic_request_acq_stop(devc, TRUE);
			while ((slogic_submitted_count(devc) > 0 ||
				slogic_active_callbacks(devc) > 0) && wait_ms < 2000) {
				g_usleep(1000);
				wait_ms++;
			}
		}
		slogic_hw_stop(devc);
		g_atomic_int_set(&devc->libusb_event_thread_run, 0);
		libusb_interrupt_event_handler(devc->sr_ctx->libusb_ctx);
		if (devc->libusb_event_thread) {
			g_thread_join(devc->libusb_event_thread);
			devc->libusb_event_thread = NULL;
		}
		if (devc->source_added) {
			sr_session_source_remove(-1);
			devc->source_added = 0;
		}
		slogic_clear_raw_queue(devc);
		if (devc->raw_queue) {
			g_async_queue_unref(devc->raw_queue);
			devc->raw_queue = NULL;
		}
		g_free(devc->transfers);
		devc->transfers = NULL;
		devc->num_transfers = 0;
		devc->submitted_transfers = 0;
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
		if (devc->conv_pool) {
			g_thread_pool_free(devc->conv_pool, FALSE, TRUE);
			devc->conv_pool = NULL;
			devc->conv_threads = 1;
		}
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		g_free(devc->raw_pending);
		g_free(devc->cross_buf);
		g_free(devc->fast_pending);
		if (devc->raw_queue)
			g_async_queue_unref(devc->raw_queue);
		g_mutex_clear(&devc->transfer_lock);
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
	struct libusb_device_descriptor des;
	const struct slogic_model *model;
	const struct slogic_model *old_model;

	if (!sdi || !sdi->priv || !new_dev)
		return;
	if (!sdi->driver || !sdi->driver->name ||
	    strcmp(sdi->driver->name, "slogic-16u3") != 0)
		return;

	devc = sdi->priv;
	old_model = devc->model;
	if (libusb_get_device_descriptor(new_dev, &des) != 0 ||
		 des.idVendor != SLOGIC_VID ||
		 !(model = slogic_model_for_pid(des.idProduct)))
		return;
	if (devc->usb_dev)
		libusb_unref_device(devc->usb_dev);
	devc->usb_dev = libusb_ref_device(new_dev);
	devc->model = model;
	if ((size_t)devc->ch_mode >= model->limit_count)
		devc->ch_mode = 0;
	devc->channel_count = (model->physical_channels >> (devc->ch_mode));
	devc->sdi = sdi;
	sdi->handle = (ds_device_handle)devc->usb_dev;
	if (old_model != model) {
		setup_channels(sdi, model->physical_channels);
		slogic_apply_channel_mode(sdi);
	}
	slogic_update_usb_speed(devc);
	slogic_map_samplerate(devc, devc->samplerate ? devc->samplerate
							     : slogic_max_rate(model, model->physical_channels >> (devc->ch_mode)));
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

static int slogic_mode_for_channels(const struct slogic_model *model, int channels)
{
	size_t i;

	if (!model)
		return -1;
	for (i = 0; i < model->limit_count; i++) {
		if ((model->physical_channels >> (i)) == channels)
			return (int)i;
	}
	return -1;
}

static int slogic_set_pattern_mode(struct slogic_context *devc, int mode)
{
	if (!devc || mode < 0 || mode >= (int)ARRAY_SIZE(pattern_modes))
		return SR_ERR_ARG;
	if (devc->model && devc->model->proto == SLOGIC_PROTO_COMBO8 && mode != 0)
		return SR_ERR_NA;
	/* Stored only; the shared core applies the pattern at the next
	 * slogic_reset + slogic_configure (done at acquisition start). */
	devc->pattern_mode = mode;
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
		*data = g_variant_new_uint64(devc->model ? SLOGIC_HW_DEPTH :
			SLOGIC_HW_DEPTH);
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16(devc->model ?
			devc->model->physical_channels : 16);
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
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_string(pattern_modes[devc->pattern_mode]);
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
		*data = g_variant_new_boolean(devc->model &&
			(devc->model->proto == SLOGIC_PROTO_U3));
		break;
	case SR_CONF_LA_CH32:
		*data = g_variant_new_boolean(devc->model &&
			devc->model->physical_channels >= 32);
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
	case SR_CONF_PATTERN_MODE: {
		const char *p = g_variant_get_string(data, NULL);
		int mode;
		for (mode = 0; mode < (int)ARRAY_SIZE(pattern_modes); mode++)
			if (!strcmp(p, pattern_modes[mode]))
				break;
		if (mode >= (int)ARRAY_SIZE(pattern_modes))
			return SR_ERR_ARG;
		return slogic_set_pattern_mode(devc, mode);
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
		if (!devc->model || nv < 0 ||
		    (size_t)nv >= devc->model->limit_count)
			return SR_ERR;
		devc->ch_mode = nv;
		slogic_apply_channel_mode(sdi);
		slogic_map_samplerate(devc, devc->samplerate);
		sr_info("channel mode -> %d ch, rate=%" PRIu64,
			devc->channel_count, devc->samplerate);
		break;
	}
	case SR_CONF_VLD_CH_NUM: {
		int nv = g_variant_get_int16(data);
		int mode = slogic_mode_for_channels(devc->model, nv);
		if (mode < 0)
			return SR_ERR;
		devc->ch_mode = mode;
		slogic_apply_channel_mode(sdi);
		slogic_map_samplerate(devc, devc->samplerate);
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
		/* Must hold the whole rate list plus its 0 terminator. */
		static uint64_t rate_buf[33];
		int rate_n = 0;
		const struct slogic_context *devc =
			(sdi && sdi->priv) ? (const struct slogic_context *)sdi->priv
					   : NULL;

		if (devc)
			slogic_build_rate_list(devc, rate_buf, &rate_n);
		else {
			unsigned int i;
			const slogic_model *m16 = slogic_model_for_pid(SLOGIC_PID_16U3); for (i = 0; m16 && i < m16->rate_count; i++)
				if (m16->rates[i] <= SR_MHZ(200))
					rate_buf[rate_n++] = m16->rates[i];
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
		*data = g_variant_new_uint64((uint64_t)slogic_channel_mode_list(
			(sdi && sdi->priv) ?
			((const struct slogic_context *)sdi->priv)->model : NULL));
		break;
	case SR_CONF_OPERATION_MODE:
		*data = g_variant_new_uint64((uint64_t)&opmode_list);
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_strv(pattern_modes, ARRAY_SIZE(pattern_modes));
		break;
	default:
		return SR_ERR_ARG;
	}
	return SR_OK;
}

static int hw_dev_acquisition_start(struct sr_dev_inst *sdi, void *cb_data)
{
	struct slogic_context *devc = sdi->priv;
	int ret;

	(void)cb_data;

	if (sdi->status != SR_ST_ACTIVE) {
		ds_set_last_error(SR_ERR_DEVICE_CLOSED);
		return SR_ERR_DEVICE_CLOSED;
	}

	devc->sdi = sdi;
	devc->num_samples = 0;
	devc->num_bytes = 0;
	g_atomic_int_set(&devc->abort, 0);
	g_atomic_int_set(&devc->fw_streaming, 0);
	devc->status = SLOGIC_ST_START;
	devc->freewheel = 0;
	devc->raw_pending_len = 0;
	devc->fast_pending_len = 0;
	devc->filt_have_prev = 0;
	devc->stream_res_len = 0;
	devc->drop_left = SLOGIC_DROP_FIRST_BYTES;
	devc->raw_received_bytes = 0;
	devc->samples_need_bytes = 0;
	devc->transfers = NULL;
	devc->num_transfers = 0;
	devc->submitted_transfers = 0;
	g_atomic_int_set(&devc->callbacks_active, 0);
	devc->raw_queue = NULL;
	devc->source_added = 0;
	devc->end_sent = 0;
	devc->user_stop = 0;
	g_atomic_int_set(&devc->discard_queue, 0);
	devc->hw_stop_attempts = 0;
	g_atomic_int_set(&devc->restart_pending, 0);
	g_atomic_int_set(&devc->start_retries, 0);

	slogic_refresh_enabled_bits(devc);
	slogic_map_samplerate(devc, devc->samplerate);
	if (devc->limit_samples && !devc->is_loop)
		devc->samples_need_bytes =
			(devc->limit_samples * (uint64_t)devc->channel_count + 7) / 8;

	sr_info("acq start: nch=%d en=%d rate=%" PRIu64 " loop=%d limit=%" PRIu64
		" speed=%s",
		devc->channel_count, devc->en_count, devc->samplerate,
		devc->is_loop, devc->limit_samples,
		slogic_speed_name(devc->usb_speed));

	/* Stop leftover capture and drain the model-specific endpoint before
	 * arming new URBs.  Note: do not reset the capture engine here - a
	 * register reset would discard the pattern generator and reference
	 * voltage configuration the user selected. */
	slogic_drain_ep(devc);

	/* reset -> configure -> run, as in the libsigrok adapter. The reset
	 * re-establishes the sample clock for the selected channel-mode/rate
	 * (needed e.g. for 4 ch @ 1400 MHz); slogic_configure then re-applies
	 * channel mask, samplerate, threshold and pattern in canonical order, so
	 * nothing the reset cleared is lost. */
	{
		slogic_transport t;
		slogic_config c;
		al_transport(devc, &t);
		al_config(devc, &c);
		if (slogic_reset(devc->model, &t) != SLOGIC_OK ||
		    slogic_configure(devc->model, &t, &c) != SLOGIC_OK) {
			sr_err("slogic reset/configure failed");
			return SR_ERR;
		}
	}

	if ((ret = start_transfers(devc)) != SR_OK) {
		sr_err("start_transfers failed");
		slogic_abort_startup(devc);
		return ret;
	}

	/* libusb is serviced by the dedicated event thread. The session source
	 * only drains the raw queue and owns the stop/end state machine. */
	ret = sr_session_source_add(-1, 0, 1, receive_data, sdi);
	if (ret != SR_OK) {
		slogic_abort_startup(devc);
		return ret;
	}
	devc->source_added = 1;

	std_session_send_df_header(sdi, LOG_PREFIX);
	/* Mark the firmware as owned before issuing RUN so a concurrent stop
	 * request cannot skip the deferred STOP path. */
	g_atomic_int_set(&devc->fw_streaming, 1);
	if ((ret = slogic_hw_start(devc)) != SR_OK) {
		slogic_abort_startup(devc);
		slogic_hw_stop(devc);
		sr_session_source_remove(-1);
		devc->source_added = 0;
		g_atomic_int_set(&devc->fw_streaming, 0);
		return ret;
	}
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct slogic_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	devc->user_stop = 1;
	slogic_request_acq_stop(devc, TRUE);
	return SR_OK;
}

SR_PRIV struct sr_dev_driver slogic16u3_driver_info = {
	.name = "slogic-16u3",
	.longname = "Sipeed SLogic Combo 8 / 16U3 / 32U3 Logic Analyzer",
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
