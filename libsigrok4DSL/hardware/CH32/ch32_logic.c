/*
 * CH32H417 Logic Analyzer driver for DSView / libsigrok4DSL
 *
 * USB protocol (WCH CH32H417 firmware):
 *   VID 1A86
 *   PID 0x5537  USB2.0 High-Speed (USBHS)
 *   PID 0x5538  USB3.0 SuperSpeed (USBSS)  = USB2 PID + 1
 *   EP1 OUT/IN  command channel
 *   EP2 IN      logic sample stream
 *   EP3 IN      ADC sample stream (not used yet)
 *
 * Commands (bulk EP1):
 *   {Cmd, Len, Buf[5]}  request
 *   {Cmd|0x10, Len, Status, Buf[5]}  response
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "../../libsigrok-internal.h"
#include "../../log.h"
#include "ch32_logic.h"
#include "ch32_vth_lut.h"

#undef LOG_PREFIX
#define LOG_PREFIX "ch32: "

#define CH32_VID                 0x1A86
#define CH32_PID_USB2            0x5537   /* High-Speed APP */
#define CH32_PID_USB3            0x5538   /* SuperSpeed APP (= USB2 + 1) */
/* Compat: older single-PID code paths */
#define CH32_PID                 CH32_PID_USB2

#define CH32_EP_CMD_OUT          0x01
#define CH32_EP_CMD_IN           0x81
#define CH32_EP_LOGIC_IN         0x82
#define CH32_EP_ADC_IN           0x83

#define CMD_SET_LOGIC_PARA       0xA0
#define CMD_SET_LOGIC_LEVEL      0xA1
#define CMD_SET_START            0xA2
#define CMD_SET_STOP             0xA3
#define CMD_GET_LOGIC_PARA       0xA4
#define CMD_GET_LOGIC_LEVEL      0xA5
#define CMD_GET_VERSION          0xAC
#define CMD_RETURN_PARA          0x10

/* HSADC (HADC) commands — EP3 stream */
#define CMD_SET_ADC_PARA         0xA6
#define CMD_SET_ADC_CHANNEL      0xA7
#define CMD_SET_ADC_START        0xA8
#define CMD_SET_ADC_STOP         0xA9
#define CMD_GET_ADC_PARA         0xAA
#define CMD_GET_ADC_CHANNEL      0xAB

/* CH32H417 HSADC product: 10-bit only, max 20 MS/s */
#define ADC_BIT_10               0x0a
#define ADC_CH0                  0x00
#define ADC_CH1                  0x01

/*
 * HSADC sample rate (calibrated with 100 Hz square on CH0):
 *   rate_Hz = 40e6 / (adc_div + 1)
 * Firmware comment said 80e6/(div+1) which is 2x high.
 * div=0 fails; div=1 → 20 MS/s (hardware max); 10-bit in LE u16.
 *
 * DSView ANALOG path (AnalogSnapshot + envelope) is 8-bit only and expects
 * interleaved samples for *all* SR_CHANNEL_ANALOG probes. Host converts
 * mono 10-bit EP3 stream → 8-bit interleaved before SR_DF_ANALOG.
 */
#define CH32_ADC_CLOCK_HZ        SR_MHZ(40)   /* calibrated base */
#define CH32_ADC_MAX_HZ          SR_MHZ(20)   /* div=1 */
#define CH32_ADC_MIN_DIV         1            /* 40/(1+1)=20M; div=0 invalid */
#define CH32_ADC_BITS            10           /* hardware */
#define CH32_ADC_BPS             2            /* raw EP3: LE u16 */
#define CH32_ADC_UI_BITS         8            /* DSView unit_bits / envelope */
#define CH32_ADC_UI_MID          128
#define CH32_ADC_VREF            3.3          /* unipolar map 0..Vref */

#define CH32_XFER_TIMEOUT_MS     1000
#define CH32_BULK_TIMEOUT_MS     200
#define CH32_NUM_TRANSFERS       8
#define CH32_TRANSFER_SIZE       (64 * 1024)

#define CH32_DEFAULT_SAMPLES     SR_Mn(1)
#define CH32_HW_DEPTH            SR_Mn(16)

/* DSView LA_CROSS_DATA: 64 samples per channel plane block (8 bytes). */
#define CH32_CROSS_SCALE         64
#define CH32_CROSS_PLANE_BYTES   8
#define CH32_MAX_PHYS_CH         16

enum {
	CH32_ST_IDLE = 0,
	CH32_ST_START,
	CH32_ST_DATA,
	CH32_ST_STOP,
	CH32_ST_FINISH,
	CH32_ST_ERROR,
};

struct ch32_samplerate_entry {
	uint8_t logic_sys;
	uint64_t rate;
};

/* Hardware rates from firmware Set_Logic_Para_Fun() (USB3).
 * Lowest HW: logic_sys 0x17 → PLL 400M / DIV64 = 6.25 MS/s. */
static const struct ch32_samplerate_entry ch32_hw_rates[] = {
	{ 0x00, SR_MHZ(200) },
	{ 0x01, SR_MHZ(187) + SR_KHZ(500) },
	{ 0x02, SR_MHZ(175) },
	{ 0x03, SR_MHZ(162) + SR_KHZ(500) },
	{ 0x04, SR_MHZ(156) + SR_KHZ(250) },
	{ 0x05, SR_MHZ(150) },
	{ 0x06, SR_MHZ(137) + SR_KHZ(500) },
	{ 0x07, SR_MHZ(125) },
	{ 0x08, SR_MHZ(112) + SR_KHZ(500) },
	{ 0x09, SR_MHZ(100) },
	{ 0x0a, SR_MHZ(93) + SR_KHZ(750) },
	{ 0x0b, SR_MHZ(87) + SR_KHZ(500) },
	{ 0x0c, SR_MHZ(81) + SR_KHZ(250) },
	{ 0x0d, SR_MHZ(78) + SR_KHZ(125) },
	{ 0x0e, SR_MHZ(75) },
	{ 0x0f, SR_MHZ(68) + SR_KHZ(750) },
	{ 0x10, SR_MHZ(50) },
	{ 0x11, SR_MHZ(40) },
	{ 0x12, SR_MHZ(25) },
	{ 0x13, SR_MHZ(20) },
	{ 0x14, SR_MHZ(12) + SR_KHZ(500) },
	{ 0x15, SR_MHZ(10) },
	{ 0x16, SR_MHZ(8) },
	{ 0x17, SR_MHZ(6) + SR_KHZ(250) }, /* DIV64 min */
};

/*
 * UI sample-rate list only: common integer MHz (high → low).
 * Odd HW PLL rates (187.5 / 156.25 / …) stay in ch32_hw_rates for
 * mapping, but are not shown in the combobox.
 */
static const uint64_t ch32_ui_rates[] = {
	SR_MHZ(200),
	SR_MHZ(100),
	SR_MHZ(50),
	SR_MHZ(40),
	SR_MHZ(25),
	SR_MHZ(20),
	SR_MHZ(10),
	SR_MHZ(5),
	SR_MHZ(2),
	SR_MHZ(1),
};

/* Host minimum selectable sample rate. */
#define CH32_UI_RATE_MIN   SR_MHZ(1)

/* HADC UI rates: hardware max 20 MS/s, integer MHz only. */
static const uint64_t ch32_adc_ui_rates[] = {
	SR_MHZ(20),
	SR_MHZ(10),
	SR_MHZ(5),
	SR_MHZ(2),
	SR_MHZ(1),
};

/* Fallback static list (demo / no-device). */
static uint64_t ch32_samplerate_list[ARRAY_SIZE(ch32_ui_rates) + 1];
static int ch32_samplerate_count;

/* Analog probe options (DAQ). */
static const int32_t ch32_probe_options[] = {
	SR_CONF_PROBE_COUPLING,
	SR_CONF_PROBE_VDIV,
	SR_CONF_PROBE_MAP_DEFAULT,
	SR_CONF_PROBE_MAP_UNIT,
	SR_CONF_PROBE_MAP_MIN,
	SR_CONF_PROBE_MAP_MAX,
};

static const uint64_t ch32_vdivs[] = {
	SR_mV(10), SR_mV(20), SR_mV(50), SR_mV(100), SR_mV(200),
	SR_mV(500), SR_V(1), SR_V(2), 0,
};

static const char *ch32_coupling[] = { "DC", "AC", NULL };
static const char *ch32_map_units[] = { "V", NULL };

enum {
	CH32_CHMODE_16 = 0,
	CH32_CHMODE_8 = 1,
};

static const struct sr_list_item filter_list[] = {
	{ SR_FILTER_NONE, "None" },
	{ SR_FILTER_1T, "1 Sample Clock" },
	{ -1, NULL },
};

/* CH32 is continuous USB stream only (no on-device DRAM buffer mode). */
static const struct sr_list_item opmode_list[] = {
	{ LO_OP_STREAM, "Stream Mode" },
	{ -1, NULL },
};

static struct sr_list_item channel_mode_list[] = {
	{ CH32_CHMODE_16, "Use 16 Channels" },
	{ CH32_CHMODE_8, "Use 8 Channels" },
	{ -1, NULL },
};

static const char *maxHeights[] = { "1X", "2X", "3X", "4X", "5X" };

/* Device options panel (like DSLogic). */
static const int32_t hwoptions[] = {
	SR_CONF_OPERATION_MODE,
	SR_CONF_FILTER,
	SR_CONF_VTH,
	SR_CONF_MAX_HEIGHT,
};

/* Session / device-option keys that UI persists. */
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

struct ch32_context {
	struct libusb_device *usb_dev;
	struct libusb_device_handle *devhdl;
	struct sr_context *sr_ctx;

	uint8_t logic_bit;     /* 8 or 16: UHSIF parallel width */
	uint8_t logic_sys;     /* firmware rate index */
	uint16_t logic_level;  /* DAC value */
	uint8_t sw_ver;
	uint8_t hw_ver;

	uint64_t samplerate;      /* effective rate shown to UI */
	uint64_t hw_samplerate;   /* actual hardware capture rate */
	int decimate_n;           /* keep 1 of N raw samples (soft downsample) */
	int decimate_phase;

	uint64_t limit_samples;
	uint64_t num_samples;
	uint64_t num_bytes;

	int channel_count;     /* 8 or 16 physical channels */
	int ch_mode;           /* CH32_CHMODE_8 / _16 */
	int filter;            /* SR_FILTER_NONE / SR_FILTER_1T */
	int max_height;
	double vth;
	int op_mode;           /* LO_OP_STREAM only */
	int is_loop;           /* 1 = rolling/loop: keep streaming past limit */
	enum libusb_speed usb_speed; /* actual link speed */
	uint16_t usb_pid;            /* 0x5537 USB2 / 0x5538 USB3 */

	/* HSADC / ANALOG: always 10-bit, max 20 MS/s */
	uint8_t adc_bit;       /* always ADC_BIT_10 */
	uint8_t adc_div;       /* firmware HSADC_ClockDivision, >= 3 */
	uint8_t adc_channel;   /* 0 or 1 (active HW channel) */
	int acq_analog;        /* 1 while running ADC EP3 stream */

	/* mono 10-bit → 8-bit interleaved expand buffer for DSView */
	uint8_t *analog_expand;
	int analog_expand_cap;

	/* software 1T glitch filter state */
	uint16_t filt_prev_in;
	uint16_t filt_prev_out;
	int filt_have_prev;

	int status;
	int abort;
	int fw_streaming; /* 1 while device capture LED/path is armed */
	int submitted_transfers;
	int num_transfers;
	struct libusb_transfer **transfers;
	int freewheel; /* 1 if using fd=-1 source */

	/* raw sample residual (incomplete 64-sample group) after convert path */
	uint8_t *raw_pending;
	int raw_pending_len;
	int raw_pending_cap;

	/* incomplete multi-byte sample across USB packet boundary (16-bit) */
	uint8_t stream_res[2];
	int stream_res_len;

	/* soft-decimation window (majority vote for digital downsample) */
	uint16_t *decim_win;
	int decim_win_cap;
	int decim_win_fill;

	/* converted LA_CROSS_DATA buffer */
	uint8_t *cross_buf;
	int cross_cap;

	/* enabled channel physical bit indices (probe->index) */
	uint8_t en_bits[CH32_MAX_PHYS_CH];
	int en_count;

	const struct sr_dev_inst *sdi;
};

SR_PRIV struct sr_dev_driver ch32_driver_info;
static struct sr_dev_driver *di = &ch32_driver_info;

/* -------------------- helpers -------------------- */

static uint64_t ch32_hw_min_rate(void)
{
	return ch32_hw_rates[ARRAY_SIZE(ch32_hw_rates) - 1].rate;
}

/* Chip supports USB3; warn if plugged into USB2-only port. */
static int ch32_usb30_capable(void)
{
	return 1;
}

static int ch32_is_usb3_link(const struct ch32_context *devc)
{
	return devc && (devc->usb_speed == LIBUSB_SPEED_SUPER ||
			devc->usb_speed == LIBUSB_SPEED_SUPER_PLUS);
}

/*
 * Firmware Set_Logic_Para_Fun() on USB2 forces fixed UHSIF clock:
 *   8-bit  → DIV10 @ 400M ≈ 40 MS/s
 *   16-bit → DIV20 @ 400M ≈ 20 MS/s
 * USB3 uses full programmable rate table (up to 200 MS/s).
 */
static uint64_t ch32_link_max_hw_rate(const struct ch32_context *devc)
{
	if (ch32_is_usb3_link(devc))
		return ch32_hw_rates[0].rate; /* 200M */
	if (devc && devc->logic_bit == 8)
		return SR_MHZ(40);
	return SR_MHZ(20);
}

static const char *ch32_speed_name(enum libusb_speed sp)
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

static int ch32_is_app_pid(uint16_t pid)
{
	return (pid == CH32_PID_USB2 || pid == CH32_PID_USB3);
}

static void ch32_apply_model_name(struct sr_dev_inst *sdi,
				  enum libusb_speed sp)
{
	const char *model;

	if (!sdi)
		return;
	if (sp == LIBUSB_SPEED_SUPER || sp == LIBUSB_SPEED_SUPER_PLUS)
		model = "CH32H417 LA USB3.0";
	else if (sp == LIBUSB_SPEED_HIGH)
		model = "CH32H417 LA USB2.0";
	else
		model = "CH32H417 Logic Analyzer";

	g_free(sdi->name);
	sdi->name = g_strdup(model);
}

/*
 * Primary: firmware PID
 *   0x5537 → USB2.0 High-Speed
 *   0x5538 → USB3.0 SuperSpeed
 * Fallback: libusb_get_device_speed / bcdUSB if PID unknown.
 */
static void ch32_update_usb_speed(struct ch32_context *devc)
{
	enum libusb_speed sp = LIBUSB_SPEED_UNKNOWN;
	enum libusb_speed sp_lib = LIBUSB_SPEED_UNKNOWN;
	struct libusb_device_descriptor des;
	uint16_t bcd = 0;
	uint16_t pid = 0;

	if (!devc || !devc->usb_dev)
		return;

	sp_lib = libusb_get_device_speed(devc->usb_dev);

	if (libusb_get_device_descriptor(devc->usb_dev, &des) == 0) {
		bcd = des.bcdUSB;
		pid = des.idProduct;
		devc->usb_pid = pid;
	} else {
		pid = devc->usb_pid;
	}

	if (pid == CH32_PID_USB3) {
		/* USB3 stack enumeration: treat as SuperSpeed for rate/UI */
		sp = LIBUSB_SPEED_SUPER;
		if (sp_lib == LIBUSB_SPEED_SUPER_PLUS)
			sp = LIBUSB_SPEED_SUPER_PLUS;
	} else if (pid == CH32_PID_USB2) {
		sp = LIBUSB_SPEED_HIGH;
		if (sp_lib == LIBUSB_SPEED_FULL)
			sp = LIBUSB_SPEED_FULL;
	} else if (sp_lib == LIBUSB_SPEED_SUPER ||
		   sp_lib == LIBUSB_SPEED_SUPER_PLUS) {
		sp = sp_lib;
	} else if ((bcd >> 8) >= 3) {
		sp = LIBUSB_SPEED_SUPER;
	} else if ((bcd >> 8) == 2) {
		sp = (sp_lib == LIBUSB_SPEED_FULL) ? LIBUSB_SPEED_FULL
						   : LIBUSB_SPEED_HIGH;
	} else {
		sp = sp_lib;
	}

	devc->usb_speed = sp;
	if (devc->sdi)
		ch32_apply_model_name((struct sr_dev_inst *)devc->sdi, sp);

	sr_info("CH32 USB: pid=0x%04x speed=%s (%d) libusb=%d bcdUSB=0x%04x max_hw=%" PRIu64 " S/s",
		pid, ch32_speed_name(sp), (int)sp, (int)sp_lib, bcd,
		ch32_link_max_hw_rate(devc));
}

/*
 * Synthesize UI sample rate from firmware rates via integer soft-decimation:
 *   effective = hw_rate / N   (N >= 1, host keeps 1 of every N raw samples)
 *
 * Never drop intermediate frequencies: pick (hw_rate, N) minimizing
 * |hw/N - want|, constrained by link max and UI min (1 MS/s).
 * Ties: smaller N (less phase jitter), then lower hw (less USB bandwidth).
 */
static void ch32_map_samplerate(struct ch32_context *devc, uint64_t want)
{
	unsigned int i;
	uint8_t best_sys = 0x17;
	uint64_t best_hw = ch32_hw_min_rate();
	uint64_t best_eff = CH32_UI_RATE_MIN;
	uint64_t best_err = UINT64_MAX;
	int best_n = 1;
	uint64_t link_max = ch32_link_max_hw_rate(devc);
	int usb3 = ch32_is_usb3_link(devc);

	if (want == 0 || want < CH32_UI_RATE_MIN)
		want = CH32_UI_RATE_MIN;
	if (want > link_max)
		want = link_max;

	if (!usb3) {
		/* USB2: single fixed HW clock; only soft-decimate */
		uint64_t hw = link_max;
		int max_n = (int)(hw / CH32_UI_RATE_MIN);
		int n;

		if (max_n < 1)
			max_n = 1;
		for (n = 1; n <= max_n; n++) {
			uint64_t eff = hw / (uint64_t)n;
			uint64_t err = (eff > want) ? (eff - want) : (want - eff);
			if (err < best_err ||
			    (err == best_err && n < best_n)) {
				best_err = err;
				best_n = n;
				best_hw = hw;
				best_eff = eff;
			}
		}
		devc->logic_sys = 0x17;
		devc->hw_samplerate = best_hw;
		devc->decimate_n = best_n;
		devc->samplerate = best_eff;
		devc->decimate_phase = 0;
		return;
	}

	/* USB3: try every firmware rate × integer N */
	for (i = 0; i < ARRAY_SIZE(ch32_hw_rates); i++) {
		uint64_t hw = ch32_hw_rates[i].rate;
		int max_n, n;

		if (hw > link_max || hw < CH32_UI_RATE_MIN)
			continue;
		max_n = (int)(hw / CH32_UI_RATE_MIN);
		if (max_n < 1)
			max_n = 1;

		/*
		 * Search N around hw/want first (cheap), then ensure N=1
		 * and neighbors are covered for exact HW picks.
		 */
		{
			int n0 = (int)((hw + want / 2) / want);
			int n_lo, n_hi;

			if (n0 < 1)
				n0 = 1;
			if (n0 > max_n)
				n0 = max_n;
			n_lo = n0 - 2;
			if (n_lo < 1)
				n_lo = 1;
			n_hi = n0 + 2;
			if (n_hi > max_n)
				n_hi = max_n;
			/* always include full range if small; else window + 1 */
			if (max_n <= 64) {
				n_lo = 1;
				n_hi = max_n;
			}

			for (n = n_lo; n <= n_hi; n++) {
				uint64_t eff = hw / (uint64_t)n;
				uint64_t err;

				if (eff < CH32_UI_RATE_MIN)
					continue;
				err = (eff > want) ? (eff - want) : (want - eff);
				if (err < best_err ||
				    (err == best_err && n < best_n) ||
				    (err == best_err && n == best_n &&
				     hw < best_hw)) {
					best_err = err;
					best_n = n;
					best_hw = hw;
					best_eff = eff;
					best_sys = ch32_hw_rates[i].logic_sys;
				}
			}
			/* N=1 always considered for exact hardware match */
			if (n_lo > 1) {
				uint64_t eff = hw;
				uint64_t err = (eff > want) ? (eff - want)
							    : (want - eff);
				if (err < best_err ||
				    (err == best_err && 1 < best_n) ||
				    (err == best_err && best_n == 1 &&
				     hw < best_hw)) {
					best_err = err;
					best_n = 1;
					best_hw = hw;
					best_eff = eff;
					best_sys = ch32_hw_rates[i].logic_sys;
				}
			}
		}
	}

	devc->logic_sys = best_sys;
	devc->hw_samplerate = best_hw;
	devc->decimate_n = best_n;
	devc->samplerate = best_eff;
	devc->decimate_phase = 0;
}

/* Insert rate into high→low unique list. */
static int ch32_rate_list_add(uint64_t *out, int n, int cap, uint64_t r)
{
	int i, j;

	if (r < CH32_UI_RATE_MIN || n >= cap)
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

/*
 * UI list: only common integer MHz ≤ link_max (min 1M).
 * Actual capture still maps via ch32_map_samplerate() onto HW + soft N.
 */
static void ch32_build_rate_list(const struct ch32_context *devc,
				 uint64_t *out, int *out_n)
{
	uint64_t link_max = ch32_link_max_hw_rate(devc);
	unsigned int i;
	int n = 0;
	const int cap = 32;

	for (i = 0; i < ARRAY_SIZE(ch32_ui_rates); i++) {
		if (ch32_ui_rates[i] <= link_max &&
		    ch32_ui_rates[i] >= CH32_UI_RATE_MIN)
			n = ch32_rate_list_add(out, n, cap, ch32_ui_rates[i]);
	}

	/* USB2 8ch max 40M already in list; ensure link_max itself if missing */
	if (link_max >= CH32_UI_RATE_MIN)
		n = ch32_rate_list_add(out, n, cap, link_max);

	out[n] = 0;
	*out_n = n;
}

/* Map UI ADC rate → adc_div; rate = 40M/(div+1), div>=1 so max 20M. */
static void ch32_map_adc_samplerate(struct ch32_context *devc, uint64_t want)
{
	unsigned int i;
	uint8_t best_div = 39; /* 1 MHz */
	uint64_t best_rate = CH32_UI_RATE_MIN;
	uint64_t best_err = UINT64_MAX;

	if (want == 0 || want < CH32_UI_RATE_MIN)
		want = CH32_UI_RATE_MIN;
	if (want > CH32_ADC_MAX_HZ)
		want = CH32_ADC_MAX_HZ;

	for (i = CH32_ADC_MIN_DIV; i < 80; i++) {
		uint64_t r = CH32_ADC_CLOCK_HZ / (uint64_t)(i + 1);
		uint64_t err;

		if (r < CH32_UI_RATE_MIN)
			break;
		if (r > CH32_ADC_MAX_HZ)
			continue;
		err = (r > want) ? (r - want) : (want - r);
		if (err < best_err) {
			best_err = err;
			best_div = (uint8_t)i;
			best_rate = r;
		}
	}
	devc->adc_div = best_div;
	devc->hw_samplerate = best_rate;
	devc->samplerate = best_rate;
	devc->decimate_n = 1;
}

static void ch32_build_adc_rate_list(uint64_t *out, int *out_n)
{
	unsigned int i;
	int n = 0;
	const int cap = 16;

	for (i = 0; i < ARRAY_SIZE(ch32_adc_ui_rates); i++)
		n = ch32_rate_list_add(out, n, cap, ch32_adc_ui_rates[i]);
	out[n] = 0;
	*out_n = n;
}

static void ch32_apply_channel_mode(struct sr_dev_inst *sdi)
{
	struct ch32_context *devc = sdi->priv;
	GSList *l;
	int n = (devc->ch_mode == CH32_CHMODE_8) ? 8 : 16;

	devc->channel_count = n;
	devc->logic_bit = (n <= 8) ? 8 : 16;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type != SR_CHANNEL_LOGIC)
			continue;
		ch->enabled = (ch->index < n) ? TRUE : FALSE;
	}
}

static int ch32_cmd(struct ch32_context *devc, uint8_t cmd,
		    const uint8_t *payload, uint8_t plen,
		    uint8_t *rsp, int *rsp_len)
{
	uint8_t buf[32];
	uint8_t rbuf[64];
	int xfer = 0;
	int r;

	if (!devc || !devc->devhdl)
		return SR_ERR;

	memset(buf, 0, sizeof(buf));
	buf[0] = cmd;
	buf[1] = plen;
	if (payload && plen)
		memcpy(&buf[2], payload, plen > 5 ? 5 : plen);

	r = libusb_bulk_transfer(devc->devhdl, CH32_EP_CMD_OUT,
				 buf, 2 + (plen > 5 ? 5 : plen),
				 &xfer, CH32_XFER_TIMEOUT_MS);
	if (r != 0) {
		sr_err("cmd 0x%02X OUT failed: %s", cmd, libusb_error_name(r));
		return SR_ERR;
	}

	/* START/STOP (logic + ADC) have no reply payload (len=0 in FW) */
	if (cmd == CMD_SET_START || cmd == CMD_SET_STOP ||
	    cmd == CMD_SET_ADC_START || cmd == CMD_SET_ADC_STOP)
		return SR_OK;

	memset(rbuf, 0, sizeof(rbuf));
	xfer = 0;
	r = libusb_bulk_transfer(devc->devhdl, CH32_EP_CMD_IN,
				 rbuf, sizeof(rbuf), &xfer, CH32_XFER_TIMEOUT_MS);
	if (r != 0) {
		sr_err("cmd 0x%02X IN failed: %s", cmd, libusb_error_name(r));
		return SR_ERR;
	}

	if (rsp && rsp_len) {
		int n = (xfer > 64) ? 64 : xfer;
		memcpy(rsp, rbuf, n);
		*rsp_len = n;
	}
	return SR_OK;
}

static int ch32_apply_logic_params(struct ch32_context *devc)
{
	uint8_t p[2];
	p[0] = devc->logic_bit;
	p[1] = devc->logic_sys;
	return ch32_cmd(devc, CMD_SET_LOGIC_PARA, p, 2, NULL, NULL);
}

/*
 * Comparator threshold DAC (CMD 0xA1).
 * Uses measured inverse LUT from tools/calibrate_ch32_vth.c
 * (HSADC0 full sweep DAC 0..1024 → v_pin). Non-linear; do not use V/3.3*N.
 *
 *   DAC 0    ≈ 3.44 V
 *   DAC 1024 ≈ 0.43 V
 */
#define CH32_VTH_MAX_V   3.44
#define CH32_VTH_MIN_V   0.43

static uint16_t ch32_vth_to_dac(double v)
{
	int mv;

	if (v >= CH32_VTH_MAX_V)
		return 0; /* highest threshold */
	if (v <= CH32_VTH_MIN_V)
		return ch32_vth_mv_to_dac[0]; /* lowest in LUT (~DAC 1023) */
	mv = (int)(v * 1000.0 + 0.5);
	if (mv < 0)
		mv = 0;
	if (mv > CH32_VTH_LUT_MV_MAX)
		mv = CH32_VTH_LUT_MV_MAX;
	return ch32_vth_mv_to_dac[mv];
}

static void ch32_set_vth(struct ch32_context *devc, double v)
{
	if (v > CH32_VTH_MAX_V)
		v = CH32_VTH_MAX_V;
	if (v < CH32_VTH_MIN_V)
		v = CH32_VTH_MIN_V;
	devc->vth = v;
	devc->logic_level = ch32_vth_to_dac(v);
}

static int ch32_apply_level(struct ch32_context *devc)
{
	uint8_t p[2];

	/* keep DAC in sync with current vth */
	devc->logic_level = ch32_vth_to_dac(devc->vth);
	p[0] = (uint8_t)(devc->logic_level >> 8);
	p[1] = (uint8_t)(devc->logic_level & 0xff);
	sr_info("VTH apply: %.3f V → DAC %u (HSADC0 cal LUT)",
		devc->vth, (unsigned)devc->logic_level);
	return ch32_cmd(devc, CMD_SET_LOGIC_LEVEL, p, 2, NULL, NULL);
}

static int ch32_apply_adc_params(struct ch32_context *devc)
{
	uint8_t p[2];
	/* Always force 10-bit (hardware product spec) */
	devc->adc_bit = ADC_BIT_10;
	if (devc->adc_div < CH32_ADC_MIN_DIV)
		devc->adc_div = CH32_ADC_MIN_DIV;
	p[0] = devc->adc_bit;
	p[1] = devc->adc_div;
	return ch32_cmd(devc, CMD_SET_ADC_PARA, p, 2, NULL, NULL);
}

static int ch32_apply_adc_channel(struct ch32_context *devc)
{
	uint8_t p[1];
	p[0] = devc->adc_channel;
	return ch32_cmd(devc, CMD_SET_ADC_CHANNEL, p, 1, NULL, NULL);
}

static void ch32_sync_adc_channel_from_probes(struct sr_dev_inst *sdi)
{
	struct ch32_context *devc = sdi->priv;
	GSList *l;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch->type == SR_CHANNEL_ANALOG && ch->enabled) {
			devc->adc_channel = (ch->index == 1) ? ADC_CH1 : ADC_CH0;
			return;
		}
	}
	devc->adc_channel = ADC_CH0;
}

static void ch32_setup_logic_probes(struct sr_dev_inst *sdi)
{
	int i;

	sr_dev_probes_free(sdi);
	for (i = 0; i < 16; i++) {
		sdi->channels = g_slist_append(sdi->channels,
			sr_channel_new(i, SR_CHANNEL_LOGIC, TRUE, probe_names[i]));
	}
	ch32_apply_channel_mode(sdi);
}

static void ch32_setup_analog_probes(struct sr_dev_inst *sdi)
{
	struct ch32_context *devc = sdi->priv;
	int i;
	/* UI path is 8-bit (envelope + AnalogSnapshot); mid-code = 128 */
	const int bits = CH32_ADC_UI_BITS;
	uint16_t mid = (uint16_t)CH32_ADC_UI_MID;

	devc->adc_bit = ADC_BIT_10;
	sr_dev_probes_free(sdi);
	for (i = 0; i < 2; i++) {
		struct sr_channel *probe;
		char name[8];

		snprintf(name, sizeof(name), "A%d", i);
		probe = sr_channel_new(i, SR_CHANNEL_ANALOG, TRUE, name);
		if (!probe)
			continue;
		probe->bits = bits;
		/* Only one HW channel at a time; default enable A0 */
		probe->enabled = (i == (int)devc->adc_channel) ? TRUE : FALSE;
		probe->coupling = SR_DC_COUPLING;
		probe->vdiv = SR_V(1);
		probe->vfactor = 1;
		probe->hw_offset = mid;
		probe->offset = mid;
		probe->trig_value = mid;
		probe->map_default = TRUE;
		probe->map_unit = ch32_map_units[0];
		/* Unipolar 0..Vref; labels: top=Vref, bottom=0 */
		probe->map_min = 0.0;
		probe->map_max = CH32_ADC_VREF;
		sdi->channels = g_slist_append(sdi->channels, probe);
	}
}

/* Count all analog probes (enabled or not) — matches AnalogSnapshot packing. */
static int ch32_analog_probe_count(const struct sr_dev_inst *sdi)
{
	GSList *l;
	int n = 0;

	if (!sdi)
		return 0;
	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch && ch->type == SR_CHANNEL_ANALOG)
			n++;
	}
	return n;
}

/*
 * Convert mono LE 10-bit EP3 samples → DSView 8-bit interleaved.
 * AnalogSnapshot first_payload counts *all* SR_CHANNEL_ANALOG channels and
 * treats data as: [ch0, ch1, ...] * unit_bytes per time step.
 * Only the active HW channel carries converted samples; others get mid.
 *
 * Invert to 8-bit so higher voltage draws toward the top of the plot
 * (DSView y grows downward with raw code; left scale puts map_max at top).
 */
static int ch32_analog_expand_feed(struct ch32_context *devc,
				   const uint8_t *raw, int raw_len)
{
	const struct sr_dev_inst *sdi = devc->sdi;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	GSList *probes = NULL;
	GSList *l;
	int n_ch;
	int active;
	int need;
	int nsamp;
	int i, c;
	int total_raw;
	const uint8_t *src;
	uint8_t *out;

	if (!sdi || raw_len <= 0)
		return SR_OK;

	n_ch = ch32_analog_probe_count(sdi);
	if (n_ch <= 0)
		n_ch = 1;
	active = (int)devc->adc_channel;
	if (active < 0 || active >= n_ch)
		active = 0;

	/* stitch residual odd byte from previous USB packet */
	total_raw = devc->stream_res_len + raw_len;
	if (total_raw < CH32_ADC_BPS) {
		if (raw_len > 0 && devc->stream_res_len < 2) {
			devc->stream_res[devc->stream_res_len] = raw[0];
			devc->stream_res_len++;
		}
		return SR_OK;
	}

	/* build contiguous raw view: residual + this packet (except new residual) */
	{
		int full = total_raw & ~1; /* even bytes only */
		int use_from_pkt = full - devc->stream_res_len;
		uint8_t *tmp = NULL;
		const uint8_t *merged;

		if (devc->stream_res_len) {
			tmp = g_try_malloc((size_t)full);
			if (!tmp)
				return SR_ERR_MALLOC;
			memcpy(tmp, devc->stream_res, (size_t)devc->stream_res_len);
			memcpy(tmp + devc->stream_res_len, raw, (size_t)use_from_pkt);
			merged = tmp;
		} else {
			merged = raw;
		}

		nsamp = full / CH32_ADC_BPS;
		need = nsamp * n_ch; /* 1 byte per ch per sample */
		if (need > devc->analog_expand_cap) {
			uint8_t *p = g_try_realloc(devc->analog_expand, (size_t)need);
			if (!p) {
				g_free(tmp);
				return SR_ERR_MALLOC;
			}
			devc->analog_expand = p;
			devc->analog_expand_cap = need;
		}
		out = devc->analog_expand;
		src = merged;

		for (i = 0; i < nsamp; i++) {
			uint16_t raw10 = (uint16_t)src[i * 2] |
					 ((uint16_t)src[i * 2 + 1] << 8);
			uint8_t u8;

			raw10 &= 0x3ff;
			/* 10-bit → 8-bit, invert for DSView top=high-V convention */
			u8 = (uint8_t)(255u - ((raw10 * 255u) / 1023u));
			for (c = 0; c < n_ch; c++)
				out[i * n_ch + c] =
					(c == active) ? u8 : (uint8_t)CH32_ADC_UI_MID;
		}

		/* new residual: last odd byte of this packet if any */
		if (total_raw & 1) {
			devc->stream_res[0] = raw[raw_len - 1];
			devc->stream_res_len = 1;
		} else {
			devc->stream_res_len = 0;
		}

		g_free(tmp);
	}

	if (nsamp <= 0)
		return SR_OK;

	for (l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch && ch->type == SR_CHANNEL_ANALOG)
			probes = g_slist_append(probes, ch);
	}
	if (!probes)
		return SR_OK;

	packet.type = SR_DF_ANALOG;
	packet.status = SR_PKT_OK;
	packet.payload = &analog;
	analog.probes = probes;
	analog.num_samples = (uint64_t)nsamp;
	analog.unit_bits = CH32_ADC_UI_BITS;
	analog.unit_pitch = 1;
	analog.mq = SR_MQ_VOLTAGE;
	analog.unit = SR_UNIT_VOLT;
	analog.mqflags = 0;
	analog.data = out;
	ds_data_forward(sdi, &packet);
	devc->num_samples += (uint64_t)nsamp;
	devc->num_bytes += (uint64_t)(nsamp * n_ch);
	g_slist_free(probes);
	return SR_OK;
}

/* Build enabled physical-bit list in UI channel order. */
static void ch32_refresh_enabled_bits(struct ch32_context *devc)
{
	GSList *l;
	int n = 0;
	int max_bit = (devc->logic_bit == 8) ? 8 : 16;

	for (l = devc->sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (!ch->enabled)
			continue;
		if (ch->index >= max_bit)
			continue;
		if (n >= CH32_MAX_PHYS_CH)
			break;
		devc->en_bits[n++] = (uint8_t)ch->index;
	}
	if (n == 0) {
		/* fallback: all physical channels */
		int i;
		for (i = 0; i < max_bit; i++)
			devc->en_bits[i] = (uint8_t)i;
		n = max_bit;
	}
	devc->en_count = n;
}

/*
 * Convert CH32 raw parallel stream → DSView LA_CROSS_DATA.
 *
 * CH32 raw (UHSIF):
 *   8-bit:  [s0][s1][s2]... each byte = D7..D0 at one sample time
 *   16-bit: [s0_lo][s0_hi][s1_lo][s1_hi]... little-endian u16 = D15..D0
 *
 * LA_CROSS_DATA (DSView):
 *   For each block of 64 samples:
 *     [ch0 plane 8B][ch1 plane 8B]...[chN-1 plane 8B]
 *   plane bit i = sample i of that channel (LSB = earlier sample).
 */
static int ch32_ensure_cross_cap(struct ch32_context *devc, int need)
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

static int ch32_ensure_pending_cap(struct ch32_context *devc, int need)
{
	uint8_t *p;
	if (need <= devc->raw_pending_cap)
		return SR_OK;
	need = (need + 1023) & ~1023;
	p = g_try_realloc(devc->raw_pending, need);
	if (!p)
		return SR_ERR_MALLOC;
	devc->raw_pending = p;
	devc->raw_pending_cap = need;
	return SR_OK;
}

static uint16_t ch32_load_sample(const uint8_t *p, int sample_bytes)
{
	if (sample_bytes == 1)
		return p[0];
	/* little-endian 16-bit parallel word */
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void ch32_pending_push_sample(struct ch32_context *devc, uint16_t s,
				     int sample_bytes)
{
	if (sample_bytes == 1) {
		devc->raw_pending[devc->raw_pending_len++] = (uint8_t)s;
	} else {
		devc->raw_pending[devc->raw_pending_len++] =
			(uint8_t)(s & 0xff);
		devc->raw_pending[devc->raw_pending_len++] =
			(uint8_t)(s >> 8);
	}
}

/* Per-bit majority over N digital samples → one output sample. */
static uint16_t ch32_majority_sample(const uint16_t *win, int n)
{
	int b, i;
	uint16_t out = 0;
	int thr;

	if (n <= 1)
		return win[0];
	/* strict majority; for even N ties keep last sample's bit */
	thr = (n / 2) + 1;
	for (b = 0; b < 16; b++) {
		int ones = 0;
		for (i = 0; i < n; i++) {
			if ((win[i] >> b) & 1u)
				ones++;
		}
		if (ones >= thr)
			out |= (uint16_t)(1u << b);
		else if (ones * 2 == n) {
			/* tie: keep last sample bit (stable edge preference) */
			if ((win[n - 1] >> b) & 1u)
				out |= (uint16_t)(1u << b);
		}
	}
	return out;
}

static uint16_t ch32_apply_1t_filter(struct ch32_context *devc, uint16_t in)
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
	if (in == devc->filt_prev_in)
		outv = in;
	else
		outv = devc->filt_prev_out;
	devc->filt_prev_in = in;
	devc->filt_prev_out = outv;
	return outv;
}

/*
 * Append USB raw samples into pending after:
 *  - reassemble multi-byte samples across packet boundaries
 *  - optional 1T glitch filter
 *  - soft decimation: majority-vote over N samples (not bare pick-1-of-N,
 *    which aliases digital edges into thin spikes)
 */
static int ch32_preprocess_append(struct ch32_context *devc,
				  const uint8_t *raw, int raw_len)
{
	const int sample_bytes = (devc->logic_bit == 8) ? 1 : 2;
	const uint8_t *p;
	int left;
	int n_est;
	uint8_t tmp[4];

	if (!raw || raw_len <= 0)
		return SR_OK;

	/* Merge stream residual (half sample from previous URB). */
	if (devc->stream_res_len > 0) {
		int need = sample_bytes - devc->stream_res_len;
		int take;

		if (need < 0)
			need = 0;
		take = (raw_len < need) ? raw_len : need;
		memcpy(devc->stream_res + devc->stream_res_len, raw, take);
		devc->stream_res_len += take;
		raw += take;
		raw_len -= take;
		if (devc->stream_res_len < sample_bytes) {
			/* still incomplete */
			return SR_OK;
		}
		/* one complete sample in stream_res, then rest of raw */
		memcpy(tmp, devc->stream_res, sample_bytes);
		devc->stream_res_len = 0;
		/* process this one sample first via recursive-style path */
		p = tmp;
		left = sample_bytes;
		/* fall through using concatenated view: handle tmp then raw */
		n_est = 1 + raw_len / sample_bytes;
		if (ch32_ensure_pending_cap(devc,
			devc->raw_pending_len + n_est * sample_bytes) != SR_OK)
			return SR_ERR_MALLOC;
		goto process_one_then_raw;
	}

	n_est = raw_len / sample_bytes;
	if (n_est <= 0) {
		/* only a partial sample in this packet */
		if (raw_len > 0 && raw_len < sample_bytes) {
			memcpy(devc->stream_res, raw, raw_len);
			devc->stream_res_len = raw_len;
		}
		return SR_OK;
	}

	if (ch32_ensure_pending_cap(devc,
		devc->raw_pending_len + n_est * sample_bytes) != SR_OK)
		return SR_ERR_MALLOC;

	p = raw;
	left = n_est * sample_bytes;
	/* save trailing partial */
	if (raw_len > left) {
		int rem = raw_len - left;
		memcpy(devc->stream_res, raw + left, rem);
		devc->stream_res_len = rem;
	}

	{
		int i, n_samples = left / sample_bytes;
		int N = devc->decimate_n > 1 ? devc->decimate_n : 1;

		if (N > 1) {
			if (!devc->decim_win || devc->decim_win_cap < N) {
				uint16_t *nw = g_try_realloc(devc->decim_win,
					(gsize)N * sizeof(uint16_t));
				if (!nw)
					return SR_ERR_MALLOC;
				devc->decim_win = nw;
				devc->decim_win_cap = N;
			}
		}

		for (i = 0; i < n_samples; i++) {
			uint16_t s = ch32_load_sample(p + i * sample_bytes,
						      sample_bytes);
			s = ch32_apply_1t_filter(devc, s);

			if (N <= 1) {
				ch32_pending_push_sample(devc, s, sample_bytes);
				continue;
			}

			/* Collect N samples, emit majority (anti-alias digital) */
			devc->decim_win[devc->decim_win_fill++] = s;
			if (devc->decim_win_fill >= N) {
				uint16_t m = ch32_majority_sample(devc->decim_win, N);
				ch32_pending_push_sample(devc, m, sample_bytes);
				devc->decim_win_fill = 0;
			}
		}
	}
	return SR_OK;

process_one_then_raw:
	{
		uint16_t s = ch32_load_sample(p, sample_bytes);
		int N = devc->decimate_n > 1 ? devc->decimate_n : 1;
		s = ch32_apply_1t_filter(devc, s);
		if (N <= 1) {
			ch32_pending_push_sample(devc, s, sample_bytes);
		} else {
			if (!devc->decim_win || devc->decim_win_cap < N) {
				uint16_t *nw = g_try_realloc(devc->decim_win,
					(gsize)N * sizeof(uint16_t));
				if (!nw)
					return SR_ERR_MALLOC;
				devc->decim_win = nw;
				devc->decim_win_cap = N;
			}
			devc->decim_win[devc->decim_win_fill++] = s;
			if (devc->decim_win_fill >= N) {
				uint16_t m = ch32_majority_sample(devc->decim_win, N);
				ch32_pending_push_sample(devc, m, sample_bytes);
				devc->decim_win_fill = 0;
			}
		}
		/* continue with remaining raw without residual (already merged) */
		if (raw_len > 0) {
			const uint8_t *save_raw = raw;
			int save_len = raw_len;
			/* recursive-style: process rest; residual already 0 */
			p = save_raw;
			left = (save_len / sample_bytes) * sample_bytes;
			if (save_len > left) {
				memcpy(devc->stream_res, save_raw + left,
				       save_len - left);
				devc->stream_res_len = save_len - left;
			}
			if (left > 0) {
				int i, n_samples = left / sample_bytes;
				for (i = 0; i < n_samples; i++) {
					s = ch32_load_sample(p + i * sample_bytes,
							     sample_bytes);
					s = ch32_apply_1t_filter(devc, s);
					if (N <= 1) {
						ch32_pending_push_sample(devc, s,
									 sample_bytes);
					} else {
						devc->decim_win[devc->decim_win_fill++] = s;
						if (devc->decim_win_fill >= N) {
							uint16_t m = ch32_majority_sample(
								devc->decim_win, N);
							ch32_pending_push_sample(devc, m,
										 sample_bytes);
							devc->decim_win_fill = 0;
						}
					}
				}
			}
		}
	}
	return SR_OK;
}

/*
 * Send firmware STOP/ADC_STOP. Must NOT be called from a libusb transfer
 * callback (sync bulk re-enters the event loop). Safe from receive_data /
 * acquisition_stop / close.
 */
static void ch32_hw_stop(struct ch32_context *devc)
{
	int ret;

	if (!devc || !devc->fw_streaming)
		return;

	if (devc->acq_analog)
		ret = ch32_cmd(devc, CMD_SET_ADC_STOP, NULL, 0, NULL, NULL);
	else
		ret = ch32_cmd(devc, CMD_SET_STOP, NULL, 0, NULL, NULL);
	if (ret != SR_OK) {
		/* keep fw_streaming so receive_data can retry */
		sr_err("HW stop cmd failed, will retry");
		return;
	}
	devc->fw_streaming = 0;
	sr_info("HW stop sent (capture LED off)");
}

static void ch32_cancel_transfers(struct ch32_context *devc)
{
	unsigned int i;

	if (!devc || !devc->transfers)
		return;
	for (i = 0; i < (unsigned)devc->num_transfers; i++) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

/* Host-side stop: abort + cancel. HW STOP is deferred (see receive_data). */
static void ch32_request_acq_stop(struct ch32_context *devc)
{
	if (!devc || devc->abort)
		return;
	devc->abort = 1;
	devc->status = CH32_ST_STOP;
	ch32_cancel_transfers(devc);
}

/* Convert complete 64-sample groups from pending → LA_CROSS_DATA. */
static int ch32_convert_push(struct ch32_context *devc,
			     const uint8_t *raw, int raw_len)
{
	const int sample_bytes = (devc->logic_bit == 8) ? 1 : 2;
	const int en = devc->en_count;
	const int raw_group = CH32_CROSS_SCALE * sample_bytes;
	const int cross_group = CH32_CROSS_PLANE_BYTES * en;
	int groups, g, c, i, consumed, left;
	uint8_t *out;
	int out_len;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	uint64_t samples_out;

	if (en < 1 || raw_len <= 0)
		return SR_OK;

	if (ch32_preprocess_append(devc, raw, raw_len) != SR_OK)
		return SR_ERR_MALLOC;

	groups = devc->raw_pending_len / raw_group;
	if (groups <= 0)
		return SR_OK;

	/* Single/buffer: stop at sample limit. Loop/rolling: keep forwarding. */
	if (devc->limit_samples && !devc->is_loop) {
		uint64_t room = (devc->num_samples < devc->limit_samples)
			? (devc->limit_samples - devc->num_samples) : 0;
		int max_groups = (int)(room / CH32_CROSS_SCALE);
		if (max_groups <= 0) {
			/*
			 * Remaining room < one 64-sample plane: treat capture
			 * as complete so the host stops the device (LED).
			 */
			devc->num_samples = devc->limit_samples;
			return SR_OK;
		}
		if (groups > max_groups)
			groups = max_groups;
	}

	out_len = groups * cross_group;
	if (ch32_ensure_cross_cap(devc, out_len) != SR_OK)
		return SR_ERR_MALLOC;

	out = devc->cross_buf;
	for (g = 0; g < groups; g++) {
		const uint8_t *blk = devc->raw_pending + g * raw_group;
		for (c = 0; c < en; c++) {
			uint64_t plane = 0;
			uint8_t bit = devc->en_bits[c];
			for (i = 0; i < CH32_CROSS_SCALE; i++) {
				uint16_t s = ch32_load_sample(
					blk + i * sample_bytes, sample_bytes);
				if ((s >> bit) & 1u)
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
			out += CH32_CROSS_PLANE_BYTES;
		}
	}

	samples_out = (uint64_t)groups * CH32_CROSS_SCALE;

	packet.type = SR_DF_LOGIC;
	packet.status = SR_PKT_OK;
	packet.payload = &logic;
	logic.length = (uint64_t)out_len;
	logic.format = LA_CROSS_DATA;
	logic.data_error = 0;
	logic.data = devc->cross_buf;
	ds_data_forward(devc->sdi, &packet);

	devc->num_samples += samples_out;
	devc->num_bytes += (uint64_t)out_len;

	/* Keep incomplete residual */
	consumed = groups * raw_group;
	left = devc->raw_pending_len - consumed;
	if (left > 0)
		memmove(devc->raw_pending, devc->raw_pending + consumed, left);
	devc->raw_pending_len = left;

	return SR_OK;
}

/* -------------------- transfer path -------------------- */

static void finish_acquisition(struct ch32_context *devc)
{
	struct sr_datafeed_packet packet;

	if (!devc || devc->status == CH32_ST_FINISH)
		return;

	/* drop residual incomplete 64-sample group */
	devc->raw_pending_len = 0;
	devc->abort = 1;

	/*
	 * Prefer HW stop from receive_data (outside transfer callback).
	 * Still try here if not yet stopped — best-effort; may fail if we
	 * are nested inside libusb_handle_events.
	 */
	ch32_hw_stop(devc);

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
	devc->status = CH32_ST_FINISH;
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct ch32_context *devc = transfer->user_data;
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

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct ch32_context *devc = transfer->user_data;
	int r;

	if (devc->status == CH32_ST_START)
		devc->status = CH32_ST_DATA;

	if (devc->abort)
		devc->status = CH32_ST_STOP;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED &&
	    transfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
		if (!devc->abort)
			sr_err("transfer status %d", transfer->status);
		devc->status = CH32_ST_STOP;
	}

	if (devc->status == CH32_ST_DATA && transfer->actual_length > 0) {
		if (devc->acq_analog) {
			/* mono 10-bit EP3 → 8-bit interleaved SR_DF_ANALOG */
			if (ch32_analog_expand_feed(devc, transfer->buffer,
						    transfer->actual_length) != SR_OK) {
				sr_err("analog expand/feed failed");
				ch32_request_acq_stop(devc);
			}
		} else if (ch32_convert_push(devc, transfer->buffer,
					     transfer->actual_length) != SR_OK) {
			sr_err("format convert failed");
			ch32_request_acq_stop(devc);
		}

		/*
		 * Natural end at sample limit: do NOT call ch32_cmd() here —
		 * sync bulk from a transfer callback can fail/reenter libusb.
		 * Abort + cancel; receive_data will send HW STOP after
		 * handle_events returns (clears capture LED).
		 */
		if (!devc->is_loop &&
		    devc->limit_samples &&
		    devc->num_samples >= devc->limit_samples)
			ch32_request_acq_stop(devc);
	}

	if (devc->status == CH32_ST_DATA && !devc->abort) {
		r = libusb_submit_transfer(transfer);
		if (r != 0) {
			sr_err("resubmit failed: %s", libusb_error_name(r));
			free_transfer(transfer);
		}
	} else {
		free_transfer(transfer);
	}
}

static int start_transfers(struct ch32_context *devc)
{
	int i;
	unsigned char *buf;
	struct libusb_transfer *xfer;
	uint8_t ep = devc->acq_analog ? CH32_EP_ADC_IN : CH32_EP_LOGIC_IN;

	devc->num_transfers = CH32_NUM_TRANSFERS;
	devc->submitted_transfers = 0;
	devc->transfers = g_try_malloc0(sizeof(struct libusb_transfer *) *
					devc->num_transfers);
	if (!devc->transfers)
		return SR_ERR_MALLOC;

	for (i = 0; i < devc->num_transfers; i++) {
		buf = g_try_malloc0(CH32_TRANSFER_SIZE);
		if (!buf)
			return SR_ERR_MALLOC;
		xfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(xfer, devc->devhdl, ep,
					  buf, CH32_TRANSFER_SIZE,
					  receive_transfer, devc,
					  CH32_BULK_TIMEOUT_MS);
		if (libusb_submit_transfer(xfer) != 0) {
			sr_err("submit transfer %d (ep=0x%02x) failed", i, ep);
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
	struct ch32_context *devc = sdi->priv;
	struct drv_context *drvc = di->priv;
	struct timeval tv = { 0, 0 };
	int completed = 0;

	(void)fd;
	(void)revents;

	libusb_handle_events_timeout_completed(drvc->sr_ctx->libusb_ctx,
					       &tv, &completed);

	/*
	 * After transfer callbacks return: safe place for sync STOP.
	 * Natural sample-limit end only sets abort inside the callback.
	 */
	if (devc->fw_streaming && devc->abort)
		ch32_hw_stop(devc);

	if (devc->status == CH32_ST_FINISH) {
		ch32_hw_stop(devc);
		if (devc->freewheel)
			sr_session_source_remove(-1);
		return TRUE;
	}
	return TRUE;
}

/* -------------------- driver API -------------------- */

static int hw_init(struct sr_context *sr_ctx)
{
	unsigned int i, n = 0;
	/* UI combobox fallback: common integer MHz only */
	for (i = 0; i < ARRAY_SIZE(ch32_ui_rates); i++)
		ch32_samplerate_list[n++] = ch32_ui_rates[i];
	ch32_samplerate_list[n] = 0;
	ch32_samplerate_count = (int)n;
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
	struct ch32_context *devc;
	GSList *devices = NULL;
	int i, ret;

	(void)options;
	drvc = di->priv;
	if (!drvc || !drvc->sr_ctx)
		return NULL;

	libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (!devlist)
		return NULL;

	for (i = 0; devlist[i]; i++) {
		if (libusb_get_device_descriptor(devlist[i], &des) != 0)
			continue;
		if (des.idVendor != CH32_VID || !ch32_is_app_pid(des.idProduct))
			continue;
		if (sr_usb_device_is_exists(devlist[i]))
			continue;

		devc = g_try_malloc0(sizeof(*devc));
		if (!devc)
			break;

		{
			uint8_t bus = libusb_get_bus_number(devlist[i]);
			uint8_t address = libusb_get_device_address(devlist[i]);
			struct sr_usb_dev_inst *usb_info;

			devc->usb_dev = libusb_ref_device(devlist[i]);
			devc->sr_ctx = drvc->sr_ctx;
			devc->usb_pid = des.idProduct;
			devc->logic_bit = 16;
			devc->ch_mode = CH32_CHMODE_16;
			devc->filter = SR_FILTER_NONE;
			devc->limit_samples = CH32_DEFAULT_SAMPLES;
			devc->channel_count = 16;
			devc->max_height = 1;
			/* ~1.65 V CMOS mid → DAC ≈ 805 (not 2048!) */
			ch32_set_vth(devc, 1.65);
			devc->op_mode = LO_OP_STREAM;
			devc->is_loop = 0;
			devc->decimate_n = 1;
			devc->usb_speed = LIBUSB_SPEED_UNKNOWN;
			devc->adc_bit = ADC_BIT_10; /* HSADC fixed 10-bit */
			devc->adc_div = 3; /* 10 MHz: 40/(3+1) */
			devc->adc_channel = ADC_CH0; /* A0 / PF8 */
			devc->acq_analog = 0;

			sdi = sr_dev_inst_new(LOGIC, SR_ST_INACTIVE,
					      "WCH", "CH32H417 Logic Analyzer", "1.0");
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
			/* USB2/USB3 decided by PID (5537/5538) */
			ch32_update_usb_speed(devc);
			/* default 100M on USB3; USB2 will clamp in map */
			ch32_map_samplerate(devc, SR_MHZ(100));
			/*
			 * Must set conn (bus/addr) like DSLogic so Windows
			 * hotplug reconnect can update the handle. Without
			 * it, USB2 re-plug often leaves a stale entry and
			 * the device "disappears" from the UI.
			 */
			usb_info = sr_usb_dev_inst_new(bus, address);
			if (usb_info) {
				usb_info->usb_dev = devc->usb_dev;
				sdi->conn = usb_info;
			} else {
				sdi->conn = NULL;
			}
			/* always create 16 probes; channel mode enables 8 or 16 */
			setup_channels(sdi, 16);
			ch32_apply_channel_mode(sdi);

			sr_info("Found CH32 LA 1A86:%04X bus=%u addr=%u handle=%p speed=%s name=%s",
				des.idProduct, bus, address, devc->usb_dev,
				ch32_speed_name(devc->usb_speed),
				sdi->name ? sdi->name : "?");
			devices = g_slist_append(devices, sdi);
		}
	}

	libusb_free_device_list(devlist, 0);
	(void)ret;
	return devices;
}

static const GSList *hw_dev_mode_list(const struct sr_dev_inst *sdi)
{
	(void)sdi;
	GSList *l = NULL;
	/* sr_mode_list order: [0]=LOGIC, [1]=ANALOG(daq), [2]=DSO */
	l = g_slist_append(l, (gpointer)&sr_mode_list[0]);
	l = g_slist_append(l, (gpointer)&sr_mode_list[1]);
	return l;
}

static int hw_dev_open(struct sr_dev_inst *sdi)
{
	struct ch32_context *devc = sdi->priv;
	uint8_t rsp[64];
	int rlen = 0;
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

	if (ch32_cmd(devc, CMD_GET_VERSION, NULL, 0, rsp, &rlen) == SR_OK &&
	    rlen >= 5) {
		devc->sw_ver = rsp[3];
		devc->hw_ver = rsp[4];
		sr_info("CH32 version sw=%u hw=%u", devc->sw_ver, devc->hw_ver);
	}

	/* Re-read speed after open (product string more reliable with handle). */
	devc->sdi = sdi;
	ch32_update_usb_speed(devc);
	ch32_map_samplerate(devc, devc->samplerate ? devc->samplerate
						   : SR_MHZ(100));

	ch32_apply_logic_params(devc);
	ch32_apply_level(devc);

	sdi->status = SR_ST_ACTIVE;
	sr_info("CH32 open: name=\"%s\" speed=%s",
		sdi->name ? sdi->name : "?",
		ch32_speed_name(devc->usb_speed));
	return SR_OK;
}

static int hw_dev_close(struct sr_dev_inst *sdi)
{
	struct ch32_context *devc;

	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	if (devc->devhdl) {
		if (sdi->status == SR_ST_ACTIVE) {
			devc->acq_analog = (sdi->mode == ANALOG) ? 1 : devc->acq_analog;
			/* force stop even if flag was lost */
			if (!devc->fw_streaming)
				devc->fw_streaming = 1;
			ch32_hw_stop(devc);
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
	struct ch32_context *devc;

	if (!sdi)
		return SR_ERR;

	hw_dev_close(sdi);
	devc = sdi->priv;
	if (devc) {
		if (devc->usb_dev)
			libusb_unref_device(devc->usb_dev);
		g_free(devc->raw_pending);
		g_free(devc->cross_buf);
		g_free(devc->decim_win);
		g_free(devc->analog_expand);
		g_free(devc);
		sdi->priv = NULL;
	}
	if (sdi->conn) {
		/* usb_dev already unref'd via priv */
		sr_usb_dev_inst_free(sdi->conn);
		sdi->conn = NULL;
	}
	sr_dev_inst_free(sdi);
	return SR_OK;
}

/*
 * Called from lib_main update_device_handle() after USB re-plug
 * (e.g. move from USB3 port to USB2 port).
 */
SR_PRIV void ch32_on_usb_reconnected(struct sr_dev_inst *sdi,
				     struct libusb_device *new_dev)
{
	struct ch32_context *devc;

	if (!sdi || !sdi->priv || !new_dev)
		return;
	if (!sdi->driver || !sdi->driver->name ||
	    strcmp(sdi->driver->name, "ch32-logic") != 0)
		return;

	devc = sdi->priv;
	if (devc->usb_dev)
		libusb_unref_device(devc->usb_dev);
	devc->usb_dev = libusb_ref_device(new_dev);
	devc->sdi = sdi;
	sdi->handle = (ds_device_handle)devc->usb_dev;
	ch32_update_usb_speed(devc);
	ch32_map_samplerate(devc, devc->samplerate ? devc->samplerate
						   : SR_MHZ(100));
	sr_info("CH32 reconnected: name=\"%s\" speed=%s max_hw=%" PRIu64,
		sdi->name ? sdi->name : "?",
		ch32_speed_name(devc->usb_speed),
		ch32_link_max_hw_rate(devc));
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
	struct ch32_context *devc;

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
		if (sdi->mode == ANALOG)
			/* 8-bit UI stream (HW is 10-bit; see ch32_analog_expand_feed) */
			*data = g_variant_new_byte(CH32_ADC_UI_BITS);
		else
			*data = g_variant_new_byte(1);
		break;
	case SR_CONF_PROBE_VDIV:
		if (ch)
			*data = g_variant_new_uint64(ch->vdiv);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_FACTOR:
		if (ch)
			*data = g_variant_new_uint64(ch->vfactor);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_COUPLING:
		if (ch)
			*data = g_variant_new_byte(ch->coupling);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_HW_OFFSET:
		if (ch)
			*data = g_variant_new_uint16(ch->hw_offset);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_OFFSET:
		if (ch)
			*data = g_variant_new_uint16(ch->offset);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_MAP_DEFAULT:
		if (ch)
			*data = g_variant_new_boolean(ch->map_default);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_MAP_UNIT:
		if (ch)
			*data = g_variant_new_string(ch->map_unit ? ch->map_unit : "V");
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_MAP_MIN:
		if (ch)
			*data = g_variant_new_double(ch->map_min);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_PROBE_MAP_MAX:
		if (ch)
			*data = g_variant_new_double(ch->map_max);
		else
			return SR_ERR_ARG;
		break;
	case SR_CONF_MAX_HEIGHT:
		*data = g_variant_new_string(maxHeights[devc->max_height]);
		break;
	case SR_CONF_MAX_HEIGHT_VALUE:
		*data = g_variant_new_byte(devc->max_height);
		break;
	case SR_CONF_HW_DEPTH:
		*data = g_variant_new_uint64(CH32_HW_DEPTH);
		break;
	case SR_CONF_TOTAL_CH_NUM:
		*data = g_variant_new_int16(16);
		break;
	case SR_CONF_VLD_CH_NUM:
		*data = g_variant_new_int16(devc->channel_count);
		break;
	case SR_CONF_STREAM:
		/* Continuous USB stream (required for UI Loop/rolling) */
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
		/* Actual cable/port link speed (for UI + low-speed warning). */
		*data = g_variant_new_int32((int32_t)devc->usb_speed);
		break;
	case SR_CONF_USB30_SUPPORT:
		/* Device silicon supports SuperSpeed. */
		*data = g_variant_new_boolean(ch32_usb30_capable());
		break;
	case SR_CONF_RLE:
		*data = g_variant_new_boolean(FALSE);
		break;
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
	struct ch32_context *devc;
	const char *stropt;
	unsigned int i;

	(void)cg;
	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;
	devc = sdi->priv;

	switch (id) {
	case SR_CONF_DEVICE_MODE: {
		int16_t mode = g_variant_get_int16(data);
		if (mode != LOGIC && mode != ANALOG)
			return SR_ERR_ARG;
		if (sdi->mode == mode)
			break;
		sdi->mode = mode;
		if (mode == ANALOG) {
			ch32_setup_analog_probes(sdi);
			ch32_map_adc_samplerate(devc,
				devc->samplerate ? devc->samplerate : SR_MHZ(10));
			if (sdi->status == SR_ST_ACTIVE) {
				ch32_apply_adc_params(devc);
				ch32_apply_adc_channel(devc);
			}
			sr_info("switch mode → ANALOG (HSADC) ch=%u bit=%u div=%u rate=%" PRIu64,
				devc->adc_channel, devc->adc_bit, devc->adc_div,
				devc->samplerate);
		} else {
			ch32_setup_logic_probes(sdi);
			ch32_map_samplerate(devc,
				devc->samplerate ? devc->samplerate : SR_MHZ(100));
			if (sdi->status == SR_ST_ACTIVE) {
				ch32_apply_logic_params(devc);
				ch32_apply_level(devc);
			}
			sr_info("switch mode → LOGIC");
		}
		break;
	}
	case SR_CONF_SAMPLERATE:
		if (sdi->mode == ANALOG) {
			ch32_map_adc_samplerate(devc, g_variant_get_uint64(data));
			if (sdi->status == SR_ST_ACTIVE)
				ch32_apply_adc_params(devc);
			sr_info("ADC samplerate=%" PRIu64 " div=%u",
				devc->samplerate, devc->adc_div);
		} else {
			ch32_map_samplerate(devc, g_variant_get_uint64(data));
			if (sdi->status == SR_ST_ACTIVE)
				ch32_apply_logic_params(devc);
			sr_info("samplerate ui=%" PRIu64 " hw=%" PRIu64
				" sys=0x%02x decim=%d",
				devc->samplerate, devc->hw_samplerate,
				devc->logic_sys, devc->decimate_n);
		}
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_EN:
		if (ch) {
			ch->enabled = g_variant_get_boolean(data);
			if (sdi->mode == ANALOG) {
				/* single active HW channel */
				if (ch->enabled) {
					GSList *l;
					for (l = sdi->channels; l; l = l->next) {
						struct sr_channel *o = l->data;
						if (o != ch && o->type == SR_CHANNEL_ANALOG)
							o->enabled = FALSE;
					}
					devc->adc_channel = (ch->index == 1) ? ADC_CH1 : ADC_CH0;
					if (sdi->status == SR_ST_ACTIVE)
						ch32_apply_adc_channel(devc);
				}
			}
		}
		break;
	case SR_CONF_PROBE_VDIV:
		if (ch)
			ch->vdiv = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_FACTOR:
		if (ch)
			ch->vfactor = g_variant_get_uint64(data);
		break;
	case SR_CONF_PROBE_COUPLING:
		if (ch)
			ch->coupling = g_variant_get_byte(data);
		break;
	case SR_CONF_PROBE_OFFSET:
		if (ch)
			ch->offset = g_variant_get_uint16(data);
		break;
	case SR_CONF_PROBE_HW_OFFSET:
		if (ch)
			ch->hw_offset = g_variant_get_uint16(data);
		break;
	case SR_CONF_PROBE_MAP_DEFAULT:
		if (ch)
			ch->map_default = g_variant_get_boolean(data);
		break;
	case SR_CONF_PROBE_MAP_UNIT:
		if (ch)
			ch->map_unit = g_variant_get_string(data, NULL);
		break;
	case SR_CONF_PROBE_MAP_MIN:
		if (ch)
			ch->map_min = g_variant_get_double(data);
		break;
	case SR_CONF_PROBE_MAP_MAX:
		if (ch)
			ch->map_max = g_variant_get_double(data);
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
		/* Hardware comparator range ≈ 1.2 V … 3.3 V (inverted DAC) */
		ch32_set_vth(devc, v);
		if (sdi->status == SR_ST_ACTIVE)
			ch32_apply_level(devc);
		else
			sr_info("VTH stored: %.2f V → DAC %u (apply on open/start)",
				devc->vth, (unsigned)devc->logic_level);
		break;
	}
	case SR_CONF_FILTER: {
		int nv = g_variant_get_int16(data);
		if (nv != SR_FILTER_NONE && nv != SR_FILTER_1T)
			return SR_ERR;
		devc->filter = nv;
		sr_info("filter -> %d", devc->filter);
		break;
	}
	case SR_CONF_CHANNEL_MODE: {
		int nv = g_variant_get_int16(data);
		if (nv != CH32_CHMODE_8 && nv != CH32_CHMODE_16)
			return SR_ERR;
		devc->ch_mode = nv;
		ch32_apply_channel_mode(sdi);
		/* USB2 max rate depends on 8/16-bit width */
		ch32_map_samplerate(devc, devc->samplerate);
		if (sdi->status == SR_ST_ACTIVE)
			ch32_apply_logic_params(devc);
		sr_info("channel mode -> %s (%d ch)",
			nv == CH32_CHMODE_8 ? "8" : "16",
			devc->channel_count);
		break;
	}
	case SR_CONF_OPERATION_MODE: {
		int nv = g_variant_get_int16(data);
		/* Hardware is always stream; only accept STREAM */
		if (nv != LO_OP_STREAM)
			return SR_ERR;
		devc->op_mode = LO_OP_STREAM;
		break;
	}
	case SR_CONF_LOOP_MODE:
		/* Rolling capture: keep USB stream past sample-limit */
		devc->is_loop = g_variant_get_boolean(data) ? 1 : 0;
		sr_info("loop/rolling mode -> %d", devc->is_loop);
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

	(void)sdi;
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
		static uint64_t rate_buf[128];
		int rate_n = 0;
		const struct ch32_context *devc =
			(sdi && sdi->priv) ? (const struct ch32_context *)sdi->priv
					   : NULL;

		if (devc && sdi->mode == ANALOG)
			ch32_build_adc_rate_list(rate_buf, &rate_n);
		else if (devc)
			ch32_build_rate_list(devc, rate_buf, &rate_n);
		else {
			memcpy(rate_buf, ch32_samplerate_list,
			       (size_t)ch32_samplerate_count * sizeof(uint64_t));
			rate_n = ch32_samplerate_count;
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
	case SR_CONF_PROBE_CONFIGS:
		*data = g_variant_new_from_data(
			G_VARIANT_TYPE("ai"), ch32_probe_options,
			ARRAY_SIZE(ch32_probe_options) * sizeof(int32_t),
			TRUE, NULL, NULL);
		break;
	case SR_CONF_PROBE_VDIV:
		*data = g_variant_new_from_data(
			G_VARIANT_TYPE("at"), ch32_vdivs,
			(ARRAY_SIZE(ch32_vdivs) - 1) * sizeof(uint64_t),
			TRUE, NULL, NULL);
		break;
	case SR_CONF_PROBE_COUPLING:
		*data = g_variant_new_strv(ch32_coupling, 2);
		break;
	case SR_CONF_PROBE_MAP_UNIT:
		*data = g_variant_new_strv(ch32_map_units, 1);
		break;
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
	struct ch32_context *devc = sdi->priv;
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
	devc->status = CH32_ST_START;
	devc->freewheel = 0;
	devc->raw_pending_len = 0;
	devc->decimate_phase = 0;
	devc->filt_have_prev = 0;
	devc->stream_res_len = 0;
	devc->decim_win_fill = 0;
	devc->acq_analog = (sdi->mode == ANALOG) ? 1 : 0;
	/* residual half-sample only used on analog EP3 path */

	if (devc->acq_analog) {
		ch32_sync_adc_channel_from_probes(sdi);
		{
			GSList *l;
			int seen = 0;
			for (l = sdi->channels; l; l = l->next) {
				struct sr_channel *ch = l->data;
				if (ch->type != SR_CHANNEL_ANALOG)
					continue;
				if (ch->enabled) {
					if (seen)
						ch->enabled = FALSE;
					else {
						seen = 1;
						devc->adc_channel =
							(ch->index == 1) ? ADC_CH1 : ADC_CH0;
					}
				}
			}
			if (!seen && sdi->channels) {
				struct sr_channel *ch = sdi->channels->data;
				ch->enabled = TRUE;
				devc->adc_channel = ADC_CH0;
			}
		}
		sr_info("acq start ANALOG: ch=%u bit=%u div=%u rate=%" PRIu64
			" loop=%d limit=%" PRIu64,
			devc->adc_channel, devc->adc_bit, devc->adc_div,
			devc->samplerate, devc->is_loop, devc->limit_samples);
		if ((ret = ch32_apply_adc_params(devc)) != SR_OK)
			return ret;
		if ((ret = ch32_apply_adc_channel(devc)) != SR_OK)
			return ret;
	} else {
		ch32_apply_channel_mode(sdi);
		ch32_refresh_enabled_bits(devc);
		sr_info("acq start LOGIC: mode=%s logic_bit=%u en=%d ui_rate=%" PRIu64
			" hw_rate=%" PRIu64 " decim=%d filter=%d loop=%d limit=%" PRIu64,
			devc->ch_mode == CH32_CHMODE_8 ? "8ch" : "16ch",
			devc->logic_bit, devc->en_count,
			devc->samplerate, devc->hw_samplerate,
			devc->decimate_n, devc->filter,
			devc->is_loop, devc->limit_samples);
		if ((ret = ch32_apply_logic_params(devc)) != SR_OK)
			return ret;
		if ((ret = ch32_apply_level(devc)) != SR_OK)
			return ret;
	}

	if ((ret = start_transfers(devc)) != SR_OK) {
		sr_err("start_transfers failed");
		return ret;
	}

	/* event pump: freewheel on Windows (no pollfds) */
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

	if (devc->acq_analog)
		ret = ch32_cmd(devc, CMD_SET_ADC_START, NULL, 0, NULL, NULL);
	else
		ret = ch32_cmd(devc, CMD_SET_START, NULL, 0, NULL, NULL);
	if (ret != SR_OK) {
		devc->abort = 1;
		devc->fw_streaming = 0;
		return ret;
	}
	/* FW arms capture LED (usb_trans_flag) on START */
	devc->fw_streaming = 1;

	std_session_send_df_header(sdi, LOG_PREFIX);
	return SR_OK;
}

static int hw_dev_acquisition_stop(const struct sr_dev_inst *sdi, void *cb_data)
{
	struct ch32_context *devc;

	(void)cb_data;
	if (!sdi || !sdi->priv)
		return SR_ERR;

	devc = sdi->priv;
	/* Main-thread path: stop device immediately so LED goes out */
	ch32_hw_stop(devc);
	if (!devc->abort) {
		devc->abort = 1;
		devc->status = CH32_ST_STOP;
		ch32_cancel_transfers(devc);
		if (devc->submitted_transfers <= 0)
			finish_acquisition(devc);
	}
	return SR_OK;
}

SR_PRIV struct sr_dev_driver ch32_driver_info = {
	.name = "ch32-logic",
	.longname = "WCH CH32H417 Logic Analyzer / HSADC",
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
