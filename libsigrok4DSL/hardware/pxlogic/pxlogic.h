/*
 * PXLogic USB logic analyzer driver for DSView / libsigrok4DSL
 * Protocol from PXView (haikumuse/libsigrok pxview-fork).
 */
#ifndef LIBSIGROK_HARDWARE_PXLOGIC_H
#define LIBSIGROK_HARDWARE_PXLOGIC_H

#include "../../libsigrok-internal.h"
#include "usb_ctrl.h"

#define PXLOGIC_FIRMWARE_VER     0x56900028
#define PXLOGIC_PWM_CLK          125000000
#define PXLOGIC_ATOMIC_SAMPLES   64
#define PXLOGIC_TRIG_MAX_PERCENT 90
#define PXLOGIC_SAMPLES_ALIGN    1023ULL

#define USB_INTERFACE_C  0
#define USB_INTERFACE_D  1

#define CAPS_MODE_LOGIC   (1 << 0)
#define CAPS_FEATURE_USB30 (1 << 6)
#define CAPS_FEATURE_BUF   (1 << 1)

enum PX_CHANNEL_ID {
	BUFFER_LOGIC250x32 = 0,
	BUFFER_LOGIC250x16,
	BUFFER_LOGIC500x16,
	BUFFER_LOGIC1000x8,
	STREAM_LOGIC50x32,
	STREAM_LOGIC125x16,
	STREAM_LOGIC250x8,
	STREAM_LOGIC500x4,
	STREAM_LOGIC1000x2,
	STREAM_LOGIC200x1,
	STREAM_LOGIC100x2,
	STREAM_LOGIC50x4,
	STREAM_LOGIC25x8,
	STREAM_LOGIC10x16,
	STREAM_LOGIC5x32,
	PX_CHMODE_COUNT
};

struct PX_caps {
	uint64_t mode_caps;
	uint64_t feature_caps;
	uint64_t channels;
	uint64_t hw_depth;
	uint8_t intest_channel;
	uint16_t default_channelmode;
	uint64_t default_timebase;
};

struct PX_profile {
	uint16_t vid;
	uint16_t pid;
	enum libusb_speed usb_speed;
	uint32_t logic_mode;
	const char *vendor;
	const char *model;
	const char *firmware;
	uint32_t firmware_version;
	const char *fpga_bit;
	const char *fpga_rst_bit;
	struct PX_caps dev_caps;
};

struct PX_channels {
	enum PX_CHANNEL_ID id;
	int mode;
	int type;
	gboolean stream;
	uint16_t num;
	uint64_t default_samplerate;
	uint64_t min_samplerate;
	uint64_t max_samplerate;
	const char *descr;
};

enum {
	PX_ST_IDLE = 0,
	PX_ST_DATA,
	PX_ST_STOP,
	PX_ST_FINISH,
};

struct PX_context {
	const struct PX_profile *profile;
	struct libusb_device *usb_dev;
	const struct sr_dev_inst *sdi;

	uint64_t cur_samplerate;
	uint64_t limit_samples;
	uint64_t limit_samples2Byte;
	uint64_t samples_counter;
	int ch_num;
	enum PX_CHANNEL_ID ch_mode;
	uint16_t samplerates_min_index;
	uint16_t samplerates_max_index;

	uint16_t op_mode;
	gboolean stream;
	int is_loop;
	int filter;
	gboolean clock_edge;
	double vth;
	uint16_t ext_trig_mode;
	gboolean trig_out_en;
	int max_height;
	uint64_t capture_ratio;

	uint32_t ch_en;
	uint32_t trig_zero;
	uint32_t trig_one;
	uint32_t trig_rise;
	uint32_t trig_fall;
	uint32_t trigger_pos_set;
	struct ctl_data cmd_data;

	enum libusb_speed usb_speed;
	uint32_t block_size;
	int acq_aborted;
	int stop;
	int status;
	int abort;
	int fw_streaming;

	unsigned int num_transfers;
	unsigned int submitted_transfers;
	struct libusb_transfer **transfers;
	int freewheel;

	uint32_t pwm0_en, pwm0_freq_set, pwm0_duty_set;
	uint32_t pwm1_en, pwm1_freq_set, pwm1_duty_set;
};

extern SR_PRIV struct sr_dev_driver pxlogic_driver_info;
SR_PRIV void pxlogic_on_usb_reconnected(struct sr_dev_inst *sdi,
					struct libusb_device *new_dev);

#endif
