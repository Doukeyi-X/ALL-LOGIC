/*
 * Alientek ATK-Logic (DL16 etc.) — community driver for ALL LOGIC
 *
 * Protocol taken from the official host:
 *   https://github.com/alientek-openedv/atk-logic
 *
 * This is not an official Alientek product or host.
 */
#ifndef LIBSIGROK_HARDWARE_ATK_LOGIC_H
#define LIBSIGROK_HARDWARE_ATK_LOGIC_H

#include "../../libsigrok-internal.h"

extern SR_PRIV struct sr_dev_driver atk_logic_driver_info;

SR_PRIV void atk_logic_on_usb_reconnected(struct sr_dev_inst *sdi,
					  struct libusb_device *new_dev);

#endif
