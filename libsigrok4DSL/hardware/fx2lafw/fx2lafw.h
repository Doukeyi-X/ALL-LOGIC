/*
 * Cypress FX2 / fx2lafw logic analyzer — community driver for ALL LOGIC
 *
 * Covers nanoDLA (https://github.com/wuxx/nanoDLA) and other PulseView
 * fx2lafw devices that already enumerate as 1D50:608C (8ch) / 608D (16ch).
 *
 * Protocol: sigrok fx2lafw. Not an official Muse Lab host.
 * Cypress 04B4:8613 is left to DSLogic — that ID is also DS boot.
 */
#ifndef LIBSIGROK_HARDWARE_FX2LAFW_H
#define LIBSIGROK_HARDWARE_FX2LAFW_H

#include "../../libsigrok-internal.h"

extern SR_PRIV struct sr_dev_driver fx2lafw_driver_info;

SR_PRIV void fx2lafw_on_usb_reconnected(struct sr_dev_inst *sdi,
					struct libusb_device *new_dev);

#endif
