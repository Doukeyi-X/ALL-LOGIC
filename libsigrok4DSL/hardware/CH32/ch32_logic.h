/*
 * CH32H417 Logic Analyzer driver
 */
#ifndef LIBSIGROK_HARDWARE_CH32_LOGIC_H
#define LIBSIGROK_HARDWARE_CH32_LOGIC_H

#include "../../libsigrok-internal.h"

extern SR_PRIV struct sr_dev_driver ch32_driver_info;

/* Update priv USB handle after hotplug re-plug (USB2/USB3 switch). */
SR_PRIV void ch32_on_usb_reconnected(struct sr_dev_inst *sdi,
				     struct libusb_device *new_dev);

#endif
