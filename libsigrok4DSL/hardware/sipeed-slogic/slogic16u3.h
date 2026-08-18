/*
 * Sipeed SLogic16U3 driver for DSView / libsigrok4DSL
 */
#ifndef LIBSIGROK_HARDWARE_SLOGIC16U3_H
#define LIBSIGROK_HARDWARE_SLOGIC16U3_H

#include "../../libsigrok-internal.h"

extern SR_PRIV struct sr_dev_driver slogic16u3_driver_info;

/* Refresh private USB handle after hotplug re-plug. */
SR_PRIV void slogic16u3_on_usb_reconnected(struct sr_dev_inst *sdi,
					   struct libusb_device *new_dev);

#endif
