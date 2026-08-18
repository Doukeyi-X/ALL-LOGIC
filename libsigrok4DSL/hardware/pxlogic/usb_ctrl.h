#ifndef LIBSIGROK_HARDWARE_PXLOGIC_USB_CTRL_H
#define LIBSIGROK_HARDWARE_PXLOGIC_USB_CTRL_H

#include <libusb.h>
#include <stdint.h>
#include "../../libsigrok-internal.h"

#define CMD_CTL_RD  0xb0

struct ctl_data {
	uint64_t sync_cur_sample;
	uint32_t trig_out_validset;
	uint32_t real_pos;
};

SR_PRIV int command_ctl_rddata(libusb_device_handle *usbdevh, struct ctl_data *data);
unsigned int usb_wr_reg(libusb_device_handle *usbdevh, unsigned int reg_addr, unsigned int reg_data);
unsigned int usb_rd_reg(libusb_device_handle *usbdevh, unsigned int reg_addr, unsigned int *reg_data);
unsigned int usb_wr_data_update(libusb_device_handle *usbdevh, unsigned int base_addr,
				int length, unsigned int mode, unsigned char *buff,
				unsigned int timeout);

#endif
