#include "usb_ctrl.h"
#include "../../log.h"

#undef LOG_PREFIX
#define LOG_PREFIX "pxlogic: "

SR_PRIV int command_ctl_rddata(libusb_device_handle *usbdevh, struct ctl_data *data)
{
	int ret;

	if (!usbdevh || !data)
		return SR_ERR;
	ret = libusb_control_transfer(usbdevh,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
		CMD_CTL_RD, 0x0000, 0x0000,
		(unsigned char *)data, sizeof(struct ctl_data), 3000);
	if (ret < 0)
		return SR_ERR;
	return SR_OK;
}

unsigned int usb_wr_reg(libusb_device_handle *usbdevh, unsigned int reg_addr,
			unsigned int reg_data)
{
	int rc, transferred = 0;
	unsigned int buf[4] = { 0 };

	buf[0] = 0xfefe0000;
	buf[1] = 0x08;
	buf[2] = reg_addr;
	buf[3] = reg_data;
	if (!usbdevh)
		return 1;
	rc = libusb_bulk_transfer(usbdevh, 0x01, (uint8_t *)buf, 16, &transferred, 1000);
	if (rc != 0) {
		sr_err("usb_wr_reg write 0x01 failed: %s", libusb_error_name(rc));
		libusb_clear_halt(usbdevh, 0x01);
		return 1;
	}
	rc = libusb_bulk_transfer(usbdevh, 0x81, (uint8_t *)buf, 16, &transferred, 1000);
	if (rc != 0) {
		sr_err("usb_wr_reg read 0x81 failed: %s", libusb_error_name(rc));
		libusb_clear_halt(usbdevh, 0x81);
		return 2;
	}
	if (buf[3] != 0xfefefefe)
		return 3;
	return 0;
}

unsigned int usb_rd_reg(libusb_device_handle *usbdevh, unsigned int reg_addr,
			unsigned int *reg_data)
{
	int rc, transferred = 0;
	unsigned int buf[4] = { 0 };

	buf[0] = 0xfefe0001;
	buf[1] = 0x08;
	buf[2] = reg_addr;
	buf[3] = 0;
	if (!usbdevh)
		return 3;
	rc = libusb_bulk_transfer(usbdevh, 0x01, (uint8_t *)buf, 16, &transferred, 1000);
	if (rc != 0) {
		sr_err("usb_rd_reg write 0x01 failed: %s", libusb_error_name(rc));
		libusb_clear_halt(usbdevh, 0x01);
		return (unsigned int)rc;
	}
	rc = libusb_bulk_transfer(usbdevh, 0x81, (uint8_t *)buf, 16, &transferred, 1000);
	if (rc != 0) {
		sr_err("usb_rd_reg read 0x81 failed: %s", libusb_error_name(rc));
		libusb_clear_halt(usbdevh, 0x81);
		return 2;
	}
	*reg_data = buf[3];
	return 0;
}

unsigned int usb_wr_data_update(libusb_device_handle *usbdevh, unsigned int base_addr,
				int length, unsigned int mode, unsigned char *buff,
				unsigned int timeout)
{
	unsigned int addr;
	int rc, transferred = 0;
	int align_length;

	align_length = (length % 4096) ? ((length / 4096 + 1) * 4096) : length;

	addr = 8192 + 6 * 4;
	if (usb_wr_reg(usbdevh, addr, base_addr) != 0)
		return 1;
	addr = 8192 + 7 * 4;
	if (usb_wr_reg(usbdevh, addr, base_addr + (unsigned)align_length) != 0)
		return 1;
	addr = 8192 + 8 * 4;
	if (usb_wr_reg(usbdevh, addr, mode) != 0)
		return 1;
	if (!usbdevh)
		return 1;
	rc = libusb_bulk_transfer(usbdevh, 0x03, buff, align_length, &transferred, timeout);
	if (rc != 0)
		return 1;
	return 0;
}
