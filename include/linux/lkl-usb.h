// SPDX-License-Identifier: GPL-2.0
#ifndef _LINUX_LKL_USB_H
#define _LINUX_LKL_USB_H

struct lkl_usb_setup {
	unsigned char bRequestType;
	unsigned char bRequest;
	unsigned short wValue;
	unsigned short wIndex;
	unsigned short wLength;
} __attribute__((packed));

struct lkl_usb_host_ops {
	void *(*submit_control)(void *cookie, const struct lkl_usb_setup *setup, void *data);
	void *(*submit_transfer)(void *cookie, unsigned char ep, void *data, int len);
	int (*poll)(void *cookie, void *handle, int *result);
	void (*cancel)(void *cookie, void *handle);
	void (*release)(void *cookie, void *handle);
	int (*set_alt)(void *cookie, unsigned char iface, unsigned char alt);
	void *cookie;
	int superspeed;
};

int lkl_usb_completion_irq(void);
int lkl_usb_attach(const struct lkl_usb_host_ops *ops);
void lkl_usb_detach(void);

#endif // _LINUX_LKL_USB_H
