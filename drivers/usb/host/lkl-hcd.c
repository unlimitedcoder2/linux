// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/lkl-usb.h>

#define DRIVER_NAME "lkl-hcd"

#ifdef LKL_HCD_DEBUG
#define lkl_hcd_dbg(...) lkl_printf(__VA_ARGS__);
#else
#define lkl_hcd_dbg(...)
#endif

#define LKL_HCD_MAX_EP 32

static_assert(sizeof(struct lkl_usb_setup) == sizeof(struct usb_ctrlrequest),
	      "lkl_usb_setup must match the USB setup packet layout");
static_assert(offsetof(struct lkl_usb_setup, wValue) ==
	      offsetof(struct usb_ctrlrequest, wValue),
	      "lkl_usb_setup fields must line up with usb_ctrlrequest");

#define PORT_C_MASK							\
	((USB_PORT_STAT_C_CONNECTION | USB_PORT_STAT_C_ENABLE |		\
	  USB_PORT_STAT_C_SUSPEND | USB_PORT_STAT_C_OVERCURRENT |	\
	  USB_PORT_STAT_C_RESET) << 16)

struct lkl_urbp {
	struct urb *urb;
	struct list_head node;
};

struct lkl_inflight {
	struct urb *urb;
	void *handle;
	unsigned long deadline;
};

static struct lkl_hcd *lkl_hcd_singleton;
static struct platform_device *lkl_hcd_pdev;

struct lkl_hcd {
	spinlock_t lock;
	struct lkl_usb_host_ops ops;
	bool attached;
	u32 port_status;
	struct list_head urb_list;
	struct lkl_inflight inflight[LKL_HCD_MAX_EP];
	struct task_struct *worker;
	wait_queue_head_t wq;
	atomic_t completed;
	int irq;
	struct platform_device *pdev;
	struct usb_hcd *hcd;
};

static struct lkl_usb_host_ops lkl_usb_pending_ops;
static volatile int lkl_usb_attach_pending;
static volatile int lkl_usb_detach_pending;

static inline struct lkl_hcd *hcd_to_lkl(struct usb_hcd *hcd)
{
	return *((struct lkl_hcd **)hcd->hcd_priv);
}

#define LKL_HCD_XFER_TIMEOUT_MS 5000

static bool lkl_hcd_ctrl_ack_only(struct urb *urb)
{
	const struct usb_ctrlrequest *req =
		(const struct usb_ctrlrequest *)urb->setup_packet;

	return usb_pipecontrol(urb->pipe) && req &&
	       (req->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD &&
	       (req->bRequestType & USB_DIR_IN) == 0 &&
	       (req->bRequest == USB_REQ_SET_ADDRESS ||
		req->bRequest == USB_REQ_SET_CONFIGURATION ||
		req->bRequest == USB_REQ_SET_INTERFACE);
}

static void *lkl_hcd_submit_one(struct lkl_hcd *lh, struct urb *urb)
{
	unsigned int pipe = urb->pipe;
	int len = urb->transfer_buffer_length;

	if (usb_pipecontrol(pipe)) {
		const struct usb_ctrlrequest *req =
			(const struct usb_ctrlrequest *)urb->setup_packet;

		lkl_hcd_dbg("lkl-hcd: xfer ctrl submit rt=0x%02x req=0x%02x len=%d\n",
			   req ? req->bRequestType : 0, req ? req->bRequest : 0, len);
		return lh->ops.submit_control(lh->ops.cookie,
					      (const struct lkl_usb_setup *)urb->setup_packet,
					      urb->transfer_buffer);
	}

	u8 ep = usb_pipeendpoint(pipe) | (usb_pipein(pipe) ? 0x80 : 0);

	lkl_hcd_dbg("lkl-hcd: xfer bulk/int submit ep=0x%02x len=%d\n", ep, len);
	return lh->ops.submit_transfer(lh->ops.cookie, ep, urb->transfer_buffer, len);
}

static void lkl_hcd_finish_urb(struct lkl_hcd *lh, struct urb *urb, int status)
{
	unsigned long flags;

	spin_lock_irqsave(&lh->lock, flags);
	usb_hcd_unlink_urb_from_ep(lh->hcd, urb);
	spin_unlock_irqrestore(&lh->lock, flags);
	usb_hcd_giveback_urb(lh->hcd, urb, status);
}

static int lkl_urb_ep_index(struct urb *urb)
{
	unsigned int pipe = urb->pipe;
	int ep;

	if (usb_pipecontrol(pipe))
		return 0;
	ep = usb_pipeendpoint(pipe);
	if (usb_pipein(pipe))
		ep |= 0x10;
	return ep;
}

static bool lkl_hcd_has_runnable(struct lkl_hcd *lh)
{
	struct lkl_urbp *up;
	unsigned long flags;
	bool found = false;

	spin_lock_irqsave(&lh->lock, flags);
	list_for_each_entry(up, &lh->urb_list, node) {
		if (!lh->inflight[lkl_urb_ep_index(up->urb)].urb) {
			found = true;
			break;
		}
	}
	spin_unlock_irqrestore(&lh->lock, flags);
	return found;
}

static int lkl_hcd_add(struct lkl_hcd *lh, bool superspeed);

static void lkl_hcd_apply_pending(struct lkl_hcd *lh)
{
	unsigned long flags;
	bool changed = false;

	if (lkl_usb_attach_pending && !lh->hcd) {
		int ret = lkl_hcd_add(lh, lkl_usb_pending_ops.superspeed);

		if (ret) {
			lkl_hcd_dbg("lkl-hcd: could not register root hub (%d)\n", ret);
			lkl_usb_attach_pending = 0;
			return;
		}
	}

	spin_lock_irqsave(&lh->lock, flags);
	if (lkl_usb_attach_pending) {
		lh->ops = lkl_usb_pending_ops;
		lh->attached = true;
		lh->port_status |= USB_PORT_STAT_CONNECTION |
				   (USB_PORT_STAT_C_CONNECTION << 16);
		lkl_usb_attach_pending = 0;
		changed = true;
	}
	if (lkl_usb_detach_pending) {
		lh->attached = false;
		lh->port_status &= ~(USB_PORT_STAT_CONNECTION |
				     USB_PORT_STAT_ENABLE);
		lh->port_status |= (USB_PORT_STAT_C_CONNECTION << 16);
		lkl_usb_detach_pending = 0;
		changed = true;
	}
	spin_unlock_irqrestore(&lh->lock, flags);

	if (changed) {
		lkl_hcd_dbg("lkl-hcd: port change applied (attached=%d, status=0x%08x), poking root hub\n",
			   lh->attached, lh->port_status);
		if (lh->hcd)
			usb_hcd_poll_rh_status(lh->hcd);
	}
}

static irqreturn_t lkl_hcd_complete_irq(int irq, void *data)
{
	struct lkl_hcd *lh = data;

	atomic_inc(&lh->completed);
	wake_up(&lh->wq);
	return IRQ_HANDLED;
}

static void lkl_hcd_submit_runnable(struct lkl_hcd *lh)
{
	for (;;) {
		struct lkl_urbp *up = NULL, *cur;
		unsigned long flags;
		struct urb *urb;
		void *h;
		int idx = 0;

		spin_lock_irqsave(&lh->lock, flags);
		list_for_each_entry(cur, &lh->urb_list, node) {
			idx = lkl_urb_ep_index(cur->urb);
			if (!lh->inflight[idx].urb) {
				up = cur;
				break;
			}
		}
		if (up)
			list_del(&up->node);
		spin_unlock_irqrestore(&lh->lock, flags);
		if (!up)
			return;

		urb = up->urb;
		kfree(up);

		if (!lh->attached || !lh->ops.submit_control) {
			lkl_hcd_finish_urb(lh, urb, -ENODEV);
			continue;
		}
		if (lkl_hcd_ctrl_ack_only(urb)) {
			const struct usb_ctrlrequest *req =
				(const struct usb_ctrlrequest *)urb->setup_packet;
			u8 iface = le16_to_cpu(req->wIndex);
			u8 alt = le16_to_cpu(req->wValue);
			int status = 0;

			if (req->bRequest == USB_REQ_SET_INTERFACE && lh->ops.set_alt) {
				status = lh->ops.set_alt(lh->ops.cookie, iface, alt);
				lkl_hcd_dbg("lkl-hcd: set_alt iface=%u alt=%u -> %d\n",
					   iface, alt, status);
			} else {
				lkl_hcd_dbg("lkl-hcd: xfer ctrl std req 0x%02x ACKed (no forward)\n",
					   req->bRequest);
			}
			urb->actual_length = 0;
			lkl_hcd_finish_urb(lh, urb, status);
			continue;
		}
		h = lkl_hcd_submit_one(lh, urb);
		if (!h) {
			lkl_hcd_dbg("lkl-hcd: xfer submit failed (host returned NULL)\n");
			lkl_hcd_finish_urb(lh, urb, -ENOMEM);
			continue;
		}
		lh->inflight[idx].urb = urb;
		lh->inflight[idx].handle = h;
		lh->inflight[idx].deadline =
			jiffies + msecs_to_jiffies(LKL_HCD_XFER_TIMEOUT_MS);
	}
}

static void lkl_hcd_poll_inflight(struct lkl_hcd *lh)
{
	int idx;

	for (idx = 0; idx < LKL_HCD_MAX_EP; idx++) {
		struct urb *urb = lh->inflight[idx].urb;
		void *h = lh->inflight[idx].handle;
		int result;

		if (!urb)
			continue;

		if (lh->ops.poll(lh->ops.cookie, h, &result)) {
			lh->ops.release(lh->ops.cookie, h);
			lh->inflight[idx].urb = NULL;
			if (result < 0) {
				lkl_hcd_dbg("lkl-hcd: xfer complete pipe=0x%x ERROR result=%d\n",
					   urb->pipe, result);
			} else {
				lkl_hcd_dbg("lkl-hcd: xfer complete pipe=0x%x actual_len=%d\n",
					   urb->pipe, result);
				urb->actual_length = result;
			}
			lkl_hcd_finish_urb(lh, urb, result < 0 ? result : 0);
			continue;
		}

		bool timed_out = !usb_pipein(urb->pipe) &&
				 time_after(jiffies, lh->inflight[idx].deadline);

		if (urb->unlinked || timed_out || kthread_should_stop()) {
			lkl_hcd_dbg("lkl-hcd: xfer cancel (pipe=0x%x unlinked=%d timeout=%d)\n",
				   urb->pipe, urb->unlinked, timed_out);
			lh->ops.cancel(lh->ops.cookie, h);
			while (!lh->ops.poll(lh->ops.cookie, h, &result))
				wait_event_timeout(lh->wq,
					lh->ops.poll(lh->ops.cookie, h, &result),
					msecs_to_jiffies(100));
			lh->ops.release(lh->ops.cookie, h);
			lh->inflight[idx].urb = NULL;
			lkl_hcd_finish_urb(lh, urb,
					   urb->unlinked ? urb->unlinked : -ETIMEDOUT);
		}
	}
}

static void lkl_hcd_drain_inflight(struct lkl_hcd *lh, int status)
{
	int idx;

	for (idx = 0; idx < LKL_HCD_MAX_EP; idx++) {
		struct urb *urb = lh->inflight[idx].urb;
		void *h = lh->inflight[idx].handle;
		int result;

		if (!urb)
			continue;
		lh->ops.cancel(lh->ops.cookie, h);
		while (!lh->ops.poll(lh->ops.cookie, h, &result))
			wait_event_timeout(lh->wq,
				lh->ops.poll(lh->ops.cookie, h, &result),
				msecs_to_jiffies(100));
		lh->ops.release(lh->ops.cookie, h);
		lh->inflight[idx].urb = NULL;
		lkl_hcd_finish_urb(lh, urb, status);
	}
}

static int lkl_hcd_worker(void *data)
{
	struct lkl_hcd *lh = data;

	while (!kthread_should_stop()) {
		int seen = atomic_read(&lh->completed);

		lkl_hcd_apply_pending(lh);
		lkl_hcd_submit_runnable(lh);
		lkl_hcd_poll_inflight(lh);

		wait_event_timeout(lh->wq,
			kthread_should_stop() ||
			atomic_read(&lh->completed) != seen ||
			lkl_hcd_has_runnable(lh) ||
			lkl_usb_attach_pending || lkl_usb_detach_pending,
			msecs_to_jiffies(50));
	}
	lkl_hcd_drain_inflight(lh, -ESHUTDOWN);
	return 0;
}

static int lkl_hcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
			       gfp_t mem_flags)
{
	struct lkl_hcd *lh = hcd_to_lkl(hcd);
	struct lkl_urbp *up;
	unsigned long flags;
	int rc;

	lkl_hcd_dbg("lkl-hcd: enqueue pipe=0x%x (ep=%d %s) len=%d attached=%d\n",
		   urb->pipe, usb_pipeendpoint(urb->pipe),
		   usb_pipein(urb->pipe) ? "IN" : "OUT",
		   urb->transfer_buffer_length, lh->attached);

	up = kmalloc(sizeof(*up), mem_flags);
	if (!up)
		return -ENOMEM;
	up->urb = urb;

	spin_lock_irqsave(&lh->lock, flags);
	rc = usb_hcd_link_urb_to_ep(hcd, urb);
	if (rc) {
		spin_unlock_irqrestore(&lh->lock, flags);
		kfree(up);
		return rc;
	}
	list_add_tail(&up->node, &lh->urb_list);
	spin_unlock_irqrestore(&lh->lock, flags);

	wake_up(&lh->wq);
	return 0;
}

static int lkl_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct lkl_hcd *lh = hcd_to_lkl(hcd);
	struct lkl_urbp *up, *tmp;
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&lh->lock, flags);
	rc = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (!rc) {
		list_for_each_entry_safe(up, tmp, &lh->urb_list, node) {
			if (up->urb != urb)
				continue;
			list_del(&up->node);
			usb_hcd_unlink_urb_from_ep(hcd, urb);
			spin_unlock_irqrestore(&lh->lock, flags);
			usb_hcd_giveback_urb(hcd, urb, status);
			kfree(up);
			return 0;
		}
	}
	spin_unlock_irqrestore(&lh->lock, flags);
	return rc;
}

static int lkl_hcd_get_frame(struct usb_hcd *hcd)
{
	return 0;
}

static int lkl_hcd_hub_status(struct usb_hcd *hcd, char *buf)
{
	struct lkl_hcd *lh = hcd_to_lkl(hcd);
	unsigned long flags;
	int ret = 0;

	if (!HCD_HW_ACCESSIBLE(hcd))
		return 0;

	spin_lock_irqsave(&lh->lock, flags);
	if (lh->port_status & PORT_C_MASK) {
		*buf = (1 << 1);
		ret = 1;
	}
	spin_unlock_irqrestore(&lh->lock, flags);
	return ret;
}

static void lkl_hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
	desc->bDescriptorType = USB_DT_HUB;
	desc->bDescLength = 9;
	desc->wHubCharacteristics =
		cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_COMMON_OCPM);
	desc->bNbrPorts = 1;
	desc->u.hs.DeviceRemovable[0] = 0;
	desc->u.hs.DeviceRemovable[1] = 0xff;
}

static void lkl_ss_hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof(*desc));
	desc->bDescriptorType = USB_DT_SS_HUB;
	desc->bDescLength = 12;
	desc->wHubCharacteristics =
		cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_COMMON_OCPM);
	desc->bNbrPorts = 1;
	desc->u.ss.bHubHdrDecLat = 0x04;
	desc->u.ss.DeviceRemovable = 0;
}

static struct {
	struct usb_bos_descriptor bos;
	struct usb_ss_cap_descriptor ss_cap;
} __packed lkl_usb3_bos_desc = {
	.bos = {
		.bLength		= USB_DT_BOS_SIZE,
		.bDescriptorType	= USB_DT_BOS,
		.wTotalLength		= cpu_to_le16(sizeof(lkl_usb3_bos_desc)),
		.bNumDeviceCaps		= 1,
	},
	.ss_cap = {
		.bLength		= USB_DT_USB_SS_CAP_SIZE,
		.bDescriptorType	= USB_DT_DEVICE_CAPABILITY,
		.bDevCapabilityType	= USB_SS_CAP_TYPE,
		.wSpeedSupported	= cpu_to_le16(USB_5GBPS_OPERATION),
		.bFunctionalitySupport	= ilog2(USB_5GBPS_OPERATION),
	},
};

static int lkl_hcd_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
			       u16 wIndex, char *buf, u16 wLength)
{
	struct lkl_hcd *lh = hcd_to_lkl(hcd);
	unsigned long flags;
	int ret = 0;

	if (!HCD_HW_ACCESSIBLE(hcd))
		return -ETIMEDOUT;

	spin_lock_irqsave(&lh->lock, flags);
	switch (typeReq) {
	case ClearHubFeature:
		break;
	case ClearPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			lh->port_status &= ~USB_PORT_STAT_SUSPEND;
			break;
		case USB_PORT_FEAT_ENABLE:
			lh->port_status &= ~USB_PORT_STAT_ENABLE;
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			lh->port_status &= ~(USB_PORT_STAT_C_CONNECTION << 16);
			break;
		case USB_PORT_FEAT_C_ENABLE:
			lh->port_status &= ~(USB_PORT_STAT_C_ENABLE << 16);
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			lh->port_status &= ~(USB_PORT_STAT_C_SUSPEND << 16);
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			lh->port_status &= ~(USB_PORT_STAT_C_OVERCURRENT << 16);
			break;
		case USB_PORT_FEAT_C_RESET:
			lh->port_status &= ~(USB_PORT_STAT_C_RESET << 16);
			break;
		case USB_PORT_FEAT_POWER:
			lh->port_status &= hcd->speed == HCD_USB3 ?
				~USB_SS_PORT_STAT_POWER : ~USB_PORT_STAT_POWER;
			break;
		case USB_PORT_FEAT_C_BH_PORT_RESET:
			lh->port_status &= ~(USB_PORT_STAT_C_BH_RESET << 16);
			break;
		default:
			break;
		}
		break;
	case GetHubDescriptor:
		if (hcd->speed == HCD_USB3) {
			if (wLength < USB_DT_SS_HUB_SIZE ||
			    wValue != (USB_DT_SS_HUB << 8)) {
				ret = -EPIPE;
				break;
			}
			lkl_ss_hub_descriptor((struct usb_hub_descriptor *)buf);
		} else {
			lkl_hub_descriptor((struct usb_hub_descriptor *)buf);
		}
		break;
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		if (hcd->speed != HCD_USB3 || (wValue >> 8) != USB_DT_BOS) {
			ret = -EPIPE;
			break;
		}
		memcpy(buf, &lkl_usb3_bos_desc, sizeof(lkl_usb3_bos_desc));
		ret = sizeof(lkl_usb3_bos_desc);
		break;
	case GetHubStatus:
		*(__le32 *)buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			ret = -EPIPE;
		*(__le32 *)buf = cpu_to_le32(lh->port_status);
		break;
	case SetHubFeature:
		ret = -EPIPE;
		break;
	case SetPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			if (hcd->speed == HCD_USB3) {
				ret = -EPIPE;
				break;
			}
			lh->port_status |= USB_PORT_STAT_SUSPEND;
			break;
		case USB_PORT_FEAT_POWER:
			lh->port_status |= hcd->speed == HCD_USB3 ?
				USB_SS_PORT_STAT_POWER : USB_PORT_STAT_POWER;
			break;
		case USB_PORT_FEAT_LINK_STATE:
		case USB_PORT_FEAT_U1_TIMEOUT:
		case USB_PORT_FEAT_U2_TIMEOUT:
			if (hcd->speed != HCD_USB3)
				ret = -EPIPE;
			break;
		case USB_PORT_FEAT_BH_PORT_RESET:
		case USB_PORT_FEAT_RESET:
			lh->port_status &= ~(USB_PORT_STAT_RESET |
					     USB_PORT_STAT_LOW_SPEED |
					     USB_PORT_STAT_HIGH_SPEED);
			if (hcd->speed == HCD_USB3) {
				lh->port_status &= ~USB_PORT_STAT_LINK_STATE;
				lh->port_status |= USB_SS_PORT_LS_U0 |
						   USB_PORT_STAT_ENABLE |
						   (USB_PORT_STAT_C_RESET << 16);
			} else {
				lh->port_status |= USB_PORT_STAT_ENABLE |
						   USB_PORT_STAT_HIGH_SPEED |
						   (USB_PORT_STAT_C_RESET << 16);
			}
			lkl_hcd_dbg("lkl-hcd: port RESET -> enabled (status=0x%08x)\n",
				   lh->port_status);
			break;
		default:
			break;
		}
		break;
	default:
		ret = -EPIPE;
		break;
	}
	spin_unlock_irqrestore(&lh->lock, flags);
	return ret;
}

static int lkl_hcd_start(struct usb_hcd *hcd)
{
	lkl_hcd_dbg("lkl-hcd start\n");
	hcd->uses_new_polling = 1;
	hcd->state = HC_STATE_RUNNING;
	return 0;
}

static void lkl_hcd_stop(struct usb_hcd *hcd)
{
	lkl_hcd_dbg("lkl-hcd stop\n");
	hcd->state = HC_STATE_HALT;
}

static struct hc_driver lkl_hc_driver = {
	.description = DRIVER_NAME,
	.product_desc = "LKL host-backed USB controller",
	.hcd_priv_size = sizeof(struct lkl_hcd *),
	.flags = HCD_USB2,
	.start = lkl_hcd_start,
	.stop = lkl_hcd_stop,
	.urb_enqueue = lkl_hcd_urb_enqueue,
	.urb_dequeue = lkl_hcd_urb_dequeue,
	.get_frame_number = lkl_hcd_get_frame,
	.hub_status_data = lkl_hcd_hub_status,
	.hub_control = lkl_hcd_hub_control,
};

static int lkl_hcd_add(struct lkl_hcd *lh, bool superspeed)
{
	struct usb_hcd *hcd;
	int ret;

	lkl_hc_driver.flags = superspeed ? HCD_USB3 : HCD_USB2;

	hcd = usb_create_hcd(&lkl_hc_driver, &lh->pdev->dev,
			     dev_name(&lh->pdev->dev));
	if (!hcd)
		return -ENOMEM;

	*((struct lkl_hcd **)hcd->hcd_priv) = lh;
	lh->hcd = hcd;

	ret = usb_add_hcd(hcd, 0, 0);
	if (ret) {
		lh->hcd = NULL;
		usb_put_hcd(hcd);
		return ret;
	}

	lkl_hcd_dbg("lkl-hcd: root hub registered as %s\n",
		   superspeed ? "SuperSpeed" : "high speed");
	return 0;
}

int lkl_usb_attach(const struct lkl_usb_host_ops *ops)
{
	if (!lkl_hcd_singleton) {
		lkl_hcd_dbg("lkl-hcd: attach rejected -- no HCD singleton (probe ran?)\n");
		return -ENODEV;
	}
	if (!ops || !ops->submit_control || !ops->submit_transfer ||
	    !ops->poll || !ops->cancel || !ops->release) {
		lkl_hcd_dbg("lkl-hcd: attach rejected -- incomplete ops\n");
		return -EINVAL;
	}

	lkl_usb_pending_ops = *ops;
	lkl_usb_attach_pending = 1;
	lkl_hcd_dbg("lkl-hcd: attach requested (worker will report connect)\n");
	return 0;
}
EXPORT_SYMBOL_GPL(lkl_usb_attach);

void lkl_usb_detach(void)
{
	lkl_hcd_dbg("lkl-hcd: detach requested\n");
	lkl_usb_detach_pending = 1;
}
EXPORT_SYMBOL_GPL(lkl_usb_detach);

static int lkl_hcd_probe(struct platform_device *pdev)
{
	struct lkl_hcd *lh;
	int ret;

	lh = kzalloc(sizeof(*lh), GFP_KERNEL);
	if (!lh)
		return -ENOMEM;

	spin_lock_init(&lh->lock);
	INIT_LIST_HEAD(&lh->urb_list);
	init_waitqueue_head(&lh->wq);

	lkl_hcd_dbg("Probe!\n");

	lh->irq = lkl_get_free_irq("lkl-hcd");
	if (lh->irq < 0) {
		ret = lh->irq;
		goto err_free;
	}
	ret = request_irq(lh->irq, lkl_hcd_complete_irq, 0, "lkl-hcd", lh);
	if (ret) {
		lkl_put_irq(lh->irq, "lkl-hcd");
		goto err_free;
	}

	lh->pdev = pdev;

	lkl_hcd_dbg("lkl-hcd starting reactor\n");
	lh->worker = kthread_run(lkl_hcd_worker, lh, "lkl-hcd");
	if (IS_ERR(lh->worker)) {
		ret = PTR_ERR(lh->worker);
		lh->worker = NULL;
		goto err_irq;
	}

	lkl_hcd_singleton = lh;
	platform_set_drvdata(pdev, lh);
	return 0;

err_irq:
	free_irq(lh->irq, lh);
	lkl_put_irq(lh->irq, "lkl-hcd");
err_free:
	kfree(lh);
	return ret;
}

int lkl_usb_completion_irq(void)
{
	return lkl_hcd_singleton ? lkl_hcd_singleton->irq : -1;
}
EXPORT_SYMBOL_GPL(lkl_usb_completion_irq);

static void lkl_hcd_remove(struct platform_device *pdev)
{
	struct lkl_hcd *lh = platform_get_drvdata(pdev);

	lkl_hcd_singleton = NULL;
	if (lh->hcd)
		usb_remove_hcd(lh->hcd);
	kthread_stop(lh->worker);
	free_irq(lh->irq, lh);
	lkl_put_irq(lh->irq, "lkl-hcd");
	if (lh->hcd)
		usb_put_hcd(lh->hcd);
	kfree(lh);
}

static struct platform_driver lkl_hcd_driver = {
	.probe = lkl_hcd_probe,
	.remove = lkl_hcd_remove,
	.driver = {
		.name = DRIVER_NAME,
	},
};

static int __init lkl_hcd_init(void)
{
	lkl_hcd_dbg("lkl-hcd init\n");

	int ret;

	ret = platform_driver_register(&lkl_hcd_driver);
	if (ret)
		return ret;
	lkl_hcd_dbg("lkl-hcd init2\n");

	lkl_hcd_pdev = platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);
	if (IS_ERR(lkl_hcd_pdev)) {
		platform_driver_unregister(&lkl_hcd_driver);
		return PTR_ERR(lkl_hcd_pdev);
	}
	lkl_hcd_dbg("lkl-hcd init3\n");

	return 0;
}

static void __exit lkl_hcd_exit(void)
{
	platform_device_unregister(lkl_hcd_pdev);
	platform_driver_unregister(&lkl_hcd_driver);
}

module_init(lkl_hcd_init);
module_exit(lkl_hcd_exit);

MODULE_DESCRIPTION("Host-backed USB host controller for LKL");
MODULE_LICENSE("GPL");
