/* drivers/usb/gadget/f_diag.c
 * Diag Function Device - Route ARM9 and ARM11 DIAG messages
 * between HOST and DEVICE.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/usbdiag.h>
#include <mach/rpc_hsusb.h>

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>

#if defined(CONFIG_MACH_MECHA)
/*#include <mach/smsc251x.h>*/
#endif
/*#define HTC_DIAG_DEBUG*/
#include <linux/debugfs.h>
#if DIAG_XPST
#include <mach/sdio_al.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include "../../char/diag/diagchar.h"
#include "../../char/diag/diagfwd.h"
#include "../../char/diag/diagmem.h"
#include "../../char/diag/diagchar_hdlc.h"
#if defined(CONFIG_MACH_MECHA)
#include "../../../arch/arm/mach-msm/7x30-smd/sdio_diag.h"
#endif

static void fdiag_debugfs_init(void);

#define USB_DIAG_IOC_MAGIC 0xFF
#define USB_DIAG_FUNC_IOC_ENABLE_SET	_IOW(USB_DIAG_IOC_MAGIC, 1, int)
#define USB_DIAG_FUNC_IOC_ENABLE_GET	_IOR(USB_DIAG_IOC_MAGIC, 2, int)
#define USB_DIAG_FUNC_IOC_REGISTER_SET  _IOW(USB_DIAG_IOC_MAGIC, 3, char *)
#define USB_DIAG_FUNC_IOC_AMR_SET	_IOW(USB_DIAG_IOC_MAGIC, 4, int)

#define USB_DIAG_NV_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 1, uint16_t *)
#define USB_DIAG_NV_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 2, uint16_t *)
#define USB_DIAG_NV_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 3, uint16_t *)
#define USB_DIAG_NV_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 4, uint16_t *)
/*
#define USB_DIAG_RC9_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 5, uint16_t *)
#define USB_DIAG_RC9_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 6, uint16_t *)
#define USB_DIAG_RC9_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 7, uint16_t *)
#define USB_DIAG_RC9_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 8, uint16_t *)
*/
#define USB_DIAG_PRL_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 9, uint16_t *)
#define USB_DIAG_PRL_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 10, uint16_t *)
#define USB_DIAG_PRL_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 11, uint16_t *)
#define USB_DIAG_PRL_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 12, uint16_t *)
#define USB_DIAG_M29_7K9K_SET _IOW(USB_DIAG_IOC_MAGIC, 13, uint16_t *)
#define USB_DIAG_M29_7KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 14, uint16_t *)
#define USB_DIAG_M29_9KONLY_SET _IOW(USB_DIAG_IOC_MAGIC, 15, uint16_t *)
#define USB_DIAG_M29_7K9KDIFF_SET _IOW(USB_DIAG_IOC_MAGIC, 16, uint16_t *)


#define USB_DIAG_FUNC_IOC_MODEM_GET	_IOR(USB_DIAG_IOC_MAGIC, 17, int)
#define SMD_MAX 8192
#define NV_TABLE_SZ  128
#define M29_TABLE_SZ  10
#define PRL_TABLE_SZ  10

#define EPST_PREFIX 0xC8
#define HPST_PREFIX 0xF1


#define NO_PST 0
#define NO_DEF_ID 1
#define DM7K9K  2
#define DM7KONLY  3
#define DM9KONLY  4
#define DM7K9KDIFF  5
#define NO_DEF_ITEM  0xff

#define MAX(x, y) (x > y ? x : y)
#endif

#if defined(CONFIG_MACH_MECHA)
int sdio_diag_init_enable;
#endif

#if DIAG_XPST
#if defined(CONFIG_MACH_VIGOR)
static	unsigned char *diag2arm9_buf_9k;
#endif
#endif

int diag_configured;

static DEFINE_SPINLOCK(ch_lock);
static LIST_HEAD(usb_diag_ch_list);

static struct usb_interface_descriptor intf_desc = {
	.bLength            =	sizeof intf_desc,
	.bDescriptorType    =	USB_DT_INTERFACE,
	.bNumEndpoints      =	2,
	.bInterfaceClass    =	0xFF,
	.bInterfaceSubClass =	0xFF,
	.bInterfaceProtocol =	0xFF,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength 			=	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType 	=	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes 		=	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize 	=	__constant_cpu_to_le16(512),
	.bInterval 			=	0,
};
static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(512),
	.bInterval        =	0,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength          =	USB_DT_ENDPOINT_SIZE,
	.bDescriptorType  =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes     =	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
	.bInterval        =	0,
};

static struct usb_descriptor_header *fs_diag_desc[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	NULL,
	};
static struct usb_descriptor_header *hs_diag_desc[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	NULL,
};

/**
 * struct diag_context - USB diag function driver private structure
 * @function: function structure for USB interface
 * @out: USB OUT endpoint struct
 * @in: USB IN endpoint struct
 * @in_desc: USB IN endpoint descriptor struct
 * @out_desc: USB OUT endpoint descriptor struct
 * @read_pool: List of requests used for Rx (OUT ep)
 * @write_pool: List of requests used for Tx (IN ep)
 * @config_work: Work item schedule after interface is configured to notify
 *               CONNECT event to diag char driver and updating product id
 *               and serial number to MODEM/IMEM.
 * @lock: Spinlock to proctect read_pool, write_pool lists
 * @cdev: USB composite device struct
 * @ch: USB diag channel
 *
 */
struct diag_context {
	struct usb_function function;
	struct usb_ep *out;
	struct usb_ep *in;
	struct usb_endpoint_descriptor  *in_desc;
	struct usb_endpoint_descriptor  *out_desc;
	struct list_head read_pool;
	struct list_head write_pool;
	struct work_struct config_work;
	spinlock_t lock;
	unsigned configured;
	struct usb_composite_dev *cdev;
	int (*update_pid_and_serial_num)(uint32_t, const char *);
	struct usb_diag_ch ch;

	/* pkt counters */
	unsigned long dpkts_tolaptop;
	unsigned long dpkts_tomodem;
	unsigned dpkts_tolaptop_pending;
#if DIAG_XPST
	spinlock_t req_lock;

	struct mutex user_lock;
#define ID_TABLE_SZ 20 /* keep this small */
	struct list_head rx_req_idle;
	struct list_head rx_req_user;
	wait_queue_head_t read_wq;
	char *user_read_buf;
	uint32_t user_read_len;
	char *user_readp;
	bool opened;
	/* list of registered command ids to be routed to userspace */
	unsigned char id_table[ID_TABLE_SZ];

	/* smd_channel_t *ch; */
	int online;
	int error;
/* for slate test */
	struct list_head rx_arm9_idle;
	struct list_head rx_arm9_done;
	struct mutex diag2arm9_lock;
	struct mutex diag2arm9_read_lock;
	struct mutex diag2arm9_write_lock;
	bool diag2arm9_opened;
	unsigned char toARM9_buf[SMD_MAX];
	unsigned char DM_buf[USB_MAX_OUT_BUF];
	unsigned read_arm9_count;
	unsigned char *read_arm9_buf;
	wait_queue_head_t read_arm9_wq;
	struct usb_request *read_arm9_req;
	u64 tx_count; /* to smd */
	u64 rx_count; /* from smd */
	u64 usb_in_count; /* to pc */
	u64 usb_out_count; /* from pc */
	int ready;
#endif
};

#include "u_xpst.c"

static inline struct diag_context *func_to_diag(struct usb_function *f)
{
	return container_of(f, struct diag_context, function);
}

static void usb_config_work_func(struct work_struct *work)
{
	struct diag_context *ctxt = container_of(work,
			struct diag_context, config_work);
	struct usb_composite_dev *cdev = ctxt->cdev;
	struct usb_gadget_strings *table;
	struct usb_string *s;

	DIAG_INFO("%s: dev=%s\n", __func__, (ctxt == mdmctxt)?DIAG_MDM:DIAG_LEGACY);
#if DIAG_XPST
	ctxt->tx_count = ctxt->rx_count = 0;
	ctxt->usb_in_count = ctxt->usb_out_count = 0;
	driver->diag_smd_count = driver->diag_qdsp_count = 0;
#endif
	if (ctxt->ch.notify && ctxt == legacyctxt)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_CONNECT, NULL);

	if (!ctxt->update_pid_and_serial_num)
		return;

	/* pass on product id and serial number to dload */
	if (!cdev->desc.iSerialNumber) {
		ctxt->update_pid_and_serial_num(
					cdev->desc.idProduct, 0);
		return;
	}

	/*
	 * Serial number is filled by the composite driver. So
	 * it is fair enough to assume that it will always be
	 * found at first table of strings.
	 */
	table = *(cdev->driver->strings);
	for (s = table->strings; s && s->s; s++)
		if (s->id == cdev->desc.iSerialNumber) {
			ctxt->update_pid_and_serial_num(
					cdev->desc.idProduct, s->s);
			break;
		}
}

static void diag_write_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct diag_context *ctxt = ep->driver_data;
	struct diag_request *d_req = req->context;
	unsigned long flags;

	ctxt->dpkts_tolaptop_pending--;

	if (!req->status) {
		if ((req->length >= ep->maxpacket) &&
				((req->length % ep->maxpacket) == 0)) {
			ctxt->dpkts_tolaptop_pending++;
			req->length = 0;
			d_req->actual = req->actual;
			d_req->status = req->status;
			/* Queue zero length packet */
			usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
			return;
		}
	}

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->write_pool);
	if (req->length != 0) {
		d_req->actual = req->actual;
		d_req->status = req->status;
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);

	if (ctxt->ch.notify)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_WRITE_DONE, d_req);
}

static void diag_read_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct diag_context *ctxt = ep->driver_data;
	struct diag_request *d_req = req->context;
	unsigned long flags;
#if DIAG_XPST
	struct usb_request *xpst_req;
	unsigned int cmd_id;
#endif

	d_req->actual = req->actual;
	d_req->status = req->status;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, &ctxt->read_pool);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	ctxt->dpkts_tomodem++;
#if DIAG_XPST
#ifdef HTC_DIAG_DEBUG
	DIAG_INFO("%s: dev=%s\n", __func__, (ctxt == mdmctxt)?DIAG_MDM:DIAG_LEGACY);
	print_hex_dump(KERN_DEBUG, "from PC: ", DUMP_PREFIX_ADDRESS, 16, 1,
			req->buf, req->actual, 1);
#endif

	cmd_id = *((unsigned short *)req->buf);

	if ((ctxt != mdmctxt) && if_route_to_userspace(ctxt, cmd_id)) {
		xpst_req = xpst_req_get(ctxt, &ctxt->rx_req_idle);
		if (xpst_req) {
			xpst_req->actual = req->actual;
			xpst_req->status = req->status;
			memcpy(xpst_req->buf, req->buf, req->actual);
			xpst_req_put(ctxt, &ctxt->rx_req_user, xpst_req);
			wake_up(&ctxt->read_wq);
			driver->nohdlc = 1;
		} else
			DIAG_INFO("%s No enough xpst_req \n", __func__);
	} else {
		driver->nohdlc = 0;
		ctxt->tx_count += req->actual;
	}
	ctxt->usb_out_count += req->actual;
#endif
	if (ctxt->ch.notify)
		ctxt->ch.notify(ctxt->ch.priv, USB_DIAG_READ_DONE, d_req);
}

/**
 * usb_diag_open() - Open a diag channel over USB
 * @name: Name of the channel
 * @priv: Private structure pointer which will be passed in notify()
 * @notify: Callback function to receive notifications
 *
 * This function iterates overs the available channels and returns
 * the channel handler if the name matches. The notify callback is called
 * for CONNECT, DISCONNECT, READ_DONE and WRITE_DONE events.
 *
 */
struct usb_diag_ch *usb_diag_open(const char *name, void *priv,
		void (*notify)(void *, unsigned, struct diag_request *))
{
	struct usb_diag_ch *ch;
	struct diag_context *ctxt = NULL;
	unsigned long flags;
	int found = 0;

	printk(KERN_DEBUG "[USB] %s: name: %s\n", __func__, name);
	spin_lock_irqsave(&ch_lock, flags);
	/* Check if we already have a channel with this name */
	list_for_each_entry(ch, &usb_diag_ch_list, list) {
		if (!strcmp(name, ch->name)) {
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&ch_lock, flags);

	if (!found) {
		/* have a static global variable already */
		if (!strcmp(name, DIAG_LEGACY)) {
			legacyctxt = ctxt = &_context;
			legacych = ch = &legacyctxt->ch;
#if DIAG_XPST
			misc_register(&htc_diag_device_fops);
			/*DMrounter*/
			misc_register(&diag2arm9_device);
			ctxt->usb_in_count = ctxt->usb_out_count = 0;
			ctxt->tx_count = ctxt->rx_count = 0;
			driver->diag_smd_count = driver->diag_qdsp_count = 0;
#endif
		}
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
		else if (!strcmp(name, DIAG_MDM)) {
			mdmctxt = ctxt = &_mdm_context;
			mdmch = ch = &ctxt->ch;
		}
#endif
		else
			return NULL;
	}

	ch->name = name;
	ch->priv = priv;
	ch->notify = notify;

	spin_lock_irqsave(&ch_lock, flags);
	list_add_tail(&ch->list, &usb_diag_ch_list);
	spin_unlock_irqrestore(&ch_lock, flags);

	DIAG_INFO("%s: ch->name:%s ctxt:%p pkts_pending:%p\n", __func__,
			ch->name, ctxt, &ctxt->dpkts_tolaptop_pending);

	return ch;
}
EXPORT_SYMBOL(usb_diag_open);

/**
 * usb_diag_close() - Close a diag channel over USB
 * @ch: Channel handler
 *
 * This function closes the diag channel.
 *
 */
void usb_diag_close(struct usb_diag_ch *ch)
{
	unsigned long flags;

	spin_lock_irqsave(&ch_lock, flags);
	ch->priv = NULL;
	ch->notify = NULL;
	/* Free-up the resources if channel is no more active */
	if (!ch->priv_usb)
		list_del(&ch->list);

	spin_unlock_irqrestore(&ch_lock, flags);
}
EXPORT_SYMBOL(usb_diag_close);

/**
 * usb_diag_free_req() - Free USB requests
 * @ch: Channel handler
 *
 * This function free read and write USB requests for the interface
 * associated with this channel.
 *
 */
void usb_diag_free_req(struct usb_diag_ch *ch)
{
	struct diag_context *ctxt = ch->priv_usb;
	struct usb_request *req;
	struct list_head *act, *tmp;

	if (!ctxt)
		return;

	list_for_each_safe(act, tmp, &ctxt->write_pool) {
		req = list_entry(act, struct usb_request, list);
		list_del(&req->list);
		usb_ep_free_request(ctxt->in, req);
	}

	list_for_each_safe(act, tmp, &ctxt->read_pool) {
		req = list_entry(act, struct usb_request, list);
		list_del(&req->list);
		usb_ep_free_request(ctxt->out, req);
	}
}
EXPORT_SYMBOL(usb_diag_free_req);

/**
 * usb_diag_alloc_req() - Allocate USB requests
 * @ch: Channel handler
 * @n_write: Number of requests for Tx
 * @n_read: Number of requests for Rx
 *
 * This function allocate read and write USB requests for the interface
 * associated with this channel. The actual buffer is not allocated.
 * The buffer is passed by diag char driver.
 *
 */
int usb_diag_alloc_req(struct usb_diag_ch *ch, int n_write, int n_read)
{
	struct diag_context *ctxt = ch->priv_usb;
	struct usb_request *req;
	int i;

	if (!ctxt)
		return -ENODEV;

	for (i = 0; i < n_write; i++) {
		req = usb_ep_alloc_request(ctxt->in, GFP_ATOMIC);
		if (!req)
			goto fail;
		req->complete = diag_write_complete;
		list_add_tail(&req->list, &ctxt->write_pool);
	}

	for (i = 0; i < n_read; i++) {
		req = usb_ep_alloc_request(ctxt->out, GFP_ATOMIC);
		if (!req)
			goto fail;
		req->complete = diag_read_complete;
		list_add_tail(&req->list, &ctxt->read_pool);
	}

	return 0;

fail:
	usb_diag_free_req(ch);
	return -ENOMEM;

}
EXPORT_SYMBOL(usb_diag_alloc_req);

/**
 * usb_diag_read() - Read data from USB diag channel
 * @ch: Channel handler
 * @d_req: Diag request struct
 *
 * Enqueue a request on OUT endpoint of the interface corresponding to this
 * channel. This function returns proper error code when interface is not
 * in configured state, no Rx requests available and ep queue is failed.
 *
 * This function operates asynchronously. READ_DONE event is notified after
 * completion of OUT request.
 *
 */
int usb_diag_read(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	struct diag_context *ctxt = ch->priv_usb;
	unsigned long flags;
	struct usb_request *req;

	if (!ctxt)
		return -ENODEV;

	spin_lock_irqsave(&ctxt->lock, flags);

	if (!ctxt->configured) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return -EIO;
	}

	if (list_empty(&ctxt->read_pool)) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: no requests available\n", __func__);
		return -EAGAIN;
	}

	req = list_first_entry(&ctxt->read_pool, struct usb_request, list);
	list_del(&req->list);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	req->buf = d_req->buf;
	req->length = d_req->length;
	req->context = d_req;
	if (usb_ep_queue(ctxt->out, req, GFP_ATOMIC)) {
		/* If error add the link to linked list again*/
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->read_pool);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: cannot queue"
				" read request\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(usb_diag_read);

/**
 * usb_diag_write() - Write data from USB diag channel
 * @ch: Channel handler
 * @d_req: Diag request struct
 *
 * Enqueue a request on IN endpoint of the interface corresponding to this
 * channel. This function returns proper error code when interface is not
 * in configured state, no Tx requests available and ep queue is failed.
 *
 * This function operates asynchronously. WRITE_DONE event is notified after
 * completion of IN request.
 *
 */
int usb_diag_write(struct usb_diag_ch *ch, struct diag_request *d_req)
{
	struct diag_context *ctxt = ch->priv_usb;
	unsigned long flags;
	struct usb_request *req = NULL;

	if (!ctxt)
		return -ENODEV;

	spin_lock_irqsave(&ctxt->lock, flags);

	if (!ctxt->configured) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return -EIO;
	}

	if (list_empty(&ctxt->write_pool)) {
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: no requests available\n", __func__);
		return -EAGAIN;
	}

	req = list_first_entry(&ctxt->write_pool, struct usb_request, list);
	list_del(&req->list);
	spin_unlock_irqrestore(&ctxt->lock, flags);

	req->buf = d_req->buf;
	req->length = d_req->length;
	req->context = d_req;
	if (usb_ep_queue(ctxt->in, req, GFP_ATOMIC)) {
		/* If error add the link to linked list again*/
		spin_lock_irqsave(&ctxt->lock, flags);
		list_add_tail(&req->list, &ctxt->write_pool);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		ERROR(ctxt->cdev, "%s: cannot queue"
				" read request\n", __func__);
		return -EIO;
	}

	ctxt->dpkts_tolaptop++;
	ctxt->dpkts_tolaptop_pending++;

	return 0;
}
EXPORT_SYMBOL(usb_diag_write);

static void diag_function_disable(struct usb_function *f)
{
	struct diag_context  *dev = func_to_diag(f);
	unsigned long flags;

	DBG(dev->cdev, "diag_function_disable\n");

	spin_lock_irqsave(&dev->lock, flags);
	dev->configured = 0;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (dev->ch.notify)
		dev->ch.notify(dev->ch.priv, USB_DIAG_DISCONNECT, NULL);

	usb_ep_disable(dev->in);
	dev->in->driver_data = NULL;

	usb_ep_disable(dev->out);
	dev->out->driver_data = NULL;

#if DIAG_XPST
	if (dev == legacyctxt) {
		dev->online = 0;
		wake_up(&dev->read_wq);
	}
#endif

}

static int diag_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct diag_context  *dev = func_to_diag(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	unsigned long flags;
	int rc = 0;
#if DIAG_XPST
	struct usb_request *req;
#endif

	dev->in_desc = ep_choose(cdev->gadget,
			(struct usb_endpoint_descriptor *)f->hs_descriptors[1],
			(struct usb_endpoint_descriptor *)f->descriptors[1]);
	dev->out_desc = ep_choose(cdev->gadget,
			(struct usb_endpoint_descriptor *)f->hs_descriptors[2],
			(struct usb_endpoint_descriptor *)f->descriptors[2]);
	dev->in->driver_data = dev;
	rc = usb_ep_enable(dev->in, dev->in_desc);
	if (rc) {
		ERROR(dev->cdev, "can't enable %s, result %d\n",
						dev->in->name, rc);
		return rc;
	}
	dev->out->driver_data = dev;
	rc = usb_ep_enable(dev->out, dev->out_desc);
	if (rc) {
		ERROR(dev->cdev, "can't enable %s, result %d\n",
						dev->out->name, rc);
		usb_ep_disable(dev->in);
		return rc;
	}
	schedule_work(&dev->config_work);

	dev->dpkts_tolaptop = 0;
	dev->dpkts_tomodem = 0;
	dev->dpkts_tolaptop_pending = 0;

	spin_lock_irqsave(&dev->lock, flags);
	dev->configured = 1;
	spin_unlock_irqrestore(&dev->lock, flags);
#if DIAG_XPST
	if (dev == legacyctxt) {
		while ((req = xpst_req_get(dev, &dev->rx_req_user)))
			xpst_req_put(dev, &dev->rx_req_idle, req);
		dev->online = 1;
		wake_up(&dev->read_wq);
	}
#endif

	return rc;
}

static void diag_function_unbind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct diag_context *ctxt = func_to_diag(f);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);

	usb_free_descriptors(f->descriptors);
	ctxt->ch.priv_usb = NULL;
}

static int diag_function_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct diag_context *ctxt = func_to_diag(f);
	struct usb_ep *ep;
	int status = -ENODEV;

	intf_desc.bInterfaceNumber =  usb_interface_id(c, f);

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_in_desc);
	if (!ep)
		goto fail;
	ctxt->in = ep;
	ep->driver_data = ctxt;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_out_desc);
	if (!ep)
		goto fail;
	ctxt->out = ep;
	ep->driver_data = ctxt;

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(fs_diag_desc);
	if (!f->descriptors)
		goto fail;

	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hs_bulk_in_desc.bEndpointAddress =
				fs_bulk_in_desc.bEndpointAddress;
		hs_bulk_out_desc.bEndpointAddress =
				fs_bulk_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(hs_diag_desc);
	}
	return 0;
fail:
	if (ctxt->out)
		ctxt->out->driver_data = NULL;
	if (ctxt->in)
		ctxt->in->driver_data = NULL;
	return status;

}

static struct usb_string diag_string_defs[] = {
	[0].s = "HTC DIAG",
	[1].s = "HTC 9K DIAG",
	{  } /* end of list */
};

static struct usb_gadget_strings diag_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		diag_string_defs,
};

static struct usb_gadget_strings *diag_strings[] = {
	&diag_string_table,
	NULL,
};

int diag_function_add(struct usb_configuration *c, const char *name,
			int (*update_pid)(uint32_t, const char *))
{
	struct diag_context *dev;
	struct usb_diag_ch *_ch;
	int found = 0, ret;

	DBG(c->cdev, "diag_function_add\n");

	list_for_each_entry(_ch, &usb_diag_ch_list, list) {
		if (!strcmp(name, _ch->name)) {
			found = 1;
			break;
		}
	}
	if (!found) {
		ERROR(c->cdev, "unable to get diag usb channel\n");
		return -ENODEV;
	}

	dev = container_of(_ch, struct diag_context, ch);
	/* claim the channel for this USB interface */
	_ch->priv_usb = dev;

	dev->update_pid_and_serial_num = update_pid;
	dev->cdev = c->cdev;
	dev->function.name = _ch->name;
	dev->function.strings = diag_strings;
	dev->function.descriptors = fs_diag_desc;
	dev->function.hs_descriptors = hs_diag_desc;
	dev->function.bind = diag_function_bind;
	dev->function.unbind = diag_function_unbind;
	dev->function.set_alt = diag_function_set_alt;
	dev->function.disable = diag_function_disable;
	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->read_pool);
	INIT_LIST_HEAD(&dev->write_pool);
	INIT_WORK(&dev->config_work, usb_config_work_func);

	if (dev == legacyctxt) {
		if (diag_string_defs[0].id == 0) {
			ret = usb_string_id(c->cdev);
			if (ret < 0)
				return ret;
			diag_string_defs[0].id = ret;
		} else
			ret = diag_string_defs[0].id;
	} else {
		if (diag_string_defs[1].id == 0) {
			ret = usb_string_id(c->cdev);
			if (ret < 0)
				return ret;
			diag_string_defs[1].id = ret;
		} else
			ret = diag_string_defs[1].id;
	}

	intf_desc.iInterface = ret;

	ret = usb_add_function(c, &dev->function);
	if (ret) {
		INFO(c->cdev, "usb_add_function failed\n");
		_ch->priv_usb = NULL;
	}

	return ret;
}


#if defined(CONFIG_DEBUG_FS)
static char debug_buffer[PAGE_SIZE];

static ssize_t debug_read_stats(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char *buf = debug_buffer;
	int temp = 0;
	struct usb_diag_ch *ch;

	list_for_each_entry(ch, &usb_diag_ch_list, list) {
		struct diag_context *ctxt;

		ctxt = ch->priv_usb;
		if (!ctxt) continue;
		temp += scnprintf(buf + temp, PAGE_SIZE - temp,
				"---Name: %s---\n"
				"endpoints: %s, %s\n"
				"dpkts_tolaptop: %lu\n"
				"dpkts_tomodem:  %lu\n"
				"pkts_tolaptop_pending: %u\n",
				ch->name,
				ctxt->in->name, ctxt->out->name,
				ctxt->dpkts_tolaptop,
				ctxt->dpkts_tomodem,
				ctxt->dpkts_tolaptop_pending);
	}

	return simple_read_from_buffer(ubuf, count, ppos, buf, temp);
}

static ssize_t debug_reset_stats(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct usb_diag_ch *ch;

	list_for_each_entry(ch, &usb_diag_ch_list, list) {
		struct diag_context *ctxt;

		ctxt = ch->priv_usb;

		ctxt->dpkts_tolaptop = 0;
		ctxt->dpkts_tomodem = 0;
		ctxt->dpkts_tolaptop_pending = 0;
	}

	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations debug_fdiag_ops = {
	.open = debug_open,
	.read = debug_read_stats,
	.write = debug_reset_stats,
};

struct dentry *dent_diag;
static void fdiag_debugfs_init(void)
{
	dent_diag = debugfs_create_dir("usb_diag", 0);
	if (IS_ERR(dent_diag))
		return;

	debugfs_create_file("status", 0444, dent_diag, 0, &debug_fdiag_ops);
}
#else
static void fdiag_debugfs_init(void)
{
	return;
}
#endif

static void diag_cleanup(void)
{
	struct diag_context *dev;
	struct list_head *act, *tmp;
	struct usb_diag_ch *_ch;
	unsigned long flags;

	debugfs_remove_recursive(dent_diag);

	list_for_each_safe(act, tmp, &usb_diag_ch_list) {
		_ch = list_entry(act, struct usb_diag_ch, list);
		dev = container_of(_ch, struct diag_context, ch);

		spin_lock_irqsave(&ch_lock, flags);
		/* Free if diagchar is not using the channel anymore */
		if (!_ch->priv)
			list_del(&_ch->list);
		spin_unlock_irqrestore(&ch_lock, flags);
	}
}

static int diag_setup(void)
{
#if DIAG_XPST
	struct diag_context *dev = &_context;
	dev->ready = 1;

	spin_lock_init(&dev->req_lock);
	mutex_init(&dev->user_lock);
	INIT_LIST_HEAD(&dev->rx_req_user);
	INIT_LIST_HEAD(&dev->rx_req_idle);
	init_waitqueue_head(&dev->read_wq);
	INIT_LIST_HEAD(&dev->rx_arm9_idle);
	INIT_LIST_HEAD(&dev->rx_arm9_done);
	init_waitqueue_head(&dev->read_arm9_wq);
	mutex_init(&dev->diag2arm9_lock);
	mutex_init(&dev->diag2arm9_read_lock);
	mutex_init(&dev->diag2arm9_write_lock);
#endif
	fdiag_debugfs_init();

	return 0;
}
