/*
 * Projector function driver
 *
 * Copyright (C) 2010 HTC Corporation
 *
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
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <mach/msm_fb.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/htc_mode_server.h>
#ifdef DBG
#undef DBG
#endif

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_INFO x)
#endif

/*16KB*/
#define TXN_MAX 16384
#define RXN_MAX 4096

#ifdef CONFIG_ARCH_MSM7X30
#define PRJ_WIDTH 480
#define PRJ_HEIGHT 800
#endif

/* number of rx and tx requests to allocate */
#define PROJ_RX_REQ_MAX 4

#if 0
#define PROJ_TX_REQ_MAX 115 /*for resolution 1280*736*2 / 16k */
#define PROJ_TX_REQ_MAX 75 /*for resolution 1024*600*2 / 16k */
#define PROJ_TX_REQ_MAX 56 /*for 8k resolution 480*800*2 / 16k */
#endif

#define BITSPIXEL 16
#define PROJECTOR_FUNCTION_NAME "projector"

#define htc_mode_info(fmt, args...) \
	printk(KERN_INFO "[htc_mode] " pr_fmt(fmt), ## args)

static struct wake_lock prj_idle_wake_lock;
static int keypad_code[] = {KEY_WAKEUP, 0, 0, 0, KEY_HOME, KEY_MENU, KEY_BACK};
static const char shortname[] = "android_projector";
static const char cand_shortname[] = "htc_cand";
static const char htcmode_shortname[] = "htcmode";

struct projector_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	int online;
	int error;

	struct list_head tx_idle;
	struct list_head rx_idle;

	int rx_done;

	u32 bitsPixel;
	u32 framesize;
	u32 width;
	u32 height;
	u8	init_done;
	u8 enabled;
	u16 frame_count;
	u32 rx_req_count;
	u32 tx_req_count;
	struct input_dev *keypad_input;
	struct input_dev *touch_input;
	char *fbaddr;

	atomic_t cand_online;
	struct switch_dev cand_sdev;
	struct switch_dev htcmode_sdev;
	struct workqueue_struct *prj_wq;
	struct work_struct notifier_work;
	struct work_struct htcmode_notifier_work;
	u8 htcmode_notify_pending;

	/* HTC Mode Protocol Info */
	struct htcmode_protocol *htcmode_proto;
};

static struct usb_interface_descriptor projector_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xFF,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor projector_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor projector_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor projector_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor projector_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_projector_descs[] = {
	(struct usb_descriptor_header *) &projector_interface_desc,
	(struct usb_descriptor_header *) &projector_fullspeed_in_desc,
	(struct usb_descriptor_header *) &projector_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_projector_descs[] = {
	(struct usb_descriptor_header *) &projector_interface_desc,
	(struct usb_descriptor_header *) &projector_highspeed_in_desc,
	(struct usb_descriptor_header *) &projector_highspeed_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string projector_string_defs[] = {
	[0].s = "HTC PROJECTOR",
	{  } /* end of list */
};

static struct usb_gadget_strings projector_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		projector_string_defs,
};

static struct usb_gadget_strings *projector_strings[] = {
	&projector_string_table,
	NULL,
};
static struct projector_dev _projector_dev;
struct device prj_dev;

struct workqueue_struct *wq_display;
struct work_struct send_fb_work;
static int start_send_fb;


struct size {
	int w;
	int h;
};

enum {
    NOT_ON_AUTOBOT,
    DOCK_ON_AUTOBOT,
    HTC_MODE_RUNNING
};
/* the value of htc_mode_status should be one of above status */
atomic_t htc_mode_status = ATOMIC_INIT(0);

static void usb_setup_andriod_projector(struct work_struct *work);
static DECLARE_WORK(conf_usb_work, usb_setup_andriod_projector);


static void usb_setup_andriod_projector(struct work_struct *work)
{
	android_switch_htc_mode();
	htc_mode_enable(1);
}

static inline struct projector_dev *proj_func_to_dev(struct usb_function *f)
{
	return container_of(f, struct projector_dev, function);
}


static struct usb_request *projector_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void projector_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
static void proj_req_put(struct projector_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *proj_req_get(struct projector_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void projector_queue_out(struct projector_dev *ctxt)
{
	int ret;
	struct usb_request *req;

	/* if we have idle read requests, get them queued */
	while ((req = proj_req_get(ctxt, &ctxt->rx_idle))) {
		req->length = RXN_MAX;
		DBG("%s: queue %p\n", __func__, req);
		ret = usb_ep_queue(ctxt->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			DBG("projector: failed to queue out req (%d)\n", ret);
			ctxt->error = 1;
			proj_req_put(ctxt, &ctxt->rx_idle, req);
			break;
		}
	}
}
/* for mouse event type, 1 :move, 2:down, 3:up */
static void projector_send_touch_event(struct projector_dev *dev,
	int iPenType, int iX, int iY)
{
	struct input_dev *tdev = dev->touch_input;
	static int b_prePenDown = false;
	static int b_firstPenDown = true;
	static int iCal_LastX;
	static int iCal_LastY;
	static int iReportCount;

	if (iPenType != 3) {
		if (b_firstPenDown) {
			input_report_abs(tdev, ABS_X, iX);
			input_report_abs(tdev, ABS_Y, iY);
			input_report_abs(tdev, ABS_PRESSURE, 100);
			input_report_abs(tdev, ABS_TOOL_WIDTH, 1);
			input_report_key(tdev, BTN_TOUCH, 1);
			input_report_key(tdev, BTN_2, 0);
			input_sync(tdev);
			b_firstPenDown = false;
			b_prePenDown = true; /* For one pen-up only */
			printk(KERN_INFO "projector: Pen down %d, %d\n", iX, iY);
		} else {
			/* don't report the same point */
			if (iX != iCal_LastX || iY != iCal_LastY) {
				input_report_abs(tdev, ABS_X, iX);
				input_report_abs(tdev, ABS_Y, iY);
				input_report_abs(tdev, ABS_PRESSURE, 100);
				input_report_abs(tdev, ABS_TOOL_WIDTH, 1);
				input_report_key(tdev, BTN_TOUCH, 1);
				input_report_key(tdev, BTN_2, 0);
				input_sync(tdev);
				iReportCount++;
				if (iReportCount < 10)
					printk(KERN_INFO "projector: Pen move %d, %d\n", iX, iY);
			}
		}
	} else if (b_prePenDown) {
		input_report_abs(tdev, ABS_X, iX);
		input_report_abs(tdev, ABS_Y, iY);
		input_report_abs(tdev, ABS_PRESSURE, 0);
		input_report_abs(tdev, ABS_TOOL_WIDTH, 0);
		input_report_key(tdev, BTN_TOUCH, 0);
		input_report_key(tdev, BTN_2, 0);
		input_sync(tdev);
		printk(KERN_INFO "projector: Pen up %d, %d\n", iX, iY);
		b_prePenDown = false;
		b_firstPenDown = true;
		iReportCount = 0;
	}
	iCal_LastX = iX;
	iCal_LastY = iY;
}

/* key code: 4 -> home, 5-> menu, 6 -> back, 0 -> system wake */
static void projector_send_Key_event(struct projector_dev *ctxt,
	int iKeycode)
{
	struct input_dev *kdev = ctxt->keypad_input;
	printk(KERN_INFO "%s keycode %d\n", __func__, iKeycode);

	/* ics will use default Generic.kl to translate linux keycode WAKEUP
	   to android keycode POWER. by this, device will suspend/resume as
	   we press power key. Even in GB, default qwerty.kl will not do
	   anything for linux keycode WAKEUP, i think we can just drop here.
	*/
	if (iKeycode == 0)
		return;

	input_report_key(kdev, keypad_code[iKeycode], 1);
	input_sync(kdev);
	input_report_key(kdev, keypad_code[iKeycode], 0);
	input_sync(kdev);
}

#ifdef CONFIG_ARCH_MSM7X30
extern char *get_fb_addr(void);
#endif

static void send_fb(struct projector_dev *ctxt)
{

	struct usb_request *req;
	char *frame;
	int xfer;
	int count = ctxt->framesize;

#ifdef CONFIG_ARCH_MSM7X30
	frame = get_fb_addr();
#else
	if (msmfb_get_fb_area())
		frame = (ctxt->fbaddr + ctxt->framesize);
	else
		frame = ctxt->fbaddr;
#endif

	while (count > 0) {
		req = proj_req_get(ctxt, &ctxt->tx_idle);
		if (req) {
			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
			memcpy(req->buf, frame, xfer);
			if (usb_ep_queue(ctxt->ep_in, req, GFP_ATOMIC) < 0) {
				proj_req_put(ctxt, &ctxt->tx_idle, req);
				printk(KERN_WARNING "%s: failed to queue req %p\n",
					__func__, req);
				break;
			}
			count -= xfer;
			frame += xfer;
		} else {
			printk(KERN_ERR "send_fb: no req to send\n");
			break;
		}
	}
}

static void send_fb2(struct projector_dev *ctxt)
{
	struct usb_request *req;
	char *frame;
	int xfer;
	int count = ctxt->framesize;

#ifdef CONFIG_ARCH_MSM7X30
	frame = get_fb_addr();
#else
	if (msmfb_get_fb_area())
		frame = (ctxt->fbaddr + ctxt->framesize);
	else
		frame = ctxt->fbaddr;
#endif

	while (count > 0 && start_send_fb) {

		while (!(req = proj_req_get(ctxt, &ctxt->tx_idle))) {
			msleep(1);

			if (!start_send_fb)
				break;
		}

		if (req) {
			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
//			printk(KERN_ERR "%s: %p\n", __func__, frame);
			memcpy(req->buf, frame, xfer);
			if (usb_ep_queue(ctxt->ep_in, req, GFP_ATOMIC) < 0) {
				proj_req_put(ctxt, &ctxt->tx_idle, req);
				printk(KERN_WARNING "%s: failed to queue req"
					    " %p\n", __func__, req);
				break;
			}
			count -= xfer;
			frame += xfer;
		} else {
			printk(KERN_ERR "send_fb: no req to send\n");
			break;
		}
	}
}

void send_fb_do_work(struct work_struct *work)
{
	while (start_send_fb) {
		send_fb2(&_projector_dev);
		msleep(1);
	}
}



static void send_info(struct projector_dev *ctxt)
{
	struct usb_request *req;

	req = proj_req_get(ctxt, &ctxt->tx_idle);
	if (req) {
		req->length = 20;
		memcpy(req->buf, "okay", 4);
		memcpy(req->buf + 4, &ctxt->bitsPixel, 4);
		#if defined(CONFIG_MACH_PARADISE)
		if (machine_is_paradise()) {
			ctxt->framesize = 320 * 480 * 2;
			printk(KERN_INFO "send_info: framesize %d\n",
				ctxt->framesize);
		}
		#endif
		memcpy(req->buf + 8, &ctxt->framesize, 4);
		memcpy(req->buf + 12, &ctxt->width, 4);
		memcpy(req->buf + 16, &ctxt->height, 4);
		if (usb_ep_queue(ctxt->ep_in, req, GFP_ATOMIC) < 0) {
			proj_req_put(ctxt, &ctxt->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req %p\n",
				__func__, req);
		}
	} else
		printk(KERN_INFO "%s: no req to send\n", __func__);
}


static void send_server_info(struct projector_dev *ctxt)
{
	struct usb_request *req;

	req = proj_req_get(ctxt, &ctxt->tx_idle);
	if (req) {
		req->length = sizeof(struct msm_server_info);
		memcpy(req->buf, &ctxt->htcmode_proto->server_info, req->length);
		if (usb_ep_queue(ctxt->ep_in, req, GFP_ATOMIC) < 0) {
			proj_req_put(ctxt, &ctxt->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req %p\n",
				__func__, req);
		}
	} else {
		printk(KERN_INFO "%s: no req to send\n", __func__);
	}
}


struct size rotate(struct size v)
{
	struct size r;
	r.w = v.h;
	r.h = v.w;
	return r;
}

static struct size get_projection_size(struct projector_dev *ctxt, struct msm_client_info *client_info)
{
	int server_width = 0;
	int server_height = 0;
	struct size client;
	struct size server;
	struct size ret;
	int perserve_aspect_ratio = client_info->display_conf & (1 << 0);
	int server_orientation = 0;
	int client_orientation = (client_info->width > client_info->height);
	int align_w = 0;

	server_width = ctxt->width;
	server_height = ctxt->height;

	server_orientation = (server_width > server_height);

	printk(KERN_INFO "%s(): perserve_aspect_ratio= %d\n", __func__, perserve_aspect_ratio);

	client.w = client_info->width;
	client.h = client_info->height;
	server.w = server_width;
	server.h = server_height;

	if (server_orientation != client_orientation)
		client = rotate(client);

	align_w = client.h * server.w > server.h * client.w;

	if (perserve_aspect_ratio) {
		if (align_w) {
			ret.w = client.w;
			ret.h = (client.w * server.h) / server.w;
		} else {
			ret.w = (client.h * server.w) / server.h;
			ret.h = client.h;
		}
	} else {
		ret = client;
	}
	return ret;
}


static void projector_get_msmfb(struct projector_dev *ctxt)
{
    struct msm_fb_info fb_info;

	msmfb_get_var(&fb_info);

	ctxt->bitsPixel = BITSPIXEL;

#ifdef CONFIG_ARCH_MSM7X30
	/* TODO: don't use fixed values */
	ctxt->width = PRJ_WIDTH;
	ctxt->height = PRJ_HEIGHT;
	ctxt->fbaddr = get_fb_addr();
	printk(KERN_INFO "projector: width %d, height %d, fb1 %p\n",
		fb_info.xres, fb_info.yres, ctxt->fbaddr);
#else
	ctxt->width = fb_info.xres;
	ctxt->height = fb_info.yres;
	ctxt->fbaddr = fb_info.fb_addr;
	printk(KERN_INFO "projector: width %d, height %d\n",
		fb_info.xres, fb_info.yres);
#endif
	ctxt->framesize = (ctxt->width)*(ctxt->height)*2;
	printk(KERN_INFO "projector: width %d, height %d %d\n",
		   fb_info.xres, fb_info.yres, ctxt->framesize);
}

static void projector_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct projector_dev *dev = &_projector_dev;
	proj_req_put(dev, &dev->tx_idle, req);
}

static void projector_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct projector_dev *ctxt = &_projector_dev;
	unsigned char *data = req->buf;
	int mouse_data[3];
	int i;
	struct size projector_size;
	DBG("%s: status %d, %d bytes\n", __func__,
		req->status, req->actual);

	if (req->status != 0) {
		ctxt->error = 1;
		proj_req_put(ctxt, &ctxt->rx_idle, req);
		return ;
	}

	/* for mouse event type, 1 :move, 2:down, 3:up */
	mouse_data[0] = *((int *)(req->buf));

	if ((data[0] == CLIENT_INFO_MESGID) && (req->actual == sizeof(struct msm_client_info))) {
		memcpy(&ctxt->htcmode_proto->client_info, req->buf, sizeof(struct msm_client_info));

		projector_size = get_projection_size(ctxt, &ctxt->htcmode_proto->client_info);
		projector_get_msmfb(ctxt);

		ctxt->htcmode_proto->server_info.mesg_id = SERVER_INFO_MESGID;
		ctxt->htcmode_proto->server_info.width = projector_size.w;
		ctxt->htcmode_proto->server_info.height = projector_size.h;
		ctxt->htcmode_proto->server_info.pixel_format = PIXEL_FORMAT_RGB565;
		ctxt->htcmode_proto->server_info.ctrl_conf = CTRL_CONF_TOUCH_EVENT_SUPPORTED |
									  CTRL_CONF_NUM_SIMULTANEOUS_TOUCH;
		send_server_info(&_projector_dev);
	} else if (!strncmp("init", data, 4)) {
		if (!ctxt->init_done) {
			projector_get_msmfb(ctxt);
			ctxt->init_done = 1;
		}
		send_info(ctxt);
		/* system wake code */
		projector_send_Key_event(ctxt, 0);
	} else if (*data == ' ') {
		send_fb(ctxt);
		ctxt->frame_count++;
		/* 30s send system wake code */
		if (ctxt->frame_count == 30 * 30) {
			projector_send_Key_event(ctxt, 0);
			ctxt->frame_count = 0;
		}
	} else if (!strncmp("startfb", data, 7)) {
		start_send_fb = true;
		queue_work(wq_display, &send_fb_work);

		ctxt->frame_count++;

		if (atomic_inc_return(&htc_mode_status) != HTC_MODE_RUNNING)
			atomic_dec(&htc_mode_status);

		htc_mode_info("startfb current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
		queue_work(ctxt->prj_wq, &ctxt->htcmode_notifier_work);

		/* 30s send system wake code */
		if (ctxt->frame_count == 30 * 30) {
			projector_send_Key_event(ctxt, 0);
			ctxt->frame_count = 0;
		}
	} else if (!strncmp("endfb", data, 5)) {
		start_send_fb = false;
		if (atomic_dec_return(&htc_mode_status) != DOCK_ON_AUTOBOT)
			atomic_inc(&htc_mode_status);

		usb_ep_fifo_flush(ctxt->ep_in);
		htc_mode_info("endfb current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
		queue_work(ctxt->prj_wq, &ctxt->htcmode_notifier_work);
	} else if (!strncmp("startcand", data, 9)) {
		atomic_set(&ctxt->cand_online, 1);
		htc_mode_info("startcand %d\n", atomic_read(&ctxt->cand_online));

		queue_work(ctxt->prj_wq, &ctxt->notifier_work);
	} else if (!strncmp("endcand", data, 7)) {
		atomic_set(&ctxt->cand_online, 0);
		htc_mode_info("endcand %d\n", atomic_read(&ctxt->cand_online));

		queue_work(ctxt->prj_wq, &ctxt->notifier_work);
	} else if (mouse_data[0] > 0) {
		 if (mouse_data[0] < 4) {
			for (i = 0; i < 3; i++)
				mouse_data[i] = *(((int *)(req->buf))+i);
			projector_send_touch_event(ctxt,
				mouse_data[0], mouse_data[1], mouse_data[2]);
		} else {
			projector_send_Key_event(ctxt, mouse_data[0]);
			printk(KERN_INFO "projector: Key command data %02x, keycode %d\n",
				*((char *)(req->buf)), mouse_data[0]);
		}
	} else if (mouse_data[0] != 0)
		printk(KERN_ERR "projector: Unknow command data %02x, mouse %d,%d,%d\n",
			*((char *)(req->buf)), mouse_data[0], mouse_data[1], mouse_data[2]);

	proj_req_put(ctxt, &ctxt->rx_idle, req);
	projector_queue_out(ctxt);
	wake_lock_timeout(&prj_idle_wake_lock, HZ / 2);
}

static int projector_create_bulk_endpoints(struct projector_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG("projector_create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG("usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG("usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG("usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG("usb_ep_autoconfig for projector ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
	for (i = 0; i < dev->rx_req_count; i++) {
		req = projector_request_new(dev->ep_out, RXN_MAX);
		if (!req)
			goto fail;
		req->complete = projector_complete_out;
		proj_req_put(dev, &dev->rx_idle, req);
	}
	for (i = 0; i < dev->tx_req_count; i++) {
		req = projector_request_new(dev->ep_in, TXN_MAX);
		if (!req)
			goto fail;
		req->complete = projector_complete_in;
		proj_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	while ((req = proj_req_get(dev, &dev->tx_idle)))
		projector_request_free(req, dev->ep_in);
	while ((req = proj_req_get(dev, &dev->rx_idle)))
		projector_request_free(req, dev->ep_out);
	printk(KERN_ERR "projector: could not allocate requests\n");
	return -1;
}

static int
projector_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct projector_dev	*dev = proj_func_to_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG("projector_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	projector_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = projector_create_bulk_endpoints(dev, &projector_fullspeed_in_desc,
			&projector_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		projector_highspeed_in_desc.bEndpointAddress =
			projector_fullspeed_in_desc.bEndpointAddress;
		projector_highspeed_out_desc.bEndpointAddress =
			projector_fullspeed_out_desc.bEndpointAddress;
	}

	DBG("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
projector_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct projector_dev	*dev = proj_func_to_dev(f);
	struct usb_request *req;

	while ((req = proj_req_get(dev, &dev->tx_idle)))
		projector_request_free(req, dev->ep_in);
	while ((req = proj_req_get(dev, &dev->rx_idle)))
		projector_request_free(req, dev->ep_out);

	dev->online = 0;
	dev->error = 1;
	switch_dev_unregister(&dev->cand_sdev);
	switch_dev_unregister(&dev->htcmode_sdev);

	if (dev->touch_input)
		input_unregister_device(dev->touch_input);
	if (dev->keypad_input)
		input_unregister_device(dev->keypad_input);
}

static int projector_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct projector_dev	*dev = proj_func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct android_dev *adev = _android_dev;
	struct android_usb_function *af;
	int ret;

	DBG("%s intf: %d alt: %d\n", __func__, intf, alt);
	ret = usb_ep_enable(dev->ep_in,
			ep_choose(cdev->gadget,
				&projector_highspeed_in_desc,
				&projector_fullspeed_in_desc));
	if (ret)
		return ret;
	ret = usb_ep_enable(dev->ep_out,
			ep_choose(cdev->gadget,
				&projector_highspeed_out_desc,
				&projector_fullspeed_out_desc));
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}

	dev->online = 0;
	list_for_each_entry(af, &adev->enabled_functions, enabled_list) {
		if (!strcmp(af->name, f->name)) {
			dev->online = 1;
			break;
		}
	}
	projector_queue_out(dev);

	return 0;
}

static void projector_function_disable(struct usb_function *f)
{
	struct projector_dev	*dev = proj_func_to_dev(f);

	DBG("projector_function_disable\n");

	start_send_fb = false;

	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	atomic_set(&dev->cand_online, 0);
	queue_work(dev->prj_wq, &dev->notifier_work);

	VDBG(dev->cdev, "%s disabled\n", dev->function.name);
}

static int projector_touch_init(struct projector_dev *dev)
{
	int x = dev->width;
	int y = dev->height;
	int ret = 0;
	struct input_dev *tdev = dev->touch_input;

	printk(KERN_INFO "%s: x=%d y=%d\n", __func__, x, y);
	dev->touch_input  = input_allocate_device();
	if (dev->touch_input == NULL) {
		printk(KERN_ERR "%s: Failed to allocate input device\n",
			__func__);
		return -1;
	}
	tdev = dev->touch_input;
	tdev->name = "projector_input";
	set_bit(EV_SYN,    tdev->evbit);
	set_bit(EV_KEY,    tdev->evbit);
	set_bit(BTN_TOUCH, tdev->keybit);
	set_bit(BTN_2,     tdev->keybit);
	set_bit(EV_ABS,    tdev->evbit);

	if (x == 0) {
		printk(KERN_ERR "%s: x=0\n", __func__);
		#if defined(CONFIG_ARCH_QSD8X50)
		x = 480;
		#elif defined(CONFIG_MACH_PARADISE)
		if (machine_is_paradise())
			x = 240;
		else
			x = 320;
		#else
		x = 320;
		#endif
	}

	if (y == 0) {
		printk(KERN_ERR "%s: y=0\n", __func__);
		#if defined(CONFIG_ARCH_QSD8X50)
		y = 800;
		#elif defined(CONFIG_MACH_PARADISE)
		if (machine_is_paradise())
			y = 400;
		else
			y = 480;
		#else
		y = 480;
		#endif
	}
	/* Set input parameters boundary. */
	input_set_abs_params(tdev, ABS_X, 0, x, 0, 0);
	input_set_abs_params(tdev, ABS_Y, 0, y, 0, 0);
	input_set_abs_params(tdev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(tdev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	input_set_abs_params(tdev, ABS_HAT0X, 0, x, 0, 0);
	input_set_abs_params(tdev, ABS_HAT0Y, 0, y, 0, 0);

	ret = input_register_device(tdev);
	if (ret) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, tdev->name);
		input_free_device(tdev);
		return -1;
	}
	printk(KERN_INFO "%s OK \n", __func__);
	return 0;
}

static int projector_keypad_init(struct projector_dev *dev)
{
	struct input_dev *kdev;
	/* Initialize input device info */
	dev->keypad_input = input_allocate_device();
	if (dev->keypad_input == NULL) {
		printk(KERN_ERR "%s: Failed to allocate input device\n",
			__func__);
		return -1;
	}
	kdev = dev->keypad_input;
	set_bit(EV_KEY, kdev->evbit);
	set_bit(KEY_HOME, kdev->keybit);
	set_bit(KEY_MENU, kdev->keybit);
	set_bit(KEY_BACK, kdev->keybit);
	set_bit(KEY_WAKEUP, kdev->keybit);

	kdev->name = "projector-Keypad";
	kdev->phys = "input2";
	kdev->id.bustype = BUS_HOST;
	kdev->id.vendor = 0x0123;
	kdev->id.product = 0x5220 /*dummy value*/;
	kdev->id.version = 0x0100;
	kdev->keycodesize = sizeof(unsigned int);

	/* Register linux input device */
	if (input_register_device(kdev) < 0) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, kdev->name);
		input_free_device(kdev);
		return -1;
	}
	printk(KERN_INFO "%s OK \n", __func__);
	return 0;
}

/* TODO: It's the way tools to enable projector */
#if 0
static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int _enabled, ret;
	ret = strict_strtol(buf, 10, (unsigned long *)&_enabled);
	if (ret < 0) {
		printk(KERN_INFO "%s: %d\n", __func__, ret);
		return 0;
	}
	printk(KERN_INFO "projector: %d\n", _enabled);

	android_enable_function(&_projector_dev.function, _enabled);
	_projector_dev.enabled = _enabled;
	return count;
}

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	buf[0] = '0' + _projector_dev.enabled;
	buf[1] = '\n';
	return 2;

}
static DEVICE_ATTR(enable, 0664, show_enable, store_enable);
#endif


static void cand_online_notify(struct work_struct *w)
{
	struct projector_dev *dev = container_of(w,
					struct projector_dev, notifier_work);
	if (atomic_read(&dev->cand_online))
		switch_set_state(&dev->cand_sdev, 1);
	else
		switch_set_state(&dev->cand_sdev, 0);
}

static void htcmode_status_notify(struct work_struct *w)
{
	struct projector_dev *dev = container_of(w,
					struct projector_dev, htcmode_notifier_work);

	switch_set_state(&dev->htcmode_sdev, atomic_read(&htc_mode_status));
}

/*
 * 1: enable; 0: disable
 */
void htc_mode_enable(int enable)
{
	struct projector_dev *ctxt = &_projector_dev;

	htc_mode_info("%s = %d\n", __func__, enable);
	htc_mode_info("current htc_mode_status = %d\n",
		    atomic_read(&htc_mode_status));

	if (enable)
		atomic_set(&htc_mode_status, DOCK_ON_AUTOBOT);
	else
		atomic_set(&htc_mode_status, NOT_ON_AUTOBOT);

	if (ctxt->prj_wq)
		queue_work(ctxt->prj_wq, &ctxt->htcmode_notifier_work);
	else
		ctxt->htcmode_notify_pending = 1;
}

int check_htc_mode_status(void)
{
	return atomic_read(&htc_mode_status);
}

static ssize_t print_cand_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", cand_shortname);
}

static ssize_t print_cand_switch_state(struct switch_dev *cand_sdev, char *buf)
{
	struct projector_dev *dev = container_of(cand_sdev,
					struct projector_dev, cand_sdev);
	return sprintf(buf, "%s\n", (atomic_read(&dev->cand_online) ?
		    "online" : "offline"));
}

static ssize_t print_htcmode_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", htcmode_shortname);
}

static ssize_t print_htcmode_switch_state(struct switch_dev *htcmode_sdev, char *buf)
{
	return sprintf(buf, "%s\n", (atomic_read(&htc_mode_status)==HTC_MODE_RUNNING ?
		    "projecting" : (atomic_read(&htc_mode_status)==DOCK_ON_AUTOBOT ? "online" : "offline")));
}

static int projector_bind_config(struct usb_configuration *c)
{
	struct projector_dev *dev = &_projector_dev;
	struct msm_fb_info fb_info;
	int ret = 0;

	printk(KERN_INFO "projector_bind_config\n");
	ret = usb_string_id(c->cdev);
	if (ret < 0)
		return ret;
	projector_string_defs[0].id = ret;
	projector_interface_desc.iInterface = ret;

	dev->prj_wq = create_singlethread_workqueue("USB_PRJ");

	INIT_WORK(&dev->notifier_work, cand_online_notify);
	INIT_WORK(&dev->htcmode_notifier_work, htcmode_status_notify);

	dev->cand_sdev.name = cand_shortname;
	dev->cand_sdev.print_name = print_cand_switch_name;
	dev->cand_sdev.print_state = print_cand_switch_state;
	ret = switch_dev_register(&dev->cand_sdev);
	if (ret < 0) {
		printk(KERN_ERR "usb cand_sdev switch_dev_register register fail\n");
		return ret;
	}

	dev->htcmode_sdev.name = htcmode_shortname;
	dev->htcmode_sdev.print_name = print_htcmode_switch_name;
	dev->htcmode_sdev.print_state = print_htcmode_switch_state;
	ret = switch_dev_register(&dev->htcmode_sdev);
	if (ret < 0) {
		printk(KERN_ERR "usb htcmode_sdev switch_dev_register register fail\n");
		return ret;
	}

	dev->cdev = c->cdev;
	dev->function.name = "projector";
	dev->function.strings = projector_strings;
	dev->function.descriptors = fs_projector_descs;
	dev->function.hs_descriptors = hs_projector_descs;
	dev->function.bind = projector_function_bind;
	dev->function.unbind = projector_function_unbind;
	dev->function.set_alt = projector_function_set_alt;
	dev->function.disable = projector_function_disable;

	msmfb_get_var(&fb_info);
	dev->bitsPixel = BITSPIXEL;
#ifdef CONFIG_ARCH_MSM7X30
	/* TODO: don't use fixed values */
	dev->width = PRJ_WIDTH;
	dev->height = PRJ_HEIGHT;
	dev->fbaddr = get_fb_addr();
#else
	dev->width = fb_info.xres;
	dev->height = fb_info.yres;
    dev->fbaddr = fb_info.fb_addr;
#endif
	dev->rx_req_count = PROJ_RX_REQ_MAX;
	dev->tx_req_count = (dev->width * dev->height * 2 / TXN_MAX) + 1;
	printk(KERN_INFO "[USB][Projector]resolution: %u*%u"
		", rx_cnt: %u, tx_cnt:%u\n", dev->width, dev->height,
		dev->rx_req_count, dev->tx_req_count);

	if (projector_touch_init(dev) < 0)
		goto err;
	if (projector_keypad_init(dev) < 0)
		goto err;

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err;

	wq_display = create_singlethread_workqueue("projector_mode");
	if (!wq_display)
		goto err;
	INIT_WORK(&send_fb_work, send_fb_do_work);

	if (dev->htcmode_notify_pending) {
		dev->htcmode_notify_pending = 0;
		queue_work(dev->prj_wq, &dev->htcmode_notifier_work);
	}

	return 0;
err:
	switch_dev_unregister(&dev->cand_sdev);
	switch_dev_unregister(&dev->htcmode_sdev);
	printk(KERN_ERR "projector gadget driver failed to initialize\n");
	return ret;
}

static int projector_setup(struct htcmode_protocol *proto)
{
	struct projector_dev *dev = &_projector_dev;
	dev->init_done = 0;
	dev->frame_count = 0;
	dev->htcmode_notify_pending = 0;
	dev->htcmode_proto = proto;
	wake_lock_init(&prj_idle_wake_lock, WAKE_LOCK_IDLE, "prj_idle_lock");

	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->tx_idle);

	return 0;
}

static int projector_ctrlrequest(struct usb_composite_dev *cdev,
				const struct usb_ctrlrequest *ctrl)
{
	int value = -EOPNOTSUPP;
	struct projector_dev *dev = &_projector_dev;

	if (((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) &&
		(ctrl->bRequest == HTC_MODE_CONTROL_REQ)) {
		if (check_htc_mode_status() == NOT_ON_AUTOBOT)
			schedule_work(&conf_usb_work);
		dev->htcmode_proto->version = le16_to_cpu(ctrl->wValue);
		printk(KERN_INFO "HTC Mode version = 0x%04X\n", dev->htcmode_proto->version);
		value = 0;
	}

	return value;
}
