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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <mach/msm_fb.h>
#include <linux/input.h>

#include <linux/usb/android_composite.h>

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

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 56 /*for 8k resolution 480*800*2 / 16k */

#define BITSPIXEL 16
#define PROJECTOR_FUNCTION_NAME "projector"

static int keypad_code[] = {KEY_WAKEUP, 0, 0, 0, KEY_HOME, KEY_MENU, KEY_BACK};
static const char shortname[] = "android_projector";

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
	struct input_dev *keypad_input;
	struct input_dev *touch_input;
	char *fbaddr;
	struct platform_device *pdev;
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

static inline struct projector_dev *func_to_dev(struct usb_function *f)
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
static void req_put(struct projector_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct projector_dev *dev, struct list_head *head)
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
	while ((req = req_get(ctxt, &ctxt->rx_idle))) {
		req->length = RXN_MAX;
		DBG("%s: queue %p\n", __func__, req);
		ret = usb_ep_queue(ctxt->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			DBG("projector: failed to queue out req (%d)\n", ret);
			ctxt->error = 1;
			req_put(ctxt, &ctxt->rx_idle, req);
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

	input_report_key(kdev, keypad_code[iKeycode], 1);
	input_sync(kdev);
	input_report_key(kdev, keypad_code[iKeycode], 0);
	input_sync(kdev);
}

static void send_fb(struct projector_dev *ctxt)
{

	struct usb_request *req;
	char *frame;
	int xfer;
	int count = ctxt->framesize;

	if (msmfb_get_fb_area())
		frame = (ctxt->fbaddr + ctxt->framesize);
	else
		frame = ctxt->fbaddr;

	while (count > 0) {
		req = req_get(ctxt, &ctxt->tx_idle);
		if (req) {
			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
			memcpy(req->buf, frame, xfer);
			if (usb_ep_queue(ctxt->ep_in, req, GFP_ATOMIC) < 0) {
				req_put(ctxt, &ctxt->tx_idle, req);
				printk(KERN_WARNING "%s: failed to queue req %p\n",
					__func__, req);
				break;
			}
			count -= xfer;
			frame += xfer;
		} else {
			DBG("send_fb: no req to send\n");
			break;
		}
	}
}
static void send_info(struct projector_dev *ctxt)
{
	struct usb_request *req;

	req = req_get(ctxt, &ctxt->tx_idle);
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
			req_put(ctxt, &ctxt->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req %p\n",
				__func__, req);
		}
	} else
		printk(KERN_INFO "%s: no req to send\n", __func__);
}

static void projector_get_msmfb(struct projector_dev *ctxt)
{
    struct msm_fb_info fb_info;

	msmfb_get_var(&fb_info);

	ctxt->bitsPixel = BITSPIXEL;
	ctxt->width = fb_info.xres;
	ctxt->height = fb_info.yres;
	ctxt->fbaddr = fb_info.fb_addr;
	printk(KERN_INFO "projector: width %d, height %d\n",
		   fb_info.xres, fb_info.yres);

	ctxt->framesize = (ctxt->width)*(ctxt->height)*2;
}

static void projector_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct projector_dev *dev = &_projector_dev;
	req_put(dev, &dev->tx_idle, req);
}

static void projector_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct projector_dev *ctxt = &_projector_dev;
	unsigned char *data = req->buf;
	int mouse_data[3];
	int i;
	DBG("%s: status %d, %d bytes\n", __func__,
		req->status, req->actual);

	if (req->status != 0) {
		ctxt->error = 1;
		req_put(ctxt, &ctxt->rx_idle, req);
		return ;
	}

	/* for mouse event type, 1 :move, 2:down, 3:up */
	mouse_data[0] = *((int *)(req->buf));

	if (!strncmp("init", data, 4)) {
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

	req_put(ctxt, &ctxt->rx_idle, req);
	projector_queue_out(ctxt);
}

static int __init create_bulk_endpoints(struct projector_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG("create_bulk_endpoints dev: %p\n", dev);

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
	for (i = 0; i < RX_REQ_MAX; i++) {
		req = projector_request_new(dev->ep_out, RXN_MAX);
		if (!req)
			goto fail;
		req->complete = projector_complete_out;
		req_put(dev, &dev->rx_idle, req);
	}
	for (i = 0; i < TX_REQ_MAX; i++) {
		req = projector_request_new(dev->ep_in, TXN_MAX);
		if (!req)
			goto fail;
		req->complete = projector_complete_in;
		req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	while ((req = req_get(dev, &dev->tx_idle)))
		projector_request_free(req, dev->ep_in);
	while ((req = req_get(dev, &dev->rx_idle)))
		projector_request_free(req, dev->ep_out);
	printk(KERN_ERR "projector: could not allocate requests\n");
	return -1;
}

static int
projector_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct projector_dev	*dev = func_to_dev(f);
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
	ret = create_bulk_endpoints(dev, &projector_fullspeed_in_desc,
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
	struct projector_dev	*dev = func_to_dev(f);
	struct usb_request *req;

	spin_lock_irq(&dev->lock);

	while ((req = req_get(dev, &dev->tx_idle)))
		projector_request_free(req, dev->ep_in);
	while ((req = req_get(dev, &dev->rx_idle)))
		projector_request_free(req, dev->ep_out);

	dev->online = 0;
	dev->error = 1;
	spin_unlock_irq(&dev->lock);
}

static int projector_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct projector_dev	*dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
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
	dev->online = 1;
	projector_queue_out(dev);

	return 0;
}

static void projector_function_disable(struct usb_function *f)
{
	struct projector_dev	*dev = func_to_dev(f);
	struct usb_composite_dev	*cdev = dev->cdev;

	DBG("projector_function_disable\n");
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static int projector_touch_init(struct projector_dev *dev)
{
	int x = dev->width;
	int y = dev->height;
	int ret = 0;
	struct input_dev *tdev = dev->touch_input;

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

static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int _enabled = simple_strtol(buf, NULL, 0);
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
static DEVICE_ATTR(enable, 0666, show_enable, store_enable);

static void prj_dev_release(struct device *dev) {}

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

	spin_lock_init(&dev->lock);

	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->tx_idle);

	dev->cdev = c->cdev;
	dev->function.name = "projector";
	dev->function.strings = projector_strings;
	dev->function.descriptors = fs_projector_descs;
	dev->function.hs_descriptors = hs_projector_descs;
	dev->function.bind = projector_function_bind;
	dev->function.unbind = projector_function_unbind;
	dev->function.set_alt = projector_function_set_alt;
	dev->function.disable = projector_function_disable;

	/* start disabled */
	dev->function.hidden = 1;

	msmfb_get_var(&fb_info);
	dev->bitsPixel = BITSPIXEL;
	dev->width = fb_info.xres;
	dev->height = fb_info.yres;
	dev->fbaddr = fb_info.fb_addr;
	if (projector_touch_init(dev) < 0)
		goto err;
	if (projector_keypad_init(dev) < 0)
		goto err;

	if (!dev->pdev)
		goto err;
	prj_dev.release = prj_dev_release;
	prj_dev.parent = &dev->pdev->dev;
	dev_set_name(&prj_dev, "interface");

	ret = device_register(&prj_dev);
	if (ret != 0) {
		DBG("projector failed to register device: %d\n", ret);
		goto err;
	}

	ret = device_create_file(&prj_dev, &dev_attr_enable);
	if (ret != 0) {
		DBG("projector device_create_file failed: %d\n", ret);
		goto err1;
	}

	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err2;

	return 0;
err2:
	device_remove_file(&prj_dev, &dev_attr_enable);
err1:
	device_unregister(&prj_dev);
err:
	printk(KERN_ERR "projector gadget driver failed to initialize\n");
	return ret;
}

static struct android_usb_function projector_function = {
	.name = "projector",
	.bind_config = projector_bind_config,
};

static int pjr_probe(struct platform_device *pdev)
{
	struct projector_dev *dev = &_projector_dev;
	dev->pdev = pdev;
	dev->init_done = 0;
	dev->frame_count = 0;
	return 0;
}

static struct platform_driver pjr_driver = {
	.probe = pjr_probe,
	.driver = { .name = PROJECTOR_FUNCTION_NAME, },
};

static void pjr_release(struct device *dev) {}


static struct platform_device pjr_device = {
	.name		= PROJECTOR_FUNCTION_NAME,
	.id		= -1,
	.dev		= {
		.release	= pjr_release,
	},
};
static int __init init(void)
{
	int r;
	printk(KERN_INFO "f_projector init\n");
	r = platform_driver_register(&pjr_driver);
	if (r < 0)
		return r;
	r = platform_device_register(&pjr_device);
	if (r < 0) {
		platform_driver_unregister(&pjr_driver);
		return r;
	}
	android_register_function(&projector_function);
	return 0;
}
module_init(init);
