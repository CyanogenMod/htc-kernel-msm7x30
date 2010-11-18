/*
 * Diag Function Device - Route DIAG frames between SMD and USB
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/mutex.h>

#include <mach/msm_smd.h>

#include <linux/usb/android_composite.h>

/*#define DIAG_DEBUG*/

#define NO_HDLC 1
#define ROUTE_TO_USERSPACE 1

struct device diag_device;

#if 1
#define TRACE(tag, data, len, decode) do {} while (0)
#else
static void TRACE(const char *tag, const void *_data, int len, int decode)
{
	const unsigned char *data = _data;
	int escape = 0;

	printk(KERN_INFO "%s", tag);
	if (decode) {
		while (len-- > 0) {
			unsigned x = *data++;
			if (x == 0x7e) {
				printk(" $$");
				escape = 0;
				continue;
			}
			if (x == 0x7d) {
				escape = 1;
				continue;
			}
			if (escape) {
				escape = 0;
				printk(" %02x", x ^ 0x20);
			} else {
				printk(" %02x", x);
			}
		}
	} else {
		while (len-- > 0) {
			printk(" %02x", *data++);
		}
		printk(" $$");
	}
	printk("\n");
}
#endif




#define HDLC_MAX 4096
#define SMD_MAX 8192

#define TX_REQ_BUF_SZ 8192
#define RX_REQ_BUF_SZ 8192

/* number of tx/rx requests to allocate */
#define TX_REQ_NUM 32
#define RX_REQ_NUM 32

struct diag_context {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	struct usb_ep *out;
	struct usb_ep *in;
	struct list_head tx_req_idle;
	struct list_head rx_req_idle;
	struct list_head rx_arm9_idle;
	struct list_head rx_arm9_done;
	spinlock_t req_lock;
#if ROUTE_TO_USERSPACE
	struct mutex user_lock;
#define ID_TABLE_SZ 10 /* keep this small */
	struct list_head rx_req_user;
	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	char *user_read_buf;
	uint32_t user_read_len;
	char *user_readp;
	bool opened;

	/* list of registered command ids to be routed to userspace */
	unsigned char id_table[ID_TABLE_SZ];
#endif
	smd_channel_t *ch;
	struct mutex smd_lock;
	int in_busy;
	int online;
	int error;
	int init_done;

	/* assembly buffer for USB->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;

	struct platform_device *pdev;
	u64 tx_count; /* to smd */
	u64 rx_count; /* from smd */

	int function_enable;

#if defined(CONFIG_MSM_N_WAY_SMD)
	smd_channel_t *chqdsp;
	struct list_head tx_qdsp_idle;
#endif
	/* for slate test */
	int is2ARM11;
	struct mutex diag2arm9_lock;
	struct mutex diag2arm9_read_lock;
	struct mutex diag2arm9_write_lock;
	bool diag2arm9_opened;
	unsigned char toARM9_buf[SMD_MAX];
	unsigned read_arm9_count;
	unsigned char *read_arm9_buf;
	wait_queue_head_t read_arm9_wq;
	struct usb_request *read_arm9_req;
};

static struct usb_interface_descriptor diag_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xFF,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor diag_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor diag_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor diag_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor diag_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_diag_descs[] = {
	(struct usb_descriptor_header *) &diag_interface_desc,
	(struct usb_descriptor_header *) &diag_fullspeed_in_desc,
	(struct usb_descriptor_header *) &diag_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_diag_descs[] = {
	(struct usb_descriptor_header *) &diag_interface_desc,
	(struct usb_descriptor_header *) &diag_highspeed_in_desc,
	(struct usb_descriptor_header *) &diag_highspeed_out_desc,
	NULL,
};

/* string descriptors: */

static struct usb_string diag_string_defs[] = {
	[0].s = "HTC DIAG",
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

static struct diag_context _context;

static inline struct diag_context *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct diag_context, function);
}

static int msm_diag_probe(struct platform_device *pdev);
static void smd_try_to_send(struct diag_context *ctxt);
static void smd_diag_notify(void *priv, unsigned event);

static void diag_queue_out(struct diag_context *ctxt);
#if defined(CONFIG_MSM_N_WAY_SMD)
static void diag_qdsp_complete_in(struct usb_ep *ept,
	struct usb_request *req);
#endif

static struct usb_request *diag_req_new(unsigned len)
{
	struct usb_request *req;
	if (len > SMD_MAX)
		return NULL;
	req = kmalloc(sizeof(struct usb_request), GFP_KERNEL);
	if (!req)
		return NULL;
	req->buf = kmalloc(len, GFP_KERNEL);
	if (!req->buf) {
		kfree(req);
		return NULL;
	}
	return req;
}

static void diag_req_free(struct usb_request *req)
{
	if (!req)
		return;

	if (req->buf) {
		kfree(req->buf);
		req->buf = 0;
	}
	kfree(req);
	req = 0;
}

/* add a request to the tail of a list */
static void req_put(struct diag_context *ctxt, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->req_lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct diag_context *ctxt,
		struct list_head *head)
{
	struct usb_request *req = 0;
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	if (!list_empty(head)) {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return req;
}

static void reqs_free(struct diag_context *ctxt, struct usb_ep *ep,
			struct list_head *head)
{
	struct usb_request *req;
	while ((req = req_get(ctxt, head))) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static void smd_diag_enable(char *src, int enable)
{
	struct diag_context *ctxt = &_context;

	printk(KERN_INFO "smd_try_open(%s): %d\n", src, enable);
	if (!ctxt->init_done)
		return;

	mutex_lock(&ctxt->smd_lock);
	if (enable) {
		if (!ctxt->ch)
			smd_open("SMD_DIAG", &ctxt->ch, ctxt, smd_diag_notify);
	} else {
		if (ctxt->ch) {
			smd_close(ctxt->ch);
			ctxt->ch = NULL;
		}
	}
	mutex_unlock(&ctxt->smd_lock);

}


#if ROUTE_TO_USERSPACE
#define USB_DIAG_IOC_MAGIC 0xFF
#define USB_DIAG_FUNC_IOC_ENABLE_SET	_IOW(USB_DIAG_IOC_MAGIC, 1, int)
#define USB_DIAG_FUNC_IOC_ENABLE_GET	_IOR(USB_DIAG_IOC_MAGIC, 2, int)
#define USB_DIAG_FUNC_IOC_REGISTER_SET  _IOW(USB_DIAG_IOC_MAGIC, 3, char *)
#define USB_DIAG_FUNC_IOC_AMR_SET	_IOW(USB_DIAG_IOC_MAGIC, 4, int)

static long diag_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	int tmp_value;
	unsigned long flags;
	unsigned char temp_id_table[ID_TABLE_SZ];

	printk(KERN_INFO "diag:diag_ioctl() cmd=%d\n", cmd);
#ifdef DIAG_DEBUG
	printk(KERN_INFO "%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);
#endif

	if (_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case USB_DIAG_FUNC_IOC_ENABLE_SET:
		if (copy_from_user(&tmp_value, argp, sizeof(int)))
			return -EFAULT;
		printk(KERN_INFO "diag: enable %d\n", tmp_value);
		android_enable_function(&_context.function, tmp_value);
		smd_diag_enable("diag_ioctl", tmp_value);
		/* force diag_read to return error when disable diag */
		if (tmp_value == 0)
			ctxt->error = 1;
		wake_up(&ctxt->read_wq);
	break;
	case USB_DIAG_FUNC_IOC_ENABLE_GET:
		tmp_value = !_context.function.hidden;
		if (copy_to_user(argp, &tmp_value, sizeof(tmp_value)))
			return -EFAULT;
	break;

	case USB_DIAG_FUNC_IOC_REGISTER_SET:
		if (copy_from_user(temp_id_table, (unsigned char *)argp, ID_TABLE_SZ))
			return -EFAULT;
		spin_lock_irqsave(&ctxt->req_lock, flags);
		memcpy(ctxt->id_table, temp_id_table, ID_TABLE_SZ);
		spin_unlock_irqrestore(&ctxt->req_lock, flags);
		break;
	case USB_DIAG_FUNC_IOC_AMR_SET:
		if (copy_from_user(&ctxt->is2ARM11, argp, sizeof(int)))
			return -EFAULT;
		printk(KERN_INFO "diag: is2ARM11 %d\n", ctxt->is2ARM11);
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static ssize_t diag_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req = 0;
	int ret = 0;

	/* we will block until we're online */
	if (!ctxt->online) {
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0 || ctxt->error)
			return -EFAULT;
	}

	mutex_lock(&ctxt->user_lock);

	if (ctxt->user_read_len && ctxt->user_readp) {
		if (count > ctxt->user_read_len)
			count = ctxt->user_read_len;
		if (copy_to_user(buf, ctxt->user_readp, count))
			ret = -EFAULT;
		else {
			ctxt->user_readp += count;
			ctxt->user_read_len -= count;
			ret = count;
		}
		goto end;
	}

	mutex_unlock(&ctxt->user_lock);
	ret = wait_event_interruptible(ctxt->read_wq,
		(req = req_get(ctxt, &ctxt->rx_req_user)) || !ctxt->online);
	mutex_lock(&ctxt->user_lock);
	if (ret < 0) {
		pr_err("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}
	if (!ctxt->online) {
		/* pr_err("%s: offline\n", __func__); */
		ret = -EIO;
		goto end;
	}
	if (req) {
		if (req->actual == 0) {
			pr_info("%s: no data\n", __func__);
			goto end;
		}
		if (count > req->actual)
			count = req->actual;
		if (copy_to_user(buf, req->buf, count)) {
			ret = -EFAULT;
			goto end;
		}
		req->actual -= count;
		if (req->actual) {
			memcpy(ctxt->user_read_buf, req->buf + count, req->actual);
			ctxt->user_read_len = req->actual;
			ctxt->user_readp = ctxt->user_read_buf;
		}
		ret = count;
	}

end:
	if (req)
		req_put(ctxt, &ctxt->rx_req_idle, req);

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static ssize_t diag_write(struct file *fp, const char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req = 0;
	int ret = 0;

	ret = wait_event_interruptible(ctxt->write_wq,
		((req = req_get(ctxt, &ctxt->tx_req_idle)) || !ctxt->online));

	mutex_lock(&ctxt->user_lock);

	if (ret < 0) {
		pr_err("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}

	if (!ctxt->online) {
		pr_err("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}

	if (count > TX_REQ_BUF_SZ)
		count = TX_REQ_BUF_SZ;

	if (req) {
		if (copy_from_user(req->buf, buf, count)) {
			ret = -EFAULT;
			goto end;
		}

		req->length = count;
		ret = usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
		if (ret < 0) {
			pr_err("%s: usb_ep_queue error %d\n", __func__, ret);
			goto end;
		}

		ret = req->length;
		/* zero this so we don't put it back to idle queue */
		req = 0;
	}

end:
	if (req)
		req_put(ctxt, &ctxt->tx_req_idle, req);

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static int diag_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	int rc = 0;

	mutex_lock(&ctxt->user_lock);

	if (ctxt->opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}


	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (!ctxt->user_read_buf) {
		ctxt->user_read_buf = kmalloc(RX_REQ_BUF_SZ, GFP_KERNEL);
		if (!ctxt->user_read_buf) {
			rc = -ENOMEM;
			goto done;
		}
	}
	ctxt->opened = true;
	/* clear the error latch */
	ctxt->error = 0;

done:
	mutex_unlock(&ctxt->user_lock);
	return rc;
}

static int diag_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;

	mutex_lock(&ctxt->user_lock);
	ctxt->opened = false;
	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (ctxt->user_read_buf) {
		kfree(ctxt->user_read_buf);
		ctxt->user_read_buf = 0;
	}
	mutex_unlock(&ctxt->user_lock);

	return 0;
}

static struct file_operations diag_fops = {
	.owner =   THIS_MODULE,
	.read =    diag_read,
	.write =   diag_write,
	.open =    diag_open,
	.release = diag_release,
	.unlocked_ioctl = diag_ioctl,
};

static struct miscdevice diag_device_fops = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag",
	.fops = &diag_fops,
};
#endif

static int diag2arm9_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	int rc = 0;
	int n;
	printk(KERN_INFO "%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_lock);
	if (ctxt->diag2arm9_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	/* clear pending data if any */
	while ((req = req_get(ctxt, &ctxt->rx_arm9_done)))
		diag_req_free(req);

	for (n = 0; n < 4; n++) {
		req = diag_req_new(SMD_MAX);
		if (!req) {
			while ((req = req_get(ctxt, &ctxt->rx_arm9_idle)))
				diag_req_free(req);
			rc = -EFAULT;
			goto done;
		}
		req_put(ctxt, &ctxt->rx_arm9_idle, req);
	}
	ctxt->read_arm9_count = 0;
	ctxt->read_arm9_buf = 0;
	ctxt->read_arm9_req = 0;
	ctxt->diag2arm9_opened = true;

	smd_diag_enable("diag2arm9_open", 1);
done:
	mutex_unlock(&ctxt->diag2arm9_lock);
	return rc;
}

static int diag2arm9_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;

	printk(KERN_INFO "%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_lock);
	ctxt->diag2arm9_opened = false;
	ctxt->is2ARM11 = 0;
	wake_up(&ctxt->read_arm9_wq);
	mutex_lock(&ctxt->diag2arm9_read_lock);
	while ((req = req_get(ctxt, &ctxt->rx_arm9_idle)))
		diag_req_free(req);
	while ((req = req_get(ctxt, &ctxt->rx_arm9_done)))
		diag_req_free(req);
	if (ctxt->read_arm9_req)
		diag_req_free(ctxt->read_arm9_req);
	mutex_unlock(&ctxt->diag2arm9_read_lock);

	/*************************************
	* If smd is closed, it will  lead to slate can't be tested.
	* slate will open it for one time
	* but close it for several times and never open
	*************************************/
	/*smd_diag_enable("diag2arm9_release", 0);*/
	mutex_unlock(&ctxt->diag2arm9_lock);

	return 0;
}

static ssize_t diag2arm9_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	int r = count;
	int writed = 0;

	mutex_lock(&ctxt->diag2arm9_write_lock);

	while (count > 0) {
		writed = count > SMD_MAX ? SMD_MAX : count;
		if (copy_from_user(ctxt->toARM9_buf, buf, writed)) {
			r = -EFAULT;
			break;
		}
		if (ctxt->ch == NULL) {
			printk(KERN_ERR "%s: ctxt->ch == NULL", __func__);
			r = -EFAULT;
			break;
		} else if (ctxt->toARM9_buf == NULL) {
			printk(KERN_ERR "%s: ctxt->toARM9_buf == NULL", __func__);
			r = -EFAULT;
			break;
		}
		smd_write(ctxt->ch, ctxt->toARM9_buf, writed);
		ctxt->tx_count += writed;
		buf += writed;
		count -= writed;
	}

	mutex_unlock(&ctxt->diag2arm9_write_lock);
	return r;
}

static ssize_t diag2arm9_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	int r = 0, xfer;
	int ret;

	mutex_lock(&ctxt->diag2arm9_read_lock);

	/* if we have data pending, give it to userspace */
	if (ctxt->read_arm9_count > 0)
		req = ctxt->read_arm9_req;
	else {
retry:
	/* get data from done queue */
		req = 0;
		ret = wait_event_interruptible(ctxt->read_arm9_wq,
				((req = req_get(ctxt, &ctxt->rx_arm9_done)) ||
				!ctxt->diag2arm9_opened));
		if (!ctxt->diag2arm9_opened) {
			if (req)
				req_put(ctxt, &ctxt->rx_arm9_idle, req);
			goto done;
		}
		if (ret < 0 || req == 0)
			goto done;

		if (req->actual == 0) {
			req_put(ctxt, &ctxt->rx_arm9_idle, req);
			goto retry;
		}
		ctxt->read_arm9_req = req;
		ctxt->read_arm9_count = req->actual;
		ctxt->read_arm9_buf = req->buf;
	}
	xfer = (ctxt->read_arm9_count < count) ? ctxt->read_arm9_count : count;
	if (copy_to_user(buf, ctxt->read_arm9_buf, xfer)) {
		printk(KERN_INFO "diag: copy_to_user fail\n");
		r = -EFAULT;
		goto done;
	}
	ctxt->read_arm9_buf += xfer;
	ctxt->read_arm9_count -= xfer;
	r += xfer;
	/* if we've emptied the buffer, release the request */
	if (ctxt->read_arm9_count == 0) {
		req_put(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
		ctxt->read_arm9_req = 0;
	}
done:
	mutex_unlock(&ctxt->diag2arm9_read_lock);
	return r;
}
static struct file_operations diag2arm9_fops = {
	.owner =   THIS_MODULE,
	.open =    diag2arm9_open,
	.release = diag2arm9_release,
	.write = diag2arm9_write,
	.read = diag2arm9_read,
};

static struct miscdevice diag2arm9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag_arm9",
	.fops = &diag2arm9_fops,
};

static void diag_in_complete(struct usb_ep *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;
#if ROUTE_TO_USERSPACE
	char c;
#endif

	ctxt->in_busy = 0;
	req_put(ctxt, &ctxt->tx_req_idle, req);

#if ROUTE_TO_USERSPACE
	c = *((char *)req->buf + req->actual - 1);
	if (c == 0x7e)
		wake_up(&ctxt->write_wq);
#endif

	smd_try_to_send(ctxt);
}

#if !NO_HDLC
static void diag_process_hdlc(struct diag_context *ctxt, void *_data, unsigned len)
{
	unsigned char *data = _data;
	unsigned count = ctxt->hdlc_count;
	unsigned escape = ctxt->hdlc_escape;
	unsigned char *hdlc = ctxt->hdlc_buf;

	while (len-- > 0) {
		unsigned char x = *data++;
		if (x == 0x7E) {
			if (count > 2) {
				/* we're just ignoring the crc here */
				TRACE("PC>", hdlc, count - 2, 0);
				if (ctxt->ch)
					smd_write(ctxt->ch, hdlc, count - 2);
			}
			count = 0;
			escape = 0;
		} else if (x == 0x7D) {
			escape = 1;
		} else {
			if (escape) {
				x = x ^ 0x20;
				escape = 0;
			}
			hdlc[count++] = x;

			/* discard frame if we overflow */
			if (count == HDLC_MAX)
				count = 0;
		}
	}

	ctxt->hdlc_count = count;
	ctxt->hdlc_escape = escape;
}
#endif

#if ROUTE_TO_USERSPACE
static int if_route_to_userspace(struct diag_context *ctxt, unsigned int cmd_id)
{
	unsigned long flags;
	int i;

	if (!ctxt->opened || cmd_id == 0)
		return 0;

	/* command ids 0xfb..0xff are not used by msm diag; we steal these ids
	 * for communication between userspace tool and host test tool.
	 */
	if (cmd_id >= 0xfb && cmd_id <= 0xff)
		return 1;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	for (i = 0; i < ARRAY_SIZE(ctxt->id_table); i++)
		if (ctxt->id_table[i] == cmd_id) {
			/* if the command id equals to any of registered ids,
			 * route to userspace to handle.
			 */
			spin_unlock_irqrestore(&ctxt->req_lock, flags);
			return 1;
		}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return 0;
}
#endif

static void diag_out_complete(struct usb_ep *ept, struct usb_request *req)
{
	struct diag_context *ctxt = req->context;

	if (req->status == 0) {
#if ROUTE_TO_USERSPACE
		unsigned int cmd_id = *((unsigned char *)req->buf);
		if (if_route_to_userspace(ctxt, cmd_id)) {
			req_put(ctxt, &ctxt->rx_req_user, req);
			wake_up(&ctxt->read_wq);
			diag_queue_out(ctxt);
			return;
		}
#endif

#if NO_HDLC
		TRACE("PC>", req->buf, req->actual, 0);
		if (ctxt->ch) {
			smd_write(ctxt->ch, req->buf, req->actual);
			ctxt->tx_count += req->actual;
		}
#else
		diag_process_hdlc(ctxt, req->buf, req->actual);
#endif
	}

	req_put(ctxt, &ctxt->rx_req_idle, req);
	diag_queue_out(ctxt);
}

static void diag_queue_out(struct diag_context *ctxt)
{
	struct usb_request *req;
	int rc;

	req = req_get(ctxt, &ctxt->rx_req_idle);
	if (!req) {
		pr_err("%s: rx req queue - out of buffer\n", __func__);
		return;
	}

	req->complete = diag_out_complete;
	req->context = ctxt;
	req->length = RX_REQ_BUF_SZ;

	rc = usb_ep_queue(ctxt->out, req, GFP_ATOMIC);
	if (rc < 0) {
		pr_err("%s: usb_ep_queue failed: %d\n", __func__, rc);
		req_put(ctxt, &ctxt->rx_req_idle, req);
	}
}

static void smd_try_to_send(struct diag_context *ctxt)
{
again:
	if (ctxt->ch && (!ctxt->in_busy)) {
		int r = smd_read_avail(ctxt->ch);

		if (r > TX_REQ_BUF_SZ) {
			printk(KERN_ERR "The SMD data is too large to send!!\n");
			return;
		}
		if (r > 0 && ctxt->is2ARM11) {
			/* to arm11 user space */
			struct usb_request *req;
			if (!ctxt->diag2arm9_opened)
				return;
			req = req_get(ctxt, &ctxt->rx_arm9_idle);
			if (!req) {
				printk(KERN_ERR "There is no enough request to ARM11!!\n");
				return;
			}
			smd_read(ctxt->ch, req->buf, r);
			ctxt->rx_count += r;
			req->actual = r;
			req_put(ctxt, &ctxt->rx_arm9_done, req);
			wake_up(&ctxt->read_arm9_wq);
			return;
		}
		if (!ctxt->online)
			return;
		if (r > 0) {
			struct usb_request *req;
			req = req_get(ctxt, &ctxt->tx_req_idle);
			if (!req) {
				pr_err("%s: tx req queue is out of buffers\n",
					__func__);
				return;
			}
			smd_read(ctxt->ch, req->buf, r);
			ctxt->rx_count += r;

			if (!ctxt->online) {
				/* printk("$$$ discard %d\n", r);*/
				req_put(ctxt, &ctxt->tx_req_idle, req);
				goto again;
			}
			req->complete = diag_in_complete;
			req->context = ctxt;
			req->length = r;

			TRACE("A9>", req->buf, r, 1);
			ctxt->in_busy = 1;
			r = usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
			if (r < 0) {
				pr_err("%s: usb_ep_queue failed: %d\n",
					__func__, r);
				req_put(ctxt, &ctxt->tx_req_idle, req);
				ctxt->in_busy = 0;
			}
		}
	}
}

static void smd_diag_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	smd_try_to_send(ctxt);
}

static int __init create_bulk_endpoints(struct diag_context *ctxt,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = ctxt->cdev;
	struct usb_ep *ep;
	struct usb_request *req;
	int n;

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	ep->driver_data = ctxt;		/* claim the endpoint */
	ctxt->in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		return -ENODEV;
	}
	ep->driver_data = ctxt;		/* claim the endpoint */
	ctxt->out = ep;

	ctxt->tx_count = ctxt->rx_count = 0;

	for (n = 0; n < RX_REQ_NUM; n++) {
		req = usb_ep_alloc_request(ctxt->out, GFP_KERNEL);
		if (!req) {
			DBG(cdev, "%s: usb_ep_alloc_request out of memory\n",
				__func__);
			goto rx_fail;
		}
		req->buf = kmalloc(RX_REQ_BUF_SZ, GFP_KERNEL);
		if (!req->buf) {
			DBG(cdev, "%s: kmalloc out of memory\n", __func__);
			goto rx_fail;
		}
		req->context = ctxt;
		req->complete = diag_out_complete;
		req_put(ctxt, &ctxt->rx_req_idle, req);
	}

	for (n = 0; n < TX_REQ_NUM; n++) {
		req = usb_ep_alloc_request(ctxt->in, GFP_KERNEL);
		if (!req) {
			DBG(cdev, "%s: usb_ep_alloc_request out of memory\n",
				__func__);
			goto tx_fail;
		}
		req->buf = kmalloc(TX_REQ_BUF_SZ, GFP_KERNEL);
		if (!req->buf) {
			DBG(cdev, "%s: kmalloc out of memory\n", __func__);
			goto tx_fail;
		}
		req->context = ctxt;
		req->complete = diag_in_complete;
		req_put(ctxt, &ctxt->tx_req_idle, req);
	}

#if defined(CONFIG_MSM_N_WAY_SMD)
	for (n = 0; n < TX_REQ_NUM; n++) {
		req = usb_ep_alloc_request(ctxt->in, GFP_KERNEL);
		if (!req) {
			DBG(cdev, "%s: usb_ep_alloc_request out of memory\n",
				__func__);
			goto qdsp_tx_fail;
		}
		req->buf = kmalloc(TX_REQ_BUF_SZ, GFP_KERNEL);
		if (!req->buf) {
			DBG(cdev, "%s: kmalloc out of memory\n", __func__);
			goto qdsp_tx_fail;
		}
		req->context = ctxt;
		req->complete = diag_qdsp_complete_in;
		req_put(ctxt, &ctxt->tx_qdsp_idle, req);
	}
#endif

	return 0;
#if defined(CONFIG_MSM_N_WAY_SMD)
qdsp_tx_fail:
	reqs_free(ctxt, ctxt->in, &ctxt->tx_qdsp_idle);
#endif
tx_fail:
	reqs_free(ctxt, ctxt->in, &ctxt->tx_req_idle);
rx_fail:
	reqs_free(ctxt, ctxt->out, &ctxt->rx_req_idle);
	return -ENOMEM;
}

static void diag_dev_release(struct device *dev) {}

static ssize_t show_diag_xfer_count(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct diag_context *ctxt = &_context;

	return  sprintf(buf, "tx_count: %llu, rx_count: %llu\n",
		ctxt->tx_count, ctxt->rx_count);
}

static DEVICE_ATTR(diag_xfer_count, 0444, show_diag_xfer_count, NULL);

static int
diag_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct diag_context	*ctxt = func_to_dev(f);
	int			id;
	int			ret;

	ctxt->cdev = cdev;

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	diag_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = create_bulk_endpoints(ctxt, &diag_fullspeed_in_desc,
			&diag_fullspeed_out_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		diag_highspeed_in_desc.bEndpointAddress =
			diag_fullspeed_in_desc.bEndpointAddress;
		diag_highspeed_out_desc.bEndpointAddress =
			diag_fullspeed_out_desc.bEndpointAddress;
	}

#if ROUTE_TO_USERSPACE
	misc_register(&diag_device_fops);
#endif
	misc_register(&diag2arm9_device);

	diag_device.release = diag_dev_release;
	diag_device.parent = &ctxt->pdev->dev;
	dev_set_name(&diag_device, "interface");
	if (device_register(&diag_device) != 0) {
		printk(KERN_ERR "diag failed to register device\n");
		return 0;
	}
	if (device_create_file(&diag_device, &dev_attr_diag_xfer_count) != 0) {
		printk(KERN_ERR "diag device_create_file failed");
		device_unregister(&diag_device);
		return 0;
	}
	ctxt->tx_count = ctxt->rx_count = 0;
	return 0;
}

static void
diag_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct diag_context	*ctxt = func_to_dev(f);
	reqs_free(ctxt, ctxt->out, &ctxt->rx_req_idle);
	reqs_free(ctxt, ctxt->in, &ctxt->tx_req_idle);

#if ROUTE_TO_USERSPACE
	misc_deregister(&diag_device_fops);
#endif
	misc_deregister(&diag2arm9_device);

	ctxt->tx_count = ctxt->rx_count = 0;
}

static int diag_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct diag_context	*ctxt = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
#if ROUTE_TO_USERSPACE
	struct usb_request *req;
#endif
	int ret;

	ret = usb_ep_enable(ctxt->in,
			ep_choose(cdev->gadget,
				&diag_highspeed_in_desc,
				&diag_fullspeed_in_desc));
	if (ret)
		return ret;
	ret = usb_ep_enable(ctxt->out,
			ep_choose(cdev->gadget,
				&diag_highspeed_out_desc,
				&diag_fullspeed_out_desc));
	if (ret) {
		usb_ep_disable(ctxt->in);
		return ret;
	}
	ctxt->online = !ctxt->function.hidden;

#if ROUTE_TO_USERSPACE
	/* recycle unhandled rx reqs to user if any */
	while ((req = req_get(ctxt, &ctxt->rx_req_user)))
		req_put(ctxt, &ctxt->rx_req_idle, req);
#endif

	if (ctxt->online) {
		diag_queue_out(ctxt);
		smd_try_to_send(ctxt);
	}
#if ROUTE_TO_USERSPACE
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->write_wq);
#endif

	return 0;
}

static void diag_function_disable(struct usb_function *f)
{
	struct diag_context	*ctxt = func_to_dev(f);

	ctxt->online = 0;
#if ROUTE_TO_USERSPACE
	wake_up(&ctxt->read_wq);
	wake_up(&ctxt->write_wq);
#endif
	usb_ep_disable(ctxt->in);
	usb_ep_disable(ctxt->out);
}
#if defined(CONFIG_MSM_N_WAY_SMD)
static void diag_qdsp_send(struct diag_context *ctxt)
{
	int ret, r;
	struct usb_request *req;
	if (ctxt->chqdsp && ctxt->online) {
		r = smd_read_avail(ctxt->chqdsp);
		if (r > SMD_MAX || r <= 0)
			return;

		req = req_get(ctxt, &ctxt->tx_qdsp_idle);
		if (!req)
			return;

		smd_read(ctxt->chqdsp, req->buf, r);
		req->length = r;

		ret = usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
		if (ret < 0) {
			printk(KERN_INFO "diag: failed to queue qdsp req %d\n",
				ret);
			req_put(ctxt, &ctxt->tx_qdsp_idle, req);
		}
	}
}

static void diag_qdsp_complete_in(struct usb_ep *ept,
		struct usb_request *req)
{
	struct diag_context *ctxt = req->context;

	req_put(ctxt, &ctxt->tx_qdsp_idle, req);
	diag_qdsp_send(ctxt);

#if ROUTE_TO_USERSPACE
	wake_up(&ctxt->write_wq);
#endif
}


static void diag_qdsp_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	diag_qdsp_send(ctxt);
}

static struct platform_driver msm_smd_qdsp_ch1_driver = {
	.probe = msm_diag_probe,
	.driver = {
		.name = "DSP_DIAG",
		.owner = THIS_MODULE,
	},
};
#endif

static int msm_diag_probe(struct platform_device *pdev)
{
	struct diag_context *ctxt = &_context;
	ctxt->pdev = pdev;
	printk(KERN_INFO "diag:msm_diag_probe(), pdev->id=0x%x\n", pdev->id);

#if defined(CONFIG_MSM_N_WAY_SMD)
	if (pdev->id == 1)
		smd_open("DSP_DIAG", &ctxt->chqdsp, ctxt, diag_qdsp_notify);
#endif
	return 0;
}

static int diag_set_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	if (_context.cdev)
		android_enable_function(&_context.function, enabled);
	_context.function_enable = !!enabled;
	smd_diag_enable("diag_set_enabled", enabled);
	return 0;
}

static int diag_get_tx_rx_count(char *buffer, struct kernel_param *kp)
{
	struct diag_context *ctxt = &_context;

	return sprintf(buffer, "tx: %llu bytes, rx: %llu bytes",
	ctxt->tx_count, ctxt->rx_count);
}
module_param_call(tx_rx_count, NULL, diag_get_tx_rx_count, NULL, 0444);

static int diag_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !_context.function.hidden;
	return 1;
}
module_param_call(enabled, diag_set_enabled, diag_get_enabled, NULL, 0664);


int diag_bind_config(struct usb_configuration *c)
{
	struct diag_context *ctxt = &_context;
	int ret;

	printk(KERN_INFO "diag_bind_config\n");

	ret = usb_string_id(c->cdev);
	if (ret < 0)
		return ret;
	diag_string_defs[0].id = ret;
	diag_interface_desc.iInterface = ret;

	ctxt->cdev = c->cdev;
	ctxt->function.name = "diag";
	ctxt->function.strings = diag_strings;
	ctxt->function.descriptors = fs_diag_descs;
	ctxt->function.hs_descriptors = hs_diag_descs;
	ctxt->function.bind = diag_function_bind;
	ctxt->function.unbind = diag_function_unbind;
	ctxt->function.set_alt = diag_function_set_alt;
	ctxt->function.disable = diag_function_disable;

/* Workaround: enable diag first */
#ifdef CONFIG_MACH_MECHA
	ctxt->function.hidden = 0;
#else
	ctxt->function.hidden = !_context.function_enable;
#endif
	if (!ctxt->function.hidden)
		smd_diag_enable("diag_bind_config", 1);

	return usb_add_function(c, &ctxt->function);
}

static struct android_usb_function diag_function = {
	.name = "diag",
	.bind_config = diag_bind_config,
};

static struct platform_driver msm_smd_ch1_driver = {
	.probe = msm_diag_probe,
	.driver = {
		.name = "SMD_DIAG",
		.owner = THIS_MODULE,
	},
};
static void diag_plat_release(struct device *dev) {}

static struct platform_device diag_plat_device = {
	.name		= "SMD_DIAG",
	.id		= -1,
	.dev		= {
		.release	= diag_plat_release,
	},
};
static int __init init(void)
{
	struct diag_context *ctxt = &_context;
	int r;

	printk(KERN_INFO "diag init\n");
	spin_lock_init(&ctxt->req_lock);
	INIT_LIST_HEAD(&ctxt->rx_req_idle);
	INIT_LIST_HEAD(&ctxt->tx_req_idle);
	INIT_LIST_HEAD(&ctxt->rx_arm9_idle);
	INIT_LIST_HEAD(&ctxt->rx_arm9_done);
#if ROUTE_TO_USERSPACE
	mutex_init(&ctxt->user_lock);
	INIT_LIST_HEAD(&ctxt->rx_req_user);
	init_waitqueue_head(&ctxt->read_wq);
	init_waitqueue_head(&ctxt->write_wq);
#endif
	init_waitqueue_head(&ctxt->read_arm9_wq);
	mutex_init(&ctxt->diag2arm9_lock);
	mutex_init(&ctxt->diag2arm9_read_lock);
	mutex_init(&ctxt->diag2arm9_write_lock);
	mutex_init(&ctxt->smd_lock);
	ctxt->is2ARM11 = 0;

	r = platform_driver_register(&msm_smd_ch1_driver);
	if (r < 0) {
		printk(KERN_ERR "%s: Register device fail\n", __func__);
		return r;
	}
#if defined(CONFIG_MSM_N_WAY_SMD)
	INIT_LIST_HEAD(&ctxt->tx_qdsp_idle);
	platform_driver_register(&msm_smd_qdsp_ch1_driver);
#endif
	r = platform_device_register(&diag_plat_device);
	if (r < 0) {
		printk(KERN_ERR "%s: Register device fail\n", __func__);
#if defined(CONFIG_MSM_N_WAY_SMD)
		platform_driver_unregister(&msm_smd_qdsp_ch1_driver);
#endif
		return r;
	}
	ctxt->init_done = 1;

	android_register_function(&diag_function);
	return 0;
}
module_init(init);
