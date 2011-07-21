/* drivers/usb/gadget/f_diag.c
 *Diag Function Device - Route ARM9 and ARM11 DIAG messages
 *between HOST and DEVICE.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#include <linux/usb/android_composite.h>

#include <mach/7x30-lte/msm_smd.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <mach/smsc251x.h>
#include "f_diag.h"
#include "../../char/diag/diagchar.h"
#include "../../char/diag/diagchar_hdlc.h"
#define WRITE_COMPLETE 0
#define READ_COMPLETE  0
#define TRUE  1
#define FALSE 0

#define MAX(x, y) (x > y ? x : y)


/*#define HTC_DIAG_DEBUG*/
#define HTC_RADIO_ROUTING 0
static DEFINE_SPINLOCK(dev_lock);

struct diag_context _context;
struct device htc_diag_device;

void *diagmem_alloc(struct diagchar_dev *driver, int size, int pool_type);
static int htc_diag_open(struct inode *ip, struct file *fp);
static struct diag_request *htc_write_d_req;
static char *htc_write_buf_copy;
static long diag2arm9_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
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
/* list of requests */
struct diag_req_entry {
	struct list_head re_entry;
	struct usb_request *usb_req;
	void *diag_request;
};



static void usb_config_work_func(struct work_struct *);

static void diag_write_complete(struct usb_ep *,
		struct usb_request *);
static struct diag_req_entry *diag_alloc_req_entry(struct usb_ep *,
		unsigned len, gfp_t);
static void diag_free_req_entry(struct usb_ep *, struct diag_req_entry *);
static void diag_read_complete(struct usb_ep *, struct usb_request *);

#if EPST_FUN
static	uint16_t nv7K9K_table[NV_TABLE_SZ] = {82,
0, 4, 5, 20, 21, 37, 258, 318, 460, 461,
462, 463, 464, 465, 466, 546, 707, 714, 854, 1943,
2825, 2953, 30000, 30001, 30002, 30003, 30004, 30005, 30006, 30007,
30008, 30010, 30013, 30014, 30015, 30018, 30019, 30021, 30031, 30032,
30033, 6, 8, 53, 54, 161, 162, 40, 42, 43,
57, 71, 82, 85, 168, 169, 170, 191, 192, 197,
219, 231, 240, 241, 256, 297, 298, 300, 423, 424,
429, 442, 450, 459, 495, 835, 855, 910, 933, 941,
945, 3634};

static	uint16_t nv7Konly_table[NV_TABLE_SZ] = {43,
25, 27, 29, 32, 33, 34, 35, 36, 176, 177,
259, 260, 261, 262, 263, 264, 265, 296, 319, 374,
375, 67, 69, 70, 74, 178, 215, 179, 255, 285,
401, 403, 405, 409, 426, 452, 1018, 4102, 906, 30026,
30027, 30028, 30029};

static	uint16_t nv7K9Kdiff_table[NV_TABLE_SZ] = {14,
11, 10, 441, 946, 30016, 30030, 562, 32768+11, 32768+10, 32768+441,
32768+946, 32768+30016, 32768+30030, 32768+562};

static	uint16_t nv9Konly_table[NV_TABLE_SZ] = {7, 579, 580, 1192, 1194, 4204, 4964, 818};

/*mode reset */
static	uint16_t M297K9K_table[M29_TABLE_SZ] = {4, 1, 2, 4, 5};
static	uint16_t M297Konly_table[M29_TABLE_SZ];
static	uint16_t M297K9Kdiff_table[M29_TABLE_SZ];
static	uint16_t M299Konly_table[M29_TABLE_SZ];

/*PRL read/write*/
static	uint16_t PRL7K9K_table[PRL_TABLE_SZ] = {1, 0};
static	uint16_t PRL7Konly_table[PRL_TABLE_SZ];
static	uint16_t PRL7K9Kdiff_table[PRL_TABLE_SZ];
static	uint16_t PRL9Konly_table[PRL_TABLE_SZ];

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
#endif
#if USB_TO_USERSPACE
#define TRX_REQ_BUF_SZ 8192


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




static long htc_diag_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	int tmp_value;
	unsigned long flags;
	unsigned char temp_id_table[ID_TABLE_SZ];

	printk(KERN_INFO "diag:htc_diag_ioctl()\n");
#ifdef HTC_DIAG_DEBUG
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

		diag_smd_enable("diag_ioctl", tmp_value);
		/* force diag_read to return error when disable diag */
		smsc251x_mdm_port_sw(tmp_value);
		if (tmp_value == 0)
			ctxt->error = 1;
		wake_up(&ctxt->read_wq);
	break;
	case USB_DIAG_FUNC_IOC_ENABLE_GET:
		tmp_value = !_context.function.disabled;
		if (copy_to_user(argp, &tmp_value, sizeof(tmp_value)))
			return -EFAULT;
	break;

	case USB_DIAG_FUNC_IOC_REGISTER_SET:
		if (copy_from_user(temp_id_table, (unsigned char *)argp, ID_TABLE_SZ))
			return -EFAULT;
		spin_lock_irqsave(&ctxt->req_lock, flags);
		memcpy(ctxt->id_table, temp_id_table, ID_TABLE_SZ);
		/*print_hex_dump(KERN_DEBUG, "ID_TABLE_SZ Data: ", 16, 1,
		DUMP_PREFIX_ADDRESS, temp_id_table, ID_TABLE_SZ, 1);*/
		spin_unlock_irqrestore(&ctxt->req_lock, flags);
		break;
	default:
		return -ENOTTY;
	}


	return 0;
}

static ssize_t htc_diag_read(struct file *fp, char __user *buf,
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
		printk("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}
	if (!ctxt->online) {
		 printk("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}
	if (req) {
		if (req->actual == 0) {
			printk("%s: no data\n", __func__);
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
	mutex_unlock(&ctxt->user_lock);

	return ret;
}

static ssize_t htc_diag_write(struct file *fp, const char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	int ret = 0;


	mutex_lock(&ctxt->user_lock);

	if (ret < 0) {
		printk("%s: wait_event_interruptible error %d\n",
			__func__, ret);
		goto end;
	}

	if (!ctxt->online) {
		printk("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}

	if (count > TRX_REQ_BUF_SZ)
		count = TRX_REQ_BUF_SZ;

	if (!htc_write_buf_copy || !htc_write_d_req) {
		ret = -EIO;
		goto end;
	}

	if (copy_from_user(htc_write_buf_copy, buf, count)) {
		ret = -EFAULT;
		printk("%s:EFAULT\n", __func__);
		goto end;
	}

	htc_write_d_req->buf = htc_write_buf_copy;
	htc_write_d_req->length = count;

	driver->in_busy_dm = 1;

	ret = diag_write(htc_write_d_req);

	if (ret < 0) {
		printk("%s: diag_write error %d\n", __func__, ret);
		goto end;
	}
		ret = count;

end:

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static int htc_diag_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	int rc = 0;

#ifdef HTC_DIAG_DEBUG
	printk(KERN_INFO "%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);
#endif
	mutex_lock(&ctxt->user_lock);

	if (ctxt->opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}

	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (!ctxt->user_read_buf) {
		ctxt->user_read_buf = kmalloc(TRX_REQ_BUF_SZ, GFP_KERNEL);
		if (!ctxt->user_read_buf) {
			rc = -ENOMEM;
			goto done;
		}
	}

	if (!htc_write_buf_copy) {
		htc_write_buf_copy = (char *)kmalloc(TRX_REQ_BUF_SZ, GFP_KERNEL);
		if (!htc_write_buf_copy) {
			rc = -ENOMEM;
			kfree(ctxt->user_read_buf);
			goto done;
		}
	}

	if (!htc_write_d_req) {
		htc_write_d_req = (struct diag_request *)kmalloc(sizeof(struct diag_request), GFP_KERNEL);
		if (!htc_write_d_req) {
			kfree(ctxt->user_read_buf);
			kfree(htc_write_buf_copy);
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

static int htc_diag_release(struct inode *ip, struct file *fp)
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
	if (!htc_write_buf_copy) {
		kfree(htc_write_buf_copy);
		htc_write_buf_copy = 0;
	}
	if (!htc_write_d_req) {
		kfree(htc_write_d_req);
		htc_write_d_req = 0;
	}

	mutex_unlock(&ctxt->user_lock);

	return 0;
}

static struct file_operations htc_diag_fops = {
	.owner =   THIS_MODULE,
	.read =    htc_diag_read,
	.write =   htc_diag_write,
	.open =    htc_diag_open,
	.release = htc_diag_release,
	.unlocked_ioctl = htc_diag_ioctl,
};

static struct miscdevice htc_diag_device_fops = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htcdiag",
	.fops = &htc_diag_fops,
};


static int if_route_to_userspace(struct diag_context *ctxt, unsigned int cmd)
{
	unsigned long flags;
	int i;
	unsigned short tmp;
	unsigned char cmd_id, cmd_num;

	tmp = (unsigned short)cmd;

	cmd_num = (unsigned char)(tmp >> 8);
	cmd_id = (unsigned char)(tmp & 0x00ff);

	if (!ctxt->opened || cmd_id == 0)
		return 0;
	/* command ids 0xfb..0xff are not used by msm diag; we steal these ids
	 * for communication between userspace tool and host test tool.
	 */
	if (cmd_id >= 0xfb && cmd_id <= 0xff)
		return 1;


	spin_lock_irqsave(&ctxt->req_lock, flags);
	for (i = 0; i < ARRAY_SIZE(ctxt->id_table); i = i+2)
		if (ctxt->id_table[i] == cmd_id) {
			/* if the command id equals to any of registered ids,
			 * route to userspace to handle.
			 */
			if (ctxt->id_table[i+1] == cmd_num || ctxt->id_table[i+1] == 0xff) {
				spin_unlock_irqrestore(&ctxt->req_lock, flags);
				return 1;
			}
		}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return 0;
}
#endif


#if HPST_FUN


int checkcmd_modem_hpst(unsigned char *buf)
{
	int j;
	uint16_t nv_num;
	uint16_t max_item;

		if (*buf == HPST_PREFIX) {
			if (*(buf+1) == 0x26 || *(buf+1) == 0x27) {
				max_item = MAX(MAX(nv7K9K_table[0], nv7Konly_table[0]),
				 MAX(nv9Konly_table[0], nv7K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < NV_TABLE_SZ; j++) {
					if (j <= nv7K9K_table[0] && nv7K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= nv7Konly_table[0] && nv7Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= nv9Konly_table[0]  && nv9Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= nv7K9Kdiff_table[0]  && nv7K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0x48 || *(buf+1) == 0x49) {
				max_item = MAX(MAX(PRL7K9K_table[0], PRL7Konly_table[0]),
				 MAX(PRL9Konly_table[0], PRL7K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < PRL_TABLE_SZ; j++) {
					if (j <= PRL7K9K_table[0] && PRL7K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= PRL7Konly_table[0] && PRL7Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= PRL9Konly_table[0]  && PRL9Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= PRL7K9Kdiff_table[0]  && PRL7K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0xC9) {
				nv_num = *(buf+2);
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				if (*(buf+2) == 0x01 || *(buf+2) == 0x11)
					return  DM7K9K;
				else
					return  NO_DEF_ITEM;

			} else if (*(buf+1) == 0x29) {
				max_item = MAX(MAX(M297K9K_table[0], M297Konly_table[0]),
				 MAX(M299Konly_table[0], M297K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < M29_TABLE_SZ; j++) {
					if (j <= M297K9K_table[0] && M297K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= M297Konly_table[0] && M297Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= M299Konly_table[0]  && M299Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= M297K9Kdiff_table[0]  && M297K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0x41 || *(buf+1) == 0x0C || *(buf+1) == 0x40) {
				return  DM7K9K;
			} else if (*(buf+1) == 0x00 || *(buf+1) == 0xCD || *(buf+1) == 0xD8) {
				return  DM7KONLY;
			} else if (*(buf+1) == 0xDF) {
				return  DM9KONLY;
			} else if (*(buf+1) == 0x4B && *(buf+2) == 0x0D) {
				return  DM7KONLY;
			} else
				printk("%s:id = 0x%x no default routing path\n", __func__, *(buf+1));
				return NO_DEF_ID;
		} else {
				/*printk("%s: not HPST_PREFIX id = 0x%x route to USB!!!\n", __func__, *buf);*/
				return NO_PST;
		}



}
#endif
#if EPST_FUN
int decode_encode_hdlc(void*data, int *len, unsigned char *buf_hdlc, int remove, int pos)
{
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	struct diag_hdlc_decode_type hdlc;
  unsigned char *buf_9k = NULL;
  int ret;


	buf_9k = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
	if (!buf_9k) {
		printk(KERN_ERR "%s:out of memory\n", __func__);
		return -ENOMEM;
	}

	hdlc.dest_ptr = buf_9k;
	hdlc.dest_size = USB_MAX_OUT_BUF;
	hdlc.src_ptr = data;
	hdlc.src_size = *len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = diag_hdlc_decode(&hdlc);
	if (!ret) {
		printk(KERN_ERR "Packet dropped due to bad HDLC coding/CRC\n");
		kfree(buf_9k);
		return -EINVAL;
	}
	if (remove)
		*((char *)buf_9k+pos) = (*((char *)buf_9k+pos) ^ 0x80);
	else
		*((char *)buf_9k+pos) = (*((char *)buf_9k+pos) | 0x80);


	send.state = DIAG_STATE_START;
	send.pkt = hdlc.dest_ptr;
	send.last = (void *)(hdlc.dest_ptr + hdlc.dest_idx - 4);
	send.terminate = 1;
	enc.dest = buf_hdlc;
	enc.dest_last = (void *)(buf_hdlc + 2*hdlc.dest_idx  - 3);
	diag_hdlc_encode(&send, &enc);

	print_hex_dump(KERN_DEBUG, "encode Data"
	, 16, 1, DUMP_PREFIX_ADDRESS, buf_hdlc, hdlc.dest_idx, 1);

	*len = hdlc.dest_idx;

	kfree(buf_9k);
	return 0;


}
int checkcmd_modem_epst(unsigned char *buf)
{
	int j;
	uint16_t nv_num;
	uint16_t max_item;

		if (*buf == EPST_PREFIX) {
			if (*(buf+1) == 0x26 || *(buf+1) == 0x27) {
				max_item = MAX(MAX(nv7K9K_table[0], nv7Konly_table[0]),
				 MAX(nv9Konly_table[0], nv7K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < NV_TABLE_SZ; j++) {
					if (j <= nv7K9K_table[0] && nv7K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= nv7Konly_table[0] && nv7Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= nv9Konly_table[0]  && nv9Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= nv7K9Kdiff_table[0]  && nv7K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0x48 || *(buf+1) == 0x49) {
				max_item = MAX(MAX(PRL7K9K_table[0], PRL7Konly_table[0]),
				 MAX(PRL9Konly_table[0], PRL7K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < PRL_TABLE_SZ; j++) {
					if (j <= PRL7K9K_table[0] && PRL7K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= PRL7Konly_table[0] && PRL7Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= PRL9Konly_table[0]  && PRL9Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= PRL7K9Kdiff_table[0]  && PRL7K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0xC9) {
				nv_num = *(buf+2);
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				if (*(buf+2) == 0x01 || *(buf+2) == 0x11)
					return  DM7K9K;
				else
					return  NO_DEF_ITEM;

			} else if (*(buf+1) == 0x29) {
				max_item = MAX(MAX(M297K9K_table[0], M297Konly_table[0]),
				 MAX(M299Konly_table[0], M297K9Kdiff_table[0]));
				nv_num = *((uint16_t *)(buf+2));
				printk("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
				for (j = 1; j < M29_TABLE_SZ; j++) {
					if (j <= M297K9K_table[0] && M297K9K_table[j] == nv_num)
						return  DM7K9K;
					if (j <= M297Konly_table[0] && M297Konly_table[j] == nv_num)
						return  DM7KONLY;
					if (j <= M299Konly_table[0]  && M299Konly_table[j] == nv_num)
						return  DM9KONLY;
					if (j <= M297K9Kdiff_table[0]  && M297K9Kdiff_table[j] == nv_num)
						return  DM7K9KDIFF;
					if (j > max_item)
						break;
				}
				return  NO_DEF_ITEM;
			} else if (*(buf+1) == 0x41 || *(buf+1) == 0x0C || *(buf+1) == 0x40) {
				return  DM7K9K;
			} else if (*(buf+1) == 0x00 || *(buf+1) == 0xCD || *(buf+1) == 0xD8
				|| *(buf+1) == 0x35 || *(buf+1) == 0x36) {
				return  DM7KONLY;
			} else if (*(buf+1) == 0xDF) {
				return  DM9KONLY;
			} else if (*(buf+1) == 0x4B && *(buf+2) == 0x0D) {
				return  DM7KONLY;
			} else
				printk("%s:id = 0x%x no default routing path\n", __func__, *(buf+1));
				return NO_DEF_ID;
		} else {
				/*rintk("%s: not EPST_PREFIX id = 0x%x route to USB!!!\n", __func__, *buf);*/
				return NO_PST;
		}



}
int modem_to_userspace(void *buf, int r, int type, int is9k)
{

	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	unsigned char value;

	if (!ctxt->diag2arm9_opened)
		return 0;
	req = req_get(ctxt, &ctxt->rx_arm9_idle);
	if (!req) {
		printk(KERN_ERR "There is no enough request to ARM11!!\n");
		return 0;
	}
	memcpy(req->buf, buf, r);


	if (type == DM7K9KDIFF) {
		value = *((uint8_t *)req->buf+1);
		if ((value == 0x27) || (value == 0x26)) {
			if (is9k == 1) {
				decode_encode_hdlc(buf, &r, req->buf, 0, 3);
			}
		}
	} else if (type == NO_DEF_ID) {
	/*in this case, cmd may reply error message*/
		value = *((uint8_t *)req->buf+2);
		printk(KERN_ERR "%s:check error cmd=0x%x message=ox%x\n", __func__
		, value, *((uint8_t *)req->buf+1));
		if ((value == 0x27) || (value == 0x26)) {
			if (is9k == 1) {
				decode_encode_hdlc(buf, &r, req->buf, 0, 4);
			}
		}
	}


	if (is9k == 1)
	print_hex_dump(KERN_DEBUG, "DM Read Packet Data"
					       " from 9k radio (first 16 Bytes): ", 16, 1, DUMP_PREFIX_ADDRESS, req->buf, 16, 1);
	else
	print_hex_dump(KERN_DEBUG, "DM Read Packet Data"
					       " from 7k radio (first 16 Bytes): ", 16, 1, DUMP_PREFIX_ADDRESS, req->buf, 16, 1);
	ctxt->rx_count += r;
	req->actual = r;
	req_put(ctxt, &ctxt->rx_arm9_done, req);
	wake_up(&ctxt->read_arm9_wq);
	return 1;
}

static long diag2arm9_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	unsigned long flags;
	uint16_t temp_nv_table[NV_TABLE_SZ];
	int table_size;
	uint16_t *table_ptr;

	if (_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC)
		return -ENOTTY;

	printk(KERN_INFO "%s:%s(parent:%s): tgid=%d\n", __func__,
	current->comm, current->parent->comm, current->tgid);

	switch (cmd) {

	case USB_DIAG_NV_7K9K_SET:
		printk(KERN_INFO "USB_DIAG_NV_7K9K_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7K9K_table;
		break;
	case USB_DIAG_NV_7KONLY_SET:
		printk(KERN_INFO "USB_DIAG_NV_7KONLY_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7Konly_table;
		break;
	case USB_DIAG_NV_9KONLY_SET:
		printk(KERN_INFO "USB_DIAG_NV_9KONLY_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv9Konly_table;
		break;
	case USB_DIAG_NV_7K9KDIFF_SET:
		printk(KERN_INFO "USB_DIAG_NV_7K9KDIFF_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7K9Kdiff_table;
		break;

	case USB_DIAG_PRL_7K9K_SET:
		printk(KERN_INFO "USB_DIAG_PRL_7K9K_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7K9K_table;
		break;
	case USB_DIAG_PRL_7KONLY_SET:
		printk(KERN_INFO "USB_DIAG_PRL_7KONLY_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7Konly_table;
		break;
	case USB_DIAG_PRL_9KONLY_SET:
		printk(KERN_INFO "USB_DIAG_PRL_9KONLY_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL9Konly_table;
		break;
	case USB_DIAG_PRL_7K9KDIFF_SET:
		printk(KERN_INFO "USB_DIAG_PRL_7K9KDIFF_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7K9Kdiff_table;
		break;

	case USB_DIAG_M29_7K9K_SET:
		printk(KERN_INFO "USB_DIAG_M29_7K9K_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297K9K_table;
		break;

	case USB_DIAG_M29_7KONLY_SET:
		printk(KERN_INFO "USB_DIAG_M29_7KONLY_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297Konly_table;
		break;
	case USB_DIAG_M29_9KONLY_SET:
		printk(KERN_INFO "USB_DIAG_M29_9KONLY_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M299Konly_table;
		break;
	case USB_DIAG_M29_7K9KDIFF_SET:
		printk(KERN_INFO "USB_DIAG_M29_7K9KDIFF_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297K9Kdiff_table;
		break;

	default:
		return -ENOTTY;
	}

		if (copy_from_user(temp_nv_table, (uint8_t *)argp, (table_size*2)))
			return -EFAULT;
		printk(KERN_INFO "%s:input %d item\n", __func__, temp_nv_table[0]);
		if (temp_nv_table[0] > table_size)
			return -EFAULT;

		spin_lock_irqsave(&ctxt->req_lock, flags);
		memcpy((uint8_t *)table_ptr, (uint8_t *)&temp_nv_table[0], (temp_nv_table[0]+1)*2);
		print_hex_dump(KERN_DEBUG, "TABLE Data: ", 16, 1,
		DUMP_PREFIX_ADDRESS, table_ptr, (*table_ptr+1)*2, 1);
		spin_unlock_irqrestore(&ctxt->req_lock, flags);



	return 0;
}
static int diag2arm9_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;
	int rc = 0;
	int n;

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

	diag_smd_enable("diag2arm9_open", 1);

done:
	mutex_unlock(&ctxt->diag2arm9_lock);
	return rc;
}

static int diag2arm9_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct usb_request *req;


	mutex_lock(&ctxt->diag2arm9_lock);
	ctxt->diag2arm9_opened = false;
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
	int path;
  unsigned char *buf_9k = NULL;
	ctxt->ch = driver->ch;

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


		path = checkcmd_modem_epst(ctxt->toARM9_buf);

		switch (path) {
		case DM7K9K:
				printk(KERN_INFO "%s:DM7K9K sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				smd_write(ctxt->ch, ctxt->toARM9_buf, writed);

				if (sdio_diag_initialized) {
					buf_9k = kzalloc(writed, GFP_KERNEL);
					if (!buf_9k) {
						printk(KERN_ERR "%s:out of memory\n", __func__);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -ENOMEM;
					}
					memcpy(buf_9k, ctxt->toARM9_buf, writed);
					msm_sdio_diag_write((void *)buf_9k, writed);
					buf_9k = NULL;
				}
				break;
		case DM9KONLY:
				printk(KERN_INFO "%s:DM9KONLY sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				if (sdio_diag_initialized) {
					buf_9k = kzalloc(writed, GFP_KERNEL);
					if (!buf_9k) {
						printk(KERN_ERR "%s:out of memory\n", __func__);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -ENOMEM;
					}
					memcpy(buf_9k, ctxt->toARM9_buf, writed);
					msm_sdio_diag_write((void *)buf_9k, writed);
					buf_9k = NULL;
				}
				break;
		case DM7K9KDIFF:
				printk(KERN_INFO "%s:DM7K9KDIFF sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				if (((ctxt->toARM9_buf[3] & 0x80) == 0x80) && sdio_diag_initialized) {
					printk(KERN_INFO "%s:DM7K9KDIFF to 9K\n", __func__);

					buf_9k = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
					if (!buf_9k) {
						printk(KERN_ERR "%s:out of memory\n", __func__);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -ENOMEM;
					}
					if (decode_encode_hdlc(ctxt->toARM9_buf, &writed, buf_9k, 1, 3)) {
						kfree(buf_9k);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -EINVAL;
					}
					msm_sdio_diag_write((void *)buf_9k, writed);
					buf_9k = NULL;

				} else {
					printk(KERN_INFO "%s:DM7K9KDIFF to 7K\n", __func__);
					smd_write(ctxt->ch, ctxt->toARM9_buf, writed);
				}
				break;
		case DM7KONLY:
				printk(KERN_INFO "%s:DM7KONLY sdio=%d\n", __func__, sdio_diag_initialized);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
				smd_write(ctxt->ch, ctxt->toARM9_buf, writed);
				break;
		case NO_DEF_ID:
		case NO_DEF_ITEM:
		default:
				printk(KERN_INFO "%s:no default routing path\n", __func__);
				print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, ctxt->toARM9_buf, writed, 1);
		}

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
	printk(KERN_INFO "%s\n", __func__);
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
		print_hex_dump(KERN_DEBUG, "DM Packet Data"
		" read from radio ", 16, 1, DUMP_PREFIX_ADDRESS, req->buf, req->actual, 1);
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
	.unlocked_ioctl = diag2arm9_ioctl,
};

static struct miscdevice diag2arm9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag_arm9",
	.fops = &diag2arm9_fops,
};


#endif



static inline struct diag_context *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct diag_context, function);
}

static void diag_function_unbind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct diag_context *ctxt = func_to_dev(f);

	if (!ctxt)
		return;
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);

#if USB_TO_USERSPACE
	misc_deregister(&htc_diag_device_fops);
#endif
#if EPST_FUN
	misc_deregister(&diag2arm9_device);
#endif
	ctxt->tx_count = ctxt->rx_count = 0;
}
static void usb_config_work_func(struct work_struct *work)
{
	struct diag_context *ctxt = &_context;
	if ((ctxt->operations) &&
		(ctxt->operations->diag_connect))
			ctxt->operations->diag_connect();
	/*send serial number to A9 sw download, only if serial_number
	* is not null and i_serial_number is non-zero
	*/
#if 0
	if (ctxt->serial_number && ctxt->i_serial_number) {
		msm_hsusb_is_serial_num_null(FALSE);
		msm_hsusb_send_serial_number(ctxt->serial_number);
	} else
		msm_hsusb_is_serial_num_null(TRUE);
	/* Send product ID to A9 for software download*/
	if (ctxt->product_id)
		msm_hsusb_send_productID(ctxt->product_id);
#endif
}

static int diag_function_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct diag_context *ctxt = func_to_dev(f);
	struct usb_ep      *ep;
	int status = -ENODEV;

	if (!ctxt)
		return status;

	ctxt->cdev = cdev;
	intf_desc.bInterfaceNumber =  usb_interface_id(c, f);

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_in_desc);
	if (!ep)
		goto fail;
	ctxt->in = ep;
	ep->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_bulk_out_desc);
	if (!ep)
		goto fail;
	ctxt->out = ep;
	ep->driver_data = cdev;

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

#if USB_TO_USERSPACE
	misc_register(&htc_diag_device_fops);
#endif
#if EPST_FUN
	misc_register(&diag2arm9_device);
#endif
	ctxt->tx_count = ctxt->rx_count = 0;
	return 0;
fail:
	if (ctxt->out)
		ctxt->out->driver_data = NULL;
	if (ctxt->in)
		ctxt->in->driver_data = NULL;

	return status;

}
static int diag_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct diag_context  *dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	unsigned long flags;
	int status = -ENODEV;
#if USB_TO_USERSPACE
	/*struct usb_request *req;*/
#endif

	if (!dev)
		return status;

	dev->in_desc = ep_choose(cdev->gadget,
			&hs_bulk_in_desc, &fs_bulk_in_desc);
	dev->out_desc = ep_choose(cdev->gadget,
			&hs_bulk_out_desc, &fs_bulk_in_desc);
	usb_ep_enable(dev->in, dev->in_desc);
	usb_ep_enable(dev->out, dev->out_desc);
	dev->i_serial_number = cdev->desc.iSerialNumber;
	dev->product_id   = cdev->desc.idProduct;
	schedule_work(&dev->diag_work);

	spin_lock_irqsave(&dev_lock , flags);
	dev->diag_configured = 1;
	spin_unlock_irqrestore(&dev_lock , flags);

	dev->online = !dev->function.disabled;
#if USB_TO_USERSPACE

	/* recycle unhandled rx reqs to user if any */
	/*while ((req = req_get(dev, &dev->rx_req_user)))
		req_put(dev, &dev->rx_req_idle, req);*/

	wake_up(&dev->read_wq);
#endif

	return 0;
}
static void diag_function_disable(struct usb_function *f)
{
	struct diag_context  *dev = func_to_dev(f);
	unsigned long flags;

	printk(KERN_INFO "diag_function_disable\n");

	dev->online = 0;
#if USB_TO_USERSPACE
	wake_up(&dev->read_wq);

#endif

	spin_lock_irqsave(&dev_lock , flags);
	dev->diag_configured = 0;
	spin_unlock_irqrestore(&dev_lock , flags);

	if (dev->in) {
		usb_ep_fifo_flush(dev->in);
		usb_ep_disable(dev->in);
		dev->in->driver_data = NULL;
	}
	if (dev->out) {
		usb_ep_fifo_flush(dev->out);
		usb_ep_disable(dev->out);
		dev->out->driver_data = NULL;
	}
	if ((dev->operations) && (driver->usb_connected) &&
		(dev->operations->diag_disconnect))
			dev->operations->diag_disconnect();
}
int diag_usb_register(struct diag_operations *func)
{
	struct diag_context *ctxt = &_context;
	unsigned long flags;
	int connected;

	if (func == NULL) {
		printk(KERN_ERR "%s:registering"
				"diag char operations NULL\n", __func__);
		return -1;
	}
	ctxt->operations = func;
	spin_lock_irqsave(&dev_lock , flags);
	connected = ctxt->diag_configured;
	spin_unlock_irqrestore(&dev_lock , flags);

	if (connected)
		if ((ctxt->operations) &&
			(ctxt->operations->diag_connect))
				ctxt->operations->diag_connect();
	return 0;
}
EXPORT_SYMBOL(diag_usb_register);

int diag_usb_unregister(void)
{
	struct diag_context *ctxt = &_context;

	ctxt->operations = NULL;
	return 0;
}
EXPORT_SYMBOL(diag_usb_unregister);

int diag_open(int num_req)
{
	struct diag_context *ctxt = &_context;
	struct diag_req_entry *write_entry;
	struct diag_req_entry *read_entry;
	int i = 0;

	for (i = 0; i < num_req; i++) {
		write_entry = diag_alloc_req_entry(ctxt->in, 0, GFP_KERNEL);
		if (write_entry) {
			write_entry->usb_req->complete = diag_write_complete;
			list_add(&write_entry->re_entry,
					&ctxt->dev_write_req_list);
		} else
			goto write_error;
	}

	for (i = 0; i < num_req ; i++) {
		read_entry = diag_alloc_req_entry(ctxt->out, 0 , GFP_KERNEL);
		if (read_entry) {
			read_entry->usb_req->complete = diag_read_complete;
			list_add(&read_entry->re_entry ,
					&ctxt->dev_read_req_list);
		} else
			goto read_error;
		}
	return 0;
read_error:
	printk(KERN_ERR "%s:read requests allocation failure\n", __func__);
	while (!list_empty(&ctxt->dev_read_req_list)) {
		read_entry = list_entry(ctxt->dev_read_req_list.next,
				struct diag_req_entry, re_entry);
		list_del(&read_entry->re_entry);
		diag_free_req_entry(ctxt->out, read_entry);
	}
write_error:
	printk(KERN_ERR "%s: write requests allocation failure\n", __func__);
	while (!list_empty(&ctxt->dev_write_req_list)) {
		write_entry = list_entry(ctxt->dev_write_req_list.next,
				struct diag_req_entry, re_entry);
		list_del(&write_entry->re_entry);
		diag_free_req_entry(ctxt->in, write_entry);
	}
	return -ENOMEM;
}
EXPORT_SYMBOL(diag_open);

void diag_close(void)
{
	struct diag_context *ctxt = &_context;
	struct diag_req_entry *req_entry;
	/* free write requests */

	while (!list_empty(&ctxt->dev_write_req_list)) {
		req_entry = list_entry(ctxt->dev_write_req_list.next,
				struct diag_req_entry, re_entry);
		list_del(&req_entry->re_entry);
		diag_free_req_entry(ctxt->in, req_entry);
	}
	/* free read requests */
	while (!list_empty(&ctxt->dev_read_req_list)) {
		req_entry = list_entry(ctxt->dev_read_req_list.next,
				struct diag_req_entry, re_entry);
		list_del(&req_entry->re_entry);
		diag_free_req_entry(ctxt->out, req_entry);
	}
	return;
}
EXPORT_SYMBOL(diag_close);

static void diag_free_req_entry(struct usb_ep *ep,
		struct diag_req_entry *req)
{
	if (req) {
		if (ep && req->usb_req)
			usb_ep_free_request(ep, req->usb_req);
		kfree(req);
	}
}

static struct diag_req_entry *diag_alloc_req_entry(struct usb_ep *ep,
		unsigned len, gfp_t kmalloc_flags)
{
	struct diag_req_entry *req;

	req = kmalloc(sizeof(struct diag_req_entry), kmalloc_flags);
	if (req == NULL)
		return NULL;

	req->usb_req  =  usb_ep_alloc_request(ep, GFP_KERNEL);
	if (req->usb_req == NULL) {
		kfree(req);
		return NULL;
	}
	req->usb_req->context = req;
	return req;
}

int diag_read(struct diag_request *d_req)
{
	unsigned long flags;
	struct usb_request *req = NULL;
	struct diag_req_entry *req_entry = NULL;
	struct diag_context *ctxt = &_context;

	spin_lock_irqsave(&dev_lock , flags);
	if (!ctxt->diag_configured) {
		spin_unlock_irqrestore(&dev_lock , flags);
		return -EIO;
	}
	if (!list_empty(&ctxt->dev_read_req_list)) {
		req_entry = list_entry(ctxt->dev_read_req_list.next ,
				struct diag_req_entry , re_entry);
		req_entry->diag_request = d_req;
		req = req_entry->usb_req;
		list_del(&req_entry->re_entry);
	}
	spin_unlock_irqrestore(&dev_lock , flags);
	if (req) {
		req->buf = d_req->buf;
		req->length = d_req->length;
		if (usb_ep_queue(ctxt->out, req, GFP_ATOMIC)) {
			/* If error add the link to the linked list again. */
			spin_lock_irqsave(&dev_lock , flags);
			list_add_tail(&req_entry->re_entry ,
					&ctxt->dev_read_req_list);
			spin_unlock_irqrestore(&dev_lock , flags);
			printk(KERN_ERR "%s:can't queue request\n", __func__);
			return -EIO;
		}
	} else {
		printk(KERN_ERR
				"%s:no requests avialable\n", __func__);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(diag_read);

int diag_write(struct diag_request *d_req)
{
	unsigned long flags;
	struct usb_request *req = NULL;
	struct diag_req_entry *req_entry = NULL;
	struct diag_context *ctxt = &_context;
	spin_lock_irqsave(&dev_lock , flags);
	if (!ctxt->diag_configured) {
		spin_unlock_irqrestore(&dev_lock , flags);
		return -EIO;
	}
	if (!list_empty(&ctxt->dev_write_req_list)) {
		req_entry = list_entry(ctxt->dev_write_req_list.next ,
				struct diag_req_entry , re_entry);
		req_entry->diag_request = d_req;
		req = req_entry->usb_req;
		list_del(&req_entry->re_entry);
	}
	spin_unlock_irqrestore(&dev_lock, flags);
	if (req) {
		req->buf = d_req->buf;
		req->length = d_req->length;
		if (usb_ep_queue(ctxt->in, req, GFP_ATOMIC)) {
			/* If error add the link to linked list again*/
			spin_lock_irqsave(&dev_lock, flags);
			list_add_tail(&req_entry->re_entry ,
					&ctxt->dev_write_req_list);
			spin_unlock_irqrestore(&dev_lock, flags);
			printk(KERN_ERR "%s: cannot queue"
					" read request\n", __func__);
			return -EIO;
		}
	} else {
		printk(KERN_ERR	"%s: no requests available\n", __func__);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL(diag_write);

static void diag_write_complete(struct usb_ep *ep ,
		struct usb_request *req)
{
	struct diag_context *ctxt = &_context;
	struct diag_req_entry *diag_req = req->context;
	struct diag_request *d_req = (struct diag_request *)
						diag_req->diag_request;
	unsigned long flags;
	/*char c;*/


	if (ctxt == NULL) {
		printk(KERN_ERR "%s: requesting"
				"NULL device pointer\n", __func__);
		return;
	}
#ifdef HTC_DIAG_DEBUG
		print_hex_dump(KERN_DEBUG, "to PC: ", 16, 1,
					   DUMP_PREFIX_ADDRESS, req->buf, req->actual, 1);
#endif
	if (req->status == WRITE_COMPLETE) {
		if ((req->length >= ep->maxpacket) &&
				((req->length % ep->maxpacket) == 0)) {
			req->length = 0;
			/* req->device = ctxt; */
			d_req->actual = req->actual;
			d_req->status = req->status;
			/* Queue zero length packet */
			usb_ep_queue(ctxt->in, req, GFP_ATOMIC);
			return;
		}
	}
	spin_lock_irqsave(&dev_lock, flags);
	list_add_tail(&diag_req->re_entry ,
			&ctxt->dev_write_req_list);
	if (req->length != 0) {
		d_req->actual = req->actual;
		d_req->status = req->status;
	}
	spin_unlock_irqrestore(&dev_lock , flags);

	if ((ctxt->operations) &&
		(ctxt->operations->diag_char_write_complete))
			ctxt->operations->diag_char_write_complete(
				d_req);
}
static void diag_read_complete(struct usb_ep *ep ,
		struct usb_request *req)
{
	 struct diag_context *ctxt = &_context;
	 struct diag_req_entry *diag_req = req->context;
	 struct diag_request *d_req = (struct diag_request *)
							diag_req->diag_request;
	 unsigned long flags;
	unsigned int cmd_id;

	if (ctxt == NULL) {
		printk(KERN_ERR "%s: requesting"
				"NULL device pointer\n", __func__);
		return;
	}
	spin_lock_irqsave(&dev_lock, flags);
	list_add_tail(&diag_req->re_entry ,
			&ctxt->dev_read_req_list);
	d_req->actual = req->actual;
	d_req->status = req->status;
	spin_unlock_irqrestore(&dev_lock, flags);
#ifdef HTC_DIAG_DEBUG
		print_hex_dump(KERN_DEBUG, "from PC: ", 16, 1,
					   DUMP_PREFIX_ADDRESS, req->buf, req->actual, 1);
#endif
#if USB_TO_USERSPACE
		cmd_id = *((unsigned short *)req->buf);
		if (if_route_to_userspace(ctxt, cmd_id)) {
			req_put(ctxt, &ctxt->rx_req_user, req);
			wake_up(&ctxt->read_wq);
			driver->nohdlc = 1;
		} else
			driver->nohdlc = 0;
#endif

	if ((ctxt->operations) &&
		(ctxt->operations->diag_char_read_complete))
			ctxt->operations->diag_char_read_complete(
				d_req);
}

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

int diag_bind_config(struct usb_configuration *c)
{
	struct diag_context *ctxt = &_context;
	int ret;

	printk(KERN_INFO "diag_bind_config\n");

	ret = usb_string_id(c->cdev);
	if (ret < 0)
		return ret;
	diag_string_defs[0].id = ret;
	intf_desc.iInterface = ret;

	ctxt->function.name = "diag";
	ctxt->function.strings = diag_strings;
	ctxt->function.descriptors = fs_diag_desc;
	ctxt->function.hs_descriptors = hs_diag_desc;
	ctxt->function.bind = diag_function_bind;
	ctxt->function.unbind = diag_function_unbind;
	ctxt->function.set_alt = diag_function_set_alt;
	ctxt->function.disable = diag_function_disable;

	INIT_LIST_HEAD(&ctxt->dev_read_req_list);
	INIT_LIST_HEAD(&ctxt->dev_write_req_list);
	INIT_WORK(&ctxt->diag_work, usb_config_work_func);
	ctxt->function.disabled = !_context.function_enable;
	smsc251x_set_diag_boot_flag(_context.function_enable);
	if (!ctxt->function.disabled)
		diag_smd_enable("diag_bind_config", 1);
#if USB_TO_USERSPACE

	spin_lock_init(&ctxt->req_lock);
	mutex_init(&ctxt->user_lock);
	INIT_LIST_HEAD(&ctxt->rx_req_user);
	init_waitqueue_head(&ctxt->read_wq);
#endif

#if EPST_FUN
	INIT_LIST_HEAD(&ctxt->rx_arm9_idle);
	INIT_LIST_HEAD(&ctxt->rx_arm9_done);
	init_waitqueue_head(&ctxt->read_arm9_wq);
	mutex_init(&ctxt->diag2arm9_lock);
	mutex_init(&ctxt->diag2arm9_read_lock);
	mutex_init(&ctxt->diag2arm9_write_lock);

#endif
	ctxt->init_done = 1;
	return usb_add_function(c, &ctxt->function);
}

static struct android_usb_function diag_function = {
	.name = "diag",
	.bind_config = diag_bind_config
};


#if HTC_RADIO_ROUTING
static ssize_t show_radio_sw(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	return sprintf(buf, "show_hub_sw\n");
}

static ssize_t store_radio_sw(struct device *dev, struct device_attribute *attr,
		char *buf, size_t count)
{
	char *buffer, *endptr;


	buffer = (char *)buf;
	modem_7_9_sw = simple_strtoull(buffer, &endptr, 16);

	return count;
}

static DEVICE_ATTR(radio_sw, 0644, show_radio_sw, store_radio_sw);

static void diag_plat_release(struct device *dev) {}

static struct platform_device diag_plat_device = {
	.name		= "f_diag",
	.id		= -1,
	.dev		= {
		.release	= diag_plat_release,
	},
};
#endif
static int __init  diag_function_init(void)
{
	/*struct diag_context *dev = &_context;
	int ret;
*/
	printk(KERN_INFO "%s\n", __func__);

#if HTC_RADIO_ROUTING
	platform_device_register(&diag_plat_device);
	if (device_create_file(&(diag_plat_device.dev), &dev_attr_radio_sw) != 0) {
		printk(KERN_ERR "diag dev_attr_diag_radio_switch failed");
	}
#endif
	android_register_function(&diag_function);
	return 0;
}

module_init(diag_function_init);

static int diag_set_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);
	printk("%s: %d\n", __func__, enabled);

	if (_context.cdev)
		android_enable_function(&_context.function, enabled);

	_context.function_enable = !!enabled;
/*don't register sdio diag driver when radio flag  is 8 8000000*/
	sdio_diag_enable = !(_context.function_enable);
/*	diag_smd_enable("diag_set_enabled", enabled);*/
	return 0;
}

static int diag_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + !_context.function.disabled;
	return 1;
}
module_param_call(enabled, diag_set_enabled, diag_get_enabled, NULL, 0664);

static int diag_get_tx_rx_count(char *buffer, struct kernel_param *kp)
{
	struct diag_context *ctxt = &_context;

	return sprintf(buffer, "tx: %llu bytes, rx: %llu bytes",
	ctxt->tx_count, ctxt->rx_count);
}
module_param_call(tx_rx_count, NULL, diag_get_tx_rx_count, NULL, 0444);



