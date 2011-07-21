/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */



#include <linux/delay.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>


#include <mach/sdio_al.h>
#include <mach/usbdiag.h>
#include <linux/diagchar.h>
#include "../../../../drivers/char/diag/diagchar.h"
#include "../../../../drivers/usb/gadget/f_diag.h"

/* number of tx/rx requests to allocate */
#define WR_REQ_NUM 30

#define SDIODIAG_DEBUG


static int msmt_sdio_diag_debug_mask;
int sdio_diag_initialized;
int sdio_diag_enable;
module_param_named(debug_mask, msmt_sdio_diag_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

#if defined(SDIODIAG_DEBUG)
static uint32_t msm_sdio_diag_read_cnt;
static uint32_t msm_sdio_diag_write_cnt;

#define HTCDBG(x...) do {		                \
		if (msmt_sdio_diag_debug_mask)	\
			printk(KERN_DEBUG x);	\
	} while (0)

#define HTCDBG_INC_READ_CNT(x) do {	                       \
		if (msmt_sdio_diag_debug_mask) {                \
			msm_sdio_diag_read_cnt += (x);                \
			printk(KERN_DEBUG "%s: total read bytes %u\n", \
		     __func__, msm_sdio_diag_read_cnt);     \
		} 																						\
	} while (0)

#define HTCDBG_INC_WRITE_CNT(x)  do {	                          \
		if (msmt_sdio_diag_debug_mask) {                \
			msm_sdio_diag_write_cnt += (x);                  \
			printk(KERN_DEBUG "%s: total written bytes %u\n", \
		     __func__, msm_sdio_diag_write_cnt);	  \
		} 																						\
	} while (0)
#else
#define HTCDBG(x...) do { } while (0)
#define HTCDBG_INC_READ_CNT(x...) do { } while (0)
#define HTCDBG_INC_WRITE_CNT(x...) do { } while (0)
#endif

struct diag_ch_info {

	wait_queue_head_t wait_q;
	struct list_head write_req_idle;
	struct list_head write_req_task;
	spinlock_t req_lock;
};


struct sdio_diag_req {
	struct list_head list;
	void *data;
	int len;

};


static struct sdio_channel *sdio_diag_ch;
static struct diag_ch_info diag_ch;
struct wake_lock sdio_diag_ch_wakelock;


static struct sdio_diag_req sdio_diag_write_req[WR_REQ_NUM];


void sdio_diag_read_data(struct work_struct *work);
void __diag_sdio_mdm_send_req(void);

static void sdio_diag_write_data(struct work_struct *work);
static DEFINE_MUTEX(sdio_wdiag_lock);
static DECLARE_WORK(work_sdio_diag_write, sdio_diag_write_data);

static struct workqueue_struct *sdio_diag_workqueue;



/* add a request to the tail of a list */
static void req_put(struct diag_ch_info *ctxt, struct list_head *head,
		struct sdio_diag_req *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->req_lock, flags);
}

/* remove a request from the head of a list */
static struct sdio_diag_req *req_get(struct diag_ch_info *ctxt,
		struct list_head *head)
{
	struct sdio_diag_req *req = 0;
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	if (!list_empty(head)) {
		req = list_first_entry(head, struct sdio_diag_req, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return req;
}


void sdio_diag_read_data(struct work_struct *work)
{
	__diag_sdio_mdm_send_req();
}

void __diag_sdio_mdm_send_req(void)
{
	void *buf = NULL;
	int *in_busy_ptr = NULL;
	struct diag_request *write_ptr_modem = NULL;
	int r;
	int type;
#if  HPST_FUN
	unsigned char value;
#endif
	if (!sdio_diag_initialized)
		return;

	if (!driver->in_busy_mdm_1) {
		buf = driver->usb_buf_in_mdm_1;
		write_ptr_modem = driver->usb_write_ptr_mdm_1;
		in_busy_ptr = &(driver->in_busy_mdm_1);
	} else if (!driver->in_busy_mdm_2) {
		buf = driver->usb_buf_in_mdm_2;
		write_ptr_modem = driver->usb_write_ptr_mdm_2;
		in_busy_ptr = &(driver->in_busy_mdm_2);
	} else {

		printk(KERN_DEBUG "%s:buf = 0x%x 1:%d 2:%d\n", __func__,
		(int) buf, driver->in_busy_mdm_1, driver->in_busy_mdm_2);

	}
	if (sdio_diag_ch && buf) {
		r = sdio_read_avail(sdio_diag_ch);

		if (r > USB_MAX_IN_BUF) {
			if (r < MAX_BUF_SIZE) {
				printk(KERN_ALERT "\n diag: SDIO DIAG sending in "
						   "packets upto %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				printk(KERN_ALERT "\n diag: SDIO DIAG sending in "
				"packets more than %d bytes", MAX_BUF_SIZE);
				return;
			}
		}

		if (r > 0) {
			if (!buf) {
				printk(KERN_INFO "Out of diagmem for MDM\n");
			} else {
				sdio_read(sdio_diag_ch, buf, r);

				if (msmt_sdio_diag_debug_mask) {
					switch (msmt_sdio_diag_debug_mask) {
					case 1:
						print_hex_dump(KERN_DEBUG, "Packet Data"
						" read from sdio diag (first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						break;
					case 2:
						print_hex_dump(KERN_DEBUG, "Packet Data"
						" read from sdio diag (first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						print_hex_dump(KERN_DEBUG, "Packet Data"
						" read from sdio diag (last 16 bytes) ", 16, 1, DUMP_PREFIX_ADDRESS, buf+r-16, 16, 1);
						break;
					default:
						print_hex_dump(KERN_DEBUG, "Packet Data"
						" read from sdio diag ", 16, 1, DUMP_PREFIX_ADDRESS, buf, r, 1);

					}
				}

#if  EPST_FUN
				type = checkcmd_modem_epst(buf);
				if (type) {
					modem_to_userspace(buf, r, type, 1);
					return;
				}
#endif
#if  HPST_FUN
				if (*((uint8_t *)buf) == HPST_PREFIX) {
					type = checkcmd_modem_hpst(buf);
					if (type == DM7K9KDIFF) {
						value = *((uint8_t *)buf+1);
						if ((value == 0x27) || (value == 0x26))
						decode_encode_hdlc(buf, &r, req->buf, 0, 3);
					} else if (type == NO_DEF_ID) {
					/*in this case, cmd may reply error message*/
						value = *((uint8_t *)buf+2);
						printk(KERN_ERR "%s:check error cmd=0x%x message=ox%x\n", __func__,
						 value, *((uint8_t *)buf+1));
						if ((value == 0x27) || (value == 0x26))
						decode_encode_hdlc(buf, &r, req->buf, 0, 4);
					}
				}
#endif
				if (driver->logging_mode == MEMORY_DEVICE_MODE) {
					write_ptr_modem->length = r;
					*in_busy_ptr = 1;
					diag_device_write(buf, MDM_DATA, write_ptr_modem);
					HTCDBG_INC_READ_CNT(r);
				}

			}
		}
	}
}

static void sdio_diag_write_data(struct work_struct *work)
{
	struct diag_ch_info *ctxt = &diag_ch;
	struct sdio_diag_req *req;
	int sz;
	int rc;
	mutex_lock(&sdio_wdiag_lock);
	sz = sdio_write_avail(sdio_diag_ch);

	req = req_get(ctxt, &ctxt->write_req_task);
	if (!req) {
		pr_err("%s: run out of req \n", __func__);
		BUG();
	}

	if (req->len <= sz) {
		rc = sdio_write(sdio_diag_ch, req->data, req->len);
		HTCDBG_INC_WRITE_CNT(req->len);
	} else
		printk("%s: req->len = %d > sz = %d\n", __func__, req->len , sz);

	if (req) {
		kfree(req->data);
		req_put(ctxt, &ctxt->write_req_idle, req);
	}
		mutex_unlock(&sdio_wdiag_lock);
}

void  msm_sdio_diag_write(void *data, int len)
{
	struct diag_ch_info *ctxt = &diag_ch;
	struct sdio_diag_req *req;

	if (!sdio_diag_initialized)
		return;

	req = req_get(ctxt, &ctxt->write_req_idle);
	if (!req) {
		pr_err("%s: run out of req \n", __func__);
		BUG();
	}
	else {

		req->data = data;
		req->len = len;
		if (msmt_sdio_diag_debug_mask) {
			switch (msmt_sdio_diag_debug_mask) {
			case 1:
				print_hex_dump(KERN_DEBUG, "Packet Data"
				" write to sdio diag (first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, req->data, 16, 1);
				break;
			case 2:
				print_hex_dump(KERN_DEBUG, "Packet Data"
				" write to sdio diag (first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, req->data, 16, 1);
				print_hex_dump(KERN_DEBUG, "Packet Data"
				" write to sdio diag (last 16 bytes) ", 16, 1, DUMP_PREFIX_ADDRESS, req->data+len-16, 16, 1);
				break;
			default:
				print_hex_dump(KERN_DEBUG, "Packet Data"
				" write to sdio diag ", 16, 1, DUMP_PREFIX_ADDRESS, req->data, len, 1);

			}
		}
		req_put(ctxt, &ctxt->write_req_task, req);
		queue_work(sdio_diag_workqueue, &work_sdio_diag_write);

	}


}

static void sdio_diag_notify(void *_dev, unsigned event)
{
	HTCDBG("%s: event %d notified\n", __func__, event);
	if (event == SDIO_EVENT_DATA_READ_AVAIL) {
		queue_work(driver->mdm_diag_workqueue, &(driver->diag_read_smd_mdm_work));
	}

}


static int sdio_diag_probe(struct platform_device *pdev)
{
	struct diag_ch_info *ctxt = &diag_ch;
	struct sdio_diag_req *req;
	int n;
	int rc = -1;

	if (sdio_diag_initialized)
		return 0;

	/* is one thread gud enough for read and write? */
	sdio_diag_workqueue = create_singlethread_workqueue("sdio_diag");
	driver->mdm_diag_workqueue = sdio_diag_workqueue;
	if (!sdio_diag_workqueue)
		return -ENOMEM;

	wake_lock_init(&sdio_diag_ch_wakelock, WAKE_LOCK_SUSPEND,
		       "sdio_diag");

	spin_lock_init(&ctxt->req_lock);
	INIT_LIST_HEAD(&ctxt->write_req_idle);
	INIT_LIST_HEAD(&ctxt->write_req_task);

	for (n = 0; n < WR_REQ_NUM; n++) {
		/* allocate for write list*/
		req = &sdio_diag_write_req[n];
		req_put(ctxt, &ctxt->write_req_idle, req);
	}


	rc = sdio_open("SDIO_DIAG", &sdio_diag_ch, NULL, sdio_diag_notify);
	if (rc < 0) {
		pr_err("%s: sido open failed %d\n", __func__, rc);

		goto err_exit;
	}
	printk("%s SDIO_DIAG okay\n", __func__);
	sdio_diag_initialized = 1;
	return 0;

err_exit:
	destroy_workqueue(sdio_diag_workqueue);
	wake_lock_destroy(&sdio_diag_ch_wakelock);
	for (n = 0; n < WR_REQ_NUM; n++) {

		req = &sdio_diag_write_req[n];
		if (!(req->data)) {
			kfree(req->data);
		}
	}
	printk("%s SDIO_DIAG probe fail\n", __func__);

		return rc;
}

struct platform_driver sdio_diag_driver = {
	.probe		= sdio_diag_probe,
	.driver		= {
		.name	= "SDIO_DIAG",
		.owner	= THIS_MODULE,
	},
};

int __init sdio_diag_init(void)
{
	if (sdio_diag_enable)
		return platform_driver_register(&sdio_diag_driver);
	else
		return 0;
}

