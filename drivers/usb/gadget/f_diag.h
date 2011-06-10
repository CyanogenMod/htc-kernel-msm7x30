/* drivers/usb/gadget/f_diag.h
 *
 * Diag Function Device - Route DIAG frames between SMD and USB
 *
 * Copyright (C) 2008-2009 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#ifndef __F_DIAG_H
#define __F_DIAG_H

#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#define USB_TO_USERSPACE 1
#define EPST_FUN 1
#define HPST_FUN 0


struct diag_context {

	struct usb_composite_dev *cdev;

	struct usb_function function;
	struct usb_ep *out;
	struct usb_ep *in;
	struct usb_endpoint_descriptor  *in_desc;
	struct usb_endpoint_descriptor  *out_desc;
	/* linked list of read requets*/
	struct list_head dev_read_req_list;
	/* linked list of write requets*/
	struct list_head dev_write_req_list;
	struct diag_operations *operations;
	struct work_struct diag_work;
	unsigned diag_configured;
	unsigned char i_serial_number;
	char *serial_number;
	unsigned short  product_id;

#if USB_TO_USERSPACE

	spinlock_t req_lock;

	struct mutex user_lock;
#define ID_TABLE_SZ 20 /* keep this small */
	struct list_head rx_req_user;
	wait_queue_head_t read_wq;
	char *user_read_buf;
	uint32_t user_read_len;
	char *user_readp;
	bool opened;
	/* list of registered command ids to be routed to userspace */
	unsigned char id_table[ID_TABLE_SZ];

	smd_channel_t *ch;

	int online;
	int error;
#endif

#if EPST_FUN
#define SMD_MAX 8192


	unsigned char hdlc_buf[SMD_MAX];
	struct list_head rx_arm9_idle;
	struct list_head rx_arm9_done;

	/* for slate test */

	struct mutex diag2arm9_lock;
	struct mutex diag2arm9_read_lock;
	struct mutex diag2arm9_write_lock;
	bool diag2arm9_opened;
	unsigned char toARM9_buf[SMD_MAX];
	unsigned read_arm9_count;
	unsigned char *read_arm9_buf;
	wait_queue_head_t read_arm9_wq;
	struct usb_request *read_arm9_req;
#endif
	int is2ARM11;
	u64 tx_count; /* to smd */
	u64 rx_count; /* from smd */
	int init_done;
	int function_enable;

};

#define USB_DIAG_IOC_MAGIC 0xFF
#define USB_DIAG_FUNC_IOC_ENABLE_SET	_IOW(USB_DIAG_IOC_MAGIC, 1, int)
#define USB_DIAG_FUNC_IOC_ENABLE_GET	_IOR(USB_DIAG_IOC_MAGIC, 2, int)
#define USB_DIAG_FUNC_IOC_REGISTER_SET  _IOW(USB_DIAG_IOC_MAGIC, 3, char *)


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

#define NV_TABLE_SZ  128
#define M29_TABLE_SZ  10
#define PRL_TABLE_SZ  10

#define NO_PST 0
#define NO_DEF_ID 1
#define DM7K9K  2
#define DM7KONLY  3
#define DM9KONLY  4
#define DM7K9KDIFF  5
#define NO_DEF_ITEM  0xff

#define EPST_PREFIX 0xC8
#define HPST_PREFIX 0xF1


extern struct diag_context _context;
extern int sdio_diag_initialized;
extern int sdio_diag_enable;
int diag_function_add(struct usb_configuration *c, const char *);
int __init sdio_diag_init(void);
void diag_sdio_mdm_send_req(int context);
void msm_sdio_diag_write(void *data, int len);
void sdio_diag_read_data(struct work_struct *work);
int checkcmd_modem_epst(unsigned char *buf);
int checkcmd_modem_hpst(unsigned char *buf);
int modem_to_userspace(void *buf, int r, int cmdtype, int is9k);
#endif /* __F_DIAG_H */

