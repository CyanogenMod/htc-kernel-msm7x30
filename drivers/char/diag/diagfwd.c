/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/diagchar.h>
#include <mach/usbdiag.h>
#include <mach/msm_smd.h>
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagchar_hdlc.h"
#include <linux/pm_runtime.h>
#include "../../usb/gadget/f_diag.h"

static int diag7k_debug_mask;
module_param_named(debug_mask, diag7k_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

MODULE_DESCRIPTION("Diag Char Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");

int diag_debug_buf_idx;
unsigned char diag_debug_buf[1024];
/* Number of maximum USB requests that the USB layer should handle at
   one time. */
#define MAX_DIAG_USB_REQUESTS 12
static unsigned int buf_tbl_size = 8; /*Number of entries in table of buffers */

#define CHK_OVERFLOW(bufStart, start, end, length) \
((bufStart <= start) && (end - start >= length)) ? 1 : 0


void __diag_smd_send_req(void)
{
	void *buf = NULL;
	int *in_busy_ptr = NULL;
	struct diag_request *write_ptr_modem = NULL;
	int type;

	if (!driver->in_busy_1) {
		buf = driver->usb_buf_in_1;
		write_ptr_modem = driver->usb_write_ptr_1;
		in_busy_ptr = &(driver->in_busy_1);
	} else if (!driver->in_busy_2) {
		buf = driver->usb_buf_in_2;
		write_ptr_modem = driver->usb_write_ptr_2;
		in_busy_ptr = &(driver->in_busy_2);
	}

	if (driver->ch && buf) {
		int r = smd_read_avail(driver->ch);

		if (r > USB_MAX_IN_BUF) {
			if (r < MAX_BUF_SIZE) {
				printk(KERN_ALERT "\n diag: SMD sending in "
						   "packets upto %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				printk(KERN_ALERT "\n diag: SMD sending in "
				"packets more than %d bytes", MAX_BUF_SIZE);
				return;
			}
		}

		if (r > 0) {
			if (!buf)
				printk(KERN_INFO "Out of diagmem for a9\n");
			else {
				APPEND_DEBUG('i');
				smd_read(driver->ch, buf, r);

				if (diag7k_debug_mask) {
					switch (diag7k_debug_mask) {
					case 1:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K(first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						break;
					case 2:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K(first 16 bytes)", 16, 1, DUMP_PREFIX_ADDRESS, buf, 16, 1);
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K(last 16 bytes) ", 16, 1, DUMP_PREFIX_ADDRESS, buf+r-16, 16, 1);
						break;
					default:
						print_hex_dump(KERN_DEBUG, "Read Packet Data"
						" from 7K ", 16, 1, DUMP_PREFIX_ADDRESS, buf, r, 1);

					}
				}

#if  EPST_FUN
		type = checkcmd_modem_epst(buf);
		if (type) {
			modem_to_userspace(buf, r, type, 0);
			return;
		}

#endif

				APPEND_DEBUG('j');
				write_ptr_modem->length = r;
				*in_busy_ptr = 1;
				diag_device_write(buf, MODEM_DATA,
							 write_ptr_modem);
			}
		}
	}
}
int diag_device_write(void *buf, int proc_num, struct diag_request *write_ptr)
{
	int i, err = 0;

	if (driver->logging_mode == USB_MODE) {
		if (proc_num == APPS_DATA) {
			driver->usb_write_ptr_svc = (struct diag_request *)
			(diagmem_alloc(driver, sizeof(struct diag_request),
				 POOL_TYPE_USB_STRUCT));
			if (driver->usb_write_ptr_svc == NULL)
				return -ENOMEM;
			driver->usb_write_ptr_svc->length = driver->used;
			driver->usb_write_ptr_svc->buf = buf;
			err = diag_write(driver->usb_write_ptr_svc);
		} else if (proc_num == MODEM_DATA) {
			write_ptr->buf = buf;
#ifdef DIAG_DEBUG
				printk(KERN_INFO "writing data to USB,"
				"pkt length %d\n", write_ptr->length);
			print_hex_dump(KERN_DEBUG, "Written Packet Data to"
					   " USB: ", 16, 1, DUMP_PREFIX_ADDRESS,
					    buf, write_ptr->length, 1);
#endif
			err = diag_write(write_ptr);
		} else if (proc_num == QDSP_DATA) {
			write_ptr->buf = buf;
			err = diag_write(write_ptr);

		} else if (proc_num == MDM_DATA) {
				printk(KERN_INFO "%s:MDM_DATA \n", __func__);
			write_ptr->buf = buf;
			err = diag_write(write_ptr);
		}
		APPEND_DEBUG('k');
	} else if (driver->logging_mode == MEMORY_DEVICE_MODE) {
		if (proc_num == APPS_DATA) {
			for (i = 0; i < driver->poolsize_usb_struct; i++)
				if (driver->buf_tbl[i].length == 0) {
					driver->buf_tbl[i].buf = buf;
					driver->buf_tbl[i].length = driver->used;

#ifdef DIAG_DEBUG
					printk(KERN_INFO "\n ENQUEUE buf ptr"
						   " and length is %x , %d\n",
			(unsigned int)(driver->buf_tbl[i].buf), driver->buf_tbl[i].length);
#endif
					break;
				}

		for (i = 0; i < driver->num_clients; i++)
			if (driver->client_map[i].pid ==
						 driver->logging_process_id)
				break;
		if (i < driver->num_clients) {
			driver->data_ready[i] |= MEMORY_DEVICE_LOG_TYPE;
			wake_up_interruptible(&driver->wait_q);
		} else
			return -EINVAL;

	} else if (proc_num == MDM_DATA) {

		for (i = 0; i < driver->num_mdmclients; i++)
			if (driver->mdmclient_map[i].pid ==
						 driver->logging_process_id)
				break;
		if (i < driver->num_mdmclients) {
			driver->mdmdata_ready[i] |= MEMORY_DEVICE_LOG_TYPE;
			wake_up_interruptible(&driver->mdmwait_q);
		} else
			return -EINVAL;
	} else if (proc_num == MODEM_DATA) {
		for (i = 0; i < driver->num_clients; i++)
			if (driver->client_map[i].pid ==
						 driver->logging_process_id)
				break;
		if (i < driver->num_clients) {
			driver->data_ready[i] |= MEMORY_DEVICE_LOG_TYPE;
			wake_up_interruptible(&driver->wait_q);
		} else
			return -EINVAL;

	}
	} else if (driver->logging_mode == NO_LOGGING_MODE) {
		if (proc_num == MODEM_DATA) {
			driver->in_busy_1 = 0;
			driver->in_busy_2 = 0;
			queue_work(driver->diag_wq, &(driver->
							diag_read_smd_work));
		} else if (proc_num == QDSP_DATA) {
			driver->in_busy_qdsp_1 = 0;
			driver->in_busy_qdsp_2 = 0;
			queue_work(driver->diag_wq, &(driver->
						diag_read_smd_qdsp_work));
		}
		err = -1;
	}
    return err;
}

void __diag_smd_qdsp_send_req(void)
{
	void *buf = NULL;
	int *in_busy_qdsp_ptr = NULL;
	struct diag_request *write_ptr_qdsp = NULL;

	if (!driver->in_busy_qdsp_1) {
		buf = driver->usb_buf_in_qdsp_1;
		write_ptr_qdsp = driver->usb_write_ptr_qdsp_1;
		in_busy_qdsp_ptr = &(driver->in_busy_qdsp_1);
	} else if (!driver->in_busy_qdsp_2) {
		buf = driver->usb_buf_in_qdsp_2;
		write_ptr_qdsp = driver->usb_write_ptr_qdsp_2;
		in_busy_qdsp_ptr = &(driver->in_busy_qdsp_2);
	}

	if (driver->chqdsp && buf) {
		int r = smd_read_avail(driver->chqdsp);

		if (r > USB_MAX_IN_BUF) {
			if (r < MAX_BUF_SIZE) {
				printk(KERN_ALERT "\n diag: SMD sending in "
						   "packets upto %d bytes", r);
				buf = krealloc(buf, r, GFP_KERNEL);
			} else {
				printk(KERN_ALERT "\n diag: SMD sending in "
				"packets more than %d bytes", MAX_BUF_SIZE);
			return;
		}
		}
		if (r > 0) {
			if (!buf)
				printk(KERN_INFO "Out of diagmem for qdsp\n");
			else {
				APPEND_DEBUG('i');
					smd_read(driver->chqdsp, buf, r);
				APPEND_DEBUG('j');
				write_ptr_qdsp->length = r;
				*in_busy_qdsp_ptr = 1;
				diag_device_write(buf, QDSP_DATA,
							 write_ptr_qdsp);
			}
		}
	}
}

static void diag_print_mask_table(void)
{
/* Enable this to print mask table when updated */
#ifdef MASK_DEBUG
	int first;
	int last;
	uint8_t *ptr = driver->msg_masks;
	int i = 0;

	while (*(uint32_t *)(ptr + 4)) {
		first = *(uint32_t *)ptr;
		ptr += 4;
		last = *(uint32_t *)ptr;
		ptr += 4;
		printk(KERN_INFO "SSID %d - %d\n", first, last);
		for (i = 0 ; i <= last - first ; i++)
			printk(KERN_INFO "MASK:%x\n", *((uint32_t *)ptr + i));
		ptr += ((last - first) + 1)*4;

	}
#endif
}

void diag_update_msg_mask(int start, int end , uint8_t *buf, int disable)
{
	int found = 0;
	int first;
	int last;
	uint8_t *ptr = driver->msg_masks;
	uint8_t *ptr_buffer_start = &(*(driver->msg_masks));
	uint8_t *ptr_buffer_end = &(*(driver->msg_masks)) + MSG_MASK_SIZE;
	unsigned long flags;

	/*mutex_lock(&driver->diagchar_mutex);*/
	spin_lock_irqsave(&driver->diagchar_lock, flags);
	/* First SSID can be zero : So check that last is non-zero */

	while (*(uint32_t *)(ptr + 4)) {
		first = *(uint32_t *)ptr;
		ptr += 4;
		last = *(uint32_t *)ptr;
		ptr += 4;
		if (start >= first && start <= last) {
			ptr += (start - first)*4;
			if (end <= last)
				if (CHK_OVERFLOW(ptr_buffer_start, ptr,
						  ptr_buffer_end,
						  (((end - start)+1)*4))) {
					if (!disable)
						memcpy(ptr, buf , ((end - start)+1)*4);
					else
						memset(ptr, 0x0 , ((end - start)+1)*4);
				 } else
					printk(KERN_CRIT "Not enough"
							 " buffer space for"
							 " MSG_MASK\n");
			else
				printk(KERN_INFO "Unable to copy"
						 " mask change\n");

			found = 1;
			break;
		} else {
			ptr += ((last - first) + 1)*4;
		}
	}
	/* Entry was not found - add new table */
	if (!found) {
		if (CHK_OVERFLOW(ptr_buffer_start, ptr, ptr_buffer_end,
				  8 + ((end - start) + 1)*4)) {
			memcpy(ptr, &(start) , 4);
			ptr += 4;
			memcpy(ptr, &(end), 4);
			ptr += 4;
			memcpy(ptr, buf , ((end - start) + 1)*4);
		} else
			printk(KERN_CRIT " Not enough buffer"
					 " space for MSG_MASK\n");
	}

	/*mutex_unlock(&driver->diagchar_mutex);*/
	spin_unlock_irqrestore(&driver->diagchar_lock, flags);

	diag_print_mask_table();

}

static void diag_update_event_mask(uint8_t *buf, int toggle, int num_bits)
{

	uint8_t *ptr = driver->event_masks;
	uint8_t *temp = buf + 2;

	mutex_lock(&driver->diagchar_mutex);
	if (!toggle)
		memset(ptr, 0 , EVENT_MASK_SIZE);
	else
		if (CHK_OVERFLOW(ptr, ptr,
				 ptr+EVENT_MASK_SIZE,
				  num_bits/8 + 1))
			memcpy(ptr, temp , num_bits/8 + 1);
		else
			printk(KERN_CRIT "Not enough buffer space "
					 "for EVENT_MASK\n");
	mutex_unlock(&driver->diagchar_mutex);
}

static void diag_update_log_mask(uint8_t *buf, int num_items)
{
	uint8_t *ptr = driver->log_masks;
	uint8_t *temp = buf;

	mutex_lock(&driver->diagchar_mutex);
	if (CHK_OVERFLOW(ptr, ptr, ptr + LOG_MASK_SIZE,
				  (num_items+7)/8))
		memcpy(ptr, temp , (num_items+7)/8);
	else
		printk(KERN_CRIT " Not enough buffer space for LOG_MASK\n");
	mutex_unlock(&driver->diagchar_mutex);
}

static void diag_update_pkt_buffer(unsigned char *buf)
{

	unsigned char *ptr = driver->pkt_buf;
	unsigned char *temp = buf;

	mutex_lock(&driver->diagchar_mutex);
	if (CHK_OVERFLOW(ptr, ptr, ptr + PKT_SIZE, driver->pkt_length))
		memcpy(ptr, temp , driver->pkt_length);
	else
		printk(KERN_CRIT " Not enough buffer space for PKT_RESP\n");
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_userspace_clients(unsigned int type)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid != 0)
			driver->data_ready[i] |= type;
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_sleeping_process(int process_id)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid == process_id) {
			driver->data_ready[i] |= PKT_TYPE;
			break;
		}
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

static int diag_process_apps_pkt(unsigned char *buf, int len)
{
	uint16_t start;
	uint16_t end, subsys_cmd_code;
	int i, cmd_code, subsys_id;
	int packet_type = 1;
	unsigned char *temp = buf;

	/* event mask */

	if ((*buf == 0x60) && (*(++buf) == 0x0)) {
		diag_update_event_mask(buf, 0, 0);
		diag_update_userspace_clients(EVENT_MASKS_TYPE);
	}
	/* check for set event mask */
	else if (*buf == 0x82) {
		buf += 4;
		diag_update_event_mask(buf, 1, *(uint16_t *)buf);
		diag_update_userspace_clients(
		EVENT_MASKS_TYPE);
	}
	/* log mask */
	else if (*buf == 0x73) {
		buf += 4;
		if (*(int *)buf == 3) {
			buf += 8;
			diag_update_log_mask(buf+4, *(int *)buf);
			diag_update_userspace_clients(LOG_MASKS_TYPE);
		}
	}
	/* Check for set message mask  */
	else if ((*buf == 0x7d) && ((*(buf+1) == 0x4) || (*(buf+1) == 0x5))) {
/*		printk("len = %d\n", len);
		print_hex_dump(KERN_DEBUG, "Read Packet Data"
					       " from message mask: ", 16, 1, DUMP_PREFIX_ADDRESS, buf, len, 1);
*/
		if (*(buf+1) == 0x4) {
			buf += 2;
			start = *(uint16_t *)buf;
			buf += 2;
			end = *(uint16_t *)buf;
			buf += 4;
			diag_update_msg_mask((uint32_t)start, (uint32_t)end , buf, 0);
			diag_update_userspace_clients(MSG_MASKS_TYPE);
		} else {
			diag_update_msg_mask(0, 0x4e, buf, 1);
			diag_update_userspace_clients(MSG_MASKS_TYPE);
		}
	}
	/* Set all run-time masks
	if ((*buf == 0x7d) && (*(++buf) == 0x5)) {
		TO DO
	} */

	/* Check for registered clients and forward packet to user-space */
	else{
		cmd_code = (int)(*(char *)buf);
		temp++;
		subsys_id = (int)(*(char *)temp);
		temp++;
		subsys_cmd_code = *(uint16_t *)temp;
		temp += 2;

		for (i = 0; i < diag_max_registration; i++) {
			if (driver->table[i].process_id != 0) {
				if (driver->table[i].cmd_code ==
				     cmd_code && driver->table[i].subsys_id ==
				     subsys_id &&
				    driver->table[i].cmd_code_lo <=
				     subsys_cmd_code &&
					  driver->table[i].cmd_code_hi >=
				     subsys_cmd_code){
					driver->pkt_length = len;
					diag_update_pkt_buffer(buf);
					diag_update_sleeping_process(
						driver->table[i].process_id);
						return 0;
				    } /* end of if */
				else if (driver->table[i].cmd_code == 255
					  && cmd_code == 75) {
					if (driver->table[i].subsys_id ==
					    subsys_id &&
					   driver->table[i].cmd_code_lo <=
					    subsys_cmd_code &&
					     driver->table[i].cmd_code_hi >=
					    subsys_cmd_code){
						driver->pkt_length = len;
						diag_update_pkt_buffer(buf);
						diag_update_sleeping_process(
							driver->table[i].
							process_id);
						return 0;
					}
				} /* end of else-if */
				else if (driver->table[i].cmd_code == 255 &&
					  driver->table[i].subsys_id == 255) {
					if (driver->table[i].cmd_code_lo <=
							 cmd_code &&
						     driver->table[i].
						    cmd_code_hi >= cmd_code){
						driver->pkt_length = len;
						diag_update_pkt_buffer(buf);
						diag_update_sleeping_process
							(driver->table[i].
							 process_id);
						return 0;
					}
				} /* end of else-if */
			} /* if(driver->table[i].process_id != 0) */
		}  /* for (i = 0; i < diag_max_registration; i++) */
	} /* else */
		return packet_type;
}

void diag_process_hdlc(void *data, unsigned len)
{
	struct diag_hdlc_decode_type hdlc;
	int ret, type = 0;
#if HPST_FUN
	unsigned char *buf_9k = NULL;
	int path;
#endif
#ifdef DIAG_DEBUG
	int i;
	printk(KERN_INFO "\n HDLC decode function, len of data  %d\n", len);
#endif
	hdlc.dest_ptr = driver->hdlc_buf;
	hdlc.dest_size = USB_MAX_OUT_BUF;
	hdlc.src_ptr = data;
	hdlc.src_size = len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = diag_hdlc_decode(&hdlc);

	if (ret) {
		type = diag_process_apps_pkt(driver->hdlc_buf,
							  hdlc.dest_idx - 3);
	} else if (driver->debug_flag) {
		printk(KERN_ERR "Packet dropped due to bad HDLC coding/CRC"
				" errors or partial packet received, packet"
				" length = %d\n", len);
		print_hex_dump(KERN_DEBUG, "Dropped Packet Data: ", 16, 1,
					   DUMP_PREFIX_ADDRESS, data, len, 1);
		driver->debug_flag = 0;
	}
#ifdef DIAG_DEBUG
	printk(KERN_INFO "\n hdlc.dest_idx = %d \n", hdlc.dest_idx);
	for (i = 0; i < hdlc.dest_idx; i++)
		printk(KERN_DEBUG "\t%x", *(((unsigned char *)
							driver->hdlc_buf)+i));
#endif
	/* ignore 2 bytes for CRC, one for 7E and send */
	if ((driver->ch) && (ret) && (type) && (hdlc.dest_idx > 3)) {
		APPEND_DEBUG('g');

#if HPST_FUN
/* check for hpst*/
		path = checkcmd_modem_hpst(data);

		switch (path) {
		case DM7K9K:
				printk(KERN_INFO "%s:DM7K9K\n", __func__);
				print_hex_dump(KERN_DEBUG, "HPST:DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, data, 16, 1);
				smd_write(driver->ch, data, len);
				buf_9k = kzalloc(len, GFP_KERNEL);
				memcpy(buf_9k, data, len);
				msm_sdio_diag_write((void *)buf_9k, len);
				buf_9k = NULL;
				break;
		case DM9KONLY:
				printk(KERN_INFO "%s:DM9KONLY\n", __func__);
				print_hex_dump(KERN_DEBUG, "HPST:DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, data, 16, 1);
				buf_9k = kzalloc(len, GFP_KERNEL);
				memcpy(buf_9k, data, len);
				msm_sdio_diag_write((void *)buf_9k, len);
				buf_9k = NULL;
				break;
		case DM7K9KDIFF:
				printk(KERN_INFO "%s:DM7K9KDIFF", __func__);
				print_hex_dump(KERN_DEBUG, "HPST:DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, data, 16, 1);
				if (((*(((uint8_t *)data)+3)) & 0xC0) == 0xC0) {
					printk(KERN_INFO "%s:DM7K9KDIFF to 9K\n", __func__);
					buf_9k = kzalloc(len, GFP_KERNEL);
					memcpy(buf_9k, data, len);
					msm_sdio_diag_write((void *)buf_9k, len);
					buf_9k = NULL;
				} else {
					printk(KERN_INFO "%s:DM7K9KDIFF to 7K\n", __func__);
					smd_write(driver->ch, data, len);
				}
				break;
		case DM7KONLY:
				printk(KERN_INFO "%s:DM7KONLY\n", __func__);
				print_hex_dump(KERN_DEBUG, "HPST:DM Packet Data"
				" write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, data, 16, 1);
				smd_write(driver->ch, data, len);
				break;
		case NO_PST:
			smd_write(driver->ch, data, len);
			break;

		case NO_DEF_ID:
		case NO_DEF_ITEM:
		default:
				print_hex_dump(KERN_DEBUG, "HPST:DM Packet Data"
				" can't write to radio ", 16, 1, DUMP_PREFIX_ADDRESS, data, 16, 1);
		}
#else
		smd_write(driver->ch, data, len);
#endif
		APPEND_DEBUG('h');
#ifdef DIAG_DEBUG
		printk(KERN_INFO "writing data to SMD, pkt length %d \n", len);
		print_hex_dump(KERN_DEBUG, "Written Packet Data to SMD: ", 16,
			       1, DUMP_PREFIX_ADDRESS, data, len, 1);
#endif
	}

}

int diagfwd_connect(void)
{
	int err;

	printk(KERN_DEBUG "diag: USB connected\n");
	err = diag_open(driver->poolsize + 3); /* 2 for A9 ; 1 for q6*/
	if (err)
		printk(KERN_ERR "diag: USB port open failed");
	driver->usb_connected = 1;
	driver->in_busy_1 = 0;
	driver->in_busy_2 = 0;
	driver->in_busy_qdsp_1 = 0;
	driver->in_busy_qdsp_2 = 0;
	driver->in_busy_mdm_1 = 0;
	driver->in_busy_mdm_2 = 0;

	/* Poll SMD channels to check for data*/
	queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	/*queue_work(driver->mdm_diag_workqueue, &(driver->diag_read_smd_mdm_work));*/

	driver->usb_read_ptr->buf = driver->usb_buf_out;
	driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
	APPEND_DEBUG('a');
	diag_read(driver->usb_read_ptr);
	APPEND_DEBUG('b');
	return 0;
}

int diagfwd_disconnect(void)
{
	unsigned char buf[324];
	unsigned char msg_mask[]= {0x7d, 0x04, 0x00, 0x00, 0x4e};

	printk(KERN_DEBUG "diag: USB disconnected\n");
	driver->usb_connected = 0;
	driver->in_busy_1 = 1;
	driver->in_busy_2 = 1;
	driver->in_busy_qdsp_1 = 1;
	driver->in_busy_qdsp_2 = 1;
	driver->in_busy_mdm_1 = 1;
	driver->in_busy_mdm_2 = 1;
	driver->debug_flag = 1;
	diag_close();

	printk(KERN_ALERT "disable QCRIL message log\n");
	memset(buf, 0, sizeof(buf));
	memcpy(buf, msg_mask, 5);
	diag_update_msg_mask(0, 0x4e, buf, 1);

	/* TBD - notify and flow control SMD */
	return 0;
}

int diagfwd_write_complete(struct diag_request *diag_write_ptr)
{
	unsigned char *buf = diag_write_ptr->buf;
	/*Determine if the write complete is for data from arm9/apps/q6 */
	/* Need a context variable here instead */
	if (buf == (void *)driver->usb_buf_in_1) {
		driver->in_busy_1 = 0;
		APPEND_DEBUG('o');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	} else if (buf == (void *)driver->usb_buf_in_2) {
		driver->in_busy_2 = 0;
		APPEND_DEBUG('O');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
	} else if (buf == (void *)driver->usb_buf_in_qdsp_1) {
		driver->in_busy_qdsp_1 = 0;
		APPEND_DEBUG('p');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	} else if (buf == (void *)driver->usb_buf_in_qdsp_2) {
		driver->in_busy_qdsp_2 = 0;
		APPEND_DEBUG('P');
		queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
	} else if (driver->in_busy_dm == 1) {
		driver->in_busy_dm = 0;
	} else if (buf == (void *)driver->usb_buf_in_mdm_1) {
		driver->in_busy_mdm_1 = 0;
		queue_work(driver->mdm_diag_workqueue, &(driver->diag_read_smd_mdm_work));
	} else if (buf == (void *)driver->usb_buf_in_mdm_2) {
		driver->in_busy_mdm_2 = 0;
		queue_work(driver->mdm_diag_workqueue, &(driver->diag_read_smd_mdm_work));
	} else {
		diagmem_free(driver, (unsigned char *)buf, POOL_TYPE_HDLC);
		diagmem_free(driver, (unsigned char *)diag_write_ptr,
							 POOL_TYPE_USB_STRUCT);
		APPEND_DEBUG('q');
	}
	return 0;
}

int diagfwd_read_complete(struct diag_request *diag_read_ptr)
{
	int len = diag_read_ptr->actual;

	APPEND_DEBUG('c');
#ifdef DIAG_DEBUG
	printk(KERN_INFO "read data from USB, pkt length %d \n",
		    diag_read_ptr->actual);
	print_hex_dump(KERN_DEBUG, "Read Packet Data from USB: ", 16, 1,
		       DUMP_PREFIX_ADDRESS, diag_read_ptr->buf,
		       diag_read_ptr->actual, 1);
#endif
	driver->read_len = len;
	if (driver->logging_mode == USB_MODE)
	    queue_work(driver->diag_wq , &(driver->diag_read_work));
	return 0;
}

static struct diag_operations diagfwdops = {
	.diag_connect = diagfwd_connect,
	.diag_disconnect = diagfwd_disconnect,
	.diag_char_write_complete = diagfwd_write_complete,
	.diag_char_read_complete = diagfwd_read_complete
};

static void diag_smd_notify(void *ctxt, unsigned event)
{
	/*	printk(KERN_INFO "%s:\n", __func__);*/
	queue_work(driver->diag_wq, &(driver->diag_read_smd_work));
}

#if defined(CONFIG_MSM_N_WAY_SMD)
static void diag_smd_qdsp_notify(void *ctxt, unsigned event)
{
	queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_work));
}
#endif

void diag_smd_enable(char *src, int enable)
{

	printk(KERN_INFO "smd_try_open(%s): %d\n", src, enable);
	if (!driver->init_done)
		return;

	mutex_lock(&driver->smd_lock);
	if (enable) {
		if (!driver->ch)
			smd_open("SMD_DIAG", &driver->ch, driver, diag_smd_notify);
	} else {
		if (driver->ch) {
			smd_close(driver->ch);
			driver->ch = NULL;
		}
	}
	mutex_unlock(&driver->smd_lock);
}

static int diag_smd_probe(struct platform_device *pdev)
{
	int r = 0;
printk("%s\n", __func__);
	if (pdev->id == 0) {
		if (driver->usb_buf_in_1 == NULL ||
					 driver->usb_buf_in_2 == NULL) {
			if (driver->usb_buf_in_1 == NULL)
				driver->usb_buf_in_1 = kzalloc(USB_MAX_IN_BUF,
								GFP_KERNEL);
			if (driver->usb_buf_in_2 == NULL)
				driver->usb_buf_in_2 = kzalloc(USB_MAX_IN_BUF,
								GFP_KERNEL);
			if (driver->usb_buf_in_1 == NULL ||
				 driver->usb_buf_in_2 == NULL)
			goto err;
	}
}

		if (driver->usb_buf_in_mdm_1 == NULL ||
					 driver->usb_buf_in_mdm_2 == NULL) {
			if (driver->usb_buf_in_mdm_1 == NULL)
				driver->usb_buf_in_mdm_1 = kzalloc(USB_MAX_IN_BUF,
								GFP_KERNEL);
			if (driver->usb_buf_in_mdm_2 == NULL)
				driver->usb_buf_in_mdm_2 = kzalloc(USB_MAX_IN_BUF,
								GFP_KERNEL);
			if (driver->usb_buf_in_mdm_1 == NULL ||
				 driver->usb_buf_in_mdm_2 == NULL) {
			goto err;
		}
	}


#if defined(CONFIG_MSM_N_WAY_SMD)

	if (pdev->id == 1) {
		if (driver->usb_buf_in_qdsp_1 == NULL ||
					 driver->usb_buf_in_qdsp_2 == NULL) {
			if (driver->usb_buf_in_qdsp_1 == NULL)
				driver->usb_buf_in_qdsp_1 = kzalloc(
						USB_MAX_IN_BUF, GFP_KERNEL);
			if (driver->usb_buf_in_qdsp_2 == NULL)
				driver->usb_buf_in_qdsp_2 = kzalloc(
						USB_MAX_IN_BUF, GFP_KERNEL);
			if (driver->usb_buf_in_qdsp_1 == NULL ||
				 driver->usb_buf_in_qdsp_2 == NULL)
			goto err;
			else
				r = smd_named_open_on_edge("DIAG", SMD_APPS_QDSP
						, &driver->chqdsp, driver,
							 diag_smd_qdsp_notify);
		}
		else
		r = smd_named_open_on_edge("DIAG", SMD_APPS_QDSP,
			&driver->chqdsp, driver, diag_smd_qdsp_notify);
	}
#endif
/*	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);*/

	printk(KERN_INFO "diag opened SMD port ; r = %d\n", r);

err:
	return 0;
}
/*
static int diagfwd_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_dev_pm_ops = {
	.runtime_suspend = diagfwd_runtime_suspend,
	.runtime_resume = diagfwd_runtime_resume,
};

*/
static struct platform_driver msm_smd_ch1_driver = {

	.probe = diag_smd_probe,
	.driver = {
		   .name = "SMD_DIAG",
		   .owner = THIS_MODULE,
		   /*.pm   = &diagfwd_dev_pm_ops,*/
		   },
};

void diag_read_work_fn(struct work_struct *work)
{
	APPEND_DEBUG('d');
	if (!driver->nohdlc)
		diag_process_hdlc(driver->usb_buf_out, driver->read_len);
	driver->usb_read_ptr->buf = driver->usb_buf_out;
	driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
	APPEND_DEBUG('e');
	diag_read(driver->usb_read_ptr);
	APPEND_DEBUG('f');
}

void diagfwd_init(void)
{


	diag_debug_buf_idx = 0;
	if (driver->usb_buf_out  == NULL &&
	     (driver->usb_buf_out = kzalloc(USB_MAX_OUT_BUF,
					 GFP_KERNEL)) == NULL)
		goto err;
	if (driver->hdlc_buf == NULL
	    && (driver->hdlc_buf = kzalloc(HDLC_MAX, GFP_KERNEL)) == NULL)
		goto err;
	if (driver->msg_masks == NULL
	    && (driver->msg_masks = kzalloc(MSG_MASK_SIZE,
					     GFP_KERNEL)) == NULL)
		goto err;
	if (driver->log_masks == NULL &&
	    (driver->log_masks = kzalloc(LOG_MASK_SIZE, GFP_KERNEL)) == NULL)
		goto err;
	if (driver->event_masks == NULL &&
	    (driver->event_masks = kzalloc(EVENT_MASK_SIZE,
					    GFP_KERNEL)) == NULL)
		goto err;
	if (driver->client_map == NULL &&
	    (driver->client_map = kzalloc
	     ((driver->num_clients) * sizeof(struct diag_client_map),
		   GFP_KERNEL)) == NULL)
		goto err;
	if (driver->mdmclient_map == NULL &&
	    (driver->mdmclient_map = kzalloc
	     ((driver->num_mdmclients) * sizeof(struct diag_client_map),
		   GFP_KERNEL)) == NULL)
		goto err;
	if (driver->buf_tbl == NULL)
			driver->buf_tbl = kzalloc(buf_tbl_size *
			  sizeof(struct diag_write_device), GFP_KERNEL);
	if (driver->buf_tbl == NULL)
		goto err;
	if (driver->mdmbuf_tbl == NULL)
			driver->mdmbuf_tbl = kzalloc(buf_tbl_size *
			  sizeof(struct diag_write_device), GFP_KERNEL);
	if (driver->mdmbuf_tbl == NULL)
		goto err;

	if (driver->data_ready == NULL &&
	     (driver->data_ready = kzalloc(driver->num_clients * sizeof(struct
					 diag_client_map), GFP_KERNEL)) == NULL)
		goto err;
	if (driver->mdmdata_ready == NULL &&
	     (driver->mdmdata_ready = kzalloc(driver->num_mdmclients * sizeof(struct
					 diag_client_map), GFP_KERNEL)) == NULL)
		goto err;
	if (driver->table == NULL &&
	     (driver->table = kzalloc(diag_max_registration*
				      sizeof(struct diag_master_table),
				       GFP_KERNEL)) == NULL)
		goto err;
	if (driver->usb_write_ptr_1 == NULL)
		driver->usb_write_ptr_1 = kzalloc(
				sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_write_ptr_1 == NULL)
			goto err;
	if (driver->usb_write_ptr_2 == NULL)
		driver->usb_write_ptr_2 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_write_ptr_2 == NULL)
					goto err;
	if (driver->usb_write_ptr_qdsp_1 == NULL)
		driver->usb_write_ptr_qdsp_1 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_write_ptr_qdsp_1 == NULL)
			goto err;
	if (driver->usb_write_ptr_qdsp_2 == NULL)
		driver->usb_write_ptr_qdsp_2 = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_write_ptr_qdsp_2 == NULL)
			goto err;
	if (driver->usb_write_ptr_mdm_1 == NULL)
			driver->usb_write_ptr_mdm_1 = kzalloc(
				sizeof(struct diag_request), GFP_KERNEL);
			if (driver->usb_write_ptr_mdm_1 == NULL)
					goto err;
	if (driver->usb_write_ptr_mdm_2 == NULL)
			driver->usb_write_ptr_mdm_2 = kzalloc(
				sizeof(struct diag_request), GFP_KERNEL);
			if (driver->usb_write_ptr_mdm_2 == NULL)
					goto err;
	if (driver->usb_read_ptr == NULL)
			driver->usb_read_ptr = kzalloc(
				sizeof(struct diag_request), GFP_KERNEL);
			if (driver->usb_read_ptr == NULL)
				goto err;
	if (driver->pkt_buf == NULL &&
	     (driver->pkt_buf = kzalloc(PKT_SIZE,
					 GFP_KERNEL)) == NULL)
		goto err;

	driver->diag_wq = create_singlethread_workqueue("diag_wq");
	INIT_WORK(&(driver->diag_read_work), diag_read_work_fn);

	platform_driver_register(&msm_smd_ch1_driver);

	mutex_init(&driver->smd_lock);
	driver->init_done = 1;
	diag_usb_register(&diagfwdops);
	/* for MDM9k */
	sdio_diag_init();

	return;
err:
		printk(KERN_INFO "\n Could not initialize diag buffers\n");
		kfree(driver->usb_buf_out);
		kfree(driver->hdlc_buf);
		kfree(driver->msg_masks);
		kfree(driver->log_masks);
		kfree(driver->event_masks);
		kfree(driver->client_map);
		kfree(driver->mdmclient_map);
		kfree(driver->buf_tbl);
		kfree(driver->mdmbuf_tbl);
		kfree(driver->data_ready);
		kfree(driver->mdmdata_ready);
		kfree(driver->table);
		kfree(driver->pkt_buf);
		kfree(driver->usb_write_ptr_1);
		kfree(driver->usb_write_ptr_2);
		kfree(driver->usb_write_ptr_qdsp_1);
		kfree(driver->usb_write_ptr_qdsp_2);
		kfree(driver->usb_write_ptr_mdm_1);
		kfree(driver->usb_write_ptr_mdm_2);
		kfree(driver->usb_read_ptr);
}

void diagfwd_exit(void)
{
	smd_close(driver->ch);
	smd_close(driver->chqdsp);
	driver->ch = 0;		/*SMD can make this NULL */
	driver->chqdsp = 0;

	if (driver->usb_connected)
		diag_close();

	platform_driver_unregister(&msm_smd_ch1_driver);

	diag_usb_unregister();

	kfree(driver->usb_buf_in_1);
	kfree(driver->usb_buf_in_2);
	kfree(driver->usb_buf_in_qdsp_1);
	kfree(driver->usb_buf_in_qdsp_2);
	kfree(driver->usb_buf_in_mdm_1);
	kfree(driver->usb_buf_in_mdm_2);
	kfree(driver->usb_buf_out);
	kfree(driver->hdlc_buf);
	kfree(driver->msg_masks);
	kfree(driver->log_masks);
	kfree(driver->event_masks);
	kfree(driver->client_map);
	kfree(driver->mdmclient_map);
	kfree(driver->buf_tbl);
	kfree(driver->mdmbuf_tbl);
	kfree(driver->data_ready);
	kfree(driver->mdmdata_ready);
	kfree(driver->table);
	kfree(driver->pkt_buf);
	kfree(driver->usb_write_ptr_1);
	kfree(driver->usb_write_ptr_2);
	kfree(driver->usb_write_ptr_qdsp_1);
	kfree(driver->usb_write_ptr_qdsp_2);
	kfree(driver->usb_write_ptr_mdm_1);
	kfree(driver->usb_write_ptr_mdm_2);
	kfree(driver->usb_read_ptr);
}
