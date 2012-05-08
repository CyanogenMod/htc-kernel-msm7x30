
/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef DIAGFWD_H
#define DIAGFWD_H

#define NO_PROCESS	0
#define NON_APPS_PROC	-1

#define DIAGLOG_MODE_NONE 0
#define DIAGLOG_MODE_HEAD 1
#define DIAGLOG_MODE_FULL 2
#define DIAGLOG_MODE_PING 3

void diagfwd_init(void);
void diagfwd_exit(void);
void diag_process_hdlc(void *data, unsigned len);
void __diag_smd_send_req(void);
void __diag_smd_qdsp_send_req(void);
void __diag_smd_wcnss_send_req(void);
void diag_usb_legacy_notifier(void *, unsigned, struct diag_request *);
long diagchar_ioctl(struct file *, unsigned int, unsigned long);
int diag_device_write(void *, int, struct diag_request *);
int mask_request_validate(unsigned char mask_buf[]);
int chk_config_get_id(void);
void diag_clear_reg(int);

/* State for diag forwarding */
#ifdef CONFIG_DIAG_OVER_USB
int diagfwd_connect(void);
int diagfwd_disconnect(void);
#endif
extern int diag_support_mdm9k;
extern int diag_debug_buf_idx;
extern unsigned char diag_debug_buf[1024];
extern unsigned diag7k_debug_mask;
extern unsigned diag9k_debug_mask;

#define SMD_FUNC_CLOSE 0
#define SMD_FUNC_OPEN_DIAG 1
#define SMD_FUNC_OPEN_BT 2
void diag_smd_enable(smd_channel_t *ch, char *src, int mode);

#endif
