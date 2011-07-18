/* copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 *  *
 *   * This software is licensed under the terms of the GNU General Public
 *    * License version 2, as published by the Free Software Foundation, and
 *     * may be copied, distributed, and modified under those terms.
 *      *
 *       * This program is distributed in the hope that it will be useful,
 *        * but WITHOUT ANY WARRANTY; without even the implied warranty of
 *         * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          * GNU General Public License for more details.
 *           *
 *            */
#ifndef _ARCH_ARM_MACH_MSM_MSM_SMD_DEBUG_H_
#define _ARCH_ARM_MACH_MSM_MSM_SMD_DEBUG_H_

#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/list.h>

#if !defined(CONFIG_ARCH_MSM7X30_LTE)
struct smd_alloc_elm {
	char name[20];
	uint32_t cid;
	uint32_t ctype;
	uint32_t ref_count;
};

struct smd_half_channel {
	unsigned state;
	unsigned char fDSR;
	unsigned char fCTS;
	unsigned char fCD;
	unsigned char fRI;
	unsigned char fHEAD;
	unsigned char fTAIL;
	unsigned char fSTATE;
	unsigned char fUNUSED;
	unsigned tail;
	unsigned head;
} __attribute__((packed));
#endif

struct smd_shared_v1 {
	struct smd_half_channel ch0;
	unsigned char data0[SMD_BUF_SIZE];
	struct smd_half_channel ch1;
	unsigned char data1[SMD_BUF_SIZE];
};

struct smd_shared_v2 {
	struct smd_half_channel ch0;
	struct smd_half_channel ch1;
};

struct smd_channel {
	volatile struct smd_half_channel *send;
	volatile struct smd_half_channel *recv;
	unsigned char *send_data;
	unsigned char *recv_data;

	unsigned fifo_mask;
	unsigned fifo_size;
	unsigned current_packet;
	unsigned n;

	struct list_head ch_list;

	void *priv;
	void (*notify)(void *priv, unsigned flags);

	int (*read)(struct smd_channel *ch, void *data, int len);
	int (*write)(struct smd_channel *ch, const void *data, int len);
	int (*read_avail)(struct smd_channel *ch);
	int (*write_avail)(struct smd_channel *ch);

	void (*update_state)(struct smd_channel *ch);
	unsigned last_state;
	void (*notify_other_cpu)(void);
	unsigned type;

	char name[32];
	struct platform_device pdev;
};

extern struct list_head smd_ch_closed_list;
extern struct list_head smd_ch_list_modem;
extern struct list_head smd_ch_list_dsp;

extern spinlock_t smd_lock;
extern spinlock_t smem_lock;

#endif
