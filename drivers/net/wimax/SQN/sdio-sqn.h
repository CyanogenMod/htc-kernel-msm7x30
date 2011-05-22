/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Andy Shevchenko <andy@smile.org.ua>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_SDIO_H
#define _SQN_SDIO_H

#include "version.h"

#include <linux/skbuff.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/timer.h>


enum sqn_card_version {
	SQN_1130 = 1
	, SQN_1210
};


/* Card private information */
struct sqn_sdio_card {
	struct sqn_private	*priv;
	struct sdio_func	*func;
	u8			rstn_wr_fifo_flag;
	u8			version;
	struct sk_buff_head	tx_queue;
	struct mutex		tx_mutex;
#define TX_QUEUE_MAX_LEN	1000	/* max length to which tx_queue is allowed to grow */
#define TX_QUEUE_WM_LEN		800	/* length, from which we will continue transmission */
	struct sk_buff_head	rx_queue;
#define RX_QUEUE_MAX_LEN	1000	/* max length to which rx_queue is allowed to grow */
#define RX_QUEUE_WM_LEN		800	/* length, from which we will continue transmission */
	struct mutex		rx_mutex;
	struct mutex		rxq_mutex;
	wait_queue_head_t	pm_waitq;
	struct wake_lock	wakelock;
    struct timer_list	wakelock_timer;

	/* Condition flags for event signaling */
	u8			pm_complete;
	u8			it_thread_should_stop;
	u8			is_card_sleeps;
	u8			waiting_pm_notification;
};


void sqn_sdio_stop_it_thread_from_itself(struct sqn_private *priv);

struct sk_buff* sqn_sdio_prepare_skb_for_tx(struct sk_buff *skb);

int sqn_sdio_tx_skb(struct sqn_sdio_card *card, struct sk_buff *skb
	, u8 claim_host);


#define SQN_SDIO_PDU_MINLEN		2
#define SQN_SDIO_PDU_MAXLEN		2047

/* Product IDs */
#define SDIO_CMN_CISTPLMID_MANF		0x1002	/* Sequans manufacture ID register */
#define SDIO_CMN_CISTPLMID_CARD		0x1004	/* Sequans SQN1130 card ID register */

#define SDIO_VENDOR_ID_SEQUANS		0x039d	/* Sequans manufacture ID */
#define SDIO_DEVICE_ID_SEQUANS_SQN1130	0x046a	/* Sequans SQN1130 card ID */
#define SDIO_DEVICE_ID_SEQUANS_SQN1210	0x1210	/* Sequans SQN1210-rev2 card ID */

#define SDIO_CCCR_CCCR_SDIO_VERSION	0x00
#define SDIO_CCCR_IO_ABORT		0x06


#define SQN_H_VERSION       		0x240C


/* FIFO dependent list */
#define SQN_SDIO_RDWR_BASE		0x2000
#define SQN_SDIO_RDWR_FIFO(x)		(SQN_SDIO_RDWR_BASE + (x)*4)

#define SQN_SDIO_RDLEN_BASE		0x2002
#define SQN_SDIO_RDLEN_FIFO(x)		(SQN_SDIO_RDLEN_BASE + (x)*4)

#define SQN_SDIO_PCRRT_BASE		0x2010
#define SQN_SDIO_PCRRT_FIFO(x)		(SQN_SDIO_PCRRT_BASE + (x)*2)

#define SQN_SDIO_PCWRT_BASE		0x2011
#define SQN_SDIO_PCWRT_FIFO(x)		(SQN_SDIO_PCWRT_BASE + (x)*2)

#define SQN_SDIO_SZ_RD_BASE		0x2018
#define SQN_SDIO_SZ_RD_FIFO(x)		(SQN_SDIO_SZ_RD_BASE + (x)*8)

#define SQN_SDIO_WM_RD_BASE		0x201a
#define SQN_SDIO_WM_RD_FIFO(x)		(SQN_SDIO_WM_RD_BASE + (x)*8)

#define SQN_SDIO_SZ_WR_BASE		0x201c
#define SQN_SDIO_SZ_WR_FIFO(x)		(SQN_SDIO_SZ_WR_BASE + (x)*8)

#define SQN_SDIO_WM_WR_BASE		0x201e
#define SQN_SDIO_WM_WR_FIFO(x)		(SQN_SDIO_WM_WR_BASE + (x)*8)

#define SQN_SDIO_ESZ_WR_FIFO0		0x2032	/* FIFO0 */
#define SQN_SDIO_ESZ_WR_FIFO1		0x2036	/* FIFO1 */
#define SQN_SDIO_ESZ_WR_FIFO2		0x0000	/* No real FIFO */

#define SQN_SDIO_RSTN_RD_BASE		0x2038
#define SQN_SDIO_RSTN_RD_FIFO(x)	(SQN_SDIO_RSTN_RD_BASE + (x)*2)

#define SQN_SDIO_RSTN_WR_BASE		0x2039
#define SQN_SDIO_RSTN_WR_FIFO(x)	(SQN_SDIO_RSTN_WR_BASE + (x)*2)

#define SQN_SDIO_RD_LEVEL_BASE		0x2048
#define SQN_SDIO_RD_FIFO_LEVEL(x)	(SQN_SDIO_RD_LEVEL_BASE + (x)*4)

#define SQN_SDIO_WR_LEVEL_BASE		0x204a
#define SQN_SDIO_WR_FIFO_LEVEL(x)	(SQN_SDIO_WR_LEVEL_BASE + (x)*4)

#define SQN_SDIO_RD_BYTESLEFT_BASE	0x2054
#define SQN_SDIO_RD_FIFO_BYTESLEFT(x)	(SQN_SDIO_RD_BYTESLEFT_BASE + (x)*4)

#define SQN_SDIO_WR_BYTESLEFT_BASE	0x2056
#define SQN_SDIO_WR_FIFO_BYTESLEFT(x)	(SQN_SDIO_WR_BYTESLEFT_BASE + (x)*4)

/* Interrupt registers */
#define SQN_SDIO_IT_EN_LSBS		0x2044
#define SQN_SDIO_IT_EN_MSBS		0x2045
#define SQN_SDIO_IT_STATUS_LSBS		0x2046
#define SQN_SDIO_IT_STATUS_MSBS		0x2047

/* Firmware loading registers */
#define SQN_H_GRSTN			0x2400
#define SQN_H_CRSTN			0x2404
#define SQN_H_SDRAMCTL_RSTN		0x2414
#define SQN_H_SDRAM_NO_EMR		0x2415
#define SQN_H_BOOT_FROM_SPI		0x2411


/* Interrupt flags (LSB) */
#define   SQN_SDIO_IT_WR_FIFO0_WM		(1 << 0)
#define   SQN_SDIO_IT_RD_FIFO0_WM		(1 << 1)
#define   SQN_SDIO_IT_WR_FIFO1_WM		(1 << 2)
#define   SQN_SDIO_IT_RD_FIFO1_WM		(1 << 3)
#define   SQN_SDIO_IT_WR_FIFO2_WM		(1 << 4)
#define   SQN_SDIO_IT_RD_FIFO2_WM		(1 << 5)
#define   SQN_SDIO_IT_RD_EMPTY_WR_FULL		(1 << 6)
#define   SQN_SDIO_IT_SW_SIGN			(1 << 7)

/* Interrupt flags (MSB) */
#define   SQN_SDIO_IT_WR_FIFO0_FULL_RST		(1 << 0)
#define   SQN_SDIO_IT_RD_FIFO0_EMPTY_RST	(1 << 1)
#define   SQN_SDIO_IT_WR_FIFO1_FULL_RST		(1 << 2)
#define   SQN_SDIO_IT_RD_FIFO1_EMPTY_RST	(1 << 3)
#define   SQN_SDIO_IT_WR_FIFO2_FULL_RST		(1 << 4)
#define   SQN_SDIO_IT_RD_FIFO2_EMPTY_RST	(1 << 5)
#define   SQN_SDIO_IT_RD_BEFORE_RDLEN		(1 << 6)
#define   SQN_SDIO_IT_UNSUPPORTED_CMD		(1 << 7)

/* Software signaling interrupts */
#define	SQN_SOC_SIGS_LSBS		0x2600
#define	SQN_HTS_SIGS			0x2608

#endif /* _SQN_SDIO_H */
