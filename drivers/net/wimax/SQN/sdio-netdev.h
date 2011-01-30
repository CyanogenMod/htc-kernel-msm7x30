/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Andy Shevchenko <andy@smile.org.ua>,
 *            Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_NETDEV_H
#define _SQN_NETDEV_H

#include <linux/netdevice.h>
#include <linux/spinlock_types.h>
#include <linux/if_ether.h>
#include <linux/wait.h>
#include <linux/types.h>


/* TODO: Move to sqn_sdio.c */
#define SQN_MAX_PDU_LEN		2048	/* Max PDU length */


extern struct ethtool_ops sqn_ethtool_ops;

struct sqn_private {
	spinlock_t		drv_lock;
	void			*card;
	struct net_device	*dev;
	struct net_device_stats	stats;
	u8			mac_addr[ETH_ALEN];
	struct task_struct	*tx_thread;  /* Thread to service TX queue */
	wait_queue_head_t	tx_waitq;
	wait_queue_head_t	rx_waitq;
	struct work_struct	rx_work_struct;
	u8			removed;


	int  (*hw_host_to_card) (struct sqn_private *priv);

	void (*add_skb_to_tx_queue) (struct sqn_private *priv
		, struct sk_buff *skb, u8 tail);

	int  (*is_tx_queue_empty) (struct sqn_private *priv);
};

#endif /* _SQN_NETDEV_H */
