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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/device.h>

#include "sdio-netdev.h"
#include "version.h"
#include "msg.h"
#include "thp.h"
#include "sdio.h"
#include "sdio-pm.h"
#include "sdio-fw.h"
#include "sdio-driver.h"

#define DRIVER_DEBUG 0
#define SKB_DEBUG 0
#define IGNORE_CARRIER_STATE 1
#define SDIO_CLAIM_HOST_DEBUG 0

/*******************************************************************/
/* Module parameter variables */
/*******************************************************************/

/** firmware_name - specifies the name of firmware binary */
char	*firmware_name	= SQN_DEFAULT_FW_NAME;

/**
 * load_firmware - boolean flag, controls whether firmware
 *  should be loaded or not
 */
int	load_firmware = 1;

bool drop_packet = false;

module_param(firmware_name, charp, S_IRUGO);
module_param(load_firmware, bool, S_IRUGO);

struct sqn_private *g_priv = 0;

//reference sdio-driver.c
extern const uint8_t  ss_macaddr[ETH_ALEN];

/*******************************************************************/
/* Network interface functions                                     */
/*******************************************************************/

static int sqn_dev_open(struct net_device *dev)
{
	struct sqn_private *priv = netdev_priv(dev);

	sqn_pr_enter();

	spin_lock(&priv->drv_lock);
	netif_wake_queue(dev);
	spin_unlock(&priv->drv_lock);

	sqn_pr_leave();
	return 0;
}


static int sqn_dev_stop(struct net_device *dev)
{
	struct sqn_private *priv = netdev_priv(dev);

	sqn_pr_enter();

	spin_lock(&priv->drv_lock);
	netif_stop_queue(dev);
	spin_unlock(&priv->drv_lock);

	sqn_pr_leave();
	return 0;
}


/*******************************************************************/
/* TX queue handlers                                               */
/*******************************************************************/

int sqn_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned long irq_flags = 0;
	struct ethhdr *eth;
	struct sqn_private *priv = netdev_priv(dev);

#if DRIVER_DEBUG
	printk(KERN_WARNING "sqn_hard_start_xmit \n");
#endif

	sqn_pr_enter();

	sqn_pr_dbg("skb->len = %d\n", skb->len);

#if SKB_DEBUG
	sqn_pr_info("%s: got skb [0x%p] from kernel, users %d\n", __func__, skb, atomic_read(&skb->users)); 
#endif

	spin_lock_irqsave(&priv->drv_lock, irq_flags);

	//HTC code: for DDTM
	if(drop_packet){
		eth = (struct ethhdr*) skb->data;
		if(memcmp(eth->h_dest, ss_macaddr, ETH_ALEN) != 0){
			sqn_pr_dbg("HTC drop_packet enabled: not THP, drop it\n");
#if DRIVER_DEBUG
			printk(KERN_WARNING "sqn_hard_start_xmit: network packet\n");
#endif
			priv->stats.tx_dropped++;
			priv->stats.tx_errors++;
			dev_kfree_skb_any(skb);
			goto out;
		}else{
			sqn_pr_dbg("HTC drop_packet enabled: THP, let it live\n");
#if DRIVER_DEBUG
			printk(KERN_WARNING "sqn_hard_start_xmit: thp packet\n");
#endif
		}
	}

	if (priv->removed) {
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_LOCKED;
	}

	if (skb->len < 1 || (skb->len > SQN_MAX_PDU_LEN)) {
		sqn_pr_dbg("skb length %d not in range (1, %d)\n", skb->len,
			   SQN_MAX_PDU_LEN);
		/*
		 * We'll never manage to send this one;
		 * drop it and return 'OK'
		 */
		priv->stats.tx_dropped++;
		priv->stats.tx_errors++;
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		goto out;
	}

	/* netif_stop_queue(priv->dev); */

	priv->add_skb_to_tx_queue(priv, skb, 1);

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;

	dev->trans_start = jiffies;

	spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
	wake_up_interruptible(&priv->tx_waitq);
out:
	sqn_pr_leave();
	return NETDEV_TX_OK;
}


static void sqn_tx_timeout(struct net_device *dev)
{
	/* struct sqn_private *priv = netdev_priv(dev); */

	sqn_pr_enter();

	sqn_pr_err("TX watch dog timeout\n");

	sqn_pr_leave();
}


static int sqn_tx_thread(void *data)
{
	struct net_device *dev = (struct net_device *) data;
	struct sqn_private *priv = netdev_priv(dev);
	int rv = 0;
	unsigned long irq_flags = 0;

	sqn_pr_enter();

	/*
	 * Set PF_NOFREEZE to prevent kernel to freeze this thread
	 * when going to suspend. We will manually stop it from
	 * driver's suspend handler.
	 */
	current->flags |= PF_NOFREEZE;

	for (;;) {
		spin_lock_irqsave(&priv->drv_lock, irq_flags);

		if (!(priv->is_tx_queue_empty(priv)) || priv->removed)
			spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
		else {
			int rv = 0;
			sqn_pr_dbg("wait for skb\n");
			spin_unlock_irqrestore(&priv->drv_lock, irq_flags);

			rv = wait_event_interruptible(priv->tx_waitq
				, !(priv->is_tx_queue_empty(priv))
				  || kthread_should_stop()
				  || priv->removed);

			/*
			 * If we've been interrupted by a signal, then we
			 * should stop a thread
			 */
			if (0 != rv) {
				sqn_pr_dbg("got a signal from kernel %d\n", rv);
				break;
			}
		}

		sqn_pr_dbg("got skb to send, wake up\n");

		if (kthread_should_stop()) {
			sqn_pr_dbg("break from main thread\n");
			break;
		}

		spin_lock_irqsave(&priv->drv_lock, irq_flags);
		if (priv->removed) {
			sqn_pr_dbg("adapter removed; wait to die...\n");
			spin_unlock_irqrestore(&priv->drv_lock, irq_flags);
			ssleep(1);
			continue;
		}
		spin_unlock_irqrestore(&priv->drv_lock, irq_flags);

		rv= priv->hw_host_to_card(priv);
		if (rv)
			sqn_pr_dbg("failed to send PDU: %d\n", rv);
	}

	sqn_pr_leave();
	return 0;
}


int sqn_start_tx_thread(struct sqn_private *priv)
{
	int rv = 0;

	sqn_pr_enter();

	priv->tx_thread = kthread_run(sqn_tx_thread, priv->dev, "sqn_tx");

	if (IS_ERR(priv->tx_thread)) {
		sqn_pr_dbg("error creating TX thread.\n");
		rv = 1;
		goto out;
	}

	sqn_pr_leave();
out:
	return rv;
}


int sqn_stop_tx_thread(struct sqn_private *priv)
{
	int rv = 0;

	sqn_pr_enter();

	kthread_stop(priv->tx_thread);
	wake_up_interruptible(&priv->tx_waitq);

	sqn_pr_leave();

	return rv;
}


/*******************************************************************/
/* RX queue handlers                                               */
/*******************************************************************/

int sqn_rx_process(struct net_device *dev, struct sk_buff *skb)
{
	int rc = 0;
	struct sqn_private *priv = netdev_priv(dev);

#if SDIO_CLAIM_HOST_DEBUG
	/* sqn_pr_info("%s+\n", __func__); */
#endif

#if DRIVER_DEBUG
	printk(KERN_WARNING "sqn_rx_process \n");
#endif

	sqn_pr_enter();

	dev->last_rx = jiffies;
	skb->protocol = eth_type_trans(skb, dev);
	skb->dev = dev;
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += skb->len;
#if SKB_DEBUG
	sqn_pr_info("%s: push skb [0x%p] to kernel, users %d\n", __func__, skb, atomic_read(&skb->users));
#endif
	netif_rx(skb);
	/* netif_receive_skb(skb); */

	sqn_pr_leave();

#if SDIO_CLAIM_HOST_DEBUG
	/* sqn_pr_info("%s-\n", __func__); */
#endif

	return rc;
}


/*******************************************************************/
/* Interface statistics                                            */
/*******************************************************************/

static struct net_device_stats *sqn_get_stats(struct net_device *dev)
{
	struct sqn_private *priv = netdev_priv(dev);

	sqn_pr_enter();
	sqn_pr_leave();

	return &priv->stats;
}


/*******************************************************************/
/* Adding and removing procedures                                  */
/*******************************************************************/


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
static const struct net_device_ops sqn_netdev_ops = {
	.ndo_open		= sqn_dev_open
	, .ndo_stop		= sqn_dev_stop
	, .ndo_start_xmit	= sqn_hard_start_xmit
	, .ndo_validate_addr	= eth_validate_addr
	, .ndo_tx_timeout	= sqn_tx_timeout
	, .ndo_get_stats	= sqn_get_stats
};
#endif


struct sqn_private *sqn_add_card(void *card, struct device *realdev)
{
	struct sqn_private *priv = 0;
	u8 dummy_wimax_mac_addr[ETH_ALEN]  = { 0x00, 0x16, 0x08, 0x00, 0x06, 0x53 };

	/* Allocate an Ethernet device and register it */
	struct net_device *dev = alloc_netdev(sizeof(struct sqn_private), "wimax%d", ether_setup);

	sqn_pr_enter();

	if (!dev) {	
		sqn_pr_err("init wimaxX device failed\n");
		goto done;
	}

	priv = netdev_priv(dev);
	g_priv = priv;
	memset(priv, 0, sizeof(struct sqn_private));

	/*
	 * Use dummy WiMAX mac address for development version (boot from
	 * flash) of WiMAX SDIO cards.  Production cards use mac address from
	 * firmware which is loaded by driver. Random ethernet address can't be
	 * used if IPv4 convergence layer is enabled on WiMAX base station.
	 */
	memcpy(priv->mac_addr, dummy_wimax_mac_addr, ETH_ALEN);

	spin_lock_init(&priv->drv_lock);

	/* Fill the private stucture */
	priv->dev = dev;
	priv->card = card;

	/* Setup the OS Interface to our functions */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)
	dev->open = sqn_dev_open;
	dev->stop = sqn_dev_stop;
	dev->hard_start_xmit = sqn_hard_start_xmit;
	dev->tx_timeout = sqn_tx_timeout;
	dev->get_stats = sqn_get_stats;
#else
	dev->netdev_ops = &sqn_netdev_ops;
#endif

	/* TODO: Make multicast possible */
	dev->flags &= ~IFF_MULTICAST;

	//wimax interface mtu must be 1400 (in spec)
	dev->mtu = 1400;
	SET_NETDEV_DEV(dev, realdev);

done:
	sqn_pr_leave();
	return priv;
}


int sqn_remove_card(struct sqn_private *priv)
{
	struct net_device *dev = priv->dev;
	unsigned long irq_flags = 0;

	sqn_pr_enter();

	dev = priv->dev;

	spin_lock_irqsave(&priv->drv_lock, irq_flags);
	priv->removed = 1;
	priv->dev = NULL;
	spin_unlock_irqrestore(&priv->drv_lock, irq_flags);

	/* kthread_stop(priv->tx_thread); */
	/* wake_up_interruptible(&priv->tx_waitq); */
	free_netdev(dev);

	sqn_pr_leave();
	return 0;
}


int sqn_start_card(struct sqn_private *priv)
{
	struct net_device *dev = priv->dev;

	sqn_pr_enter();

	if (register_netdev(dev)) {
		sqn_pr_err("cannot register ethX device\n");
		return -1;
	}

	sqn_pr_dbg("starting TX thread...\n");
	/* TODO: move waitq initializatio to add_card() */
	init_waitqueue_head(&priv->tx_waitq);
	init_waitqueue_head(&priv->rx_waitq);
	if (sqn_start_tx_thread(priv))
		goto err_init_adapter;

	sqn_pr_info("%s: Sequans WiMAX adapter\n", dev->name);

#if IGNORE_CARRIER_STATE
    netif_carrier_on(priv->dev);
#else
	/* In release version this should be uncommented */
	/* netif_carrier_off(priv->dev); */
#endif

done:
	sqn_pr_leave();
	return 0;

err_init_adapter:
	/* TODO: Free allocated resources */
	sqn_pr_err("error while init adapter\n");
	free_netdev(dev);
	priv = NULL;

	goto done;
}


int sqn_stop_card(struct sqn_private *priv)
{
	struct net_device *dev = priv->dev;

	sqn_pr_enter();

	unregister_netdev(dev);

	sqn_pr_leave();
	return 0;
}
