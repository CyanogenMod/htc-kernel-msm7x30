/* linux/drivers/net/msm_rmnet.c
 *
 * Virtual Ethernet Interface for MSM7K Networking
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/if_arp.h>
#include <linux/msm_rmnet.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/msm_smd.h>
#include <mach/peripheral-loader.h>

/* XXX should come from smd headers */
#define SMD_PORT_ETHER0 11

/* allow larger frames */
#define RMNET_DATA_LEN 2000

#define HEADROOM_FOR_QOS    8

static const char *ch_name[8] = {
	"SMD_DATA5",
	"SMD_DATA6",
	"SMD_DATA7",
	"SMD_DATA8",
	"SMD_DATA9",
	"SMD_DATA12",
	"SMD_DATA13",
	"SMD_DATA14",
};

static struct completion *port_complete[8];

struct rmnet_private
{
	smd_channel_t *ch;
	struct net_device_stats stats;
	const char *chname;
	struct wake_lock wake_lock;
#ifdef CONFIG_MSM_RMNET_DEBUG
	ktime_t last_packet;
	unsigned long wakeups_xmit;
	unsigned long wakeups_rcv;
	unsigned long timeout_us;
#endif
	struct sk_buff *skb;
	spinlock_t lock;
	struct tasklet_struct tsklt;
	u32 operation_mode;    /* IOCTL specified mode (protocol, QoS header) */
	struct platform_driver pdrv;
	struct completion complete;
	void *pil;
	struct mutex pil_lock;
};

static uint msm_rmnet_modem_wait;
module_param_named(modem_wait, msm_rmnet_modem_wait,
		   uint, S_IRUGO | S_IWUSR | S_IWGRP);

/* Forward declaration */
static int rmnet_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd);

static int count_this_packet(void *_hdr, int len)
{
	struct ethhdr *hdr = _hdr;

	if (len >= ETH_HLEN && hdr->h_proto == htons(ETH_P_ARP))
		return 0;

	return 1;
}

#ifdef CONFIG_MSM_RMNET_DEBUG
static unsigned long timeout_us;

#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * If early suspend is enabled then we specify two timeout values,
 * screen on (default), and screen is off.
 */
static unsigned long timeout_suspend_us;
static struct device *rmnet0;

/* Set timeout in us when the screen is off. */
static ssize_t timeout_suspend_store(struct device *d, struct device_attribute *attr, const char *buf, size_t n)
{
	timeout_suspend_us = simple_strtoul(buf, NULL, 10);
	return n;
}

static ssize_t timeout_suspend_show(struct device *d,
				    struct device_attribute *attr,
				    char *buf)
{
	return sprintf(buf, "%lu\n", (unsigned long) timeout_suspend_us);
}

static DEVICE_ATTR(timeout_suspend, 0664, timeout_suspend_show, timeout_suspend_store);

static void rmnet_early_suspend(struct early_suspend *handler) {
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
		p->timeout_us = timeout_suspend_us;
	}
}

static void rmnet_late_resume(struct early_suspend *handler) {
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
		p->timeout_us = timeout_us;
	}
}

static struct early_suspend rmnet_power_suspend = {
	.suspend = rmnet_early_suspend,
	.resume = rmnet_late_resume,
};

static int __init rmnet_late_init(void)
{
	register_early_suspend(&rmnet_power_suspend);
	return 0;
}

late_initcall(rmnet_late_init);
#endif

/* Returns 1 if packet caused rmnet to wakeup, 0 otherwise. */
static int rmnet_cause_wakeup(struct rmnet_private *p) {
	int ret = 0;
	ktime_t now;
	if (p->timeout_us == 0) /* Check if disabled */
		return 0;

	/* Use real (wall) time. */
	now = ktime_get_real();

	if (ktime_us_delta(now, p->last_packet) > p->timeout_us) {
		ret = 1;
	}
	p->last_packet = now;
	return ret;
}

static ssize_t wakeups_xmit_show(struct device *d,
				 struct device_attribute *attr,
				 char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->wakeups_xmit);
}

DEVICE_ATTR(wakeups_xmit, 0444, wakeups_xmit_show, NULL);

static ssize_t wakeups_rcv_show(struct device *d, struct device_attribute *attr,
		char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", p->wakeups_rcv);
}

DEVICE_ATTR(wakeups_rcv, 0444, wakeups_rcv_show, NULL);

/* Set timeout in us. */
static ssize_t timeout_store(struct device *d, struct device_attribute *attr,
		const char *buf, size_t n)
{
#ifndef CONFIG_HAS_EARLYSUSPEND
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	p->timeout_us = timeout_us = simple_strtoul(buf, NULL, 10);
#else
/* If using early suspend/resume hooks do not write the value on store. */
	timeout_us = simple_strtoul(buf, NULL, 10);
#endif
	return n;
}

static ssize_t timeout_show(struct device *d, struct device_attribute *attr,
			    char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
	p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", timeout_us);
}

DEVICE_ATTR(timeout, 0664, timeout_show, timeout_store);
#endif

static __be16 rmnet_ip_type_trans(struct sk_buff *skb, struct net_device *dev)
{
	__be16 protocol = 0;

	skb->dev = dev;

	/* Determine L3 protocol */
	switch (skb->data[0] & 0xf0) {
	case 0x40:
		protocol = htons(ETH_P_IP);
		break;
	case 0x60:
		protocol = htons(ETH_P_IPV6);
		break;
	default:
		pr_err("rmnet_recv() L3 protocol decode error: 0x%02x",
		       skb->data[0] & 0xf0);
		/* skb will be dropped in uppder layer for unknown protocol */
	}
	return protocol;
}

/* Called in soft-irq context */
static void smd_net_data_handler(unsigned long arg)
{
	struct net_device *dev = (struct net_device *) arg;
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb;
	void *ptr = 0;
	int sz;
	u32 opmode = p->operation_mode;
	unsigned long flags;

	for (;;) {
		sz = smd_cur_packet_size(p->ch);
		if (sz == 0) break;
		if (smd_read_avail(p->ch) < sz) break;

		if (RMNET_IS_MODE_IP(opmode) ? (sz > dev->mtu) :
						(sz > (dev->mtu + ETH_HLEN))) {
			pr_err("rmnet_recv() discarding %d len (%d mtu)\n",
				sz, RMNET_IS_MODE_IP(opmode) ?
					dev->mtu : (dev->mtu + ETH_HLEN));
			ptr = 0;
		} else {
			skb = dev_alloc_skb(sz + NET_IP_ALIGN);
			if (skb == NULL) {
				pr_err("rmnet_recv() cannot allocate skb\n");
			} else {
				skb->dev = dev;
				skb_reserve(skb, NET_IP_ALIGN);
				ptr = skb_put(skb, sz);
				wake_lock_timeout(&p->wake_lock, HZ / 2);
				if (smd_read(p->ch, ptr, sz) != sz) {
					pr_err("rmnet_recv() smd lied about avail?!");
					ptr = 0;
					dev_kfree_skb_irq(skb);
				} else {
					/* Handle Rx frame format */
					spin_lock_irqsave(&p->lock, flags);
					opmode = p->operation_mode;
					spin_unlock_irqrestore(&p->lock, flags);

					if (RMNET_IS_MODE_IP(opmode)) {
						/* Driver in IP mode */
						skb->protocol =
						  rmnet_ip_type_trans(skb, dev);
					} else {
						/* Driver in Ethernet mode */
						skb->protocol =
						  eth_type_trans(skb, dev);
					}
					if (RMNET_IS_MODE_IP(opmode) ||
					    count_this_packet(ptr, skb->len)) {
#ifdef CONFIG_MSM_RMNET_DEBUG
						p->wakeups_rcv +=
							rmnet_cause_wakeup(p);
#endif
						p->stats.rx_packets++;
						p->stats.rx_bytes += skb->len;
					}
					netif_rx(skb);
				}
				continue;
			}
		}
		if (smd_read(p->ch, ptr, sz) != sz)
			pr_err("rmnet_recv() smd lied about avail?!");
	}
}

static DECLARE_TASKLET(smd_net_data_tasklet, smd_net_data_handler, 0);

static int _rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	smd_channel_t *ch = p->ch;
	int smd_ret;
	struct QMI_QOS_HDR_S *qmih;
	u32 opmode;
	unsigned long flags;

	/* For QoS mode, prepend QMI header and assign flow ID from skb->mark */
	spin_lock_irqsave(&p->lock, flags);
	opmode = p->operation_mode;
	spin_unlock_irqrestore(&p->lock, flags);

	if (RMNET_IS_MODE_QOS(opmode)) {
		qmih = (struct QMI_QOS_HDR_S *)
			skb_push(skb, sizeof(struct QMI_QOS_HDR_S));
		qmih->version = 1;
		qmih->flags = 0;
		qmih->flow_id = skb->mark;
	}

	dev->trans_start = jiffies;
	smd_ret = smd_write(ch, skb->data, skb->len);
	if (smd_ret != skb->len) {
		pr_err("%s: smd_write returned error %d", __func__, smd_ret);
		goto xmit_out;
	}

	if (RMNET_IS_MODE_IP(opmode) ||
	    count_this_packet(skb->data, skb->len)) {
		p->stats.tx_packets++;
		p->stats.tx_bytes += skb->len;
#ifdef CONFIG_MSM_RMNET_DEBUG
		p->wakeups_xmit += rmnet_cause_wakeup(p);
#endif
	}

xmit_out:
	/* data xmited, safe to release skb */
	dev_kfree_skb_irq(skb);
	return 0;
}

static void _rmnet_resume_flow(unsigned long param)
{
	struct net_device *dev = (struct net_device *)param;
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb = NULL;
	unsigned long flags;

	/* xmit and enable the flow only once even if
	   multiple tasklets were scheduled by smd_net_notify */
	spin_lock_irqsave(&p->lock, flags);
	if (p->skb && (smd_write_avail(p->ch) >= p->skb->len)) {
		skb = p->skb;
		p->skb = NULL;
		spin_unlock_irqrestore(&p->lock, flags);
		_rmnet_xmit(skb, dev);
		netif_wake_queue(dev);
	} else
		spin_unlock_irqrestore(&p->lock, flags);
}

static void msm_rmnet_unload_modem(void *pil)
{
	/* printk("%s() ++\n", __func__); */
	printk("pil: %s", ( pil? "true" : "false" ));
	if (pil)
		pil_put(pil);
	/* printk("%s() --\n", __func__); */
}

static void *msm_rmnet_load_modem(struct net_device *dev)
{
	void *pil;
	int rc;
	struct rmnet_private *p = netdev_priv(dev);

	/* printk("%s() ++\n", __func__); */

	pil = pil_get("modem");
	if (IS_ERR(pil))
		pr_err("%s: modem load failed\n", __func__);
	else if (msm_rmnet_modem_wait) {
		printk("wait_for_completion_interruptible_timeout ..\n");
		rc = wait_for_completion_interruptible_timeout(
			&p->complete,
			msecs_to_jiffies(msm_rmnet_modem_wait * 1000));
		printk("wait_for_completion_interruptible_timeout .. Done\n");
		if (!rc){
			rc = -ETIMEDOUT;
			printk("Modem load timeout\n");
		}
		if (rc < 0) {
			pr_err("%s: wait for rmnet port failed %d\n",
			       __func__, rc);
			msm_rmnet_unload_modem(pil);
			pil = ERR_PTR(rc);
		}
	}

	/* printk("%s() --\n", __func__); */
	return pil;
}

static void smd_net_notify(void *_dev, unsigned event)
{
	struct rmnet_private *p = netdev_priv((struct net_device *)_dev);

	if (event != SMD_EVENT_DATA)
		return;

	spin_lock(&p->lock);
	if (p->skb && (smd_write_avail(p->ch) >= p->skb->len))
		tasklet_hi_schedule(&p->tsklt);

	spin_unlock(&p->lock);

	if (smd_read_avail(p->ch) &&
	    (smd_read_avail(p->ch) >= smd_cur_packet_size(p->ch))) {
		smd_net_data_tasklet.data = (unsigned long) _dev;
		tasklet_schedule(&smd_net_data_tasklet);
	}
}

static int __rmnet_open(struct net_device *dev)
{
	int r;
	void *pil;
	struct rmnet_private *p = netdev_priv(dev);

	/* printk("%s() ++\n", __func__); */

	mutex_lock(&p->pil_lock);
	if (!p->pil) {
		pil = msm_rmnet_load_modem(dev);
		if (IS_ERR(pil)) {
			mutex_unlock(&p->pil_lock);
			return PTR_ERR(pil);
		}
		p->pil = pil;
	}
	mutex_unlock(&p->pil_lock);

	if (!p->ch) {
		r = smd_open(p->chname, &p->ch, dev, smd_net_notify);

		if (r < 0)
			return -ENODEV;
	}

	/* printk("%s() --\n", __func__); */
	return 0;
}

static int __rmnet_close(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	int rc;
	unsigned long flags;

	if (p->ch) {
		rc = smd_close(p->ch);
		spin_lock_irqsave(&p->lock, flags);
		p->ch = 0;
		spin_unlock_irqrestore(&p->lock, flags);
		return rc;
	} else
		return -EBADF;
}

static int rmnet_open(struct net_device *dev)
{
	int rc = 0;

	pr_info("msm_rmnet: %s\n",  __func__);

	rc = __rmnet_open(dev);

	netif_start_queue(dev);

	return rc;
}

static int rmnet_stop(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);

	pr_info("rmnet_stop()\n");

	netif_stop_queue(dev);
	tasklet_kill(&p->tsklt);

	/* TODO: unload modem safely,
	   currently, this causes unnecessary unloads */
	/*
	mutex_lock(&p->pil_lock);
	msm_rmnet_unload_modem(p->pil);
	p->pil = NULL;
	mutex_unlock(&p->pil_lock);
	*/

	return 0;
}

static int rmnet_change_mtu(struct net_device *dev, int new_mtu)
{
	if (0 > new_mtu || RMNET_DATA_LEN < new_mtu)
		return -EINVAL;

	dev->mtu = new_mtu;

	return 0;
}

static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	smd_channel_t *ch = p->ch;
	unsigned long flags;

	if (netif_queue_stopped(dev)) {
		pr_err("fatal: rmnet_xmit called when netif_queue is stopped");
		return 0;
	}

	spin_lock_irqsave(&p->lock, flags);
	if (smd_write_avail(ch) < skb->len) {
		netif_stop_queue(dev);
		p->skb = skb;
		spin_unlock_irqrestore(&p->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&p->lock, flags);

	_rmnet_xmit(skb, dev);

	return 0;
}

static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	return &p->stats;
}

static void rmnet_set_multicast_list(struct net_device *dev)
{
}

static void rmnet_tx_timeout(struct net_device *dev)
{
	pr_info("rmnet_tx_timeout()\n");
}


static const struct net_device_ops rmnet_ops_ether = {
	.ndo_open		= rmnet_open,
	.ndo_stop		= rmnet_stop,
	.ndo_start_xmit		= rmnet_xmit,
	.ndo_get_stats		= rmnet_get_stats,
	.ndo_set_multicast_list = rmnet_set_multicast_list,
	.ndo_tx_timeout		= rmnet_tx_timeout,
	.ndo_do_ioctl		= rmnet_ioctl,
	.ndo_change_mtu		= rmnet_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static const struct net_device_ops rmnet_ops_ip = {
	.ndo_open		= rmnet_open,
	.ndo_stop		= rmnet_stop,
	.ndo_start_xmit		= rmnet_xmit,
	.ndo_get_stats		= rmnet_get_stats,
	.ndo_set_multicast_list = rmnet_set_multicast_list,
	.ndo_tx_timeout		= rmnet_tx_timeout,
	.ndo_do_ioctl		= rmnet_ioctl,
	.ndo_change_mtu		= rmnet_change_mtu,
	.ndo_set_mac_address	= 0,
	.ndo_validate_addr	= 0,
};

static int rmnet_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct rmnet_private *p = netdev_priv(dev);
	u32 old_opmode = p->operation_mode;
	unsigned long flags;
	int prev_mtu = dev->mtu;
	int rc = 0;

	/* Process IOCTL command */
	switch (cmd) {
	case RMNET_IOCTL_SET_LLP_ETHERNET:  /* Set Ethernet protocol   */
		/* Perform Ethernet config only if in IP mode currently*/
		if (p->operation_mode & RMNET_MODE_LLP_IP) {
			ether_setup(dev);
			random_ether_addr(dev->dev_addr);
			dev->mtu = prev_mtu;

			dev->netdev_ops = &rmnet_ops_ether;
			spin_lock_irqsave(&p->lock, flags);
			p->operation_mode &= ~RMNET_MODE_LLP_IP;
			p->operation_mode |= RMNET_MODE_LLP_ETH;
			spin_unlock_irqrestore(&p->lock, flags);
			pr_info("rmnet_ioctl(): "
				"set Ethernet protocol mode\n");
		}
		break;

	case RMNET_IOCTL_SET_LLP_IP:        /* Set RAWIP protocol      */
		/* Perform IP config only if in Ethernet mode currently*/
		if (p->operation_mode & RMNET_MODE_LLP_ETH) {

			/* Undo config done in ether_setup() */
			dev->header_ops         = 0;  /* No header */
			dev->type               = ARPHRD_RAWIP;
			dev->hard_header_len    = 0;
			dev->mtu                = prev_mtu;
			dev->addr_len           = 0;
			dev->flags              &= ~(IFF_BROADCAST|
						     IFF_MULTICAST);

			dev->netdev_ops = &rmnet_ops_ip;
			spin_lock_irqsave(&p->lock, flags);
			p->operation_mode &= ~RMNET_MODE_LLP_ETH;
			p->operation_mode |= RMNET_MODE_LLP_IP;
			spin_unlock_irqrestore(&p->lock, flags);
			pr_info("rmnet_ioctl(): set IP protocol mode\n");
		}
		break;

	case RMNET_IOCTL_GET_LLP:           /* Get link protocol state */
		ifr->ifr_ifru.ifru_data =
			(void *)(p->operation_mode &
				(RMNET_MODE_LLP_ETH|RMNET_MODE_LLP_IP));
		break;

	case RMNET_IOCTL_SET_QOS_ENABLE:    /* Set QoS header enabled  */
		spin_lock_irqsave(&p->lock, flags);
		p->operation_mode |= RMNET_MODE_QOS;
		spin_unlock_irqrestore(&p->lock, flags);
		pr_info("rmnet_ioctl(): set QMI QOS header enable\n");
		break;

	case RMNET_IOCTL_SET_QOS_DISABLE:   /* Set QoS header disabled */
		spin_lock_irqsave(&p->lock, flags);
		p->operation_mode &= ~RMNET_MODE_QOS;
		spin_unlock_irqrestore(&p->lock, flags);
		pr_info("rmnet_ioctl(): set QMI QOS header disable\n");
		break;

	case RMNET_IOCTL_GET_QOS:           /* Get QoS header state    */
		ifr->ifr_ifru.ifru_data =
			(void *)(p->operation_mode & RMNET_MODE_QOS);
		break;

	case RMNET_IOCTL_GET_OPMODE:        /* Get operation mode      */
		ifr->ifr_ifru.ifru_data = (void *)p->operation_mode;
		break;

	case RMNET_IOCTL_OPEN:              /* Open transport port     */
		rc = __rmnet_open(dev);
		pr_info("rmnet_ioctl(): open transport port\n");
		break;

	case RMNET_IOCTL_CLOSE:             /* Close transport port    */
		rc = __rmnet_close(dev);
		pr_info("rmnet_ioctl(): close transport port\n");
		break;

	default:
		pr_err("error: rmnet_ioct called for unsupported cnd %d", cmd);
		return -EINVAL;
	}

	pr_debug("%s: dev=%s cmd=0x%x opmode old=0x%08x new=0x%08x\n",
		__func__, p->chname, cmd, old_opmode, p->operation_mode);
	return rc;
}


static void __init rmnet_setup(struct net_device *dev)
{
	pr_info("msm_rmnet: %s\n",  __func__);

	/* Using Ethernet mode by default */
	dev->netdev_ops = &rmnet_ops_ether;
	ether_setup(dev);

	/* set this after calling ether_setup */
	dev->mtu = RMNET_DATA_LEN;
	dev->needed_headroom = HEADROOM_FOR_QOS;

	random_ether_addr(dev->dev_addr);

	dev->watchdog_timeo = 1000; /* 10 seconds? */
	pr_info("msm_rmnet: %s complete\n",  __func__);
}

static int msm_rmnet_smd_probe(struct platform_device *pdev)
{
	int i;

	pr_info("msm_rmnet: %s\n",  __func__);
	for (i = 0; i < 8; i++)
		if (!strcmp(pdev->name, ch_name[i])) {
			complete_all(port_complete[i]);
			break;
		}

	pr_info("msm_rmnet: %s complete\n",  __func__);
	return 0;
}

static int __init rmnet_init(void)
{
	int ret;
	struct device *d;
	struct net_device *dev;
	struct rmnet_private *p;
	unsigned n;

	pr_info("msm_rmnet: %s\n",  __func__);

#ifdef CONFIG_MSM_RMNET_DEBUG
	timeout_us = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
	timeout_suspend_us = 0;
#endif
#endif

	for (n = 0; n < 8; n++) {
		dev = alloc_netdev(sizeof(struct rmnet_private),
				   "rmnet%d", rmnet_setup);

		if (!dev)
			return -ENOMEM;

		d = &(dev->dev);
		p = netdev_priv(dev);
		p->chname = ch_name[n];
		/* Initial config uses Ethernet */
		p->operation_mode = RMNET_MODE_LLP_ETH;
		p->skb = NULL;
		spin_lock_init(&p->lock);
		tasklet_init(&p->tsklt, _rmnet_resume_flow,
				(unsigned long)dev);
		wake_lock_init(&p->wake_lock, WAKE_LOCK_SUSPEND, ch_name[n]);
#ifdef CONFIG_MSM_RMNET_DEBUG
		p->timeout_us = timeout_us;
		p->wakeups_xmit = p->wakeups_rcv = 0;
#endif

		init_completion(&p->complete);
		port_complete[n] = &p->complete;
		mutex_init(&p->pil_lock);
		p->pdrv.probe = msm_rmnet_smd_probe;
		p->pdrv.driver.name = ch_name[n];
		p->pdrv.driver.owner = THIS_MODULE;
		ret = platform_driver_register(&p->pdrv);
		if (ret) {
			printk("rmnet_init platform_driver_register, fail\n");
			free_netdev(dev);
			return ret;
		}

		ret = register_netdev(dev);
		if (ret) {
			platform_driver_unregister(&p->pdrv);
			free_netdev(dev);
			return ret;
		}

#ifdef CONFIG_MSM_RMNET_DEBUG
		if (device_create_file(d, &dev_attr_timeout))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_xmit))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_rcv))
			continue;
#ifdef CONFIG_HAS_EARLYSUSPEND
		if (device_create_file(d, &dev_attr_timeout_suspend))
			continue;

		/* Only care about rmnet0 for suspend/resume tiemout hooks. */
		if (n == 0)
			rmnet0 = d;
#endif
#endif
	}

	pr_info("msm_rmnet: %s complete\n",  __func__);
	return 0;
}

module_init(rmnet_init);
