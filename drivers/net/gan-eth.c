/*
 *	gan-eth.c: "gannet" compatibility for Kineto GAN
 *
 *	Packets received on UDP PORT_RX:
 *		strip headers, add ETH header and reintroduce
 *	Packets sent via interface:
 *		strip ETH header, send to UDP localhost:PORT_TX
 *
 *	And yes, this is a very hackish way to add/strip IP/UDP headers.
 *	Blame Kineto.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/workqueue.h>
#include <linux/errno.h>

#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>

#include <linux/socket.h>
#include <linux/in.h>

#include <net/udp.h>

#define MODULE_NAME	"ganeth"

#define PORT_TX	13001
#define PORT_RX	13010

#define IP_DEST	INADDR_LOOPBACK

struct ganeth_priv {
	struct socket *tx, *rx;
	struct sockaddr_in tx_saddr, rx_saddr;

	struct workqueue_struct *workqueue;
	struct work_struct tx_work;
	struct sk_buff_head queue;
};

static struct net_device *netdev;

static int ganeth_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

static int ganeth_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static int ganeth_sendmsg(struct net_device *dev, void *buf, int len) {
	struct ganeth_priv *priv = netdev_priv(dev);
	struct msghdr msg = {
		.msg_name = &priv->tx_saddr,
		.msg_namelen = sizeof(priv->tx_saddr),
		.msg_flags = MSG_NOSIGNAL | MSG_DONTWAIT,
	};
	struct kvec iov = {
		.iov_base = buf,
		.iov_len = len,
	};
	int err;

	err = kernel_sendmsg(priv->tx, &msg, &iov, 1, len);
	if (err >= 0) {
		dev->stats.tx_packets++;
		dev->stats.tx_bytes += len;
	} else {
		dev->stats.tx_errors++;
	}

	return err;
}

static void ganeth_tx_work(struct work_struct *work)
{
	struct ganeth_priv *priv = netdev_priv(netdev);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&priv->queue))) {
		ganeth_sendmsg(netdev, skb->data + ETH_HLEN, skb->len - ETH_HLEN);
		dev_kfree_skb(skb);
	}
}

static netdev_tx_t ganeth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ganeth_priv *priv = netdev_priv(dev);

	if (skb->protocol != htons(ETH_P_IP)) {
		if (skb->protocol == htons(ETH_P_IPV6)) {
			/* Silently drop IPV6 */
			goto out_drop;
		}

		pr_warning("%s: dropping packet with protocol %d\n", dev->name, ntohs(skb->protocol));
		goto out_tx_err;
	}

	if (skb->len < ETH_HLEN) {
		pr_err("%s: short packet\n", dev->name);
		goto out_tx_err;
	}

	skb_queue_tail(&priv->queue, skb);
	queue_work(priv->workqueue, &priv->tx_work);
	return NETDEV_TX_OK;

out_tx_err:
	dev->stats.tx_errors++;
out_drop:
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void ganeth_tx_timeout(struct net_device *dev)
{
	netif_wake_queue(dev);
}

static void ganeth_rx_data_ready(struct sock *sk, int len)
{
	struct sk_buff *skb, *rxskb;
	struct ethhdr *eth;
	void *buf;
	unsigned int ulen;
	int err;

	skb = skb_recv_datagram(sk, 0, 1, &err);

	if (!skb)
		goto out_err;

	ulen = skb->len - sizeof(struct udphdr);

	if (!skb_csum_unnecessary(skb)) {
		if (udp_lib_checksum_complete(skb)) {
			pr_err("%s: checksum error\n", netdev->name);
			goto out_err;
		}
	}

	dst_confirm(skb_dst(skb));

	rxskb = dev_alloc_skb(ulen + ETH_HLEN + NET_IP_ALIGN);
	if (!rxskb) {
		pr_err("%s: failed to allocate skb\n", netdev->name);
		goto out_err;
	}

	skb_reserve(rxskb, NET_IP_ALIGN);

	/* Ethernet header */
	eth = (struct ethhdr *)skb_put(rxskb, ETH_HLEN);
	memset(eth->h_dest, 0, ETH_ALEN);
	memcpy(eth->h_source, netdev->dev_addr, ETH_ALEN);
	eth->h_proto = htons(ETH_P_IP);

	/* data */
	buf = skb_put(rxskb, ulen);
	memcpy(buf, skb->data + sizeof(struct udphdr), ulen);

	rxskb->dev = netdev;

	skb_reset_mac_header(rxskb);
	rxskb->protocol = eth->h_proto;
	skb_pull(rxskb, ETH_HLEN); /* Eat ethernet header */

	rxskb->ip_summed = CHECKSUM_NONE;
	rxskb->pkt_type = PACKET_HOST;

	if (netif_rx(rxskb) == NET_RX_DROP)
		goto out_err;

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += rxskb->len;
	skb_free_datagram(sk, skb);
	return;

out_err:
	netdev->stats.rx_errors++;
	netdev->stats.rx_dropped++;
	skb_free_datagram(sk, skb);
	return;

}

static const struct net_device_ops ganeth_netdev_ops = {
	.ndo_open = ganeth_open,
	.ndo_stop = ganeth_stop,
	.ndo_start_xmit = ganeth_start_xmit,
	.ndo_tx_timeout = ganeth_tx_timeout,
};

static void __init ganeth_setup(struct net_device *dev)
{
	ether_setup(dev);

	dev->netdev_ops = &ganeth_netdev_ops;

	dev->flags |= IFF_NOARP;
	dev->features |= NETIF_F_NETNS_LOCAL;

	dev->mtu = 1320;

	dev->watchdog_timeo = msecs_to_jiffies(2000);

	random_ether_addr(dev->dev_addr);
}

static int __init ganeth_init(void)
{
	struct net_device *dev;
	struct ganeth_priv *priv;
	int err;

	netdev = dev = alloc_netdev(sizeof(*priv), "gannet%d", ganeth_setup);

	if (!dev)
		return -ENOMEM;

	err = register_netdev(dev);
	if (err) {
		pr_err("%s: error registering netdev\n", __func__);
		goto out_free;
	}

	priv = netdev_priv(dev);

	priv->workqueue = create_workqueue("gannet");
	INIT_WORK(&priv->tx_work, ganeth_tx_work);
	skb_queue_head_init(&priv->queue);

	/* tx */
	err = sock_create_kern(PF_INET, SOCK_DGRAM, IPPROTO_UDP, &priv->tx);
	if (err < 0) {
		pr_err("%s: error creating tx socket\n", dev->name);
		goto out_unregister;
	}

	memset(&priv->tx_saddr, 0, sizeof(priv->tx_saddr));
	priv->tx_saddr.sin_family = AF_INET;
	priv->tx_saddr.sin_addr.s_addr = htonl(IP_DEST);
	priv->tx_saddr.sin_port = htons(PORT_TX);

	/* rx */
	err = sock_create_kern(PF_INET, SOCK_DGRAM, IPPROTO_UDP, &priv->rx);
	if (err < 0) {
		pr_err("%s: error creating rx socket\n", dev->name);
		goto out_release_tx;
	}
	priv->rx->sk->sk_data_ready = ganeth_rx_data_ready;

	memset(&priv->rx_saddr, 0, sizeof(priv->rx_saddr));
	priv->rx_saddr.sin_family = AF_INET;
	priv->rx_saddr.sin_addr.s_addr = htonl(INADDR_ANY);
	priv->rx_saddr.sin_port = htons(PORT_RX);

	err = priv->rx->ops->bind(
		priv->rx,
		(struct sockaddr *)&priv->rx_saddr,
		sizeof(priv->rx_saddr)
	);
	if (err < 0) {
		pr_err("%s: error binding rx socket\n", dev->name);
		goto out_release_rx;
	}

	return 0;

out_release_rx:
	sock_release(priv->rx);
out_release_tx:
	sock_release(priv->tx);
out_unregister:
	unregister_netdev(dev);
out_free:
	free_netdev(dev);
	return err;
}

static void __exit ganeth_exit(void)
{
	struct ganeth_priv *priv;

	if (!netdev)
		return;

	priv = netdev_priv(netdev);
	sock_release(priv->rx);
	sock_release(priv->tx);
	unregister_netdev(netdev);
	destroy_workqueue(priv->workqueue);
	free_netdev(netdev);

	netdev = NULL;
}

module_init(ganeth_init);
module_exit(ganeth_exit);

MODULE_AUTHOR("Christopher Lais <chris+android@zenthought.org>");
MODULE_DESCRIPTION("Virtual IP over UDP Ethernet Device");
MODULE_LICENSE("GPL");
MODULE_ALIAS("gannet");
