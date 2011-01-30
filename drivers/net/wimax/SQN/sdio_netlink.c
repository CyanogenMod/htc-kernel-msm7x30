#include "sdio_netlink.h"

struct sock *netlink_sock;

void udp_broadcast(int gid,void *payload)
{
	struct sk_buff	*skb;
	struct nlmsghdr	*nlh;
	int size=strlen(payload)+1;
	int	len = NLMSG_SPACE(size);
	void *data;
	int ret;
	
	skb = alloc_skb(len, GFP_KERNEL);
	if (!skb)
		return;
	nlh= NLMSG_PUT(skb, 0, 0, 0, size);
	nlh->nlmsg_flags = 0;
	data=NLMSG_DATA(nlh);
	memcpy(data, payload, size);
	NETLINK_CB(skb).pid = 0;         /* from kernel */
	NETLINK_CB(skb).dst_group = gid;  /* unicast */
	ret=netlink_broadcast(netlink_sock, skb, 0, gid, GFP_KERNEL);

	if (ret <0)
	{
		printk("[SDIO] %s send failed\n", __func__);
		return;
	}
	return;
	
nlmsg_failure:			/* Used by NLMSG_PUT */
	if (skb)
		kfree_skb(skb);
}

void MyTimerFunction(unsigned long data)
{
	udp_broadcast(1,"ResetWimax_BySDIO\n");
}

void udp_receive(struct sk_buff  *skb)
{
}

int sdio_netlink_register(void)
{
	netlink_sock = netlink_kernel_create(&init_net, NETLINK_USERSOCK, 0,udp_receive, NULL, THIS_MODULE);
	return 0;
}

void sdio_netlink_deregister(void)
{
	sock_release(netlink_sock->sk_socket);
	printk("[SDIO] %s: netlink driver remove successfully\n", __func__);
}

