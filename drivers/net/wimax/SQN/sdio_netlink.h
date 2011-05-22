#ifndef _SDIO_NETLINK_H
#define _SDIO_NETLINK_H

#include <linux/module.h>  
#include <linux/kernel.h> 
#include <linux/init.h> 
#include <net/sock.h>
#include <net/netlink.h>
#include <linux/skbuff.h>

void udp_broadcast(int gid,void *payload);
void MyTimerFunction(unsigned long data);
void udp_receive(struct sk_buff  *skb);
int sdio_netlink_register(void);
void sdio_netlink_deregister(void);

#endif
