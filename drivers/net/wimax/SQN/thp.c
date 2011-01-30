/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2009 SEQUANS Communications
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */


#include <linux/version.h>
#include <linux/if_ether.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#include <linux/devfs_fs_kernel.h>
#endif
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/netdevice.h>
#include <linux/cdev.h>
#include <linux/byteorder/generic.h>


#include "version.h"
#include "msg.h"
#include "thp.h"

#include "thp_ioctl.h"

#define THP_TRACE	0	/* print info messages from THP read/write handlers */
#define THP_HEADER_DUMP	0	/* verbosely dump header of THP TX/RX packets */

#define THP_DEBUG 0
#define SKB_DEBUG 0
#define DRVREV  SQN_MODULE_VERSION

extern bool drop_packet;

const uint8_t  host_macaddr[ETH_ALEN] 	= {0x00, 0x16, 0x08, 0xff, 0x00, 0x01};
const uint8_t  ss_macaddr[ETH_ALEN] 	= {0x00, 0x16, 0x08, 0xff, 0x00, 0x00};

extern int sqn_sdio_dump_net_pkt(int on);
extern int mmc_wimax_get_thp_log(void);

// Queue of packets destined to the Connection Manager
// TODO: check size of the queue, it's should always be one.
struct sk_buff_head to_sqntool_queue;

DECLARE_WAIT_QUEUE_HEAD(to_sqntool_wait);

struct net_device *this_device = NULL;

struct packet_type rx_packet_type = { 0 };
extern int mmc_wimax_uart_switch(int uart);

uint8_t is_thp_packet(uint8_t  *dest_addr)
{
	return (memcmp(dest_addr, host_macaddr, ETH_ALEN)==0);
}

inline struct ethhdr *skb2ethhdr(const struct sk_buff *skb)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
	return (struct ethhdr *)skb->mac.raw;
#else
	return (struct ethhdr *)skb_mac_header(skb);
#endif
}


/* TODO: Fix kernel version */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,10)
int thp_handler(struct sk_buff *skb, struct net_device *pDev, struct packet_type *pPt, struct net_device *pOrigDev)
#else
int thp_handler(struct sk_buff *skb, struct net_device *pDev, struct packet_type *pPt)
#endif
{
	struct sk_buff *skb_thp = 0;
	struct ethhdr *eth = 0;

	sqn_pr_enter();

	/* We need only ETH_P_802_2 protocol packets with THP mac address */
	eth = skb2ethhdr(skb);
	if(ntohs(skb->protocol) != ETH_P_802_2 || !is_thp_packet(eth->h_dest)) {
		//for DDTM, drop all NOT THP packets
		if(drop_packet) {
			sqn_pr_dbg("HTC CODE: drop packet for DDTM\n");
			skb->pkt_type = PACKET_OTHERHOST;
		}
		goto not_thp_out;
	}

	skb_thp = skb_clone(skb, GFP_ATOMIC);
    /* Bugz 22554: strip CRC at the end of packet */
    skb_trim(skb_thp, skb_thp->len - 4);

#if THP_TRACE
	sqn_pr_info("%s: RX packet, len = %d\n", __func__, skb_thp->len);
#endif
	sqn_pr_dbg("RX THP packet, length %d\n", skb_thp->len);
	skb_queue_tail(&to_sqntool_queue, skb_thp);

	if(skb_queue_len(&to_sqntool_queue) == 256){
		skb_thp = skb_dequeue(&to_sqntool_queue);
		kfree_skb(skb_thp);
	}

	wake_up_interruptible(&to_sqntool_wait);	//Wake up wait queue

thp_out:
    dev_kfree_skb_any(skb); 
	sqn_pr_leave();
	return NET_RX_DROP;
not_thp_out:
	dev_kfree_skb_any(skb);
	sqn_pr_leave();
	return NET_RX_SUCCESS;
}

// Initialization function for THP handler
int  init_thp_handler(struct net_device *dev)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "init_thp_handler +\n");
#endif

	skb_queue_head_init(&to_sqntool_queue);

	/* Define type of intercepted packets */
	rx_packet_type.type = htons(ETH_P_ALL);    /* Intercept all packets */
	rx_packet_type.dev  = dev;
	rx_packet_type.func = thp_handler;     /* Network packet handler function */

	/* Register packet handler */
	dev_add_pack(&rx_packet_type);

#if THP_DEBUG
	printk(KERN_WARNING "init_thp_handler -\n");
#endif
	sqn_pr_leave();

	return 0;
}

// Clean up function for THP handler
void cleanup_thp_handler(void)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "cleanup_thp_handler +\n");
#endif

	/* unregister packet handler */
	dev_remove_pack(&rx_packet_type);

	if(!skb_queue_empty(&to_sqntool_queue))
		skb_queue_purge(&to_sqntool_queue) ;

#if THP_DEBUG
	printk(KERN_WARNING "cleanup_thp_handler -\n");
#endif
	sqn_pr_leave();
}

#define PROC_DIR_NAME 	"kthp"
#define DRV_REVISION 	"kthp/drvrev"
#define IFACE_FILENAME 	"kthp/iface_name"

char procfs_dir[64] = PROC_DIR_NAME;

static struct proc_dir_entry*   kthp_proc_dir;

extern struct net_device *this_device;

/** PROC_FS Read Functions */

static int ifacename_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = 0;

	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "ifacename_read +\n");
#endif

	if(this_device)
		len += sprintf(page, "%s\n",  this_device->name);

	*eof = 1;

#if THP_DEBUG
	printk(KERN_WARNING "ifacename_read -\n");
#endif
	sqn_pr_leave();

	return len;
}


static int drvrev_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = 0;

	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "drvrev_read +\n");
#endif

	len += sprintf(page, "%s\n",  SQN_MODULE_VERSION);

	*eof = 1;

	sqn_pr_leave();

	return len;
}

static int install_entry(char *entry_name, read_proc_t* read_func)
{
	struct proc_dir_entry* proc;

	sqn_pr_enter();

	proc = create_proc_read_entry(entry_name, S_IFREG | S_IRUGO | S_IWUSR
			, NULL, (read_proc_t*)read_func, NULL);

	if (proc) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
		proc->owner = THIS_MODULE;
#endif
	} else {
		printk(KERN_ALERT"/proc/ %s failed", entry_name);
		return 1;
	}

	sqn_pr_leave();

	return 0;
}

int init_procfs_handler(void)
{
	sqn_pr_enter();

	kthp_proc_dir = proc_mkdir(procfs_dir, NULL);

	if (kthp_proc_dir) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
		kthp_proc_dir->owner = THIS_MODULE;
#endif
	} else {
		remove_proc_entry(PROC_DIR_NAME, NULL);
		return 1;
	}

	if(install_entry(IFACE_FILENAME, ifacename_read) ||
			install_entry(DRV_REVISION, drvrev_read))
	{
		return 1;
	}

#if THP_DEBUG
	printk(KERN_WARNING "drvrev_read -\n");
#endif
	sqn_pr_leave();

	return 0;
}

void cleanup_procfs_handler(void)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "cleanup_procfs_handler +\n");
#endif

	remove_proc_entry(IFACE_FILENAME, NULL);
	remove_proc_entry(DRV_REVISION, NULL);

	if (kthp_proc_dir)
		remove_proc_entry(procfs_dir, NULL);

#if THP_DEBUG
	printk(KERN_WARNING "cleanup_procfs_handler -\n");
#endif
	sqn_pr_leave();
}


#define THP_FILENAME   "thp"


static dev_t dev_num;
//static int dev_index;
static struct cdev *thp_dev;
static struct class *thp_class;
static struct device *thp_device;

static uint8_t once_open_flag = 0;

const char thp_filename[64] = THP_FILENAME;

/********************File operations*****************************/
static int thp_open(struct inode*, struct file*);

static ssize_t thp_release(struct inode*, struct file*);

static ssize_t thp_read(struct file*, char*, size_t, loff_t*);

static ssize_t thp_write(struct file *file, const char *buf,
		size_t count, loff_t *ppos);

static unsigned int thp_poll(struct file *filp, poll_table *wait);

static int thp_ioctl(struct inode*, struct file*, unsigned int, unsigned long);

struct file_operations thp_fops =
{
	.owner	=	THIS_MODULE
	, .open	=	thp_open
	, .release=	thp_release
	, .read	=	thp_read
	, .write=	thp_write
	, .poll	=	thp_poll
	, .ioctl =	thp_ioctl
};

/********************** File Operations BEGIN  *****************************/

static int thp_open(struct inode * inode, struct file * filp)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "thp_open +\n");
#endif

	// allow multiple open() call for supporting ioctl on HTC Supersonic
	/*
	if(once_open_flag)
		return -EBUSY;
	*/

	once_open_flag = 1;

#if THP_DEBUG
	printk(KERN_WARNING "thp_open -\n");
#endif
	sqn_pr_leave();

	return 0;
}

static ssize_t thp_release(struct inode *inode, struct file *filp)
{
	sqn_pr_enter();

	once_open_flag = 0;

	if(!skb_queue_empty(&to_sqntool_queue))
		skb_queue_purge(&to_sqntool_queue);

	sqn_pr_leave();

	return 0;
}

static ssize_t thp_read(struct file *filp, char *buf, size_t count, loff_t*ppos)
{
	DECLARE_WAITQUEUE(wait, current);

	struct sk_buff_head *head = &to_sqntool_queue;
	struct sk_buff *curr = NULL;
	ssize_t retval;
	const struct sqn_thp_header *th = 0;

	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "thp_read +\n");
#endif

	add_wait_queue(&to_sqntool_wait, &wait);
	retval = -ERESTARTSYS;

	if(0 == this_device) {
		printk(KERN_WARNING "thp_read() device removed\n");
		retval = -EINVAL;
		goto out;
	}

	while(1)
	{
		if(!skb_queue_empty(head))
			break;
		if(signal_pending(current) || 0 == this_device) {
			printk(KERN_WARNING "thp_read() interrupted by signal\n");
			retval = -EINTR;
			goto out;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	curr = skb_dequeue(head);

	if (count < curr->len) {
		printk(KERN_WARNING "%s: userspace buffer is too small (%u bytes)"
			" to hold THP packet (%u bytes)\n"
			, __func__, count, curr->len);
		retval = -EINVAL;
		goto free_skb;
	} else {
		count =  curr->len;
	}

	if(copy_to_user(buf, curr->data, count))
	{
		printk(KERN_ERR "error copying data to user space\n");
		retval = -EFAULT;
		goto free_skb;
	}

    if (mmc_wimax_get_thp_log()) {
        sqn_pr_info("%s: [to_user]: len = %d\n", __func__, count);
        th = (struct sqn_thp_header *) curr->data;
        sqn_pr_info("%s:  PKTLen: %4u | TVer: 0x0%x | Flags: 0x0%x | Len: %4u"
    		" | SeqNum: %5u | AckNum: %5u | TLen: %5u\n", __func__
    		, count
    		, th->transport_version
    		, th->flags
    		, be16_to_cpu(th->length)
    		, be16_to_cpu(th->seq_number)
    		, be16_to_cpu(th->ack_number)
    		, be32_to_cpu(th->total_length));
        sqn_pr_dbg_dump("THP RX:", curr->data, count);
    }

#if SKB_DEBUG
    sqn_pr_info("%s: free skb [0x%p], users %d\n", __func__, curr, atomic_read(&curr->users));
#endif

    retval = (ssize_t)count;
free_skb:

    dev_kfree_skb_any(curr);

out:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&to_sqntool_wait, &wait);

#if THP_DEBUG
	printk(KERN_WARNING "thp_read -\n");
#endif
	sqn_pr_leave();

	return retval;
}


static ssize_t thp_write(struct file *file, const char *buf,
		size_t count, loff_t *ppos)
{

	ssize_t retval = -ENOMEM;
	struct sk_buff *skb;
	struct ethhdr ethh;
	int size = count + ETH_HLEN;
	const struct sqn_thp_header *th = 0;

	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "thp_write +\n");
#endif

	if(0 == this_device)
		return -ENODEV;

	skb = __dev_alloc_skb(size, GFP_ATOMIC | GFP_DMA);
	if(skb == NULL)
		return retval;

#if SKB_DEBUG
    sqn_pr_info("%s: [0x%p] alloc skb, users %d\n", __func__, skb, atomic_read(&skb->users)); 
#endif

	memcpy(ethh.h_dest, ss_macaddr, ETH_ALEN);
	memcpy(ethh.h_source, host_macaddr, ETH_ALEN);
	ethh.h_proto = htons(count);

	memcpy(skb->data, &ethh, sizeof(struct ethhdr));
	skb_put(skb, sizeof(struct ethhdr));

	if(copy_from_user(skb->tail, buf, count)) {
		dev_kfree_skb_any(skb);
		return  -EFAULT;
	}
	skb_put(skb, count);

    if (mmc_wimax_get_thp_log()) {
        sqn_pr_info("%s: [from_user]: len = %d\n", __func__, count);
        th = (struct sqn_thp_header *) buf;
    	sqn_pr_info("%s: PKTLen: %4u | TVer: 0x0%x | Flags: 0x0%x | Len: %4u"
    		" | SeqNum: %5u | AckNum: %5u | TLen: %5u\n", __func__
    		, count
    		, th->transport_version
    		, th->flags
    		, be16_to_cpu(th->length)
    		, be16_to_cpu(th->seq_number)
    		, be16_to_cpu(th->ack_number)
    		, be32_to_cpu(th->total_length));

        sqn_pr_dbg_dump("THP TX:", skb->data, count);
    }	

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)
	this_device->hard_start_xmit(skb, this_device);
#else
	this_device->netdev_ops->ndo_start_xmit(skb, this_device);
#endif

	retval = count;

#if THP_DEBUG
	printk(KERN_WARNING "thp_write -\n");
#endif
	sqn_pr_leave();

	return retval;
}


/*

*/
static unsigned int thp_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	sqn_pr_enter();
#if THP_DEBUG
	//printk(KERN_WARNING "thp_poll +\n");
#endif

	poll_wait(filp, &to_sqntool_wait, wait);

	if (0 == this_device) {
		printk(KERN_WARNING "thp_poll() device removed\n");
		mask = POLLERR;
	} else if(skb_queue_empty(&to_sqntool_queue)) {
		mask = 0;
	} else {
		mask = (POLLIN | POLLRDNORM);
	}

#if THP_DEBUG
	//printk(KERN_WARNING "thp_poll -\n");
#endif
	sqn_pr_leave();

	return mask;
}

static int thp_ioctl(struct inode* dev, struct file* handle, unsigned int cmd, unsigned long arg)
{
#if THP_DEBUG
	printk(KERN_WARNING "thp_ioctl +\n");
#endif
	sqn_pr_enter();

	switch (cmd) {
		case IOCTL_DROP_PACKETS:
			printk(KERN_WARNING "IOCTL_DROP_PACKETS arg=%d\n",(int)arg);
			if(arg == 1)
				drop_packet = true;
			else
				drop_packet = false;
			break;

        case IOCTL_SWITCH_UART:
			printk(KERN_WARNING "IOCTL_SWITCH_UART arg=%d\n",(int)arg);
			if(arg == 1) 
			    mmc_wimax_uart_switch(2); // Wimax
			else 
				mmc_wimax_uart_switch(0); // USB
			break;  
        
		 case IOCTL_SWITCH_NETLOG:
			printk(KERN_WARNING "IOCTL_SWITCH_NETLOG arg=%d\n",(int)arg);
			if(arg == 0) 
			    sqn_sdio_dump_net_pkt(0); // Enable netlog
			else 
				sqn_sdio_dump_net_pkt(1); // Disable netlog
			break;  

		default:
			printk(KERN_WARNING "UNKNOWN OPERATION in thp_ioctl\n");
			return -1;
	}

	sqn_pr_leave();
#if THP_DEBUG
	printk(KERN_WARNING "thp_ioctl -\n");
#endif

	return 0;
}

int init_thp_devfile(void)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "init_thp_devfile +\n");
#endif

	//Dynamic allocation of device number
	if(alloc_chrdev_region(&dev_num, 0, 1, thp_filename))
		return -ENOMEM;

	thp_dev = cdev_alloc();
	if(thp_dev == NULL)
		return -ENOMEM;

	thp_dev->ops = &thp_fops;
	thp_dev->owner = THIS_MODULE;

	if(cdev_add(thp_dev, dev_num, 1))
		return -ENOMEM;

	thp_class = class_create(THIS_MODULE, thp_filename);
	if (IS_ERR(thp_class))
	{
		printk("class_create error(0x%x)\n",(unsigned int)(thp_class));
		return PTR_ERR(thp_class);
	}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	thp_device = device_create(thp_class, NULL, dev_num, NULL, thp_filename);
#else
	thp_device = device_create(thp_class, NULL, dev_num, thp_filename);
#endif
	if (IS_ERR(thp_device))
	{
		printk("device_create error(0x%x)\n", (unsigned int)(thp_device));
		return PTR_ERR(thp_device);
	}

#if THP_DEBUG
	printk(KERN_WARNING "init_thp_devfile -\n");
#endif
	sqn_pr_leave();

	return 0;
}

/**
  \brief dev-fs cleanup function

 *   This function is called in module cleanup function
 */
void cleanup_thp_devfile(void)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "cleanup_thp_devfile +\n");
#endif

	/* Unregister entry from /dev */
	device_destroy(thp_class, dev_num);
	class_destroy(thp_class);
	unregister_chrdev_region(dev_num, 1);
	cdev_del(thp_dev);

#if THP_DEBUG
	printk(KERN_WARNING "cleanup_thp_devfile -\n");
#endif
	sqn_pr_leave();
}
/********************** File Operations END  *****************************/

int thp_wimax_uart_switch(int on)
{
    printk("%s on%d\n", __func__, on);

	if (on) {
        mmc_wimax_uart_switch(2); // Wimax
	}
	else {
        mmc_wimax_uart_switch(0); // USB
	}

	return 0;
}

int init_thp(struct net_device* dev)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "init_thp +\n");
#endif

	if (0 == this_device) {
		if(init_procfs_handler()) {
			return -1;
		}

		if(init_thp_devfile())
			return -1;

		/* Don't call init_thp_handler() here, it will be called from
		 * probe() before interrupts are enabled, to ensure that we will
		 * catch all THP packets as soon as they appear
		 */
		/* if (init_thp_handler(dev)) */
			/* return -1; */

		this_device = dev;

		sqn_pr_info("KTHP initialized\n");
	}

#if THP_DEBUG
	printk(KERN_WARNING "init_thp -\n");
#endif
	sqn_pr_leave();

	return 0;
}


void cleanup_thp(void)
{
	sqn_pr_enter();
#if THP_DEBUG
	printk(KERN_WARNING "cleanup_thp +\n");
#endif

	if (this_device) {
		cleanup_procfs_handler();
		cleanup_thp_handler();
		cleanup_thp_devfile();
		this_device = 0;
		sqn_pr_info("KTHP cleaned up\n");
	}

#if THP_DEBUG
	printk(KERN_WARNING "cleanup_thp -\n");
#endif
	sqn_pr_leave();
}
