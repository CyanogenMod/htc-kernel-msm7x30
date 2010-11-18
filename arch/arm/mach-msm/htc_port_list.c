/* arch/arm/mach-msm/htc_port_list.c
 * Copyright (C) 2009 HTC Corporation.
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

#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/list.h>
#include <mach/msm_iomap.h>
#include <net/tcp.h>
#include "smd_private.h"

static struct mutex port_lock;
static struct wake_lock port_suspend_lock;
static uint16_t *port_list = NULL;
static int usb_enable = 0;
struct p_list {
	struct list_head list;
	int no;
};
static struct p_list curr_port_list;
static int packet_filter_flag = 1;
struct class *p_class;
static struct miscdevice portlist_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-portlist",
};

static ssize_t htc_show(struct device *dev,  struct device_attribute *attr,  char *buf) {
	char *s = buf;
	mutex_lock(&port_lock);
	s += sprintf(s, "%d\n", packet_filter_flag);
	mutex_unlock(&port_lock);
	return s - buf;
}

static ssize_t htc_store(struct device *dev, struct device_attribute *attr,  const char *buf, size_t count) {
	int ret;

	mutex_lock(&port_lock);
	if (!strncmp(buf, "0", strlen("0"))) {
		packet_filter_flag = 0;
		printk(KERN_INFO "[Port list] Disable Packet filter\n");
		if (port_list != NULL)
			port_list[0] = packet_filter_flag;
		else
			printk(KERN_ERR "[Port list] port_list == NULL\n");
		ret = count;
	} else if (!strncmp(buf, "1", strlen("1"))) {
		packet_filter_flag = 1;
		printk(KERN_INFO "[Port list] Enable Packet filter\n");
		if (port_list != NULL)
			port_list[0] = packet_filter_flag;
		else
			printk(KERN_ERR "[Port list] port_list == NULL\n");
		ret = count;
	} else {
		printk(KERN_ERR "[Port list] flag: invalid argument\n");
		ret = -EINVAL;
	}
	mutex_unlock(&port_lock);

	return ret;
}

static DEVICE_ATTR(flag, 0666, htc_show, htc_store);

static int port_list_enable(int enable) {
	if (port_list[0] != enable) {
		port_list[0] = enable;
		if (enable)
			printk(KERN_INFO "[Port list] port_list is enabled.\n");
		else
			printk(KERN_INFO "[Port list] port_list is disabled.\n");
	}
	return 0;
}

static void update_port_list(void) {
	size_t count = 0;
	size_t i = 0;
	struct list_head *listptr;
	struct p_list *entry;

	list_for_each(listptr, &curr_port_list.list) {
		entry = list_entry(listptr, struct p_list, list);
		count++;
		printk(KERN_INFO "[Port list] [%d] = %d\n", count, entry->no);
		if (count <= 127)
			port_list[count] = entry->no;
	}
	if (count < 127)
		for (i = count + 1; i <= 127; i++ )
			port_list[i] = 0;

	if (usb_enable) {
		port_list_enable(0);
	} else {
		if (count <= 127)
			port_list_enable(1);
		else
			port_list_enable(0);
	}
}

static struct p_list *add_list(int no) {
	struct p_list *ptr;

	ptr = kmalloc(sizeof(struct p_list), GFP_KERNEL);
	if(ptr) {
		ptr->no = no;
		list_add_tail(&ptr->list, &curr_port_list.list);
		printk(KERN_INFO "[Port list] Add port [%d]\n", no);
	}
	return (ptr);
}

static void remove_list(int no) {
	struct list_head *listptr;
	struct p_list *entry;
	int get_list = 0;

	list_for_each(listptr, &curr_port_list.list) {
		entry = list_entry(listptr, struct p_list, list);
		if (entry->no == no) {
			printk(KERN_INFO "[Port list] Remove port [%d]\n", entry->no);
			list_del(&entry->list);
			kfree(entry);
			get_list = 1;
			break;
		}
	}
	if(!get_list)
		printk(KERN_INFO "[Port list] Remove failed! Port number is not in list!\n");
}

int add_or_remove_port(struct sock *sk, int add_or_remove) {
	struct inet_sock *inet = inet_sk(sk);
	uint32_t port_list_phy_addr;
	__be32 src = inet->rcv_saddr;
	__u16 srcp = ntohs(inet->sport);

	wake_lock(&port_suspend_lock);
	if (!packet_filter_flag) {
		wake_unlock(&port_suspend_lock);
		return 0;
	}

	if (sk->sk_state != TCP_LISTEN) {
		wake_unlock(&port_suspend_lock);
		return 0;
	}

	/* if source IP != 127.0.0.1 */
	if (src != 0x0100007F && srcp != 0) {
		if (port_list == NULL) {
			port_list = smem_alloc(SMEM_ID_VENDOR2, 256);
			port_list_phy_addr = MSM_SHARED_RAM_PHYS + ((uint32_t)port_list - (uint32_t)MSM_SHARED_RAM_BASE);
			printk(KERN_INFO "[Port list] Virtual Address of port_list: [%p]\n", port_list);
			printk(KERN_INFO "[Port list] Physical Address of port_list: [%X]\n", port_list_phy_addr);
			if (port_list == NULL) {
				printk(KERN_INFO "[Port list] port_list is NULL.\n");
				wake_unlock(&port_suspend_lock);
				return 0;
			} else {
				port_list[0] = packet_filter_flag;
			}
		}

		mutex_lock(&port_lock);
		if (add_or_remove)
			add_list(srcp);
		else
			remove_list(srcp);
		update_port_list();
		mutex_unlock(&port_lock);
	}
	wake_unlock(&port_suspend_lock);
	return 0;
}
EXPORT_SYMBOL(add_or_remove_port);

int update_port_list_charging_state(int enable) {
	size_t count = 0;

	wake_lock(&port_suspend_lock);
	if (!packet_filter_flag) {
		wake_unlock(&port_suspend_lock);
		return 0;
	}

	if (port_list == NULL) {
		printk(KERN_INFO "[Port list] port_list is NULL.\n");
		wake_unlock(&port_suspend_lock);
		return 0;
	}

	usb_enable = enable;
	mutex_lock(&port_lock);
	if (usb_enable) {
		port_list_enable(0);
	} else {
		for (count = 1; count <= 127; count++) {
			if (!port_list[count]) {
				break;
			}
		}
		if (count <= 127)
			port_list_enable(1);
		else
			port_list_enable(0);
	}
	mutex_unlock(&port_lock);
	wake_unlock(&port_suspend_lock);
	return 0;
}
EXPORT_SYMBOL(update_port_list_charging_state);

static int __init port_list_init(void) {
	int ret;
	uint32_t port_list_phy_addr;
	wake_lock_init(&port_suspend_lock, WAKE_LOCK_SUSPEND, "port_list");
	mutex_init(&port_lock);
	port_list = smem_alloc(SMEM_ID_VENDOR2, 256);
	memset(&curr_port_list, 0, sizeof(curr_port_list));
	INIT_LIST_HEAD(&curr_port_list.list);

	port_list_phy_addr = MSM_SHARED_RAM_PHYS + ((uint32_t)port_list - (uint32_t)MSM_SHARED_RAM_BASE);
	printk(KERN_INFO "[Port list] init()\n");
	printk(KERN_INFO "[Port list] Virtual Address for port_list: [%p]\n", port_list);
	printk(KERN_INFO "[Port list] Physical Address for port_list: [%X]\n", port_list_phy_addr);

	ret = misc_register(&portlist_misc);
	if (ret < 0) {
		printk(KERN_ERR "[Port list] failed to register misc device!\n");
		goto err_misc_register;
	}

	p_class = class_create(THIS_MODULE, "htc_portlist");
	if (IS_ERR(p_class)) {
		ret = PTR_ERR(p_class);
		p_class = NULL;
		printk(KERN_ERR "[Port list] class_create failed!\n");
		goto err_class_create;
	}

	portlist_misc.this_device = device_create(p_class, NULL, 0 , NULL, "packet_filter");
	if (IS_ERR(portlist_misc.this_device)) {
		ret = PTR_ERR(portlist_misc.this_device);
		portlist_misc.this_device = NULL;
		printk(KERN_ERR "[Port list] device_create failed!\n");
		goto err_device_create;
	}

	ret = device_create_file(portlist_misc.this_device, &dev_attr_flag);
	if (ret < 0) {
		printk(KERN_ERR "[Port list] devices_create_file failed!\n");
		goto err_device_create_file;
	}
	if (port_list != NULL) {
		port_list[0] = packet_filter_flag;
	} else {
		printk(KERN_INFO "[Port list] port_list is NULL.\n");
		printk(KERN_INFO "[Port list] packet filter is disabled.\n");
	}

	return 0;

err_device_create_file:
	device_destroy(p_class, 0);
err_device_create:
	class_destroy(p_class);
err_class_create:
	misc_deregister(&portlist_misc);
err_misc_register:
	return ret;
}

static void __exit port_list_exit(void) {
	int ret;
	struct list_head *listptr;
	struct p_list *entry;

	device_remove_file(portlist_misc.this_device, &dev_attr_flag);
	device_destroy(p_class, 0);
	class_destroy(p_class);

	ret = misc_deregister(&portlist_misc);
	if (ret < 0)
		printk(KERN_ERR "[Port list] failed to unregister misc device!\n");

	list_for_each(listptr, &curr_port_list.list) {
		entry = list_entry(listptr, struct p_list, list);
		kfree(entry);
	}
}

late_initcall(port_list_init);
module_exit(port_list_exit);

MODULE_AUTHOR("Mio Su <Mio_Su@htc.com>");
MODULE_DESCRIPTION("HTC port list driver");
MODULE_LICENSE("GPL");
