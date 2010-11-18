/* arch/arm/mach-msm/htc_bluetooth.c
 * 
 * Code to extract Bluetooth bd_address information 
 * from ATAG set up by the bootloader. 
 *
 * Copyright (C) 2009 HTC Corporation
 * Author:Yomin Lin <yomin_lin@htc.com>
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/setup.h>

/* configuration tags specific to Bluetooth*/
#define ATAG_BLUETOOTH	  0x43294329

#define ATAG_BT_DEBUG
#define MAX_BT_SIZE 0x8U
static unsigned char bt_bd_ram[MAX_BT_SIZE];

unsigned char *get_bt_bd_ram(void)
{
	return (bt_bd_ram);
}

EXPORT_SYMBOL(get_bt_bd_ram);
#ifdef ATAG_BT_DEBUG
static int __init parse_tag_bt(const struct tag *tag)
{
	unsigned char *dptr=(unsigned char *)(&tag->u);
	unsigned size;
    unsigned i;
	unsigned char *ptr;

	size=min((tag->hdr.size-2)*sizeof(__u32),MAX_BT_SIZE);

	ptr=dptr;
        printk("BT Data size= %d, 0x%x,", tag->hdr.size, tag->hdr.tag);
	for(i=0;i<size;i++)
	{
		printk("%02x,", *ptr++);
	}

        memcpy((void *)bt_bd_ram ,(void *)dptr, size);
        return 0;
}
#else
static int __init parse_tag_bt(const struct tag *tag)
{
	unsigned char *dptr=(unsigned char *)(&tag->u);
	unsigned size;

        size=min((tag->hdr.size-2)*sizeof(__u32),MAX_BT_SIZE);
        memcpy((void *)bt_bd_ram ,(void *)dptr, size);
        return 0;
}
#endif
 __tagtable(ATAG_BLUETOOTH, parse_tag_bt);
