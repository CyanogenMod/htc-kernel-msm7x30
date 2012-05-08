/* arch/arm/mach-msm/htc_bluetooth.c
 *
 * Code to extract Bluetooth bd_address information
 * from ATAG set up by the bootloader.
 *
 * Copyright (C) 2010 HTC Corporation
 * Author:Yomin Lin <yomin_lin@htc.com>
 * Author:Allen Ou <allen_ou@htc.com>
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include <mach/htc_bdaddress.h>

#define ATAG_BT_DEBUG

/* configuration tags specific to Bluetooth*/
#define ATAG_BLUETOOTH 0x43294329
#define MAX_BT_SIZE 0x8U

static unsigned char bt_bd_ram[MAX_BT_SIZE];
static char bdaddress[20];

static unsigned char *get_bt_bd_ram(void)
{
	return (bt_bd_ram);
}

static int __init parse_tag_bt(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;
	#ifdef ATAG_BT_DEBUG
    unsigned i;
	#endif

	size = min((tag->hdr.size-2)*sizeof(__u32), MAX_BT_SIZE);
	memcpy((void *)bt_bd_ram, (void *)dptr, size);

	#ifdef ATAG_BT_DEBUG
	printk(KERN_INFO "BT Data size= %d, 0x%x,",
			tag->hdr.size, tag->hdr.tag);

	for (i = 0; i < size; i++)
		printk(KERN_INFO "%02x,", bt_bd_ram[i]);
	#endif

	return 0;
}
__tagtable(ATAG_BLUETOOTH, parse_tag_bt);

void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x",
			cTemp[0], cTemp[1], cTemp[2],
			cTemp[3], cTemp[4], cTemp[5]);

	printk(KERN_INFO "BD_ADDRESS=%s\n", bdaddress);
}
module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");

