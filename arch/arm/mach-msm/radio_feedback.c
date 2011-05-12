/* arch/arm/mach-msm/radio_feedback.c
 *
 * Copyright (C) 2010 HTC Corporation.
 * Author: YaWen Su <YaWen_Su@htc.com>
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
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <mach/msm_smd.h>
#include <mach/msm_iomap.h>
#include <linux/fcntl.h>

#include "smd_private.h"
#include "acpuclock.h"

#define RADIO_FEEDBACK_IOCTL_MAGIC	'p'
#define RADIO_FEEDBACK_GET_CDLOG_INFO	_IOW(RADIO_FEEDBACK_IOCTL_MAGIC, 89, unsigned)

#define HTC_SMEM_PARAM_BASE_ADDR	0x004FC000
#define HTC_SMEM_PARAM_SIZE		0x30C

struct msm_radio_feedback_config {
	uint32_t start_addr;
	uint32_t max_size;
};

struct mutex radio_feedback_lock;
static uint32_t radio_feedback_addr;
struct msm_radio_feedback_config config;
static long radio_feedback_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	int rc = 0;
	switch (cmd) {
	case RADIO_FEEDBACK_GET_CDLOG_INFO:
		radio_feedback_addr = (uint32_t)ioremap(HTC_SMEM_PARAM_BASE_ADDR, HTC_SMEM_PARAM_SIZE);
		/* start addr(4 bytes): HTC_SMEM_PARAM_BASE_ADDR + 0x304 */
		memcpy(&config.start_addr, (void *)(radio_feedback_addr + 0x304), 4);
		printk("start addr: 0x%x\n", config.start_addr);
		/* max size(4 bytes): HTC_SMEM_PARAM_BASE_ADDR + 0x308 */
		memcpy(&config.max_size, (void *)(radio_feedback_addr + 0x308), 4);
		printk("max_size: 0x%x\n", config.max_size);
		if(copy_to_user((void *)arg, &config, sizeof(config)))
			rc = -EFAULT;
		break;
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int radio_feedback_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pgoff;
	size_t size = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff != 0)
		return -EINVAL;

	if (size <= config.max_size)
		pgoff = config.start_addr >> PAGE_SHIFT;
	else
		return -EINVAL;

	vma->vm_flags |= VM_IO | VM_RESERVED;
	if (io_remap_pfn_range(vma, vma->vm_start, pgoff,
			       size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static struct file_operations radio_feedback_fops = {
	.owner = THIS_MODULE,
	.mmap = radio_feedback_mmap,
	.unlocked_ioctl = radio_feedback_ioctl,
};

static struct miscdevice radio_feedback_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "radio_feedback",
	.fops = &radio_feedback_fops,
};

static int __init radio_feedback_init(void)
{
	int ret;
	ret = misc_register(&radio_feedback_misc);
	if (ret < 0) {
		pr_err("failed to register misc device!\n");
		return ret;
	}
	mutex_init(&radio_feedback_lock);
	return ret;
}

static void __exit radio_feedback_exit(void)
{
	int ret;
	ret = misc_deregister(&radio_feedback_misc);
	if (ret < 0)
		pr_err("failed to unregister misc device!\n");
}

module_init(radio_feedback_init);
module_exit(radio_feedback_exit);
