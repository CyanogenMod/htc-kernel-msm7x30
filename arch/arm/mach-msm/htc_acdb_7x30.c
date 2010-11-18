/* arch/arm/mach-msm/htc_acdb_7x30.c
 *
 * Copyright (C) 2010 HTC Corporation
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

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/delay.h>

#include <mach/msm_smd.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/htc_acdb_7x30.h>
#include "smd_private.h"

#define ACDB_IOCTL_MAGIC 'd'
#define HTC_ACDB_UPDATE_SIZE	_IOR(ACDB_IOCTL_MAGIC, 1, size_t)
#define HTC_ACDB_UPDATE_TABLE	_IOW(ACDB_IOCTL_MAGIC, 2, \
					struct acdb_update_info *)
#define HTC_ACDB_GET_SMEM_SIZE  _IOR(ACDB_IOCTL_MAGIC, 3, uint32_t)

#define D(fmt, args...) printk(KERN_INFO "htc-acdb: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "htc-acdb: "fmt, ##args)

#define SHARE_PAGES 4

#define HTC_DEF_ACDB_SMEM_SIZE        (0x50000)
#define HTCPROG	0x30100002
#define HTCVERS 0
#define ONCRPC_ALLOC_ACDB_MEM_PROC (1)
#define ONCRPC_UPDATE_TABLE_SIZE_PROC (2)
#define ONCRPC_REINIT_ACDB_TABLE_PROC (3)

static volatile uint32_t htc_acdb_vir_addr;
static struct msm_rpc_endpoint *endpoint;
static struct mutex api_lock;
static struct mutex rpc_connect_lock;
static int acdb_inited;
static struct acdb_ops default_acdb_ops;
static struct acdb_ops *the_ops = &default_acdb_ops;
static uint32_t acdb_smem_size;

struct acdb_update_info {
	uint32_t size;
	int done;
};

void acdb_register_ops(struct acdb_ops *ops)
{
	the_ops = ops;
}

static int is_rpc_connect(void)
{
	mutex_lock(&rpc_connect_lock);
	if (endpoint == NULL) {
		endpoint = msm_rpc_connect(HTCPROG,
				HTCVERS, 0);
		if (IS_ERR(endpoint)) {
			pr_err("%s: init rpc failed! rc = %ld\n",
				__func__, PTR_ERR(endpoint));
			mutex_unlock(&rpc_connect_lock);
			return -1;
		}
	}
	mutex_unlock(&rpc_connect_lock);
	return 0;
}

static int htc_acdb_open(struct inode *inode, struct file *file)
{
	int reply_value;
	int rc = -EIO;
	struct set_smem_req {
		struct rpc_request_hdr hdr;
		uint32_t size;
	} req_smem;

	struct set_smem_rep {
		struct rpc_reply_hdr hdr;
		int n;
	} rep_smem;

	D("open\n");

	mutex_lock(&api_lock);

	if (!htc_acdb_vir_addr) {
		if (is_rpc_connect() == -1)
			goto done;

		if (the_ops->get_smem_size)
			acdb_smem_size = the_ops->get_smem_size();
		else
			acdb_smem_size = HTC_DEF_ACDB_SMEM_SIZE;

		pr_info("%s: smem size %d\n", __func__, acdb_smem_size);

		req_smem.size = cpu_to_be32(acdb_smem_size);
		rc = msm_rpc_call_reply(endpoint,
					ONCRPC_ALLOC_ACDB_MEM_PROC,
					&req_smem, sizeof(req_smem),
					&rep_smem, sizeof(rep_smem),
					5 * HZ);

		reply_value = be32_to_cpu(rep_smem.n);
		if (reply_value != 0 || rc < 0) {
			E("open failed: ALLOC_ACDB_MEM_PROC error %d.\n", rc);
			goto done;
		}
		htc_acdb_vir_addr =
			(uint32_t)smem_alloc(SMEM_ID_VENDOR1,
					acdb_smem_size);
		if (!htc_acdb_vir_addr) {
			E("open failed: smem_alloc error\n");
			goto done;
		}
	}

	rc = 0;
done:
	mutex_unlock(&api_lock);
	return rc;
}

static int htc_acdb_release(struct inode *inode, struct file *file)
{
	D("release\n");
	return 0;
}

static long
htc_acdb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	uint32_t total_size;
	struct acdb_update_info info;

	mutex_lock(&api_lock);
	switch (cmd) {
	case HTC_ACDB_UPDATE_SIZE: {
		int reply_value;
		struct update_size_req {
			struct rpc_request_hdr hdr;
			uint32_t size;
		} sz_req;

		struct update_size_rep {
			struct rpc_reply_hdr hdr;
			int ret;
		} sz_rep;

		if (acdb_inited)
			break;

		if (copy_from_user(&total_size,
			(void *)arg, sizeof(uint32_t))) {
			rc = -EFAULT;
			break;
		}
		D("htc_acdb_ioctl: total size = %d\n", total_size);
		sz_req.size = cpu_to_be32(total_size);

		rc = msm_rpc_call_reply(endpoint,
			ONCRPC_UPDATE_TABLE_SIZE_PROC,
			&sz_req, sizeof(sz_req),
			&sz_rep, sizeof(sz_rep), 5 * HZ);

		reply_value = be32_to_cpu(sz_rep.ret);
		if (reply_value != 0) {
			D("htc_acdb_ioctl: rpc update size failed %d\n",
				reply_value);
			rc = reply_value;
		}

		break;
	}
	case HTC_ACDB_UPDATE_TABLE: {
		int reply_value;
		struct update_table_req {
			struct rpc_request_hdr hdr;
			uint32_t update_size;
			int done;
		} tbl_req;

		struct update_table_rep {
			struct rpc_reply_hdr hdr;
			int ret;
		} tbl_rep;

		if (acdb_inited)
			break;

		if (copy_from_user(&info, (void *)arg,
			sizeof(struct acdb_update_info))) {
			rc = -EFAULT;
			break;
		}

		if (info.size <= 0) {
			E("invalid table size\n");
			rc = -EINVAL;
			break;
		}

		tbl_req.update_size = cpu_to_be32(info.size);
		tbl_req.done = cpu_to_be32(info.done);

		if (info.done)
			D("htc_acdb_ioctl: update table %d bytes, done %d\n",
				info.size, info.done);

		rc = msm_rpc_call_reply(endpoint,
			ONCRPC_REINIT_ACDB_TABLE_PROC,
			&tbl_req, sizeof(tbl_req),
			&tbl_rep, sizeof(tbl_rep), 5 * HZ);

		reply_value = be32_to_cpu(tbl_rep.ret);
		if (reply_value != 0) {
			D("htc_acdb_iotcl: rpc update table failed %d\n",
				reply_value);
			rc = reply_value;
		} else if (info.done)
			acdb_inited = 1;

		msleep(10);
		break;
	}
	case HTC_ACDB_GET_SMEM_SIZE: {
		if (copy_to_user((void *) arg,
			&acdb_smem_size, sizeof(uint32_t))) {
			E("htc_acdb_ioctl: get smem size failed\n");
			rc = -EFAULT;
		}
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&api_lock);
	return rc;
}

static int htc_acdb_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pgoff;
	int rc = -EINVAL;
	size_t size;

	D("mmap\n");

	mutex_lock(&api_lock);

	size = vma->vm_end - vma->vm_start;

	if (vma->vm_pgoff != 0) {
		E("mmap failed: page offset %lx\n", vma->vm_pgoff);
		goto done;
	}

	if (!htc_acdb_vir_addr) {
		E("mmap failed: smem region not allocated\n");
		rc = -EIO;
		goto done;
	}

	pgoff = MSM_SHARED_RAM_PHYS +
		(htc_acdb_vir_addr - (uint32_t)MSM_SHARED_RAM_BASE);
	pgoff = ((pgoff + 4095) & ~4095);
	htc_acdb_vir_addr = ((htc_acdb_vir_addr + 4095) & ~4095);

	if (pgoff <= 0) {
		E("pgoff wrong. %ld\n", pgoff);
		goto done;
	}

	if (size <= acdb_smem_size) {
		pgoff = pgoff >> PAGE_SHIFT;
	} else {
		E("size > acdb_smem_size %d\n", acdb_smem_size);
		goto done;
	}

	vma->vm_flags |= VM_IO | VM_RESERVED;
	rc = io_remap_pfn_range(vma, vma->vm_start, pgoff,
				size, vma->vm_page_prot);

	if (rc < 0)
		E("mmap failed: remap error %d\n", rc);

done:	mutex_unlock(&api_lock);
	return rc;
}


static const struct file_operations htc_acdb_fops = {
	.owner = THIS_MODULE,
	.open = htc_acdb_open,
	.release = htc_acdb_release,
	.mmap = htc_acdb_mmap,
	.unlocked_ioctl = htc_acdb_ioctl,
};

static struct miscdevice htc_acdb_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acdb",
	.fops = &htc_acdb_fops,
};

static int __init htc_acdb_init(void)
{
	int ret = 0;

	mutex_init(&api_lock);
	mutex_init(&rpc_connect_lock);
	ret = misc_register(&htc_acdb_misc);
	if (ret < 0)
		pr_err("%s: failed to register misc device!\n", __func__);

	return ret;
}

static void __exit htc_acdb_exit(void)
{
	misc_deregister(&htc_acdb_misc);
}

module_init(htc_acdb_init);
module_exit(htc_acdb_exit);
