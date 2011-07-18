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
#include <linux/firmware.h>

#include <mach/msm_smd.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/htc_acdb_7x30.h>
#include "smd_private.h"

#define ACDB_IOCTL_MAGIC 'd'
#define HTC_REINIT_ACDB    _IOW(ACDB_IOCTL_MAGIC, 1, unsigned)

#define D(fmt, args...) printk(KERN_INFO "htc-acdb: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "htc-acdb: "fmt, ##args)

#define SHARE_PAGES 4

#define HTC_DEF_ACDB_SMEM_SIZE        (0x50000)
#define HTC_DEF_ACDB_RADIO_BUFFER_SIZE        (1024 * 2048)
#define HTCPROG	0x30100002
#define HTCVERS 0
#define ONCRPC_ALLOC_ACDB_MEM_PROC (1)
#define ONCRPC_UPDATE_TABLE_SIZE_PROC (2)
#define ONCRPC_REINIT_ACDB_TABLE_PROC (3)

static volatile uint32_t htc_acdb_vir_addr;
static struct msm_rpc_endpoint *endpoint;
static struct mutex api_lock;
static struct mutex rpc_connect_lock;
static struct acdb_ops default_acdb_ops;
static struct acdb_ops *the_ops = &default_acdb_ops;
static uint32_t acdb_smem_size;
static char acdb_init_file[64];

static int acdb_init(char*);

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
			pr_aud_err("%s: init rpc failed! rc = %ld\n",
				__func__, PTR_ERR(endpoint));
			mutex_unlock(&rpc_connect_lock);
			return -1;
		}
	}
	mutex_unlock(&rpc_connect_lock);
	return 0;
}

static int update_acdb_table_size(uint32_t total_size)
{
	int reply_value = -1, rc = 0;

	struct update_size_req {
		struct rpc_request_hdr hdr;
		uint32_t size;
	} sz_req;

	struct update_size_rep {
		struct rpc_reply_hdr hdr;
		int ret;
	} sz_rep;

	D("%s: total size = %d\n", __func__, total_size);
	sz_req.size = cpu_to_be32(total_size);
	sz_rep.ret = cpu_to_be32(reply_value);

	rc = msm_rpc_call_reply(endpoint,
		ONCRPC_UPDATE_TABLE_SIZE_PROC,
		&sz_req, sizeof(sz_req),
		&sz_rep, sizeof(sz_rep), 5 * HZ);

	reply_value = be32_to_cpu(sz_rep.ret);
	if (reply_value != 0) {
		D("%s: rpc update size failed %d\n",
			__func__, reply_value);
		rc = reply_value;
	}

	return rc;
}

static int update_acdb_table(uint32_t size, int done)
{
	int reply_value = -1, rc = 0;
	struct update_table_req {
		struct rpc_request_hdr hdr;
		uint32_t update_size;
		int done;
	} tbl_req;

	struct update_table_rep {
		struct rpc_reply_hdr hdr;
		int ret;
	} tbl_rep;

	if (size <= 0 || size > acdb_smem_size) {
		E("invalid table size %d\n", size);
		return -EINVAL;
	}

	tbl_req.update_size = cpu_to_be32(size);
	tbl_req.done = cpu_to_be32(done);
	tbl_rep.ret = cpu_to_be32(reply_value);

	if (done)
		D("%s: update table %d bytes, done %d\n",
			__func__, size, done);

	rc = msm_rpc_call_reply(endpoint,
		ONCRPC_REINIT_ACDB_TABLE_PROC,
		&tbl_req, sizeof(tbl_req),
		&tbl_rep, sizeof(tbl_rep), 5 * HZ);

	reply_value = be32_to_cpu(tbl_rep.ret);
	if (reply_value != 0) {
		E("%s: rpc update table failed %d\n",
			__func__, reply_value);
		rc = reply_value;
	}

	return rc;
}

static int htc_reinit_acdb(char* filename) {
	int rc = 0;

	if (strlen(filename) < 0) {
		rc = -EINVAL;
		goto done;
	}
	if (strcmp(filename, acdb_init_file)) {
		rc = acdb_init(filename);
		if (!rc)
			strcpy(acdb_init_file, filename);
	}
done:
	D("%s: load '%s', return %d\n", __func__, filename, rc);
	return rc;

}

static int htc_acdb_open(struct inode *inode, struct file *file)
{
	int reply_value = -1;
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

		pr_aud_info("%s: smem size %d\n", __func__, acdb_smem_size);

		req_smem.size = cpu_to_be32(acdb_smem_size);
		rep_smem.n = cpu_to_be32(reply_value);

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
		htc_acdb_vir_addr = ((htc_acdb_vir_addr + 4095) & ~4095);
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
	char filename[64];

	mutex_lock(&api_lock);
	switch (cmd) {
	case HTC_REINIT_ACDB:
		memset(filename, 0, sizeof(filename));
		rc = copy_from_user(&filename, (void *)arg, sizeof(filename));
		if (!rc)
			rc = htc_reinit_acdb(filename);
		break;

	default:
		rc = -EINVAL;
	}
	mutex_unlock(&api_lock);
	return rc;
}

static const struct file_operations htc_acdb_fops = {
	.owner = THIS_MODULE,
	.open = htc_acdb_open,
	.release = htc_acdb_release,
	.unlocked_ioctl = htc_acdb_ioctl,
};

static struct miscdevice htc_acdb_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acdb",
	.fops = &htc_acdb_fops,
};

static int acdb_init(char *filename)
{
	const void *db;
	const struct firmware *fw;
	uint32_t size = 0, ptr, acdb_radio_buffer_size = 0;
	int rc = 0;

	pr_aud_info("acdb: load '%s'\n", filename);
	if (request_firmware(&fw, filename, htc_acdb_misc.this_device) < 0) {
		pr_aud_err("acdb: load '%s' failed...\n", filename);
		return -ENODEV;
	}

	if (fw == NULL)
		return -EINVAL;
	db = (void*) fw->data;

	if (the_ops->get_acdb_radio_buffer_size)
		acdb_radio_buffer_size = the_ops->get_acdb_radio_buffer_size();
	else
		acdb_radio_buffer_size = HTC_DEF_ACDB_RADIO_BUFFER_SIZE;

	if (fw->size > acdb_radio_buffer_size) {
		E("acdb_init: table size too large (%d bytes)\n", fw->size);
		rc = -EINVAL;
		goto fail;
	}

	rc = update_acdb_table_size(fw->size);
	if (rc < 0)
		goto fail;

	for (ptr = 0; ptr < fw->size; ptr += acdb_smem_size) {
		if (fw->size - ptr >= acdb_smem_size)
			size = acdb_smem_size;
		else
			size = fw->size - ptr;

		memcpy((void *) htc_acdb_vir_addr, (void *) db + ptr, size);
		if (ptr + size == fw->size)
			rc = update_acdb_table(size, 1);
		else
			rc = update_acdb_table(size, 0);

		if (rc < 0)
			goto fail;
		else
			msleep(10);
	}
	rc = 0;
fail:
	release_firmware(fw);
	return rc;
}

static int __init htc_acdb_init(void)
{
	int ret = 0;

	mutex_init(&api_lock);
	mutex_init(&rpc_connect_lock);
	ret = misc_register(&htc_acdb_misc);
	if (ret < 0)
		pr_aud_err("%s: failed to register misc device!\n", __func__);

	return ret;
}

static void __exit htc_acdb_exit(void)
{
	misc_deregister(&htc_acdb_misc);
}

module_init(htc_acdb_init);
module_exit(htc_acdb_exit);
