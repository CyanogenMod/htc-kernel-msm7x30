/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/rmt_storage_server.h>

#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>

static int handle_rmt_storage_call(struct msm_rpc_server *server,
				struct rpc_request_hdr *req, unsigned len);

struct rmt_storage_sync_data {
	uint32_t cb_id;
	struct msm_rpc_client_info cinfo;
};

struct rmt_storage_sync_sts_data {
	uint32_t cb_id;
	struct msm_rpc_client_info cinfo;
	int token;
};

struct rmt_storage_server_info {
	unsigned long cids;
	struct rmt_shrd_mem_param rmt_shrd_mem;
	int open_excl;
	atomic_t total_events;
	wait_queue_head_t event_q;
	struct list_head event_list;
	struct list_head client_info_list;
	/* Lock to protect event list and client info list */
	spinlock_t lock;
	/* Wakelock to be acquired when processing requests from modem */
	struct wake_lock wlock;
	atomic_t wcount;
	struct rmt_storage_sync_data sync_data;
	struct rmt_storage_sync_sts_data sync_sts_data;
	uint32_t final_call;
	wait_queue_head_t final_waitq;
	struct shared_ramfs_entry *smem_info;
};

struct rmt_storage_kevent {
	struct list_head list;
	struct rmt_storage_event event;
};

struct rmt_storage_client_info {
	struct list_head list;
	uint32_t handle;
	struct msm_rpc_client_info cinfo;
};

static struct rmt_storage_server_info *_rms;

#define RMT_STORAGE_SRV_APIPROG            0x3000009C
#define RMT_STORAGE_SRV_APIVERS            0x00010001

#define RMT_STORAGE_SRV_API_NULL_PROC 0
#define RMT_STORAGE_SRV_OPEN_PROC 2
#define RMT_STORAGE_SRV_CLOSE_PROC 3
#define RMT_STORAGE_SRV_WRITE_BLOCK_PROC 4
#define RMT_STORAGE_SRV_GET_DEV_ERROR_PROC 5
#define RMT_STORAGE_SRV_WRITE_IOVEC_PROC 6
#define RMT_STORAGE_SRV_REGISTER_CALLBACK_PROC 7
#define RMT_STORAGE_SRV_REGISTER_SYNC_PROC 8
#define RMT_STORAGE_SRV_REGISTER_SYNC_STATUS_PROC 10

#define RMT_STORAGE_EVENT_FUNC_PTR_TYPE_PROC 1
#define RMT_STORAGE_REG_SYNC_CB_TYPE_PROC 2
#define RMT_STORAGE_REG_SYNC_STATUS_CB_TYPE_PROC 3

static struct msm_rpc_server rmt_storage_server = {
	.prog = RMT_STORAGE_SRV_APIPROG,
	.vers = RMT_STORAGE_SRV_APIVERS,
	.rpc_call = handle_rmt_storage_call,
};

struct rmt_storage_open_args {
	uint32_t len;
};

struct rmt_storage_close_args {
	uint32_t handle;
};

struct rmt_storage_get_err_args {
	uint32_t handle;
};

struct rmt_storage_write_block_args {
	uint32_t handle;
	uint32_t data_phy_addr;
	uint32_t sector_addr;
	uint32_t num_sector;
};

struct rmt_storage_write_iovec_args {
	uint32_t handle;
	uint32_t count;
};

struct rmt_storage_cb_args {
	uint32_t handle;
	uint32_t cb_id;
	uint32_t data;
};

static struct msm_rpc_client_info *rmt_storage_get_client_info(uint32_t handle)
{
	struct rmt_storage_client_info *rmtfs_cinfo;

	spin_lock(&_rms->lock);
	list_for_each_entry(rmtfs_cinfo, &_rms->client_info_list, list) {
		if (rmtfs_cinfo->handle == handle) {
			spin_unlock(&_rms->lock);
			return &rmtfs_cinfo->cinfo;
		}
	}

	spin_unlock(&_rms->lock);
	return NULL;
}

static int  rmt_storage_delete_client_info(uint32_t handle)
{
	struct rmt_storage_client_info *rmtfs_cinfo;

	spin_lock(&_rms->lock);
	list_for_each_entry(rmtfs_cinfo, &_rms->client_info_list, list) {
		if (rmtfs_cinfo->handle == handle) {
			list_del(&rmtfs_cinfo->list);
			spin_unlock(&_rms->lock);
			kfree(rmtfs_cinfo);
			return 0;
		}
	}
	spin_unlock(&_rms->lock);
	return -EINVAL;
}

static int rmt_storage_add_client_info(uint32_t handle)
{
	struct rmt_storage_client_info *rmtfs_cinfo;

	rmtfs_cinfo = kmalloc(sizeof(struct rmt_storage_client_info),
			GFP_KERNEL);
	if (!rmtfs_cinfo)
		return -ENOMEM;

	msm_rpc_server_get_requesting_client(&rmtfs_cinfo->cinfo);
	rmtfs_cinfo->handle = handle;

	spin_lock(&_rms->lock);
	list_add_tail(&rmtfs_cinfo->list, &_rms->client_info_list);
	spin_unlock(&_rms->lock);
	return 0;
}

struct rmt_storage_kcb {
	uint32_t cb_id;
	uint32_t err_code;
	uint32_t data;
};

static int rmt_storage_callback_arg(struct msm_rpc_server *server,
				void *buf, void *data)
{
	struct rmt_storage_cb *args = data;
	struct rmt_storage_kcb *req = buf;
	int size;

	req->cb_id = cpu_to_be32(args->cb_id);
	req->err_code = cpu_to_be32(args->err_code);
	req->data = cpu_to_be32(args->data);
	size = sizeof(struct rmt_storage_kcb);
	return size;
}

static int rmt_storage_send_callback(struct rmt_storage_cb *args)
{
	struct msm_rpc_client_info *cinfo;

	cinfo = rmt_storage_get_client_info(args->handle);
	if (!cinfo) {
		pr_err("%s: No client information found\n", __func__);
		return -EINVAL;
	}

	return msm_rpc_server_cb_req(&rmt_storage_server, cinfo,
			RMT_STORAGE_EVENT_FUNC_PTR_TYPE_PROC,
			rmt_storage_callback_arg, args,
			NULL, NULL, -1);
}

struct rmt_storage_sync_args {
	uint32_t cb_id;
};

static int rmt_storage_send_sync_arg(struct msm_rpc_server *server,
				void *buf, void *data)
{
	struct rmt_storage_sync_data *args = data;
	struct rmt_storage_sync_args *req = buf;
	int size;

	req->cb_id = cpu_to_be32(args->cb_id);
	size = sizeof(struct rmt_storage_sync_args);
	return size;
}

struct rmt_storage_sync_recv_arg {
	int data;
};

static int rmt_storage_receive_sync_arg(struct msm_rpc_server *server,
				void *buf, void *data)
{
	struct rmt_storage_sync_recv_arg *args = data;
	struct rmt_storage_sync_recv_arg *recv_args = buf;

	args->data = be32_to_cpu(recv_args->data);
	_rms->sync_sts_data.token = args->data;
	return 0;
}

static int rmt_storage_force_sync(void)
{
	int rc;
	struct rmt_storage_sync_recv_arg args;
	struct rmt_storage_sync_data *s_data;

	s_data = &_rms->sync_data;
	rc = msm_rpc_server_cb_req(&rmt_storage_server, &s_data->cinfo,
			RMT_STORAGE_REG_SYNC_CB_TYPE_PROC,
			rmt_storage_send_sync_arg, s_data,
			rmt_storage_receive_sync_arg, &args, -1);
	if (rc)
		pr_err("%s: force sync cb req failed: %d\n", __func__, rc);

	return 0;
}

struct rmt_storage_sync_sts_cb_arg {
	uint32_t cb_id;
	int token;
};

static int rmt_storage_send_sync_sts_arg(struct msm_rpc_server *server,
				void *buf, void *data)
{
	struct rmt_storage_sync_sts_data *args = data;
	struct rmt_storage_sync_sts_cb_arg *req = buf;
	int size;

	req->cb_id = cpu_to_be32(args->cb_id);
	req->token = cpu_to_be32(args->token);
	size = sizeof(struct rmt_storage_sync_sts_cb_arg);
	return size;
}

static int rmt_storage_receive_sync_sts_arg(struct msm_rpc_server *server,
				void *buf, void *data)
{
	struct rmt_storage_sync_recv_arg *args = data;
	struct rmt_storage_sync_recv_arg *recv_args = buf;

	args->data = be32_to_cpu(recv_args->data);
	return 0;
}

static int rmt_storage_get_sync_status(void)
{
	int rc;
	struct rmt_storage_sync_recv_arg args;
	struct rmt_storage_sync_sts_data *sts_data;

	sts_data = &_rms->sync_sts_data;
	rc = msm_rpc_server_cb_req(&rmt_storage_server, &sts_data->cinfo,
			RMT_STORAGE_REG_SYNC_STATUS_CB_TYPE_PROC,
			rmt_storage_send_sync_sts_arg, sts_data,
			rmt_storage_receive_sync_sts_arg, &args, -1);
	if (rc) {
		pr_err("%s: sync status cb req failed: %d\n", __func__, rc);
		return rc;
	}
	return args.data;
}

static void put_event(struct rmt_storage_server_info *rms,
			struct rmt_storage_kevent *kevent)
{
	spin_lock(&rms->lock);
	list_add_tail(&kevent->list, &rms->event_list);
	spin_unlock(&rms->lock);
}

static struct rmt_storage_kevent *get_event(struct rmt_storage_server_info *rms)
{
	struct rmt_storage_kevent *kevent = NULL;

	spin_lock(&rms->lock);
	if (!list_empty(&rms->event_list)) {
		kevent = list_first_entry(&rms->event_list,
			struct rmt_storage_kevent, list);
		list_del(&kevent->list);
	}
	spin_unlock(&rms->lock);
	return kevent;
}

static int handle_rmt_storage_call(struct msm_rpc_server *server,
				struct rpc_request_hdr *req, unsigned len)
{
	int i, rc;
	void *reply;
	uint32_t size;
	uint32_t result = RMT_STORAGE_NO_ERROR;
	uint32_t rpc_status = RPC_ACCEPTSTAT_SUCCESS;
	struct rmt_storage_server_info *rms = _rms;
	struct rmt_storage_event *event_args;
	struct rmt_storage_kevent *kevent;

	kevent = kmalloc(sizeof(struct rmt_storage_kevent), GFP_KERNEL);
	if (!kevent) {
		rpc_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		goto out;
	}
	event_args = &kevent->event;

	switch (req->procedure) {
	case RMT_STORAGE_SRV_API_NULL_PROC:
		kfree(kevent);
		return 0;

	case RMT_STORAGE_SRV_OPEN_PROC: {
		struct rmt_storage_open_args *args;
		uint32_t len;

		pr_info("%s: rmt_storage open event\n", __func__);
		result = find_first_zero_bit(&rms->cids, sizeof(rms->cids));
		if (result > MAX_NUM_CLIENTS) {
			kfree(kevent);
			pr_err("%s: Max clients are reached\n", __func__);
			result = 0;
			goto out;
		}

		__set_bit(result, &rms->cids);

		args = (struct rmt_storage_open_args *)(req + 1);
		len = be32_to_cpu(args->len);
		snprintf(event_args->path, len, "%s", (char *) (args + 1));
		pr_info("open partition %s\n\n", event_args->path);
		event_args->id = RMT_STORAGE_OPEN;
		event_args->handle = result;
		break;
	}

	case RMT_STORAGE_SRV_CLOSE_PROC: {
		struct rmt_storage_close_args *args;

		pr_info("%s: rmt_storage close event\n", __func__);
		args = (struct rmt_storage_close_args *)(req + 1);
		event_args->handle = be32_to_cpu(args->handle);
		event_args->id = RMT_STORAGE_CLOSE;
		clear_bit(event_args->handle, &rms->cids);
		rc = rmt_storage_delete_client_info(event_args->handle);
		if (rc < 0)
			result = RMT_STORAGE_ERROR_PARAM;
		break;
	}

	case RMT_STORAGE_SRV_WRITE_BLOCK_PROC: {
		struct rmt_storage_write_block_args *args;
		struct rmt_storage_iovec_desc *xfer;

		pr_info("%s: rmt_storage write block event\n", __func__);
		args = (struct rmt_storage_write_block_args *)(req + 1);
		event_args->handle = be32_to_cpu(args->handle);
		xfer = &event_args->xfer_desc[0];
		xfer->sector_addr = be32_to_cpu(args->sector_addr);
		xfer->data_phy_addr = be32_to_cpu(args->data_phy_addr);

		if (xfer->data_phy_addr < rms->rmt_shrd_mem.start ||
		   xfer->data_phy_addr > (rms->rmt_shrd_mem.start +
		   rms->rmt_shrd_mem.size)) {
			result = RMT_STORAGE_ERROR_PARAM;
			goto out;
		}

		xfer->num_sector = be32_to_cpu(args->num_sector);
		event_args->xfer_cnt = 1;
		event_args->id = RMT_STORAGE_WRITE;

		if (atomic_inc_return(&rms->wcount) == 1)
			wake_lock(&rms->wlock);

		pr_info("sec_addr = %u, data_addr = %x, num_sec = %d\n\n",
			xfer->sector_addr, xfer->data_phy_addr,
			xfer->num_sector);
		break;
	}

	case RMT_STORAGE_SRV_GET_DEV_ERROR_PROC: {
		struct rmt_storage_get_err_args *args;

		/* Not implemented */
		args = (struct rmt_storage_get_err_args *)(req + 1);
		event_args->handle = be32_to_cpu(args->handle);
		kfree(kevent);
		goto out;
	}

	case RMT_STORAGE_SRV_WRITE_IOVEC_PROC: {
		uint32_t ent;
		struct rmt_storage_iovec_desc *iovec, *xfer;
		struct rmt_storage_write_iovec_args *args;

		pr_info("%s: rmt_storage write iovec event\n", __func__);
		args = (struct rmt_storage_write_iovec_args *)(req + 1);
		event_args->handle = be32_to_cpu(args->handle);
		ent = be32_to_cpu(args->count);
		pr_info("handle = %d\n", event_args->handle);

		iovec = (struct rmt_storage_iovec_desc *)(args + 1);
		for (i = 0; i < ent; i++) {
			xfer = &event_args->xfer_desc[i];
			xfer->sector_addr = be32_to_cpu(iovec->sector_addr);
			xfer->data_phy_addr = be32_to_cpu(iovec->data_phy_addr);

			if (xfer->data_phy_addr < rms->rmt_shrd_mem.start ||
			   xfer->data_phy_addr > (rms->rmt_shrd_mem.start +
			   rms->rmt_shrd_mem.size)) {
				result = RMT_STORAGE_ERROR_PARAM;
				goto out;
			}

			xfer->num_sector = be32_to_cpu(iovec->num_sector);
			iovec += 1;
			pr_info("sec_addr = %u, data_addr = %x, num_sec = %d\n",
				xfer->sector_addr, xfer->data_phy_addr,
				xfer->num_sector);
		}
		event_args->xfer_cnt = be32_to_cpu(*((uint32_t *)iovec));
		event_args->id = RMT_STORAGE_WRITE;
		if (atomic_inc_return(&rms->wcount) == 1)
			wake_lock(&rms->wlock);

		pr_info("iovec transfer count = %d\n\n", event_args->xfer_cnt);

		break;
	}

	case RMT_STORAGE_SRV_REGISTER_CALLBACK_PROC: {
		struct rmt_storage_cb_args *args;

		pr_info("%s: rmt_storage register callback event\n", __func__);
		args = (struct rmt_storage_cb_args *)(req + 1);
		event_args->handle = be32_to_cpu(args->handle);
		event_args->cb_id = be32_to_cpu(args->cb_id);
		event_args->cb_data = be32_to_cpu(args->data);
		event_args->id = RMT_STORAGE_REGISTER_CB;
		rc = rmt_storage_add_client_info(event_args->handle);
		if (rc < 0)
			rpc_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		break;
	}

	case RMT_STORAGE_SRV_REGISTER_SYNC_PROC: {
		struct rmt_storage_sync_args *args;
		struct rmt_storage_sync_data *s_data;

		pr_info("%s: register force sync callback\n", __func__);
		s_data = &_rms->sync_data;
		args = (struct rmt_storage_sync_args *)(req + 1);
		s_data->cb_id = be32_to_cpu(args->cb_id);
		msm_rpc_server_get_requesting_client(&s_data->cinfo);
		goto out;
	}

	case RMT_STORAGE_SRV_REGISTER_SYNC_STATUS_PROC: {
		struct rmt_storage_sync_args *args;
		struct rmt_storage_sync_sts_data *sts_data;

		pr_info("%s: register sync status callback\n", __func__);
		sts_data = &_rms->sync_sts_data;
		args = (struct rmt_storage_sync_args *)(req + 1);
		sts_data->cb_id = be32_to_cpu(args->cb_id);
		msm_rpc_server_get_requesting_client(&sts_data->cinfo);
		goto out;
	}

	default:
		kfree(kevent);
		pr_err("%s: program 0x%08x:%d: unknown procedure %d\n",
		       __func__, req->prog, req->vers, req->procedure);
		return -ENODEV;
	}

	put_event(rms, kevent);
	atomic_inc(&rms->total_events);
	wake_up(&rms->event_q);

out:

	reply = msm_rpc_server_start_accepted_reply(server, req->xid,
							rpc_status);
	*(uint32_t *)reply = cpu_to_be32(result);
	size = sizeof(uint32_t);
	rc = msm_rpc_server_send_accepted_reply(server, size);
	if (rc)
		pr_err("%s: send accepted reply failed: %d\n", __func__, rc);

	return 1;
}

static int rmt_storage_open(struct inode *ip, struct file *fp)
{
	int ret = 0;

	spin_lock(&_rms->lock);

	if (!_rms->open_excl)
		_rms->open_excl = 1;
	else
		ret = -EBUSY;

	_rms->smem_info->server_status = 1;
	spin_unlock(&_rms->lock);
	return ret;
}

static int rmt_storage_release(struct inode *ip, struct file *fp)
{

	spin_lock(&_rms->lock);
	_rms->open_excl = 0;
	spin_unlock(&_rms->lock);
	_rms->smem_info->server_status = 0;

	return 0;
}

static long rmt_storage_ioctl(struct file *fp, unsigned int cmd,
			    unsigned long arg)
{
	int ret = 0;
	struct rmt_storage_server_info *rms = _rms;
	struct rmt_storage_kevent *kevent;
	struct rmt_storage_cb cb;

	switch (cmd) {

	case RMT_STORAGE_SHRD_MEM_PARAM:
		pr_info("%s: get shared memory parameters ioctl\n", __func__);
		if (copy_to_user((void __user *)arg, &rms->rmt_shrd_mem,
			sizeof(struct rmt_shrd_mem_param))) {
			pr_err("%s: copy to user failed\n\n", __func__);
			ret = -EFAULT;
		}
		break;

	case RMT_STORAGE_WAIT_FOR_REQ:
		pr_info("%s: wait for request ioctl\n", __func__);
		if (atomic_read(&rms->total_events) == 0) {
			ret = wait_event_interruptible(rms->event_q,
				atomic_read(&rms->total_events) != 0);
		}
		if (ret < 0)
			break;
		atomic_dec(&rms->total_events);

		kevent = get_event(rms);
		WARN_ON(kevent == NULL);
		if (copy_to_user((void __user *)arg, &kevent->event,
			sizeof(struct rmt_storage_event))) {
			pr_err("%s: copy to user failed\n\n", __func__);
			ret = -EFAULT;
		}
		kfree(kevent);
		break;

	case RMT_STORAGE_SEND_STATUS:
		pr_info("%s: send callback ioctl\n", __func__);
		if (copy_from_user(&cb, (void __user *)arg,
				sizeof(struct rmt_storage_cb))) {
			pr_err("%s: copy from user failed\n\n", __func__);
			ret = -EFAULT;
			if (atomic_dec_return(&rms->wcount) == 0)
				wake_unlock(&rms->wlock);
			break;
		}
		ret = rmt_storage_send_callback(&cb);
		if (rms->final_call) {
			if (atomic_dec_return(&rms->wcount) == 0) {
				rms->final_call = 0;
				printk(KERN_INFO"%s:rms->final_call=%d\n", __func__, rms->final_call);
				wake_up(&rms->final_waitq);
			} else
				printk(KERN_INFO"%s: rms->wcount != 0\n", __func__);
		}
		if (ret < 0)
			pr_err("%s: send callback failed with ret val = %d\n",
				__func__, ret);
		if (atomic_dec_return(&rms->wcount) == 0)
			wake_unlock(&rms->wlock);

		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

void wait_rmt_final_call_back(int timeout)
{
	int rc;
	struct rmt_storage_server_info *rms = _rms;
	rms->final_call = 1;

	rc = wait_event_timeout(rms->final_waitq, rms->final_call == 0, timeout * HZ);
	_rms->smem_info->server_status = 0;
	if (!rc)
		printk(KERN_INFO"%s: TIME OUT\n", __func__);
	else
		printk(KERN_INFO"%s: rc = %d\n", __func__, rc);
}

static int rmt_storage_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct rmt_storage_server_info *rms = _rms;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	int ret = -EINVAL;

	if (vma->vm_pgoff != 0) {
		pr_err("%s: error: zero offset is required\n", __func__);
		goto out;
	}

	if (vsize > rms->rmt_shrd_mem.size) {
		pr_err("%s: error: size mismatch\n", __func__);
		goto out;
	}

	ret = io_remap_pfn_range(vma, vma->vm_start,
			rms->rmt_shrd_mem.start >> PAGE_SHIFT,
			vsize, vma->vm_page_prot);
	if (ret < 0)
		pr_err("%s: failed with return val %d \n", __func__, ret);
out:
	return ret;
}

const struct file_operations rmt_storage_fops = {
	.owner = THIS_MODULE,
	.open = rmt_storage_open,
	.unlocked_ioctl	 = rmt_storage_ioctl,
	.mmap = rmt_storage_mmap,
	.release = rmt_storage_release,
};

static struct miscdevice rmt_storage_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rmt_storage",
	.fops = &rmt_storage_fops,
};

static ssize_t
set_force_sync(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	if (!!value)
		rmt_storage_force_sync();
	return count;
}

static ssize_t
show_sync_sts(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	ret = rmt_storage_get_sync_status();
	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static DEVICE_ATTR(force_sync, S_IRUGO | S_IWUSR, NULL, set_force_sync);
static DEVICE_ATTR(sync_sts, S_IRUGO | S_IWUSR, show_sync_sts, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_force_sync.attr,
	&dev_attr_sync_sts.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static int rmt_storage_server_probe(struct platform_device *pdev)
{
	struct rmt_storage_server_info *rms;
	struct shared_ramfs_entry *smem_info;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("%s: No resources for rmt_storage server\n", __func__);
		return -ENODEV;
	}

	smem_info = pdev->dev.platform_data;
	if (!smem_info) {
		pr_err("%s: platform data is not available\n", __func__);
		return -EINVAL;
	}

	rms = kzalloc(sizeof(struct rmt_storage_server_info), GFP_KERNEL);
	if (!rms) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}

	rms->rmt_shrd_mem.start = res->start;
	rms->rmt_shrd_mem.size = resource_size(res);
	init_waitqueue_head(&rms->event_q);
	spin_lock_init(&rms->lock);
	atomic_set(&rms->total_events, 0);
	INIT_LIST_HEAD(&rms->event_list);
	INIT_LIST_HEAD(&rms->client_info_list);
	/* The client expects a non-zero return value for
	 * its open requests. Hence reserve 0 bit.  */
	__set_bit(0, &rms->cids);
	atomic_set(&rms->wcount, 0);
	wake_lock_init(&rms->wlock, WAKE_LOCK_SUSPEND, "rmt_storage");
	rms->final_call = 0;
	init_waitqueue_head(&rms->final_waitq);

	ret = misc_register(&rmt_storage_device);
	if (ret) {
		pr_err("%s: Unable to register misc device %d\n", __func__,
				MISC_DYNAMIC_MINOR);
		wake_lock_destroy(&rms->wlock);
		kfree(rms);
		return ret;
	}

	ret = msm_rpc_create_server(&rmt_storage_server);
	if (ret) {
		pr_err("%s: Unable to register rmt storage server\n", __func__);
		ret = misc_deregister(&rmt_storage_device);
		if (ret)
			pr_err("%s: Unable to deregister misc device %d\n",
					__func__, MISC_DYNAMIC_MINOR);
		wake_lock_destroy(&rms->wlock);
		kfree(rms);
		return ret;
	}
	rms->smem_info = smem_info;

	ret = sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp);
	if (ret)
		pr_info("%s: Failed to create sysfs node: %d\n", __func__, ret);

	pr_info("%s: Remote storage RPC server initialized\n", __func__);
	_rms = rms;
	return 0;
}

#if 0
static void rmt_storage_server_shutdown(struct platform_device *pdev)
{
	if (!_rms->smem_info) {
		pr_err("%s: platform data is not available\n", __func__);
		return;
	}
	_rms->smem_info->server_status = 0;
}
#endif

static struct platform_driver rmt_storage_driver = {
	.probe	= rmt_storage_server_probe,
	/* don't need shutdown function(rmt_storage_server_shutdown)
		since we need to backup NV before poweroff/reset */
	.shutdown = NULL,
	.driver	= {
		.name	= "rmt_storage",
		.owner	= THIS_MODULE,
	},
};

static int __init rmt_storage_server_init(void)
{
	return platform_driver_register(&rmt_storage_driver);
}

module_init(rmt_storage_server_init);
MODULE_DESCRIPTION("Remote Storage RPC Server");
MODULE_LICENSE("Dual BSD/GPL");
