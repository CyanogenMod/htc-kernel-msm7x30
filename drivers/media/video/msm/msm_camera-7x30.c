/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*FIXME: most allocations need not be GFP_ATOMIC*/
/* FIXME: management of mutexes */
/* FIXME: msm_pmem_region_lookup return values */
/* FIXME: way too many copy to/from user */
/* FIXME: does region->active mean free */
/* FIXME: check limits on command lenghts passed from userspace */
/* FIXME: __msm_release: which queues should we flush when opencnt != 0 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <mach/board.h>
#include <linux/leds.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/android_pmem.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <media/msm_camera-7x30.h>
#include <media/msm_camera_sensor.h>
#include <mach/camera.h>
#include <mach/msm_flashlight.h>
#include <linux/delay.h>
DEFINE_MUTEX(hlist_mut);
#include <asm/cacheflush.h>
#include <linux/rtc.h>
#include <linux/slab.h>

DEFINE_MUTEX(pp_prev_lock);
DEFINE_MUTEX(pp_snap_lock);
DEFINE_MUTEX(ctrl_cmd_lock);

#define MSM_MAX_CAMERA_SENSORS 5
#define CAMERA_STOP_SNAPSHOT 42

#define ERR_USER_COPY(to) pr_err("[CAM]%s(%d): copy %s user\n", \
				__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)

static struct class *msm_class;
static dev_t msm_devno;
static LIST_HEAD(msm_sensors);
struct  msm_control_device *g_v4l2_control_device;
int g_v4l2_opencnt;

#define __CONTAINS(r, v, l, field) ({				\
	typeof(r) __r = r;					\
	typeof(v) __v = v;					\
	typeof(v) __e = __v + l;				\
	int res = __v >= __r->field &&				\
		__e <= __r->field + __r->len;			\
	res;							\
})

#define CONTAINS(r1, r2, field) ({				\
	typeof(r2) __r2 = r2;					\
	__CONTAINS(r1, __r2->field, __r2->len, field);		\
})

#define IN_RANGE(r, v, field) ({				\
	typeof(r) __r = r;					\
	typeof(v) __vv = v;					\
	int res = ((__vv >= __r->field) &&			\
		(__vv < (__r->field + __r->len)));		\
	res;							\
})

#define OVERLAPS(r1, r2, field) ({				\
	typeof(r1) __r1 = r1;					\
	typeof(r2) __r2 = r2;					\
	typeof(__r2->field) __v = __r2->field;			\
	typeof(__v) __e = __v + __r2->len - 1;			\
	int res = (IN_RANGE(__r1, __v, field) ||		\
		   IN_RANGE(__r1, __e, field));                 \
	res;							\
})

static inline void free_qcmd(struct msm_queue_cmd *qcmd)
{
	if (!qcmd || !atomic_read(&qcmd->on_heap))
		return;
	if (!atomic_sub_return(1, &qcmd->on_heap))
		kfree(qcmd);
}

static void msm_queue_init(struct msm_device_queue *queue, const char *name)
{
	spin_lock_init(&queue->lock);
	queue->len = 0;
	queue->max = 0;
	queue->name = name;
	INIT_LIST_HEAD(&queue->list);
	init_waitqueue_head(&queue->wait);
}

static void msm_enqueue(struct msm_device_queue *queue,
		struct list_head *entry)
{
	unsigned long flags;
	spin_lock_irqsave(&queue->lock, flags);
	queue->len++;

	if (queue->len > queue->max) {
		queue->max = queue->len;
#if 0
		pr_info("[CAM]%s: queue %s new max is %d\n", __func__,
			queue->name, queue->max);
#else
		/* HTC */
		if(queue->max < 1024)
			pr_info("[CAM]%s: queue %s new max is %d\n", __func__,
			queue->name, queue->max);
#endif
	}
	list_add_tail(entry, &queue->list);
	wake_up(&queue->wait);
	CDBG("%s: woke up %s\n", __func__, queue->name);
	spin_unlock_irqrestore(&queue->lock, flags);
}

static void msm_enqueue_vpe(struct msm_device_queue *queue,
               struct list_head *entry)
{
       unsigned long flags;
       spin_lock_irqsave(&queue->lock, flags);
       queue->len++;
       if (queue->len > queue->max) {
               queue->max = queue->len;
               pr_info("[CAM]%s: queue %s new max is %d\n", __func__,
                       queue->name, queue->max);
       }
       list_add_tail(entry, &queue->list);
    CDBG("%s: woke up %s\n", __func__, queue->name);
       spin_unlock_irqrestore(&queue->lock, flags);
}

#define msm_dequeue(queue, member) ({				\
	unsigned long flags;					\
	struct msm_device_queue *__q = (queue);			\
	struct msm_queue_cmd *qcmd = 0;				\
	spin_lock_irqsave(&__q->lock, flags);			\
	if (!__q->len)   \
		pr_info("[CAM]%s: queue name: %s: queue len: %d", \
				__func__, __q->name, __q->len); \
	if (!list_empty(&__q->list)) {				\
		__q->len--;					\
		qcmd = list_first_entry(&__q->list,		\
				struct msm_queue_cmd, member);	\
		if ((qcmd) && (&qcmd->member) && (&qcmd->member.next)) {  \
		list_del_init(&qcmd->member);			\
		}                                   \
	}												\
		spin_unlock_irqrestore(&__q->lock, flags);	\
	qcmd;							\
})

#define msm_queue_drain(queue, member) do {			\
	unsigned long flags;					\
	struct msm_device_queue *__q = (queue);			\
	struct msm_queue_cmd *qcmd;				\
	spin_lock_irqsave(&__q->lock, flags);			\
	CDBG("%s: draining queue %s\n", __func__, __q->name);	\
	while (!list_empty(&__q->list)) {			\
		__q->len--;					\
		pr_info("[CAM]%s,q->len = %d\n", __func__, __q->len);	\
		qcmd = list_first_entry(&__q->list,		\
			struct msm_queue_cmd, member);		\
		if (qcmd) {                         \
			if ((&qcmd->member) && (&qcmd->member.next) && (&qcmd->member.prev)) \
				list_del_init(&qcmd->member);			\
		free_qcmd(qcmd);				\
		}                                   \
	};							\
	__q->len = 0;									\
	spin_unlock_irqrestore(&__q->lock, flags);		\
} while(0)

static int check_overlap(struct hlist_head *ptype,
			unsigned long paddr,
			unsigned long len)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region t = { .paddr = paddr, .len = len };
	struct hlist_node *node;

	hlist_for_each_entry(region, node, ptype, list) {
		if (CONTAINS(region, &t, paddr) ||
				CONTAINS(&t, region, paddr) ||
				OVERLAPS(region, &t, paddr)) {
			printk(KERN_ERR
				" region (PHYS %p len %ld)"
				" clashes with registered region"
				" (paddr %p len %ld)\n",
				(void *)t.paddr, t.len,
				(void *)region->paddr, region->len);
			return -1;
		}
	}

	return 0;
}

static int check_pmem_info(struct msm_pmem_info *info, int len)
{
	/* HTC */
	if (info->offset & (PAGE_SIZE - 1)) {
		pr_err("[CAM]%s: pmem offset is not page-aligned\n", __func__);
		goto error;
	}

	if (info->offset < len &&
	    info->offset + info->len <= len &&
	    info->y_off < len &&
	    info->cbcr_off < len)
		return 0;

error:
	pr_err("[CAM]%s: check failed: off %d len %d y %d cbcr %d (total len %d)\n",
		__func__,
		info->offset,
		info->len,
		info->y_off,
		info->cbcr_off,
		len);
	return -EINVAL;
}

static int msm_pmem_table_add(struct hlist_head *ptype,
	struct msm_pmem_info *info)
{
	struct file *file;
	unsigned long paddr;
	unsigned long kvstart;
	unsigned long len;
	int rc;
	struct msm_pmem_region *region;

	rc = get_pmem_file(info->fd, &paddr, &kvstart, &len, &file);
	if (rc < 0) {
		pr_err("[CAM]%s: get_pmem_file fd %d error %d\n",
			__func__,
			info->fd, rc);
		return rc;
	}

	if (!info->len)
		info->len = len;

	rc = check_pmem_info(info, len);
	if (rc < 0)
		return rc;

	paddr += info->offset;
	kvstart += info->offset;
	len = info->len;

	if (check_overlap(ptype, paddr, len) < 0)
		return -EINVAL;

	CDBG("%s: type %d, paddr 0x%lx, vaddr 0x%lx\n",
		__func__,
		info->type, paddr, (unsigned long)info->vaddr);

	region = kzalloc(sizeof(struct msm_pmem_region), GFP_KERNEL);
	if (!region)
		return -ENOMEM;

	INIT_HLIST_NODE(&region->list);

	region->paddr = paddr;
	region->kvaddr = kvstart;
	region->len = len;
	region->file = file;
	memcpy(&region->info, info, sizeof(region->info));

	hlist_add_head(&(region->list), ptype);

	return 0;
}

/* return of 0 means failure */
static uint8_t msm_pmem_region_lookup(struct hlist_head *ptype,
	int pmem_type, struct msm_pmem_region *reg, uint8_t maxcount)
{
	struct msm_pmem_region *region;
	struct msm_pmem_region *regptr;
	struct hlist_node *node, *n;

	uint8_t rc = 0;

	regptr = reg;
	mutex_lock(&hlist_mut);
	hlist_for_each_entry_safe(region, node, n, ptype, list) {
		if (region->info.type == pmem_type && region->info.active) {
			*regptr = *region;
			rc += 1;
			if (rc >= maxcount)
				break;
			regptr++;
		}
	}
	mutex_unlock(&hlist_mut);
	return rc;
}

static uint8_t msm_pmem_region_lookup_2(struct hlist_head *ptype,
                                       int pmem_type,
                                       struct msm_pmem_region *reg,
                                       uint8_t maxcount)
{
       struct msm_pmem_region *region;
       struct msm_pmem_region *regptr;
       struct hlist_node *node, *n;
       uint8_t rc = 0;
       regptr = reg;
       mutex_lock(&hlist_mut);
       hlist_for_each_entry_safe(region, node, n, ptype, list) {
               if (region->info.type == pmem_type && region->info.active) {
                       printk(KERN_ERR "info.type=%d, pmem_type = %d,"
                                                       "info.active = %d,\n",
                               region->info.type, pmem_type,
                               region->info.active);
                       *regptr = *region;
                       region->info.type = MSM_PMEM_VIDEO;
                       rc += 1;
                       if (rc >= maxcount)
                               break;
                       regptr++;
               }
       }
       mutex_unlock(&hlist_mut);
       return rc;
}

static int msm_pmem_frame_ptov_lookup(struct msm_sync *sync,
		unsigned long pyaddr,
		unsigned long pcbcraddr,
		struct msm_pmem_info *pmem_info,
		int clear_active)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->pmem_frames, list) {
		if (pyaddr == (region->paddr + region->info.y_off) &&
				pcbcraddr == (region->paddr +
						region->info.cbcr_off) &&
				region->info.active) {
			/* offset since we could pass vaddr inside
			 * a registerd pmem buffer
			 */
			memcpy(pmem_info, &region->info, sizeof(*pmem_info));
			if (clear_active)
				region->info.active = 0;
			return 0;
		}
	}

	return -EINVAL;
}

static unsigned long msm_pmem_stats_ptov_lookup(struct msm_sync *sync,
		unsigned long addr, int *fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		if (addr == region->paddr && region->info.active) {
			/* offset since we could pass vaddr inside a
			 * registered pmem buffer */
			*fd = region->info.fd;
			region->info.active = 0;
			return (unsigned long)(region->info.vaddr);
		}
	}
#if 1
	/* HTC */
	printk("[CAM]msm_pmem_stats_ptov_lookup: lookup vaddr..\n");
	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		if (addr == (unsigned long)(region->info.vaddr)) {
			/* offset since we could pass vaddr inside a
			 * registered pmem buffer */
			*fd = region->info.fd;
			region->info.active = 0;
			return (unsigned long)(region->info.vaddr);
		}
	}
#endif
	return 0;
}

static unsigned long msm_pmem_frame_vtop_lookup(struct msm_sync *sync,
		unsigned long buffer,
		uint32_t yoff, uint32_t cbcroff, int fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region,
		node, n, &sync->pmem_frames, list) {
		if (((unsigned long)(region->info.vaddr) == buffer) &&
				(region->info.y_off == yoff) &&
				(region->info.cbcr_off == cbcroff) &&
				(region->info.fd == fd) &&
				(region->info.active == 0)) {
			region->info.active = 1;
			return region->paddr;
		}
	}

	return 0;
}

static unsigned long msm_pmem_stats_vtop_lookup(
		struct msm_sync *sync,
		unsigned long buffer,
		int fd)
{
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	hlist_for_each_entry_safe(region, node, n, &sync->pmem_stats, list) {
		if (((unsigned long)(region->info.vaddr) == buffer) &&
				(region->info.fd == fd) &&
				region->info.active == 0) {
			region->info.active = 1;
			return region->paddr;
		}
	}

	return 0;
}

static int __msm_pmem_table_del(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
{
	int rc = 0;
	struct msm_pmem_region *region;
	struct hlist_node *node, *n;

	switch (pinfo->type) {
	case MSM_PMEM_VIDEO:
	case MSM_PMEM_PREVIEW:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
	case MSM_PMEM_VIDEO_VPE:
		hlist_for_each_entry_safe(region, node, n,
			&sync->pmem_frames, list) {

			if (pinfo->type == region->info.type &&
					pinfo->vaddr == region->info.vaddr &&
					pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
		hlist_for_each_entry_safe(region, node, n,
			&sync->pmem_stats, list) {

			if (pinfo->type == region->info.type &&
					pinfo->vaddr == region->info.vaddr &&
					pinfo->fd == region->info.fd) {
				hlist_del(node);
				put_pmem_file(region->file);
				kfree(region);
			}
		}
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_pmem_table_del(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_pmem_table_del(sync, &info);
}

static int __msm_get_frame(struct msm_sync *sync,
		struct msm_frame *frame)
{
	int rc = 0;

	struct msm_pmem_info pmem_info;
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_vfe_resp *vdata;
	struct msm_vfe_phy_info *pphy;

	if (&sync->frame_q) {
	qcmd = msm_dequeue(&sync->frame_q, list_frame);
	}

	if (!qcmd) {
		pr_err("[CAM]%s: no preview frame.\n", __func__);
		return -EAGAIN;
	}

	vdata = (struct msm_vfe_resp *)(qcmd->command);
	pphy = &vdata->phy;

	rc = msm_pmem_frame_ptov_lookup(sync,
			pphy->y_phy,
			pphy->cbcr_phy,
			&pmem_info,
			1); /* mark frame in use */
	CDBG("%s:  get frame, lookup address "
			"y %x cbcr %x\n",
			__func__,
			pphy->y_phy,
			pphy->cbcr_phy);
	if (rc < 0) {
		pr_err("[CAM]%s: cannot get frame, invalid lookup address "
			"y %x cbcr %x\n",
			__func__,
			pphy->y_phy,
			pphy->cbcr_phy);
		goto err;
	}

	frame->ts = qcmd->ts;
	frame->buffer = (unsigned long)pmem_info.vaddr;
	frame->y_off = pmem_info.y_off;
	frame->cbcr_off = pmem_info.cbcr_off;
	frame->fd = pmem_info.fd;
	frame->path = vdata->phy.output_id;
	CDBG("%s: y %x, cbcr %x, qcmd %x, virt_addr %x\n",
		__func__,
		pphy->y_phy, pphy->cbcr_phy, (int) qcmd, (int) frame->buffer);

err:
	free_qcmd(qcmd);
	return rc;
}

static int msm_get_frame(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_frame frame;

	if (copy_from_user(&frame,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	rc = __msm_get_frame(sync, &frame);
	if (rc < 0)
		return rc;

	/* read the frame after the status has been read */
	rmb();

	if (sync->croplen) {
		if (frame.croplen != sync->croplen) {
			pr_err("[CAM]%s: invalid frame croplen %d,"
				"expecting %d\n",
				__func__,
				frame.croplen,
				sync->croplen);
			return -EINVAL;
		}

		if (copy_to_user((void *)frame.cropinfo,
				sync->cropinfo,
				sync->croplen)) {
			ERR_COPY_TO_USER();
			return -EFAULT;
		}
	}

	if (copy_to_user((void *)arg,
				&frame, sizeof(struct msm_frame))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	CDBG("%s: got frame\n", __func__);

	return rc;
}

static int msm_enable_vfe(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;
	struct camera_enable_cmd cfg;

	if (copy_from_user(&cfg,
			arg,
			sizeof(struct camera_enable_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (sync->vfefn.vfe_enable)
		rc = sync->vfefn.vfe_enable(&cfg);

	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static int msm_disable_vfe(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;
	struct camera_enable_cmd cfg;

	if (copy_from_user(&cfg,
			arg,
			sizeof(struct camera_enable_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (sync->vfefn.vfe_disable)
		rc = sync->vfefn.vfe_disable(&cfg, NULL);

	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static struct msm_queue_cmd *__msm_control(struct msm_sync *sync,
		struct msm_device_queue *queue,
		struct msm_queue_cmd *qcmd,
		int timeout)
{
	int rc;

	msm_enqueue(&sync->event_q, &qcmd->list_config);

	if (!queue)
		return NULL;

	/* wait for config status */
	rc = wait_event_interruptible_timeout(
			queue->wait,
			!list_empty_careful(&queue->list),
			timeout);
	if (list_empty_careful(&queue->list)) {
		if (!rc)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			pr_err("[CAM]%s: wait_event error %d\n", __func__, rc);
			/* HTC */
			/* qcmd may be still on the event_q, in which case we
			 * need to remove it.  Alternatively, qcmd may have
			 * been dequeued and processed already, in which case
			 * the list removal will be a no-op.
			*/
			list_del_init(&qcmd->list_config);
			return ERR_PTR(rc);
		}
	}

	qcmd = msm_dequeue(queue, list_control);
	BUG_ON(!qcmd);

	return qcmd;
}

static struct msm_queue_cmd *__msm_control_nb(struct msm_sync *sync,
					struct msm_queue_cmd *qcmd_to_copy)
{
	/* Since this is a non-blocking command, we cannot use qcmd_to_copy and
	 * its data, since they are on the stack.  We replicate them on the heap
	 * and mark them on_heap so that they get freed when the config thread
	 * dequeues them.
	 */

	struct msm_ctrl_cmd *udata;
	struct msm_ctrl_cmd *udata_to_copy = qcmd_to_copy->command;

	struct msm_queue_cmd *qcmd =
			kzalloc(sizeof(*qcmd_to_copy) +
				sizeof(*udata_to_copy) +
				udata_to_copy->length,
				GFP_KERNEL);
	if (!qcmd) {
		pr_err("[CAM]%s: out of memory\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	*qcmd = *qcmd_to_copy;
	udata = qcmd->command = qcmd + 1;
	memcpy(udata, udata_to_copy, sizeof(*udata));
	udata->value = udata + 1;
	memcpy(udata->value, udata_to_copy->value, udata_to_copy->length);

	atomic_set(&qcmd->on_heap, 1);

	/* qcmd_resp will be set to NULL */
	return __msm_control(sync, NULL, qcmd, 0);
}

static int msm_control(struct msm_control_device *ctrl_pmsm,
			int block,
			void __user *arg)
{
	int rc = 0;

	struct msm_sync *sync = ctrl_pmsm->pmsm->sync;
	void __user *uptr;
	struct msm_ctrl_cmd udata;
	struct msm_queue_cmd qcmd;
	struct msm_queue_cmd *qcmd_resp = NULL;
	uint8_t data[50];

	if (copy_from_user(&udata, arg, sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		rc = -EFAULT;
		goto end;
	}

	uptr = udata.value;
	udata.value = data;
	if (udata.type == CAMERA_STOP_SNAPSHOT)
		sync->get_pic_abort = 1;

	atomic_set(&(qcmd.on_heap), 0);
	qcmd.type = MSM_CAM_Q_CTRL;
	qcmd.command = &udata;

	if (udata.length) {
		if (udata.length > sizeof(data)) {
			pr_err("[CAM]%s: user data too large (%d, max is %d)\n",
					__func__,
					udata.length,
					sizeof(data));
			rc = -EIO;
			goto end;
		}
		if (copy_from_user(udata.value, uptr, udata.length)) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
			goto end;
		}
	}

	if (unlikely(!block)) {
		qcmd_resp = __msm_control_nb(sync, &qcmd);
		goto end;
	}

	qcmd_resp = __msm_control(sync,
				  &ctrl_pmsm->ctrl_q,
				  &qcmd, MAX_SCHEDULE_TIMEOUT);

	if (!qcmd_resp || IS_ERR(qcmd_resp)) {
		/* Do not free qcmd_resp here.  If the config thread read it,
		 * then it has already been freed, and we timed out because
		 * we did not receive a MSM_CAM_IOCTL_CTRL_CMD_DONE.  If the
		 * config thread itself is blocked and not dequeueing commands,
		 * then it will either eventually unblock and process them,
		 * or when it is killed, qcmd will be freed in
		 * msm_release_config.
		 */
		rc = PTR_ERR(qcmd_resp);
		qcmd_resp = NULL;
		goto end;
	}

	if (qcmd_resp->command) {
		udata = *(struct msm_ctrl_cmd *)qcmd_resp->command;
		if (udata.length > 0) {
			if (copy_to_user(uptr,
					 udata.value,
					 udata.length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto end;
			}
		}
		udata.value = uptr;

		if (copy_to_user((void *)arg, &udata,
				sizeof(struct msm_ctrl_cmd))) {
			ERR_COPY_TO_USER();
			rc = -EFAULT;
			goto end;
		}
	}

end:
	free_qcmd(qcmd_resp);
	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

/* Divert frames for post-processing by delivering them to the config thread;
 * when post-processing is done, it will return the frame to the frame thread.
 */
static int msm_divert_frame(struct msm_sync *sync,
		struct msm_vfe_resp *data,
		struct msm_stats_event_ctrl *se)
{
	struct msm_pmem_info pinfo;
	struct msm_postproc buf;
	int rc;

	pr_info("[CAM]%s: preview PP sync->pp_mask %d\n", __func__, sync->pp_mask);

	if (!(sync->pp_mask & PP_PREV)) {
		pr_err("[CAM]%s: diverting preview frame but not in PP_PREV!\n",
			__func__);
		return -EINVAL;
	}

	rc = msm_pmem_frame_ptov_lookup(sync,
			data->phy.y_phy,
			data->phy.cbcr_phy,
			&pinfo,
			0);  /* do clear the active flag */
	if (rc < 0) {
		CDBG("%s: msm_pmem_frame_ptov_lookup failed\n", __func__);
		return rc;
	}

	buf.fmain.buffer = (unsigned long)pinfo.vaddr;
	buf.fmain.y_off = pinfo.y_off;
	buf.fmain.cbcr_off = pinfo.cbcr_off;
	buf.fmain.fd = pinfo.fd;

	CDBG("%s: buf %ld fd %d\n",
		__func__, buf.fmain.buffer,
		buf.fmain.fd);
	if (copy_to_user((void *)(se->stats_event.data),
			&(buf.fmain),
			sizeof(struct msm_frame))) {
		ERR_COPY_TO_USER();
		return -EFAULT;
	}

	return 0;
}

static int msm_divert_snapshot(struct msm_sync *sync,
		struct msm_vfe_resp *data,
		struct msm_stats_event_ctrl *se)
{
	struct msm_postproc buf;
	struct msm_pmem_region region;

	CDBG("%s: preview PP sync->pp_mask %d\n", __func__, sync->pp_mask);

	if (!(sync->pp_mask & (PP_SNAP|PP_RAW_SNAP))) {
		pr_err("[CAM]%s: diverting snapshot but not in PP_SNAP!\n",
			__func__);
		return -EINVAL;
	}

	memset(&region, 0, sizeof(region));
	buf.fmnum = msm_pmem_region_lookup(&sync->pmem_frames,
					MSM_PMEM_MAINIMG,
					&region, 1);
	if (buf.fmnum == 1) {
		buf.fmain.buffer = (uint32_t)region.info.vaddr;
		buf.fmain.y_off  = region.info.y_off;
		buf.fmain.cbcr_off = region.info.cbcr_off;
		buf.fmain.fd = region.info.fd;
	} else {
		if (buf.fmnum > 1)
			pr_err("[CAM]%s: MSM_PMEM_MAINIMG lookup found %d\n",
				__func__, buf.fmnum);
		buf.fmnum = msm_pmem_region_lookup(&sync->pmem_frames,
					MSM_PMEM_RAW_MAINIMG,
					&region, 1);
		if (buf.fmnum == 1) {
			buf.fmain.path = MSM_FRAME_PREV_2;
			buf.fmain.buffer = (uint32_t)region.info.vaddr;
			buf.fmain.fd = region.info.fd;
		} else {
			pr_err("[CAM]%s: pmem lookup fail (found %d)\n",
				__func__, buf.fmnum);
			return -EIO;
		}
	}

	CDBG("%s: snapshot copy_to_user!\n", __func__);
	if (copy_to_user((void *)(se->stats_event.data), &buf, sizeof(buf))) {
		ERR_COPY_TO_USER();
		return -EFAULT;
	}

	return 0;
}

static int msm_get_stats(struct msm_sync *sync, void __user *arg)
{
	int timeout;
	int rc = 0;

	struct msm_stats_event_ctrl se;

	struct msm_queue_cmd *qcmd = NULL;
	struct msm_ctrl_cmd  *ctrl = NULL;
	struct msm_vfe_resp  *data = NULL;
	struct msm_stats_buf stats;

	if (copy_from_user(&se, arg,
			sizeof(struct msm_stats_event_ctrl))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	timeout = (int)se.timeout_ms;

	CDBG("%s: timeout %d\n", __func__, timeout);
	rc = wait_event_interruptible_timeout(
			sync->event_q.wait,
			!list_empty_careful(&sync->event_q.list),
			msecs_to_jiffies(timeout));
	if (list_empty_careful(&sync->event_q.list)) {
		if (rc == 0)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			pr_err("[CAM]%s: error %d\n", __func__, rc);
			return rc;
		}
	}
	CDBG("%s: returned from wait: %d\n", __func__, rc);

	rc = 0;

	qcmd = msm_dequeue(&sync->event_q, list_config);
	BUG_ON(!qcmd);

	/* HTC: check qcmd */
	if (!qcmd) {
		rc = -EFAULT;
		pr_err("[CAM]%s: qcmd is NULL, rc %d\n", __func__, rc);
		return rc;
	}

	CDBG("%s: received from DSP %d\n", __func__, qcmd->type);

	/* order the reads of stat/snapshot buffers */
	rmb();

	switch (qcmd->type) {
	case MSM_CAM_Q_VFE_EVT:
	case MSM_CAM_Q_VFE_MSG:
		data = (struct msm_vfe_resp *)(qcmd->command);

		/* adsp event and message */
		se.resptype = MSM_CAM_RESP_STAT_EVT_MSG;

		/* 0 - msg from aDSP, 1 - event from mARM */
		se.stats_event.type   = data->evt_msg.type;
		se.stats_event.msg_id = data->evt_msg.msg_id;
		se.stats_event.len    = data->evt_msg.len;

		CDBG("%s: qcmd->type %d length %d msd_id %d\n", __func__,
			qcmd->type,
			se.stats_event.len,
			se.stats_event.msg_id);

		if ((data->type == VFE_MSG_STATS_AF) ||
				(data->type == VFE_MSG_STATS_WE) ||
				(data->type == VFE_MSG_STATS_AEC) ||
				(data->type == VFE_MSG_STATS_IHIST) ||
				(data->type == VFE_MSG_STATS_AWB)) {

			stats.buffer =
			msm_pmem_stats_ptov_lookup(sync,
					data->phy.sbuf_phy,
					&(stats.fd));
			if (!stats.buffer) {
				pr_err("[CAM]%s: msm_pmem_stats_ptov_lookup error\n",
					__func__);
#if 1
				/* HTC */
				se.resptype = MSM_CAM_RESP_MAX;
#else
				rc = -EINVAL;
				goto failure;
#endif
			}

			if (copy_to_user((void *)(se.stats_event.data),
					&stats,
					sizeof(struct msm_stats_buf))) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		} else if ((data->evt_msg.len > 0) &&
				(data->type == VFE_MSG_GENERAL)) {
			if (copy_to_user((void *)(se.stats_event.data),
					data->evt_msg.data,
					data->evt_msg.len)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		} else {
			if ((sync->pp_mask & PP_PREV) &&
				(data->type == VFE_MSG_OUTPUT_P))
					rc = msm_divert_frame(sync, data, &se);
			else if ((sync->pp_mask & (PP_SNAP|PP_RAW_SNAP)) &&
				  (data->type == VFE_MSG_SNAPSHOT ||
				   data->type == VFE_MSG_OUTPUT_S))
					rc = msm_divert_snapshot(sync,
								data, &se);
		}
		break;

	case MSM_CAM_Q_CTRL:
		/* control command from control thread */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("%s: qcmd->type %d length %d\n", __func__,
			qcmd->type, ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
						ctrl->value,
						ctrl->length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		}

		se.resptype = MSM_CAM_RESP_CTRL;

		/* what to control */
		se.ctrl_cmd.type = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
		se.ctrl_cmd.resp_fd = ctrl->resp_fd;
		break;

	case MSM_CAM_Q_V4L2_REQ:
		/* control command from v4l2 client */
		ctrl = (struct msm_ctrl_cmd *)(qcmd->command);

		CDBG("%s: qcmd->type %d len %d\n", __func__, qcmd->type, ctrl->length);

		if (ctrl->length > 0) {
			if (copy_to_user((void *)(se.ctrl_cmd.value),
					ctrl->value, ctrl->length)) {
				ERR_COPY_TO_USER();
				rc = -EFAULT;
				goto failure;
			}
		}

		/* 2 tells config thread this is v4l2 request */
		se.resptype = MSM_CAM_RESP_V4L2;

		/* what to control */
		se.ctrl_cmd.type   = ctrl->type;
		se.ctrl_cmd.length = ctrl->length;
		break;

	default:
		rc = -EFAULT;
		goto failure;
	} /* switch qcmd->type */

	if (copy_to_user((void *)arg, &se, sizeof(se))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
		goto failure;
	}

failure:
	free_qcmd(qcmd);

	CDBG("%s: %d\n", __func__, rc);
	return rc;
}

static int msm_ctrl_cmd_done(struct msm_control_device *ctrl_pmsm,
		void __user *arg)
{
	void __user *uptr;
	struct msm_queue_cmd *qcmd = &ctrl_pmsm->qcmd;
	struct msm_ctrl_cmd *command = &ctrl_pmsm->ctrl;

	if (copy_from_user(command, arg, sizeof(*command))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	atomic_set(&qcmd->on_heap, 0);
	qcmd->command = command;
	uptr = command->value;

	if (command->length > 0) {
		command->value = ctrl_pmsm->ctrl_data;
		if (command->length > sizeof(ctrl_pmsm->ctrl_data)) {
			pr_err("[CAM]%s: user data %d is too big (max %d)\n",
				__func__, command->length,
				sizeof(ctrl_pmsm->ctrl_data));
			return -EINVAL;
		}
		if (copy_from_user(command->value,
					uptr,
					command->length)) {
			ERR_COPY_FROM_USER();
			return -EFAULT;
		}
	} else
		command->value = NULL;

	CDBG("%s: end\n", __func__);

	/* wake up control thread */
	msm_enqueue(&ctrl_pmsm->ctrl_q, &qcmd->list_control);

	return 0;
}

static int msm_config_vpe(struct msm_sync *sync, void __user *arg)
{
       struct msm_vpe_cfg_cmd cfgcmd;
       if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
               ERR_COPY_FROM_USER();
               return -EFAULT;
       }
       CDBG("%s: cmd_type %d\n", __func__, cfgcmd.cmd_type);
       switch (cfgcmd.cmd_type) {
       case CMD_VPE:
               return sync->vpefn.vpe_config(&cfgcmd, NULL);
       default:
               pr_err("[CAM]%s: unknown command type %d\n",
                       __func__, cfgcmd.cmd_type);
       }
       return -EINVAL;
}

static int msm_config_vfe(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;
	struct msm_pmem_region region[8];
	struct axidata axi_data;

	if (!sync->vfefn.vfe_config) {
		pr_err("[CAM]%s: no vfe_config!\n", __func__);
		return -EIO;
	}

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	memset(&axi_data, 0, sizeof(axi_data));

	CDBG("%s: cmd_type %d\n", __func__, cfgcmd.cmd_type);

	switch (cfgcmd.cmd_type) {
	case CMD_STATS_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AEC_AWB, &region[0],
					NUM_STAT_OUTPUT_BUFFERS);

	/* HTC: check axi_data.bufnum1 if out of bound of "region" array */
	   if (!axi_data.bufnum1 || axi_data.bufnum1 >=
			(sizeof(region)/sizeof(struct msm_pmem_region))) {
		  pr_err("[CAM]%s %d: pmem region lookup error or out of bound\n",
				__func__, __LINE__);
		  return -EINVAL;
		}

		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AF, &region[axi_data.bufnum1],
					NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1 || !axi_data.bufnum2) {
			pr_err("[CAM]%s: pmem region lookup error\n", __func__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AF_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
					MSM_PMEM_AF, &region[0],
					NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AEC_AWB, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AEC_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AEC, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_AWB_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_AWB, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);

	case CMD_STATS_IHIST_ENABLE:
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats,
			MSM_PMEM_IHIST, &region[0],
			NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
		return sync->vfefn.vfe_config(&cfgcmd, &axi_data);
	case CMD_STATS_RS_ENABLE:
	case CMD_STATS_CS_ENABLE:
		return -EINVAL;
	case CMD_GENERAL:
	case CMD_STATS_DISABLE:
		return sync->vfefn.vfe_config(&cfgcmd, NULL);
	default:
		pr_err("[CAM]%s: unknown command type %d\n",
			__func__, cfgcmd.cmd_type);
	}

	return -EINVAL;
}

static int msm_vpe_frame_cfg(struct msm_sync *sync,
                               void *cfgcmdin)
{
       int rc = -EIO;
       struct axidata axi_data;
       void *data = &axi_data;
       struct msm_pmem_region region[8];
       int pmem_type;

       struct msm_vpe_cfg_cmd *cfgcmd;
       cfgcmd = (struct msm_vpe_cfg_cmd *)cfgcmdin;

       memset(&axi_data, 0, sizeof(axi_data));
       CDBG("In vpe_frame_cfg cfgcmd->cmd_type = %d \n",
               cfgcmd->cmd_type);
       switch (cfgcmd->cmd_type) {
       case CMD_AXI_CFG_VPE:
               pmem_type = MSM_PMEM_VIDEO_VPE;
               axi_data.bufnum1 =
                       msm_pmem_region_lookup_2(&sync->pmem_frames, pmem_type,
                                                               &region[0], 8);
               CDBG("axi_data.bufnum1 = %d\n", axi_data.bufnum1);
               if (!axi_data.bufnum1) {
                       pr_err("[CAM]%s %d: pmem region lookup error\n",
                               __func__, __LINE__);
                       return -EINVAL;
               }
               pmem_type = MSM_PMEM_VIDEO;
               break;
       default:
               pr_err("[CAM]%s: unknown command type %d\n",
                       __func__, cfgcmd->cmd_type);
               break;
       }
       axi_data.region = &region[0];
       CDBG("out vpe_frame_cfg cfgcmd->cmd_type = %d \n",
               cfgcmd->cmd_type);
       /* send the AXI configuration command to driver */
       if (sync->vpefn.vpe_config)
               rc = sync->vpefn.vpe_config(cfgcmd, data);
       return rc;
}

static int msm_frame_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;
	struct msm_pmem_region region[8];
	int pmem_type;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {

	case CMD_AXI_CFG_PREVIEW:
		pmem_type = MSM_PMEM_PREVIEW;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum2) {
			pr_err("[CAM]%s %d: pmem region lookup error (empty %d)\n",
				__func__, __LINE__,
				hlist_empty(&sync->pmem_frames));
			return -EINVAL;
		}
		break;

	case CMD_AXI_CFG_VIDEO:
		pmem_type = MSM_PMEM_PREVIEW;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_VIDEO;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[axi_data.bufnum1],
				(8-(axi_data.bufnum1)));
		if (!axi_data.bufnum2) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;


	case CMD_AXI_CFG_SNAP:
		pmem_type = MSM_PMEM_THUMBNAIL;
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		pmem_type = MSM_PMEM_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[axi_data.bufnum1],
				(8-(axi_data.bufnum1)));
		if (!axi_data.bufnum2) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_RAW_PICT_AXI_CFG:
		pmem_type = MSM_PMEM_RAW_MAINIMG;
		axi_data.bufnum2 =
			msm_pmem_region_lookup(&sync->pmem_frames, pmem_type,
				&region[0], 8);
		if (!axi_data.bufnum2) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		break;

	case CMD_GENERAL:
		data = NULL;
		break;

	default:
		pr_err("[CAM]%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	axi_data.region = &region[0];

	/* send the AXI configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, data);

	return rc;
}

static int msm_get_sensor_info(struct msm_sync *sync, void __user *arg)
{
	int rc = 0;
	struct msm_camsensor_info info;
	struct msm_camera_sensor_info *sdata;

	if (copy_from_user(&info,
			arg,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	sdata = sync->pdev->dev.platform_data;
	CDBG("%s: sensor_name %s\n", __func__, sdata->sensor_name);

	memcpy(&info.name[0],
		sdata->sensor_name,
		MAX_SENSOR_NAME);
	info.flash_enabled = !!sdata->flash_cfg;

	/* copy back to user space */
	if (copy_to_user((void *)arg,
			&info,
			sizeof(struct msm_camsensor_info))) {
		ERR_COPY_TO_USER();
		rc = -EFAULT;
	}

	return rc;
}

static int __msm_put_frame_buf(struct msm_sync *sync,
		struct msm_frame *pb)
{
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	int rc = -EIO;

	pphy = msm_pmem_frame_vtop_lookup(sync,
		pb->buffer,
		pb->y_off, pb->cbcr_off, pb->fd);

	if (pphy != 0) {
		CDBG("%s: rel: vaddr %lx, paddr %lx\n",
			__func__,
			pb->buffer, pphy);
		cfgcmd.cmd_type = CMD_FRAME_BUF_RELEASE;
		cfgcmd.value    = (void *)pb;
		if (sync->vfefn.vfe_config)
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
	} else {
		pr_err("[CAM]%s: msm_pmem_frame_vtop_lookup failed\n",
			__func__);
		rc = -EINVAL;
	}

	return rc;
}

static int msm_put_frame_buffer(struct msm_sync *sync, void __user *arg)
{
	struct msm_frame buf_t;

	if (copy_from_user(&buf_t,
				arg,
				sizeof(struct msm_frame))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_put_frame_buf(sync, &buf_t);
}

static int __msm_register_pmem(struct msm_sync *sync,
		struct msm_pmem_info *pinfo)
{
	int rc = 0;

	switch (pinfo->type) {
	case MSM_PMEM_VIDEO:
	case MSM_PMEM_PREVIEW:
	case MSM_PMEM_THUMBNAIL:
	case MSM_PMEM_MAINIMG:
	case MSM_PMEM_RAW_MAINIMG:
	case MSM_PMEM_VIDEO_VPE:
		rc = msm_pmem_table_add(&sync->pmem_frames, pinfo);
		break;

	case MSM_PMEM_AEC_AWB:
	case MSM_PMEM_AF:
	case MSM_PMEM_AEC:
	case MSM_PMEM_AWB:
	case MSM_PMEM_RS:
	case MSM_PMEM_CS:
	case MSM_PMEM_IHIST:
	case MSM_PMEM_SKIN:

		rc = msm_pmem_table_add(&sync->pmem_stats, pinfo);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_register_pmem(struct msm_sync *sync, void __user *arg)
{
	struct msm_pmem_info info;

	if (copy_from_user(&info, arg, sizeof(info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	return __msm_register_pmem(sync, &info);
}

static int msm_stats_axi_cfg(struct msm_sync *sync,
		struct msm_vfe_cfg_cmd *cfgcmd)
{
	int rc = -EIO;
	struct axidata axi_data;
	void *data = &axi_data;

	struct msm_pmem_region region[3];
	int pmem_type = MSM_PMEM_MAX;

	memset(&axi_data, 0, sizeof(axi_data));

	switch (cfgcmd->cmd_type) {
	case CMD_STATS_AXI_CFG:
		pmem_type = MSM_PMEM_AEC_AWB;
		break;
	case CMD_STATS_AF_AXI_CFG:
		pmem_type = MSM_PMEM_AF;
		break;
	case CMD_GENERAL:
		data = NULL;
		break;
	default:
		pr_err("[CAM]%s: unknown command type %d\n",
			__func__, cfgcmd->cmd_type);
		return -EINVAL;
	}

	if (cfgcmd->cmd_type != CMD_GENERAL) {
		axi_data.bufnum1 =
			msm_pmem_region_lookup(&sync->pmem_stats, pmem_type,
				&region[0], NUM_STAT_OUTPUT_BUFFERS);
		if (!axi_data.bufnum1) {
			pr_err("[CAM]%s %d: pmem region lookup error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		axi_data.region = &region[0];
	}

	/* send the AEC/AWB STATS configuration command to driver */
	if (sync->vfefn.vfe_config)
		rc = sync->vfefn.vfe_config(cfgcmd, &axi_data);

	return rc;
}

static int msm_put_stats_buffer(struct msm_sync *sync, void __user *arg)
{
	int rc = -EIO;

	struct msm_stats_buf buf;
	unsigned long pphy;
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&buf, arg,
				sizeof(struct msm_stats_buf))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	CDBG("%s\n", __func__);
	pphy = msm_pmem_stats_vtop_lookup(sync, buf.buffer, buf.fd);

	if (pphy != 0) {
		if (buf.type == STAT_AEAW)
			cfgcmd.cmd_type = CMD_STATS_BUF_RELEASE;
		else if (buf.type == STAT_AF)
			cfgcmd.cmd_type = CMD_STATS_AF_BUF_RELEASE;
		else if (buf.type == STAT_AEC)
			cfgcmd.cmd_type = CMD_STATS_AEC_BUF_RELEASE;
		else if (buf.type == STAT_AWB)
			cfgcmd.cmd_type = CMD_STATS_AWB_BUF_RELEASE;
		else if (buf.type == STAT_IHIST)
			cfgcmd.cmd_type = CMD_STATS_IHIST_BUF_RELEASE;
		else {
			pr_err("[CAM]%s: invalid buf type %d\n",
				__func__,
				buf.type);
			rc = -EINVAL;
			goto put_done;
		}

		cfgcmd.value = (void *)&buf;

		if (sync->vfefn.vfe_config) {
			rc = sync->vfefn.vfe_config(&cfgcmd, &pphy);
			if (rc < 0)
				pr_err("[CAM]%s: vfe_config error %d\n",
					__func__, rc);
		} else
			pr_err("[CAM]%s: vfe_config is NULL\n", __func__);
	} else {
		pr_err("[CAM]%s: NULL physical address\n", __func__);
		rc = -EINVAL;
	}

put_done:
	return rc;
}

static int msm_axi_config(struct msm_sync *sync, void __user *arg)
{
	struct msm_vfe_cfg_cmd cfgcmd;

	if (copy_from_user(&cfgcmd, arg, sizeof(cfgcmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	switch (cfgcmd.cmd_type) {
	case CMD_AXI_CFG_VIDEO:
	case CMD_AXI_CFG_PREVIEW:
	case CMD_AXI_CFG_SNAP:
	case CMD_RAW_PICT_AXI_CFG:
		return msm_frame_axi_cfg(sync, &cfgcmd);
        case CMD_AXI_CFG_VPE:
                return msm_vpe_frame_cfg(sync, (void *)&cfgcmd);

	case CMD_STATS_AXI_CFG:
	case CMD_STATS_AF_AXI_CFG:
		return msm_stats_axi_cfg(sync, &cfgcmd);

	default:
		pr_err("[CAM]%s: unknown command type %d\n",
			__func__,
			cfgcmd.cmd_type);
		return -EINVAL;
	}

	return 0;
}

static int __msm_get_pic(struct msm_sync *sync, struct msm_ctrl_cmd *ctrl)
{
	int rc = 0;
	int tm;

	struct msm_queue_cmd *qcmd = NULL;

	tm = (int)ctrl->timeout_ms;

	rc = wait_event_interruptible_timeout(
			sync->pict_q.wait,
			!list_empty_careful(
				&sync->pict_q.list) || sync->get_pic_abort,
			msecs_to_jiffies(tm));

	if (sync->get_pic_abort == 1) {
		sync->get_pic_abort = 0;
		return -ENODATA;
	}
	if (list_empty_careful(&sync->pict_q.list)) {
		if (rc == 0)
			return -ETIMEDOUT;
		if (rc < 0) {
			pr_err("[CAM]%s: rc %d\n", __func__, rc);
			return rc;
		}
	}

	rc = 0;

	qcmd = msm_dequeue(&sync->pict_q, list_pict);
	BUG_ON(!qcmd);

	/* HTC: check qcmd */
    if (!qcmd) {
      rc = -EFAULT;
      pr_err("[CAM]%s: qcmd is NULL, rc %d\n", __func__, rc);
	  return rc;
    }

	if (qcmd->command != NULL) {
		struct msm_ctrl_cmd *q =
			(struct msm_ctrl_cmd *)qcmd->command;
		ctrl->type = q->type;
		ctrl->status = q->status;
	} else {
		ctrl->type = -1;
		ctrl->status = -1;
	}

	free_qcmd(qcmd);

	return rc;
}

static int msm_get_pic(struct msm_sync *sync, void __user *arg)
{
	struct msm_ctrl_cmd ctrlcmd_t;
	struct msm_pmem_region pic_pmem_region;
	int rc;
	unsigned long end;
	int cline_mask;

	if (copy_from_user(&ctrlcmd_t,
				arg,
				sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	rc = __msm_get_pic(sync, &ctrlcmd_t);
	if (rc < 0)
		return rc;

	if (sync->croplen) {
		if (ctrlcmd_t.length != sync->croplen) {
			pr_err("[CAM]%s: invalid len %d < %d\n",
				__func__,
				ctrlcmd_t.length,
				sync->croplen);
			return -EINVAL;
		}
		if (copy_to_user(ctrlcmd_t.value,
				sync->cropinfo,
				sync->croplen)) {
			ERR_COPY_TO_USER();
			return -EFAULT;
		}
	}

	/* HTC */
	if (msm_pmem_region_lookup(&sync->pmem_frames,
			MSM_PMEM_MAINIMG,
			&pic_pmem_region, 1) == 0) {
		pr_err("[CAM]%s pmem region lookup error\n", __func__);
		pr_info("[CAM]%s probably getting RAW\n", __func__);
		if (msm_pmem_region_lookup(&sync->pmem_frames,
				MSM_PMEM_RAW_MAINIMG,
				&pic_pmem_region, 1) == 0) {
			pr_err("[CAM]%s RAW pmem region lookup error\n", __func__);
			return -EIO;
		}
	}

	cline_mask = cache_line_size() - 1;
	end = pic_pmem_region.kvaddr + pic_pmem_region.len;
	end = (end + cline_mask) & ~cline_mask;

	pr_info("[CAM]%s: flushing cache for [%08lx, %08lx)\n",
		__func__,
		pic_pmem_region.kvaddr, end);

	dmac_inv_range((const void *)pic_pmem_region.kvaddr,
			(const void *)end);

	/* HTC end */

	CDBG("%s: copy snapshot frame to user\n", __func__);
	if (copy_to_user((void *)arg,
		&ctrlcmd_t,
		sizeof(struct msm_ctrl_cmd))) {
		ERR_COPY_TO_USER();
		return -EFAULT;
	}
	return 0;
}

static int msm_set_crop(struct msm_sync *sync, void __user *arg)
{
	struct crop_info crop;

	if (copy_from_user(&crop,
				arg,
				sizeof(struct crop_info))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	if (!sync->croplen) {
		sync->cropinfo = kzalloc(crop.len, GFP_KERNEL);
		if (!sync->cropinfo)
			return -ENOMEM;
	} else if (sync->croplen < crop.len)
		return -EINVAL;

	if (copy_from_user(sync->cropinfo,
				crop.info,
				crop.len)) {
		ERR_COPY_FROM_USER();
		kfree(sync->cropinfo);
		return -EFAULT;
	}

	sync->croplen = crop.len;

	return 0;
}

static int msm_pp_grab(struct msm_sync *sync, void __user *arg)
{
	uint32_t enable;
	if (copy_from_user(&enable, arg, sizeof(enable))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
		enable &= PP_MASK;
		if (enable & (enable - 1)) {
			pr_err("[CAM]%s: error: more than one PP request!\n",
				__func__);
			return -EINVAL;
		}
		if (sync->pp_mask) {
			pr_err("[CAM]%s: postproc %x is already enabled\n",
				__func__, sync->pp_mask & enable);
			return -EINVAL;
		}

		CDBG("%s: sync->pp_mask %d enable %d\n", __func__,
			sync->pp_mask, enable);
		sync->pp_mask |= enable;
	}

	return 0;
}

static int msm_pp_release(struct msm_sync *sync, void __user *arg)
{
	uint32_t mask;
	if (copy_from_user(&mask, arg, sizeof(uint32_t))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	mask &= PP_MASK;
	if (!sync->pp_mask) {
		pr_warning("[CAM]%s: pp not in progress for %x\n", __func__,
			mask);
		return -EINVAL;
	}
	if (sync->pp_mask & PP_PREV) {

		if (mask & PP_PREV) {
			mutex_lock(&pp_prev_lock);
			if (!sync->pp_prev) {
				pr_err("[CAM]%s: no preview frame to deliver!\n",
					__func__);
				mutex_unlock(&pp_prev_lock);
				return -EINVAL;
			}
			pr_info("[CAM]%s: delivering pp_prev\n", __func__);

			msm_enqueue(&sync->frame_q, &sync->pp_prev->list_frame);
			sync->pp_prev = NULL;
			mutex_unlock(&pp_prev_lock);
		} else if (!(mask & PP_PREV)) {
			sync->pp_mask &= ~PP_PREV;
		}
		goto done;
	}

	if (((mask & PP_SNAP) && (sync->pp_mask & PP_SNAP)) ||
		((mask & PP_RAW_SNAP) && (sync->pp_mask & PP_RAW_SNAP))) {
		mutex_lock(&pp_snap_lock);
		if (!sync->pp_snap) {
			pr_err("[CAM]%s: no snapshot to deliver!\n", __func__);
			mutex_unlock(&pp_snap_lock);
			return -EINVAL;
		}
		pr_info("[CAM]%s: delivering pp_snap\n", __func__);
		msm_enqueue(&sync->pict_q, &sync->pp_snap->list_pict);
		sync->pp_snap = NULL;
		mutex_unlock(&pp_snap_lock);
		sync->pp_mask &=
			(mask & PP_SNAP) ? ~PP_SNAP : ~PP_RAW_SNAP;
	}

done:
	return 0;
}

static long msm_ioctl_common(struct msm_cam_device *pmsm,
		unsigned int cmd,
		void __user *argp)
{
	CDBG("%s\n", __func__);
	switch (cmd) {
	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(pmsm->sync, argp);
	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(pmsm->sync, argp);
	default:
		return -EINVAL;
	}
}

int msm_camera_flash(struct msm_sync *sync, int level)
{
	int flash_level;
	static int high_enabled;

	if (!sync->sdata->flash_cfg) {
		pr_err("[CAM]%s: camera flash is not supported.\n", __func__);
		return -EINVAL;
	}

	if (!sync->sdata->flash_cfg->num_flash_levels) {
		pr_err("[CAM]%s: no flash levels.\n", __func__);
		return -EINVAL;
	}

	switch (level) {
	case MSM_CAMERA_LED_HIGH:
		/*flash_level = sync->sdata->flash_cfg->num_flash_levels - 1;*/
		flash_level = FL_MODE_FLASH;
		sync->sdata->led_high_enabled = 0; /* reset led high*/
#ifdef CONFIG_FLASH_BACKLIGHT_OFF
		led_brightness_switch("lcd-backlight", LED_OFF);
		pr_info("sleep 40ms for turn off backlight: E\n");
		msleep(40);
		pr_info("sleep 40ms for turn off backlight: X\n");
#endif
		high_enabled = 1;
		break;
	case MSM_CAMERA_LED_LOW:
		/*flash_level = sync->sdata->flash_cfg->num_flash_levels / 2;*/
		flash_level = FL_MODE_PRE_FLASH;
		sync->sdata->led_high_enabled = 1; /* set led high*/
		high_enabled = 0;
		break;
	case MSM_CAMERA_LED_OFF:
		flash_level = 0;
#ifdef CONFIG_FLASH_BACKLIGHT_OFF
		{
		int rc = sync->sdata->flash_cfg->camera_flash(flash_level);
		if (high_enabled == 1) {
			led_brightness_switch("lcd-backlight", LED_FULL);
		}
		high_enabled = 0;

		return rc;
		}
#else
		break;
#endif
	default:
		pr_err("[CAM]%s: invalid flash level %d.\n", __func__, level);
		return -EINVAL;
	}

	return sync->sdata->flash_cfg->camera_flash(flash_level);
}

static long msm_ioctl_config(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_cam_device *pmsm = filep->private_data;

	CDBG("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_CONFIG_VFE:
		/* Coming from config thread for update */
		rc = msm_config_vfe(pmsm->sync, argp);
		break;

        case MSM_CAM_IOCTL_CONFIG_VPE:
                /* Coming from config thread for update */
                rc = msm_config_vpe(pmsm->sync, argp);
                break;

	case MSM_CAM_IOCTL_GET_STATS:
		/* Coming from config thread wait
		 * for vfe statistics and control requests */
		rc = msm_get_stats(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_ENABLE_VFE:
		/* This request comes from control thread:
		 * enable either QCAMTASK or VFETASK */
		rc = msm_enable_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_DISABLE_VFE:
		/* This request comes from control thread:
		 * disable either QCAMTASK or VFETASK */
		rc = msm_disable_vfe(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_VFE_APPS_RESET:
		msm_camio_vfe_blk_reset();
		rc = 0;
		break;

	case MSM_CAM_IOCTL_RELEASE_STATS_BUFFER:
		rc = msm_put_stats_buffer(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_AXI_CONFIG:
	case MSM_CAM_IOCTL_AXI_VPE_CONFIG:
		rc = msm_axi_config(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SET_CROP:
		rc = msm_set_crop(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_PICT_PP:
		/* Grab one preview frame or one snapshot
		 * frame.
		 */
		rc = msm_pp_grab(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_PICT_PP_DONE:
		/* Release the preview of snapshot frame
		 * that was grabbed.
		 */
		rc = msm_pp_release(pmsm->sync, argp);
		break;

	case MSM_CAM_IOCTL_SENSOR_IO_CFG:
		rc = pmsm->sync->sctrl.s_config(argp);
		break;

	case MSM_CAM_IOCTL_FLASH_LED_CFG: {
		uint32_t led_state;
		if (copy_from_user(&led_state, argp, sizeof(led_state))) {
			ERR_COPY_FROM_USER();
			rc = -EFAULT;
		} else{
			rc = msm_camera_flash(pmsm->sync, led_state);
		}
		break;
	}

	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	CDBG("%s: cmd %d DONE\n", __func__, _IOC_NR(cmd));
	return rc;
}

static int msm_unblock_poll_frame(struct msm_sync *);

static long msm_ioctl_frame(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_cam_device *pmsm = filep->private_data;


	switch (cmd) {
	case MSM_CAM_IOCTL_GETFRAME:
		/* Coming from frame thread to get frame
		 * after SELECT is done */
		rc = msm_get_frame(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER:
		rc = msm_put_frame_buffer(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME:
		rc = msm_unblock_poll_frame(pmsm->sync);
		break;
	default:
		break;
	}

	return rc;
}


static long msm_ioctl_control(struct file *filep, unsigned int cmd,
	unsigned long arg)
{
	int rc = -EINVAL;
	void __user *argp = (void __user *)arg;
	struct msm_control_device *ctrl_pmsm = filep->private_data;
	struct msm_cam_device *pmsm = ctrl_pmsm->pmsm;

	switch (cmd) {
	case MSM_CAM_IOCTL_CTRL_COMMAND:
		/* Coming from control thread, may need to wait for
		 * command status */
		mutex_lock(&ctrl_cmd_lock);
		rc = msm_control(ctrl_pmsm, 1, argp);
		mutex_unlock(&ctrl_cmd_lock);
		break;
	case MSM_CAM_IOCTL_CTRL_COMMAND_2:
		/* Sends a message, returns immediately */
		rc = msm_control(ctrl_pmsm, 0, argp);
		break;
	case MSM_CAM_IOCTL_CTRL_CMD_DONE:
		/* Config thread calls the control thread to notify it
		 * of the result of a MSM_CAM_IOCTL_CTRL_COMMAND.
		 */
		rc = msm_ctrl_cmd_done(ctrl_pmsm, argp);
		break;
	case MSM_CAM_IOCTL_GET_PICTURE:
		rc = msm_get_pic(pmsm->sync, argp);
		break;
	case MSM_CAM_IOCTL_GET_SENSOR_INFO:
		rc = msm_get_sensor_info(pmsm->sync, argp);
		break;
	default:
		rc = msm_ioctl_common(pmsm, cmd, argp);
		break;
	}

	return rc;
}


static void msm_show_time(void){
	struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("[CAM]>>>>>>>>(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)<<<<<<<\n",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	return;
}


static int __msm_release(struct msm_sync *sync)
{
	struct msm_pmem_region *region;
	struct hlist_node *hnode;
	struct hlist_node *n;
       pr_info("[CAM]%s:sync->opencnt:%d \n", __func__, sync->opencnt);
	mutex_lock(&sync->lock);
	if (sync->opencnt)
		sync->opencnt--;

	if (!sync->opencnt) {

		msm_show_time();

		/* need to clean up system resource */
		if (sync->vfefn.vfe_release)
			sync->vfefn.vfe_release(sync->pdev);

		kfree(sync->cropinfo);
		sync->cropinfo = NULL;
		sync->croplen = 0;

		hlist_for_each_entry_safe(region, hnode, n,
				&sync->pmem_frames, list) {
			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}

		hlist_for_each_entry_safe(region, hnode, n,
				&sync->pmem_stats, list) {
			hlist_del(hnode);
			put_pmem_file(region->file);
			kfree(region);
		}
		msm_queue_drain(&sync->pict_q, list_pict);

		/* HTC */
		wake_unlock(&sync->wake_suspend_lock);
		wake_unlock(&sync->wake_lock);

		sync->apps_id = NULL;
		CDBG("%s: completed\n", __func__);
	}
	mutex_unlock(&sync->lock);

	return 0;
}

static int msm_release_config(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_cam_device *pmsm = filep->private_data;
	pr_info("[CAM]%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		pr_info("[CAM]release config atomic_set pmsm->opened as 0\n");
		if ((&pmsm->sync->event_q) != 0) {
			msm_queue_drain(&pmsm->sync->event_q, list_config);
			atomic_set(&pmsm->opened, 0);
		} else {
			printk("[CAM]%s  fatal error : &pmsm->sync->event_q is 0 !!!\n", __func__);
		}
	}
	return rc;
}

static int msm_release_control(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_control_device *ctrl_pmsm = filep->private_data;
	struct msm_cam_device *pmsm = ctrl_pmsm->pmsm;
	pr_info("[CAM]%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
	g_v4l2_opencnt--;
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		msm_queue_drain(&ctrl_pmsm->ctrl_q, list_control);
		kfree(ctrl_pmsm);
	}
	return rc;
}

static int msm_release_frame(struct inode *node, struct file *filep)
{
	int rc;
	struct msm_cam_device *pmsm = filep->private_data;
	pr_info("[CAM]%s: %s\n", __func__, filep->f_path.dentry->d_name.name);
	rc = __msm_release(pmsm->sync);
	if (!rc) {
		pr_info("[CAM]release frame atomic_set pmsm->opened as 0\n");
		msm_queue_drain(&pmsm->sync->frame_q, list_frame);
		atomic_set(&pmsm->opened, 0);
	}
	return rc;
}

static int msm_unblock_poll_frame(struct msm_sync *sync)
{
	unsigned long flags;
	CDBG("%s\n", __func__);
	spin_lock_irqsave(&sync->frame_q.lock, flags);
	sync->unblock_poll_frame = 1;
	wake_up(&sync->frame_q.wait);
	spin_unlock_irqrestore(&sync->frame_q.lock, flags);
	return 0;
}

static unsigned int __msm_poll_frame(struct msm_sync *sync,
		struct file *filep,
		struct poll_table_struct *pll_table)
{
	int rc = 0;
	unsigned long flags;

	poll_wait(filep, &sync->frame_q.wait, pll_table);

	spin_lock_irqsave(&sync->frame_q.lock, flags);
	if (!list_empty_careful(&sync->frame_q.list))
		/* frame ready */
		rc = POLLIN | POLLRDNORM;
	if (sync->unblock_poll_frame) {
		CDBG("%s: sync->unblock_poll_frame is true\n", __func__);
		rc |= POLLPRI;
		sync->unblock_poll_frame = 0;
	}
	spin_unlock_irqrestore(&sync->frame_q.lock, flags);

	return rc;
}

static unsigned int msm_poll_frame(struct file *filep,
	struct poll_table_struct *pll_table)
{
	struct msm_cam_device *pmsm = filep->private_data;
	return __msm_poll_frame(pmsm->sync, filep, pll_table);
}

/*
 * This function executes in interrupt context.
 */

static void *msm_vfe_sync_alloc(int size,
			void *syncdata __attribute__((unused)),
			gfp_t gfp)
{
	struct msm_queue_cmd *qcmd =
		kzalloc(sizeof(struct msm_queue_cmd) + size, gfp);
	if (qcmd) {
		/* Becker and kant */
		memset(qcmd, 0x0, sizeof(struct msm_queue_cmd) + size);

		atomic_set(&qcmd->on_heap, 1);
		return qcmd + 1;
	}
	return NULL;
}

static void *msm_vpe_sync_alloc(int size,
                       void *syncdata __attribute__((unused)),
                       gfp_t gfp)
{
       struct msm_queue_cmd *qcmd =
               kmalloc(sizeof(struct msm_queue_cmd) + size, gfp);
       if (qcmd) {
		atomic_set(&qcmd->on_heap, 1);
               return qcmd + 1;
       }
       return NULL;
}

static void msm_vfe_sync_free(void *ptr)
{
	if (ptr) {
		struct msm_queue_cmd *qcmd =
			(struct msm_queue_cmd *)ptr;
		qcmd--;
		if (atomic_read(&qcmd->on_heap))
			kfree(qcmd);
	}
}

static void msm_vpe_sync_free(void *ptr)
{
       if (ptr) {
               struct msm_queue_cmd *qcmd =
                       (struct msm_queue_cmd *)ptr;
               qcmd--;
		if (atomic_read(&qcmd->on_heap))
                       kfree(qcmd);
       }
}

/*
 * This function may execute in interrupt context.
 */

static void msm_vfe_sync(struct msm_vfe_resp *vdata,
		enum msm_queue qtype, void *syncdata,
		gfp_t gfp)
{
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_sync *sync = (struct msm_sync *)syncdata;

	if (!sync) {
		pr_err("[CAM]%s: no context in dsp callback.\n", __func__);
		return;
	}
    /* HTC */
	if (!sync->opencnt) {
		pr_err("[CAM]%s: SPURIOUS INTERRUPT\n", __func__);
		return;
	}

	qcmd = ((struct msm_queue_cmd *)vdata) - 1;
	qcmd->type = qtype;
	qcmd->command = vdata;

	ktime_get_ts(&(qcmd->ts));

	if (qtype != MSM_CAM_Q_VFE_MSG)
		goto vfe_for_config;

	CDBG("%s: vdata->type %d\n", __func__, vdata->type);
        switch (vdata->type) {
        case VFE_MSG_OUTPUT_P:
		if (sync->pp_mask & PP_PREV) {
			CDBG("%s: PP_PREV in progress: phy_y %x phy_cbcr %x\n",
				__func__,
				vdata->phy.y_phy,
				vdata->phy.cbcr_phy);
			mutex_lock(&pp_prev_lock);
			if (sync->pp_prev)
				pr_warning("[CAM]%s: overwriting pp_prev!\n",
					__func__);
			pr_info("[CAM]%s: sending preview to config\n", __func__);
			sync->pp_prev = qcmd;
			mutex_unlock(&pp_prev_lock);
			break;
		}
		CDBG("%s: msm_enqueue frame_q\n", __func__);
		msm_enqueue(&sync->frame_q, &qcmd->list_frame);
			if (atomic_read(&qcmd->on_heap))
				atomic_add(1, &qcmd->on_heap);
		break;

        case VFE_MSG_OUTPUT_V:
                //printk(KERN_ERR "dis_en = %d \n", *sync->vpefn.dis);
                if (*(sync->vpefn.dis)) {
                        if (sync->cropinfo != NULL)
                                vdata->vpe_bf.vpe_crop =
                                *(struct video_crop_t *)(sync->cropinfo);
                        memset(&(vdata->vpe_bf), 0, sizeof(vdata->vpe_bf));
                        vdata->vpe_bf.y_phy = vdata->phy.y_phy;
                        vdata->vpe_bf.cbcr_phy = vdata->phy.cbcr_phy;
                        vdata->vpe_bf.ts = (qcmd->ts);
                        vdata->vpe_bf.frame_id = vdata->phy.frame_id;
                        qcmd->command = vdata;
                        msm_enqueue_vpe(&sync->vpe_q, &qcmd->list_vpe_frame);
                        return;
                } else {
                        if (sync->vpefn.vpe_cfg_update(sync->cropinfo)) {
                                CDBG("%s: msm_enqueue video frame to vpe "
                                        "time = %ld\n",
                                        __func__, qcmd->ts.tv_nsec);
                                sync->vpefn.send_frame_to_vpe(
                                        vdata->phy.y_phy,
                                        vdata->phy.cbcr_phy,
                                        &(qcmd->ts));
                                kfree(qcmd);
                                return;
                        } else {
                                CDBG("%s: msm_enqueue video frame_q\n",
                                        __func__);
                                msm_enqueue(&sync->frame_q,
                                        &qcmd->list_frame);
			if (atomic_read(&qcmd->on_heap))
				atomic_add(1, &qcmd->on_heap);
                                break;
                         }
                }

	case VFE_MSG_SNAPSHOT:
		if (sync->pp_mask & (PP_SNAP | PP_RAW_SNAP)) {
			CDBG("%s: PP_SNAP in progress: pp_mask %x\n",
				__func__, sync->pp_mask);
			mutex_lock(&pp_snap_lock);
			if (sync->pp_snap)
				pr_warning("[CAM]%s: overwriting pp_snap!\n",
					__func__);
			pr_info("[CAM]%s: sending snapshot to config\n",
				__func__);
			sync->pp_snap = qcmd;
			mutex_unlock(&pp_snap_lock);
				break;
			}

		msm_enqueue(&sync->pict_q, &qcmd->list_pict);
			if (atomic_read(&qcmd->on_heap))
				atomic_add(1, &qcmd->on_heap);
		break;

	case VFE_MSG_STATS_AWB:
		CDBG("%s: qtype %d, AWB stats, enqueue event_q.\n",
		     __func__, vdata->type);
		break;

		case VFE_MSG_STATS_AEC:
		CDBG("%s: qtype %d, AEC stats, enqueue event_q.\n",
		     __func__, vdata->type);
		break;

		case VFE_MSG_STATS_IHIST:
		CDBG("%s: qtype %d, ihist stats, enqueue event_q.\n",
		     __func__, vdata->type);
		break;
		case VFE_MSG_GENERAL:
		CDBG("%s: qtype %d, general msg, enqueue event_q.\n",
		    __func__, vdata->type);
		break;

		default:
		CDBG("%s: qtype %d not handled\n", __func__, vdata->type);
		/* fall through, send to config. */
	}

vfe_for_config:
	CDBG("%s: msm_enqueue event_q\n", __func__);
	msm_enqueue(&sync->event_q, &qcmd->list_config);
}

static void msm_vpe_sync(struct msm_vpe_resp *vdata,
                                               enum msm_queue qtype,
                                               void *syncdata,
                                               void *ts, gfp_t gfp)
{
       struct msm_queue_cmd *qcmd = NULL;
       struct msm_sync *sync = (struct msm_sync *)syncdata;
       if (!sync) {
               pr_err("[CAM]%s: no context in dsp callback.\n", __func__);
               return;
       }

       qcmd = ((struct msm_queue_cmd *)vdata) - 1;
       qcmd->type = qtype;
       qcmd->command = vdata;
       qcmd->ts = *((struct timespec *)ts);

       if (qtype != MSM_CAM_Q_VPE_MSG)
               goto vpe_for_config;

       CDBG("%s: vdata->type %d\n", __func__, vdata->type);
       switch (vdata->type) {
       case VPE_MSG_OUTPUT_V:
               CDBG("%s: msm_enqueue video frame_q from VPE \n", __func__);
               msm_enqueue(&sync->frame_q, &qcmd->list_frame);
			if (atomic_read(&qcmd->on_heap))
				atomic_add(1, &qcmd->on_heap);
               break;
       default:
               CDBG("%s: qtype %d not handled\n", __func__, vdata->type);
               /* fall through, send to config. */
       }
vpe_for_config:
       CDBG("%s: msm_enqueue event_q\n", __func__);
       msm_enqueue(&sync->event_q, &qcmd->list_config);
}

static struct msm_vpe_callback msm_vpe_s = {
       .vpe_resp = msm_vpe_sync,
       .vpe_alloc = msm_vpe_sync_alloc,
       .vpe_free = msm_vpe_sync_free,
};

static struct msm_vfe_callback msm_vfe_s = {
	.vfe_resp = msm_vfe_sync,
	.vfe_alloc = msm_vfe_sync_alloc,
	.vfe_free = msm_vfe_sync_free,
};

static int __msm_open(struct msm_sync *sync, const char *const apps_id)
{
	int rc = 0;

	mutex_lock(&sync->lock);
	if (sync->apps_id && strcmp(sync->apps_id, apps_id)
				&& (!strcmp(MSM_APPS_ID_V4L2, apps_id))) {
		pr_err("[CAM]%s(%s): sensor %s is already opened for %s\n",
			__func__,
			apps_id,
			sync->sdata->sensor_name,
			sync->apps_id);
		rc = -EBUSY;
		goto msm_open_done;
	}
	sync->apps_id = apps_id;

	if (!sync->opencnt) {
		/* HTC */
		wake_lock(&sync->wake_suspend_lock);
		wake_lock(&sync->wake_lock);

		msm_camvfe_fn_init(&sync->vfefn, sync);
		if (sync->vfefn.vfe_init) {
			sync->get_pic_abort = 0;
			rc = sync->vfefn.vfe_init(&msm_vfe_s,
				sync->pdev);
			if (rc < 0) {
				pr_err("[CAM]%s: vfe_init failed at %d\n",
					__func__, rc);
				goto msm_open_done;
			}
			rc = sync->sctrl.s_init(sync->sdata);
			if (rc < 0) {
				pr_err("[CAM]%s: sensor init failed: %d\n",
					__func__, rc);
				/* HTC */
				sync->vfefn.vfe_release(sync->pdev);
				goto msm_open_done;
			}
		} else {
			pr_err("[CAM]%s: no sensor init func\n", __func__);
			rc = -ENODEV;
			goto msm_open_done;
		}
		msm_camvpe_fn_init(&sync->vpefn, sync);

		if (rc >= 0) {
			INIT_HLIST_HEAD(&sync->pmem_frames);
			INIT_HLIST_HEAD(&sync->pmem_stats);
                        if (sync->vpefn.vpe_reg)
                                sync->vpefn.vpe_reg(&msm_vpe_s);
			sync->unblock_poll_frame = 0;
		}
	}
	sync->opencnt++;

msm_open_done:
	mutex_unlock(&sync->lock);
	return rc;
}

static int msm_open_common(struct inode *inode, struct file *filep,
			   int once)
{
	int rc;
	struct msm_cam_device *pmsm =
		container_of(inode->i_cdev, struct msm_cam_device, cdev);

	pr_info("[CAM]%s: open %s\n", __func__, filep->f_path.dentry->d_name.name);

	if (atomic_cmpxchg(&pmsm->opened, 0, 1) && once) {
		pr_err("[CAM]%s: %s is already opened.\n",
			__func__,
			filep->f_path.dentry->d_name.name);
		return -EBUSY;
	}

	rc = nonseekable_open(inode, filep);
	if (rc < 0) {
		pr_err("[CAM]%s: nonseekable_open error %d\n", __func__, rc);
		return rc;
	}

	rc = __msm_open(pmsm->sync, MSM_APPS_ID_PROP);
	if (rc < 0)
		return rc;

	filep->private_data = pmsm;

	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static int msm_open(struct inode *inode, struct file *filep)
{
	msm_show_time();
	return msm_open_common(inode, filep, 1);
}

static int msm_open_control(struct inode *inode, struct file *filep)
{
	int rc;

	struct msm_control_device *ctrl_pmsm =
		kzalloc(sizeof(struct msm_control_device), GFP_KERNEL);
	if (!ctrl_pmsm)
		return -ENOMEM;

	rc = msm_open_common(inode, filep, 0);
	if (rc < 0) {
		kfree(ctrl_pmsm);
		return rc;
	}

	ctrl_pmsm->pmsm = filep->private_data;
	filep->private_data = ctrl_pmsm;

	msm_queue_init(&ctrl_pmsm->ctrl_q, "control");

	if (!g_v4l2_opencnt)
		g_v4l2_control_device = ctrl_pmsm;

	g_v4l2_opencnt++;

	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static int __msm_v4l2_control(struct msm_sync *sync,
		struct msm_ctrl_cmd *out)
{
	int rc = 0;

	struct msm_queue_cmd *qcmd = NULL, *rcmd = NULL;
	struct msm_ctrl_cmd *ctrl;
	struct msm_device_queue *v4l2_ctrl_q = &g_v4l2_control_device->ctrl_q;

	/* wake up config thread, 4 is for V4L2 application */
	qcmd = kzalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	if (!qcmd) {
		pr_err("[CAM]%s: cannot allocate buffer\n", __func__);
		rc = -ENOMEM;
		goto end;
	}
	qcmd->type = MSM_CAM_Q_V4L2_REQ;
	qcmd->command = out;
	atomic_set(&qcmd->on_heap, 1);

	if (out->type == V4L2_CAMERA_EXIT) {
		rcmd = __msm_control(sync, NULL, qcmd, out->timeout_ms);
		if (rcmd == NULL) {
			rc = PTR_ERR(rcmd);
			goto end;
		}
	}

	rcmd = __msm_control(sync, v4l2_ctrl_q, qcmd, out->timeout_ms);
	if (IS_ERR(rcmd)) {
		rc = PTR_ERR(rcmd);
		goto end;
	}

	ctrl = (struct msm_ctrl_cmd *)(rcmd->command);
	/* FIXME: we should just set out->length = ctrl->length; */
	BUG_ON(out->length < ctrl->length);
	memcpy(out->value, ctrl->value, ctrl->length);

end:
	free_qcmd(rcmd);
	CDBG("%s: rc %d\n", __func__, rc);
	return rc;
}

static const struct file_operations msm_fops_config = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl_config,
	.release = msm_release_config,
};

static const struct file_operations msm_fops_control = {
	.owner = THIS_MODULE,
	.open = msm_open_control,
	.unlocked_ioctl = msm_ioctl_control,
	.release = msm_release_control,
};

static const struct file_operations msm_fops_frame = {
	.owner = THIS_MODULE,
	.open = msm_open,
	.unlocked_ioctl = msm_ioctl_frame,
	.release = msm_release_frame,
	.poll = msm_poll_frame,
};

static int msm_setup_cdev(struct msm_cam_device *msm,
			int node,
			dev_t devno,
			const char *suffix,
			const struct file_operations *fops)
{
	int rc = -ENODEV;

	struct device *device =
		device_create(msm_class, NULL,
			devno, NULL,
			"%s%d", suffix, node);

	if (IS_ERR(device)) {
		rc = PTR_ERR(device);
		pr_err("[CAM]%s: error creating device: %d\n", __func__, rc);
		return rc;
	}

	cdev_init(&msm->cdev, fops);
	msm->cdev.owner = THIS_MODULE;

	rc = cdev_add(&msm->cdev, devno, 1);
	if (rc < 0) {
		pr_err("[CAM]%s: error adding cdev: %d\n", __func__, rc);
		device_destroy(msm_class, devno);
		return rc;
	}

	return rc;
}

static int msm_tear_down_cdev(struct msm_cam_device *msm, dev_t devno)
{
	cdev_del(&msm->cdev);
	device_destroy(msm_class, devno);
	return 0;
}

static uint32_t led_ril_status_value;
static uint32_t led_wimax_status_value;
static uint32_t led_hotspot_status_value;
static uint16_t led_low_temp_limit;
static uint16_t led_low_cap_limit;
static struct kobject *led_status_obj;

static ssize_t led_ril_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_ril_status_value);
	return length;
}

static ssize_t led_ril_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_ril_status_value = tmp;
	pr_info("[CAM]led_ril_status_value = %d\n", led_ril_status_value);
	return count;
}

static ssize_t led_wimax_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_wimax_status_value);
	return length;
}

static ssize_t led_wimax_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_wimax_status_value = tmp;
	pr_info("[CAM]led_wimax_status_value = %d\n", led_wimax_status_value);
	return count;
}

static ssize_t led_hotspot_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_hotspot_status_value);
	return length;
}

static ssize_t led_hotspot_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

	led_hotspot_status_value = tmp;
	pr_info("[CAM]led_hotspot_status_value = %d\n", led_hotspot_status_value);
	return count;
}

static ssize_t low_temp_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_temp_limit);
	return length;
}

static ssize_t low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit);
	return length;
}

static DEVICE_ATTR(led_ril_status, 0644,
	led_ril_status_get,
	led_ril_status_set);

static DEVICE_ATTR(led_wimax_status, 0644,
	led_wimax_status_get,
	led_wimax_status_set);

static DEVICE_ATTR(led_hotspot_status, 0644,
	led_hotspot_status_get,
	led_hotspot_status_set);

static DEVICE_ATTR(low_temp_limit, 0444,
	low_temp_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit, 0444,
	low_cap_limit_get,
	NULL);

static int msm_camera_sysfs_init(struct msm_sync *sync)
{
	int ret = 0;
	CDBG("msm_camera:kobject creat and add\n");
	led_status_obj = kobject_create_and_add("camera_led_status", NULL);
	if (led_status_obj == NULL) {
		pr_info("[CAM]msm_camera: subsystem_register failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_ril_status.attr);
	if (ret) {
		pr_info("[CAM]msm_camera: sysfs_create_file ril failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_wimax_status.attr);
	if (ret) {
		pr_info("[CAM]msm_camera: sysfs_create_file wimax failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_hotspot_status.attr);
	if (ret) {
		pr_info("[CAM]msm_camera: sysfs_create_file hotspot failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_temp_limit.attr);
	if (ret) {
		pr_info("[CAM]msm_camera:sysfs_create_file low_temp_limit failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit.attr);
	if (ret) {
		pr_info("[CAM]msm_camera: sysfs_create_file low_cap_limit failed\n");
		ret = -EFAULT;
		goto error;
	}

	led_low_temp_limit = sync->sdata->flash_cfg->low_temp_limit;
	led_low_cap_limit = sync->sdata->flash_cfg->low_cap_limit;

	return ret;
error:
	kobject_del(led_status_obj);
	return ret;
}


int msm_v4l2_register(struct msm_v4l2_driver *drv)
{
	/* FIXME: support multiple sensors */
	if (list_empty(&msm_sensors))
		return -ENODEV;

	drv->sync = list_first_entry(&msm_sensors, struct msm_sync, list);
	drv->open      = __msm_open;
	drv->release   = __msm_release;
	drv->ctrl      = __msm_v4l2_control;
	drv->reg_pmem  = __msm_register_pmem;
	drv->get_frame = __msm_get_frame;
	drv->put_frame = __msm_put_frame_buf;
	drv->get_pict  = __msm_get_pic;
	drv->drv_poll  = __msm_poll_frame;

	return 0;
}
EXPORT_SYMBOL(msm_v4l2_register);

int msm_v4l2_unregister(struct msm_v4l2_driver *drv)
{
	drv->sync = NULL;
	return 0;
}
EXPORT_SYMBOL(msm_v4l2_unregister);

static int msm_sync_init(struct msm_sync *sync,
		struct platform_device *pdev,
		int (*sensor_probe)(struct msm_camera_sensor_info *,
				struct msm_sensor_ctrl *), int camera_node)
{
	int rc = 0;
	struct msm_sensor_ctrl sctrl;
	sync->sdata = pdev->dev.platform_data;

	msm_queue_init(&sync->event_q, "event");
	msm_queue_init(&sync->frame_q, "frame");
	msm_queue_init(&sync->pict_q, "pict");
	msm_queue_init(&sync->vpe_q, "vpe");

	/* HTC */
	wake_lock_init(&sync->wake_suspend_lock, WAKE_LOCK_SUSPEND, "msm_camera_wake");
	wake_lock_init(&sync->wake_lock, WAKE_LOCK_IDLE, "msm_camera");

	rc = msm_camio_probe_on(pdev);
	if (rc < 0)
		return rc;
	sctrl.node = camera_node;
	pr_info("[CAM]sctrl.node %d\n", sctrl.node);
	rc = sensor_probe(sync->sdata, &sctrl);
	if (rc >= 0) {
		sync->pdev = pdev;
		sync->sctrl = sctrl;
	}
	msm_camio_probe_off(pdev);
	if (rc < 0) {
		pr_err("[CAM]%s: failed to initialize %s\n",
			__func__,
			sync->sdata->sensor_name);
		/* HTC */
		wake_lock_destroy(&sync->wake_suspend_lock);
		wake_lock_destroy(&sync->wake_lock);
		return rc;
	}

	sync->opencnt = 0;
	mutex_init(&sync->lock);
	CDBG("%s: initialized %s\n", __func__, sync->sdata->sensor_name);
	return rc;
}

static int msm_sync_destroy(struct msm_sync *sync)
{
	/* HTC */
	wake_lock_destroy(&sync->wake_suspend_lock);
	wake_lock_destroy(&sync->wake_lock);
	return 0;
}

static int msm_device_init(struct msm_cam_device *pmsm,
		struct msm_sync *sync,
		int node)
{
	int dev_num = 3 * node;
	int rc = msm_setup_cdev(pmsm, node,
		MKDEV(MAJOR(msm_devno), dev_num),
		"control", &msm_fops_control);
	if (rc < 0) {
		pr_err("[CAM]%s: error creating control node: %d\n", __func__, rc);
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 1, node,
		MKDEV(MAJOR(msm_devno), dev_num + 1),
		"config", &msm_fops_config);
	if (rc < 0) {
		pr_err("[CAM]%s: error creating config node: %d\n", __func__, rc);
		msm_tear_down_cdev(pmsm, MKDEV(MAJOR(msm_devno),
				dev_num));
		return rc;
	}

	rc = msm_setup_cdev(pmsm + 2, node,
		MKDEV(MAJOR(msm_devno), dev_num + 2),
		"frame", &msm_fops_frame);
	if (rc < 0) {
		pr_err("[CAM]%s: error creating frame node: %d\n", __func__, rc);
		msm_tear_down_cdev(pmsm,
			MKDEV(MAJOR(msm_devno), dev_num));
		msm_tear_down_cdev(pmsm + 1,
			MKDEV(MAJOR(msm_devno), dev_num + 1));
		return rc;
	}

	atomic_set(&pmsm[0].opened, 0);
	atomic_set(&pmsm[1].opened, 0);
	atomic_set(&pmsm[2].opened, 0);

	pmsm[0].sync = sync;
	pmsm[1].sync = sync;
	pmsm[2].sync = sync;

	return rc;
}

int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(struct msm_camera_sensor_info *,
			struct msm_sensor_ctrl *))
{
	struct msm_cam_device *pmsm = NULL;
	struct msm_sync *sync;
	int rc = -ENODEV;
	static int camera_node = 0;

	if (camera_node >= MSM_MAX_CAMERA_SENSORS) {
		pr_err("[CAM]%s: too many camera sensors\n", __func__);
		return rc;
	}

	if (!msm_class) {
		/* There are three device nodes per sensor */
		rc = alloc_chrdev_region(&msm_devno, 0,
				3 * MSM_MAX_CAMERA_SENSORS,
				"msm_camera");
		if (rc < 0) {
			pr_err("[CAM]%s: failed to allocate chrdev: %d\n", __func__,
				rc);
			return rc;
		}

		msm_class = class_create(THIS_MODULE, "msm_camera");
		if (IS_ERR(msm_class)) {
			rc = PTR_ERR(msm_class);
			pr_err("[CAM]%s: create device class failed: %d\n",
				__func__, rc);
			return rc;
		}
	}

	pmsm = kzalloc(sizeof(struct msm_cam_device) * 3 +
			sizeof(struct msm_sync), GFP_ATOMIC);
	if (!pmsm)
		return -ENOMEM;
	sync = (struct msm_sync *)(pmsm + 3);

	rc = msm_sync_init(sync, dev, sensor_probe, camera_node);
	if (rc < 0) {
		kfree(pmsm);
		return rc;
	}

	CDBG("%s: setting camera node %d\n", __func__, camera_node);
	rc = msm_device_init(pmsm, sync, camera_node);
	if (rc < 0) {
		msm_sync_destroy(sync);
		kfree(pmsm);
		return rc;
	}

	if (!!sync->sdata->flash_cfg)
		msm_camera_sysfs_init(sync);
	camera_node++;
	list_add(&sync->list, &msm_sensors);
	return rc;
}
EXPORT_SYMBOL(msm_camera_drv_start);
