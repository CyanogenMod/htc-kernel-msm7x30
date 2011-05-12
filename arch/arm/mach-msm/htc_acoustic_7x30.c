/* arch/arm/mach-msm/htc_acoustic_7x30.c
 *
 * Copyright (C) 2009 HTC Corporation
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
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mfd/msm-adie-codec.h>
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/qdsp5v2/audio_acdb.h>
#include <mach/htc_headset_mgr.h>

#include <mach/msm_smd.h>
#include <mach/msm_rpcrouter.h>
#include "smd_private.h"

#include <mach/htc_acdb.h>
#include <mach/htc_acoustic_7x30.h>
#include "board-vision.h"

#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_ADIE_SIZE	_IOW(ACOUSTIC_IOCTL_MAGIC, 15, size_t)
#define SHARE_MEMORY_SIZE	_IOR(ACOUSTIC_IOCTL_MAGIC, 16, size_t)
#define ACOUSTIC_UPDATE_ADIE	_IOW(ACOUSTIC_IOCTL_MAGIC, 24, \
					struct profile_action_info *)
#define ACOUSTIC_UPDATE_ACDB	_IOW(ACOUSTIC_IOCTL_MAGIC, 25, unsigned int)
#define ACOUSTIC_UPDATE_VOICE_ACDB	_IOW(ACOUSTIC_IOCTL_MAGIC, 26, \
						unsigned int)
#define ACOUSTIC_GET_AUDIENCE_STATE	_IOW(ACOUSTIC_IOCTL_MAGIC, 27, int)
#define ACOUSTIC_GET_AIC3254_STATE	_IOW(ACOUSTIC_IOCTL_MAGIC, 28, int)
#define ACOUSTIC_MIC_DISABLE	_IOW(ACOUSTIC_IOCTL_MAGIC, 29, int)
#define ACOUSTIC_REINIT_ACDB    _IOW(ACOUSTIC_IOCTL_MAGIC, 30, unsigned)
#define ACOUSTIC_GET_BACK_MIC_STATE	_IOW(ACOUSTIC_IOCTL_MAGIC, 31, int)
#define ACOUSTIC_MUTE_HEADSET 	_IOW(ACOUSTIC_IOCTL_MAGIC, 32, int)
#define ACOUSTIC_GET_TABLES 	_IOW(ACOUSTIC_IOCTL_MAGIC, 33, unsigned)
#define ACOUSTIC_ENABLE_BACK_MIC	_IOW(ACOUSTIC_IOCTL_MAGIC, 34, unsigned)

#define D(fmt, args...) printk(KERN_INFO "[AUD] htc-acoustic: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "[AUD] htc-acoustic: "fmt, ##args)

#define SHARE_PAGES 4

#define HTC_ACDB_TABLE_SIZE        (0x55000)
#define HTCPROG	0x30100002
#define HTCVERS 0
#define ONCRPC_ALLOC_ACOUSTIC_MEM_PROC (1)
#define ONCRPC_ADD_ACDB_TABLE_PROC     (2)
#define ONCRPC_SET_VOICE_ACDB_PROC  (4)

extern void *htc_acdb_data;

struct audio_config_database *db;
struct acdb_config temp;
static struct mutex api_lock;
static struct mutex rpc_connect_lock;
static struct acoustic_ops *the_ops;

struct acdb_id {
	u32 tx_dev_id;
	u32 tx_acdb_id;
	u32 rx_dev_id;
	u32 rx_acdb_id;
};

static struct msm_rpc_endpoint *endpoint;
struct set_smem_req {
	struct rpc_request_hdr hdr;
	uint32_t size;
};

struct set_smem_rep {
	struct rpc_reply_hdr hdr;
	int n;
};

enum {
	MEDIA_SETTING = 0,
	VOICE_SETTING = 1,
};

struct class *htc_class;
void *htc_adie_table;
uint32_t action_offset;

struct profile_action_info {
	u32 act_sz;
	char name[64];
	u32 freq;
	u32 setting;
};

void acoustic_register_ops(struct acoustic_ops *ops)
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

static int acoustic_open(struct inode *inode, struct file *file)
{
	D("open\n");
	return 0;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	D("release\n");
	return 0;
}

static long
acoustic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0, i, j, setting_sz;
	int reg_dev = 0;
	size_t sz;
	struct profile_action_info act_info;
	struct msm_snddev_info *dev_info;
	struct snddev_icodec_state *icodec;
	struct adie_codec_hwsetting_entry *entry;
	struct adie_codec_action_unit *htc_adie_ptr;
	struct acdb_id cur_acdb_id;
	char filename[64];

	mutex_lock(&api_lock);
	switch (cmd) {
	case ACOUSTIC_ADIE_SIZE:
		if (copy_from_user(&sz, (void *)arg, sizeof(size_t))) {
			rc = -EFAULT;
			break;
		}
		D("setting size = %d\n", sz);

		if (htc_adie_table) {
			kfree(htc_adie_table);
			htc_adie_table = NULL;
		}

		if (!htc_adie_table) {
			/* allocate 4 pages for adie table*/
			htc_adie_table = kzalloc(16384, GFP_KERNEL);
			if (!htc_adie_table) {
				E("cannot allocate enough memory.\n");
				rc = -EINVAL;
				break;
			}
			action_offset = 0;
		}

		break;
	case ACOUSTIC_UPDATE_ADIE:
		if (copy_from_user(&act_info, (void *)arg,
			sizeof(struct profile_action_info))) {
			E("copy_from_user failed\n");
			rc = -EFAULT;
			break;
		}

		if (act_info.act_sz < 1) {
			E("can't update setting of %s without"
				"active action\n", act_info.name);
			rc = -EFAULT;
			break;
		}

		/* set un-initialized frequency to default 8K */
		if (act_info.freq == 0)
			act_info.freq = 8000;

		/* got registry devices through msm_snddev_* API
		 * which defined in "audio_dev_ctl.h" */
		reg_dev = msm_snddev_devcount();
		for (i = 0; i < reg_dev; i++) {
			dev_info = audio_dev_ctrl_find_dev(i);
			if (strncmp(act_info.name, dev_info->name,
				strlen(dev_info->name)) == 0)
			break;
		}

		if (i < reg_dev) {
			icodec = (struct snddev_icodec_state *)dev_info->private_data;
			setting_sz = icodec->data->profile->setting_sz;

			for (j = 0; j < setting_sz; j++) {
				entry = icodec->data->profile->settings;
				if (act_info.freq == entry[j].freq_plan) {
					htc_adie_ptr =
					(struct adie_codec_action_unit *)
					(htc_adie_table + action_offset);

					if (act_info.setting == VOICE_SETTING) {
						D("Update adie (voice) of %s\n", dev_info->name);
						entry[j].voc_action = htc_adie_ptr;
						entry[j].voc_action_sz = act_info.act_sz;
						} else {
						D("Update adie (media) of %s\n", dev_info->name);
						entry[j].midi_action = htc_adie_ptr;
						entry[j].midi_action_sz = act_info.act_sz;
						/* assign new setting pass from user space
						 * default adie setting is for midi */
						entry[j].actions = htc_adie_ptr;
						entry[j].action_sz = act_info.act_sz;
						}

						break;
				}
			}
			if (j >= setting_sz) {
				E("can't find device with frequency %d\n",
				  act_info.freq);
				rc = -EFAULT;
			}
		} else {
			E("can't find registry device with name %s\n", act_info.name);
			rc = -EFAULT;
		}

		/* calculate next address of action presentation */
		action_offset +=
			act_info.act_sz * sizeof(struct adie_codec_action_unit);
		break;

	case ACOUSTIC_UPDATE_ACDB:
		if (copy_from_user(&cur_acdb_id, (void *)arg,
			sizeof(struct acdb_id))) {
			rc = -EFAULT;
			break;
		}
		pr_aud_info("%s update ACDB ID : (%d, %d, %d, %d)\n",
			__func__,
			cur_acdb_id.tx_dev_id,
			cur_acdb_id.rx_dev_id,
			cur_acdb_id.tx_acdb_id,
			cur_acdb_id.rx_acdb_id);

		if (cur_acdb_id.tx_acdb_id > 0) {
			dev_info = audio_dev_ctrl_find_dev(cur_acdb_id.tx_dev_id);
			dev_info->acdb_id = cur_acdb_id.tx_acdb_id;
		}

		if (cur_acdb_id.rx_acdb_id > 0) {
			dev_info = audio_dev_ctrl_find_dev(cur_acdb_id.rx_dev_id);
			dev_info->acdb_id = cur_acdb_id.rx_acdb_id;
		}

		break;
	case ACOUSTIC_UPDATE_VOICE_ACDB: {
		struct update_acdb_req {
			struct rpc_request_hdr hdr;
			uint32_t tx_id;
			uint32_t rx_id;
		} acdb_req;

		struct voice_acdb_id {
			uint32_t tx_id;
			uint32_t rx_id;
		} voc;

		if (copy_from_user(&voc, (void *)arg,
			sizeof(struct voice_acdb_id))) {
			rc = -EFAULT;
			break;
		}

		if (is_rpc_connect() == -1) {
			rc = -EIO;
			break;
		}

		acdb_req.tx_id = cpu_to_be32(voc.tx_id);
		acdb_req.rx_id = cpu_to_be32(voc.rx_id);

		rc = msm_rpc_call(endpoint, ONCRPC_SET_VOICE_ACDB_PROC,
				    &acdb_req, sizeof(acdb_req), 5 * HZ);
		D("update voice ACDB (%d, %d), rc %d\n",
			voc.tx_id, voc.rx_id, rc);
		break;
	}
	case ACOUSTIC_GET_AUDIENCE_STATE: {
		int support_a1026 = 0;
		if (the_ops->support_audience)
			support_a1026 = the_ops->support_audience();
		D("support_a1026: %d\n", support_a1026);
		if (copy_to_user((void *) arg,
			&support_a1026, sizeof(int))) {
			E("acoustic_ioctl: get engineerID failed\n");
			rc = -EFAULT;
		}
		break;
	}
	case ACOUSTIC_GET_AIC3254_STATE: {
		int support_aic3254 = 0;
		if (the_ops->support_aic3254)
			support_aic3254 = the_ops->support_aic3254();
		D("support_aic3254: %d\n", support_aic3254);
		if (copy_to_user((void *) arg,
			&support_aic3254, sizeof(int))) {
			E("acoustic_ioctl: copy to user failed\n");
			rc = -EFAULT;
		}
		break;
	}
	case ACOUSTIC_MIC_DISABLE: {
		int mic = -1;
		if (copy_from_user(&mic, (void *)arg, sizeof(int))) {
			rc = -EFAULT;
			break;
		}
		if (the_ops->mic_disable)
			the_ops->mic_disable(mic);
		break;
	}
	case ACOUSTIC_REINIT_ACDB:
		rc = copy_from_user(&filename, (void *)arg, sizeof(filename));
		if (!rc)
			rc = htc_reinit_acdb(filename);
		break;
	case ACOUSTIC_GET_BACK_MIC_STATE: {
		int support_back_mic = 0;
		if (the_ops->support_back_mic)
			support_back_mic = the_ops->support_back_mic();
		D("support_back_mic: %d\n", support_back_mic);
		if (copy_to_user((void *) arg,
			&support_back_mic, sizeof(int))) {
			E("acoustic_ioctl: ACOUSTIC_GET_BACK_MIC_STATE failed\n");
			rc = -EFAULT;
		}
		break;
	}
	case ACOUSTIC_MUTE_HEADSET: {
		int en;
		if (copy_from_user(&en, (void *)arg,
			sizeof(int))) {
			rc = -EFAULT;
			break;
		}
		if (the_ops->mute_headset_amp)
			the_ops->mute_headset_amp(en);
		break;
	}

	case ACOUSTIC_GET_TABLES: {
		if (the_ops->get_acoustic_tables) {
			struct acoustic_tables tb;
			memset(tb.aic3254, '\0', PROPERTY_VALUE_MAX);
			memset(tb.adie, '\0', PROPERTY_VALUE_MAX);
			memset(tb.spkamp, '\0', PROPERTY_VALUE_MAX);
			memset(tb.acdb, '\0', PROPERTY_VALUE_MAX);
			the_ops->get_acoustic_tables(&tb);
			if (copy_to_user((void *) arg,
				&tb, sizeof(tb))) {
				E("acoustic_ioctl: ACOUSTIC_ACOUSTIC_GET_TABLES failed\n");
				rc = -EFAULT;
			}
		}
		else
			rc = -EFAULT;
		break;
	}
	case ACOUSTIC_ENABLE_BACK_MIC:  {
		int en = 0;
		if (copy_from_user(&en, (void *)arg, sizeof(int))) {
			rc = -EFAULT;
			break;
		}
		if (the_ops->enable_back_mic)
			the_ops->enable_back_mic(en);
		break;
	}
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&api_lock);
	return rc;
}

static int acoustic_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size;
	int rc = -EINVAL;

	mutex_lock(&api_lock);
	size = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff != 0) {
		E("mmap failed: page offset %lx\n", vma->vm_pgoff);
		goto done;
	}
	vma->vm_flags |= VM_RESERVED;
	rc = io_remap_pfn_range(vma,
		vma->vm_start,
		virt_to_phys((void *)htc_adie_table) >> PAGE_SHIFT,
		size,
		vma->vm_page_prot);

	if (rc < 0)
		E("mmap failed: remap error %d\n", rc);
done:
	mutex_unlock(&api_lock);
	return rc;
}


static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.mmap = acoustic_mmap,
	.unlocked_ioctl = acoustic_ioctl,
};

static struct miscdevice acoustic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic",
	.fops = &acoustic_fops,
};

int do_rpc(int num)
{
	struct acdb_req {
		struct rpc_request_hdr hdr;
		int32_t device_id;
		uint32_t sample_rate;
		uint32_t len;
		uint32_t num;
	} req;

	req.device_id = cpu_to_be32(0);
	req.sample_rate = cpu_to_be32(8000);
	req.len = cpu_to_be32(100);
	req.num = cpu_to_be32(num);

	return msm_rpc_call(endpoint, ONCRPC_ADD_ACDB_TABLE_PROC,
			    &req, sizeof(req), 5 * HZ);
}

static ssize_t attr_show(struct device *dev,
	struct device_attribute *attr,  char *buf)
{
    return 0;
}

static ssize_t attr_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(sysattr, 0664, attr_show, attr_store);

int enable_mic_bias(int on)
{
	pr_aud_info("%s called %d\n", __func__, on);
	if (the_ops->enable_mic_bias)
		the_ops->enable_mic_bias(on, 1);

	return 0;
}

static int __init acoustic_init(void)
{
	int ret = 0;

	mutex_init(&api_lock);
	mutex_init(&rpc_connect_lock);
	ret = misc_register(&acoustic_misc);
	if (ret < 0) {
		pr_aud_err("failed to register misc device!\n");
		return ret;
	}

	htc_class = class_create(THIS_MODULE, "htc_acoustic");
	if (IS_ERR(htc_class)) {
		ret = PTR_ERR(htc_class);
		htc_class = NULL;
		goto err_create_class;
	}

	acoustic_misc.this_device = device_create(htc_class, NULL, 0 , NULL, "hac");
	if (IS_ERR(acoustic_misc.this_device)) {
		ret = PTR_ERR(acoustic_misc.this_device);
		acoustic_misc.this_device = NULL;
		goto err_create_class;
	}

	ret = device_create_file(acoustic_misc.this_device, &dev_attr_sysattr);
	if (ret < 0)
		goto err_create_class_device;

#if defined(CONFIG_HTC_HEADSET_MGR)
	{
		struct headset_notifier notifier;
		notifier.id = HEADSET_REG_MIC_BIAS;
		notifier.func = enable_mic_bias;
		headset_notifier_register(&notifier);
	}
#endif

	return 0;

err_create_class_device:
	device_destroy(htc_class, 0);
err_create_class:
	return ret;
}

static void __exit acoustic_exit(void)
{
	device_remove_file(acoustic_misc.this_device, &dev_attr_sysattr);
	device_destroy(htc_class, 0);
	class_destroy(htc_class);

	misc_deregister(&acoustic_misc);
}

module_init(acoustic_init);
module_exit(acoustic_exit);
