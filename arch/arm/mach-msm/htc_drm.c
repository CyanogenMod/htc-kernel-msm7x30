/* arch/arm/mach-msm/htc_drm-8x60.c
 *
 * Copyright (C) 2011 HTC Corporation.
 * Author: Eddic Hsien <eddic_hsien@htc.com>
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/random.h>

#ifndef CONFIG_ARCH_MSM7X30
#include <mach/scm.h>
#else	/* CONFIG_ARCH_MSM7X30 */
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/semaphore.h>
#include <mach/msm_rpcrouter.h>
#include <mach/oem_rapi_client.h>

#define OEM_RAPI_PROG  0x3000006B
#define OEM_RAPI_VERS  0x00010001

#define OEM_RAPI_NULL_PROC                        0
#define OEM_RAPI_RPC_GLUE_CODE_INFO_REMOTE_PROC   1
#define OEM_RAPI_STREAMING_FUNCTION_PROC          2

#define OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE 128
#endif	/* CONFIG_ARCH_MSM7X30 */

#define DEVICE_NAME "htcdrm"

#define HTCDRM_IOCTL_WIDEVINE	0x2563

#define DEVICE_ID_LEN			32
#define WIDEVINE_KEYBOX_LEN		128

#undef PDEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO "%s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

#undef PERR
#define PERR(fmt, args...) printk(KERN_ERR "%s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

#ifndef CONFIG_ARCH_MSM7X30
#define UP(S)
#else
#define UP(S) up(S)
#endif
static int htcdrm_major;
static struct class *htcdrm_class;
static const struct file_operations htcdrm_fops;

typedef struct _htc_drm_msg_s {
	int func;
	int offset;
	unsigned char *req_buf;
	int req_len;
	unsigned char *resp_buf;
	int resp_len;
} htc_drm_msg_s;

enum {
		HTC_OEMCRYPTO_STORE_KEYBOX = 1,
		HTC_OEMCRYPTO_GET_KEYBOX,
		HTC_OEMCRYPTO_IDENTIFY_DEVICE,
		HTC_OEMCRYPTO_GET_RANDOM,
		HTC_OEMCRYPTO_IS_KEYBOX_VALID,
};

#ifdef CONFIG_ARCH_MSM7X30
struct htc_keybox_dev {
	struct platform_device *pdev;
	struct cdev cdev;
	struct device *device;
	struct class *class;
	dev_t dev_num;
	unsigned char keybox_buf[OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];
	struct semaphore sem;
} htc_keybox_dev;

static struct msm_rpc_client *rpc_client;
static uint32_t open_count;
static DEFINE_MUTEX(oem_rapi_client_lock);
/* TODO: check where to allocate memory for return */
static int oem_rapi_client_cb(struct msm_rpc_client *client,
			      struct rpc_request_hdr *req,
			      struct msm_rpc_xdr *xdr)
{
	uint32_t cb_id, accept_status;
	int rc;
	void *cb_func;
	uint32_t temp;

	struct oem_rapi_client_streaming_func_cb_arg arg;
	struct oem_rapi_client_streaming_func_cb_ret ret;

	arg.input = NULL;
	ret.out_len = NULL;
	ret.output = NULL;

	xdr_recv_uint32(xdr, &cb_id);                    /* cb_id */
	xdr_recv_uint32(xdr, &arg.event);                /* enum */
	xdr_recv_uint32(xdr, (uint32_t *)(&arg.handle)); /* handle */
	xdr_recv_uint32(xdr, &arg.in_len);               /* in_len */
	xdr_recv_bytes(xdr, (void **)&arg.input, &temp); /* input */
	xdr_recv_uint32(xdr, &arg.out_len_valid);        /* out_len */
	if (arg.out_len_valid) {
		ret.out_len = kmalloc(sizeof(*ret.out_len), GFP_KERNEL);
		if (!ret.out_len) {
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
			goto oem_rapi_send_ack;
		}
	}

	xdr_recv_uint32(xdr, &arg.output_valid);         /* out */
	if (arg.output_valid) {
		xdr_recv_uint32(xdr, &arg.output_size);  /* ouput_size */

		ret.output = kmalloc(arg.output_size, GFP_KERNEL);
		if (!ret.output) {
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
			goto oem_rapi_send_ack;
		}
	}

	cb_func = msm_rpc_get_cb_func(client, cb_id);
	if (cb_func) {
		rc = ((int (*)(struct oem_rapi_client_streaming_func_cb_arg *,
			       struct oem_rapi_client_streaming_func_cb_ret *))
		      cb_func)(&arg, &ret);
		if (rc)
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		else
			accept_status = RPC_ACCEPTSTAT_SUCCESS;
	} else
		accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;

 oem_rapi_send_ack:
	xdr_start_accepted_reply(xdr, accept_status);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS) {
		uint32_t temp = sizeof(uint32_t);
		xdr_send_pointer(xdr, (void **)&(ret.out_len), temp,
				 xdr_send_uint32);

		/* output */
		if (ret.output && ret.out_len)
			xdr_send_bytes(xdr, (const void **)&ret.output,
					     ret.out_len);
		else {
			temp = 0;
			xdr_send_uint32(xdr, &temp);
		}
	}
	rc = xdr_send_msg(xdr);
	if (rc)
		pr_err("%s: sending reply failed: %d\n", __func__, rc);

	kfree(arg.input);
	kfree(ret.out_len);
	kfree(ret.output);

	return 0;
}

static int oem_rapi_client_streaming_function_arg(struct msm_rpc_client *client,
						  struct msm_rpc_xdr *xdr,
						  void *data)
{
	int cb_id;
	struct oem_rapi_client_streaming_func_arg *arg = data;

	cb_id = msm_rpc_add_cb_func(client, (void *)arg->cb_func);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;

	xdr_send_uint32(xdr, &arg->event);                /* enum */
	xdr_send_uint32(xdr, &cb_id);                     /* cb_id */
	xdr_send_uint32(xdr, (uint32_t *)(&arg->handle)); /* handle */
	xdr_send_uint32(xdr, &arg->in_len);               /* in_len */
	xdr_send_bytes(xdr, (const void **)&arg->input,
			     &arg->in_len);                     /* input */
	xdr_send_uint32(xdr, &arg->out_len_valid);        /* out_len */
	xdr_send_uint32(xdr, &arg->output_valid);         /* output */

	/* output_size */
	if (arg->output_valid)
		xdr_send_uint32(xdr, &arg->output_size);

	return 0;
}

static int oem_rapi_client_streaming_function_ret(struct msm_rpc_client *client,
						  struct msm_rpc_xdr *xdr,
						  void *data)
{
	struct oem_rapi_client_streaming_func_ret *ret = data;
	uint32_t temp;

	/* out_len */
	xdr_recv_pointer(xdr, (void **)&(ret->out_len), sizeof(uint32_t),
			 xdr_recv_uint32);

	/* output */
	if (ret->out_len && *ret->out_len)
		xdr_recv_bytes(xdr, (void **)&ret->output, &temp);

	return 0;
}

int oem_rapi_client_streaming_function(
	struct msm_rpc_client *client,
	struct oem_rapi_client_streaming_func_arg *arg,
	struct oem_rapi_client_streaming_func_ret *ret)
{
	return msm_rpc_client_req2(client,
				   OEM_RAPI_STREAMING_FUNCTION_PROC,
				   oem_rapi_client_streaming_function_arg, arg,
				   oem_rapi_client_streaming_function_ret,
				   ret, -1);
}
EXPORT_SYMBOL(oem_rapi_client_streaming_function);

int oem_rapi_client_close(void)
{
	mutex_lock(&oem_rapi_client_lock);
	if (open_count > 0) {
		if (--open_count == 0) {
			msm_rpc_unregister_client(rpc_client);
			pr_info("%s: disconnected from remote oem rapi server\n",
				__func__);
		}
	}
	mutex_unlock(&oem_rapi_client_lock);
	return 0;
}
EXPORT_SYMBOL(oem_rapi_client_close);

struct msm_rpc_client *oem_rapi_client_init(void)
{
	mutex_lock(&oem_rapi_client_lock);
	if (open_count == 0) {
		rpc_client = msm_rpc_register_client2("oemrapiclient",
						      OEM_RAPI_PROG,
						      OEM_RAPI_VERS, 0,
						      oem_rapi_client_cb);
		if (!IS_ERR(rpc_client))
			open_count++;
	} else {
		/* increase the counter */
		open_count++;
	}
	mutex_unlock(&oem_rapi_client_lock);
	return rpc_client;
}
EXPORT_SYMBOL(oem_rapi_client_init);

static ssize_t htc_keybox_read(struct htc_keybox_dev *dev, char *buf, size_t size, loff_t p)
{
	unsigned int count = size;
	int ret_rpc = 0;
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	unsigned char nullbuf[OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];

	memset(dev->keybox_buf, 56, OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE);
	memset(nullbuf, 0, OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE);

	printk(KERN_INFO "htc_keybox_read start:\n");
	if (p >= OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE)
		return count ? -ENXIO : 0;

	printk(KERN_INFO "htc_keybox_read oem_rapi_client_streaming_function start:\n");
	if (count == 0xFF) {
		arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_DEVICE_ID;
		memset(dev->keybox_buf, 57, OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE);
		printk(KERN_INFO "htc_keybox_read: OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_DEVICE_ID\n");
	} else if (count > OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p) {
		count = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p;
		arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_KEYBOX;
		printk(KERN_INFO "htc_keybox_read: OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_KEYBOX\n");
	} else {
		arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_KEYBOX;
		printk(KERN_INFO "htc_keybox_read: OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_KEYBOX\n");
	}
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = 0;
	arg.input = (char *)nullbuf;
	arg.out_len_valid = 1;
	arg.output_valid = 1;
	arg.output_size = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE;
	ret.out_len = NULL;
	ret.output = NULL;

	ret_rpc = oem_rapi_client_streaming_function(rpc_client, &arg, &ret);
	if (ret_rpc) {
		printk(KERN_INFO "%s: Get data from modem failed: %d\n", __func__, ret_rpc);
		return -EFAULT;
	}
	printk(KERN_INFO "%s: Data obtained from modem %d, ", __func__, *(ret.out_len));
	memcpy(dev->keybox_buf, ret.output, *(ret.out_len));
	kfree(ret.out_len);
	kfree(ret.output);

	return 0;
}

static ssize_t htc_keybox_write(struct htc_keybox_dev *dev, const char *buf, size_t size, loff_t p)
{
	unsigned int count = size;
	int ret_rpc = 0;
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;

	printk(KERN_INFO "htc_keybox_write start:\n");
	if (p >= OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE)
		return count ? -ENXIO : 0;
	if (count > OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p)
		count = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p;
	printk(KERN_INFO "htc_keybox_write oem_rapi_client_streaming_function start:\n");
	arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_WRITE_KEYBOX;
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE;
	arg.input = (char *)dev->keybox_buf;
	arg.out_len_valid = 0;
	arg.output_valid = 0;
	arg.output_size = 0;
	ret.out_len = NULL;
	ret.output = NULL;

	ret_rpc = oem_rapi_client_streaming_function(rpc_client, &arg, &ret);
	if (ret_rpc) {
		printk(KERN_INFO "%s: Send data from modem failed: %d\n", __func__, ret_rpc);
		return -EFAULT;
	}
	printk(KERN_INFO "%s: Data sent to modem %s\n", __func__, dev->keybox_buf);

	return 0;
}
static struct htc_keybox_dev *keybox_dev;
#endif /* CONFIG_ARCH_MSM7X30 */

static unsigned char *htc_device_id;
static unsigned char *htc_keybox;

static long htcdrm_ioctl(struct file *file, unsigned int command, unsigned long arg)
{
	htc_drm_msg_s hmsg;
	int ret = 0;
	unsigned char *ptr;

	PDEBUG("command = %x\n", command);
	switch (command) {
	case HTCDRM_IOCTL_WIDEVINE:
		if (copy_from_user(&hmsg, (void __user *)arg, sizeof(hmsg))) {
			PERR("copy_from_user error (msg)");
			return -EFAULT;
		}
#ifdef CONFIG_ARCH_MSM7X30
		oem_rapi_client_init();
		if (down_interruptible(&keybox_dev->sem)) {
			PERR("interrupt error");
			return -EFAULT;
		}
#endif /* CONFIG_ARCH_MSM7X30 */
		PDEBUG("func = %x\n", hmsg.func);
		switch (hmsg.func) {
		case HTC_OEMCRYPTO_STORE_KEYBOX:
			if ((hmsg.req_buf == NULL) || (hmsg.req_len != WIDEVINE_KEYBOX_LEN)) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
			if (copy_from_user(htc_keybox, (void __user *)hmsg.req_buf, hmsg.req_len)) {
				PERR("copy_from_user error (keybox)");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
#ifdef CONFIG_ARCH_MSM7X30
			memcpy(keybox_dev->keybox_buf, htc_keybox, WIDEVINE_KEYBOX_LEN);
			ret = htc_keybox_write(keybox_dev, htc_keybox, hmsg.req_len, hmsg.offset);
			oem_rapi_client_close();
#else
			ret = secure_access_item(1, ITEM_KEYBOX_PROVISION, hmsg.req_len,
					htc_keybox);
#endif	/* CONFIG_ARCH_MSM7X30 */
			if (ret)
				PERR("provision keybox failed (%d)\n", ret);
			UP(&keybox_dev->sem);
			break;
		case HTC_OEMCRYPTO_GET_KEYBOX:
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len ||
					((hmsg.offset + hmsg.resp_len) > WIDEVINE_KEYBOX_LEN)) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
#ifdef CONFIG_ARCH_MSM7X30
			ret = htc_keybox_read(keybox_dev, htc_keybox, hmsg.resp_len, hmsg.offset);
			oem_rapi_client_close();
			htc_keybox = keybox_dev->keybox_buf;
#else
			ret = secure_access_item(0, ITEM_KEYBOX_DATA, WIDEVINE_KEYBOX_LEN,
					htc_keybox);
#endif	/* CONFIG_ARCH_MSM7X30 */
			if (ret)
				PERR("get keybox failed (%d)\n", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, htc_keybox + hmsg.offset, hmsg.resp_len)) {
					PERR("copy_to_user error (keybox)");
					UP(&keybox_dev->sem);
					return -EFAULT;
				}
			}
			UP(&keybox_dev->sem);
			break;
		case HTC_OEMCRYPTO_IDENTIFY_DEVICE:
			if ((hmsg.resp_buf == NULL) || ((hmsg.resp_len != DEVICE_ID_LEN) && (hmsg.resp_len != 0xFF))) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}

#ifdef CONFIG_ARCH_MSM7X30
			ret = htc_keybox_read(keybox_dev, htc_device_id, hmsg.resp_len, hmsg.offset);
			oem_rapi_client_close();
			htc_device_id = keybox_dev->keybox_buf;
#else
			ret = secure_access_item(0, ITEM_DEVICE_ID, DEVICE_ID_LEN,
					htc_device_id);
#endif	/* CONFIG_ARCH_MSM7X30 */
			if (ret)
				PERR("get device ID failed (%d)\n", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, htc_device_id, DEVICE_ID_LEN)) {
					PERR("copy_to_user error (device ID)");
					UP(&keybox_dev->sem);
					return -EFAULT;
				}
			}
			UP(&keybox_dev->sem);
			break;
		case HTC_OEMCRYPTO_GET_RANDOM:
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
			ptr = kzalloc(hmsg.resp_len, GFP_KERNEL);
			if (ptr == NULL) {
				PERR("allocate the space for random data failed\n");
				UP(&keybox_dev->sem);
				return -1;
			}
#ifdef CONFIG_ARCH_MSM7X30
			get_random_bytes(ptr, hmsg.resp_len);
			printk(KERN_INFO "%s: Data get from random entropy ", __func__);
#else
			get_random_bytes(ptr, hmsg.resp_len);
			/* FIXME: second time of this function call will hang
			ret = secure_access_item(0, ITEM_RAND_DATA, hmsg.resp_len, ptr);
			*/
#endif	/* CONFIG_ARCH_MSM7X30 */
			if (ret)
				PERR("get random data failed (%d)\n", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, ptr, hmsg.resp_len)) {
					PERR("copy_to_user error (random data)");
					kfree(ptr);
					UP(&keybox_dev->sem);
					return -EFAULT;
				}
			}
			UP(&keybox_dev->sem);
			kfree(ptr);
			break;
		case HTC_OEMCRYPTO_IS_KEYBOX_VALID:
#ifndef CONFIG_ARCH_MSM7X30
			ret = secure_access_item(0, ITEM_VALIDATE_KEYBOX, WIDEVINE_KEYBOX_LEN, NULL);
#endif
			UP(&keybox_dev->sem);
			return ret;
		default:
			UP(&keybox_dev->sem);
			PERR("func error\n");
			return -EFAULT;
		}
		break;

	default:
		PERR("command error\n");
		return -EFAULT;
	}
	return ret;
}

static const struct file_operations htcdrm_fops = {
	.unlocked_ioctl = htcdrm_ioctl,
	.owner = THIS_MODULE,
};


static int __init htcdrm_init(void)
{
	int ret;

	htc_device_id = kzalloc(DEVICE_ID_LEN, GFP_KERNEL);
	if (htc_device_id == NULL) {
		PERR("allocate the space for device ID failed\n");
		return -1;
	}
	htc_keybox = kzalloc(WIDEVINE_KEYBOX_LEN, GFP_KERNEL);
	if (htc_keybox == NULL) {
		PERR("allocate the space for keybox failed\n");
		kfree(htc_device_id);
		return -1;
	}

	ret = register_chrdev(0, DEVICE_NAME, &htcdrm_fops);
	if (ret < 0) {
		PERR("register module fail\n");
		kfree(htc_device_id);
		kfree(htc_keybox);
		return ret;
	}
	htcdrm_major = ret;

	htcdrm_class = class_create(THIS_MODULE, "htcdrm");
	device_create(htcdrm_class, NULL, MKDEV(htcdrm_major, 0), NULL, DEVICE_NAME);

#ifdef CONFIG_ARCH_MSM7X30
	keybox_dev = kzalloc(sizeof(htc_keybox_dev), GFP_KERNEL);
	if (keybox_dev == NULL) {
		PERR("allocate space for keybox_dev failed\n");
		kfree(keybox_dev);
		return -1;
	}
	sema_init(&keybox_dev->sem, 1);
#endif
	PDEBUG("register module ok\n");
	return 0;
}


static void  __exit htcdrm_exit(void)
{
	device_destroy(htcdrm_class, MKDEV(htcdrm_major, 0));
	class_unregister(htcdrm_class);
	class_destroy(htcdrm_class);
	unregister_chrdev(htcdrm_major, DEVICE_NAME);
	kfree(htc_device_id);
	kfree(htc_keybox);
#ifdef CONFIG_ARCH_MSM7X30
	kfree(keybox_dev);
#endif
	PDEBUG("un-registered module ok\n");
}

module_init(htcdrm_init);
module_exit(htcdrm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jon Tsai");
