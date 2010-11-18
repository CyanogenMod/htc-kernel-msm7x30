/* drivers/i2c/chips/bma150.c - bma150 G-sensor driver
 *
 * Copyright (C) 2008-2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/bma150.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include<linux/earlysuspend.h>

static struct i2c_client *this_client;

struct bma150_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend early_suspend;
};

static struct bma150_platform_data *pdata;
static atomic_t PhoneOn_flag = ATOMIC_INIT(0);
#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(_name, _mode, _show, _store)

static int BMA_I2C_RxData(char *rxData, int length)
{
	int retry;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		},
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (retry = 0; retry <= 100; retry++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;
		else
			mdelay(10);
	}
	if (retry > 100) {
		printk(KERN_ERR "%s: retry over 100\n", __func__);
		return -EIO;
	}	else
	return 0;


}

static int BMA_I2C_TxData(char *txData, int length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (retry = 0; retry <= 100; retry++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;
		else
			mdelay(10);
	}
	if (retry > 100) {
		printk(KERN_ERR "%s: retry over 100\n", __func__);
		return -EIO;
	}	else
	return 0;
}
static int BMA_Init(void)
{
	char buffer[4] = "";
	int ret;
	buffer[0] = RANGE_BWIDTH_REG;
	ret = BMA_I2C_RxData(buffer, 2);
	if (ret < 0)
		return -1;
	buffer[3] = buffer[1]|1<<3;
	buffer[2] = SMB150_CONF2_REG;
	buffer[1] = (buffer[0]&0xe0);
	buffer[0] = RANGE_BWIDTH_REG;
	ret = BMA_I2C_TxData(buffer, 4);
	if (ret < 0)
		return -1;
	return 0;

}

static int BMA_TransRBuff(short *rbuf)
{
	char buffer[6];
	int ret;
	memset(buffer, 0, 6);
	buffer[0] = X_AXIS_LSB_REG;
	ret = BMA_I2C_RxData(buffer, 6);
	if (ret < 0)
		return 0;
	rbuf[0] = buffer[1]<<2|buffer[0]>>6;
	if (rbuf[0]&0x200)
		rbuf[0] -= 1<<10;
	rbuf[1] = buffer[3]<<2|buffer[2]>>6;
	if (rbuf[1]&0x200)
		rbuf[1] -= 1<<10;
	rbuf[2] = buffer[5]<<2|buffer[4]>>6;
	if (rbuf[2]&0x200)
		rbuf[2] -= 1<<10;
	return 1;
}
/*
static int BMA_set_range(char range)
{
	char buffer[2];
	int ret;
	buffer[0] = RANGE_BWIDTH_REG;
	ret = BMA_I2C_RxData(buffer, 1);
	if (ret < 0)
		return -1;
	buffer[1] = (buffer[0]&0xe7)|range<<3;
	buffer[0] = RANGE_BWIDTH_REG;
	ret = BMA_I2C_TxData(buffer, 2);

	return ret;
}
*/
/*
static int BMA_get_range(void)
{
	char buffer;
	int ret;
	buffer = RANGE_BWIDTH_REG;
	ret = BMA_I2C_RxData(&buffer, 1);
	if (ret < 0)
		return -1;
	buffer = (buffer&0x18)>>3;
	return buffer;
}
*/
/*
static int BMA_reset_int(void)
{
	char buffer[2];
	int ret;
	buffer[0] = SMB150_CTRL_REG;
	ret = BMA_I2C_RxData(buffer, 1);
	if (ret < 0)
		return -1;
	buffer[1] = (buffer[0]&0xbf)|0x40;
	buffer[0] = SMB150_CTRL_REG;
	ret = BMA_I2C_TxData(buffer, 2);

	return ret;
}
*/
/* set  operation mode 0 = normal, 1 = sleep*/
static int BMA_set_mode(char mode)
{
	char buffer[2] = "";
	int ret;
	buffer[0] = SMB150_CTRL_REG;
	ret = BMA_I2C_RxData(buffer, 1);
	if (ret < 0)
		return -1;
	buffer[1] = (buffer[0]&0xfe)|mode;
	buffer[0] = SMB150_CTRL_REG;
	ret = BMA_I2C_TxData(buffer, 2);
	return ret;
}

static int BMA_GET_INT(void)
{
	int ret;
	ret = gpio_get_value(pdata->intr);
	return ret;
}

static int bma_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int bma_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int bma_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{

	void __user *argp = (void __user *)arg;

	char rwbuf[8] = "";
	int ret = -1;
	short buf[8], temp;
	int kbuf = 0;

	switch (cmd) {
	case BMA_IOCTL_READ:
	case BMA_IOCTL_WRITE:
	case BMA_IOCTL_SET_MODE:
	case BMA_IOCTL_SET_CALI_MODE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_from_user(&buf, argp, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		if (copy_from_user(&kbuf, argp, sizeof(kbuf)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case BMA_IOCTL_INIT:
		ret = BMA_Init();
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		ret = BMA_I2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		ret = BMA_I2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		pdata->gs_kvalue = kbuf;
		printk(KERN_INFO "%s: Write calibration value: 0x%X\n",
			__func__, pdata->gs_kvalue);
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		ret = BMA_TransRBuff(&buf[0]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if ((pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
			rwbuf[0] = 0;
			rwbuf[1] = 0;
			rwbuf[2] = 0;
		} else {
			rwbuf[0] = (pdata->gs_kvalue >> 16) & 0xFF;
			rwbuf[1] = (pdata->gs_kvalue >>  8) & 0xFF;
			rwbuf[2] =  pdata->gs_kvalue        & 0xFF;
		}
		break;
	case BMA_IOCTL_SET_MODE:
		BMA_set_mode(rwbuf[0]);
		break;
	case BMA_IOCTL_GET_INT:
		temp = BMA_GET_INT();
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (pdata)
			temp = pdata->chip_layout;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (pdata)
			temp = pdata->calibration_mode;
		break;
	case BMA_IOCTL_SET_CALI_MODE:
		if (pdata)
			pdata->calibration_mode = rwbuf[0];
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case BMA_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_to_user(argp, &buf, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_INT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static void bma150_early_suspend(struct early_suspend *handler)
{
	if (!atomic_read(&PhoneOn_flag)) {
		BMA_set_mode(BMA_MODE_SLEEP);
	} else
		printk(KERN_DEBUG "bma150_early_suspend: PhoneOn_flag is set\n");
}

static void bma150_early_resume(struct early_suspend *handler)
{
	BMA_set_mode(BMA_MODE_NORMAL);
}
static ssize_t bma150_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", atomic_read(&PhoneOn_flag));
	return (s - buf);
}

static ssize_t bma150_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		atomic_set(&PhoneOn_flag, 1);
		printk(KERN_DEBUG "bma150_store: PhoneOn_flag=%d\n", atomic_read(&PhoneOn_flag));
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		atomic_set(&PhoneOn_flag, 0);
		printk(KERN_DEBUG "bma150_store: PhoneOn_flag=%d\n", atomic_read(&PhoneOn_flag));
		return count;
	}
	printk(KERN_ERR "bma150_store: invalid argument\n");
	return -EINVAL;

}

static DEVICE_ACCESSORY_ATTR(PhoneOnOffFlag, 0666, \
	bma150_show, bma150_store);

int bma150_registerAttr(void)
{
	int ret;
	struct class *htc_accelerometer_class;
	struct device *accelerometer_dev;

	htc_accelerometer_class = class_create(THIS_MODULE, "htc_accelerometer");
	if (IS_ERR(htc_accelerometer_class)) {
		ret = PTR_ERR(htc_accelerometer_class);
		htc_accelerometer_class = NULL;
		goto err_create_class;
	}

	accelerometer_dev = device_create(htc_accelerometer_class,
				NULL, 0, "%s", "accelerometer");
	if (unlikely(IS_ERR(accelerometer_dev))) {
		ret = PTR_ERR(accelerometer_dev);
		accelerometer_dev = NULL;
		goto err_create_accelerometer_device;
	}

	/* register the attributes */
	ret = device_create_file(accelerometer_dev, &dev_attr_PhoneOnOffFlag);
	if (ret)
		goto err_create_accelerometer_device_file;

	return 0;

err_create_accelerometer_device_file:
	device_unregister(accelerometer_dev);
err_create_accelerometer_device:
	class_destroy(htc_accelerometer_class);
err_create_class:

	return ret;
}

static struct file_operations bma_fops = {
	.owner = THIS_MODULE,
	.open = bma_open,
	.release = bma_release,
	.ioctl = bma_ioctl,
};

static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150",
	.fops = &bma_fops,
};

int bma150_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bma150_data *bma;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	bma = kzalloc(sizeof(struct bma150_data), GFP_KERNEL);
	if (!bma) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, bma);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "bma150_init_client: platform data is NULL\n");
		goto exit_platform_data_null;
	}

	pdata->gs_kvalue = gs_kvalue;
	printk(KERN_INFO "BMA150 G-sensor I2C driver: gs_kvalue = 0x%X\n",
		pdata->gs_kvalue);

	this_client = client;

	err = BMA_Init();
	if (err < 0) {
		printk(KERN_ERR "bma150_probe: bma_init failed\n");
		goto exit_init_failed;
	}

	err = misc_register(&bma_device);
	if (err) {
		printk(KERN_ERR "bma150_probe: device register failed\n");
		goto exit_misc_device_register_failed;
	}

	bma->early_suspend.suspend = bma150_early_suspend;
	bma->early_suspend.resume = bma150_early_resume;
	register_early_suspend(&bma->early_suspend);

	err = bma150_registerAttr();
	if (err) {
		printk(KERN_ERR "%s: set spi_bma150_registerAttr fail!\n", __func__);
		goto err_registerAttr;
	}
	return 0;
err_registerAttr:

exit_misc_device_register_failed:
exit_init_failed:
exit_platform_data_null:
	kfree(bma);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int bma150_remove(struct i2c_client *client)
{
	struct bma150_data *bma = i2c_get_clientdata(client);
	kfree(bma);
	return 0;
}

static const struct i2c_device_id bma150_id[] = {
	{ BMA150_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver bma150_driver = {
	.probe = bma150_probe,
	.remove = bma150_remove,
	.id_table	= bma150_id,
	.driver = {
		   .name = BMA150_I2C_NAME,
		   },
};

static int __init bma150_init(void)
{
	printk(KERN_INFO "BMA150 G-sensor driver: init\n");
	return i2c_add_driver(&bma150_driver);
}

static void __exit bma150_exit(void)
{
	i2c_del_driver(&bma150_driver);
}

module_init(bma150_init);
module_exit(bma150_exit);

MODULE_DESCRIPTION("BMA150 G-sensor driver");
MODULE_LICENSE("GPL");

