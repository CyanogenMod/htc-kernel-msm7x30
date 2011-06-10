/* drivers/usb/gadget/smsc251x.c - SMSC usb hub driver
 *
 * Copyright (C) 2009 HTC Corporation.
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
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <linux/jiffies.h>
#include <mach/msm_hsusb.h>
#include <mach/smsc251x.h>
#include <linux/slab.h>


/*#define HTC_HUB_DEBUG*/

#define I2C_READ_RETRY_TIMES			10
#define I2C_WRITE_RETRY_TIMES			10
#define MICROP_I2C_WRITE_BLOCK_SIZE		80

/*Manufacture: HTC*/
uint8_t manufac_str[MANUFACUTRE_STRING_LEN] = {0x48, 0x0, 0x54, 0x0, 0x43, 0x0};
/*Product: ADR6400L*/
int8_t product_str[PRODUCT_STRING_LEN] = {0x41, 0x0, 0x44, 0x0, 0x52, 0x0, 0x36, 0x0, 0x34, 0x0, 0x30, 0x0, 0x30, 0x0, 0x4c};
/*Serial: HTC_Mecha*/
uint8_t serial_str[SERIAL_STRING_LEN] = {0x48, 0x0, 0x54, 0x0, 0x43, 0x0, 0x5f, 0x0, 0x4d, 0x0, 0x65, 0x0, 0x63, 0x0, 0x68, 0x0, 0x61};

struct smsc251x_data {
	struct i2c_client *client;
};

static struct smsc251x_data *private_data;
static DEFINE_MUTEX(smsc251x_i2c_rw_mutex);
static int diag_boot_enable;
static int hub_status;
static int mdm_port_status;
static int hub_ctrl_by_qb;

static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}
int i2c_smsc251x_read(uint8_t address, uint8_t *data, uint8_t length)
{
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = private_data->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &address,
		},
		{
			.addr = private_data->client->addr,
			.flags = I2C_M_RD,
			.len = length+1,
			.buf = data,
		}
	};
	mutex_lock(&smsc251x_i2c_rw_mutex);
	for (retry = 0; retry < I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(private_data->client->adapter, msg, 2) == 2) {
			printk(KERN_ERR"R [%02X] = %s\n", address, hex2string(data, length+1));
			break;
		}
		hr_msleep(5);
	}
	if (retry == I2C_READ_RETRY_TIMES) {
		printk(KERN_ERR "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
			mutex_unlock(&smsc251x_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&smsc251x_i2c_rw_mutex);
	return 0;

}

int i2c_smsc251x_write(uint8_t address, uint8_t *data, uint8_t length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];

	struct i2c_msg msg[] = {
		{
			.addr = private_data->client->addr,
			.flags = 0,
			.len = length + 2,
			.buf = buf,
		}
	};

	printk(KERN_DEBUG"%s: slave=%x, addr=%x, length=%d data=%d\n", __func__, private_data->client->addr, address, length, *data);

	printk(KERN_DEBUG"W [%02X] = %s\n", address, hex2string(data, length));
	buf[0] = address;
	buf[1] = length;
	memcpy((void *)&buf[2], (void *)data, length);

	mutex_lock(&smsc251x_i2c_rw_mutex);
	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(private_data->client->adapter, msg, 1) == 1)
			break;
		hr_msleep(5);
	}

	if (retry == I2C_WRITE_RETRY_TIMES) {
		printk(KERN_ERR "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&smsc251x_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&smsc251x_i2c_rw_mutex);
	return 0;
}

void smsc251x_set_diag_boot_flag(int enable)
{
	diag_boot_enable = enable;
}

static int smsc251x_switch(uint8_t hub_enable, uint8_t mdm_port_on)
{
	struct smsc251x_platform_data *pdata;
	uint8_t data[9] = {0x24, 0x4, 0x12, 0x25, 0xa0, 0xa, 0x8b, 0x20, 0x2};
	uint8_t data1[6] = {0x1, 0x32, 0x1, 0x32, 0x32, 0x0};
	uint8_t data2[1] = {0x1};
	uint8_t data3[1];
	uint8_t ptr[10];
	uint8_t port_status;
	uint8_t temp[1];

	struct i2c_client *client = private_data->client;
	pdata = client->dev.platform_data;
	memset(ptr, 0, sizeof(ptr));

	printk(KERN_INFO "%s:mfg mode: %d\n", __func__, board_mfg_mode());
	if (board_mfg_mode() == 5) {
		printk(KERN_INFO "%s: not control usb hub in offmode charging\n", __func__);
		return 1;
	}
	if (pdata) {
		if (hub_enable) {
			pdata->usb_hub_gpio_config(0);
			mdelay(5);
			pdata->usb_hub_gpio_config(1);
			/* timing budget to make sure PMGPIO is ready
			   before demanding i2c */
			mdelay(10);

			i2c_smsc251x_write(SMSC_VENDOR_ID, data, 9);
			i2c_smsc251x_read(SMSC_VENDOR_ID, ptr, 9);
			i2c_smsc251x_write(SMSC_MAX_POWER_SELF, data1, 6);
			i2c_smsc251x_read(SMSC_MAX_POWER_SELF, ptr, 6);

			printk(KERN_INFO "%s: mdm port control %d:%d:%d\n",
					__func__, mdm_port_on,
					mdm_port_status, diag_boot_enable);

			i2c_smsc251x_read(SMSC_PORT_DISABLE_SELF, ptr, 1);
			port_status = ptr[1];
			if (mdm_port_on || mdm_port_status || diag_boot_enable)
				data3[0] = 0x00;
			else
				data3[0] = (port_status | (1 << 2));

			i2c_smsc251x_read(SMSC_CONFIG_DATA_3, ptr, 1);
			/*String descriptor support*/
			temp[0] = (0x1 | ptr[1]);
			i2c_smsc251x_write(SMSC_CONFIG_DATA_3, temp, 1);
			temp[0] = MANUFACUTRE_STRING_LEN;
			i2c_smsc251x_write(SMSC_MANUFACTURER_STRING_LEN, temp, 1);
			temp[0] = PRODUCT_STRING_LEN;
			i2c_smsc251x_write(SMSC_PRODUCT_STRING_LEN, temp, 1);
			temp[0] = SERIAL_STRING_LEN;
			i2c_smsc251x_write(SMSC_SERIAL_STRING_LEN, temp, 1);

			i2c_smsc251x_write(SMSC_MANUFACTURER_STRING, manufac_str, MANUFACUTRE_STRING_LEN);
			i2c_smsc251x_write(SMSC_PRODUCT_STRING, product_str, PRODUCT_STRING_LEN);
			i2c_smsc251x_write(SMSC_SERIAL_STRING, serial_str, SERIAL_STRING_LEN);

			i2c_smsc251x_write(SMSC_PORT_DISABLE_SELF, data3, 1);

			i2c_smsc251x_write(SMSC_STATUS_CMD, data2, 1);
		} else {
			pdata->usb_hub_gpio_config(0);
		}
		hub_status = hub_enable;
		return 0;
	} else
		return 1;
}

int smsc251x_mdm_port_sw(uint8_t enable)
{
	/* once user disable port via ##3424, clear boot config */
	if (!enable)
		diag_boot_enable = 0;

	mdm_port_status = enable;
	return smsc251x_switch(1, enable);
}

static int smsc251x_usb_hub_switch(uint8_t enable)
{
	return smsc251x_switch(enable, 0);
}

static int smsc251x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct smsc251x_platform_data *pdata;

	printk("%s: slave=%x\n", __func__, client->addr);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR"%s: need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	private_data = kzalloc(sizeof(struct smsc251x_data), GFP_KERNEL);
	if (private_data == NULL) {
		printk(KERN_ERR"%s: allocate atmel_ts_data failed\n", __func__);
		return -ENOMEM;
	}
	private_data->client = client;
	i2c_set_clientdata(client, private_data);

	pdata = client->dev.platform_data;
	if (pdata && pdata->register_switch_func)
		pdata->register_switch_func(smsc251x_usb_hub_switch);

	if (board_mfg_mode() == 5) {
		printk(KERN_INFO "%s: disable usb hub in offmode charging\n", __func__);
		if (pdata->usb_hub_gpio_config)
			pdata->usb_hub_gpio_config(0);
		return 1;
	}

	return 0;
}

static int smsc251x_remove(struct i2c_client *client)
{
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

	kfree(ts);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

#endif

static const struct i2c_device_id smsc251x_i2c_id[] = {
	{ SMSC251X_NAME, 0 },
	{ }
};

static struct i2c_driver smsc251x_driver = {
	.id_table = smsc251x_i2c_id,
	.probe = smsc251x_probe,
	.remove = smsc251x_remove,
/*#ifndef CONFIG_HAS_EARLYSUSPEND*/
#if 0
	.suspend = smsc251x_suspend,
	.resume = smsc251x_resume,
#endif
	.driver = {
			.name = SMSC251X_NAME,
	},
};

static int hub_set_enabled(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);

	printk(KERN_INFO "%s: %d\n", __func__, enabled);
	smsc251x_usb_hub_switch(enabled);
	return 0;
}

static int hub_get_enabled(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + hub_status;
	return 1;
}
module_param_call(hub_enabled, hub_set_enabled, hub_get_enabled, NULL, 0664);

static int hub_set_enabled_qb(const char *val, struct kernel_param *kp)
{
	int enabled = simple_strtol(val, NULL, 0);

	printk(KERN_INFO "%s: %d\n", __func__, enabled);
	if (enabled) {
		if (!hub_ctrl_by_qb) {
			printk(KERN_ERR "%s: Quick boot is not set"
					" when enable hub \n", __func__);
			return -EIO;
		}
		hub_ctrl_by_qb = 0;
	} else {
		if (hub_ctrl_by_qb) {
			printk(KERN_ERR "%s: Quick boot is already set"
					" when disable hub\n", __func__);
			return -EIO;
		}
		hub_ctrl_by_qb = 1;
	}
	smsc251x_usb_hub_switch(enabled);
	return 0;
}

static int hub_get_enabled_qb(char *buffer, struct kernel_param *kp)
{
	buffer[0] = '0' + hub_status;
	return 1;
}
module_param_call(hub_enabled_quickboot, hub_set_enabled_qb,
					hub_get_enabled_qb, NULL, 0664);

#ifdef HTC_HUB_DEBUG
static ssize_t show_hub_sw(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	char log[128];
	int i, p;
	uint8_t result[32];

	i2c_smsc251x_read(0, result, 16);
	for (i = 1, p = 0; i < 16 ; i++)	{
		p += sprintf(log+p, "%x,",  result[i]);
	}
	return sprintf(buf, "result:%s\n", log);
}

static ssize_t store_hub_sw(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	char *buffer, *endptr;
	uint8_t addr;
	uint8_t data;
	uint8_t result[2];
	buffer = (char *)buf;

	data = 0x00;
	i2c_smsc251x_write(0xff, &data, 1);

	addr = simple_strtoull(buffer, &endptr, 16);
	buffer = endptr+1;

	data = simple_strtoull(buffer, &endptr, 16);



	i2c_smsc251x_write(addr, &data, 1);
	i2c_smsc251x_read(addr, result, 1);
	if (data != result[1])
		printk("write fail\n");
	else
		printk("write okay\n");
	return count;
}

static DEVICE_ATTR(hub_sw, 0644, show_hub_sw, store_hub_sw);

static void diag_plat_release(struct device *dev) {}

static struct platform_device diag_plat_device = {
	.name		= "hub",
	.id		= -1,
	.dev		= {
		.release	= diag_plat_release,
	},
};
#endif


static int __devinit smsc251x_init(void)
{
	printk("%s\n", __func__);

#ifdef HTC_HUB_DEBUG
	platform_device_register(&diag_plat_device);
	if (device_create_file(&(diag_plat_device.dev), &dev_attr_hub_sw) != 0) {
		printk(KERN_ERR "diag dev_attr_diag_port_switch failed");
	}
#endif

	return i2c_add_driver(&smsc251x_driver);
}

static void __exit smsc251x_exit(void)
{
	i2c_del_driver(&smsc251x_driver);
}
module_init(smsc251x_init);
module_exit(smsc251x_exit);

MODULE_DESCRIPTION("SMSC-251x usb hub driver");
MODULE_LICENSE("GPL");


