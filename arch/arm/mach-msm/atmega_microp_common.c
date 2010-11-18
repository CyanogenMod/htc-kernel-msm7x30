/* arch/arm/mach-msm/atmega_microp_common.c
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
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <mach/atmega_microp.h>
#include <asm/mach-types.h>
#include <linux/earlysuspend.h>
#include <mach/drv_callback.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/irq.h>
#include <asm/uaccess.h>
#include "proc_comm.h"


#define I2C_READ_RETRY_TIMES			10
#define I2C_WRITE_RETRY_TIMES			10
#define MICROP_I2C_WRITE_BLOCK_SIZE		80

static struct i2c_client *private_microp_client;
static struct microp_ops *board_ops;

static int microp_rw_delay;

static char *hex2string(uint8_t *data, int len)
{
	static char buf[MICROP_I2C_WRITE_BLOCK_SIZE*4];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int i2c_read_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	struct microp_i2c_client_data *cdata;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	cdata = i2c_get_clientdata(client);
	mutex_lock(&cdata->microp_i2c_rw_mutex);
	hr_msleep(1);
	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msgs, 2) == 2)
			break;
		msleep(microp_rw_delay);
	}
	mutex_unlock(&cdata->microp_i2c_rw_mutex);
	dev_dbg(&client->dev, "R [%02X] = %s\n",
			addr, hex2string(data, length));

	if (retry > I2C_READ_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
	uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int i;
	struct microp_i2c_client_data *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	cdata = i2c_get_clientdata(client);
	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->microp_i2c_rw_mutex);
	hr_msleep(1);
	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(microp_rw_delay);
	}
	if (retry > I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->microp_i2c_rw_mutex);
		return -EIO;
	}
	if (addr == MICROP_I2C_WCMD_LCM_BURST_EN)
		udelay(500);/*1.5ms for microp SPI write */
	mutex_unlock(&cdata->microp_i2c_rw_mutex);

	return 0;
}

int microp_i2c_read(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_read_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_read);

int microp_i2c_write(uint8_t addr, uint8_t *data, int length)
{
	struct i2c_client *client = private_microp_client;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}

	if (i2c_write_block(client, addr, data, length) < 0)	{
		dev_err(&client->dev, "%s: write microp i2c fail\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(microp_i2c_write);

void microp_mobeam_enable(int enable)
{
	if (enable)
		microp_rw_delay = 500;
	else
		microp_rw_delay = 5;
}
EXPORT_SYMBOL(microp_mobeam_enable);

void microp_register_ops(struct microp_ops *ops)
{
	board_ops = ops;
}

int microp_function_check(struct i2c_client *client, uint8_t category)
{
	struct microp_i2c_platform_data *pdata;
	int i, ret = -1;

	pdata = client->dev.platform_data;

	for (i = 0; i < pdata->num_functions; i++) {
		if (pdata->microp_function[i].category == category) {
			ret = i;
			break;
		}
	}
	if (ret < 0)
		pr_err("%s: No function %d !!\n", __func__, category);

	return ret;
}

int microp_write_interrupt(struct i2c_client *client,
		uint16_t interrupt, uint8_t enable)
{
	uint8_t data[2], addr;
	int ret = -1;

	if (enable)
		addr = MICROP_I2C_WCMD_GPI_INT_CTL_EN;
	else
		addr = MICROP_I2C_WCMD_GPI_INT_CTL_DIS;

	data[0] = interrupt >> 8;
	data[1] = interrupt & 0xFF;
	ret = i2c_write_block(client, addr, data, 2);

	if (ret < 0)
		dev_err(&client->dev, "%s: %s 0x%x interrupt failed\n",
			__func__, (enable ? "enable" : "disable"), interrupt);
	return ret;
}

int microp_read_adc(uint8_t *data)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int ret = 0;

	client = private_microp_client;
	cdata = i2c_get_clientdata(client);

	mutex_lock(&cdata->microp_adc_mutex);
	if (i2c_write_block(client, MICROP_I2C_WCMD_READ_ADC_VALUE_REQ,
			data, 2) < 0) {
		dev_err(&client->dev, "%s: request adc fail\n", __func__);
		ret = -EIO;
		goto exit;
	}
	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_RCMD_ADC_VALUE, data, 2) < 0) {
		dev_err(&client->dev, "%s: read adc fail\n", __func__);
		ret = -EIO;
		goto exit;
	}
exit:
	mutex_unlock(&cdata->microp_adc_mutex);
	return ret;
}

EXPORT_SYMBOL(microp_read_adc);

int microp_read_gpio_status(uint8_t *data)
{
	struct i2c_client *client;
	struct microp_i2c_platform_data *pdata;
	int length;

	client = private_microp_client;
	pdata = client->dev.platform_data;

	if (pdata->cmd_diff & CMD_83_DIFF)
		length = 2;
	else
		length = 3;
	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_RCMD_GPIO_STATUS,
			data, length) < 0) {
		dev_err(&client->dev, "%s: read gpio status fail\n", __func__);
		return -EIO;
	}
	return 0;
}

static void microp_pm_power_off(struct i2c_client *client)
{
	return;
}

static void microp_reset_system(void)
{
	return;
}

static int microp_oj_intr_enable(struct i2c_client *client, uint8_t enable)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);
	enable = enable ? 1 : 0;
	return microp_write_interrupt(client,
			cdata->int_pin.int_oj, enable);
}

static int microp_spi_enable(struct i2c_client *client, uint8_t enable)
{
	uint8_t data;
	int ret = 0;

	data = enable ? 1 : 0;
	ret = i2c_write_block(client, MICROP_I2C_WCMD_SPI_EN, &data, 1);
	if (ret != 0)
		printk(KERN_ERR "%s: set SPI %s fail\n", __func__,
			(enable ? "enable" : "disable"));

	return ret;
}

int microp_spi_vote_enable(int spi_device, uint8_t enable)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[2] = {0, 0};
	int ret = 0;

	if (!client)	{
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return -EIO;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	if (spi_device == SPI_OJ)
		microp_oj_intr_enable(client, enable);

	mutex_lock(&cdata->microp_adc_mutex);
	if (enable)
		cdata->spi_devices_vote |= spi_device;
	else
		cdata->spi_devices_vote &= ~spi_device;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_SPI_BL_STATUS, data, 2);
	if (ret != 0) {
		printk(KERN_ERR "%s: read SPI/BL status fail\n", __func__);
		goto exit;
	}

	if ((data[1] & 0x01) ==
		((pdata->spi_devices & cdata->spi_devices_vote) ? 1 : 0))
		goto exit;

	if (pdata->spi_devices & cdata->spi_devices_vote)
		enable = 1;
	else
		enable = 0;
	mutex_unlock(&cdata->microp_adc_mutex);

	ret = microp_spi_enable(client, enable);
	return ret;

exit:
	mutex_unlock(&cdata->microp_adc_mutex);
	return ret;

}

EXPORT_SYMBOL(microp_spi_vote_enable);

static void microp_reset_microp(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;

	pdata = client->dev.platform_data;

	gpio_set_value(pdata->gpio_reset, 0);
	udelay(120);
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(5);
}

static ssize_t microp_version_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%04X\n", cdata->version);
}

static DEVICE_ATTR(version, 0644, microp_version_show, NULL);

static ssize_t microp_reset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int val;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val != 1)
		return -EINVAL;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	microp_reset_microp(client);
	if (board_ops->init_microp_func)
		board_ops->init_microp_func(client);

	return count;
}

static DEVICE_ATTR(reset, 0644, NULL, microp_reset_store);

static ssize_t microp_gpio_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t data[3] = {0, 0, 0};
	int ret;

	microp_read_gpio_status(data);
	ret = sprintf(buf, "PB = 0x%x, PC = 0x%x, PD = 0x%x\n",
				data[0], data[1], data[2]);

	return ret;
}

static ssize_t microp_gpio_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;
	int enable = 0, tmp[3] = {0, 0, 0};
	uint8_t addr, data[3] = {0, 0, 0};

	sscanf(buf, "%d %d %d %d", &enable, &tmp[0], &tmp[1], &tmp[2]);

	if (enable != 0 && enable != 1)
		return -EINVAL;

	client = to_i2c_client(dev);
	cdata = i2c_get_clientdata(client);

	if (enable)
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_EN;
	else
		addr = MICROP_I2C_WCMD_GPO_LED_STATUS_DIS;
	data[0] = (uint8_t)tmp[0];
	data[1] = (uint8_t)tmp[1];
	data[2] = (uint8_t)tmp[2];
	i2c_write_block(client, addr, data, 3);

	return count;
}

static DEVICE_ATTR(gpio, 0644,  microp_gpio_show,
			microp_gpio_store);

static irqreturn_t microp_intr_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client;
	struct microp_i2c_client_data *cdata;

	client = to_i2c_client(dev_id);
	cdata = i2c_get_clientdata(client);

	disable_irq_nosync(client->irq);
	queue_work(cdata->microp_queue, &cdata->microp_intr_work);
	return IRQ_HANDLED;
}

static void microp_int_dispatch(u32 status)
{
	unsigned int mask;
	int irq;

	while (status) {
		mask = status & -status;
		irq = fls(mask) - 1;
		status &= ~mask;
		generic_handle_irq(FIRST_MICROP_IRQ + irq);
	}
}

static enum hrtimer_restart hr_dispath_irq_func(struct hrtimer *data)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);
	microp_int_dispatch(cdata->intr_status);
	cdata->intr_status = 0;
	return HRTIMER_NORESTART;
}


static void microp_intr_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;
	uint8_t data[3];
	uint16_t intr_status = 0;
	int sd_insert = 0;
	ktime_t zero_debounce;

	zero_debounce = ktime_set(0, 0);  /* No debounce time */

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}

	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	memset(data, 0x00, sizeof(data));
	if (i2c_read_block(client, MICROP_I2C_RCMD_GPI_INT_STATUS,
			data, 2) < 0)
		dev_err(&client->dev, "%s: read interrupt status fail\n",
				__func__);
	intr_status = data[0]<<8 | data[1];
	if (i2c_write_block(client, MICROP_I2C_WCMD_GPI_INT_STATUS_CLR,
			data, 2) < 0)
		dev_err(&client->dev, "%s: clear interrupt status fail\n",
				__func__);

	if (intr_status & cdata->int_pin.int_reset) {
		dev_info(&client->dev, "Reset button is pressed\n");
		microp_reset_system();
	}
	if (intr_status & cdata->int_pin.int_simcard) {
		dev_info(&client->dev, "SIM Card is plugged/unplugged\n");
		microp_pm_power_off(client);
	}

	if (intr_status & cdata->int_pin.int_sdcard) {
		dev_info(&client->dev, "SD Card is plugged/unplugged\n");
		msleep(300);
		microp_read_gpio_status(data);
		sd_insert = ((data[0] << 16 | data[1] << 8 | data[2])
				& cdata->gpio.sdcard) ? 1 : 0;
		if (sd_insert != cdata->sdcard_is_in) {
			cdata->sdcard_is_in = sd_insert;
			cnf_driver_event("sdcard_detect", &cdata->sdcard_is_in);
		}
	}

	cdata->intr_status = intr_status;
	hrtimer_start(&cdata->gen_irq_timer, zero_debounce, HRTIMER_MODE_REL);
	enable_irq(client->irq);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void microp_early_suspend(struct early_suspend *h)
{
	struct microp_i2c_client_data *cdata;
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_platform_data *pdata;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	atomic_set(&cdata->microp_is_suspend, 1);
}

static void microp_late_resume(struct early_suspend *h)
{
	struct i2c_client *client = private_microp_client;
	struct microp_i2c_client_data *cdata;
	struct microp_i2c_platform_data *pdata;

	if (!client) {
		printk(KERN_ERR "%s: dataset: client is empty\n", __func__);
		return;
	}
	cdata = i2c_get_clientdata(client);
	pdata = client->dev.platform_data;

	atomic_set(&cdata->microp_is_suspend, 0);
}
#endif

static int __devexit microp_i2c_remove(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;

	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cdata->early_suspend);
#endif

	if (client->irq)
		free_irq(client->irq, &client->dev);

	gpio_free(pdata->gpio_reset);

	device_remove_file(&client->dev, &dev_attr_reset);
	device_remove_file(&client->dev, &dev_attr_version);
	device_remove_file(&client->dev, &dev_attr_gpio);
	destroy_workqueue(cdata->microp_queue);
	kfree(cdata);

	return 0;
}

static int microp_i2c_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
	return 0;
}

static int microp_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static void register_microp_devices(struct platform_device *devices, int num)
{
	int i;
	for (i = 0; i < num; i++) {
		platform_device_register(devices + i);
		dev_set_drvdata(&(devices + i)->dev, private_microp_client);
	}
}

static int microp_i2c_probe(struct i2c_client *client
	, const struct i2c_device_id *id)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	uint8_t data[6];
	int ret;

	cdata = kzalloc(sizeof(struct microp_i2c_client_data), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);

	mutex_init(&cdata->microp_adc_mutex);
	mutex_init(&cdata->microp_i2c_rw_mutex);

	private_microp_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		ret = -EBUSY;
		dev_err(&client->dev, "failed on get pdata\n");
		goto err_exit;
	}
	pdata->dev_id = (void *)&client->dev;
	microp_rw_delay = 5;

	ret = i2c_read_block(client, MICROP_I2C_RCMD_VERSION, data, 2);
	if (ret || !(data[0] && data[1])) {
		ret = -ENODEV;
		dev_err(&client->dev, "failed on get microp version\n");
		goto err_exit;
	}
	dev_info(&client->dev, "microp version [%02X][%02X]\n",
			data[0], data[1]);

	ret = gpio_request(pdata->gpio_reset, "atmega_microp");
	if (ret < 0) {
		dev_err(&client->dev, "failed on request gpio reset\n");
		goto err_exit;
	}
	ret = gpio_direction_output(pdata->gpio_reset, 1);
	if (ret < 0) {
		dev_err(&client->dev,
				"failed on gpio_direction_output reset\n");
		goto err_gpio_reset;
	}

	cdata->version = data[0] << 8 | data[1];
	atomic_set(&cdata->microp_is_suspend, 0);

	cdata->spi_devices_vote = pdata->spi_devices_init;

	cdata->intr_status = 0;
	hrtimer_init(&cdata->gen_irq_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cdata->gen_irq_timer.function = hr_dispath_irq_func;

	cdata->microp_queue = create_singlethread_workqueue("microp_work_q");
	if (cdata->microp_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}

	if (client->irq) {
		INIT_WORK(&cdata->microp_intr_work, microp_intr_work_func);

		ret = request_irq(client->irq, microp_intr_irq_handler,
			IRQF_TRIGGER_LOW, "microp_intrrupt",
			&client->dev);
		if (ret) {
			dev_err(&client->dev, "request_irq failed\n");
			goto err_intr;
		}
		ret = set_irq_wake(client->irq, 1);
		if (ret) {
			dev_err(&client->dev, "set_irq_wake failed\n");
			goto err_intr;
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	cdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	cdata->early_suspend.suspend = microp_early_suspend;
	cdata->early_suspend.resume = microp_late_resume;
	register_early_suspend(&cdata->early_suspend);
#endif
	ret = device_create_file(&client->dev, &dev_attr_reset);
	ret = device_create_file(&client->dev, &dev_attr_version);
	ret = device_create_file(&client->dev, &dev_attr_gpio);

	register_microp_devices(pdata->microp_devices, pdata->num_devices);
	if (board_ops->init_microp_func) {
		ret = board_ops->init_microp_func(client);
		if (ret) {
			dev_err(&client->dev,
				"failed on microp function initialize\n");
			goto err_fun_init;
		}
	}

	return 0;

err_fun_init:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&cdata->early_suspend);
#endif
	device_remove_file(&client->dev, &dev_attr_reset);
	device_remove_file(&client->dev, &dev_attr_version);
	device_remove_file(&client->dev, &dev_attr_gpio);
	destroy_workqueue(cdata->microp_queue);
err_intr:
err_create_work_queue:
	kfree(cdata);
err_gpio_reset:
	gpio_free(pdata->gpio_reset);
err_exit:
	private_microp_client = NULL;
err_cdata:
	return ret;
}

static const struct i2c_device_id microp_i2c_id[] = {
	{ MICROP_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver microp_i2c_driver = {
	.driver = {
		   .name = MICROP_I2C_NAME,
		   },
	.id_table = microp_i2c_id,
	.probe = microp_i2c_probe,
	.suspend = microp_i2c_suspend,
	.resume = microp_i2c_resume,
	.remove = __devexit_p(microp_i2c_remove),
};

static void microp_irq_ack(unsigned int irq)
{
	;
}

static void microp_irq_mask(unsigned int irq)
{
	;
}

static void microp_irq_unmask(unsigned int irq)
{
	;
}

static struct irq_chip microp_irq_chip = {
	.name = "microp",
	.disable = microp_irq_mask,
	.ack = microp_irq_ack,
	.mask = microp_irq_mask,
	.unmask = microp_irq_unmask,
};

static int __init microp_common_init(void)
{
	int ret;
	int n, MICROP_IRQ_END = FIRST_MICROP_IRQ + NR_MICROP_IRQS;

	for (n = FIRST_MICROP_IRQ; n < MICROP_IRQ_END; n++) {
		set_irq_chip(n, &microp_irq_chip);
		set_irq_handler(n, handle_level_irq);
		set_irq_flags(n, IRQF_VALID);
	}

	ret = i2c_add_driver(&microp_i2c_driver);
	if (ret)
		return ret;
	return 0;
}

static void __exit microp_common_exit(void)
{
	i2c_del_driver(&microp_i2c_driver);
}

module_init(microp_common_init);
module_exit(microp_common_exit);

MODULE_AUTHOR("Eric Huang <Eric.SP_Huang@htc.com>");
MODULE_DESCRIPTION("Atmega MicroP driver");
MODULE_LICENSE("GPL");
