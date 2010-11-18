/* linux/driver/spi/spi_oj.c
 *
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
 *
 *
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>

#define tSRAD	160
#define tSWW	30

#define OJ_MOTION              (26)
#define OJ_SHUTDOWN            (35)
#define OJ_RST                 (36)

/* #define OJ_DEBUG */
static struct spi_device        *spidev;
struct work_struct oj_work;

u8 oj_reg_read(u8 reg)
{
	u8 val;
	struct spi_msg msg;
	int err = 0;
	if (!spidev)
		return 0;
	udelay(1);

	msg.buffer[0] = reg;
	msg.len = 0;
	err += spi_read_write_lock(spidev, &msg, NULL, 1, 1);
	udelay(tSRAD);
	err += spi_read_write_lock(spidev, NULL, &val, 1, 0);
#ifdef OJ_DEBUG
	if (err != 0)
		printk("oj_reg_read fail\n");
#endif
	return val;
}
EXPORT_SYMBOL(oj_reg_read);

int oj_reg_write(u8 reg, u8 val)
{
	struct spi_msg msg;
	int err = 0;
	if (!spidev)
		return 0;
#ifdef OJ_DEBUG
	printk(KERN_INFO "write %x to reg%x\n", val, reg);
#endif
	msg.buffer[0] = reg | 0x80;
	msg.buffer[1] = val;
	msg.len = 0;
	udelay(1);
	err += spi_read_write_lock(spidev, &msg, NULL, 2, 1);
#ifdef OJ_DEBUG
	if (err != 0)
		printk("oj_reg_write fail\n");
#endif
	return 0;
}
EXPORT_SYMBOL(oj_reg_write);

int oj_burst_read(char *buf, int len)
{
	struct spi_msg msg;
	int err = 0;
	if (!spidev)
		return 0;

	msg.buffer[0] = 0x02;
	msg.len = 0;
	udelay(1);
	err += spi_read_write_lock(spidev, &msg, NULL, 1, 1);
	udelay(tSRAD);
	err += spi_read_write_lock(spidev, NULL, buf, len, 0);
#ifdef OJ_DEBUG
	if (err != 0)
		printk("oj_burst_read fail\n");
#endif
	return 0;
}
EXPORT_SYMBOL(oj_burst_read);

#ifdef OJ_DEBUG
static irqreturn_t oj_interrupt(int irq, void *data)
{
	int motion;
	/* printk("oj interrupt\n");*/
	motion = gpio_get_value(OJ_MOTION);
	if (motion)
		return IRQ_HANDLED;
	schedule_work(&oj_work);
	return IRQ_HANDLED;
}

static void oj_do_work(struct work_struct *w)
{
	char motion_val;
	printk(KERN_INFO "%s\n", __func__);
	printk(KERN_INFO "reg02 = 0x%x\n", oj_reg_read(0x02));
	printk(KERN_INFO "reg03 = 0x%x\n", oj_reg_read(0x03));
	printk(KERN_INFO "reg04 = 0x%x\n", oj_reg_read(0x04));
	motion_val = oj_reg_read(0x02);
	if (motion_val & (1 << 7)) {
		printk(KERN_INFO "reg02 = 0x%x\n", oj_reg_read(0x02));
		printk(KERN_INFO "reg03 = 0x%x\n", oj_reg_read(0x03));
		printk(KERN_INFO "reg04 = 0x%x\n", oj_reg_read(0x04));
	}

	printk(KERN_INFO "gpio26 = %d\n", gpio_get_value(26));
}
#endif

static int oj_spi_probe(struct spi_device *spi)
{
#ifdef OJ_DEBUG
	int irq_oj;
	int ret;
#endif
	printk(KERN_INFO "%s \n", __func__);
	spidev = spi;

#ifdef OJ_DEBUG
	gpio_set_value(OJ_RST, 0);
	udelay(20);
	gpio_set_value(OJ_RST, 1);
	mdelay(10);
	oj_reg_write(0x3a, 0x5a);
	mdelay(23);
	printk(KERN_INFO "reg02 = 0x%x\n", oj_reg_read(0x02));
	printk(KERN_INFO "reg03 = 0x%x\n", oj_reg_read(0x03));
	printk(KERN_INFO "reg04 = 0x%x\n", oj_reg_read(0x04));
	printk(KERN_INFO "reg00 = 0x%x\n", oj_reg_read(0x00));
	printk(KERN_INFO "gpio26 = %d\n", gpio_get_value(26));

	ret = gpio_request(OJ_MOTION, "OJ irq");
	if (ret < 0) {
		printk(KERN_ERR "gpio_request error\n");
		return 0;
	}

	ret = gpio_direction_input(OJ_MOTION);
	if (ret < 0) {
		printk(KERN_ERR "gpio_drection_input eror\n");
		gpio_free(OJ_MOTION);
		return 0;
	}

	irq_oj = gpio_to_irq(OJ_MOTION);
	if (irq_oj < 0) {
		printk(KERN_ERR "gpio_to_irq error\n");
		gpio_free(OJ_MOTION);
		return 0;
	}

	ret = request_irq(irq_oj, oj_interrupt,
				IRQF_TRIGGER_FALLING, "OJ irq", NULL);
	if (ret < 0) {
		printk(KERN_ERR "%s(): request_irq  fail\n", __func__);
		gpio_free(OJ_MOTION);
	}
	INIT_WORK(&oj_work, oj_do_work);
#endif
	return 0 ;
}

static int __exit oj_spi_remove(struct spi_device *spi)
{
	spidev = NULL;
	return 0;
}


static struct spi_driver spi_oj = {
	.driver = {
		.name  = "spi_oj",
		.owner = THIS_MODULE,
	},
	.probe         = oj_spi_probe,
	.remove        = __exit_p(oj_spi_remove),
};

static int __init spi_oj_init(void)
{
	int rc;
	rc = spi_register_driver(&spi_oj);
	return rc;
}
module_init(spi_oj_init);

static void __exit spi_oj_exit(void)
{
	spi_unregister_driver(&spi_oj);
}
module_exit(spi_oj_exit);

#ifdef OJ_DEBUG
static int oj_set_debug(const char *val, struct kernel_param *kp)
{
	printk(KERN_INFO "reg02 = 0x%x\n", oj_reg_read(0x02));
	printk(KERN_INFO "reg03 = 0x%x\n", oj_reg_read(0x03));
	printk(KERN_INFO "reg04 = 0x%x\n", oj_reg_read(0x04));
	printk(KERN_INFO "reg00 = 0x%x\n", oj_reg_read(0x00));
	printk(KERN_INFO "gpio26 = %d\n", gpio_get_value(26));
	return 0;
}

static int oj_get_debug(char *buffer, struct kernel_param *kp)
{
	oj_reg_write(0x02, 0);
	return 1;
}

//module_param_call(debug, oj_set_debug, oj_get_debug, 0, 0664);
#endif /*OJ_DEBUG*/
