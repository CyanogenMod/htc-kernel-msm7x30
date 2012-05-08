/*
 * arch/arm/plat-omap/include/mach/gpio.h
 *
 * OMAP GPIO handling defines and functions
 *
 * Copyright (C) 2003-2005 Nokia Corporation
 *
 * Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __ASM_ARCH_OMAP_GPIO_H
#define __ASM_ARCH_OMAP_GPIO_H

#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>

#define OMAP1_MPUIO_BASE			0xfffb5000

/*
 * These are the omap15xx/16xx offsets. The omap7xx offset are
 * OMAP_MPUIO_ / 2 offsets below.
 */
#define OMAP_MPUIO_INPUT_LATCH		0x00
#define OMAP_MPUIO_OUTPUT		0x04
#define OMAP_MPUIO_IO_CNTL		0x08
#define OMAP_MPUIO_KBR_LATCH		0x10
#define OMAP_MPUIO_KBC			0x14
#define OMAP_MPUIO_GPIO_EVENT_MODE	0x18
#define OMAP_MPUIO_GPIO_INT_EDGE	0x1c
#define OMAP_MPUIO_KBD_INT		0x20
#define OMAP_MPUIO_GPIO_INT		0x24
#define OMAP_MPUIO_KBD_MASKIT		0x28
#define OMAP_MPUIO_GPIO_MASKIT		0x2c
#define OMAP_MPUIO_GPIO_DEBOUNCING	0x30
#define OMAP_MPUIO_LATCH		0x34

#define OMAP34XX_NR_GPIOS		6

/*
 * OMAP1510 GPIO registers
 */
#define OMAP1510_GPIO_DATA_INPUT	0x00
#define OMAP1510_GPIO_DATA_OUTPUT	0x04
#define OMAP1510_GPIO_DIR_CONTROL	0x08
#define OMAP1510_GPIO_INT_CONTROL	0x0c
#define OMAP1510_GPIO_INT_MASK		0x10
#define OMAP1510_GPIO_INT_STATUS	0x14
#define OMAP1510_GPIO_PIN_CONTROL	0x18

#define OMAP1510_IH_GPIO_BASE		64

/*
 * OMAP1610 specific GPIO registers
 */
#define OMAP1610_GPIO_REVISION		0x0000
#define OMAP1610_GPIO_SYSCONFIG		0x0010
#define OMAP1610_GPIO_SYSSTATUS		0x0014
#define OMAP1610_GPIO_IRQSTATUS1	0x0018
#define OMAP1610_GPIO_IRQENABLE1	0x001c
#define OMAP1610_GPIO_WAKEUPENABLE	0x0028
#define OMAP1610_GPIO_DATAIN		0x002c
#define OMAP1610_GPIO_DATAOUT		0x0030
#define OMAP1610_GPIO_DIRECTION		0x0034
#define OMAP1610_GPIO_EDGE_CTRL1	0x0038
#define OMAP1610_GPIO_EDGE_CTRL2	0x003c
#define OMAP1610_GPIO_CLEAR_IRQENABLE1	0x009c
#define OMAP1610_GPIO_CLEAR_WAKEUPENA	0x00a8
#define OMAP1610_GPIO_CLEAR_DATAOUT	0x00b0
#define OMAP1610_GPIO_SET_IRQENABLE1	0x00dc
#define OMAP1610_GPIO_SET_WAKEUPENA	0x00e8
#define OMAP1610_GPIO_SET_DATAOUT	0x00f0

/*
 * OMAP7XX specific GPIO registers
 */
#define OMAP7XX_GPIO_DATA_INPUT		0x00
#define OMAP7XX_GPIO_DATA_OUTPUT	0x04
#define OMAP7XX_GPIO_DIR_CONTROL	0x08
#define OMAP7XX_GPIO_INT_CONTROL	0x0c
#define OMAP7XX_GPIO_INT_MASK		0x10
#define OMAP7XX_GPIO_INT_STATUS		0x14

/*
 * omap2+ specific GPIO registers
 */
#define OMAP24XX_GPIO_REVISION		0x0000
#define OMAP24XX_GPIO_IRQSTATUS1	0x0018
#define OMAP24XX_GPIO_IRQSTATUS2	0x0028
#define OMAP24XX_GPIO_IRQENABLE2	0x002c
#define OMAP24XX_GPIO_IRQENABLE1	0x001c
#define OMAP24XX_GPIO_WAKE_EN		0x0020
#define OMAP24XX_GPIO_CTRL		0x0030
#define OMAP24XX_GPIO_OE		0x0034
#define OMAP24XX_GPIO_DATAIN		0x0038
#define OMAP24XX_GPIO_DATAOUT		0x003c
#define OMAP24XX_GPIO_LEVELDETECT0	0x0040
#define OMAP24XX_GPIO_LEVELDETECT1	0x0044
#define OMAP24XX_GPIO_RISINGDETECT	0x0048
#define OMAP24XX_GPIO_FALLINGDETECT	0x004c
#define OMAP24XX_GPIO_DEBOUNCE_EN	0x0050
#define OMAP24XX_GPIO_DEBOUNCE_VAL	0x0054
#define OMAP24XX_GPIO_CLEARIRQENABLE1	0x0060
#define OMAP24XX_GPIO_SETIRQENABLE1	0x0064
#define OMAP24XX_GPIO_CLEARWKUENA	0x0080
#define OMAP24XX_GPIO_SETWKUENA		0x0084
#define OMAP24XX_GPIO_CLEARDATAOUT	0x0090
#define OMAP24XX_GPIO_SETDATAOUT	0x0094

#define OMAP4_GPIO_REVISION		0x0000
#define OMAP4_GPIO_EOI			0x0020
#define OMAP4_GPIO_IRQSTATUSRAW0	0x0024
#define OMAP4_GPIO_IRQSTATUSRAW1	0x0028
#define OMAP4_GPIO_IRQSTATUS0		0x002c
#define OMAP4_GPIO_IRQSTATUS1		0x0030
#define OMAP4_GPIO_IRQSTATUSSET0	0x0034
#define OMAP4_GPIO_IRQSTATUSSET1	0x0038
#define OMAP4_GPIO_IRQSTATUSCLR0	0x003c
#define OMAP4_GPIO_IRQSTATUSCLR1	0x0040
#define OMAP4_GPIO_IRQWAKEN0		0x0044
#define OMAP4_GPIO_IRQWAKEN1		0x0048
#define OMAP4_GPIO_IRQENABLE1		0x011c
#define OMAP4_GPIO_WAKE_EN		0x0120
#define OMAP4_GPIO_IRQSTATUS2		0x0128
#define OMAP4_GPIO_IRQENABLE2		0x012c
#define OMAP4_GPIO_CTRL			0x0130
#define OMAP4_GPIO_OE			0x0134
#define OMAP4_GPIO_DATAIN		0x0138
#define OMAP4_GPIO_DATAOUT		0x013c
#define OMAP4_GPIO_LEVELDETECT0		0x0140
#define OMAP4_GPIO_LEVELDETECT1		0x0144
#define OMAP4_GPIO_RISINGDETECT		0x0148
#define OMAP4_GPIO_FALLINGDETECT	0x014c
#define OMAP4_GPIO_DEBOUNCENABLE	0x0150
#define OMAP4_GPIO_DEBOUNCINGTIME	0x0154
#define OMAP4_GPIO_CLEARIRQENABLE1	0x0160
#define OMAP4_GPIO_SETIRQENABLE1	0x0164
#define OMAP4_GPIO_CLEARWKUENA		0x0180
#define OMAP4_GPIO_SETWKUENA		0x0184
#define OMAP4_GPIO_CLEARDATAOUT		0x0190
#define OMAP4_GPIO_SETDATAOUT		0x0194

#define OMAP_MPUIO(nr)		(OMAP_MAX_GPIO_LINES + (nr))
#define OMAP_GPIO_IS_MPUIO(nr)	((nr) >= OMAP_MAX_GPIO_LINES)

#define OMAP_GPIO_IRQ(nr)	(OMAP_GPIO_IS_MPUIO(nr) ? \
				 IH_MPUIO_BASE + ((nr) & 0x0f) : \
				 IH_GPIO_BASE + (nr))

#define METHOD_MPUIO		0
#define METHOD_GPIO_1510	1
#define METHOD_GPIO_1610	2
#define METHOD_GPIO_7XX		3
#define METHOD_GPIO_24XX	5
#define METHOD_GPIO_44XX	6

struct omap_gpio_dev_attr {
	int bank_width;		/* GPIO bank width */
	bool dbck_flag;		/* dbck required or not - True for OMAP3&4 */
};

struct omap_gpio_platform_data {
	u16 virtual_irq_start;
	int bank_type;
	int bank_width;		/* GPIO bank width */
	int bank_stride;	/* Only needed for omap1 MPUIO */
	bool dbck_flag;		/* dbck required or not - True for OMAP3&4 */
};

/* TODO: Analyze removing gpio_bank_count usage from driver code */
extern int gpio_bank_count;

extern void omap2_gpio_prepare_for_idle(int off_mode);
extern void omap2_gpio_resume_after_idle(void);
extern void omap_set_gpio_debounce(int gpio, int enable);
extern void omap_set_gpio_debounce_time(int gpio, int enable);
extern void omap_gpio_save_context(void);
extern void omap_gpio_restore_context(void);
/*-------------------------------------------------------------------------*/

/* Wrappers for "new style" GPIO calls, using the new infrastructure
 * which lets us plug in FPGA, I2C, and other implementations.
 * *
 * The original OMAP-specific calls should eventually be removed.
 */

#include <linux/errno.h>
#include <asm-generic/gpio.h>

static inline int gpio_get_value(unsigned gpio)
{
	return __gpio_get_value(gpio);
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	__gpio_set_value(gpio, value);
}

static inline int gpio_cansleep(unsigned gpio)
{
	return __gpio_cansleep(gpio);
}

static inline int gpio_to_irq(unsigned gpio)
{
	return __gpio_to_irq(gpio);
}

static inline int irq_to_gpio(unsigned irq)
{
	int tmp;

	/* omap1 SOC mpuio */
	if (cpu_class_is_omap1() && (irq < (IH_MPUIO_BASE + 16)))
		return (irq - IH_MPUIO_BASE) + OMAP_MAX_GPIO_LINES;

	/* SOC gpio */
	tmp = irq - IH_GPIO_BASE;
	if (tmp < OMAP_MAX_GPIO_LINES)
		return tmp;

	/* we don't supply reverse mappings for non-SOC gpios */
	return -EIO;
}

#endif
