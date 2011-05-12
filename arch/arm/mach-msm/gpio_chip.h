/* arch/arm/mach-msm/gpio_chip.h
 *
 * Copyright (C) 2007 Google, Inc.
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

#ifndef _LINUX_GPIO_CHIP_H
#define _LINUX_GPIO_CHIP_H

#include <linux/list.h>
#include <linux/gpio.h>

#define GPIOF_IRQF_MASK         0x0000ffff /* use to specify edge detection without */
#define GPIOF_IRQF_TRIGGER_NONE 0x00010000 /* IRQF_TRIGGER_NONE is 0 which also means "as already configured" */
#define GPIOF_INPUT             0x00020000
#define GPIOF_DRIVE_OUTPUT      0x00040000
#define GPIOF_OUTPUT_LOW        0x00080000
#define GPIOF_OUTPUT_HIGH       0x00100000

#define MSM_GPIO_BROKEN_INT_CLEAR 1

struct old_gpio_chip {
	struct gpio_chip gpio_chip;
#define gpio_chip old_gpio_chip
	struct device	*dev; /* used for pmic8058-gpio*/

	spinlock_t lock;
	unsigned int start;
	unsigned int end;

	int (*configure)(struct gpio_chip *chip, unsigned int gpio, unsigned long flags);
	int (*get_irq_num)(struct gpio_chip *chip, unsigned int gpio, unsigned int *irqp, unsigned long *irqnumflagsp);
	int (*read)(struct gpio_chip *chip, unsigned int gpio);
	int (*write)(struct gpio_chip *chip, unsigned int gpio, unsigned on);
	int (*read_detect_status)(struct gpio_chip *chip, unsigned int gpio);
	int (*clear_detect_status)(struct gpio_chip *chip, unsigned int gpio);
};

struct msm_gpio_regs {
	void __iomem *out;
	void __iomem *in;
	void __iomem *int_status;
	void __iomem *int_clear;
	void __iomem *int_en;
	void __iomem *int_edge;
	void __iomem *int_pos;
	void __iomem *oe;
};

struct msm_gpio_chip {
	struct gpio_chip        chip;
	struct msm_gpio_regs    regs;
#if MSM_GPIO_BROKEN_INT_CLEAR
	unsigned                int_status_copy;
#endif
	unsigned int            both_edge_detect;
	unsigned int            int_enable[2]; /* 0: awake, 1: sleep */
	unsigned int            int_enable_mask[2]; /* 0: awake, 1: sleep */
};

int register_gpio_chip(struct gpio_chip *gpio_chip);

#endif
