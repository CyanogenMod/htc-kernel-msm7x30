/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Qualcomm PMIC8058 driver header file
 *
 */

#include <linux/irq.h>
#include <linux/mfd/core.h>

/* PM8058 interrupt numbers */
#define PM8058_FIRST_IRQ	PMIC8058_IRQ_BASE

#ifdef CONFIG_MSM_SSBI
#define PM8058_FIRST_GPIO_IRQ		0
struct pm8058_gpio {
	u8		direction;
	u8		output_buffer;
	u8		output_value;
	u8		pull;
	u8		vin_sel;	/* 0..7 */
	u8		out_strength;
	u8		function;
	u8		inv_int_pol;	/* invert interrupt polarity */
};
#else
#define PM8058_FIRST_GPIO_IRQ	PM8058_FIRST_IRQ
struct pm8058_gpio {
	int		direction;
	int		output_buffer;
	int		output_value;
	int		pull;
	int		vin_sel;	/* 0..7 */
	int		out_strength;
	int		function;
	int		inv_int_pol;	/* invert interrupt polarity */
};
#endif

#define PM8058_FIRST_MPP_IRQ	(PM8058_FIRST_GPIO_IRQ + NR_PMIC8058_GPIO_IRQS)
#define PM8058_FIRST_MISC_IRQ	(PM8058_FIRST_MPP_IRQ + NR_PMIC8058_MPP_IRQS)

#define	PM8058_IRQ_KEYPAD	(PM8058_FIRST_MISC_IRQ)
#define	PM8058_IRQ_KEYSTUCK	(PM8058_FIRST_MISC_IRQ + 1)
#define PM8058_IRQ_CHGVAL	(PM8058_FIRST_MISC_IRQ + 2)

#define PM8058_IRQS		NR_PMIC8058_IRQS

#define PM8058_GPIOS		NR_PMIC8058_GPIO_IRQS
#define PM8058_MPPS		NR_PMIC8058_MPP_IRQS

/* IRQ# for GPIO number = 1 .. PM8058_GPIOS */
#define PM8058_IRQ_GPIO(n)	(PM8058_FIRST_GPIO_IRQ + (n) - 1)

#define PM8058_MAX_SUBDEVICES	16
#define FIRST_BOARD_GPIO	NR_GPIO_IRQS

struct pm8058_chip;

struct pm8058_keypad_platform_data {
	const char		*name;
	int			num_drv;
	int			num_sns;
	/* delay in ms = 1 << scan_delay_shift, 0-7 */
	int			scan_delay_shift;
	/* # of 32kHz clock cycles, 1-4 */
	int			drv_hold_clks;
	/* in increments of 5ms, max 20ms */
	int			debounce_ms;

	/* size must be num_drv * num_sns
	 * index is (drv * num_sns + sns) */
	const unsigned short	*keymap;

	int			(*init)(struct device *dev);
};

struct pm8058_sub_devices_data {
	const char		*name;
	int			(*init)(struct device *dev);
	void			*driver_data;
};

struct pm8058_platform_data {
	/* This table is only needed for misc interrupts. */
	unsigned int	pm_irqs[PM8058_IRQS];	/* block*8 + bit-pos */
#ifdef CONFIG_MSM_SSBI
	int					(*init)(struct device *dev);
	/* child devices */
	struct pm8058_keypad_platform_data	*keypad_pdata;
	struct pm8058_sub_devices_data sub_devices_htc[PM8058_MAX_SUBDEVICES];
#else
	int 		(*init)(struct pm8058_chip *pm_chip);
#endif
	int		num_subdevs;
	struct mfd_cell sub_devices[PM8058_MAX_SUBDEVICES];
	unsigned int				irq_base;
	unsigned int				gpio_base;
};

/* GPIO parameters */
/* direction */
#ifdef CONFIG_MSM_SSBI
#define	PM_GPIO_DIR_OUT			0x02
#define	PM_GPIO_DIR_IN			0x01
#else
#define	PM_GPIO_DIR_OUT			0x01
#define	PM_GPIO_DIR_IN			0x02
#endif
#define	PM_GPIO_DIR_BOTH		(PM_GPIO_DIR_OUT | PM_GPIO_DIR_IN)

/* output_buffer */
#define	PM_GPIO_OUT_BUF_OPEN_DRAIN	1
#define	PM_GPIO_OUT_BUF_CMOS		0

/* pull */
#define	PM_GPIO_PULL_UP_30		0
#define	PM_GPIO_PULL_UP_1P5		1
#define	PM_GPIO_PULL_UP_31P5		2
#define	PM_GPIO_PULL_UP_1P5_30		3
#define	PM_GPIO_PULL_DN			4
#define	PM_GPIO_PULL_NO			5

/* vin_sel: Voltage Input Select */
#define	PM_GPIO_VIN_VPH			0	/* VPH_PWR */
#define	PM_GPIO_VIN_BB			1	/* VIN from output of 3.3 buck_boost */
#define	PM_GPIO_VIN_S3			2	/* S3 1.8 V */
#define	PM_GPIO_VIN_L3			3	/* LDO3 2.85 V or 1.8 V */
#define	PM_GPIO_VIN_L7			4	/* LDO7 1.8 V */
#define	PM_GPIO_VIN_L6			5	/* LDO6 3.075 V */
#define	PM_GPIO_VIN_L5			6	/* LDO5 2.85 V */
#define	PM_GPIO_VIN_L2			7	/* LDO2 2.6 V */

/* out_strength */
#define	PM_GPIO_STRENGTH_NO		0
#define	PM_GPIO_STRENGTH_HIGH		1
#define	PM_GPIO_STRENGTH_MED		2
#define	PM_GPIO_STRENGTH_LOW		3

/* function */
#define	PM_GPIO_FUNC_NORMAL		0
#define	PM_GPIO_FUNC_PAIRED		1
#define	PM_GPIO_FUNC_1			2
#define	PM_GPIO_FUNC_2			3
#define	PM_GPIO_DTEST1			4
#define	PM_GPIO_DTEST2			5
#define	PM_GPIO_DTEST3			6
#define	PM_GPIO_DTEST4			7



#ifdef CONFIG_MSM_SSBI

struct pm8058_pin_config {
	u8		dir;
	u8		output_buffer;
	u8		output_value;
	u8		pull_up;
	u8		vin_src;
	u8		strength;
	u8		func;
	u8		flags;
};

#define PM8058_GPIO_INPUT		0x01
#define PM8058_GPIO_OUTPUT		0x02
#define PM8058_GPIO_OUTPUT_HIGH 	0x04

#define PM8058_NUM_GPIO_IRQS		PM8058_GPIOS
#define PM8058_NUM_MPP_IRQS		PM8058_MPPS
#define PM8058_NUM_KEYPAD_IRQS		2
#define PM8058_NUM_IRQS			(PM8058_NUM_GPIO_IRQS + \
					 PM8058_NUM_MPP_IRQS + \
					 PM8058_NUM_KEYPAD_IRQS)

/* be careful if you change this since this is used to map irq <-> gpio */

/*#define PM8058_FIRST_MPP_IRQ		(PM8058_FIRST_GPIO_IRQ + \
					 PM8058_NUM_GPIO_IRQS)*/
#define PM8058_FIRST_KEYPAD_IRQ		(PM8058_FIRST_MPP_IRQ + \
					 PM8058_NUM_MPP_IRQS)

#define PM8058_KEYPAD_IRQ		(PM8058_FIRST_KEYPAD_IRQ + 0)
#define PM8058_KEYPAD_STUCK_IRQ		(PM8058_FIRST_KEYPAD_IRQ + 1)

#define PM8058_GPIO_TO_IRQ(base, gpio)	(PM8058_FIRST_GPIO_IRQ + \
					 (base) + (gpio))

/* these need to match the irq counts/offsets above above */
#define PM8058_FIRST_GPIO		PM8058_FIRST_GPIO_IRQ
#define PM8058_NUM_GPIOS		PM8058_NUM_GPIO_IRQS
#define PM8058_FIRST_MPP		PM8058_FIRST_MPP_IRQ
#define PM8058_NUM_MPP			PM8058_NUM_MPP_IRQS

#define PM8058_GPIO(base, gpio)		((base) + (gpio) + PM8058_FIRST_GPIO)


#define PM8058_GPIO_PIN_CONFIG(v, d, p, s, fn, fl) \
	{ \
		.vin_src	= (v), \
		.dir		= (d), \
		.pull_up	= (p), \
		.strength	= (s), \
		.func		= (fn), \
		.flags		= (fl), \
	}

/* gpio pin flags */
#define PM8058_GPIO_OPEN_DRAIN		0x10
#define PM8058_GPIO_HIGH_Z		0x20
#define PM8058_GPIO_INV_IRQ_POL		0x40
#define PM8058_GPIO_CONFIGURED		0x80 /* FOR INTERNAL USE ONLY */


int pm8058_readb(struct device *dev, u16 addr, u8 *val);
int pm8058_writeb(struct device *dev, u16 addr, u8 val);
int pm8058_write_buf(struct device *dev, u16 addr, u8 *buf, int cnt);
int pm8058_read_buf(struct device *dev, u16 addr, u8 *buf, int cnt);
int pm8058_gpio_mux_cfg(struct device *dev, unsigned int gpio,
			struct pm8058_pin_config *cfg);
int pm8058_gpio_mux(unsigned int gpio, struct pm8058_pin_config *cfg);
int gpio_set_dir_htc(struct device *dev, unsigned gpio, int dir);
#else
int pm8058_gpio_set(struct pm8058_chip *pm_chip, unsigned gpio, int value);
int pm8058_gpio_get(struct pm8058_chip *pm_chip, unsigned gpio);
#endif
int pm8058_read(struct pm8058_chip *pm_chip, u16 addr, u8 *values,
		unsigned int len);
int pm8058_write(struct pm8058_chip *pm_chip, u16 addr, u8 *values,
		 unsigned int len);
int pm8058_gpio_cfg(int num, int direction, int output_buffer,
			int output_value, int pull, int vin_sel, int out_strength, int function, int inv_int_pol);
int pm8058_gpio_config(int gpio, struct pm8058_gpio *param);

int pm8058_gpio_config_h(struct pm8058_chip *pm_chip, int gpio,
			 struct pm8058_gpio *param);
int pm8058_gpio_set_direction(struct pm8058_chip *pm_chip,
			      unsigned gpio, int direction);
int pm8058_mpp_get(struct pm8058_chip *pm_chip, unsigned mpp);

int pm8058_gpio_config_kypd_drv(int gpio_start, int num_gpios);
int pm8058_gpio_config_kypd_sns(int gpio_start, int num_gpios);

int pm8058_rev_is_a0(struct pm8058_chip *pm_chip);
int pm8058_rev_is_b0(struct pm8058_chip *pm_chip);

