/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Qualcomm PMIC8058 MPP driver
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include "gpio_chip.h"

#define PM8058_MPP_TO_INT(n) (PMIC8058_IRQ_BASE + NR_PMIC8058_GPIO_IRQS + (n))

/* MPP Control Registers */
#define	SSBI_MPP_CNTRL_BASE		0x50
#define	SSBI_MPP_CNTRL(n)		(SSBI_MPP_CNTRL_BASE + (n))

/* MPP Type */
#define	PM8058_MPP_TYPE_MASK		0xE0
#define	PM8058_MPP_TYPE_SHIFT		5

/* MPP Config Level */
#define	PM8058_MPP_CONFIG_LVL_MASK	0x1C
#define	PM8058_MPP_CONFIG_LVL_SHIFT	2

/* MPP Config Control */
#define	PM8058_MPP_CONFIG_CTL_MASK	0x03

static int pm8058_mpp_get_irq_num(struct gpio_chip *chip,
				   unsigned int gpio,
				   unsigned int *irqp,
				   unsigned long *irqnumflagsp)
{
	gpio -= chip->start;
	*irqp = PM8058_MPP_TO_INT(gpio);
	if (irqnumflagsp)
		*irqnumflagsp = 0;
	return 0;
}
/*
static int pm8058_mpp_read(struct gpio_chip *chip, unsigned n)
{
#if 0
	struct pm8058 *pm_chip;
	return 0;
#else
	struct pm8058_chip	*pm_chip;

	n -= chip->start;
	pm_chip = dev_get_drvdata(chip->dev);
	return pm8058_mpp_get(pm_chip, n);
#endif
}
*/
struct msm_gpio_chip pm8058_mpp_chip = {
	.chip = {
		.start = NR_GPIO_IRQS + NR_PMIC8058_GPIO_IRQS,
		.end = NR_GPIO_IRQS + NR_PMIC8058_GPIO_IRQS +
			NR_PMIC8058_MPP_IRQS - 1,
		.get_irq_num = pm8058_mpp_get_irq_num,
		/*
		.read = pm8058_mpp_read,
		*/
	}
};

int pm8058_mpp_config(unsigned mpp, unsigned type, unsigned level,
		      unsigned control)
{
	u8	config;
	int	rc;
	struct pm8058_chip *pm_chip;

	if (mpp >= PM8058_MPPS)
		return -EINVAL;

	pm_chip = dev_get_drvdata(pm8058_mpp_chip.chip.dev);

	config = (type << PM8058_MPP_TYPE_SHIFT) & PM8058_MPP_TYPE_MASK;
	config |= (level << PM8058_MPP_CONFIG_LVL_SHIFT) &
			PM8058_MPP_CONFIG_LVL_MASK;
	config |= control & PM8058_MPP_CONFIG_CTL_MASK;

	rc = pm8058_write_buf(pm8058_mpp_chip.chip.dev, SSBI_MPP_CNTRL(mpp), &config, 1);
	if (rc)
		pr_err("%s: pm8058_write(): rc=%d\n", __func__, rc);

	return rc;
}

static int __devinit pm8058_mpp_probe(struct platform_device *pdev)
{
	int	rc;

	pm8058_mpp_chip.chip.dev = &pdev->dev;
	rc = register_gpio_chip(&pm8058_mpp_chip.chip);
	pr_info("%s: register_gpio_chip(): rc=%d\n", __func__, rc);

	return rc;
}

static int __devexit pm8058_mpp_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver pm8058_mpp_driver = {
	.probe		= pm8058_mpp_probe,
	.remove		= __devexit_p(pm8058_mpp_remove),
	.driver		= {
		.name = "pm8058-mpp",
		.owner = THIS_MODULE,
	},
};

static int __init pm8058_mpp_init(void)
{
	return platform_driver_register(&pm8058_mpp_driver);
}

static void __exit pm8058_mpp_exit(void)
{
	platform_driver_unregister(&pm8058_mpp_driver);
}

subsys_initcall(pm8058_mpp_init);
module_exit(pm8058_mpp_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("PMIC8058 MPP driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pm8058-mpp");

