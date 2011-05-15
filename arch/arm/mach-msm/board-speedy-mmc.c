/* linux/arch/arm/mach-msm/board-speedy-mmc.c
 *
 * Copyright (C) 2008 HTC Corporation.
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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>

#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>

#include <asm/mach/mmc.h>

#include "devices.h"
#include "board-speedy.h"
#include "proc_comm.h"


extern int msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);

/* ---- SDCARD ---- */
static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(58, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* CLK */
	PCOM_GPIO_CFG(59, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(60, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(58, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(59, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(60, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */
};

static uint opt_disable_sdcard;

static uint32_t movinand_on_gpio_table[] = {
	PCOM_GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(69, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */
	PCOM_GPIO_CFG(115, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT4 */
	PCOM_GPIO_CFG(114, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT5 */
	PCOM_GPIO_CFG(113, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT6 */
	PCOM_GPIO_CFG(112, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT7 */
};
static int __init speedy_disablesdcard_setup(char *str)
{
	int cal = simple_strtol(str, NULL, 0);

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_speedy.disable_sdcard=", speedy_disablesdcard_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

static struct mmc_vdd_xlat mmc_vdd_table[] = {
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2900 },
};

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t speedy_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		mdelay(5);
		vreg_enable(vreg_sdslot);
		udelay(500);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			printk(KERN_INFO "%s: Setting level to %u\n",
				__func__, mmc_vdd_table[i].level);
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
			return 0;
		}
	}

	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int speedy_sdslot_status(struct device *dev)
{
	/* No detection pin, only TP, always return true. */
	return (1);
}

#define SPEEDY_MMC_VDD		(MMC_VDD_28_29 | MMC_VDD_29_30)

static unsigned int speedy_sdslot_type = MMC_TYPE_SD;

static struct mmc_platform_data speedy_sdslot_data = {
	.ocr_mask	= SPEEDY_MMC_VDD,
	.status		= speedy_sdslot_status,
	.translate_vdd	= speedy_sdslot_switchvdd,
	.slot_type	= &speedy_sdslot_type,
	.dat0_gpio	= 63,
	.non_hot_plug = 1,	/* Used to control the timer for SD card status check */
};

static unsigned int speedy_emmcslot_type = MMC_TYPE_MMC;
static struct mmc_platform_data speedy_movinand_data = {
	.ocr_mask	=  SPEEDY_MMC_VDD,
	.slot_type	= &speedy_emmcslot_type,
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
};

/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(110, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(116, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(110, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(147, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* WLAN IRQ */
};

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
 */
static struct embedded_sdio_data speedy_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	},
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
speedy_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int speedy_wifi_cd;	/* WiFi virtual 'card detect' status */

static unsigned int speedy_wifi_status(struct device *dev)
{
	return speedy_wifi_cd;
}

static struct mmc_platform_data speedy_wifi_data = {
	.ocr_mask		= MMC_VDD_28_29,
	.status			= speedy_wifi_status,
	.register_status_notify	= speedy_wifi_status_register,
	.embedded_sdio		= &speedy_wifi_emb_data,
};

int speedy_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	speedy_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(speedy_wifi_set_carddetect);

static struct pm8058_gpio pmic_gpio_sleep_clk_output = {
	.direction      = PM_GPIO_DIR_OUT,
	.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
	.output_value   = 0,
	.pull           = PM_GPIO_PULL_NO,
	.vin_sel        = PM_GPIO_VIN_S3,      /* S3 1.8 V */
	.out_strength   = PM_GPIO_STRENGTH_HIGH,
	.function       = PM_GPIO_FUNC_2,
};

#define ID_WIFI	0
#define ID_BT	1
#define CLK_OFF	0
#define CLK_ON	1
static DEFINE_SPINLOCK(speedy_w_b_slock);
int speedy_sleep_clk_state_wifi = CLK_OFF;
int speedy_sleep_clk_state_bt = CLK_OFF;

int speedy_wifi_bt_sleep_clk_ctl(int on, int id)
{
	int err = 0;
	unsigned long flags;

	printk(KERN_DEBUG "%s ON=%d, ID=%d\n", __func__, on, id);

	spin_lock_irqsave(&speedy_w_b_slock, flags);
	if (on) {
		if ((CLK_OFF == speedy_sleep_clk_state_wifi)
			&& (CLK_OFF == speedy_sleep_clk_state_bt)) {
			printk(KERN_DEBUG "EN SLEEP CLK\n");
			pmic_gpio_sleep_clk_output.function = PM_GPIO_FUNC_2;
			err = pm8058_gpio_config(SPEEDY_WIFI_BT_SLEEP_CLK_EN,
					&pmic_gpio_sleep_clk_output);
			if (err) {
				spin_unlock_irqrestore(&speedy_w_b_slock,
							flags);
				printk(KERN_ERR "ERR EN SLEEP CLK, ERR=%d\n",
					err);
				return err;
			}
		}

		if (id == ID_BT)
			speedy_sleep_clk_state_bt = CLK_ON;
		else
			speedy_sleep_clk_state_wifi = CLK_ON;
	} else {
		if (((id == ID_BT) && (CLK_OFF == speedy_sleep_clk_state_wifi))
			|| ((id == ID_WIFI)
			&& (CLK_OFF == speedy_sleep_clk_state_bt))) {
			printk(KERN_DEBUG "DIS SLEEP CLK\n");
			pmic_gpio_sleep_clk_output.function =
						PM_GPIO_FUNC_NORMAL;
			err = pm8058_gpio_config(
					SPEEDY_WIFI_BT_SLEEP_CLK_EN,
					&pmic_gpio_sleep_clk_output);
			if (err) {
				spin_unlock_irqrestore(&speedy_w_b_slock,
							flags);
				printk(KERN_ERR "ERR DIS SLEEP CLK, ERR=%d\n",
					err);
				return err;
			}
		} else {
			printk(KERN_DEBUG "KEEP SLEEP CLK ALIVE\n");
		}

		if (id)
			speedy_sleep_clk_state_bt = CLK_OFF;
		else
			speedy_sleep_clk_state_wifi = CLK_OFF;
	}
	spin_unlock_irqrestore(&speedy_w_b_slock, flags);

	return 0;
}
EXPORT_SYMBOL(speedy_wifi_bt_sleep_clk_ctl);

int speedy_wifi_power(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
		config_gpio_table(wifi_on_gpio_table,
			ARRAY_SIZE(wifi_on_gpio_table));
	} else {
		config_gpio_table(wifi_off_gpio_table,
			ARRAY_SIZE(wifi_off_gpio_table));
	}

	speedy_wifi_bt_sleep_clk_ctl(on, ID_WIFI);
	gpio_set_value(SPEEDY_GPIO_WIFI_SHUTDOWN_N, on); /* WIFI_SHUTDOWN */
	mdelay(120);
	return 0;
}
EXPORT_SYMBOL(speedy_wifi_power);

int speedy_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	return 0;
}


// /* ---------------- WiMAX GPIO Settings --------------- */
static uint32_t wimax_on_gpio_table[] = {
	PCOM_GPIO_CFG(38, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(39, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(40, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT3 */
	PCOM_GPIO_CFG(41, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT2 */
	PCOM_GPIO_CFG(42, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT1 */
	PCOM_GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* DAT0 */	 
};

static uint32_t wimax_off_gpio_table[] = {
	PCOM_GPIO_CFG(38, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(39, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(40, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(41, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(42, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(43, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* DAT0 */	
};


static void (*wimax_status_cb)(int card_present, void *dev_id);
static void *wimax_status_cb_devid;
static int mmc_wimax_cd = 0;
static int mmc_wimax_sdio_status = 0;
static int mmc_wimax_netlog_status = 0;
static int mmc_wimax_sdio_interrupt_log_status = 0;
static int mmc_wimax_netlog_withraw_status = 0;
static int mmc_wimax_cliam_host_status = 0;
static int mmc_wimax_busclk_pwrsave = 1; // Default is dynamic CLK OFF
static int mmc_wimax_CMD53_timeout_trigger_counter = 0;
static int mmc_wimax_hostwakeup_gpio = PM8058_GPIO_PM_TO_SYS(SPEEDY_WiMAX_HOST_WAKEUP);
static int mmc_wimax_thp_log_status = 0;
static int mmc_wimax_sdio_hw_reset = 0; // Rollback to default disabled HW RESET
static int mmc_wimax_packet_filter = 0; 

static int mmc_wimax_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	if (wimax_status_cb)
		return -EAGAIN;
	printk("%s\n", __func__);
	wimax_status_cb = callback;
	wimax_status_cb_devid = dev_id;
	return 0;
}

static unsigned int mmc_wimax_status(struct device *dev)
{
	printk("%s\n", __func__);
	return mmc_wimax_cd;
}

void mmc_wimax_set_carddetect(int val)
{
	printk("%s: %d\n", __func__, val);
	mmc_wimax_cd = val;
	if (wimax_status_cb) {
		wimax_status_cb(val, wimax_status_cb_devid);
	} else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
}
EXPORT_SYMBOL(mmc_wimax_set_carddetect);

static unsigned int speedy_wimax_type = MMC_TYPE_SDIO_WIMAX;

static struct mmc_platform_data mmc_wimax_data = {
	.ocr_mask		= MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30,
	.status			= mmc_wimax_status,
	.register_status_notify	= mmc_wimax_status_register,
	.embedded_sdio		= NULL,
	.slot_type		= &speedy_wimax_type,
};

struct _vreg
{
	const char *name;
	unsigned id;
};

int mmc_wimax_power(int on)
{
	printk("%s\n", __func__);

	if (on) {
		/*Power ON sequence*/
		gpio_set_value(91, 1); /* V_WIMAX_3V3_EN */
		mdelay(5);
		gpio_set_value(90, 1); /* V_WIMAX_PVDD_EN */
		mdelay(5);
		gpio_set_value(93, 1); /* V_WIMAX_1V2_RF_EN */
		gpio_set_value(92, 1); /* V_WIMAX_DVDD_EN */
		mdelay(5);

		mdelay(1);			
		config_gpio_table(wimax_on_gpio_table,
			  ARRAY_SIZE(wimax_on_gpio_table));
		mdelay(60);
		gpio_set_value(21, 1); /* WIMAX_EXT_RSTz */
		mdelay(2);
	} else {
		/* Power OFF sequence */
		config_gpio_table(wimax_off_gpio_table,
		ARRAY_SIZE(wimax_off_gpio_table));

		gpio_set_value(21, 1); /* WIMAX_EXT_RSTz */
		mdelay(5);
		gpio_set_value(92, 0); /* V_WIMAX_DVDD_EN */
		gpio_set_value(93, 0); /* V_WIMAX_1V2_RF_EN */
		gpio_set_value(21, 0); /* WIMAX_EXT_RSTz */
		mdelay(5);
		gpio_set_value(91, 0); /* V_WIMAX_3V3_EN */
		gpio_set_value(90, 0); /* V_WIMAX_PVDD_EN */
		mdelay(5);
	}
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_power);

int wimax_uart_switch = 0;
int mmc_wimax_uart_switch(int uart)
{
	printk("%s uart:%d\n", __func__, uart);
	wimax_uart_switch = uart;
	gpio_set_value(95, uart?1:0); // CPU_WIMAX_SW
		
	return 0; 
}
EXPORT_SYMBOL(mmc_wimax_uart_switch);

int mmc_wimax_get_uart_switch(void)
{
	printk("%s uart:%d\n", __func__, wimax_uart_switch);
	return wimax_uart_switch?1:0; 	
}
EXPORT_SYMBOL(mmc_wimax_get_uart_switch);

int mmc_wimax_set_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_status);

int mmc_wimax_get_status(void)
{
	return mmc_wimax_sdio_status;
}
EXPORT_SYMBOL(mmc_wimax_get_status);

int mmc_wimax_set_cliam_host_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_cliam_host_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_cliam_host_status);

int mmc_wimax_get_cliam_host_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_status);
	return mmc_wimax_cliam_host_status;
}
EXPORT_SYMBOL(mmc_wimax_get_cliam_host_status);

int mmc_wimax_set_netlog_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_netlog_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_netlog_status);

int mmc_wimax_get_netlog_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_status);
	return mmc_wimax_netlog_status;
}
EXPORT_SYMBOL(mmc_wimax_get_netlog_status);

int mmc_wimax_set_netlog_withraw_status(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_netlog_withraw_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_netlog_withraw_status);

int mmc_wimax_get_netlog_withraw_status(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_netlog_withraw_status);
	return mmc_wimax_netlog_withraw_status;
}
EXPORT_SYMBOL(mmc_wimax_get_netlog_withraw_status);

int mmc_wimax_set_sdio_interrupt_log(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_interrupt_log_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_sdio_interrupt_log);

int mmc_wimax_get_sdio_interrupt_log(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_interrupt_log_status);
	return mmc_wimax_sdio_interrupt_log_status;
}
EXPORT_SYMBOL(mmc_wimax_get_sdio_interrupt_log);

int mmc_wimax_set_packet_filter(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_packet_filter = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_packet_filter);

int mmc_wimax_get_packet_filter(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_packet_filter);
	return mmc_wimax_packet_filter;
}
EXPORT_SYMBOL(mmc_wimax_get_packet_filter);

int mmc_wimax_set_thp_log(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_thp_log_status = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_thp_log);

int mmc_wimax_get_thp_log(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_thp_log_status);
	return mmc_wimax_thp_log_status;
}
EXPORT_SYMBOL(mmc_wimax_get_thp_log);

int mmc_wimax_set_busclk_pwrsave(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_busclk_pwrsave = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_busclk_pwrsave);

int mmc_wimax_get_busclk_pwrsave(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_busclk_pwrsave);
	return mmc_wimax_busclk_pwrsave;
}
EXPORT_SYMBOL(mmc_wimax_get_busclk_pwrsave);

int mmc_wimax_set_sdio_hw_reset(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_sdio_hw_reset = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_sdio_hw_reset);

int mmc_wimax_get_sdio_hw_reset(void)
{
	//printk(KERN_INFO "%s status:%d\n", __func__, mmc_wimax_sdio_hw_reset);
	return mmc_wimax_sdio_hw_reset;
}
EXPORT_SYMBOL(mmc_wimax_get_sdio_hw_reset);

int mmc_wimax_set_CMD53_timeout_trigger_counter(int counter)
{
	printk(KERN_INFO "%s counter:%d\n", __func__, counter);
	mmc_wimax_CMD53_timeout_trigger_counter = counter;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_CMD53_timeout_trigger_counter);

int mmc_wimax_get_CMD53_timeout_trigger_counter(void)
{
	//printk(KERN_INFO "%s counter:%d\n", __func__, mmc_wimax_CMD53_timeout_trigger_counter);
	return mmc_wimax_CMD53_timeout_trigger_counter;
}
EXPORT_SYMBOL(mmc_wimax_get_CMD53_timeout_trigger_counter);

int mmc_wimax_get_hostwakeup_gpio(void)
{
	return mmc_wimax_hostwakeup_gpio;
}
EXPORT_SYMBOL(mmc_wimax_get_hostwakeup_gpio);

static int mmc_wimax_is_gpio_irq_enabled = 0;

int mmc_wimax_set_gpio_irq_enabled(int on)
{
	printk(KERN_INFO "%s on:%d\n", __func__, on);
	mmc_wimax_is_gpio_irq_enabled = on;
	return 0;
}
EXPORT_SYMBOL(mmc_wimax_set_gpio_irq_enabled);

int mmc_wimax_get_gpio_irq_enabled(void)
{
	return mmc_wimax_is_gpio_irq_enabled;
}
EXPORT_SYMBOL(mmc_wimax_get_gpio_irq_enabled);

void mmc_wimax_enable_host_wakeup(int on)
{
	if (mmc_wimax_sdio_status)
	{	
		if (on) {
			if (!mmc_wimax_is_gpio_irq_enabled) {
				printk("set GPIO%d as waketup source\n", mmc_wimax_get_hostwakeup_gpio());
				enable_irq(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));
				enable_irq_wake(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));
				mmc_wimax_is_gpio_irq_enabled = 1;
			}
		}
		else {
			if (mmc_wimax_is_gpio_irq_enabled) {
				printk("disable GPIO%d wakeup source\n", mmc_wimax_get_hostwakeup_gpio());
				disable_irq_wake(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));				
                disable_irq_nosync(MSM_GPIO_TO_INT(mmc_wimax_get_hostwakeup_gpio()));
				mmc_wimax_is_gpio_irq_enabled = 0;
			}
		}
	}
	else {
		printk("%s mmc_wimax_sdio_status is OFF\n", __func__);
	}
}
EXPORT_SYMBOL(mmc_wimax_enable_host_wakeup);

int __init speedy_init_mmc(unsigned int sys_rev)
{
	uint32_t id;
	wifi_status_cb = NULL;
	sdslot_vreg_enabled = 0;

	printk(KERN_INFO "speedy: %s\n", __func__);
	/* SDC1: Initial WiMAX */
	msm_add_sdcc(1, &mmc_wimax_data,0,0);

	/* SDC2: MoviNAND */
	config_gpio_table(movinand_on_gpio_table,
		ARRAY_SIZE(movinand_on_gpio_table));
	msm_add_sdcc(2, &speedy_movinand_data, 0, 0);

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(SPEEDY_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	gpio_set_value(SPEEDY_GPIO_WIFI_SHUTDOWN_N, 0);

	msm_add_sdcc(3, &speedy_wifi_data, 0, 0);

	if (opt_disable_sdcard) {
		printk(KERN_INFO "speedy: SD-Card interface disabled\n");
		goto done;
	}

	vreg_sdslot = vreg_get(0, "gp10");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	msm_add_sdcc(4, &speedy_sdslot_data, 0, 0);
done:

	return 0;
}
