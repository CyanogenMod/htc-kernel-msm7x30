/* linux/arch/arm/mach-msm/board-saga-audio.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/delay.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include "board-saga.h"
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/qdsp5v2/audio_def.h>
#include <mach/qdsp5v2/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <mach/htc_acdb_7x30.h>
#include <linux/spi/spi_aic3254.h>

static struct mutex bt_sco_lock;
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);

#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)

#define SAGA_ACDB_RADIO_BUFFER_SIZE (1024 * 3072)

static struct q5v2_hw_info q5v2_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_gain[VOC_NB_INDEX] = 200,
		.min_gain[VOC_NB_INDEX] = -1400,
		.max_gain[VOC_WB_INDEX] = -200,
		.min_gain[VOC_WB_INDEX] = -1800,
	},
	[Q5V2_HW_HEADSET] = {
		.max_gain[VOC_NB_INDEX] = 400,
		.min_gain[VOC_NB_INDEX] = -1600,
		.max_gain[VOC_WB_INDEX] = 200,
		.min_gain[VOC_WB_INDEX] = -1800,
	},
	[Q5V2_HW_SPEAKER] = {
		.max_gain[VOC_NB_INDEX] = -100,
		.min_gain[VOC_NB_INDEX] = -1600,
		.max_gain[VOC_WB_INDEX] = -100,
		.min_gain[VOC_WB_INDEX] = -1600,
	},
	[Q5V2_HW_BT_SCO] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = -1500,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = -1500,
	},
	[Q5V2_HW_TTY] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
	},
	[Q5V2_HW_HS_SPKR] = {
		.max_gain[VOC_NB_INDEX] = -500,
		.min_gain[VOC_NB_INDEX] = -2000,
		.max_gain[VOC_WB_INDEX] = -500,
		.min_gain[VOC_WB_INDEX] = -2000,
	},
	[Q5V2_HW_USB_HS] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
	[Q5V2_HW_HAC] = {
		.max_gain[VOC_NB_INDEX] = 1000,
		.min_gain[VOC_NB_INDEX] = -500,
		.max_gain[VOC_WB_INDEX] = 1000,
		.min_gain[VOC_WB_INDEX] = -500,
	},
};

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_OUT, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_IN, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_SYNC, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_CLK, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
};

static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_OUT, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_IN, 1, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_SYNC, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(SAGA_GPIO_BT_PCM_CLK, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};

void saga_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_SPK_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_SPK_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
	}
}

void saga_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		msleep(60);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void saga_snddev_hs_spk_pamp_on(int en)
{
	saga_snddev_poweramp_on(en);
	saga_snddev_hsed_pamp_on(en);
}

void saga_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_EP_EN), 1);
		mdelay(5);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_EP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
}

void saga_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_aud_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1) {
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
		}
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(SAGA_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(SAGA_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(SAGA_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

/* power up internal/externnal mic shared GPIO */
void saga_mic_bias_enable(int en, int shift)
{
	pr_aud_info("%s: %d\n", __func__, en);
	if (en)
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_ALWAYS);
	else
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_OFF);
}

void saga_snddev_imic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_ALWAYS);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_OFF);
	}
}

void saga_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		/* change MICSELECT to pmic gpio 10 after XB */
		if (system_rev > 0)
			gpio_set_value(
				PM8058_GPIO_PM_TO_SYS(SAGA_AUD_MICPATH_SEL_XB)
				, 1);
		else
			gpio_set_value(SAGA_AUD_MICPATH_SEL_XA, 1);
	} else {
		if (system_rev > 0)
			gpio_set_value(
				PM8058_GPIO_PM_TO_SYS(SAGA_AUD_MICPATH_SEL_XB)
				, 1);
		else
			gpio_set_value(SAGA_AUD_MICPATH_SEL_XA, 0);
	}
}

void saga_snddev_fmspk_pamp_on(int en)
{
	if (en) {
		msleep(30);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_SPK_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_SPK;
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_SPK_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_SPK;
	}
}

void saga_snddev_fmhs_pamp_on(int en)
{
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_HP_EN), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_FM_HS;
		mdelay(5);
	} else {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(SAGA_AUD_HP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_FM_HS;
	}
}

int saga_get_rx_vol(uint8_t hw, int network, int level)
{
	struct q5v2_hw_info *info;
	int vol, maxv, minv;

	info = &q5v2_audio_hw[hw];
	maxv = info->max_gain[network];
	minv = info->min_gain[network];
	vol = minv + ((maxv - minv) * level) / 100;
	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);
	return vol;
}

void saga_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			saga_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			saga_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			saga_snddev_receiver_pamp_on(en);
		if (curr_rx_mode & BIT_FM_SPK)
			saga_snddev_fmspk_pamp_on(en);
		if (curr_rx_mode & BIT_FM_HS)
			saga_snddev_fmhs_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);
	}
}

uint32_t saga_get_acdb_radio_buffer_size(void)
{
	return SAGA_ACDB_RADIO_BUFFER_SIZE;
}

int saga_support_aic3254(void)
{
	return 1;
}

int saga_support_back_mic(void)
{
	return 1;
}

void saga_get_acoustic_tables(struct acoustic_tables *tb)
{
	switch (system_rev) {
	case 0:
		strcpy(tb->aic3254, "AIC3254_REG_DualMic.csv");
		break;
	case 1:
		strcpy(tb->aic3254, "AIC3254_REG_DualMic_XB.csv");
		break;
	default:
		strcpy(tb->aic3254, "AIC3254_REG_DualMic.txt");
	}
}

static struct acdb_ops acdb = {
	.get_acdb_radio_buffer_size = saga_get_acdb_radio_buffer_size,
};

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= saga_snddev_poweramp_on,
	.headset_enable	= saga_snddev_hsed_pamp_on,
	.handset_enable	= saga_snddev_receiver_pamp_on,
	.headset_speaker_enable	= saga_snddev_hs_spk_pamp_on,
	.bt_sco_enable	= saga_snddev_bt_sco_pamp_on,
	.int_mic_enable = saga_snddev_imic_pamp_on,
	.ext_mic_enable = saga_snddev_emic_pamp_on,
	.fm_headset_enable = saga_snddev_fmhs_pamp_on,
	.fm_speaker_enable = saga_snddev_fmspk_pamp_on,
};

static struct q5v2audio_icodec_ops iops = {
	.support_aic3254 = saga_support_aic3254,
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = saga_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = saga_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = saga_mic_bias_enable,
	.support_aic3254 = saga_support_aic3254,
	.support_back_mic = saga_support_back_mic,
	.get_acoustic_tables = saga_get_acoustic_tables
};

int saga_panel_sleep_in(void)
{
	/* Todo: Add panel sleep in panel driver........ */
	return 0;
}
static struct aic3254_ctl_ops cops = {
	.rx_amp_enable = saga_rx_amp_enable,
	.panel_sleep_in = saga_panel_sleep_in
};

void __init saga_audio_init(void)
{
	struct pm8058_gpio audio_pwr_28 = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.vin_sel        = 6,
	};

	struct pm8058_gpio audio_pwr_18 = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.vin_sel        = 4,
	};

	mutex_init(&bt_sco_lock);

#ifdef CONFIG_MSM7KV2_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_icodec_ops(&iops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	acoustic_register_ops(&acoustic);
	acdb_register_ops(&acdb);
#endif
	aic3254_register_ctl_ops(&cops);

	/* Init PMIC GPIO */
	pm8058_gpio_config(SAGA_AUD_HP_EN, &audio_pwr_28);
	pm8058_gpio_config(SAGA_AUD_EP_EN, &audio_pwr_28);
	pm8058_gpio_config(SAGA_AUD_SPK_EN, &audio_pwr_28);
	pm8058_gpio_config(SAGA_AUD_MICPATH_SEL_XB, &audio_pwr_28);
	/* Rest AIC3254 */
	pm8058_gpio_config(SAGA_AUD_A3254_RSTz, &audio_pwr_18);
	mdelay(1);
	audio_pwr_18.output_value = 1;
	pm8058_gpio_config(SAGA_AUD_A3254_RSTz, &audio_pwr_18);
	audio_pwr_18.output_value = 0;

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(SAGA_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(SAGA_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(SAGA_GPIO_BT_PCM_CLK, 0);
	gpio_set_value(36, 1);
	mutex_unlock(&bt_sco_lock);
}
