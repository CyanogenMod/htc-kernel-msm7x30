/* linux/arch/arm/mach-msm/board-vivow-audio.c
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
#include <mach/tpa2051d3.h>
#include <mach/gpio.h>
#include <mach/dal.h>
#include "board-vivow.h"
#include <mach/qdsp5v2_1x/snddev_icodec.h>
#include <mach/qdsp5v2_1x/snddev_ecodec.h>
#include <mach/qdsp5v2_1x/audio_def.h>
#include <mach/qdsp5v2_1x/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <mach/htc_acdb_7x30.h>
#include <linux/spi/spi_aic3254.h>

static struct mutex bt_sco_lock;
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);
void vivow_back_mic_enable(int);


#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)

#define VIVOW_ACDB_SMEM_SIZE        (0xE000)
#define VIVOW_ACDB_RADIO_BUFFER_SIZE (1024 * 3072)

static struct q5v2_hw_info q5v2_audio_hw[Q5V2_HW_COUNT] = {
	[Q5V2_HW_HANDSET] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
	},
	[Q5V2_HW_HEADSET] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
	},
	[Q5V2_HW_SPEAKER] = {
		.max_gain[VOC_NB_INDEX] = 0,
		.min_gain[VOC_NB_INDEX] = 0,
		.max_gain[VOC_WB_INDEX] = 0,
		.min_gain[VOC_WB_INDEX] = 0,
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
		.max_gain[VOC_NB_INDEX] = 100,
		.min_gain[VOC_NB_INDEX] = -1900,
		.max_gain[VOC_WB_INDEX] = 100,
		.min_gain[VOC_WB_INDEX] = -1900,
	},
};

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_OUT, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_IN, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_SYNC, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_CLK, 0, GPIO_INPUT,
			GPIO_PULL_DOWN, GPIO_2MA),
};

static unsigned aux_pcm_gpio_on[] = {
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_OUT, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_IN, 1, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_SYNC, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(VIVOW_GPIO_BT_PCM_CLK, 1, GPIO_OUTPUT,
			GPIO_NO_PULL, GPIO_2MA),
};


void vivow_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVOW_AUD_SPK_SD), 1);
		mdelay(30);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
	} else {
		/* Reset AIC3254 */
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVOW_AUD_SPK_SD), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
	}
}

void vivow_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVOW_AUD_AMP_EN), 1);
		mdelay(30);
		set_headset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
	} else {
		set_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVOW_AUD_AMP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void vivow_snddev_hs_spk_pamp_on(int en)
{
	vivow_snddev_poweramp_on(en);
	vivow_snddev_hsed_pamp_on(en);
}

void vivow_snddev_bt_sco_pamp_on(int en)
{
	static int bt_sco_refcount;
	pr_aud_info("%s %d\n", __func__, en);
	mutex_lock(&bt_sco_lock);
	if (en) {
		if (++bt_sco_refcount == 1)
			config_gpio_table(aux_pcm_gpio_on,
					ARRAY_SIZE(aux_pcm_gpio_on));
	} else {
		if (--bt_sco_refcount == 0) {
			config_gpio_table(aux_pcm_gpio_off,
					ARRAY_SIZE(aux_pcm_gpio_off));
			gpio_set_value(VIVOW_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(VIVOW_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(VIVOW_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

void vivow_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVOW_AUD_AMP_EN), 1);
		mdelay(20);
		set_handset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		set_handset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIVOW_AUD_AMP_EN), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
}

void vivow_snddev_usb_headset_on(int en)
{
	struct vreg *vreg_ncp;
	int ret;

	vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(vreg_ncp));
		return;
	}
	pr_err("%s %d\n",__func__, en);

	if (en) {
		gpio_set_value(VIVOW_AUDIOz_UART_SW, 0);
		gpio_set_value(VIVOW_USBz_AUDIO_SW, 1);
		ret = vreg_enable(vreg_ncp);
	} else {
		ret = vreg_disable(vreg_ncp);
		gpio_set_value(VIVOW_AUDIOz_UART_SW, 1);
		gpio_set_value(VIVOW_USBz_AUDIO_SW, 0);
	}
}

void vivow_snddev_imic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
		vivow_back_mic_enable(1);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
		vivow_back_mic_enable(0);
	}
}

void vivow_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(VIVOW_AUD_MICPATH_SEL, 1);
	} else
		gpio_set_value(VIVOW_AUD_MICPATH_SEL, 0);
}

void vivow_back_mic_enable(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(VIVOW_AUD_MICPATH_SEL, 0);
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_ALWAYS);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_OFF);
	}
}

int vivow_get_rx_vol(uint8_t hw, int network, int level)
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

void vivow_mic_bias_enable(int en, int shift)
{
	pr_aud_info("%s: %d\n", __func__, en);

	if (en)
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_ALWAYS);
	else
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_OFF);
}

void vivow_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			vivow_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			vivow_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			vivow_snddev_receiver_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}

uint32_t vivow_get_smem_size(void)
{
	return VIVOW_ACDB_SMEM_SIZE;
}

uint32_t vivow_get_acdb_radio_buffer_size(void)
{
	return VIVOW_ACDB_RADIO_BUFFER_SIZE;
}

int vivow_support_aic3254(void)
{
	return 1;
}

int vivow_support_back_mic(void)
{
	return 1;
}

void vivow_get_acoustic_tables(struct acoustic_tables *tb)
{
	strcpy(tb->aic3254_dsp, "CodecDSPID_BCLK.txt\0");

	if (system_rev == 0) {
		strcpy(tb->aic3254,
				"AIC3254_REG_DualMic.csv\0");
	} else {
		strcpy(tb->aic3254,
				"AIC3254_REG_DualMicXB.csv\0");
		strcpy(tb->aic3254_dsp,
				"CodecDSPID.txt\0");

	}
}

static struct q5v2audio_icodec_ops iops = {
	.support_aic3254 = vivow_support_aic3254,
};

static struct acdb_ops acdb = {
	.get_acdb_radio_buffer_size = vivow_get_acdb_radio_buffer_size,
};

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= vivow_snddev_poweramp_on,
	.headset_enable	= vivow_snddev_hsed_pamp_on,
	.handset_enable	= vivow_snddev_receiver_pamp_on,
	.bt_sco_enable = vivow_snddev_bt_sco_pamp_on,
	.headset_speaker_enable = vivow_snddev_hs_spk_pamp_on,
	.usb_headset_enable = vivow_snddev_usb_headset_on,
	.int_mic_enable = vivow_snddev_imic_pamp_on,
	.ext_mic_enable = vivow_snddev_emic_pamp_on,
	.fm_headset_enable = vivow_snddev_hsed_pamp_on,
	.fm_speaker_enable = vivow_snddev_poweramp_on,
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = vivow_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = vivow_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = vivow_mic_bias_enable,
	.support_aic3254 = vivow_support_aic3254,
	.support_back_mic = vivow_support_back_mic,
	.enable_back_mic =  vivow_back_mic_enable,
	.get_acoustic_tables = vivow_get_acoustic_tables
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable = vivow_rx_amp_enable,
};


void __init vivow_audio_init(void)
{

	struct pm8058_gpio tpa2051_pwr = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = 6,      /* S3 1.8 V */
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
	};

	mutex_init(&bt_sco_lock);
#ifdef CONFIG_MSM7KV2_1X_AUDIO
	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_icodec_ops(&iops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	acoustic_register_ops(&acoustic);
	acdb_register_ops(&acdb);
#endif
	aic3254_register_ctl_ops(&cops);

	pm8058_gpio_config(VIVOW_AUD_SPK_SD, &tpa2051_pwr);
	pm8058_gpio_config(VIVOW_AUD_AMP_EN, &tpa2051_pwr);

	gpio_request(VIVOW_AUD_MICPATH_SEL, "aud_mic_sel");
	gpio_direction_output(VIVOW_AUD_MICPATH_SEL, 1);
	gpio_set_value(VIVOW_AUD_MICPATH_SEL, 0);

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(VIVOW_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(VIVOW_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(VIVOW_GPIO_BT_PCM_CLK, 0);
	mutex_unlock(&bt_sco_lock);
}


