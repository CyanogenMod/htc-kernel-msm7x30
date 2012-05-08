/* linux/arch/arm/mach-msm/board-runnymede-audio.c
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
#include <mach/pmic.h>
#include <mach/dal.h>
#include "board-runnymede.h"
#include <mach/qdsp5v2_2x/snddev_icodec.h>
#include <mach/qdsp5v2_2x/snddev_ecodec.h>
#include <mach/qdsp5v2_2x/audio_def.h>
#include <mach/qdsp5v2_2x/voice.h>
#include <mach/htc_acoustic_7x30.h>
#include <mach/htc_acdb_7x30.h>
#include <mach/board_htc.h>
#include <linux/spi/spi_aic3254.h>

static struct mutex bt_sco_lock;
static int curr_rx_mode;
static atomic_t aic3254_ctl = ATOMIC_INIT(0);
void runnymede_back_mic_enable(int);
static int audio_2v85_usage_counter;
static struct mutex audio_2v85_usage_lock;


#define BIT_SPEAKER	(1 << 0)
#define BIT_HEADSET	(1 << 1)
#define BIT_RECEIVER	(1 << 2)
#define BIT_FM_SPK	(1 << 3)
#define BIT_FM_HS	(1 << 4)
#define PMGPIO(x) (x-1)
#define runnymede_ACDB_SMEM_SIZE        (0xE000)
#define runnymede_ACDB_RADIO_BUFFER_SIZE (1024 * 3072)
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
	GPIO_CFG(138, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_CLK  */
};


static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static unsigned int runnymede_audio_2v85_power_on(int en)
{
	struct vreg *vreg_2v85;
	int ret;
	pr_aud_info("%s %d", __func__, en);

	vreg_2v85 = vreg_get(NULL, "gp9");

	if (IS_ERR(vreg_2v85)) {
		pr_aud_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "vreg_2v85", PTR_ERR(vreg_2v85));
		return -ENODEV;
	}

	if (en)
		ret = vreg_enable(vreg_2v85);
	else
		ret = vreg_disable(vreg_2v85);

	vreg_put(vreg_2v85);
	pr_aud_info("%s %d ret %d\n", __func__, en, ret);

	return ret;
}

static unsigned int runnymede_audio_2v85_enable(int en)
{
	int rc = 0;

	mutex_lock(&audio_2v85_usage_lock);
	if (en) {
		if (audio_2v85_usage_counter == 0)
			rc = runnymede_audio_2v85_power_on(1);
		audio_2v85_usage_counter++;
	} else {
		if (audio_2v85_usage_counter > 0) {
			audio_2v85_usage_counter--;
			if (audio_2v85_usage_counter == 0)
				rc = runnymede_audio_2v85_power_on(0);
		} else
			pr_aud_info("%s: counter error!\n", __func__);
	}
	mutex_unlock(&audio_2v85_usage_lock);

	return rc;
}

void runnymede_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		mdelay(30);
		gpio_request(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_SPK_SD),
						"AMP_EN");
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_SPK_SD), 1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_SPEAKER;
	} else {
		/* Reset AIC3254 */
		gpio_request(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_SPK_SD),
						"AMP_EN");
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_SPK_SD), 0);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_SPEAKER;
	}
}

void runnymede_snddev_hsed_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN), 1);
		gpio_request(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN),
						"HP_AMP_EN");
		mdelay(30);
		set_headset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_HEADSET;
	} else {
		set_headset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN), 0);
		gpio_request(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN),
						"HP_AMP_EN");
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_HEADSET;
	}
}

void runnymede_snddev_hs_spk_pamp_on(int en)
{
	runnymede_snddev_poweramp_on(en);
	runnymede_snddev_hsed_pamp_on(en);
}

void runnymede_snddev_bt_sco_pamp_on(int en)
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
			gpio_set_value(runnymede_GPIO_BT_PCM_OUT, 0);
			gpio_set_value(runnymede_GPIO_BT_PCM_SYNC, 0);
			gpio_set_value(runnymede_GPIO_BT_PCM_CLK, 0);
		}
	}
	mutex_unlock(&bt_sco_lock);
}

void runnymede_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN), 1);
		gpio_request(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN),
						"HP_AMP_EN");
		mdelay(20);
		set_handset_amp(1);
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode |= BIT_RECEIVER;
	} else {
		set_handset_amp(0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN), 0);
		gpio_request(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_AMP_EN),
						"HP_AMP_EN");
		if (!atomic_read(&aic3254_ctl))
			curr_rx_mode &= ~BIT_RECEIVER;
	}
}

void runnymede_snddev_usb_headset_on(int en)
{
#if 0
	struct vreg *vreg_ncp;
	int ret;

	vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(vreg_ncp)) {
		pr_aud_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(vreg_ncp));
		return;
	}
	pr_aud_err("%s %d\n", __func__, en);

	if (en) {
		gpio_set_value(runnymede_AUDIOz_UART_SW, 0);
		gpio_set_value(runnymede_USBz_AUDIO_SW, 1);
		ret = vreg_enable(vreg_ncp);
	} else {
		ret = vreg_disable(vreg_ncp);
		gpio_set_value(runnymede_AUDIOz_UART_SW, 1);
		gpio_set_value(runnymede_USBz_AUDIO_SW, 0);
	}
#endif
}

void runnymede_snddev_imic_pamp_on(int en)
{
	pr_aud_info("%s: %d\n", __func__, en);

	if (en) {
		runnymede_audio_2v85_enable(en);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_STEREO_REC), 1);
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
		runnymede_back_mic_enable(1);
	} else {
		runnymede_back_mic_enable(0);
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(runnymede_AUD_STEREO_REC), 0);
		runnymede_audio_2v85_enable(en);
	}
}

void runnymede_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en)
		gpio_set_value(runnymede_AUD_MICPATH_SEL, 1);
	else
		gpio_set_value(runnymede_AUD_MICPATH_SEL, 0);
}

void runnymede_back_mic_enable(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		gpio_set_value(runnymede_AUD_MICPATH_SEL, 0);
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_ALWAYS);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_1, PM_HSED_ENABLE_OFF);
	}
}

int runnymede_get_rx_vol(uint8_t hw, int network, int level)
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

void runnymede_mic_bias_enable(int en, int shift)
{
	pr_aud_info("%s: %d\n", __func__, en);

	if (en) {
		runnymede_audio_2v85_enable(en);
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_ALWAYS);
	} else {
		pmic_hsed_enable(PM_HSED_CONTROLLER_2, PM_HSED_ENABLE_OFF);
		runnymede_audio_2v85_enable(en);
	}
}

void runnymede_rx_amp_enable(int en)
{
	if (curr_rx_mode != 0) {
		atomic_set(&aic3254_ctl, 1);
		pr_aud_info("%s: curr_rx_mode 0x%x, en %d\n",
			__func__, curr_rx_mode, en);
		if (curr_rx_mode & BIT_SPEAKER)
			runnymede_snddev_poweramp_on(en);
		if (curr_rx_mode & BIT_HEADSET)
			runnymede_snddev_hsed_pamp_on(en);
		if (curr_rx_mode & BIT_RECEIVER)
			runnymede_snddev_receiver_pamp_on(en);
		atomic_set(&aic3254_ctl, 0);;
	}
}

uint32_t runnymede_get_smem_size(void)
{
	return runnymede_ACDB_SMEM_SIZE;
}

uint32_t runnymede_get_acdb_radio_buffer_size(void)
{
	return runnymede_ACDB_RADIO_BUFFER_SIZE;
}

int runnymede_support_aic3254(void)
{
	return 1;
}

int runnymede_support_adie(void)
{
	return 1;
}

int runnymede_support_back_mic(void)
{
	return 1;
}

void runnymede_get_acoustic_tables(struct acoustic_tables *tb)
{
		strcpy(tb->aic3254,
				"IOTable.txt\0");
		strcpy(tb->aic3254_dsp,
				"CodecDSPID.txt\0");
}

int runnymede_support_beats(void)
{
	/* this means HW support 1V for beats */
	if (get_engineerid() > 0x0000)
		return 1;
	else
		return 0;
}

void runnymede_enable_beats(int en)
{
	pr_aud_info("%s: %d\n", __func__, en);
	set_beats_on(en);
}

static struct q5v2audio_icodec_ops iops = {
	.support_aic3254 = runnymede_support_aic3254,
	.support_adie = runnymede_support_adie,
};

static struct acdb_ops acdb = {
	.get_acdb_radio_buffer_size = runnymede_get_acdb_radio_buffer_size,
};

static struct q5v2audio_analog_ops ops = {
	.speaker_enable	= runnymede_snddev_poweramp_on,
	.headset_enable	= runnymede_snddev_hsed_pamp_on,
	.handset_enable	= runnymede_snddev_receiver_pamp_on,
	.bt_sco_enable = runnymede_snddev_bt_sco_pamp_on,
	.headset_speaker_enable = runnymede_snddev_hs_spk_pamp_on,
	.usb_headset_enable = runnymede_snddev_usb_headset_on,
	.int_mic_enable = runnymede_snddev_imic_pamp_on,
	.ext_mic_enable = runnymede_snddev_emic_pamp_on,
	.fm_headset_enable = runnymede_snddev_hsed_pamp_on,
	.fm_speaker_enable = runnymede_snddev_poweramp_on,
};

static struct q5v2audio_ecodec_ops eops = {
	.bt_sco_enable  = runnymede_snddev_bt_sco_pamp_on,
};

static struct q5v2voice_ops vops = {
	.get_rx_vol = runnymede_get_rx_vol,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = runnymede_mic_bias_enable,
	.support_aic3254 = runnymede_support_aic3254,
	.support_back_mic = runnymede_support_back_mic,
	.enable_back_mic =  runnymede_back_mic_enable,
	.get_acoustic_tables = runnymede_get_acoustic_tables,
	.support_beats = runnymede_support_beats,
	.enable_beats = runnymede_enable_beats,
};

static struct aic3254_ctl_ops cops = {
	.rx_amp_enable = runnymede_rx_amp_enable,
};


void __init runnymede_audio_init(void)
{

	mutex_init(&bt_sco_lock);
	mutex_init(&audio_2v85_usage_lock);
	audio_2v85_usage_counter = 0;

	htc_7x30_register_analog_ops(&ops);
	htc_7x30_register_icodec_ops(&iops);
	htc_7x30_register_ecodec_ops(&eops);
	htc_7x30_register_voice_ops(&vops);
	acoustic_register_ops(&acoustic);
	acdb_register_ops(&acdb);
	aic3254_register_ctl_ops(&cops);

	gpio_request(runnymede_AUD_MICPATH_SEL, "aud_mic_sel");
	gpio_direction_output(runnymede_AUD_MICPATH_SEL, 1);
	gpio_set_value(runnymede_AUD_MICPATH_SEL, 0);

	mutex_lock(&bt_sco_lock);
	config_gpio_table(aux_pcm_gpio_off, ARRAY_SIZE(aux_pcm_gpio_off));
	gpio_set_value(runnymede_GPIO_BT_PCM_OUT, 0);
	gpio_set_value(runnymede_GPIO_BT_PCM_SYNC, 0);
	gpio_set_value(runnymede_GPIO_BT_PCM_CLK, 0);
	mutex_unlock(&bt_sco_lock);
}
