/* linux/driver/spi/spi_aic3254.h
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
 */

#ifndef __SPI_AIC3254_H__
#define __SPI_AIC3254_H__

#include <linux/ioctl.h>

typedef struct _CODEC_SPI_CMD {
	unsigned char act;
	unsigned char reg;
	unsigned char data;
} CODEC_SPI_CMD;

typedef struct _CODEC_SPI_CMD_PARAM {
	CODEC_SPI_CMD *data;
	unsigned int len;
} CODEC_SPI_CMD_PARAM;

struct AIC3254_PARAM {
	unsigned int row_num;
	unsigned int col_num;
	void *cmd_data;
};

struct CODEC_CFG {
	unsigned char tb_idx;
	unsigned char index;
};

/* IO CONTROL definition of AIC3254 */
#define AIC3254_IOCTL_MAGIC     's'
#define AIC3254_SET_TX_PARAM     _IOW(AIC3254_IOCTL_MAGIC, 0x10, unsigned)
#define AIC3254_SET_RX_PARAM     _IOW(AIC3254_IOCTL_MAGIC, 0x11, unsigned)
#define AIC3254_CONFIG_TX        _IOW(AIC3254_IOCTL_MAGIC, 0x12, unsigned int)
#define AIC3254_CONFIG_RX        _IOW(AIC3254_IOCTL_MAGIC, 0x13, unsigned int)
#define AIC3254_SET_DSP_PARAM    _IOW(AIC3254_IOCTL_MAGIC, 0x20, unsigned)
#define AIC3254_CONFIG_MEDIA     _IOW(AIC3254_IOCTL_MAGIC, 0x21, unsigned int)
#define AIC3254_CONFIG_VOICE     _IOW(AIC3254_IOCTL_MAGIC, 0x22, unsigned int)
#define AIC3254_CONFIG_VOLUME_L  _IOW(AIC3254_IOCTL_MAGIC, 0x23, unsigned int)
#define AIC3254_CONFIG_VOLUME_R  _IOW(AIC3254_IOCTL_MAGIC, 0x24, unsigned int)
#define AIC3254_POWERDOWN        _IOW(AIC3254_IOCTL_MAGIC, 0x25, unsigned int)
#define AIC3254_LOOPBACK         _IOW(AIC3254_IOCTL_MAGIC, 0x26, unsigned int)
#define AIC3254_DUMP_PAGES       _IOW(AIC3254_IOCTL_MAGIC, 0x30, unsigned int)
#define AIC3254_READ_REG         _IOWR(AIC3254_IOCTL_MAGIC, 0x31, unsigned)
#define AIC3254_WRITE_REG        _IOW(AIC3254_IOCTL_MAGIC, 0x32, unsigned)
#define AIC3254_RESET        _IOW(AIC3254_IOCTL_MAGIC, 0x33, unsigned int)

#define AIC3254_MAX_PAGES	255
#define AIC3254_MAX_REGS        128
#define AIC3254_MAX_RETRY	10

#define IO_CTL_ROW_MAX		64
#define IO_CTL_COL_MAX		1024
#define MINIDSP_ROW_MAX		32
#define MINIDSP_COL_MAX		16384

enum aic3254_uplink_mode {
	INITIAL = 0,
	CALL_UPLINK_IMIC_RECEIVER = 1,
	CALL_UPLINK_EMIC_HEADSET,
	CALL_UPLINK_IMIC_HEADSET,
	CALL_UPLINK_IMIC_SPEAKER,
	CALL_UPLINK_IMIC_RECEIVER_DUALMIC,
	CALL_UPLINK_EMIC_HEADSET_DUALMIC,
	CALL_UPLINK_IMIC_SPEAKER_DUALMIC,
	CALL_UPLINK_IMIC_RECIVER_TESTSIM,
	CALL_UPLINK_EMIC_HEADSET_TESTSIM,
	CALL_UPLINK_IMIC_SPEAKER_TESTSIM,
	VOICERECORD_IMIC = 15,
	VOICERECORD_EMIC,
	VIDEORECORD_IMIC,
	VIDEORECORD_EMIC,
	VOICERECOGNITION_IMIC,
	VOICERECOGNITION_EMIC,
	FM_IN_SPEAKER,
	FM_IN_HEADSET,
	TTY_IN_HCO,
	TTY_IN_VCO,
	TTY_IN_FULL,
	UPLINK_OFF = 29,
	UPLINK_WAKEUP,
	POWER_OFF,
	SLEEP_WITH_HP_IN,
	VOICERECORD_IMIC_PLAYBACK_SPEAKER,
	VOICERECORD_EMIC_PLAYBACK_HEADSET,
	VOICERECORD_IMIC_PLAYBACK_HEADSET,
};

enum aic3254_downlink_mode {
	CALL_DOWNLINK_IMIC_RECEIVER = 1,
	CALL_DOWNLINK_EMIC_HEADSET,
	CALL_DOWNLINK_IMIC_HEADSET,
	CALL_DOWNLINK_IMIC_SPEAKER,
	CALL_DOWNLINK_IMIC_RECEIVER_DUALMIC,
	CALL_DOWNLINK_EMIC_HEADSET_DUALMIC,
	CALL_DOWNLINK_IMIC_SPEAKER_DUALMIC,
	CALL_DOWNLINK_IMIC_RECIVER_TESTSIM,
	CALL_DOWNLINK_EMIC_HEADSET_TESTSIM,
	CALL_DOWNLINK_IMIC_SPEAKER_TESTSIM,
	PLAYBACK_RECEIVER,
	PLAYBACK_HEADSET,
	PLAYBACK_SPEAKER = 13,
	RING_HEADSET_SPEAKER,
	PLAYBACK_SPEAKER_ALT,
	USB_AUDIO,
	FM_OUT_SPEAKER = 21,
	FM_OUT_HEADSET,
	TTY_OUT_HCO,
	TTY_OUT_VCO,
	TTY_OUT_FULL,
	MUSE,
	HAC,
	LPM_IMIC_RECEIVER,
	DOWNLINK_OFF = 29,
	DOWNLINK_WAKEUP,
};

struct aic3254_ctl_ops {
	void (*tx_amp_enable)(int en);
	void (*rx_amp_enable)(int en);
	int (*panel_sleep_in)(void);
	void (*reset_3254)(void);
	void (*spibus_enable)(int en);
	CODEC_SPI_CMD_PARAM *downlink_off;
	CODEC_SPI_CMD_PARAM *uplink_off;
	CODEC_SPI_CMD_PARAM *downlink_on;
	CODEC_SPI_CMD_PARAM *uplink_on;
	CODEC_SPI_CMD_PARAM *lb_dsp_init;
	CODEC_SPI_CMD_PARAM *lb_downlink_receiver;
	CODEC_SPI_CMD_PARAM *lb_downlink_speaker;
	CODEC_SPI_CMD_PARAM *lb_downlink_headset;
	CODEC_SPI_CMD_PARAM *lb_uplink_imic;
	CODEC_SPI_CMD_PARAM *lb_uplink_emic;
	CODEC_SPI_CMD_PARAM *lb_receiver_imic;
	CODEC_SPI_CMD_PARAM *lb_speaker_imic;
	CODEC_SPI_CMD_PARAM *lb_headset_emic;
	CODEC_SPI_CMD_PARAM *lb_receiver_bmic;
	CODEC_SPI_CMD_PARAM *lb_speaker_bmic;
	CODEC_SPI_CMD_PARAM *lb_headset_bmic;
};

void aic3254_register_ctl_ops(struct aic3254_ctl_ops *ops);
void aic3254_set_mode(int config, int mode);
void aic3254_set_mic_bias(int en);
#endif /* __SPI_AIC3254_H__*/
