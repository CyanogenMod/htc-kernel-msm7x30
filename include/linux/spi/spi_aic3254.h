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
#define AIC3254_DUMP_PAGES       _IOW(AIC3254_IOCTL_MAGIC, 0x30, unsigned int)
#define AIC3254_READ_REG         _IOWR(AIC3254_IOCTL_MAGIC, 0x31, unsigned)
#define AIC3254_WRITE_REG        _IOW(AIC3254_IOCTL_MAGIC, 0x32, unsigned)

#define AIC3254_MAX_PAGES	255
#define AIC3254_MAX_REGS        128
#define AIC3254_MAX_RETRY	10

enum aic3254_uplink_mode {
	INITIAL = 0,
	CALL_UPLINK_IMIC_RECEIVER = 1,
	CALL_UPLINK_EMIC_HEADSET,
	CALL_UPLINK_IMIC_HEADSET,
	CALL_UPLINK_IMIC_SPEAKER,
	VOICERECORD_IMIC = 15,
	VOICERECORD_EMIC,
	UPLINK_OFF = 29,
	UPLINK_WAKEUP,
	POWER_OFF,
};

enum aic3254_downlink_mode {
	PLAYBACK_SPEAKER = 13,
	FM_OUT_SPEAKER = 21,
	FM_OUT_HEADSET,
	DOWNLINK_OFF = 29,
	DOWNLINK_WAKEUP,
};

struct aic3254_ctl_ops {
	void (*tx_amp_enable)(int en);
	void (*rx_amp_enable)(int en);
	int (*panel_sleep_in)(void);
};

void aic3254_register_ctl_ops(struct aic3254_ctl_ops *ops);
#endif /* __SPI_AIC3254_H__*/
