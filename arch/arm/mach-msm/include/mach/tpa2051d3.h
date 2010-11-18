/*
 * Definitions for tpa2051d3 speaker amp chip.
 */
#ifndef TPA2051D3_H
#define TPA2051D3_H

#include <linux/ioctl.h>

#define TPA2051D3_I2C_NAME "tpa2051d3"
#define SPKR_OUTPUT 0
#define HEADSET_OUTPUT 1
#define DUAL_OUTPUT 2
#define HANDSET_OUTPUT 3
#define MODE_CMD_LEM 9
struct tpa2051d3_platform_data {
	uint32_t gpio_tpa2051_spk_en;
};

struct tpa2051_config_data {
        unsigned int data_len;
        unsigned int mode_num;
        unsigned char *cmd_data;  /* [mode][mode_kind][reserve][cmds..] */
};

enum TPA2051_Mode {
	TPA2051_MODE_OFF,
	TPA2051_MODE_PLAYBACK_SPKR,
	TPA2051_MODE_PLAYBACK_HEADSET,
	TPA2051_MODE_RING,
	TPA2051_MODE_VOICECALL_SPKR,
	TPA2051_MODE_VOICECALL_HEADSET,
	TPA2051_MODE_FM_SPKR,
	TPA2051_MODE_FM_HEADSET,
	TPA2051_MODE_HANDSET,
	TPA2051_MAX_MODE
};
#define TPA2051_IOCTL_MAGIC 'a'
#define TPA2051_SET_CONFIG	_IOW(TPA2051_IOCTL_MAGIC, 0x01,	unsigned)
#define TPA2051_READ_CONFIG	_IOW(TPA2051_IOCTL_MAGIC, 0x02, unsigned)
#define TPA2051_SET_MODE        _IOW(TPA2051_IOCTL_MAGIC, 0x03, unsigned)
#define TPA2051_SET_PARAM       _IOW(TPA2051_IOCTL_MAGIC, 0x04,  unsigned)

void set_speaker_amp(int on);
void set_headset_amp(int on);
void set_speaker_headset_amp(int on);
void set_handset_amp(int on);
#endif

