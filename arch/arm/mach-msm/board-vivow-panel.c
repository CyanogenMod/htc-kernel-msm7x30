/* linux/arch/arm/mach-msm/board-vivow-panel.c
 *
 * Copyright (C) 2008 HTC Corporation.
 * Author: Jay Tu <jay_tu@htc.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/panel_id.h>
#include "../../../drivers/video/msm/mdp_hw.h"

#include "pmic.h"
#include "board-vivow.h"
#include "devices.h"
#include "proc_comm.h"

#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while(0)
#endif
#define DEFAULT_BRIGHTNESS 100
extern int panel_type;

#define DRIVER_IC_CUT2			4
#define PANEL_VIVOW_SHARP		1
#define PANEL_VIVOW_SONY		2
#define PANEL_VIVOW_SHARP_CUT2		(PANEL_VIVOW_SHARP | DRIVER_IC_CUT2)
#define PANEL_VIVOW_SONY_CUT2		(PANEL_VIVOW_SONY | DRIVER_IC_CUT2)
#define	PANEL_VIVOW_HITACHI		PANEL_ID_VIVOW_HITACHI

#define VIVOW_BR_DEF_USER_PWM		143
#define VIVOW_BR_MIN_USER_PWM		30
#define VIVOW_BR_MAX_USER_PWM		255
#define VIVOW_BR_DEF_SONY_PANEL_PWM		135
#define VIVOW_BR_MIN_SONY_PANEL_PWM		8
#define VIVOW_BR_MAX_SONY_PANEL_PWM		255
#define VIVOW_BR_DEF_HITACHI_PANEL_PWM		156
#define VIVOW_BR_MIN_HITACHI_PANEL_PWM		9
#define VIVOW_BR_MAX_HITACHI_PANEL_PWM		255

#define LCM_CMD(_cmd, _delay, ...)                              \
{                                                               \
        .cmd = _cmd,                                            \
        .delay = _delay,                                        \
        .vals = (u8 []){__VA_ARGS__},                           \
        .len = sizeof((u8 []){__VA_ARGS__}) / sizeof(u8)        \
}
static struct clk *axi_clk;
static int color_enhancement = 0;

static struct vreg *V_LCMIO_1V8, *V_LCM_2V85;
static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
	CABC_STATE,
};

struct mddi_cmd {
        unsigned char cmd;
        unsigned delay;
        unsigned char *vals;
        unsigned len;
};

static struct mddi_cmd hitachi_renesas_cmd[] = {
	LCM_CMD(0x2A, 0, 0x00, 0x00, 0x01, 0xDF),
	LCM_CMD(0x2B, 0, 0x00, 0x00, 0x03, 0x1F),
	LCM_CMD(0x36, 0, 0x00, 0x00, 0x00, 0x00),
	LCM_CMD(0x3A, 0, 0x55, 0x55, 0x55, 0x55),//set_pixel_format 0x66 for 18bit/pixel, 0x77 for 24bit/pixel
	LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
	LCM_CMD(0x35, 0, 0x00, 0x00, 0x00, 0x00),//TE enable
	LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
//	LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
//	LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
//	LCM_CMD(0x29, 100,0x00,0x00,0x00,0x00),
};

static struct mddi_cmd hitachi_renesas_driving_cmd[] = {
	LCM_CMD(0xB0, 0, 0x04, 0x00, 0x00, 0x00),
	LCM_CMD(0xC1, 0, 0x43, 0x00, 0x00, 0x00,
			 0x31, 0x00, 0x00, 0x00,
			 0x00, 0x00, 0x00, 0x00,
			 0x1C, 0x00, 0x00, 0x00,
			 0x1C, 0x00, 0x00, 0x00,
			 0x32, 0x00, 0x00, 0x00,
			 0x32, 0x00, 0x00, 0x00,
			 0x21, 0x00, 0x00, 0x00,
			 0x4A, 0x00, 0x00, 0x00,
			 0x16, 0x00, 0x00, 0x00,
			 0xA5, 0x00, 0x00, 0x00,
			 0x0F, 0x00, 0x00, 0x00,
			 0x58, 0x00, 0x00, 0x00,
			 0x21, 0x00, 0x00, 0x00,
			 0x01, 0x00, 0x00, 0x00,),
	LCM_CMD(0xC2, 0, 0x10, 0x00, 0x00, 0x00,
			 0x05, 0x00, 0x00, 0x00,
			 0x05, 0x00, 0x00, 0x00,
			 0x07, 0x00, 0x00, 0x00,
			 0x01, 0x00, 0x00, 0x00,
			 0x04, 0x00, 0x00, 0x00,),
	LCM_CMD(0xB0, 0, 0x03, 0x00, 0x00, 0x00),
};


static struct mddi_cmd hitachi_renesas_backlight_cmd[] = {
	LCM_CMD(0xB9, 0, 0x00, 0x00, 0x00, 0x00,
			 0xff, 0x00, 0x00, 0x00,
			 0x03, 0x00, 0x00, 0x00,
			 0x08, 0x00, 0x00, 0x00,),
};

struct mdp_reg vivow_mdp_init_color[] = {
{0x94800,0x000000,0x0},
{0x94804,0x020302,0x0},
{0x94808,0x030303,0x0},
{0x9480C,0x030303,0x0},
{0x94810,0x040404,0x0},
{0x94814,0x040404,0x0},
{0x94818,0x050505,0x0},
{0x9481C,0x060506,0x0},
{0x94820,0x060606,0x0},
{0x94824,0x070607,0x0},
{0x94828,0x070708,0x0},
{0x9482C,0x080708,0x0},
{0x94830,0x090809,0x0},
{0x94834,0x09080A,0x0},
{0x94838,0x0A090A,0x0},
{0x9483C,0x0A0A0B,0x0},
{0x94840,0x0B0A0C,0x0},
{0x94844,0x0C0B0D,0x0},
{0x94848,0x0D0C0D,0x0},
{0x9484C,0x0D0C0E,0x0},
{0x94850,0x0E0D0F,0x0},
{0x94854,0x0F0E10,0x0},
{0x94858,0x100E11,0x0},
{0x9485C,0x100F11,0x0},
{0x94860,0x111012,0x0},
{0x94864,0x121113,0x0},
{0x94868,0x131114,0x0},
{0x9486C,0x131215,0x0},
{0x94870,0x141316,0x0},
{0x94874,0x151417,0x0},
{0x94878,0x161518,0x0},
{0x9487C,0x171619,0x0},
{0x94880,0x181719,0x0},
{0x94884,0x19171A,0x0},
{0x94888,0x19181B,0x0},
{0x9488C,0x1A191C,0x0},
{0x94890,0x1B1A1D,0x0},
{0x94894,0x1C1B1E,0x0},
{0x94898,0x1D1C1F,0x0},
{0x9489C,0x1E1D20,0x0},
{0x948A0,0x1F1E21,0x0},
{0x948A4,0x201F22,0x0},
{0x948A8,0x212023,0x0},
{0x948AC,0x222124,0x0},
{0x948B0,0x232225,0x0},
{0x948B4,0x242226,0x0},
{0x948B8,0x252328,0x0},
{0x948BC,0x262429,0x0},
{0x948C0,0x27252A,0x0},
{0x948C4,0x28262B,0x0},
{0x948C8,0x29272C,0x0},
{0x948CC,0x2A282D,0x0},
{0x948D0,0x2B292E,0x0},
{0x948D4,0x2C2A2F,0x0},
{0x948D8,0x2D2B30,0x0},
{0x948DC,0x2E2C31,0x0},
{0x948E0,0x2F2D33,0x0},
{0x948E4,0x302E34,0x0},
{0x948E8,0x312F35,0x0},
{0x948EC,0x323036,0x0},
{0x948F0,0x333137,0x0},
{0x948F4,0x343238,0x0},
{0x948F8,0x353339,0x0},
{0x948FC,0x36343B,0x0},
{0x94900,0x37353C,0x0},
{0x94904,0x38363D,0x0},
{0x94908,0x39373E,0x0},
{0x9490C,0x3A383F,0x0},
{0x94910,0x3B3941,0x0},
{0x94914,0x3C3A42,0x0},
{0x94918,0x3D3C43,0x0},
{0x9491C,0x3E3D44,0x0},
{0x94920,0x3F3E45,0x0},
{0x94924,0x403F47,0x0},
{0x94928,0x414048,0x0},
{0x9492C,0x434149,0x0},
{0x94930,0x44424A,0x0},
{0x94934,0x45434B,0x0},
{0x94938,0x46444D,0x0},
{0x9493C,0x47454E,0x0},
{0x94940,0x48464F,0x0},
{0x94944,0x494750,0x0},
{0x94948,0x4A4852,0x0},
{0x9494C,0x4B4953,0x0},
{0x94950,0x4C4A54,0x0},
{0x94954,0x4D4B55,0x0},
{0x94958,0x4E4C57,0x0},
{0x9495C,0x504D58,0x0},
{0x94960,0x514E59,0x0},
{0x94964,0x524F5A,0x0},
{0x94968,0x53505B,0x0},
{0x9496C,0x54515D,0x0},
{0x94970,0x55525E,0x0},
{0x94974,0x56535F,0x0},
{0x94978,0x575460,0x0},
{0x9497C,0x585562,0x0},
{0x94980,0x595663,0x0},
{0x94984,0x5A5764,0x0},
{0x94988,0x5C5865,0x0},
{0x9498C,0x5D5967,0x0},
{0x94990,0x5E5A68,0x0},
{0x94994,0x5F5B69,0x0},
{0x94998,0x605C6A,0x0},
{0x9499C,0x615D6C,0x0},
{0x949A0,0x625E6D,0x0},
{0x949A4,0x635F6E,0x0},
{0x949A8,0x64606F,0x0},
{0x949AC,0x656171,0x0},
{0x949B0,0x666272,0x0},
{0x949B4,0x676373,0x0},
{0x949B8,0x696474,0x0},
{0x949BC,0x6A6576,0x0},
{0x949C0,0x6B6677,0x0},
{0x949C4,0x6C6778,0x0},
{0x949C8,0x6D6879,0x0},
{0x949CC,0x6E697B,0x0},
{0x949D0,0x6F6A7C,0x0},
{0x949D4,0x706A7D,0x0},
{0x949D8,0x716B7E,0x0},
{0x949DC,0x726C80,0x0},
{0x949E0,0x736D81,0x0},
{0x949E4,0x756E82,0x0},
{0x949E8,0x766F83,0x0},
{0x949EC,0x777084,0x0},
{0x949F0,0x787186,0x0},
{0x949F4,0x797287,0x0},
{0x949F8,0x7A7388,0x0},
{0x949FC,0x7B7489,0x0},
{0x94A00,0x7C758A,0x0},
{0x94A04,0x7D768C,0x0},
{0x94A08,0x7E778D,0x0},
{0x94A0C,0x7F788E,0x0},
{0x94A10,0x80798F,0x0},
{0x94A14,0x817A90,0x0},
{0x94A18,0x827B92,0x0},
{0x94A1C,0x847C93,0x0},
{0x94A20,0x857D94,0x0},
{0x94A24,0x867E95,0x0},
{0x94A28,0x877F96,0x0},
{0x94A2C,0x888097,0x0},
{0x94A30,0x898199,0x0},
{0x94A34,0x8A829A,0x0},
{0x94A38,0x8B839B,0x0},
{0x94A3C,0x8C849C,0x0},
{0x94A40,0x8D859D,0x0},
{0x94A44,0x8E869E,0x0},
{0x94A48,0x8F87A0,0x0},
{0x94A4C,0x9088A1,0x0},
{0x94A50,0x9189A2,0x0},
{0x94A54,0x9289A3,0x0},
{0x94A58,0x938AA4,0x0},
{0x94A5C,0x948BA5,0x0},
{0x94A60,0x958CA6,0x0},
{0x94A64,0x978DA7,0x0},
{0x94A68,0x988EA8,0x0},
{0x94A6C,0x998FAA,0x0},
{0x94A70,0x9A90AB,0x0},
{0x94A74,0x9B91AC,0x0},
{0x94A78,0x9C92AD,0x0},
{0x94A7C,0x9D93AE,0x0},
{0x94A80,0x9E94AF,0x0},
{0x94A84,0x9F95B0,0x0},
{0x94A88,0xA096B1,0x0},
{0x94A8C,0xA197B2,0x0},
{0x94A90,0xA298B3,0x0},
{0x94A94,0xA399B4,0x0},
{0x94A98,0xA49AB5,0x0},
{0x94A9C,0xA59BB6,0x0},
{0x94AA0,0xA69CB7,0x0},
{0x94AA4,0xA79DB8,0x0},
{0x94AA8,0xA89EBA,0x0},
{0x94AAC,0xA99FBB,0x0},
{0x94AB0,0xAAA0BC,0x0},
{0x94AB4,0xABA1BD,0x0},
{0x94AB8,0xACA2BE,0x0},
{0x94ABC,0xADA3BF,0x0},
{0x94AC0,0xAEA4C0,0x0},
{0x94AC4,0xAFA5C1,0x0},
{0x94AC8,0xB0A6C2,0x0},
{0x94ACC,0xB1A6C3,0x0},
{0x94AD0,0xB2A7C4,0x0},
{0x94AD4,0xB3A8C4,0x0},
{0x94AD8,0xB4A9C5,0x0},
{0x94ADC,0xB5AAC6,0x0},
{0x94AE0,0xB6ABC7,0x0},
{0x94AE4,0xB7ACC8,0x0},
{0x94AE8,0xB8ADC9,0x0},
{0x94AEC,0xB9AECA,0x0},
{0x94AF0,0xBAAFCB,0x0},
{0x94AF4,0xBBB0CC,0x0},
{0x94AF8,0xBCB1CD,0x0},
{0x94AFC,0xBDB2CE,0x0},
{0x94B00,0xBEB3CF,0x0},
{0x94B04,0xBFB4D0,0x0},
{0x94B08,0xC0B5D1,0x0},
{0x94B0C,0xC1B6D2,0x0},
{0x94B10,0xC2B7D2,0x0},
{0x94B14,0xC3B7D3,0x0},
{0x94B18,0xC4B8D4,0x0},
{0x94B1C,0xC5B9D5,0x0},
{0x94B20,0xC6BAD6,0x0},
{0x94B24,0xC7BBD7,0x0},
{0x94B28,0xC8BCD8,0x0},
{0x94B2C,0xC9BDD9,0x0},
{0x94B30,0xCABED9,0x0},
{0x94B34,0xCABFDA,0x0},
{0x94B38,0xCBBFDB,0x0},
{0x94B3C,0xCCC0DC,0x0},
{0x94B40,0xCDC1DD,0x0},
{0x94B44,0xCEC2DE,0x0},
{0x94B48,0xCFC3DE,0x0},
{0x94B4C,0xD0C4DF,0x0},
{0x94B50,0xD1C5E0,0x0},
{0x94B54,0xD2C5E1,0x0},
{0x94B58,0xD3C6E2,0x0},
{0x94B5C,0xD4C7E3,0x0},
{0x94B60,0xD5C8E3,0x0},
{0x94B64,0xD5C8E4,0x0},
{0x94B68,0xD6C9E5,0x0},
{0x94B6C,0xD7CAE6,0x0},
{0x94B70,0xD8CBE7,0x0},
{0x94B74,0xD9CBE7,0x0},
{0x94B78,0xDACCE8,0x0},
{0x94B7C,0xDBCDE9,0x0},
{0x94B80,0xDCCEEA,0x0},
{0x94B84,0xDDCEEA,0x0},
{0x94B88,0xDDCFEB,0x0},
{0x94B8C,0xDED0EC,0x0},
{0x94B90,0xDFD0ED,0x0},
{0x94B94,0xE0D1ED,0x0},
{0x94B98,0xE1D1EE,0x0},
{0x94B9C,0xE2D2EF,0x0},
{0x94BA0,0xE2D2F0,0x0},
{0x94BA4,0xE3D3F0,0x0},
{0x94BA8,0xE4D4F1,0x0},
{0x94BAC,0xE5D4F2,0x0},
{0x94BB0,0xE6D5F2,0x0},
{0x94BB4,0xE6D5F3,0x0},
{0x94BB8,0xE7D5F4,0x0},
{0x94BBC,0xE8D6F5,0x0},
{0x94BC0,0xE9D6F5,0x0},
{0x94BC4,0xEAD7F6,0x0},
{0x94BC8,0xEAD7F7,0x0},
{0x94BCC,0xEBD7F7,0x0},
{0x94BD0,0xECD8F8,0x0},
{0x94BD4,0xEDD8F9,0x0},
{0x94BD8,0xEDD8FA,0x0},
{0x94BDC,0xEED8FA,0x0},
{0x94BE0,0xEFD9FB,0x0},
{0x94BE4,0xEFD9FC,0x0},
{0x94BE8,0xF0D9FC,0x0},
{0x94BEC,0xF1D9FD,0x0},
{0x94BF0,0xF1D9FE,0x0},
{0x94BF4,0xF2D9FE,0x0},
{0x94BF8,0xF3D9FF,0x0},
{0x94BFC,0xF3D9FF,0x0},
{0x90070,0x000017,0x7},
};

enum led_brightness vivow_brightness_value = DEFAULT_BRIGHTNESS;// multiple definition of `brightness_value' in board-glacier-panel.c

void vivow_mdp_color_enhancement(struct mdp_device *mdp_dev)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	mdp->write_regs(mdp, vivow_mdp_init_color, ARRAY_SIZE(vivow_mdp_init_color));
}

static struct msm_mdp_platform_data mdp_pdata = {
	.overrides = MSM_MDP4_MDDI_DMA_SWITCH
#ifdef CONFIG_OVERLAY_FORCE_UPDATE
	| MSM_MDP_FORCE_UPDATE
#endif
	,
};

static void
do_renesas_cmd(struct msm_mddi_client_data *client_data, struct mddi_cmd *cmd_table, ssize_t size)
{
	struct mddi_cmd *pcmd = NULL;
	for (pcmd = cmd_table; pcmd < cmd_table + size; pcmd++) {
		client_data->remote_write_vals(client_data, pcmd->vals,
			pcmd->cmd, pcmd->len);
		if (pcmd->delay)
			hr_msleep(pcmd->delay);
	}
}

static int vivow_shrink_pwm(int brightness)
{
	int level;
	unsigned int min_pwm, def_pwm, max_pwm;

	if(panel_type == PANEL_VIVOW_HITACHI) {
		min_pwm = VIVOW_BR_MIN_HITACHI_PANEL_PWM;
		def_pwm = VIVOW_BR_DEF_HITACHI_PANEL_PWM;
		max_pwm = VIVOW_BR_MAX_HITACHI_PANEL_PWM;
		if (brightness <= VIVOW_BR_DEF_USER_PWM) {
			if (brightness <= VIVOW_BR_MIN_USER_PWM)
				level = min_pwm;
			else
				level = (def_pwm - min_pwm) *
					(brightness - VIVOW_BR_MIN_USER_PWM) /
					(VIVOW_BR_DEF_USER_PWM - VIVOW_BR_MIN_USER_PWM) +
					min_pwm;
		} else
			level = (max_pwm - def_pwm) *
			(brightness - VIVOW_BR_DEF_USER_PWM) /
			(VIVOW_BR_MAX_USER_PWM - VIVOW_BR_DEF_USER_PWM) +
			def_pwm;
	} else {
		min_pwm = VIVOW_BR_MIN_SONY_PANEL_PWM;
		def_pwm = VIVOW_BR_DEF_SONY_PANEL_PWM;
		max_pwm = VIVOW_BR_MAX_SONY_PANEL_PWM;
		if (brightness <= VIVOW_BR_DEF_USER_PWM) {
			if (brightness <= VIVOW_BR_MIN_USER_PWM)
				level = min_pwm;
			else
				level = (def_pwm - min_pwm) *
					(brightness - VIVOW_BR_MIN_USER_PWM) /
					(VIVOW_BR_DEF_USER_PWM - VIVOW_BR_MIN_USER_PWM) +
					min_pwm;
		} else
			level = (max_pwm - def_pwm) *
			(brightness - VIVOW_BR_DEF_USER_PWM) /
			(VIVOW_BR_MAX_USER_PWM - VIVOW_BR_DEF_USER_PWM) +
			def_pwm;
	}

	return level;
}

/* use one flag to have better backlight on/off performance */
static int vivow_set_dim = 1;

static void vivow_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;
	struct mddi_cmd *pcmd = hitachi_renesas_backlight_cmd;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;
	shrink_br = vivow_shrink_pwm(val);

	if(!client) {
		pr_info("null mddi client");
		return;
	}

	mutex_lock(&cabc.lock);
	if(panel_type == PANEL_VIVOW_HITACHI) {
		pcmd->vals[4] = shrink_br;
		client->remote_write(client, 0x04, 0xB0);
	        client->remote_write_vals(client, pcmd->vals, pcmd->cmd, pcmd->len);
		client->remote_write(client, 0x03, 0xB0);
	} else {
		if (vivow_set_dim == 1) {
			client->remote_write(client, 0x2C, 0x5300);
			/* we dont need set dim again */
			vivow_set_dim = 0;
		}
		client->remote_write(client, 0x00, 0x5500);
		client->remote_write(client, shrink_br, 0x5100);
	}

	vivow_brightness_value = val;
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
vivow_get_brightness(struct led_classdev *led_cdev)
{
	/*FIXME:workaround for NOVATEK driver IC*/
#if 0
	struct msm_mddi_client_data *client = cabc.client_data;
	return client->remote_read(client, 0x5100);
#else
	return vivow_brightness_value;
#endif
}

static void vivow_backlight_switch(int on)
{
	enum led_brightness val;

	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;

		/* LED core uses get_brightness for default value
		 * If the physical layer is not ready, we should
		 * not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		vivow_set_brightness(&cabc.lcd_backlight, val);
		/* set next backlight value with dim */
		vivow_set_dim = 1;
	} else {
		clear_bit(GATE_ON, &cabc.status);
		vivow_set_brightness(&cabc.lcd_backlight, 0);
	}
}

static int vivow_cabc_switch(int on)
{
	struct msm_mddi_client_data *client = cabc.client_data;

	if (test_bit(CABC_STATE, &cabc.status) == on)
               return 1;

	if (on) {
		printk(KERN_DEBUG "turn on CABC\n");
		set_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x01, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	} else {
		printk(KERN_DEBUG "turn off CABC\n");
		clear_bit(CABC_STATE, &cabc.status);
		mutex_lock(&cabc.lock);
		client->remote_write(client, 0x00, 0x5500);
		client->remote_write(client, 0x2C, 0x5300);
		mutex_unlock(&cabc.lock);
	}
	return 1;
}

static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count);
#define CABC_ATTR(name) __ATTR(name, 0644, auto_backlight_show, auto_backlight_store)

static struct device_attribute auto_attr = CABC_ATTR(auto);
static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - 1, "%d\n",
				test_bit(CABC_STATE, &cabc.status));
	return i;
}

static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned long res;

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		printk(KERN_ERR "invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	if (vivow_cabc_switch(!!res))
		count = -EIO;

err_out:
	return count;
}

static int vivow_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = vivow_set_brightness;
	cabc.lcd_backlight.brightness_get = vivow_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;

	err = device_create_file(cabc.lcd_backlight.dev, &auto_attr);
	if (err)
		goto err_out;

	return 0;

err_out:
	device_remove_file(&pdev->dev, &auto_attr);
err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

/* ------------------------------------------------------------------- */

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

#define REG_WAIT (0xffff)

struct nov_regs {
	unsigned reg;
	unsigned val;
};

static struct nov_regs sharp_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 120},
	{0x89C3, 0x80},
	{0x92C2, 0x08},
	{0x0180, 0x14},
	{0x0280, 0x11},
	{0x0380, 0x33},
	{0x0480, 0x63},
	{0x0580, 0x63},
	{0x0680, 0x63},
	{0x0780, 0x00},
	{0x0880, 0x44},
	{0x0980, 0x54},
	{0x0A80, 0x10},
	{0x0B80, 0x55},
	{0x0C80, 0x55},
	{0x0D80, 0x30},
	{0x0E80, 0x44},
	{0x0F80, 0x54},
	{0x1080, 0x30},
	{0x1280, 0x77},
	{0x1380, 0x21},
	{0x1480, 0x0E},
	{0x1580, 0xB7},
	{0x1680, 0xEC},
	{0x1780, 0x00},
	{0x1880, 0x00},
	{0x1980, 0x00},
	{0x1C80, 0x00},
	{0x1F80, 0x05},
	{0x2480, 0x2B},
	{0x2580, 0x2F},
	{0x2680, 0x3D},
	{0x2780, 0x48},
	{0x2880, 0x1A},
	{0x2980, 0x2E},
	{0x2A80, 0x5F},
	{0x2B80, 0x42},
	{0x2D80, 0x20},
	{0x2F80, 0x27},
	{0x3080, 0x78},
	{0x3180, 0x1A},
	{0x3280, 0x40},
	{0x3380, 0x55},
	{0x3480, 0x5F},
	{0x3580, 0x7B},
	{0x3680, 0x92},
	{0x3780, 0x42},
	{0x3880, 0x3D},
	{0x3980, 0x40},
	{0x3A80, 0x4E},
	{0x3B80, 0x5A},
	{0x3D80, 0x19},
	{0x3F80, 0x2E},
	{0x4080, 0x5F},
	{0x4180, 0x55},
	{0x4280, 0x1F},
	{0x4380, 0x26},
	{0x4480, 0x8E},
	{0x4580, 0x19},
	{0x4680, 0x41},
	{0x4780, 0x55},
	{0x4880, 0x77},
	{0x4980, 0x92},
	{0x4A80, 0xA9},
	{0x4B80, 0x59},
	{0x4C80, 0x2B},
	{0x4D80, 0x2F},
	{0x4E80, 0x3D},
	{0x4F80, 0x48},
	{0x5080, 0x1A},
	{0x5180, 0x2E},
	{0x5280, 0x5F},
	{0x5380, 0x42},
	{0x5480, 0x20},
	{0x5580, 0x27},
	{0x5680, 0x78},
	{0x5780, 0x1A},
	{0x5880, 0x40},
	{0x5980, 0x55},
	{0x5A80, 0x5F},
	{0x5B80, 0x7B},
	{0x5C80, 0x92},
	{0x5D80, 0x42},
	{0x5E80, 0x3D},
	{0x5F80, 0x40},
	{0x6080, 0x4E},
	{0x6180, 0x5A},
	{0x6280, 0x19},
	{0x6380, 0x2E},
	{0x6480, 0x5F},
	{0x6580, 0x55},
	{0x6680, 0x1F},
	{0x6780, 0x26},
	{0x6880, 0x8E},
	{0x6980, 0x19},
	{0x6A80, 0x41},
	{0x6B80, 0x55},
	{0x6C80, 0x77},
	{0x6D80, 0x92},
	{0x6E80, 0xA9},
	{0x6F80, 0x59},
	{0x7080, 0x2B},
	{0x7180, 0x2F},
	{0x7280, 0x3D},
	{0x7380, 0x48},
	{0x7480, 0x1A},
	{0x7580, 0x2E},
	{0x7680, 0x5F},
	{0x7780, 0x42},
	{0x7880, 0x20},
	{0x7980, 0x27},
	{0x7A80, 0x78},
	{0x7B80, 0x1A},
	{0x7C80, 0x40},
	{0x7D80, 0x55},
	{0x7E80, 0x5F},
	{0x7F80, 0x7B},
	{0x8080, 0x92},
	{0x8180, 0x42},
	{0x8280, 0x3D},
	{0x8380, 0x40},
	{0x8480, 0x4E},
	{0x8580, 0x5A},
	{0x8680, 0x19},
	{0x8780, 0x2E},
	{0x8880, 0x5F},
	{0x8980, 0x55},
	{0x8A80, 0x1F},
	{0x8B80, 0x26},
	{0x8C80, 0x8E},
	{0x8D80, 0x19},
	{0x8E80, 0x41},
	{0x8F80, 0x55},
	{0x9080, 0x77},
	{0x9180, 0x92},
	{0x9280, 0xA9},
	{0x9380, 0x59},
	{0x9480, 0xB5},
	{0x9580, 0x04},
	{0x9680, 0x18},
	{0x9780, 0xB0},
	{0x9880, 0x20},
	{0x9980, 0x28},
	{0x9A80, 0x08},
	{0x9B80, 0x04},
	{0x9C80, 0x12},
	{0x9D80, 0x00},
	{0x9E80, 0x00},
	{0x9F80, 0x12},
	{0xA080, 0x00},
	{0xA280, 0x00},
	{0xA380, 0x3C},
	{0xA480, 0x01},
	{0xA580, 0xC0},
	{0xA680, 0x01},
	{0xA780, 0x00},
	{0xA980, 0x00},
	{0xAA80, 0x00},
	{0xAB80, 0x70},
	{0xE780, 0x11},
	{0xE880, 0x11},
	{0xED80, 0x0A},
	{0xEE80, 0x80},
	{0xF780, 0x0D},
	{0x2900, 0x00},
	{0x3500, 0x00},
	{0x4400, 0x02},
	{0x4401, 0x58},
};
static struct nov_regs sony_init_seq[] = {
	{0x1100, 0x00},
	{REG_WAIT, 120},
	{0x0480, 0x63},
	{0x0580, 0x63},
	{0x0680, 0x63},
	{0x5E00, 0x06},
	{0x1DC0, 0x3F},
	{0x1EC0, 0x40},
	{0x3BC0, 0xF3},
	{0x3DC0, 0xEF},
	{0x3FC0, 0xEB},
	{0x40C0, 0xE7},
	{0x41C0, 0xE3},
	{0x42C0, 0xDF},
	{0x43C0, 0xDB},
	{0x44C0, 0xD7},
	{0x45C0, 0xD3},
	{0x46C0, 0xCF},
	{0x3500, 0x00},
	{0x4400, 0x02},
	{0x4401, 0x58},
	{0x2900, 0x00},
};

static int
vivow_mddi_init(struct msm_mddi_bridge_platform_data *bridge_data,
		     struct msm_mddi_client_data *client_data)
{
	int i = 0, array_size = 0;
	unsigned reg, val;
	struct nov_regs *init_seq= NULL;
	B(KERN_DEBUG "%s\n", __func__);
	client_data->auto_hibernate(client_data, 0);

	if(panel_type == PANEL_VIVOW_HITACHI) {
		do_renesas_cmd(client_data, hitachi_renesas_cmd, ARRAY_SIZE(hitachi_renesas_cmd));
	}
	else {
		if (panel_type == PANEL_VIVOW_SONY
			|| panel_type == PANEL_VIVOW_SONY_CUT2) {
			init_seq = sony_init_seq;
			array_size = ARRAY_SIZE(sony_init_seq);
		} else {
			init_seq = sharp_init_seq;
			array_size = ARRAY_SIZE(sharp_init_seq);
		}

		for (i = 0; i < array_size; i++) {
			reg = cpu_to_le32(init_seq[i].reg);
			val = cpu_to_le32(init_seq[i].val);
			if (reg == REG_WAIT)
				hr_msleep(val);
			else {
				client_data->remote_write(client_data, val, reg);
				if (reg == 0x1100)
					client_data->send_powerdown(client_data);
			}
		}
	}
	client_data->auto_hibernate(client_data, 1);

	if(axi_clk)
		clk_set_rate(axi_clk, 0);

	return 0;
}

static int
vivow_mddi_uninit(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int
vivow_panel_blank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);
	client_data->auto_hibernate(client_data, 0);
	if(panel_type == PANEL_VIVOW_HITACHI) {
		client_data->remote_write(client_data, 0x0, 0x28);
		vivow_backlight_switch(LED_OFF);
		client_data->remote_write(client_data, 0x0, 0xB8);
		client_data->remote_write(client_data, 0x0, 0x10);
		hr_msleep(72);
	}else {

		client_data->remote_write(client_data, 0x0, 0x5300);
		vivow_backlight_switch(LED_OFF);
		client_data->remote_write(client_data, 0, 0x2800);
		client_data->remote_write(client_data, 0, 0x1000);
	}
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static int
vivow_panel_unblank(struct msm_mddi_bridge_platform_data *bridge_data,
			struct msm_mddi_client_data *client_data)
{
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);


	client_data->auto_hibernate(client_data, 0);
	if(panel_type == PANEL_VIVOW_HITACHI) {
		if (color_enhancement == 0) {
			vivow_mdp_color_enhancement(mdp_pdata.mdp_dev);
			color_enhancement = 1;
		}
		client_data->remote_write(client_data, 0x0, 0x11);
		hr_msleep(125);
		vivow_backlight_switch(LED_FULL);
		do_renesas_cmd(client_data, hitachi_renesas_driving_cmd, ARRAY_SIZE(hitachi_renesas_driving_cmd));
		client_data->remote_write(client_data, 0x0, 0x29);

	}else {
		client_data->remote_write(client_data, 0x24, 0x5300);
		client_data->remote_write(client_data, 0x0A, 0x22C0);
		hr_msleep(30);
		vivow_backlight_switch(LED_FULL);
	}
	client_data->auto_hibernate(client_data, 1);
	return 0;
}

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = vivow_mddi_init,
	.uninit = vivow_mddi_uninit,
	.blank = vivow_panel_blank,
	.unblank = vivow_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 52,
		.height = 86,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
		.vsync_gpio = 30,
	},
};

static struct msm_mddi_bridge_platform_data renesas_client_data = {
	.init = vivow_mddi_init,
	.uninit = vivow_mddi_uninit,
	.blank = vivow_panel_blank,
	.unblank = vivow_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.width = 52,
		.height = 86,
		.output_format = 0,
	},
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
		.vsync_gpio = 30,
	},
};

static void
mddi_power(struct msm_mddi_client_data *client_data, int on)
{
	int rc;
	unsigned config;
	B(KERN_DEBUG "%s(%d)\n", __func__, __LINE__);

	if (on) {
		if(axi_clk)
			clk_set_rate(axi_clk, 192000000);

		config = PCOM_GPIO_CFG(VIVOW_MDDI_TE, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID1, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID0, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);

		if(panel_type == PANEL_VIVOW_HITACHI) {
			vreg_enable(V_LCMIO_1V8);
			vreg_enable(V_LCM_2V85);
			hr_msleep(1);

			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(5);
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(5);
		} else {
			vreg_enable(V_LCM_2V85);
			hr_msleep(3);
			vreg_enable(V_LCMIO_1V8);
			hr_msleep(5);

			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(1);
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(1);
			gpio_set_value(VIVOW_LCD_RSTz, 1);
			hr_msleep(15);
		}

	} else {

		if(panel_type == PANEL_VIVOW_HITACHI) {
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(V_LCM_2V85);
			vreg_disable(V_LCMIO_1V8);
		} else {
			hr_msleep(80);
			gpio_set_value(VIVOW_LCD_RSTz, 0);
			hr_msleep(10);
			vreg_disable(V_LCMIO_1V8);
			vreg_disable(V_LCM_2V85);
		}

		config = PCOM_GPIO_CFG(VIVOW_MDDI_TE, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
		config = PCOM_GPIO_CFG(VIVOW_LCD_ID0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
		rc = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	}
}

static void mddi_fixup(uint16_t *mfr_name, uint16_t *product_code)
{
	printk(KERN_INFO "mddi fixup\n");
	if (panel_type == PANEL_VIVOW_SONY
		|| panel_type == PANEL_VIVOW_SONY_CUT2) {
		*mfr_name = 0xb9f6;
		*product_code = 0x5560;
	}else {
		*mfr_name = 0xb9f6;
		*product_code = 0x1408;
	}
}

static struct msm_mddi_platform_data mddi_pdata = {
	.fixup = mddi_fixup,
	.power_client = mddi_power,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5560),
			.name = "mddi_c_b9f6_5560",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			.product_id = (0xb9f6 << 16 | 0x1408),
			.name = "mddi_renesas_b9f6_61408",
			.id = 1,
			.client_data = &renesas_client_data,
			.clk_rate = 0,
		},
	},
};

static struct platform_driver vivow_backlight_driver = {
	.probe = vivow_backlight_probe,
	.driver = {
		.name = "nov_cabc",
		.owner = THIS_MODULE,
	},
};


int __init vivow_init_panel(unsigned int sys_rev)
{
	int rc;

	B(KERN_INFO "%s(%d): enter. panel_type 0x%08x\n", __func__, __LINE__, panel_type);

	//use dmap for hitachi panel
	if(panel_type == PANEL_VIVOW_HITACHI)
	{
		mdp_pdata.overrides = 0;
		pr_err("%s: mdp_pdata.overrides = 0\n", __func__);
	}

	msm_device_mdp.dev.platform_data = &mdp_pdata;
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	if (panel_type & DRIVER_IC_CUT2 || panel_type == PANEL_VIVOW_HITACHI)
		mddi_pdata.clk_rate = 384000000;
	else
		mddi_pdata.clk_rate = 256000000;

	mddi_pdata.type = MSM_MDP_MDDI_TYPE_II;

	axi_clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(axi_clk)) {
		pr_err("%s: failed to get axi clock\n", __func__);
		return PTR_ERR(axi_clk);
	}

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	rc = platform_device_register(&msm_device_mddi0);
	if (rc)
		return rc;

	if(panel_type == PANEL_VIVOW_HITACHI)
		vivow_backlight_driver.driver.name = "renesas_backlight";

	rc = platform_driver_register(&vivow_backlight_driver);
	if (rc)
		return rc;

	/* lcd panel power */
	/* 2.85V -- LDO20 */
	V_LCM_2V85 = vreg_get(NULL, "gp13");

	if (IS_ERR(V_LCM_2V85)) {
		pr_err("%s: LCM_2V85 get failed (%ld)\n",
			__func__, PTR_ERR(V_LCM_2V85));
		return -1;
	}
	V_LCMIO_1V8 = vreg_get(NULL, "lvsw0");

	if (IS_ERR(V_LCMIO_1V8)) {
		pr_err("%s: LCMIO_1V8 get failed (%ld)\n",
		       __func__, PTR_ERR(V_LCMIO_1V8));
		return -1;
	}

	return 0;
}
