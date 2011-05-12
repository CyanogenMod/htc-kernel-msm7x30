/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <media/msm_camera_sensor.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include "ov8810.h"


/* CAMIF output resolutions */
/* 816x612, 24MHz MCLK 96MHz PCLK */
#define OV8810_FULL_SIZE_DUMMY_PIXELS	0
#define OV8810_FULL_SIZE_DUMMY_LINES	0
#define OV8810_FULL_SIZE_WIDTH		3280
#define OV8810_FULL_SIZE_HEIGHT		2456

#define OV8810_QTR_SIZE_DUMMY_PIXELS	0
#define OV8810_QTR_SIZE_DUMMY_LINES	0
#define OV8810_QTR_SIZE_WIDTH		1632
#define OV8810_QTR_SIZE_HEIGHT		1224

#define OV8810_HRZ_FULL_BLK_PIXELS	696  /*stella 1203*/
#define OV8810_VER_FULL_BLK_LINES	44
#define OV8810_HRZ_QTR_BLK_PIXELS	890
#define OV8810_VER_QTR_BLK_LINES	44

static int cam_mode_sel = 0; /* 0: photo, 1: video@30fps, 2: video@24fps */
/* 240: 26, 365: 24, 589: 21 */
const int ov8810_ver_qtr_blk_lines_array[] = {44, 44, 365};

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8				0x00000100

/* Omnivision8810 product ID register address */
#define OV8810_PIDH_REG			0x300A
#define OV8810_PIDL_REG			0x300B

/* Omnivision8810 product ID */
#define OV8810_PID			0x88
/* Omnivision8810 version */
#define OV8810_VER			0x10

/* Time in milisecs for waiting for the sensor to reset */
#define OV8810_RESET_DELAY_MSECS	66

#define OV8810_DEFAULT_CLOCK_RATE	24000000

/* Registers*/
/* PLL Registers */
#define REG_PRE_PLL_CLK_DIV		0x3011 /*0x0305*/
#define REG_PLL_MULTIPLIER		0x3010	
#define REG_VT_CLK_DIV			0x300E	/*[7:4]VT_SYS_DIV, [3-0]VT_PIX_DIV*/	
#define REG_OP_CLK_DIV			0x300F	/*[7:4]OP_SYS_DIV, [3-0]OP_PIX_DIV*/

/* ISP Enable Control */
#define REG_ISP_ENABLE_CONTROL_00	0x3302
#define REG_ISP_ENABLE_CONTROL_01	0x3301

/* AWB Control */
#define REG_AWB_CTRL_0			0x3320
#define REG_AWB_CTRL_1			0x3321
#define REG_AWB_CTRL_2			0x3322
#define REG_AWB_CTRL_8			0x3328

/* Output Size */
#define REG_X_OUTPUT_SIZE_MSB		0x302C
#define REG_X_OUTPUT_SIZE_LSB		0x302D
#define REG_Y_OUTPUT_SIZE_MSB		0x302E
#define REG_Y_OUTPUT_SIZE_LSB		0x302F

/*Reserved register */
#define REG_BINNING_CONTROL		0x3091

/* Frame Fotmat */
#define REG_FRAME_LENGTH_LINES_MSB	0x3020
#define REG_FRAME_LENGTH_LINES_LSB	0x3021
#define REG_LINE_LENGTH_PCK_MSB		0x3022
#define REG_LINE_LENGTH_PCK_LSB		0x3023
#define REG_EXTRA_VSYNC_WIDTH_MSB	0x301E
#define REG_EXTRA_VSYNC_WIDTH_LSB	0x301F

#define REG_X_ADDR_START_HIGH		0x3024
#define REG_X_ADDR_START_LOW		0x3025
#define REG_Y_ADDR_START_HIGH		0x3026
#define REG_Y_ADDR_START_LOW		0x3027
#define REG_X_ADDR_END_HIGH		0x3028
#define REG_X_ADDR_END_LOW		0x3029
#define REG_Y_ADDR_END_HIGH		0x302A
#define REG_Y_ADDR_END_LOW		0x302B

/* Gain setting register */
#define OV8810_GAIN			0x3000
#define OV8810_AEC_MSB			0x3002
#define OV8810_AEC_LSB			0x3003

/* additional gain function provided by OV8810,
 * original gain can changed to 1x, 2x or 4x
 * to increase the gain that OV8810 can provide */
#define OV8810_REG_MUL_GAIN		0x3006
#define MUL_GAIN_INIT_VALUE		0x00

#define OV8810_MAX_EXPOSURE_GAIN	0x1FF

/* Mode select register */
#define OV8810_REG_MODE_SELECT		0x30FA	/* image system */
#define OV8810_MODE_SELECT_STREAM	0x01	/* start streaming */
#define OV8810_MODE_SELECT_SW_STANDBY	0x00	/* software standby */
#define OV8810_REG_SOFTWARE_RESET	0x3012	/* 0x0103 */
#define OV8810_SOFTWARE_RESET		0x80	/* 0x01 */

/* AF Total steps parameters */
#define OV8810_AF_MSB			0x30EC
#define OV8810_AF_LSB			0x30ED

#define OV8810_STEPS_NEAR_TO_CLOSEST_INF	42 /*43 stella0122 */
#define OV8810_TOTAL_STEPS_NEAR_TO_FAR		42 /*43 stella0122 */

/*Test pattern*/
/* Color bar pattern selection */
#define OV8810_COLOR_BAR_PATTERN_SEL_REG	0x307B

/* Color bar enabling control */
#define OV8810_COLOR_BAR_ENABLE_REG		0x307D

/* I2C Address of the Sensor */
#define OV8810_I2C_SLAVE_ID		0x6C

/*LSC table length*/
#define LSC_table_length 144
/*============================================================================
TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#if 0
typedef struct reg_addr_val_pair_struct {
	uint16_t reg_addr;
	uint8_t reg_val;
} reg_struct_type;
#endif

struct awb_lsc_struct_type {
       unsigned int caBuff[8];  /*awb_calibartion*/
	struct reg_addr_val_pair_struct LSC_table[150];  /*lsc_calibration*/
	uint32_t LSC_table_CRC;
};

enum ov8810_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum ov8810_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

/*LSC calibration*/
int global_mode;
/*TODO: should be use a header file to reference this function*/
extern unsigned char *get_cam_awb_cal(void);

static int sensor_probe_node = 0;
static int preview_frame_count = 0;

static struct wake_lock ov8810_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&ov8810_wake_lock, WAKE_LOCK_IDLE, "ov8810");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&ov8810_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&ov8810_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&ov8810_wake_lock);
}

/*============================================================================
DATA DECLARATIONS
============================================================================*/

/*  96MHz PCLK @ 24MHz MCLK inc*/
 /*stella1223 start*/
static struct reg_addr_val_pair_struct ov8810_init_settings_array[] =
{
	/* Sensor clk setup */
	{REG_OP_CLK_DIV, 0x04},
	{REG_VT_CLK_DIV, 0x05},
#if 1   /* weiting0414 prevent capture hang restore CLK */
	{REG_PLL_MULTIPLIER, 0x28}, /*0x28 96MHz PCLK 0x18 64MHz PCLK*/
	{REG_PRE_PLL_CLK_DIV, 0x22},
#else
        {REG_PLL_MULTIPLIER, 0x14}, /*Reduce internal clock to prevent hang Weiting0331*/
        {REG_PRE_PLL_CLK_DIV, 0x21},
#endif
	{OV8810_GAIN, 8}, /*0x30},*/
	{OV8810_AEC_MSB, 0x04},
	{OV8810_AEC_LSB, 0xc4}, /*stella 1203*/
	{REG_ISP_ENABLE_CONTROL_00, 0x20},
	{0x30b2, 0x13}, /*driving strength*/
	{0x30a0, 0x40},
	{0x3098, 0x24},
	{0x3099, 0x81},
	{0x309a, 0x64},
	{0x309b, 0x00},
	{0x309d, 0x64},
	{0x309e, 0x2d},
	{REG_AWB_CTRL_0, 0xc2}, /*set wb manual*/
	{REG_AWB_CTRL_1, 0x02},
	{REG_AWB_CTRL_2, 0x04},
	{REG_AWB_CTRL_8, 0x40},
	{0x3329, 0xe3}, /*00},*/  /*stella 1203*/
	{0x3306, 0x00},
	{0x3316, 0x03},
	{0x3079, 0x0a},
	/*stella 1203*/
	{0x3058, 0x01},
	{0x3059, 0xa0},
	{0x306b, 0x00},
	{0x3065, 0x50},
	{0x3067, 0x40},
	{0x3069, 0x80},
	{0x3071, 0x40},/*50 BLC trigger by gain 40 BLC every frame */
	{0x3300, 0xef},
	{0x3334, 0x02},
	{0x3331, 0x08}, /*BLC level 8813*/ /*stella 1203*/
	{0x3332, 0x08}, /*8813*/
	{0x3333, 0x41},
       /*Stella1221 for adding init size */
       {0x30f8, 0x45},
	{REG_FRAME_LENGTH_LINES_MSB,
	((OV8810_QTR_SIZE_HEIGHT + OV8810_VER_QTR_BLK_LINES) & 0xFF00) >> 8},
	{REG_FRAME_LENGTH_LINES_LSB,
	((OV8810_QTR_SIZE_HEIGHT + OV8810_VER_QTR_BLK_LINES) & 0x00FF)},
	{REG_LINE_LENGTH_PCK_MSB,
	((OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS) & 0xFF00) >> 8},
	{REG_LINE_LENGTH_PCK_LSB,
	((OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS) & 0x00FF)},
	{REG_X_ADDR_START_HIGH, 0x00},
	{REG_X_ADDR_START_LOW, 0x04}, /*stella 1203*/
	{REG_Y_ADDR_START_HIGH, 0x00},
	{REG_Y_ADDR_START_LOW, 0x00},
	{REG_X_ADDR_END_HIGH, 0x0c},
	{REG_X_ADDR_END_LOW, 0xdb}, /*stella 1203*/
	{REG_Y_ADDR_END_HIGH, 0x09},
	{REG_Y_ADDR_END_LOW, 0x9f},
	{REG_X_OUTPUT_SIZE_MSB, (OV8810_QTR_SIZE_WIDTH & 0xFF00) >> 8},
	{REG_X_OUTPUT_SIZE_LSB, (OV8810_QTR_SIZE_WIDTH & 0x00FF)},
	{REG_Y_OUTPUT_SIZE_MSB, (OV8810_QTR_SIZE_HEIGHT & 0xFF00) >> 8},
	{REG_Y_OUTPUT_SIZE_LSB, (OV8810_QTR_SIZE_HEIGHT & 0x00FF)},
	/*Stella1221 for adding init size */
	/* {REG_BINNING_CONTROL, 0x00},*/ /*stella 1203*/
	{OV8810_REG_MUL_GAIN, MUL_GAIN_INIT_VALUE},
	{0x3082, 0x80},
	{0x331e, 0x94},
	{0x331f, 0x6e},
	{0x3092, 0x00},
	{0x3094, 0x01},
	{0x3090, 0x2b}, /* for AN version 8a */ /*changed by Stella for 8813*/
	{0x30ab, 0x44},
	{0x3095, 0x0a},
	{0x308d, 0x00},
	{0x3082, 0x00},
	{0x3080, 0x40},
	{0x30aa, 0x59},
	{0x30a9, 0x00},
	{0x30be, 0x08},
	{0x309f, 0x23},
	{0x3065, 0x40},
	{0x3068, 0x00},
	{0x30bf, 0x80},
	{0x309c, 0x00},
	{0x3084, 0x44}, /*added by stella for 8813*/
	{0x3016, 0x03}, /*added by stella for 8813*/
	{0x30e9, 0x09}, /*changed by stella for 8813*/
	{0x3075, 0x29},
	{0x3076, 0x29},
	{0x3077, 0x29},
	{0x3078, 0x29},
	{0x306a, 0x05},
	{0x3015, 0x33}, /*changed by stella for 8813*/
	/*stella 1203 start*/
	{0x3090, 0x36},
	{0x333e, 0x00},
	{0x306a, 0x05},
	/*stella 1203 end*/
	{0x3087, 0x41},
	{0x3090, 0x97}, /*99, QCT=97*/
	{0x309e, 0x1b},
	{0x30e3, 0x0e},
	{0x30f0, 0x00},
	{0x30f2, 0x00},
	{0x30f4, 0x90},
	/*stella 1203 start*/
	{0x3347, 0x00},
	{0x3347, 0x00},
#if 0
	{0x3092, 0x00}, //marked by QCT
	{0x30f0, 0x10}, //marked by QCT
	{0x30f1, 0x56}, //marked by QCT
	{0x30fb, 0x8e}, //marked by QCT
	{0x30f3, 0xa7}, //marked by QCT
#endif
	{0x3091, 0x08}, /*QCT for 8813*/
	{0x3090, 0x97}, /*QCT for 8813*/
	{0x30fb, 0xc9}, /*QCT for 8813*/
	{0x308d, 0x02},
	{0x30e7, 0x41},
	{0x30b3, 0x08},
	{0x33e5, 0x00},  /*30e5*/
	{0x350e, 0x40},  /*305e*/
	{0x301f, 0x00},
	{0x309f, 0x23},
	{0x3013, 0xc0},
	{0x30e1, 0x90},
	{0x3058, 0x01},
	{0x3500, 0x40}, /* vsync_new */
	{REG_BINNING_CONTROL, 0x00}, /*stella 0126*/
	/*stella 1203 end*/
};

/*Vincent for LSC calibration*/
static struct reg_addr_val_pair_struct lsc_table_array[] =
{
	{0x3358, 0x1f  },//{0x3358, 0x18},
	{0x3359, 0x14  },//{0x3359, 0x0f},
	{0x335a, 0x0f  },//{0x335a, 0x0c},
	{0x335b, 0x0d  },//{0x335b, 0x0a},
	{0x335c, 0x0d  },//{0x335c, 0x0a},
	{0x335d, 0x0f  },//{0x335d, 0x0b},
	{0x335e, 0x14  },//{0x335e, 0x0d},
	{0x335f, 0x1d  },//{0x335f, 0x15},
	{0x3360, 0x0f  },//{0x3360, 0x0b},
	{0x3361, 0x0a  },//{0x3361, 0x09},
	{0x3362, 0x07  },//{0x3362, 0x06},
	{0x3363, 0x06  },//{0x3363, 0x05},
	{0x3364, 0x06  },//{0x3364, 0x05},
	{0x3365, 0x07  },//{0x3365, 0x06},
	{0x3366, 0x09  },//{0x3366, 0x08},
	{0x3367, 0x0d  },//{0x3367, 0x0b},
	{0x3368, 0x09  },//{0x3368, 0x07},
	{0x3369, 0x06  },//{0x3369, 0x05},
	{0x336a, 0x04  },//{0x336a, 0x03},
	{0x336b, 0x03  },//{0x336b, 0x02},
	{0x336c, 0x03  },//{0x336c, 0x02},
	{0x336d, 0x04  },//{0x336d, 0x03},
	{0x336e, 0x06  },//{0x336e, 0x04},
	{0x336f, 0x09  },//{0x336f, 0x06},
	{0x3370, 0x07  },//{0x3370, 0x05},
	{0x3371, 0x04  },//{0x3371, 0x04},
	{0x3372, 0x01  },//{0x3372, 0x01},
	{0x3373, 0x00  },//{0x3373, 0x00},
	{0x3374, 0x00  },//{0x3374, 0x00},
	{0x3375, 0x01  },//{0x3375, 0x01},
	{0x3376, 0x04  },//{0x3376, 0x03},
	{0x3377, 0x07  },//{0x3377, 0x05},
	{0x3378, 0x08  },//{0x3378, 0x05},
	{0x3379, 0x04  },//{0x3379, 0x03},
	{0x337a, 0x01  },//{0x337a, 0x01},
	{0x337b, 0x00  },//{0x337b, 0x00},
	{0x337c, 0x00  },//{0x337c, 0x00},
	{0x337d, 0x01  },//{0x337d, 0x00},
	{0x337e, 0x04  },//{0x337e, 0x02},
	{0x337f, 0x07  },//{0x337f, 0x05},
	{0x3380, 0x09  },//{0x3380, 0x06},
	{0x3381, 0x06  },//{0x3381, 0x04},
	{0x3382, 0x04  },//{0x3382, 0x03},
	{0x3383, 0x02  },//{0x3383, 0x02},
	{0x3384, 0x02  },//{0x3384, 0x01},
	{0x3385, 0x04  },//{0x3385, 0x02},
	{0x3386, 0x06  },//{0x3386, 0x03},
	{0x3387, 0x09  },//{0x3387, 0x05},
	{0x3388, 0x0f  },//{0x3388, 0x0a},
	{0x3389, 0x0a  },//{0x3389, 0x07},
	{0x338a, 0x07  },//{0x338a, 0x05},
	{0x338b, 0x07  },//{0x338b, 0x04},
	{0x338c, 0x07  },//{0x338c, 0x04},
	{0x338d, 0x07  },//{0x338d, 0x05},
	{0x338e, 0x0a  },//{0x338e, 0x06},
	{0x338f, 0x0f  },//{0x338f, 0x09},
	{0x3390, 0x1d  },//{0x3390, 0x12},
	{0x3391, 0x12  },//{0x3391, 0x0d},
	{0x3392, 0x0d  },//{0x3392, 0x09},
	{0x3393, 0x0b  },//{0x3393, 0x08},
	{0x3394, 0x0b  },//{0x3394, 0x08},
	{0x3395, 0x0d  },//{0x3395, 0x09},
	{0x3396, 0x12  },//{0x3396, 0x0c},
	{0x3397, 0x1a  },//{0x3397, 0x11},
	{0x3398, 0x0f  },//{0x3398, 0x10},
	{0x3399, 0x0d  },//{0x3399, 0x10},
	{0x339a, 0x0e  },//{0x339a, 0x10},
	{0x339b, 0x0f  },//{0x339b, 0x0e},
	{0x339c, 0x11  },//{0x339c, 0x0e},
	{0x339d, 0x0d  },//{0x339d, 0x0f},
	{0x339e, 0x12  },//{0x339e, 0x0e},
	{0x339f, 0x0e  },//{0x339f, 0x0f},
	{0x33a0, 0x0f  },//{0x33a0, 0x0f},
	{0x33a1, 0x0f  },//{0x33a1, 0x0f},
	{0x33a2, 0x10  },//{0x33a2, 0x0f},
	{0x33a3, 0x10  },//{0x33a3, 0x10},
	{0x33a4, 0x0f  },//{0x33a4, 0x0e},
	{0x33a5, 0x0d  },//{0x33a5, 0x10},
	{0x33a6, 0x0f  },//{0x33a6, 0x11},
	{0x33a7, 0x10  },//{0x33a7, 0x10},
	{0x33a8, 0x10  },//{0x33a8, 0x10},
	{0x33a9, 0x0f  },//{0x33a9, 0x0f},
	{0x33aa, 0x10  },//{0x33aa, 0x0e},
	{0x33ab, 0x0e  },//{0x33ab, 0x0f},
	{0x33ac, 0x10  },//{0x33ac, 0x10},
	{0x33ad, 0x11  },//{0x33ad, 0x10},
	{0x33ae, 0x11  },//{0x33ae, 0x10},
	{0x33af, 0x0f  },//{0x33af, 0x0f},
	{0x33b0, 0x0f  },//{0x33b0, 0x0e},
	{0x33b1, 0x0d  },//{0x33b1, 0x0f},
	{0x33b2, 0x0d  },//{0x33b2, 0x0f},
	{0x33b3, 0x0e  },//{0x33b3, 0x0f},
	{0x33b4, 0x0f  },//{0x33b4, 0x0f},
	{0x33b5, 0x10  },//{0x33b5, 0x0f},
	{0x33b6, 0x12  },//{0x33b6, 0x0e},
	{0x33b7, 0x0d  },//{0x33b7, 0x0d},
	{0x33b8, 0x0c  },//{0x33b8, 0x0c},
	{0x33b9, 0x0c  },//{0x33b9, 0x0c},
	{0x33ba, 0x0c  },//{0x33ba, 0x0d},
	{0x33bb, 0x0b  },//{0x33bb, 0x0f},
	{0x33bc, 0x1b  },//{0x33bc, 0x16},
	{0x33bd, 0x1b  },//{0x33bd, 0x17},
	{0x33be, 0x1d  },//{0x33be, 0x17},
	{0x33bf, 0x1d  },//{0x33bf, 0x17},
	{0x33c0, 0x1e  },//{0x33c0, 0x17},
	{0x33c1, 0x1c  },//{0x33c1, 0x14},
	{0x33c2, 0x1a  },//{0x33c2, 0x17},
	{0x33c3, 0x17  },//{0x33c3, 0x14},
	{0x33c4, 0x15  },//{0x33c4, 0x13},
	{0x33c5, 0x16  },//{0x33c5, 0x13},
	{0x33c6, 0x19  },//{0x33c6, 0x14},
	{0x33c7, 0x1e  },//{0x33c7, 0x15},
	{0x33c8, 0x16  },//{0x33c8, 0x15},
	{0x33c9, 0x12  },//{0x33c9, 0x12},
	{0x33ca, 0x10  },//{0x33ca, 0x10},
	{0x33cb, 0x10  },//{0x33cb, 0x10},
	{0x33cc, 0x14  },//{0x33cc, 0x12},
	{0x33cd, 0x19  },//{0x33cd, 0x14},
	{0x33ce, 0x16  },//{0x33ce, 0x15},
	{0x33cf, 0x12  },//{0x33cf, 0x12},
	{0x33d0, 0x10  },//{0x33d0, 0x10},
	{0x33d1, 0x11  },//{0x33d1, 0x10},
	{0x33d2, 0x14  },//{0x33d2, 0x12},
	{0x33d3, 0x1a  },//{0x33d3, 0x14},
	{0x33d4, 0x18  },//{0x33d4, 0x16},
	{0x33d5, 0x15  },//{0x33d5, 0x13},
	{0x33d6, 0x13  },//{0x33d6, 0x12},
	{0x33d7, 0x14  },//{0x33d7, 0x12},
	{0x33d8, 0x17  },//{0x33d8, 0x13},
	{0x33d9, 0x1b  },//{0x33d9, 0x15},
	{0x33da, 0x18  },//{0x33da, 0x18},
	{0x33db, 0x1a  },//{0x33db, 0x15},
	{0x33dc, 0x1b  },//{0x33dc, 0x15},
	{0x33dd, 0x1b  },//{0x33dd, 0x15},
	{0x33de, 0x1b  },//{0x33de, 0x15},
	{0x33df, 0x1c  },//{0x33df, 0x14},
	{0x3350, 0x06  },//{0x3350, 0x06},
	{0x3351, 0xab  },//{0x3351, 0xab},
	{0x3352, 0x05  },//{0x3352, 0x05},
	{0x3353, 0x00  },//{0x3353, 0x00},
	{0x3354, 0x04  },//{0x3354, 0x04},
	{0x3355, 0xf8  },//{0x3355, 0xf8},
	{0x3356, 0x07  },//{0x3356, 0x07},
	{0x3357, 0x74  },//{0x3357, 0x74},
	/* lsc setting on sensor*/
	{0x3300, 0xff}, /*enable lsc on sensor*/
	/*move to the last*/
	{OV8810_REG_MODE_SELECT, OV8810_MODE_SELECT_STREAM},
};

/*1632x1224; 24MHz MCLK 96MHz PCLK*/
static struct reg_addr_val_pair_struct ov8810_qtr_settings_array[] =
{
	{0x30f8, 0x45},
	{REG_FRAME_LENGTH_LINES_MSB,
	((OV8810_QTR_SIZE_HEIGHT + OV8810_VER_QTR_BLK_LINES) & 0xFF00) >> 8},
	{REG_FRAME_LENGTH_LINES_LSB,
	((OV8810_QTR_SIZE_HEIGHT + OV8810_VER_QTR_BLK_LINES) & 0x00FF)},
	{REG_LINE_LENGTH_PCK_MSB,
	((OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS) & 0xFF00) >> 8},
	{REG_LINE_LENGTH_PCK_LSB,
	((OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS) & 0x00FF)},
	{REG_X_ADDR_START_HIGH, 0x00},
	{REG_X_ADDR_START_LOW, 0x04}, /*stella 1203*/
	{REG_Y_ADDR_START_HIGH, 0x00},
	{REG_Y_ADDR_START_LOW, 0x00},
	{REG_X_ADDR_END_HIGH, 0x0c},
	{REG_X_ADDR_END_LOW, 0xd8}, /*stella 1203=db*/ /*QCT:d8*/
	{REG_Y_ADDR_END_HIGH, 0x09},
	{REG_Y_ADDR_END_LOW, 0x9f},
	{REG_X_OUTPUT_SIZE_MSB, (OV8810_QTR_SIZE_WIDTH & 0xFF00) >> 8},
	{REG_X_OUTPUT_SIZE_LSB, (OV8810_QTR_SIZE_WIDTH & 0x00FF)},
	{REG_Y_OUTPUT_SIZE_MSB, (OV8810_QTR_SIZE_HEIGHT & 0xFF00) >> 8},
	{REG_Y_OUTPUT_SIZE_LSB, (OV8810_QTR_SIZE_HEIGHT & 0x00FF)},
	 /*stella1202 for capture over exposure issue due to user space use 2X line count*/
	{0x3068, 0x00},  /*changed for color edge, stella 1203*/
	{0x307e, 0x00},
	{0x3071, 0x40},/*50 BLC trigger by gain 40 BLC every frame */
	{REG_ISP_ENABLE_CONTROL_01, 0x0B},
	{REG_BINNING_CONTROL, 0x00}, //stella0127
	{0x331c, 0x00},
	{0x331d, 0x00},
	{0x308a, 0x02},
	{0x3072, 0x0d},
	{0x3319, 0x04},
	{0x309e, 0x09},
	{0x300e, 0x05},
	{0x300f, 0x04},
	{0x33e4, 0x07},  /*lsc for 2:1 down sampling*/
};

 /*stella1223 end*/

/* 3280x2456 Sensor Raw; 24MHz MCLK 96MHz PCLK*/
static struct reg_addr_val_pair_struct ov8810_full_settings_array[] =
{
	{0x30f8, 0x40},
	{REG_FRAME_LENGTH_LINES_MSB,
	((OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES) & 0xFF00) >> 8},
	{REG_FRAME_LENGTH_LINES_LSB,
	((OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES) & 0x00FF)},
	{REG_LINE_LENGTH_PCK_MSB,
	((OV8810_FULL_SIZE_WIDTH + OV8810_HRZ_FULL_BLK_PIXELS) & 0xFF00) >> 8},
	{REG_LINE_LENGTH_PCK_LSB,
	((OV8810_FULL_SIZE_WIDTH + OV8810_HRZ_FULL_BLK_PIXELS) & 0x00FF)},
	{REG_X_ADDR_START_HIGH, 0x00},
	{REG_X_ADDR_START_LOW, 0x02}, /*stella 1203*/
	{REG_Y_ADDR_START_HIGH, 0x00},
	{REG_Y_ADDR_START_LOW, 0x00},
	{REG_X_ADDR_END_HIGH, 0x0c},
	{REG_X_ADDR_END_LOW, 0xdd}, /*stella 1203*/
	{REG_Y_ADDR_END_HIGH, 0x09},
	{REG_Y_ADDR_END_LOW, 0x9f},
	{REG_X_OUTPUT_SIZE_MSB, (OV8810_FULL_SIZE_WIDTH & 0xFF00) >> 8},
	{REG_X_OUTPUT_SIZE_LSB, (OV8810_FULL_SIZE_WIDTH & 0x00FF)},
	{REG_Y_OUTPUT_SIZE_MSB, (OV8810_FULL_SIZE_HEIGHT & 0xFF00) >> 8},
	{REG_Y_OUTPUT_SIZE_LSB, (OV8810_FULL_SIZE_HEIGHT & 0x00FF)},
	/*stella1202 for capture over exposure issue
	  due to user space use 2X line count */
	{0x3068, 0x00}, /* changed for color edge stella 1203*/
	{0x307e, 0x00},
	{REG_ISP_ENABLE_CONTROL_01, 0x0B},
	{REG_BINNING_CONTROL, 0x00}, //stella0127
	{0x331c, 0x28},
	{0x331d, 0x21},
	{0x308a, 0x01},
	{0x3072, 0x01},
	{0x3319, 0x06},
	{0x309e, 0x1b},
	{0x300e, 0x05},
	{0x300f, 0x04},
	{0x33e4, 0x02},  /*lsc for full resolution*/
};

/* AF Tuning Parameters */

static uint16_t ov8810_step_position_table[OV8810_TOTAL_STEPS_NEAR_TO_FAR+1];

static uint8_t ov8810_damping_threshold = 10;
static uint8_t ov8810_damping_course_step = 4;
static uint8_t ov8810_damping_fine_step = 10;
static uint8_t ov8810_damping_time_wait;
static uint16_t ov8810_focus_debug; /*don't init to 0*/
static uint16_t ov8810_use_default_damping = 1;
static uint16_t ov8810_use_threshold_damping = 1; /*set to FALSE if too slow*/
/*static uint32_t stored_line_length_ratio = 1 * Q8;*/

/*Andy1217 write Line 1 frame ealier before Gain*/
struct backup_line_gain_struct {
	uint32_t line;
	uint8_t mul;
	uint16_t gain;
	uint32_t extra_line_length;
};

static struct backup_line_gain_struct backup_line_gain[2];

static uint16_t write_cnt;
static uint16_t updated_BLC; /* only set to 0x50 after 1st update again*/

uint8_t S3_to_0 = 0x1; /* 0x9 */

/* static Variables*/
static uint16_t step_position_table[OV8810_TOTAL_STEPS_NEAR_TO_FAR+1];


/* FIXME: Changes from here */
struct ov8810_work {
	struct work_struct work;
};

static struct  ov8810_work *ov8810_sensorw;
static struct  i2c_client *ov8810_client;

static struct vreg *vreg_af_actuator;

struct ov8810_ctrl {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider; 		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; 	/* init to 1 * 0x00000400 */
	uint16_t fps;

	int16_t  curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum ov8810_resolution_t prev_res;
	enum ov8810_resolution_t pict_res;
	enum ov8810_resolution_t curr_res;
	enum ov8810_test_mode_t  set_test;

	unsigned short imgaddr;
};


static struct ov8810_ctrl *ov8810_ctrl;
static struct platform_device *ov8810_pdev;

struct ov8810_waitevent{
	uint32_t waked_up;
	wait_queue_head_t event_wait;
};
static struct ov8810_waitevent ov8810_event;

static DECLARE_WAIT_QUEUE_HEAD(ov8810_wait_queue);
DECLARE_MUTEX(ov8810_sem);


/*=============================================================*/

static int ov8810_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr  = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};
	CDBG("%s: saddr=0x%X\n", __func__, saddr);
	CDBG("%s: raddr=0x%X\n", __func__, *rxdata);

	if (i2c_transfer(ov8810_client->adapter, msgs, 2) < 0) {
		pr_err("[CAM]ov8810_i2c_rxdata failed!\n");
		return -EIO;
	}
	CDBG("%s: rxdata=0x%X\n", __func__, *rxdata);

	return 0;
}
static int32_t ov8810_i2c_txdata(unsigned short saddr,
				 unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = length,
		 .buf = txdata,
		 },
	};
	if (i2c_transfer(ov8810_client->adapter, msg, 1) < 0) {
		pr_err("[CAM]ov8810_i2c_txdata faild 0x%x\n", ov8810_client->addr);
		return -EIO;
	}

	return 0;
}


static int32_t ov8810_i2c_read(unsigned short raddr,
				unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	int count = 0;
	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
retry:
	rc = ov8810_i2c_rxdata(ov8810_client->addr, buf, rlen);

	if (rc < 0) {
		pr_err("[CAM]ov8810_i2c_read 0x%x failed!\n", raddr);
		printk(KERN_ERR "starting read retry policy count:%d\n", count);
		udelay(10);
		count++;
		if (count < 20) {
			if (count > 10)
				udelay(100);
		} else
			return rc;
		goto retry;
	}

	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	return rc;
}


static int32_t ov8810_i2c_write_b(unsigned short saddr,
				unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	int count = 0;
	CDBG("i2c_write_w_b, addr = 0x%x, val = 0x%x!\n", waddr, bdata);

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

retry:
	CDBG("i2c_write_b addr = %d, val = %d\n", waddr, bdata);
	rc = ov8810_i2c_txdata(saddr, buf, 3);

	if (rc < 0) {
		pr_err("[CAM]i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			 waddr, bdata);
		pr_err(KERN_ERR "starting read retry policy count:%d\n", count);
		udelay(10);
		count++;
		if (count < 20) {
			if (count > 10)
				udelay(100);
		} else
			return rc;
		goto retry;
	}

	return rc;
}


/*for LSC calibration*/
static int ov8810_update_lsc_table(struct sensor_cfg_data *cdata)
{
	int i = 0;
	pr_info("[CAM][LSC calibration]ov8810_update_lsc_table\n");
	for (i = 0; i < 144; i++) {
		ov8810_i2c_write_b(
			ov8810_client->addr,
			cdata->cfg.lsctable.lsc_table[i].reg_addr,
			cdata->cfg.lsctable.lsc_table[i].reg_val);
		pr_info("[CAM][LSC calibration]update_lsc_table: 0x%x, 0x%x\n",
				cdata->cfg.lsctable.lsc_table[i].reg_addr,
				cdata->cfg.lsctable.lsc_table[i].reg_val);
	}
	/*enable lsc on sensor*/
	ov8810_i2c_write_b(ov8810_client->addr, 0x3300, 0xff);
	/*mirror on*/
	ov8810_i2c_write_b(ov8810_client->addr, 0x30f8, 0x45);
	/*mirror on*/
	ov8810_i2c_write_b(ov8810_client->addr, 0x3316, 0x03);
	return 1;

}

/*20100330 vincent for LSC calibration*/
static int ov8810_LSC_calibration_set_rawflag(struct sensor_cfg_data *cdata)
{
	global_mode = 1;
	return 1;
}

#define MAX_FUSE_ID_INFO 11
static int ov8810_i2c_read_fuseid(struct sensor_cfg_data *cdata)
{
	unsigned short fuse_id[MAX_FUSE_ID_INFO];
	int count = 0;

	ov8810_i2c_write_b(ov8810_client->addr, 0x30d5, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30d6, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30d7, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30d8, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30d9, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30da, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30db, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30dc, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30dd, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30de, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x30df, 0xff);
	ov8810_i2c_write_b(ov8810_client->addr, 0x303e, 0x55);
	ov8810_i2c_read(0x30d5, &fuse_id[0], 2);
	ov8810_i2c_read(0x30d6, &fuse_id[1], 2);
	ov8810_i2c_read(0x30d7, &fuse_id[2], 2);
	ov8810_i2c_read(0x30d8, &fuse_id[3], 2);
	ov8810_i2c_read(0x30d9, &fuse_id[4], 2);
	ov8810_i2c_read(0x30da, &fuse_id[5], 2);
	ov8810_i2c_read(0x30db, &fuse_id[6], 2);
	ov8810_i2c_read(0x30dc, &fuse_id[7], 2);
	ov8810_i2c_read(0x30dd, &fuse_id[8], 2);
	ov8810_i2c_read(0x30de, &fuse_id[9], 2);
	ov8810_i2c_read(0x30df, &fuse_id[10], 2);
	cdata->cfg.fuse.fuse_id_word1 = (uint32_t) fuse_id[0];
	cdata->cfg.fuse.fuse_id_word2 = (uint32_t) fuse_id[1];
	cdata->cfg.fuse.fuse_id_word3 = 0;
	cdata->cfg.fuse.fuse_id_word4 = 0;
	for (count = 0; count < MAX_FUSE_ID_INFO; count++)
		pr_info("[CAM]Ov8810 Get fuse: fuse_id[%d]: %x\n",
			count, fuse_id[count]);
	return 0;
}


static int32_t ov8810_af_i2c_write(uint16_t data)
{
	uint8_t code_val_msb, code_val_lsb; /* S3_to_0; */
	int32_t rc = 0;
	/* S3_to_0 = 0x9;  S[3:0] */
	code_val_msb = data >> 4; /* D[9:4] */
	code_val_lsb = ((data & 0x000F) << 4) | S3_to_0;

	CDBG("code value = %d ,D[9:4] = %d ,D[3:0] = %d\n",
		data, code_val_msb, code_val_lsb);
	rc = ov8810_i2c_write_b(ov8810_client->addr,
				OV8810_AF_MSB, code_val_msb);

	if (rc < 0) {
		pr_err("[CAM]Unable to write code_val_msb = %d\n", code_val_msb);
		return rc;
	}

	rc = ov8810_i2c_write_b(ov8810_client->addr,
				OV8810_AF_LSB, code_val_lsb);
	if (rc < 0) {
		pr_err("[CAM]Unable to write code_val_lsb = %disclaimer\n",
			code_val_lsb);
		return rc;
	}

	return rc;
} /* ov8810_af_i2c_write */

static int32_t ov8810_move_focus(int direction, int32_t num_steps)
{

	int8_t step_direction;
	int8_t dest_step_position;
	uint16_t dest_lens_position, target_dist, small_step;
	int16_t next_lens_position;
	int32_t rc = 0;

	if (num_steps == 0) {
		return rc;
	}

	if (direction == MOVE_NEAR) {
		step_direction = 1;
	} else if (direction == MOVE_FAR) {
		step_direction = -1;
	} else {
		pr_err("[CAM]Illegal focus direction\n");
		return -EINVAL;; /* CAMERA_INVALID_PARM; */
	}

	CDBG("%s, interpolate\n", __func__);
	dest_step_position =
		ov8810_ctrl->curr_step_pos + (step_direction * num_steps);

	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > OV8810_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = OV8810_TOTAL_STEPS_NEAR_TO_FAR;

	dest_lens_position = ov8810_step_position_table[dest_step_position];

	/* Taking small damping steps */
	target_dist = step_direction *
		(dest_lens_position - ov8810_ctrl->curr_lens_pos);

	if (target_dist == 0) {
		return rc;
	}

	if (ov8810_use_threshold_damping &&
		(step_direction < 0) &&
		(target_dist >=
		ov8810_step_position_table[ov8810_damping_threshold])) {

		/* change to variable */
		small_step = (uint16_t)(target_dist/ov8810_damping_fine_step);
		ov8810_damping_time_wait = 1;
	} else {
		small_step = (uint16_t)(target_dist/ov8810_damping_course_step);
		ov8810_damping_time_wait = 4;
	}

	for (next_lens_position =
		ov8810_ctrl->curr_lens_pos + (step_direction * small_step);
		(step_direction * next_lens_position) <=
		(step_direction * dest_lens_position);
		next_lens_position += (step_direction * small_step)) {

		if (ov8810_af_i2c_write(next_lens_position) < 0)
			return -EBUSY;

		ov8810_ctrl->curr_lens_pos = next_lens_position;

		if (ov8810_ctrl->curr_lens_pos != dest_lens_position) {
			mdelay(ov8810_damping_time_wait);
		}
	}

	if (ov8810_ctrl->curr_lens_pos != dest_lens_position) {

		if (ov8810_af_i2c_write(dest_lens_position) < 0) {
			return -EBUSY;
		}
	}

	/* Storing the current lens Position */
	ov8810_ctrl->curr_lens_pos = dest_lens_position;
	ov8810_ctrl->curr_step_pos = dest_step_position;

	CDBG("done\n");
	return rc;
}

static int32_t ov8810_set_default_focus(uint8_t af_step)
{
	int16_t position;
	int32_t rc = 0;
	ov8810_damping_time_wait = 4;

	if (ov8810_use_default_damping) {

		/* when lens is uninitialized */
		if (ov8810_ctrl->curr_lens_pos == -1
			|| (ov8810_focus_debug == 1)) {

		position = ov8810_step_position_table[ov8810_damping_threshold];
		rc =  ov8810_af_i2c_write(position);

		if (rc < 0) {
			return rc;
		}

		ov8810_ctrl->curr_step_pos = ov8810_damping_threshold;
		ov8810_ctrl->curr_lens_pos = position;
		mdelay(ov8810_damping_time_wait);
		}

		rc = ov8810_move_focus(MOVE_FAR, ov8810_ctrl->curr_step_pos);
		if (rc < 0)
			return rc;
	} else {
		rc = ov8810_af_i2c_write(ov8810_step_position_table[0]);
		if (rc < 0)
			return rc;

		ov8810_ctrl->curr_step_pos = 0;
		ov8810_ctrl->curr_lens_pos = ov8810_step_position_table[0];
	}

	return rc;
}


static void ov8810_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */

	uint32_t divider, d1, d2;

	uint16_t snapshot_height, preview_height, preview_width, snapshot_width;

	if (ov8810_ctrl->prev_res == QTR_SIZE) {
		preview_width =
			OV8810_QTR_SIZE_WIDTH  + OV8810_HRZ_QTR_BLK_PIXELS ;
		preview_height =
			OV8810_QTR_SIZE_HEIGHT + ov8810_ver_qtr_blk_lines_array[cam_mode_sel] ;
	} else {
		/* full size resolution used for preview. */
		preview_width =
			OV8810_FULL_SIZE_WIDTH + OV8810_HRZ_FULL_BLK_PIXELS ;
		preview_height =
			OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES ;
	}

	if (ov8810_ctrl->pict_res == QTR_SIZE) {
		snapshot_width =
			OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS ;
		snapshot_height =
			OV8810_QTR_SIZE_HEIGHT + ov8810_ver_qtr_blk_lines_array[cam_mode_sel] ;

	} else {
		snapshot_width =
			OV8810_FULL_SIZE_WIDTH + OV8810_HRZ_FULL_BLK_PIXELS;
		snapshot_height =
			OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES;
	}

	d1 = preview_height * 0x00000400 / snapshot_height;
	d2 = preview_width * 0x00000400 / snapshot_width;

	divider = (uint32_t) (d1 * d2) / 0x00000400;
	*pfps = (uint16_t)(fps * divider / 0x00000400);

} /* endof ov8810_get_pict_fps */

static uint16_t ov8810_get_prev_lines_pf(void)
{
	if (ov8810_ctrl->prev_res == QTR_SIZE) {
		return (OV8810_QTR_SIZE_HEIGHT + ov8810_ver_qtr_blk_lines_array[cam_mode_sel]);
	} else  {
		return (OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES);
	}
}

static uint16_t ov8810_get_prev_pixels_pl(void)
{
	if (ov8810_ctrl->prev_res == QTR_SIZE) {
		return (OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS);
	} else  {
		return (OV8810_FULL_SIZE_WIDTH + OV8810_HRZ_FULL_BLK_PIXELS);
}
}

static uint16_t ov8810_get_pict_lines_pf(void)
{
	if (ov8810_ctrl->pict_res == QTR_SIZE) {
		return (OV8810_QTR_SIZE_HEIGHT + ov8810_ver_qtr_blk_lines_array[cam_mode_sel]);
	} else  {
		return (OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES);
	}
}

static uint16_t ov8810_get_pict_pixels_pl(void)
{
	if (ov8810_ctrl->pict_res == QTR_SIZE) {
		return (OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS);
	} else  {
		return (OV8810_FULL_SIZE_WIDTH + OV8810_HRZ_FULL_BLK_PIXELS);
	}
}

static uint32_t ov8810_get_pict_max_exp_lc(void)
{
	if (ov8810_ctrl->pict_res == QTR_SIZE) {
		return (OV8810_QTR_SIZE_HEIGHT + ov8810_ver_qtr_blk_lines_array[cam_mode_sel]);
	} else  {
		return (OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES);
	}
}

static int32_t ov8810_set_fps(struct fps_cfg *fps)
{
	int32_t rc = 0;
	ov8810_ctrl->fps_divider = fps->fps_div;
	ov8810_ctrl->pict_fps_divider = fps->pict_fps_div;
	ov8810_ctrl->fps = fps->f_mult;
	return rc;
}


static int32_t ov8810_write_exp_gain
			(uint16_t mul, uint16_t gain, uint32_t line)
{
	uint16_t aec_msb;
	uint16_t aec_lsb;
	int32_t rc = 0;
	uint32_t total_lines_per_frame;
	uint32_t total_pixels_per_line;
	/*uint32_t line_length_ratio = 1 * Q8;*/
	/**uint8_t ov8810_offset = 2; */
	uint32_t extra_line_length = 0;
	uint16_t extra_line_msb = 0;
	uint16_t extra_line_lsb = 0;
	uint32_t phy_line = 0;
	uint8_t phy_mul = MUL_GAIN_INIT_VALUE;
	uint16_t phy_gain = 0;
	uint32_t phy_extra_line_length = 0;
	const uint16_t postpone_frames = 4;
	uint16_t do_write = 1; /* assume do things */
	uint16_t ori_reg_mul_gain;
	uint8_t ori_reg_mul_gain_8bit;

	CDBG("%s start, mul = %d gain = %d line = %d\n", __func__,
		mul, gain, line);

	if (ov8810_ctrl->curr_res == QTR_SIZE) {
		total_lines_per_frame =
			(OV8810_QTR_SIZE_HEIGHT + ov8810_ver_qtr_blk_lines_array[cam_mode_sel]);
		total_pixels_per_line =
			OV8810_QTR_SIZE_WIDTH + OV8810_HRZ_QTR_BLK_PIXELS;
	} else {
		total_lines_per_frame =
			(OV8810_FULL_SIZE_HEIGHT + OV8810_VER_FULL_BLK_LINES);
		total_pixels_per_line =
			OV8810_FULL_SIZE_WIDTH + OV8810_HRZ_FULL_BLK_PIXELS;
	}

	if (line > total_lines_per_frame - 4) {
		extra_line_length =
			(uint32_t)(line - (total_lines_per_frame-4));
		line = total_lines_per_frame - 4;
	} else {
		extra_line_length = (uint16_t)0;
	}

	phy_line = line;
	phy_mul = mul;
	phy_gain = gain;
	phy_extra_line_length = extra_line_length;

	/* postpone writing gain only apply to preview */
	if (ov8810_ctrl->sensormode == SENSOR_PREVIEW_MODE) {

	/* need time to wait for aec stable (prevent black preview) */
	mdelay(6);

	CDBG("Stella: write_cnt=%d, pre_line = %d, line = %d," \
		"pre_mul = %d mul = %d," \
		"pre_gain = %d gain = %d," \
		"pre_extra_line_length =%d extra_line_length = %d\n",
		write_cnt,
		backup_line_gain[1].line, line,
		backup_line_gain[1].mul, mul,
		backup_line_gain[1].gain, gain,
		backup_line_gain[1].extra_line_length, extra_line_length);

	if (write_cnt == 0 && (
		backup_line_gain[1].line != line ||
		backup_line_gain[1].mul != mul ||
		backup_line_gain[1].gain != gain ||
		backup_line_gain[1].extra_line_length != extra_line_length)) {

		backup_line_gain[1].line = line;
		backup_line_gain[1].mul = mul;
		backup_line_gain[1].gain = gain;
		backup_line_gain[1].extra_line_length = extra_line_length;
		phy_line = backup_line_gain[1].line;
		phy_mul = backup_line_gain[0].mul;
		phy_gain = backup_line_gain[0].gain;
		phy_extra_line_length = backup_line_gain[0].extra_line_length;
		write_cnt++;
	} else if (write_cnt >= 1 && write_cnt < postpone_frames) {
		phy_line = backup_line_gain[1].line;
		phy_mul = backup_line_gain[1].mul;
		phy_gain = backup_line_gain[1].gain;
		phy_extra_line_length = backup_line_gain[1].extra_line_length;

		CDBG("updated_BLC = %d\n", updated_BLC);
		if (updated_BLC == 5) {
			/*50 BLC trigger by gain 40 BLC every frame */
			pr_info("[CAM]### BLC to 0x50 ###\n");
#if 0
			ov8810_i2c_write_b(ov8810_client->addr, 0x3071, 0x50);
#endif
		}
		if (updated_BLC <= 5)
			updated_BLC++;

		if (write_cnt > 1)
			do_write = 0;
		write_cnt++;
	} else {
		write_cnt = 0;
		do_write = 0;
	}

	if (do_write) {
		backup_line_gain[0].line = phy_line;
		backup_line_gain[0].mul = phy_mul;
		backup_line_gain[0].gain = phy_gain;
		backup_line_gain[0].extra_line_length = phy_extra_line_length;
	}

	}
#if 0
	pr_info("[CAM]Stella: backup_line_gain[0].line = %d\n",
		backup_line_gain[0].line);
	pr_info("[CAM]Stella: backup_line_gain[0].mul = %d\n",
		backup_line_gain[0].mul);
	pr_info("[CAM]Stella: backup_line_gain[0].gain = %d\n",
		backup_line_gain[0].gain);
	pr_info("[CAM]Stella: backup_line_gain[0].extra_line_length = %d\n",
		backup_line_gain[0].extra_line_length);
	pr_info("[CAM]Stella: backup_line_gain[1].line = %d\n",
		backup_line_gain[1].line);
	pr_info("[CAM]Stella: backup_line_gain[1].mul = %d\n",
		backup_line_gain[1].mul);
	pr_info("[CAM]Stella: backup_line_gain[1].gain = %d\n",
		backup_line_gain[1].gain);
	pr_info("[CAM]Stella: backup_line_gain[1].extra_line_length = %d\n",
		backup_line_gain[1].extra_line_length);

	pr_info("[CAM]Stella: phy_line=%d\n", phy_line);
	pr_info("[CAM]Stella: phy_gain=%d\n", phy_gain);
	pr_info("[CAM]Stella: phy_extra_line_length=%d\n", phy_extra_line_length);
#endif

	extra_line_msb = (uint16_t)(phy_extra_line_length & 0xFF00) >> 8;
	extra_line_lsb = (uint16_t)(phy_extra_line_length & 0x00FF);

	aec_msb = (uint16_t)(phy_line & 0xFF00) >> 8;
	aec_lsb = (uint16_t)(phy_line & 0x00FF);

	if (!do_write)
		return rc;

/*Move the read function out of group update to prevent hang Weiting0331*/
	rc = ov8810_i2c_read(OV8810_REG_MUL_GAIN,
		&ori_reg_mul_gain, 2);
	if (rc < 0) {
		pr_err("[CAM]read OV8810_REG_MUL_GAIN fail\n");
		return rc;
	}


	/* since we do STREAM ON here, don't do group update for snapshot */
	if (ov8810_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		/*for group update top*/
		/* weiting0414 prevent capture hang, enable 0x30b7[2] */
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30b7, 0x8c);
		if (rc < 0)
			return rc;
	}

	/* FIXME: prevent black preview by restoring 0x30bf -> 0x80 */
	rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30bf, 0x80);
	if (rc < 0)
		return rc;

	rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_AEC_MSB, (uint8_t)aec_msb);
	if (rc < 0)
		return rc;

	rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_AEC_LSB, (uint8_t)aec_lsb);
	if (rc < 0)
		return rc;

	ori_reg_mul_gain_8bit =
		(uint8_t)((ori_reg_mul_gain & 0xFF00) >> 8);
	CDBG("%s, read OV8810_REG_MUL_GAIN ori_reg_mul_gain = %x\n",
		__func__, ori_reg_mul_gain_8bit);
	ori_reg_mul_gain_8bit =
		(ori_reg_mul_gain_8bit & 0xFC) | (phy_mul & 0x03);
	CDBG("%s, read OV8810_REG_MUL_GAIN ori_reg_mul_gain = %x\n",
		__func__, ori_reg_mul_gain_8bit);
	rc = ov8810_i2c_write_b(ov8810_client->addr,
		OV8810_REG_MUL_GAIN, ori_reg_mul_gain_8bit);
	if (rc < 0)
		return rc;

	rc = ov8810_i2c_write_b(ov8810_client->addr,
		OV8810_GAIN, (uint8_t)phy_gain);
	if (rc < 0)
		return rc;

	rc = ov8810_i2c_write_b(ov8810_client->addr,
		REG_EXTRA_VSYNC_WIDTH_MSB, (uint8_t)extra_line_msb);
	if (rc < 0)
		return rc;

	rc = ov8810_i2c_write_b(ov8810_client->addr,
		REG_EXTRA_VSYNC_WIDTH_LSB, (uint8_t)extra_line_lsb);
	if (rc < 0)
		return rc;

	if (ov8810_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		/* for group update bottom */
		/* weiting0414 prevent capture hang , enable 0x30b7[2] */
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30b7, 0x84);
		if (rc < 0)
			return rc;

		/* for group update enable */
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30ff, 0xff);
		if (rc < 0)
			return rc;
		/* weiting0414 prevent capture hang ,
			retry I2C write to make sure enable */
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30ff, 0xff);
		if (rc < 0)
			return rc;
	}

	if (ov8810_ctrl->sensormode == SENSOR_RAW_SNAPSHOT_MODE) {
	    pr_info("[CAM]sleep 500 ms for safety raw snapshot");
		msleep(500);
    }

	/* STREAM ON for SNAPSHOT */
	if (ov8810_ctrl->sensormode == SENSOR_SNAPSHOT_MODE) {
		pr_info("[CAM]ov8810_ctrl: STREAM ON for SNAPSHOT\n");
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_REG_MODE_SELECT,
			OV8810_MODE_SELECT_STREAM);
		if (rc < 0)
			return rc;
		msleep(50);
	}

	/*stored_line_length_ratio = line_length_ratio;*/
	return rc;

} /* endof ov8810_write_exp_gain*/

/* ### this function is not called for userspace ### */
static int32_t ov8810_set_pict_exp_gain
			(uint16_t mul, uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = ov8810_write_exp_gain(mul, gain, line);
	return rc;
} /* endof ov8810_set_pict_exp_gain*/

/* remove test code */
#if 0
static int32_t ov8810_test(enum ov8810_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF) {
		return rc;
	}

	/* Activate  the Color bar test pattern */
	if (mo == TEST_1) {
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_COLOR_BAR_ENABLE_REG, 0xa0);
		if (rc < 0) {
			return rc;
		}
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			0x3085, 0x20);
		if (rc < 0) {
			return rc;
		}
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			0x306c, 0x00);
		if (rc < 0) {
			return rc;
		}
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_COLOR_BAR_PATTERN_SEL_REG, 0x02);
		if (rc < 0) {
			return rc;
		}
	}

	return rc;

}
#endif


uint32_t Crc32CheckSumByte(uint8_t *pData, uint32_t uiLen, uint32_t preValue)
{
	const uint32_t crc32table[256] = {
		/* 0x00 */ 0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
		/* 0x04 */ 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
		/* 0x08 */ 0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
		/* 0x0C */ 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
		/* 0x10 */ 0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
		/* 0x14 */ 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
		/* 0x18 */ 0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
		/* 0x1C */ 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
		/* 0x20 */ 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
		/* 0x24 */ 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
		/* 0x28 */ 0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
		/* 0x2C */ 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
		/* 0x30 */ 0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
		/* 0x34 */ 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
		/* 0x38 */ 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
		/* 0x3C */ 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
		/* 0x40 */ 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
		/* 0x44 */ 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
		/* 0x48 */ 0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
		/* 0x4C */ 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
		/* 0x50 */ 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
		/* 0x54 */ 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
		/* 0x58 */ 0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
		/* 0x5C */ 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
		/* 0x60 */ 0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
		/* 0x64 */ 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
		/* 0x68 */ 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
		/* 0x6C */ 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
		/* 0x70 */ 0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
		/* 0x74 */ 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
		/* 0x78 */ 0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
		/* 0x7C */ 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
		/* 0x80 */ 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
		/* 0x84 */ 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
		/* 0x88 */ 0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
		/* 0x8C */ 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
		/* 0x90 */ 0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
		/* 0x94 */ 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
		/* 0x98 */ 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
		/* 0x9C */ 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
		/* 0xA0 */ 0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
		/* 0xA4 */ 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
		/* 0xA8 */ 0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
		/* 0xAC */ 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
		/* 0xB0 */ 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
		/* 0xB4 */ 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
		/* 0xB8 */ 0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
		/* 0xBC */ 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
		/* 0xC0 */ 0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
		/* 0xC4 */ 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
		/* 0xC8 */ 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
		/* 0xCC */ 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
		/* 0xD0 */ 0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
		/* 0xD4 */ 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
		/* 0xD8 */ 0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
		/* 0xDC */ 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
		/* 0xE0 */ 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
		/* 0xE4 */ 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
		/* 0xE8 */ 0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
		/* 0xEC */ 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
		/* 0xF0 */ 0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
		/* 0xF4 */ 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
		/* 0xF8 */ 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
		/* 0xFC */ 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,};
	uint32_t i, CheckSum, cvalue;

	CheckSum = preValue;
	for (i = 0; i < uiLen; i++) {
		cvalue = *pData;
		CheckSum =
			(CheckSum>>8) ^
			crc32table[(CheckSum & 0xFF) ^
			(cvalue & 0xFF)];
		pData++;
	}
	return CheckSum;
}

static int32_t HTC_update_ov8810_lsc_registers(void)
{
	int i;
	struct awb_lsc_struct_type *awb_lsc_data_ptr;
	awb_lsc_data_ptr = (struct awb_lsc_struct_type *)get_cam_awb_cal();

	for (i = 0; i < 8; i++) {
		pr_info(KERN_INFO"[LSC calibration]  read AWB table 0x%x\n",
			awb_lsc_data_ptr->caBuff[i]);
	}

	for (i = 0; i < LSC_table_length; i++) {
		pr_info("[CAM][LSC calibration]  read LSC table 0x%x, 0x%x\n",
			awb_lsc_data_ptr->LSC_table[i].reg_addr,
			awb_lsc_data_ptr->LSC_table[i].reg_val);
	}

       if (awb_lsc_data_ptr->LSC_table_CRC ==
		Crc32CheckSumByte(
			(uint8_t *) awb_lsc_data_ptr->LSC_table,
			150 * sizeof(struct reg_addr_val_pair_struct), 0) &&
		awb_lsc_data_ptr->LSC_table_CRC != 0) {

		pr_info("[CAM][LSC calibration]checksum pass,use calibrated LSC\n");

		for (i = 0; i < LSC_table_length; i++) {
			ov8810_i2c_write_b(ov8810_client->addr,
				awb_lsc_data_ptr->LSC_table[i].reg_addr,
				awb_lsc_data_ptr->LSC_table[i].reg_val);
		}
		/*enable lsc on sensor*/
		ov8810_i2c_write_b(ov8810_client->addr, 0x3300, 0xff);
		/*move to the last*/
		ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_REG_MODE_SELECT, OV8810_MODE_SELECT_STREAM);

	} else {/*use default LSC table*/
	      pr_info("[CAM][LSC calibration]checksum fail\n");
	      return false;
	}
	return true;
}

static int32_t initialize_ov8810_registers(void)
{
	int32_t i, array_length;
	int32_t rc = 0;

	struct msm_camera_sensor_info *sdata = ov8810_pdev->dev.platform_data;

	mdelay(5);
	ov8810_i2c_write_b(
		ov8810_client->addr,
		OV8810_REG_SOFTWARE_RESET,
		OV8810_SOFTWARE_RESET);
	mdelay(5);
	ov8810_i2c_write_b(
		ov8810_client->addr,
		OV8810_REG_MODE_SELECT,
		OV8810_MODE_SELECT_SW_STANDBY);
	mdelay(1);
	array_length = sizeof(ov8810_init_settings_array) /
		sizeof(ov8810_init_settings_array[0]);

	/* Configure sensor for Preview mode and Snapshot mode */
	for (i = 0; i < array_length; i++) {
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			ov8810_init_settings_array[i].reg_addr,
			ov8810_init_settings_array[i].reg_val);
		if (rc < 0)
			return rc;
	}

	/*use calibrated LSC table*/
  if (!sdata->sensor_lc_disable) { /* 0902 disable old LSC method */
	if (HTC_update_ov8810_lsc_registers()) {
		pr_info("[CAM][LSC calibration] use calibrated LSC table done!\n");
	} else {/*use default LSC table*/
		array_length =
			sizeof(lsc_table_array) / sizeof(lsc_table_array[0]);

		for (i = 0; i < array_length; i++) {
			rc = ov8810_i2c_write_b(ov8810_client->addr,
				lsc_table_array[i].reg_addr,
				lsc_table_array[i].reg_val);
		}
		pr_info("[CAM][LSC calibration] use default LSC table done\n");
	}
  } else {
	  /* add streaming on */
	  ov8810_i2c_write_b(ov8810_client->addr,
		  OV8810_REG_MODE_SELECT, OV8810_MODE_SELECT_STREAM);
  }
	return rc;
} /* end of initialize_ov8810_ov8m0vc_registers. */

static int32_t ov8810_setting(int rt)
{
	int32_t rc = 0;
	int32_t i, array_length;
	static int16_t did_snapshot;
	uint16_t ori_reg_mul_gain;
	uint8_t ori_reg_mul_gain_8bit;

	uint16_t i2c_ret = 0;

	write_cnt = 0;

	pr_info("[CAM]ov8810_setting rt = %d\n", rt);

	if (rt == FULL_SIZE) {

		ov8810_i2c_read(0x30b7, &i2c_ret, 1);
		pr_info("[CAM]0x30b7, i2c_ret = 0x%X\n", i2c_ret);
    /*Retry writing group update bottom to ensure capture settings can be updated Weiting0331*/
		while (i2c_ret != 0x84) {

		/* for group update bottom */
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30b7, 0x84);
		if (rc < 0)
			return rc;

		/* for group update enable */
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30ff, 0xff);
		if (rc < 0)
			return rc;

		msleep(50);
		ov8810_i2c_read(0x30b7, &i2c_ret, 1);
		pr_info("[CAM]retry 0x30b7, i2c_ret = 0x%X\n", i2c_ret);
		};
	}

	rc = ov8810_i2c_write_b(ov8810_client->addr,
		OV8810_REG_MODE_SELECT,
		OV8810_MODE_SELECT_SW_STANDBY);
	if (rc < 0) {
		return rc;
	}

	ov8810_i2c_read(OV8810_REG_MODE_SELECT, &i2c_ret, 1);
	pr_info("[CAM]OV8810_REG_MODE_SELECT, i2c_ret = 0x%X\n", i2c_ret);

	switch (rt) {

	case QTR_SIZE:

		array_length = sizeof(ov8810_qtr_settings_array) /
			sizeof(ov8810_qtr_settings_array[0]);

		/* Configure sensor for XGA preview mode */
		for (i = 0; i < array_length; i++) {
			rc = ov8810_i2c_write_b(ov8810_client->addr,
				ov8810_qtr_settings_array[i].reg_addr,
				ov8810_qtr_settings_array[i].reg_val);

			if (rc < 0) {
				return rc;
			}
		}

/* reconfigure the qtr height to adjust frame rate */
{
		uint16_t fl_line = 0;
		fl_line = OV8810_QTR_SIZE_HEIGHT +
			ov8810_ver_qtr_blk_lines_array[cam_mode_sel];
		pr_info("%s fl_line = %d\n", __func__, __LINE__);
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			REG_FRAME_LENGTH_LINES_MSB,
			(fl_line & 0xFF00) >> 8);
		if (rc < 0)
			return rc;
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			REG_FRAME_LENGTH_LINES_LSB,
			fl_line & 0x00FF);
		if (rc < 0)
			return rc;
#if 0
		if (cam_mode_sel > 0)  {
		pr_info("[CAM]andy write binning ctrl 0x00, cam_mode_sel %d\n", cam_mode_sel);
		    rc = ov8810_i2c_write_b(ov8810_client->addr, //weiting ori c0
			   REG_BINNING_CONTROL, 0x00);
		if (rc < 0)
			return rc;

		}
#endif
}


#if 1 /* this is supposed to prevent abnormal color when restart preview */

		if (!did_snapshot)
		{
			memset(&backup_line_gain, 0,
				sizeof(struct backup_line_gain_struct));
			backup_line_gain[0].line = 0x4c4;
			backup_line_gain[0].mul = MUL_GAIN_INIT_VALUE;
			backup_line_gain[0].gain = 8; /*0x30;*/
			backup_line_gain[0].extra_line_length = 0;
		}

		CDBG("backup_line_gain[0].line = %d" \
			"backup_line_gain[0].mul = %d" \
			"backup_line_gain[0].gain = %d" \
			"backup_line_gain[0].extra_line_length = %d",
			backup_line_gain[0].line,
			backup_line_gain[0].mul,
			backup_line_gain[0].gain,
			backup_line_gain[0].extra_line_length);

		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_AEC_MSB,
			(uint8_t)((backup_line_gain[0].line & 0xFF00) >> 8));
		if (rc < 0)
			return rc;

		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_AEC_LSB,
			(uint8_t)(backup_line_gain[0].line & 0x00FF));
		if (rc < 0)
			return rc;

		rc = ov8810_i2c_read(OV8810_REG_MUL_GAIN, &ori_reg_mul_gain, 2);
		if (rc < 0) {
			pr_err("[CAM]read OV8810_REG_MUL_GAIN fail\n");
			return rc;
		}
		ori_reg_mul_gain_8bit =
			(uint8_t)((ori_reg_mul_gain & 0xFF00)>>8);
		CDBG("%s, read OV8810_REG_MUL_GAIN ori_reg_mul_gain = %x\n",
			__func__, ori_reg_mul_gain_8bit);
		ori_reg_mul_gain_8bit =
			(ori_reg_mul_gain_8bit & 0xFC) |
			(backup_line_gain[0].mul & 0x03);
		CDBG("%s, read OV8810_REG_MUL_GAIN ori_reg_mul_gain = %x\n",
			__func__, ori_reg_mul_gain_8bit);
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_REG_MUL_GAIN, ori_reg_mul_gain_8bit);
		if (rc < 0)
			return rc;

		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_GAIN,
			(uint8_t)(backup_line_gain[0].gain & 0x00FF));
		if (rc < 0)
			return rc;

		rc = ov8810_i2c_write_b(ov8810_client->addr,
			REG_EXTRA_VSYNC_WIDTH_MSB,
			(uint8_t)((backup_line_gain[0].extra_line_length
			& 0xFF00) >> 8));
		if (rc < 0)
			return rc;

		rc = ov8810_i2c_write_b(ov8810_client->addr,
			REG_EXTRA_VSYNC_WIDTH_LSB,
			(uint8_t)(backup_line_gain[0].extra_line_length
			& 0x00FF));
		if (rc < 0)
			return rc;

#endif
		did_snapshot = 0;

		ov8810_ctrl->curr_res = QTR_SIZE;

		break;

	case FULL_SIZE:

		array_length = sizeof(ov8810_full_settings_array) /
			sizeof(ov8810_full_settings_array[0]);
		/* Configure sensor for QXGA capture mode */
		for (i = 0; i < array_length; i++) {
			rc = ov8810_i2c_write_b(ov8810_client->addr,
				ov8810_full_settings_array[i].reg_addr,
				ov8810_full_settings_array[i].reg_val);
			if (rc < 0)
				return rc;
		}
		did_snapshot = 1;
		ov8810_ctrl->curr_res = FULL_SIZE;
		break;

	default:
		rc = -EFAULT;
		return rc;
	}

	/*disablt LSC for calibration*/
	pr_info("[CAM][LSC calibration] global_mode=%d!!!!\n", global_mode);
	/*take raw picture for LSC calibration*/
	if (global_mode) {
		/*disable sensor LSC*/
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x3300, 0xef);
		/*mirror off*/
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x30f8, 0x00);
		/*mirror off*/
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x3316, 0x02);
		pr_info("[CAM][LSC calibration]turn off LSC!Mirror On\n");

		/*fix gain & linecount*/
		/*Gain=0x9,exp=008d*/
		/*so luma taget = 100 to mfg light source*/
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x3000, 0x9);
		/*AEC_MSB*/
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x3002, 0x00);
		/*AEC_LSB*/
		rc = ov8810_i2c_write_b(ov8810_client->addr, 0x3003, 0x8d);
		pr_info("[CAM][LSC calibration]fix gain & linecount\n");
		global_mode = 0;
	}

	if (ov8810_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		msleep(50);
		rc = ov8810_i2c_write_b(ov8810_client->addr,
			OV8810_REG_MODE_SELECT,
			OV8810_MODE_SELECT_STREAM);
		if (rc < 0)
			return rc;

		updated_BLC = 0;
	}

/* remove test code
	rc = ov8810_test(ov8810_ctrl->set_test);
	if (rc < 0)
		return rc;
*/

	return rc;
} /*endof  ov8810_setting*/

static int32_t ov8810_video_config(int mode)
{
	int32_t rc = 0;
	static int pre_sel = 0;
	int cur_sel = (cam_mode_sel > 1)?1:0;

	ov8810_ctrl->sensormode = mode;

	pr_info("[CAM]%s cam_mode_sel %d cur_sel %d \n", __func__, cam_mode_sel, cur_sel);

	preview_frame_count = 0;

	if (ov8810_ctrl->curr_res != ov8810_ctrl->prev_res
		|| pre_sel != cur_sel
		)  {
	       rc = ov8810_setting(ov8810_ctrl->prev_res);
	       if (rc < 0)
	               return rc;

	} else {
	       ov8810_ctrl->curr_res = ov8810_ctrl->prev_res;
	}

	pre_sel = cur_sel;

	ov8810_ctrl->sensormode = mode;

	return rc;

} /*end of ov354_video_config*/

static int32_t ov8810_snapshot_config(int mode)
{
	int32_t rc = 0;
	ov8810_ctrl->sensormode = mode;

	if (ov8810_ctrl->curr_res != ov8810_ctrl->pict_res) {
		rc = ov8810_setting(ov8810_ctrl->pict_res);
		if (rc < 0)
			return rc;
	} else {
		ov8810_ctrl->curr_res = ov8810_ctrl->pict_res;
	}
	ov8810_ctrl->sensormode = mode;

	return rc;

} /*end of ov8810_snapshot_config*/

static int32_t ov8810_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	ov8810_ctrl->sensormode = mode;
	if (ov8810_ctrl->curr_res != ov8810_ctrl->pict_res) {
		rc = ov8810_setting(ov8810_ctrl->pict_res);
		if (rc < 0)
			return rc;
	} else {
		ov8810_ctrl->curr_res = ov8810_ctrl->pict_res;
	} /* Update sensor resolution */

	ov8810_ctrl->sensormode = mode;

	return rc;

} /*end of ov8810_raw_snapshot_config*/

static int32_t ov8810_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *sinfo = ov8810_pdev->dev.platform_data;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = ov8810_video_config(mode);
		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_info("[CAM]KPI PA: start sensor snapshot config: %d\n", __LINE__);
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		rc = ov8810_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		/*global_mode = 1; //20100330 vincent lsc calibration*/
		pr_info("[CAM]KPI PA: start sensor snapshot config: %d\n", __LINE__);
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		rc = ov8810_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t ov8810_power_down(void)
{
	return 0;
}

static int ov8810_probe_read_id(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t  chipidh = 0; /*, chipidl;*/
	uint16_t  def_chipid = 0;
	msleep(20);
	pr_info("[CAM]%s, ov8810_probe_init_sensor 1\n", __func__);
	/* 3. Read sensor Model ID: */
	if (ov8810_i2c_read(OV8810_PIDH_REG, &chipidh, 2) < 0) {
		rc = -1;
		pr_err("[CAM]read sensor id fail\n");
	}

	pr_info("[CAM]ov8810 model_id + ver = 0x%x\n", chipidh);

	/* 4. Compare sensor ID to OV8810 ID: */
	def_chipid = (((OV8810_PID << 8) & 0xFF00) + (OV8810_VER & 0x00FF));
	pr_info("[CAM]%s, Expected id=0x%x\n", __func__, def_chipid);

	if (chipidh < def_chipid) {
		rc = -ENODEV;
		pr_err("[CAM]read sensor id incorrect\n");
	}

	pr_info("[CAM]%s, vreg_get vreg_af_actuator\n", __func__);
	vreg_af_actuator = vreg_get(0, "gp5");
	if (IS_ERR(vreg_af_actuator))
		return PTR_ERR(vreg_af_actuator);

#ifdef CONFIG_ARCH_QSD8X50
	data->camera_set_source(MAIN_SOURCE);
#endif
	pr_info("[CAM]ov8810_probe_init_sensor finishes\n");
	return rc;
}

static int ov8810_sensor_open_init(struct msm_camera_sensor_info *data)
{

	int i;
	int32_t  rc = 0;
	/*stella0122*/
	uint16_t ov8810_nl_region_boundary = 5; /*3;*/
	uint16_t ov8810_nl_region_code_per_step = 35; /*101;*/
	uint16_t ov8810_l_region_code_per_step = 20; /*18;*/
	int timeout;
	pr_info("[CAM]Calling ov8810_sensor_open_init\n");

	down(&ov8810_sem);

	if (data == NULL) {
		pr_info("[CAM]data is a NULL pointer\n");
		return -EINVAL;
	}
	/*check whether resume done*/
	timeout = wait_event_interruptible_timeout(
		ov8810_event.event_wait,
		ov8810_event.waked_up,
		30*HZ);
	pr_info("[CAM]wait event : %d timeout:%d\n", ov8810_event.waked_up, timeout);
	if (timeout == 0) {
		up(&ov8810_sem);
		return rc;
	}
	msm_camio_probe_on(ov8810_pdev);
	ov8810_ctrl = kzalloc(sizeof(struct ov8810_ctrl), GFP_KERNEL);
	if (!ov8810_ctrl) {
		pr_err("[CAM]ov8810_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov8810_ctrl->curr_lens_pos = -1;
	ov8810_ctrl->fps_divider = 1 * 0x00000400;
	ov8810_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov8810_ctrl->set_test = TEST_OFF;
	ov8810_ctrl->prev_res = QTR_SIZE;
	ov8810_ctrl->pict_res = FULL_SIZE;
	ov8810_ctrl->curr_res = INVALID_SIZE;
	if (data)
		ov8810_ctrl->sensordata = data;

	/*switch pclk and mclk between main cam and 2nd cam*/
	/*only for supersonic*/
	pr_info("[CAM]doing clk switch (ov8810)\n");
	if(data->camera_clk_switch != NULL)
		data->camera_clk_switch();
	
	/* enable mclk first */
	msm_camio_clk_rate_set(OV8810_DEFAULT_CLOCK_RATE);
	msleep(20);

	msm_camio_camif_pad_reg_reset();
	msleep(20);

	/*PWD and RST config*/
	pr_info("[CAM]%s, GPIO(%d) sensor_pwd 0\n", __func__, data->sensor_pwd);
	rc = gpio_request(data->sensor_pwd, "ov8810");
	if (!rc)
		gpio_direction_output(data->sensor_pwd, 0);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", data->sensor_pwd);
	gpio_free(data->sensor_pwd);
	msleep(5);

	rc = gpio_request(data->sensor_reset, "ov8810");
	if (!rc)
		gpio_direction_output(data->sensor_reset, 1);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", data->sensor_reset);
	gpio_free(data->sensor_reset);
	msleep(1);

	/*read sensor id*/
	rc = ov8810_probe_read_id(data);

	ov8810_ctrl->sensormode = SENSOR_PREVIEW_MODE ;

	pr_info("[CAM]%s, initialize_ov8810_registers: %d\n", __func__, __LINE__);
	if (rc < 0)
		goto init_fail;
#ifdef CONFIG_ARCH_QSD8X50
      /* Initialize Sensor registers */
	rc = initialize_ov8810_registers();
	if (rc < 0)
		return rc;
#endif

	pr_info("[CAM]%s, enable AF actuator %d\n", __func__, __LINE__);

	/* enable AF actuator */
	rc = vreg_enable(vreg_af_actuator);
	if (!rc) {
		
		rc = vreg_set_level(vreg_af_actuator, 2800); /*2v8*/
		if (rc)
		{
		pr_err("[CAM]vreg_af_actuator vreg_set_level 2v8 failed!\n");
		goto init_fail;
		}
		}
	else {
		pr_err("[CAM]vreg_af_actuator vreg_enable failed!\n");
		goto init_fail;	
	}

	msleep(20);

	pr_info("[CAM]%s, set step_position_table %d\n", __func__, __LINE__);

	ov8810_ctrl->fps = 30*Q8;

	step_position_table[0] = 0;

	for (i = 1; i <= OV8810_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= ov8810_nl_region_boundary) {
			ov8810_step_position_table[i] =
				ov8810_step_position_table[i-1] +
				ov8810_nl_region_code_per_step;
		} else {
			ov8810_step_position_table[i] =
				ov8810_step_position_table[i-1] +
				ov8810_l_region_code_per_step;
		}
	}

	 /* generate test pattern */
	pr_info("[CAM]%s, generate test pattern, %d, rc=%d\n",
		__func__, __LINE__, rc);

	if (rc >= 0)
		goto init_done;
	    /* reset the driver state */
init_fail:
	pr_err("[CAM]%s: init_fail\n", __func__);
	vreg_disable(vreg_af_actuator);
	if (ov8810_ctrl) {
		kfree(ov8810_ctrl);
		ov8810_ctrl = NULL;
	}
init_done:
	up(&ov8810_sem);
	pr_info("[CAM]%s: init_done\n", __func__);
	return rc;

} /*endof ov8810_sensor_open_init*/

static int ov8810_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov8810_wait_queue);
	return 0;
}

static const struct i2c_device_id ov8810_i2c_id[] = {
	{ "ov8810", 0},
	{ }
};

static int ov8810_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_info("[CAM]ov8810_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[CAM]i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov8810_sensorw = kzalloc(sizeof(struct ov8810_work), GFP_KERNEL);
	if (!ov8810_sensorw) {
		pr_err("[CAM]kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov8810_sensorw);
	ov8810_init_client(client);
	ov8810_client = client;

	msleep(50);

	pr_info("[CAM]ov8810_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("[CAM]ov8810_probe failed! rc = %d\n", rc);
	return rc;
}

static int ov8810_probe_init_done(const struct msm_camera_sensor_info *data)
{
	int rc;
	rc = gpio_request(data->sensor_pwd, "ov8810");
	if (!rc)
		gpio_direction_output(data->sensor_pwd, 1);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", data->sensor_pwd);
	gpio_free(data->sensor_pwd);
	mdelay(1);
#ifdef CONFIG_ARCH_QSD8X50
	rc = gpio_request(data->sensor_reset, "ov8810");
	if (!rc)
		gpio_direction_output(data->sensor_reset, 0);
	else
		pr_err("GPIO (%d) request faile\n", data->sensor_reset);
	gpio_free(data->sensor_reset);
#endif
	return 0;
}

static int ov8810_suspend(struct platform_device *pdev, pm_message_t state)
{
	int rc;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;

	if (!sinfo->need_suspend)
		return 0;
	ov8810_event.waked_up = 0;

	pr_info("[CAM]ov8810: camera suspend\n");

	pr_info("[CAM]%s, vreg_af_actuator vreg_disable\n", __func__);
	vreg_disable(vreg_af_actuator);

	rc = gpio_request(sinfo->sensor_reset, "ov8810");
	if (!rc)
		gpio_direction_output(sinfo->sensor_reset, 0);
	else
		pr_info("[CAM]ov8810: request GPIO(sensor_reset) :%d faile\n",
			sinfo->sensor_reset);

	gpio_free(sinfo->sensor_reset);
	msleep(10);
		rc = gpio_request(sinfo->sensor_pwd, "ov8810");
	if (!rc)
		gpio_direction_output(sinfo->sensor_pwd, 0);
	else
		pr_info("[CAM]ov8810: request GPIO(sensor_reset) :%d faile\n",
			sinfo->sensor_pwd);

	gpio_free(sinfo->sensor_pwd);

	pr_info("[CAM]ov8810:suspend done\n");
	return rc;
}

static void ov8810_resume(struct early_suspend *handler)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = ov8810_pdev->dev.platform_data;
	pr_info("[CAM]ov8810_resume\n");

	/*check whether need resume*/
	if (!sinfo->need_suspend)
		return;

	/*check whether already suspend*/
	if (ov8810_event.waked_up == 1) {
		pr_info("[CAM]Ov8810: No nesesary to do Resume\n");
		return;
	}

	mdelay(5);
	/*power down setup*/
	pr_info("[CAM]%s, sensor_pwd 0\n", __func__);
	rc = gpio_request(sinfo->sensor_pwd, "ov8810");
	if (!rc)
		gpio_direction_output(sinfo->sensor_pwd, 0);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", sinfo->sensor_pwd);
	gpio_free(sinfo->sensor_pwd);
	mdelay(5);
	/*reset setup */
	rc = gpio_request(sinfo->sensor_reset, "ov8810");
	if (!rc)
		gpio_direction_output(sinfo->sensor_reset, 1);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", sinfo->sensor_reset);
	gpio_free(sinfo->sensor_reset);

	/*init msm,clk ,GPIO,enable*/
	pr_info("[CAM]%s, msm_camio_probe_on\n", __func__);
	msm_camio_probe_on(ov8810_pdev);
	msm_camio_clk_enable(CAMIO_MDC_CLK);

	/*set MCLK*/
	pr_info("[CAM]%s, msm_camio_clk_rate_set = %d\n",
		__func__, OV8810_DEFAULT_CLOCK_RATE);
	msm_camio_clk_rate_set(OV8810_DEFAULT_CLOCK_RATE);
	msleep(100);

	/*read sensor id*/
	rc = ov8810_probe_read_id(sinfo);
	if (rc < 0)
		pr_err("[CAM]OV8810 resume faile :can not read sensor ID\n");

	/* Initialize Sensor registers */
	rc = initialize_ov8810_registers();
	if (rc < 0)
		return;
	msleep(20);
	/*resume done*/
	ov8810_probe_init_done(sinfo);
	/*turn off MCLK*/
	msm_camio_probe_off(ov8810_pdev);
	msm_camio_clk_disable(CAMIO_MDC_CLK);

	ov8810_event.waked_up = 1;
	pr_info("[CAM]ov8810:resume done\n");
	wake_up(&ov8810_event.event_wait);
	return;
}


static int __exit ov8810_i2c_remove(struct i2c_client *client)
{
	struct ov8810_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	deinit_suspend();
	ov8810_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov8810_i2c_driver = {
	.id_table = ov8810_i2c_id,
	.probe	= ov8810_i2c_probe,
	.remove = __exit_p(ov8810_i2c_remove),
	.driver = {
		.name = "ov8810",
	},
};


static struct early_suspend early_suspend_ov8810 = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN+1,
	.resume = ov8810_resume,
	.suspend = NULL,
};

static const char *Ov8810Vendor = "OmniVision";
static const char *Ov8810NAME = "ov8810";
static const char *Ov8810Size = "8M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", Ov8810Vendor, Ov8810NAME, Ov8810Size);
	ret = strlen(buf) + 1;

	return ret;
}

DEFINE_MUTEX(cam_mode_lock);

static ssize_t sensor_read_cam_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	mutex_lock(&cam_mode_lock);
	length = sprintf(buf, "%d\n", cam_mode_sel);
	mutex_unlock(&cam_mode_lock);
	return length;
}

static ssize_t sensor_set_cam_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;
	mutex_lock(&cam_mode_lock);
	tmp = buf[0] - 0x30; /* only get the first char */
	cam_mode_sel = tmp;
	mutex_unlock(&cam_mode_lock);
	return count;
}

static ssize_t sensor_read_node(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", sensor_probe_node);
	return length;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(cam_mode, 0644, sensor_read_cam_mode, sensor_set_cam_mode);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);

static struct kobject *android_ov8810;

static int ov8810_sysfs_init(void)
{
	int ret = 0;
	pr_info("[CAM]ov8810:kobject creat and add\n");
	android_ov8810 = kobject_create_and_add("android_camera", NULL);
	if (android_ov8810 == NULL) {
		pr_info("[CAM]ov8810_sysfs_init: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM]Ov8810:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov8810, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]ov8810_sysfs_init: sysfs_create_file failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(android_ov8810, &dev_attr_cam_mode.attr);
	if (ret) {
		pr_info("[CAM]ov8810_sysfs_init: dev_attr_cam_mode failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(android_ov8810, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM]ov8810_sysfs_init: dev_attr_node failed\n");
		ret = -EFAULT;
		goto error;
	}

	return ret;

error:
	kobject_del(android_ov8810);
	return ret;
}

#ifdef CONFIG_ARCH_MSM7X30
uint8_t ov8810_preview_skip_frame(void)
{
	if (ov8810_ctrl->sensormode == SENSOR_PREVIEW_MODE && preview_frame_count < 2) {
		preview_frame_count++;
		return 1;
	}
	return 0;
}
#endif

int ov8810_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	down(&ov8810_sem);

	CDBG("ov8810_sensor_config: cfgtype = %d\n",
	  cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
				ov8810_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			ov8810_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				ov8810_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				ov8810_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				ov8810_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				ov8810_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = ov8810_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				ov8810_write_exp_gain(
					cdata.cfg.exp_gain.mul,
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				ov8810_set_pict_exp_gain(
					cdata.cfg.exp_gain.mul,
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = ov8810_set_sensor_mode(cdata.mode,
						cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = ov8810_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				ov8810_move_focus(
					cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				ov8810_set_default_focus(
					(uint8_t)cdata.cfg.focus.steps);
			break;

		case CFG_SET_EFFECT:
			rc = ov8810_set_default_focus(
					(uint8_t)cdata.cfg.effect);
			break;

		case CFG_I2C_IOCTL_R_OTP:{
			rc = ov8810_i2c_read_fuseid(&cdata);
			if (copy_to_user
				(argp, &cdata, sizeof(struct sensor_cfg_data))
			    )
			rc = -EFAULT;
			}
			break;

		case CFG_SET_OV_LSC:
			rc = ov8810_update_lsc_table(&cdata);
			break;

		/*20100330 vincent for lsc calibration*/
		case CFG_SET_OV_LSC_RAW_CAPTURE:
			rc = ov8810_LSC_calibration_set_rawflag(&cdata);
			break;

		default:
			rc = -EFAULT;
			break;
		}

	prevent_suspend();
	up(&ov8810_sem);

	return rc;
}




static int ov8810_sensor_release(void)
{
	int rc = -EBADF;

	down(&ov8810_sem);
	msleep(35);

	if (ov8810_ctrl) {
		rc = gpio_request(ov8810_ctrl->sensordata->sensor_pwd, "ov8810");
		if (!rc)
			gpio_direction_output(ov8810_ctrl->sensordata->sensor_pwd, 1);
		else
			pr_err("[CAM]GPIO (%d) request faile\n", ov8810_ctrl->sensordata->sensor_pwd);
		gpio_free(ov8810_ctrl->sensordata->sensor_pwd);
#ifdef CONFIG_ARCH_QSD8X50
	/*Pull low RST*/
	gpio_request(ov8810_ctrl->sensordata->sensor_reset, "ov8810");
	gpio_direction_output(ov8810_ctrl->sensordata->sensor_reset, 0);
	gpio_free(ov8810_ctrl->sensordata->sensor_reset);
#endif
	}

	pr_info("[CAM]vreg_af_actuator vreg_disable\n");
	vreg_disable(vreg_af_actuator);

	msleep(20);

	pr_info("[CAM]%s, %d\n", __func__, __LINE__);

	msm_camio_probe_off(ov8810_pdev);
	if (ov8810_ctrl) {
		kfree(ov8810_ctrl);
		ov8810_ctrl = NULL;
	}
	mdelay(3);
	allow_suspend();
	pr_info("[CAM]ov8810_release completed\n");
	up(&ov8810_sem);

	return rc;
}

static int ov8810_sensor_probe(struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&ov8810_i2c_driver);
	if (rc < 0 || ov8810_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}

	pr_info("[CAM]ov8810 s->node %d\n", s->node);
	sensor_probe_node = s->node;
	/*switch pclk and mclk between main cam and 2nd cam*/
	/*only for supersonic*/
	pr_info("[CAM]Ov8810: doing clk switch (ov8810)\n");
	if(info->camera_clk_switch != NULL)
		info->camera_clk_switch();
	mdelay(5);
	/*power down setup*/
	rc = gpio_request(info->sensor_pwd, "ov8810");
	if (!rc)
		gpio_direction_output(info->sensor_pwd, 0);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", info->sensor_pwd);
	gpio_free(info->sensor_pwd);
	mdelay(5);
	/*reset setup */
	rc = gpio_request(info->sensor_reset, "ov8810");
	if (!rc)
		gpio_direction_output(info->sensor_reset, 1);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", info->sensor_reset);
	gpio_free(info->sensor_reset);

	/*set MCLK*/
	pr_info("[CAM]%s, msm_camio_clk_rate_set %d\n",
		__func__, OV8810_DEFAULT_CLOCK_RATE);
       msm_camio_clk_rate_set(OV8810_DEFAULT_CLOCK_RATE);
	msleep(100);
	/*read sensor id*/
	rc = ov8810_probe_read_id(info);
	if (rc < 0)
		goto probe_fail;

	/* Initialize Sensor registers */
	rc = initialize_ov8810_registers();
	if (rc < 0)
		return rc;

	if (info->camera_main_set_probe != NULL)
		info->camera_main_set_probe(true);

	init_suspend();
	s->s_init = ov8810_sensor_open_init;
	s->s_release = ov8810_sensor_release;
	s->s_config  = ov8810_sensor_config;

#ifdef CONFIG_ARCH_MSM7X30
	info->preview_skip_frame = ov8810_preview_skip_frame;
#endif

	msleep(20);
	ov8810_probe_init_done(info);
	/*register late resuem*/
	register_early_suspend(&early_suspend_ov8810);
	/*init wait event*/
	init_waitqueue_head(&ov8810_event.event_wait);
	/*init waked_up value*/
	ov8810_event.waked_up = 1;
	/*write sysfs*/
	ov8810_sysfs_init();
	pr_info("[CAM]%s: ov8810_probe_init_done %d\n",  __func__, __LINE__);
	goto probe_done;

probe_fail:
	pr_err("[CAM]SENSOR PROBE FAILS!\n");
probe_done:
	return rc;

}

#ifndef CONFIG_ARCH_QSD8X50
static int ov8810_vreg_enable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	int rc;
	pr_info("[CAM]%s camera vreg on\n", __func__);

	if (sdata->camera_power_on == NULL) {
		pr_err("[CAM]sensor platform_data didnt register\n");
		return -EIO;
	}
	rc = sdata->camera_power_on();
	return rc;
}
#endif

#if 0
static int ov8810_vreg_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	int rc;
	printk(KERN_INFO "%s camera vreg off\n", __func__);
	if (sdata->camera_power_off == NULL) {
		pr_err("[CAM]sensor platform_data didnt register\n");
		return -EIO;
	}
	rc = sdata->camera_power_off();
	return rc;
}
#endif

static int __ov8810_probe(struct platform_device *pdev)
{
	
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	printk("[CAM]__ov8810_probe\n");
	ov8810_pdev = pdev;

	if (sdata->camera_main_get_probe != NULL) {
		if (sdata->camera_main_get_probe()) {
			pr_info("[CAM]__ov8810_probe camera main get probed already.\n");
			return 0;
		}
	}

#ifndef CONFIG_ARCH_QSD8X50
    {
    int rc;
	rc = gpio_request(sdata->sensor_pwd, "ov8810");
	if (!rc)
		gpio_direction_output(sdata->sensor_pwd, 1);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", sdata->sensor_pwd);
	gpio_free(sdata->sensor_pwd);
	udelay(200);

	rc = ov8810_vreg_enable(pdev);
	if (rc < 0)
		pr_err("[CAM]__ov8810_probe fail sensor power on error\n");
	}
#endif

	return msm_camera_drv_start(pdev, ov8810_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov8810_probe,
	.driver = {
		.name = "msm_camera_ov8810",
		.owner = THIS_MODULE,
	},
	.suspend = ov8810_suspend,
};

static int __init ov8810_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov8810_init);
