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
#ifdef CONFIG_MSM_CAMERA_8X60
#include <mach/camera-8x60.h>
#elif defined(CONFIG_MSM_CAMERA_7X30)
#include <mach/camera-7x30.h>
#else
#include <mach/camera.h>
#endif
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include "ov8830.h"


/* CAMIF output resolutions */
/* 816x612, 24MHz MCLK 96MHz PCLK */
#define OV8830_QTR_SIZE_DUMMY_PIXELS	0
#define OV8830_QTR_SIZE_DUMMY_LINES	0
#define OV8830_QTR_SIZE_WIDTH		1640
#define OV8830_QTR_SIZE_HEIGHT		1232

#define OV8830_FULL_SIZE_DUMMY_PIXELS	0
#define OV8830_FULL_SIZE_DUMMY_LINES	0
#define OV8830_FULL_SIZE_WIDTH		3280
#define OV8830_FULL_SIZE_HEIGHT		2464

#define OV8830_VIDE_SIZE_DUMMY_PIXELS	0
#define OV8830_VIDE_SIZE_DUMMY_LINES	0
#if defined(CONFIG_MACH_RUNNYMEDE)
#define OV8830_VIDEO_SIZE_WIDTH		1640
#define OV8830_VIDEO_SIZE_HEIGHT	1232
#define OV8830_VIDEO_SIZE_WIDTH_FAST   1632 /* 1632 */ /* 1640 */ /* 656 */
#define OV8830_VIDEO_SIZE_HEIGHT_FAST   768 /* 768 */ /* 1232 */ /* 496 */
#else
#define OV8830_VIDEO_SIZE_WIDTH		3088
#define OV8830_VIDEO_SIZE_HEIGHT	1736
#define OV8830_VIDEO_SIZE_WIDTH_FAST   1640
#define OV8830_VIDEO_SIZE_HEIGHT_FAST   916
#endif

#define OV8830_HRZ_QTR_BLK_PIXELS	1968
#define OV8830_VER_QTR_BLK_LINES	36
#define OV8830_HRZ_FULL_BLK_PIXELS	328  /*stella 1203*/
#define OV8830_VER_FULL_BLK_LINES	36
#if defined(CONFIG_MACH_RUNNYMEDE)
#define OV8830_HRZ_VIDEO_BLK_PIXELS	1968
#define OV8830_VER_VIDEO_BLK_LINES	36
#define OV8830_HRZ_VIDEO_BLK_PIXELS_FAST  1976 /* 1976 */ /* 1968 */ /* 2952 */
#define OV8830_VER_VIDEO_BLK_LINES_FAST    116 /* 116 */ /* 96 */ /* 168 */
#else
#define OV8830_HRZ_VIDEO_BLK_PIXELS	520
#define OV8830_VER_VIDEO_BLK_LINES	149
#define OV8830_HRZ_VIDEO_BLK_PIXELS_FAST  1968
#define OV8830_VER_VIDEO_BLK_LINES_FAST		82
#endif

#define OV8830_MIN_COARSE_INTEGRATION_TIME 1
#define OV8830_OFFSET				12

static int cam_mode_sel = 0; /* 0: photo, 1: video@30fps, 2: video@24fps */
/* 240: 26, 365: 24, 589: 21 */
const int ov8830_ver_qtr_blk_lines_array[] = {44, 44, 365};

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8				0x00000100

/* Omnivision8830 product ID register address */
#define OV8830_PIDH_REG			0x300A
#define OV8830_PIDL_REG			0x300B

/* Omnivision8830 product ID */
#define OV8830_PID			0x88
/* Omnivision8830 version */
#define OV8830_VER			0x30

/* Time in milisecs for waiting for the sensor to reset */
#define OV8830_RESET_DELAY_MSECS	66

#define OV8830_DEFAULT_CLOCK_RATE	24000000

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

#define REG_VSYNC_WIDTH_MSB	0x380E  /*px301E*/
#define REG_VSYNC_WIDTH_LSB	0x380F  /*0x301F*/

#define REG_X_ADDR_START_HIGH		0x3024
#define REG_X_ADDR_START_LOW		0x3025
#define REG_Y_ADDR_START_HIGH		0x3026
#define REG_Y_ADDR_START_LOW		0x3027
#define REG_X_ADDR_END_HIGH		0x3028
#define REG_X_ADDR_END_LOW		0x3029
#define REG_Y_ADDR_END_HIGH		0x302A
#define REG_Y_ADDR_END_LOW		0x302B

/* Gain setting register */
#define OV8830_GAIN			0x3000

#define OV8830_GAIN_MSB			0x350A
#define OV8830_GAIN_LSB			0x350B

#define OV8830_AEC_MSB_24	        0x3500 /*easter for frame rate=10fps 20110526*/
#define OV8830_AEC_MSB			0x3501
#define OV8830_AEC_LSB			0x3502

/* additional gain function provided by OV8830,
 * original gain can changed to 1x, 2x or 4x
 * to increase the gain that OV8830 can provide */
#define OV8830_REG_MUL_GAIN		0x3006
#define MUL_GAIN_INIT_VALUE		0x00

#define OV8830_MAX_EXPOSURE_GAIN	0x1FF

/* Mode select register */
#define OV8830_REG_MODE_SELECT		0x30FA	/* image system */
#define OV8830_MODE_SELECT_STREAM	0x01	/* start streaming */
#define OV8830_MODE_SELECT_SW_STANDBY	0x00	/* software standby */
#define OV8830_REG_SOFTWARE_RESET	0x3012	/* 0x0103 */
#define OV8830_SOFTWARE_RESET		0x80	/* 0x01 */

#define OV8830_REG_MODE_FLIP		0x3820
#define OV8830_REG_MODE_MIRROR		0x3821

#define OV8830_REG_FLIP		0x52
#define OV8830_REG_MIRROR		0x08

/* AF Total steps parameters */
#define OV8830_AF_MSB			0x30EC
#define OV8830_AF_LSB			0x30ED

#define OV8830_AF_I2C_ADDR 0x18
#define OV8830_VCM_CODE_MSB 0x04
#define OV8830_VCM_CODE_LSB 0x05
#define OV8830_SW_DAMPING_STEP 10
#define OV8830_MAX_FPS 30

#define OV8830_STEPS_NEAR_TO_CLOSEST_INF	42 /*43 stella0122 */
#define OV8830_TOTAL_STEPS_NEAR_TO_FAR		42 /*43 stella0122 */

/*Test pattern*/
/* Color bar pattern selection */
#define OV8830_COLOR_BAR_PATTERN_SEL_REG	0x307B

/* Color bar enabling control */
#define OV8830_COLOR_BAR_ENABLE_REG		0x307D

/* I2C Address of the Sensor */
#define OV8830_I2C_SLAVE_ID		0x6C

/*LSC table length*/
#define LSC_table_length 144

/*============================================================================
TYPE DECLARATIONS
============================================================================*/

struct awb_lsc_struct_type {
       unsigned int caBuff[8];  /*awb_calibartion*/
	struct reg_addr_val_pair_struct LSC_table[150];  /*lsc_calibration*/
	uint32_t LSC_table_CRC;
};

enum ov8830_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum ov8830_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	QVGA_SIZE,
	VIDEO_SIZE,
	FAST_VIDEO_SIZE,
	INVALID_SIZE
};

/*LSC calibration*/
int global_mode;
/*TODO: should be use a header file to reference this function*/
extern unsigned char *get_cam_awb_cal(void);

static int ov8830_common_deinit(void);

static int sensor_probe_node = 0;
static int preview_frame_count = 0;

static struct wake_lock ov8830_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&ov8830_wake_lock, WAKE_LOCK_IDLE, "ov8830");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&ov8830_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&ov8830_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&ov8830_wake_lock);
}

/*============================================================================
DATA DECLARATIONS
============================================================================*/

/* AF Tuning Parameters */

static uint16_t ov8830_step_position_table[OV8830_TOTAL_STEPS_NEAR_TO_FAR+1];

/*static uint32_t stored_line_length_ratio = 1 * Q8;*/

static uint16_t write_cnt;
/*static uint16_t updated_BLC;*/ /* only set to 0x50 after 1st update again*/

uint8_t S3_to_0 = 0x1; /* 0x9 */

/* static Variables*/
static uint16_t step_position_table[OV8830_TOTAL_STEPS_NEAR_TO_FAR+1];


/* FIXME: Changes from here */
struct ov8830_work {
	struct work_struct work;
};

static struct  ov8830_work *ov8830_sensorw;
static struct  i2c_client *ov8830_client;
static uint16_t ov8830_pos_tbl[OV8830_TOTAL_STEPS_NEAR_TO_FAR + 1];

static struct vreg *vreg_af_actuator;

enum ov8830_reg_update_t{
	REG_INIT,
	REG_PERIODIC
};


struct ov8830_ctrl {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider; 		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; 	/* init to 1 * 0x00000400 */
	uint16_t fps;

	int16_t  curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint16_t my_reg_dig_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum ov8830_resolution_t prev_res;
	enum ov8830_resolution_t pict_res;
	enum ov8830_resolution_t curr_res;
	enum ov8830_test_mode_t  set_test;
  enum ov8830_reg_update_t reg_update;

	unsigned short imgaddr;
};


static struct ov8830_ctrl *ov8830_ctrl;
static struct platform_device *ov8830_pdev;

struct ov8830_waitevent{
	uint32_t waked_up;
	wait_queue_head_t event_wait;
};
static struct ov8830_waitevent ov8830_event;

static DECLARE_WAIT_QUEUE_HEAD(ov8830_wait_queue);
DEFINE_SEMAPHORE(ov8830_sem);


/*=============================================================*/

static int ov8830_i2c_rxdata(unsigned short saddr,
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
	CDBG("[CAM] %s: saddr=0x%X\n", __func__, saddr);
	CDBG("[CAM] %s: raddr=0x%X\n", __func__, *rxdata);

	if (i2c_transfer(ov8830_client->adapter, msgs, 2) < 0) {
		pr_err("[CAM]ov8830_i2c_rxdata failed!\n");
		return -EIO;
	}
	CDBG("[CAM] %s: rxdata=0x%X\n", __func__, *rxdata);

	return 0;
}
static int32_t ov8830_i2c_txdata(unsigned short saddr,
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
	if (i2c_transfer(ov8830_client->adapter, msg, 1) < 0) {
		pr_err("[CAM]ov8830_i2c_txdata faild 0x%x\n", ov8830_client->addr);
		return -EIO;
	}

	return 0;
}


static int32_t ov8830_i2c_read(unsigned short raddr,
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
	rc = ov8830_i2c_rxdata(ov8830_client->addr, buf, rlen);

	if (rc < 0) {
		pr_err("[CAM]ov8830_i2c_read 0x%x failed!\n", raddr);
		pr_err("[CAM] starting read retry policy count:%d\n", count);
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


static int32_t ov8830_i2c_write_b(unsigned short saddr,
				unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	int count = 0;
	CDBG("[CAM] i2c_write_w_b, addr = 0x%x, val = 0x%x!\n", waddr, bdata);

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

retry:
	CDBG("[CAM] i2c_write_b addr = %d, val = %d\n", waddr, bdata);
	rc = ov8830_i2c_txdata(saddr, buf, 3);

	if (rc < 0) {
		pr_err("[CAM]i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			 waddr, bdata);
		pr_err(KERN_ERR "[CAM] starting read retry policy count:%d\n", count);
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
static int ov8830_update_lsc_table(struct sensor_cfg_data *cdata)
{
	int i = 0;
	pr_info("[CAM][LSC calibration]ov8830_update_lsc_table\n");
	for (i = 0; i < 144; i++) {
		ov8830_i2c_write_b(
			ov8830_client->addr,
			cdata->cfg.lsctable.lsc_table[i].reg_addr,
			cdata->cfg.lsctable.lsc_table[i].reg_val);
		pr_info("[CAM][LSC calibration]update_lsc_table: 0x%x, 0x%x\n",
				cdata->cfg.lsctable.lsc_table[i].reg_addr,
				cdata->cfg.lsctable.lsc_table[i].reg_val);
	}
	/*enable lsc on sensor*/
	ov8830_i2c_write_b(ov8830_client->addr, 0x3300, 0xff);
	/*mirror on*/
	ov8830_i2c_write_b(ov8830_client->addr, 0x30f8, 0x45);
	/*mirror on*/
	ov8830_i2c_write_b(ov8830_client->addr, 0x3316, 0x03);
	return 1;

}

/*20100330 vincent for LSC calibration*/
static int ov8830_LSC_calibration_set_rawflag(struct sensor_cfg_data *cdata)
{
	global_mode = 1;
	return 1;
}

#define MAX_FUSE_ID_INFO 17
unsigned short fuse_id[MAX_FUSE_ID_INFO] = {0};

static int ov8830_i2c_read_fuseid(struct sensor_cfg_data *cdata)
{
#if 0 /* move to ov8830_sensor_probe() */
	unsigned short fuse_id[MAX_FUSE_ID_INFO];
	int count = 0;

	ov8830_i2c_write_b(ov8830_client->addr, 0x3d84, 0xc0);
	ov8830_i2c_write_b(ov8830_client->addr, 0x3d81, 0x01);
	ov8830_i2c_read(0x3d00, &fuse_id[0], 1);
	ov8830_i2c_read(0x3d01, &fuse_id[1], 1);
	ov8830_i2c_read(0x3d02, &fuse_id[2], 1);
	ov8830_i2c_read(0x3d03, &fuse_id[3], 1);
	ov8830_i2c_read(0x3d04, &fuse_id[4], 1);
	ov8830_i2c_read(0x3d05, &fuse_id[5], 1);
	ov8830_i2c_read(0x3d06, &fuse_id[6], 1);
	ov8830_i2c_read(0x3d07, &fuse_id[7], 1);
	ov8830_i2c_read(0x3d08, &fuse_id[8], 1);
	ov8830_i2c_read(0x3d09, &fuse_id[9], 1);
	ov8830_i2c_read(0x3d0A, &fuse_id[10], 1);
	ov8830_i2c_read(0x3d0B, &fuse_id[11], 1);
	ov8830_i2c_read(0x3d0C, &fuse_id[12], 1);
	ov8830_i2c_read(0x3d0D, &fuse_id[13], 1);
	ov8830_i2c_read(0x3d0E, &fuse_id[14], 1);
	ov8830_i2c_read(0x3d0F, &fuse_id[15], 1);

	for (count = 0; count < MAX_FUSE_ID_INFO; count++)
		pr_info("[CAM]Ov8830 Get fuse: fuse_id[%d]: %x\n",
		count, fuse_id[count]);
#endif

	pr_info("[CAM]ov8830_i2c_read_fuseid\n");

	/* Wafer number */
	cdata->cfg.fuse.fuse_id_word1 =
		(uint32_t) (fuse_id[0]<<16) | (fuse_id[1]<<8) | (fuse_id[2]);
	/* X and Y coordinate */
	cdata->cfg.fuse.fuse_id_word2 = (uint32_t) (fuse_id[3]<<8) | (fuse_id[4]);
	cdata->cfg.fuse.fuse_id_word3 = 0;
	cdata->cfg.fuse.fuse_id_word4 = 0;

	pr_info("[CAM] read_fuseid: fuse_id_word = %x %x %x %x\n", cdata->cfg.fuse.fuse_id_word1,
		cdata->cfg.fuse.fuse_id_word2, cdata->cfg.fuse.fuse_id_word3, cdata->cfg.fuse.fuse_id_word4);
	return 0;
}

static void ov8830_setup_af_tbl(void)
{
  uint32_t i;
  uint16_t ov8830_nl_region_boundary1 = 3;
  uint16_t ov8830_nl_region_boundary2 = 5;
  uint16_t ov8830_nl_region_code_per_step1 = 40;
  uint16_t ov8830_nl_region_code_per_step2 = 20;
  uint16_t ov8830_l_region_code_per_step = 16;

  ov8830_pos_tbl[0] = 0;

  for (i = 1; i <= OV8830_TOTAL_STEPS_NEAR_TO_FAR; i++) {
    if (i <= ov8830_nl_region_boundary1)
      ov8830_pos_tbl[i] = ov8830_pos_tbl[i-1] +
      ov8830_nl_region_code_per_step1;
    else if (i <= ov8830_nl_region_boundary2)
      ov8830_pos_tbl[i] = ov8830_pos_tbl[i-1] +
      ov8830_nl_region_code_per_step2;
    else
      ov8830_pos_tbl[i] = ov8830_pos_tbl[i-1] +
      ov8830_l_region_code_per_step;
  }
}

static int32_t
ov8830_go_to_position(uint32_t lens_pos, uint8_t mask)
{
	int32_t rc = 0;
	unsigned char buf[2];
	uint8_t vcm_code_msb, vcm_code_lsb;

	vcm_code_msb = (lens_pos >> 8) & 0x3;
	vcm_code_lsb = lens_pos & 0xFF;

	buf[0] = OV8830_VCM_CODE_MSB;
	buf[1] = vcm_code_msb;

	rc = ov8830_i2c_txdata(OV8830_AF_I2C_ADDR >> 1, buf, 2);

	if (rc < 0)
		pr_err("[CAM]i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n", OV8830_AF_I2C_ADDR >> 1, buf[0], buf[1]);

	buf[0] = OV8830_VCM_CODE_LSB;
	buf[1] = vcm_code_lsb;

	rc = ov8830_i2c_txdata(OV8830_AF_I2C_ADDR >> 1, buf, 2);

	if (rc < 0)
		pr_err("[CAM]i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n", OV8830_AF_I2C_ADDR >> 1, buf[0], buf[1]);

	return rc;
}

static int32_t
ov8830_move_focus(int direction, int32_t num_steps)
{
  uint16_t ov8830_sw_damping_time_wait = 1;
  uint16_t ov8830_damping_threshold = 10;
  uint8_t ov8830_mode_mask = 0x02;
  int16_t step_direction;
  int16_t curr_lens_pos;
  int16_t curr_step_pos;
  int16_t dest_lens_pos;
  int16_t dest_step_pos;
  int16_t target_dist;
  int16_t small_step;
  int16_t next_lens_pos;
  int16_t time_wait_per_step;
  int32_t rc = 0, time_wait;
  int8_t ov8830_sw_damping_required = 0;
  uint16_t ov8830_max_fps_val;

  if (num_steps > OV8830_TOTAL_STEPS_NEAR_TO_FAR)
      num_steps = OV8830_TOTAL_STEPS_NEAR_TO_FAR;
  else if (num_steps == 0)
      return -EINVAL;

  if (direction == MOVE_NEAR)
      step_direction = 1;
  else if (direction == MOVE_FAR)
      step_direction = -1;
  else
      return -EINVAL;

  /* need to decide about default position and power supplied
   * at start up and reset */
  curr_lens_pos = ov8830_ctrl->curr_lens_pos;
  curr_step_pos = ov8830_ctrl->curr_step_pos;

  if (curr_lens_pos < ov8830_ctrl->init_curr_lens_pos)
      curr_lens_pos = ov8830_ctrl->init_curr_lens_pos;

  dest_step_pos = curr_step_pos + (step_direction * num_steps);

  if (dest_step_pos < 0)
      dest_step_pos = 0;
  else if (dest_step_pos > OV8830_TOTAL_STEPS_NEAR_TO_FAR)
      dest_step_pos = OV8830_TOTAL_STEPS_NEAR_TO_FAR;

  if (dest_step_pos == ov8830_ctrl->curr_step_pos)
      return rc;

  dest_lens_pos = ov8830_pos_tbl[dest_step_pos];
  target_dist = step_direction * (dest_lens_pos - curr_lens_pos);

  ov8830_max_fps_val = OV8830_MAX_FPS;

  /* HW damping */
  if (step_direction < 0
    && target_dist >= ov8830_pos_tbl[ov8830_damping_threshold]) {
    ov8830_sw_damping_required = 1;
    time_wait = 1000000
      / ov8830_max_fps_val
      - OV8830_SW_DAMPING_STEP * ov8830_sw_damping_time_wait * 1000;
  } else
    time_wait = 1000000 / ov8830_max_fps_val;

  time_wait_per_step = (int16_t) (time_wait / target_dist);

  if (time_wait_per_step >= 800)
    /* ~800 */
    ov8830_mode_mask = 0x5;
  else if (time_wait_per_step >= 400)
    /* ~400 */
    ov8830_mode_mask = 0x4;
  else if (time_wait_per_step >= 200)
    /* 200~400 */
    ov8830_mode_mask = 0x3;
  else if (time_wait_per_step >= 100)
    /* 100~200 */
    ov8830_mode_mask = 0x2;
  else if (time_wait_per_step >= 50)
    /* 50~100 */
    ov8830_mode_mask = 0x1;
  else {
    if (time_wait >= 17600)
      ov8830_mode_mask = 0x0D;
    else if (time_wait >= 8800)
      ov8830_mode_mask = 0x0C;
    else if (time_wait >= 4400)
      ov8830_mode_mask = 0x0B;
    else if (time_wait >= 2200)
      ov8830_mode_mask = 0x0A;
    else
      ov8830_mode_mask = 0x09;
  }

  if (ov8830_sw_damping_required) {
    small_step = (uint16_t) target_dist / OV8830_SW_DAMPING_STEP;
    if ((target_dist % OV8830_SW_DAMPING_STEP) != 0)
      small_step = small_step + 1;

    for (next_lens_pos = curr_lens_pos + (step_direction*small_step);
      (step_direction*next_lens_pos) <= (step_direction*dest_lens_pos);
      next_lens_pos += (step_direction*small_step)) {
      rc = ov8830_go_to_position(next_lens_pos, ov8830_mode_mask);
      if (rc < 0) {
      CDBG("[CAM] ov8830_go_to_position Failed in Move Focus!!!\n");
      return rc;
      }
      curr_lens_pos = next_lens_pos;
      mdelay(ov8830_sw_damping_time_wait);
    }

    if (curr_lens_pos != dest_lens_pos) {
      rc = ov8830_go_to_position(dest_lens_pos, ov8830_mode_mask);
      if (rc < 0) {
      pr_err("[CAM]ov8830_go_to_position Failed in Move Focus!!!\n");
      return rc;
      }
      mdelay(ov8830_sw_damping_time_wait);
    }
  } else {
    rc = ov8830_go_to_position(dest_lens_pos, ov8830_mode_mask);
    if (rc < 0) {
      pr_err("[CAM]ov8830_go_to_position Failed in Move Focus!!!\n");
      return rc;
    }
  }

  ov8830_ctrl->curr_lens_pos = dest_lens_pos;
  ov8830_ctrl->curr_step_pos = dest_step_pos;

  return rc;
}

static int32_t
ov8830_set_default_focus(void)
{
  int32_t rc = 0;
  if (ov8830_ctrl->curr_step_pos != 0) {
    rc = ov8830_move_focus(MOVE_FAR, ov8830_ctrl->curr_step_pos);
    if (rc < 0) {
      pr_err("[CAM]ov8830_set_default_focus Failed!!!\n");
      return rc;
    }
  } else {
    rc = ov8830_go_to_position(0, 0x02);
    if (rc < 0) {
      pr_err("[CAM]ov8830_go_to_position Failed!!!\n");
      return rc;
    }
  }

  ov8830_ctrl->curr_lens_pos = 0;
  ov8830_ctrl->init_curr_lens_pos = 0;
  ov8830_ctrl->curr_step_pos = 0;

  return rc;
}

static void ov8830_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */

	uint32_t divider, d1, d2;

	uint16_t snapshot_height, preview_height, preview_width, snapshot_width;

	if (ov8830_ctrl->prev_res == QTR_SIZE) {
		preview_width =
			OV8830_QTR_SIZE_WIDTH  + OV8830_HRZ_QTR_BLK_PIXELS ;
		preview_height =
			OV8830_QTR_SIZE_HEIGHT + ov8830_ver_qtr_blk_lines_array[cam_mode_sel] ;
	} else if (ov8830_ctrl->prev_res == VIDEO_SIZE) {
		preview_width =
			OV8830_VIDEO_SIZE_WIDTH  + OV8830_HRZ_VIDEO_BLK_PIXELS;
		preview_height =
			OV8830_VIDEO_SIZE_HEIGHT + OV8830_VER_VIDEO_BLK_LINES;
    } else if (ov8830_ctrl->prev_res == FAST_VIDEO_SIZE) {
    preview_width =
      OV8830_VIDEO_SIZE_WIDTH_FAST	+ OV8830_HRZ_VIDEO_BLK_PIXELS_FAST;
    preview_height =
      OV8830_VIDEO_SIZE_HEIGHT_FAST + OV8830_VER_VIDEO_BLK_LINES_FAST;
	} else {
		/* full size resolution used for preview. */
		preview_width =
			OV8830_FULL_SIZE_WIDTH + OV8830_HRZ_FULL_BLK_PIXELS ;
		preview_height =
			OV8830_FULL_SIZE_HEIGHT + OV8830_VER_FULL_BLK_LINES ;
	}

	if (ov8830_ctrl->pict_res == QTR_SIZE) {
		snapshot_width =
			OV8830_QTR_SIZE_WIDTH + OV8830_HRZ_QTR_BLK_PIXELS ;
		snapshot_height =
			OV8830_QTR_SIZE_HEIGHT + ov8830_ver_qtr_blk_lines_array[cam_mode_sel] ;

	} else {
		snapshot_width =
			OV8830_FULL_SIZE_WIDTH + OV8830_HRZ_FULL_BLK_PIXELS;
		snapshot_height =
			OV8830_FULL_SIZE_HEIGHT + OV8830_VER_FULL_BLK_LINES;
	}

	d1 = preview_height * 0x00000400 / snapshot_height;
	d2 = preview_width * 0x00000400 / snapshot_width;

	divider = (uint32_t) (d1 * d2) / 0x00000400;
	*pfps = (uint16_t)(fps * divider / 0x00000400);

} /* endof ov8830_get_pict_fps */

static uint16_t ov8830_get_prev_lines_pf(void)
{
  if (ov8830_ctrl->prev_res == QTR_SIZE) {
    return (OV8830_QTR_SIZE_HEIGHT + ov8830_ver_qtr_blk_lines_array[cam_mode_sel]);
  } else if (ov8830_ctrl->prev_res == VIDEO_SIZE) {
    return (OV8830_VIDEO_SIZE_HEIGHT + OV8830_VER_VIDEO_BLK_LINES);
  } else if (ov8830_ctrl->prev_res == FAST_VIDEO_SIZE) {
    return (OV8830_VIDEO_SIZE_HEIGHT_FAST + OV8830_VER_VIDEO_BLK_LINES_FAST);
  } else  {
    return (OV8830_FULL_SIZE_HEIGHT + OV8830_VER_FULL_BLK_LINES);
  }

}

static uint16_t ov8830_get_prev_pixels_pl(void)
{
    if (ov8830_ctrl->prev_res == QTR_SIZE) {
      return (OV8830_QTR_SIZE_WIDTH + OV8830_HRZ_QTR_BLK_PIXELS);
    } else if (ov8830_ctrl->prev_res == VIDEO_SIZE) {
      return (OV8830_VIDEO_SIZE_WIDTH + OV8830_HRZ_VIDEO_BLK_PIXELS);
	  } else if (ov8830_ctrl->prev_res == FAST_VIDEO_SIZE) {
      return (OV8830_VIDEO_SIZE_WIDTH_FAST + OV8830_HRZ_VIDEO_BLK_PIXELS_FAST);
	  } else {
      return (OV8830_FULL_SIZE_WIDTH + OV8830_HRZ_FULL_BLK_PIXELS);
  }
}

static uint16_t ov8830_get_pict_lines_pf(void)
{
	if (ov8830_ctrl->pict_res == QTR_SIZE) {
		return (OV8830_QTR_SIZE_HEIGHT + ov8830_ver_qtr_blk_lines_array[cam_mode_sel]);
	} else  {
		return (OV8830_FULL_SIZE_HEIGHT + OV8830_VER_FULL_BLK_LINES);
	}
}

static uint16_t ov8830_get_pict_pixels_pl(void)
{
	if (ov8830_ctrl->pict_res == QTR_SIZE) {
		return (OV8830_QTR_SIZE_WIDTH + OV8830_HRZ_QTR_BLK_PIXELS);
	} else  {
		return (OV8830_FULL_SIZE_WIDTH + OV8830_HRZ_FULL_BLK_PIXELS);
	}
}

static uint32_t ov8830_get_pict_max_exp_lc(void)
{
	if (ov8830_ctrl->pict_res == QTR_SIZE) {
		return (OV8830_QTR_SIZE_HEIGHT + ov8830_ver_qtr_blk_lines_array[cam_mode_sel]);
	} else  {
		return (OV8830_FULL_SIZE_HEIGHT + OV8830_VER_FULL_BLK_LINES);
	}
}


static int32_t ov8830_write_exp_gain
			(uint16_t mul, uint16_t gain, uint32_t line)
{
	uint32_t aec_msb_24; /*easter for frame rate=10fps 20110526*/
	uint16_t aec_msb;
	uint16_t aec_lsb;
	int32_t rc = 0;
	uint32_t total_lines_per_frame;
	uint32_t total_pixels_per_line;
	uint16_t offset = OV8830_OFFSET;
	/*uint32_t line_length_ratio = 1 * Q8;*/
	/*uint8_t ov8830_offset = 2; */
	/*uint32_t extra_line_length = 0;*/
	/*uint16_t extra_line_msb = 0;*/
	/*uint16_t extra_line_lsb = 0;*/
	uint32_t phy_line = 0;
	uint32_t phy_line_2 = 0;
	uint8_t phy_mul = MUL_GAIN_INIT_VALUE;
	uint16_t phy_gain = 0;
	/*uint32_t phy_extra_line_length = 0;*/
	uint16_t lf_msb;
	uint16_t lf_lsb;
	uint32_t fps_divider;

	CDBG("[CAM] %s start, mul = %d gain = %d line = %d\n", __func__,
		mul, gain, line);

	if (ov8830_ctrl->curr_res == QTR_SIZE) {
		total_lines_per_frame =
			(OV8830_QTR_SIZE_HEIGHT + ov8830_ver_qtr_blk_lines_array[cam_mode_sel]);
		total_pixels_per_line =
			OV8830_QTR_SIZE_WIDTH + OV8830_HRZ_QTR_BLK_PIXELS;
	} else if (ov8830_ctrl->curr_res == VIDEO_SIZE) {
		total_lines_per_frame =
			OV8830_VIDEO_SIZE_HEIGHT + OV8830_VER_VIDEO_BLK_LINES;
		total_pixels_per_line =
			OV8830_VIDEO_SIZE_WIDTH + OV8830_HRZ_VIDEO_BLK_PIXELS;
	} else if (ov8830_ctrl->curr_res == FAST_VIDEO_SIZE) {
		total_lines_per_frame =
			OV8830_VIDEO_SIZE_HEIGHT_FAST + OV8830_VER_VIDEO_BLK_LINES_FAST;
		total_pixels_per_line =
			OV8830_VIDEO_SIZE_WIDTH_FAST + OV8830_HRZ_VIDEO_BLK_PIXELS_FAST;
	} else {
		total_lines_per_frame =
			(OV8830_FULL_SIZE_HEIGHT + OV8830_VER_FULL_BLK_LINES);
		total_pixels_per_line =
			OV8830_FULL_SIZE_WIDTH + OV8830_HRZ_FULL_BLK_PIXELS;
	}

	phy_line = line;
	phy_mul = mul;
	phy_gain = gain;
	/*phy_extra_line_length = extra_line_length;*/

	if (ov8830_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		fps_divider = ov8830_ctrl->fps_divider;
		ov8830_ctrl->my_reg_gain = gain;
		ov8830_ctrl->my_reg_dig_gain = mul;
		ov8830_ctrl->my_reg_line_count = (uint16_t)line;
	} else
		fps_divider = ov8830_ctrl->pict_fps_divider;

	/*extra_line_msb = (uint16_t)(phy_extra_line_length & 0xFF00) >> 8;*/
	/*extra_line_lsb = (uint16_t)(phy_extra_line_length & 0x00FF);*/

	phy_line_2 = phy_line << 4;  /*1995*/
	aec_msb_24 = (uint32_t)(phy_line_2 & 0xFF0000) >> 16;/*easter for frame rate=10fps 20110526*/
	aec_msb = (uint16_t)(phy_line_2 & 0xFF00) >> 8;
	aec_lsb = (uint16_t)(phy_line_2 & 0x00FF);


	rc = ov8830_i2c_write_b(ov8830_client->addr, OV8830_AEC_MSB_24, (uint8_t)aec_msb_24);
	if (rc < 0)
		return rc;/*easter for frame rate=10fps 20110526*/

	rc = ov8830_i2c_write_b(ov8830_client->addr, OV8830_AEC_MSB, (uint8_t)aec_msb);
	if (rc < 0)
		return rc;

	rc = ov8830_i2c_write_b(ov8830_client->addr, OV8830_AEC_LSB, (uint8_t)aec_lsb);
	if (rc < 0)
		return rc;

  rc = ov8830_i2c_write_b(ov8830_client->addr, OV8830_GAIN_LSB, (uint8_t)phy_gain);
	if (rc < 0)
		return rc;

	if (phy_line * 0x400 <= (total_lines_per_frame - offset) * fps_divider) {
		total_lines_per_frame = (total_lines_per_frame * fps_divider) / 0x400;
		lf_msb = (uint16_t)((total_lines_per_frame) & 0xFF00) >> 8;
		lf_lsb = (uint16_t)((total_lines_per_frame) & 0x00FF);
	} else {
		lf_msb = (uint16_t)((total_lines_per_frame + (phy_line - (total_lines_per_frame - offset))) & 0xFF00) >> 8;
		lf_lsb = (uint16_t)((total_lines_per_frame + (phy_line - (total_lines_per_frame - offset))) & 0x00FF);
	}

	rc = ov8830_i2c_write_b(ov8830_client->addr, REG_VSYNC_WIDTH_MSB, (uint8_t)lf_msb);
	if (rc < 0)
		return rc;

	rc = ov8830_i2c_write_b(ov8830_client->addr, REG_VSYNC_WIDTH_LSB, (uint8_t)lf_lsb);
	if (rc < 0)
		return rc;

	/*stored_line_length_ratio = line_length_ratio;*/
	return rc;

} /* endof ov8830_write_exp_gain*/

/* ### this function is not called for userspace ### */
static int32_t ov8830_set_pict_exp_gain
			(uint16_t mul, uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = ov8830_write_exp_gain(mul, gain, line);
	return rc;
} /* endof ov8830_set_pict_exp_gain*/

static int32_t ov8830_set_fps(struct fps_cfg *fps)
{
	int32_t rc = 0;
	uint32_t delay;
	uint32_t min_coarse = OV8830_MIN_COARSE_INTEGRATION_TIME;
	uint32_t pre_fps_divider = ov8830_ctrl->fps_divider;
	uint32_t pre_fps = ov8830_ctrl->fps;
	ov8830_ctrl->fps_divider = fps->fps_div;
	ov8830_ctrl->pict_fps_divider = fps->pict_fps_div;
	ov8830_ctrl->fps = fps->f_mult;

	if (ov8830_ctrl->sensormode == SENSOR_PREVIEW_MODE &&
		(ov8830_ctrl->my_reg_gain != 0 || ov8830_ctrl->my_reg_line_count != 0) &&
		(pre_fps != ov8830_ctrl->fps || pre_fps_divider != ov8830_ctrl->fps_divider)) {
		min_coarse = (min_coarse * ov8830_ctrl->fps_divider + 0x400 - 1) / 0x400;
		if (ov8830_ctrl->my_reg_line_count < min_coarse)
			ov8830_ctrl->my_reg_line_count = min_coarse;
		rc =
			ov8830_write_exp_gain(ov8830_ctrl->my_reg_dig_gain, ov8830_ctrl->my_reg_gain,
				ov8830_ctrl->my_reg_line_count);

		delay = (1000 * Q8 / pre_fps) + 1;
		mdelay(delay);
	}

	return rc;
}

/* remove test code */
#if 0
static int32_t ov8830_test(enum ov8830_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF) {
		return rc;
	}

	/* Activate  the Color bar test pattern */
	if (mo == TEST_1) {
		rc = ov8830_i2c_write_b(ov8830_client->addr,
			OV8830_COLOR_BAR_ENABLE_REG, 0xa0);
		if (rc < 0) {
			return rc;
		}
		rc = ov8830_i2c_write_b(ov8830_client->addr,
			0x3085, 0x20);
		if (rc < 0) {
			return rc;
		}
		rc = ov8830_i2c_write_b(ov8830_client->addr,
			0x306c, 0x00);
		if (rc < 0) {
			return rc;
		}
		rc = ov8830_i2c_write_b(ov8830_client->addr,
			OV8830_COLOR_BAR_PATTERN_SEL_REG, 0x02);
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

#if 0
static int32_t HTC_update_ov8830_lsc_registers(void)
{
	int i;
	struct awb_lsc_struct_type *awb_lsc_data_ptr;
	awb_lsc_data_ptr = (struct awb_lsc_struct_type *)get_cam_awb_cal();

	for (i = 0; i < 8; i++) {
		pr_info(KERN_INFO"[CAM][LSC calibration]  read AWB table 0x%x\n",
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
			ov8830_i2c_write_b(ov8830_client->addr,
				awb_lsc_data_ptr->LSC_table[i].reg_addr,
				awb_lsc_data_ptr->LSC_table[i].reg_val);
		}
		/*enable lsc on sensor*/
		ov8830_i2c_write_b(ov8830_client->addr, 0x3300, 0xff);
		/*move to the last*/
		ov8830_i2c_write_b(ov8830_client->addr,
			OV8830_REG_MODE_SELECT, OV8830_MODE_SELECT_STREAM);

	} else {/*use default LSC table*/
	      pr_info("[CAM][LSC calibration]checksum fail\n");
	      return false;
	}
	return true;
}
#endif

static int32_t ov8830_i2c_write_table(
	struct ov8830_i2c_reg_conf *reg_cfg_tbl, int num)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num; i++) {
		rc = ov8830_i2c_write_b(ov8830_client->addr,
			reg_cfg_tbl->waddr, reg_cfg_tbl->bdata);
		if (rc < 0)
			break;
		reg_cfg_tbl++;
	}

	return rc;
}


static int32_t initialize_ov8830_registers(void)
{
	int32_t rc = 0;

	/* Configure sensor for Preview mode and Snapshot mode */
   rc = ov8830_i2c_write_table(ov8830_regs.full_mipi, ov8830_regs.full_mipi_size);

	return rc;
} /* end of initialize_ov8830_ov8m0vc_registers. */

static int32_t ov8830_setting(int rt)
{
	int32_t rc = 0;
	/*int32_t i, array_length;*/
	static int16_t did_snapshot;
	/*uint16_t ori_reg_mul_gain;*/
	/*uint8_t ori_reg_mul_gain_8bit;*/
	struct msm_camera_csi_params ov8830_csi_params;
	struct msm_camera_sensor_info *sinfo = ov8830_pdev->dev.platform_data;

	write_cnt = 0;

	if (sinfo->csi_if) {
		if (ov8830_ctrl->reg_update == REG_INIT) {
			/* config mipi csi controller */
			ov8830_csi_params.data_format = CSI_10BIT;
			ov8830_csi_params.lane_cnt = 2;
			ov8830_csi_params.lane_assign = 0xe4;
			ov8830_csi_params.dpcm_scheme = 0;
			ov8830_csi_params.settle_cnt = 20;
			ov8830_csi_params.mipi_driving_strength = 0;
			ov8830_csi_params.hs_impedence = 0x0F;
			rc = msm_camio_csi_config(&ov8830_csi_params);

			ov8830_ctrl->reg_update = REG_PERIODIC;
			pr_info("[CAM]after set csi config\n");
		}
	}

	pr_info("[CAM]ov8830_setting,rt=%d\n", rt);

	switch (rt) {

	case QTR_SIZE:
	case VIDEO_SIZE:
	case FAST_VIDEO_SIZE:

    rc = ov8830_i2c_write_table(ov8830_regs.common_mipi, ov8830_regs.common_mipi_size);

    if (rc < 0)
      return rc;

		if (rt == VIDEO_SIZE) {
			pr_info("[CAM]ov8830_setting(VIDEO_SIZE)\n");
			rc = ov8830_i2c_write_table(ov8830_regs.video_mipi, ov8830_regs.video_mipi_size);
		} else if (rt == FAST_VIDEO_SIZE) {
			pr_info("[CAM]ov8830_setting(FAST_VIDEO_SIZE)\n");
			rc = ov8830_i2c_write_table(ov8830_regs.fast_video_mipi, ov8830_regs.fast_video_mipi_size);
		} else {
			pr_info("[CAM]ov8830_setting(QTR_SIZE)\n");
			rc = ov8830_i2c_write_table(ov8830_regs.qtr_mipi, ov8830_regs.qtr_mipi_size);
		}

    if (rc < 0) {
			pr_info("[CAM]fail,QTR_SIZE\n");
			return rc;
    }

		if (rt == VIDEO_SIZE) {
			ov8830_ctrl->curr_res = VIDEO_SIZE;
		} else if (rt == FAST_VIDEO_SIZE) {
			ov8830_ctrl->curr_res = FAST_VIDEO_SIZE;
		} else {
			ov8830_ctrl->curr_res = QTR_SIZE;
		}

		if (ov8830_ctrl->sensormode == SENSOR_PREVIEW_MODE &&
				(ov8830_ctrl->my_reg_gain != 0 || ov8830_ctrl->my_reg_dig_gain != 0 ||
				ov8830_ctrl->my_reg_line_count != 0))
			ov8830_write_exp_gain(ov8830_ctrl->my_reg_dig_gain,
				ov8830_ctrl->my_reg_gain, ov8830_ctrl->my_reg_line_count);

		/*streaming on*/
		rc = ov8830_i2c_write_b(ov8830_client->addr,
		0x0100, 0x01);
		if (rc < 0)
			return rc;

		if (sinfo->mirror_mode) {
			if (rt == QTR_SIZE || rt == FAST_VIDEO_SIZE) {
				rc = ov8830_i2c_write_b(ov8830_client->addr,
				OV8830_REG_MODE_FLIP, OV8830_REG_FLIP+1);  /*+1 for binning*/
				if (rc < 0)
					return rc;

				rc = ov8830_i2c_write_b(ov8830_client->addr,
				OV8830_REG_MODE_MIRROR, OV8830_REG_MIRROR+1); /*+1 for binning*/
				if (rc < 0)
					return rc;
			} else {
				rc = ov8830_i2c_write_b(ov8830_client->addr,
				OV8830_REG_MODE_FLIP, OV8830_REG_FLIP);
				if (rc < 0)
					return rc;

				rc = ov8830_i2c_write_b(ov8830_client->addr,
				OV8830_REG_MODE_MIRROR, OV8830_REG_MIRROR);
				if (rc < 0)
					return rc;
			}
		}

		did_snapshot = 0;

		break;

	case FULL_SIZE:

    pr_info("[CAM]ov8830_setting FULL_SIZE\n");

    rc = ov8830_i2c_write_table(ov8830_regs.common_mipi, ov8830_regs.common_mipi_size);

    if (rc < 0)
      return rc;

    rc = ov8830_i2c_write_table(ov8830_regs.full_mipi, ov8830_regs.full_mipi_size);
    if (rc < 0) {
			pr_info("[CAM]fail\n");
			return rc;
    }

		ov8830_ctrl->curr_res = FULL_SIZE;

		if (ov8830_ctrl->sensormode == SENSOR_PREVIEW_MODE &&
				(ov8830_ctrl->my_reg_gain != 0 || ov8830_ctrl->my_reg_dig_gain != 0 ||
				ov8830_ctrl->my_reg_line_count != 0))
			ov8830_write_exp_gain(ov8830_ctrl->my_reg_dig_gain,
				ov8830_ctrl->my_reg_gain, ov8830_ctrl->my_reg_line_count);

		/*streaming on*/
		rc = ov8830_i2c_write_b(ov8830_client->addr,
		0x0100, 0x01);
		if (rc < 0)
			return rc;

		if (sinfo->mirror_mode) {
			rc = ov8830_i2c_write_b(ov8830_client->addr,
			OV8830_REG_MODE_FLIP, OV8830_REG_FLIP);
			if (rc < 0)
				return rc;

			rc = ov8830_i2c_write_b(ov8830_client->addr,
			OV8830_REG_MODE_MIRROR, OV8830_REG_MIRROR);
			if (rc < 0)
				return rc;
		}

		did_snapshot = 1;
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
		rc = ov8830_i2c_write_b(ov8830_client->addr, 0x3300, 0xef);
		/*mirror off*/
		rc = ov8830_i2c_write_b(ov8830_client->addr, 0x30f8, 0x00);
		/*mirror off*/
		rc = ov8830_i2c_write_b(ov8830_client->addr, 0x3316, 0x02);
		pr_info("[CAM][LSC calibration]turn off LSC!Mirror On\n");

		/*fix gain & linecount*/
		/*Gain=0x9,exp=008d*/
		/*so luma taget = 100 to mfg light source*/
		rc = ov8830_i2c_write_b(ov8830_client->addr, 0x3000, 0x9);
		/*AEC_MSB*/
		rc = ov8830_i2c_write_b(ov8830_client->addr, 0x3002, 0x00);
		/*AEC_LSB*/
		rc = ov8830_i2c_write_b(ov8830_client->addr, 0x3003, 0x8d);
		pr_info("[CAM][LSC calibration]fix gain & linecount\n");
		global_mode = 0;
	}

/* remove test code
	rc = ov8830_test(ov8830_ctrl->set_test);
	if (rc < 0)
		return rc;
*/

	return rc;
} /*endof  ov8830_setting*/

static int32_t ov8830_video_config(int mode)
{
    int32_t rc = 0;
    static int pre_sel = 0;
    int cur_sel = (cam_mode_sel > 1)?1:0;
    enum ov8830_resolution_t curr_res = ov8830_ctrl->curr_res;
    uint32_t curr_mode = ov8830_ctrl->sensormode;

    ov8830_ctrl->sensormode = mode;

    pr_info("[CAM]%s cam_mode_sel %d cur_sel %d \n", __func__, cam_mode_sel, cur_sel);

    preview_frame_count = 0;

    if (ov8830_ctrl->curr_res != ov8830_ctrl->prev_res
      || pre_sel != cur_sel
      )  {
				rc = ov8830_setting(ov8830_ctrl->prev_res);
				if (rc < 0) {
					ov8830_ctrl->curr_res = curr_res;
					ov8830_ctrl->sensormode = curr_mode;
					return rc;
				}
    } else {
				ov8830_ctrl->curr_res = ov8830_ctrl->prev_res;
    }

    pre_sel = cur_sel;

    return rc;

} /*end of ov354_video_config*/

static int32_t ov8830_snapshot_config(int mode)
{
	int32_t rc = 0;
	enum ov8830_resolution_t curr_res = ov8830_ctrl->curr_res;
	uint32_t curr_mode = ov8830_ctrl->sensormode;

	ov8830_ctrl->sensormode = mode;

	if (ov8830_ctrl->curr_res != ov8830_ctrl->pict_res) {
		rc = ov8830_setting(ov8830_ctrl->pict_res);
		if (rc < 0) {
			ov8830_ctrl->curr_res = curr_res;
			ov8830_ctrl->sensormode = curr_mode;
			return rc;
		}
	} else {
		ov8830_ctrl->curr_res = ov8830_ctrl->pict_res;
	}

	return rc;

} /*end of ov8830_snapshot_config*/

static int32_t ov8830_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	enum ov8830_resolution_t curr_res = ov8830_ctrl->curr_res;
	uint32_t curr_mode = ov8830_ctrl->sensormode;

	ov8830_ctrl->sensormode = mode;
	if (ov8830_ctrl->curr_res != ov8830_ctrl->pict_res) {
		rc = ov8830_setting(ov8830_ctrl->pict_res);
		if (rc < 0) {
			ov8830_ctrl->curr_res = curr_res;
			ov8830_ctrl->sensormode = curr_mode;
			return rc;
		}
	} else {
		ov8830_ctrl->curr_res = ov8830_ctrl->pict_res;
	} /* Update sensor resolution */

	return rc;

} /*end of ov8830_raw_snapshot_config*/

static int32_t ov8830_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *sinfo = ov8830_pdev->dev.platform_data;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		ov8830_ctrl->prev_res = res; /* VIDEO_SIZE, FAST_VIDEO_SIZE, FULL_SIZE, QTR_SIZE */
		rc = ov8830_video_config(mode);
		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_info("[CAM]KPI PA: start sensor snapshot config: %d\n", __LINE__);
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		ov8830_ctrl->pict_res = res;
		rc = ov8830_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		/*global_mode = 1; //20100330 vincent lsc calibration*/
		pr_info("[CAM]KPI PA: start sensor snapshot config: %d\n", __LINE__);
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		ov8830_ctrl->pict_res = res;
		rc = ov8830_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t ov8830_power_down(void)
{
	return 0;
}

static int ov8830_probe_read_id(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t  chipidh = 0; /*, chipidl;*/
	uint16_t  def_chipid = 0;
	msleep(20);
	pr_info("[CAM]%s, ov8830_probe_init_sensor 1\n", __func__);
	/* 3. Read sensor Model ID: */
	if (ov8830_i2c_read(OV8830_PIDH_REG, &chipidh, 2) < 0) {
		rc = -1;
		pr_err("[CAM]read sensor id fail\n");
	}

	pr_info("[CAM]ov8830 model_id + ver = 0x%x\n", chipidh);

	/* 4. Compare sensor ID to OV8830 ID: */
	def_chipid = (((OV8830_PID << 8) & 0xFF00) + (OV8830_VER & 0x00FF));
	pr_info("[CAM]%s, Expected id=0x%x\n", __func__, def_chipid);

	if (chipidh < def_chipid) {
		rc = -ENODEV;
		pr_err("[CAM]read sensor id incorrect\n");
	}

	pr_info("[CAM]%s, vreg_get vreg_af_actuator\n", __func__);
	/*vreg_af_actuator = vreg_get(0, "gp5");*/
	if (IS_ERR(vreg_af_actuator))
		return PTR_ERR(vreg_af_actuator);

#ifdef CONFIG_ARCH_QSD8X50
	data->camera_set_source(MAIN_SOURCE);
#endif
	pr_info("[CAM]ov8830_probe_init_sensor finishes\n");
	return rc;
}

#ifndef CONFIG_ARCH_QSD8X50
static int ov8830_vreg_enable(struct platform_device *pdev)
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

static int ov8830_sensor_open_init(struct msm_camera_sensor_info *data)
{

	int i;
	int32_t  rc = 0;
	/*stella0122*/
	uint16_t ov8830_nl_region_boundary = 5; /*3;*/
	uint16_t ov8830_nl_region_code_per_step = 35; /*101;*/
	uint16_t ov8830_l_region_code_per_step = 20; /*18;*/
	int timeout;
	int count = 0;
	pr_info("[CAM]Calling ov8830_sensor_open_init\n");
	down(&ov8830_sem);

	if (data == NULL) {
		pr_info("[CAM]data is a NULL pointer\n");
		return -EINVAL;
	}
	/*check whether resume done*/
	timeout = wait_event_interruptible_timeout(
		ov8830_event.event_wait,
		ov8830_event.waked_up,
		30*HZ);
	pr_info("[CAM]wait event : %d timeout:%d\n", ov8830_event.waked_up, timeout);
	if (timeout == 0) {
		up(&ov8830_sem);
		return rc;
	}
	msm_camio_probe_on(ov8830_pdev);
	ov8830_ctrl = kzalloc(sizeof(struct ov8830_ctrl), GFP_KERNEL);
	if (!ov8830_ctrl) {
		pr_err("[CAM]ov8830_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov8830_ctrl->curr_lens_pos = -1;
	ov8830_ctrl->fps_divider = 1 * 0x00000400;
	ov8830_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov8830_ctrl->set_test = TEST_OFF;
	ov8830_ctrl->prev_res = QTR_SIZE;
	ov8830_ctrl->pict_res = FULL_SIZE;
	ov8830_ctrl->curr_res = INVALID_SIZE;
	ov8830_ctrl->my_reg_gain = 0;
	ov8830_ctrl->my_reg_dig_gain = 0;
	ov8830_ctrl->my_reg_line_count = 0;
	if (data)
		ov8830_ctrl->sensordata = data;

	pr_info("[CAM]%s  add open retry policy !!!\n", __func__);
retry:
	rc = ov8830_vreg_enable(ov8830_pdev);
	if (rc < 0) {
		pr_err("[CAM]fail,__ov8830_probe rc < 0\n");
	}

	/*PWD and RST config*/
	pr_info("[CAM]%s, GPIO(%d) sensor_pwd 0\n", __func__, data->sensor_pwd);
	if (data->sensor_pwd >= 0) {
		rc = gpio_request(data->sensor_pwd, "ov8830");
		if (!rc)
			gpio_direction_output(data->sensor_pwd, 0);
		else
			pr_err("[CAM]GPIO (%d) request faile\n", data->sensor_pwd);
		gpio_free(data->sensor_pwd);
	} else {
      if (data->camera_pm8058_power != NULL) {
		if (data->camera_pm8058_power(0) < 0)
		  pr_err("[CAM]camera_pm8058_power(0): request failed\n");
	  }
	}
	msleep(5);

	if (data->camera_clk_switch != NULL)
		data->camera_clk_switch();
	mdelay(5);

	/*power down setup*/
	if (data->sensor_pwd >= 0) {
		rc = gpio_request(data->sensor_pwd, "ov8830");
		if (!rc)
			gpio_direction_output(data->sensor_pwd, 1);
		else
			pr_err("[CAM]GPIO (%d) request faile\n", data->sensor_pwd);
		gpio_free(data->sensor_pwd);
	} else {
		if (data->camera_pm8058_power != NULL) {
			if (data->camera_pm8058_power(1) < 0)
				goto init_fail;
		}
	}
	mdelay(5);

	data->pdata->camera_gpio_on();

	/*set MCLK*/
	pr_info("[CAM]%s, msm_camio_clk_rate_set %d\n",
		__func__, OV8830_DEFAULT_CLOCK_RATE);

       msm_camio_clk_rate_set(OV8830_DEFAULT_CLOCK_RATE);

	msleep(10);

	if (data->vcm_pwd) {
		rc = gpio_request(data->vcm_pwd, "ov8830");
		if (!rc) {
			gpio_direction_output(data->vcm_pwd, 1);
		} else {
			pr_err("[CAM]GPIO (%d) request failed\n", data->vcm_pwd);
			goto init_fail;
		}
		gpio_free(data->vcm_pwd);
	}

	/*read sensor id*/
	rc = ov8830_probe_read_id(data);
	if (rc < 0) {
		pr_err("[CAM]starting open retry policy count:%d\n", count);
		mdelay(10);
		count++;
		if (count < 10) {
			ov8830_common_deinit();
			mdelay(10);
		} else
			goto init_fail;
		goto retry;
	}

	ov8830_ctrl->sensormode = SENSOR_PREVIEW_MODE ;

	if (rc < 0)
		goto init_fail;

	pr_info("[CAM]%s, enable AF actuator %d\n", __func__, __LINE__);

	msleep(1);

	ov8830_ctrl->fps = 30*Q8;

	step_position_table[0] = 0;

	for (i = 1; i <= OV8830_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= ov8830_nl_region_boundary) {
			ov8830_step_position_table[i] =
				ov8830_step_position_table[i-1] +
				ov8830_nl_region_code_per_step;
		} else {
			ov8830_step_position_table[i] =
				ov8830_step_position_table[i-1] +
				ov8830_l_region_code_per_step;
		}
	}

	 /* generate test pattern */
	pr_info("[CAM]%s, generate test pattern, %d, rc=%d\n",
		__func__, __LINE__, rc);

	/* set up lens position table */
	ov8830_setup_af_tbl();
	ov8830_go_to_position(0, 0);
	ov8830_ctrl->curr_lens_pos = 0;
	ov8830_ctrl->curr_step_pos = 0;

	if (rc >= 0)
		goto init_done;
	    /* reset the driver state */
init_fail:
	pr_err("[CAM]%s: init_fail\n", __func__);
	/*vreg_disable(vreg_af_actuator);*/
	if (ov8830_ctrl) {
		kfree(ov8830_ctrl);
		ov8830_ctrl = NULL;
	}
init_done:
	up(&ov8830_sem);
	pr_info("[CAM]%s: init_done\n", __func__);
	return rc;

} /*endof ov8830_sensor_open_init*/

static int ov8830_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov8830_wait_queue);
	return 0;
}

static const struct i2c_device_id ov8830_i2c_id[] = {
	{ "ov8830", 0},
	{ }
};

static int ov8830_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_info("[CAM]ov8830_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[CAM]i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov8830_sensorw = kzalloc(sizeof(struct ov8830_work), GFP_KERNEL);
	if (!ov8830_sensorw) {
		pr_err("[CAM]kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov8830_sensorw);
	ov8830_init_client(client);
	ov8830_client = client;

	msleep(50);

	pr_info("[CAM]ov8830_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("[CAM]ov8830_probe failed! rc = %d\n", rc);
	return rc;
}

static int ov8830_vreg_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	int rc;
	pr_info("[CAM] %s camera vreg off\n", __func__);
	if (sdata->camera_power_off == NULL) {
		pr_err("[CAM]sensor platform_data didnt register\n");
		return -EIO;
	}
	rc = sdata->camera_power_off();
	return rc;
}

static int ov8830_probe_init_done(const struct msm_camera_sensor_info *data)
{
	int rc;

	if (data->sensor_pwd >= 0) {
		rc = gpio_request(data->sensor_pwd, "ov8830");
		if (!rc)
			gpio_direction_output(data->sensor_pwd, 0);
		else
			pr_err("[CAM]GPIO (%d) request faile\n", data->sensor_pwd);
		gpio_free(data->sensor_pwd);
	} else {
      if (data->camera_pm8058_power != NULL) {
		if (data->camera_pm8058_power(0) < 0)
		  pr_err("[CAM]camera_pm8058_power(0): request failed\n");
	  }
	}
	mdelay(1);

  if (data->vcm_pwd) {
    rc = gpio_request(data->vcm_pwd, "ov8830");
    if (!rc)
      gpio_direction_output(data->vcm_pwd, 0);
    else
      pr_err("[CAM]GPIO (%d) request faile\n", data->vcm_pwd);
    gpio_free(data->vcm_pwd);
  }
  mdelay(1);


  data->pdata->camera_gpio_off();
  mdelay(1);

  pr_info("[CAM] data->power_down_disable=%d", data->power_down_disable);
  if (!data->power_down_disable) {
    ov8830_vreg_disable(ov8830_pdev);
  }

	return 0;
}

static int ov8830_suspend(struct platform_device *pdev, pm_message_t state)
{
	int rc;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;

	if (!sinfo->need_suspend)
		return 0;
	ov8830_event.waked_up = 0;

	pr_info("[CAM]ov8830: camera suspend\n");

	pr_info("[CAM]%s, vreg_af_actuator vreg_disable\n", __func__);
	/*vreg_disable(vreg_af_actuator);*/

	rc = gpio_request(sinfo->sensor_reset, "ov8830");
	if (!rc)
		gpio_direction_output(sinfo->sensor_reset, 0);
	else
		pr_info("[CAM]ov8830: request GPIO(sensor_reset) :%d faile\n",
			sinfo->sensor_reset);

	gpio_free(sinfo->sensor_reset);
	msleep(10);
		rc = gpio_request(sinfo->sensor_pwd, "ov8830");
	if (!rc)
		gpio_direction_output(sinfo->sensor_pwd, 0);
	else
		pr_info("[CAM]ov8830: request GPIO(sensor_reset) :%d faile\n",
			sinfo->sensor_pwd);

	gpio_free(sinfo->sensor_pwd);

	pr_info("[CAM]ov8830:suspend done\n");
	return rc;
}

static void ov8830_resume(struct early_suspend *handler)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = ov8830_pdev->dev.platform_data;
	pr_info("[CAM]ov8830_resume\n");

	/*check whether need resume*/
	if (!sinfo->need_suspend)
		return;

	/*check whether already suspend*/
	if (ov8830_event.waked_up == 1) {
		pr_info("[CAM]Ov8830: No nesesary to do Resume\n");
		return;
	}

	mdelay(5);
	/*power down setup*/
	pr_info("[CAM]%s, sensor_pwd 0\n", __func__);
	rc = gpio_request(sinfo->sensor_pwd, "ov8830");
	if (!rc)
		gpio_direction_output(sinfo->sensor_pwd, 0);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", sinfo->sensor_pwd);
	gpio_free(sinfo->sensor_pwd);
	mdelay(5);
	/*reset setup */
	rc = gpio_request(sinfo->sensor_reset, "ov8830");
	if (!rc)
		gpio_direction_output(sinfo->sensor_reset, 1);
	else
		pr_err("[CAM]GPIO (%d) request faile\n", sinfo->sensor_reset);
	gpio_free(sinfo->sensor_reset);

	/*init msm,clk ,GPIO,enable*/
	pr_info("[CAM]%s, msm_camio_probe_on\n", __func__);
	msm_camio_probe_on(ov8830_pdev);
	msm_camio_clk_enable(CAMIO_MDC_CLK);

	/*set MCLK*/
	pr_info("[CAM]%s, msm_camio_clk_rate_set = %d\n",
		__func__, OV8830_DEFAULT_CLOCK_RATE);
	msm_camio_clk_rate_set(OV8830_DEFAULT_CLOCK_RATE);
	msleep(100);

	/*read sensor id*/
	rc = ov8830_probe_read_id(sinfo);
	if (rc < 0)
		pr_err("[CAM]OV8830 resume faile :can not read sensor ID\n");

	/* Initialize Sensor registers */
	rc = initialize_ov8830_registers();
	if (rc < 0)
		return;
	msleep(20);
	/*resume done*/
	ov8830_probe_init_done(sinfo);
	/*turn off MCLK*/
	msm_camio_probe_off(ov8830_pdev);
	msm_camio_clk_disable(CAMIO_MDC_CLK);

	ov8830_event.waked_up = 1;
	pr_info("[CAM]ov8830:resume done\n");
	wake_up(&ov8830_event.event_wait);
	return;
}


static int __exit ov8830_i2c_remove(struct i2c_client *client)
{
	struct ov8830_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	deinit_suspend();
	ov8830_client = NULL;
	kfree(sensorw);
	sensorw = NULL;
	return 0;
}

static struct i2c_driver ov8830_i2c_driver = {
	.id_table = ov8830_i2c_id,
	.probe	= ov8830_i2c_probe,
	.remove = __exit_p(ov8830_i2c_remove),
	.driver = {
		.name = "ov8830",
	},
};


static struct early_suspend early_suspend_ov8830 = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN+1,
	.resume = ov8830_resume,
	.suspend = NULL,
};

static const char *Ov8830Vendor = "OmniVision";
static const char *Ov8830NAME = "ov8830";
static const char *Ov8830Size = "8M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", Ov8830Vendor, Ov8830NAME, Ov8830Size);
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

static struct kobject *android_ov8830;

static int ov8830_sysfs_init(void)
{
	int ret = 0;
	pr_info("[CAM]ov8830:kobject creat and add\n");
	android_ov8830 = kobject_create_and_add("android_camera", NULL);
	if (android_ov8830 == NULL) {
		pr_info("[CAM]ov8830_sysfs_init: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM]ov8830:sysfs_create_file\n");
	ret = sysfs_create_file(android_ov8830, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]ov8830_sysfs_init: sysfs_create_file failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(android_ov8830, &dev_attr_cam_mode.attr);
	if (ret) {
		pr_info("[CAM]ov8830_sysfs_init: dev_attr_cam_mode failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(android_ov8830, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM]ov8830_sysfs_init: dev_attr_node failed\n");
		ret = -EFAULT;
		goto error;
	}

	return ret;

error:
	kobject_del(android_ov8830);
	return ret;
}

#ifdef CONFIG_ARCH_MSM7X30
uint8_t ov8830_preview_skip_frame(void)
{
	if (ov8830_ctrl->sensormode == SENSOR_PREVIEW_MODE && preview_frame_count < 2) {
		preview_frame_count++;
		return 1;
	}
	return 0;
}
#endif

int ov8830_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	down(&ov8830_sem);

	CDBG("[CAM] ov8830_sensor_config: cfgtype = %d\n",
	  cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
				ov8830_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			ov8830_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				ov8830_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				ov8830_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				ov8830_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				ov8830_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = ov8830_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				ov8830_write_exp_gain(
					cdata.cfg.exp_gain.mul,
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				ov8830_set_pict_exp_gain(
					cdata.cfg.exp_gain.mul,
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = ov8830_set_sensor_mode(cdata.mode,
						cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = ov8830_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				ov8830_move_focus(
					cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc = ov8830_set_default_focus();
			break;

		case CFG_SET_EFFECT:
			rc = ov8830_set_default_focus();
			break;

		case CFG_I2C_IOCTL_R_OTP:{
			rc = ov8830_i2c_read_fuseid(&cdata);
			if (copy_to_user
				(argp, &cdata, sizeof(struct sensor_cfg_data))
			    )
			rc = -EFAULT;
			}
			break;

		case CFG_SET_OV_LSC:
			rc = ov8830_update_lsc_table(&cdata);
			break;

		/*20100330 vincent for lsc calibration*/
		case CFG_SET_OV_LSC_RAW_CAPTURE:
			rc = ov8830_LSC_calibration_set_rawflag(&cdata);
			break;

		default:
			rc = -EFAULT;
			break;
		}

	prevent_suspend();
	up(&ov8830_sem);

	return rc;
}


static int ov8830_common_deinit(void)
{
	int rc = -EBADF;

	/*power down setup*/
	if (ov8830_ctrl->sensordata->sensor_pwd >= 0) {
		rc = gpio_request(ov8830_ctrl->sensordata->sensor_pwd, "ov8830");
		if (!rc)
			gpio_direction_output(ov8830_ctrl->sensordata->sensor_pwd, 0);
		else
			pr_err("[CAM]GPIO (%d) request faile\n", ov8830_ctrl->sensordata->sensor_pwd);
		gpio_free(ov8830_ctrl->sensordata->sensor_pwd);

	} else {
      if (ov8830_ctrl->sensordata->camera_pm8058_power != NULL) {
		if (ov8830_ctrl->sensordata->camera_pm8058_power(0) < 0)
		  pr_err("[CAM]camera_pm8058_power(0): request failed\n");
	  }
	}
	mdelay(5);

	if (ov8830_ctrl->sensordata->vcm_pwd) {
		rc = gpio_request(ov8830_ctrl->sensordata->vcm_pwd, "ov8830");
		if (!rc)
			gpio_direction_output(ov8830_ctrl->sensordata->vcm_pwd, 0);
		else
			pr_err("[CAM]GPIO (%d) request faile\n", ov8830_ctrl->sensordata->vcm_pwd);
		gpio_free(ov8830_ctrl->sensordata->vcm_pwd);
	}
	mdelay(1);

	msm_camio_probe_off(ov8830_pdev);

	ov8830_ctrl->sensordata->pdata->camera_gpio_off();

	if (!ov8830_ctrl->sensordata->power_down_disable) {
		rc = ov8830_vreg_disable(ov8830_pdev);
	}

	return rc;
}

static int ov8830_sensor_release(void)
{
	int rc = -EBADF;

	down(&ov8830_sem);
	msleep(35);

	pr_info("[CAM]ov8830_sensor_release");


/*HTC_START Horng 20110905*/
#ifdef CONFIG_MSM_CAMERA_8X60
	msm_mipi_csi_disable();
#endif
/*HTC_END*/


	rc = ov8830_common_deinit();

	msleep(20);

	allow_suspend();
	pr_info("[CAM]ov8830_release completed\n");
	up(&ov8830_sem);

	return rc;
}

static int ov8830_sensor_probe(struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	pr_info("[CAM]ov8830_sensor_probe()\n");

	rc = i2c_add_driver(&ov8830_i2c_driver);

	if (rc < 0 || ov8830_client == NULL) {
		rc = -ENOTSUPP;
		pr_err("[CAM]__ov8830_probe, rc < 0 or ov8830_client == NULL\n");
		return rc;
	}

	pr_info("[CAM]ov8830 s->node %d\n", s->node);
	sensor_probe_node = s->node;
	/*switch pclk and mclk between main cam and 2nd cam*/
	/*only for supersonic*/
	pr_info("[CAM]ov8830: doing clk switch (ov8830)\n");

  rc = ov8830_vreg_enable(ov8830_pdev);
	if (rc < 0) {
		pr_err("[CAM]__ov8830_probe rc < 0\n");
	}

	if (info->camera_clk_switch != NULL)
		info->camera_clk_switch();
	mdelay(5);

	/*power down setup*/
	pr_info("[CAM]%s, GPIO(%d) sensor_pwd 0\n", __func__, info->sensor_pwd);
	if (info->sensor_pwd >= 0) {
		rc = gpio_request(info->sensor_pwd, "ov8830");
		if (!rc)
			gpio_direction_output(info->sensor_pwd, 1);
		else
			pr_err("[CAM]GPIO (%d) request faile\n", info->sensor_pwd);
		gpio_free(info->sensor_pwd);
	} else {
		if (info->camera_pm8058_power != NULL) {
			if (info->camera_pm8058_power(1) < 0)
				goto probe_fail;
		}
	}
	mdelay(5);

	info->pdata->camera_gpio_on();

	/*set MCLK*/
	pr_info("[CAM]%s, msm_camio_clk_rate_set %d\n",
		__func__, OV8830_DEFAULT_CLOCK_RATE);

       msm_camio_clk_rate_set(OV8830_DEFAULT_CLOCK_RATE);

	msleep(100);
	/*read sensor id*/
	rc = ov8830_probe_read_id(info);

  if (rc < 0) {
		goto probe_fail;
  }

	/* Initialize Sensor registers */
	rc = initialize_ov8830_registers();
	if (rc < 0)
		return rc;

/*
	if (info->camera_main_set_probe != NULL)
		info->camera_main_set_probe(true);
*/

#if 1 /* Get OTP fuse id for OV sensor*/
ov8830_i2c_write_b(ov8830_client->addr, 0x3d84, 0xc0);
ov8830_i2c_write_b(ov8830_client->addr, 0x3d81, 0x01);
ov8830_i2c_read(0x3d00, &fuse_id[0], 1);
ov8830_i2c_read(0x3d01, &fuse_id[1], 1);
ov8830_i2c_read(0x3d02, &fuse_id[2], 1);
ov8830_i2c_read(0x3d03, &fuse_id[3], 1);
ov8830_i2c_read(0x3d04, &fuse_id[4], 1);
ov8830_i2c_read(0x3d05, &fuse_id[5], 1);
ov8830_i2c_read(0x3d06, &fuse_id[6], 1);
ov8830_i2c_read(0x3d07, &fuse_id[7], 1);
ov8830_i2c_read(0x3d08, &fuse_id[8], 1);
ov8830_i2c_read(0x3d09, &fuse_id[9], 1);
ov8830_i2c_read(0x3d0A, &fuse_id[10], 1);
ov8830_i2c_read(0x3d0B, &fuse_id[11], 1);
ov8830_i2c_read(0x3d0C, &fuse_id[12], 1);
ov8830_i2c_read(0x3d0D, &fuse_id[13], 1);
ov8830_i2c_read(0x3d0E, &fuse_id[14], 1);
ov8830_i2c_read(0x3d0F, &fuse_id[15], 1);
#endif

	init_suspend();
	s->s_init = ov8830_sensor_open_init;
	s->s_release = ov8830_sensor_release;
	s->s_config  = ov8830_sensor_config;

#ifdef CONFIG_ARCH_MSM7X30
	info->preview_skip_frame = ov8830_preview_skip_frame;
#endif

	msleep(20);
	ov8830_probe_init_done(info);
	/*register late resuem*/
	register_early_suspend(&early_suspend_ov8830);
	/*init wait event*/
	init_waitqueue_head(&ov8830_event.event_wait);
	/*init waked_up value*/
	ov8830_event.waked_up = 1;
	/*write sysfs*/
	ov8830_sysfs_init();
	pr_info("[CAM]%s: ov8830_probe_init_done %d\n",  __func__, __LINE__);
	goto probe_done;

probe_fail:
	pr_err("[CAM]SENSOR PROBE FAILS!\n");
	ov8830_probe_init_done(info);
probe_done:
	return rc;

}

static int __ov8830_probe(struct platform_device *pdev)
{
/*
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
*/
	pr_info("[CAM]__ov8830_probe\n");
	ov8830_pdev = pdev;

/*
	if (sdata->camera_main_get_probe != NULL) {
		if (sdata->camera_main_get_probe()) {
			pr_info("[CAM]__ov8830_probe camera main get probed already.\n");
			return 0;
		}
	}
*/

	return msm_camera_drv_start(pdev, ov8830_sensor_probe);  /*msm_camera_drv_start(pdev, ov8830_sensor_probe)*/
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov8830_probe,
	.driver = {
		.name = "msm_camera_ov8830",
		.owner = THIS_MODULE,
	},
	.suspend = ov8830_suspend,
};

static int __init ov8830_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov8830_init);
