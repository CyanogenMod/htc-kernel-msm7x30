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
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include "s5k3h1gx.h"

/* CAMIF output resolutions */
/* 816x612, 24MHz MCLK 96MHz PCLK */
#define SENSOR_FULL_SIZE_WIDTH 3280
#define SENSOR_FULL_SIZE_HEIGHT 2464

#define SENSOR_QTR_SIZE_WIDTH 1640
#define SENSOR_QTR_SIZE_HEIGHT 1232

#define SENSOR_HRZ_FULL_BLK_PIXELS 190
#define SENSOR_VER_FULL_BLK_LINES 16 /* 0 */
#define SENSOR_HRZ_QTR_BLK_PIXELS 1830
#define SENSOR_VER_QTR_BLK_LINES 16 /* 8 */
#define SENSOR_VER_QTR_BLK_LINES_PARALLEL 611 /* 16 */ /* 8 */

#define S5K3H1GX_AF_I2C_ADDR 0x18
#define S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR 42 /* 36 */
#define S5K3H1GX_SW_DAMPING_STEP 10
#define S5K3H1GX_MAX_FPS 30
#define S5K3H1GX_MAX_FPS_PARALLEL 30 /* 22 */

/*=============================================================
 SENSOR REGISTER DEFINES
==============================================================*/

#define S5K3H1GX_REG_MODEL_ID 0x0000
#define S5K3H1GX_MODEL_ID 0x3810

/* Color bar pattern selection */
#define S5K3H1GX_COLOR_BAR_PATTERN_SEL_REG 0x0601

#define REG_LINE_LENGTH_PCK_MSB 0x0342
#define REG_LINE_LENGTH_PCK_LSB 0x0343
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB 0x0204
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB 0x0205
#define REG_COARSE_INTEGRATION_TIME_MSB 0x0202
#define REG_COARSE_INTEGRATION_TIME_LSB 0x0203

#define S5K3H1GX_REG_GROUP_PARAMETER_HOLD 0x0104
#define S5K3H1GX_GROUP_PARAMETER_HOLD 0x01
#define S5K3H1GX_GROUP_PARAMETER_UNHOLD 0x00

////////////////////////////

#define Q8 0x00000100
#define SENSOR_DEFAULT_CLOCK_RATE 24000000

////////////////////////////////////////////////////////////

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

enum s5k3h1gx_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum s5k3h1gx_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum s5k3h1gx_reg_update_t{
	REG_INIT,
	REG_PERIODIC
};

static int sensor_probe_node = 0;
static int preview_frame_count = 0;

static struct wake_lock s5k3h1gx_wake_lock;

static inline void init_suspend(void)
{
	wake_lock_init(&s5k3h1gx_wake_lock, WAKE_LOCK_IDLE, "s5k3h1gx");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&s5k3h1gx_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&s5k3h1gx_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&s5k3h1gx_wake_lock);
}

/*============================================================================
DATA DECLARATIONS
============================================================================*/

/*  96MHz PCLK @ 24MHz MCLK inc*/


/* FIXME: Changes from here */
struct s5k3h1gx_work {
  struct work_struct work;
};

static struct  s5k3h1gx_work *s5k3h1gx_sensorw;
static struct  i2c_client *s5k3h1gx_client;
static uint16_t s5k3h1gx_pos_tbl[S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR + 1];

struct s5k3h1gx_ctrl {
  const struct  msm_camera_sensor_info *sensordata;

  uint32_t sensormode;
  uint32_t fps_divider; /* init to 1 * 0x00000400 */
  uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */
  uint16_t fps;

  int16_t  curr_lens_pos;
  uint16_t curr_step_pos;
  uint16_t init_curr_lens_pos;
  uint16_t my_reg_gain;
  uint32_t my_reg_line_count;
  uint16_t total_lines_per_frame;

  enum s5k3h1gx_resolution_t prev_res;
  enum s5k3h1gx_resolution_t pict_res;
  enum s5k3h1gx_resolution_t curr_res;
  enum s5k3h1gx_test_mode_t set_test;
  enum s5k3h1gx_reg_update_t reg_update;

  unsigned short imgaddr;
};


static struct s5k3h1gx_ctrl *s5k3h1gx_ctrl;
static struct platform_device *s5k3h1gx_pdev;

struct s5k3h1gx_waitevent{
	uint32_t waked_up;
	wait_queue_head_t event_wait;
};

static DECLARE_WAIT_QUEUE_HEAD(s5k3h1gx_wait_queue);
DECLARE_MUTEX(s5k3h1gx_sem);


/*=============================================================*/

static int s5k3h1gx_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(s5k3h1gx_client->adapter, msgs, 2) < 0) {
		pr_err("s5k3h1gx_i2c_rxdata failed!\n");
		return -EIO;
	}
	pr_info("%s: rxdata=0x%X\n", __func__, *rxdata);

	return 0;
}
static int32_t s5k3h1gx_i2c_txdata(unsigned short saddr,
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
	if (i2c_transfer(s5k3h1gx_client->adapter, msg, 1) < 0) {
		pr_err("s5k3h1gx_i2c_txdata failed 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t s5k3h1gx_i2c_read_b(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k3h1gx_i2c_rxdata(saddr, buf, 1);
	if (rc < 0)
		return rc;

	*rdata = buf[0];

	if (rc < 0)
		pr_info("s5k3h1gx_i2c_read failed!\n");

	return rc;
}


static int32_t s5k3h1gx_i2c_read(unsigned short raddr,
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
	rc = s5k3h1gx_i2c_rxdata(s5k3h1gx_client->addr, buf, rlen);

	if (rc < 0) {
		pr_err("s5k3h1gx_i2c_read 0x%x failed!\n", raddr);
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


static int32_t s5k3h1gx_i2c_write_b(unsigned short saddr,
  unsigned short waddr, uint8_t bdata)
{
  int32_t rc = -EFAULT;
  unsigned char buf[3];
  int count = 0;

  memset(buf, 0, sizeof(buf));
  buf[0] = (waddr & 0xFF00) >> 8;
  buf[1] = (waddr & 0x00FF);
  buf[2] = bdata;

retry:
  rc = s5k3h1gx_i2c_txdata(saddr, buf, 3);

  if (rc < 0) {
    pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x\n",
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

static void s5k3h1gx_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;
	uint16_t snapshot_height, preview_height, preview_width, snapshot_width;
	struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

	if (s5k3h1gx_ctrl->prev_res == QTR_SIZE) {
		preview_width =
			SENSOR_QTR_SIZE_WIDTH  + SENSOR_HRZ_QTR_BLK_PIXELS;

		if (sinfo->csi_if)
			preview_height =
				SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES;
		else
			preview_height =
				SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES_PARALLEL;
	} else {
		/* full size resolution used for preview. */
		preview_width =
			SENSOR_FULL_SIZE_WIDTH + SENSOR_HRZ_FULL_BLK_PIXELS;
		preview_height =
			SENSOR_FULL_SIZE_HEIGHT + SENSOR_VER_FULL_BLK_LINES;
	}

	if (s5k3h1gx_ctrl->pict_res == QTR_SIZE) {
		snapshot_width =
			SENSOR_QTR_SIZE_WIDTH + SENSOR_HRZ_QTR_BLK_PIXELS;

		if (sinfo->csi_if)
			snapshot_height =
				SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES;
		else
			snapshot_height =
				SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES_PARALLEL;
	} else {
		snapshot_width =
			SENSOR_FULL_SIZE_WIDTH + SENSOR_HRZ_FULL_BLK_PIXELS;
		snapshot_height =
			SENSOR_FULL_SIZE_HEIGHT + SENSOR_VER_FULL_BLK_LINES;
	}

	d1 = preview_height * 0x00000400 / snapshot_height;
	d2 = preview_width * 0x00000400 / snapshot_width;

	divider = (uint32_t) (d1 * d2) / 0x00000400;
	*pfps = (uint16_t)(fps * divider / 0x00000400);

} /* endof s5k3h1gx_get_pict_fps */

static uint16_t s5k3h1gx_get_prev_lines_pf(void)
{
	struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

	if (s5k3h1gx_ctrl->prev_res == QTR_SIZE) {
		if (sinfo->csi_if)
			return (SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES);
		else
			return (SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES_PARALLEL);
	} else  {
		return (SENSOR_FULL_SIZE_HEIGHT + SENSOR_VER_FULL_BLK_LINES);
	}
}

static uint16_t s5k3h1gx_get_prev_pixels_pl(void)
{
	if (s5k3h1gx_ctrl->prev_res == QTR_SIZE) {
		return (SENSOR_QTR_SIZE_WIDTH + SENSOR_HRZ_QTR_BLK_PIXELS);
	} else  {
		return (SENSOR_FULL_SIZE_WIDTH + SENSOR_HRZ_FULL_BLK_PIXELS);
}
}

static uint16_t s5k3h1gx_get_pict_lines_pf(void)
{
	struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

	if (s5k3h1gx_ctrl->pict_res == QTR_SIZE) {
		if (sinfo->csi_if)
			return (SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES);
		else
			return (SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES_PARALLEL);
	} else  {
		return (SENSOR_FULL_SIZE_HEIGHT + SENSOR_VER_FULL_BLK_LINES);
	}
}

static uint16_t s5k3h1gx_get_pict_pixels_pl(void)
{
	if (s5k3h1gx_ctrl->pict_res == QTR_SIZE) {
		return (SENSOR_QTR_SIZE_WIDTH + SENSOR_HRZ_QTR_BLK_PIXELS);
	} else  {
		return (SENSOR_FULL_SIZE_WIDTH + SENSOR_HRZ_FULL_BLK_PIXELS);
	}
}

static uint32_t s5k3h1gx_get_pict_max_exp_lc(void)
{
	struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

	if (s5k3h1gx_ctrl->pict_res == QTR_SIZE) {
		if (sinfo->csi_if)
			return (SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES);
		else
			return (SENSOR_QTR_SIZE_HEIGHT + SENSOR_VER_QTR_BLK_LINES_PARALLEL);
	} else  {
		return (SENSOR_FULL_SIZE_HEIGHT + SENSOR_VER_FULL_BLK_LINES);
	}
}

static int32_t s5k3h1gx_set_fps(struct fps_cfg *fps)
{
	int32_t rc = 0;
	s5k3h1gx_ctrl->fps_divider = fps->fps_div;
	s5k3h1gx_ctrl->pict_fps_divider = fps->pict_fps_div;
	s5k3h1gx_ctrl->fps = fps->f_mult;
	return rc;
}

static int32_t s5k3h1gx_i2c_write_table(
	struct s5k3h1gx_i2c_reg_conf *reg_cfg_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr,
			reg_cfg_tbl->waddr, reg_cfg_tbl->bdata);
		if (rc < 0)
			break;
		reg_cfg_tbl++;
	}

	return rc;
}

static int32_t s5k3h1gx_write_exp_gain
  (uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	uint16_t max_legal_gain = 0x0200;
	uint32_t ll_ratio; /* Q10 */
	uint32_t ll_pck, fl_lines;
	uint16_t offset = 8; /* 4; */     /* kipper */
	uint32_t gain_msb, gain_lsb;
	uint32_t intg_t_msb, intg_t_lsb;
	uint32_t ll_pck_msb, ll_pck_lsb;
	struct s5k3h1gx_i2c_reg_conf tbl[3];
	struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

	CDBG("Line:%d s5k3h1gx_write_exp_gain \n", __LINE__);

	if (s5k3h1gx_ctrl->sensormode == SENSOR_PREVIEW_MODE) {

		s5k3h1gx_ctrl->my_reg_gain = gain;
		s5k3h1gx_ctrl->my_reg_line_count = (uint16_t)line;

		if (sinfo->csi_if)
			fl_lines = SENSOR_QTR_SIZE_HEIGHT +
				SENSOR_VER_QTR_BLK_LINES;
		else
			fl_lines = SENSOR_QTR_SIZE_HEIGHT +
				SENSOR_VER_QTR_BLK_LINES_PARALLEL;

		ll_pck = SENSOR_QTR_SIZE_WIDTH +
			SENSOR_HRZ_QTR_BLK_PIXELS;

	} else {

		fl_lines = SENSOR_FULL_SIZE_HEIGHT +
			SENSOR_VER_FULL_BLK_LINES;

		ll_pck = SENSOR_FULL_SIZE_WIDTH +
			SENSOR_HRZ_FULL_BLK_PIXELS;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	/* in Q10 */
	line = (line * s5k3h1gx_ctrl->fps_divider);

	pr_info("s5k3h1gx_ctrl->fps_divider = %d\n", s5k3h1gx_ctrl->fps_divider);
	pr_info("fl_lines = %d\n", fl_lines);
	pr_info("line = %d\n", line);
	if ((fl_lines-offset) < (line / 0x400))    /* kipper, 3H1 maximum coarse_integration_time = frame length lines -8 */
		ll_ratio = (line / (fl_lines - offset));
	else
		ll_ratio = 0x400;
	pr_info("ll_ratio = %d\n", ll_ratio);

	/* update gain registers */
	pr_info("gain = %d\n", gain);
	gain_msb = (gain & 0xFF00) >> 8;
	gain_lsb = gain & 0x00FF;
	tbl[0].waddr = S5K3H1GX_REG_GROUP_PARAMETER_HOLD;
	tbl[0].bdata = S5K3H1GX_GROUP_PARAMETER_HOLD;
	tbl[1].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB;
	tbl[1].bdata = gain_msb;
	tbl[2].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB;
	tbl[2].bdata = gain_lsb;
	rc = s5k3h1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));
	if (rc < 0)
		goto write_gain_done;

	ll_pck = ll_pck * ll_ratio;
	pr_info("ll_pck/0x400 = %d\n", ll_pck / 0x400);
	ll_pck_msb = ((ll_pck / 0x400) & 0xFF00) >> 8;
	ll_pck_lsb = (ll_pck / 0x400) & 0x00FF;
	tbl[0].waddr = REG_LINE_LENGTH_PCK_MSB;
	tbl[0].bdata = ll_pck_msb;
	tbl[1].waddr = REG_LINE_LENGTH_PCK_LSB;
	tbl[1].bdata = ll_pck_lsb;
	rc = s5k3h1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl)-1);
	if (rc < 0)
		goto write_gain_done;

	line = line / ll_ratio;
	pr_info("line = %d\n", line);
	intg_t_msb = (line & 0xFF00) >> 8;
	intg_t_lsb = (line & 0x00FF);
	tbl[0].waddr = REG_COARSE_INTEGRATION_TIME_MSB;
	tbl[0].bdata = intg_t_msb;
	tbl[1].waddr = REG_COARSE_INTEGRATION_TIME_LSB;
	tbl[1].bdata = intg_t_lsb;
	tbl[2].waddr = S5K3H1GX_REG_GROUP_PARAMETER_HOLD;
	tbl[2].bdata = S5K3H1GX_GROUP_PARAMETER_UNHOLD;
	rc = s5k3h1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));

write_gain_done:
	return rc;
}

/* ### this function is not called for userspace ### */
static int32_t s5k3h1gx_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = s5k3h1gx_write_exp_gain(gain, line);

	return rc;
} /* endof s5k3h1gx_set_pict_exp_gain*/

static int32_t initialize_s5k3h1gx_registers(void)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

	mdelay(5);

	if (sinfo->csi_if) {
		if (s5k3h1gx_regs.init_mipi_size > 0)
			rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.init_mipi, s5k3h1gx_regs.init_mipi_size);
	} else {
		if (s5k3h1gx_regs.init_parallel_size > 0)
			rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.init_parallel, s5k3h1gx_regs.init_parallel_size);
	}

	return rc;
} /* end of initialize_s5k3h1gx_ov8m0vc_registers. */

static int32_t s5k3h1gx_setting(int rt)
{
	int32_t rc = 0;
	struct msm_camera_csi_params s5k3h1gx_csi_params;
	struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;


	if (sinfo->csi_if) {
		if (s5k3h1gx_ctrl->reg_update == REG_INIT) {
			/* config mipi csi controller */
			s5k3h1gx_csi_params.data_format = CSI_10BIT;
			s5k3h1gx_csi_params.lane_cnt = 2;
			s5k3h1gx_csi_params.lane_assign = 0xe4;
			s5k3h1gx_csi_params.dpcm_scheme = 0;
			s5k3h1gx_csi_params.settle_cnt = 20;
			rc = msm_camio_csi_config(&s5k3h1gx_csi_params);

			s5k3h1gx_ctrl->reg_update = REG_PERIODIC;
		}
	}

    if (sinfo->csi_if) {
      rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.common_mipi, s5k3h1gx_regs.common_mipi_size);
    } else {
      rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.common_parallel, s5k3h1gx_regs.common_parallel_size);
    }
    if (rc < 0)
      return rc;

	switch (rt) {
	case QTR_SIZE:
		pr_err("s5k3h1gx_setting(QTR_SIZE)\n");
		if (sinfo->csi_if) {
			rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.qtr_mipi, s5k3h1gx_regs.qtr_mipi_size);
		} else {
			rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.qtr_parallel, s5k3h1gx_regs.qtr_parallel_size);
			if (rc < 0)
				return rc;

			msleep(200);
			rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr, 0x0100, 0x01);
		}
		if (rc < 0)
			return rc;

		s5k3h1gx_ctrl->curr_res = QTR_SIZE;

		/* test pattern */
#if 0
		rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr, S5K3H1GX_COLOR_BAR_PATTERN_SEL_REG, 0x02);
		if (rc < 0)
			return rc;
#endif
		break;

	case FULL_SIZE:
		pr_err("s5k3h1gx_setting(FULL_SIZE)\n");
		if (sinfo->csi_if) {
			rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.full_mipi, s5k3h1gx_regs.full_mipi_size);
		} else {
			rc = s5k3h1gx_i2c_write_table(s5k3h1gx_regs.full_parallel, s5k3h1gx_regs.full_parallel_size);
			if (rc < 0)
				return rc;

			msleep(100);
			rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr, 0x0100, 0x01);
		}
		if (rc < 0)
			return rc;

		/* test pattern */
#if 0
		rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr, S5K3H1GX_COLOR_BAR_PATTERN_SEL_REG, 0x02);
		if (rc < 0)
			return rc;
#endif

		s5k3h1gx_ctrl->curr_res = FULL_SIZE;
		break;

	default:
		rc = -EFAULT;
		return rc;
	}

	return rc;
} /* end of s5k3h1gx_setting */

static int32_t s5k3h1gx_video_config(int mode)
{
	int32_t rc = 0;
	s5k3h1gx_ctrl->sensormode = mode;
	if (s5k3h1gx_ctrl->curr_res != s5k3h1gx_ctrl->prev_res) {
	rc = s5k3h1gx_setting(s5k3h1gx_ctrl->prev_res);
		if (rc < 0)
			return rc;
	} else {
		s5k3h1gx_ctrl->curr_res = s5k3h1gx_ctrl->prev_res;
	}
		s5k3h1gx_ctrl->sensormode = mode;

	preview_frame_count = 0;
	rc =
		s5k3h1gx_write_exp_gain(s5k3h1gx_ctrl->my_reg_gain,
			s5k3h1gx_ctrl->my_reg_line_count);

	return rc;

} /* end of s5k3h1gx_video_config */

static int32_t s5k3h1gx_snapshot_config(int mode)
{
	int32_t rc = 0;
	s5k3h1gx_ctrl->sensormode = mode;

	if (s5k3h1gx_ctrl->curr_res != s5k3h1gx_ctrl->pict_res) {
		rc = s5k3h1gx_setting(s5k3h1gx_ctrl->pict_res);
		if (rc < 0)
			return rc;
	} else {
		s5k3h1gx_ctrl->curr_res = s5k3h1gx_ctrl->pict_res;
	}
	s5k3h1gx_ctrl->sensormode = mode;

	return rc;

} /*end of s5k3h1gx_snapshot_config*/

static int32_t s5k3h1gx_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	s5k3h1gx_ctrl->sensormode = mode;
	if (s5k3h1gx_ctrl->curr_res != s5k3h1gx_ctrl->pict_res) {
		rc = s5k3h1gx_setting(s5k3h1gx_ctrl->pict_res);
		if (rc < 0)
			return rc;
	} else {
		s5k3h1gx_ctrl->curr_res = s5k3h1gx_ctrl->pict_res;
	} /* Update sensor resolution */

	s5k3h1gx_ctrl->sensormode = mode;

	return rc;

} /*end of s5k3h1gx_raw_snapshot_config*/

static int32_t s5k3h1gx_set_sensor_mode(int mode,
  int res)
{
  int32_t rc = 0;

  switch (mode) {
    case SENSOR_PREVIEW_MODE:
      rc = s5k3h1gx_video_config(mode);
      break;

    case SENSOR_SNAPSHOT_MODE:
      rc = s5k3h1gx_snapshot_config(mode);
      break;

    case SENSOR_RAW_SNAPSHOT_MODE:
      rc = s5k3h1gx_raw_snapshot_config(mode);
      break;

    default:
      rc = -EINVAL;
      break;
  }

  return rc;
}

static int s5k3h1gx_vreg_enable(struct platform_device *pdev)
{
  struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
  int rc;
  pr_info("%s camera vreg on\n", __func__);

  if (sdata->camera_power_on == NULL) {
    pr_err("sensor platform_data didnt register\n");
    return -EIO;
  }
  rc = sdata->camera_power_on();
  return rc;
}

static int s5k3h1gx_vreg_disable(struct platform_device *pdev)
{
  struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
  int rc;
  pr_info("%s camera vreg off\n", __func__);

  if (sdata->camera_power_off == NULL) {
    pr_err("sensor platform_data didnt register\n");
    return -EIO;
  }
  rc = sdata->camera_power_off();
  return rc;
}

static int s5k3h1gx_probe_init_done(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	pr_info("[Camera] gpio_request(%d, \"s5k3h1gx\")\n", data->sensor_pwd);
	rc = gpio_request(data->sensor_pwd, "s5k3h1gx");
	if (!rc) {
		gpio_direction_output(data->sensor_pwd, 0);
	} else {
		pr_err("GPIO (%d) request failed\n", data->sensor_pwd);
	}
	gpio_free(data->sensor_pwd);

	if (data->vcm_pwd) {
		msleep(5);
		rc = gpio_request(data->vcm_pwd, "s5k3h1gx");
		if (!rc)
			gpio_direction_output(data->vcm_pwd, 0);
		else
			pr_err("GPIO (%d) request faile\n", data->vcm_pwd);
		gpio_free(data->vcm_pwd);
	}

	s5k3h1gx_vreg_disable(s5k3h1gx_pdev);
	return 0;
}

static int32_t s5k3h1gx_power_down(void)
{
  return 0;
}

static int s5k3h1gx_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;

	pr_info("%s\n", __func__);
	pr_info("[Camera] gpio_request(%d, \"s5k3h1gx\")\n", data->sensor_pwd);
	rc = gpio_request(data->sensor_pwd, "s5k3h1gx");
	if (!rc) {
		gpio_direction_output(data->sensor_pwd, 1);
	} else {
		pr_err("GPIO (%d) request failed\n", data->sensor_pwd);
		goto init_probe_fail;
	}
	gpio_free(data->sensor_pwd);

	if (data->vcm_pwd) {
		msleep(5);
		rc = gpio_request(data->vcm_pwd, "s5k3h1gx");
		if (!rc)
			gpio_direction_output(data->vcm_pwd, 1);
		else
			pr_err("GPIO (%d) request faile\n", data->vcm_pwd);
		gpio_free(data->vcm_pwd);

		msleep(1);
	}

	/* Read sensor Model ID: */
	rc = s5k3h1gx_i2c_read(S5K3H1GX_REG_MODEL_ID, &chipid, 2);
	if (rc < 0) {
		pr_err("read sensor id fail\n");
		goto init_probe_fail;
	}

	/* Compare sensor ID to S5K3H1GX ID: */
	pr_info("%s, Expected id=0x%x\n", __func__, S5K3H1GX_MODEL_ID);
	pr_info("%s, Read id=0x%x\n", __func__, chipid);

	if (chipid != S5K3H1GX_MODEL_ID) {
		pr_err("sensor model id is incorrect\n");
		rc = -ENODEV;
		goto init_probe_fail;
	}

	pr_info("s5k3h1gx_probe_init_sensor finishes\n");
	goto init_probe_done;

init_probe_fail:
	s5k3h1gx_probe_init_done(data);
init_probe_done:
	return rc;
}

static void s5k3h1gx_setup_af_tbl(void)
{
  int i;
  uint16_t s5k3h1gx_nl_region_boundary1 = 3;
  uint16_t s5k3h1gx_nl_region_boundary2 = 5;
  uint16_t s5k3h1gx_nl_region_code_per_step1 = 40;
  uint16_t s5k3h1gx_nl_region_code_per_step2 = 20;
  uint16_t s5k3h1gx_l_region_code_per_step = 16;

  s5k3h1gx_pos_tbl[0] = 0;

  for (i = 1; i <= S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR; i++) {
    if (i <= s5k3h1gx_nl_region_boundary1)
      s5k3h1gx_pos_tbl[i] = s5k3h1gx_pos_tbl[i-1] +
      s5k3h1gx_nl_region_code_per_step1;
    else if (i <= s5k3h1gx_nl_region_boundary2)
      s5k3h1gx_pos_tbl[i] = s5k3h1gx_pos_tbl[i-1] +
      s5k3h1gx_nl_region_code_per_step2;
    else
      s5k3h1gx_pos_tbl[i] = s5k3h1gx_pos_tbl[i-1] +
      s5k3h1gx_l_region_code_per_step;
  }
}

static int32_t
s5k3h1gx_go_to_position(uint32_t lens_pos, uint8_t mask)
{
  int32_t rc = 0;
  unsigned char buf[2];
  uint8_t code_val_msb, code_val_lsb;

  code_val_msb = lens_pos >> 4;
  code_val_lsb = (lens_pos & 0x000F) << 4;
  code_val_lsb |= mask;

  buf[0] = code_val_msb;
  buf[1] = code_val_lsb;
  rc = s5k3h1gx_i2c_txdata(S5K3H1GX_AF_I2C_ADDR >> 1, buf, 2);
  if (rc < 0)
    pr_err("i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n",
      S5K3H1GX_AF_I2C_ADDR >> 1, buf[0], buf[1]);

  return rc;
}

static int s5k3h1gx_i2c_read_fuseid(struct sensor_cfg_data *cdata)
{

	int32_t  rc;
	unsigned short i, R1, R2, R3;
	unsigned short  OTP[10];

	pr_info("%s: sensor OTP information:\n", __func__);

	for (i = 0; i < 10; i++)
		OTP[i] = 5;

	rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr, 0x3124, 0x10);
	if (rc < 0)
		pr_info("%s: i2c_write_b 0x30FB fail\n", __func__);

	rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr, 0x3127, 0xF1);
	if (rc < 0)
		pr_info("%s: i2c_write_b 0x30FB fail\n", __func__);

	mdelay(4);

	for (i = 0; i < 10; i++) {
		rc = s5k3h1gx_i2c_write_b(s5k3h1gx_client->addr, 0x312B, i);
			if (rc < 0)
			pr_info("%s: i2c_write_b 0x310C fail\n", __func__);
		rc = s5k3h1gx_i2c_read_b(s5k3h1gx_client->addr, 0x312C, &R1);
			if (rc < 0)
			pr_info("%s: i2c_read_b 0x310F fail\n", __func__);
		rc = s5k3h1gx_i2c_read_b(s5k3h1gx_client->addr, 0x312D, &R2);
			if (rc < 0)
			pr_info("%s: i2c_read_b 0x310E fail\n", __func__);
		rc = s5k3h1gx_i2c_read_b(s5k3h1gx_client->addr, 0x312E, &R3);
			if (rc < 0)
			pr_info("%s: i2c_read_b 0x310D fail\n", __func__);

		if ((R3&0x0F) != 0)
			OTP[i] = (short)(R3&0x0F);
		else if ((R2&0x0F) != 0)
			OTP[i] = (short)(R2&0x0F);
		else if ((R2>>4) != 0)
			OTP[i] = (short)(R2>>4);
		else if ((R1&0x0F) != 0)
			OTP[i] = (short)(R1&0x0F);
		else
			OTP[i] = (short)(R1>>4);

	}
	pr_info("%s: VenderID=%x,LensID=%x,SensorID=%x%x\n", __func__,
		OTP[0], OTP[1], OTP[2], OTP[3]);
	pr_info("%s: ModuleFuseID= %x%x%x%x%x%x\n", __func__,
		OTP[4], OTP[5], OTP[6], OTP[7], OTP[8], OTP[9]);

    cdata->cfg.fuse.fuse_id_word1 = 0;
    cdata->cfg.fuse.fuse_id_word2 = 0;
	cdata->cfg.fuse.fuse_id_word3 = (OTP[0]);
	cdata->cfg.fuse.fuse_id_word4 =
		(OTP[4]<<20) |
		(OTP[5]<<16) |
		(OTP[6]<<12) |
		(OTP[7]<<8) |
		(OTP[8]<<4) |
		(OTP[9]);

	pr_info("s5k3h1gx: fuse->fuse_id_word1:%d\n",
		cdata->cfg.fuse.fuse_id_word1);
	pr_info("s5k3h1gx: fuse->fuse_id_word2:%d\n",
		cdata->cfg.fuse.fuse_id_word2);
	pr_info("s5k3h1gx: fuse->fuse_id_word3:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word3);
	pr_info("s5k3h1gx: fuse->fuse_id_word4:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word4);
	return 0;
}

static int s5k3h1gx_sensor_open_init(struct msm_camera_sensor_info *data)
{
  int32_t rc = 0;
  struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

  pr_info("Calling s5k3h1gx_sensor_open_init\n");

  down(&s5k3h1gx_sem);

  if (data == NULL) {
    pr_info("data is a NULL pointer\n");
    goto init_fail;
  }

  s5k3h1gx_ctrl = kzalloc(sizeof(struct s5k3h1gx_ctrl), GFP_KERNEL);

  if (!s5k3h1gx_ctrl) {
    rc = -ENOMEM;
    goto init_fail;
  }

  s5k3h1gx_ctrl->curr_lens_pos = -1;
  s5k3h1gx_ctrl->fps_divider = 1 * 0x00000400;
  s5k3h1gx_ctrl->pict_fps_divider = 1 * 0x00000400;
  s5k3h1gx_ctrl->set_test = TEST_OFF;
  s5k3h1gx_ctrl->prev_res = QTR_SIZE;
  s5k3h1gx_ctrl->pict_res = FULL_SIZE;
  s5k3h1gx_ctrl->curr_res = INVALID_SIZE;
  s5k3h1gx_ctrl->reg_update = REG_INIT;

  if (data)
    s5k3h1gx_ctrl->sensordata = data;

  /* switch pclk and mclk between main cam and 2nd cam */
  pr_info("doing clk switch (s5k3h1gx)\n");

  if(data->camera_clk_switch != NULL)
    data->camera_clk_switch();

  s5k3h1gx_vreg_enable(s5k3h1gx_pdev);

  msm_camio_probe_on(s5k3h1gx_pdev);

  /* for parallel interface */
  if (!sinfo->csi_if) {
    mdelay(20);
    msm_camio_camif_pad_reg_reset();
    mdelay(20);
  }

  /* read sensor id */
  rc = s5k3h1gx_probe_init_sensor(data);

  if (rc < 0)
    goto init_fail;

  /* set up lens position table */
  s5k3h1gx_setup_af_tbl();
  s5k3h1gx_go_to_position(0, 0);
  s5k3h1gx_ctrl->curr_lens_pos = 0;
  s5k3h1gx_ctrl->curr_step_pos = 0;

  goto init_done;

init_fail:
  pr_err("%s: init_fail\n", __func__);

init_done:
  up(&s5k3h1gx_sem);
  pr_info("%s: init_done\n", __func__);
  return rc;

} /* end of s5k3h1gx_sensor_open_init */

static int s5k3h1gx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k3h1gx_wait_queue);
	return 0;
}

static const struct i2c_device_id s5k3h1gx_i2c_id[] = {
  { "s5k3h1gx", 0},
  { }
};

static int s5k3h1gx_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_info("s5k3h1gx_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	s5k3h1gx_sensorw = kzalloc(sizeof(struct s5k3h1gx_work), GFP_KERNEL);
	if (!s5k3h1gx_sensorw) {
		pr_err("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k3h1gx_sensorw);
	s5k3h1gx_init_client(client);
	s5k3h1gx_client = client;

	msleep(50);

	pr_info("s5k3h1gx_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("s5k3h1gx_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit s5k3h1gx_i2c_remove(struct i2c_client *client)
{
	struct s5k3h1gx_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	deinit_suspend();
	s5k3h1gx_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver s5k3h1gx_i2c_driver = {
  .id_table = s5k3h1gx_i2c_id,
  .probe	= s5k3h1gx_i2c_probe,
  .remove = __exit_p(s5k3h1gx_i2c_remove),
  .driver = {
    .name = "s5k3h1gx",
  },
};

static const char *S5K3H1GXVendor = "samsung";
static const char *S5K3H1GXNAME = "S5K3H1GX";
static const char *S5K3H1GXSize = "8M";

static ssize_t sensor_vendor_show( struct device *dev,
  struct device_attribute *attr, char *buf)
{
  ssize_t ret = 0;

  sprintf(buf, "%s %s %s\n", S5K3H1GXVendor, S5K3H1GXNAME, S5K3H1GXSize);
  ret = strlen(buf) + 1;

  return ret;
}

static ssize_t sensor_read_node( struct device *dev,
  struct device_attribute *attr, char *buf)
{
  ssize_t length;
  length = sprintf(buf, "%d\n", sensor_probe_node);
  return length;
}

static DEVICE_ATTR(sensor, 0444, sensor_vendor_show, NULL);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);

static struct kobject *android_s5k3h1gx = NULL;

static int s5k3h1gx_sysfs_init(void)
{
  int ret = 0;
  pr_info("s5k3h1gx:kobject creat and add\n");
  android_s5k3h1gx = kobject_create_and_add("android_camera", NULL);
  if (android_s5k3h1gx == NULL) {
    pr_info("s5k3h1gx_sysfs_init: subsystem_register failed\n");
    ret = -ENOMEM;
    return ret ;
  }
  pr_info("s5k3h1gx:sysfs_create_file\n");
  ret = sysfs_create_file(android_s5k3h1gx, &dev_attr_sensor.attr);
  if (ret) {
    pr_info("s5k3h1gx_sysfs_init: sysfs_create_file failed\n");
    ret = -EFAULT;
    goto error;
  }

  ret = sysfs_create_file(android_s5k3h1gx, &dev_attr_node.attr);
  if (ret) {
    pr_info("s5k3h1gx_sysfs_init: dev_attr_node failed\n");
    ret = -EFAULT;
    goto error;
  }

  return ret;

error:
  kobject_del(android_s5k3h1gx);
  return ret;
}

static int32_t
s5k3h1gx_move_focus(int direction, int32_t num_steps)
{
  uint16_t s5k3h1gx_sw_damping_time_wait = 1;
  uint16_t s5k3h1gx_damping_threshold = 10;
  uint8_t s5k3h1gx_mode_mask = 0x02;
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
  int8_t s5k3h1gx_sw_damping_required = 0;
  uint16_t s5k3h1gx_max_fps_val;
  struct msm_camera_sensor_info *sinfo = s5k3h1gx_pdev->dev.platform_data;

  if (num_steps > S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR)
      num_steps = S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR;
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
  curr_lens_pos = s5k3h1gx_ctrl->curr_lens_pos;
  curr_step_pos = s5k3h1gx_ctrl->curr_step_pos;

  if (curr_lens_pos < s5k3h1gx_ctrl->init_curr_lens_pos)
      curr_lens_pos = s5k3h1gx_ctrl->init_curr_lens_pos;

  dest_step_pos = curr_step_pos + (step_direction * num_steps);

  if (dest_step_pos < 0)
      dest_step_pos = 0;
  else if (dest_step_pos > S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR)
      dest_step_pos = S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR;

  if (dest_step_pos == s5k3h1gx_ctrl->curr_step_pos)
      return rc;

  dest_lens_pos = s5k3h1gx_pos_tbl[dest_step_pos];
  target_dist = step_direction * (dest_lens_pos - curr_lens_pos);

  if (sinfo->csi_if) {
    s5k3h1gx_max_fps_val = S5K3H1GX_MAX_FPS;
  } else {
    s5k3h1gx_max_fps_val = S5K3H1GX_MAX_FPS_PARALLEL;
  }

  /* HW damping */
  if (step_direction < 0
    && target_dist >= s5k3h1gx_pos_tbl[s5k3h1gx_damping_threshold]) {
    s5k3h1gx_sw_damping_required = 1;
    time_wait = 1000000
      / s5k3h1gx_max_fps_val
      - S5K3H1GX_SW_DAMPING_STEP * s5k3h1gx_sw_damping_time_wait * 1000;
  } else
    time_wait = 1000000 / s5k3h1gx_max_fps_val;

  time_wait_per_step = (int16_t) (time_wait / target_dist);

  if (time_wait_per_step >= 800)
    /* ~800 */
    s5k3h1gx_mode_mask = 0x5;
  else if (time_wait_per_step >= 400)
    /* ~400 */
    s5k3h1gx_mode_mask = 0x4;
  else if (time_wait_per_step >= 200)
    /* 200~400 */
    s5k3h1gx_mode_mask = 0x3;
  else if (time_wait_per_step >= 100)
    /* 100~200 */
    s5k3h1gx_mode_mask = 0x2;
  else if (time_wait_per_step >= 50)
    /* 50~100 */
    s5k3h1gx_mode_mask = 0x1;
  else {
    if (time_wait >= 17600)
      s5k3h1gx_mode_mask = 0x0D;
    else if (time_wait >= 8800)
      s5k3h1gx_mode_mask = 0x0C;
    else if (time_wait >= 4400)
      s5k3h1gx_mode_mask = 0x0B;
    else if (time_wait >= 2200)
      s5k3h1gx_mode_mask = 0x0A;
    else
      s5k3h1gx_mode_mask = 0x09;
  }

  if (s5k3h1gx_sw_damping_required) {
    small_step = (uint16_t) target_dist / S5K3H1GX_SW_DAMPING_STEP;
    if ((target_dist % S5K3H1GX_SW_DAMPING_STEP) != 0)
      small_step = small_step + 1;

    for (next_lens_pos = curr_lens_pos + (step_direction*small_step);
      (step_direction*next_lens_pos) <= (step_direction*dest_lens_pos);
      next_lens_pos += (step_direction*small_step)) {
      rc = s5k3h1gx_go_to_position(next_lens_pos, s5k3h1gx_mode_mask);
      if (rc < 0) {
      CDBG("s5k3h1gx_go_to_position Failed in Move Focus!!!\n");
      return rc;
      }
      curr_lens_pos = next_lens_pos;
      mdelay(s5k3h1gx_sw_damping_time_wait);
    }

    if (curr_lens_pos != dest_lens_pos) {
      rc = s5k3h1gx_go_to_position(dest_lens_pos, s5k3h1gx_mode_mask);
      if (rc < 0) {
      pr_err("s5k3h1gx_go_to_position Failed in Move Focus!!!\n");
      return rc;
      }
      mdelay(s5k3h1gx_sw_damping_time_wait);
    }
  } else {
    rc = s5k3h1gx_go_to_position(dest_lens_pos, s5k3h1gx_mode_mask);
    if (rc < 0) {
      pr_err("s5k3h1gx_go_to_position Failed in Move Focus!!!\n");
      return rc;
    }
  }

  s5k3h1gx_ctrl->curr_lens_pos = dest_lens_pos;
  s5k3h1gx_ctrl->curr_step_pos = dest_step_pos;

  return rc;
}

static int32_t
s5k3h1gx_set_default_focus(void)
{
  int32_t rc = 0;
  if (s5k3h1gx_ctrl->curr_step_pos != 0) {
    rc = s5k3h1gx_move_focus(MOVE_FAR, s5k3h1gx_ctrl->curr_step_pos);
    if (rc < 0) {
      pr_err("s5k3h1gx_set_default_focus Failed!!!\n");
      return rc;
    }
  } else {
    rc = s5k3h1gx_go_to_position(0, 0x02);
    if (rc < 0) {
      pr_err("s5k3h1gx_go_to_position Failed!!!\n");
      return rc;
    }
  }

  s5k3h1gx_ctrl->curr_lens_pos = 0;
  s5k3h1gx_ctrl->init_curr_lens_pos = 0;
  s5k3h1gx_ctrl->curr_step_pos = 0;

  return rc;
}

uint8_t s5k3h1gx_preview_skip_frame(void)
{
	if (s5k3h1gx_ctrl->sensormode == SENSOR_PREVIEW_MODE && preview_frame_count < 2) {
		preview_frame_count++;
		return 1;
	}
	return 0;
}

int s5k3h1gx_sensor_config(void __user *argp)
{
  struct sensor_cfg_data cdata;
  long rc = 0;

  if (copy_from_user(&cdata,
    (void *)argp,
    sizeof(struct sensor_cfg_data)))
    return -EFAULT;

  down(&s5k3h1gx_sem);

  CDBG("s5k3h1gx_sensor_config: cfgtype = %d\n",
    cdata.cfgtype);
  switch (cdata.cfgtype) {
  case CFG_GET_PICT_FPS:
    s5k3h1gx_get_pict_fps(
      cdata.cfg.gfps.prevfps,
      &(cdata.cfg.gfps.pictfps));

    if (copy_to_user((void *)argp,
      &cdata,
      sizeof(struct sensor_cfg_data)))
      rc = -EFAULT;
    break;

  case CFG_GET_PREV_L_PF:
    cdata.cfg.prevl_pf =
      s5k3h1gx_get_prev_lines_pf();

    if (copy_to_user((void *)argp,
      &cdata,
      sizeof(struct sensor_cfg_data)))
      rc = -EFAULT;
    break;

  case CFG_GET_PREV_P_PL:
    cdata.cfg.prevp_pl =
      s5k3h1gx_get_prev_pixels_pl();

    if (copy_to_user((void *)argp,
      &cdata,
      sizeof(struct sensor_cfg_data)))
      rc = -EFAULT;
    break;

  case CFG_GET_PICT_L_PF:
    cdata.cfg.pictl_pf =
      s5k3h1gx_get_pict_lines_pf();

    if (copy_to_user((void *)argp,
      &cdata,
      sizeof(struct sensor_cfg_data)))
      rc = -EFAULT;
    break;

  case CFG_GET_PICT_P_PL:
    cdata.cfg.pictp_pl =
      s5k3h1gx_get_pict_pixels_pl();

    if (copy_to_user((void *)argp,
      &cdata,
      sizeof(struct sensor_cfg_data)))
      rc = -EFAULT;
    break;

  case CFG_GET_PICT_MAX_EXP_LC:
    cdata.cfg.pict_max_exp_lc =
      s5k3h1gx_get_pict_max_exp_lc();

    if (copy_to_user((void *)argp,
      &cdata,
      sizeof(struct sensor_cfg_data)))
      rc = -EFAULT;
    break;

  case CFG_SET_FPS:
  case CFG_SET_PICT_FPS:
    rc = s5k3h1gx_set_fps(&(cdata.cfg.fps));
    break;

  case CFG_SET_EXP_GAIN:
    rc =
      s5k3h1gx_write_exp_gain(
        cdata.cfg.exp_gain.gain,
        cdata.cfg.exp_gain.line);
    break;

  case CFG_SET_PICT_EXP_GAIN:
    rc =
      s5k3h1gx_set_pict_exp_gain(
        cdata.cfg.exp_gain.gain,
        cdata.cfg.exp_gain.line);
    break;

  case CFG_SET_MODE:
    rc = s5k3h1gx_set_sensor_mode(cdata.mode,
      cdata.rs);
    break;

  case CFG_PWR_DOWN:
    rc = s5k3h1gx_power_down();
    break;

  case CFG_MOVE_FOCUS:
    rc =
      s5k3h1gx_move_focus(
      cdata.cfg.focus.dir,
      cdata.cfg.focus.steps);
    break;

  case CFG_SET_DEFAULT_FOCUS:
    rc =
      s5k3h1gx_set_default_focus();
    break;

	case CFG_I2C_IOCTL_R_OTP:{
		pr_info("Line:%d CFG_I2C_IOCTL_R_OTP \n", __LINE__);
		rc = s5k3h1gx_i2c_read_fuseid(&cdata);
		if (copy_to_user(argp, &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		}
		break;
  default:
    rc = -EFAULT;
    break;
  }

  prevent_suspend();
  up(&s5k3h1gx_sem);

  return rc;
}

static int s5k3h1gx_sensor_release(void)
{
	int rc = -EBADF;

	down(&s5k3h1gx_sem);
	pr_info("%s, %d\n", __func__, __LINE__);

	if (s5k3h1gx_ctrl) {
		rc = gpio_request(s5k3h1gx_ctrl->sensordata->sensor_pwd, "s5k3h1gx");
		if (!rc)
			gpio_direction_output(s5k3h1gx_ctrl->sensordata->sensor_pwd, 0);
		else
			pr_err("GPIO (%d) request failed\n", s5k3h1gx_ctrl->sensordata->sensor_pwd);
		gpio_free(s5k3h1gx_ctrl->sensordata->sensor_pwd);
	}

	if (s5k3h1gx_ctrl->sensordata->vcm_pwd) {
		msleep(5);
		rc = gpio_request(s5k3h1gx_ctrl->sensordata->vcm_pwd, "s5k3h1gx");
		if (!rc)
			gpio_direction_output(s5k3h1gx_ctrl->sensordata->vcm_pwd, 0);
		else
			pr_err("GPIO (%d) request faile\n", s5k3h1gx_ctrl->sensordata->vcm_pwd);
		gpio_free(s5k3h1gx_ctrl->sensordata->vcm_pwd);
	}

	msm_camio_probe_off(s5k3h1gx_pdev);
	s5k3h1gx_vreg_disable(s5k3h1gx_pdev);

	if (s5k3h1gx_ctrl) {
		kfree(s5k3h1gx_ctrl);
		s5k3h1gx_ctrl = NULL;
	}

	allow_suspend();
	pr_info("s5k3h1gx_release completed\n");
	up(&s5k3h1gx_sem);

	return rc;
}

static int s5k3h1gx_sensor_probe(struct msm_camera_sensor_info *info,
  struct msm_sensor_ctrl *s)
{
  int rc = 0;
  printk("s5k3h1gx_sensor_probe()\n");

  rc = i2c_add_driver(&s5k3h1gx_i2c_driver);
  if (rc < 0 || s5k3h1gx_client == NULL) {
    rc = -ENOTSUPP;
    goto probe_fail;
  }

  pr_info("ov8810 s->node %d\n", s->node);
  sensor_probe_node = s->node;

  /* switch PCLK and MCLK to Main cam */
  pr_info("s5k3h1gx: s5k3h1gx_sensor_probe: switch clk\n");
  if(info->camera_clk_switch != NULL)
    info->camera_clk_switch();

  mdelay(20);

  pr_info("[Camera] gpio_request(%d, \"s5k3h1gx\")\n", info->sensor_pwd);
  rc = gpio_request(info->sensor_pwd, "s5k3h1gx");
  if (!rc)
    gpio_direction_output(info->sensor_pwd, 1);
  else
    pr_err("GPIO (%d) request failed\n", info->sensor_pwd);
  gpio_free(info->sensor_pwd);

  msleep(100);

  /* read sensor id */
  rc = s5k3h1gx_probe_init_sensor(info);

  if (rc < 0)
    goto probe_fail;

  /* Initialize Sensor registers */
  rc = initialize_s5k3h1gx_registers();
  if (rc < 0)
    return rc;

  if (info->camera_main_set_probe != NULL)
    info->camera_main_set_probe(true);

  init_suspend();
  s->s_init = s5k3h1gx_sensor_open_init;
  s->s_release = s5k3h1gx_sensor_release;
  s->s_config  = s5k3h1gx_sensor_config;
  s5k3h1gx_probe_init_done(info);
  s5k3h1gx_sysfs_init();
  info->preview_skip_frame = s5k3h1gx_preview_skip_frame;

  pr_info("%s: s5k3h1gx_probe_init_done %d\n",  __func__, __LINE__);
  goto probe_done;

probe_fail:
  pr_err("SENSOR PROBE FAILS!\n");
probe_done:
  return rc;

}

static int __s5k3h1gx_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	printk("__s5k3h1gx_probe\n");
	s5k3h1gx_pdev = pdev;

	if (sdata->camera_main_get_probe != NULL) {
		if (sdata->camera_main_get_probe()) {
			pr_info("__s5k3h1gx_probe camera main get probed already.\n");
			return 0;
		}
	}

	rc = s5k3h1gx_vreg_enable(pdev);
	if (rc < 0)
		pr_err("__s5k3h1gx_probe fail sensor power on error\n");

	return msm_camera_drv_start(pdev, s5k3h1gx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k3h1gx_probe,
	.driver = {
		.name = "msm_camera_s5k3h1gx",
		.owner = THIS_MODULE,
	},
};

static int __init s5k3h1gx_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k3h1gx_init);
