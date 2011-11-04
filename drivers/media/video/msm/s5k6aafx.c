/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera_sensor.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include "s5k6aafx.h"
#include <linux/slab.h>

struct s5k6aafx_work {
	struct work_struct work;
};

static struct s5k6aafx_work *s5k6aafx_sensorw;
static struct i2c_client *s5k6aafx_client;

struct s5k6aafx_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

static struct s5k6aafx_ctrl *s5k6aafx_ctrl;
static struct platform_device *s5k6aafx_pdev;

static int op_mode;
static DECLARE_WAIT_QUEUE_HEAD(s5k6aafx_wait_queue);
DECLARE_MUTEX(s5k6aafx_sem);

static int sensor_probe_node = 0;
static enum frontcam_t previous_mirror_mode;
static int32_t config_csi;
static enum wb_mode current_wb = CAMERA_AWB_AUTO;
static int s5k6aafx_set_wb(enum wb_mode wb_value);

#define MAX_I2C_RETRIES 20
static int i2c_transfer_retry(struct i2c_adapter *adap,
			struct i2c_msg *msgs,
			int len)
{
	int i2c_retry = 0;
	int ns; /* number sent */

	while (i2c_retry++ < MAX_I2C_RETRIES) {
		ns = i2c_transfer(adap, msgs, len);
		if (ns == len)
			break;
		pr_err("[CAM]%s: try %d/%d: i2c_transfer sent: %d, len %d\n",
			__func__,
			i2c_retry, MAX_I2C_RETRIES, ns, len);
		msleep(10);
	}

	return ns == len ? 0 : -EIO;
}


static int s5k6aafx_i2c_txdata(unsigned short saddr,
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

	if (i2c_transfer_retry(s5k6aafx_client->adapter, msg, 1) < 0) {
		pr_info("[CAM]s5k6aafx_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int s5k6aafx_i2c_write(unsigned short saddr,
				 unsigned short waddr, unsigned short wdata)
{
	int rc = -EIO;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));

	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	rc = s5k6aafx_i2c_txdata(saddr, buf, 4);
	if (rc < 0)
		pr_info("[CAM]i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		     waddr, wdata);

	return rc;
}

static int s5k6aafx_i2c_write_table(struct s5k6aafx_i2c_reg_conf const
				       *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = s5k6aafx_i2c_write(s5k6aafx_client->addr,
		       reg_conf_tbl->waddr, reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static int s5k6aafx_i2c_rxdata(unsigned short saddr,
			      unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = 2,
		 .buf = rxdata,
		 },
		{
		 .addr = saddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxdata,
		 },
	};

	if (i2c_transfer_retry(s5k6aafx_client->adapter, msgs, 2) < 0) {
		pr_info("[CAM]s5k6aafx_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int s5k6aafx_i2c_read(unsigned short saddr,
				unsigned short raddr, unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k6aafx_i2c_rxdata(saddr, buf, 2);
	if (rc < 0){
		printk(KERN_ERR "s5k6aafx_i2c_read failed!\n");
		return rc;
	}
	
	*rdata = buf[0] << 8 | buf[1];

	return rc;
}

static int s5k6aafx_gpio_pull(int gpio_pin, int pull_mode)
{
	int rc = 0;
	rc = gpio_request(gpio_pin, "s5k6aafx");
	if (!rc)
		gpio_direction_output(gpio_pin, pull_mode);
	else
		pr_err("[CAM]GPIO(%d) request failed\n", gpio_pin);
	gpio_free(gpio_pin);
	return rc;
}

static int s5k6aafx_set_front_camera_mode(enum frontcam_t frontcam_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE || previous_mirror_mode == frontcam_value)  
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (frontcam_value) {
	case CAMERA_MIRROR:
		/*mirror and flip*/
		if (!s5k6aafx_ctrl->sensordata->full_size_preview &&
			!s5k6aafx_ctrl->sensordata->power_down_disable &&
			!s5k6aafx_ctrl->sensordata->mirror_mode) {
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D4, 0x0002);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D6, 0x0002);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0262, 0x0002);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0264, 0x0002);
		} else { /* for flyer */
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D4, 0x0001);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D6, 0x0001);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0262, 0x0001);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0264, 0x0001);
		}
		break;
	case CAMERA_REVERSE:
		/*reverse mode*/
		if (!s5k6aafx_ctrl->sensordata->full_size_preview &&
			!s5k6aafx_ctrl->sensordata->power_down_disable &&
			!s5k6aafx_ctrl->sensordata->mirror_mode) {
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D4, 0x0003);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D6, 0x0003);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0262, 0x0003);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0264, 0x0003);
		} else { /* for flyer */
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D4, 0x0000);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D6, 0x0000);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0262, 0x0000);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0264, 0x0000);
		}
		break;

	case CAMERA_PORTRAIT_REVERSE:
		/*portrait reverse mode*/
		if (!s5k6aafx_ctrl->sensordata->full_size_preview &&
			!s5k6aafx_ctrl->sensordata->power_down_disable &&
			!s5k6aafx_ctrl->sensordata->mirror_mode) {
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D4, 0x0000);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D6, 0x0000);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0262, 0x0000);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0264, 0x0000);
		} else { /* for flyer */
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D4, 0x0003);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x02D6, 0x0003);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0262, 0x0003);
			s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0264, 0x0003);
		}
		break;

	default:
		break;
	}

	s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_TC_GP_PrevConfigChanged, 0x0001);

	previous_mirror_mode = frontcam_value;
	
	msleep(80);

	return 0;
}


static int s5k6aafx_set_sensor_mode(int mode)
{
	struct msm_camera_csi_params s5k6aafx_csi_params;
	struct msm_camera_sensor_info *sinfo = s5k6aafx_pdev->dev.platform_data;

	if (config_csi == 0) {
		if (sinfo->csi_if) {
			/* config mipi csi controller */
			pr_info("set csi config\n");
			s5k6aafx_csi_params.data_format = CSI_8BIT;
			s5k6aafx_csi_params.lane_cnt = 1;
			s5k6aafx_csi_params.lane_assign = 0xe4;
			s5k6aafx_csi_params.dpcm_scheme = 0;
			s5k6aafx_csi_params.settle_cnt = 0x20;
			msm_camio_csi_config(&s5k6aafx_csi_params);
			mdelay(20);
			config_csi = 1;
		}
	}
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		pr_info("[CAM]s5k6aafx:sensor set mode: preview\n");
		op_mode = SENSOR_PREVIEW_MODE;

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_I2C_MODE, S5K6AAFX_I2C_MODE_GENERAL);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDH, S5K6AAFX_ADDH_SW_REG_INT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x01F4);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0000); /* REG_TC_GP_EnableCapture */
		/* REG_TC_GP_EnableCaptureChanged */
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0400);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x007F);
		s5k6aafx_set_wb(current_wb);

		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_info("[CAM]s5k6aafx:sensor set mode: snapshot\n");
		op_mode = SENSOR_SNAPSHOT_MODE;

		mdelay(100);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_I2C_MODE, S5K6AAFX_I2C_MODE_GENERAL);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDH, S5K6AAFX_ADDH_SW_REG_INT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x01F4);
		/* REG_TC_GP_EnableCapture */
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);
		/* REG_TC_GP_EnableCaptureChanged */
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s5k6aafx_set_FPS(struct fps_cfg *fps)
{
	/* input is new fps in Q8 format */
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	if (fps->f_mult < 1 * 0x100 || fps->f_mult > 30 * 0x100)
		return -EINVAL;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_3TC_PCFG_usFrTimeType, 1);
	s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_3TC_PCFG_usMaxFrTimeMsecMult10, 1000 * 0x100 * 10 / fps->f_mult);
	s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_3TC_PCFG_usMinFrTimeMsecMult10, 1000 * 0x100 * 10 / fps->f_mult);

	s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_TC_GP_PrevConfigChanged, 0x0001);

	return 0;
}

static int s5k6aafx_set_effect(int effect)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (effect) {
	case CAMERA_EFFECT_OFF:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EE, 0x0000);
		break;
	case CAMERA_EFFECT_MONO:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EE, 0x0001);
		break;
	case CAMERA_EFFECT_NEGATIVE:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EE, 0x0002);
		break;
	case CAMERA_EFFECT_SEPIA:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EE, 0x0003);
		break;
	case CAMERA_EFFECT_AQUA:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EE, 0x0004);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int s5k6aafx_set_antibanding(enum antibanding_mode antibanding_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (antibanding_value) {
	case CAMERA_ANTI_BANDING_50HZ:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DC, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DE, 0x0001);
		break;
	case CAMERA_ANTI_BANDING_60HZ:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DC, 0x0002);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DE, 0x0001);
		break;
	case CAMERA_ANTI_BANDING_AUTO:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DC, 0x0002);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DE, 0x0001);
		break;
	}
	return 0;
}


static int s5k6aafx_set_brightness(enum brightness_t brightness_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (brightness_value) {
	case CAMERA_BRIGHTNESS_N4:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0xFF81);
		break;
	case CAMERA_BRIGHTNESS_N3:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0xFFA1);
		break;
	case CAMERA_BRIGHTNESS_N2:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0xFFC1);
		break;
	case CAMERA_BRIGHTNESS_N1:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0xFFE1);
		break;
	case CAMERA_BRIGHTNESS_D:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0x0000);
		break;
	case CAMERA_BRIGHTNESS_P1:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0x001F);
		break;
	case CAMERA_BRIGHTNESS_P2:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0x003F);
		break;
	case CAMERA_BRIGHTNESS_P3:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0x005F);
		break;
	case CAMERA_BRIGHTNESS_P4:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E4, 0x007F);
		break;
	default:
		 break;
	}
	return 0;
}

static int s5k6aafx_set_wb(enum wb_mode wb_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (wb_value) {
	case CAMERA_AWB_AUTO: /*auto*/
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0400, 0x007F);
		break;
	case CAMERA_AWB_CLOUDY: /*cloudy*/
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0400, 0x0077);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D0, 0x0185);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D2, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D4, 0x0100);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D6, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D8, 0x0150);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DA, 0x0001);
		break;
	case CAMERA_AWB_INDOOR_HOME: /*Fluorescent*/
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0400, 0x0077);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D0, 0x0110);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D2, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D4, 0x0100);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D6, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D8, 0x0235);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DA, 0x0001);
		break;
	case CAMERA_AWB_INDOOR_OFFICE: /*Incandescent*/
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0400, 0x0077);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D0, 0x00D9);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D2, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D4, 0x0100);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D6, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D8, 0x0400);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DA, 0x0001);
		break;
	case CAMERA_AWB_SUNNY: /*outdoor*/
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x0400, 0x0077);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D0, 0x0175);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D2, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D4, 0x0100);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D6, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03D8, 0x0160);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x03DA, 0x0001);
		break;
	default:
		break;
	}
	current_wb = wb_value;
	return 0;
}


static int s5k6aafx_set_sharpness(enum sharpness_mode sharpness_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (sharpness_value) {
	case CAMERA_SHARPNESS_X0:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EA, 0xFF81);
		break;
	case CAMERA_SHARPNESS_X1:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EA, 0xFFC1);
		break;
	case CAMERA_SHARPNESS_X2:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EA, 0x0000);
		break;
	case CAMERA_SHARPNESS_X3:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EA, 0x003F);
		break;
	case CAMERA_SHARPNESS_X4:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01EA, 0x007F);
		break;
	default:
		break;
	}
	return 0;
}


static int s5k6aafx_set_saturation(enum saturation_mode saturation_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (saturation_value) {
	case CAMERA_SATURATION_X0:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E8, 0xFF81);
		break;
	case CAMERA_SATURATION_X05:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E8, 0xFFC1);
		break;
	case CAMERA_SATURATION_X1:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E8, 0x0000);
		break;
	case CAMERA_SATURATION_X15:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E8, 0x003F);
		break;
	case CAMERA_SATURATION_X2:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E8, 0x007F);
		break;
	default:
		break;
	}
	return 0;
}

static int s5k6aafx_set_contrast(enum contrast_mode contrast_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_ADDH_SW_REG_INT);

	switch (contrast_value) {
	case CAMERA_CONTRAST_N2:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E6, 0xFF81);
		break;
	case CAMERA_CONTRAST_N1:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E6, 0xFFC1);
		break;
	case CAMERA_CONTRAST_D:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E6, 0x0000);
		break;
	case CAMERA_CONTRAST_P1:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E6, 0x0029/*0x0028*/);
		break;
	case CAMERA_CONTRAST_P2:
		s5k6aafx_i2c_write(s5k6aafx_client->addr, 0x01E6, 0x0058/*0x0050*/);
		break;
	default:
		break;
	}
	return 0;
}



static int s5k6aafx_set_qtr_size_mode(enum qtr_size_mode qtr_size_mode_value)
{
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_I2C_MODE_GENERAL);
	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_W_ADDH, S5K6AAFX_ADDH_SW_REG_INT);

	switch (qtr_size_mode_value) {
	case NORMAL_QTR_SIZE_MODE:
		pr_info("NORMAL_QTR_SIZE_MODE\n");
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021C);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0003);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0224);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0000);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021E);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0226);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x020A);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_ADJ_FULL_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_ADJ_FULL_SIZE_HEIGHT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0212);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_FULL_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_FULL_SIZE_HEIGHT);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x01FA);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_ADJ_FULL_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_ADJ_FULL_SIZE_HEIGHT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR,  (S5K6AAFX_FULL_SIZE_WIDTH-S5K6AAFX_ADJ_FULL_SIZE_WIDTH)/2);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR,  (S5K6AAFX_FULL_SIZE_HEIGHT-S5K6AAFX_ADJ_FULL_SIZE_HEIGHT)/2);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_FULL_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_FULL_SIZE_HEIGHT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR,  0x0000);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR,  0x0000);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021A);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		mdelay(100);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021E);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0226);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		mdelay(600);

		break;
	case LARGER_QTR_SIZE_MODE:
		pr_info("LARGER_QTR_SIZE_MODE\n");
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021C);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0000);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0224);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021E);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0226);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x020A);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_HEIGHT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0212);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_HEIGHT);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x01FA);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_HEIGHT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, (S5K6AAFX_FULL_SIZE_WIDTH-S5K6AAFX_720P_SIZE_WIDTH)/2);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, (S5K6AAFX_FULL_SIZE_HEIGHT-S5K6AAFX_720P_SIZE_HEIGHT)/2);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_WIDTH);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, S5K6AAFX_720P_SIZE_HEIGHT);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, (S5K6AAFX_FULL_SIZE_WIDTH-S5K6AAFX_720P_SIZE_WIDTH)/2);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, (S5K6AAFX_FULL_SIZE_HEIGHT-S5K6AAFX_720P_SIZE_HEIGHT)/2);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021A);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		mdelay(100);

		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x021E);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_W_ADDL, 0x0226);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, 0x0001);

		mdelay(200);

		break;
	default:
		break;
	}
	return 0;
}

#if 0
static int s5k6aafx_set_metering_mode(enum aec_metering_mode metering_value)
{
	uint16_t weight_table[32];
	uint8_t i;

	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	for (i = 0; i < 32; i++)
		weight_table[i] = 0x0101;

	if (metering_value == CAMERA_METERING_CENTERING) {
		weight_table[9] = 0x0303;
		weight_table[10] = 0x0303;
		weight_table[13] = 0x0303; /* 0x0305 */
		weight_table[14] = 0x0303; /* 0x0503 */
		weight_table[17] = 0x0303; /* 0x0305 */
		weight_table[18] = 0x0303; /* 0x0503 */
		weight_table[21] = 0x0303;
		weight_table[22] = 0x0303;
	} else if (metering_value == CAMERA_METERING_SPOT) {
		weight_table[13] = 0x0501;
		weight_table[14] = 0x0105;
		weight_table[17] = 0x0501;
		weight_table[18] = 0x0105;
	} else if (metering_value >= CAMERA_METERING_ZONE1 &&
		metering_value <= CAMERA_METERING_ZONE16) {
		i = metering_value - CAMERA_METERING_ZONE1;
		i += (i & 0xFC); /* i=i+((int)(i/4))*4 */
		weight_table[i] = 0x0505;
		weight_table[i+4] = 0x0505;
	}

	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_I2C_MODE, S5K6AAFX_I2C_MODE_GENERAL);
	s5k6aafx_i2c_write(s5k6aafx_client->addr,
		S5K6AAFX_REG_W_ADDH, S5K6AAFX_ADDH_SW_REG_INT);
	s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_W_ADDL, 0x100E);

	for (i = 0; i < 32; i++) {
		CDBG("write table[%d]=%x\n", i, weight_table[i]);
		s5k6aafx_i2c_write(s5k6aafx_client->addr,
			S5K6AAFX_REG_WR, weight_table[i]);
	}

	return 0;
}
#endif

static int s5k6aafx_sensor_read_id(const struct msm_camera_sensor_info *data)
{
	uint16_t model_id;
	int rc = 0;
	
	pr_info("[CAM]s5k6aafx_sensor_read_id\n");
	/* Read the Model ID of the sensor */
	rc = s5k6aafx_i2c_write(s5k6aafx_client->addr,
	       S5K6AAFX_REG_I2C_MODE, S5K6AAFX_I2C_MODE_GENERAL);
	if (rc < 0)
		goto init_probe_fail;
	rc = s5k6aafx_i2c_write(s5k6aafx_client->addr,
	       S5K6AAFX_REG_R_ADDH, S5K6AAFX_ADDH_SW_REG_INT);
	if (rc < 0)
		goto init_probe_fail;
	rc = s5k6aafx_i2c_write(s5k6aafx_client->addr,
	       S5K6AAFX_REG_R_ADDL, S5K6AAFX_REG_MODEL_ID);
	if (rc < 0)
		goto init_probe_fail;
	rc = s5k6aafx_i2c_read(s5k6aafx_client->addr,
		S5K6AAFX_REG_WR, &model_id);
	if (rc < 0)
		goto init_probe_fail;

	pr_info("[CAM]s5k6aafx: model_id = 0x%x\n", model_id);
	/* Check if it matches it with the value in Datasheet */
	if (model_id != S5K6AAFX_MODEL_ID) {
		pr_info("[CAM]invalid model id\n");
		rc = -EINVAL;
		goto init_probe_fail;
	}

init_probe_fail:
	return rc;

}

static int s5k6aafx_vreg_enable(struct platform_device *pdev)
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

static int s5k6aafx_vreg_disable(struct platform_device *pdev)
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

int s5k6aafx_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int rc = 0;
	config_csi = 0;

	s5k6aafx_ctrl = kzalloc(sizeof(struct s5k6aafx_ctrl), GFP_KERNEL);
	if (!s5k6aafx_ctrl) {
		pr_info("[CAM]s5k6aafx_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	printk("s5k6aafx_regs.reset_init after 0x%X\n",
		(unsigned int)s5k6aafx_regs.reset_init);

	if (data == NULL) {
		pr_err("[CAM]%s sensor data is NULL\n", __func__);
		return -EINVAL;
	}
	s5k6aafx_ctrl->sensordata = data;


	if (!data->power_down_disable)
		s5k6aafx_vreg_enable(s5k6aafx_pdev);


	/*switch PCLK and MCLK to 2nd cam*/
	pr_info("[CAM]s5k6aafx: s5k6aafx_sensor_probe switch clk\n");
	if (data->camera_clk_switch != NULL)
		data->camera_clk_switch();


	if (data->camera_pm8058_power != NULL) {
		if (data->camera_pm8058_power(1) < 0)
			goto init_fail;
	} else {
		if (s5k6aafx_gpio_pull(data->sensor_pwd, 1) < 0)
			goto init_fail;
	}
	mdelay(1);


	data->pdata->camera_gpio_on();
	mdelay(1);


	/*MCLK enable*/
	pr_info("[CAM]s5k6aafx: MCLK enable clk\n");
	msm_camio_probe_on(s5k6aafx_pdev);
	mdelay(1);

	if (s5k6aafx_gpio_pull(data->sensor_reset, 1) < 0)
		goto init_fail;
	/*sugest by samsung change from 100ms to 10ms to pass CTS 2nd cam launch time*/
	mdelay(10);

	/*reset sensor*/
	rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.reset_init[0],
		s5k6aafx_regs.reset_init_size);
	if (rc < 0)
		goto init_fail;
	mdelay(10);

	/*T&P setting*/
	rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.TP_init[0],
		s5k6aafx_regs.TP_init_size);
	if (rc < 0)
		goto init_fail;

	/*analog setting*/
	rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.analog_setting_init[0],
		s5k6aafx_regs.analog_setting_init_size);
	if (rc < 0)
		goto init_fail;
	mdelay(10);

	/*set initial register*/
	rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.register_init[0],
		s5k6aafx_regs.register_init_size);
	if (rc < 0)
		goto init_fail;

	/*set clock*/
	if (data->csi_if) {/*mipi*/
		pr_info("set mipi sensor clk\n");
		rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.mipi_clk_init[0],
			s5k6aafx_regs.mipi_clk_init_size);
		if (rc < 0)
			goto init_fail;
		/*sugest by samsung to pass CTS 2nd cam launch time*/
		/*mdelay(134);*/
	} else { /*parallel*/
		if (!data->full_size_preview) {
			rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.clk_init[0],
				s5k6aafx_regs.clk_init_size);
		} else { /* for flyer */
			pr_info("[CAM]%s: clk_init_tb2\n", __func__);
			rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.clk_init_tb2[0],
				s5k6aafx_regs.clk_init_tb2_size);
		}

		if (rc < 0)
			goto init_fail;
		/*sugest by samsung to pass CTS 2nd cam launch time*/
		/*mdelay(100);*/
	}

	/* preview configuration */
	if (!data->full_size_preview) {
		pr_info("[CAM]%s: pre_snap_conf_init\n", __func__);
		rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.prev_snap_conf_init[0],
			s5k6aafx_regs.prev_snap_conf_init_size);
	} else {
		pr_info("[CAM]%s: pre_snap_conf_init_tb2\n", __func__);
		rc = s5k6aafx_i2c_write_table(&s5k6aafx_regs.prev_snap_conf_init_tb2[0],
			s5k6aafx_regs.prev_snap_conf_init_tb2_size);
	}

	if (rc < 0)
		goto init_fail;

	if (data->csi_if) {/*mipi*/
		mdelay(200);
		// MIPI Non_continous enable
		s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_W_ADDH, 0xD000);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_W_ADDL, 0xB0CC);
		s5k6aafx_i2c_write(s5k6aafx_client->addr, S5K6AAFX_REG_WR, 0x000B);
	}

	if (!data->csi_if)
		msm_camio_camif_pad_reg_reset();

	rc = s5k6aafx_sensor_read_id(data);
	if (rc < 0)
		goto init_fail;

	op_mode = -1;
	previous_mirror_mode = -1;
init_done:
	return rc;

init_fail:
	kfree(s5k6aafx_ctrl);
	s5k6aafx_ctrl = NULL;
	return rc;
}

static int s5k6aafx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k6aafx_wait_queue);
	return 0;
}

int s5k6aafx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;
	if (copy_from_user(&cfg_data,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = s5k6aafx_set_sensor_mode(cfg_data.mode);
		break;
	case CFG_SET_EFFECT:
		rc = s5k6aafx_set_effect(cfg_data.cfg.effect);
		break;
	case CFG_SET_FPS:
		rc = s5k6aafx_set_FPS(&(cfg_data.cfg.fps));
		break;
	case CFG_SET_ANTIBANDING:
		rc = s5k6aafx_set_antibanding
				(cfg_data.cfg.antibanding_value);
		break;
	case CFG_SET_BRIGHTNESS:
		rc = s5k6aafx_set_brightness
				(cfg_data.cfg.brightness_value);
		break;
	case CFG_SET_WB:
		rc = s5k6aafx_set_wb(cfg_data.cfg.wb_value);
		break;
	case CFG_SET_SHARPNESS:
		rc = s5k6aafx_set_sharpness
			(cfg_data.cfg.sharpness_value);
		break;
	case CFG_SET_SATURATION:
		rc = s5k6aafx_set_saturation
			(cfg_data.cfg.saturation_value);
		break;
	case CFG_SET_CONTRAST:
		rc = s5k6aafx_set_contrast(cfg_data.cfg.contrast_value);
		break;
	case CFG_SET_FRONT_CAMERA_MODE:
		rc = s5k6aafx_set_front_camera_mode(cfg_data.cfg.frontcam_value);
		break;
	case CFG_SET_QTR_SIZE_MODE:
		rc = s5k6aafx_set_qtr_size_mode(cfg_data.cfg.qtr_size_mode_value);
		break;
#if 0
	case CFG_SET_EXPOSURE_MODE:
		rc = s5k6aafx_set_metering_mode
			(cfg_data.cfg.metering_value);
		break;
#endif
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int s5k6aafx_sensor_release(void)
{
	int rc = 0;
	struct msm_camera_sensor_info *sdata = s5k6aafx_pdev->dev.platform_data;

	down(&s5k6aafx_sem);

	if (s5k6aafx_ctrl)
		s5k6aafx_gpio_pull(s5k6aafx_ctrl->sensordata->sensor_reset, 0);
	mdelay(1);


	if (s5k6aafx_ctrl) {
		if (s5k6aafx_ctrl->sensordata->camera_pm8058_power != NULL)
		  s5k6aafx_ctrl->sensordata->camera_pm8058_power(0);
		else
		  s5k6aafx_gpio_pull(s5k6aafx_ctrl->sensordata->sensor_pwd, 0);
	}
	mdelay(1);


	msm_camio_probe_off(s5k6aafx_pdev);

	sdata->pdata->camera_gpio_off();
	mdelay(1);

	if (!sdata->power_down_disable) {
		s5k6aafx_vreg_disable(s5k6aafx_pdev);
	}

	if (s5k6aafx_ctrl) {
		kfree(s5k6aafx_ctrl);
		s5k6aafx_ctrl = NULL;
	}

	up(&s5k6aafx_sem);
	return rc;
}

static const char *S5K6AAFXVendor = "Samsung";
static const char *S5K6AAFXNAME = "s5k6aafx";
static const char *S5K6AAFXSize = "1M";
static uint32_t htcwc_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", S5K6AAFXVendor, S5K6AAFXNAME, S5K6AAFXSize);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t htcwc_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", htcwc_value);
	return length;
}

static ssize_t htcwc_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

#if 0
	if (strcmp(current->comm,"com.android.camera")!=0){
		pr_info("[CAM]No permission : not camera ap\n");
		return -EINVAL;
	}
#endif

	htcwc_value = tmp;
	//pr_info("[CAM]current_comm = %s\n", current->comm);
	pr_info("[CAM]htcwc_value = %d\n", htcwc_value);
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
static DEVICE_ATTR(htcwc, 0777, htcwc_get, htcwc_set);
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);

static struct kobject *android_s5k6aafx;

static int s5k6aafx_sysfs_init(void)
{
	int ret ;
	pr_info("[CAM]s5k6aafx:kobject creat and add\n");
	android_s5k6aafx = kobject_create_and_add("android_camera2", NULL);
	if (android_s5k6aafx == NULL) {
		pr_info("[CAM]s5k6aafx_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM]s5k6aafx:sysfs_create_file\n");
	ret = sysfs_create_file(android_s5k6aafx, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]s5k6aafx_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_s5k6aafx);
	}

	ret = sysfs_create_file(android_s5k6aafx, &dev_attr_htcwc.attr);
	if (ret) {
		pr_info("[CAM]s5k6aafx_sysfs_init: sysfs_create_file htcwc failed\n");
		kobject_del(android_s5k6aafx);
	}

       ret = sysfs_create_file(android_s5k6aafx, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM]s5k6aafx_sysfs_init: dev_attr_node failed\n");
		kobject_del(android_s5k6aafx);
	}

	return 0 ;
}


static int s5k6aafx_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_camera_sensor_info *info = s5k6aafx_pdev->dev.platform_data;

	if (s5k6aafx_client || s5k6aafx_sensorw) {
		if (s5k6aafx_client)
			pr_info("[CAM]s5k6aafx_i2c_probe s5k6aafx_client existed 0x%X\n",
				s5k6aafx_client->addr);
		return 0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s5k6aafx_sensorw = kzalloc(sizeof(struct s5k6aafx_work), GFP_KERNEL);

	if (!s5k6aafx_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k6aafx_sensorw);
	s5k6aafx_init_client(client);
	s5k6aafx_client = client;
	pr_info("[CAM]s5k6aafx_i2c_probe s5k6aafx_client->addr %d\n",
		s5k6aafx_client->addr);

	rc = s5k6aafx_sensor_read_id(info);
	if (rc < 0) {
		goto probe_failure;
	}

	s5k6aafx_client = client;

	pr_info("[CAM]s5k6aafx_i2c_probe succeeded! s5k6aafx_client->addr 0x%X\n",
		s5k6aafx_client->addr);

	return 0;

probe_failure:
	kfree(s5k6aafx_sensorw);
	s5k6aafx_sensorw = NULL;
	s5k6aafx_client = NULL;
	pr_err("[CAM]s5k6aafx_i2c_probe failed!\n");
	return rc;
}

static const struct i2c_device_id s5k6aafx_i2c_id[] = {
	{"s5k6aafx", 0},
	{},
};

static struct i2c_driver s5k6aafx_i2c_driver = {
	.id_table = s5k6aafx_i2c_id,
	.probe = s5k6aafx_i2c_probe,
	.remove = __exit_p(s5k6aafx_i2c_remove),
	.driver = {
		   .name = "s5k6aafx",
		   },
};

static int s5k6aafx_sensor_probe(struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = 0;
	pr_info("[CAM]s5k6aafx s->node %d\n", s->node);
	sensor_probe_node = s->node;

	if (info->camera_pm8058_power != NULL) {
		if (info->camera_pm8058_power(1) < 0)
			goto probe_fail;
	} else {
		if (s5k6aafx_gpio_pull(info->sensor_pwd, 1) < 0)
			goto probe_fail;
	}
	mdelay(1);


	/*switch clk source*/
	pr_info("[CAM]s5k6aafx: s5k6aafx_sensor_probe switch clk\n");
	if(info->camera_clk_switch != NULL)
		info->camera_clk_switch();


	/*MCLK enable*/
	pr_info("[CAM]s5k6aafx: MCLK enable clk\n");
	mdelay(1);

	if (s5k6aafx_gpio_pull(info->sensor_reset, 1) < 0)
		goto probe_fail;
	mdelay(1);

	/* reset i2c client to null */
	s5k6aafx_client = NULL;
	rc = i2c_add_driver(&s5k6aafx_i2c_driver);
	if (rc < 0 || s5k6aafx_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

#if 0 /* move into i2c_probe */
	rc = s5k6aafx_sensor_read_id(info);
	if (rc < 0) {
			goto probe_fail;
	}
#endif

	if (s5k6aafx_client->addr == 0x5a >> 1) {
		printk("s5k6aafx_client->addr == 0x5a, COB \n");
		s5k6aafx_regs = s5k6aafx_regs_cob;
	}

	s->s_init = s5k6aafx_sensor_open_init;
	s->s_release = s5k6aafx_sensor_release;
	s->s_config = s5k6aafx_sensor_config;

	/*init done*/
	mdelay(800);

	s5k6aafx_gpio_pull(info->sensor_reset, 0);
	mdelay(1);

	if (info->camera_pm8058_power != NULL)
		info->camera_pm8058_power(0);
	else
		s5k6aafx_gpio_pull(info->sensor_pwd, 0);
	mdelay(1);

	info->pdata->camera_gpio_off();
	mdelay(1);

	if (!info->power_down_disable)
		s5k6aafx_vreg_disable(s5k6aafx_pdev);

	s5k6aafx_sysfs_init();

	mdelay(5);

probe_done:
	pr_info("[CAM]%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
probe_fail:
	info->pdata->camera_gpio_off();
	mdelay(10);
	if (!info->power_down_disable)
		s5k6aafx_vreg_disable(s5k6aafx_pdev);
	pr_err("[CAM]S5K6AAFX probe failed\n");
	return rc;

}

static int __s5k6aafx_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;

	s5k6aafx_pdev = pdev;
	if (!sdata->power_down_disable) {
	rc = s5k6aafx_vreg_enable(pdev);


	/*switch clk source*/
	pr_info("[CAM]s5k6aafx: __s5k6aafx_probe switch clk\n");
	if (sdata->camera_clk_switch != NULL)
		sdata->camera_clk_switch();


	if (sdata->camera_pm8058_power != NULL) {
		if (sdata->camera_pm8058_power(1) < 0)
			pr_info("[CAM]s5k6aafx: __s5k6aafx_probe sensor pwd failed\n");
	} else {
		if (s5k6aafx_gpio_pull(sdata->sensor_pwd, 1) < 0)
			pr_info("[CAM]s5k6aafx: __s5k6aafx_probe sensor pwd failed\n");
	}
	mdelay(1);

	sdata->pdata->camera_gpio_on();
	if (rc < 0)
		pr_err("[CAM]__s5k6aafx_probe fail sensor power on error\n");
	}

	return msm_camera_drv_start(pdev, s5k6aafx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k6aafx_probe,
	.driver = {
#ifdef CONFIG_MSM_CAMERA_8X60
		   .name = "msm_camera_webcam",
#else
		   .name = "msm_camera_s5k6aafx",
#endif
		   .owner = THIS_MODULE,
		   },
};

static int __init s5k6aafx_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k6aafx_init);
