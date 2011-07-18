/*
 * HTC Corporation Proprietary Rights Acknowledgment
 *
 * Copyright (C) 2008 HTC Corporation
 *
 * All Rights Reserved.
 *
 * The information contained in this work is the exclusive property
 * of HTC Corporation("HTC").  Only the user who is legally authorized
 * by HTC ("Authorized User") has right to employ this work within the
 * scope of this statement.  Nevertheless, the Authorized User shall not
 * use this work for any purpose other than the purpose agreed by HTC.
 * Any and all addition or modification to this work shall be  unconditionally
 * granted back to HTC and such addition or modification shall be solely
 * owned by HTC.  No right is granted under this statement, including but not
 * limited to, distribution, reproduction, and transmission, except as
 * otherwise provided in this statement.  Any other usage of this work shall
 *  be subject to the further written consent of HTC.
 */


#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>

#ifdef CONFIG_MSM_CAMERA_8X60
#include <mach/camera-8x60.h>
#endif

#include <media/msm_camera_sensor.h>

#include <mach/gpio.h>
#include "mt9v113.h"
#include <asm/mach-types.h>

/* mt9v113 Registers and their values */
/* Sensor Core Registers */

#define  MT9V113_MODEL_ID     	0x2280 /* Model ID */
#define  MT9V113_MODEL_ID_ADDR  0x0000 /* the address for reading Model ID*/
static int op_mode;
static int32_t config_csi = 0;
/* Read Mode */
#define MT9V113_REG_READ_MODE_ADDR_1	0x2717
#define MT9V113_REG_READ_MODE_ADDR_2	0x272D
#define MT9V113_READ_NORMAL_MODE	0x0024	/* without mirror/flip */
#define MT9V113_READ_MIRROR_FLIP	0x0027	/* with mirror/flip */

struct mt9v113_work {
	struct work_struct work;
};

static struct mt9v113_work *mt9v113_sensorw;
static struct i2c_client *mt9v113_client;

struct mt9v113_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
};

static struct mt9v113_ctrl_t *mt9v113_ctrl;
static struct platform_device *mt9v113_pdev;

static DECLARE_WAIT_QUEUE_HEAD(mt9v113_wait_queue);

/* DECLARE_MUTEX(mt9v113_sem); */

static int sensor_probe_node = 0;

#define MAX_I2C_RETRIES 20
#define CHECK_STATE_TIME 100

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

static int mt9v113_i2c_txdata(unsigned short saddr,
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

#if 1
	if (i2c_transfer_retry(mt9v113_client->adapter, msg, 1) < 0) {
		pr_err("[CAM]mt9v113_i2c_txdata failed\n");
		return -EIO;
	}
#else
	if (i2c_transfer(mt9v113_client->adapter, msg, 1) < 0) {
		pr_info("[CAM]mt9v113_i2c_txdata failed\n");
		return -EIO;
	}
#endif

	return 0;
}

static int mt9v113_i2c_write(unsigned short saddr,
				 unsigned short waddr, unsigned short wdata,
				 enum mt9v113_width width)
{
	int rc = -EIO;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN:{
			buf[0] = (waddr & 0xFF00) >> 8;
			buf[1] = (waddr & 0x00FF);
			buf[2] = (wdata & 0xFF00) >> 8;
			buf[3] = (wdata & 0x00FF);

			rc = mt9v113_i2c_txdata(saddr, buf, 4);
		}
		break;

	case BYTE_LEN:{
			buf[0] = waddr;
			buf[1] = wdata;
			rc = mt9v113_i2c_txdata(saddr, buf, 2);
		}
		break;

	default:
		break;
	}

	if (rc < 0)
		pr_info("[CAM]i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		     waddr, wdata);

	return rc;
}

static int mt9v113_i2c_write_table(struct mt9v113_i2c_reg_conf
				       *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mt9v113_i2c_write(mt9v113_client->addr,
				       reg_conf_tbl->waddr, reg_conf_tbl->wdata,
				       reg_conf_tbl->width);
		if (rc < 0) {
		pr_err("[CAM]%s: num_of_items_in_table=%d\n", __func__,
			num_of_items_in_table);
			break;
		}
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}

static int mt9v113_i2c_rxdata(unsigned short saddr,
			      unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = saddr,
		 .flags = 0,
		 .len = 2,  /* .len = 1, */
		 .buf = rxdata,
		 },
		{
		 .addr = saddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxdata,
		 },
	};

#if 1
	if (i2c_transfer_retry(mt9v113_client->adapter, msgs, 2) < 0) {
		pr_err("[CAM]mt9v113_i2c_rxdata failed!\n");
		return -EIO;
	}
#else
	if (i2c_transfer(mt9v113_client->adapter, msgs, 2) < 0) {
		pr_info("[CAM]mt9v113_i2c_rxdata failed!\n");
		return -EIO;
	}
#endif

	return 0;
}

/*read 2 bytes data from sensor via I2C */
static int32_t mt9v113_i2c_read_w(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9v113_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		CDBG("mt9v113_i2c_read_w failed!\n");

	return rc;
}

#if 0
static int mt9v113_i2c_read(unsigned short saddr,
				unsigned short raddr, unsigned char *rdata)
{
	int rc = 0;
	unsigned char buf[1];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = mt9v113_i2c_rxdata(saddr, buf, 1);
	if (rc < 0)
		return rc;
	*rdata = buf[0];
	if (rc < 0)
		pr_info("[CAM]mt9v113_i2c_read failed!\n");

	return rc;
}
#endif
static int mt9v113_i2c_write_bit(unsigned short saddr, unsigned short raddr,
unsigned short bit, unsigned short state)
{
	int rc;
	unsigned short check_value;
	unsigned short check_bit;

	if (state)
		check_bit = 0x0001 << bit;
	else
		check_bit = 0xFFFF & (~(0x0001 << bit));
	pr_info(" mt9v113_i2c_write_bit check_bit:0x%4x", check_bit);
	rc = mt9v113_i2c_read_w(saddr, raddr, &check_value);
	if (rc < 0)
	  return rc;

	pr_info("%s: mt9v113: 0x%4x reg value = 0x%4x\n", __func__,
		raddr, check_value);
	if (state)
		check_value = (check_value | check_bit);
	else
		check_value = (check_value & check_bit);

	pr_info("%s: mt9v113: Set to 0x%4x reg value = 0x%4x\n", __func__,
		raddr, check_value);

	rc = mt9v113_i2c_write(saddr, raddr, check_value,
		WORD_LEN);
	return rc;
}

static int mt9v113_i2c_check_bit(unsigned short saddr, unsigned short raddr,
unsigned short bit, int check_state)
{
	int k;
	unsigned short check_value;
	unsigned short check_bit;
	check_bit = 0x0001 << bit;
	for (k = 0; k < CHECK_STATE_TIME; k++) {/* retry 100 times */
		mt9v113_i2c_read_w(mt9v113_client->addr,
			      raddr, &check_value);
		if (check_state) {
			if ((check_value & check_bit))
			break;
		} else {
			if (!(check_value & check_bit))
			break;
		}
		msleep(1);
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("%s failed addr:0x%2x data check_bit:0x%2x",
			__func__, raddr, check_bit);
		return -1;
	}
	return 1;
}
/*
static int mt9v113_set_gpio(int num, int status)
{
	int rc = 0;
	rc = gpio_request(num, "mt9v113");
	if (!rc) {
		gpio_direction_output(num, status);
		msleep(5);
	} else
		pr_err("GPIO(%d) request faile", status);

	gpio_free(num);

	return rc;
}
*/
static inline int resume(void)
{
	int k = 0, rc = 0;
	unsigned short check_value;

	/* enter SW Active mode */
	/* write 0x0016[5] to 1  */
	rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0016, &check_value);
	if (rc < 0)
	  return rc;

	pr_info("[CAM]%s: mt9v113: 0x0016 reg value = 0x%x\n", __func__,
		check_value);

	check_value = (check_value|0x0020);

	pr_info("[CAM]%s: mt9v113: Set to 0x0016 reg value = 0x%x\n", __func__,
		check_value);

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0016, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("[CAM]%s: Enter Active mode fail\n", __func__);
		return rc;
	}

	/* write 0x0018[0] to 0 */
	rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0018, &check_value);
	if (rc < 0)
	  return rc;

	pr_info("[CAM]%s: mt9v113: 0x0018 reg value = 0x%x\n", __func__,
		check_value);

	check_value = (check_value & 0xFFFE);

	pr_info("[CAM]%s: mt9v113: Set to 0x0018 reg value = 0x%x\n", __func__,
		check_value);

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0018, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("[CAM]%s: Enter Active mode fail\n", __func__);
		return rc;
	}

	/* check 0x0018[14] is 0 */
	for (k = 0; k < CHECK_STATE_TIME; k++) {/* retry 100 times */
		mt9v113_i2c_read_w(mt9v113_client->addr,
			  0x0018, &check_value);

		pr_info("[CAM]%s: mt9v113: 0x0018 reg value = 0x%x\n", __func__,
			check_value);

		if (!(check_value & 0x4000)) {/* check state of 0x0018 */
			pr_info("[CAM]%s: (check 0x0018[14] is 0) k=%d\n",
				__func__, k);
			break;
		}
		msleep(1);	/*MAX: delay 100ms */
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("[CAM]%s: check status time out (check 0x0018[14] is 0)\n",
			__func__);
		return -EIO;
	}

	/* check 0x31E0 is 0x003 */
	for (k = 0; k < CHECK_STATE_TIME; k++) {/* retry 100 times */
		rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x31E0,
			&check_value);
		if (check_value == 0x0003) { /* check state of 0x31E0 */
			pr_info("[CAM]%s: (check 0x31E0 is 0x003 ) k=%d\n",
				__func__, k);
			break;
		}
		msleep(1);	/*MAX: delay 100ms */
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("[CAM]%s: check status time out (check 0x31E0 is 0x003 )\n",
			__func__);
		return -EIO;
	}

	/* write 0x31E0 to 0x0001 */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x31E0, 0x0001,
	WORD_LEN);
	if (rc < 0) {
		pr_err("[CAM]%s: Enter Active mode fail\n", __func__);
		return rc;
	}

    msleep(2);

	return rc;

}

static inline int suspend(void)
{
	int k = 0, rc = 0;
	unsigned short check_value;

	/* enter SW Standby mode */
	/* write 0x0018[3] to 1 */
	rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0018, &check_value);
	if (rc < 0)
	  return rc;

	check_value = (check_value|0x0008);

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0018, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("[CAM]%s: Enter standy mode fail\n", __func__);
		return rc;
	}
	/* write 0x0018[0] to 1 */
	rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0018, &check_value);
	if (rc < 0)
	  return rc;

	check_value = (check_value|0x0001);

	pr_info("[CAM]%s: mt9v113: Set to 0x0018 reg value = 0x%x\n", __func__,
		check_value);

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0018, check_value,
		WORD_LEN);
	if (rc < 0) {
		pr_err("[CAM]%s: Enter standy mode fail\n", __func__);
		return rc;
	}

	/* check 0x0018[14] is 1 */
	for (k = 0; k < CHECK_STATE_TIME; k++) {/* retry 100 times */
		mt9v113_i2c_read_w(mt9v113_client->addr,
			  0x0018, &check_value);
		if ((check_value & 0x4000)) { /* check state of 0x0018 */
			pr_info("[CAM]%s: ( check 0x0018[14] is 1 ) k=%d\n",
				__func__, k);
			break;
		}
		msleep(1);	/*MAX: delay 100ms */
	}
	if (k == CHECK_STATE_TIME) {
		pr_err("[CAM]%s: check status time out\n", __func__);
		return -EIO;
	}
    msleep(2);
	return rc;

}


static int mt9v113_reset(const struct msm_camera_sensor_info *dev)
{
	int rc = 0;

	pr_info("[CAM]%s: ++++\n", __func__);

	rc = gpio_request(dev->sensor_reset, "mt9v113");

	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 1);
		msleep(2);
		rc = gpio_direction_output(dev->sensor_reset, 0);
		msleep(1);
	} else
		pr_err("[CAM]GPIO(%d) request faile", dev->sensor_reset);

	gpio_free(dev->sensor_reset);
	return rc;
}


static int mt9v113_reg_init(void)
{
	int rc = 0, k = 0;
	unsigned short check_value;

    /* Power Up Start */
	pr_info("[CAM]%s: Power Up Start\n", __func__);

	rc = mt9v113_i2c_write_table(&mt9v113_regs.power_up_tbl[0],
				     mt9v113_regs.power_up_tbl_size);
	if (rc < 0) {
		pr_err("[CAM]%s: Power Up fail\n", __func__);
		goto reg_init_fail;
	}

	/* RESET and MISC Control */
	pr_info("[CAM]%s: RESET and MISC Control\n", __func__);

	rc = mt9v113_i2c_write(mt9v113_client->addr,
					0x0018, 0x4028, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_check_bit(mt9v113_client->addr, 0x0018, 14, 0);
	if (rc < 0)
		goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr,
					0x001A, 0x0003, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;
	mdelay(2);

	rc = mt9v113_i2c_write(mt9v113_client->addr,
					0x001A, 0x0000, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;
	mdelay(2);

	rc = mt9v113_i2c_write(mt9v113_client->addr,
					0x0018, 0x4028, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_check_bit(mt9v113_client->addr, 0x0018, 14, 0);
	if (rc < 0)
		goto reg_init_fail;


#ifdef CONFIG_MSM_CAMERA_8X60
	/*RESET_AND_MISC_CONTROL Parallel output port en MIPI*/
	rc = mt9v113_i2c_write_bit(mt9v113_client->addr, 0x001A, 9, 0);
	if (rc < 0)
	  goto reg_init_fail;

	/*MIPI control*/
	/* ---------------------------------------------------------------------- */
	/* Apply Aptina vendor's suggestion to fix incorrect color issue for MIPI */
	/* Set to enter STB(standby) after waiting for EOF(end of frame)          */
	rc = mt9v113_i2c_write_bit(mt9v113_client->addr, 0x3400, 4, 1);
	if (rc < 0)
	  goto reg_init_fail;
	/* ---------------------------------------------------------------------- */
	rc = mt9v113_i2c_write_bit(mt9v113_client->addr, 0x3400, 9, 1);
	if (rc < 0)
	  goto reg_init_fail;
	/*OFIFO_control_sstatus*/
	rc = mt9v113_i2c_write_bit(mt9v113_client->addr, 0x321C, 7, 0);
	if (rc < 0)
	  goto reg_init_fail;
#else
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x001A, 0x0210, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;
#endif


	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x001E, 0x0777, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;


	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0016, 0x42DF, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;


	/* PLL Setup Start */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0014, 0xB04B, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0014, 0xB049, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0010, 0x021C, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0012, 0x0000, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0014, 0x244B, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	msleep(10);


	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0014, 0x304B, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_check_bit(mt9v113_client->addr, 0x0014, 15, 1);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0014, 0xB04A, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	/* write a serial i2c cmd from register_init_tbl of mt9v113_reg */
	rc = mt9v113_i2c_write_table(&mt9v113_regs.register_init_1[0],
			mt9v113_regs.register_init_size_1);
	if (rc < 0)
	  goto reg_init_fail;

	/* write 0x3210[3] bit to 1 */
	rc = mt9v113_i2c_write_bit(mt9v113_client->addr, 0x3210, 3, 1);
	if (rc < 0)
	  goto reg_init_fail;

	/* write a serial i2c cmd from register_init_tb2 of mt9v113_reg */
	rc = mt9v113_i2c_write_table(&mt9v113_regs.register_init_2[0],
			mt9v113_regs.register_init_size_2);
	if (rc < 0)
	  goto reg_init_fail;

	/*the last three commands in the Mode-set up Preview (VGA) /
	Capture Mode (VGA) */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0006, WORD_LEN);
	for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
			&check_value);
		if (check_value == 0x0000) /* check state of 0xA103 */
			break;
		msleep(1);
	}
	if (k == CHECK_STATE_TIME)
		goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
			&check_value);
		if (check_value == 0x0000) /* check state of 0xA103 */
			break;
		msleep(1);
	}
	if (k == CHECK_STATE_TIME)
		goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA102, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x000F, WORD_LEN);
	if (rc < 0)
	  goto reg_init_fail;

	return rc;
reg_init_fail:
	pr_err("[CAM]mt9v113 register initial fail\n");
	return rc;
}


/* 0726 add new feature */
static int pre_mirror_mode;
static int mt9v113_set_front_camera_mode(enum frontcam_t frontcam_value)
{
	int rc = 0;
	int k = 0;
	unsigned short check_value;

	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	pr_info("[CAM]%s: frontcam_value=%d\n", __func__, frontcam_value);

	switch (frontcam_value) {
	case CAMERA_MIRROR:
		/*mirror and flip*/
	if (mt9v113_ctrl->sensordata->mirror_mode) {
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2717, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0024, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x272D, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0024, WORD_LEN);
	} else {
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2717, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0027, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x272D, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0027, WORD_LEN);
	}

	if (rc < 0)
		return -EIO;

		break;
	case CAMERA_REVERSE:
		/*reverse mode*/
	if (mt9v113_ctrl->sensordata->mirror_mode) {
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2717, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0025, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x272D, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0025, WORD_LEN);
	} else {
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2717, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0026, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x272D, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0026, WORD_LEN);
	}

	if (rc < 0)
		return -EIO;

		break;

	case CAMERA_PORTRAIT_REVERSE:
	/*portrait reverse mode*//* 0x25: do mirror */
	if (mt9v113_ctrl->sensordata->mirror_mode) {
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2717, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0026, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x272D, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0026, WORD_LEN);
	} else {
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2717, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0025, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x272D, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0025, WORD_LEN);
	}

	if (rc < 0)
		return -EIO;

		break;
	default:
		break;
	}

	/* refresh sensor */
	if (pre_mirror_mode != frontcam_value) {
	pr_info("%s: re-flash\n", __func__);

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0006, WORD_LEN);

	for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
			0xA103, WORD_LEN);
		rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
			&check_value);
		if (check_value == 0x0000) /* check state of 0xA103 */
			break;
		msleep(1);
	}
	if (k == CHECK_STATE_TIME) /* time out */
		return -EIO;

	}
	pre_mirror_mode = frontcam_value;

	msleep(20);

#if 0
	for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
			0xA103, WORD_LEN);
		rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
			&check_value);
		if (rc > 0 && check_value == 0x0000) /* check state of 0xA103 */
			break;

		msleep(1);
		pr_info("[CAM]%s: count =%d\n", __func__, k);
	}
	if (k == CHECK_STATE_TIME) /* time out */
		return -EIO;
#endif

	return 0;
}


static int mt9v113_set_sensor_mode(int mode)
{
	int rc = 0 , k;
	uint16_t check_value = 0;
	struct msm_camera_csi_params mt9v113_csi_params;
	struct msm_camera_sensor_info *sinfo = mt9v113_pdev->dev.platform_data;

	if (config_csi == 0) {
		if (sinfo->csi_if) {
			/* config mipi csi controller */
			pr_info("set csi config\n");
			mt9v113_csi_params.data_format = CSI_8BIT;
			mt9v113_csi_params.lane_cnt = 1;
			mt9v113_csi_params.lane_assign = 0xe4;
			mt9v113_csi_params.dpcm_scheme = 0;
			mt9v113_csi_params.settle_cnt = 0x14;
			msm_camio_csi_config(&mt9v113_csi_params);
			mdelay(20);
			config_csi = 1;

			rc = resume();
			if (rc < 0)
				pr_err("mt9v113 resume failed\n");

#ifdef CONFIG_MSM_CAMERA_8X60
			/* Apply sensor mirror/flip */
			pr_info("mt9v113_sensor_open_init() , Apply sensor mirror/flip\n");
			mt9v113_set_front_camera_mode(CAMERA_MIRROR);
#endif
		}
	}
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		op_mode = SENSOR_PREVIEW_MODE;
		pr_info("[CAM]mt9v113:sensor set mode: preview\n");

		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		if (rc < 0)
			return rc;

		/*rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0001,
		WORD_LEN);*/
		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0002,
		WORD_LEN);
		if (rc < 0)
			return rc;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA104,	WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0003) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) {
			pr_err("[CAM]%s: Preview fail\n", __func__);
			return -EIO;
		}
		/*prevent preview image segmentation*/
		msleep(150);
		break;

	case SENSOR_SNAPSHOT_MODE:
		op_mode = SENSOR_SNAPSHOT_MODE;
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		pr_info("[CAM]mt9v113:sensor set mode: snapshot\n");

		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103,
			WORD_LEN);
		if (rc < 0)
			return rc;

		/* rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0002,
		WORD_LEN); */
		rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0001,
		WORD_LEN);
		if (rc < 0)
			return rc;

		for (k = 0; k < CHECK_STATE_TIME; k++) {/* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA104, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0003)/* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) {
			pr_err("[CAM]%s: Snapshot fail\n", __func__);
			return -EIO;
		}
		break;

	case SENSOR_GET_EXP:
		pr_err("[CAM]%s: SENSOR_GET_EXP not support\n", __func__);
		return 0;

	default:
		return -EINVAL;
	}

	return rc;
}


static int mt9v113_set_antibanding(enum antibanding_mode antibanding_value)
{
	int rc = 0;
	pr_info("[CAM]%s: antibanding_value =%d\n", __func__, antibanding_value);

	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;
	switch (antibanding_value) {
	case CAMERA_ANTI_BANDING_50HZ:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA404, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C0, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_ANTI_BANDING_60HZ:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA404, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0080, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_ANTI_BANDING_AUTO: /*default 60Mhz */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA404, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0080, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	default:
		pr_info("[CAM]%s: Not support antibanding value = %d\n",
		   __func__, antibanding_value);
		return -EINVAL;
	}
	return 0;

}

static int mt9v113_set_sharpness(enum sharpness_mode sharpness_value)
{
	int rc = 0;

	pr_info("[CAM]%s: sharpness_value = %d\n", __func__, sharpness_value);
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	switch (sharpness_value) {
	case CAMERA_SHARPNESS_X0:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB22, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0000, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x326C, 0x0400, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SHARPNESS_X1:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB22, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0001, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x326C, 0x0600, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SHARPNESS_X2:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB22, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0003, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x326C, 0x0900, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SHARPNESS_X3:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB22, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x326C, 0x0B00, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SHARPNESS_X4:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB22, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0007, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x326C, 0x0FF0, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	default:
		pr_info("[CAM]%s: Not support sharpness value = %d\n",
		   __func__, sharpness_value);
		return -EINVAL;
	}
	return 0;
}


static int mt9v113_set_saturation(enum saturation_mode saturation_value)
{
	int rc = 0;

	pr_info("[CAM]%s: saturation_value = %d\n", __func__, saturation_value);
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;
	switch (saturation_value) {
	case CAMERA_SATURATION_X0:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB20, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0010, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB24, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0009, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SATURATION_X05:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB20, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0035, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB24, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0025, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SATURATION_X1:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB20, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0048, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB24, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0033, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SATURATION_X15:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB20, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0063, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB24, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0045, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	case CAMERA_SATURATION_X2:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB20, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0076, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB24, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0053, WORD_LEN);
			if (rc < 0)
				return -EIO;

		break;
	default:
		pr_info("[CAM]%s: Not support saturation value = %d\n",
		   __func__, saturation_value);
		return -EINVAL;
	}
	return 0;
}

static int mt9v113_set_contrast(enum contrast_mode contrast_value)
{
	int rc = 0;

	pr_info("[CAM]%s: contrast_value = %d\n", __func__, contrast_value);
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	switch (contrast_value) {
	case CAMERA_CONTRAST_N2:
		rc = mt9v113_i2c_write_table(&mt9v113_regs.contract_tb0[0],
			mt9v113_regs.contract_tb0_size);
		if (rc < 0) {
			pr_err("[CAM]%s: contract_tb0 fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_CONTRAST_N1:
		rc = mt9v113_i2c_write_table(&mt9v113_regs.contract_tb1[0],
			mt9v113_regs.contract_tb1_size);
		if (rc < 0) {
			pr_err("[CAM]%s: contract_tb1 fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_CONTRAST_D:
		rc = mt9v113_i2c_write_table(&mt9v113_regs.contract_tb2[0],
			mt9v113_regs.contract_tb2_size);
		if (rc < 0) {
			pr_err("[CAM]%s: contract_tb2 fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_CONTRAST_P1:
		rc = mt9v113_i2c_write_table(&mt9v113_regs.contract_tb3[0],
			mt9v113_regs.contract_tb3_size);
		if (rc < 0) {
			pr_err("[CAM]%s: contract_tb3 fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_CONTRAST_P2:
		rc = mt9v113_i2c_write_table(&mt9v113_regs.contract_tb4[0],
			mt9v113_regs.contract_tb4_size);
		if (rc < 0) {
			pr_err("[CAM]%s: contract_tb4 fail\n", __func__);
			return -EIO;
		}

		break;
	default:
		pr_info("[CAM]%s: Not support contrast value = %d\n",
		   __func__, contrast_value);
		return -EINVAL;
	}
	return 0;
}

/* 20110103 add new effect feature */
static int pre_effect;
static int mt9v113_set_effect(int effect)
{
	int rc = 0, k = 0;
	unsigned short check_value;

	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	pr_info("[CAM]%s: effect = %d\n", __func__, effect);

	/* 20110103 add new effect feature */
	if (pre_effect == effect)
		return 0;

	switch (effect) {
	case CAMERA_EFFECT_OFF:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2759, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6440, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x275B, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6440, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2763, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0xB023, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		break;

	case CAMERA_EFFECT_MONO:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2759, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6441, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x275B, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6441, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2763, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0xB023, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		break;

	case CAMERA_EFFECT_NEGATIVE:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2759, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6443, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x275B, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6443, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2763, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0xB023, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		break;

	case CAMERA_EFFECT_SEPIA:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2759, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6442, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x275B, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6442, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2763, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0xB023, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		break;

	case CAMERA_EFFECT_AQUA:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2759, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6442, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x275B, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x6442, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0x2763, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x30D0, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);
		if (rc < 0)
			return -EIO;

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		break;
	default:
		pr_info("[CAM]%s: Not support effect = %d\n",
		   __func__, effect);
		return -EINVAL;
	}

	/* 20110103 add new effect feature */
	pre_effect = effect;

	return 0;
}

static int mt9v113_set_brightness(enum brightness_t brightness_value)
{
	int rc = 0;
	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	pr_info("[CAM]%s: brightness_value = %d\n", __func__, brightness_value);

	switch (brightness_value) {
	case CAMERA_BRIGHTNESS_N4: /* -2 */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x001E, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00CA, WORD_LEN);
			if (rc < 0)
				return -EIO;

			break;

	case CAMERA_BRIGHTNESS_N3: /* -1.5 */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0025, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C9, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case CAMERA_BRIGHTNESS_N2:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0028, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C9, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case CAMERA_BRIGHTNESS_N1:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x002E, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C9, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case CAMERA_BRIGHTNESS_D: /* default 0 */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0033, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C9, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case CAMERA_BRIGHTNESS_P1:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x003B, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C8, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case CAMERA_BRIGHTNESS_P2:
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0046, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C8, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case CAMERA_BRIGHTNESS_P3: /* 1.5 */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x004D, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C8, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	case CAMERA_BRIGHTNESS_P4: /* 2 */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA24F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0054, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xAB1F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x00C8, WORD_LEN);
		if (rc < 0)
			return -EIO;

		break;
	default:
		pr_info("[CAM]%s: Not support brightness value = %d\n",
			__func__, brightness_value);
		 return -EINVAL;
	}
	return 0;
}

static int mt9v113_set_wb(enum wb_mode wb_value)
{
	int rc = 0, k = 0;
	unsigned short check_value;

	if (op_mode == SENSOR_SNAPSHOT_MODE)
		return 0;

	pr_info("[CAM]%s: wb_value = %d\n", __func__, wb_value);
	switch (wb_value) {
	case CAMERA_AWB_AUTO:/* AUTO */
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA11F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0001, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		rc = mt9v113_i2c_write_table(&mt9v113_regs.wb_auto[0],
			mt9v113_regs.wb_auto_size);
		if (rc < 0) {
			pr_err("[CAM]%s: wb_auto fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_AWB_INDOOR_HOME:/*Fluorescent*/
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA115, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0000, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA11F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0000, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		rc = mt9v113_i2c_write_table(&mt9v113_regs.wb_fluorescent[0],
			mt9v113_regs.wb_fluorescent_size);
		if (rc < 0) {
			pr_err("[CAM]%s: wb_fluorescent fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_AWB_INDOOR_OFFICE:/*Incandescent*/
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA115, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0000, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA11F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0000, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		rc = mt9v113_i2c_write_table(&mt9v113_regs.wb_incandescent[0],
			mt9v113_regs.wb_incandescent_size);
		if (rc < 0) {
			pr_err("[CAM]%s: wb_incandescent fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_AWB_SUNNY:/*daylight*/
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA115, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0000, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA11F, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0000, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C, 0xA103, WORD_LEN);
	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0990, 0x0005, WORD_LEN);

		for (k = 0; k < CHECK_STATE_TIME; k++) {  /* retry 100 times */
			rc = mt9v113_i2c_write(mt9v113_client->addr, 0x098C,
				0xA103, WORD_LEN);
			rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0990,
				&check_value);
			if (check_value == 0x0000) /* check state of 0xA103 */
				break;
			msleep(1);
		}
		if (k == CHECK_STATE_TIME) /* time out */
			return -EIO;

		rc = mt9v113_i2c_write_table(&mt9v113_regs.wb_daylight[0],
			mt9v113_regs.wb_daylight_size);
		if (rc < 0) {
			pr_err("[CAM]%s: wb_daylight fail\n", __func__);
			return -EIO;
		}

		break;
	case CAMERA_AWB_CLOUDY: /*Not support */
	default:
		pr_info("[CAM]%s: Not support wb_value = %d\n",
		   __func__, wb_value);
		return -EINVAL;
	}
	return 0;
}


#ifdef CONFIG_MSM_CAMERA_8X60
static int mt9v113_vreg_enable(struct platform_device *pdev)
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
/*
static int mt9v113_vreg_disable(struct platform_device *pdev)
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
*/
#endif

static int mt9v113_sensor_init(void)
{
	/* uint8_t model_id_h = 0,model_id_l = 0; */
	uint16_t model_id;
	int rc = 0;

	/* Read the Model ID of the sensor */
	rc = mt9v113_i2c_read_w(mt9v113_client->addr,
			      MT9V113_MODEL_ID_ADDR, &model_id);
	if (rc < 0) {
		pr_err("[CAM]%s: I2C read fail\n", __func__);
		goto init_probe_fail;
	}

	pr_info("[CAM]%s: mt9v113: model_id = 0x%x\n", __func__, model_id);
	/* Check if it matches it with the value in Datasheet */
	if (model_id != MT9V113_MODEL_ID) {
	    pr_err("[CAM]%s: Sensor is not MT9V113\n", __func__);
		rc = -EINVAL;
		goto init_probe_fail;
	}


	return rc;
init_probe_fail:
	return rc;

}

int mt9v113_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int rc = 0;

	if (data == NULL) {
		pr_err("[CAM]%s sensor data is NULL\n", __func__);
		return -EINVAL;
	}

	mt9v113_ctrl = kzalloc(sizeof(struct mt9v113_ctrl_t), GFP_KERNEL);
	if (!mt9v113_ctrl) {
		pr_info("[CAM]mt9v113_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9v113_ctrl->sensordata = data;
	data->pdata->camera_gpio_on();
	/*switch PCLK and MCLK to 2nd cam*/
	pr_info("[CAM]mt9v113: mt9v113_sensor_open_init: switch clk\n");
	if (data->camera_clk_switch != NULL)
		data->camera_clk_switch();

	msleep(1);

	/* Configure CAM GPIO ON (CAM_MCLK)*/
	pr_info("[CAM]%s msm_camio_probe_on()\n", __func__);
	msm_camio_probe_on(mt9v113_pdev);

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	msleep(3);

#ifndef CONFIG_MSM_CAMERA_8X60
	msm_camio_camif_pad_reg_reset();
#endif

	/*read ID*/
	rc = mt9v113_sensor_init();
	if (rc < 0) {
		pr_info("[CAM]mt9v113_sensor_init failed!\n");
		goto init_fail;
	}

#ifdef CONFIG_MSM_CAMERA_8X60

#else
	/* standby mode to Active mode */
	rc = resume();
	if (rc < 0) {
		pr_err("[CAM]%s: Enter Active mode fail\n", __func__);
		goto init_fail;
	}
#endif
	config_csi = 0;
init_done:
	pr_info("[CAM]%s:----\n", __func__);
	return rc;

init_fail:
	kfree(mt9v113_ctrl);
	return rc;
}

static int mt9v113_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9v113_wait_queue);
	return 0;
}

int mt9v113_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;
	if (copy_from_user(&cfg_data,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	pr_info("[CAM]mt9v113_ioctl, cfgtype = %d, mode = %d\n",
	     cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = mt9v113_set_sensor_mode(cfg_data.mode);
		break;
	case CFG_SET_EFFECT:
		rc = mt9v113_set_effect(cfg_data.cfg.effect);
		break;
	case CFG_SET_ANTIBANDING:
		rc = mt9v113_set_antibanding(cfg_data.cfg.antibanding_value);
		break;
	case CFG_SET_BRIGHTNESS:
		rc = mt9v113_set_brightness(cfg_data.cfg.brightness_value);
		break;
	case CFG_SET_WB:
		rc = mt9v113_set_wb(cfg_data.cfg.wb_value);
		break;
	case CFG_SET_SHARPNESS:
		rc = mt9v113_set_sharpness(cfg_data.cfg.sharpness_value);
		break;
	case CFG_SET_SATURATION:
		rc = mt9v113_set_saturation(cfg_data.cfg.saturation_value);
		break;
	case CFG_SET_CONTRAST:
		rc = mt9v113_set_contrast(cfg_data.cfg.contrast_value);
		break;
	case CFG_SET_FRONT_CAMERA_MODE: /*0726 add new feature*/
		rc = mt9v113_set_front_camera_mode(cfg_data.cfg.frontcam_value);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int mt9v113_sensor_release(void)
{
	int rc = 0;
	uint16_t check_value = 0;
	struct msm_camera_sensor_info *sdata = mt9v113_pdev->dev.platform_data;

	/* enter SW standby mode */
	pr_info("[CAM]%s: enter SW standby mode\n", __func__);
	suspend();
	
	/* Do streaming Off */
	/* write 0x0016[5] to 0  */
	rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0016, &check_value);
	if (rc < 0)
	  goto sensor_release;

	pr_info("[CAM]%s: mt9v113: 0x0016 reg value = 0x%x\n",
		__func__, check_value);

	check_value = (check_value&0xFFDF);

	pr_info("[CAM]%s: mt9v113: Set to 0x0016 reg value = 0x%x\n",
		__func__, check_value);

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0016,
		check_value, WORD_LEN);
	if (rc < 0) {
		pr_err("[CAM]%s: Enter Standby mode fail\n", __func__);
		goto sensor_release;
	}

	/*0709: optical ask : CLK switch to Main Cam after 2nd Cam release*/
	if (sdata->camera_clk_switch != NULL && sdata->cam_select_pin) {
	pr_info("[CAM]%s: doing clk switch to Main CAM)\n", __func__);
	rc = gpio_request(sdata->cam_select_pin, "mt9v113");
	if (rc < 0)
		pr_err("[CAM]GPIO (%d) request fail\n", sdata->cam_select_pin);
	else
		gpio_direction_output(sdata->cam_select_pin, 0);
	gpio_free(sdata->cam_select_pin);
	}

	msleep(1);

	/* Configure CAM GPIO OFF (CAM_MCLK)*/
	pr_info("[CAM]%s msm_camio_probe_off()\n", __func__);
	msm_camio_probe_off(mt9v113_pdev);
	sdata->pdata->camera_gpio_off();

sensor_release:
	kfree(mt9v113_ctrl);
	mt9v113_ctrl = NULL;

	return rc;
}

static const char *mt9v113Vendor = "Micron";
static const char *mt9v113NAME = "mt9v113";
static const char *mt9v113Size = "VGA CMOS";
static uint32_t htcwc_value;

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", mt9v113Vendor, mt9v113NAME, mt9v113Size);
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

	htcwc_value = tmp;
	/* pr_info("[CAM]current_comm = %s\n", current->comm); */
	pr_info("[CAM]mt9v113 : htcwc_value = %d\n", htcwc_value);
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

static struct kobject *android_mt9v113;

static int mt9v113_sysfs_init(void)
{
	int ret ;
	pr_info("[CAM]mt9v113:kobject creat and add\n");
	android_mt9v113 = kobject_create_and_add("android_camera2", NULL);
	if (android_mt9v113 == NULL) {
		pr_info("[CAM]mt9v113_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("[CAM]mt9v113:sysfs_create_file\n");
	ret = sysfs_create_file(android_mt9v113, &dev_attr_sensor.attr);
	if (ret) {
		pr_info("[CAM]mt9v113_sysfs_init: sysfs_create_file " \
		"failed\n");
		kobject_del(android_mt9v113);
	}

	/* Camera AP detecte 2nd Cam*/
	ret = sysfs_create_file(android_mt9v113, &dev_attr_htcwc.attr);
	if (ret) {
		pr_info("[CAM]mt9v113_sysfs_init: sysfs_create_file htcwc failed\n");
		kobject_del(android_mt9v113);
	}

	ret = sysfs_create_file(android_mt9v113, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM]mt9v113_sysfs_init: dev_attr_node failed\n");
		kobject_del(android_mt9v113);
	}

	return 0 ;
}


static int mt9v113_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	mt9v113_sensorw = kzalloc(sizeof(struct mt9v113_work), GFP_KERNEL);

	if (!mt9v113_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9v113_sensorw);
	mt9v113_init_client(client);
	mt9v113_client = client;

	pr_info("[CAM]mt9v113_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(mt9v113_sensorw);
	mt9v113_sensorw = NULL;
	pr_info("[CAM]mt9v113_probe failed!\n");
	return rc;
}

static const struct i2c_device_id mt9v113_i2c_id[] = {
	{"mt9v113", 0},
	{},
};

static struct i2c_driver mt9v113_i2c_driver = {
	.id_table = mt9v113_i2c_id,
	.probe = mt9v113_i2c_probe,
	.remove = __exit_p(mt9v113_i2c_remove),
	.driver = {
		   .name = "mt9v113",
		   },
};

static int mt9v113_sensor_probe(struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
#ifdef CONFIG_MSM_CAMERA_8X60
	uint16_t check_value = 0;
#endif
	int rc = i2c_add_driver(&mt9v113_i2c_driver);
	if (rc < 0 || mt9v113_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	pr_info("[CAM]mt9v113 s->node %d\n", s->node);
	sensor_probe_node = s->node;

	/*switch clk source*/
	pr_info("[CAM]mt9v113: mt9v113_sensor_probe switch clk\n");
	if (info->camera_clk_switch != NULL)
		info->camera_clk_switch();

	/*Config reset */
	if (mt9v113_reset(info) < 0)
		goto probe_fail;

	/*MCLK enable*/
	pr_info("[CAM]mt9v113: MCLK enable clk\n");
	msm_camio_clk_rate_set(24000000);
	msleep(1);

    /* follow optical team Power Flow */
	rc = gpio_request(info->sensor_reset, "mt9v113");
	if (!rc) {
		rc = gpio_direction_output(info->sensor_reset, 1);
		msleep(1);
	} else
		pr_err("[CAM]GPIO(%d) request faile", info->sensor_reset);
	gpio_free(info->sensor_reset);

    msleep(2);
	/* rc = mt9v113_sensor_init(info); */
	rc = mt9v113_sensor_init();
	if (rc < 0)
		goto probe_fail;

	/*set initial register*/
	rc = mt9v113_reg_init();
	if (rc < 0) {
		pr_err("[CAM]%s: mt9v113_reg_init fail\n", __func__);
		goto probe_fail;
	}

	rc = suspend(); /* set standby mode */
	if (rc < 0) {
		pr_err("[CAM]%s: mt9v113 init_suspend fail\n", __func__);
		goto probe_fail;
	}


#ifdef CONFIG_MSM_CAMERA_8X60
	/* Do streaming Off */
	/* write 0x0016[5] to 0  */
	rc = mt9v113_i2c_read_w(mt9v113_client->addr, 0x0016, &check_value);
	if (rc < 0)
	  return rc;

	pr_info("[CAM]%s: mt9v113: 0x0016 reg value = 0x%x\n",
		__func__, check_value);

	check_value = (check_value&0xFFDF);

	pr_info("[CAM]%s: mt9v113: Set to 0x0016 reg value = 0x%x\n",
		__func__, check_value);

	rc = mt9v113_i2c_write(mt9v113_client->addr, 0x0016,
		check_value, WORD_LEN);
	if (rc < 0) {
		pr_err("[CAM]%s: Enter Standby mode fail\n", __func__);
		return rc;
	}
#endif


	s->s_init = mt9v113_sensor_open_init;
	s->s_release = mt9v113_sensor_release;
	s->s_config = mt9v113_sensor_config;

	/*init done*/
	msleep(10);

	mt9v113_sysfs_init();

probe_done:
	pr_info("[CAM]%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
probe_fail:
	pr_err("[CAM]mt9v113 probe faile\n");
	return rc;

}

static int __mt9v113_probe(struct platform_device *pdev)
{
#ifdef CONFIG_MSM_CAMERA_8X60
	int rc;
	struct msm_camera_sensor_info *sdata = pdev->dev.platform_data;
	sdata->pdata->camera_gpio_on();
	rc = mt9v113_vreg_enable(pdev);
	if (rc < 0)
		pr_err("[CAM]__mt9v113_probe fail sensor power on error\n");
#endif

	pr_info("[CAM]__mt9v113_probe\n");
	mt9v113_pdev = pdev;

	return msm_camera_drv_start(pdev, mt9v113_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9v113_probe,
	.driver = {
#ifdef CONFIG_MSM_CAMERA_8X60
		   .name = "msm_camera_webcam",
#else
		   .name = "msm_camera_mt9v113",
#endif
		   .owner = THIS_MODULE,
		   },
};

static int __init mt9v113_init(void)
{
	pr_info("[CAM]mt9v113_init\n");
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9v113_init);
