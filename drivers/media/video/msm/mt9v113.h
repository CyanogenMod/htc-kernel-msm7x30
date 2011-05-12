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



#ifndef MT9V113_H
#define MT9V113_H

#include <linux/types.h>
#include <mach/camera.h>

extern struct mt9v113_reg mt9v113_regs;

enum mt9v113_width {
	WORD_LEN,
	BYTE_LEN
};

struct mt9v113_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
	enum mt9v113_width width;
	unsigned short mdelay_time;
};

struct mt9v113_reg {
	struct mt9v113_i2c_reg_conf *power_up_tbl;
	uint16_t power_up_tbl_size;
	struct mt9v113_i2c_reg_conf *register_init_1;
	uint16_t register_init_size_1;
	struct mt9v113_i2c_reg_conf *register_init_2;
	uint16_t register_init_size_2;
	struct mt9v113_i2c_reg_conf *contract_tb0;
	uint16_t contract_tb0_size;
	struct mt9v113_i2c_reg_conf *contract_tb1;
	uint16_t contract_tb1_size;
	struct mt9v113_i2c_reg_conf *contract_tb2;
	uint16_t contract_tb2_size;
	struct mt9v113_i2c_reg_conf *contract_tb3;
	uint16_t contract_tb3_size;
	struct mt9v113_i2c_reg_conf *contract_tb4;
	uint16_t contract_tb4_size;
	struct mt9v113_i2c_reg_conf *wb_auto;
	uint16_t wb_auto_size;
	struct mt9v113_i2c_reg_conf *wb_fluorescent;
	uint16_t wb_fluorescent_size;
	struct mt9v113_i2c_reg_conf *wb_incandescent;
	uint16_t wb_incandescent_size;
	struct mt9v113_i2c_reg_conf *wb_daylight;
	uint16_t wb_daylight_size;
};

#endif /* MT9D112_H */
