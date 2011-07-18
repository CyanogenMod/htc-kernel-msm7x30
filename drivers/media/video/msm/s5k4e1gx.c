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
#include <media/msm_camera.h>
#include <media/msm_camera_sensor.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "s5k4e1gx.h"
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>

#define S5K4E1GX_REG_MODEL_ID			0x0000
#define S5K4E1GX_MODEL_ID			    0x4E10

#define S5K4E1GX_REVISION_ID			0x0002
#define S5K4E1GX_REVISION_EVT2			    0x00
#define S5K4E1GX_REVISION_EVT3			    0x10

#define S5K4E1GX_REG_MASK_CORRUPTED_FRAME	0x0105
#define S5K4E1GX_MASK_CORRUPTED_FRAME	0x01


static uint8_t sensor_evt_ver = 2;
static struct platform_device *s5k4e1_pdev;
static struct wake_lock s5k4e1gx_wake_lock;

static int sensor_probe_node = 0;
static int preview_frame_count = 0;
static int fps_mode_sel;

static inline void init_suspend(void)
{
	wake_lock_init(&s5k4e1gx_wake_lock, WAKE_LOCK_IDLE, "s5k4e1gx");
}

static inline void deinit_suspend(void)
{
	wake_lock_destroy(&s5k4e1gx_wake_lock);
}

static inline void prevent_suspend(void)
{
	wake_lock(&s5k4e1gx_wake_lock);
}

static inline void allow_suspend(void)
{
	wake_unlock(&s5k4e1gx_wake_lock);
}

/* PLL Registers */
#define REG_PRE_PLL_CLK_DIV			0x0305
#define REG_PLL_MULTIPLIER_MSB			0x0306
#define REG_PLL_MULTIPLIER_LSB			0x0307
#define REG_VT_SYS_CLK_DIV			0x30B5

/* Output Size */
#define REG_X_OUTPUT_SIZE_MSB			0x034C
#define REG_X_OUTPUT_SIZE_LSB			0x034D
#define REG_Y_OUTPUT_SIZE_MSB			0x034E
#define REG_Y_OUTPUT_SIZE_LSB			0x034F

/* Binning */
#define REG_X_EVEN_INC				0x0381
#define REG_X_ODD_INC				0x0383
#define REG_Y_EVEN_INC				0x0385
#define REG_Y_ODD_INC				0x0387

/* Reserved register */
#define REG_H_BINNING				0x30A9
#define REG_V_BINNING				0x300E

/* Frame Fotmat */
#define REG_FRAME_LENGTH_LINES_MSB		0x0340
#define REG_FRAME_LENGTH_LINES_LSB		0x0341
#define REG_LINE_LENGTH_PCK_MSB			0x0342
#define REG_LINE_LENGTH_PCK_LSB			0x0343

/* Analog Setting Register */
#define REG_CDS_TEST				0x300F
#define REG_RST_OFFSET1				0x3013
#define REG_RMP_INIT				0x3017
#define REG_COMP_BIAS				0x301B

/* Parallel Setting Register */
#define REG_DPHY_BANDCTRL			0x30F1
#define REG_PCLK_INV				0x3110
#define REG_PCLK_DELAY				0x3117
#define REG_V_H_SYNC_STRENGTH			0x3119
#define REG_DATA_PCLK_STRENGTH			0x311A

#define REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB	0x0204
#define REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB	0x0205
#define REG_COARSE_INTEGRATION_TIME_MSB   	0x0202
#define REG_COARSE_INTEGRATION_TIME_LSB   	0x0203

#define REG_FINE_INTEGRATION_TIME_MSB		0x0200
#define REG_FINE_INTEGRATION_TIME_LSB		0x0201
#define REG_COARSE_INTEGRATION_TIME_MSB		0x0202
#define REG_COARSE_INTEGRATION_TIME_LSB		0x0203

/* Mode select register */
#define S5K4E1GX_REG_MODE_SELECT		0x0100
#define S5K4E1GX_MODE_SELECT_STREAM		    0x01	/* start streaming */
#define S5K4E1GX_MODE_SELECT_SW_STANDBY		    0x00	/* software standby */

/* Read Mode */
#define S5K4E1GX_REG_READ_MODE			0x0101
#define S5K4E1GX_READ_NORMAL_MODE		    0x00	/* without mirror/flip */
#define S5K4E1GX_READ_MIRROR_FLIP		    0x03	/* with mirror/flip */

#define S5K4E1GX_REG_SOFTWARE_RESET		0x0103
#define S5K4E1GX_SOFTWARE_RESET			    0x01

#define S5K4E1GX_MIPI_REG_SOFTWARE_RESET		0x3030
#define S5K4E1GX_MIPI_SOFTWARE_RESET			    0x06


#define S5K4E1GX_REG_GROUP_PARAMETER_HOLD	0x0104
#define S5K4E1GX_GROUP_PARAMETER_HOLD		    0x01
#define S5K4E1GX_GROUP_PARAMETER_UNHOLD		    0x00

#define REG_TEST_PATTERN_MODE			0x0601

#define S5K4E1GX_AF_I2C_ADDR			0x18
#define S5K4E1GX_STEPS_NEAR_TO_CLOSEST_INF	36
#define S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR	36
#define S5K4E1GX_SW_DAMPING_STEP		10
#define S5K4E1GX_MAX_FPS			30

#define S5K4E1GX_MAX_SNAPSHOT_EXP_LC		3961

/* 1126 for MIPI */
#define REG_MIPI_LANE_MODE		0x30E2
#define MHz 1000000
#define MCLK 24

enum s5k4e1gx_reg_update_t{
	REG_INIT,
	REG_PERIODIC
};

struct reg_struct {
    // PLL Setting
	uint8_t pre_pll_clk_div;		/* 0x0305 */
	uint8_t pll_multiplier_msb;		/* 0x0306 */
	uint8_t pll_multiplier_lsb;		/* 0x0307 */
	uint8_t vt_sys_clk_div;			/* 0x30B5 */

	/* Add for MIPI */
	uint8_t MIPI_data_lane_modes;	/* 0x30E2 */

	uint8_t DPHY_bandctrl;			/* 0x30F1 */
    // Read Mode
	uint8_t read_mode;			/* 0x0101 */
    // Output Size Setting
	uint8_t x_output_size_msb;		/* 0x034C */
	uint8_t x_output_size_lsb;		/* 0x034D */
	uint8_t y_output_size_msb;		/* 0x034E */
	uint8_t y_output_size_lsb;		/* 0x034F */
	uint8_t x_even_inc;			/* 0x0381 */
	uint8_t x_odd_inc;			/* 0x0383 */
	uint8_t y_even_inc;			/* 0x0385 */
	uint8_t y_odd_inc;			/* 0x0387 */
	uint8_t h_binning;			/* 0x30A9 */
	uint8_t v_binning;			/* 0x300E */
    // Integration Setting
	uint8_t frame_length_lines_msb;		/* 0x0340 */
	uint8_t frame_length_lines_lsb;		/* 0x0341 */
	uint8_t line_length_pck_msb;		/* 0x0342 */
	uint8_t line_length_pck_lsb;		/* 0x0343 */
    // Parallel Setting
	uint8_t pclk_inv;			/* 0x3110 */
	uint8_t pclk_delay;			/* 0x3117 */
	uint8_t v_h_strength;			/* 0x3119 */
	uint8_t data_pclk_strength;		/* 0x311A */
	// Analog Setting
	uint8_t cds_test;			/* 0x300F */
	uint8_t rst_offset1;			/* 0x3013 */
	uint8_t rmp_init;			/* 0x3017 */
	uint8_t comp_bias;			/* 0x301B */

	uint8_t analogue_gain_code_global_msb;	/* 0x0204 */
	uint8_t analogue_gain_code_global_lsb;	/* 0x0205 */
	uint8_t coarse_integration_time_msb;	/* 0x0202 */
	uint8_t coarse_integration_time_lsb;	/* 0x0203 */
	uint32_t  size_h;
	uint32_t  blk_l;
	uint32_t  size_w;
	uint32_t  blk_p;
};

struct reg_struct s5k4e1gx_reg_cand[2] = {
	{/*Preview, for normal speed*/
		0x06,  /* pre_pll_clk_div               REG=0x0305 */
		0x00,  /* pll_multiplier_msb            REG=0x0306 */
		0x50,  /* pll_multiplier_lsb            REG=0x0307 */
		0x00,  /* vt_sys_clk_div                REG=0x30B5 */

		/* Add for MIPI */
		0x02,  /* MIPI_data_lane_modes          REG=0x30E2 */

		0xB0,  /* DPHY_bandctrl                 REG=0x30F1 */
		0x00,  /* read_mode                     REG=0x0101 */
		0x05,  /* x_output_size_msb             REG=0x034C */
		0x18,  /* x_output_size_lsb             REG=0x034D */
		0x03,  /* y_output_size_msb             REG=0x034E */
		0xD4,  /* y_output_size_lsb             REG=0x034F */
		0x01,  /* x_even_inc                    REG=0x0381 */
		0x01,  /* x_odd_inc                     REG=0x0383 */
		0x01,  /* y_even_inc                    REG=0x0385 */
		0x03,  /* y_odd_inc                     REG=0x0387 */
		0x02,  /* h_binning                     REG=0x30A9 */
		0xEB,  /* v_binning                     REG=0x300E */
		0x03,  /* frame_length_lines_msb        REG=0x0340 */
		0xE0,  /* frame_length_lines_lsb        REG=0x0341 */
		0x0A,  /* line_length_pck_msb           REG=0x0342 */
		0xB2,  /* line_length_pck_lsb           REG=0x0343 */
		0x10,  /* pclk_inv;                     REG=0x3110 */
		0x0C,  /* pclk_delay;                   REG=0x3117 */
		0x0A,  /* v_h_strength;                 REG=0x3119 */
		0xAA,  /* data_pclk_strength;           REG=0x311A */
		0x82,  /* cds_test                      REG=0x300F */
		0xC0,  /* rst_offset1                   REG=0x3013 */
		0xA4,  /* rmp_init                      REG=0x3017 */ /*0x94*/
		0x88,  /* comp_bias                   REG=0x301B */ /*0x83*/
		0x00,  /* analogue_gain_code_global_msb REG=0x0204 */
		0x20,  /* analogue_gain_code_global_lsb REG=0x0205 */
		0x03,  /* coarse_integration_time_msb   REG=0x0202 */
		0x1F,  /* coarse_integration_time_lsb   REG=0x0203 */
		 980,  /* size_h*/
		  12,  /* blk_l*/
		1304,  /* size_w*/
		1434   /* blk_p*/
	},
	{/*Preview, for FAST speed*/
		0x06,  /* pre_pll_clk_div               REG=0x0305 */
		0x00,  /* pll_multiplier_msb            REG=0x0306 */
		0x6A,  /* pll_multiplier_lsb            REG=0x0307 */
		0x00,  /* vt_sys_clk_div                REG=0x30B5 */

		/* Add for MIPI */
		0x02,  /* MIPI_data_lane_modes          REG=0x30E2 */

		0xD0,  /* DPHY_bandctrl                 REG=0x30F1 */
		0x00,  /* read_mode                     REG=0x0101 */
		0x05,  /* x_output_size_msb             REG=0x034C */
		0x18,  /* x_output_size_lsb             REG=0x034D */
		0x01,  /* y_output_size_msb             REG=0x034E */
		0xEA,  /* y_output_size_lsb             REG=0x034F */
		0x01,  /* x_even_inc                    REG=0x0381 */
		0x01,  /* x_odd_inc                     REG=0x0383 */
		0x01,  /* y_even_inc                    REG=0x0385 */
		0x07,  /* y_odd_inc                     REG=0x0387 */
		0x02,  /* h_binning                     REG=0x30A9 */
		0xEB,  /* v_binning                     REG=0x300E */
		0x02,  /* frame_length_lines_msb        REG=0x0340 */
		0x05,  /* frame_length_lines_lsb        REG=0x0341 */
		0x0A,  /* line_length_pck_msb           REG=0x0342 */
		0xB2,  /* line_length_pck_lsb           REG=0x0343 */
		0x00,  /* pclk_inv;                     REG=0x3110 */
		0x06,  /* pclk_delay;                   REG=0x3117 */
		0x0F,  /* v_h_strength;                 REG=0x3119 */
		0xFF,  /* data_pclk_strength;           REG=0x311A */
		0x82,  /* cds_test                      REG=0x300F */
		0xC0,  /* rst_offset1                   REG=0x3013 */
		0x94,  /* rmp_init                      REG=0x3017 */ /*0x94*/
		0x83,  /* comp_bias                   REG=0x301B */ /*0x83*/
		0x00,  /* analogue_gain_code_global_msb REG=0x0204 */
		0x80,  /* analogue_gain_code_global_lsb REG=0x0205 */
		0x01,  /* coarse_integration_time_msb   REG=0x0202 */
		0xF0,  /* coarse_integration_time_lsb   REG=0x0203 */
		 490,  /* size_h*/
		  27,  /* blk_l*/
		1304,/*size_w*/
		1434   /* blk_p*/
	}
};

/* 1126 for MIPI */
struct reg_struct s5k4e1gx_reg_zero_shutter[1] = {
	{/*Preview, for zero_shutter*/
		0x04,  /* pre_pll_clk_div               REG=0x0305 */
		0x00,  /* pll_multiplier_msb            REG=0x0306 */
		0x66,  /* pll_multiplier_lsb            REG=0x0307 */
		0x01,  /* vt_sys_clk_div                REG=0x30B5 */

		/* Add for MIPI */
		0x02,  /* MIPI_data_lane_modes          REG=0x30E2 */

		0xA0,  /* DPHY_bandctrl                 REG=0x30F1 */
		0x00,  /* read_mode                     REG=0x0101 */
		0x0A,  /* x_output_size_msb             REG=0x034C */
		0x30,  /* x_output_size_lsb             REG=0x034D */
		0x07,  /* y_output_size_msb             REG=0x034E */
		0xA8,  /* y_output_size_lsb             REG=0x034F */
		0x01,  /* x_even_inc                    REG=0x0381 */
		0x01,  /* x_odd_inc                     REG=0x0383 */
		0x01,  /* y_even_inc                    REG=0x0385 */
		0x01,  /* y_odd_inc                     REG=0x0387 */
		0x03,  /* h_binning                     REG=0x30A9 */
		0xE8,  /* v_binning                     REG=0x300E */
		0x07,  /* frame_length_lines_msb        REG=0x0340 */
		0xB4,  /* frame_length_lines_lsb        REG=0x0341 */
		0x0A,  /* line_length_pck_msb           REG=0x0342 */
		0xB2,  /* line_length_pck_lsb           REG=0x0343 */
		0x10,  /* pclk_inv;                     REG=0x3110 */
		0x0C,  /* pclk_delay;                   REG=0x3117 */
		0x0A,  /* v_h_strength;                 REG=0x3119 */
		0xAA,  /* data_pclk_strength;           REG=0x311A */
		0x82,  /* cds_test                      REG=0x300F */
		0xC0,  /* rst_offset1                   REG=0x3013 */
		0xA4,  /* rmp_init                      REG=0x3017 */ /*0x94*/
		0x88,  /* comp_bias                   REG=0x301B */ /*0x83*/
		0x00,  /* analogue_gain_code_global_msb REG=0x0204 */
		0x80,  /* analogue_gain_code_global_lsb REG=0x0205 */
		0x04,  /* coarse_integration_time_msb   REG=0x0202 */
		0x12,  /* coarse_integration_time_lsb   REG=0x0203 */
		1960,  /* size_h*/
		  12,  /* blk_l*/
		2608,  /* size_w*/
		130   /* blk_p*/
	},
};

struct reg_struct s5k4e1gx_reg_pat[2] = {
	{/*Preview*/
		0x06,  /* pre_pll_clk_div               REG=0x0305 */
		0x00,  /* pll_multiplier_msb            REG=0x0306 */
		0x50,  /* pll_multiplier_lsb            REG=0x0307 */
		0x00,  /* vt_sys_clk_div                REG=0x30B5 */

		/* Add for MIPI */
		0x02,  /* MIPI_data_lane_modes          REG=0x30E2 */

		0xB0,  /* DPHY_bandctrl                 REG=0x30F1 */
		0x00,  /* read_mode                     REG=0x0101 */
		0x05,  /* x_output_size_msb             REG=0x034C */
		0x18,  /* x_output_size_lsb             REG=0x034D */
		0x03,  /* y_output_size_msb             REG=0x034E */
		0xD4,  /* y_output_size_lsb             REG=0x034F */
		0x01,  /* x_even_inc                    REG=0x0381 */
		0x01,  /* x_odd_inc                     REG=0x0383 */
		0x01,  /* y_even_inc                    REG=0x0385 */
		0x03,  /* y_odd_inc                     REG=0x0387 */
		0x02,  /* h_binning                     REG=0x30A9 */
		0xEB,  /* v_binning                     REG=0x300E */
		0x03,  /* frame_length_lines_msb        REG=0x0340 */
		0xE0,  /* frame_length_lines_lsb        REG=0x0341 */
		0x0A,  /* line_length_pck_msb           REG=0x0342 */
		0xB2,  /* line_length_pck_lsb           REG=0x0343 */
		0x10,  /* pclk_inv;                     REG=0x3110 */
		0x0C,  /* pclk_delay;                   REG=0x3117 */
		0x0A,  /* v_h_strength;                 REG=0x3119 */
		0xAA,  /* data_pclk_strength;           REG=0x311A */
		0x82,  /* cds_test                      REG=0x300F */
		0xC0,  /* rst_offset1                   REG=0x3013 */
		0xA4,  /* rmp_init                      REG=0x3017 */ /*0x94*/
		0x88,  /* comp_bias                   REG=0x301B */ /*0x83*/
		0x00,  /* analogue_gain_code_global_msb REG=0x0204 */
		0x20,  /* analogue_gain_code_global_lsb REG=0x0205 */
		0x03,  /* coarse_integration_time_msb   REG=0x0202 */
		0x1F,  /* coarse_integration_time_lsb   REG=0x0203 */
		 980,  /* size_h*/
		  12,  /* blk_l*/
		1304,  /* size_w*/
		1434   /* blk_p*/
	},
	{ /*Snapshot*/
		0x06,  /* pre_pll_clk_div               REG=0x0305 */
		0x00,  /* pll_multiplier_msb            REG=0x0306 */
		0x50,  /* pll_multiplier_lsb            REG=0x0307 */
		0x00,  /* vt_sys_clk_div                REG=0x30B5 */

		/* Add for MIPI */
		0x02,  /* MIPI_data_lane_modes          REG=0x30E2 */

		0xB0,  /* DPHY_bandctrl                 REG=0x30F1 */
		0x00,  /* read_mode                     REG=0x0101 */
		0x0A,  /* x_output_size_msb             REG=0x034C */
		0x30,  /* x_output_size_lsb             REG=0x034D */
		0x07,  /* y_output_size_msb             REG=0x034E */
		0xA8,  /* y_output_size_lsb             REG=0x034F */
		0x01,  /* x_even_inc                    REG=0x0381 */
		0x01,  /* x_odd_inc                     REG=0x0383 */
		0x01,  /* y_even_inc                    REG=0x0385 */
		0x01,  /* y_odd_inc                     REG=0x0387 */
		0x03,  /* h_binning                     REG=0x30A9 */
		0xE8,  /* v_binning                     REG=0x300E */
		0x07,  /* frame_length_lines_msb        REG=0x0340 */
		0xB4,  /* frame_length_lines_lsb        REG=0x0341 */
		0x0A,  /* line_length_pck_msb           REG=0x0342 */
		0xB2,  /* line_length_pck_lsb           REG=0x0343 */
		0x10,  /* pclk_inv;                     REG=0x3110 */
		0x0C,  /* pclk_delay;                   REG=0x3117 */
		0x0A,  /* v_h_strength;                 REG=0x3119 */
		0xAA,  /* data_pclk_strength;           REG=0x311A */
		0x82,  /* cds_test	                REG=0x300F */
		0xC0,  /* rst_offset1			REG=0x3013 */
		0xA4,  /* rmp_init			REG=0x3017 */	/*0x94*/
		0x88,  /* comp_bias			REG=0x301B */	/*0x71*/
		0x00,  /* analogue_gain_code_global_msb REG=0x0204 */
		0x80,  /* analogue_gain_code_global_lsb REG=0x0205 */
		0x07,  /* coarse_integration_time_msb   REG=0x0202 */
		0xA8,  /* coarse_integration_time_lsb   REG=0x0203 */
		1960,  /* size_h */
		  12,  /* blk_l*/
		2608,  /* size_w*/
		 130   /* blk_p*/
	}
};

struct s5k4e1gx_work {
	struct work_struct work;
};
static struct s5k4e1gx_work *s5k4e1gx_sensorw;
static struct i2c_client *s5k4e1gx_client;
static uint16_t s5k4e1gx_pos_tbl[S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR+1];

struct s5k4e1gx_ctrl {
	const struct msm_camera_sensor_info *sensordata;

	int sensormode;
	uint32_t fps_divider; 		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; 	/* init to 1 * 0x00000400 */
	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum msm_s_resolution prev_res;
	enum msm_s_resolution pict_res;
	enum msm_s_resolution curr_res;
	enum msm_s_test_mode  set_test;
	enum s5k4e1gx_reg_update_t reg_update;
};

static struct s5k4e1gx_ctrl *s5k4e1gx_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(s5k4e1gx_wait_queue);
DEFINE_MUTEX(s5k4e1gx_mutex);


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


int s5k4e1gx_i2c_lens_tx_data(unsigned char slave_addr, char *txData, int length)
{
	int rc;
	struct i2c_msg msg[] = {
		{
			.addr = slave_addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	rc = i2c_transfer_retry(s5k4e1gx_client->adapter, msg, 1);
	if (rc < 0) {
		printk(KERN_ERR "s5k4e1gx_i2c_lens_tx_data: i2c_transfer error %d\n", rc);
		return rc;
	}
	return 0;
}

static int s5k4e1gx_i2c_lens_write(unsigned char slave_addr, unsigned char u_addr, unsigned char u_data)
{
	unsigned char buf[2] = { u_addr, u_data };
	return s5k4e1gx_i2c_lens_tx_data(slave_addr, buf, sizeof(buf));
}

/*#define I2C_AF_WRITE(command, data) if (s5k4e1gx_i2c_lens_write(AF_I2C_ID >> 1, command, data) < 0) return -EIO;*/
#define I2C_AF_WRITE(command, data) s5k4e1gx_i2c_lens_write(S5K4E1GX_AF_I2C_ADDR >> 1, command, data);

static int s5k4e1gx_i2c_rxdata(unsigned short saddr, unsigned char *rxdata,
	int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr   = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr   = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};

	if (i2c_transfer_retry(s5k4e1gx_client->adapter, msgs, 2) < 0) {
		pr_err("[CAM]s5k4e1gx_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k4e1gx_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
		.addr  = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer_retry(s5k4e1gx_client->adapter, msg, 1) < 0) {
		pr_err("[CAM]s5k4e1gx_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t s5k4e1gx_i2c_read_b(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k4e1gx_i2c_rxdata(saddr, buf, 1);
	if (rc < 0)
		return rc;

	*rdata = buf[0];

	if (rc < 0)
		printk("[CAM]s5k4e1gx_i2c_read failed!\n");

	return rc;
}

static int32_t s5k4e1gx_i2c_write_b(unsigned short saddr, unsigned short waddr,
	unsigned char bdata)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	rc = s5k4e1gx_i2c_txdata(saddr, buf, 3);

	if (rc < 0)
		pr_err("[CAM]i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			 waddr, bdata);

	return rc;
}

static int32_t s5k4e1gx_i2c_write_table(
	struct s5k4e1gx_i2c_reg_conf *reg_cfg_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
			reg_cfg_tbl->waddr, reg_cfg_tbl->bdata);
		if (rc < 0)
			break;
		reg_cfg_tbl++;
	}

	return rc;
}

static int32_t s5k4e1gx_i2c_read_w(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = s5k4e1gx_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("[CAM]s5k4e1gx_i2c_read failed!\n");

	return rc;
}

static int s5k4e1gx_probe_init_done(const struct msm_camera_sensor_info *data)
{
	gpio_free(data->sensor_reset);
	return 0;
}

static int s5k4e1gx_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t  rc;
	uint16_t chipid = 0;
	//unsigned short i, R1, R2, R3;
	unsigned short evt_ver, reg_status;

	struct msm_camera_sensor_info *sdata = s5k4e1_pdev->dev.platform_data;

	rc = gpio_request(data->sensor_reset, "s5k4e1gx");
	if (!rc) {
		gpio_direction_output(data->sensor_reset, 0);
		mdelay(5);
		gpio_direction_output(data->sensor_reset, 1);
	}
	else
		goto init_probe_done;

	mdelay(20);

	pr_info("[CAM]s5k4e1gx_sensor_init(): reseting sensor.\n");

	rc = s5k4e1gx_i2c_read_w(s5k4e1gx_client->addr,
		S5K4E1GX_REG_MODEL_ID, &chipid);
	if (rc < 0)
		goto init_probe_fail;

	if (chipid != S5K4E1GX_MODEL_ID) {
		pr_err("[CAM]S5K4E1GX wrong model_id = 0x%x\n", chipid);
		rc = -ENODEV;
		goto init_probe_fail;
	}

	/*Reset sensor*/
	if (sdata->csi_if) {
		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
			S5K4E1GX_MIPI_REG_SOFTWARE_RESET, S5K4E1GX_MIPI_SOFTWARE_RESET);
	} else {
		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
			S5K4E1GX_REG_SOFTWARE_RESET, S5K4E1GX_SOFTWARE_RESET);
	}

	if (rc < 0)
		goto init_probe_fail;

	/* Add Lens Correction Common Setting For Maverick*/
	if (!sdata->sensor_lc_disable) {
		pr_info("[CAM]sensor_lc_disable=%d\n", sdata->sensor_lc_disable);

		rc = s5k4e1gx_i2c_write_table(s5k4e1gx_regs.lc_common,
			s5k4e1gx_regs.lc_common_size);
		if (rc < 0)
			goto init_probe_fail;
	}

    /* Add analog settings For Maverick*/
	rc = s5k4e1gx_i2c_read_b(s5k4e1gx_client->addr, S5K4E1GX_REVISION_ID, &evt_ver);
	if (!(rc < 0)) {
		printk("[CAM]sensor evt version : 0x%x\n", evt_ver);
		if (evt_ver == S5K4E1GX_REVISION_EVT3)
			sensor_evt_ver = 3;
	}



    /* Modified the Setting for different sensor revision */
	if (machine_is_saga()) {
		printk("[CAM]use analog_settings_saga\n");
		if (sdata->zero_shutter_mode)
		rc = s5k4e1gx_i2c_write_table(
			s5k4e1gx_regs.analog_settings_saga_zero_shutter,
			s5k4e1gx_regs.analog_settings_saga_zero_shutter_size);
		else
		rc = s5k4e1gx_i2c_write_table(s5k4e1gx_regs.analog_settings_saga,
				s5k4e1gx_regs.analog_settings_saga_size);
		if (rc < 0)
			goto init_probe_fail;


		s5k4e1gx_reg_cand[0].comp_bias = 0x83;
		s5k4e1gx_reg_cand[1].comp_bias = 0x83;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].comp_bias = 0x83;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].comp_bias = 0x75;

		s5k4e1gx_reg_cand[0].pll_multiplier_lsb = 0x65;
		s5k4e1gx_reg_cand[1].pll_multiplier_lsb = 0x65;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].pll_multiplier_lsb = 0x65;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].pll_multiplier_lsb = 0x65;

		s5k4e1gx_reg_cand[0].vt_sys_clk_div = 0x01;
		s5k4e1gx_reg_cand[1].vt_sys_clk_div = 0x01;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].vt_sys_clk_div = 0x01;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].vt_sys_clk_div = 0x01;

		s5k4e1gx_reg_cand[0].DPHY_bandctrl = 0xA0;
		s5k4e1gx_reg_cand[1].DPHY_bandctrl = 0xA0;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].DPHY_bandctrl = 0xA0;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].DPHY_bandctrl = 0xA0;
	} else if (sensor_evt_ver == 3) {
		printk("[CAM]use analog_settings_evt3\n");
		rc = s5k4e1gx_i2c_write_table(s5k4e1gx_regs.analog_settings_evt3,
				s5k4e1gx_regs.analog_settings_evt3_size);
		if (rc < 0)
			goto init_probe_fail;
	} else {
		printk("[CAM]use analog_settings_evt2\n");
		rc = s5k4e1gx_i2c_write_table(s5k4e1gx_regs.analog_settings_evt2,
				s5k4e1gx_regs.analog_settings_evt2_size);
		if (rc < 0)
			goto init_probe_fail;

	    // Preview Analog Setting for EVT2
		s5k4e1gx_reg_pat[S_RES_PREVIEW].DPHY_bandctrl   = 0xD0;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].cds_test 	= 0x00;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].rst_offset1 	= 0x90;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].rmp_init	= 0x84;
		s5k4e1gx_reg_pat[S_RES_PREVIEW].comp_bias	= 0x77;

		s5k4e1gx_reg_cand[0].DPHY_bandctrl   = 0xD0;
		s5k4e1gx_reg_cand[1].DPHY_bandctrl   = 0xD0;
		s5k4e1gx_reg_cand[0].cds_test 	= 0x00;
		s5k4e1gx_reg_cand[1].cds_test 	= 0x00;
		s5k4e1gx_reg_cand[0].rst_offset1 	= 0x90;
		s5k4e1gx_reg_cand[1].rst_offset1 	= 0x90;
		s5k4e1gx_reg_cand[0].rmp_init	= 0x84;
		s5k4e1gx_reg_cand[1].rmp_init	= 0x84;
		s5k4e1gx_reg_cand[0].comp_bias	= 0x77;
		s5k4e1gx_reg_cand[1].comp_bias	= 0x77;

	    // Snapshot Analog Setting for EVT2
		s5k4e1gx_reg_pat[S_RES_CAPTURE].DPHY_bandctrl   = 0xD0;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].cds_test 	= 0x00;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].rst_offset1 	= 0xA0;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].rmp_init	= 0x94;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].comp_bias	= 0x77;
	}


	s5k4e1gx_reg_pat[S_RES_PREVIEW].v_h_strength = 0x0F; /* 0x0A; */
	s5k4e1gx_reg_pat[S_RES_CAPTURE].v_h_strength = 0x0F; /* 0x0A; */
	s5k4e1gx_reg_pat[S_RES_PREVIEW].data_pclk_strength = 0xFA; /* 0xEA; */
	s5k4e1gx_reg_pat[S_RES_CAPTURE].data_pclk_strength = 0xFA; /* 0xEA; */
	s5k4e1gx_reg_pat[S_RES_PREVIEW].read_mode = 0;
	s5k4e1gx_reg_pat[S_RES_CAPTURE].read_mode = 0;

	s5k4e1gx_reg_cand[0].v_h_strength = 0x0F; /* 0x0A; */
	s5k4e1gx_reg_cand[1].v_h_strength = 0x0F; /* 0x0A; */
	s5k4e1gx_reg_cand[0].data_pclk_strength = 0xFA; /* 0xEA; */
	s5k4e1gx_reg_cand[1].data_pclk_strength = 0xFA; /* 0xEA; */
	s5k4e1gx_reg_cand[0].read_mode = 0;
	s5k4e1gx_reg_cand[1].read_mode = 0;

  if (machine_is_lexikon()) {
    s5k4e1gx_reg_pat[S_RES_PREVIEW].read_mode = S5K4E1GX_READ_MIRROR_FLIP;
    s5k4e1gx_reg_pat[S_RES_CAPTURE].read_mode = S5K4E1GX_READ_MIRROR_FLIP;
    s5k4e1gx_reg_cand[0].read_mode = S5K4E1GX_READ_MIRROR_FLIP;
    s5k4e1gx_reg_cand[1].read_mode = S5K4E1GX_READ_MIRROR_FLIP;
  }

#ifdef CONFIG_ARCH_MSM7227
    /* Individual Setting for Each Project */
	if (machine_is_latte()) {
		s5k4e1gx_reg_pat[S_RES_PREVIEW].pclk_delay = 0x0E; 	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_pat[S_RES_CAPTURE].pclk_delay = 0x0E; 	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_pat[S_RES_PREVIEW].v_h_strength 	= 0x0F;	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_pat[S_RES_CAPTURE].v_h_strength 	= 0x0F;	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_pat[S_RES_PREVIEW].data_pclk_strength = 0xFA;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].data_pclk_strength = 0xFA;

		s5k4e1gx_reg_cand[0].pclk_delay = 0x0E; 	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_cand[1].pclk_delay = 0x0E; 	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_cand[0].v_h_strength 	= 0x0F;	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_cand[1].v_h_strength 	= 0x0F;	/* Kevin add to enhance setup time */
		s5k4e1gx_reg_cand[0].data_pclk_strength = 0xFA;
		s5k4e1gx_reg_cand[1].data_pclk_strength = 0xFA;
	}

	if (machine_is_liberty()) {
		s5k4e1gx_reg_pat[S_RES_PREVIEW].v_h_strength = 0x0F; /* 0x0A; */
		s5k4e1gx_reg_pat[S_RES_CAPTURE].v_h_strength = 0x0F; /* 0x0A; */
		s5k4e1gx_reg_pat[S_RES_PREVIEW].data_pclk_strength = 0xFA; /* 0xEA; */
		s5k4e1gx_reg_pat[S_RES_CAPTURE].data_pclk_strength = 0xFA; /* 0xEA; */
		s5k4e1gx_reg_pat[S_RES_PREVIEW].read_mode = S5K4E1GX_READ_MIRROR_FLIP;
		s5k4e1gx_reg_pat[S_RES_CAPTURE].read_mode = S5K4E1GX_READ_MIRROR_FLIP;

		s5k4e1gx_reg_cand[0].v_h_strength = 0x0F; /* 0x0A; */
		s5k4e1gx_reg_cand[1].v_h_strength = 0x0F; /* 0x0A; */
		s5k4e1gx_reg_cand[0].data_pclk_strength = 0xFA; /* 0xEA; */
		s5k4e1gx_reg_cand[1].data_pclk_strength = 0xFA; /* 0xEA; */
		s5k4e1gx_reg_cand[0].read_mode = S5K4E1GX_READ_MIRROR_FLIP;
		s5k4e1gx_reg_cand[1].read_mode = S5K4E1GX_READ_MIRROR_FLIP;
	}
#endif

	if (data->camera_clk_switch != NULL) {
	rc = s5k4e1gx_i2c_read_b(s5k4e1gx_client->addr, 0x3110, &reg_status);
		if (rc < 0) {
			pr_info("[CAM]%s: 0x3110 read_b fail\n", __func__);
			goto init_probe_fail;
		}

		reg_status = (reg_status|0x01); /* set 0x3110[0] as high */

		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
				0x3110, reg_status);

		if (rc < 0) {
			pr_info("[CAM]%s: 0x3110 write_b fail\n", __func__);
			goto init_probe_fail;
		}
	}

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
			S5K4E1GX_REG_MODE_SELECT, S5K4E1GX_MODE_SELECT_SW_STANDBY);
	if (rc < 0)
		goto init_probe_fail;

	goto init_probe_done;

init_probe_fail:
	s5k4e1gx_probe_init_done(data);
init_probe_done:
	return rc;
}

static int s5k4e1gx_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k4e1gx_wait_queue);
	return 0;
}

static const struct i2c_device_id s5k4e1gx_i2c_id[] = {
	{ "s5k4e1gx", 0},
	{ }
};

static void s5k4e1gx_setup_af_tbl(void)
{
	int i;
	uint16_t s5k4e1gx_nl_region_boundary1 = 3;
	uint16_t s5k4e1gx_nl_region_boundary2 = 5;
	uint16_t s5k4e1gx_nl_region_code_per_step1 = 40;
	uint16_t s5k4e1gx_nl_region_code_per_step2 = 20;
	uint16_t s5k4e1gx_l_region_code_per_step = 12; /* 10 */  /* 20100519 modify for VCM full 10cm focus. */

	s5k4e1gx_pos_tbl[0] = 0;

	for(i=1; i <= S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR; i++){
		 if ( i <= s5k4e1gx_nl_region_boundary1)
			 s5k4e1gx_pos_tbl[i] = s5k4e1gx_pos_tbl[i-1] +
				 s5k4e1gx_nl_region_code_per_step1;
		 else if ( i <= s5k4e1gx_nl_region_boundary2)
			 s5k4e1gx_pos_tbl[i] = s5k4e1gx_pos_tbl[i-1] +
				 s5k4e1gx_nl_region_code_per_step2;
		 else
			 s5k4e1gx_pos_tbl[i] = s5k4e1gx_pos_tbl[i-1] +
				 s5k4e1gx_l_region_code_per_step;
	}
}

static int s5k4e1gx_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_info("[CAM]s5k4e1gx_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("[CAM]i2c_check_functionality failed\n");
		goto probe_failure;
	}

	s5k4e1gx_sensorw = kzalloc(sizeof(struct s5k4e1gx_work), GFP_KERNEL);
	if (!s5k4e1gx_sensorw) {
		pr_err("[CAM]kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k4e1gx_sensorw);
	s5k4e1gx_init_client(client);
	s5k4e1gx_client = client;

	mdelay(50);

	pr_info("[CAM]s5k4e1gx_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	pr_err("[CAM]s5k4e1gx_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit s5k4e1gx_i2c_remove(struct i2c_client *client)
{
	struct s5k4e1gx_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	deinit_suspend();
	s5k4e1gx_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver s5k4e1gx_i2c_driver = {
	.id_table = s5k4e1gx_i2c_id,
	.probe  = s5k4e1gx_i2c_probe,
	.remove = __exit_p(s5k4e1gx_i2c_remove),
	.driver = {
		.name = "s5k4e1gx",
	},
};

static int32_t s5k4e1gx_test(enum msm_s_test_mode mo)
{
	int32_t rc = 0;

	if (mo == S_TEST_OFF)
		rc = 0;
	else
		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
			REG_TEST_PATTERN_MODE, (uint16_t)mo);

	return rc;
}



static void s5k4e1gx_extra_settings_for_mipi(enum msm_s_setting rt)
{
	if (machine_is_saga()) {
		if (rt == S_RES_PREVIEW) {
			/* outif_enable[7], data_type[5:0](2Bh = bayer 10bit) */
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30BF, 0xAB);
			/* video_offset[7:4] 3260%12 */
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30C0, 0xA0);
			/* video_data_length 1600 = 1304 * 1.25 */
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30C8, 0x06);
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30C9, 0x5E);

			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30E3, 0x38);
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30E4, 0x40);
		} else {
			/* outif_enable[7], data_type[5:0](2Bh = bayer 10bit) */
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30BF, 0xAB);
			/* video_offset[7:4] 3260%12 */
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30C0, 0x80);
			/* video_data_length 3260 = 2608 * 1.25 */
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30C8, 0x0C);
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30C9, 0xBC);

			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30E3, 0x19);
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30E4, 0x64);
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30E5, 0xF0);
		}
	}
}


static int32_t s5k4e1gx_setting(enum msm_s_reg_update rupdate,
				enum msm_s_setting rt)
{
	int32_t rc = 0;
	uint16_t num_lperf;
	struct msm_camera_csi_params s5k4e1gx_csi_params;
	struct msm_camera_sensor_info *sdata = s5k4e1_pdev->dev.platform_data;

	if (sdata->csi_if) {
		if (s5k4e1gx_ctrl->reg_update == S_REG_INIT) {
			/* config mipi csi controller */
			s5k4e1gx_csi_params.data_format = CSI_10BIT;
			s5k4e1gx_csi_params.lane_cnt = 2;
			s5k4e1gx_csi_params.lane_assign = 0xe4;
			s5k4e1gx_csi_params.dpcm_scheme = 0;
			s5k4e1gx_csi_params.settle_cnt = 30;/*0120: QCT //20;*/
			rc = msm_camio_csi_config(&s5k4e1gx_csi_params);

			s5k4e1gx_ctrl->reg_update = S_UPDATE_PERIODIC;
		}
	}

	switch (rupdate) {
	case S_UPDATE_PERIODIC:
		/* 1126 for improve shutter of MIPI */
		if (machine_is_saga() && sdata->zero_shutter_mode) {
		pr_info("[CAM]%s:return 0 (S_UPDATE_PERIODIC state)\n", __func__);
			return 0;
		}

		if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {
			struct s5k4e1gx_i2c_reg_conf tbl_1[] = {
			{
			    /* Standby */
				S5K4E1GX_REG_MODE_SELECT,
					S5K4E1GX_MODE_SELECT_SW_STANDBY},
			    /* Output Size */
				{REG_X_OUTPUT_SIZE_MSB,
					s5k4e1gx_reg_pat[rt].x_output_size_msb},
				{REG_X_OUTPUT_SIZE_LSB,
					s5k4e1gx_reg_pat[rt].x_output_size_lsb},
				{REG_Y_OUTPUT_SIZE_MSB,
					s5k4e1gx_reg_pat[rt].y_output_size_msb},
				{REG_Y_OUTPUT_SIZE_LSB,
					s5k4e1gx_reg_pat[rt].y_output_size_lsb},
			    /* Binning */
				{REG_X_EVEN_INC,
					s5k4e1gx_reg_pat[rt].x_even_inc},
				{REG_X_ODD_INC,
					s5k4e1gx_reg_pat[rt].x_odd_inc},
				{REG_Y_EVEN_INC,
					s5k4e1gx_reg_pat[rt].y_even_inc},
				{REG_Y_ODD_INC,
					s5k4e1gx_reg_pat[rt].y_odd_inc},
				{REG_H_BINNING,
					s5k4e1gx_reg_pat[rt].h_binning},
				{REG_V_BINNING,
					s5k4e1gx_reg_pat[rt].v_binning},
			    /* Additional */
 				{S5K4E1GX_REG_MASK_CORRUPTED_FRAME,
					S5K4E1GX_MASK_CORRUPTED_FRAME}

			};

			struct s5k4e1gx_i2c_reg_conf tbl_2[] = {
				{REG_FRAME_LENGTH_LINES_MSB,
					0},
				{REG_FRAME_LENGTH_LINES_LSB,
					0},
			    /* Analog setting */
				{REG_CDS_TEST,
					s5k4e1gx_reg_pat[rt].cds_test},
				{REG_RST_OFFSET1,
					s5k4e1gx_reg_pat[rt].rst_offset1},
				{REG_RMP_INIT,
					s5k4e1gx_reg_pat[rt].rmp_init},
				{REG_COMP_BIAS,
					s5k4e1gx_reg_pat[rt].comp_bias},
			    /* CLK/DATA SYNC setting */
				{REG_PCLK_INV,
					s5k4e1gx_reg_pat[rt].pclk_inv},
				{REG_PCLK_DELAY,
					s5k4e1gx_reg_pat[rt].pclk_delay},
				{REG_V_H_SYNC_STRENGTH,
					s5k4e1gx_reg_pat[rt].v_h_strength},
				{REG_DATA_PCLK_STRENGTH,
					s5k4e1gx_reg_pat[rt].data_pclk_strength},

			    /* Streaming ON*/
#ifndef CONFIG_MSM_CAMERA_7X30
			{S5K4E1GX_REG_MODE_SELECT, S5K4E1GX_MODE_SELECT_STREAM},
#endif
			};

			rc = s5k4e1gx_i2c_write_table(&tbl_1[0],
				ARRAY_SIZE(tbl_1));
			if (rc < 0)
				return rc;

			if (sdata->csi_if) {
				s5k4e1gx_extra_settings_for_mipi(rt);
			}

			/* Lens Correction for Preview or Capture */
		if (!sdata->sensor_lc_disable) {
		pr_info("[CAM]sensor_lc_disable=%d\n", sdata->sensor_lc_disable);
			if (rt == S_RES_PREVIEW) {
			rc = s5k4e1gx_i2c_write_table(s5k4e1gx_regs.lc_preview,
					s5k4e1gx_regs.lc_preview_size);
				if (rc < 0)
					return rc;
			} else {
			rc = s5k4e1gx_i2c_write_table(s5k4e1gx_regs.lc_capture,
					s5k4e1gx_regs.lc_capture_size);
				if (rc < 0)
					return rc;
			}
		}

			num_lperf = (uint16_t)
				((s5k4e1gx_reg_pat[rt].frame_length_lines_msb << 8)
				& 0xFF00)
				+ s5k4e1gx_reg_pat[rt].frame_length_lines_lsb;

			num_lperf = num_lperf * s5k4e1gx_ctrl->fps_divider / 0x0400;

			tbl_2[0] = (struct s5k4e1gx_i2c_reg_conf)
				{REG_FRAME_LENGTH_LINES_MSB, (num_lperf & 0xFF00) >> 8};
			tbl_2[1] = (struct s5k4e1gx_i2c_reg_conf)
				{REG_FRAME_LENGTH_LINES_LSB, (num_lperf & 0x00FF)};

			rc = s5k4e1gx_i2c_write_table(&tbl_2[0],
				ARRAY_SIZE(tbl_2));
			if (rc < 0)
				return rc;
#ifdef CONFIG_MSM_CAMERA_7X30
			/*only streaming on in preview mode*/
			if(rt == S_RES_PREVIEW){
				mdelay(200);
				s5k4e1gx_i2c_write_b(
					s5k4e1gx_client->addr,
					S5K4E1GX_REG_MODE_SELECT,
					S5K4E1GX_MODE_SELECT_STREAM);
			} else {
				mdelay(100);
			}
#endif
			mdelay(5);
			rc = s5k4e1gx_test(s5k4e1gx_ctrl->set_test);
			if (rc < 0)
				return rc;
		}
		break; /* UPDATE_PERIODIC */

	case S_REG_INIT:

		if (rt == S_RES_PREVIEW || rt == S_RES_CAPTURE) {
			struct s5k4e1gx_i2c_reg_conf tbl_3[] =
			{
			     /* PLL setting */
				{REG_PRE_PLL_CLK_DIV,
					s5k4e1gx_reg_pat[rt].pre_pll_clk_div},
				{REG_PLL_MULTIPLIER_MSB,
					s5k4e1gx_reg_pat[rt].pll_multiplier_msb},
				{REG_PLL_MULTIPLIER_LSB,
					s5k4e1gx_reg_pat[rt].pll_multiplier_lsb},
				{REG_VT_SYS_CLK_DIV,
					s5k4e1gx_reg_pat[rt].vt_sys_clk_div},
				{REG_DPHY_BANDCTRL,
					s5k4e1gx_reg_pat[rt].DPHY_bandctrl},

			    /* Read Mode Setting */
				{S5K4E1GX_REG_READ_MODE,
					s5k4e1gx_reg_pat[rt].read_mode},

			    /* Output Size */
				{REG_X_OUTPUT_SIZE_MSB,
					s5k4e1gx_reg_pat[rt].x_output_size_msb},
				{REG_X_OUTPUT_SIZE_LSB,
					s5k4e1gx_reg_pat[rt].x_output_size_lsb},
				{REG_Y_OUTPUT_SIZE_MSB,
					s5k4e1gx_reg_pat[rt].y_output_size_msb},
				{REG_Y_OUTPUT_SIZE_LSB,
					s5k4e1gx_reg_pat[rt].y_output_size_lsb},

			    /* Binning */
				{REG_X_EVEN_INC,
					s5k4e1gx_reg_pat[rt].x_even_inc},
				{REG_X_ODD_INC,
					s5k4e1gx_reg_pat[rt].x_odd_inc },
				{REG_Y_EVEN_INC,
					s5k4e1gx_reg_pat[rt].y_even_inc},
				{REG_Y_ODD_INC,
					s5k4e1gx_reg_pat[rt].y_odd_inc},
				{REG_H_BINNING,
					s5k4e1gx_reg_pat[rt].h_binning},
				{REG_V_BINNING,
					s5k4e1gx_reg_pat[rt].v_binning},

			    /* Frame format */
				{REG_FRAME_LENGTH_LINES_MSB,
					s5k4e1gx_reg_pat[rt].frame_length_lines_msb},
				{REG_FRAME_LENGTH_LINES_LSB,
					s5k4e1gx_reg_pat[rt].frame_length_lines_lsb},
				{REG_LINE_LENGTH_PCK_MSB,
					s5k4e1gx_reg_pat[rt].line_length_pck_msb},
				{REG_LINE_LENGTH_PCK_LSB,
					s5k4e1gx_reg_pat[rt].line_length_pck_lsb},


			    /* CLK/DATA SYNC setting */
				{REG_PCLK_INV,
					s5k4e1gx_reg_pat[rt].pclk_inv},
				{REG_PCLK_DELAY,
					s5k4e1gx_reg_pat[rt].pclk_delay},
				{REG_V_H_SYNC_STRENGTH,
					s5k4e1gx_reg_pat[rt].v_h_strength},
				{REG_DATA_PCLK_STRENGTH,
					s5k4e1gx_reg_pat[rt].data_pclk_strength},
			    /* Parameter Hold */
				{S5K4E1GX_REG_GROUP_PARAMETER_HOLD,
					S5K4E1GX_GROUP_PARAMETER_HOLD},
			    /* Integration Setting */
				{REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB,
					s5k4e1gx_reg_pat[rt].analogue_gain_code_global_msb},
				{REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB,
					s5k4e1gx_reg_pat[rt].analogue_gain_code_global_lsb},
				{REG_COARSE_INTEGRATION_TIME_MSB,
					s5k4e1gx_reg_pat[rt].coarse_integration_time_msb},
				{REG_COARSE_INTEGRATION_TIME_LSB,
					s5k4e1gx_reg_pat[rt].coarse_integration_time_lsb},
			    /* Parameter Unhold */
				{S5K4E1GX_REG_GROUP_PARAMETER_HOLD,
					S5K4E1GX_GROUP_PARAMETER_UNHOLD},

			};	/* end of struct s5k4e1gx_i2c_reg_conf tbl_3[] */

			/* 1126 for improve shutter of MIPI */
		struct s5k4e1gx_i2c_reg_conf tbl_4[] =
		{
			     /* PLL setting */
			{REG_PRE_PLL_CLK_DIV,
			s5k4e1gx_reg_zero_shutter[rt].pre_pll_clk_div},
			{REG_PLL_MULTIPLIER_MSB,
			s5k4e1gx_reg_zero_shutter[rt].pll_multiplier_msb},
			{REG_PLL_MULTIPLIER_LSB,
			s5k4e1gx_reg_zero_shutter[rt].pll_multiplier_lsb},
			{REG_VT_SYS_CLK_DIV,
			s5k4e1gx_reg_zero_shutter[rt].vt_sys_clk_div},
			{REG_MIPI_LANE_MODE,
			s5k4e1gx_reg_zero_shutter[rt].MIPI_data_lane_modes},
			{REG_DPHY_BANDCTRL,
			s5k4e1gx_reg_zero_shutter[rt].DPHY_bandctrl},
			/* Read Mode Setting */
			{S5K4E1GX_REG_READ_MODE,
			s5k4e1gx_reg_pat[rt].read_mode},
			/* Output Size */
			{REG_X_OUTPUT_SIZE_MSB,
			s5k4e1gx_reg_zero_shutter[rt].x_output_size_msb},
			{REG_X_OUTPUT_SIZE_LSB,
			s5k4e1gx_reg_zero_shutter[rt].x_output_size_lsb},
			{REG_Y_OUTPUT_SIZE_MSB,
			s5k4e1gx_reg_zero_shutter[rt].y_output_size_msb},
			{REG_Y_OUTPUT_SIZE_LSB,
			s5k4e1gx_reg_zero_shutter[rt].y_output_size_lsb},

			 /* Binning */
			{REG_X_EVEN_INC,
			s5k4e1gx_reg_pat[rt].x_even_inc},
			{REG_X_ODD_INC,
			s5k4e1gx_reg_pat[rt].x_odd_inc },
			{REG_Y_EVEN_INC,
			s5k4e1gx_reg_pat[rt].y_even_inc},
			{REG_Y_ODD_INC,
			s5k4e1gx_reg_zero_shutter[rt].y_odd_inc},
			{REG_H_BINNING,
			s5k4e1gx_reg_zero_shutter[rt].h_binning},
			{REG_V_BINNING,
			s5k4e1gx_reg_zero_shutter[rt].v_binning},
			/* Frame format */
			{REG_FRAME_LENGTH_LINES_MSB,
			s5k4e1gx_reg_zero_shutter[rt].frame_length_lines_msb},
			{REG_FRAME_LENGTH_LINES_LSB,
			s5k4e1gx_reg_zero_shutter[rt].frame_length_lines_lsb},
			{REG_LINE_LENGTH_PCK_MSB,
			s5k4e1gx_reg_zero_shutter[rt].line_length_pck_msb},
			{REG_LINE_LENGTH_PCK_LSB,
			s5k4e1gx_reg_zero_shutter[rt].line_length_pck_lsb},
			/* Integration Setting */
			{REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB,
		s5k4e1gx_reg_zero_shutter[rt].analogue_gain_code_global_msb},
			{REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB,
		s5k4e1gx_reg_zero_shutter[rt].analogue_gain_code_global_lsb},
			{REG_COARSE_INTEGRATION_TIME_MSB,
		s5k4e1gx_reg_zero_shutter[rt].coarse_integration_time_msb},
			{REG_COARSE_INTEGRATION_TIME_LSB,
		s5k4e1gx_reg_zero_shutter[rt].coarse_integration_time_lsb},

			/* other MIPI setting */
			/* outif_enable[7], data_type[5:0](2Bh = bayer 10bit) */
			{0x30BF, 0xAB},
			/* video_offset[7:4] 3260%12 */
			{0x30C0, 0x80},
			/* video_data_length 3260 = 2608 * 1.25 */
			{0x30C8, 0x0C},
			{0x30C9, 0xBC},
		};	/* end of struct s5k4e1gx_i2c_reg_conf tbl_4[] */

		    /* Standby */
			rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
					S5K4E1GX_REG_MODE_SELECT,
					S5K4E1GX_MODE_SELECT_SW_STANDBY);
				if (rc < 0)
					return rc;


			if (sdata->csi_if) {
				 /* 1126 for improve shutter of MIPI */
			  if (!sdata->zero_shutter_mode)
				s5k4e1gx_extra_settings_for_mipi(rt);
			  else
				pr_info("[CAM]%s: improve shutter lag\n", __func__);
			} else {
				/*awii: for the analog*/
				if (fps_mode_sel == 1) {
					s5k4e1gx_i2c_write_b
						(s5k4e1gx_client->addr, 0x30BC, 0xB0);
					s5k4e1gx_i2c_write_b
						(s5k4e1gx_client->addr, 0x302B, 0x01);
					s5k4e1gx_i2c_write_b
						(s5k4e1gx_client->addr, 0x30BE, 0x1A);
				} else {
					s5k4e1gx_i2c_write_b
						(s5k4e1gx_client->addr, 0x302B, 0x00);
					s5k4e1gx_i2c_write_b
						(s5k4e1gx_client->addr, 0x30BC, 0xA0);
					s5k4e1gx_i2c_write_b
						(s5k4e1gx_client->addr, 0x30BE, 0x08);
				}
			}

		    /* Write Setting Table */
			/* 1126 for improve shutter of MIPI */
			if (sdata->csi_if && sdata->zero_shutter_mode) {
			pr_info("[CAM]s5k4e1gx_setting(): setup tb1_4\n");
			rc = s5k4e1gx_i2c_write_table(&tbl_4[0],
					ARRAY_SIZE(tbl_4));
			} else {
			rc = s5k4e1gx_i2c_write_table(&tbl_3[0],
					ARRAY_SIZE(tbl_3));
				if (rc < 0)
					return rc;
			}

			/* Streaming ON */
			/* 1126 for improve shutter of MIPI */
			if (!sdata->csi_if)
			s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x3110, 0x10);

#ifdef CONFIG_MSM_CAMERA_7X30
	/*only streaming on in preview mode and zero_shutter_mode on MIPI*/
	if (rt == S_RES_PREVIEW && sdata->csi_if && sdata->zero_shutter_mode) {
		pr_info("[CAM]%s: delay 200ms before Streaming ON\n", __func__);
		mdelay(200);
	}
#endif

			rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
					S5K4E1GX_REG_MODE_SELECT,
					S5K4E1GX_MODE_SELECT_STREAM);
				if (rc < 0)
					return rc;

		    /* reset fps_divider */
			s5k4e1gx_ctrl->fps_divider = 1 * 0x0400;

			if (rt == S_RES_PREVIEW) {
				s5k4e1gx_ctrl->my_reg_gain = (uint16_t)
					((s5k4e1gx_reg_pat[rt].analogue_gain_code_global_msb << 8) & 0xFF00)
					+ s5k4e1gx_reg_pat[rt].analogue_gain_code_global_lsb;
				s5k4e1gx_ctrl->my_reg_line_count = (uint16_t)
					((s5k4e1gx_reg_pat[rt].coarse_integration_time_msb << 8) & 0xFF00)
					+ s5k4e1gx_reg_pat[rt].coarse_integration_time_lsb;
			}
		}
		break; /* case REG_INIT: */

	default:
		rc = -EINVAL;
		break;
	} /* switch (rupdate) */

	return rc;
}

static int s5k4e1gx_i2c_read_fuseid(struct sensor_cfg_data *cdata)
{

	int32_t  rc;
	unsigned short i, R1, R2, R3;
	unsigned short  OTP[10] = {0};

	pr_info("[CAM]%s: sensor OTP information:\n", __func__);

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30F9, 0x0E);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_b 0x30F9 fail\n", __func__);

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30FA, 0x0A);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_b 0x30FA fail\n", __func__);

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30FB, 0x71);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_b 0x30FB fail\n", __func__);

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x30FB, 0x70);
	if (rc < 0)
		pr_info("[CAM]%s: i2c_write_b 0x30FB fail\n", __func__);

	mdelay(4);

	for (i = 0; i < 10; i++) {
		rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x310C, i);
			if (rc < 0)
			pr_info("[CAM]%s: i2c_write_b 0x310C fail\n", __func__);
		rc = s5k4e1gx_i2c_read_b(s5k4e1gx_client->addr, 0x310F, &R1);
			if (rc < 0)
			pr_info("[CAM]%s: i2c_read_b 0x310F fail\n", __func__);
		rc = s5k4e1gx_i2c_read_b(s5k4e1gx_client->addr, 0x310E, &R2);
			if (rc < 0)
			pr_info("[CAM]%s: i2c_read_b 0x310E fail\n", __func__);
		rc = s5k4e1gx_i2c_read_b(s5k4e1gx_client->addr, 0x310D, &R3);
			if (rc < 0)
			pr_info("[CAM]%s: i2c_read_b 0x310D fail\n", __func__);

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
	pr_info("[CAM]%s: VenderID=%x,LensID=%x,SensorID=%x%x\n", __func__,
		OTP[0], OTP[1], OTP[2], OTP[3]);
	pr_info("[CAM]%s: ModuleFuseID= %x%x%x%x%x%x\n", __func__,
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

	pr_info("[CAM]s5k4e1gx: fuse->fuse_id_word1:%d\n",
		cdata->cfg.fuse.fuse_id_word1);
	pr_info("[CAM]s5k4e1gx: fuse->fuse_id_word2:%d\n",
		cdata->cfg.fuse.fuse_id_word2);
	pr_info("[CAM]s5k4e1gx: fuse->fuse_id_word3:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word3);
	pr_info("[CAM]s5k4e1gx: fuse->fuse_id_word4:0x%08x\n",
		cdata->cfg.fuse.fuse_id_word4);
	return 0;
}
static int s5k4e1gx_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int32_t  rc;
	int16_t toUseCand;
	struct msm_camera_sensor_info *sinfo = s5k4e1_pdev->dev.platform_data;

	if (fps_mode_sel == 1) {
		pr_info("[CAM]s5k4e1gx_setting PREVIEW AT 60FPS THIS TIME\n");
		toUseCand = 1;
	} else {
		pr_info("[CAM]s5k4e1gx_setting PREVIEW AT normal\n");
		toUseCand = 0;
	}

	/* 1126 for improve shutter of MIPI */
	if (machine_is_saga()) {
	pr_info("[CAM]%s: get s5k4e1gx_reg_zero_shutter setting\n", __func__);
	memcpy(&(s5k4e1gx_reg_pat[S_RES_PREVIEW]),
		&(s5k4e1gx_reg_zero_shutter[0]), sizeof(struct reg_struct));
	} else {
	memcpy(&(s5k4e1gx_reg_pat[S_RES_PREVIEW]),
		&(s5k4e1gx_reg_cand[toUseCand]), sizeof(struct reg_struct));
	}
	s5k4e1gx_ctrl = kzalloc(sizeof(struct s5k4e1gx_ctrl), GFP_KERNEL);
	if (!s5k4e1gx_ctrl) {
		pr_err("[CAM]s5k4e1gx_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	s5k4e1gx_ctrl->fps_divider = 1 * 0x00000400;
	s5k4e1gx_ctrl->pict_fps_divider = 1 * 0x00000400;
	s5k4e1gx_ctrl->set_test = S_TEST_OFF;
	s5k4e1gx_ctrl->prev_res = S_QTR_SIZE;
	s5k4e1gx_ctrl->pict_res = S_FULL_SIZE;
	s5k4e1gx_ctrl->reg_update = REG_INIT;

	if (data)
		s5k4e1gx_ctrl->sensordata = data;

	/* 1126 for improve shutter of MIPI */
	if (data->csi_if && data->zero_shutter_mode) {
		pr_info("[CAM]%s: set prev_res = S_FULL_SIZE\n", __func__);
		s5k4e1gx_ctrl->prev_res = S_FULL_SIZE;
	}

	/*switch PCLK and MCLK to Main cam*/
	if (data && data->camera_clk_switch != NULL) {
		pr_info("[CAM]%s: switch clk\n", __func__);
		data->camera_clk_switch();
		msleep(10);

		/* Configure CAM GPIO ON (CAM_MCLK)*/
		pr_info("[CAM]%s msm_camio_probe_on()\n", __func__);
		msm_camio_probe_on(s5k4e1_pdev);
	}

	/* enable mclk first */
	msm_camio_clk_rate_set(24000000);


	/* Force reset MIPI sensor for SAGA */
	/*1126 for improve shutter of MIPI*/
	if (machine_is_saga() && !data->zero_shutter_mode) {
		rc = s5k4e1gx_probe_init_sensor(data);
		if (rc < 0)
			printk("[CAM]s5k4e1gx_sensor_open_init() call s5k4e1gx_probe_init_sensor() failed !!!\n");
	}


	/* for parallel interface */
	if (!sinfo->csi_if) {
		mdelay(20);
		msm_camio_camif_pad_reg_reset();
		mdelay(20);
	}

	if (s5k4e1gx_ctrl->prev_res == S_QTR_SIZE)
		rc = s5k4e1gx_setting(S_REG_INIT, S_RES_PREVIEW);
	else {/*1126 for improve shutter of MIPI*/
		if (machine_is_saga() && sinfo->zero_shutter_mode)
			rc = s5k4e1gx_setting(S_REG_INIT, S_RES_PREVIEW);
		else
			rc = s5k4e1gx_setting(S_REG_INIT, S_RES_CAPTURE);
	}

	if (rc < 0) {
		pr_err("[CAM]s5k4e1gx_setting failed. rc = %d\n", rc);
		goto init_fail1;
	}

	/* set up lens position talbe */
	s5k4e1gx_setup_af_tbl();
	gpio_request(s5k4e1gx_ctrl->sensordata->vcm_pwd, "s5k4e1gx");
	gpio_direction_output(s5k4e1gx_ctrl->sensordata->vcm_pwd,	1);
	I2C_AF_WRITE(0x0, 0x00);
	s5k4e1gx_ctrl->curr_lens_pos = 0;
	s5k4e1gx_ctrl->curr_step_pos = 0;
	gpio_free(s5k4e1gx_ctrl->sensordata->vcm_pwd);
	goto init_done;

init_fail1:
	if (data)
		s5k4e1gx_probe_init_done(data);

	if (s5k4e1gx_ctrl) {
		kfree(s5k4e1gx_ctrl);
		s5k4e1gx_ctrl = NULL;
	}
init_done:
	return rc;
}

static int32_t s5k4e1gx_power_down(void)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *sdata = s5k4e1_pdev->dev.platform_data;

	s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, S5K4E1GX_REG_MODE_SELECT, S5K4E1GX_MODE_SELECT_SW_STANDBY);
	mdelay(110);

	if (!sdata->csi_if) {
	s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr, 0x3110, 0x11);
	mdelay(120);
	}

	return rc;
}

static int s5k4e1gx_sensor_release(void)
{
	int rc = -EBADF;
	struct msm_camera_sensor_info *sdata = s5k4e1_pdev->dev.platform_data;

	mutex_lock(&s5k4e1gx_mutex);
	/*SW stand by*/
	s5k4e1gx_power_down();
	/*HW stand by*/
	if (s5k4e1gx_ctrl) {
		gpio_request(s5k4e1gx_ctrl->sensordata->vcm_pwd, "s5k4e1gx");
		gpio_direction_output(s5k4e1gx_ctrl->sensordata->vcm_pwd, 0);
		gpio_free(s5k4e1gx_ctrl->sensordata->vcm_pwd);
	}

	if (s5k4e1gx_ctrl) {
		kfree(s5k4e1gx_ctrl);
		s5k4e1gx_ctrl = NULL;
	}

	allow_suspend();

	if (sdata->camera_clk_switch != NULL && sdata->cam_select_pin) {
	/*0730: optical ask : CLK switch to Main Cam after 2nd Cam release*/
	pr_info("[CAM]%s: doing clk switch to Main CAM)\n", __func__);
	rc = gpio_request(sdata->cam_select_pin, "s5k4e1gx");
	if (rc < 0)
		pr_err("[CAM]GPIO (%d) request fail\n", sdata->cam_select_pin);
	else
		gpio_direction_output(sdata->cam_select_pin, 0);
	gpio_free(sdata->cam_select_pin);

	msleep(5);
	/* CLK switch set 0 */

	pr_info("[CAM]%s msm_camio_probe_off()\n", __func__);
	msm_camio_probe_off(s5k4e1_pdev);
	}

	pr_info("[CAM]s5k4e1gx_release completed\n");
	mutex_unlock(&s5k4e1gx_mutex);
	return rc;
}

static void s5k4e1gx_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;	/*Q10 */
	uint32_t d1;
	uint32_t d2;

	d1 =
		(uint32_t)(
		((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l) *
		0x00000400) /
		(s5k4e1gx_reg_pat[S_RES_CAPTURE].size_h +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_l));

	d2 =
		(uint32_t)(
		((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_w +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_p) *
		0x00000400) /
		 (s5k4e1gx_reg_pat[S_RES_CAPTURE].size_w +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_p));


	divider = (uint32_t) (d1 * d2) / 0x00000400;
	/* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t)(fps * divider / 0x00000400);
}

static uint16_t s5k4e1gx_get_prev_lines_pf(void)
{
	return s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
		s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l;
}

static uint16_t s5k4e1gx_get_prev_pixels_pl(void)
{
	return s5k4e1gx_reg_pat[S_RES_PREVIEW].size_w +
		s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_p;
}

static uint16_t s5k4e1gx_get_pict_lines_pf(void)
{
	return s5k4e1gx_reg_pat[S_RES_CAPTURE].size_h +
		s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_l;
}

static uint16_t s5k4e1gx_get_pict_pixels_pl(void)
{
	return s5k4e1gx_reg_pat[S_RES_CAPTURE].size_w +
		s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_p;
}

static uint32_t s5k4e1gx_get_pict_max_exp_lc(void)
{
	uint32_t snapshot_lines_per_frame;

	if (s5k4e1gx_ctrl->pict_res == S_QTR_SIZE)
		snapshot_lines_per_frame =
		s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
		s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l;
	else
		snapshot_lines_per_frame = S5K4E1GX_MAX_SNAPSHOT_EXP_LC * 3;

	return snapshot_lines_per_frame;
}

static int32_t s5k4e1gx_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int32_t rc = 0;

	s5k4e1gx_ctrl->fps_divider = fps->fps_div;

	CDBG("s5k3e2fx_ctrl->fps_divider = %d\n",
		s5k4e1gx_ctrl->fps_divider);
/* sync from s5k3e2gx.c */
#if 0
	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
		REG_FRAME_LENGTH_LINES_MSB,
		(((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l) *
			s5k4e1gx_ctrl->fps_divider / 0x400) & 0xFF00) >> 8);
	if (rc < 0)
		goto set_fps_done;

	rc = s5k4e1gx_i2c_write_b(s5k4e1gx_client->addr,
		REG_FRAME_LENGTH_LINES_LSB,
		(((s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l) *
			s5k4e1gx_ctrl->fps_divider / 0x400) & 0x00FF));
#endif

#if 0
set_fps_done:
#endif
	return rc;
}

static int32_t s5k4e1gx_write_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	uint16_t max_legal_gain = 0x0200;
	uint32_t ll_ratio; /* Q10 */
	uint32_t ll_pck, fl_lines;
	uint16_t offset = 12; /* 4, Kevin 20100122 for abnormal yellow color*/
	uint32_t  gain_msb, gain_lsb;
	uint32_t  intg_t_msb, intg_t_lsb;
	uint32_t  ll_pck_msb, ll_pck_lsb;
	int less_size;

	struct s5k4e1gx_i2c_reg_conf tbl[3];

	/*
	printk("[CAM]s5k4e1gx_write_exp_gain    gain=%d   line=%d\n", gain, line);
	*/
	if ((gain == 0) || (line == 0))
		return rc;

	if (s5k4e1gx_ctrl->sensormode == SENSOR_PREVIEW_MODE) {

		s5k4e1gx_ctrl->my_reg_gain = gain;
		s5k4e1gx_ctrl->my_reg_line_count = (uint16_t)line;

		fl_lines = s5k4e1gx_reg_pat[S_RES_PREVIEW].size_h +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_l;

		ll_pck = s5k4e1gx_reg_pat[S_RES_PREVIEW].size_w +
			s5k4e1gx_reg_pat[S_RES_PREVIEW].blk_p;

	} else {

		fl_lines = s5k4e1gx_reg_pat[S_RES_CAPTURE].size_h +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_l;

		ll_pck = s5k4e1gx_reg_pat[S_RES_CAPTURE].size_w +
			s5k4e1gx_reg_pat[S_RES_CAPTURE].blk_p;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	/* in Q10 */
	line = (line * s5k4e1gx_ctrl->fps_divider);

	if (fl_lines < (line / 0x400))
		ll_ratio = (line / (fl_lines - offset));
	else
		ll_ratio = 0x400;

	/* update gain registers */
	gain_msb = (gain & 0xFF00) >> 8;
	gain_lsb = gain & 0x00FF;
	tbl[0].waddr = S5K4E1GX_REG_GROUP_PARAMETER_HOLD;
	tbl[0].bdata = S5K4E1GX_GROUP_PARAMETER_HOLD;
	tbl[1].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_MSB;
	tbl[1].bdata = gain_msb;
	tbl[2].waddr = REG_ANALOGUE_GAIN_CODE_GLOBAL_LSB;
	tbl[2].bdata = gain_lsb;
	rc = s5k4e1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));
	if (rc < 0)
		goto write_gain_done;

	ll_pck = ll_pck * ll_ratio;
	ll_pck_msb = ((ll_pck / 0x400) & 0xFF00) >> 8;
	ll_pck_lsb = (ll_pck / 0x400) & 0x00FF;
	tbl[0].waddr = REG_LINE_LENGTH_PCK_MSB;
	tbl[0].bdata = ll_pck_msb;
	tbl[1].waddr = REG_LINE_LENGTH_PCK_LSB;
	tbl[1].bdata = ll_pck_lsb;
	less_size = ARRAY_SIZE(tbl) - 1;
	rc = s5k4e1gx_i2c_write_table(&tbl[0], less_size);
	if (rc < 0)
		goto write_gain_done;

	line = line / ll_ratio;
	intg_t_msb = (line & 0xFF00) >> 8;
	intg_t_lsb = (line & 0x00FF);
	tbl[0].waddr = REG_COARSE_INTEGRATION_TIME_MSB;
	tbl[0].bdata = intg_t_msb;
	tbl[1].waddr = REG_COARSE_INTEGRATION_TIME_LSB;
	tbl[1].bdata = intg_t_lsb;
	tbl[2].waddr = S5K4E1GX_REG_GROUP_PARAMETER_HOLD;
	tbl[2].bdata = S5K4E1GX_GROUP_PARAMETER_UNHOLD;
	rc = s5k4e1gx_i2c_write_table(&tbl[0], ARRAY_SIZE(tbl));

write_gain_done:
	return rc;
}

#ifdef CONFIG_MSM_CAMERA_7X30
static uint32_t s5k4e1gx_get_snapshot_fps(uint32_t line)
{
	uint32_t line_length_pix;
	uint32_t fps;
	uint32_t i,pclk,pclk_power=1;
	unsigned short pclk_div;
	/*caculate fps depence on line count*/
	/*caculate line length pix depence on sensor config*/
	line_length_pix = (s5k4e1gx_reg_pat[S_RES_CAPTURE].line_length_pck_msb << 8) +
		s5k4e1gx_reg_pat[S_RES_CAPTURE].line_length_pck_lsb;
	/*caculate pclk depence on sensor config*/
	/*pclk = ((((mclk /pll_clk_div)*internal_multipler) * pll_scaler) * 2^vt_pclk )/vt_pclk*/
	s5k4e1gx_i2c_read_b(s5k4e1gx_client->addr, 0x30BE , &pclk_div);
	pclk = ((MCLK / s5k4e1gx_reg_pat[S_RES_CAPTURE].pre_pll_clk_div) * 2) *
		((s5k4e1gx_reg_pat[S_RES_CAPTURE].pll_multiplier_msb << 8)+
		s5k4e1gx_reg_pat[S_RES_CAPTURE].pll_multiplier_lsb);
	for(i = 1 ; i <=s5k4e1gx_reg_pat[S_RES_CAPTURE].vt_sys_clk_div ; i++)
		pclk_power *= 2;
	pclk = (pclk * pclk_power) / pclk_div;
	fps = ((pclk*MHz)/line_length_pix)/line;
	if (fps > 15)
		fps = 15;
	return fps;
}
#endif

static int32_t s5k4e1gx_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	CDBG("Line:%d s5k4e1gx_set_pict_exp_gain \n", __LINE__);
	if (s5k4e1gx_ctrl->sensordata->zero_shutter_mode)
		return rc;

	rc = s5k4e1gx_write_exp_gain(gain, line);
#ifdef CONFIG_MSM_CAMERA_7X30
	{
	uint32_t delaytime, fps;
	s5k4e1gx_i2c_write_b(
		s5k4e1gx_client->addr,
		S5K4E1GX_REG_MODE_SELECT,
		S5K4E1GX_MODE_SELECT_STREAM);

	fps = s5k4e1gx_get_snapshot_fps(line);
	delaytime = 1000/fps;
	mdelay(2*delaytime);
	}
#endif
	return rc;
}

static int32_t s5k4e1gx_video_config(int mode, int res)
{
	int32_t rc;

	switch (res) {
		case S_QTR_SIZE:
			rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);
			if (rc < 0)
				return rc;

			pr_info("[CAM]s5k4e1gx sensor configuration done!\n");
		break;

		case S_FULL_SIZE:
			rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
			if (rc < 0)
				return rc;

		break;

		default:
			return 0;
	} /* switch */

	s5k4e1gx_ctrl->prev_res = res;
	s5k4e1gx_ctrl->curr_res = res;
	s5k4e1gx_ctrl->sensormode = mode;

	preview_frame_count = 0;
	rc =
		s5k4e1gx_write_exp_gain(s5k4e1gx_ctrl->my_reg_gain,
			s5k4e1gx_ctrl->my_reg_line_count);

	return rc;
}

static int32_t s5k4e1gx_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)
		return rc;

	s5k4e1gx_ctrl->curr_res = s5k4e1gx_ctrl->pict_res;
	s5k4e1gx_ctrl->sensormode = mode;

	return rc;
}

static int32_t s5k4e1gx_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
       pr_info("[CAM]s5k4e1gx_raw_snapshot_config\n");
	rc = s5k4e1gx_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);
	if (rc < 0)
		return rc;

	s5k4e1gx_ctrl->curr_res = s5k4e1gx_ctrl->pict_res;
	s5k4e1gx_ctrl->sensormode = mode;

	return rc;
}

static int32_t s5k4e1gx_set_sensor_mode(int mode, int res)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info *sinfo = s5k4e1_pdev->dev.platform_data;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = s5k4e1gx_video_config(mode, res);
		break;

	case SENSOR_SNAPSHOT_MODE:
		pr_info("[CAM]KPI PA: start sensor snapshot config\n");
		/* Check V-sync frame timer Start */
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		rc = s5k4e1gx_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		pr_info("[CAM]KPI PA: start sensor raw snapshot config\n");
		sinfo->kpi_sensor_start = ktime_to_ns(ktime_get());
		rc = s5k4e1gx_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}
static int32_t s5k4e1gx_go_to_position(uint32_t lens_pos,
									   uint8_t mask)
{
	int32_t rc = 0;
	unsigned char buf[2];
	uint8_t code_val_msb, code_val_lsb;

	code_val_msb = lens_pos >> 4;
	code_val_lsb = (lens_pos & 0x000F) << 4;
	code_val_lsb |= mask;

	buf[0] = code_val_msb;
	buf[1] = code_val_lsb;
	rc = s5k4e1gx_i2c_txdata(S5K4E1GX_AF_I2C_ADDR >> 1, buf, 2);
	if (rc < 0)
	pr_err("[CAM]i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n",
		S5K4E1GX_AF_I2C_ADDR >> 1, buf[0], buf[1]);

	return rc;
}

static int32_t s5k4e1gx_move_focus(int direction, int32_t num_steps)
{
	uint16_t s5k4e1gx_sw_damping_time_wait = 5;
	uint16_t s5k4e1gx_damping_threshold = 5;
	uint8_t  s5k4e1gx_mode_mask = 0x02;
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
	int8_t s5k4e1gx_sw_damping_required = 1;
	int32_t s5k4e1gx_sw_damping_step_dynamic = S5K4E1GX_SW_DAMPING_STEP;

	if (num_steps > S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR;
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
	curr_lens_pos = s5k4e1gx_ctrl->curr_lens_pos;
	curr_step_pos = s5k4e1gx_ctrl->curr_step_pos;

	if (curr_lens_pos < s5k4e1gx_ctrl->init_curr_lens_pos)
		curr_lens_pos = s5k4e1gx_ctrl->init_curr_lens_pos;

	dest_step_pos = curr_step_pos + (step_direction * num_steps);

	if (dest_step_pos < 0)
		dest_step_pos = 0;
	else if (dest_step_pos > S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_pos = S5K4E1GX_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_pos == s5k4e1gx_ctrl->curr_step_pos)
		return rc;

	dest_lens_pos = s5k4e1gx_pos_tbl[dest_step_pos];
	target_dist = step_direction * (dest_lens_pos - curr_lens_pos);

	if (num_steps > 2) {
		s5k4e1gx_sw_damping_step_dynamic = 4;
		s5k4e1gx_sw_damping_time_wait = 4;
	} else {
		s5k4e1gx_sw_damping_step_dynamic = 2;
		s5k4e1gx_sw_damping_time_wait = 2;
	}

	/* HW damping */
	if (step_direction < 0 && target_dist >= s5k4e1gx_pos_tbl[s5k4e1gx_damping_threshold]) {
		/*s5k4e1gx_sw_damping_required = 1;*/
		s5k4e1gx_sw_damping_step_dynamic = S5K4E1GX_SW_DAMPING_STEP;
		s5k4e1gx_sw_damping_time_wait = 1;
		time_wait = 1000000 / S5K4E1GX_MAX_FPS - S5K4E1GX_SW_DAMPING_STEP * s5k4e1gx_sw_damping_time_wait * 1000;
	} else
		time_wait = 1000000 / S5K4E1GX_MAX_FPS;

	time_wait_per_step = (int16_t)(time_wait / target_dist);

	if (time_wait_per_step >= 800)
		/* ~800 */
		s5k4e1gx_mode_mask = 0x5;
	else if (time_wait_per_step >= 400)
		/* ~400 */
		s5k4e1gx_mode_mask = 0x4;
	else if (time_wait_per_step >= 200)
		/* 200~400 */
		s5k4e1gx_mode_mask = 0x3;
	else if (time_wait_per_step >= 100)
		/* 100~200 */
		s5k4e1gx_mode_mask = 0x2;
	else if (time_wait_per_step >= 50)
		/* 50~100 */
		s5k4e1gx_mode_mask = 0x1;
	else {
		if(time_wait >= 17600)
			s5k4e1gx_mode_mask = 0x0D;
		else if (time_wait >= 8800)
			s5k4e1gx_mode_mask = 0x0C;
		else if (time_wait >= 4400)
			s5k4e1gx_mode_mask = 0x0B;
		else if (time_wait >= 2200)
			s5k4e1gx_mode_mask = 0x0A;
		else
			s5k4e1gx_mode_mask = 0x09;
	}

	if (s5k4e1gx_sw_damping_required) {
		small_step = (uint16_t)target_dist/s5k4e1gx_sw_damping_step_dynamic;
		if ((target_dist % s5k4e1gx_sw_damping_step_dynamic) != 0)
			small_step = small_step + 1;

		for (next_lens_pos = curr_lens_pos + (step_direction * small_step);
			(step_direction * next_lens_pos) <= (step_direction * dest_lens_pos);
			 next_lens_pos += (step_direction * small_step)) {
			rc = s5k4e1gx_go_to_position(next_lens_pos, s5k4e1gx_mode_mask);
			if (rc < 0) {
				CDBG("s5k4e1gx_go_to_position Failed in Move Focus!!!\n");
				return rc;
			}
			curr_lens_pos = next_lens_pos;
			mdelay(s5k4e1gx_sw_damping_time_wait);
		}

		if(curr_lens_pos != dest_lens_pos) {
			rc = s5k4e1gx_go_to_position(dest_lens_pos, s5k4e1gx_mode_mask);
			if (rc < 0) {
				pr_err("[CAM]s5k4e1gx_go_to_position Failed in Move Focus!!!\n");
				return rc;
			}
			mdelay(s5k4e1gx_sw_damping_time_wait);
		}
	} else {
		rc = s5k4e1gx_go_to_position(dest_lens_pos, s5k4e1gx_mode_mask);
		if (rc < 0) {
			pr_err("[CAM]s5k4e1gx_go_to_position Failed in Move Focus!!!\n");
			return rc;
		}
	}

	s5k4e1gx_ctrl->curr_lens_pos = dest_lens_pos;
	s5k4e1gx_ctrl->curr_step_pos = dest_step_pos;

	return rc;
}

static int32_t s5k4e1gx_set_default_focus(void)
{
	int32_t rc = 0;
	if (s5k4e1gx_ctrl->curr_step_pos != 0) {
		rc = s5k4e1gx_move_focus(MOVE_FAR, s5k4e1gx_ctrl->curr_step_pos);
		if (rc < 0) {
			pr_err("[CAM]s5k4e1gx_set_default_focus Failed!!!\n");
			return rc;
		}
	} else {
		rc = s5k4e1gx_go_to_position(0, 0x02);
		if (rc < 0) {
			pr_err("[CAM]s5k4e1gx_go_to_position Failed!!!\n");
			return rc;
		}
	}

	s5k4e1gx_ctrl->curr_lens_pos = 0;
	s5k4e1gx_ctrl->init_curr_lens_pos = 0;
	s5k4e1gx_ctrl->curr_step_pos = 0;

	return rc;
}

uint8_t s5k4e1gx_preview_skip_frame(void)
{
	if (s5k4e1gx_ctrl->sensordata->zero_shutter_mode)
		return 0;

	if (s5k4e1gx_ctrl->sensormode == SENSOR_PREVIEW_MODE && preview_frame_count < 1) {
		preview_frame_count++;
		return 1;
	}
	return 0;
}

static int s5k4e1gx_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	mutex_lock(&s5k4e1gx_mutex);

	/*
	CDBG("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
	*/

	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		s5k4e1gx_get_pict_fps(cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp, &cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = s5k4e1gx_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = s5k4e1gx_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = s5k4e1gx_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl = s5k4e1gx_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
			s5k4e1gx_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = s5k4e1gx_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc =
			s5k4e1gx_write_exp_gain(cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		CDBG("Line:%d CFG_SET_PICT_EXP_GAIN \n", __LINE__);
		rc =
			s5k4e1gx_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc =
			s5k4e1gx_set_sensor_mode(
			cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = s5k4e1gx_power_down();
		break;

	case CFG_MOVE_FOCUS:
		rc =
			s5k4e1gx_move_focus(
			cdata.cfg.focus.dir,
			cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc =
			s5k4e1gx_set_default_focus();
		break;
	case CFG_I2C_IOCTL_R_OTP:{
		pr_info("[CAM]Line:%d CFG_I2C_IOCTL_R_OTP \n", __LINE__);
		rc = s5k4e1gx_i2c_read_fuseid(&cdata);
		if (copy_to_user(argp, &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		}
		break;
	case CFG_GET_AF_MAX_STEPS:
	case CFG_SET_EFFECT:
	case CFG_SET_LENS_SHADING:
	default:
		rc = -EINVAL;
		break;
	}

    prevent_suspend();
	mutex_unlock(&s5k4e1gx_mutex);
	return rc;
}


static const char *S5K4E1GXVendor = "samsung";
static const char *S5K4E1GXNAME = "s5k4e1gx";
static const char *S5K4E1GXSize = "5M";

static ssize_t sensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s %s %s\n", S5K4E1GXVendor, S5K4E1GXNAME, S5K4E1GXSize);
	ret = strlen(buf) + 1;

	return ret;
}

DEFINE_MUTEX(fps_mode_lock);

static ssize_t sensor_read_fps_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	mutex_lock(&fps_mode_lock);
	length = sprintf(buf, "%d\n", fps_mode_sel);
	mutex_unlock(&fps_mode_lock);
	return length;
}

static ssize_t sensor_set_fps_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;
	mutex_lock(&fps_mode_lock);
	tmp = buf[0] - 0x30;
	fps_mode_sel = tmp;
	mutex_unlock(&fps_mode_lock);
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
static DEVICE_ATTR(node, 0444, sensor_read_node, NULL);
static DEVICE_ATTR(fps_mode, 0777,
	sensor_read_fps_mode, sensor_set_fps_mode);

static struct kobject *android_s5k4e1gx = NULL;

static int s5k4e1gx_sysfs_init(void)
{
	int ret ;
	printk(KERN_INFO "s5k4e1gx_sysfs_init : kobject_create_and_add\n");
	android_s5k4e1gx = kobject_create_and_add("android_camera", NULL);
	if (android_s5k4e1gx == NULL) {
		printk(KERN_INFO "s5k4e1gx_sysfs_init: subsystem_register " \
		"failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	printk(KERN_INFO "s5k4e1gx_sysfs_init : sysfs_create_file\n");
	ret = sysfs_create_file(android_s5k4e1gx, &dev_attr_sensor.attr);
	if (ret) {
		printk(KERN_INFO "s5k4e1gx_sysfs_init : sysfs_create_file " \
		"failed\n");
		kobject_del(android_s5k4e1gx);
	}
	ret = sysfs_create_file(android_s5k4e1gx, &dev_attr_fps_mode.attr);
	if (ret) {
		printk(KERN_INFO "s5k4e1gx_sysfs_init : sysfs_create_file " \
		"failed\n");
		ret = -EFAULT;
		return ret ;
	}
        ret = sysfs_create_file(android_s5k4e1gx, &dev_attr_node.attr);
	if (ret) {
		pr_info("[CAM]s5k4e1gx_sysfs_init: dev_attr_node failed\n");
		ret = -EFAULT;
		return ret;
	}

	return 0 ;
}


static int s5k4e1gx_sensor_probe(struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	printk("[CAM]s5k4e1gx_sensor_probe()\n");

	rc = i2c_add_driver(&s5k4e1gx_i2c_driver);
	if (rc < 0 || s5k4e1gx_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}

	pr_info("[CAM]s5k4e1gx s->node %d\n", s->node);
	sensor_probe_node = s->node;

	/*switch PCLK and MCLK to Main cam*/
	if (info->camera_clk_switch != NULL) {
		pr_info("[CAM]s5k4e1gx: s5k4e1gx_sensor_probe: switch clk\n");
		info->camera_clk_switch();
	}

	msm_camio_clk_rate_set(24000000);
	mdelay(20);

	rc = s5k4e1gx_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	init_suspend();
	s->s_init = s5k4e1gx_sensor_open_init;
	s->s_release = s5k4e1gx_sensor_release;
	s->s_config  = s5k4e1gx_sensor_config;
	s5k4e1gx_probe_init_done(info);
	s5k4e1gx_sysfs_init();
	info->preview_skip_frame = s5k4e1gx_preview_skip_frame;
	return rc;

probe_fail:
	pr_err("[CAM]SENSOR PROBE FAILS!\n");
	return rc;
}



static int s5k4e1gx_vreg_enable(struct platform_device *pdev)
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

#if 0
static int s5k4e1gx_vreg_disable(struct platform_device *pdev)
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

static int __s5k4e1gx_probe(struct platform_device *pdev)
{
	int rc;
	printk("[CAM]__s5k4e1gx_probe\n");
	s5k4e1_pdev = pdev;
	rc = s5k4e1gx_vreg_enable(pdev);
	if (rc < 0)
		pr_err("[CAM]__s5k4e1gx_probe fail sensor power on error\n");

	return msm_camera_drv_start(pdev, s5k4e1gx_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k4e1gx_probe,
	.driver = {
		.name = "msm_camera_s5k4e1gx",
	},
};

static int __init s5k4e1gx_init(void)
{
	printk("[CAM]s5k4e1gx_init\n");
	return platform_driver_register(&msm_camera_driver);
}

static void __exit s5k4e1gx_exit(void){
	platform_driver_unregister(&msm_camera_driver);
}

module_init(s5k4e1gx_init);
module_exit(s5k4e1gx_exit);

MODULE_DESCRIPTION("camera sensor driver");
MODULE_LICENSE("GPL");



