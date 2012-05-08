/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */

#ifndef __LINUX_MSM_CAMERA_SENSOR_H
#define __LINUX_MSM_CAMERA_SENSOR_H

#include <linux/types.h>
#include <asm/sizes.h>

#define CFG_SET_MODE			0
#define CFG_SET_EFFECT			1
#define CFG_START			2
#define CFG_PWR_UP			3
#define CFG_PWR_DOWN			4
#define CFG_WRITE_EXPOSURE_GAIN		5
#define CFG_SET_DEFAULT_FOCUS		6
#define CFG_MOVE_FOCUS			7
#define CFG_REGISTER_TO_REAL_GAIN	8
#define CFG_REAL_TO_REGISTER_GAIN	9
#define CFG_SET_FPS			10
#define CFG_SET_PICT_FPS		11
#define CFG_SET_BRIGHTNESS		12
#define CFG_SET_CONTRAST		13
#define CFG_SET_ZOOM			14
#define CFG_SET_EXPOSURE_MODE		15
#define CFG_SET_WB			16
#define CFG_SET_ANTIBANDING		17
#define CFG_SET_EXP_GAIN		18
#define CFG_SET_PICT_EXP_GAIN		19
#define CFG_SET_LENS_SHADING		20
#define CFG_GET_PICT_FPS		21
#define CFG_GET_PREV_L_PF		22
#define CFG_GET_PREV_P_PL		23
#define CFG_GET_PICT_L_PF		24
#define CFG_GET_PICT_P_PL		25
#define CFG_GET_AF_MAX_STEPS		26
#define CFG_GET_PICT_MAX_EXP_LC		27
#define CFG_I2C_IOCTL_R_OTP	28
#define CFG_SET_OV_LSC	29
#define CFG_SET_SHARPNESS 30
#define CFG_SET_SATURATION 31
#define CFG_SET_OV_LSC_RAW_CAPTURE 32
#define CFG_SET_ISO			33
#define CFG_SET_COORDINATE		34
#define CFG_RUN_AUTO_FOCUS		35
#define CFG_CANCEL_AUTO_FOCUS		36
#define CFG_GET_EXP_FOR_LED		37
#define CFG_UPDATE_AEC_FOR_LED		38
#define CFG_SET_FRONT_CAMERA_MODE	39
#define CFG_SET_QCT_LSC_RAW_CAPTURE 40
#define CFG_SET_QTR_SIZE_MODE		41
#define CFG_GET_AF_STATE		42
#define CFG_SET_DMODE			43
#define CFG_SET_CALIBRATION	44
#define CFG_SET_AF_MODE		45
#define CFG_GET_SP3D_L_FRAME	46
#define CFG_GET_SP3D_R_FRAME	47
#define CFG_SET_FLASHLIGHT		48
#define CFG_SEND_WB_INFO        49
#define CFG_SET_FLASHLIGHT_EXP_DIV 50
#define CFG_GET_ISO             51
#define CFG_GET_EXP_GAIN	52
#define CFG_SET_FRAMERATE 	53

#ifdef CONFIG_CAMERA_3D
#define CFG_SENSOR_INIT			54
#define CFG_GET_3D_CALI_DATA 	55
#define CFG_GET_CALIB_DATA		56
#define CFG_SET_SCENE_MODE      57
#define CFG_SET_AEC_AWB_LOCK_MODE 58
#define CFG_MAX                 59
#else
#ifdef CONFIG_RAWCHIP
#define CFG_SET_EXP_GAIN_YUSHAN	54
#define CFG_GET_CUR_STEPS      55
#define CFG_MAX        	        56
#endif
#endif




#define MOVE_NEAR	0
#define MOVE_FAR	1

#define SENSOR_PREVIEW_MODE		0
#define SENSOR_SNAPSHOT_MODE		1
#define SENSOR_RAW_SNAPSHOT_MODE	2
#define SENSOR_VIDEO_MODE	3
#define SENSOR_VIDEO_60FPS_MODE	4
#define SENSOR_GET_EXP 5


#define SENSOR_QTR_SIZE			0
#define SENSOR_FULL_SIZE		1
#define SENSOR_QVGA_SIZE		2
#define SENSOR_VIDEO_SIZE		3
#define SENSOR_INVALID_SIZE		4

#define CAMERA_EFFECT_OFF		0
#define CAMERA_EFFECT_MONO		1
#define CAMERA_EFFECT_NEGATIVE		2
#define CAMERA_EFFECT_SOLARIZE		3
#define CAMERA_EFFECT_SEPIA		4
#define CAMERA_EFFECT_POSTERIZE		5
#define CAMERA_EFFECT_WHITEBOARD	6
#define CAMERA_EFFECT_BLACKBOARD	7
#define CAMERA_EFFECT_AQUA		8
#define CAMERA_EFFECT_MAX		9

#define CAMERA_3D_MODE 0
#define CAMERA_2D_MODE 1


struct sensor_pict_fps {
	uint16_t prevfps;
	uint16_t pictfps;
};

struct exp_gain_cfg {
	uint16_t gain;
	uint32_t line;
	uint16_t mul;
};

struct focus_cfg {
	int32_t steps;
	int dir;
	int coarse_delay;
	int fine_delay;
	int step_dir;
	int init_code_offset_max;
	uint16_t curr_steps;
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
};

struct wb_info_cfg {
	uint16_t red_gain;
	uint16_t green_gain;
	uint16_t blue_gain;
};


/*Becker for AWB calibration*/
struct fuse_id{
	uint32_t fuse_id_word1;
	uint32_t fuse_id_word2;
	uint32_t fuse_id_word3;
	uint32_t fuse_id_word4;
};

/*Vincent for LSC calibration*/
struct reg_addr_val_pair_struct {
	uint16_t reg_addr;
	uint8_t reg_val;
};

struct lsc_cfg{
	struct reg_addr_val_pair_struct lsc_table[144]; /*OV LSC table*/
};


/* For 2nd CAM (mt9v113) */
enum antibanding_mode{
	CAMERA_ANTI_BANDING_50HZ,
	CAMERA_ANTI_BANDING_60HZ,
	CAMERA_ANTI_BANDING_AUTO,
};

enum brightness_t{
	CAMERA_BRIGHTNESS_N3,
	CAMERA_BRIGHTNESS_N2,
	CAMERA_BRIGHTNESS_N1,
	CAMERA_BRIGHTNESS_D,
	CAMERA_BRIGHTNESS_P1,
	CAMERA_BRIGHTNESS_P2,
	CAMERA_BRIGHTNESS_P3,
	CAMERA_BRIGHTNESS_P4,
	CAMERA_BRIGHTNESS_N4,
};

enum frontcam_t{
	CAMERA_MIRROR,
	CAMERA_REVERSE,
	CAMERA_PORTRAIT_REVERSE, /* 0916 for 3rd party */
};

enum wb_mode{
	CAMERA_AWB_AUTO,/*auto*/
	CAMERA_AWB_CLOUDY,/*Cloudy*/
	CAMERA_AWB_INDOOR_HOME,/*Fluorescent*/
	CAMERA_AWB_INDOOR_OFFICE,/*Incandescent*/
	CAMERA_AWB_SUNNY,/*daylight*/
};

enum iso_mode{
  CAMERA_ISO_AUTO = 0,
  CAMERA_ISO_DEBLUR,
  CAMERA_ISO_100,
  CAMERA_ISO_200,
  CAMERA_ISO_400,
  CAMERA_ISO_800,
  CAMERA_ISO_1250,
  CAMERA_ISO_1600,
  CAMERA_ISO_MAX
};

enum sharpness_mode{
	CAMERA_SHARPNESS_X0,
	CAMERA_SHARPNESS_X1,
	CAMERA_SHARPNESS_X2,
	CAMERA_SHARPNESS_X3,
	CAMERA_SHARPNESS_X4,
	CAMERA_SHARPNESS_X5,
	CAMERA_SHARPNESS_X6,
};

enum saturation_mode{
	CAMERA_SATURATION_X0,
	CAMERA_SATURATION_X05,
	CAMERA_SATURATION_X1,
	CAMERA_SATURATION_X15,
	CAMERA_SATURATION_X2,
};

enum contrast_mode{
	CAMERA_CONTRAST_P2,
	CAMERA_CONTRAST_P1,
	CAMERA_CONTRAST_D,
	CAMERA_CONTRAST_N1,
	CAMERA_CONTRAST_N2,
};

enum qtr_size_mode{
	NORMAL_QTR_SIZE_MODE,
	LARGER_QTR_SIZE_MODE,
};

enum sensor_af_mode{
	SENSOR_AF_MODE_AUTO,
	SENSOR_AF_MODE_NORMAL,
	SENSOR_AF_MODE_MACRO,
};

struct Sp3d_OTP{
	unsigned long long  coefA1;
	unsigned long long  coefB1;
	unsigned long long  coefC1;
	unsigned long long  coefA2;
	unsigned long long  coefB2;
	unsigned long long  coefC2;
	unsigned long long  coefA3;
	unsigned long long  coefB3;
	unsigned long long  coefC3;
};

struct otp_cfg{
	struct Sp3d_OTP master_otp;
	struct Sp3d_OTP slave_otp;
	uint8_t sp3d_id[11];
	uint16_t sp3d_otp_version;
};

struct flash_cfg{
	uint8_t flash_enable;
	uint16_t exp_pre;
	uint16_t exp_off;
	uint16_t luma_pre;
	uint16_t luma_off;
};

struct exp_cfg{
	uint16_t AGC_Gain1;
	uint16_t AGC_Gain2;
	uint16_t ExposureTimeNum0;
	uint16_t ExposureTimeNum1;
	uint16_t ExposureTimeNum2;
	uint16_t ExposureTimeNum3;
	uint16_t ExposureTimeDen0;
	uint16_t ExposureTimeDen1;
	uint16_t ExposureTimeDen2;
	uint16_t ExposureTimeDen3;
	uint16_t AF_area;
	uint16_t flicker_compansation;
};

#ifdef CONFIG_CAMERA_3D
struct sensor_3d_exp_cfg {
	uint16_t gain;
	uint32_t line;
	uint16_t r_gain;
	uint16_t b_gain;
	uint16_t gr_gain;
	uint16_t gb_gain;
	uint16_t gain_adjust;
};

struct sensor_3d_cali_data_t{
	unsigned char left_p_matrix[3][4][8];
	unsigned char right_p_matrix[3][4][8];
	unsigned char square_len[8];
	unsigned char focal_len[8];
	unsigned char pixel_pitch[8];
	uint16_t left_r;
	uint16_t left_b;
	uint16_t left_gb;
	uint16_t left_af_far;
	uint16_t left_af_mid;
	uint16_t left_af_short;
	uint16_t left_af_5um;
	uint16_t left_af_50up;
	uint16_t left_af_50down;
	uint16_t right_r;
	uint16_t right_b;
	uint16_t right_gb;
	uint16_t right_af_far;
	uint16_t right_af_mid;
	uint16_t right_af_short;
	uint16_t right_af_5um;
	uint16_t right_af_50up;
	uint16_t right_af_50down;
};

struct sensor_init_cfg {
	uint8_t prev_res;
	uint8_t pict_res;
};

struct sensor_calib_data {
	/* Color Related Measurements */
	uint16_t r_over_g;
	uint16_t b_over_g;
	uint16_t gr_over_gb;

	/* Lens Related Measurements */
	uint16_t macro_2_inf;
	uint16_t inf_2_macro;
	uint16_t stroke_amt;
	uint16_t af_pos_1m;
	uint16_t af_pos_inf;
};

struct sensor_large_data {
	int cfgtype;
	union {
		struct sensor_3d_cali_data_t sensor_3d_cali_data;
	} data;
};
#endif

#ifdef CONFIG_CAMERA_3D
enum bestshot_mode {
  BESTSHOT_OFF = 0,
  BESTSHOT_LANDSCAPE = 1,
  BESTSHOT_SNOW,
  BESTSHOT_BEACH,
  BESTSHOT_SUNSET,
  BESTSHOT_NIGHT, /*5*/
  BESTSHOT_PORTRAIT,
  BESTSHOT_BACKLIGHT,
  BESTSHOT_SPORTS,
  BESTSHOT_ANTISHAKE,
  BESTSHOT_FLOWERS, /*10*/
  BESTSHOT_CANDLELIGHT,
  BESTSHOT_FIREWORKS,
  BESTSHOT_PARTY,
  BESTSHOT_NIGHT_PORTRAIT,
  BESTSHOT_THEATRE, /*15*/
  BESTSHOT_ACTION,
  BESTSHOT_AR,
  BESTSHOT_MAX
};

enum aec_awb_lock_mode{
	UNLOCK_AEC_UNLOCK_AWB_MODE,   /*default*/
	UNLOCK_AEC_LOCK_AWB_MODE,
	LOCK_AEC_UNLOCK_AWB_MODE,
	LOCK_AEC_LOCK_AWB_MODE,
};
#endif

struct sensor_cfg_data {
	int cfgtype;
	int mode;
	int rs;
	uint8_t max_steps;

	union {
		int8_t af_area;
		int8_t effect;
		uint8_t lens_shading;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
#ifdef CONFIG_CAMERA_3D
		struct sensor_init_cfg init_info;
#endif
		uint16_t flash_exp_div;
		uint16_t real_iso_value;
		uint16_t down_framerate;
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg exp_gain;
		struct focus_cfg focus;
		struct fps_cfg fps;
		struct wb_info_cfg wb_info;

#ifdef CONFIG_CAMERA_3D
		struct sensor_3d_exp_cfg sensor_3d_exp;
		struct sensor_calib_data calib_info;
#endif

		struct fuse_id fuse;
		struct lsc_cfg lsctable;/*Vincent for LSC calibration*/
		struct otp_cfg sp3d_otp_cfg;
		struct flash_cfg flash_data;
		struct exp_cfg exp_info;
		/* For 2nd CAM */
		enum antibanding_mode antibanding_value;
		enum brightness_t brightness_value;
		enum frontcam_t frontcam_value;
		enum wb_mode wb_value;
		enum iso_mode iso_value;
		enum sharpness_mode sharpness_value;
		enum saturation_mode saturation_value;
		enum contrast_mode  contrast_value;
		enum qtr_size_mode qtr_size_mode_value;
		enum sensor_af_mode af_mode_value;

#ifdef CONFIG_CAMERA_3D
		enum aec_awb_lock_mode aec_awb_lock_mode_value;
		enum bestshot_mode bestshot_mode_value;
#endif
	} cfg;
};




#define GET_NAME			0
#define GET_PREVIEW_LINE_PER_FRAME	1
#define GET_PREVIEW_PIXELS_PER_LINE	2
#define GET_SNAPSHOT_LINE_PER_FRAME	3
#define GET_SNAPSHOT_PIXELS_PER_LINE	4
#define GET_SNAPSHOT_FPS		5
#define GET_SNAPSHOT_MAX_EP_LINE_CNT	6

#endif /* __LINUX_MSM_CAMERA_H */
