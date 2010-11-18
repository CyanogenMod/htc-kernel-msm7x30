/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */

#ifndef __LINUX_MSM_CAMERA_H
#define __LINUX_MSM_CAMERA_H

#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>

#define MSM_CAM_IOCTL_MAGIC 'm'

#define MSM_CAM_IOCTL_GET_SENSOR_INFO \
	_IOR(MSM_CAM_IOCTL_MAGIC, 1, struct msm_camsensor_info *)

#define MSM_CAM_IOCTL_REGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 2, struct msm_pmem_info *)

#define MSM_CAM_IOCTL_UNREGISTER_PMEM \
	_IOW(MSM_CAM_IOCTL_MAGIC, 3, unsigned)

#define MSM_CAM_IOCTL_CTRL_COMMAND \
	_IOW(MSM_CAM_IOCTL_MAGIC, 4, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_CONFIG_VFE  \
	_IOW(MSM_CAM_IOCTL_MAGIC, 5, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_STATS \
	_IOR(MSM_CAM_IOCTL_MAGIC, 6, struct msm_camera_stats_event_ctrl *)

#define MSM_CAM_IOCTL_GETFRAME \
	_IOR(MSM_CAM_IOCTL_MAGIC, 7, struct msm_camera_get_frame *)

#define MSM_CAM_IOCTL_ENABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 8, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_CTRL_CMD_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 9, struct camera_cmd *)

#define MSM_CAM_IOCTL_CONFIG_CMD \
	_IOW(MSM_CAM_IOCTL_MAGIC, 10, struct camera_cmd *)

#define MSM_CAM_IOCTL_DISABLE_VFE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 11, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_PAD_REG_RESET2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 12, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_VFE_APPS_RESET \
	_IOW(MSM_CAM_IOCTL_MAGIC, 13, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_FRAME_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 14, struct camera_enable_cmd *)

#define MSM_CAM_IOCTL_RELEASE_STATS_BUFFER \
	_IOW(MSM_CAM_IOCTL_MAGIC, 15, struct msm_stats_buf *)

#define MSM_CAM_IOCTL_AXI_CONFIG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 16, struct msm_camera_vfe_cfg_cmd *)

#define MSM_CAM_IOCTL_GET_PICTURE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 17, struct msm_camera_ctrl_cmd *)

#define MSM_CAM_IOCTL_SET_CROP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 18, struct crop_info *)

#define MSM_CAM_IOCTL_PP \
	_IOW(MSM_CAM_IOCTL_MAGIC, 19, uint8_t *)

#define MSM_CAM_IOCTL_PP_DONE \
	_IOW(MSM_CAM_IOCTL_MAGIC, 20, struct msm_snapshot_pp_status *)

#define MSM_CAM_IOCTL_SENSOR_IO_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 21, struct sensor_cfg_data *)

#define MSM_CAMERA_LED_OFF  0
#define MSM_CAMERA_LED_LOW  1
#define MSM_CAMERA_LED_HIGH 2
#define MSM_CAMERA_LED_LOW_FOR_SNAPSHOT 3

#define MSM_CAM_IOCTL_FLASH_LED_CFG \
	_IOW(MSM_CAM_IOCTL_MAGIC, 22, unsigned *)

#define MSM_CAM_IOCTL_UNBLOCK_POLL_FRAME \
	_IO(MSM_CAM_IOCTL_MAGIC, 23)

#define MSM_CAM_IOCTL_CTRL_COMMAND_2 \
	_IOW(MSM_CAM_IOCTL_MAGIC, 24, struct msm_ctrl_cmd *)

#define MSM_CAM_IOCTL_ENABLE_OUTPUT_IND \
	_IOW(MSM_CAM_IOCTL_MAGIC, 25, uint32_t *)

#define MAX_SENSOR_NUM  3
#define MAX_SENSOR_NAME 32

#define PP_SNAP		1
#define PP_RAW_SNAP	(1<<1)
#define PP_PREV		(1<<2)
#define PP_MASK		(PP_SNAP|PP_RAW_SNAP|PP_PREV)

#define MSM_CAM_CTRL_CMD_DONE  0
#define MSM_CAM_SENSOR_VFE_CMD 1

/*****************************************************
 *  structure
 *****************************************************/

/* define five type of structures for userspace <==> kernel
 * space communication:
 * command 1 - 2 are from userspace ==> kernel
 * command 3 - 4 are from kernel ==> userspace
 *
 * 1. control command: control command(from control thread),
 *                     control status (from config thread);
 */
struct msm_ctrl_cmd {
	uint16_t type;
	uint16_t length;
	void *value;
	uint16_t status;
	uint32_t timeout_ms;
	int resp_fd; /* FIXME: to be used by the kernel, pass-through for now */
};

struct msm_vfe_evt_msg {
	unsigned short type; /* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len; /* size in, number of bytes out */
	void *data;
	unsigned short exttype;
};

#define MSM_CAM_RESP_CTRL         0
#define MSM_CAM_RESP_STAT_EVT_MSG 1
#define MSM_CAM_RESP_V4L2         2
#define MSM_CAM_RESP_MAX          3

/* this one is used to send ctrl/status up to config thread */
struct msm_stats_event_ctrl {
	/* 0 - ctrl_cmd from control thread,
	 * 1 - stats/event kernel,
	 * 2 - V4L control or read request */
	int resptype;
	int timeout_ms;
	struct msm_ctrl_cmd ctrl_cmd;
	/* struct  vfe_event_t  stats_event; */
	struct msm_vfe_evt_msg stats_event;
};

/* 2. config command: config command(from config thread); */
struct msm_camera_cfg_cmd {
	/* what to config:
	 * 1 - sensor config, 2 - vfe config */
	uint16_t cfg_type;

	/* sensor config type */
	uint16_t cmd_type;
	uint16_t queue;
	uint16_t length;
	void *value;
};

#ifdef CONFIG_720P_CAMERA
#define CMD_GENERAL 0
#define CMD_AXI_CFG_SNAP    	    1
#define CMD_AXI_CFG_PREVIEW		    2
#define CMD_AXI_CFG_VIDEO		    3
#define CMD_RAW_PICT_AXI_CFG        4

#define CMD_FRAME_BUF_RELEASE       5
#define CMD_PREV_BUF_CFG            6
#define CMD_SNAP_BUF_RELEASE        7
#define CMD_SNAP_BUF_CFG            8
#define CMD_STATS_DISABLE           9
#define CMD_STATS_AEC_AWB_ENABLE   10
#define CMD_STATS_AF_ENABLE        11
#define CMD_STATS_AEC_ENABLE       12
#define CMD_STATS_AWB_ENABLE       13
#define CMD_STATS_ENABLE           14

#define CMD_STATS_AXI_CFG          15
#define CMD_STATS_AEC_AXI_CFG      16
#define CMD_STATS_AF_AXI_CFG       17
#define CMD_STATS_AWB_AXI_CFG      18
#define CMD_STATS_RS_AXI_CFG       19
#define CMD_STATS_CS_AXI_CFG       20
#define CMD_STATS_IHIST_AXI_CFG    21
#define CMD_STATS_SKIN_AXI_CFG     22

#define CMD_STATS_BUF_RELEASE       23
#define CMD_STATS_AEC_BUF_RELEASE   24
#define CMD_STATS_AF_BUF_RELEASE    25
#define CMD_STATS_AWB_BUF_RELEASE   26
#define CMD_STATS_RS_BUF_RELEASE    27
#define CMD_STATS_CS_BUF_RELEASE    28
#define CMD_STATS_IHIST_BUF_RELEASE 29
#define CMD_STATS_SKIN_BUF_RELEASE  30

#define UPDATE_STATS_INVALID        31
#else

//Just for build pass (Horng test)
//------------------------------------
#define CMD_AXI_CFG_SNAP    	    1
#define CMD_AXI_CFG_PREVIEW		    2
#define CMD_AXI_CFG_VIDEO		    3
//------------------------------------

#define CMD_GENERAL			0
#define CMD_AXI_CFG_OUT1		1
#define CMD_AXI_CFG_SNAP_O1_AND_O2	2
#define CMD_AXI_CFG_OUT2		3
#define CMD_PICT_T_AXI_CFG		4
#define CMD_PICT_M_AXI_CFG		5
#define CMD_RAW_PICT_AXI_CFG		6
#define CMD_STATS_AXI_CFG		7
#define CMD_STATS_AF_AXI_CFG		8
#define CMD_FRAME_BUF_RELEASE		9
#define CMD_PREV_BUF_CFG		10
#define CMD_SNAP_BUF_RELEASE		11
#define CMD_SNAP_BUF_CFG		12
#define CMD_STATS_DISABLE		13
#define CMD_STATS_AEC_AWB_ENABLE	14
#define CMD_STATS_AF_ENABLE		15
#define CMD_STATS_BUF_RELEASE		16
#define CMD_STATS_AF_BUF_RELEASE	17
#define CMD_STATS_ENABLE        18
#define UPDATE_STATS_INVALID		19

#endif

/* vfe config command: config command(from config thread)*/
struct msm_vfe_cfg_cmd {
	int cmd_type;
	uint16_t length;
	void *value;
};

#define MAX_CAMERA_ENABLE_NAME_LEN 32
struct camera_enable_cmd {
	char name[MAX_CAMERA_ENABLE_NAME_LEN];
};

#ifdef CONFIG_720P_CAMERA

#define MSM_PMEM_VIDEO        0
#define MSM_PMEM_PREVIEW      1
#define MSM_PMEM_THUMBNAIL    2
#define MSM_PMEM_MAINIMG      3
#define MSM_PMEM_RAW_MAINIMG  4
#define MSM_PMEM_AEC_AWB      5
#define MSM_PMEM_AF           6
#define MSM_PMEM_AEC          7
#define MSM_PMEM_AWB          8
#define MSM_PMEM_RS           9
#define MSM_PMEM_CS           10
#define MSM_PMEM_IHIST        11
#define MSM_PMEM_SKIN         12
#define MSM_PMEM_MAX          13

#else

//Just for build pass (Horng test)
//------------------------------------
#define MSM_PMEM_VIDEO        0
#define MSM_PMEM_PREVIEW      1
//------------------------------------

#define MSM_PMEM_OUTPUT1		0
#define MSM_PMEM_OUTPUT2		1
#define MSM_PMEM_OUTPUT1_OUTPUT2	2
#define MSM_PMEM_THUMBNAIL		3
#define MSM_PMEM_MAINIMG		4
#define MSM_PMEM_RAW_MAINIMG		5
#define MSM_PMEM_AEC_AWB		6
#define MSM_PMEM_AF			7
#define MSM_PMEM_MAX			8

#endif

#define FRAME_PREVIEW_OUTPUT1		0
#define FRAME_PREVIEW_OUTPUT2		1
#define FRAME_SNAPSHOT			2
#define FRAME_THUMBAIL			3
#define FRAME_RAW_SNAPSHOT		4
#define FRAME_MAX			5

struct msm_pmem_info {
	int type;
	int fd;
	void *vaddr;
	uint32_t offset;
	uint32_t len;
	uint32_t y_off; /* relative to offset */
	uint32_t cbcr_off; /* relative to offset */
	uint8_t vfe_can_write;
};

struct outputCfg {
	uint32_t height;
	uint32_t width;

	uint32_t window_height_firstline;
	uint32_t window_height_lastline;
};

#ifndef CONFIG_720P_CAMERA

#define OUTPUT_1	0
#define OUTPUT_2	1
#define OUTPUT_1_AND_2	2
#define CAMIF_TO_AXI_VIA_OUTPUT_2		3
#define OUTPUT_1_AND_CAMIF_TO_AXI_VIA_OUTPUT_2	4
#define OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1	5
#define LAST_AXI_OUTPUT_MODE_ENUM = OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1 6

#define MSM_FRAME_PREV_1	0
#define MSM_FRAME_PREV_2	1
#define MSM_FRAME_ENC		2

#else

#define OUTPUT_1 0
#define OUTPUT_2 1
#define OUTPUT_1_AND_2 2
#define OUTPUT_1_AND_3 3
#define CAMIF_TO_AXI_VIA_OUTPUT_2 4
#define OUTPUT_1_AND_CAMIF_TO_AXI_VIA_OUTPUT_2 5
#define OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1 6
#define LAST_AXI_OUTPUT_MODE_ENUM = OUTPUT_2_AND_CAMIF_TO_AXI_VIA_OUTPUT_1 7

#define MSM_FRAME_PREV_1	0
#define MSM_FRAME_PREV_2	1
#define MSM_FRAME_ENC		2

#define OUTPUT_TYPE_P  1
#define OUTPUT_TYPE_T  2
#define OUTPUT_TYPE_S  3
#define OUTPUT_TYPE_V  4

#endif

struct msm_frame {
	int path;
	unsigned long buffer;
	uint32_t y_off;
	uint32_t cbcr_off;
	int fd;

	void *cropinfo;
	int croplen;
};

#define STAT_AEAW	0
#define STAT_AF		1
#define STAT_MAX	2

struct msm_stats_buf {
	int type;
	unsigned long buffer;
	int fd;
};

#define MSM_V4L2_VID_CAP_TYPE	0
#define MSM_V4L2_STREAM_ON	1
#define MSM_V4L2_STREAM_OFF	2
#define MSM_V4L2_SNAPSHOT	3
#define MSM_V4L2_QUERY_CTRL	4
#define MSM_V4L2_GET_CTRL	5
#define MSM_V4L2_SET_CTRL	6
#define MSM_V4L2_QUERY		7
#define MSM_V4L2_GET_CROP 8
#define MSM_V4L2_SET_CROP 9
#define MSM_V4L2_MAX 10

#define V4L2_CAMERA_EXIT 43
struct crop_info {
	void *info;
	int len;
};

struct msm_postproc {
	int ftnum;
	struct msm_frame fthumnail;
	int fmnum;
	struct msm_frame fmain;
};

struct msm_snapshot_pp_status {
	void *status;
};

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
#define CFG_SET_OV_LSC	29 	/*vincent for LSC calibration*/
#define CFG_SET_SHARPNESS 30
#define CFG_SET_SATURATION 31
#define CFG_SET_OV_LSC_RAW_CAPTURE 32/*20100330 vincent for LSC calibration*/
#define CFG_SET_ISO			33
#define CFG_SET_COORDINATE		34
#define CFG_RUN_AUTO_FOCUS		35
#define CFG_CANCEL_AUTO_FOCUS		36
#define CFG_GET_EXP_FOR_LED		37
#define CFG_UPDATE_AEC_FOR_LED		38
#define CFG_SET_FRONT_CAMERA_MODE	39
#define CFG_MAX        			40

#define MOVE_NEAR	0
#define MOVE_FAR	1

#define SENSOR_PREVIEW_MODE		0
#define SENSOR_SNAPSHOT_MODE		1
#define SENSOR_RAW_SNAPSHOT_MODE	2
#define SENSOR_GET_EXP 3

#define SENSOR_QTR_SIZE			0
#define SENSOR_FULL_SIZE		1
#define SENSOR_INVALID_SIZE		2

#define CAMERA_EFFECT_OFF		0
#define CAMERA_EFFECT_MONO		1
#define CAMERA_EFFECT_NEGATIVE		2
#define CAMERA_EFFECT_SOLARIZE		3
#define CAMERA_EFFECT_PASTEL		4
#define CAMERA_EFFECT_MOSAIC		5
#define CAMERA_EFFECT_RESIZE		6
#define CAMERA_EFFECT_SEPIA		7
#define CAMERA_EFFECT_POSTERIZE		8
#define CAMERA_EFFECT_WHITEBOARD	9
#define CAMERA_EFFECT_BLACKBOARD	10
#define CAMERA_EFFECT_AQUA		11
#define CAMERA_EFFECT_MAX		12

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
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
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

enum sharpness_mode{
	CAMERA_SHARPNESS_X0,
	CAMERA_SHARPNESS_X1,
	CAMERA_SHARPNESS_X2,
	CAMERA_SHARPNESS_X3,
	CAMERA_SHARPNESS_X4,
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

struct sensor_cfg_data {
	int cfgtype;
	int mode;
	int rs;
	uint8_t max_steps;

	union {
		int8_t effect;
		uint8_t lens_shading;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg exp_gain;
		struct focus_cfg focus;
		struct fps_cfg fps;
		struct fuse_id fuse;
		struct lsc_cfg lsctable;/*Vincent for LSC calibration*/
		enum antibanding_mode antibanding_value;
		enum brightness_t brightness_value;
		enum frontcam_t frontcam_value;
		enum wb_mode wb_value;
		enum sharpness_mode sharpness_value;
		enum saturation_mode saturation_value;
		enum contrast_mode  contrast_value;
	} cfg;
};

#define GET_NAME			0
#define GET_PREVIEW_LINE_PER_FRAME	1
#define GET_PREVIEW_PIXELS_PER_LINE	2
#define GET_SNAPSHOT_LINE_PER_FRAME	3
#define GET_SNAPSHOT_PIXELS_PER_LINE	4
#define GET_SNAPSHOT_FPS		5
#define GET_SNAPSHOT_MAX_EP_LINE_CNT	6

struct msm_camsensor_info {
	char name[MAX_SENSOR_NAME];
	uint8_t flash_enabled;
};
#endif /* __LINUX_MSM_CAMERA_H */
