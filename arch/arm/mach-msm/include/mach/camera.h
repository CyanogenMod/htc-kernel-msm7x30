/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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
 */

#ifndef __ASM__ARCH_CAMERA_H
#define __ASM__ARCH_CAMERA_H

#include <linux/list.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include "linux/types.h"

#include <mach/board.h>
#include <media/msm_camera.h>
#include <mach/msm_subsystem_map.h>

#define CONFIG_MSM_CAMERA_DEBUG
#ifdef CONFIG_MSM_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define PAD_TO_2K(a, b) ((!b) ? a : (((a)+2047) & ~2047))

#define MSM_CAMERA_MSG 0
#define MSM_CAMERA_EVT 1
#define NUM_WB_EXP_NEUTRAL_REGION_LINES 4
#define NUM_WB_EXP_STAT_OUTPUT_BUFFERS  3
#define NUM_AUTOFOCUS_MULTI_WINDOW_GRIDS 16
#define NUM_STAT_OUTPUT_BUFFERS      3
#define NUM_AF_STAT_OUTPUT_BUFFERS      3
#define max_control_command_size 260
#define CROP_LEN 36

enum vfe_mode_of_operation{
	VFE_MODE_OF_OPERATION_CONTINUOUS,
	VFE_MODE_OF_OPERATION_SNAPSHOT,
	VFE_MODE_OF_OPERATION_VIDEO,
	VFE_MODE_OF_OPERATION_RAW_SNAPSHOT,
	VFE_MODE_OF_OPERATION_ZSL,
	VFE_LAST_MODE_OF_OPERATION_ENUM
};

enum msm_queue {
	MSM_CAM_Q_CTRL,     /* control command or control command status */
	MSM_CAM_Q_VFE_EVT,  /* adsp event */
	MSM_CAM_Q_VFE_MSG,  /* adsp message */
	MSM_CAM_Q_V4L2_REQ, /* v4l2 request */
	MSM_CAM_Q_VPE_MSG,  /* vpe message */
	MSM_CAM_Q_PP_MSG,  /* pp message */
};

enum vfe_resp_msg {
	VFE_EVENT,
	VFE_MSG_GENERAL,
	VFE_MSG_SNAPSHOT,
	VFE_MSG_OUTPUT_P,   /* preview (continuous mode ) */
	VFE_MSG_OUTPUT_T,   /* thumbnail (snapshot mode )*/
	VFE_MSG_OUTPUT_S,   /* main image (snapshot mode )*/
	VFE_MSG_OUTPUT_V,   /* video   (continuous mode ) */
	VFE_MSG_STATS_AEC,
	VFE_MSG_STATS_AF,
	VFE_MSG_STATS_AWB,
	VFE_MSG_STATS_RS,
	VFE_MSG_STATS_CS,
	VFE_MSG_STATS_IHIST,
	VFE_MSG_STATS_SKIN,
	VFE_MSG_STATS_WE, /* AEC + AWB */
	VFE_MSG_SYNC_TIMER0,
	VFE_MSG_SYNC_TIMER1,
	VFE_MSG_SYNC_TIMER2,
	VFE_MSG_COMMON,
	VFE_MSG_V32_START,
	VFE_MSG_V32_START_RECORDING,
	VFE_MSG_V32_CAPTURE,
	VFE_MSG_OUTPUT_IRQ,
};

enum vpe_resp_msg {
	VPE_MSG_GENERAL,
	VPE_MSG_OUTPUT_V,   /* video   (continuous mode ) */
	VPE_MSG_OUTPUT_ST_L,
	VPE_MSG_OUTPUT_ST_R,
};

enum msm_stereo_state {
	STEREO_VIDEO_IDLE,
	STEREO_VIDEO_ACTIVE,
	STEREO_SNAP_IDLE,
	STEREO_SNAP_STARTED,
	STEREO_SNAP_BUFFER1_PROCESSING,
	STEREO_SNAP_BUFFER2_PROCESSING,
	STEREO_RAW_SNAP_IDLE,
	STEREO_RAW_SNAP_STARTED,
};

enum msm_ispif_intftype {
	PIX0,
	RDI0,
	PIX1,
	RDI1,
	PIX2,
	RDI2,
};

enum msm_ispif_vc {
	VC0,
	VC1,
	VC2,
	VC3,
};

enum msm_ispif_cid {
	CID0,
	CID1,
	CID2,
	CID3,
	CID4,
	CID5,
	CID6,
	CID7,
	CID8,
	CID9,
	CID10,
	CID11,
	CID12,
	CID13,
	CID14,
	CID15,
};

struct msm_ispif_params {
	uint8_t intftype;
	uint16_t cid_mask;
	uint8_t csid;
};

struct msm_ispif_params_list {
	uint32_t len;
	struct msm_ispif_params params[3];
};

struct msm_vpe_phy_info {
	uint32_t sbuf_phy;
	uint32_t y_phy;
	uint32_t cbcr_phy;
	uint8_t  output_id; /* VFE31_OUTPUT_MODE_PT/S/V */
	uint32_t frame_id;
};

struct msm_camera_csid_vc_cfg {
	uint8_t cid;
	uint8_t dt;
	uint8_t decode_format;
};

struct msm_camera_csid_lut_params {
	uint8_t num_cid;
	struct msm_camera_csid_vc_cfg *vc_cfg;
};

struct msm_camera_csid_params {
	uint8_t lane_cnt;
	uint8_t lane_assign;
	struct msm_camera_csid_lut_params lut_params;
};

struct msm_camera_csiphy_params {
	uint8_t lane_cnt;
	uint8_t settle_cnt;
};

struct msm_camera_csi2_params {
	struct msm_camera_csid_params csid_params;
	struct msm_camera_csiphy_params csiphy_params;
};

#define VFE31_OUTPUT_MODE_PT (0x1 << 0)
#define VFE31_OUTPUT_MODE_S (0x1 << 1)
#define VFE31_OUTPUT_MODE_V (0x1 << 2)
#define VFE31_OUTPUT_MODE_P (0x1 << 3)
#define VFE31_OUTPUT_MODE_T (0x1 << 4)

#define CSI_EMBED_DATA 0x12
#define CSI_RAW8    0x2A
#define CSI_RAW10   0x2B
#define CSI_RAW12   0x2C

#define CSI_DECODE_6BIT 0
#define CSI_DECODE_8BIT 1
#define CSI_DECODE_10BIT 2

struct msm_vfe_phy_info {
	uint32_t sbuf_phy;
	uint32_t y_phy;
	uint32_t cbcr_phy;
	uint8_t  output_id; /* VFE31_OUTPUT_MODE_PT/S/V */
	uint32_t frame_id;
};

struct msm_vfe_stats_msg {
	uint8_t awb_ymin;
	uint32_t aec_buff;
	uint32_t awb_buff;
	uint32_t af_buff;
	uint32_t ihist_buff;
	uint32_t rs_buff;
	uint32_t cs_buff;
	uint32_t skin_buff;
	uint32_t status_bits;
	uint32_t frame_id;
};

struct video_crop_t{
	uint32_t  in1_w;
	uint32_t  out1_w;
	uint32_t  in1_h;
	uint32_t  out1_h;
	uint32_t  in2_w;
	uint32_t  out2_w;
	uint32_t  in2_h;
	uint32_t  out2_h;
	uint8_t update_flag;
};

struct msm_vpe_buf_info {
	uint32_t y_phy;
	uint32_t cbcr_phy;
	struct   timespec ts;
	uint32_t frame_id;
	struct	 video_crop_t vpe_crop;
};

struct msm_vfe_resp {
	enum vfe_resp_msg type;
	struct msm_cam_evt_msg evt_msg;
	struct msm_vfe_phy_info phy;
	struct msm_vfe_stats_msg stats_msg;
	struct msm_vpe_buf_info vpe_bf;
	void    *extdata;
	int32_t extlen;
};

struct msm_vpe_resp {
	enum vpe_resp_msg type;
	struct msm_cam_evt_msg evt_msg;
	struct msm_vpe_phy_info phy;
	void    *extdata;
	int32_t extlen;
};

struct msm_vpe_callback {
	void (*vpe_resp)(struct msm_vpe_resp *,
					enum msm_queue, void *syncdata,
		void *time_stamp, gfp_t gfp);
	void* (*vpe_alloc)(int, void *syncdata, gfp_t gfp);
	void (*vpe_free)(void *ptr);
};

struct msm_vfe_callback {
	void (*vfe_resp)(struct msm_vfe_resp *,
		enum msm_queue, void *syncdata,
		gfp_t gfp);
	void* (*vfe_alloc)(int, void *syncdata, gfp_t gfp);
	void (*vfe_free)(void *ptr);
};

struct msm_camvfe_fn {
	int (*vfe_init)(struct msm_vfe_callback *,
			struct platform_device *);
	int (*vfe_enable)(struct camera_enable_cmd *);
	int (*vfe_config)(struct msm_vfe_cfg_cmd *, void *);
	int (*vfe_disable)(struct camera_enable_cmd *,
			struct platform_device *dev);
	void (*vfe_release)(struct platform_device *);
	void (*vfe_stop)(void);
};

struct msm_camvfe_params {
	struct msm_vfe_cfg_cmd *vfe_cfg;
	void *data;
};

struct msm_mctl_pp_params {
	struct msm_mctl_pp_cmd *cmd;
	void *data;
};

struct msm_camvpe_fn {
	int (*vpe_reg)(struct msm_vpe_callback *);
	int (*vpe_cfg_update) (void *);
	void (*send_frame_to_vpe) (uint32_t y_phy, uint32_t cbcr_phy,
		struct timespec *ts, int output_id);
	int (*vpe_config)(struct msm_vpe_cfg_cmd *, void *);
	void (*vpe_cfg_offset)(int frame_pack, uint32_t pyaddr,
		uint32_t pcbcraddr, struct timespec *ts, int output_id,
		struct msm_st_half st_half, int frameid);
	int *dis;
};

struct msm_sensor_ctrl {
	int (*s_init)(const struct msm_camera_sensor_info *);
	int (*s_release)(void);
	int (*s_config)(void __user *);
	enum msm_camera_type s_camera_type;
	uint32_t s_mount_angle;
	enum msm_st_frame_packing s_video_packing;
	enum msm_st_frame_packing s_snap_packing;
};

struct msm_actuator_ctrl {
	int (*a_init_table)(void);
	int (*a_power_down)(void);
	int (*a_create_subdevice)(void *, void *);
	int (*a_config)(void __user *);
};

struct msm_strobe_flash_ctrl {
	int (*strobe_flash_init)
		(struct msm_camera_sensor_strobe_flash_data *);
	int (*strobe_flash_release)
		(struct msm_camera_sensor_strobe_flash_data *, int32_t);
	int (*strobe_flash_charge)(int32_t, int32_t, uint32_t);
};

/* this structure is used in kernel */
struct msm_queue_cmd {
	struct list_head list_config;
	struct list_head list_control;
	struct list_head list_frame;
	struct list_head list_pict;
	struct list_head list_vpe_frame;
	enum msm_queue type;
	void *command;
	atomic_t on_heap;
	struct timespec ts;
	uint32_t error_code;
};

struct msm_device_queue {
	struct list_head list;
	spinlock_t lock;
	wait_queue_head_t wait;
	int max;
	int len;
	const char *name;
};

struct msm_sync {
	/* These two queues are accessed from a process context only
	 * They contain pmem descriptors for the preview frames and the stats
	 * coming from the camera sensor.
	*/
	struct hlist_head pmem_frames;
	struct hlist_head pmem_stats;

	/* The message queue is used by the control thread to send commands
	 * to the config thread, and also by the DSP to send messages to the
	 * config thread.  Thus it is the only queue that is accessed from
	 * both interrupt and process context.
	 */
	struct msm_device_queue event_q;

	/* This queue contains preview frames. It is accessed by the DSP (in
	 * in interrupt context, and by the frame thread.
	 */
	struct msm_device_queue frame_q;
	int unblock_poll_frame;
	int unblock_poll_pic_frame;

	/* This queue contains snapshot frames.  It is accessed by the DSP (in
	 * interrupt context, and by the control thread.
	 */
	struct msm_device_queue pict_q;
	int get_pic_abort;
	struct msm_device_queue vpe_q;

	struct msm_camera_sensor_info *sdata;
	struct msm_camvfe_fn vfefn;
	struct msm_camvpe_fn vpefn;
	struct msm_sensor_ctrl sctrl;
	struct msm_strobe_flash_ctrl sfctrl;
	struct msm_actuator_ctrl actctrl;
	struct wake_lock wake_lock;
	struct platform_device *pdev;
	int16_t ignore_qcmd_type;
	uint8_t ignore_qcmd;
	uint8_t opencnt;
	void *cropinfo;
	int  croplen;
	int  core_powered_on;

	struct fd_roi_info fdroiinfo;

	atomic_t vpe_enable;
	uint32_t pp_mask;
	uint8_t pp_frame_avail;
	struct msm_queue_cmd *pp_prev;
	struct msm_queue_cmd *pp_snap;
	struct msm_queue_cmd *pp_thumb;
	int video_fd;

	const char *apps_id;

	struct mutex lock;
	struct list_head list;
	uint8_t liveshot_enabled;
	struct msm_cam_v4l2_device *pcam_sync;

	uint8_t stereocam_enabled;
	struct msm_queue_cmd *pp_stereocam;
	struct msm_queue_cmd *pp_stereocam2;
	struct msm_queue_cmd *pp_stereosnap;
	enum msm_stereo_state stereo_state;
	int stcam_quality_ind;
	uint32_t stcam_conv_value;

	spinlock_t pmem_frame_spinlock;
	spinlock_t pmem_stats_spinlock;
	spinlock_t abort_pict_lock;
	int snap_count;
	int thumb_count;
};

#define MSM_APPS_ID_V4L2 "msm_v4l2"
#define MSM_APPS_ID_PROP "msm_qct"

struct msm_cam_device {
	struct msm_sync *sync; /* most-frequently accessed */
	struct device *device;
	struct cdev cdev;
	/* opened is meaningful only for the config and frame nodes,
	 * which may be opened only once.
	 */
	atomic_t opened;
};

struct msm_control_device {
	struct msm_cam_device *pmsm;

	/* Used for MSM_CAM_IOCTL_CTRL_CMD_DONE responses */
	uint8_t ctrl_data[max_control_command_size];
	struct msm_ctrl_cmd ctrl;
	struct msm_queue_cmd qcmd;

	/* This queue used by the config thread to send responses back to the
	 * control thread.  It is accessed only from a process context.
	 */
	struct msm_device_queue ctrl_q;
};

struct register_address_value_pair {
	uint16_t register_address;
	uint16_t register_value;
};

struct msm_pmem_region {
	struct hlist_node list;
	unsigned long paddr;
	unsigned long len;
	struct file *file;
	struct msm_pmem_info info;
	struct msm_mapped_buffer *msm_buffer;
	int subsys_id;
};

struct axidata {
	uint32_t bufnum1;
	uint32_t bufnum2;
	uint32_t bufnum3;
	struct msm_pmem_region *region;
};

#ifdef CONFIG_MSM_CAMERA_FLASH
int msm_camera_flash_set_led_state(
	struct msm_camera_sensor_flash_data *fdata,
	unsigned led_state);
int msm_strobe_flash_init(struct msm_sync *sync, uint32_t sftype);
int msm_flash_ctrl(struct msm_camera_sensor_info *sdata,
			struct flash_ctrl_data *flash_info);
#else
static inline int msm_camera_flash_set_led_state(
	struct msm_camera_sensor_flash_data *fdata,
	unsigned led_state)
{
	return -ENOTSUPP;
}
static inline int msm_strobe_flash_init(
	struct msm_sync *sync, uint32_t sftype)
{
	return -ENOTSUPP;
}
static inline int msm_flash_ctrl(
		struct msm_camera_sensor_info *sdata,
		struct flash_ctrl_data *flash_info)
{
	return -ENOTSUPP;
}
#endif



void msm_camvfe_init(void);
int msm_camvfe_check(void *);
void msm_camvfe_fn_init(struct msm_camvfe_fn *, void *);
void msm_camvpe_fn_init(struct msm_camvpe_fn *, void *);
int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(const struct msm_camera_sensor_info *,
					struct msm_sensor_ctrl *));

enum msm_camio_clk_type {
	CAMIO_VFE_MDC_CLK,
	CAMIO_MDC_CLK,
	CAMIO_VFE_CLK,
	CAMIO_VFE_AXI_CLK,

	CAMIO_VFE_CAMIF_CLK,
	CAMIO_VFE_PBDG_CLK,
	CAMIO_CAM_MCLK_CLK,
	CAMIO_CAMIF_PAD_PBDG_CLK,

	CAMIO_CSI0_VFE_CLK,
	CAMIO_CSI1_VFE_CLK,
	CAMIO_VFE_PCLK,

	CAMIO_CSI_SRC_CLK,
	CAMIO_CSI0_CLK,
	CAMIO_CSI1_CLK,
	CAMIO_CSI0_PCLK,
	CAMIO_CSI1_PCLK,

	CAMIO_CSI1_SRC_CLK,
	CAMIO_CSI_PIX_CLK,
	CAMIO_CSI_PIX1_CLK,
	CAMIO_CSI_RDI_CLK,
	CAMIO_CSI_RDI1_CLK,
	CAMIO_CSI_RDI2_CLK,
	CAMIO_CSIPHY0_TIMER_CLK,
	CAMIO_CSIPHY1_TIMER_CLK,

	CAMIO_JPEG_CLK,
	CAMIO_JPEG_PCLK,
	CAMIO_VPE_CLK,
	CAMIO_VPE_PCLK,

	CAMIO_CSI0_PHY_CLK,
	CAMIO_CSI1_PHY_CLK,
	CAMIO_CSIPHY_TIMER_SRC_CLK,

	CAMIO_MAX_CLK
};

enum msm_camio_clk_src_type {
	MSM_CAMIO_CLK_SRC_INTERNAL,
	MSM_CAMIO_CLK_SRC_EXTERNAL,
	MSM_CAMIO_CLK_SRC_MAX
};

enum msm_s_test_mode {
	S_TEST_OFF,
	S_TEST_1,
	S_TEST_2,
	S_TEST_3
};

enum msm_s_resolution {
	S_QTR_SIZE,
	S_FULL_SIZE,
	S_INVALID_SIZE
};

enum msm_s_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	S_REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	S_UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	S_UPDATE_ALL,
	/* Not valid update */
	S_UPDATE_INVALID
};

enum msm_s_setting {
	S_RES_PREVIEW,
	S_RES_CAPTURE
};

enum msm_bus_perf_setting {
	S_INIT,
	S_PREVIEW,
	S_VIDEO,
	S_CAPTURE,
	S_ZSL,
	S_STEREO_VIDEO,
	S_STEREO_CAPTURE,
	S_DEFAULT,
	S_EXIT
};

enum msm_cam_mode {
	MODE_R,
	MODE_L,
	MODE_DUAL
};

int msm_camio_enable(struct platform_device *dev);
int msm_camio_jpeg_clk_enable(void);
int msm_camio_jpeg_clk_disable(void);
int msm_camio_vpe_clk_enable(uint32_t);
int msm_camio_vpe_clk_disable(void);

void msm_camio_mode_config(enum msm_cam_mode mode);
int  msm_camio_clk_enable(enum msm_camio_clk_type clk);
int  msm_camio_clk_disable(enum msm_camio_clk_type clk);
int  msm_camio_clk_config(uint32_t freq);
void msm_camio_clk_rate_set(int rate);
int msm_camio_vfe_clk_rate_set(int rate);
void msm_camio_clk_rate_set_2(struct clk *clk, int rate);
void msm_camio_clk_set_min_rate(struct clk *clk, int rate);
void msm_camio_clk_axi_rate_set(int rate);
void msm_disable_io_gpio_clk(struct platform_device *);

void msm_camio_camif_pad_reg_reset(void);
void msm_camio_camif_pad_reg_reset_2(void);

void msm_camio_vfe_blk_reset(void);

int32_t msm_camio_3d_enable(const struct msm_camera_sensor_info *sinfo);
void msm_camio_3d_disable(void);
void msm_camio_clk_sel(enum msm_camio_clk_src_type);
void msm_camio_disable(struct platform_device *);
int msm_camio_probe_on(struct platform_device *);
int msm_camio_probe_off(struct platform_device *);
int msm_camio_sensor_clk_off(struct platform_device *);
int msm_camio_sensor_clk_on(struct platform_device *);
int msm_camio_csi_config(struct msm_camera_csi_params *csi_params);
int msm_camio_csiphy_config(struct msm_camera_csiphy_params *csiphy_params);
int msm_camio_csid_config(struct msm_camera_csid_params *csid_params);
void msm_io_read_interrupt(void);
int add_axi_qos(void);
int update_axi_qos(uint32_t freq);
void release_axi_qos(void);
void msm_io_w(u32 data, void __iomem *addr);
void msm_io_w_mb(u32 data, void __iomem *addr);
u32 msm_io_r(void __iomem *addr);
u32 msm_io_r_mb(void __iomem *addr);
void msm_io_dump(void __iomem *addr, int size);
void msm_io_memcpy(void __iomem *dest_addr, void __iomem *src_addr, u32 len);
void msm_camio_set_perf_lvl(enum msm_bus_perf_setting);

void *msm_isp_sync_alloc(int size, gfp_t gfp);

void msm_isp_sync_free(void *ptr);
#endif
