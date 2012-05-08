/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
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
#ifdef CONFIG_MSM_CAMERA_LEGACY
#include <media/msm_camera.h>
#else
#include <media/msm_camera-7x30.h>
#endif

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define CDBG(fmt, args...) printk(KERN_INFO "[CAM] msm_camera: " fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define MSM_CAMERA_MSG 0
#define MSM_CAMERA_EVT 1
#define NUM_WB_EXP_NEUTRAL_REGION_LINES 4
#define NUM_WB_EXP_STAT_OUTPUT_BUFFERS  3
#define NUM_AUTOFOCUS_MULTI_WINDOW_GRIDS 16
#define NUM_STAT_OUTPUT_BUFFERS      3
#define NUM_AF_STAT_OUTPUT_BUFFERS      3
#define max_control_command_size 150

enum VFE_ModeOfOperation{
  VFE_MODE_OF_OPERATION_CONTINUOUS,
  VFE_MODE_OF_OPERATION_SNAPSHOT,
  VFE_MODE_OF_OPERATION_VIDEO,
  VFE_MODE_OF_OPERATION_RAW_SNAPSHOT,
  VFE_MODE_OF_OPERATION_MULTISHOT,
  VFE_LAST_MODE_OF_OPERATION_ENUM
};

enum msm_queue {
	MSM_CAM_Q_CTRL,     /* control command or control command status */
	MSM_CAM_Q_VFE_EVT,  /* adsp event */
	MSM_CAM_Q_VFE_MSG,  /* adsp message */
	MSM_CAM_Q_V4L2_REQ, /* v4l2 request */
	MSM_CAM_Q_VPE_MSG,  /* adsp message */
};

enum vfe_resp_msg {
	VFE_EVENT,
	VFE_MSG_GENERAL,
	VFE_MSG_SNAPSHOT,
#ifdef CONFIG_MSM_CAMERA_LEGACY
#ifndef CONFIG_720P_CAMERA
	VFE_MSG_OUTPUT1,
	VFE_MSG_OUTPUT2,
#else
	VFE_MSG_OUTPUT_P,   /* preview (continuous mode ) */
	VFE_MSG_OUTPUT_T,   /* thumbnail (snapshot mode )*/
	VFE_MSG_OUTPUT_S,   /* main image (snapshot mode )*/
	VFE_MSG_OUTPUT_V,   /* video   (continuous mode ) */
#endif
	VFE_MSG_STATS_AF,
	VFE_MSG_STATS_WE,
#else /* CONFIG_MSM_CAMERA_7X30 */
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
#endif


};

enum vpe_resp_msg {
       VPE_EVENT,
       VPE_MSG_GENERAL,
       VPE_MSG_SNAPSHOT,
       VPE_MSG_OUTPUT_P,   /* preview (continuous mode ) */
       VPE_MSG_OUTPUT_T,   /* thumbnail (snapshot mode )*/
       VPE_MSG_OUTPUT_S,   /* main image (snapshot mode )*/
       VPE_MSG_OUTPUT_V,   /* video   (continuous mode ) */
};

struct msm_vpe_phy_info {
       uint32_t sbuf_phy;
       uint32_t y_phy;
       uint32_t cbcr_phy;
       uint8_t  output_id; /* VFE31_OUTPUT_MODE_PT/S/V */
       uint32_t frame_id;
};

#define VFE31_OUTPUT_MODE_PT (0x1 << 0)
#define VFE31_OUTPUT_MODE_S (0x1 << 1)
#define VFE31_OUTPUT_MODE_V (0x1 << 2)

struct msm_vfe_phy_info {
	uint32_t sbuf_phy;
	uint32_t y_phy;
	uint32_t cbcr_phy;
	uint8_t  output_id; /* VFE31_OUTPUT_MODE_PT/S/V */
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
       struct   video_crop_t vpe_crop;
};

struct msm_vfe_resp {
	enum vfe_resp_msg type;
	struct msm_vfe_evt_msg evt_msg;
	struct msm_vfe_phy_info phy;
	struct msm_vpe_buf_info vpe_bf;
	void    *extdata;
	int32_t extlen;
};

struct msm_vpe_evt_msg {
	unsigned short type; /* 1 == event (RPC), 0 == message (adsp) */
	unsigned short msg_id;
	unsigned int len; /* size in, number of bytes out */
	uint32_t frame_id;
	void *data;
};

struct msm_vpe_resp {
	enum vpe_resp_msg type;
	struct msm_vpe_evt_msg evt_msg;
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
	int (*vfe_init)(struct msm_vfe_callback *, struct platform_device *);
	int (*vfe_enable)(struct camera_enable_cmd *);
	int (*vfe_config)(struct msm_vfe_cfg_cmd *, void *);
	int (*vfe_disable)(struct camera_enable_cmd *,
		struct platform_device *dev);
	void (*vfe_release)(struct platform_device *);
};

struct msm_vpe_cfg_cmd {
	int cmd_type;
	uint16_t length;
	void *value;
};

struct msm_camvpe_fn {
	int (*vpe_reg)(struct msm_vpe_callback *);
	int (*vpe_cfg_update) (void *);
	void (*send_frame_to_vpe) (uint32_t y_phy, uint32_t cbcr_phy,
		struct timespec *ts);
	int (*vpe_config)(struct msm_vpe_cfg_cmd *, void *);
	int *dis;
};

struct msm_sensor_ctrl {
	int (*s_init)(struct msm_camera_sensor_info *);
	int (*s_release)(void);
	int (*s_config)(void __user *);
	int node;
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
	/* These two queues are accessed from a process context only.  They contain
	 * pmem descriptors for the preview frames and the stats coming from the
	 * camera sensor.
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
	atomic_t send_output_s;

	atomic_t num_drop_output_s; /* num of snapshot frames to drop */
	atomic_t has_dropped_output_s; /* whether the latest snapshot frame was dropped or not. */

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
	struct wake_lock wake_suspend_lock;
	struct wake_lock wake_lock;
	struct platform_device *pdev;
	int16_t ignore_qcmd_type;
	uint8_t ignore_qcmd;
	uint8_t opencnt;
	void *cropinfo;
	int  croplen;

	atomic_t vpe_enable;
	uint32_t pp_mask;
	uint8_t pp_frame_avail;
	struct msm_queue_cmd *pp_prev;
	struct msm_queue_cmd *pp_snap;
	struct msm_queue_cmd *pp_thumb;
	int video_fd;

	/* When this flag is set, we send preview-frame notifications to config
	 * as well as to the frame queue.  By default, the flag is cleared.
	 */
	uint32_t report_preview_to_config;

	const char *apps_id;

	struct mutex lock;
	struct list_head list;
	uint8_t liveshot_enabled;

	spinlock_t pmem_frame_spinlock;
	spinlock_t pmem_stats_spinlock;
	spinlock_t abort_pict_lock;
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
	unsigned long kvaddr;
	unsigned long len;
	struct file *file;
	struct msm_pmem_info info;
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
	static inline int msm_strobe_flash_init(
		struct msm_sync *sync, uint32_t sftype)
	{
		return -ENOTSUPP;
	}
#else
	static inline int msm_camera_flash_set_led_state(
		struct msm_camera_sensor_flash_data *fdata,
		unsigned led_state)
	{
		return -ENOTSUPP;
	}
#endif

/* Below functions are added for V4L2 kernel APIs */
struct msm_v4l2_driver {
	struct msm_sync *sync;
	int (*open)(struct msm_sync *, const char *apps_id);
	int (*release)(struct msm_sync *);
	int (*ctrl)(struct msm_sync *, struct msm_ctrl_cmd *);
	int (*reg_pmem)(struct msm_sync *, struct msm_pmem_info *);
	int (*get_frame) (struct msm_sync *, struct msm_frame *);
	int (*put_frame) (struct msm_sync *, struct msm_frame *);
	int (*get_pict) (struct msm_sync *, struct msm_ctrl_cmd *);
	unsigned int (*drv_poll) (struct msm_sync *, struct file *,
				struct poll_table_struct *);
};

int msm_v4l2_register(struct msm_v4l2_driver *);
int msm_v4l2_unregister(struct msm_v4l2_driver *);

void msm_camvfe_init(void);
int msm_camvfe_check(void *);
void msm_camvfe_fn_init(struct msm_camvfe_fn *, void *);
void msm_camvpe_fn_init(struct msm_camvpe_fn *, void *);
int msm_camera_drv_start(struct platform_device *dev,
		int (*sensor_probe)(struct msm_camera_sensor_info *,
					struct msm_sensor_ctrl *));

enum msm_camio_clk_type {
	CAMIO_VFE_MDC_CLK,
	CAMIO_MDC_CLK,
	CAMIO_VFE_CLK,
	CAMIO_VFE_AXI_CLK,
	CAMIO_VFE_CLK_FOR_MIPI_2_LANE,
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
	CAMIO_JPEG_CLK,
	CAMIO_JPEG_PCLK,
	CAMIO_VPE_CLK,
	CAMIO_VPE_PCLK,
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

int msm_camio_enable(struct platform_device *dev);

int  msm_camio_clk_enable(enum msm_camio_clk_type clk);
int  msm_camio_clk_disable(enum msm_camio_clk_type clk);
int  msm_camio_clk_config(uint32_t freq);
void msm_camio_clk_rate_set(int rate);
void msm_camio_clk_axi_rate_set(int rate);

void msm_camio_camif_pad_reg_reset(void);
void msm_camio_camif_pad_reg_reset_2(void);

void msm_camio_vfe_blk_reset(void);

void msm_camio_clk_sel(enum msm_camio_clk_src_type);
void msm_camio_disable(struct platform_device *);
int msm_camio_probe_on(struct platform_device *);
int msm_camio_probe_off(struct platform_device *);

#ifdef CONFIG_MSM_CAMERA_7X30
void msm_camio_clk_rate_set_2(struct clk *clk, int rate);
void msm_disable_io_gpio_clk(struct platform_device *);
int msm_camio_read_camif_status(void);
#endif

#ifdef CONFIG_MSM_CAMERA_8X60
int msm_camio_vpe_clk_enable(void);
int msm_camio_vpe_clk_disable(void);
void msm_camio_clk_rate_set_2(struct clk *clk, int rate);
void msm_camio_clk_set_min_rate(struct clk *clk, int rate);
void msm_disable_io_gpio_clk(struct platform_device *);
int msm_camio_sensor_clk_off(struct platform_device *);
int msm_camio_sensor_clk_on(struct platform_device *);
#endif

int msm_camio_jpeg_clk_enable(void);
int msm_camio_jpeg_clk_disable(void);

int add_axi_qos(void);
int request_axi_qos(uint32_t freq);
int update_axi_qos(uint32_t freq);
void release_axi_qos(void);

int msm_camio_csi_config(struct msm_camera_csi_params *csi_params);
void msm_io_w(u32 data, void __iomem *addr);
void msm_io_w_mb(u32 data, void __iomem *addr);
u32 msm_io_r(void __iomem *addr);
u32 msm_io_r_mb(void __iomem *addr);
void msm_io_dump(void __iomem *addr, int size);
void msm_io_memcpy(void __iomem *dest_addr, void __iomem *src_addr, u32 len);
void msm_camio_set_perf_lvl(enum msm_bus_perf_setting);

void msm_camio_disable_csi_log(void);
#endif
