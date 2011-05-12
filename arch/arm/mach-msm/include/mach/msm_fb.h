/* arch/arm/mach-msm/include/mach/msm_fb.h
 *
 * Internal shared definitions for various MSM framebuffer parts.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MSM_FB_H_
#define _MSM_FB_H_

#include <linux/device.h>

struct mddi_info;
struct mdp_device;

/* output interface format */
#define MSM_MDP_OUT_IF_FMT_RGB565 0
#define MSM_MDP_OUT_IF_FMT_RGB666 1
#define MSM_MDP_OUT_IF_FMT_RGB888 2

/* mdp override operations */
#define MSM_MDP_PANEL_IGNORE_PIXEL_DATA		(1 << 0)
#define MSM_MDP_PANEL_FLIP_UD			(1 << 1)
#define MSM_MDP_PANEL_FLIP_LR			(1 << 2)
#define MSM_MDP4_MDDI_DMA_SWITCH		(1 << 3)
#define MSM_MDP_DMA_PACK_ALIGN_LSB		(1 << 4)
#define MSM_MDP_RGB_PANEL_SELE_REFRESH		(1 << 5)
#define MSM_MDP_ABL_ENABLE			(1 << 6)

/* mddi type */
#define MSM_MDP_MDDI_TYPE_I	 0
#define MSM_MDP_MDDI_TYPE_II	 1


/* fb override operations */
#define MSM_FB_PM_DISABLE				(1<<0)

struct fb_cmap;
struct mdp_histogram;
struct gamma_curvy;

struct msm_fb_data {
	int xres;	/* x resolution in pixels */
	int yres;	/* y resolution in pixels */
	int width;	/* disply width in mm */
	int height;	/* display height in mm */
	unsigned output_format;
	unsigned overrides;
};

struct msmfb_callback {
	void (*func)(struct msmfb_callback *);
};

enum {
	MSM_MDDI_PMDH_INTERFACE = 0,
	MSM_MDDI_EMDH_INTERFACE,
	MSM_EBI2_INTERFACE,
	MSM_LCDC_INTERFACE,
	MSM_DTV_INTERFACE,
	MSM_TV_INTERFACE,

	MSM_MDP_NUM_INTERFACES = MSM_TV_INTERFACE + 1
};

#define MSMFB_CAP_PARTIAL_UPDATES	(1 << 0)
#define MSMFB_CAP_CABC			(1 << 1)

struct msm_lcdc_timing {
	unsigned int clk_rate;		/* dclk freq */
	unsigned int hsync_pulse_width;	/* in dclks */
	unsigned int hsync_back_porch;	/* in dclks */
	unsigned int hsync_front_porch;	/* in dclks */
	unsigned int hsync_skew;	/* in dclks */
	unsigned int vsync_pulse_width;	/* in lines */
	unsigned int vsync_back_porch;	/* in lines */
	unsigned int vsync_front_porch;	/* in lines */

	/* control signal polarity */
	unsigned int vsync_act_low:1;
	unsigned int hsync_act_low:1;
	unsigned int den_act_low:1;
};

struct msm_panel_data {
	/* turns off the fb memory */
	int (*suspend)(struct msm_panel_data *);
	/* turns on the fb memory */
	int (*resume)(struct msm_panel_data *);
	/* turns off the panel */
	int (*blank)(struct msm_panel_data *);
	/* turns on the panel */
	int (*unblank)(struct msm_panel_data *);
	/* for msmfb shutdown() */
	int (*shutdown)(struct msm_panel_data *);
	void (*wait_vsync)(struct msm_panel_data *);
	void (*request_vsync)(struct msm_panel_data *, struct msmfb_callback *);
	void (*clear_vsync)(struct msm_panel_data *);
	void (*dump_vsync)(void);
	/* change timing on the fly */
	int (*adjust_timing)(struct msm_panel_data *, struct msm_lcdc_timing *,
			u32 xres, u32 yres);
	/* FIXME */
	int (*recover_vsync)(struct msm_panel_data *);
	/* from the enum above */
	unsigned interface_type;
	/* data to be passed to the fb driver */
	struct msm_fb_data *fb_data;

	/* capabilities supported by the panel */
	uint32_t caps;
	/*
	 * For samsung driver IC, we always need to indicate where
	 * to draw. So we pass update_into to mddi client.
	 *
	 */
	struct {
		int left;
		int top;
		int eright; /* exclusive */
		int ebottom; /* exclusive */
	} update_info;
};

enum {
	MDP_DMA_P = 0,
	MDP_DMA_S,
};

struct mdp_reg {
    uint32_t reg;
    uint32_t val;
    uint32_t mask;
};

struct msm_mdp_platform_data {
	/* from the enum above */
	int dma_channel;
	unsigned overrides;
	unsigned color_format;
	int tearing_check;
	unsigned sync_config;
	unsigned sync_thresh;
	unsigned sync_start_pos;
	struct mdp_device *mdp_dev;
	struct gamma_curvy *abl_gamma_tbl;
};

struct msm_mddi_client_data {
	void (*suspend)(struct msm_mddi_client_data *);
	void (*resume)(struct msm_mddi_client_data *);
	void (*activate_link)(struct msm_mddi_client_data *);
	void (*remote_write)(struct msm_mddi_client_data *, uint32_t val,
			     uint32_t reg);
	void (*remote_write_vals)(struct msm_mddi_client_data *, uint8_t * val,
			     uint32_t reg, unsigned int nr_bytes);
	uint32_t (*remote_read)(struct msm_mddi_client_data *, uint32_t reg);
	void (*auto_hibernate)(struct msm_mddi_client_data *, int);
	void (*send_powerdown)(struct msm_mddi_client_data *);
	/* custom data that needs to be passed from the board file to a 
	 * particular client */
	void *private_client_data;
	struct resource *fb_resource;
	/* from the list above */
	unsigned interface_type;
};

struct msm_mddi_platform_data {
	unsigned int clk_rate;
	void (*power_client)(struct msm_mddi_client_data *, int on);

	/* fixup the mfr name, product id */
	void (*fixup)(uint16_t *mfr_name, uint16_t *product_id);

	struct resource *fb_resource; /*optional*/
	/* number of clients in the list that follows */
	int num_clients;
	unsigned type;
	/* array of client information of clients */
	struct {
		unsigned product_id; /* mfr id in top 16 bits, product id
				      * in lower 16 bits
				      */
		char *name;	/* the device name will be the platform
				 * device name registered for the client,
				 * it should match the name of the associated
				 * driver
				 */
		unsigned id;	/* id for mddi client device node, will also
				 * be used as device id of panel devices, if
				 * the client device will have multiple panels
				 * space must be left here for them
				 */
		void *client_data;	/* required private client data */
		unsigned int clk_rate;	/* optional: if the client requires a
					* different mddi clk rate
					*/
	} client_platform_data[];
};

struct msm_lcdc_panel_ops {
	int	(*init)(struct msm_lcdc_panel_ops *);
	int	(*uninit)(struct msm_lcdc_panel_ops *);
	int	(*blank)(struct msm_lcdc_panel_ops *);
	int	(*unblank)(struct msm_lcdc_panel_ops *);
	int	(*shutdown)(struct msm_lcdc_panel_ops *);
#ifdef CONFIG_PANEL_SELF_REFRESH
	int	(*refresh_enable)(struct msm_lcdc_panel_ops *);
	int	(*refresh_disable)(struct msm_lcdc_panel_ops *);
#endif
};

struct msm_lcdc_platform_data {
	struct msm_lcdc_panel_ops	*panel_ops;
	struct msm_lcdc_timing		*timing;
	int				fb_id;
	struct msm_fb_data		*fb_data;
	struct resource			*fb_resource;
};

struct msm_tvenc_platform_data {
	struct msm_tvenc_panel_ops	*panel_ops;
	int				fb_id;
	struct msm_fb_data		*fb_data;
	struct resource			*fb_resource;
	int (*video_relay)(int on_off);
};

struct mdp_blit_req;
struct fb_info;
struct mdp_overlay;
struct msmfb_overlay_data;

struct mdp_device {
	struct device dev;
	void (*dma)(struct mdp_device *mdp, uint32_t addr,
		    uint32_t stride, uint32_t w, uint32_t h, uint32_t x,
		    uint32_t y, struct msmfb_callback *callback, int interface);
	void (*dma_wait)(struct mdp_device *mdp, int interface);
	int (*blit)(struct mdp_device *mdp, struct fb_info *fb,
		    struct mdp_blit_req *req);
#ifdef CONFIG_FB_MSM_OVERLAY
	int (*overlay_get)(struct mdp_device *mdp, struct fb_info *fb,
		    struct mdp_overlay *req);
	int (*overlay_set)(struct mdp_device *mdp, struct fb_info *fb,
		    struct mdp_overlay *req);
	int (*overlay_unset)(struct mdp_device *mdp, struct fb_info *fb,
		    int ndx);
	int (*overlay_play)(struct mdp_device *mdp, struct fb_info *fb,
		    struct msmfb_overlay_data *req, struct file **p_src_file);
#endif
	void (*set_grp_disp)(struct mdp_device *mdp, uint32_t disp_id);
	void (*configure_dma)(struct mdp_device *mdp);
	int (*check_output_format)(struct mdp_device *mdp, int bpp);
	int (*set_output_format)(struct mdp_device *mdp, int bpp);
	void (*set_panel_size)(struct mdp_device *mdp, int width, int height);
#if defined (CONFIG_FB_MSM_MDP_ABL)
	int (*lut_update)(struct mdp_device *mdp, struct fb_info *fb,
		    struct fb_cmap *cmap);
	int (*do_histogram)(struct mdp_device *mdp, struct mdp_histogram *hist_in,
		    struct mdp_histogram *hist_out);
	int (*start_histogram)(struct mdp_device *mdp, struct fb_info *fb);
	int (*stop_histogram)(struct mdp_device *mdp, struct fb_info *fb);
	int (*get_gamma_curvy)(struct mdp_device *mdp, struct gamma_curvy *gc);
	struct gamma_curvy *abl_gamma_tbl;
#endif
	unsigned color_format;
	unsigned overrides;
	uint32_t width;		/*panel width*/
	uint32_t height;	/*panel height*/
};

struct class_interface;
int register_mdp_client(struct class_interface *class_intf);

int register_dtv_client(struct class_interface *class_intf);

/**** private client data structs go below this line ***/

/*
 * Panel private data, include backlight stuff
 * 9/28 09', Jay
 * */
struct panel_data {
	int panel_id;
	u32 caps;
	int shrink;
	/* backlight data */
	u8 *pwm;
	int min_level;
	/* default_br used in turn on backlight, must sync with setting in user space */
	int default_br;
	int vsync_gpio;
	int (*shrink_br)(int brightness);
	int (*change_cabcmode)(struct msm_mddi_client_data *client_data,
			int mode, u8 dimming);
};

struct msm_mddi_bridge_platform_data {
	/* from board file */
	int (*init)(struct msm_mddi_bridge_platform_data *,
		    struct msm_mddi_client_data *);
	int (*uninit)(struct msm_mddi_bridge_platform_data *,
		      struct msm_mddi_client_data *);
	/* passed to panel for use by the fb driver */
	int (*blank)(struct msm_mddi_bridge_platform_data *,
		     struct msm_mddi_client_data *);
	int (*unblank)(struct msm_mddi_bridge_platform_data *,
		       struct msm_mddi_client_data *);
	int (*shutdown)(struct msm_mddi_bridge_platform_data *,
		       struct msm_mddi_client_data *);
	struct msm_fb_data fb_data;
	struct panel_data panel_conf;
	/* for those MDDI client which need to re-position display region
	   after each update or static electricity strike. It should be
	   implemented in board-xxx-panel due to the function itself need to
	   send the screen dimensional info of screen to MDDI client.
	*/
	void (*adjust)(struct msm_mddi_client_data *);
#define SAMSUNG_D  0
#define SAMSUNG_S6 1
	int bridge_type;
	int panel_type;
	uint32_t caps;
	/* backlight data */
	u8 *pwm;
};

/*
 * This is used to communicate event between msm_fb, mddi, mddi_client, 
 * and board.
 * It's mainly used to reset the display system.
 * Also, it is used for battery power policy.
 *
 */
#define NOTIFY_MDDI     0x00000000
#define NOTIFY_POWER    0x00000001
#define NOTIFY_MSM_FB   0x00000010

extern int register_display_notifier(struct notifier_block *nb);
extern int display_notifier_call_chain(unsigned long val, void *data);
 
#define display_notifier(fn, pri) {                     \
	static struct notifier_block fn##_nb =          \
	{ .notifier_call = fn, .priority = pri };       \
	register_display_notifier(&fn##_nb);		\
}

#if (defined(CONFIG_USB_FUNCTION_PROJECTOR) || defined(CONFIG_USB_ANDROID_PROJECTOR))
/* For USB Projector to quick access the frame buffer info */
struct msm_fb_info {
	unsigned char *fb_addr;
	int msmfb_area;
	int xres;
	int yres;
};

extern int msmfb_get_var(struct msm_fb_info *tmp);
extern int msmfb_get_fb_area(void);
#if defined (CONFIG_FB_MSM_MDP_ABL)
extern struct mdp_histogram mdp_hist;
extern struct completion mdp_hist_comp;
#endif
#endif

#endif
