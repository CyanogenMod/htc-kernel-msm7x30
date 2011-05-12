/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MDP4_H
#define MDP4_H



#define MDP4_NONBLOCKING	/* enable non blocking ioctl */

#define MDP4_OVERLAYPROC0_BASE	0x10000
#define MDP4_OVERLAYPROC1_BASE	0x18000

#define MDP_OVERLAY0_TERM 0x20
#define MDP_OVERLAY1_TERM 0x40

#define MDP4_VIDEO_BASE 0x20000
#define MDP4_VIDEO_OFF 0x10000

#define MDP4_RGB_BASE 0x40000
#define MDP4_RGB_OFF 0x10000

enum {		/* display */
	PRIMARY_INTF_SEL,
	SECONDARY_INTF_SEL,
	EXTERNAL_INTF_SEL
};

enum {
	LCDC_RGB_INTF,			/* 0 */
	DTV_INTF = LCDC_RGB_INTF,	/* 0 */
	MDDI_LCDC_INTF,			/* 1 */
	MDDI_INTF,			/* 2 */
	EBI2_INTF,			/* 3 */
	TV_INTF = EBI2_INTF,		/* 3 */
	DSI_VIDEO_INTF,
	DSI_CMD_INTF
};

enum {
	MDDI_PRIMARY_SET,
	MDDI_SECONDARY_SET,
	MDDI_EXTERNAL_SET
};

enum {
	EBI2_LCD0,
	EBI2_LCD1
};

enum {
	OVERLAY_MODE_NONE,
	OVERLAY_MODE_BLT
};

enum {
	OVERLAY_REFRESH_ON_DEMAND,
	OVERLAY_REFRESH_VSYNC,
	OVERLAY_REFRESH_VSYNC_HALF,
	OVERLAY_REFRESH_VSYNC_QUARTER
};

enum {
	OVERLAY_FRAMEBUF,
	OVERLAY_DIRECTOUT
};

/* system interrupts */
#define INTR_OVERLAY0_DONE		BIT(0)
#define INTR_OVERLAY1_DONE		BIT(1)
#define INTR_DMA_S_DONE			BIT(2)
#define INTR_DMA_E_DONE			BIT(3)
#define INTR_DMA_P_DONE			BIT(4)
#define INTR_VG1_HISTOGRAM		BIT(5)
#define INTR_VG2_HISTOGRAM		BIT(6)
#define INTR_PRIMARY_VSYNC		BIT(7)
#define INTR_PRIMARY_INTF_UDERRUN	BIT(8)
#define INTR_EXTERNAL_VSYNC		BIT(9)
#define INTR_EXTERNAL_INTF_UDERRUN	BIT(10)
#define INTR_DMA_P_HISTOGRAM		BIT(17)
#define INTR_MDP_HIST_DONE       	BIT(20) //DMA_P histogram interrupt

/* histogram interrupts */
#define INTR_HIST_DONE			BIT(1)
#define INTR_HIST_RESET_SEQ_DONE	BIT(0)


#ifdef CONFIG_FB_MSM_OVERLAY
#define MDP4_ANY_INTR_MASK     (INTR_OVERLAY0_DONE| \
				INTR_PRIMARY_INTF_UDERRUN | \
				INTR_DMA_P_HISTOGRAM)
#else
#define MDP4_ANY_INTR_MASK	(INTR_DMA_P_DONE| \
				INTR_DMA_P_HISTOGRAM)
#endif
enum {
	OVERLAY_PIPE_RGB1,
	OVERLAY_PIPE_RGB2,
	OVERLAY_PIPE_VG1,	/* video/graphic */
	OVERLAY_PIPE_VG2,
	OVERLAY_PIPE_MAX
};

/* 2 VG pipes can be shared by RGB and VIDEO */
#define MDP4_MAX_PIPE   (OVERLAY_PIPE_MAX + 2)

#define OVERLAY_TYPE_RGB        0x01
#define OVERLAY_TYPE_VIDEO      0x02

enum {
	MDP4_MIXER0,
	MDP4_MIXER1,
	MDP4_MIXER_MAX
};

#define MDP4_MAX_MIXER	2

enum {
	OVERLAY_PLANE_INTERLEAVED,
	OVERLAY_PLANE_PLANAR,
	OVERLAY_PLANE_PSEUDO_PLANAR
};

enum {
	MDP4_MIXER_STAGE_UNUNSED,	/* pipe not used */
	MDP4_MIXER_STAGE_BASE,
	MDP4_MIXER_STAGE0,	/* zorder 0 */
	MDP4_MIXER_STAGE1,	/* zorder 1 */
	MDP4_MIXER_STAGE2	/* zorder 2 */
};

#define MDP4_MAX_STAGE	4

enum {
	MDP4_FRAME_FORMAT_LINEAR,
	MDP4_FRAME_FORMAT_ARGB_TILE,
	MDP4_FRAME_FORMAT_VIDEO_SUPERTILE
};

enum {
	MDP4_CHROMA_RGB,
	MDP4_CHROMA_H2V1,
	MDP4_CHROMA_H1V2,
	MDP4_CHROMA_420
};

#define MDP4_BLEND_BG_TRANSP_EN		BIT(9)
#define MDP4_BLEND_FG_TRANSP_EN		BIT(8)
#define MDP4_BLEND_BG_MOD_ALPHA		BIT(7)
#define MDP4_BLEND_BG_INV_ALPHA		BIT(6)
#define MDP4_BLEND_BG_ALPHA_FG_CONST	(0 << 4)
#define MDP4_BLEND_BG_ALPHA_BG_CONST	(1 << 4)
#define MDP4_BLEND_BG_ALPHA_FG_PIXEL	(2 << 4)
#define MDP4_BLEND_BG_ALPHA_BG_PIXEL	(3 << 4)
#define MDP4_BLEND_FG_MOD_ALPHA		BIT(3)
#define MDP4_BLEND_FG_INV_ALPHA		BIT(2)
#define MDP4_BLEND_FG_ALPHA_FG_CONST	(0 << 0)
#define MDP4_BLEND_FG_ALPHA_BG_CONST	(1 << 0)
#define MDP4_BLEND_FG_ALPHA_FG_PIXEL	(2 << 0)
#define MDP4_BLEND_FG_ALPHA_BG_PIXEL	(3 << 0)

#define MDP4_FORMAT_SOLID_FILL		BIT(22)
#define MDP4_FORMAT_UNPACK_ALIGN_MSB	BIT(18)
#define MDP4_FORMAT_UNPACK_TIGHT	BIT(17)
#define MDP4_FORMAT_90_ROTATED		BIT(12)
#define MDP4_FORMAT_ALPHA_ENABLE	BIT(8)

#define MDP4_OP_DEINT_ODD_REF  	BIT(19)
#define MDP4_OP_DEINT_EN	BIT(18)
#define MDP4_OP_IGC_LUT_EN	BIT(16)
#define MDP4_OP_DITHER_EN     	BIT(15)
#define MDP4_OP_FLIP_UD		BIT(14)
#define MDP4_OP_FLIP_LR		BIT(13)
#define MDP4_OP_CSC_EN		BIT(11)
#define MDP4_OP_SRC_DATA_YCBCR	BIT(9)
#define MDP4_OP_SCALEY_FIR 		(0 << 4)
#define MDP4_OP_SCALEY_MN_PHASE 	(1 << 4)
#define MDP4_OP_SCALEY_PIXEL_RPT	(2 << 4)
#define MDP4_OP_SCALEX_FIR 		(0 << 2)
#define MDP4_OP_SCALEX_MN_PHASE 	(1 << 2)
#define MDP4_OP_SCALEX_PIXEL_RPT 	(2 << 2)
#define MDP4_OP_SCALEY_EN	BIT(1)
#define MDP4_OP_SCALEX_EN	BIT(0)

#define MDP4_PIPE_PER_MIXER	2

#define MDP4_MAX_PLANE		4


struct mdp4_overlay_pipe {
	uint32_t pipe_used;
	uint32_t pipe_type;		/* rgb, video/graphic */
	uint32_t pipe_num;
	uint32_t pipe_ndx;
	uint32_t mixer_num;		/* which mixer used */
	uint32_t mixer_stage;		/* which stage of mixer used */
	uint32_t src_format;
	uint32_t src_width;	/* source img width */
	uint32_t src_height;	/* source img height */
	uint32_t src_w;		/* roi */
	uint32_t src_h;		/* roi */
	uint32_t src_x;		/* roi */
	uint32_t src_y;		/* roi */
	uint32_t dst_w;		/* roi */
	uint32_t dst_h;		/* roi */
	uint32_t dst_x;		/* roi */
	uint32_t dst_y;		/* roi */
	uint32_t op_mode;
	uint32_t transp;
	uint32_t blend_op;
	uint32_t phasex_step;
	uint32_t phasey_step;
	uint32_t alpha;
	uint32_t is_fg;		/* control alpha & color key */
	uint32_t srcp0_addr;	/* interleave, luma */
	uint32_t srcp0_ystride;
	uint32_t srcp1_addr;	/* pseudoplanar, chroma plane */
	uint32_t srcp1_ystride;
	uint32_t srcp2_addr;	/* planar color 2*/
	uint32_t srcp2_ystride;
	uint32_t srcp3_addr;	/* alpha/color 3 */
	uint32_t srcp3_ystride;
	uint32_t fetch_plane;
	uint32_t frame_format;		/* video */
	uint32_t chroma_site;		/* video */
	uint32_t chroma_sample;		/* video */
	uint32_t solid_fill;
	uint32_t vc1_reduce;		/* video */
	uint32_t fatch_planes;		/* video */
	uint32_t unpack_align_msb;/* 0 to LSB, 1 to MSB */
	uint32_t unpack_tight;/* 0 for loose, 1 for tight */
	uint32_t unpack_count;/* 0 = 1 component, 1 = 2 component ... */
	uint32_t rotated_90; /* has been rotated 90 degree */
	uint32_t bpp;	/* byte per pixel */
	uint32_t alpha_enable;/*  source has alpha */
	/*
	 * number of bits for source component,
	 * 0 = 1 bit, 1 = 2 bits, 2 = 6 bits, 3 = 8 bits
	 */
	uint32_t a_bit;	/* component 3, alpha */
	uint32_t r_bit;	/* component 2, R_Cr */
	uint32_t b_bit;	/* component 1, B_Cb */
	uint32_t g_bit;	/* component 0, G_lumz */
	/*
	 * unpack pattern
	 * A = C3, R = C2, B = C1, G = C0
	 */
	uint32_t element3; /* 0 = C0, 1 = C1, 2 = C2, 3 = C3 */
	uint32_t element2; /* 0 = C0, 1 = C1, 2 = C2, 3 = C3 */
	uint32_t element1; /* 0 = C0, 1 = C1, 2 = C2, 3 = C3 */
	uint32_t element0; /* 0 = C0, 1 = C1, 2 = C2, 3 = C3 */
	struct completion comp;
#ifdef CONFIG_FB_MSM_WRITE_BACK
	ulong blt_addr; /* blt mode addr */
	uint32_t blt_cnt;
	uint32_t blt_end;
#endif
	struct mdp_overlay req_data;
	struct mdp_info *mdp;
};

struct mdp4_pipe_desc {
        uint32_t ref_cnt;
        struct mdp4_overlay_pipe *player;
};

#ifdef CONFIG_FB_MSM_WRITE_BACK
extern spinlock_t mdp_spin_lock;
extern uint32_t mdp_intr_mask;
#endif



void mdp4_enable_clk_irq(void);
void mdp4_disable_clk_irq(void);
int mdp4_lcdc_on(struct platform_device *pdev);
int mdp4_lcdc_off(struct platform_device *pdev);
void mdp4_intr_clear_set(ulong clear, ulong set);
void mdp4_dma_p_cfg(void);
void mdp4_hw_init(struct mdp_info *mdp);
void mdp4_isr_read(int);
void mdp4_clear_lcdc(struct mdp_info *mdp);
void mdp4_mixer_blend_init(struct mdp_info *mdp, int mixer_num);
void mdp4_vg_qseed_init(struct mdp_info *mdp, int vg_num);
void mdp4_vg_csc_mv_setup(struct mdp_info *mdp, int vp_num);
void mdp4_vg_csc_pre_bv_setup(struct mdp_info *mdp, int vp_num);
void mdp4_vg_csc_post_bv_setup(struct mdp_info *mdp, int vp_num);
void mdp4_vg_csc_pre_lv_setup(struct mdp_info *mdp, int vp_num);
void mdp4_vg_csc_post_lv_setup(struct mdp_info *mdp, int vp_num);
void mdp4_mixer1_csc_mv_setup(struct mdp_info *mdp);
void mdp4_mixer1_csc_pre_bv_setup(struct mdp_info *mdp);
void mdp4_mixer1_csc_post_bv_setup(struct mdp_info *mdp);
void mdp4_mixer1_csc_pre_lv_setup(struct mdp_info *mdp);
void mdp4_mixer1_csc_post_lv_setup(struct mdp_info *mdp);
void mdp4_overlay_format_to_pipe(uint32_t format, struct mdp4_overlay_pipe *pipe);
uint32_t mdp4_overlay_format(struct mdp4_overlay_pipe *pipe);
uint32_t mdp4_overlay_unpack_pattern(struct mdp4_overlay_pipe *pipe);
uint32_t mdp4_overlay_op_mode(struct mdp4_overlay_pipe *pipe);
void mdp4_overlay_rgb_setup(struct mdp4_overlay_pipe *pipe);
void mdp4_overlay_reg_flush(struct mdp4_overlay_pipe *pipe, int all);
void mdp4_mixer_blend_setup(struct mdp4_overlay_pipe *pipe);
struct mdp4_overlay_pipe *mdp4_overlay_stage_pipe(int mixer, int stage);
void mdp4_mixer_stage_up(struct mdp4_overlay_pipe *pipe);
void mdp4_mixer_stage_down(struct mdp4_overlay_pipe *pipe);
int mdp4_mixer_stage_can_run(struct mdp4_overlay_pipe *pipe);
void mdp4_overlayproc_cfg(struct mdp4_overlay_pipe *pipe);
void mdp4_mddi_overlay(void *priv, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y);
int mdp4_overlay_format2type(uint32_t format);
int mdp4_overlay_format2pipe(struct mdp4_overlay_pipe *pipe);
int mdp4_overlay_get(struct mdp_device *mdp_dev, struct fb_info *info, struct mdp_overlay *req);
int mdp4_overlay_set(struct mdp_device *mdp_dev, struct fb_info *info, struct mdp_overlay *req);
int mdp4_overlay_unset(struct mdp_device *mdp_dev, struct fb_info *info, int ndx);
int mdp4_overlay_play(struct mdp_device *mdp_dev, struct fb_info *info, struct msmfb_overlay_data *req,
				struct file **pp_src_file);
int mdp4_overlay_change_z_order_vg_pipes(struct fb_info *info);
struct mdp4_overlay_pipe *mdp4_overlay_pipe_alloc(int ptype, bool usevg);
void mdp4_overlay_pipe_free(struct mdp4_overlay_pipe *pipe);
void mdp4_overlay_dmap_cfg(struct mdp4_overlay_pipe *pipe, int lcdc);
void mdp4_overlay_dmap_xy(struct mdp4_overlay_pipe *pipe);
void mdp4_overlay_dmae_cfg(struct mdp4_overlay_pipe *pipe, int lcdc);
void mdp4_overlay_dmae_xy(struct mdp4_overlay_pipe *pipe);
void mdp4_overlay_dmas_cfg(struct mdp4_overlay_pipe *pipe, int lcdc);
void mdp4_overlay_dmas_xy(struct mdp4_overlay_pipe *pipe);
int mdp4_overlay_active(struct mdp_info *mdp, int mixer);
void mdp4_overlay0_done_lcdc(void);
void mdp4_overlay0_done_mddi(void);
void mdp4_dma_s_done_mddi(void);
void mdp4_mddi_overlay_restore(void);
void mdp4_mddi_dma_s_kickoff(struct mdp_info *mdp, struct mdp4_overlay_pipe *pipe);
void mdp4_mddi_overlay_kickoff(struct mdp_info *mdp,
				struct mdp4_overlay_pipe *pipe);
void mdp4_rgb_igc_lut_setup(struct mdp_info *mdp, int num);
void mdp4_vg_igc_lut_setup(struct mdp_info *mdp, int num);
void mdp4_mixer_gc_lut_setup(struct mdp_info *mdp, int mixer_num);
void mdp4_fetch_cfg(struct mdp_info *mdp, uint32_t clk, uint32_t pclk);
uint32_t mdp4_rgb_igc_lut_cvt(uint32_t ndx);
void mdp_pipe_kickoff(struct mdp_info *mdp, uint32_t term);

#ifdef CONFIG_FB_MSM_WRITE_BACK
void mdp4_dma_p_done_mddi(void);
int mdp4_overlay_blt(struct mdp_device *mdp_dev,struct fb_info *info, struct msmfb_overlay_blt *req,
		struct file **pp_src_file);
void mdp4_mddi_overlay_blt(ulong addr);
void mdp4_lcdc_overlay_blt(ulong addr);
#endif

#endif /* MDP_H */
