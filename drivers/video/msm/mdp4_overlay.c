/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/file.h>
#include <linux/android_pmem.h>
#include <linux/major.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <mach/debug_display.h>

#include "mdp_hw.h"
#include "mdp4.h"

#define OVERLAY_UPDATE_SCREEN		6	/*Refer to user_data field in struct mdp_overlay*/
#define OVERLAY_UPDATE_SCREEN_EN	1
#define OVERLAY_UPDATE_SCREEN_DIS	0

#ifdef CONFIG_PANEL_SELF_REFRESH
extern struct panel_icm_info *panel_icm;
extern wait_queue_head_t panel_update_wait_queue;
#endif
extern int get_fb_phys_info(unsigned long *start, unsigned long *len, int fb_num);

static int z_order_change = 0;

struct mdp4_overlay_ctrl {
	struct mdp4_pipe_desc ov_pipe[OVERLAY_PIPE_MAX];/* 4 */
	struct mdp4_overlay_pipe plist[MDP4_MAX_PIPE];	/* 4 + 2 */
	struct mdp4_overlay_pipe *stage[MDP4_MAX_MIXER][MDP4_MAX_STAGE];
} mdp4_overlay_db = {
	.plist = {
		{
			.pipe_type = OVERLAY_TYPE_RGB,
			.pipe_num = OVERLAY_PIPE_RGB1,
			.pipe_ndx = 1,
		},
		{
			.pipe_type = OVERLAY_TYPE_RGB,
			.pipe_num = OVERLAY_PIPE_RGB2,
			.pipe_ndx = 2,
		},
		{
			.pipe_type = OVERLAY_TYPE_RGB, /* shared */
			.pipe_num = OVERLAY_PIPE_VG1,
			.pipe_ndx = 3,
			.pipe_used = 1,	/* mark used to NOT shared */
		},
		{
			.pipe_type = OVERLAY_TYPE_RGB, /* shared */
			.pipe_num = OVERLAY_PIPE_VG2,
			.pipe_ndx = 4,
		},
		{
			.pipe_type = OVERLAY_TYPE_VIDEO, /* shared */
			.pipe_num = OVERLAY_PIPE_VG1,
			.pipe_ndx = 5,
		},
		{
			.pipe_type = OVERLAY_TYPE_VIDEO, /* shared */
			.pipe_num = OVERLAY_PIPE_VG2,
			.pipe_ndx = 6,
		}
	}
};

static struct mdp4_overlay_ctrl *ctrl = &mdp4_overlay_db;

void mdp4_overlay_dmae_cfg(struct mdp4_overlay_pipe *pipe, int atv)
{
	uint32_t dmae_cfg_reg;

	if (atv)
		dmae_cfg_reg = DMA_DEFLKR_EN;
	else
		dmae_cfg_reg = 0;

	if (pipe->mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB565) {
		dmae_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	} else if (pipe->mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB666) {
		dmae_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	} else {
		dmae_cfg_reg |= DMA_DSTC0G_8BITS |	/* 888 24BPP */
		    DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
	}

	dmae_cfg_reg |= DMA_PACK_PATTERN_RGB;

	/* dma2 config register */
	mdp_writel(pipe->mdp, dmae_cfg_reg, 0xb0000);

	if (atv) {
		mdp_writel(pipe->mdp, 0xeb0010, 0xb0070);
		mdp_writel(pipe->mdp, 0xf00010, 0xb0074);
		mdp_writel(pipe->mdp, 0xf00010, 0xb0078);
		mdp_writel(pipe->mdp, 0x80, 0xb3000);
		mdp_writel(pipe->mdp, 0x1800040, 0xb3010);
		mdp_writel(pipe->mdp, 0x1000080, 0xb3014);
		mdp_writel(pipe->mdp, 0x67686970, 0xb4004);
	} else {
		mdp_writel(pipe->mdp, 0xff0000, 0xb0070);
		mdp_writel(pipe->mdp, 0xff0000, 0xb0074);
		mdp_writel(pipe->mdp, 0xff0000, 0xb0078);
	}
}

void mdp4_overlay_dmae_xy(struct mdp4_overlay_pipe *pipe)
{
	/* dma_e source */
	mdp_writel(pipe->mdp, (pipe->src_height << 16 | pipe->src_width)
			, 0xb0004);
	mdp_writel(pipe->mdp, pipe->srcp0_addr, 0xb0008);
	mdp_writel(pipe->mdp, pipe->srcp0_ystride, 0xb000c);

	/* dma_e dest */
	mdp_writel(pipe->mdp, (pipe->dst_y << 16 | pipe->dst_x), 0xb0010);
}

void mdp4_overlay_parameters_check(struct mdp4_overlay_pipe *pipe)
{
	uint32_t dst_neww = 0, dst_newh = 0;

	if(pipe == NULL || pipe->mdp == NULL)
		return;

	if(pipe->dst_x < 0 || pipe->dst_x > pipe->mdp->mdp_dev.width) {
		PR_DISP_ERR("%s(%d) Incorrect parameter dstx=%d", __func__, __LINE__, pipe->dst_x);
		pipe->dst_x = 0;
	}

	if(pipe->dst_y < 0 || pipe->dst_y > pipe->mdp->mdp_dev.height) {
		PR_DISP_ERR("%s(%d) Incorrect parameter dsty=%d", __func__, __LINE__, pipe->dst_y);
		pipe->dst_y = 0;
	}

	if(pipe->dst_w < 0 || pipe->dst_w > pipe->mdp->mdp_dev.width) {
		PR_DISP_ERR("%s(%d) Incorrect parameter dstw=%d", __func__, __LINE__, pipe->dst_w);
		pipe->dst_w = pipe->mdp->mdp_dev.width;
	}

	if(pipe->dst_y < 0 || pipe->dst_y > pipe->mdp->mdp_dev.height) {
		PR_DISP_ERR("%s(%d) Incorrect parameter dsty=%d", __func__, __LINE__, pipe->dst_y);
		pipe->dst_y = pipe->mdp->mdp_dev.height;
	}

	if(pipe->dst_w + pipe->dst_x > pipe->mdp->mdp_dev.width) {
		dst_neww = pipe->mdp->mdp_dev.width - pipe->dst_x;
		PR_DISP_ERR("%s(%d) Incorrect parameter dstx=%d dstw=%d found, change new dstw=%d", __func__, __LINE__,
			pipe->dst_x, pipe->dst_w, dst_neww);
		pipe->dst_w = dst_neww;
	}

	if(pipe->dst_h + pipe->dst_y > pipe->mdp->mdp_dev.height) {
		dst_newh = pipe->mdp->mdp_dev.height - pipe->dst_y;
		PR_DISP_ERR("%s(%d) Incorrect parameter dsty=%d dsth=%d found, change new dsth=%d", __func__, __LINE__,
			pipe->dst_y, pipe->dst_h, dst_newh);
		pipe->dst_h = dst_newh;
	}
}


void mdp4_overlay_dmap_cfg(struct mdp4_overlay_pipe *pipe, int lcdc)
{
	uint32_t	dmap_cfg_reg = 0;

	dmap_cfg_reg |= DMA_DITHER_EN;


	if (pipe->mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB565) {
		dmap_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	} else if (pipe->mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB666) {
		dmap_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	} else {
		dmap_cfg_reg |= DMA_DSTC0G_8BITS |	/* 888 16BPP */
		    DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
	}

	if (lcdc) {
		if (pipe->mdp->mdp_dev.overrides & MSM_MDP_DMA_PACK_ALIGN_LSB)
			dmap_cfg_reg |= DMA_PACK_ALIGN_LSB;
		else
			dmap_cfg_reg |= DMA_PACK_ALIGN_MSB;
	}

	/* dmap config register */
	mdp_writel(pipe->mdp, dmap_cfg_reg, 0x90000);

}

void mdp4_overlay_dmap_xy(struct mdp4_overlay_pipe *pipe)
{

	/* dma_p source */
	mdp_writel(pipe->mdp, (pipe->src_height << 16 | pipe->src_width), 0x90004);
#ifdef CONFIG_FB_MSM_WRITE_BACK
       if (pipe->blt_addr) {
               /* overlay ouput is RGB565 */
		uint32_t off;
               off = 0;
               if (pipe->blt_cnt & 0x01)
                      off = pipe->src_height * pipe->src_width * 2;
               mdp_writel(pipe->mdp, pipe->blt_addr+off , 0x90008);
              /* RGB888, output of overlay blending */
               mdp_writel(pipe->mdp, pipe->src_width * 2, 0x9000c);
       } else {
               mdp_writel(pipe->mdp, pipe->srcp0_addr, 0x90008);
               mdp_writel(pipe->mdp, pipe->srcp0_ystride, 0x9000c);
       }
#else
               mdp_writel(pipe->mdp, pipe->srcp0_addr, 0x90008);
               mdp_writel(pipe->mdp, pipe->srcp0_ystride, 0x9000c);
#endif

	/* dma_p dest */
	mdp_writel(pipe->mdp, (pipe->dst_y << 16 | pipe->dst_x), 0x90010);
}

void mdp4_overlay_dmas_cfg(struct mdp4_overlay_pipe *pipe, int lcdc)
{
	uint32_t	dmas_cfg_reg = 0;

	dmas_cfg_reg |= DMA_DITHER_EN;
	if(pipe->src_format == MDP_RGBA_8888) {
		dmas_cfg_reg |= DMA_PACK_PATTERN_BGR;
		dmas_cfg_reg |= (1 << 26);
	} else {
		dmas_cfg_reg |= DMA_PACK_PATTERN_RGB;
		dmas_cfg_reg |= DMA_IBUF_FORMAT_RGB565;
	}

	dmas_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
	    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;

	if (lcdc)
		dmas_cfg_reg |= DMA_PACK_ALIGN_MSB;

	/* dmas config register */
	mdp_writel(pipe->mdp, dmas_cfg_reg, 0xA0000);

}

void mdp4_overlay_dmas_xy(struct mdp4_overlay_pipe *pipe)
{

	/* dmas source */
	mdp_writel(pipe->mdp, (pipe->dst_h << 16 | pipe->dst_w), 0xA0004);
	mdp_writel(pipe->mdp, pipe->srcp0_addr, 0xA0008);
	mdp_writel(pipe->mdp, pipe->srcp0_ystride, 0xA000c);

	/* dmas dest */
	mdp_writel(pipe->mdp, (pipe->dst_y << 16 | pipe->dst_x), 0xA0010);
}

#define MDP4_VG_PHASE_STEP_DEFAULT	0x20000000
#define MDP4_VG_PHASE_STEP_SHIFT	29

static int mdp4_leading_0(uint32_t num)
{
	uint32_t bit = 0x80000000;
	int i;

	for (i = 0; i < 32; i++) {
		if (bit & num)
			return i;
		bit >>= 1;
	}

	return i;
}

static uint32_t mdp4_scale_phase_step(int f_num, uint32_t src, uint32_t dst)
{
	uint32_t val;
	int	n;

	n = mdp4_leading_0(src);
	if (n > f_num)
		n = f_num;
	val = src << n;	/* maximum to reduce lose of resolution */
	val /= dst;
	if (n < f_num) {
		n = f_num - n;
		val <<= n;
	}

	return val;
}

static void mdp4_scale_setup(struct mdp4_overlay_pipe *pipe)
{

	pipe->phasex_step = MDP4_VG_PHASE_STEP_DEFAULT;
	pipe->phasey_step = MDP4_VG_PHASE_STEP_DEFAULT;

	if (pipe->dst_h && pipe->src_h != pipe->dst_h) {
		if (pipe->dst_h >= pipe->src_h * 8)	/* too much */
			return;
		pipe->op_mode |= MDP4_OP_SCALEY_EN;

		if (pipe->pipe_num >= OVERLAY_PIPE_VG1) {
			if (pipe->dst_h <= (pipe->src_h / 4))
				pipe->op_mode |= MDP4_OP_SCALEY_MN_PHASE;
			else
				pipe->op_mode |= MDP4_OP_SCALEY_FIR;
		}

		pipe->phasey_step = mdp4_scale_phase_step(29,
					pipe->src_h, pipe->dst_h);
	}

	if (pipe->dst_w && pipe->src_w != pipe->dst_w) {
		if (pipe->dst_w >= pipe->src_w * 8)	/* too much */
			return;
		pipe->op_mode |= MDP4_OP_SCALEX_EN;

		if (pipe->pipe_num >= OVERLAY_PIPE_VG1) {
			if (pipe->dst_w <= (pipe->src_w / 4))
				pipe->op_mode |= MDP4_OP_SCALEX_MN_PHASE;
			else
				pipe->op_mode |= MDP4_OP_SCALEX_FIR;
		}

		pipe->phasex_step = mdp4_scale_phase_step(29,
					pipe->src_w, pipe->dst_w);
	}
}

void mdp4_overlay_rgb_setup(struct mdp4_overlay_pipe *pipe)
{
	uint32_t rgb_base;
	uint32_t src_size, src_xy, dst_size, dst_xy;
	uint32_t format, pattern;
	uint32_t dst_newx = 0, dst_newy = 0;

	rgb_base = MDP4_RGB_BASE;
	rgb_base += (MDP4_RGB_OFF * pipe->pipe_num);

	mdp4_overlay_parameters_check(pipe);

	src_size = ((pipe->src_h << 16) | pipe->src_w);
	src_xy = ((pipe->src_y << 16) | pipe->src_x);
	dst_size = ((pipe->dst_h << 16) | pipe->dst_w);

	format = mdp4_overlay_format(pipe);
	pattern = mdp4_overlay_unpack_pattern(pipe);

#ifdef MDP4_IGC_LUT_ENABLE
	pipe->op_mode = MDP4_OP_IGC_LUT_EN;
#else
	pipe->op_mode = 0;
#endif

        /* workaroud for the panel with wrong direction */
        if (pipe->mdp->mdp_dev.overrides & MSM_MDP_PANEL_FLIP_UD) {
		pipe->op_mode ^= MDP4_OP_FLIP_UD;
		dst_newy = pipe->mdp->mdp_dev.height - pipe->dst_y - pipe->dst_h;
		dst_xy = dst_newy << 16;
	}
	else
		dst_xy = pipe->dst_y << 16;

        if (pipe->mdp->mdp_dev.overrides & MSM_MDP_PANEL_FLIP_LR) {
		pipe->op_mode ^= MDP4_OP_FLIP_LR;
		dst_newx = pipe->mdp->mdp_dev.width - pipe->dst_x - pipe->dst_w;
		dst_xy |= dst_newx;
	}
	else
		dst_xy |= pipe->dst_x;

	mdp4_scale_setup(pipe);

	mdp_writel(pipe->mdp, src_size, rgb_base + 0x0000);	/* MDP_RGB_SRC_SIZE */
	mdp_writel(pipe->mdp, src_xy, rgb_base + 0x0004);	/* MDP_RGB_SRC_XY */
	mdp_writel(pipe->mdp, src_xy, rgb_base + 0x0008);	/* MDP_RGB_DST_SIZE */
	mdp_writel(pipe->mdp, dst_xy, rgb_base + 0x000c);	/* MDP_RGB_DST_XY */

	mdp_writel(pipe->mdp, pipe->srcp0_addr, rgb_base + 0x0010);
	mdp_writel(pipe->mdp, pipe->srcp0_ystride, rgb_base + 0x0040);

	mdp_writel(pipe->mdp, format, rgb_base + 0x0050);/* MDP_RGB_SRC_FORMAT */
	mdp_writel(pipe->mdp, pattern, rgb_base + 0x0054);/* MDP_RGB_SRC_UNPACK_PATTERN */
	mdp_writel(pipe->mdp, pipe->op_mode, rgb_base + 0x0058);/* MDP_RGB_OP_MODE */
	mdp_writel(pipe->mdp, pipe->phasex_step, rgb_base + 0x005c);
	mdp_writel(pipe->mdp, pipe->phasey_step, rgb_base + 0x0060);
}

void mdp4_overlay_vg_setup(struct mdp4_overlay_pipe *pipe)
{
	uint32_t vg_base;
	uint32_t frame_size, src_size, src_xy, dst_size, dst_xy;
	uint32_t format, pattern;
	uint32_t dst_newx = 0, dst_newy = 0;
	int pnum;

	pnum = pipe->pipe_num - OVERLAY_PIPE_VG1; /* start from 0 */
	vg_base = MDP4_VIDEO_BASE;
	vg_base += (MDP4_VIDEO_OFF * pnum);

	mdp4_overlay_parameters_check(pipe);

	frame_size = ((pipe->src_height << 16) | pipe->src_width);
	src_size = ((pipe->src_h << 16) | pipe->src_w);
	src_xy = ((pipe->src_y << 16) | pipe->src_x);
	dst_size = ((pipe->dst_h << 16) | pipe->dst_w);

	format = mdp4_overlay_format(pipe);
	pattern = mdp4_overlay_unpack_pattern(pipe);

#ifdef MDP4_IGC_LUT_ENABLE
	pipe->op_mode = MDP4_OP_IGC_LUT_EN;
#else
	pipe->op_mode = 0;
#endif

	/* not RGB use VG pipe */
	if (pipe->pipe_type != OVERLAY_TYPE_RGB)
		pipe->op_mode |= (MDP4_OP_CSC_EN | MDP4_OP_SRC_DATA_YCBCR);


	mdp4_scale_setup(pipe);

        /* workaroud for the panel with wrong direction */
        if (pipe->mdp->mdp_dev.overrides & MSM_MDP_PANEL_FLIP_UD) {
		pipe->op_mode ^= MDP4_OP_FLIP_UD;
		dst_newy = pipe->mdp->mdp_dev.height - pipe->dst_y - pipe->dst_h;
		dst_xy = dst_newy << 16;
	}
	else
		dst_xy = pipe->dst_y << 16;

        if (pipe->mdp->mdp_dev.overrides & MSM_MDP_PANEL_FLIP_LR) {
		pipe->op_mode ^= MDP4_OP_FLIP_LR;
		dst_newx = pipe->mdp->mdp_dev.width - pipe->dst_x - pipe->dst_w;
		dst_xy |= dst_newx;
	}
	else
		dst_xy |= pipe->dst_x;

	if(pipe->dst_x < 0 || pipe->dst_y < 0)
		PR_DISP_ERR("%s(%d) Incorrect overlay input pipex=%d pipey=%d newx=%d newy=%d width=%d height=%d\n", __func__, __LINE__, pipe->dst_x, pipe->dst_y, dst_newx, dst_newy, pipe->mdp->mdp_dev.width, pipe->mdp->mdp_dev.height);

	mdp_writel(pipe->mdp, src_size, vg_base + 0x0000);	/* MDP_RGB_SRC_SIZE */
	mdp_writel(pipe->mdp, src_xy, vg_base + 0x0004);	/* MDP_RGB_SRC_XY */
	mdp_writel(pipe->mdp, dst_size, vg_base + 0x0008);	/* MDP_RGB_DST_SIZE */
	mdp_writel(pipe->mdp, dst_xy, vg_base + 0x000c);	/* MDP_RGB_DST_XY */
	mdp_writel(pipe->mdp, frame_size, vg_base + 0x0048);	/* TILE frame size */

	/* luma component plane */
	mdp_writel(pipe->mdp, pipe->srcp0_addr, vg_base + 0x0010);

	/* chroma component plane */
	mdp_writel(pipe->mdp, pipe->srcp1_addr, vg_base + 0x0014);


	mdp_writel(pipe->mdp, pipe->srcp1_ystride << 16 | pipe->srcp0_ystride, vg_base + 0x0040);

	mdp_writel(pipe->mdp, format, vg_base + 0x0050);	/* MDP_RGB_SRC_FORMAT */
	mdp_writel(pipe->mdp, pattern, vg_base + 0x0054);	/* MDP_RGB_SRC_UNPACK_PATTERN */
	mdp_writel(pipe->mdp, pipe->op_mode, vg_base + 0x0058); /* MDP_RGB_OP_MODE */
	mdp_writel(pipe->mdp, pipe->phasex_step, vg_base + 0x005c);
	mdp_writel(pipe->mdp, pipe->phasey_step, vg_base + 0x0060);

	if (pipe->op_mode & MDP4_OP_DITHER_EN) {
		mdp_writel(pipe->mdp, pipe->r_bit << 4 | pipe->b_bit << 2 | pipe->g_bit, vg_base + 0x0068);
	}
}

int mdp4_overlay_format2type(uint32_t format)
{
	switch (format) {
	case MDP_RGB_565:
	case MDP_RGB_888:
	case MDP_BGR_565:
	case MDP_XRGB_8888:
	case MDP_ARGB_8888:
	case MDP_RGBA_8888:
	case MDP_BGRA_8888:
	case MDP_RGBX_8888:
		return OVERLAY_TYPE_RGB;
	case MDP_YCRYCB_H2V1:
	case MDP_Y_CRCB_H2V1:
	case MDP_Y_CBCR_H2V1:
	case MDP_Y_CRCB_H2V2:
	case MDP_Y_CBCR_H2V2:
	case MDP_Y_CBCR_H2V2_TILE:
	case MDP_Y_CRCB_H2V2_TILE:
		return OVERLAY_TYPE_VIDEO;
	default:
		return -ERANGE;
	}

}

#define C3_ALPHA	3	/* alpha */
#define C2_R_Cr		2	/* R/Cr */
#define C1_B_Cb		1	/* B/Cb */
#define C0_G_Y		0	/* G/luma */

int mdp4_overlay_format2pipe(struct mdp4_overlay_pipe *pipe)
{
	switch (pipe->src_format) {
	case MDP_RGB_565:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 0;
		pipe->r_bit = 1;	/* R, 5 bits */
		pipe->b_bit = 1;	/* B, 5 bits */
		pipe->g_bit = 2;	/* G, 6 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 2;
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 2;	/* 2 bpp */
		break;
	case MDP_RGB_888:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 0;
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 2;
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 3;	/* 3 bpp */
		break;
	case MDP_BGR_565:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 0;
		pipe->r_bit = 1;	/* R, 5 bits */
		pipe->b_bit = 1;	/* B, 5 bits */
		pipe->g_bit = 2;	/* G, 6 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 2;
		pipe->element2 = C1_B_Cb;	/* B */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C2_R_Cr;	/* R */
		pipe->bpp = 2;	/* 2 bpp */
		break;
	case MDP_XRGB_8888:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C3_ALPHA;	/* alpha */
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 4;		/* 4 bpp */
		break;
	case MDP_ARGB_8888:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 1;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C3_ALPHA;	/* alpha */
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 4;		/* 4 bpp */
		break;
	case MDP_RGBA_8888:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 1;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C3_ALPHA;	/* alpha */
		pipe->element2 = C1_B_Cb;	/* B */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C2_R_Cr;	/* R */
		pipe->bpp = 4;		/* 4 bpp */
		break;
	case MDP_RGBX_8888:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C3_ALPHA;	/* alpha */
		pipe->element2 = C1_B_Cb;	/* B */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C2_R_Cr;	/* R */
		pipe->bpp = 4;		/* 4 bpp */
		break;
	case MDP_BGRA_8888:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 3;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 1;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C3_ALPHA;	/* alpha */
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 4;		/* 4 bpp */
		break;
	case MDP_YCRYCB_H2V1:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_INTERLEAVED;
		pipe->a_bit = 0;	/* alpha, 4 bits */
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 3;
		pipe->element3 = C0_G_Y;	/* G */
		pipe->element2 = C2_R_Cr;	/* R */
		pipe->element1 = C0_G_Y;	/* G */
		pipe->element0 = C1_B_Cb;	/* B */
		pipe->bpp = 2;		/* 2 bpp */
		pipe->chroma_sample = MDP4_CHROMA_H2V1;
		break;
	case MDP_Y_CRCB_H2V1:
	case MDP_Y_CBCR_H2V1:
	case MDP_Y_CRCB_H2V2:
	case MDP_Y_CBCR_H2V2:
		pipe->frame_format = MDP4_FRAME_FORMAT_LINEAR;
		pipe->fetch_plane = OVERLAY_PLANE_PSEUDO_PLANAR;
		pipe->a_bit = 0;
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 1;		/* 2 */
		pipe->element3 = C0_G_Y;	/* not used */
		pipe->element2 = C0_G_Y;	/* not used */
		if (pipe->src_format == MDP_Y_CRCB_H2V1) {
			pipe->element1 = C2_R_Cr;	/* R */
			pipe->element0 = C1_B_Cb;	/* B */
			pipe->chroma_sample = MDP4_CHROMA_H2V1;
		} else if (pipe->src_format == MDP_Y_CBCR_H2V1) {
			pipe->element1 = C1_B_Cb;	/* B */
			pipe->element0 = C2_R_Cr;	/* R */
			pipe->chroma_sample = MDP4_CHROMA_H2V1;
		} else if (pipe->src_format == MDP_Y_CRCB_H2V2) {
			pipe->element1 = C2_R_Cr;	/* R */
			pipe->element0 = C1_B_Cb;	/* B */
			pipe->chroma_sample = MDP4_CHROMA_420;
		} else if (pipe->src_format == MDP_Y_CBCR_H2V2) {
			pipe->element1 = C1_B_Cb;	/* B */
			pipe->element0 = C2_R_Cr;	/* R */
			pipe->chroma_sample = MDP4_CHROMA_420;
		}
		pipe->bpp = 2;	/* 2 bpp */
		break;
	case MDP_Y_CBCR_H2V2_TILE:
	case MDP_Y_CRCB_H2V2_TILE:
		pipe->frame_format = MDP4_FRAME_FORMAT_VIDEO_SUPERTILE;
		pipe->fetch_plane = OVERLAY_PLANE_PSEUDO_PLANAR;
		pipe->a_bit = 0;
		pipe->r_bit = 3;	/* R, 8 bits */
		pipe->b_bit = 3;	/* B, 8 bits */
		pipe->g_bit = 3;	/* G, 8 bits */
		pipe->alpha_enable = 0;
		pipe->unpack_tight = 1;
		pipe->unpack_align_msb = 0;
		pipe->unpack_count = 1;		/* 2 */
		pipe->element3 = C0_G_Y;	/* not used */
		pipe->element2 = C0_G_Y;	/* not used */
		if (pipe->src_format == MDP_Y_CRCB_H2V2_TILE) {
			pipe->element1 = C2_R_Cr;	/* R */
			pipe->element0 = C1_B_Cb;	/* B */
			pipe->chroma_sample = MDP4_CHROMA_420;
		} else if (pipe->src_format == MDP_Y_CBCR_H2V2_TILE) {
			pipe->element1 = C1_B_Cb;	/* B */
			pipe->element0 = C2_R_Cr;	/* R */
			pipe->chroma_sample = MDP4_CHROMA_420;
		}
		pipe->bpp = 2;	/* 2 bpp */
		break;
	default:
		/* not likely */
		return -ERANGE;
	}

	return 0;
}

/*
 * color_key_convert: output with 12 bits color key
 */
static uint32_t color_key_convert(int start, int num, uint32_t color)
{
	uint32_t data;

	data = (color >> start) & ((1 << num) - 1);

	/* convert to 8 bits */
	if (num == 5)
		data = ((data << 3) | (data >> 2));
	else if (num == 6)
		data = ((data << 2) | (data >> 4));

	/* convert 8 bits to 12 bits */
	data = (data << 4) | (data >> 4);

	return data;
}

void transp_color_key(int format, uint32_t transp,
			uint32_t *c0, uint32_t *c1, uint32_t *c2)
{
	int b_start, g_start, r_start;
	int b_num, g_num, r_num;

	switch (format) {
	case MDP_RGB_565:
		b_start = 0;
		g_start = 5;
		r_start = 11;
		r_num = 5;
		g_num = 6;
		b_num = 5;
		break;
	case MDP_RGB_888:
	case MDP_XRGB_8888:
	case MDP_ARGB_8888:
	case MDP_BGRA_8888:
		b_start = 0;
		g_start = 8;
		r_start = 16;
		r_num = 8;
		g_num = 8;
		b_num = 8;
		break;
	case MDP_RGBA_8888:
	case MDP_RGBX_8888:
		b_start = 16;
		g_start = 8;
		r_start = 0;
		r_num = 8;
		g_num = 8;
		b_num = 8;
		break;
	case MDP_BGR_565:
		b_start = 11;
		g_start = 5;
		r_start = 0;
		r_num = 5;
		g_num = 6;
		b_num = 5;
		break;
	case MDP_Y_CBCR_H2V2:
	case MDP_Y_CBCR_H2V1:
		b_start = 8;
		g_start = 16;
		r_start = 0;
		r_num = 8;
		g_num = 8;
		b_num = 8;
		break;
	case MDP_Y_CRCB_H2V2:
	case MDP_Y_CRCB_H2V1:
		b_start = 0;
		g_start = 16;
		r_start = 8;
		r_num = 8;
		g_num = 8;
		b_num = 8;
		break;
	default:
		b_start = 0;
		g_start = 8;
		r_start = 16;
		r_num = 8;
		g_num = 8;
		b_num = 8;
		break;
	}

	*c0 = color_key_convert(g_start, g_num, transp);
	*c1 = color_key_convert(b_start, b_num, transp);
	*c2 = color_key_convert(r_start, r_num, transp);
}

uint32_t mdp4_overlay_format(struct mdp4_overlay_pipe *pipe)
{
	uint32_t	format;

	format = 0;

	if (pipe->solid_fill)
		format |= MDP4_FORMAT_SOLID_FILL;

	if (pipe->unpack_align_msb)
		format |= MDP4_FORMAT_UNPACK_ALIGN_MSB;

	if (pipe->unpack_tight)
		format |= MDP4_FORMAT_UNPACK_TIGHT;

	if (pipe->alpha_enable)
		format |= MDP4_FORMAT_ALPHA_ENABLE;

	format |= (pipe->unpack_count << 13);
	format |= ((pipe->bpp - 1) << 9);
	format |= (pipe->a_bit << 6);
	format |= (pipe->r_bit << 4);
	format |= (pipe->b_bit << 2);
	format |= pipe->g_bit;

	format |= (pipe->frame_format << 29);

	if (pipe->fetch_plane == OVERLAY_PLANE_PSEUDO_PLANAR) {
		/* video/graphic */
		format |= (pipe->fetch_plane << 19);
		format |= (pipe->chroma_site << 28);
		format |= (pipe->chroma_sample << 26);
	}

	return format;
}

uint32_t mdp4_overlay_unpack_pattern(struct mdp4_overlay_pipe *pipe)
{
	return (pipe->element3 << 24) | (pipe->element2 << 16) |
			(pipe->element1 << 8) | pipe->element0;
}
/*
 * mdp4_overlayproc_cfg: only be called from base layer
 */
void mdp4_overlayproc_cfg(struct mdp4_overlay_pipe *pipe)
{
	uint32_t data;
	uint32_t overlay_base;

	if (pipe->mixer_num == MDP4_MIXER1)
		overlay_base = MDP4_OVERLAYPROC1_BASE;/* 0x18000 */
	else
		overlay_base = MDP4_OVERLAYPROC0_BASE;/* 0x10000 */

	/* MDP_OVERLAYPROC_CFG */
	mdp_writel(pipe->mdp, 0x01, overlay_base + 0x0004); /* directout */
	data = pipe->src_height;
	data <<= 16;
	data |= pipe->src_width;
	mdp_writel(pipe->mdp, data, overlay_base + 0x0008); /* ROI, height + width */
       /*
        * BLT only siupport at primary display
        */
#ifdef CONFIG_FB_MSM_WRITE_BACK
       if (pipe->mixer_num == MDP4_MIXER0 && pipe->blt_addr) {
		int off;
		int mddi = 0;
		mddi = mdp_readl(pipe->mdp, 0x0038);
		mddi &= 0x02;

		if (!mddi){
                       mdp_writel(pipe->mdp, pipe->blt_addr, overlay_base + 0x000c);
                       mdp_writel(pipe->mdp, pipe->src_width * 2, overlay_base + 0x0010);
                       /* overlay ouput is RGB565 */
                       off = pipe->src_height * pipe->src_width * 2;
                       mdp_writel(pipe->mdp, pipe->blt_addr + off, overlay_base + 0x001c);
                       /* LCDC - FRAME BUFFER + vsync rate */
                       mdp_writel(pipe->mdp, 0x02, overlay_base + 0x0004);
		} else {        /* MDDI */
                       off = 0;
                       if (pipe->blt_cnt & 0x01)
                               off = pipe->src_height * pipe->src_width * 2;
                       mdp_writel(pipe->mdp, pipe->blt_addr+off, overlay_base + 0x000c);
                       /* overlay ouput is RGB565 */
                       mdp_writel(pipe->mdp, pipe->src_width * 2, overlay_base + 0x0010);
                       mdp_writel(pipe->mdp, pipe->blt_addr + off, overlay_base + 0x001c);
                       /* MDDI - BLT + on demand */
			mdp_writel(pipe->mdp, 0x02, overlay_base + 0x0004);
               }
       } else {
               mdp_writel(pipe->mdp, data, overlay_base + 0x0008);
               mdp_writel(pipe->mdp, pipe->srcp0_addr, overlay_base + 0x000c);
               mdp_writel(pipe->mdp, pipe->srcp0_ystride, overlay_base + 0x0010);
               mdp_writel(pipe->mdp, 0x01, overlay_base + 0x0004);
       }
#else
	mdp_writel(pipe->mdp, data, overlay_base + 0x0008);/* ROI, height + width */
	mdp_writel(pipe->mdp, pipe->srcp0_addr, overlay_base + 0x000c);
	mdp_writel(pipe->mdp, pipe->srcp0_ystride, overlay_base + 0x0010);
#endif


#ifdef MDP4_IGC_LUT_ENABLE
	mdp_writel(pipe->mdp, 0x4, overlay_base + 0x0014); /* GC_LUT_EN, 888 */
#endif
#ifdef CONFIG_FB_MSM_WRITE_BACK
	mdp_writel(pipe->mdp, 0x1, overlay_base + 0x0014);
#endif
}

int mdp4_overlay_active(struct mdp_info* mdp, int mixer)
{
	uint32_t data, mask, i;
	int p1, p2;

	data = mdp_readl(mdp, 0x10100);
	p1 = 0;
	p2 = 0;
	for (i = 0; i < 8; i++) {
		mask = data & 0x0f;
		if (mask) {
			if (mask <= 4)
				p1++;
			else
				p2++;
		}
		data >>= 4;
	}

	if (mixer)
		return p2;
	else
		return p1;
}

void mdp4_mixer_stage_up(struct mdp4_overlay_pipe *pipe)
{
	uint32_t data, mask, snum, stage, mixer, pnum;

	stage = pipe->mixer_stage;
	mixer = pipe->mixer_num;
	pnum = pipe->pipe_num;

	/* MDP_LAYERMIXER_IN_CFG, shard by both mixer 0 and 1  */
	data = mdp_readl(pipe->mdp, 0x10100);

	if (mixer == MDP4_MIXER1)
		stage += 8;

	if (pipe->pipe_num >= OVERLAY_PIPE_VG1) {/* VG1 and VG2 */
		pnum -= OVERLAY_PIPE_VG1; /* start from 0 */
		snum = 0;
		snum += (4 * pnum);
	} else {
		snum = 8;
		snum += (4 * pnum);	/* RGB1 and RGB2 */
	}

	mask = 0x0f;
	mask <<= snum;
	stage <<= snum;
	data &= ~mask;	/* clear old bits */

	data |= stage;

	mdp_writel(pipe->mdp, data, 0x10100); /* MDP_LAYERMIXER_IN_CFG */

	data = mdp_readl(pipe->mdp, 0x10100);

	ctrl->stage[pipe->mixer_num][pipe->mixer_stage] = pipe;	/* keep it */
}

void mdp4_mixer_stage_down(struct mdp4_overlay_pipe *pipe)
{
	uint32_t data, mask, snum, stage, mixer, pnum;

	stage = pipe->mixer_stage;
	mixer = pipe->mixer_num;
	pnum = pipe->pipe_num;

	if (pipe != ctrl->stage[mixer][stage])	/* not runing */
		return;

	/* MDP_LAYERMIXER_IN_CFG, shard by both mixer 0 and 1  */
	data = mdp_readl(pipe->mdp, 0x10100);

	if (mixer == MDP4_MIXER1)
		stage += 8;

	if (pipe->pipe_num >= OVERLAY_PIPE_VG1) {/* VG1 and VG2 */
		pnum -= OVERLAY_PIPE_VG1; /* start from 0 */
		snum = 0;
		snum += (4 * pnum);
	} else {
		snum = 8;
		snum += (4 * pnum);	/* RGB1 and RGB2 */
	}

	mask = 0x0f;
	mask <<= snum;
	data &= ~mask;	/* clear old bits */

	mdp_writel(pipe->mdp, data, 0x10100); /* MDP_LAYERMIXER_IN_CFG */

	data = mdp_readl(pipe->mdp, 0x10100);

	ctrl->stage[pipe->mixer_num][pipe->mixer_stage] = NULL;	/* clear it */
}

void mdp4_mixer_blend_setup(struct mdp4_overlay_pipe *pipe)
{
	struct mdp4_overlay_pipe *bg_pipe;
	uint32_t overlay_base;
	uint32_t c0, c1, c2, blend_op;
	int off;

	if (pipe->mixer_num) 	/* mixer number, /dev/fb0, /dev/fb1 */
		overlay_base = MDP4_OVERLAYPROC1_BASE;/* 0x18000 */
	else
		overlay_base = MDP4_OVERLAYPROC0_BASE;/* 0x10000 */

	/* stage 0 to stage 2 */
	off = 0x20 * (pipe->mixer_stage - MDP4_MIXER_STAGE0);

	bg_pipe = mdp4_overlay_stage_pipe(pipe->mixer_num,
					MDP4_MIXER_STAGE_BASE);
	if (bg_pipe == NULL) {
		PR_DISP_INFO("%s: Error: no bg_pipe\n", __func__);
		return;
	}


	blend_op = 0;
	if (pipe->is_fg) {
		blend_op |= (MDP4_BLEND_FG_ALPHA_FG_CONST |
				MDP4_BLEND_BG_ALPHA_BG_CONST);
		mdp_writel(pipe->mdp, pipe->alpha, overlay_base + off + 0x108);
		mdp_writel(pipe->mdp, 0xff - pipe->alpha, overlay_base + off + 0x10c);
	} else {
		if (bg_pipe->alpha_enable && pipe->alpha_enable) {
			/* both pipe have alpha */
			blend_op |= (MDP4_BLEND_FG_ALPHA_BG_PIXEL |
				MDP4_BLEND_FG_INV_ALPHA |
				MDP4_BLEND_BG_ALPHA_BG_PIXEL);
	} else if (bg_pipe->alpha_enable && pipe->alpha_enable == 0) {
		blend_op = (MDP4_BLEND_BG_ALPHA_BG_PIXEL |
				MDP4_BLEND_FG_ALPHA_BG_PIXEL |
				MDP4_BLEND_FG_INV_ALPHA);
		}
	}

	if (pipe->transp != MDP_TRANSP_NOP) {
		if (pipe->is_fg) {
			transp_color_key(pipe->src_format, pipe->transp,
						&c0, &c1, &c2);
			// FIXME: We don't enable this function until framework can support color keying.
			//blend_op |= MDP4_BLEND_FG_TRANSP_EN; /* Fg blocked */
			/* lower limit */
			mdp_writel(pipe->mdp, (c1 << 16 | c0), overlay_base + off + 0x110); /* low */
			mdp_writel(pipe->mdp, c2, overlay_base + off + 0x114);/* low */
			/* upper limit */
			mdp_writel(pipe->mdp, (c1 << 16 | c0), overlay_base + off + 0x118);/* high */
			mdp_writel(pipe->mdp, c2, overlay_base + off + 0x11c);/* high */
		} else {
			transp_color_key(bg_pipe->src_format, pipe->transp,
						&c0, &c1, &c2);
			//blend_op |= MDP4_BLEND_BG_TRANSP_EN; /* bg blocked */
			/* lower limit */
			mdp_writel(pipe->mdp, (c1 << 16 | c0), overlay_base + off + 0x180);/* low */
			mdp_writel(pipe->mdp, c2, overlay_base + off + 0x184);/* low */
			/* upper limit */
			mdp_writel(pipe->mdp, (c1 << 16 | c0), overlay_base + off + 0x188);/* high */
			mdp_writel(pipe->mdp, c2, overlay_base + off + 0x18c);/* high */
		}
	}

	mdp_writel(pipe->mdp, blend_op, overlay_base + off + 0x104);
}

void mdp4_overlay_reg_flush(struct mdp4_overlay_pipe *pipe, int all)
{
	uint32_t bits = 0;

	if (pipe->mixer_num == MDP4_MIXER1)
		bits |= 0x02;
	else
		bits |= 0x01;

	if (all) {
		if (pipe->pipe_num <= OVERLAY_PIPE_RGB2) {
			if (pipe->pipe_num == OVERLAY_PIPE_RGB2)
				bits |= 0x20;
			else
				bits |= 0x10;
		} else {
			if (pipe->pipe_num == OVERLAY_PIPE_VG2)
				bits |= 0x08;
			else
				bits |= 0x04;
		}
	}

	mdp_writel(pipe->mdp, bits, 0x18000);	/* MDP_OVERLAY_REG_FLUSH */
}

struct mdp4_overlay_pipe *mdp4_overlay_stage_pipe(int mixer, int stage)
{
	return ctrl->stage[mixer][stage];
}

struct mdp4_overlay_pipe *mdp4_overlay_ndx2pipe(int ndx)
{
	struct mdp4_overlay_pipe *pipe;

	if (ndx <= 0 || ndx > MDP4_MAX_PIPE)
		return NULL;

	pipe = &ctrl->plist[ndx - 1];	/* ndx start from 1 */

	if (pipe->pipe_used == 0)
		return NULL;

	return pipe;
}

struct mdp4_overlay_pipe *mdp4_overlay_pipe_alloc(int ptype, bool usevg)
{
	int i = 0;
	struct mdp4_overlay_pipe *pipe;

	if (usevg)
		i = 2;
	pipe = &ctrl->plist[i];
	for (; i < MDP4_MAX_PIPE; i++) {
		if (pipe->pipe_type == ptype && pipe->pipe_used == 0) {
			init_completion(&pipe->comp);
			PR_DISP_INFO("mdp4_overlay_pipe_alloc: pipe=%x ndx=%d\n",
				(int)pipe, pipe->pipe_ndx);
			return pipe;
		}
		pipe++;
	}

	PR_DISP_INFO("mdp4_overlay_pipe_alloc: ptype=%d FAILED\n",
                                                        ptype);

	return NULL;
}


void mdp4_overlay_pipe_free(struct mdp4_overlay_pipe *pipe)
{
	uint32_t ptype, num, ndx;
	struct mdp4_pipe_desc  *pd;

	PR_DISP_INFO("mdp4_overlay_pipe_free: pipe=%x ndx=%d\n",
					(int)pipe, pipe->pipe_ndx);
	pd = &ctrl->ov_pipe[pipe->pipe_num];
	if (pd->ref_cnt)
		pd->ref_cnt--;

	pd->player = NULL;

	ptype = pipe->pipe_type;
	num = pipe->pipe_num;
	ndx = pipe->pipe_ndx;


	memset(pipe, 0, sizeof(*pipe));

	pipe->pipe_type = ptype;
	pipe->pipe_num = num;
	pipe->pipe_ndx = ndx;
}

int mdp4_overlay_req_check(uint32_t id, uint32_t z_order, uint32_t mixer)
{
	struct mdp4_overlay_pipe *pipe;
	/* HTC to support multiple overlays, correct stage array mapping
         *
	 * base layer == 1, reserved for frame buffer
	 * zorder 0 == stage 0 == 2
	 * zorder 1 == stage 1 == 3
	 * zorder 2 == stage 2 == 4
	 */
	//FIXME: original design has problem, therefore, prevent query out of range stage array here
	if(z_order + MDP4_MIXER_STAGE0 >= MDP4_MAX_STAGE) {
		PR_DISP_ERR("%s(%d) zorder=%d out of stage arry range\n", __func__, __LINE__, z_order);
		return -EPERM;
	}

	pipe = ctrl->stage[mixer][z_order+MDP4_MIXER_STAGE0];

	if (pipe == NULL)
		return 0;

	if (pipe->pipe_ndx == id)	/* same req, recycle */
		return 0;

	if (id == MSMFB_NEW_REQUEST) {  /* new request */
		if (pipe->pipe_num >= OVERLAY_PIPE_VG1) /* share pipe */
			return 0;
	}

	return -EPERM;
}

static int mdp4_overlay_req2pipe(struct mdp_overlay *req, int mixer,
			struct mdp4_overlay_pipe **ppipe)
{
	struct mdp4_overlay_pipe *pipe;
	struct mdp4_pipe_desc  *pd;
	int ret, ptype;

	if (mixer >= MDP4_MAX_MIXER) {
		PR_DISP_ERR("mpd_overlay_req2pipe: mixer out of range!\n");
		return -ERANGE;
	}

	if (req->z_order < 0 || req->z_order > 2) {
		PR_DISP_ERR("mpd_overlay_req2pipe: z_order=%d out of range!\n",
				req->z_order);
		return -ERANGE;
	}

	if (req->src_rect.h == 0 || req->src_rect.w == 0) {
		PR_DISP_ERR("mpd_overlay_req2pipe: src img of zero size!\n");
		return -EINVAL;
	}


	if (req->dst_rect.h > (req->src_rect.h * 8)) {	/* too much */
		PR_DISP_ERR("mpd_overlay_req2pipe: too much (h)!\n");
		return -ERANGE;
	}

	if (req->src_rect.h > (req->dst_rect.h * 8)) {	/* too little */
		PR_DISP_ERR("mpd_overlay_req2pipe: too little (h)!\n");
		return -ERANGE;
	}

	if (req->dst_rect.w > (req->src_rect.w * 8)) {	/* too much */
		PR_DISP_ERR("mpd_overlay_req2pipe: too much (w)!\n");
		return -ERANGE;
	}

	if (req->src_rect.w > (req->dst_rect.w * 8)) {	/* too little */
		PR_DISP_ERR("mpd_overlay_req2pipe: too little (w)!\n");
		return -ERANGE;
	}

	//Current mdp only support the maximum 1/4 downscaling
	if(req->src_rect.w > req->dst_rect.w * 4 || req->src_rect.h > req->dst_rect.h * 4) {
		PR_DISP_ERR("mdp_overlay_req2pipe: Don't support this kind of downscalig srcw=%d srch=%d dstw=%d dsth=%d\n",
		req->src_rect.w, req->src_rect.h, req->dst_rect.w, req->dst_rect.h);
		return -EINVAL;
	}

	ptype = mdp4_overlay_format2type(req->src.format);
	if (ptype < 0)
		return ptype;

	if (req->id == MSMFB_NEW_REQUEST)  /* new request */
		pipe = mdp4_overlay_pipe_alloc(ptype, true);
	else
		pipe = mdp4_overlay_ndx2pipe(req->id);

	if (pipe == NULL)
		return -ENOMEM;

	/* no down scale at rgb pipe */
	if (pipe->pipe_num <= OVERLAY_PIPE_RGB2) {
		if ((req->src_rect.h > req->dst_rect.h) ||
			(req->src_rect.w > req->dst_rect.w)) {
				PR_DISP_ERR("mpd_overlay_req2pipe: h>h || w>w!\n");
				return -ERANGE;
		}
	}

	pipe->src_format = req->src.format;
	ret = mdp4_overlay_format2pipe(pipe);

	if (ret < 0)
		return ret;

	/*
	 * base layer == 1, reserved for frame buffer
	 * zorder 0 == stage 0 == 2
	 * zorder 1 == stage 1 == 3
	 * zorder 2 == stage 2 == 4
	 */
	if (req->id == MSMFB_NEW_REQUEST) {  /* new request */
		pd = &ctrl->ov_pipe[pipe->pipe_num];
		pd->ref_cnt++;
		pipe->pipe_used++;
		pipe->mixer_num = mixer;
		PR_DISP_INFO("mpd4_overlay_req2pipe: zorder=%d pipe_num=%d\n",
				req->z_order, pipe->pipe_num);
	}

	pipe->mixer_stage = req->z_order + MDP4_MIXER_STAGE0;

	pipe->src_width = req->src.width & 0x07ff;	/* source img width */
	pipe->src_height = req->src.height & 0x07ff;	/* source img height */
	pipe->src_h = req->src_rect.h & 0x07ff;
	pipe->src_w = req->src_rect.w & 0x07ff;
	pipe->src_y = req->src_rect.y & 0x07ff;
	pipe->src_x = req->src_rect.x & 0x07ff;
	pipe->dst_h = req->dst_rect.h & 0x07ff;
	pipe->dst_w = req->dst_rect.w & 0x07ff;
	pipe->dst_y = req->dst_rect.y & 0x07ff;
	pipe->dst_x = req->dst_rect.x & 0x07ff;

	mdp4_overlay_parameters_check(pipe);

	pipe->op_mode = 0;

	if (req->flags & MDP_FLIP_LR)
		pipe->op_mode |= MDP4_OP_FLIP_LR;

	if (req->flags & MDP_FLIP_UD)
		pipe->op_mode |= MDP4_OP_FLIP_UD;

	if (req->flags & MDP_DITHER)
		pipe->op_mode |= MDP4_OP_DITHER_EN;

	if (req->flags & MDP_DEINTERLACE)
		pipe->op_mode |= MDP4_OP_DEINT_EN;

	pipe->is_fg = req->is_fg;/* control alpha and color key */

	pipe->alpha = req->alpha & 0x0ff;

	pipe->transp = req->transp_mask;

	*ppipe = pipe;

	return 0;
}


static int get_img(struct msmfb_data *img, struct fb_info *info,
	unsigned long *start, unsigned long *len, struct file **pp_file)
{
	int put_needed, ret = 0, fb_num;
	struct file *file;
#ifdef CONFIG_ANDROID_PMEM
	unsigned long vstart;
#endif

#ifdef CONFIG_ANDROID_PMEM
	if (!get_pmem_file(img->memory_id, start, &vstart, len, pp_file))
		return 0;
#endif
	file = fget_light(img->memory_id, &put_needed);
	if (file == NULL)
		return -1;

	if (MAJOR(file->f_dentry->d_inode->i_rdev) == FB_MAJOR) {
		fb_num = MINOR(file->f_dentry->d_inode->i_rdev);
		if (get_fb_phys_info(start, len, fb_num))
			ret = -1;
		else
			*pp_file = file;
	} else
		ret = -1;
	if (ret)
		fput_light(file, put_needed);
	return ret;
}
#ifdef CONFIG_FB_MSM_WRITE_BACK
int mdp4_overlay_blt(struct mdp_device *mdp_dev,struct fb_info *info, struct msmfb_overlay_blt *req,
		struct file **pp_src_file)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	int lcdc=0;
	ulong addr;
	addr=get_pmem_id_addr(0);
	lcdc = mdp_readl(mdp, 0xc0000);
	if (mfd == NULL)
		return -ENODEV;
	if (req->enable) {
		if (lcdc)
			mdp4_lcdc_overlay_blt(addr); /* enable */
		else
			mdp4_mddi_overlay_blt(addr); /* enable */
	} else {
		if (lcdc)
			mdp4_lcdc_overlay_blt(0); /* disable */
		else
			mdp4_mddi_overlay_blt(0); /* disable */

		PR_DISP_INFO("mdp4_overlay_blt: END\n");
	}

	return 0;
}
#endif

int mdp4_overlay_change_z_order_vg_pipes(struct fb_info *info)
{
	struct mdp4_overlay_pipe *pipeS1,*pipeS2;

	pipeS1 = &ctrl->plist[4];	/* ndx start from 1 */
	pipeS2 = &ctrl->plist[5];	/* ndx start from 1 */

	if(ctrl->stage[pipeS1->mixer_num][pipeS1->mixer_stage] == NULL ||
		ctrl->stage[pipeS2->mixer_num][pipeS2->mixer_stage] == NULL) {
		PR_DISP_ERR("mdp4_overlay_change_z_order_vg_pipes: less than two VG pipes are active \n");
		z_order_change = 0;
		return -ENODEV;
	} else {
		z_order_change = 1;
	}
	return 0;
}

static int mdp4_overlay_change_z_order_vg_pipes_actual(struct mdp4_overlay_pipe *pipe)
{
	int data;
	int pipe1=0, pipe2=0;
	int temp;
	struct mdp4_overlay_pipe *pipeS1,*pipeS2;

	pipeS1 = &ctrl->plist[4];	/* ndx start from 1 */
	pipeS2 = &ctrl->plist[5];	/* ndx start from 1 */

	z_order_change = 0;
	if(ctrl->stage[pipeS1->mixer_num][pipeS1->mixer_stage] == NULL ||
		ctrl->stage[pipeS2->mixer_num][pipeS2->mixer_stage] == NULL) {
		PR_DISP_ERR("mdp4_overlay_change_z_order_vg_pipes: less than two VG pipes are active \n");
		mdp4_mixer_stage_up(pipe);
		return -ENODEV;
	}

	temp = pipeS1->mixer_stage;
	pipeS1->mixer_stage = pipeS2->mixer_stage;
	pipeS2->mixer_stage = temp;

	pipeS1->req_data.z_order = pipeS1->mixer_stage - MDP4_MIXER_STAGE0;
	pipeS2->req_data.z_order = pipeS2->mixer_stage - MDP4_MIXER_STAGE0;

	PR_DISP_INFO("mdp4_overlay_change_z_order_vg_pipes\n");
	data = mdp_readl(pipeS1->mdp, 0x10100);
	PR_DISP_INFO("mdp4_overlay_change_z_order_vg_pipes data = 0x%x\n", data);
	pipe1 = (data & 0xf) << 4;
	PR_DISP_INFO("mdp4_overlay_change_z_order_vg_pipes pipe1 = 0x%x\n",pipe1);
	pipe2 = (data & 0xf0) >> 4;
	PR_DISP_INFO("mdp4_overlay_change_z_order_vg_pipes pipe2 = 0x%x\n", pipe2);
	data &= 0xffffff00;
	data |= pipe1;
	data |= pipe2;
	PR_DISP_INFO("mdp4_overlay_change_z_order_vg_pipes data = 0x%x\n", data);
	mdp_writel(pipeS1->mdp, data, 0x10100);
	return 0;
 }

int mdp4_overlay_get(struct mdp_device *mdp_dev, struct fb_info *info, struct mdp_overlay *req)
{
	struct mdp4_overlay_pipe *pipe;

	pipe = mdp4_overlay_ndx2pipe(req->id);
	if (pipe == NULL)
		return -ENODEV;

	*req = pipe->req_data;

	return 0;
}


static int mdp4_pull_mode(struct mdp4_overlay_pipe *pipe)
{
	uint32_t lcdc;
	if (pipe->mixer_num == MDP4_MIXER1){ /* DTV */
		lcdc = mdp_readl(pipe->mdp, 0xd0000);
		lcdc &= 0x01;
	}
	else{           /* LCDC */
		lcdc = mdp_readl(pipe->mdp, 0x0038);
		lcdc &= 0x08;
	}
	return lcdc;
}

int mdp4_overlay_set(struct mdp_device *mdp_dev, struct fb_info *info, struct mdp_overlay *req)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	int ret, mixer;
	struct mdp4_overlay_pipe *pipe;
	int lcdc;
#ifdef CONFIG_PANEL_SELF_REFRESH
	unsigned long irq_flags = 0;
#endif

	if (req->src.format == MDP_FB_FORMAT)
		req->src.format = MDP_RGB_565;//mfd->fb_imgType;


#ifdef CONFIG_PANEL_SELF_REFRESH
	if (mdp->mdp_dev.overrides & MSM_MDP_RGB_PANEL_SELE_REFRESH) {
		panel_icm->force_leave();
		spin_lock_irqsave(&panel_icm->lock, irq_flags);
		panel_icm->panel_update = 1;
		spin_unlock_irqrestore(&panel_icm->lock, irq_flags);
		wake_up(&panel_update_wait_queue);
		mutex_lock(&panel_icm->icm_lock);
		panel_icm->icm_doable = false;
		mutex_unlock(&panel_icm->icm_lock);
	}
#endif

	mixer = info->node; /* minor number of char device */

	ret = mdp4_overlay_req2pipe(req, mixer, &pipe);
	if (ret < 0) {
		return ret;
	}

	pipe->mdp = mdp;

	/* return id back to user */
	req->id = pipe->pipe_ndx;	/* pipe_ndx start from 1 */
	pipe->req_data = *req;		/* keep original req */

	if(req->user_data[OVERLAY_UPDATE_SCREEN] == OVERLAY_UPDATE_SCREEN_EN
		&& pipe->srcp0_addr) {
		clk_enable(mdp->clk);
		if (pipe->pipe_num >= OVERLAY_PIPE_VG1)
			mdp4_overlay_vg_setup(pipe);	/* video/graphic pipe */
		else
			mdp4_overlay_rgb_setup(pipe);	/* rgb pipe */

		pipe->req_data.user_data[OVERLAY_UPDATE_SCREEN] = OVERLAY_UPDATE_SCREEN_DIS;

		mdp4_mixer_blend_setup(pipe);
		mdp4_mixer_stage_up(pipe);

		lcdc = mdp_readl(mdp, 0xc0000);
		if (lcdc) /* LCDC mode */
			mdp4_overlay_reg_flush(pipe, 1);
		clk_disable(mdp->clk);
	}


	return 0;
}

int mdp4_overlay_unset(struct mdp_device *mdp_dev, struct fb_info *info, int ndx)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	struct mdp4_overlay_pipe *pipe;
	int pull;

	pipe = mdp4_overlay_ndx2pipe(ndx);

	if (pipe == NULL) {
		return -ENODEV;
	}

	pipe->mdp = mdp;
	clk_enable(mdp->clk);

	pull = mdp4_pull_mode(pipe);

	mdp4_mixer_stage_down(pipe);

	if (pull) /* LCDC or DTV mode */
		mdp4_overlay_reg_flush(pipe, 0);
#ifdef CONFIG_FB_MSM_WRITE_BACK
	else
		mdp4_overlay0_done_mddi();
#endif

	mdp4_overlay_pipe_free(pipe);
#ifdef CONFIG_PANEL_SELF_REFRESH
	if (mdp->mdp_dev.overrides & MSM_MDP_RGB_PANEL_SELE_REFRESH) {
		mutex_lock(&panel_icm->icm_lock);
		if(panel_icm->icm_suspend == false)
			panel_icm->icm_doable = true;
		mutex_unlock(&panel_icm->icm_lock);
	}
#endif

	clk_disable(mdp->clk);
	return 0;
}

struct tile_desc {
	uint32_t width;  /* tile's width */
	uint32_t height; /* tile's height */
	uint32_t row_tile_w; /* tiles per row's width */
	uint32_t row_tile_h; /* tiles per row's height */
};

void tile_samsung(struct tile_desc *tp)
{
	/*
	 * each row of samsung tile consists of two tiles in height
	 * and two tiles in width which means width should align to
	 * 64 x 2 bytes and height should align to 32 x 2 bytes.
	 * video decoder generate two tiles in width and one tile
	 * in height which ends up height align to 32 X 1 bytes.
	 */
	tp->width = 64;		/* 64 bytes */
	tp->row_tile_w = 2;	/* 2 tiles per row's width */
	tp->height = 32;	/* 32 bytes */
	tp->row_tile_h = 1;	/* 1 tiles per row's height */
}

uint32_t tile_mem_size(struct mdp4_overlay_pipe *pipe, struct tile_desc *tp)
{
	uint32_t tile_w, tile_h;
	uint32_t row_num_w, row_num_h;


	tile_w = tp->width * tp->row_tile_w;
	tile_h = tp->height * tp->row_tile_h;

	row_num_w = (pipe->src_width + tile_w - 1) / tile_w;
	row_num_h = (pipe->src_height + tile_h - 1) / tile_h;
	return ((row_num_w * row_num_h * tile_w * tile_h) + 8191) & ~8191;
}

int mdp4_overlay_play(struct mdp_device *mdp_dev, struct fb_info *info, struct msmfb_overlay_data *req,
		struct file **pp_src_file)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	struct msmfb_data *img;
	struct mdp4_overlay_pipe *pipe;
	struct mdp4_pipe_desc *pd;
	ulong start, addr;
	ulong len = 0;
	struct file *p_src_file = 0;
	int pull;
#ifdef CONFIG_PANEL_SELF_REFRESH
	unsigned long irq_flags = 0;
#endif

	pipe = mdp4_overlay_ndx2pipe(req->id);
	if (pipe == NULL)
		return -ENODEV;

#ifdef CONFIG_PANEL_SELF_REFRESH
	if (mdp->mdp_dev.overrides & MSM_MDP_RGB_PANEL_SELE_REFRESH) {
		panel_icm->force_leave();
		spin_lock_irqsave(&panel_icm->lock, irq_flags);
		panel_icm->panel_update = 1;
		spin_unlock_irqrestore(&panel_icm->lock, irq_flags);
		wake_up(&panel_update_wait_queue);
	}
#endif

	pd = &ctrl->ov_pipe[pipe->pipe_num];
	if (pd->player && pipe != pd->player) {
		if (pipe->pipe_type == OVERLAY_TYPE_RGB) {
			return 0; /* ignore it, kicked out already */
		}
	}

        pd->player = pipe;      /* keep */

	img = &req->data;
	get_img(img, info, &start, &len, &p_src_file);
	if (len == 0) {
		PR_DISP_ERR("mdp_overlay_play: could not retrieve"
				       " image from memory\n");
		return -1;
	}
	*pp_src_file = p_src_file;

	addr = start + img->offset;
	pipe->srcp0_addr = addr;
	pipe->srcp0_ystride = pipe->src_width * pipe->bpp;
	pipe->mdp = mdp;

	clk_set_rate(mdp->ebi1_clk, 153000000);
	clk_enable(mdp->clk);

	if (pipe->fetch_plane == OVERLAY_PLANE_PSEUDO_PLANAR) {
		if (pipe->frame_format == MDP4_FRAME_FORMAT_VIDEO_SUPERTILE) {
			struct tile_desc tile;

			tile_samsung(&tile);
			pipe->srcp1_addr = addr + tile_mem_size(pipe, &tile);
		} else
			pipe->srcp1_addr = addr +
					pipe->src_width * pipe->src_height;

		pipe->srcp0_ystride = pipe->src_width;
		pipe->srcp1_ystride = pipe->src_width;
	}

	if (pipe->pipe_num >= OVERLAY_PIPE_VG1)
		mdp4_overlay_vg_setup(pipe);	/* video/graphic pipe */
	else
		mdp4_overlay_rgb_setup(pipe);	/* rgb pipe */

	mdp4_mixer_blend_setup(pipe);
	if (z_order_change == 1)
		mdp4_overlay_change_z_order_vg_pipes_actual(pipe);
	else
		mdp4_mixer_stage_up(pipe);

	pull = mdp4_pull_mode(pipe);


	if (pull)	/* LCDC or DTV mode */
		mdp4_overlay_reg_flush(pipe, 1);
		if (pipe->mixer_stage != MDP4_MIXER_STAGE_BASE) { /* done */
			clk_disable(mdp->clk);
			mod_timer(&mdp->standby_timer,
				jiffies + msecs_to_jiffies(1000));
			return 0;
		}
#if 0
	else
		mdp4_mddi_overlay_kickoff(mdp, pipe);
#endif
	clk_disable(mdp->clk);
	mod_timer(&mdp->standby_timer,
		jiffies + msecs_to_jiffies(1000));

	return 0;
}
