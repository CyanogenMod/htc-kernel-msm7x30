/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include <linux/msm_mdp.h>
#include <mach/msm_fb.h>
#include <mach/debug_display.h>

#include "mdp_hw.h"
#include "mdp4.h"

static struct mdp4_overlay_pipe *mddi_pipe = NULL;
static struct mdp_info *mddi_mdp;
#ifdef CONFIG_FB_MSM_WRITE_BACK
static struct completion mddi_comp;
static ulong blt_addr;
#endif
#define WHOLESCREEN

#ifdef CONFIG_FB_MSM_WRITE_BACK
void mdp4_mddi_overlay_blt(ulong addr)
{
       unsigned long flag;

       spin_lock_irqsave(&mdp_spin_lock, flag);
       if (addr) {
		mdp_intr_mask |= INTR_DMA_P_DONE;
		if(mddi_pipe!=NULL){
			mdp_writel(mddi_pipe->mdp,mdp_intr_mask,MDP_INTR_ENABLE);
			mddi_pipe->blt_cnt = 0;
			mddi_pipe->blt_end = 0;
			mddi_pipe->blt_addr = addr;
			}
		blt_addr = addr;
       } else {
		mddi_pipe->blt_end = 1; /* mark as end */
		mdp4_dma_p_done_mddi();
       }
       spin_unlock_irqrestore(&mdp_spin_lock, flag);
}
void mdp4_blt_xy_update(struct mdp4_overlay_pipe *pipe)
{
       uint32_t off, addr;

       if (pipe->blt_addr == 0)
               return;

       /* overlay ouput is RG565 */
       off = 0;
       if (pipe->blt_cnt & 0x01)
               off = pipe->src_height * pipe->src_width * 2;

	addr = pipe->blt_addr+ off;

       /* dmap */
	mdp_writel(pipe->mdp, addr, 0x90008);
       /* overlay 0 */
       mdp_writel(pipe->mdp, addr, MDP4_OVERLAYPROC0_BASE + 0x000c);
       mdp_writel(pipe->mdp, addr, MDP4_OVERLAYPROC0_BASE + 0x001c);
}

/*
 * mdp4_dmap_done_mddi: called from isr
 */
void mdp4_dma_p_done_mddi(void)
{
       if (mddi_pipe->blt_end) {
               mddi_pipe->blt_addr = 0;
               mdp_intr_mask &= ~INTR_DMA_P_DONE;
	       mdp_writel(mddi_pipe->mdp,mdp_intr_mask,MDP_INTR_ENABLE);
               mdp4_overlayproc_cfg(mddi_pipe);
               mdp4_overlay_dmap_xy(mddi_pipe);
               return;
       }
}
/*
 * mdp4_overlay0_done_mddi: called from isr
 */
void mdp4_overlay0_done_mddi(void)
{
	if (mddi_pipe->blt_addr) {
		if (mddi_pipe->blt_cnt == 0) {
			mdp4_overlayproc_cfg(mddi_pipe);
			mdp4_overlay_dmap_xy(mddi_pipe);
			mddi_pipe->blt_cnt++;
			/* BLT start from next frame */
		} else {
			mdp4_blt_xy_update(mddi_pipe);
			mddi_pipe->blt_cnt++;

			/* start DMAP */
			mdp_writel(mddi_pipe->mdp,0x0,0x000c);
		}
	}

}
#endif



void mdp4_overlay_update_lcd(struct mdp_info *mdp, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y)
{
	//uint8_t *src;
	int ptype;
	uint32_t mddi_ld_param;
	uint16_t mddi_vdo_packet_reg;
	struct mdp4_overlay_pipe *pipe = NULL;
	int data;

	mddi_mdp = mdp;		/* keep it */


	if (mddi_pipe == NULL) {

		ptype = mdp4_overlay_format2type(MDP_RGB_565);
		pipe = mdp4_overlay_pipe_alloc(ptype, false);

		pipe->pipe_type = ptype;
		pipe->pipe_used = 1;
		pipe->mdp = mdp;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = MDP_RGB_565;
		mdp4_overlay_format2pipe(pipe);

		mddi_pipe = pipe; /* keep it */
#ifdef CONFIG_FB_MSM_WRITE_BACK
                init_completion(&mddi_comp);
                mddi_pipe->blt_cnt = 0;
                mddi_pipe->blt_end = 0;
                mddi_pipe->blt_addr = blt_addr;
#endif

		mddi_ld_param = 0;

		mddi_vdo_packet_reg = MDDI_VDO_PACKET_PRIM;
		if (mdp->hw_version >= MDP4_REVISION_V2_1) {
			data = mdp_readl(mdp, MDP_AXI_RDMASTER_CONFIG);
                        data &= ~0x0300;        /* bit 8, 9, MASTER4 */
                        if (width == 540) /* qHD, 540x960 */
                                data |= 0x0200;
                        else
                                data |= 0x0100;

			mdp_writel(mdp, data, MDP_AXI_RDMASTER_CONFIG);
		}

/*     FIXME: currently we use only one display

		if (mfd->panel_info.type == MDDI_PANEL) {
			if (mfd->panel_info.pdest == DISPLAY_1)
				mddi_ld_param = 0;
			else
				mddi_ld_param = 1;
		} else {
			mddi_ld_param = 2;
		}
*/
		mdp_writel(mdp, mddi_ld_param, 0x00090);
		if ( mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB888 )
			mdp_writel(mdp, (MDDI_VDO_PACKET_DESC_RGB888 << 16) | mddi_vdo_packet_reg, 0x00094);
		else
			mdp_writel(mdp, (MDDI_VDO_PACKET_DESC_RGB565 << 16) | mddi_vdo_packet_reg, 0x00094);
		mdp_writel(mdp, 0x01, 0x00098);
	} else {
		pipe = mddi_pipe;
	}

	pipe->src_height = height;
	pipe->src_width = width;
	pipe->src_h = height;
	pipe->src_w = width;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_h = height;
	pipe->dst_w = width;
	pipe->dst_y = 0;
	pipe->dst_x = 0;
	pipe->srcp0_addr = (uint32_t)addr;
	pipe->srcp0_ystride = (stride + 31) & ~31;

	pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;

	/* 16/32 bits framebuffer */
	if (mddi_mdp->dma_config_dirty)
	{
		if(mddi_mdp->dma_format == DMA_IBUF_FORMAT_RGB565) {
			pipe->src_format = MDP_RGB_565;
			pipe->srcp0_ystride = pipe->src_width * 2;
		} else if(mddi_mdp->dma_format == DMA_IBUF_FORMAT_XRGB8888) {
			pipe->src_format = MDP_RGBA_8888;
			pipe->srcp0_ystride = pipe->src_width * 4;
		}
		mdp4_overlay_format2pipe(pipe);
		mddi_mdp->dma_config_dirty = false;
	}

	mdp4_overlay_rgb_setup(pipe);

	mdp4_mixer_stage_up(pipe);

	mdp4_overlayproc_cfg(pipe);

	mdp4_overlay_dmap_xy(pipe);

	mdp4_overlay_dmap_cfg(pipe, 0);

}

void mdp4_mddi_overlay_kickoff(struct mdp_info *mdp,
				struct mdp4_overlay_pipe *pipe)
{

#ifdef MDP4_NONBLOCKING
	mdp_writel(mdp, 0, 0x0004);
#endif
}
void mdp4_dma_s_update_lcd(struct mdp_info *mdp)
{
	uint32_t mddi_ld_param = 1;
	uint16_t mddi_vdo_packet_reg = MDDI_VDO_PACKET_PRIM;
	struct mdp4_overlay_pipe *pipe = NULL;

	mddi_mdp = mdp;		/* keep it */

	pipe = mddi_pipe;

	/*config PIXELSIZE*/
	mdp4_overlay_dmas_xy(pipe);

	/*config for dma_s_cfg_reg*/
	mdp4_overlay_dmas_cfg(pipe, 0);

	mdp_writel(mdp, mddi_ld_param, 0x00090);
	mdp_writel(mdp, (MDDI_VDO_PACKET_DESC_RGB565 << 16) | mddi_vdo_packet_reg, 0x00094);
	mdp_writel(mdp, 1, 0x00098);
}


void mdp4_mddi_dma_s_kickoff(struct mdp_info *mdp,
				struct mdp4_overlay_pipe *pipe)
{
	mdp_writel(mdp, 0, 0x00010);
}

void mdp4_mddi_overlay(void *priv, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y)
{
        struct mdp_info *mdp = priv;

#ifdef CONFIG_FB_MSM_WRITE_BACK
	if (mdp->hw_version < MDP4_REVISION_V2_1 &&
		mdp->mdp_dev.overrides & MSM_MDP4_MDDI_DMA_SWITCH) {
		mdp4_overlay_update_lcd(mdp, addr, stride, width, height, x, y);

		if(mdp4_overlay_active(mdp, MDP4_MIXER0) > 1){
			mdp4_overlay0_done_mddi();
			mdp4_mddi_overlay_kickoff(mdp, mddi_pipe);
		}
		else {
			if(mddi_pipe->blt_addr){
				mdp4_overlay0_done_mddi();
				mdp4_mddi_overlay_kickoff(mdp, mddi_pipe);
			}else{
				mdp4_dma_s_update_lcd(mdp);
				mdp4_mddi_dma_s_kickoff(mdp, mddi_pipe);
			}
		}
	} else {
		mdp4_overlay_update_lcd(mdp, addr, stride, width, height, x, y);
		mdp4_overlay0_done_mddi();
		mdp4_mddi_overlay_kickoff(mdp, mddi_pipe);
	}
#else
	if (mdp->hw_version < MDP4_REVISION_V2_1 &&
		mdp->mdp_dev.overrides & MSM_MDP4_MDDI_DMA_SWITCH) {
		mdp4_overlay_update_lcd(mdp, addr, stride, width, height, x, y);
		if(mdp4_overlay_active(mdp, MDP4_MIXER0) > 1)
			mdp4_mddi_overlay_kickoff(mdp, mddi_pipe);
		else {
			mdp4_dma_s_update_lcd(mdp);
			mdp4_mddi_dma_s_kickoff(mdp, mddi_pipe);
		}
	} else {
		mdp4_overlay_update_lcd(mdp, addr, stride, width, height, x, y);
		mdp4_mddi_overlay_kickoff(mdp, mddi_pipe);
	}
#endif
}

