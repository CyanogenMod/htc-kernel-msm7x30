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

#include "mdp_hw.h"
#include "mdp4.h"

static struct mdp4_overlay_pipe *mddi_pipe = NULL;
static struct mdp_info *mddi_mdp;

#define WHOLESCREEN

void mdp4_overlay_update_lcd(struct mdp_info *mdp, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y)
{
	int ptype;
	uint32_t mddi_ld_param;
	uint16_t mddi_vdo_packet_reg;
	struct mdp4_overlay_pipe *pipe = NULL;

	mddi_mdp = mdp;		/* keep it */


	if (mddi_pipe == NULL) {

		ptype = mdp4_overlay_format2type(MDP_RGB_565);
		pipe = mdp4_overlay_pipe_alloc(ptype);

		pipe->pipe_type = ptype;
		pipe->mdp = mdp;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = MDP_RGB_565;
		mdp4_overlay_format2pipe(pipe);

		mddi_pipe = pipe; /* keep it */

		mddi_ld_param = 0;

		mddi_vdo_packet_reg = MDDI_VDO_PACKET_PRIM;

		mdp_writel(mdp, mddi_ld_param, 0x00090);
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
	pipe->srcp0_ystride = stride;

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

	if (mdp->mdp_dev.overrides & MSM_MDP4_MDDI_DMA_SWITCH) {

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
}

