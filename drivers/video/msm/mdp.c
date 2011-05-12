/* drivers/video/msm_fb/mdp.c
 *
 * MSM MDP Interface (used by framebuffer core)
 *
 * Copyright (C) 2007 QUALCOMM Incorporated
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

#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/file.h>
#include <linux/android_pmem.h>
#include <linux/major.h>
#include <linux/msm_hw3d.h>

#include <mach/msm_iomap.h>
#include <mach/msm_fb.h>
#include <linux/platform_device.h>

#include "mdp_hw.h"
#include "mdp_ppp.h"
#include <asm/mach-types.h>

#include <mach/debug_display.h>

struct class *mdp_class;

#define MDP_CMD_DEBUG_ACCESS_BASE (0x10000)

static DECLARE_WAIT_QUEUE_HEAD(mdp_ppp_waitqueue);
static unsigned int mdp_irq_mask;
static unsigned int mdp_dma_timer_enable = 0;
struct clk *mdp_clk_to_disable_later = 0;
static unsigned int mdp_dma_user_requested = 0;
static struct  mdp_blit_req *timeout_req;
static uint32_t mdp_reg_addr = 0;
static uint32_t mdp_reg_val = 0;

#ifdef CONFIG_FB_MSM_OVERLAY
extern int mdp4_overlay_get(struct mdp_device *mdp_dev, struct fb_info *info, struct mdp_overlay *req);
extern int mdp4_overlay_set(struct mdp_device *mdp_dev, struct fb_info *info, struct mdp_overlay *req);
extern int mdp4_overlay_unset(struct mdp_device *mdp_dev, struct fb_info *info, int ndx);
extern int mdp4_overlay_play(struct mdp_device *mdp_dev, struct fb_info *info, struct msmfb_overlay_data *req,
				struct file **pp_src_file);
extern void mdp4_mddi_overlay(void *priv, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y);
#include "mdp4.h"
#endif
#ifdef CONFIG_MSM_MDP40
spinlock_t mdp_spin_lock;
uint32_t mdp_intr_mask = MDP4_ANY_INTR_MASK;
#endif

#if defined (CONFIG_FB_MSM_MDP_ABL)
#define MDP_HIST_MAX_BIN 32
struct mdp_histogram mdp_hist;
struct completion mdp_hist_comp;
bool  mdp_is_hist_start = false;
static DEFINE_MUTEX(mdp_hist_mutex);
DEFINE_MUTEX(mdp_lut_push_sem);
static int mdp_lut_push;
static int mdp_lut_push_i;
static unsigned int mdp_hist_r[MDP_HIST_MAX_BIN];
static unsigned int mdp_hist_g[MDP_HIST_MAX_BIN];
static unsigned int mdp_hist_b[MDP_HIST_MAX_BIN];
extern bool mdp_is_hist_start;
#endif


static void mdp_do_standby_timer(unsigned long data)
{
	struct mdp_info *mdp = (struct mdp_info *) data;
	if (!mdp_irq_mask) {
		clk_set_rate(mdp->ebi1_clk, 0);
		mdp->state |= MDP_STATE_STANDBY;
	} else {
		mod_timer(&mdp->standby_timer,
			jiffies + msecs_to_jiffies(200));
	}
}

static int locked_enable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	BUG_ON(!mask);

	/* if the mask bits are already set return an error, this interrupt
	 * is already enabled */
	if (mdp_irq_mask & mask) {
		PR_DISP_ERR("mdp irq already on %x %x\n", mdp_irq_mask, mask);
		return -1;
	}
	/* if the mdp irq is not already enabled enable it */
	if (!mdp_irq_mask) {
		clk_enable(mdp->clk);
		enable_irq(mdp->irq);
		if (mdp->state & MDP_STATE_STANDBY) {
#ifdef CONFIG_MSM_MDP40
			clk_set_rate(mdp->ebi1_clk, 153000000);
#else
			clk_set_rate(mdp->ebi1_clk, 128000000);
#endif
			mdp->state &= ~MDP_STATE_STANDBY;
		} else {
			del_timer_sync(&mdp->standby_timer);
			barrier();
		}
	}

	/* clear out any previous irqs for the requested mask*/
	mdp_writel(mdp, mask, MDP_INTR_CLEAR);

	/* update the irq mask to reflect the fact that the interrupt is
	 * enabled */
	mdp_irq_mask |= mask;
	mdp_writel(mdp, mdp_irq_mask, MDP_INTR_ENABLE);
	return 0;
}

static int enable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	unsigned long flags=0;
	int ret;

	spin_lock_irqsave(&mdp->lock, flags);
	ret = locked_enable_mdp_irq(mdp, mask);
	spin_unlock_irqrestore(&mdp->lock, flags);
	return ret;
}

static int locked_disable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	/* this interrupt is already disabled! */
	if (!(mdp_irq_mask & mask)) {
		PR_DISP_ERR("mdp irq already off %x %x\n",
		       mdp_irq_mask, mask);
		return -1;
	}
	/* update the irq mask to reflect the fact that the interrupt is
	 * disabled */
	mdp_irq_mask &= ~(mask);
	mdp_writel(mdp, mdp_irq_mask, MDP_INTR_ENABLE);

	/* if no one is waiting on the interrupt, disable it */
	if (!mdp_irq_mask) {
		disable_irq_nosync(mdp->irq);
		if (mdp->clk)
			clk_disable(mdp->clk);
		if (!(mdp->state & MDP_STATE_STANDBY))
			mod_timer(&mdp->standby_timer,
				jiffies + msecs_to_jiffies(200));
	}
	return 0;
}

int disable_mdp_irq(struct mdp_info *mdp, uint32_t mask)
{
	unsigned long irq_flags=0;
	int ret;

	spin_lock_irqsave(&mdp->lock, irq_flags);
	ret = locked_disable_mdp_irq(mdp, mask);
	spin_unlock_irqrestore(&mdp->lock, irq_flags);
	return ret;
}

static irqreturn_t mdp_isr(int irq, void *data)
{
	uint32_t status;
	unsigned long irq_flags=0;
	struct mdp_info *mdp = data;
	int i;
#if defined (CONFIG_FB_MSM_MDP_ABL)
	uint32_t hist_status;
	uint32_t hist_enable;
#endif

	spin_lock_irqsave(&mdp->lock, irq_flags);

	status = mdp_readl(mdp, MDP_INTR_STATUS);
	mdp_writel(mdp, status, MDP_INTR_CLEAR);

//	PR_DISP_INFO("%s: status=%08x (irq_mask=%08x)\n", __func__, status,
//		mdp_irq_mask);

	if (mdp_dma_timer_enable) {
		del_timer_sync(&mdp->dma_timer);
		mdp_dma_timer_enable = 0;
		PR_DISP_ERR("%s: stop dma timer\n", __func__);
	}

	status &= mdp_irq_mask;
#ifdef CONFIG_MSM_MDP40
	if (mdp->hw_version < MDP4_REVISION_V2_1 &&
		mdp->mdp_dev.overrides & MSM_MDP4_MDDI_DMA_SWITCH) {
		if(status && mdp->out_if[MSM_MDDI_PMDH_INTERFACE].dma_cb != NULL)
			status |= (INTR_OVERLAY0_DONE | MDP_DMA_S_DONE);
	}
#endif
	for (i = 0; i < MSM_MDP_NUM_INTERFACES; ++i) {
		struct mdp_out_interface *out_if = &mdp->out_if[i];
		if (status & out_if->dma_mask) {
			if (out_if->dma_cb) {
				out_if->dma_cb->func(out_if->dma_cb);
				out_if->dma_cb = NULL;
			}
			wake_up(&out_if->dma_waitqueue);
		}
		if (status & out_if->irq_mask) {
			out_if->irq_cb->func(out_if->irq_cb);
			out_if->irq_cb = NULL;
		}
	}

#ifndef CONFIG_MSM_MDP40
	mdp_ppp_handle_isr(mdp, status);
#endif

#if defined (CONFIG_FB_MSM_MDP_ABL)
	if (status & INTR_DMA_P_HISTOGRAM) {

		hist_status = mdp_readl(mdp, MDP_DMA_P_HIST_INTR_STATUS);
		mdp_writel(mdp, hist_status, MDP_DMA_P_HIST_INTR_CLEAR);
		hist_enable = mdp_readl(mdp, MDP_DMA_P_HIST_INTR_ENABLE);
//		printk("%s: hist_enable=%08x, hist_status=%08x\n", __func__, hist_enable, hist_status);

		hist_status &= hist_enable;

		if (hist_status & INTR_HIST_DONE) {
			if (mdp_hist.r)
				memcpy(mdp_hist.r, mdp->base + 0x95100,
						mdp_hist.bin_cnt*4);
			if (mdp_hist.g)
				memcpy(mdp_hist.g, mdp->base + 0x95200,
						mdp_hist.bin_cnt*4);
			if (mdp_hist.b)
				memcpy(mdp_hist.b, mdp->base + 0x95300,
						mdp_hist.bin_cnt*4);
			complete(&mdp_hist_comp);

                        if (mdp_is_hist_start == true) {
				mdp_writel(mdp, mdp_hist.frame_cnt, 0x95004);
				mdp_writel(mdp, 1, 0x95000);
                        }
		}
		// Clear INTR_DMA_P_HISTOGRAM to prevent disabling interrupt
		status &= ~(INTR_DMA_P_HISTOGRAM);
	}

	if (status & INTR_PRIMARY_INTF_UDERRUN) {
		/* When underun occurs mdp clear the histogram registers
		* that are set before in hw_init so restore them back so
		*
		* that histogram works.*/
		mdp_writel(mdp, 1, 0x95010);
		mdp_writel(mdp, INTR_HIST_DONE, 0x9501c);
		status &= ~(INTR_PRIMARY_INTF_UDERRUN);

		if (mdp_is_hist_start == true) {
			mdp_writel(mdp, mdp_hist.frame_cnt, 0x95004);
			mdp_writel(mdp, 1, 0x95000);
		}
	}
#endif

	if (status)
		locked_disable_mdp_irq(mdp, status);

	spin_unlock_irqrestore(&mdp->lock, irq_flags);
	return IRQ_HANDLED;
}

static void mdp_do_dma_timer(unsigned long data)
{
	uint32_t status;
	struct mdp_info *mdp = (struct mdp_info *) data;
	unsigned long irq_flags=0;
	int i;
	spin_lock_irqsave(&mdp->lock, irq_flags);
	status = mdp_readl(mdp, MDP_INTR_STATUS);
	mdp_writel(mdp, mdp_irq_mask, MDP_INTR_CLEAR);

	for (i = 0; i < MSM_MDP_NUM_INTERFACES; ++i) {
		struct mdp_out_interface *out_if = &mdp->out_if[i];
		if (mdp_irq_mask & out_if->dma_mask) {
			if (out_if->dma_cb) {
				out_if->dma_cb->func(out_if->dma_cb);
				out_if->dma_cb = NULL;
			}
			wake_up(&out_if->dma_waitqueue);
		}
		if (mdp_irq_mask & out_if->irq_mask) {
			out_if->irq_cb->func(out_if->irq_cb);
			out_if->irq_cb = NULL;
		}
	}

	if (mdp_irq_mask & DL0_ROI_DONE)
		mdp_ppp_handle_isr(mdp, DL0_ROI_DONE);

	locked_disable_mdp_irq(mdp, mdp_irq_mask);

	spin_unlock_irqrestore(&mdp->lock, irq_flags);

}

static uint32_t mdp_check_mask(struct mdp_info *mdp, uint32_t mask)
{
	uint32_t ret;
	unsigned long irq_flags=0;

	spin_lock_irqsave(&mdp->lock, irq_flags);
	ret = mdp_irq_mask & mask;
	spin_unlock_irqrestore(&mdp->lock, irq_flags);
	return ret;
}

void mdp_dump_blit(struct mdp_blit_req *req)
{
	PR_DISP_INFO("%s: src: w=%d h=%d f=0x%x offs=0x%x mem_id=%d\n", __func__,
		req->src.width, req->src.height, req->src.format,
		req->src.offset, req->src.memory_id);
	PR_DISP_INFO("%s: dst: w=%d h=%d f=0x%x offs=0x%x mem_id=%d\n", __func__,
		req->dst.width, req->dst.height, req->dst.format,
		req->dst.offset, req->dst.memory_id);
	PR_DISP_INFO("%s: src_rect: x=%d y=%d w=%d h=%d\n", __func__,
		req->src_rect.x, req->src_rect.y, req->src_rect.w,
		req->src_rect.h);
	PR_DISP_INFO("%s: dst_rect: x=%d y=%d w=%d h=%d\n", __func__,
		req->dst_rect.x, req->dst_rect.y, req->dst_rect.w,
		req->dst_rect.h);
	PR_DISP_INFO("%s: alpha=0x%08x\n", __func__, req->alpha);
	PR_DISP_INFO("%s: transp_max=0x%08x\n", __func__, req->transp_mask);
	PR_DISP_INFO("%s: flags=%08x\n", __func__, req->flags);
}

int mdp_wait(struct mdp_info *mdp, uint32_t mask, wait_queue_head_t *wq)
{
	int ret = 0;
	unsigned long irq_flags=0;

//	PR_DISP_INFO("%s: WAITING for 0x%x\n", __func__, mask);
	wait_event_timeout(*wq, !mdp_check_mask(mdp, mask), HZ);

	spin_lock_irqsave(&mdp->lock, irq_flags);
	if (mdp_irq_mask & mask) {
		locked_disable_mdp_irq(mdp, mask);
		PR_DISP_WARN("%s: timeout waiting for mdp to complete 0x%x\n",
			   __func__, mask);
	if(timeout_req)
		mdp_dump_blit(timeout_req);

		ret = -ETIMEDOUT;
	} else {
//		PR_DISP_INFO("%s: SUCCESS waiting for 0x%x\n", __func__, mask);
	}
	spin_unlock_irqrestore(&mdp->lock, irq_flags);

	return ret;
}

static void mdp_dma_wait(struct mdp_device *mdp_dev, int interface)
{
#define MDP_MAX_TIMEOUTS 20
	static int timeout_count;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	unsigned int mask = 0;
	wait_queue_head_t *wq;

	switch (interface) {
	case MSM_MDDI_PMDH_INTERFACE:
	case MSM_MDDI_EMDH_INTERFACE:
	case MSM_LCDC_INTERFACE:
	case MSM_TV_INTERFACE:
		BUG_ON(!mdp->out_if[interface].registered);
		mask = mdp->out_if[interface].dma_mask;
		wq = &mdp->out_if[interface].dma_waitqueue;
		break;
	default:
		PR_DISP_ERR("%s: Unknown interface %d\n", __func__, interface);
		BUG();
	}

	if (mdp_wait(mdp, mask, wq) == -ETIMEDOUT)
		timeout_count++;
	else
		timeout_count = 0;

	if (timeout_count > MDP_MAX_TIMEOUTS) {
		PR_DISP_ERR("mdp: dma failed %d times, somethings wrong!\n",
		       MDP_MAX_TIMEOUTS);
		BUG();
	}
}
/*
static int mdp_ppp_wait(struct mdp_info *mdp)
{
	return mdp_wait(mdp, DL0_ROI_DONE, &mdp_ppp_waitqueue);
}
*/
#ifndef CONFIG_MSM_MDP40
static void mdp_dmas_to_mddi(void *priv, uint32_t addr, uint32_t stride,
		uint32_t width, uint32_t height, uint32_t x, uint32_t y)
{
	struct mdp_info *mdp = priv;
	uint32_t dma2_cfg;
	uint32_t video_packet_parameter = 0;
	uint16_t ld_param = 1;

	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

	dma2_cfg |= mdp->dma_format;

#if defined CONFIG_MSM_MDP22 || defined CONFIG_MSM_MDP30
	if (mdp->dma_format == DMA_IBUF_FORMAT_RGB888_OR_ARGB8888)
#else
	if (mdp->dma_format == DMA_IBUF_FORMAT_XRGB8888)
#endif
		dma2_cfg |= DMA_PACK_PATTERN_BGR;
	else
		dma2_cfg |= DMA_PACK_PATTERN_RGB;

	dma2_cfg |= DMA_OUT_SEL_MDDI;

	dma2_cfg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;

	dma2_cfg |= DMA_DITHER_EN;

	if (mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB565) {
		dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
		video_packet_parameter = MDDI_VDO_PACKET_DESC_RGB565;
	} else if (mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB666) {
		dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
		video_packet_parameter = MDDI_VDO_PACKET_DESC_RGB666;
	}

	/* setup size, address, and stride */
	mdp_writel(mdp, (height << 16) | (width), MDP_DMA_S_SIZE);
	mdp_writel(mdp, addr, MDP_DMA_S_IBUF_ADDR);
	mdp_writel(mdp, stride, MDP_DMA_S_IBUF_Y_STRIDE);

	/* set y & x offset and MDDI transaction parameters */
	mdp_writel(mdp, (y << 16) | (x), MDP_DMA_S_OUT_XY);
	mdp_writel(mdp, ld_param, MDP_MDDI_PARAM_WR_SEL);
	if (mdp->mdp_dev.overrides & MSM_MDP_PANEL_IGNORE_PIXEL_DATA) {
		mdp_writel(mdp, (video_packet_parameter << 16) | 0xE3,
			MDP_MDDI_PARAM);
	}
	else {
		mdp_writel(mdp, (video_packet_parameter << 16) | MDDI_VDO_PACKET_PRIM,
			MDP_MDDI_PARAM);
	}

	mdp_writel(mdp, dma2_cfg, MDP_DMA_S_CONFIG);
	mdp_writel(mdp, 0, MDP_DMA_S_START);
}

static void mdp_dma_to_mddi(void *priv, uint32_t addr, uint32_t stride,
			    uint32_t width, uint32_t height, uint32_t x,
			    uint32_t y)
{
	struct mdp_info *mdp = priv;
	uint32_t dma2_cfg = 0;
	uint32_t video_packet_parameter = 0;
	uint16_t ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */

#if !defined(CONFIG_MSM_MDP30)
	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

#endif
	dma2_cfg |= mdp->dma_format;

#if defined CONFIG_MSM_MDP22 || defined CONFIG_MSM_MDP30
	if (mdp->dma_format == DMA_IBUF_FORMAT_RGB888_OR_ARGB8888)
#else
	if (mdp->dma_format == DMA_IBUF_FORMAT_XRGB8888)
#endif
		dma2_cfg |= DMA_PACK_PATTERN_BGR;
	else
		dma2_cfg |= DMA_PACK_PATTERN_RGB;

	dma2_cfg |= DMA_OUT_SEL_MDDI;

	dma2_cfg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;

#if !defined(CONFIG_MSM_MDP30)
	dma2_cfg |= DMA_DITHER_EN;
#endif

	if (mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB565) {
		dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
		video_packet_parameter = MDDI_VDO_PACKET_DESC_RGB565;
	} else if (mdp->mdp_dev.color_format == MSM_MDP_OUT_IF_FMT_RGB666) {
		dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
		video_packet_parameter = MDDI_VDO_PACKET_DESC_RGB666;
	}


#if defined(CONFIG_MSM_MDP30) || defined(CONFIG_MSM_MDP302)
	writel(height << 16 | width, mdp->base + 0x90004);
	writel(addr, mdp->base + 0x90008);
	writel(stride, mdp->base + 0x9000c);

	/* set y & x offset and MDDI transaction parameters */
	writel(y << 16 | x, mdp->base + 0x90010);
	writel(ld_param, mdp->base + 0x00090);
	writel((video_packet_parameter << 16) | MDDI_VDO_PACKET_PRIM,
		mdp->base + 0x00094);

	writel(dma2_cfg, mdp->base + 0x90000);

	/* start DMA2 */
	writel(0, mdp->base + 0x0044);
#elif defined(CONFIG_MSM_MDP22)
	/* setup size, address, and stride */
	mdp_writel(mdp, (height << 16) | (width),
		   MDP_CMD_DEBUG_ACCESS_BASE + 0x0184);
	mdp_writel(mdp, addr, MDP_CMD_DEBUG_ACCESS_BASE + 0x0188);
	mdp_writel(mdp, stride, MDP_CMD_DEBUG_ACCESS_BASE + 0x018C);

	/* set y & x offset and MDDI transaction parameters */
	mdp_writel(mdp, (y << 16) | (x), MDP_CMD_DEBUG_ACCESS_BASE + 0x0194);
	mdp_writel(mdp, ld_param, MDP_CMD_DEBUG_ACCESS_BASE + 0x01a0);
	mdp_writel(mdp, (video_packet_parameter << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_CMD_DEBUG_ACCESS_BASE + 0x01a4);

	mdp_writel(mdp, dma2_cfg, MDP_CMD_DEBUG_ACCESS_BASE + 0x0180);

	/* start DMA2 */
	mdp_writel(mdp, 0, MDP_CMD_DEBUG_ACCESS_BASE + 0x0044);
#else
	/* setup size, address, and stride */
	mdp_writel(mdp, (height << 16) | (width), MDP_DMA_P_SIZE);
	mdp_writel(mdp, addr, MDP_DMA_P_IBUF_ADDR);
	mdp_writel(mdp, stride, MDP_DMA_P_IBUF_Y_STRIDE);

	/* set y & x offset and MDDI transaction parameters */
	mdp_writel(mdp, (y << 16) | (x), MDP_DMA_P_OUT_XY);
	mdp_writel(mdp, ld_param, MDP_MDDI_PARAM_WR_SEL);
	mdp_writel(mdp, (video_packet_parameter << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_MDDI_PARAM);

	mdp_writel(mdp, dma2_cfg, MDP_DMA_P_CONFIG);
	mdp_writel(mdp, 0, MDP_DMA_P_START);
#endif
}
#endif	/* ifndef CONFIG_MSM_MDP40 */

void mdp_dma(struct mdp_device *mdp_dev, uint32_t addr, uint32_t stride,
	     uint32_t width, uint32_t height, uint32_t x, uint32_t y,
	     struct msmfb_callback *callback, int interface)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	struct mdp_out_interface *out_if;
	unsigned long flags;

	if (interface < 0 || interface >= MSM_MDP_NUM_INTERFACES ||
	    !mdp->out_if[interface].registered) {
		PR_DISP_ERR("%s: Unknown interface: %d\n", __func__, interface);
		BUG();
	}
	out_if = &mdp->out_if[interface];

	spin_lock_irqsave(&mdp->lock, flags);
	if (locked_enable_mdp_irq(mdp, out_if->dma_mask)) {
		mdp_dma_user_requested++;
                if (mdp_dma_user_requested > 2) {
                        PR_DISP_ERR("%s: really busy? start dma timer\n", __func__);
			/* something wrong in dma, workaround it */
			mdp_dma_timer_enable = 1;
			mdp_dma_user_requested = 0;
                } else {
			PR_DISP_ERR("%s: busy\n", __func__);
			goto done;
		}
	} else
		mdp_dma_user_requested = 0;

	out_if->dma_cb = callback;
	out_if->dma_start(out_if->priv, addr, stride, width, height, x, y);

	if (mdp_dma_timer_enable) {
		mdp_writel(mdp, mdp_irq_mask & ~out_if->dma_mask, MDP_INTR_ENABLE);
		PR_DISP_ERR("%s: start dma timer\n", __func__);
		mod_timer(&mdp->dma_timer,
			jiffies + msecs_to_jiffies(30));
	}
done:
	spin_unlock_irqrestore(&mdp->lock, flags);
}


void mdp_configure_dma(struct mdp_device *mdp_dev)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	uint32_t dma_cfg;

	if (!mdp->dma_config_dirty)
		return;
	dma_cfg = mdp_readl(mdp, MDP_DMA_P_CONFIG);
	dma_cfg &= ~DMA_IBUF_FORMAT_MASK;
	dma_cfg &= ~DMA_PACK_PATTERN_MASK;
	dma_cfg |= (mdp->dma_format | mdp->dma_pack_pattern);
	mdp_writel(mdp, dma_cfg, MDP_DMA_P_CONFIG);
	mdp->dma_config_dirty = false;

	return;
}

int mdp_check_output_format(struct mdp_device *mdp_dev, int bpp)
{
	switch (bpp) {
	case 16:
	case 24:
	case 32:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

void mdp_set_panel_size(struct mdp_device *mdp_dev, int width, int height)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	mdp->mdp_dev.width = width;
	mdp->mdp_dev.height = height;
}

int mdp_set_output_format(struct mdp_device *mdp_dev, int bpp)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	uint32_t format, pack_pattern = DMA_PACK_PATTERN_RGB;

	switch (bpp) {
	case 16:
		format = DMA_IBUF_FORMAT_RGB565;
		pack_pattern = DMA_PACK_PATTERN_RGB;
		break;
#if defined CONFIG_MSM_MDP22 || defined CONFIG_MSM_MDP30
	case 24:
	case 32:
		format = DMA_IBUF_FORMAT_RGB888_OR_ARGB8888;
		break;
#else
	case 24:
		format = DMA_IBUF_FORMAT_RGB888;
		pack_pattern = DMA_PACK_PATTERN_BGR;
		break;
	case 32:
		format = DMA_IBUF_FORMAT_XRGB8888;
		pack_pattern = DMA_PACK_PATTERN_BGR;
		break;
#endif
	default:
		return -EINVAL;
	}
	if (format != mdp->dma_format || pack_pattern != mdp->dma_pack_pattern) {
		mdp->dma_format = format;
		mdp->dma_pack_pattern = pack_pattern;
		mdp->dma_config_dirty = true;
	}

	return 0;
}

int mdp_blit(struct mdp_device *mdp_dev, struct fb_info *fb,
	     struct mdp_blit_req *req)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	timeout_req = req;
	return mdp_ppp_blit(mdp, fb, req);
}



#if defined (CONFIG_FB_MSM_MDP_ABL)
int mdp_lut_update(struct mdp_device *mdp_dev, struct fb_info *fb,
	     struct fb_cmap *cmap)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
        unsigned int i, addr, val;
        unsigned short *r, *g, *b;
	int mdp_lut_i = 0;

        for (i = 0; i < cmap->len; i++) {
		r = cmap->red++;
		g = cmap->green++;
		b = cmap->blue++;

		addr = 0x94800 + (0x400 * mdp_lut_i) + cmap->start * 4 + i * 4;
		val = ((*g & 0xff) |((*b & 0xff) << 8) | ((*r & 0xff) << 16));
		mdp_writel(mdp, val, addr);
        }

	/* lcdc case */
        mdp_writel(mdp, (mdp_lut_i << 10) | 0x17, 0x90070);
        mdp_lut_i = (mdp_lut_i + 1)%2;

#if 0 //non lcdc case
        mutex_lock(&mdp_lut_push_sem);
        mdp_lut_push = 1;
        mdp_lut_push_i = mdp_lut_i;
        mutex_unlock(&mdp_lut_push_sem);
        mdp_lut_i = (mdp_lut_i + 1)%2;
#endif
	return 0;
}

static void mdp_lut_enable(struct mdp_info *mdp)
{
        if (mdp_lut_push) {
                mutex_lock(&mdp_lut_push_sem);
                mdp_lut_push = 0;
                mutex_unlock(&mdp_lut_push_sem);
		mdp_writel(mdp, (mdp_lut_push_i << 10) | 0x17 | 1<<3, 0x90070);
        }
}

void mdp_pipe_kickoff(struct mdp_info *mdp, uint32_t term)
{
	/* complete all the writes before starting */
        wmb();

        if (term == MDP_OVERLAY0_TERM) {
                mdp_lut_enable(mdp);
		mdp_writel(mdp, 0, 0x0004);
        } else if (term == MDP_OVERLAY1_TERM) {
                mdp_lut_enable(mdp);
		mdp_writel(mdp, 0, 0x0008);
        }

}

int mdp_start_histogram(struct mdp_device *mdp_dev, struct fb_info *info)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
        int ret = 0;
	int enable = 0;

	mutex_lock(&mdp_hist_mutex);
	if (mdp_is_hist_start == true) {
               PR_DISP_ERR("%s histogram already started\n", __func__);
               ret= -EPERM;
               goto mdp_hist_start_err;
	}

	mdp_is_hist_start = true;

#if 1 //CONFIG_FB_MSM_LCDC
	enable = mdp_readl(mdp, 0x0050);
	enable |= 0x80;
	mdp_writel(mdp, enable, 0x0050);
#endif

	mdp_writel(mdp, 1, 0x95004);
	mdp_writel(mdp, 1, 0x95000);
mdp_hist_start_err:
       mutex_unlock(&mdp_hist_mutex);
       return ret;
}

int mdp_stop_histogram(struct mdp_device *mdp_dev, struct fb_info *info)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	int ret = 0;
	int enable = 0;

	mutex_lock(&mdp_hist_mutex);
	if (!mdp_is_hist_start) {
		PR_DISP_ERR("%s histogram already stopped\n", __func__);
		ret = -EPERM;
		goto mdp_hist_stop_err;
	}
	mdp_is_hist_start = false;

       /* disable the irq for histogram since we handled it
          when the control reaches here */
#if 1 //CONFIG_FB_MSM_LCDC
	enable = mdp_readl(mdp, 0x0050);
	enable &= ~0x80;
	mdp_writel(mdp, enable, 0x0050);
#endif

mdp_hist_stop_err:
       mutex_unlock(&mdp_hist_mutex);
       return ret;
}

static int mdp_do_histogram(struct mdp_device *mdp_dev, struct mdp_histogram *hist, struct mdp_histogram *out)
{
	int ret = 0;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	if (!hist->frame_cnt || (hist->bin_cnt == 0) ||
                                 (hist->bin_cnt > MDP_HIST_MAX_BIN))
                return -EINVAL;
	mutex_lock(&mdp_hist_mutex);
	if (!mdp_is_hist_start) {
		PR_DISP_ERR("%s histogram not started\n", __func__);
		mutex_unlock(&mdp_hist_mutex);
		return -EPERM;
	}
	enable_mdp_irq(mdp, INTR_DMA_P_HISTOGRAM | INTR_PRIMARY_INTF_UDERRUN);
	mutex_unlock(&mdp_hist_mutex);

        INIT_COMPLETION(mdp_hist_comp);

        mdp_hist.bin_cnt = hist->bin_cnt;
	mdp_hist.frame_cnt = hist->frame_cnt;
        mdp_hist.r = (hist->r) ? mdp_hist_r : 0;
        mdp_hist.g = (hist->g) ? mdp_hist_g : 0;
        mdp_hist.b = (hist->b) ? mdp_hist_b : 0;

        wait_for_completion_killable(&mdp_hist_comp);

	out->r = mdp_hist.r;
	out->g = mdp_hist.g;
	out->b = mdp_hist.b;
	disable_mdp_irq(mdp, INTR_DMA_P_HISTOGRAM | INTR_PRIMARY_INTF_UDERRUN);
        return ret;
}

int mdp_get_gamma_curvy(struct mdp_device *mdp_dev, struct gamma_curvy *gc)
{
	uint32_t *ref_y_gamma;
	uint32_t *ref_y_shade;
	uint32_t *ref_bl_lvl;
	uint32_t *ref_y_lvl;
	int i = 0;

	if (!mdp_dev->abl_gamma_tbl)
		return -1;

	ref_y_gamma = mdp_dev->abl_gamma_tbl->ref_y_gamma;
	ref_y_shade = mdp_dev->abl_gamma_tbl->ref_y_shade;
	ref_bl_lvl = mdp_dev->abl_gamma_tbl->ref_bl_lvl;
	ref_y_lvl = mdp_dev->abl_gamma_tbl->ref_y_lvl;

	/* size fo ref_Y_gamma should be the same as size of ref_Y_shade*/
	if (sizeof(gc->ref_y_gamma) / 4 != sizeof(gc->ref_y_shade) / 4)
	    return -1;

	/* size fo ref_bl_lvl should be the same as size of ref_Y_lvl*/
	if (sizeof(gc->ref_bl_lvl) / 4 != sizeof(gc->ref_y_lvl) / 4)
	    return -1;

	gc->gamma_len = sizeof(gc->ref_y_gamma) / 4;
	gc->bl_len = sizeof(gc->ref_bl_lvl) / 4;

	for (i = 0; i < gc->gamma_len; i++) {
		gc->ref_y_gamma[i] = ref_y_gamma[i];
		gc->ref_y_shade[i] = ref_y_shade[i];
	}

	for (i = 0; i < gc->bl_len; i++) {
		gc->ref_bl_lvl[i] = ref_bl_lvl[i];
		gc->ref_y_lvl[i] = ref_y_lvl[i];
	}
	return 0;
}

#endif

void mdp_set_grp_disp(struct mdp_device *mdp_dev, unsigned disp_id)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	disp_id &= 0xf;
	mdp_writel(mdp, disp_id, MDP_FULL_BYPASS_WORD43);
}

/* used by output interface drivers like mddi and lcdc */
int mdp_out_if_register(struct mdp_device *mdp_dev, int interface,
			void *private_data, uint32_t dma_mask,
			mdp_dma_start_func_t dma_start)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	unsigned long flags=0;
	int ret = 0;

	if (interface < 0 || interface >= MSM_MDP_NUM_INTERFACES) {
		PR_DISP_ERR("%s: invalid interface (%d)\n", __func__, interface);
		return -EINVAL;
	}

	spin_lock_irqsave(&mdp->lock, flags);

	if (mdp->out_if[interface].registered) {
		PR_DISP_ERR("%s: interface (%d) already registered\n", __func__,
		       interface);
		ret = -EINVAL;
		goto done;
	}

	init_waitqueue_head(&mdp->out_if[interface].dma_waitqueue);
	mdp->out_if[interface].registered = 1;
	mdp->out_if[interface].priv = private_data;
	mdp->out_if[interface].dma_mask = dma_mask;
	mdp->out_if[interface].dma_start = dma_start;
	mdp->out_if[interface].dma_cb = NULL;

done:
	spin_unlock_irqrestore(&mdp->lock, flags);
	return ret;
}

int mdp_out_if_req_irq(struct mdp_device *mdp_dev, int interface,
		       uint32_t mask, struct msmfb_callback *cb)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	unsigned long flags=0;
	int ret = 0;

	if (interface < 0 || interface >= MSM_MDP_NUM_INTERFACES) {
		PR_DISP_ERR("%s: invalid interface (%d)\n", __func__, interface);
		BUG();
	} else if (!mdp->out_if[interface].registered) {
		PR_DISP_ERR("%s: interface (%d) not registered\n", __func__,
		       interface);
		BUG();
	}

	spin_lock_irqsave(&mdp->lock, flags);

	if (mask) {
		ret = locked_enable_mdp_irq(mdp, mask);
		if (ret) {
			PR_DISP_ERR("%s: busy\n", __func__);
			goto done;
		}
		mdp->out_if[interface].irq_mask = mask;
		mdp->out_if[interface].irq_cb = cb;
	} else {
		locked_disable_mdp_irq(mdp, mask);
		mdp->out_if[interface].irq_mask = 0;
		mdp->out_if[interface].irq_cb = NULL;
	}

done:
	spin_unlock_irqrestore(&mdp->lock, flags);
	return ret;
}

int register_mdp_client(struct class_interface *cint)
{
	if (!mdp_class) {
		PR_DISP_ERR("mdp: no mdp_class when registering mdp client\n");
		return -ENODEV;
	}
	cint->class = mdp_class;
	return class_interface_register(cint);
}

static ssize_t mdp_reg_addr_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "0x%x\n", mdp_reg_addr);

        return ret;
}

static ssize_t mdp_reg_addr_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	unsigned reg_addr;

	sscanf(buf, "%x", &reg_addr);

	if (reg_addr == 0)
		return -EINVAL;

	mdp_reg_addr = reg_addr;

        return count;
}

static DEVICE_ATTR(reg_addr, 0644,  mdp_reg_addr_show,
                        mdp_reg_addr_store);

static ssize_t mdp_reg_val_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct mdp_device *mdp_dev = container_of(dev, struct mdp_device, dev);
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	int ret;

	if (mdp_reg_addr == 0)
		return -EINVAL;

	mdp_reg_val = mdp_readl(mdp, mdp_reg_addr);
	ret = sprintf(buf, "0x%x\n", mdp_reg_val);
        return ret;
}

static ssize_t mdp_reg_val_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	struct mdp_device *mdp_dev = container_of(dev, struct mdp_device, dev);
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	unsigned reg_val,val;
	unsigned mask = 0xffffffff;

	if (mdp_reg_addr == 0)
		return -EINVAL;

	sscanf(buf, "%x %x", &reg_val, &mask);
	mdp_reg_val = reg_val;
	val = mdp_readl(mdp, mdp_reg_addr);
	val &= ~mask;
	mdp_reg_val &= mask;
	mdp_reg_val |= val;
	mdp_writel(mdp, mdp_reg_val, mdp_reg_addr);
        return count;
}

static DEVICE_ATTR(reg_val, 0644,  mdp_reg_val_show,
                        mdp_reg_val_store);


static int
mdp_write_reg_mask(struct mdp_info *mdp, uint32_t reg, uint32_t val, uint32_t mask)
{
	uint32_t oldval, newval;

	oldval = mdp_readl(mdp, reg);

	oldval &= (~mask);
	val &= mask;
	newval = oldval | val;

	mdp_writel(mdp, newval, reg);

	return 0;

}

static int
mdp_write_regs(struct mdp_info *mdp, const struct mdp_reg *reglist, int size)
{
	const struct mdp_reg *reg_seq = reglist;
	int i;

	for (i = 0; i < size; i++) {
		if (reg_seq[i].mask == 0x0)
			mdp_writel(mdp, reg_seq[i].val, reg_seq[i].reg);
		else
			mdp_write_reg_mask(mdp, reg_seq[i].reg, reg_seq[i].val, reg_seq[i].mask);
	}

	return 0;
}

#ifdef CONFIG_MSM_MDP40
static int mdp_hw_version(void)
{
	int mdp_hw_revision;
	char *cp;
	uint32_t *hp;


	mdp_hw_revision = MDP4_REVISION_NONE;

	/* tlmmgpio2 shadow */
	cp = (char *)ioremap(0xac001270, 0x16);

	if (cp == NULL)
		return -1;

	hp = (uint32_t *)cp;	/* HW_REVISION_NUMBER */
	mdp_hw_revision = *hp;
	iounmap(cp);

	mdp_hw_revision >>= 28;	/* bit 31:28 */
	mdp_hw_revision &= 0x0f;

	printk(KERN_INFO "%s: mdp_hw_revision=%x\n",
				__func__, mdp_hw_revision);
	return mdp_hw_revision;
}
#endif

int mdp_probe(struct platform_device *pdev)
{
	struct resource *resource;
	int ret = -EINVAL;
	struct mdp_info *mdp;
	struct msm_mdp_platform_data *pdata = pdev->dev.platform_data;


	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		PR_DISP_ERR("mdp: can not get mdp mem resource!\n");
		return -ENOMEM;
	}

	mdp = kzalloc(sizeof(struct mdp_info), GFP_KERNEL);
	if (!mdp)
		return -ENOMEM;

	spin_lock_init(&mdp->lock);

	mdp->irq = platform_get_irq(pdev, 0);
	if (mdp->irq < 0) {
		PR_DISP_ERR("mdp: can not get mdp irq\n");
		ret = mdp->irq;
		goto error_get_irq;
	}

	mdp->base = ioremap(resource->start,
			    resource->end - resource->start);
	if (mdp->base == 0) {
		PR_DISP_ERR("msmfb: cannot allocate mdp regs!\n");
		ret = -ENOMEM;
		goto error_ioremap;
	}

	mdp->mdp_dev.dma = mdp_dma;
	mdp->mdp_dev.dma_wait = mdp_dma_wait;
	mdp->mdp_dev.blit = mdp_blit;
#ifdef CONFIG_FB_MSM_OVERLAY
	mdp->mdp_dev.overlay_get = mdp4_overlay_get;
	mdp->mdp_dev.overlay_set = mdp4_overlay_set;
	mdp->mdp_dev.overlay_unset = mdp4_overlay_unset;
	mdp->mdp_dev.overlay_play = mdp4_overlay_play;
#endif
	mdp->mdp_dev.set_grp_disp = mdp_set_grp_disp;
	mdp->mdp_dev.set_output_format = mdp_set_output_format;
	mdp->mdp_dev.set_panel_size = mdp_set_panel_size;
	mdp->mdp_dev.check_output_format = mdp_check_output_format;
	mdp->mdp_dev.configure_dma = mdp_configure_dma;

#if defined (CONFIG_FB_MSM_MDP_ABL)
	mdp->mdp_dev.lut_update = mdp_lut_update;
	mdp->mdp_dev.do_histogram = mdp_do_histogram;
	mdp->mdp_dev.start_histogram = mdp_start_histogram;
	mdp->mdp_dev.stop_histogram = mdp_stop_histogram;
	mdp->mdp_dev.get_gamma_curvy = mdp_get_gamma_curvy;
#endif
	mdp->enable_irq = enable_mdp_irq;
	mdp->disable_irq = disable_mdp_irq;
	mdp->write_regs = mdp_write_regs;

	if (pdata == NULL || pdata->overrides == 0)
		mdp->mdp_dev.overrides = 0;
	else if(pdata->overrides)
		mdp->mdp_dev.overrides = pdata->overrides;

	if (pdata != NULL)
		pdata->mdp_dev = &mdp->mdp_dev;

	if (pdata == NULL || pdata->color_format == 0)
		mdp->mdp_dev.color_format = MSM_MDP_OUT_IF_FMT_RGB565;
	else if(pdata->color_format)
		mdp->mdp_dev.color_format = pdata->color_format;

#ifdef CONFIG_MSM_MDP40
	mdp->hw_version = mdp_hw_version();

	if (mdp->hw_version < MDP4_REVISION_V2_1 &&
		mdp->mdp_dev.overrides & MSM_MDP4_MDDI_DMA_SWITCH) {
		ret = mdp_out_if_register(&mdp->mdp_dev,
			MSM_MDDI_PMDH_INTERFACE, mdp, INTR_OVERLAY0_DONE
			| MDP_DMA_S_DONE, mdp4_mddi_overlay);
	} else {
		ret = mdp_out_if_register(&mdp->mdp_dev,
			MSM_MDDI_PMDH_INTERFACE, mdp, INTR_OVERLAY0_DONE,
			mdp4_mddi_overlay);
	}
#else
	if (pdata == NULL || pdata->dma_channel == MDP_DMA_P) {
		ret = mdp_out_if_register(&mdp->mdp_dev,
				MSM_MDDI_PMDH_INTERFACE, mdp, MDP_DMA_P_DONE,
				mdp_dma_to_mddi);
	} else if (pdata->dma_channel == MDP_DMA_S) {
		ret = mdp_out_if_register(&mdp->mdp_dev,
				MSM_MDDI_PMDH_INTERFACE, mdp, MDP_DMA_S_DONE,
				mdp_dmas_to_mddi);
	}
#endif

#if defined (CONFIG_FB_MSM_MDP_ABL)
	if (mdp->mdp_dev.overrides & MSM_MDP_ABL_ENABLE) {
		mdp->mdp_dev.abl_gamma_tbl = pdata->abl_gamma_tbl;
	}
#endif
	if (ret)
		goto error_mddi_pmdh_register;

	mdp->clk = clk_get(&pdev->dev, "mdp_clk");
	if (IS_ERR(mdp->clk)) {
		PR_DISP_INFO("mdp: failed to get mdp clk");
		ret = PTR_ERR(mdp->clk);
		goto error_get_mdp_clk;
	}

        mdp->ebi1_clk = clk_get(NULL, "ebi1_clk");
        if (IS_ERR(mdp->ebi1_clk)) {
                PR_DISP_ERR("mdp: failed to get ebi1 clk\n");
                ret = PTR_ERR(mdp->ebi1_clk);
                goto error_get_ebi1_clk;
        }

	ret = request_irq(mdp->irq, mdp_isr, IRQF_DISABLED, "msm_mdp", mdp);
	if (ret)
		goto error_request_irq;
	disable_irq(mdp->irq);

	clk_enable(mdp->clk);
	mdp_clk_to_disable_later = mdp->clk;

#ifdef CONFIG_MSM_MDP40
	//MDP_DISP_INTF_SEL
	if (mdp_readl(mdp, 0xc0000))
		mdp_writel(mdp, 0x8, 0x0038);
	else
		mdp_writel(mdp, 0xa, 0x0038); //mddi
	//FIXME: should select mddi or lcdc interface
	//mdp_writel(mdp, 0x8, 0x0038); //lcdc
#endif

#ifdef CONFIG_MSM_MDP40
	mdp4_hw_init(mdp);
#else
	mdp_hw_init(mdp);
#endif

#if defined CONFIG_MSM_MDP302
	/* enable the tearing check in MDP */
	if(pdata != NULL && pdata->tearing_check)
		mdp_check_tearing(mdp, pdata);
#endif
	/* register mdp device */
	mdp->mdp_dev.dev.parent = &pdev->dev;
	mdp->mdp_dev.dev.class = mdp_class;
	dev_set_name(&mdp->mdp_dev.dev, "mdp%d", pdev->id);

	/* if you can remove the platform device you'd have to implement
	 * this:
	mdp_dev.release = mdp_class; */

	ret = device_register(&mdp->mdp_dev.dev);
	if (ret)
		goto error_device_register;

	ret = device_create_file(&mdp->mdp_dev.dev, &dev_attr_reg_addr);
	ret = device_create_file(&mdp->mdp_dev.dev, &dev_attr_reg_val);

	setup_timer(&mdp->standby_timer, mdp_do_standby_timer, (unsigned long )mdp);
	setup_timer(&mdp->dma_timer, mdp_do_dma_timer, (unsigned long )mdp);

#if defined (CONFIG_FB_MSM_MDP_ABL)
	init_completion(&mdp_hist_comp);
#endif
	PR_DISP_INFO("%s: initialized\n", __func__);

	return 0;

error_device_register:
	free_irq(mdp->irq, mdp);
error_request_irq:
	clk_put(mdp->ebi1_clk);
error_get_ebi1_clk:
	clk_put(mdp->clk);
error_get_mdp_clk:
error_mddi_pmdh_register:
	iounmap(mdp->base);
error_ioremap:
error_get_irq:
	kfree(mdp);
	return ret;
}

static struct platform_driver msm_mdp_driver = {
	.probe = mdp_probe,
	.driver = {.name = "msm_mdp"},
};

static int __init mdp_lateinit(void)
{
	if (mdp_clk_to_disable_later)
		clk_disable(mdp_clk_to_disable_later);
	return 0;
}

static int __init mdp_init(void)
{
	mdp_class = class_create(THIS_MODULE, "msm_mdp");
	if (IS_ERR(mdp_class)) {
		PR_DISP_ERR("Error creating mdp class\n");
		return PTR_ERR(mdp_class);
	}
	return platform_driver_register(&msm_mdp_driver);
}

subsys_initcall(mdp_init);
late_initcall(mdp_lateinit);
