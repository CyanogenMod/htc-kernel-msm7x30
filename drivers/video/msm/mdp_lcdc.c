/* drivers/video/msm/mdp_lcdc.c
 *
 * Copyright (c) 2009 Google Inc.
 * Copyright (c) 2009 QUALCOMM Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Dima Zavin <dima@android.com>
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/msm_mdp.h>
#include <mach/msm_fb.h>
#include <mach/debug_display.h>

#include "mdp_hw.h"
#ifdef CONFIG_MSM_MDP40
#include "mdp4.h"
#endif
#ifdef CONFIG_PANEL_SELF_REFRESH
#include <linux/kthread.h>
#endif

#if 0
#define D(fmt, args...) printk(KERN_INFO "Dispaly: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

#if defined(CONFIG_ARCH_MSM7227)
#define LCDC_MUX_CTL (MSM_TGPIO1_BASE + 0x278)
#endif


static struct mdp_device *mdp_dev;

#ifdef CONFIG_MSM_MDP40
static struct mdp4_overlay_pipe *lcdc_pipe;
#endif

#ifdef CONFIG_PANEL_SELF_REFRESH
#if 0
#define ICM_DBG(s...) printk("[icm]" s)
#else
#define ICM_DBG(s...) do {} while (0)
#endif

/* set the timeout to 200 milliseconds */
#define PANEL_ENTER_IDLE_TIMEOUT HZ/5
/* Afetr setting ICM=1, we need to keep sending the RGB signal more than 2-frame */
#define PANEL_IDLE_STABLE_TIMEOUT 48

static struct task_struct *th_display;
struct panel_icm_info *panel_icm;
DECLARE_WAIT_QUEUE_HEAD(panel_update_wait_queue);
#endif

#ifdef CONFIG_PANEL_SELF_REFRESH
static int icm_check_panel_update(void)
{
	int ret;
	unsigned long irq_flags = 0;

	spin_lock_irqsave(&panel_icm->lock, irq_flags);
	ret = panel_icm->panel_update;
	spin_unlock_irqrestore(&panel_icm->lock, irq_flags);
	return ret;
}

static int icm_thread(void *data)
{
	struct mdp_lcdc_info *lcdc;
	struct msm_lcdc_panel_ops *panel_ops;
	int rc;
	unsigned long irq_flags = 0;

	lcdc = data;
	panel_ops = lcdc->pdata->panel_ops;
	while (1) {
		rc = wait_event_timeout(panel_update_wait_queue, icm_check_panel_update() == 1, PANEL_ENTER_IDLE_TIMEOUT);
		ICM_DBG("ICM Thread:wake up rc=%d \n", rc);
		mutex_lock(&panel_icm->icm_lock);
		if (rc == 0 && icm_check_panel_update() != 1) {/* wait_timeout */
			ICM_DBG("EnterICM: icm_mode=%d icm_doable=%d \n", panel_icm->icm_mode, panel_icm->icm_doable);
			if (panel_icm->icm_mode == false && panel_icm->icm_doable == true) {

				if (panel_ops->refresh_enable)
					panel_ops->refresh_enable(panel_ops);

				panel_icm->icm_mode = true;
				msleep(PANEL_IDLE_STABLE_TIMEOUT);

				mdp_writel(lcdc->mdp, 0, MDP_LCDC_EN);
				clk_disable(lcdc->pad_pclk);
				clk_disable(lcdc->pclk);
				clk_disable(lcdc->mdp_clk);
				panel_icm->clock_enabled = false;
				PR_DISP_DEBUG("EnterICM: enter ICM MODE done!!!\n");
			}
		} else {/* get update event, no timeout */
			ICM_DBG("Leave ICM: icm_mode=%d icm_doable=%d \n", panel_icm->icm_mode, panel_icm->icm_doable);
			if (panel_icm->icm_mode == true && panel_icm->icm_doable == true) {
				clk_enable(lcdc->mdp_clk);
				clk_enable(lcdc->pclk);
				clk_enable(lcdc->pad_pclk);
				mdp_writel(lcdc->mdp, 1, MDP_LCDC_EN);
				panel_icm->clock_enabled = true;

				if (panel_ops->refresh_disable)
					panel_ops->refresh_disable(panel_ops);

				panel_icm->icm_mode = false;
				PR_DISP_DEBUG("LeaveICM: leave ICM MODE done !!!\n");
			}
			spin_lock_irqsave(&panel_icm->lock, irq_flags);
			panel_icm->panel_update = 0;
			spin_unlock_irqrestore(&panel_icm->lock, irq_flags);
		}
		mutex_unlock(&panel_icm->icm_lock);
	} /* end while */
	return 0;
}

static void icm_force_leave(void)
{
	struct msm_lcdc_panel_ops *panel_ops;
	unsigned long irq_flags = 0;

	panel_ops = panel_icm->lcdc->pdata->panel_ops;

	mutex_lock(&panel_icm->icm_lock);
	ICM_DBG("Force Leave ICM: icm_mode=%d icm_doable=%d \n", panel_icm->icm_mode, panel_icm->icm_doable);
	if (panel_icm->icm_mode == true) {
		clk_enable(panel_icm->lcdc->mdp_clk);
		clk_enable(panel_icm->lcdc->pclk);
		clk_enable(panel_icm->lcdc->pad_pclk);
		mdp_writel(panel_icm->lcdc->mdp, 1, MDP_LCDC_EN);
		panel_icm->clock_enabled = true;
		if (panel_ops->refresh_disable)
			panel_ops->refresh_disable(panel_ops);
		panel_icm->icm_mode = false;
		panel_icm->icm_doable = true;
                PR_DISP_INFO("ForceLeaveICM: leave ICM MODE done !!!\n");
	}
	spin_lock_irqsave(&panel_icm->lock, irq_flags);
        panel_icm->panel_update = 0;
        spin_unlock_irqrestore(&panel_icm->lock, irq_flags);
	mutex_unlock(&panel_icm->icm_lock);
}

static int icm_init(struct mdp_lcdc_info *lcdc)
{
	int ret = 0;

	/* init panel_icm_info */
	panel_icm = kzalloc(sizeof(struct panel_icm_info), GFP_KERNEL);
	if (!panel_icm)
		return -ENOMEM;
	panel_icm->icm_doable = 1;
	panel_icm->clock_enabled = true;
	panel_icm->lcdc = lcdc;
	panel_icm->force_leave = icm_force_leave;
	panel_icm->icm_suspend = false;
	mutex_init(&panel_icm->icm_lock);
	th_display = kthread_run(icm_thread, lcdc, "panel-enterIdle");
	if (IS_ERR(th_display)) {
		ret = PTR_ERR(th_display);
		PR_DISP_ERR("%s: panel_icm_thread  create fail:%d!!!\n", __func__, ret);
		goto	error_create_thread;
	}
	return ret;
error_create_thread:
	kfree(panel_icm);
	return ret;
}
#endif
#ifdef CONFIG_FB_MSM_WRITE_BACK
void mdp4_lcdc_overlay_blt(ulong addr)
{
       unsigned long flag;

       spin_lock_irqsave(&mdp_spin_lock, flag);
       lcdc_pipe->blt_addr = addr;
       lcdc_pipe->blt_cnt = 0;
       spin_unlock_irqrestore(&mdp_spin_lock, flag);
       mdp_writel(lcdc_pipe->mdp,0x0,0xc0000);
       mdelay(100);
       mdp4_overlayproc_cfg(lcdc_pipe);
       mdp4_overlay_dmap_xy(lcdc_pipe);
       mdelay(100);
       mdp_writel(lcdc_pipe->mdp,0x1,0xc0000);
}
#endif

static int lcdc_unblank(struct msm_panel_data *fb_panel)
{
	struct mdp_lcdc_info *lcdc = panel_to_lcdc(fb_panel);
	struct msm_lcdc_panel_ops *panel_ops = lcdc->pdata->panel_ops;

	PR_DISP_INFO("%s: ()\n", __func__);

	if (panel_ops->unblank)
		panel_ops->unblank(panel_ops);

	return 0;
}

static int lcdc_blank(struct msm_panel_data *fb_panel)
{
	struct mdp_lcdc_info *lcdc = panel_to_lcdc(fb_panel);
	struct msm_lcdc_panel_ops *panel_ops = lcdc->pdata->panel_ops;

	PR_DISP_INFO("%s: ()\n", __func__);

	if (panel_ops->blank)
		panel_ops->blank(panel_ops);

	return 0;
}

static int lcdc_shutdown(struct msm_panel_data *fb_panel)
{
        struct mdp_lcdc_info *lcdc = panel_to_lcdc(fb_panel);
        struct msm_lcdc_panel_ops *panel_ops = lcdc->pdata->panel_ops;

        PR_DISP_INFO("%s: ()\n", __func__);

	if (panel_ops->shutdown)
	        panel_ops->shutdown(panel_ops);

        return 0;
}

static int lcdc_suspend(struct msm_panel_data *fb_panel)
{
	struct mdp_lcdc_info *lcdc = panel_to_lcdc(fb_panel);
	struct msm_lcdc_panel_ops *panel_ops = lcdc->pdata->panel_ops;

	PR_DISP_INFO("%s: suspending\n", __func__);

#if defined(CONFIG_ARCH_MSM7227)
	writel(0x0, LCDC_MUX_CTL);
	D("suspend_lcdc_mux_ctl = %x\n", readl(LCDC_MUX_CTL));
#endif
#ifdef CONFIG_PANEL_SELF_REFRESH
	if (lcdc->mdp->mdp_dev.overrides & MSM_MDP_RGB_PANEL_SELE_REFRESH) {
		mutex_lock(&panel_icm->icm_lock);
		panel_icm->icm_doable = false;
		panel_icm->icm_suspend = true;
		PR_DISP_INFO("[ICM %s]: icm mode=%d, clock_enabled=%d\n", __func__, panel_icm->icm_mode, panel_icm->clock_enabled);
		if (panel_icm->icm_mode == true && panel_icm->clock_enabled == false) {
			if (panel_ops->refresh_disable)
				panel_ops->refresh_disable(panel_ops);
			panel_icm->icm_mode = false;
		} else {
			mdp_writel(lcdc->mdp, 0, MDP_LCDC_EN);
			clk_disable(lcdc->pad_pclk);
			clk_disable(lcdc->pclk);
			clk_disable(lcdc->mdp_clk);
		}
		panel_icm->clock_enabled = false;
		mutex_unlock(&panel_icm->icm_lock);
	} else {
		mdp_writel(lcdc->mdp, 0, MDP_LCDC_EN);
		clk_disable(lcdc->pad_pclk);
		clk_disable(lcdc->pclk);
		clk_disable(lcdc->mdp_clk);
	}
#else
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_EN);
	clk_disable(lcdc->pad_pclk);
	clk_disable(lcdc->pclk);
	clk_disable(lcdc->mdp_clk);
#endif
	if (panel_ops->uninit)
		panel_ops->uninit(panel_ops);

	return 0;
}

static int lcdc_resume(struct msm_panel_data *fb_panel)
{
	struct mdp_lcdc_info *lcdc = panel_to_lcdc(fb_panel);
	struct msm_lcdc_panel_ops *panel_ops = lcdc->pdata->panel_ops;

	PR_DISP_INFO("%s: resuming\n", __func__);

	if (panel_ops->init) {
		if (panel_ops->init(panel_ops) < 0)
			PR_DISP_ERR("LCD init fail!\n");
	}

	clk_enable(lcdc->mdp_clk);
	clk_enable(lcdc->pclk);
	clk_enable(lcdc->pad_pclk);
#if defined(CONFIG_ARCH_MSM7227)
	writel(0x1, LCDC_MUX_CTL);
	D("resume_lcdc_mux_ctl = %x\n", readl(LCDC_MUX_CTL));
#endif

	mdp_writel(lcdc->mdp, 1, MDP_LCDC_EN);
#ifdef CONFIG_PANEL_SELF_REFRESH
	if (lcdc->mdp->mdp_dev.overrides & MSM_MDP_RGB_PANEL_SELE_REFRESH) {
		mutex_lock(&panel_icm->icm_lock);
		panel_icm->icm_doable = true;
		panel_icm->clock_enabled = true;
		panel_icm->icm_suspend = false;
		mutex_unlock(&panel_icm->icm_lock);
	}
#endif

	return 0;
}

static int lcdc_hw_init(struct mdp_lcdc_info *lcdc)
{
	struct msm_panel_data *fb_panel = &lcdc->fb_panel_data;
	uint32_t dma_cfg;

	clk_enable(lcdc->mdp_clk);
	clk_enable(lcdc->pclk);
	clk_enable(lcdc->pad_pclk);

#ifdef CONFIG_MSM_MDP40
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_EN);
	mdelay(10);
#endif
	clk_set_rate(lcdc->pclk, lcdc->parms.clk_rate);
	clk_set_rate(lcdc->pad_pclk, lcdc->parms.clk_rate);
	/* write the lcdc params */
	mdp_writel(lcdc->mdp, lcdc->parms.hsync_ctl, MDP_LCDC_HSYNC_CTL);
	mdp_writel(lcdc->mdp, lcdc->parms.vsync_period, MDP_LCDC_VSYNC_PERIOD);
	mdp_writel(lcdc->mdp, lcdc->parms.vsync_pulse_width,
		   MDP_LCDC_VSYNC_PULSE_WIDTH);
	mdp_writel(lcdc->mdp, lcdc->parms.display_hctl, MDP_LCDC_DISPLAY_HCTL);
	mdp_writel(lcdc->mdp, lcdc->parms.display_vstart,
		   MDP_LCDC_DISPLAY_V_START);
	mdp_writel(lcdc->mdp, lcdc->parms.display_vend, MDP_LCDC_DISPLAY_V_END);
	mdp_writel(lcdc->mdp, lcdc->parms.hsync_skew, MDP_LCDC_HSYNC_SKEW);

	mdp_writel(lcdc->mdp, 0, MDP_LCDC_BORDER_CLR);
	mdp_writel(lcdc->mdp, 0xff, MDP_LCDC_UNDERFLOW_CTL);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_ACTIVE_HCTL);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_ACTIVE_V_START);
	mdp_writel(lcdc->mdp, 0, MDP_LCDC_ACTIVE_V_END);
	mdp_writel(lcdc->mdp, lcdc->parms.polarity, MDP_LCDC_CTL_POLARITY);
	/* config the dma_p block that drives the lcdc data */
	mdp_writel(lcdc->mdp, lcdc->fb_start, MDP_DMA_P_IBUF_ADDR);
	mdp_writel(lcdc->mdp, (((fb_panel->fb_data->yres & 0x7ff) << 16) |
			       (fb_panel->fb_data->xres & 0x7ff)),
		   MDP_DMA_P_SIZE);
	mdp_writel(lcdc->mdp, 0, MDP_DMA_P_OUT_XY);

	dma_cfg = mdp_readl(lcdc->mdp, MDP_DMA_P_CONFIG);
	if (lcdc->mdp->mdp_dev.overrides & MSM_MDP_DMA_PACK_ALIGN_LSB)
		dma_cfg &= ~DMA_PACK_ALIGN_MSB;
	else
		dma_cfg |= DMA_PACK_ALIGN_MSB;

	dma_cfg |= (DMA_PACK_PATTERN_RGB |
		   DMA_DITHER_EN);
	dma_cfg |= DMA_OUT_SEL_LCDC;
	dma_cfg &= ~DMA_DST_BITS_MASK;
	if(lcdc->color_format == MSM_MDP_OUT_IF_FMT_RGB565)
		dma_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	else if (lcdc->color_format == MSM_MDP_OUT_IF_FMT_RGB666)
		dma_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	else if (lcdc->color_format == MSM_MDP_OUT_IF_FMT_RGB888)
		dma_cfg |= DMA_DSTC0G_8BITS | DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;

	mdp_writel(lcdc->mdp, dma_cfg, MDP_DMA_P_CONFIG);

	/* enable the lcdc timing generation */
	mdp_writel(lcdc->mdp, 1, MDP_LCDC_EN);

	return 0;
}

static void lcdc_wait_vsync(struct msm_panel_data *panel)
{
	struct mdp_lcdc_info *lcdc = panel_to_lcdc(panel);
	int ret;

	ret = wait_event_timeout(lcdc->vsync_waitq, lcdc->got_vsync, HZ / 2);
	if (!ret && !lcdc->got_vsync)
		PR_DISP_ERR("%s: timeout waiting for VSYNC\n", __func__);
	lcdc->got_vsync = 0;
}

static void lcdc_request_vsync(struct msm_panel_data *fb_panel,
			       struct msmfb_callback *vsync_cb)
{
	struct mdp_lcdc_info *lcdc = panel_to_lcdc(fb_panel);

	/* the vsync callback will start the dma */
	vsync_cb->func(vsync_cb);
	lcdc->got_vsync = 0;
	mdp_out_if_req_irq(mdp_dev, MSM_LCDC_INTERFACE, MDP_LCDC_FRAME_START,
			   &lcdc->frame_start_cb);
	lcdc_wait_vsync(fb_panel);
}

static void lcdc_clear_vsync(struct msm_panel_data *fb_panel)
{
	struct mdp_lcdc_info *lcdc = panel_to_lcdc(fb_panel);
	lcdc->got_vsync = 0;
	mdp_out_if_req_irq(mdp_dev, MSM_LCDC_INTERFACE, 0, NULL);
}

/* called in irq context with mdp lock held, when mdp gets the
 * MDP_LCDC_FRAME_START interrupt */
static void lcdc_frame_start(struct msmfb_callback *cb)
{
	struct mdp_lcdc_info *lcdc;

	lcdc = container_of(cb, struct mdp_lcdc_info, frame_start_cb);

	lcdc->got_vsync = 1;
	wake_up(&lcdc->vsync_waitq);
}

#ifdef CONFIG_MSM_MDP40
static void lcdc_overlay_start(void *priv, uint32_t addr, uint32_t stride,
			   uint32_t width, uint32_t height, uint32_t x,
			   uint32_t y)
{
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);

	struct mdp4_overlay_pipe *pipe;
	pipe = lcdc_pipe;
	pipe->srcp0_addr = addr;

	if (mdp->dma_config_dirty)
	{
		if(mdp->dma_format == DMA_IBUF_FORMAT_RGB565) {
			pipe->src_format = MDP_RGB_565;
			pipe->srcp0_ystride = pipe->src_width * 2;
		} else if(mdp->dma_format == DMA_IBUF_FORMAT_XRGB8888) {
			pipe->src_format = MDP_RGBA_8888;
			pipe->srcp0_ystride = pipe->src_width * 4;
		}
		mdp4_overlay_format2pipe(pipe);
		mdp_writel(pipe->mdp, 0, MDP_LCDC_EN);
		mdelay(30);
		mdp4_overlay_dmap_xy(pipe);
		mdp4_overlay_dmap_cfg(pipe, 1);
		mdp4_overlayproc_cfg(pipe);
		mdp4_overlay_rgb_setup(pipe);
		mdp4_overlay_reg_flush(pipe, 1); /* rgb1 and mixer0 */
		mdp_writel(pipe->mdp, 1, MDP_LCDC_EN);
		mdp->dma_config_dirty = false;
	} else {
		mdp4_overlay_rgb_setup(pipe);
		mdp4_overlay_reg_flush(pipe, 1); /* rgb1 and mixer0 */
	}

}
#else
static void lcdc_dma_start(void *priv, uint32_t addr, uint32_t stride,
			   uint32_t width, uint32_t height, uint32_t x,
			   uint32_t y)
{
	struct mdp_lcdc_info *lcdc = priv;
	struct mdp_info *mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	if (mdp->dma_config_dirty)
	{
		mdp_writel(lcdc->mdp, 0, MDP_LCDC_EN);
		mdelay(30);
		mdp_dev->configure_dma(mdp_dev);
		mdp_writel(lcdc->mdp, 1, MDP_LCDC_EN);
	}
	mdp_writel(lcdc->mdp, stride, MDP_DMA_P_IBUF_Y_STRIDE);
	mdp_writel(lcdc->mdp, addr, MDP_DMA_P_IBUF_ADDR);
}
#endif

static void precompute_timing_parms(struct mdp_lcdc_info *lcdc)
{
	struct msm_lcdc_timing *timing = lcdc->pdata->timing;
	struct msm_fb_data *fb_data = lcdc->pdata->fb_data;
	unsigned int hsync_period;
	unsigned int hsync_start_x;
	unsigned int hsync_end_x;
	unsigned int vsync_period;
	unsigned int display_vstart;
	unsigned int display_vend;

//#if !defined(CONFIG_MACH_FLYER_SCREEN_ROTATION)
	hsync_period = (timing->hsync_pulse_width + timing->hsync_back_porch +
			fb_data->xres + timing->hsync_front_porch);
	hsync_start_x = (timing->hsync_pulse_width + timing->hsync_back_porch);
	hsync_end_x = hsync_start_x + fb_data->xres - 1;

	vsync_period = (timing->vsync_pulse_width + timing->vsync_back_porch +
			fb_data->yres + timing->vsync_front_porch);
	vsync_period *= hsync_period;

	display_vstart = timing->vsync_pulse_width + timing->vsync_back_porch;
	display_vstart *= hsync_period;
	display_vstart += timing->hsync_skew;

	display_vend = (timing->vsync_pulse_width + timing->vsync_back_porch +
			 fb_data->yres) * hsync_period;
	display_vend += timing->hsync_skew - 1;

	/* register values we pre-compute at init time from the timing
	 * information in the panel info */
	lcdc->parms.hsync_ctl = (((hsync_period & 0xfff) << 16) |
				 (timing->hsync_pulse_width & 0xfff));
	lcdc->parms.vsync_period = vsync_period & 0xffffff;
	lcdc->parms.vsync_pulse_width = (timing->vsync_pulse_width *
					 hsync_period) & 0xffffff;

	lcdc->parms.display_hctl = (((hsync_end_x & 0xfff) << 16) |
				    (hsync_start_x & 0xfff));
	lcdc->parms.display_vstart = display_vstart & 0xffffff;
	lcdc->parms.display_vend = display_vend & 0xffffff;
	lcdc->parms.hsync_skew = timing->hsync_skew & 0xfff;
	lcdc->parms.polarity = ((timing->hsync_act_low << 0) |
				(timing->vsync_act_low << 1) |
				(timing->den_act_low << 2));
	lcdc->parms.clk_rate = timing->clk_rate;
}

static int mdp_lcdc_probe(struct platform_device *pdev)
{
	struct msm_lcdc_platform_data *pdata = pdev->dev.platform_data;
	struct mdp_lcdc_info *lcdc;
	int ret = 0;
#ifdef CONFIG_MSM_MDP40
	struct mdp4_overlay_pipe *pipe;
	int ptype;
#endif

	if (!pdata) {
		PR_DISP_ERR("%s: no LCDC platform data found\n", __func__);
		return -EINVAL;
	}

	lcdc = kzalloc(sizeof(struct mdp_lcdc_info), GFP_KERNEL);
	if (!lcdc)
		return -ENOMEM;

	/* We don't actually own the clocks, the mdp does. */
	lcdc->mdp_clk = clk_get(mdp_dev->dev.parent, "mdp_clk");
	if (IS_ERR(lcdc->mdp_clk)) {
		PR_DISP_ERR("%s: failed to get mdp_clk\n", __func__);
		ret = PTR_ERR(lcdc->mdp_clk);
		goto err_get_mdp_clk;
	}

	lcdc->pclk = clk_get(mdp_dev->dev.parent, "lcdc_pclk_clk");
	if (IS_ERR(lcdc->pclk)) {
		PR_DISP_ERR("%s: failed to get lcdc_pclk\n", __func__);
		ret = PTR_ERR(lcdc->pclk);
		goto err_get_pclk;
	}

	lcdc->pad_pclk = clk_get(mdp_dev->dev.parent, "lcdc_pad_pclk_clk");
	if (IS_ERR(lcdc->pad_pclk)) {
		PR_DISP_ERR("%s: failed to get lcdc_pad_pclk\n", __func__);
		ret = PTR_ERR(lcdc->pad_pclk);
		goto err_get_pad_pclk;
	}

	init_waitqueue_head(&lcdc->vsync_waitq);
	lcdc->pdata = pdata;
	lcdc->frame_start_cb.func = lcdc_frame_start;

	platform_set_drvdata(pdev, lcdc);
#ifdef CONFIG_MSM_MDP40
	mdp_out_if_register(mdp_dev, MSM_LCDC_INTERFACE, lcdc, INTR_OVERLAY0_DONE,
			    lcdc_overlay_start);
#else
	mdp_out_if_register(mdp_dev, MSM_LCDC_INTERFACE, lcdc, MDP_DMA_P_DONE,
			    lcdc_dma_start);
#endif
	lcdc->mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	precompute_timing_parms(lcdc);

	lcdc->fb_start = pdata->fb_resource->start;
	//lcdc->mdp = container_of(mdp_dev, struct mdp_info, mdp_dev);
	if(lcdc->mdp->mdp_dev.color_format)
		lcdc->color_format = lcdc->mdp->mdp_dev.color_format;
	else
		lcdc->color_format = MSM_MDP_OUT_IF_FMT_RGB565;

#ifdef CONFIG_MSM_MDP40
	if (lcdc_pipe == NULL) {
		ptype = mdp4_overlay_format2type(MDP_RGB_565);
		pipe = mdp4_overlay_pipe_alloc(ptype, false);
		if (!pipe)
			goto err_mdp4_overlay_pipe_alloc;
		pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
		pipe->pipe_used = 1;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = MDP_RGB_565;
		mdp4_overlay_format2pipe(pipe);
		pipe->mdp = lcdc->mdp;

		lcdc_pipe = pipe; /* keep it */
	} else {
		pipe = lcdc_pipe;
	}

	pipe->src_height = pdata->fb_data->yres;
	pipe->src_width = pdata->fb_data->xres;
	pipe->src_h = pdata->fb_data->yres;
	pipe->src_w = pdata->fb_data->xres;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->srcp0_addr = (uint32_t) lcdc->fb_start;

	pipe->srcp0_ystride = pdata->fb_data->xres * 2;

	mdp4_overlay_rgb_setup(pipe);
	mdp4_mixer_stage_up(pipe);
#endif

	lcdc->fb_panel_data.suspend = lcdc_suspend;
	lcdc->fb_panel_data.resume = lcdc_resume;
	lcdc->fb_panel_data.wait_vsync = lcdc_wait_vsync;
	lcdc->fb_panel_data.request_vsync = lcdc_request_vsync;
	lcdc->fb_panel_data.clear_vsync = lcdc_clear_vsync;
	lcdc->fb_panel_data.blank = lcdc_blank;
	lcdc->fb_panel_data.unblank = lcdc_unblank;
	lcdc->fb_panel_data.fb_data = pdata->fb_data;
	lcdc->fb_panel_data.interface_type = MSM_LCDC_INTERFACE;
	lcdc->fb_panel_data.shutdown = lcdc_shutdown;
	ret = lcdc_hw_init(lcdc);
	if (ret) {
		PR_DISP_ERR("%s: Cannot initialize the mdp_lcdc\n", __func__);
		goto err_hw_init;
	}
	lcdc->fb_pdev.name = "msm_panel";
	lcdc->fb_pdev.id = pdata->fb_id;
	lcdc->fb_pdev.resource = pdata->fb_resource;
	lcdc->fb_pdev.num_resources = 1;
	lcdc->fb_pdev.dev.platform_data = &lcdc->fb_panel_data;


	ret = platform_device_register(&lcdc->fb_pdev);
	if (ret) {
		PR_DISP_ERR("%s: Cannot register msm_panel pdev\n", __func__);
		goto err_plat_dev_reg;
	}

	PR_DISP_INFO("%s: initialized\n", __func__);
#ifdef CONFIG_PANEL_SELF_REFRESH
	if (lcdc->mdp->mdp_dev.overrides & MSM_MDP_RGB_PANEL_SELE_REFRESH) {
		ret = icm_init(lcdc);
		if (ret) {
			PR_DISP_ERR("%s: Cannot init dispaly selfrefresh \n", __func__);
			goto err_plat_dev_reg;
		}
	}
#endif

	return 0;

err_plat_dev_reg:
err_hw_init:
#ifdef CONFIG_MSM_MDP40
err_mdp4_overlay_pipe_alloc:
#endif
	platform_set_drvdata(pdev, NULL);
	clk_put(lcdc->pad_pclk);
err_get_pad_pclk:
	clk_put(lcdc->pclk);
err_get_pclk:
	clk_put(lcdc->mdp_clk);
err_get_mdp_clk:
	kfree(lcdc);
	return ret;
}

static int mdp_lcdc_remove(struct platform_device *pdev)
{
	struct mdp_lcdc_info *lcdc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	clk_put(lcdc->pclk);
	clk_put(lcdc->pad_pclk);
	kfree(lcdc);

	return 0;
}

static struct platform_driver mdp_lcdc_driver = {
	.probe = mdp_lcdc_probe,
	.remove = mdp_lcdc_remove,
	.driver = {
		.name	= "msm_mdp_lcdc",
		.owner	= THIS_MODULE,
	},
};

static int mdp_lcdc_add_mdp_device(struct device *dev,
				   struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (mdp_dev)
		return 0;
	mdp_dev = container_of(dev, struct mdp_device, dev);
	return platform_driver_register(&mdp_lcdc_driver);
}

static void mdp_lcdc_remove_mdp_device(struct device *dev,
				       struct class_interface *class_intf)
{
	/* might need locking if mulitple mdp devices */
	if (dev != &mdp_dev->dev)
		return;
	platform_driver_unregister(&mdp_lcdc_driver);
	mdp_dev = NULL;
}

static struct class_interface mdp_lcdc_interface = {
	.add_dev = &mdp_lcdc_add_mdp_device,
	.remove_dev = &mdp_lcdc_remove_mdp_device,
};

static int __init mdp_lcdc_init(void)
{
	return register_mdp_client(&mdp_lcdc_interface);
}

module_init(mdp_lcdc_init);
