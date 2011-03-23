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
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <mach/clk.h>
#include <linux/android_pmem.h>
#include <linux/msm_rotator.h>
#include <linux/io.h>
#include <mach/msm_rotator_imem.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/major.h>

#define DRIVER_NAME "msm_rotator"

#define MSM_ROTATOR_BASE (msm_rotator_dev->io_base)
#define MSM_ROTATOR_INTR_ENABLE			(MSM_ROTATOR_BASE+0x0020)
#define MSM_ROTATOR_INTR_STATUS			(MSM_ROTATOR_BASE+0x0024)
#define MSM_ROTATOR_INTR_CLEAR			(MSM_ROTATOR_BASE+0x0028)
#define MSM_ROTATOR_START			(MSM_ROTATOR_BASE+0x0030)
#define MSM_ROTATOR_MAX_BURST_SIZE		(MSM_ROTATOR_BASE+0x0050)
#define MSM_ROTATOR_HW_VERSION			(MSM_ROTATOR_BASE+0x0070)
#define MSM_ROTATOR_SRC_SIZE			(MSM_ROTATOR_BASE+0x1108)
#define MSM_ROTATOR_SRCP0_ADDR			(MSM_ROTATOR_BASE+0x110c)
#define MSM_ROTATOR_SRCP1_ADDR			(MSM_ROTATOR_BASE+0x1110)
#define MSM_ROTATOR_SRC_YSTRIDE1		(MSM_ROTATOR_BASE+0x111c)
#define MSM_ROTATOR_SRC_YSTRIDE2		(MSM_ROTATOR_BASE+0x1120)
#define MSM_ROTATOR_SRC_FORMAT			(MSM_ROTATOR_BASE+0x1124)
#define MSM_ROTATOR_SRC_UNPACK_PATTERN1		(MSM_ROTATOR_BASE+0x1128)
#define MSM_ROTATOR_SUB_BLOCK_CFG		(MSM_ROTATOR_BASE+0x1138)
#define MSM_ROTATOR_OUT_PACK_PATTERN1		(MSM_ROTATOR_BASE+0x1154)
#define MSM_ROTATOR_OUTP0_ADDR			(MSM_ROTATOR_BASE+0x1168)
#define MSM_ROTATOR_OUTP1_ADDR			(MSM_ROTATOR_BASE+0x116c)
#define MSM_ROTATOR_OUT_YSTRIDE1		(MSM_ROTATOR_BASE+0x1178)
#define MSM_ROTATOR_OUT_YSTRIDE2		(MSM_ROTATOR_BASE+0x117c)
#define MSM_ROTATOR_SRC_XY			(MSM_ROTATOR_BASE+0x1200)
#define MSM_ROTATOR_SRC_IMAGE_SIZE		(MSM_ROTATOR_BASE+0x1208)

#define MSM_ROTATOR_MAX_ROT	0x07
#define MSM_ROTATOR_MAX_H	0x1fff
#define MSM_ROTATOR_MAX_W	0x1fff

/* from lsb to msb */
#define GET_PACK_PATTERN(a, x, y, z, bit) \
			(((a)<<((bit)*3))|((x)<<((bit)*2))|((y)<<(bit))|(z))
#define CLR_G 0x0
#define CLR_B 0x1
#define CLR_R 0x2
#define CLR_ALPHA 0x3

#define CLR_Y  CLR_G
#define CLR_CB CLR_B
#define CLR_CR CLR_R

#define ROTATIONS_TO_BITMASK(r) ((((r) & MDP_ROT_90) ? 1 : 0)  | \
				 (((r) & MDP_FLIP_LR) ? 2 : 0) | \
				 (((r) & MDP_FLIP_UD) ? 4 : 0))

#define IMEM_NO_OWNER -1;

#define MAX_SESSIONS 16
#define INVALID_SESSION -1

struct tile_parm {
	unsigned int width;  /* tile's width */
	unsigned int height; /* tile's height */
	unsigned int row_tile_w; /* tiles per row's width */
	unsigned int row_tile_h; /* tiles per row's height */
};

struct msm_rotator_dev {
	void __iomem *io_base;
	int irq;
	struct msm_rotator_img_info *img_info[MAX_SESSIONS];
	struct clk *pclk;
	struct clk *axi_clk;
	int rot_clk_state;
	int rot_clk_off_bysuspend;
	struct delayed_work rot_clk_work;
	struct clk *imem_clk;
	int imem_clk_state;
	int imem_clk_off_bysuspend;
	struct delayed_work imem_clk_work;
	struct platform_device *pdev;
	struct cdev cdev;
	struct device *device;
	struct class *class;
	dev_t dev_num;
	int processing;
	int last_session_idx;
	struct mutex rotator_lock;
	struct mutex imem_lock;
	int imem_owner;
	wait_queue_head_t wq;
};

#define chroma_addr(start, w, h, bpp) ((start) + ((h) * (w) * (bpp)))

#define COMPONENT_5BITS 1
#define COMPONENT_6BITS 2
#define COMPONENT_8BITS 3

static struct msm_rotator_dev *msm_rotator_dev;

enum {
	CLK_EN,
	CLK_DIS
};

static bool in_kernel = false;
int msm_rotator_imem_allocate(int requestor)
{
	int rc = 1;

#ifdef CONFIG_MSM_ROTATOR_USE_IMEM
	switch (requestor) {
	case ROTATOR_REQUEST:
		if (mutex_trylock(&msm_rotator_dev->imem_lock)) {
			msm_rotator_dev->imem_owner = ROTATOR_REQUEST;
			rc = 1;
		} else
			rc = 0;
		break;
	case JPEG_REQUEST:
		mutex_lock(&msm_rotator_dev->imem_lock);
		msm_rotator_dev->imem_owner = JPEG_REQUEST;
		rc = 1;
		break;
	default:
		rc = 0;
	}
#else
	if (requestor == JPEG_REQUEST)
		rc = 1;
#endif

	if (rc == 1) {
		/* workq instance might be running, wait for it */
		if(!cancel_delayed_work(&msm_rotator_dev->imem_clk_work))
			flush_scheduled_work();

		if (msm_rotator_dev->imem_clk_state == CLK_DIS) {
			printk(KERN_DEBUG "%s(%d) clk_enable clkstate=%d\n", __func__, __LINE__, msm_rotator_dev->imem_clk_state);
			clk_enable(msm_rotator_dev->imem_clk);
			msm_rotator_dev->imem_clk_state = CLK_EN;
		}
	}

	return rc;
}
EXPORT_SYMBOL(msm_rotator_imem_allocate);

void msm_rotator_imem_free(int requestor)
{
#ifdef CONFIG_MSM_ROTATOR_USE_IMEM
	if (msm_rotator_dev->imem_owner == requestor) {
		schedule_delayed_work(&msm_rotator_dev->imem_clk_work, HZ);
		mutex_unlock(&msm_rotator_dev->imem_lock);
	}
#else
	if (requestor == JPEG_REQUEST)
		schedule_delayed_work(&msm_rotator_dev->imem_clk_work, HZ);
#endif
}
EXPORT_SYMBOL(msm_rotator_imem_free);

#ifdef CONFIG_MSM_ROTATOR_USE_IMEM
static void msm_rotator_imem_clk_work_f(struct work_struct *work)
{
	if (mutex_trylock(&msm_rotator_dev->imem_lock)) {
		if (msm_rotator_dev->imem_clk_state == CLK_EN) {
			printk(KERN_DEBUG "%s(%d) clk_disable clkstate=%d\n", __func__, __LINE__, msm_rotator_dev->imem_clk_state);
			clk_disable(msm_rotator_dev->imem_clk);
			msm_rotator_dev->imem_clk_state = CLK_DIS;
		}
		mutex_unlock(&msm_rotator_dev->imem_lock);
	}
}
#endif

/* enable clocks needed by rotator block */
static void enable_rot_clks(void)
{
	clk_enable(msm_rotator_dev->pclk);
	clk_enable(msm_rotator_dev->axi_clk);
}

/* disable clocks needed by rotator block */
static void disable_rot_clks(void)
{
	clk_disable(msm_rotator_dev->pclk);
	clk_disable(msm_rotator_dev->axi_clk);
}

static void msm_rotator_rot_clk_work_f(struct work_struct *work)
{
	if (mutex_trylock(&msm_rotator_dev->rotator_lock)) {
		if (msm_rotator_dev->rot_clk_state == CLK_EN) {
			disable_rot_clks();
			msm_rotator_dev->rot_clk_state = CLK_DIS;
		}
		mutex_unlock(&msm_rotator_dev->rotator_lock);
	}
}

static irqreturn_t msm_rotator_isr(int irq, void *dev_id)
{
	if (msm_rotator_dev->processing) {
		msm_rotator_dev->processing = 0;
		wake_up(&msm_rotator_dev->wq);
	} else
		printk(KERN_WARNING "%s: unexpected interrupt\n", DRIVER_NAME);

	return IRQ_HANDLED;
}

static int get_bpp(int format)
{
	switch (format) {
	case MDP_RGB_565:
	case MDP_BGR_565:
		return 2;

	case MDP_XRGB_8888:
	case MDP_ARGB_8888:
	case MDP_RGBA_8888:
	case MDP_BGRA_8888:
	case MDP_RGBX_8888:
		return 4;

	case MDP_Y_CBCR_H2V2:
	case MDP_Y_CRCB_H2V2:
	case MDP_Y_CRCB_H2V2_TILE:
	case MDP_Y_CBCR_H2V2_TILE:
		return 1;

	case MDP_RGB_888:
		return 3;

	case MDP_YCRYCB_H2V1:
		return 2;/* YCrYCb interleave */

	case MDP_Y_CRCB_H2V1:
	case MDP_Y_CBCR_H2V1:
		return 1;

	default:
		return -1;
	}

}

static int msm_rotator_ycxcx_h2v1(struct msm_rotator_img_info *info,
				  unsigned int in_paddr,
				  unsigned int out_paddr,
				  unsigned int use_imem,
				  int new_session)
{
	int bpp;

	if (info->src.format != info->dst.format)
		return -EINVAL;

	bpp = get_bpp(info->src.format);
	if (bpp < 0)
		return -ENOTTY;

	iowrite32(in_paddr, MSM_ROTATOR_SRCP0_ADDR);
	iowrite32(chroma_addr(in_paddr, info->src.width, info->src.height, bpp),
		  MSM_ROTATOR_SRCP1_ADDR);
	iowrite32(out_paddr +
			((info->dst_y * info->dst.width) + info->dst_x),
		  MSM_ROTATOR_OUTP0_ADDR);
	iowrite32(chroma_addr(out_paddr, info->dst.width, info->dst.height,
			      bpp) +
			((info->dst_y * info->dst.width) + info->dst_x),
		  MSM_ROTATOR_OUTP1_ADDR);

	if (new_session) {
		iowrite32(info->src.width |
			  info->src.width << 16,
			  MSM_ROTATOR_SRC_YSTRIDE1);
		if (info->rotations & MDP_ROT_90)
			iowrite32(info->dst.width |
				  info->dst.width*2 << 16,
				  MSM_ROTATOR_OUT_YSTRIDE1);
		else
			iowrite32(info->dst.width |
				  info->dst.width << 16,
				  MSM_ROTATOR_OUT_YSTRIDE1);
		if (info->src.format == MDP_Y_CBCR_H2V1) {
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
		} else {
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
		}
		iowrite32((1  << 18) | 		/* chroma sampling 1=H2V1 */
			  (ROTATIONS_TO_BITMASK(info->rotations) << 9) |
			  1 << 8,      		/* ROT_EN */
			  MSM_ROTATOR_SUB_BLOCK_CFG);
		iowrite32(0 << 29 | 		/* frame format 0 = linear */
			  (use_imem ? 0 : 1) << 22 | /* tile size */
			  2 << 19 | 		/* fetch planes 2 = pseudo */
			  0 << 18 | 		/* unpack align */
			  1 << 17 | 		/* unpack tight */
			  1 << 13 | 		/* unpack count 0=1 component */
			  (bpp-1) << 9 |	/* src Bpp 0=1 byte ... */
			  0 << 8  | 		/* has alpha */
			  0 << 6  | 		/* alpha bits 3=8bits */
			  3 << 4  | 		/* R/Cr bits 1=5 2=6 3=8 */
			  3 << 2  | 		/* B/Cb bits 1=5 2=6 3=8 */
			  3 << 0,   		/* G/Y  bits 1=5 2=6 3=8 */
			  MSM_ROTATOR_SRC_FORMAT);
	}

	return 0;
}

static int msm_rotator_ycxcx_h2v2(struct msm_rotator_img_info *info,
				  unsigned int in_paddr,
				  unsigned int out_paddr,
				  unsigned int use_imem,
				  int new_session)
{
	int bpp;

	if (info->src.format != info->dst.format)
		return -EINVAL;

	bpp = get_bpp(info->src.format);
	if (bpp < 0)
		return -ENOTTY;

	iowrite32(in_paddr, MSM_ROTATOR_SRCP0_ADDR);
	iowrite32(chroma_addr(in_paddr, info->src.width, info->src.height, bpp),
		  MSM_ROTATOR_SRCP1_ADDR);
	iowrite32(out_paddr +
			((info->dst_y * info->dst.width) + info->dst_x),
		  MSM_ROTATOR_OUTP0_ADDR);
	iowrite32(chroma_addr(out_paddr, info->dst.width, info->dst.height,
			      bpp) +
			((info->dst_y * info->dst.width)/2 + info->dst_x),
		  MSM_ROTATOR_OUTP1_ADDR);

	if (new_session) {
		iowrite32(info->src.width |
			  info->src.width << 16,
			  MSM_ROTATOR_SRC_YSTRIDE1);
		iowrite32(info->dst.width |
			  info->dst.width << 16,
			  MSM_ROTATOR_OUT_YSTRIDE1);
		if (info->src.format == MDP_Y_CBCR_H2V2) {
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
		} else {
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
		}
		iowrite32((3  << 18) | 		/* chroma sampling 3=4:2:0 */
			  (ROTATIONS_TO_BITMASK(info->rotations) << 9) |
			  1 << 8,      		/* ROT_EN */
			  MSM_ROTATOR_SUB_BLOCK_CFG);
		iowrite32(0 << 29 | 		/* frame format 0 = linear */
			  (use_imem ? 0 : 1) << 22 | /* tile size */
			  2 << 19 | 		/* fetch planes 2 = pseudo */
			  0 << 18 | 		/* unpack align */
			  1 << 17 | 		/* unpack tight */
			  1 << 13 | 		/* unpack count 0=1 component */
			  (bpp-1) << 9  |	/* src Bpp 0=1 byte ... */
			  0 << 8  | 		/* has alpha */
			  0 << 6  | 		/* alpha bits 3=8bits */
			  3 << 4  | 		/* R/Cr bits 1=5 2=6 3=8 */
			  3 << 2  | 		/* B/Cb bits 1=5 2=6 3=8 */
			  3 << 0,   		/* G/Y  bits 1=5 2=6 3=8 */
			  MSM_ROTATOR_SRC_FORMAT);
	}
	return 0;
}

static unsigned int tile_size(unsigned int src_width,
		unsigned int src_height,
		const struct tile_parm *tp)
{
	unsigned int tile_w, tile_h;
	unsigned int row_num_w, row_num_h;
	tile_w = tp->width * tp->row_tile_w;
	tile_h = tp->height * tp->row_tile_h;
	row_num_w = (src_width + tile_w - 1) / tile_w;
	row_num_h = (src_height + tile_h - 1) / tile_h;
	return ((row_num_w * row_num_h * tile_w * tile_h) + 8191) & ~8191;
}

static int msm_rotator_ycxcx_h2v2_tile(struct msm_rotator_img_info *info,
				  unsigned int in_paddr,
				  unsigned int out_paddr,
				  unsigned int use_imem,
				  int new_session)
{
	int bpp;
	unsigned int offset = 0;
	/*
	 * each row of samsung tile consists of two tiles in height
	 * and two tiles in width which means width should align to
	 * 64 x 2 bytes and height should align to 32 x 2 bytes.
	 * video decoder generate two tiles in width and one tile
	 * in height which ends up height align to 32 X 1 bytes.
	 */
	const struct tile_parm tile = {64, 32, 2, 1};
	if ((info->src.format == MDP_Y_CRCB_H2V2_TILE &&
		info->dst.format != MDP_Y_CRCB_H2V2) ||
		(info->src.format == MDP_Y_CBCR_H2V2_TILE &&
		info->dst.format != MDP_Y_CBCR_H2V2))
		return -EINVAL;

	bpp = get_bpp(info->src.format);
	if (bpp < 0)
		return -ENOTTY;

	offset = tile_size(info->src.width, info->src.height, &tile);

	iowrite32(in_paddr, MSM_ROTATOR_SRCP0_ADDR);
	iowrite32(in_paddr + offset, MSM_ROTATOR_SRCP1_ADDR);
	iowrite32(out_paddr +
			((info->dst_y * info->dst.width) + info->dst_x),
		  MSM_ROTATOR_OUTP0_ADDR);
	iowrite32(chroma_addr(out_paddr, info->dst.width, info->dst.height,
			      bpp) +
			((info->dst_y * info->dst.width)/2 + info->dst_x),
		  MSM_ROTATOR_OUTP1_ADDR);

	if (new_session) {
		iowrite32(info->src.width |
			  info->src.width << 16,
			  MSM_ROTATOR_SRC_YSTRIDE1);

		iowrite32(info->dst.width |
			  info->dst.width << 16,
			  MSM_ROTATOR_OUT_YSTRIDE1);
		if (info->src.format == MDP_Y_CBCR_H2V2_TILE) {
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
		} else {
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
		}
		iowrite32((3  << 18) | 		/* chroma sampling 3=4:2:0 */
			  (ROTATIONS_TO_BITMASK(info->rotations) << 9) |
			  1 << 8,      		/* ROT_EN */
			  MSM_ROTATOR_SUB_BLOCK_CFG);
		iowrite32(2 << 29 | 		/* frame format 2 = supertile */
			  (use_imem ? 0 : 1) << 22 | /* tile size */
			  2 << 19 | 		/* fetch planes 2 = pseudo */
			  0 << 18 | 		/* unpack align */
			  1 << 17 | 		/* unpack tight */
			  1 << 13 | 		/* unpack count 0=1 component */
			  (bpp-1) << 9  |	/* src Bpp 0=1 byte ... */
			  0 << 8  | 		/* has alpha */
			  0 << 6  | 		/* alpha bits 3=8bits */
			  3 << 4  | 		/* R/Cr bits 1=5 2=6 3=8 */
			  3 << 2  | 		/* B/Cb bits 1=5 2=6 3=8 */
			  3 << 0,   		/* G/Y  bits 1=5 2=6 3=8 */
			  MSM_ROTATOR_SRC_FORMAT);
	}
	return 0;
}

static int msm_rotator_ycrycb(struct msm_rotator_img_info *info,
			      unsigned int in_paddr,
			      unsigned int out_paddr,
			      unsigned int use_imem,
			      int new_session)
{
	int bpp;

	if (info->src.format != info->dst.format)
		return -EINVAL;

	bpp = get_bpp(info->src.format);
	if (bpp < 0)
		return -ENOTTY;

	iowrite32(in_paddr, MSM_ROTATOR_SRCP0_ADDR);
	iowrite32(out_paddr +
			((info->dst_y * info->dst.width) + info->dst_x),
		  MSM_ROTATOR_OUTP0_ADDR);

	if (new_session) {
		iowrite32(info->src.width,
			  MSM_ROTATOR_SRC_YSTRIDE1);
		iowrite32(info->dst.width,
			  MSM_ROTATOR_OUT_YSTRIDE1);
		iowrite32(GET_PACK_PATTERN(CLR_Y, CLR_CR, CLR_Y, CLR_CB, 8),
			  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
		iowrite32(GET_PACK_PATTERN(CLR_Y, CLR_CR, CLR_Y, CLR_CB, 8),
			  MSM_ROTATOR_OUT_PACK_PATTERN1);
		iowrite32((1  << 18) | 		/* chroma sampling 1=H2V1 */
			  (ROTATIONS_TO_BITMASK(info->rotations) << 9) |
			  1 << 8,      		/* ROT_EN */
			  MSM_ROTATOR_SUB_BLOCK_CFG);
		iowrite32(0 << 29 | 		/* frame format 0 = linear */
			  (use_imem ? 0 : 1) << 22 | /* tile size */
			  0 << 19 | 		/* fetch planes 0=interleaved */
			  0 << 18 | 		/* unpack align */
			  1 << 17 | 		/* unpack tight */
			  3 << 13 | 		/* unpack count 0=1 component */
			  (bpp-1) << 9 |	/* src Bpp 0=1 byte ... */
			  0 << 8  | 		/* has alpha */
			  0 << 6  | 		/* alpha bits 3=8bits */
			  3 << 4  | 		/* R/Cr bits 1=5 2=6 3=8 */
			  3 << 2  | 		/* B/Cb bits 1=5 2=6 3=8 */
			  3 << 0,   		/* G/Y  bits 1=5 2=6 3=8 */
			  MSM_ROTATOR_SRC_FORMAT);
	}

	return 0;
}

static int msm_rotator_rgb_types(struct msm_rotator_img_info *info,
				 unsigned int in_paddr,
				 unsigned int out_paddr,
				 unsigned int use_imem,
				 int new_session)
{
	int bpp, abits, rbits, gbits, bbits;

	if (info->src.format != info->dst.format)
		return -EINVAL;

	bpp = get_bpp(info->src.format);
	if (bpp < 0)
		return -ENOTTY;

	iowrite32(in_paddr, MSM_ROTATOR_SRCP0_ADDR);
	iowrite32(out_paddr +
			((info->dst_y * info->dst.width) + info->dst_x) * bpp,
		  MSM_ROTATOR_OUTP0_ADDR);

	if (new_session) {
		iowrite32(info->src.width * bpp, MSM_ROTATOR_SRC_YSTRIDE1);
		iowrite32(info->dst.width * bpp, MSM_ROTATOR_OUT_YSTRIDE1);
		iowrite32((0  << 18) | 		/* chroma sampling 0=rgb */
			  (ROTATIONS_TO_BITMASK(info->rotations) << 9) |
			  1 << 8,      		/* ROT_EN */
			  MSM_ROTATOR_SUB_BLOCK_CFG);
		switch (info->src.format) {
		case MDP_RGB_565:
			iowrite32(GET_PACK_PATTERN(0, CLR_R, CLR_G, CLR_B, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, CLR_R, CLR_G, CLR_B, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
			abits = 0;
			rbits = COMPONENT_5BITS;
			gbits = COMPONENT_6BITS;
			bbits = COMPONENT_5BITS;
			break;

		case MDP_BGR_565:
			iowrite32(GET_PACK_PATTERN(0, CLR_B, CLR_G, CLR_R, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, CLR_B, CLR_G, CLR_R, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
			abits = 0;
			rbits = COMPONENT_5BITS;
			gbits = COMPONENT_6BITS;
			bbits = COMPONENT_5BITS;
			break;

		case MDP_RGB_888:
			iowrite32(GET_PACK_PATTERN(0, CLR_R, CLR_G, CLR_B, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(0, CLR_R, CLR_G, CLR_B, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
			abits = 0;
			rbits = COMPONENT_8BITS;
			gbits = COMPONENT_8BITS;
			bbits = COMPONENT_8BITS;
			break;

		case MDP_ARGB_8888:
		case MDP_RGBA_8888:
		case MDP_XRGB_8888:
		case MDP_RGBX_8888:
			iowrite32(GET_PACK_PATTERN(CLR_ALPHA, CLR_R, CLR_G,
						   CLR_B, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(CLR_ALPHA, CLR_R, CLR_G,
						   CLR_B, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
			abits = COMPONENT_8BITS;
			rbits = COMPONENT_8BITS;
			gbits = COMPONENT_8BITS;
			bbits = COMPONENT_8BITS;
			break;

		case MDP_BGRA_8888:
			iowrite32(GET_PACK_PATTERN(CLR_ALPHA, CLR_B, CLR_G,
						   CLR_R, 8),
				  MSM_ROTATOR_SRC_UNPACK_PATTERN1);
			iowrite32(GET_PACK_PATTERN(CLR_ALPHA, CLR_B, CLR_G,
						   CLR_R, 8),
				  MSM_ROTATOR_OUT_PACK_PATTERN1);
			abits = COMPONENT_8BITS;
			rbits = COMPONENT_8BITS;
			gbits = COMPONENT_8BITS;
			bbits = COMPONENT_8BITS;
			break;

		default:
			return -EINVAL;
		}
		iowrite32(0 << 29 | 		/* frame format 0 = linear */
			  (use_imem ? 0 : 1) << 22 | /* tile size */
			  0 << 19 | 		/* fetch planes 0=interleaved */
			  0 << 18 | 		/* unpack align */
			  1 << 17 | 		/* unpack tight */
			  (abits ? 3 : 2) << 13 | /* unpack count 0=1 comp */
			  (bpp-1) << 9 | 	/* src Bpp 0=1 byte ... */
			  (abits ? 1 : 0) << 8  | /* has alpha */
			  abits << 6  | 	/* alpha bits 3=8bits */
			  rbits << 4  | 	/* R/Cr bits 1=5 2=6 3=8 */
			  bbits << 2  | 	/* B/Cb bits 1=5 2=6 3=8 */
			  gbits << 0,   	/* G/Y  bits 1=5 2=6 3=8 */
			  MSM_ROTATOR_SRC_FORMAT);
	}

	return 0;
}

static int msm_rotator_do_rotate(unsigned long arg)
{
	int rc = 0;
	unsigned int status;
	struct msm_rotator_data_info info;
	unsigned int in_paddr, out_paddr, vaddr;
	unsigned long len;
	struct file *src_file = 0;
	struct file *dst_file = 0;
	int use_imem = 0;
	int s;

	if (copy_from_user(&info, (void __user *)arg, sizeof(info)))
		return -EFAULT;

	rc = get_pmem_file(info.src.memory_id, (unsigned long *)&in_paddr,
			   (unsigned long *)&vaddr, (unsigned long *)&len,
			   &src_file);
	if (rc) {
		printk(KERN_ERR "%s: in get_pmem_file() failed id=0x%08x\n",
		       DRIVER_NAME, info.src.memory_id);
		return rc;
	}
	in_paddr += info.src.offset;

	rc = get_pmem_file(info.dst.memory_id, (unsigned long *)&out_paddr,
			   (unsigned long *)&vaddr, (unsigned long *)&len,
			   &dst_file);
	if (rc) {
		printk(KERN_ERR "%s: out get_pmem_file() failed id=0x%08x\n",
		       DRIVER_NAME, info.dst.memory_id);
		return rc;
	}
	out_paddr += info.dst.offset;

	mutex_lock(&msm_rotator_dev->rotator_lock);
	for (s = 0; s < MAX_SESSIONS; s++)
		if ((msm_rotator_dev->img_info[s] != NULL) &&
			(info.session_id ==
			(unsigned int)msm_rotator_dev->img_info[s]
			))
			break;

	if (s == MAX_SESSIONS) {
		dev_dbg(msm_rotator_dev->device,
			"%s() : Attempt to use invalid session_id %d\n",
			__func__, s);
		rc = -EINVAL;
		goto do_rotate_unlock_mutex;
	}
	cancel_delayed_work(&msm_rotator_dev->rot_clk_work);
	if (msm_rotator_dev->rot_clk_state == CLK_DIS) {
		enable_rot_clks();
		msm_rotator_dev->rot_clk_state = CLK_EN;
	}
	enable_irq(msm_rotator_dev->irq);

#ifdef CONFIG_MSM_ROTATOR_USE_IMEM
	use_imem = msm_rotator_imem_allocate(ROTATOR_REQUEST);
#else
	use_imem = 0;
#endif
	/*
	 * workaround for a hardware bug. rotator hardware hangs when we
	 * use write burst beat size 16 on 128X128 tile fetch mode. As a
	 * temporary fix use 0x42 for BURST_SIZE when imem used.
	 */
	if (use_imem)
		iowrite32(0x42, MSM_ROTATOR_MAX_BURST_SIZE);

	iowrite32(((msm_rotator_dev->img_info[s]->src_rect.h & 0x1fff)
				<< 16) |
		  (msm_rotator_dev->img_info[s]->src_rect.w & 0x1fff),
		  MSM_ROTATOR_SRC_SIZE);
	iowrite32(((msm_rotator_dev->img_info[s]->src_rect.y & 0x1fff)
				<< 16) |
		  (msm_rotator_dev->img_info[s]->src_rect.x & 0x1fff),
		  MSM_ROTATOR_SRC_XY);
	iowrite32(((msm_rotator_dev->img_info[s]->src.height & 0x1fff)
				<< 16) |
		  (msm_rotator_dev->img_info[s]->src.width & 0x1fff),
		  MSM_ROTATOR_SRC_IMAGE_SIZE);

	switch (msm_rotator_dev->img_info[s]->src.format) {
	case MDP_RGB_565:
	case MDP_BGR_565:
	case MDP_RGB_888:
	case MDP_ARGB_8888:
	case MDP_RGBA_8888:
	case MDP_XRGB_8888:
	case MDP_BGRA_8888:
	case MDP_RGBX_8888:
		rc = msm_rotator_rgb_types(msm_rotator_dev->img_info[s],
					   in_paddr, out_paddr,
					   use_imem,
					   msm_rotator_dev->last_session_idx
								!= s);
		break;
	case MDP_Y_CBCR_H2V2:
	case MDP_Y_CRCB_H2V2:
		rc = msm_rotator_ycxcx_h2v2(msm_rotator_dev->img_info[s],
					    in_paddr, out_paddr, use_imem,
					    msm_rotator_dev->last_session_idx
								!= s);
		break;
	case MDP_Y_CRCB_H2V2_TILE:
	case MDP_Y_CBCR_H2V2_TILE:
		rc = msm_rotator_ycxcx_h2v2_tile(msm_rotator_dev->img_info[s],
				in_paddr, out_paddr, use_imem,
				msm_rotator_dev->last_session_idx
				!= s);
	break;

	case MDP_Y_CBCR_H2V1:
	case MDP_Y_CRCB_H2V1:
		rc = msm_rotator_ycxcx_h2v1(msm_rotator_dev->img_info[s],
					    in_paddr, out_paddr, use_imem,
					    msm_rotator_dev->last_session_idx
								!= s);
		break;
	case MDP_YCRYCB_H2V1:
		rc = msm_rotator_ycrycb(msm_rotator_dev->img_info[s],
				in_paddr, out_paddr, use_imem,
				msm_rotator_dev->last_session_idx != s);
		break;
	default:
		rc = -EINVAL;
		goto do_rotate_exit;
	}

	if (rc != 0) {
		msm_rotator_dev->last_session_idx = INVALID_SESSION;
		goto do_rotate_exit;
	}

	iowrite32(3, MSM_ROTATOR_INTR_ENABLE);

	msm_rotator_dev->processing = 1;
	iowrite32(0x1, MSM_ROTATOR_START);

	wait_event(msm_rotator_dev->wq,
		   (msm_rotator_dev->processing == 0));
	status = (unsigned char)ioread32(MSM_ROTATOR_INTR_STATUS);
	if ((status & 0x03) != 0x01)
		rc = -EFAULT;
	iowrite32(0, MSM_ROTATOR_INTR_ENABLE);
	iowrite32(3, MSM_ROTATOR_INTR_CLEAR);

do_rotate_exit:
	disable_irq(msm_rotator_dev->irq);
	//If imem_allocated is called successfully, then call imem_free to prevent unlock imem_lock twice.
	if(use_imem)
		msm_rotator_imem_free(ROTATOR_REQUEST);
	schedule_delayed_work(&msm_rotator_dev->rot_clk_work, HZ);
do_rotate_unlock_mutex:
	if (src_file)
		put_pmem_file(src_file);
	if (dst_file)
		put_pmem_file(dst_file);
	mutex_unlock(&msm_rotator_dev->rotator_lock);
	dev_dbg(msm_rotator_dev->device, "%s() returning rc = %d\n",
		__func__, rc);
	return rc;
}

static int msm_rotator_start(unsigned long arg)
{
	struct msm_rotator_img_info info;
	int rc = 0;
	int s;
	int first_free_index = INVALID_SESSION;

	if (copy_from_user(&info, (void __user *)arg, sizeof(info)))
		return -EFAULT;

	if ((info.rotations > MSM_ROTATOR_MAX_ROT) ||
	    (info.src.height > MSM_ROTATOR_MAX_H) ||
	    (info.src.width > MSM_ROTATOR_MAX_W) ||
	    (info.dst.height > MSM_ROTATOR_MAX_H) ||
	    (info.dst.width > MSM_ROTATOR_MAX_W) ||
	    ((info.src_rect.x + info.src_rect.w) > info.src.width) ||
	    ((info.src_rect.y + info.src_rect.h) > info.src.height) ||
	    ((info.rotations & MDP_ROT_90) &&
		((info.dst_x + info.src_rect.h) > info.dst.width)) ||
	    ((info.rotations & MDP_ROT_90) &&
		((info.dst_y + info.src_rect.w) > info.dst.height)) ||
	    (!(info.rotations & MDP_ROT_90) &&
		((info.dst_x + info.src_rect.w) > info.dst.width)) ||
	    (!(info.rotations & MDP_ROT_90) &&
		((info.dst_y + info.src_rect.h) > info.dst.height)))
		return -EINVAL;

	switch (info.src.format) {
	case MDP_RGB_565:
	case MDP_BGR_565:
	case MDP_RGB_888:
	case MDP_ARGB_8888:
	case MDP_RGBA_8888:
	case MDP_XRGB_8888:
	case MDP_RGBX_8888:
	case MDP_BGRA_8888:
	case MDP_Y_CBCR_H2V2:
	case MDP_Y_CRCB_H2V2:
	case MDP_Y_CBCR_H2V1:
	case MDP_Y_CRCB_H2V1:
	case MDP_YCRYCB_H2V1:
	case MDP_Y_CRCB_H2V2_TILE:
	case MDP_Y_CBCR_H2V2_TILE:
		break;
	default:
		return -EINVAL;
	}

	switch (info.dst.format) {
	case MDP_RGB_565:
	case MDP_BGR_565:
	case MDP_RGB_888:
	case MDP_ARGB_8888:
	case MDP_RGBA_8888:
	case MDP_XRGB_8888:
	case MDP_BGRA_8888:
	case MDP_Y_CBCR_H2V2:
	case MDP_Y_CRCB_H2V2:
	case MDP_Y_CBCR_H2V1:
	case MDP_Y_CRCB_H2V1:
	case MDP_YCRYCB_H2V1:
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&msm_rotator_dev->rotator_lock);
	for (s = 0; s < MAX_SESSIONS; s++) {
		if ((msm_rotator_dev->img_info[s] != NULL) &&
			(info.session_id ==
			(unsigned int)msm_rotator_dev->img_info[s]
			)) {
			*(msm_rotator_dev->img_info[s]) = info;

			if (msm_rotator_dev->last_session_idx == s)
				msm_rotator_dev->last_session_idx =
				INVALID_SESSION;
			break;
		}

		if ((msm_rotator_dev->img_info[s] == NULL) &&
			(first_free_index ==
			INVALID_SESSION))
			first_free_index = s;
	}

	if ((s == MAX_SESSIONS) && (first_free_index != INVALID_SESSION)) {
		/* allocate a session id */
		msm_rotator_dev->img_info[first_free_index] =
			kzalloc(sizeof(struct msm_rotator_img_info),
					GFP_KERNEL);
		if (!msm_rotator_dev->img_info[first_free_index]) {
			printk(KERN_ERR "%s : unable to alloc mem\n",
					__func__);
			rc = -ENOMEM;
			goto rotator_start_exit;
		}
		info.session_id = (unsigned int)
			msm_rotator_dev->img_info[first_free_index];
		*(msm_rotator_dev->img_info[first_free_index]) = info;

	if(!in_kernel) {
		if (copy_to_user((void __user *)arg, &info, sizeof(info)))
			rc = -EFAULT;
	} else
		memcpy((void*)arg, &info, sizeof(info));
	} else if (s == MAX_SESSIONS) {
		dev_dbg(msm_rotator_dev->device, "%s: all sessions in use\n",
			__func__);
		rc = -EBUSY;
	}

rotator_start_exit:
	mutex_unlock(&msm_rotator_dev->rotator_lock);

	return rc;
}

static int msm_rotator_finish(unsigned long arg)
{
	int rc = 0;
	int s;
	unsigned int session_id;

	if (copy_from_user(&session_id, (void __user *)arg, sizeof(s)))
		return -EFAULT;

	mutex_lock(&msm_rotator_dev->rotator_lock);
	for (s = 0; s < MAX_SESSIONS; s++) {
		if ((msm_rotator_dev->img_info[s] != NULL) &&
			(session_id ==
			(unsigned int)msm_rotator_dev->img_info[s])) {
			if (msm_rotator_dev->last_session_idx == s)
				msm_rotator_dev->last_session_idx =
					INVALID_SESSION;
			kfree(msm_rotator_dev->img_info[s]);
			msm_rotator_dev->img_info[s] = NULL;
			break;
		}
	}

	if (s == MAX_SESSIONS)
		rc = -EINVAL;
	mutex_unlock(&msm_rotator_dev->rotator_lock);
	return rc;
}

static int msm_rotator_ioctl(struct inode *inode, struct file *file,
			     unsigned cmd, unsigned long arg)
{
	if (_IOC_TYPE(cmd) != MSM_ROTATOR_IOCTL_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case MSM_ROTATOR_IOCTL_START:
		return msm_rotator_start(arg);
	case MSM_ROTATOR_IOCTL_ROTATE:
		return msm_rotator_do_rotate(arg);
	case MSM_ROTATOR_IOCTL_FINISH:
		return msm_rotator_finish(arg);

	default:
		dev_dbg(msm_rotator_dev->device,
			"unexpected IOCTL %d\n", cmd);
		return -ENOTTY;
	}
}

static const struct file_operations msm_rotator_fops = {
	.owner = THIS_MODULE,
	.ioctl = msm_rotator_ioctl,
};

static int __devinit msm_rotator_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct resource *res;
	struct msm_rotator_platform_data *pdata = NULL;
	int i, number_of_clks;
	uint32_t ver;

	msm_rotator_dev = kzalloc(sizeof(struct msm_rotator_dev), GFP_KERNEL);
	if (!msm_rotator_dev) {
		printk(KERN_ERR "%s Unable to allocate memory for struct\n",
		       __func__);
		return -ENOMEM;
	}
	for (i = 0; i < MAX_SESSIONS; i++)
		msm_rotator_dev->img_info[i] = NULL;
	msm_rotator_dev->last_session_idx = INVALID_SESSION;

	pdata = pdev->dev.platform_data;
	number_of_clks = pdata->number_of_clocks;

	msm_rotator_dev->imem_owner = IMEM_NO_OWNER;
	mutex_init(&msm_rotator_dev->imem_lock);

	msm_rotator_dev->rot_clk_off_bysuspend = 0;
	msm_rotator_dev->imem_clk_off_bysuspend = 0;

	msm_rotator_dev->imem_clk_state = CLK_DIS;
	INIT_DELAYED_WORK(&msm_rotator_dev->imem_clk_work,
			  msm_rotator_imem_clk_work_f);
	msm_rotator_dev->imem_clk = NULL;
	msm_rotator_dev->pdev = pdev;
	for (i = 0; i < number_of_clks; i++) {
		if (pdata->rotator_clks[i].clk_type == ROTATOR_IMEMCLK_CLK) {
			msm_rotator_dev->imem_clk =
			clk_get(&msm_rotator_dev->pdev->dev,
				pdata->rotator_clks[i].clk_name);
			if (IS_ERR(msm_rotator_dev->imem_clk)) {
				rc = PTR_ERR(msm_rotator_dev->imem_clk);
				msm_rotator_dev->imem_clk = NULL;
				printk(KERN_ERR "%s: cannot get imem_clk "
					"rc=%d\n", DRIVER_NAME, rc);
				goto error_imem_clk;
			}
			if (pdata->rotator_clks[i].clk_rate)
				clk_set_min_rate(msm_rotator_dev->imem_clk,
					pdata->rotator_clks[i].clk_rate);
		}
		if (pdata->rotator_clks[i].clk_type == ROTATOR_PCLK_CLK) {
			msm_rotator_dev->pclk =
			clk_get(&msm_rotator_dev->pdev->dev,
				pdata->rotator_clks[i].clk_name);
			if (IS_ERR(msm_rotator_dev->pclk)) {
				rc = PTR_ERR(msm_rotator_dev->pclk);
				msm_rotator_dev->pclk = NULL;
				printk(KERN_ERR "%s: cannot get pclk rc=%d\n",
					DRIVER_NAME, rc);
				goto error_pclk;
			}

			if (pdata->rotator_clks[i].clk_rate)
				clk_set_min_rate(msm_rotator_dev->pclk,
					pdata->rotator_clks[i].clk_rate);
		}

		if (pdata->rotator_clks[i].clk_type == ROTATOR_AXICLK_CLK) {
			msm_rotator_dev->axi_clk =
			clk_get(&msm_rotator_dev->pdev->dev,
				pdata->rotator_clks[i].clk_name);
			if (IS_ERR(msm_rotator_dev->axi_clk)) {
				rc = PTR_ERR(msm_rotator_dev->axi_clk);
				msm_rotator_dev->axi_clk = NULL;
				printk(KERN_ERR "%s: cannot get axi clk "
					"rc=%d\n", DRIVER_NAME, rc);
			goto error_axi_clk;
			}

			if (pdata->rotator_clks[i].clk_rate)
				clk_set_min_rate(msm_rotator_dev->axi_clk,
					pdata->rotator_clks[i].clk_rate);
		}
	}
	msm_rotator_dev->rot_clk_state = CLK_DIS;
	INIT_DELAYED_WORK(&msm_rotator_dev->rot_clk_work,
			  msm_rotator_rot_clk_work_f);

	mutex_init(&msm_rotator_dev->rotator_lock);

	platform_set_drvdata(pdev, msm_rotator_dev);


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk(KERN_ALERT
		       "%s: could not get IORESOURCE_MEM\n", DRIVER_NAME);
		rc = -ENODEV;
		goto error_get_resource;
	}
	msm_rotator_dev->io_base = ioremap(res->start,
					   resource_size(res));

#ifdef CONFIG_MSM_ROTATOR_USE_IMEM
	if (msm_rotator_dev->imem_clk)
		clk_enable(msm_rotator_dev->imem_clk);
#endif
	enable_rot_clks();
	ver = ioread32(MSM_ROTATOR_HW_VERSION);
	disable_rot_clks();

#ifdef CONFIG_MSM_ROTATOR_USE_IMEM
	if (msm_rotator_dev->imem_clk)
		clk_disable(msm_rotator_dev->imem_clk);
#endif
	if (ver != pdata->hardware_version_number) {
		printk(KERN_ALERT "%s: invalid HW version\n", DRIVER_NAME);
		rc = -ENODEV;
		goto error_get_resource;
	}
	msm_rotator_dev->irq = platform_get_irq(pdev, 0);
	if (msm_rotator_dev->irq < 0) {
		printk(KERN_ALERT "%s: could not get IORESOURCE_IRQ\n",
		       DRIVER_NAME);
		rc = -ENODEV;
		goto error_get_irq;
	}
	rc = request_irq(msm_rotator_dev->irq, msm_rotator_isr,
			 IRQF_TRIGGER_RISING, DRIVER_NAME, NULL);
	if (rc) {
		printk(KERN_ERR "%s: request_irq() failed\n", DRIVER_NAME);
		goto error_get_irq;
	}
	/* we enable the IRQ when we need it in the ioctl */
	disable_irq(msm_rotator_dev->irq);

	rc = alloc_chrdev_region(&msm_rotator_dev->dev_num, 0, 1, DRIVER_NAME);
	if (rc < 0) {
		printk(KERN_ERR "%s: alloc_chrdev_region Failed rc = %d\n",
		       __func__, rc);
		goto error_get_irq;
	}

	msm_rotator_dev->class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(msm_rotator_dev->class)) {
		rc = PTR_ERR(msm_rotator_dev->class);
		printk(KERN_ERR "%s: couldn't create class rc = %d\n",
		       DRIVER_NAME, rc);
		goto error_class_create;
	}

	msm_rotator_dev->device = device_create(msm_rotator_dev->class, NULL,
						msm_rotator_dev->dev_num, NULL,
						DRIVER_NAME);
	if (IS_ERR(msm_rotator_dev->device)) {
		rc = PTR_ERR(msm_rotator_dev->device);
		printk(KERN_ERR "%s: device_create failed %d\n",
		       DRIVER_NAME, rc);
		goto error_class_device_create;
	}

	cdev_init(&msm_rotator_dev->cdev, &msm_rotator_fops);
	rc = cdev_add(&msm_rotator_dev->cdev,
		      MKDEV(MAJOR(msm_rotator_dev->dev_num), 0),
		      1);
	if (rc < 0) {
		printk(KERN_ERR "%s: cdev_add failed %d\n", __func__, rc);
		goto error_cdev_add;
	}

	init_waitqueue_head(&msm_rotator_dev->wq);

	dev_dbg(msm_rotator_dev->device, "probe successful\n");
	return rc;

error_cdev_add:
	device_destroy(msm_rotator_dev->class, msm_rotator_dev->dev_num);
error_class_device_create:
	class_destroy(msm_rotator_dev->class);
error_class_create:
	unregister_chrdev_region(msm_rotator_dev->dev_num, 1);
error_get_irq:
	iounmap(msm_rotator_dev->io_base);
error_get_resource:
	mutex_destroy(&msm_rotator_dev->rotator_lock);
	clk_put(msm_rotator_dev->axi_clk);
error_axi_clk:
	clk_put(msm_rotator_dev->pclk);
error_pclk:
	if (msm_rotator_dev->imem_clk)
		clk_put(msm_rotator_dev->imem_clk);
error_imem_clk:
	mutex_destroy(&msm_rotator_dev->imem_lock);
	kfree(msm_rotator_dev);

	return rc;
}

static int __devexit msm_rotator_remove(struct platform_device *plat_dev)
{
	int i;

	free_irq(msm_rotator_dev->irq, NULL);
	mutex_destroy(&msm_rotator_dev->rotator_lock);
	cdev_del(&msm_rotator_dev->cdev);
	device_destroy(msm_rotator_dev->class, msm_rotator_dev->dev_num);
	class_destroy(msm_rotator_dev->class);
	unregister_chrdev_region(msm_rotator_dev->dev_num, 1);
	iounmap(msm_rotator_dev->io_base);
	if (msm_rotator_dev->imem_clk) {
		if (msm_rotator_dev->imem_clk_state == CLK_EN)
			clk_disable(msm_rotator_dev->imem_clk);
		clk_put(msm_rotator_dev->imem_clk);
		msm_rotator_dev->imem_clk = NULL;
	}
	if (msm_rotator_dev->rot_clk_state == CLK_EN)
		disable_rot_clks();
	clk_put(msm_rotator_dev->pclk);
	clk_put(msm_rotator_dev->axi_clk);
	msm_rotator_dev->pclk = NULL;
	msm_rotator_dev->axi_clk = NULL;
	mutex_destroy(&msm_rotator_dev->imem_lock);
	for (i = 0; i < MAX_SESSIONS; i++)
		if (msm_rotator_dev->img_info[i] != NULL)
			kfree(msm_rotator_dev->img_info[i]);
	kfree(msm_rotator_dev);
	return 0;
}

#ifdef CONFIG_PM
static int msm_rotator_suspend(struct platform_device *dev, pm_message_t state)
{
	mutex_lock(&msm_rotator_dev->imem_lock);
	if (msm_rotator_dev->imem_clk_state == CLK_EN) {
		printk(KERN_DEBUG "%s(%d) clk_disable clkstate %d", __func__, __LINE__, msm_rotator_dev->imem_clk_state);
		msm_rotator_dev->imem_clk_state = CLK_DIS;
		clk_disable(msm_rotator_dev->imem_clk);
		msm_rotator_dev->imem_clk_off_bysuspend = 1;
	}
	mutex_unlock(&msm_rotator_dev->imem_lock);

	mutex_lock(&msm_rotator_dev->rotator_lock);
	if (msm_rotator_dev->rot_clk_state == CLK_EN) {
		msm_rotator_dev->rot_clk_state = CLK_DIS;
		disable_rot_clks();
		msm_rotator_dev->rot_clk_off_bysuspend = 1;
	}
	mutex_unlock(&msm_rotator_dev->rotator_lock);
	return 0;
}

static int msm_rotator_resume(struct platform_device *dev)
{
	mutex_lock(&msm_rotator_dev->imem_lock);
	if(msm_rotator_dev->imem_clk_off_bysuspend) {
		if (msm_rotator_dev->imem_clk_state == CLK_DIS) {
			printk(KERN_DEBUG "%s(%d) clk_enable clkstate %d", __func__, __LINE__, msm_rotator_dev->imem_clk_state);
			msm_rotator_dev->imem_clk_state = CLK_EN;
			clk_enable(msm_rotator_dev->imem_clk);
		}
		msm_rotator_dev->imem_clk_off_bysuspend = 0;
	}
	mutex_unlock(&msm_rotator_dev->imem_lock);

	mutex_lock(&msm_rotator_dev->rotator_lock);
	if(msm_rotator_dev->rot_clk_off_bysuspend) {
		if (msm_rotator_dev->rot_clk_state == CLK_DIS) {
			msm_rotator_dev->rot_clk_state = CLK_EN;
			enable_rot_clks();
		}
		msm_rotator_dev->rot_clk_off_bysuspend = 0;
	}
	mutex_unlock(&msm_rotator_dev->rotator_lock);

	return 0;
}
#endif

static struct platform_driver msm_rotator_platform_driver = {
	.probe = msm_rotator_probe,
	.remove = __devexit_p(msm_rotator_remove),
#ifdef CONFIG_PM
	.suspend = msm_rotator_suspend,
	.resume = msm_rotator_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME
	}
};

static int __init msm_rotator_init(void)
{
	return platform_driver_register(&msm_rotator_platform_driver);
}

static void __exit msm_rotator_exit(void)
{
	return platform_driver_unregister(&msm_rotator_platform_driver);
}

module_init(msm_rotator_init);
module_exit(msm_rotator_exit);

MODULE_DESCRIPTION("MSM Offline Image Rotator driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
