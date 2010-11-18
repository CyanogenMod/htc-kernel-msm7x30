/* include/linux/msm_mdp.h
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
#ifndef _MSM_MDP_H_
#define _MSM_MDP_H_

#include <linux/types.h>

#define MSMFB_IOCTL_MAGIC 'm'
#define MSMFB_GRP_DISP          _IOW(MSMFB_IOCTL_MAGIC, 1, unsigned int)
#define MSMFB_BLIT              _IOW(MSMFB_IOCTL_MAGIC, 2, unsigned int)
#ifdef CONFIG_MSM_MDP40
#define MSMFB_OVERLAY_SET       _IOWR(MSMFB_IOCTL_MAGIC, 135, \
						struct mdp_overlay)
#define MSMFB_OVERLAY_UNSET     _IOW(MSMFB_IOCTL_MAGIC, 136, unsigned int)
#define MSMFB_OVERLAY_PLAY      _IOW(MSMFB_IOCTL_MAGIC, 137, \
						struct msmfb_overlay_data)
#define MSMFB_OVERLAY_GET      _IOR(MSMFB_IOCTL_MAGIC, 140, \
						struct mdp_overlay)
#endif

enum {
	MDP_RGB_565,      // RGB 565 planer
	MDP_XRGB_8888,    // RGB 888 padded
	MDP_Y_CBCR_H2V2,  // Y and CbCr, pseudo planer w/ Cb is in MSB
	MDP_ARGB_8888,    // ARGB 888
	MDP_RGB_888,      // RGB 888 planer
	MDP_Y_CRCB_H2V2,  // Y and CrCb, pseudo planer w/ Cr is in MSB
	MDP_YCRYCB_H2V1,  // YCrYCb interleave
	MDP_Y_CRCB_H2V1,  // Y and CrCb, pseduo planer w/ Cr is in MSB
	MDP_Y_CBCR_H2V1,   // Y and CrCb, pseduo planer w/ Cr is in MSB
	MDP_RGBA_8888,    // ARGB 888
	MDP_BGRA_8888,    // ARGB 888
	MDP_RGBX_8888,    // RGBX 888
#ifdef CONFIG_MSM_MDP40
	MDP_Y_CRCB_H2V2_TILE,  /* Y and CrCb, pseudo planer tile */
	MDP_Y_CBCR_H2V2_TILE,  /* Y and CbCr, pseudo planer tile */
#endif
	MDP_IMGTYPE_LIMIT, // Non valid image type after this enum
	MDP_IMGTYPE2_START = 0x10000,
	MDP_BGR_565 = MDP_IMGTYPE2_START,      /* BGR 565 planer */
	MDP_FB_FORMAT,    /* framebuffer format */
	MDP_IMGTYPE_LIMIT2 /* Non valid image type after this enum */
};

enum {
	PMEM_IMG,
	FB_IMG,
};

/* flag values */
#define MDP_ROT_NOP 0
#define MDP_FLIP_LR 0x1
#define MDP_FLIP_UD 0x2
#define MDP_ROT_90 0x4
#define MDP_ROT_180 (MDP_FLIP_UD|MDP_FLIP_LR)
#define MDP_ROT_270 (MDP_ROT_90|MDP_FLIP_UD|MDP_FLIP_LR)
#define MDP_ROT_MASK 0x7
#define MDP_DITHER 0x8
#define MDP_BLUR 0x10
#define MDP_BLEND_FG_PREMULT 0x20000
#define MDP_DEINTERLACE 0x80000000
#define MDP_SHARPENING  0x40000000
#define MDP_TRANSP_NOP 0xffffffff
#define MDP_ALPHA_NOP 0xff

struct mdp_rect {
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
};

struct mdp_img {
	uint32_t width;
	uint32_t height;
	uint32_t format;
	uint32_t offset;
	int memory_id;		/* the file descriptor */
};

struct mdp_blit_req {
	struct mdp_img src;
	struct mdp_img dst;
	struct mdp_rect src_rect;
	struct mdp_rect dst_rect;
	uint32_t alpha;
	uint32_t transp_mask;
	uint32_t flags;
	int sharpening_strength;  /* -127 <--> 127, default 64 */
};

struct mdp_blit_req_list {
	uint32_t count;
	struct mdp_blit_req req[];
};

#ifdef CONFIG_MSM_MDP40
struct msmfb_data {
	uint32_t offset;
	int memory_id;
	int id;
};

#define MSMFB_NEW_REQUEST -1

struct msmfb_overlay_data {
	uint32_t id;
	struct msmfb_data data;
};

struct msmfb_img {
	uint32_t width;
	uint32_t height;
	uint32_t format;
};

struct mdp_overlay {
	struct msmfb_img src;
	struct mdp_rect src_rect;
	struct mdp_rect dst_rect;
	uint32_t z_order;	/* stage number */
	uint32_t is_fg;		/* control alpha & transp */
	uint32_t alpha;
	uint32_t transp_mask;
	uint32_t flags;
	uint32_t id;
	uint32_t user_data[8];
};
#endif
#endif //_MSM_MDP_H_
