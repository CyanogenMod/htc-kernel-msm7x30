/*
 * linux/drivers/video/omap2/dss/dss_features.c
 *
 * Copyright (C) 2010 Texas Instruments
 * Author: Archit Taneja <archit@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <video/omapdss.h>
#include <plat/cpu.h>

#include "dss.h"
#include "dss_features.h"

/* Defines a generic omap register field */
struct dss_reg_field {
	u8 start, end;
};

struct dss_param_range {
	int min, max;
};

struct omap_dss_features {
	const struct dss_reg_field *reg_fields;
	const int num_reg_fields;

	const u32 has_feature;

	const int num_mgrs;
	const int num_ovls;
	const enum omap_display_type *supported_displays;
	const enum omap_color_mode *supported_color_modes;
	const char * const *clksrc_names;
	const struct dss_param_range *dss_params;
};

/* This struct is assigned to one of the below during initialization */
static const struct omap_dss_features *omap_current_dss_features;

static const struct dss_reg_field omap2_dss_reg_fields[] = {
	[FEAT_REG_FIRHINC]			= { 11, 0 },
	[FEAT_REG_FIRVINC]			= { 27, 16 },
	[FEAT_REG_FIFOLOWTHRESHOLD]		= { 8, 0 },
	[FEAT_REG_FIFOHIGHTHRESHOLD]		= { 24, 16 },
	[FEAT_REG_FIFOSIZE]			= { 8, 0 },
	[FEAT_REG_HORIZONTALACCU]		= { 9, 0 },
	[FEAT_REG_VERTICALACCU]			= { 25, 16 },
	[FEAT_REG_DISPC_CLK_SWITCH]		= { 0, 0 },
	[FEAT_REG_DSIPLL_REGN]			= { 0, 0 },
	[FEAT_REG_DSIPLL_REGM]			= { 0, 0 },
	[FEAT_REG_DSIPLL_REGM_DISPC]		= { 0, 0 },
	[FEAT_REG_DSIPLL_REGM_DSI]		= { 0, 0 },
};

static const struct dss_reg_field omap3_dss_reg_fields[] = {
	[FEAT_REG_FIRHINC]			= { 12, 0 },
	[FEAT_REG_FIRVINC]			= { 28, 16 },
	[FEAT_REG_FIFOLOWTHRESHOLD]		= { 11, 0 },
	[FEAT_REG_FIFOHIGHTHRESHOLD]		= { 27, 16 },
	[FEAT_REG_FIFOSIZE]			= { 10, 0 },
	[FEAT_REG_HORIZONTALACCU]		= { 9, 0 },
	[FEAT_REG_VERTICALACCU]			= { 25, 16 },
	[FEAT_REG_DISPC_CLK_SWITCH]		= { 0, 0 },
	[FEAT_REG_DSIPLL_REGN]			= { 7, 1 },
	[FEAT_REG_DSIPLL_REGM]			= { 18, 8 },
	[FEAT_REG_DSIPLL_REGM_DISPC]		= { 22, 19 },
	[FEAT_REG_DSIPLL_REGM_DSI]		= { 26, 23 },
};

static const struct dss_reg_field omap4_dss_reg_fields[] = {
	[FEAT_REG_FIRHINC]			= { 12, 0 },
	[FEAT_REG_FIRVINC]			= { 28, 16 },
	[FEAT_REG_FIFOLOWTHRESHOLD]		= { 15, 0 },
	[FEAT_REG_FIFOHIGHTHRESHOLD]		= { 31, 16 },
	[FEAT_REG_FIFOSIZE]			= { 15, 0 },
	[FEAT_REG_HORIZONTALACCU]		= { 10, 0 },
	[FEAT_REG_VERTICALACCU]			= { 26, 16 },
	[FEAT_REG_DISPC_CLK_SWITCH]		= { 9, 8 },
	[FEAT_REG_DSIPLL_REGN]			= { 8, 1 },
	[FEAT_REG_DSIPLL_REGM]			= { 20, 9 },
	[FEAT_REG_DSIPLL_REGM_DISPC]		= { 25, 21 },
	[FEAT_REG_DSIPLL_REGM_DSI]		= { 30, 26 },
};

static const enum omap_display_type omap2_dss_supported_displays[] = {
	/* OMAP_DSS_CHANNEL_LCD */
	OMAP_DISPLAY_TYPE_DPI | OMAP_DISPLAY_TYPE_DBI,

	/* OMAP_DSS_CHANNEL_DIGIT */
	OMAP_DISPLAY_TYPE_VENC,
};

static const enum omap_display_type omap3430_dss_supported_displays[] = {
	/* OMAP_DSS_CHANNEL_LCD */
	OMAP_DISPLAY_TYPE_DPI | OMAP_DISPLAY_TYPE_DBI |
	OMAP_DISPLAY_TYPE_SDI | OMAP_DISPLAY_TYPE_DSI,

	/* OMAP_DSS_CHANNEL_DIGIT */
	OMAP_DISPLAY_TYPE_VENC,
};

static const enum omap_display_type omap3630_dss_supported_displays[] = {
	/* OMAP_DSS_CHANNEL_LCD */
	OMAP_DISPLAY_TYPE_DPI | OMAP_DISPLAY_TYPE_DBI |
	OMAP_DISPLAY_TYPE_DSI,

	/* OMAP_DSS_CHANNEL_DIGIT */
	OMAP_DISPLAY_TYPE_VENC,
};

static const enum omap_display_type omap4_dss_supported_displays[] = {
	/* OMAP_DSS_CHANNEL_LCD */
	OMAP_DISPLAY_TYPE_DBI | OMAP_DISPLAY_TYPE_DSI,

	/* OMAP_DSS_CHANNEL_DIGIT */
	OMAP_DISPLAY_TYPE_VENC | OMAP_DISPLAY_TYPE_HDMI,

	/* OMAP_DSS_CHANNEL_LCD2 */
	OMAP_DISPLAY_TYPE_DPI | OMAP_DISPLAY_TYPE_DBI |
	OMAP_DISPLAY_TYPE_DSI,
};

static const enum omap_color_mode omap2_dss_supported_color_modes[] = {
	/* OMAP_DSS_GFX */
	OMAP_DSS_COLOR_CLUT1 | OMAP_DSS_COLOR_CLUT2 |
	OMAP_DSS_COLOR_CLUT4 | OMAP_DSS_COLOR_CLUT8 |
	OMAP_DSS_COLOR_RGB12U | OMAP_DSS_COLOR_RGB16 |
	OMAP_DSS_COLOR_RGB24U | OMAP_DSS_COLOR_RGB24P,

	/* OMAP_DSS_VIDEO1 */
	OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB24U |
	OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_YUV2 |
	OMAP_DSS_COLOR_UYVY,

	/* OMAP_DSS_VIDEO2 */
	OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB24U |
	OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_YUV2 |
	OMAP_DSS_COLOR_UYVY,
};

static const enum omap_color_mode omap3_dss_supported_color_modes[] = {
	/* OMAP_DSS_GFX */
	OMAP_DSS_COLOR_CLUT1 | OMAP_DSS_COLOR_CLUT2 |
	OMAP_DSS_COLOR_CLUT4 | OMAP_DSS_COLOR_CLUT8 |
	OMAP_DSS_COLOR_RGB12U | OMAP_DSS_COLOR_ARGB16 |
	OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB24U |
	OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_ARGB32 |
	OMAP_DSS_COLOR_RGBA32 | OMAP_DSS_COLOR_RGBX32,

	/* OMAP_DSS_VIDEO1 */
	OMAP_DSS_COLOR_RGB24U | OMAP_DSS_COLOR_RGB24P |
	OMAP_DSS_COLOR_RGB12U | OMAP_DSS_COLOR_RGB16 |
	OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_UYVY,

	/* OMAP_DSS_VIDEO2 */
	OMAP_DSS_COLOR_RGB12U | OMAP_DSS_COLOR_ARGB16 |
	OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB24U |
	OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_YUV2 |
	OMAP_DSS_COLOR_UYVY | OMAP_DSS_COLOR_ARGB32 |
	OMAP_DSS_COLOR_RGBA32 | OMAP_DSS_COLOR_RGBX32,
};

static const enum omap_color_mode omap4_dss_supported_color_modes[] = {
	/* OMAP_DSS_GFX */
	OMAP_DSS_COLOR_CLUT1 | OMAP_DSS_COLOR_CLUT2 |
	OMAP_DSS_COLOR_CLUT4 | OMAP_DSS_COLOR_CLUT8 |
	OMAP_DSS_COLOR_RGB12U | OMAP_DSS_COLOR_ARGB16 |
	OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB24U |
	OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_ARGB32 |
	OMAP_DSS_COLOR_RGBA32 | OMAP_DSS_COLOR_RGBX32 |
	OMAP_DSS_COLOR_ARGB16_1555,

	/* OMAP_DSS_VIDEO1 */
	OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB12U |
	OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_ARGB16_1555 |
	OMAP_DSS_COLOR_RGBA32 | OMAP_DSS_COLOR_NV12 |
	OMAP_DSS_COLOR_RGBA16 | OMAP_DSS_COLOR_RGB24U |
	OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_UYVY |
	OMAP_DSS_COLOR_ARGB16 | OMAP_DSS_COLOR_XRGB16_1555 |
	OMAP_DSS_COLOR_ARGB32 | OMAP_DSS_COLOR_RGBX16 |
	OMAP_DSS_COLOR_RGBX32,

       /* OMAP_DSS_VIDEO2 */
	OMAP_DSS_COLOR_RGB16 | OMAP_DSS_COLOR_RGB12U |
	OMAP_DSS_COLOR_YUV2 | OMAP_DSS_COLOR_ARGB16_1555 |
	OMAP_DSS_COLOR_RGBA32 | OMAP_DSS_COLOR_NV12 |
	OMAP_DSS_COLOR_RGBA16 | OMAP_DSS_COLOR_RGB24U |
	OMAP_DSS_COLOR_RGB24P | OMAP_DSS_COLOR_UYVY |
	OMAP_DSS_COLOR_ARGB16 | OMAP_DSS_COLOR_XRGB16_1555 |
	OMAP_DSS_COLOR_ARGB32 | OMAP_DSS_COLOR_RGBX16 |
	OMAP_DSS_COLOR_RGBX32,
};

static const char * const omap2_dss_clk_source_names[] = {
	[OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC]	= "N/A",
	[OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI]	= "N/A",
	[OMAP_DSS_CLK_SRC_FCK]			= "DSS_FCLK1",
};

static const char * const omap3_dss_clk_source_names[] = {
	[OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC]	= "DSI1_PLL_FCLK",
	[OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI]	= "DSI2_PLL_FCLK",
	[OMAP_DSS_CLK_SRC_FCK]			= "DSS1_ALWON_FCLK",
};

static const char * const omap4_dss_clk_source_names[] = {
	[OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC]	= "PLL1_CLK1",
	[OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI]	= "PLL1_CLK2",
	[OMAP_DSS_CLK_SRC_FCK]			= "DSS_FCLK",
	[OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC]	= "PLL2_CLK1",
	[OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DSI]	= "PLL2_CLK2",
};

static const struct dss_param_range omap2_dss_param_range[] = {
	[FEAT_PARAM_DSS_FCK]			= { 0, 173000000 },
	[FEAT_PARAM_DSIPLL_REGN]		= { 0, 0 },
	[FEAT_PARAM_DSIPLL_REGM]		= { 0, 0 },
	[FEAT_PARAM_DSIPLL_REGM_DISPC]		= { 0, 0 },
	[FEAT_PARAM_DSIPLL_REGM_DSI]		= { 0, 0 },
	[FEAT_PARAM_DSIPLL_FINT]		= { 0, 0 },
	[FEAT_PARAM_DSIPLL_LPDIV]		= { 0, 0 },
};

static const struct dss_param_range omap3_dss_param_range[] = {
	[FEAT_PARAM_DSS_FCK]			= { 0, 173000000 },
	[FEAT_PARAM_DSIPLL_REGN]		= { 0, (1 << 7) - 1 },
	[FEAT_PARAM_DSIPLL_REGM]		= { 0, (1 << 11) - 1 },
	[FEAT_PARAM_DSIPLL_REGM_DISPC]		= { 0, (1 << 4) - 1 },
	[FEAT_PARAM_DSIPLL_REGM_DSI]		= { 0, (1 << 4) - 1 },
	[FEAT_PARAM_DSIPLL_FINT]		= { 750000, 2100000 },
	[FEAT_PARAM_DSIPLL_LPDIV]		= { 1, (1 << 13) - 1},
};

static const struct dss_param_range omap4_dss_param_range[] = {
	[FEAT_PARAM_DSS_FCK]			= { 0, 186000000 },
	[FEAT_PARAM_DSIPLL_REGN]		= { 0, (1 << 8) - 1 },
	[FEAT_PARAM_DSIPLL_REGM]		= { 0, (1 << 12) - 1 },
	[FEAT_PARAM_DSIPLL_REGM_DISPC]		= { 0, (1 << 5) - 1 },
	[FEAT_PARAM_DSIPLL_REGM_DSI]		= { 0, (1 << 5) - 1 },
	[FEAT_PARAM_DSIPLL_FINT]		= { 500000, 2500000 },
	[FEAT_PARAM_DSIPLL_LPDIV]		= { 0, (1 << 13) - 1 },
};

/* OMAP2 DSS Features */
static const struct omap_dss_features omap2_dss_features = {
	.reg_fields = omap2_dss_reg_fields,
	.num_reg_fields = ARRAY_SIZE(omap2_dss_reg_fields),

	.has_feature	=
		FEAT_LCDENABLEPOL | FEAT_LCDENABLESIGNAL |
		FEAT_PCKFREEENABLE | FEAT_FUNCGATED |
		FEAT_ROWREPEATENABLE | FEAT_RESIZECONF,

	.num_mgrs = 2,
	.num_ovls = 3,
	.supported_displays = omap2_dss_supported_displays,
	.supported_color_modes = omap2_dss_supported_color_modes,
	.clksrc_names = omap2_dss_clk_source_names,
	.dss_params = omap2_dss_param_range,
};

/* OMAP3 DSS Features */
static const struct omap_dss_features omap3430_dss_features = {
	.reg_fields = omap3_dss_reg_fields,
	.num_reg_fields = ARRAY_SIZE(omap3_dss_reg_fields),

	.has_feature	=
		FEAT_GLOBAL_ALPHA | FEAT_LCDENABLEPOL |
		FEAT_LCDENABLESIGNAL | FEAT_PCKFREEENABLE |
		FEAT_FUNCGATED | FEAT_ROWREPEATENABLE |
		FEAT_LINEBUFFERSPLIT | FEAT_RESIZECONF |
		FEAT_DSI_PLL_FREQSEL | FEAT_DSI_REVERSE_TXCLKESC,

	.num_mgrs = 2,
	.num_ovls = 3,
	.supported_displays = omap3430_dss_supported_displays,
	.supported_color_modes = omap3_dss_supported_color_modes,
	.clksrc_names = omap3_dss_clk_source_names,
	.dss_params = omap3_dss_param_range,
};

static const struct omap_dss_features omap3630_dss_features = {
	.reg_fields = omap3_dss_reg_fields,
	.num_reg_fields = ARRAY_SIZE(omap3_dss_reg_fields),

	.has_feature    =
		FEAT_GLOBAL_ALPHA | FEAT_LCDENABLEPOL |
		FEAT_LCDENABLESIGNAL | FEAT_PCKFREEENABLE |
		FEAT_PRE_MULT_ALPHA | FEAT_FUNCGATED |
		FEAT_ROWREPEATENABLE | FEAT_LINEBUFFERSPLIT |
		FEAT_RESIZECONF | FEAT_DSI_PLL_PWR_BUG |
		FEAT_DSI_PLL_FREQSEL,

	.num_mgrs = 2,
	.num_ovls = 3,
	.supported_displays = omap3630_dss_supported_displays,
	.supported_color_modes = omap3_dss_supported_color_modes,
	.clksrc_names = omap3_dss_clk_source_names,
	.dss_params = omap3_dss_param_range,
};

/* OMAP4 DSS Features */
/* For OMAP4430 ES 1.0 revision */
static const struct omap_dss_features omap4430_es1_0_dss_features  = {
	.reg_fields = omap4_dss_reg_fields,
	.num_reg_fields = ARRAY_SIZE(omap4_dss_reg_fields),

	.has_feature	=
		FEAT_GLOBAL_ALPHA | FEAT_PRE_MULT_ALPHA |
		FEAT_MGR_LCD2 | FEAT_GLOBAL_ALPHA_VID1 |
		FEAT_CORE_CLK_DIV | FEAT_LCD_CLK_SRC |
		FEAT_DSI_DCS_CMD_CONFIG_VC | FEAT_DSI_VC_OCP_WIDTH |
		FEAT_DSI_GNQ | FEAT_HANDLE_UV_SEPARATE | FEAT_ATTR2,

	.num_mgrs = 3,
	.num_ovls = 3,
	.supported_displays = omap4_dss_supported_displays,
	.supported_color_modes = omap4_dss_supported_color_modes,
	.clksrc_names = omap4_dss_clk_source_names,
	.dss_params = omap4_dss_param_range,
};

/* For all the other OMAP4 versions */
static const struct omap_dss_features omap4_dss_features = {
	.reg_fields = omap4_dss_reg_fields,
	.num_reg_fields = ARRAY_SIZE(omap4_dss_reg_fields),

	.has_feature	=
		FEAT_GLOBAL_ALPHA | FEAT_PRE_MULT_ALPHA |
		FEAT_MGR_LCD2 | FEAT_GLOBAL_ALPHA_VID1 |
		FEAT_CORE_CLK_DIV | FEAT_LCD_CLK_SRC |
		FEAT_DSI_DCS_CMD_CONFIG_VC | FEAT_DSI_VC_OCP_WIDTH |
		FEAT_DSI_GNQ | FEAT_HDMI_CTS_SWMODE |
		FEAT_HANDLE_UV_SEPARATE | FEAT_ATTR2,

	.num_mgrs = 3,
	.num_ovls = 3,
	.supported_displays = omap4_dss_supported_displays,
	.supported_color_modes = omap4_dss_supported_color_modes,
	.clksrc_names = omap4_dss_clk_source_names,
	.dss_params = omap4_dss_param_range,
};

/* Functions returning values related to a DSS feature */
int dss_feat_get_num_mgrs(void)
{
	return omap_current_dss_features->num_mgrs;
}

int dss_feat_get_num_ovls(void)
{
	return omap_current_dss_features->num_ovls;
}

unsigned long dss_feat_get_param_min(enum dss_range_param param)
{
	return omap_current_dss_features->dss_params[param].min;
}

unsigned long dss_feat_get_param_max(enum dss_range_param param)
{
	return omap_current_dss_features->dss_params[param].max;
}

enum omap_display_type dss_feat_get_supported_displays(enum omap_channel channel)
{
	return omap_current_dss_features->supported_displays[channel];
}

enum omap_color_mode dss_feat_get_supported_color_modes(enum omap_plane plane)
{
	return omap_current_dss_features->supported_color_modes[plane];
}

bool dss_feat_color_mode_supported(enum omap_plane plane,
		enum omap_color_mode color_mode)
{
	return omap_current_dss_features->supported_color_modes[plane] &
			color_mode;
}

const char *dss_feat_get_clk_source_name(enum omap_dss_clk_source id)
{
	return omap_current_dss_features->clksrc_names[id];
}

/* DSS has_feature check */
bool dss_has_feature(enum dss_feat_id id)
{
	return omap_current_dss_features->has_feature & id;
}

void dss_feat_get_reg_field(enum dss_feat_reg_field id, u8 *start, u8 *end)
{
	if (id >= omap_current_dss_features->num_reg_fields)
		BUG();

	*start = omap_current_dss_features->reg_fields[id].start;
	*end = omap_current_dss_features->reg_fields[id].end;
}

void dss_features_init(void)
{
	if (cpu_is_omap24xx())
		omap_current_dss_features = &omap2_dss_features;
	else if (cpu_is_omap3630())
		omap_current_dss_features = &omap3630_dss_features;
	else if (cpu_is_omap34xx())
		omap_current_dss_features = &omap3430_dss_features;
	else if (omap_rev() == OMAP4430_REV_ES1_0)
		omap_current_dss_features = &omap4430_es1_0_dss_features;
	else if (cpu_is_omap44xx())
		omap_current_dss_features = &omap4_dss_features;
	else
		DSSWARN("Unsupported OMAP version");
}
