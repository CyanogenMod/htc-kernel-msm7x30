/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>

#include <mach/msm_iomap.h>
#include <mach/clk.h>
#include <mach/internal_power_rail.h>

#include "clock.h"
#include "clock-7x30.h"
#include "proc_comm.h"

enum {
	NOMINAL,
	HIGH,
	MSMC1_END
};

struct clk_freq_tbl {
	uint32_t	freq_hz;
	uint32_t	src;
	uint32_t	md_val;
	uint32_t	ns_val;
	uint32_t	mode;
	unsigned	msmc1;
};

struct clk_local {
	uint32_t	count;
	uint32_t	type;
	uint32_t	md_reg;
	uint32_t	ns_reg;
	uint32_t	freq_mask;
	uint32_t	br_en_mask;
	uint32_t	root_en_mask;
	int		parent;
	uint32_t	*children;
	struct clk_freq_tbl	*freq_tbl;
	struct clk_freq_tbl	*current_freq;
};


enum {
	SRC_PLL0 = 4, /* Modem PLL */
	SRC_PLL1 = 1, /* Global PLL */
	SRC_PLL3 = 3, /* Multimedia/Peripheral PLL or Backup PLL1 */
	SRC_PLL4 = 2, /* Display PLL */
	SRC_LPXO = 6, /* Low power XO. */
	SRC_MAX       /* Used for sources that can't be turned on/off. */
};

static uint32_t src_pll_tbl[] = {
	[SRC_PLL0] = PLL_0,
	[SRC_PLL1] = PLL_1,
	[SRC_PLL3] = PLL_3,
	[SRC_PLL4] = PLL_4,
};

#define B(x)	BIT(x)
#define BM(msb, lsb)	(((((uint32_t)-1) << (31-msb)) >> (31-msb+lsb)) << lsb)
#define BVAL(msb, lsb, val)	(((val) << lsb) & BM(msb, lsb))

#define MD8(m, n)		(BVAL(15, 8, m) | BVAL(7, 0, ~(n)))
#define N8(msb, lsb, m, n)	(BVAL(msb, lsb, ~(n-m)))
#define MD16(m, n)		(BVAL(31, 16, m) | BVAL(15, 0, ~(n)))
#define N16(m, n)		(BVAL(31, 16, ~(n-m)))
#define SPDIV(s, d)		(BVAL(4, 3, d-1) | BVAL(2, 0, s))
#define SDIV(s, d)		(BVAL(6, 3, d-1) | BVAL(2, 0, s))
#define F_MASK_BASIC		(BM(6, 3)|BM(2, 0))
#define F_MASK_MND16		(BM(31, 16)|BM(4, 3)|BM(2, 0))
#define F_MASK_MND8(m, l)	(BM(m, l)|BM(4, 3)|BM(2, 0))

#define F_RAW(f, s, m_v, n_v, mde, v) { \
	.freq_hz = f, \
	.src = s, \
	.md_val = m_v, \
	.ns_val = n_v, \
	.mode = mde, \
	.msmc1 = v \
	}

#define FREQ_END	0
#define F_BASIC(f, s, div, v) F_RAW(f, s, 0, SDIV(s, div), 0, v)
#define F_MND16(f, s, div, m, n, v) \
	F_RAW(f, s, MD16(m, n), N16(m, n)|SPDIV(s, div), !!(n), v)
#define F_MND8(f, nmsb, nlsb, s, div, m, n, v) \
	F_RAW(f, s, MD8(m, n), N8(nmsb, nlsb, m, n)|SPDIV(s, div), !!(n), v)
#define F_END	F_RAW(FREQ_END, SRC_MAX, 0, 0, 0, MSMC1_END)

static struct clk_freq_tbl clk_tbl_csi[] = {
	F_MND8(153600000, 24, 17, SRC_PLL1, 2, 2, 5, NOMINAL),
	F_MND8(192000000, 24, 17, SRC_PLL1, 4, 0, 0, NOMINAL),
	F_MND8(384000000, 24, 17, SRC_PLL1, 2, 0, 0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_tcxo[] = {
	F_RAW(19200000, SRC_MAX, 0, 0, 0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_uartdm[] = {
	F_MND16( 3686400, SRC_PLL3, 3,   3, 200, NOMINAL),
	F_MND16( 7372800, SRC_PLL3, 3,   3, 100, NOMINAL),
	F_MND16(14745600, SRC_PLL3, 3,   3,  50, NOMINAL),
	F_MND16(46400000, SRC_PLL3, 3, 145, 768, NOMINAL),
	F_MND16(51200000, SRC_PLL3, 3,   5,  24, NOMINAL),
	F_MND16(58982400, SRC_PLL3, 3,   6,  25, NOMINAL),
	F_MND16(64000000, SRC_PLL1, 4,   1,   3, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdh[] = {
	F_BASIC( 73728000, SRC_PLL3, 10, NOMINAL),
	F_BASIC( 92160000, SRC_PLL3,  8, NOMINAL),
	F_BASIC(122880000, SRC_PLL3,  6, NOMINAL),
	F_BASIC(184320000, SRC_PLL3,  4, NOMINAL),
	F_BASIC(245760000, SRC_PLL3,  3, NOMINAL),
	F_BASIC(368640000, SRC_PLL3,  2, NOMINAL),
	F_BASIC(384000000, SRC_PLL1,  2, NOMINAL),
	F_BASIC(445500000, SRC_PLL4,  2, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_grp[] = {
	F_BASIC( 24576000, SRC_LPXO,  1, NOMINAL),
	F_BASIC( 46080000, SRC_PLL3, 16, NOMINAL),
	F_BASIC( 49152000, SRC_PLL3, 15, NOMINAL),
	F_BASIC( 52662875, SRC_PLL3, 14, NOMINAL),
	F_BASIC( 56713846, SRC_PLL3, 13, NOMINAL),
	F_BASIC( 61440000, SRC_PLL3, 12, NOMINAL),
	F_BASIC( 67025454, SRC_PLL3, 11, NOMINAL),
	F_BASIC( 73728000, SRC_PLL3, 10, NOMINAL),
	F_BASIC( 81920000, SRC_PLL3,  9, NOMINAL),
	F_BASIC( 92160000, SRC_PLL3,  8, NOMINAL),
	F_BASIC(105325714, SRC_PLL3,  7, NOMINAL),
	F_BASIC(122880000, SRC_PLL3,  6, NOMINAL),
	F_BASIC(147456000, SRC_PLL3,  5, NOMINAL),
	F_BASIC(184320000, SRC_PLL3,  4, NOMINAL),
	F_BASIC(192000000, SRC_PLL1,  4, NOMINAL),
	F_BASIC(245760000, SRC_PLL3,  3, HIGH),
	/* Sync to AXI. Hence this "rate" is not fixed. */
	F_RAW(1, SRC_MAX, 0, B(14), 0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdc1_3[] = {
	F_MND8(  144000, 19, 12, SRC_LPXO, 1,   1,  171, NOMINAL),
	F_MND8(  400000, 19, 12, SRC_LPXO, 1,   2,  123, NOMINAL),
	F_MND8(16027000, 19, 12, SRC_PLL3, 3,  14,  215, NOMINAL),
	F_MND8(17000000, 19, 12, SRC_PLL3, 4,  19,  206, NOMINAL),
	F_MND8(20480000, 19, 12, SRC_PLL3, 4,  23,  212, NOMINAL),
	F_MND8(24576000, 19, 12, SRC_LPXO, 1,   0,    0, NOMINAL),
	F_MND8(49152000, 19, 12, SRC_PLL3, 3,   1,    5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_sdc2_4[] = {
	F_MND8(  144000, 20, 13, SRC_LPXO, 1,   1,  171, NOMINAL),
	F_MND8(  400000, 20, 13, SRC_LPXO, 1,   2,  123, NOMINAL),
	F_MND8(16027000, 20, 13, SRC_PLL3, 3,  14,  215, NOMINAL),
	F_MND8(17000000, 20, 13, SRC_PLL3, 4,  19,  206, NOMINAL),
	F_MND8(20480000, 20, 13, SRC_PLL3, 4,  23,  212, NOMINAL),
	F_MND8(24576000, 20, 13, SRC_LPXO, 1,   0,    0, NOMINAL),
	F_MND8(49152000, 20, 13, SRC_PLL3, 3,   1,    5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_core[] = {
	F_BASIC( 46080000, SRC_PLL3, 16, NOMINAL),
	F_BASIC( 49152000, SRC_PLL3, 15, NOMINAL),
	F_BASIC( 52663000, SRC_PLL3, 14, NOMINAL),
	F_BASIC( 92160000, SRC_PLL3,  8, NOMINAL),
	F_BASIC(122880000, SRC_PLL3,  6, NOMINAL),
	F_BASIC(147456000, SRC_PLL3,  5, NOMINAL),
	F_BASIC(153600000, SRC_PLL1,  5, NOMINAL),
	F_BASIC(192000000, SRC_PLL1,  4, HIGH),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_lcdc[] = {
	F_MND16(24576000, SRC_LPXO, 1,   0,   0, NOMINAL),
	F_MND16(30720000, SRC_PLL3, 4,   1,   6, NOMINAL),
	F_MND16(40960000, SRC_PLL3, 2,   1,   9, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mdp_vsync[] = {
	F_RAW(24576000, SRC_LPXO, 0, 0, 0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mi2s_codec[] = {
	F_MND16( 2048000, SRC_LPXO, 4,   1,   3, NOMINAL),
	F_MND16(12288000, SRC_LPXO, 2,   0,   0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mi2s[] = {
	F_MND16(12288000, SRC_LPXO, 2,   0,   0, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_midi[] = {
	F_MND8(98304000, 19, 12, SRC_PLL3, 3,  2,  5, NOMINAL),
	F_END,
};
static struct clk_freq_tbl clk_tbl_sdac[] = {
	F_MND16( 256000, SRC_LPXO, 4,   1,    24, NOMINAL),
	F_MND16( 352800, SRC_LPXO, 1, 147, 10240, NOMINAL),
	F_MND16( 384000, SRC_LPXO, 4,   1,    16, NOMINAL),
	F_MND16( 512000, SRC_LPXO, 4,   1,    12, NOMINAL),
	F_MND16( 705600, SRC_LPXO, 1, 147,  5120, NOMINAL),
	F_MND16( 768000, SRC_LPXO, 4,   1,     8, NOMINAL),
	F_MND16(1024000, SRC_LPXO, 4,   1,     6, NOMINAL),
	F_MND16(1411200, SRC_LPXO, 1, 147,  2560, NOMINAL),
	F_MND16(1536000, SRC_LPXO, 4,   1,     4, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_tv[] = {
	F_MND8(27000000, 23, 16, SRC_PLL4, 2,  2,  33, NOMINAL),
	F_MND8(74250000, 23, 16, SRC_PLL4, 2,  1,   6, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_usb[] = {
	F_MND8(60000000, 23, 16, SRC_PLL1, 2,  5,  32, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_vfe_jpeg[] = {
	F_MND16( 36864000, SRC_PLL3, 4,   1,   5, NOMINAL),
	F_MND16( 46080000, SRC_PLL3, 4,   1,   4, NOMINAL),
	F_MND16( 61440000, SRC_PLL3, 4,   1,   3, NOMINAL),
	F_MND16( 73728000, SRC_PLL3, 2,   1,   5, NOMINAL),
	F_MND16( 81920000, SRC_PLL3, 3,   1,   3, NOMINAL),
	F_MND16( 92160000, SRC_PLL3, 4,   1,   2, NOMINAL),
	F_MND16( 98304000, SRC_PLL3, 3,   2,   5, NOMINAL),
	F_MND16(105326000, SRC_PLL3, 2,   2,   7, NOMINAL),
	F_MND16(122880000, SRC_PLL3, 2,   1,   3, NOMINAL),
	F_MND16(147456000, SRC_PLL3, 2,   2,   5, NOMINAL),
	F_MND16(153600000, SRC_PLL1, 2,   2,   5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_cam[] = {
	F_MND16( 6000000, SRC_PLL1, 4,   1,  32, NOMINAL),
	F_MND16( 8000000, SRC_PLL1, 4,   1,  24, NOMINAL),
	F_MND16(12000000, SRC_PLL1, 4,   1,  16, NOMINAL),
	F_MND16(16000000, SRC_PLL1, 4,   1,  12, NOMINAL),
	F_MND16(19200000, SRC_PLL1, 4,   1,  10, NOMINAL),
	F_MND16(24000000, SRC_PLL1, 4,   1,   8, NOMINAL),
	F_MND16(32000000, SRC_PLL1, 4,   1,   6, NOMINAL),
	F_MND16(48000000, SRC_PLL1, 4,   1,   4, NOMINAL),
	F_MND16(64000000, SRC_PLL1, 4,   1,   3, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_vpe[] = {
	F_MND8( 24576000, 22, 15, SRC_LPXO, 1,   0,   0, NOMINAL),
	F_MND8( 30720000, 22, 15, SRC_PLL3, 4,   1,   6, NOMINAL),
	F_MND8( 61440000, 22, 15, SRC_PLL3, 4,   1,   3, NOMINAL),
	F_MND8( 81920000, 22, 15, SRC_PLL3, 3,   1,   3, NOMINAL),
	F_MND8(122880000, 22, 15, SRC_PLL3, 3,   1,   2, NOMINAL),
	F_MND8(147456000, 22, 15, SRC_PLL3, 1,   1,   5, NOMINAL),
	F_MND8(153600000, 22, 15, SRC_PLL1, 1,   1,   5, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_mfc[] = {
	F_MND8( 24576000, 24, 17, SRC_LPXO, 1,   0,   0, NOMINAL),
	F_MND8( 30720000, 24, 17, SRC_PLL3, 4,   1,   6, NOMINAL),
	F_MND8( 61440000, 24, 17, SRC_PLL3, 4,   1,   3, NOMINAL),
	F_MND8( 81920000, 24, 17, SRC_PLL3, 3,   1,   3, NOMINAL),
	F_MND8(122880000, 24, 17, SRC_PLL3, 3,   1,   2, NOMINAL),
	F_MND8(147456000, 24, 17, SRC_PLL3, 1,   1,   5, NOMINAL),
	F_MND8(153600000, 24, 17, SRC_PLL1, 1,   1,   5, NOMINAL),
	F_MND8(170667000, 24, 17, SRC_PLL1, 1,   2,   9, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_spi[] = {
	F_MND8( 9963243, 19, 12, SRC_PLL3, 4,   7,   129, NOMINAL),
	F_MND8(26331429, 19, 12, SRC_PLL3, 4,  34,   241, NOMINAL),
	F_END,
};

static struct clk_freq_tbl clk_tbl_lpa_codec[] = {
	F_RAW(1, SRC_MAX, 0,  0,  0, MSMC1_END), /* src = MI2S_CODEC_RX */
	F_RAW(2, SRC_MAX, 0,  1,  0, MSMC1_END), /* src = ECODEC_CIF */
	F_RAW(3, SRC_MAX, 0,  2,  0, MSMC1_END), /* src = MI2S */
	F_RAW(4, SRC_MAX, 0,  3,  0, MSMC1_END), /* src = SDAC */
	F_END,
};

static struct clk_freq_tbl dummy_freq = F_END;

#define MND	1 /* Integer predivider and fractional MN:D divider. */
#define BASIC	2 /* Integer divider. */
#define NORATE	3 /* Just on/off. */

#define C(x) L_7X30_##x##_CLK

#define CLK_LOCAL(id, t, md, ns, f_msk, br, root, tbl, par, chld_lst) \
	[C(id)] = { \
	.type = t, \
	.md_reg = md, \
	.ns_reg = ns, \
	.freq_mask = f_msk, \
	.br_en_mask = br, \
	.root_en_mask = root, \
	.parent = C(par), \
	.children = chld_lst, \
	.freq_tbl = tbl, \
	.current_freq = &dummy_freq, \
	}

#define CLK_BASIC(id, ns, br, root, tbl, par) \
		CLK_LOCAL(id, BASIC, 0, ns, F_MASK_BASIC, br, root, tbl, \
								par, NULL)
#define CLK_MND8_P(id, ns, m, l, br, root, tbl, par, chld_lst) \
		CLK_LOCAL(id, MND, (ns-4), ns, F_MASK_MND8(m, l), br, root, \
							tbl, par, chld_lst)
#define CLK_MND8(id, ns, m, l, br, root, tbl, chld_lst) \
		CLK_MND8_P(id, ns, m, l, br, root, tbl, NONE, chld_lst)
#define CLK_MND16(id, ns, br, root, tbl, par, chld_lst) \
		CLK_LOCAL(id, MND, (ns-4), ns, F_MASK_MND16, br, root, tbl, \
								par, chld_lst)
#define CLK_1RATE(id, ns, br, root, tbl) \
		CLK_LOCAL(id, BASIC, 0, ns, 0, br, root, tbl, NONE, NULL)
#define CLK_SLAVE(id, ns, br, par) \
		CLK_LOCAL(id, NORATE, 0, ns, 0, br, 0, NULL, par, NULL)
#define CLK_NORATE(id, ns, br, root) \
		CLK_LOCAL(id, NORATE, 0, ns, 0, br, root, NULL, NONE, NULL)
#define CLK_GLBL(id, glbl, root) \
		CLK_LOCAL(id, NORATE, 0, glbl, 0, 0, root, NULL, \
							GLBL_ROOT, NULL)
#define CLK_BRIDGE(id, glbl, root, par) \
		CLK_LOCAL(id, NORATE, 0, glbl, 0, 0, root, NULL, par, NULL)

#define REG_BASE(off) (MSM_CLK_CTL_BASE + off)
#define REG(off) (MSM_CLK_CTL_SH2_BASE + off)
#define MNCNTR_EN_MASK		B(8)
#define MNCNTR_RST_MASK		B(7)
#define MNCNTR_MODE_MASK	BM(6, 5)
#define MNCNTR_MODE		BVAL(6, 5, 0x2) /* Dual-edge mode. */

/* Register offsets used more than once. */
#define CSI_NS			0x0174
#define EMDH_NS			0x0050
#define PMDH_NS			0x008C
#define UART_NS			0x00E0
#define UART2_NS		0x0464
#define USBH_MD			0x02BC
#define USBH_NS			0x02C0
#define USBH2_NS		0x046C
#define USBH3_NS		0x0470
#define CAM_VFE_NS		0x0044
#define GLBL_CLK_ENA_SC		0x03BC
#define GLBL_CLK_ENA_2_SC	0x03C0
#define GRP_NS			0x0084
#define SDAC_NS			0x009C
#define TV_NS			0x00CC
#define HDMI_NS			0x0484
#define MI2S_RX_NS		0x0070
#define MI2S_TX_NS		0x0078
#define MI2S_NS			0x02E0
#define MFC_NS			0x0154
#define LPA_NS			0x02E8
#define MDC_NS			0x007C
#define MDP_LCDC_NS		0x0390
#define MDP_VSYNC_REG		0x0460

#if CONFIG_MSM_AMSS_VERSION >= 1200
#define PLL_ENA_REG		0x0264
#else
#define PLL_ENA_REG		0x0260
#endif

#define LPA_CORE_CLK_MA0	0x04F4
#define LPA_CORE_CLK_MA2	0x04FC
#define SH2_OWN_GLBL_REG	0x0404
#define SH2_OWN_APPS1_REG	0x040C
#define SH2_OWN_APPS2_REG	0x0414
#define SH2_OWN_ROW1_REG	0x041C
#define SH2_OWN_ROW2_REG	0x0424
#define SH2_OWN_APPS3_REG	0x0444

static uint32_t *pll_status_addr[NUM_PLL] = {
	[PLL_0] = REG_BASE(0x318),
	[PLL_1] = REG_BASE(0x334),
	[PLL_2] = REG_BASE(0x350),
	[PLL_3] = REG_BASE(0x36C),
	[PLL_4] = REG_BASE(0x254),
	[PLL_5] = REG_BASE(0x258),
	[PLL_6] = REG_BASE(0x4EC),
};

static uint32_t pll_count[NUM_PLL];

static uint32_t chld_grp_3d_src[] = 	{C(IMEM), C(GRP_3D), C(NONE)};
static uint32_t chld_mdp_lcdc_p[] = 	{C(MDP_LCDC_PAD_PCLK), C(NONE)};
static uint32_t chld_mfc[] = 		{C(MFC_DIV2), C(NONE)};
static uint32_t chld_mi2s_codec_rx[] =	{C(MI2S_CODEC_RX_S), C(NONE)};
static uint32_t chld_mi2s_codec_tx[] =	{C(MI2S_CODEC_TX_S), C(NONE)};
static uint32_t chld_mi2s[] = 		{C(MI2S_S), C(NONE)};
static uint32_t chld_sdac[] = 		{C(SDAC_M), C(NONE)};
static uint32_t chld_tv[] = 		{C(TV_DAC), C(TV_ENC), C(TSIF_REF),
					 C(HDMI), C(NONE)};
static uint32_t chld_usb_src[] = 	{C(USB_HS), C(USB_HS_CORE),
					 C(USB_HS2), C(USB_HS2_CORE),
					 C(USB_HS3), C(USB_HS3_CORE),
					 C(NONE)};
uint32_t chld_vfe[] = 			{C(VFE_MDC), C(VFE_CAMIF), C(CSI0_VFE),
					 C(NONE)};

static struct clk_local clk_local_tbl[] = {
	CLK_NORATE(MDC,		MDC_NS, B(9), B(11)),
	CLK_NORATE(LPA_CORE,	LPA_NS, B(5), 0),

	CLK_1RATE(I2C,		0x0068, B(9), B(11),	clk_tbl_tcxo),
	CLK_1RATE(I2C_2,	0x02D8, B(0), B(2),	clk_tbl_tcxo),
	CLK_1RATE(QUP_I2C,	0x04F0, B(0), B(2),	clk_tbl_tcxo),
	CLK_1RATE(UART1,	UART_NS, B(5), B(4),	clk_tbl_tcxo),
	CLK_1RATE(UART2,	UART2_NS, B(5), B(4),	clk_tbl_tcxo),

	CLK_BASIC(EMDH,	EMDH_NS,    0, B(11),	clk_tbl_mdh, AXI_LI_ADSP_A),
	CLK_BASIC(PMDH,	PMDH_NS,    0, B(11),	clk_tbl_mdh, AXI_LI_ADSP_A),
	CLK_BASIC(MDP,	0x014C, B(9), B(11),	clk_tbl_mdp_core, AXI_MDP),

	CLK_MND8_P(VPE, 0x015C, 22, 15, B(9), B(11), clk_tbl_vpe,
							AXI_VPE, NULL),
	CLK_MND8_P(MFC, MFC_NS, 24, 17, B(9), B(11), clk_tbl_mfc,
							AXI_MFC, chld_mfc),
	CLK_SLAVE(MFC_DIV2, MFC_NS, B(15), MFC),

	CLK_MND8(SDC1,	0x00A4, 19, 12, B(9), B(11),	clk_tbl_sdc1_3,	NULL),
	CLK_MND8(SDC2,	0x00AC, 20, 13, B(9), B(11),	clk_tbl_sdc2_4,	NULL),
	CLK_MND8(SDC3,	0x00B4, 19, 12, B(9), B(11),	clk_tbl_sdc1_3,	NULL),
	CLK_MND8(SDC4,	0x00BC, 20, 13, B(9), B(11),	clk_tbl_sdc2_4,	NULL),
	CLK_MND8(SPI,	0x02C8, 19, 12, B(9), B(11),	clk_tbl_spi,	NULL),
	CLK_MND8(MIDI,	0x02D0, 19, 12, B(9), B(11),	clk_tbl_midi,	NULL),
	CLK_MND8_P(USB_HS_SRC, USBH_NS, 23, 16, 0, B(11), clk_tbl_usb,
					AXI_LI_ADSP_A,	chld_usb_src),
	CLK_SLAVE(USB_HS,	USBH_NS,	B(9),	USB_HS_SRC),
	CLK_SLAVE(USB_HS_CORE,	USBH_NS,	B(13),	USB_HS_SRC),
	CLK_SLAVE(USB_HS2,	USBH2_NS,	B(9),	USB_HS_SRC),
	CLK_SLAVE(USB_HS2_CORE,	USBH2_NS,	B(4),	USB_HS_SRC),
	CLK_SLAVE(USB_HS3,	USBH3_NS,	B(9),	USB_HS_SRC),
	CLK_SLAVE(USB_HS3_CORE,	USBH3_NS,	B(4),	USB_HS_SRC),
	CLK_MND8(TV,	TV_NS, 23, 16, 0, B(11), clk_tbl_tv, chld_tv),
	CLK_SLAVE(HDMI,		HDMI_NS, B(9),		TV),
	CLK_SLAVE(TV_DAC,	TV_NS, B(12),		TV),
	CLK_SLAVE(TV_ENC,	TV_NS, B(9),		TV),
	/* Hacking root & branch into one param. */
	CLK_SLAVE(TSIF_REF,	0x00C4, B(9)|B(11),	TV),

	CLK_MND16(UART1DM, 0x00D4, B(9), B(11), clk_tbl_uartdm, NONE, NULL),
	CLK_MND16(UART2DM, 0x00DC, B(9), B(11), clk_tbl_uartdm, NONE, NULL),
	CLK_MND16(JPEG,    0x0164, B(9), B(11), clk_tbl_vfe_jpeg,
							AXI_LI_JPEG, NULL),
	CLK_MND16(CAM_M, 0x0374, 0, B(9), clk_tbl_cam, NONE, NULL),
	CLK_MND16(VFE, CAM_VFE_NS, B(9), B(13), clk_tbl_vfe_jpeg,
							AXI_LI_VFE, chld_vfe),
	CLK_SLAVE(VFE_MDC, CAM_VFE_NS, B(11), VFE),
	CLK_SLAVE(VFE_CAMIF, CAM_VFE_NS, B(15), VFE),
	CLK_SLAVE(CSI0_VFE, CSI_NS, B(15), VFE),

	CLK_MND16(SDAC, SDAC_NS, B(9), B(11), clk_tbl_sdac,
							NONE, chld_sdac),
	CLK_SLAVE(SDAC_M, SDAC_NS, B(12), SDAC),

	CLK_MND16(MDP_LCDC_PCLK, MDP_LCDC_NS, B(9), B(11), clk_tbl_mdp_lcdc,
							NONE, chld_mdp_lcdc_p),
	CLK_SLAVE(MDP_LCDC_PAD_PCLK, MDP_LCDC_NS, B(12), MDP_LCDC_PCLK),
	CLK_1RATE(MDP_VSYNC, MDP_VSYNC_REG, B(0), 0, clk_tbl_mdp_vsync),

	CLK_MND16(MI2S_CODEC_RX_M, MI2S_RX_NS, B(12), B(11),
				clk_tbl_mi2s_codec, NONE, chld_mi2s_codec_rx),
	CLK_SLAVE(MI2S_CODEC_RX_S, MI2S_RX_NS, B(9), MI2S_CODEC_RX_M),

	CLK_MND16(MI2S_CODEC_TX_M, MI2S_TX_NS, B(12), B(11),
				clk_tbl_mi2s_codec, NONE, chld_mi2s_codec_tx),
	CLK_SLAVE(MI2S_CODEC_TX_S, MI2S_TX_NS, B(9), MI2S_CODEC_TX_M),

	CLK_MND16(MI2S_M, MI2S_NS, B(12), B(11),
				clk_tbl_mi2s, NONE, chld_mi2s),
	CLK_SLAVE(MI2S_S, MI2S_NS, B(9), MI2S_M),

	CLK_LOCAL(GRP_2D, BASIC, 0, 0x0034, F_MASK_BASIC | (7 << 12),
			B(7), B(11), clk_tbl_grp, AXI_GRP_2D, NULL),
	CLK_LOCAL(GRP_3D_SRC, BASIC, 0, GRP_NS, F_MASK_BASIC | (7 << 12),
			0, B(11), clk_tbl_grp, AXI_LI_GRP, chld_grp_3d_src),
	CLK_SLAVE(GRP_3D, GRP_NS, B(7), GRP_3D_SRC),
	CLK_SLAVE(IMEM, GRP_NS, B(9), GRP_3D_SRC),
	CLK_LOCAL(LPA_CODEC, BASIC, 0, LPA_NS, BM(1, 0), B(9), 0,
					clk_tbl_lpa_codec, NONE, NULL),

	CLK_MND8(CSI0, CSI_NS, 24, 17, B(9), B(11), clk_tbl_csi, NULL),

	/* For global clocks to be on we must have GLBL_ROOT_ENA set */
	CLK_NORATE(GLBL_ROOT,	GLBL_CLK_ENA_SC, 0,	B(29)),

	/* Peripheral bus clocks. */
	CLK_GLBL(ADM,	 	GLBL_CLK_ENA_SC,	B(5)),
	CLK_GLBL(CAMIF_PAD_P,	GLBL_CLK_ENA_SC,	B(9)),
	CLK_GLBL(CSI0_P,	GLBL_CLK_ENA_SC,	B(30)),
	CLK_GLBL(EMDH_P,	GLBL_CLK_ENA_2_SC,	B(3)),
	CLK_GLBL(GRP_2D_P,	GLBL_CLK_ENA_SC,	B(24)),
	CLK_GLBL(GRP_3D_P,	GLBL_CLK_ENA_2_SC,	B(17)),
	CLK_GLBL(JPEG_P,	GLBL_CLK_ENA_2_SC,	B(24)),
	CLK_GLBL(LPA_P,		GLBL_CLK_ENA_2_SC,	B(7)),
	CLK_GLBL(MDP_P,		GLBL_CLK_ENA_2_SC,	B(6)),
	CLK_GLBL(MFC_P,		GLBL_CLK_ENA_2_SC,	B(26)),
	CLK_GLBL(PMDH_P,	GLBL_CLK_ENA_2_SC,	B(4)),
	CLK_GLBL(ROTATOR_IMEM,	GLBL_CLK_ENA_2_SC,	B(23)),
	CLK_GLBL(ROTATOR_P,	GLBL_CLK_ENA_2_SC,	B(25)),
	CLK_GLBL(SDC1_P,	GLBL_CLK_ENA_SC,	B(7)),
	CLK_GLBL(SDC2_P,	GLBL_CLK_ENA_SC,	B(8)),
	CLK_GLBL(SDC3_P,	GLBL_CLK_ENA_SC,	B(27)),
	CLK_GLBL(SDC4_P,	GLBL_CLK_ENA_SC,	B(28)),
	CLK_GLBL(SPI_P,		GLBL_CLK_ENA_2_SC,	B(10)),
	CLK_GLBL(TSIF_P,	GLBL_CLK_ENA_SC,	B(18)),
	CLK_GLBL(UART1DM_P,	GLBL_CLK_ENA_SC,	B(17)),
	CLK_GLBL(UART2DM_P,	GLBL_CLK_ENA_SC,	B(26)),
	CLK_GLBL(USB_HS2_P,	GLBL_CLK_ENA_2_SC,	B(8)),
	CLK_GLBL(USB_HS3_P,	GLBL_CLK_ENA_2_SC,	B(9)),
	CLK_GLBL(USB_HS_P,	GLBL_CLK_ENA_SC,	B(25)),
	CLK_GLBL(VFE_P,		GLBL_CLK_ENA_2_SC,	B(27)),

	/* AXI bridge clocks. */
	CLK_BRIDGE(AXI_LI_APPS,	GLBL_CLK_ENA_SC,	B(2),	GLBL_ROOT),
	CLK_BRIDGE(AXI_LI_ADSP_A, GLBL_CLK_ENA_2_SC,	B(14),	AXI_LI_APPS),
	CLK_BRIDGE(AXI_LI_JPEG,	GLBL_CLK_ENA_2_SC,	B(19),	AXI_LI_APPS),
	CLK_BRIDGE(AXI_LI_VFE,	GLBL_CLK_ENA_SC,	B(23),	AXI_LI_APPS),
	CLK_BRIDGE(AXI_MDP,	GLBL_CLK_ENA_2_SC,	B(29),	AXI_LI_APPS),

	CLK_BRIDGE(AXI_IMEM,	GLBL_CLK_ENA_2_SC,	B(18),	GLBL_ROOT),

	CLK_BRIDGE(AXI_LI_VG,	GLBL_CLK_ENA_SC,	B(3),	GLBL_ROOT),
	CLK_BRIDGE(AXI_GRP_2D,	GLBL_CLK_ENA_SC,	B(21),	AXI_LI_VG),
	CLK_BRIDGE(AXI_LI_GRP,	GLBL_CLK_ENA_SC,	B(22),	AXI_LI_VG),
	CLK_BRIDGE(AXI_MFC,	GLBL_CLK_ENA_2_SC,	B(20),	AXI_LI_VG),
	CLK_BRIDGE(AXI_ROTATOR,	GLBL_CLK_ENA_2_SC,	B(22),	AXI_LI_VG),
	CLK_BRIDGE(AXI_VPE,	GLBL_CLK_ENA_2_SC,	B(21),	AXI_LI_VG),
};

static DEFINE_SPINLOCK(clock_reg_lock);
static DEFINE_SPINLOCK(pll_vote_lock);

#define PLL_ACTIVE_MASK	B(16)
void pll_enable(uint32_t pll)
{
	uint32_t reg_val;
	unsigned long flags;

	BUG_ON(pll >= NUM_PLL);

	spin_lock_irqsave(&pll_vote_lock, flags);
	if (!pll_count[pll]) {
#if CONFIG_MSM_AMSS_VERSION >= 1200
		reg_val = readl(REG(PLL_ENA_REG));
		reg_val |= (1 << pll);
		writel(reg_val, REG(PLL_ENA_REG));
#else
		reg_val = readl(REG_BASE(PLL_ENA_REG));
		reg_val |= (1 << pll);
		writel(reg_val, REG_BASE(PLL_ENA_REG));
#endif
	}
	pll_count[pll]++;
	spin_unlock_irqrestore(&pll_vote_lock, flags);

	/* Wait until PLL is enabled. */
	while ((readl(pll_status_addr[pll]) & PLL_ACTIVE_MASK) == 0)
		;
}

static void src_enable(uint32_t src)
{
	/* SRC_MAX is used as a placeholder for some freqencies that don't
	 * have any direct PLL dependency. */
	if (src == SRC_MAX || src == SRC_LPXO)
		return;

	pll_enable(src_pll_tbl[src]);
}

void pll_disable(uint32_t pll)
{
	uint32_t reg_val;
	unsigned long flags;

	BUG_ON(pll >= NUM_PLL);

	spin_lock_irqsave(&pll_vote_lock, flags);
	if (pll_count[pll])
		pll_count[pll]--;
	else
		pr_warning("Reference count mismatch in PLL disable!\n");

	if (pll_count[pll] == 0) {
#if CONFIG_MSM_AMSS_VERSION >= 1200
		reg_val = readl(REG(PLL_ENA_REG));
		reg_val &= ~(1 << pll);
		writel(reg_val, REG(PLL_ENA_REG));
#else
		reg_val = readl(REG_BASE(PLL_ENA_REG));
		reg_val &= ~(1 << pll);
		writel(reg_val, REG_BASE(PLL_ENA_REG));
#endif
	}
	spin_unlock_irqrestore(&pll_vote_lock, flags);
}

static void src_disable(uint32_t src)
{
	/* SRC_MAX is used as a placeholder for some freqencies that don't
	 * have any direct PLL dependency. */
	if (src == SRC_MAX || src == SRC_LPXO)
		return;

	pll_disable(src_pll_tbl[src]);

}

static unsigned msmc1_votes[MSMC1_END];
static unsigned msmc1_level;

static int update_msmc1(void)
{
	int err, target, mvolts;

	target = mvolts = msmc1_votes[HIGH] ? 1200 : 1100;

	if (target == msmc1_level)
		return 0;

	err = msm_proc_comm(PCOM_CLKCTL_RPC_MIN_MSMC1, &mvolts, NULL);
	if (err)
		goto out;
	if (mvolts) {
		err = -EINVAL;
		goto out;
	}
	msmc1_level = target;
out:
	return err;
}

static void unvote_msmc1(unsigned level)
{
	if (level >= ARRAY_SIZE(msmc1_votes))
		return;

	if (msmc1_votes[level]) {
		msmc1_votes[level]--;
	} else {
		pr_warning("%s: Reference counts are incorrect\n", __func__);
		return;
	}

	update_msmc1();
}

static int vote_msmc1(unsigned level)
{
	int ret;

	if (level >= ARRAY_SIZE(msmc1_votes))
		return 0;

	msmc1_votes[level]++;
	ret = update_msmc1();
	if (ret)
		msmc1_votes[level]--;

	return ret;
}

/*
 * SoC specific register-based control of clocks.
 */
static int _soc_clk_enable(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	void *ns_reg = REG(t->ns_reg);
	uint32_t reg_val = 0;

	reg_val = readl(ns_reg);
	if (t->type == MND) {
		/* mode can be either 0 or 1. So the R-value of the
		 * expression will evaluate to MNCNTR_EN_MASK or 0. This
		 * avoids the need for a "if(mode == 1)". A "&" will not work
		 * here. */
		reg_val |= (MNCNTR_EN_MASK * t->current_freq->mode);
		writel(reg_val, ns_reg);
	}
	if (t->root_en_mask) {
		reg_val |= t->root_en_mask;
		writel(reg_val, ns_reg);
	}
	if (t->br_en_mask) {
		reg_val |= t->br_en_mask;
		writel(reg_val, ns_reg);
	}
	return 0;
}

static void _soc_clk_disable(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	void *ns_reg = REG(t->ns_reg);
	uint32_t reg_val = 0;

	reg_val = readl(ns_reg);

	if (t->br_en_mask) {
		reg_val &= ~(t->br_en_mask);
		writel(reg_val, ns_reg);
	}
	if (t->root_en_mask) {
		reg_val &= ~(t->root_en_mask);
		writel(reg_val, ns_reg);
	}
	if (t->type == MND) {
		reg_val &= ~MNCNTR_EN_MASK;
		writel(reg_val, ns_reg);
	}
}

static int soc_clk_enable_nolock(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	int ret = 0;

	if (!t->count) {
		ret = vote_msmc1(t->current_freq->msmc1);
		if (ret)
			return ret;
		if (t->parent != C(NONE)) {
			ret = soc_clk_enable_nolock(t->parent);
			if (ret)
				return ret;
		}
		src_enable(t->current_freq->src);
		ret = _soc_clk_enable(id);
	}
	t->count++;

	return ret;
}

static void soc_clk_disable_nolock(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];

	if (!t->count) {
		pr_warning("Reference count mismatch in clock disable!\n");
		return;
	}
	if (t->count)
		t->count--;
	if (t->count == 0) {
		_soc_clk_disable(id);
		src_disable(t->current_freq->src);
		unvote_msmc1(t->current_freq->msmc1);
		if (t->parent != C(NONE))
			soc_clk_disable_nolock(t->parent);
	}

	return;
}

static int update_pwr_rail(unsigned id, int enable)
{
	int pwr_id = 0;
	switch (id) {
	case C(AXI_ROTATOR):
		pwr_id = PWR_RAIL_ROTATOR_CLK;
		break;
	case C(GRP_2D):
		pwr_id = PWR_RAIL_GRP_2D_CLK;
		break;
	case C(GRP_3D):
		pwr_id = PWR_RAIL_GRP_CLK;
		break;
	case C(MFC):
		pwr_id = PWR_RAIL_MFC_CLK;
		break;
	case C(VFE):
		pwr_id = PWR_RAIL_VFE_CLK;
		break;
	case C(VPE):
		pwr_id = PWR_RAIL_VPE_CLK;
		break;
	default:
		return 0;
	}

	return internal_pwr_rail_ctl_auto(pwr_id, enable);
}

static int soc_clk_enable(unsigned id)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	ret = soc_clk_enable_nolock(id);
	if (ret)
		goto unlock;
	/*
	 * The modem might modify the register bits for the clock branch when
	 * the rail is enabled/disabled, so enable the rail inside the lock
	 * instead of outside it.
	 */
	ret = update_pwr_rail(id, 1);
	if (ret)
		soc_clk_disable_nolock(id);
unlock:
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	return ret;
}

static void soc_clk_disable(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	update_pwr_rail(id, 0);
	soc_clk_disable_nolock(id);
	spin_unlock_irqrestore(&clock_reg_lock, flags);
}

static void soc_clk_auto_off(unsigned id)
{
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	_soc_clk_disable(id);
	spin_unlock_irqrestore(&clock_reg_lock, flags);
}

static long soc_clk_round_rate(unsigned id, unsigned rate)
{
	struct clk_local *t = &clk_local_tbl[id];
	struct clk_freq_tbl *f;

	if (t->type != MND && t->type != BASIC)
		return -EINVAL;

	for (f = t->freq_tbl; f->freq_hz != FREQ_END; f++)
		if (f->freq_hz >= rate)
			return f->freq_hz;

	return -EPERM;
}

static int soc_clk_set_rate(unsigned id, unsigned rate)
{
	struct clk_local *t = &clk_local_tbl[id];
	struct clk_freq_tbl *cf = t->current_freq;
	struct clk_freq_tbl *nf;
	uint32_t *chld = t->children;
	void *ns_reg = REG(t->ns_reg);
	void *md_reg = REG(t->md_reg);
	uint32_t reg_val = 0;
	int i, ret = 0;
	unsigned long flags;
	long rounded;

	rounded = soc_clk_round_rate(id, rate);
	if (rounded != rate)
		pr_warning("Use clk_round_rate() before clk_set_rate() with "
			   "clock %u\n", id);
	rate = rounded;

	if (t->type != MND && t->type != BASIC)
		return -EPERM;

	spin_lock_irqsave(&clock_reg_lock, flags);

	if (rate == cf->freq_hz)
		goto release_lock;

	for (nf = t->freq_tbl; nf->freq_hz != FREQ_END; nf++)
		if (nf->freq_hz == rate)
			break;

	if (nf->freq_hz == FREQ_END) {
		ret = -EINVAL;
		goto release_lock;
	}

	if (t->freq_mask == 0) {
		t->current_freq = nf;
		goto release_lock;
	}

	/* Disable all branches before changing rate to prevent jitter. */
	for (i = 0; chld && chld[i] != C(NONE); i++) {
		struct clk_local *ch = &clk_local_tbl[chld[i]];
		/* Don't bother turning off if it is already off.
		 * Checking ch->count is cheaper (cache) than reading and
		 * writing to a register (uncached/unbuffered). */
		if (ch->count) {
			reg_val = readl(REG(ch->ns_reg));
			reg_val &= ~(ch->br_en_mask);
			writel(reg_val, REG(ch->ns_reg));
		}
	}

	if (t->count) {
		_soc_clk_disable(id);

		ret = vote_msmc1(nf->msmc1);
		if (ret)
			goto msmc1_err;
		/* Turn on PLL of the new freq. */
		src_enable(nf->src);
	}

	/* Some clocks share the same register, so must be careful when
	 * assuming a register doesn't need to be re-read. */
	reg_val = readl(ns_reg);
	if (t->type == MND) {
		reg_val |= MNCNTR_RST_MASK;
		writel(reg_val, ns_reg);
		/* TODO: Currently writing 0's into reserved bits for 8-bit
		 * MND. Can be avoided by adding md_mask. */
		if (nf->mode)
			writel(nf->md_val, md_reg);
		reg_val &= ~MNCNTR_MODE_MASK;
		reg_val |= (MNCNTR_MODE * nf->mode);
	}
	reg_val &= ~(t->freq_mask);
	reg_val |= nf->ns_val;
	writel(reg_val, ns_reg);

	if (t->type == MND) {
		reg_val &= ~MNCNTR_RST_MASK;
		writel(reg_val, ns_reg);
	}

	if (t->count) {
		/* Turn off PLL of the old freq. */
		src_disable(cf->src);
		unvote_msmc1(cf->msmc1);
	}

	/* Current freq must be updated before _soc_clk_enable() is called to
	 * make sure the MNCNTR_E bit is set correctly. */
	t->current_freq = nf;

msmc1_err:
	if (t->count)
		_soc_clk_enable(id);
	/* Enable only branches that were ON before. */
	for (i = 0; chld && chld[i] != C(NONE); i++) {
		struct clk_local *ch = &clk_local_tbl[chld[i]];
		if (ch->count) {
			reg_val = readl(REG(ch->ns_reg));
			reg_val |= ch->br_en_mask;
			writel(reg_val, REG(ch->ns_reg));
		}
	}

release_lock:
	spin_unlock_irqrestore(&clock_reg_lock, flags);
	return ret;
}

static int soc_clk_set_min_rate(unsigned id, unsigned rate)
{
	long rounded = soc_clk_round_rate(id, rate);
	return soc_clk_set_rate(id, rounded);
}

static int soc_clk_set_max_rate(unsigned id, unsigned rate)
{
	return -EPERM;
}

static int soc_clk_set_flags(unsigned id, unsigned clk_flags)
{
	uint32_t regval, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&clock_reg_lock, flags);
	switch (id) {
	case C(VFE):
		regval = readl(REG(CAM_VFE_NS));
		/* Flag values chosen for backward compatibility
		 * with proc_comm remote clock control. */
		if (clk_flags == 0x00000100) {
			/* Select external source. */
			regval |= B(14);
		} else if (clk_flags == 0x00000200) {
			/* Select internal source. */
			regval &= ~B(14);
		} else
			ret = -EINVAL;

		writel(regval, REG(CAM_VFE_NS));
		break;
	default:
		ret = -EPERM;
	}
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	return ret;
}

static unsigned soc_clk_get_rate(unsigned id)
{
	struct clk_local *t = &clk_local_tbl[id];
	unsigned long flags;
	unsigned ret = 0;

	if (t->type == NORATE)
		return -EINVAL;

	spin_lock_irqsave(&clock_reg_lock, flags);
	ret = t->current_freq->freq_hz;
	spin_unlock_irqrestore(&clock_reg_lock, flags);

	/* Return 0 if the rate has never been set. Might not be correct,
	 * but it's good enough. */
	if (ret == FREQ_END)
		ret = 0;

	return ret;
}

static unsigned soc_clk_is_enabled(unsigned id)
{
	return !!(clk_local_tbl[id].count);
}

struct clk_ops clk_ops_7x30 = {
	.enable = soc_clk_enable,
	.disable = soc_clk_disable,
	.auto_off = soc_clk_auto_off,
	.reset = NULL, /* Uses proc_comm */
	.set_rate = soc_clk_set_rate,
	.set_min_rate = soc_clk_set_min_rate,
	.set_max_rate = soc_clk_set_max_rate,
	.set_flags = soc_clk_set_flags,
	.get_rate = soc_clk_get_rate,
	.is_enabled = soc_clk_is_enabled,
	.round_rate = soc_clk_round_rate,
};

enum {
	SH2_OWN_GLBL,
	SH2_OWN_APPS1,
	SH2_OWN_APPS2,
	SH2_OWN_ROW1,
	SH2_OWN_ROW2,
	SH2_OWN_APPS3,
	NUM_OWNERSHIP
};
static __initdata uint32_t ownership_regs[NUM_OWNERSHIP];

static void __init cache_ownership(void)
{
	ownership_regs[SH2_OWN_GLBL] = readl(REG_BASE(SH2_OWN_GLBL_REG));
	ownership_regs[SH2_OWN_APPS1] = readl(REG_BASE(SH2_OWN_APPS1_REG));
	ownership_regs[SH2_OWN_APPS2] = readl(REG_BASE(SH2_OWN_APPS2_REG));
	ownership_regs[SH2_OWN_ROW1] = readl(REG_BASE(SH2_OWN_ROW1_REG));
	ownership_regs[SH2_OWN_ROW2] = readl(REG_BASE(SH2_OWN_ROW2_REG));
	ownership_regs[SH2_OWN_APPS3] = readl(REG_BASE(SH2_OWN_APPS3_REG));
}

static void __init print_ownership(void)
{
	pr_info("Clock ownership\n");
	pr_info("  GLBL  : %08x\n", ownership_regs[SH2_OWN_GLBL]);
	pr_info("  APPS  : %08x %08x %08x\n", ownership_regs[SH2_OWN_APPS1],
		ownership_regs[SH2_OWN_APPS2], ownership_regs[SH2_OWN_APPS3]);
	pr_info("  ROW   : %08x %08x\n", ownership_regs[SH2_OWN_ROW1],
		ownership_regs[SH2_OWN_ROW2]);
}

/*
 * This is a many-to-one mapping since we don't know how the remote clock code
 * has decided to handle the dependencies between clocks for a particular
 * hardware block. We determine the ownership for all the clocks on a block by
 * checking the ownership bit of one register (usually the ns register).
 */
#define O(x) &ownership_regs[x]
static struct clk_local_ownership {
	uint32_t *reg;
	uint32_t bit;
} ownership_map[] __initdata = {
	[C(GRP_2D)]			= { O(SH2_OWN_APPS1), B(6) },
	[C(GRP_2D_P)]			= { O(SH2_OWN_APPS1), B(6) },
	[C(HDMI)]			= { O(SH2_OWN_APPS1), B(31) },
	[C(JPEG)]			= { O(SH2_OWN_APPS1), B(0) },
	[C(JPEG_P)]			= { O(SH2_OWN_APPS1), B(0) },
	[C(LPA_CODEC)]			= { O(SH2_OWN_APPS1), B(23) },
	[C(LPA_CORE)]			= { O(SH2_OWN_APPS1), B(23) },
	[C(LPA_P)]			= { O(SH2_OWN_APPS1), B(23) },
	[C(MI2S_CODEC_RX_M)]		= { O(SH2_OWN_APPS1), B(12) },
	[C(MI2S_CODEC_RX_S)]		= { O(SH2_OWN_APPS1), B(12) },
	[C(MI2S_CODEC_TX_M)]		= { O(SH2_OWN_APPS1), B(14) },
	[C(MI2S_CODEC_TX_S)]		= { O(SH2_OWN_APPS1), B(14) },
	[C(MIDI)]			= { O(SH2_OWN_APPS1), B(22) },
	[C(SDAC)]			= { O(SH2_OWN_APPS1), B(26) },
	[C(VFE)]			= { O(SH2_OWN_APPS1), B(8) },
	[C(VFE_CAMIF)]			= { O(SH2_OWN_APPS1), B(8) },
	[C(VFE_MDC)]			= { O(SH2_OWN_APPS1), B(8) },
	[C(VFE_P)]			= { O(SH2_OWN_APPS1), B(8) },

	[C(GRP_3D)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(GRP_3D_P)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(GRP_3D_SRC)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(IMEM)]			= { O(SH2_OWN_APPS2), B(0) },
	[C(MDP_LCDC_PAD_PCLK)]		= { O(SH2_OWN_APPS2), B(4) },
	[C(MDP_LCDC_PCLK)]		= { O(SH2_OWN_APPS2), B(4) },
	[C(MDP_P)]			= { O(SH2_OWN_APPS2), B(4) },
	[C(MDP_VSYNC)]			= { O(SH2_OWN_APPS2), B(28) },
	[C(TSIF_REF)]			= { O(SH2_OWN_APPS2), B(5) },
	[C(TSIF_P)]			= { O(SH2_OWN_APPS2), B(5) },
	[C(TV)]				= { O(SH2_OWN_APPS2), B(2) },
	[C(TV_DAC)]			= { O(SH2_OWN_APPS2), B(2) },
	[C(TV_ENC)]			= { O(SH2_OWN_APPS2), B(2) },

	[C(EMDH)]			= { O(SH2_OWN_ROW1), B(7) },
	[C(EMDH_P)]			= { O(SH2_OWN_ROW1), B(7) },
	[C(I2C)]			= { O(SH2_OWN_ROW1), B(11) },
	[C(I2C_2)]			= { O(SH2_OWN_ROW1), B(12) },
	[C(MDC)]			= { O(SH2_OWN_ROW1), B(17) },
	[C(PMDH)]			= { O(SH2_OWN_ROW1), B(19) },
	[C(PMDH_P)]			= { O(SH2_OWN_ROW1), B(19) },
	[C(SDC1)]			= { O(SH2_OWN_ROW1), B(23) },
	[C(SDC1_P)]			= { O(SH2_OWN_ROW1), B(23) },
	[C(SDC2)]			= { O(SH2_OWN_ROW1), B(25) },
	[C(SDC2_P)]			= { O(SH2_OWN_ROW1), B(25) },
	[C(SDC3)]			= { O(SH2_OWN_ROW1), B(27) },
	[C(SDC3_P)]			= { O(SH2_OWN_ROW1), B(27) },
	[C(SDC4)]			= { O(SH2_OWN_ROW1), B(29) },
	[C(SDC4_P)]			= { O(SH2_OWN_ROW1), B(29) },
	[C(UART2)]			= { O(SH2_OWN_ROW1), B(0) },
	[C(USB_HS2)]			= { O(SH2_OWN_ROW1), B(2) },
	[C(USB_HS2_CORE)]		= { O(SH2_OWN_ROW1), B(2) },
	[C(USB_HS2_P)]			= { O(SH2_OWN_ROW1), B(2) },
	[C(USB_HS3)]			= { O(SH2_OWN_ROW1), B(4) },
	[C(USB_HS3_CORE)]		= { O(SH2_OWN_ROW1), B(4) },
	[C(USB_HS3_P)]			= { O(SH2_OWN_ROW1), B(4) },

	[C(QUP_I2C)]			= { O(SH2_OWN_ROW2), B(3) },
	[C(SPI)]			= { O(SH2_OWN_ROW2), B(1) },
	[C(SPI_P)]			= { O(SH2_OWN_ROW2), B(1) },
	[C(UART1)]			= { O(SH2_OWN_ROW2), B(9) },
	[C(UART1DM)]			= { O(SH2_OWN_ROW2), B(6) },
	[C(UART1DM_P)]			= { O(SH2_OWN_ROW2), B(6) },
	[C(UART2DM)]			= { O(SH2_OWN_ROW2), B(8) },
	[C(UART2DM_P)]			= { O(SH2_OWN_ROW2), B(8) },
	[C(USB_HS)]			= { O(SH2_OWN_ROW2), B(11) },
	[C(USB_HS_CORE)]		= { O(SH2_OWN_ROW2), B(11) },
	[C(USB_HS_SRC)]			= { O(SH2_OWN_ROW2), B(11) },
	[C(USB_HS_P)]			= { O(SH2_OWN_ROW2), B(11) },

	[C(CAM_M)]			= { O(SH2_OWN_APPS3), B(6) },
	[C(CAMIF_PAD_P)]		= { O(SH2_OWN_APPS3), B(6) },
	[C(CSI0)]			= { O(SH2_OWN_APPS3), B(11) },
	[C(CSI0_VFE)]			= { O(SH2_OWN_APPS3), B(11) },
	[C(CSI0_P)]			= { O(SH2_OWN_APPS3), B(11) },
	[C(MDP)]			= { O(SH2_OWN_APPS3), B(0) },
	[C(MFC)]			= { O(SH2_OWN_APPS3), B(2) },
	[C(MFC_DIV2)]			= { O(SH2_OWN_APPS3), B(2) },
	[C(MFC_P)]			= { O(SH2_OWN_APPS3), B(2) },
	[C(VPE)]			= { O(SH2_OWN_APPS3), B(4) },

	[C(ADM)]			= { O(SH2_OWN_GLBL), B(8) },
	[C(AXI_ROTATOR)]		= { O(SH2_OWN_GLBL), B(13) },
	[C(ROTATOR_IMEM)]		= { O(SH2_OWN_GLBL), B(13) },
	[C(ROTATOR_P)]			= { O(SH2_OWN_GLBL), B(13) },
};

struct clk_ops * __init clk_7x30_is_local(uint32_t id)
{
	uint32_t local, bit = ownership_map[id].bit;
	uint32_t *reg = ownership_map[id].reg;

	BUG_ON(id >= ARRAY_SIZE(ownership_map) || !reg);

	local = *reg & bit;
	return local ? &clk_ops_7x30 : NULL;
}

static struct reg_init {
	void *reg;
	uint32_t mask;
	uint32_t val;
} ri_list[] __initdata = {
	/* Enable UMDX_P clock. Known to causes issues, so never turn off. */
	{REG(GLBL_CLK_ENA_2_SC), B(2), B(2)},

	{REG(EMDH_NS), BM(18, 17) , BVAL(18, 17, 0x3)}, /* RX div = div-4. */
	{REG(PMDH_NS), BM(18, 17), BVAL(18, 17, 0x3)}, /* RX div = div-4. */
	/* MI2S_CODEC_RX_S src = MI2S_CODEC_RX_M. */
	{REG(MI2S_RX_NS), B(14), 0x0},
	/* MI2S_CODEC_TX_S src = MI2S_CODEC_TX_M. */
	{REG(MI2S_TX_NS), B(14), 0x0},
	{REG(MI2S_NS), B(14), 0x0}, /* MI2S_S src = MI2S_M. */
	/* Allow DSP to decide the LPA CORE src. */
	{REG(LPA_CORE_CLK_MA0), B(0), B(0)},
	{REG(LPA_CORE_CLK_MA2), B(0), B(0)},
	{REG(0x02EC), 0xF, 0xD}, /* MI2S_CODEC_RX_S div = div-8. */
	{REG(0x02F0), 0xF, 0xD}, /* MI2S_CODEC_TX_S div = div-8. */
	{REG(0x02E4), 0xF, 0x3}, /* MI2S_S div = div-4. */
	{REG(MDC_NS), 0x3, 0x3}, /* MDC src = external MDH src. */
	{REG(SDAC_NS), BM(15, 14), 0x0}, /* SDAC div = div-1. */
	/* Disable sources TCXO/5 & TCXO/6. UART1 src = TCXO*/
	{REG(UART_NS), BM(26, 25) | BM(2, 0), 0x0},
	{REG(MDP_VSYNC_REG), 0xC, 0x4}, /* MDP VSYNC src = LPXO. */
	/* HDMI div = div-1, non-inverted. tv_enc_src = tv_clk_src */
	{REG(HDMI_NS), 0x7, 0x0},
	{REG(TV_NS), BM(15, 14), 0x0}, /* tv_clk_src_div2 = div-1 */

	/* USBH core clocks src = USB_HS_SRC. */
	{REG(USBH_NS), B(15), B(15)},
	{REG(USBH2_NS), B(6), B(6)},
	{REG(USBH3_NS), B(6), B(6)},
};

#define set_1rate(clk) \
	soc_clk_set_rate(C(clk), clk_local_tbl[C(clk)].freq_tbl->freq_hz)
__init int clk_7x30_init(void)
{
	int i;
	uint32_t val;

	cache_ownership();
	print_ownership();

	/* When we have no local clock control, the rest of the code in this
	 * function is a NOP since writes to shadow regions that we don't own
	 * are ignored. */

	/* Disable all the child clocks of USB_HS_SRC. This needs to be done
	 * before the register init loop since it changes the source of the
	 * USB HS core clocks. */
	for (i = 0; chld_usb_src[i] != C(NONE); i++)
		_soc_clk_disable(chld_usb_src[i]);

	soc_clk_set_rate(C(USB_HS_SRC), clk_tbl_usb[0].freq_hz);

	for (i = 0; i < ARRAY_SIZE(ri_list); i++) {
		val = readl(ri_list[i].reg);
		val &= ~ri_list[i].mask;
		val |= ri_list[i].val;
		writel(val, ri_list[i].reg);
	}

	/* This is just to update the driver data structures. The actual
	 * register set up is taken care of in the register init loop
	 * or is the default value out of reset. */
	set_1rate(I2C);
	set_1rate(I2C_2);
	set_1rate(QUP_I2C);
	set_1rate(UART1);
	set_1rate(UART2);
	set_1rate(LPA_CODEC);

	return 0;
}
