/* arch/arm/mach-msm/clock.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
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
 */

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_H
#define __ARCH_ARM_MACH_MSM_CLOCK_H

#include <linux/list.h>
#include <mach/clk.h>
#include <linux/timer.h>
#if defined (CONFIG_ARCH_MSM7X30)
#include "clock-pcom.h"
#include "clock-7x30.h"
#endif

#define CLKFLAG_USE_MAX_TO_SET		(0x00000001)
#define CLKFLAG_AUTO_OFF		(0x00000002)
#define CLKFLAG_USE_MIN_TO_SET		(0x00000004)
#define CLKFLAG_SHARED			(0x00000008)
#define CLKFLAG_HANDLE			(0x00000010)
#define CLKFLAG_DEFER			(0x00000020)

#define CLKFLAG_ARCH_MSM7X00A		(0x00010000)
#define CLKFLAG_ARCH_QSD8X50		(0x00020000)
#define CLKFLAG_ARCH_ALL		(0xffff0000)

struct clk_ops {
	int (*enable)(unsigned id);
	void (*disable)(unsigned id);
	void (*auto_off)(unsigned id);
	int (*reset)(unsigned id, enum clk_reset_action action);
	int (*set_rate)(unsigned id, unsigned rate);
	int (*set_min_rate)(unsigned id, unsigned rate);
	int (*set_max_rate)(unsigned id, unsigned rate);
	int (*set_flags)(unsigned id, unsigned flags);
	unsigned (*get_rate)(unsigned id);
	unsigned (*is_enabled)(unsigned id);
	long (*round_rate)(unsigned id, unsigned rate);
};

struct clk {
	uint32_t id;
	uint32_t remote_id;
	uint32_t count;
	uint32_t flags;
	const char *name;
	struct clk_ops *ops;
	struct hlist_node list;
	struct device *dev;
	struct hlist_head handles;
	struct timer_list defer_clk_timer;
};

struct clk_handle {
	struct clk clk;
	struct clk *source;
	unsigned long rate;
};

#define A11S_CLK_CNTL_ADDR		(MSM_CSR_BASE + 0x100)
#define A11S_CLK_SEL_ADDR		(MSM_CSR_BASE + 0x104)
#define A11S_VDD_SVS_PLEVEL_ADDR	(MSM_CSR_BASE + 0x124)

/* clock IDs used by the modem processor */

#define ACPU_CLK		0   /* Applications processor clock */
#define ADM_CLK			1   /* Applications data mover clock */
#define ADSP_CLK		2   /* ADSP clock */
#define EBI1_CLK		3   /* External bus interface 1 clock */
#define EBI2_CLK		4   /* External bus interface 2 clock */
#define ECODEC_CLK		5   /* External CODEC clock */
#define EMDH_CLK		6   /* External MDDI host clock */
#define GP_CLK			7   /* General purpose clock */
#define GRP_CLK			8   /* Graphics clock */
#define I2C_CLK			9   /* I2C clock */
#define ICODEC_RX_CLK		10  /* Internal CODEX RX clock */
#define ICODEC_TX_CLK		11  /* Internal CODEX TX clock */
#define IMEM_CLK		12  /* Internal graphics memory clock */
#define MDC_CLK			13  /* MDDI client clock */
#define MDP_CLK			14  /* Mobile display processor clock */
#define PBUS_CLK		15  /* Peripheral bus clock */
#define PCM_CLK			16  /* PCM clock */
#define PMDH_CLK		17  /* Primary MDDI host clock */
#define SDAC_CLK		18  /* Stereo DAC clock */
#define SDC1_CLK		19  /* Secure Digital Card clocks */
#define SDC1_PCLK		20
#define SDC2_CLK		21
#define SDC2_PCLK		22
#define SDC3_CLK		23
#define SDC3_PCLK		24
#define SDC4_CLK		25
#define SDC4_PCLK		26
#define TSIF_CLK		27  /* Transport Stream Interface clocks */
#define TSIF_REF_CLK		28
#define TV_DAC_CLK		29  /* TV clocks */
#define TV_ENC_CLK		30
#define UART1_CLK		31  /* UART clocks */
#define UART2_CLK		32
#define UART3_CLK		33
#define UART1DM_CLK		34
#define UART2DM_CLK		35
#define USB_HS_CLK		36  /* High speed USB core clock */
#define USB_HS_PCLK		37  /* High speed USB pbus clock */
#define USB_OTG_CLK		38  /* Full speed USB clock */
#define VDC_CLK			39  /* Video controller clock */
#if defined(CONFIG_ARCH_QSD8X50) || defined(CONFIG_ARCH_MSM7227) || defined(CONFIG_ARCH_MSM7X00A) || defined(CONFIG_ARCH_MSM7X30)
#define VFE_MDC_CLK		40  /* VFE MDDI client clock */
#define VFE_CLK			41  /* Camera / Video Front End clock */
#else
/* older AMSS versions had these flipped */
#define VFE_MDC_CLK		41  /* VFE MDDI client clock */
#define VFE_CLK			40  /* Camera / Video Front End clock */
#endif
#define LCDC_PCLK		42
#define LCDC_PAD_PCLK		43
#define MDP_VSYNC_CLK		44
#define SPI_CLK			45
#define VFE_AXI_CLK		46
#define USB_HS2_CLK		47  /* High speed USB 2 core clock */
#define USB_HS2_PCLK		48  /* High speed USB 2 pbus clock */
#define USB_HS3_CLK		49  /* High speed USB 3 core clock */
#define USB_HS3_PCLK		50  /* High speed USB 3 pbus clock */
#define GRP_PCLK		51  /* Graphics pbus clock */
#define USB_PHY_CLK		52  /* USB PHY clock */
#define USB_HS_CORE_CLK		53  /* High speed USB 1 core clock */
#define USB_HS2_CORE_CLK	54  /* High speed USB 2 core clock */
#define USB_HS3_CORE_CLK	55  /* High speed USB 3 core clock */
#define CAM_MCLK_CLK		56
#define CAMIF_PAD_PCLK		57
#define GRP_2D_CLK		58
#define GRP_2D_PCLK		59
#define I2S_CLK			60
#define JPEG_CLK		61
#define JPEG_PCLK		62
#define LPA_CODEC_CLK		63
#define LPA_CORE_CLK		64
#define LPA_PCLK		65
#define MDC_IO_CLK		66
#define MDC_PCLK		67
#define MFC_CLK			68
#define MFC_DIV2_CLK		69
#define MFC_PCLK		70
#define QUP_I2C_CLK		71
#define ROTATOR_IMEM_CLK	72
#define ROTATOR_PCLK		73
#define VFE_CAMIF_CLK		74
#define VFE_PCLK		75
#define VPE_CLK			76
#define I2C_2_CLK		77
#define MI2S_CODEC_RX_SCLK	78
#define MI2S_CODEC_RX_MCLK	79
#define MI2S_CODEC_TX_SCLK	80
#define MI2S_CODEC_TX_MCLK	81
#define PMDH_PCLK		82
#define EMDH_PCLK		83
#define SPI_PCLK		84
#define TSIF_PCLK		85
#define MDP_PCLK		86
#define SDAC_MCLK		87
#define MI2S_HDMI_CLK		88
#define MI2S_HDMI_MCLK		89
#define AXI_ROTATOR_CLK	90
#define HDMI_CLK		91
#define CSI0_CLK		92
#define CSI0_VFE_CLK		93
#define CSI0_PCLK		94
#define CSI1_CLK		95
#define CSI1_VFE_CLK		96
#define CSI1_PCLK		97
#define GSBI_CLK		98
#define GSBI_PCLK		99
#define NR_CLKS			100

extern struct clk msm_clocks[];

void clk_enter_sleep(int from_idle);
void clk_exit_sleep(void);

#if defined (CONFIG_ARCH_MSM7X30)
enum {
	PLL_0 = 0,
	PLL_1,
	PLL_2,
	PLL_3,
	PLL_4,
	PLL_5,
	PLL_6,
	NUM_PLL
};

enum clkvote_client {
	CLKVOTE_ACPUCLK = 0,
	CLKVOTE_PMQOS,
	CLKVOTE_MAX,
};

unsigned long clk_get_max_axi_khz(void);
#endif

#endif

