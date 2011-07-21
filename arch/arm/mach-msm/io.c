/* arch/arm/mach-msm/io.c
 *
 * MSM7K io support
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>

#include <mach/hardware.h>
#include <asm/page.h>
#include <mach/msm_iomap.h>
#include <asm/mach/map.h>

#include <mach/board.h>

#define MSM_DEVICE(name) { \
		.virtual = (unsigned long) MSM_##name##_BASE, \
		.pfn = __phys_to_pfn(MSM_##name##_PHYS), \
		.length = MSM_##name##_SIZE, \
		.type = MT_DEVICE_NONSHARED, \
	 }

#ifdef CONFIG_ARCH_MSM8X60
/* msm_shared_ram_phys default value of 0x00100000 is the most common value
 * and should work as-is for any target without stacked memory.
 */
unsigned int msm_shared_ram_phys = 0x00100000;

static void msm_map_io(struct map_desc *io_desc, int size)
{
	int i;

	BUG_ON(!size);
	for (i = 0; i < size; i++)
		if (io_desc[i].virtual == (unsigned long)MSM_SHARED_RAM_BASE)
			io_desc[i].pfn = __phys_to_pfn(msm_shared_ram_phys);

	iotable_init(io_desc, size);
}

static struct map_desc msm8x60_io_desc[] __initdata = {
	MSM_DEVICE(QGIC_DIST),
	MSM_DEVICE(QGIC_CPU),
	MSM_DEVICE(TMR),
	MSM_DEVICE(TMR0),
	MSM_DEVICE(RPM_MPM),
	MSM_DEVICE(ACC),
	MSM_DEVICE(ACC0),
	MSM_DEVICE(ACC1),
	MSM_DEVICE(SAW0),
	MSM_DEVICE(SAW1),
	MSM_DEVICE(GCC),
	MSM_DEVICE(TLMM),
	MSM_DEVICE(DMOV_ADM0),
	MSM_DEVICE(DMOV_ADM1),
	MSM_DEVICE(SCPLL),
	MSM_DEVICE(RPM),
	MSM_DEVICE(CLK_CTL),
	MSM_DEVICE(MMSS_CLK_CTL),
	MSM_DEVICE(LPASS_CLK_CTL),
	MSM_DEVICE(TCSR),
	MSM_DEVICE(IMEM),
#ifdef CONFIG_MSM_DEBUG_UART
	MSM_DEVICE(DEBUG_UART),
#endif
	MSM_DEVICE(SIC_NON_SECURE),
	{
		.virtual =  (unsigned long) MSM_SHARED_RAM_BASE,
		.length =   MSM_SHARED_RAM_SIZE,
		.type =     MT_DEVICE,
	},
	MSM_DEVICE(QFPROM),
	MSM_DEVICE(EBI1_CH0),
};

void __init msm_map_msm8x60_io(void)
{
	msm_map_io(msm8x60_io_desc, ARRAY_SIZE(msm8x60_io_desc));
}
#else
static struct map_desc msm_io_desc[] __initdata = {
	MSM_DEVICE(VIC),
	MSM_DEVICE(CSR),
#ifdef CONFIG_ARCH_MSM7X30
	MSM_DEVICE(TMR),
#else
	MSM_DEVICE(GPT),
#endif
	MSM_DEVICE(DMOV),
	MSM_DEVICE(GPIO1),
	MSM_DEVICE(GPIO2),
	MSM_DEVICE(CLK_CTL),
#ifdef CONFIG_ARCH_MSM7X30
	MSM_DEVICE(CLK_CTL_SH2),
#endif
#ifdef CONFIG_ARCH_MSM7227
	MSM_DEVICE(TGPIO1),
#endif
#ifdef CONFIG_ARCH_QSD8X50
	MSM_DEVICE(SIRC),
	MSM_DEVICE(SCPLL),
#endif
	MSM_DEVICE(AD5),
	MSM_DEVICE(MDC),
#ifdef CONFIG_ARCH_MSM7X30
	MSM_DEVICE(ACC),
	MSM_DEVICE(SAW),
	MSM_DEVICE(GCC),
	MSM_DEVICE(TCSR),
#endif
#ifdef CONFIG_MSM_DEBUG_UART
	MSM_DEVICE(DEBUG_UART),
#endif
#ifdef CONFIG_ARCH_QSD8X50
	MSM_DEVICE(TCSR),
#endif
#ifdef CONFIG_CACHE_L2X0
	{
		.virtual =  (unsigned long) MSM_L2CC_BASE,
		.pfn =      __phys_to_pfn(MSM_L2CC_PHYS),
		.length =   MSM_L2CC_SIZE,
		.type =     MT_DEVICE,
	},
#endif
	{
		.virtual =  (unsigned long) MSM_SHARED_RAM_BASE,
		.pfn =      __phys_to_pfn(MSM_SHARED_RAM_PHYS),
		.length =   MSM_SHARED_RAM_SIZE,
		.type =     MT_DEVICE,
	},
	MSM_DEVICE(SDC2),
};

void __init msm_map_common_io(void)
{
#ifdef CONFIG_ARCH_MSM_ARM11
	/* Make sure the peripheral register window is closed, since
	 * we will use PTE flags (TEX[1]=1,B=0,C=1) to determine which
	 * pages are peripheral interface or not.
	 */
	asm("mcr p15, 0, %0, c15, c2, 4" : : "r" (0));
#endif
#ifdef CONFIG_ARCH_QSD8X50
	unsigned int unused;

	/* The bootloader may not have done it, so disable predecode repair
	 * cache for thumb2 (DPRC, set bit 4 in PVR0F2) due to a bug.
	 */
	asm volatile ("mrc p15, 0, %0, c15, c15, 2\n\t"
		      "orr %0, %0, #0x10\n\t"
		      "mcr p15, 0, %0, c15, c15, 2"
		      : "=&r" (unused));
#endif
#ifdef CONFIG_ARCH_QSD8X50
	/* clear out EFSR and ADFSR on boot */
	asm volatile ("mcr p15, 7, %0, c15, c0, 1\n\t"
		      "mcr p15, 0, %0, c5, c1, 0"
		      : : "r" (0));
#endif

	iotable_init(msm_io_desc, ARRAY_SIZE(msm_io_desc));
}
#endif /* CONFIG_ARCH_MSM8X60 */

void __iomem *
__msm_ioremap(unsigned long phys_addr, size_t size, unsigned int mtype)
{
	if (mtype == MT_DEVICE) {
#ifdef CONFIG_ARCH_MSM7227
		/* The peripherals in the 88000000 - D0000000 range
		 * are only accessable by type MT_DEVICE_NONSHARED.
		 * Adjust mtype as necessary to make this "just work."
		 */
		 if ((phys_addr >= 0x88000000) && (phys_addr < 0xD0000000))
			mtype = MT_DEVICE_NONSHARED;
#else
		/* The peripherals in the 88000000 - F0000000 range
		 * are only accessable by type MT_DEVICE_NONSHARED.
		 * Adjust mtype as necessary to make this "just work."
		 */
		if ((phys_addr >= 0x88000000) && (phys_addr < 0xF0000000))
			mtype = MT_DEVICE_NONSHARED;
#endif
	}

	return __arm_ioremap(phys_addr, size, mtype);
}

EXPORT_SYMBOL(__msm_ioremap);
