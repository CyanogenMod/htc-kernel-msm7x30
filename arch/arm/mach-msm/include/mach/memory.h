/* arch/arm/mach-msm/include/mach/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
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

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* physical offset of RAM */
#if defined(CONFIG_ARCH_QSD8X50)
#define PHYS_OFFSET		UL(0x20000000)
#elif defined(CONFIG_ARCH_MSM7225)
#define PHYS_OFFSET		UL(0x02E00000)
#elif defined(CONFIG_ARCH_MSM7227)
#define PHYS_OFFSET		UL(0x12C00000)
#elif defined(CONFIG_ARCH_MSM7230) && (defined(CONFIG_MACH_VIVO) || defined(CONFIG_MACH_SAGA) || defined(CONFIG_MACH_FLYER) || defined(CONFIG_MACH_ICON) || defined(CONFIG_MACH_EXPRESS_KT))
#define PHYS_OFFSET		UL(0x04400000)
#elif defined(CONFIG_ARCH_MSM7230)
#define PHYS_OFFSET		UL(0x04000000)
#elif defined(CONFIG_ARCH_MSM7630) && defined(CONFIG_MACH_MECHA)
#define PHYS_OFFSET		UL(0x05200000)
#elif defined(CONFIG_ARCH_MSM7630) && defined(CONFIG_MACH_SPEEDY)
#define PHYS_OFFSET		UL(0x04000000)
#elif defined(CONFIG_ARCH_MSM7630) && (defined(CONFIG_MACH_VIVOW) || defined(CONFIG_MACH_EXPRESS) || defined(CONFIG_MACH_KINGDOM))
#define PHYS_OFFSET		UL(0x05000000)
#elif defined(CONFIG_ARCH_MSM7630)
#define PHYS_OFFSET		UL(0x04A00000)
#elif defined(CONFIG_ARCH_MSM8X60)
#define PHYS_OFFSET UL(CONFIG_PHYS_OFFSET)
#else
#define PHYS_OFFSET		UL(0x10000000)
#endif

#define MAX_PHYSMEM_BITS 32
#define SECTION_SIZE_BITS 28

/* Certain configurations of MSM7x30 have multiple memory banks.
*  One or more of these banks can contain holes in the memory map as well.
*  These macros define appropriate conversion routines between the physical
*  and virtual address domains for supporting these configurations using
*  SPARSEMEM and a 3G/1G VM split.
*/

#if defined(CONFIG_ARCH_MSM7X30)

#define EBI0_PHYS_OFFSET PHYS_OFFSET
#define EBI0_PAGE_OFFSET PAGE_OFFSET
#define EBI0_SIZE 0x10000000

#define EBI1_PHYS_OFFSET 0x40000000
#define EBI1_PAGE_OFFSET (EBI0_PAGE_OFFSET + EBI0_SIZE)

#if (defined(CONFIG_SPARSEMEM) && defined(CONFIG_VMSPLIT_3G))

#define __phys_to_virt(phys)				\
	((phys) >= EBI1_PHYS_OFFSET ?			\
	(phys) - EBI1_PHYS_OFFSET + EBI1_PAGE_OFFSET :	\
	(phys) - EBI0_PHYS_OFFSET + EBI0_PAGE_OFFSET)

#define __virt_to_phys(virt)				\
	((virt) >= EBI1_PAGE_OFFSET ?			\
	(virt) - EBI1_PAGE_OFFSET + EBI1_PHYS_OFFSET :	\
	(virt) - EBI0_PAGE_OFFSET + EBI0_PHYS_OFFSET)

#endif

#endif

#define HAS_ARCH_IO_REMAP_PFN_RANGE

#define CONSISTENT_DMA_SIZE (4*SZ_1M)

#ifndef __ASSEMBLY__
void *alloc_bootmem_aligned(unsigned long size, unsigned long alignment);
void clean_and_invalidate_caches(unsigned long, unsigned long, unsigned long);
void clean_caches(unsigned long, unsigned long, unsigned long);
void invalidate_caches(unsigned long, unsigned long, unsigned long);

#ifdef CONFIG_ARCH_MSM_ARM11
void write_to_strongly_ordered_memory(void);

#include <asm/mach-types.h>

#if defined(CONFIG_ARCH_MSM7227)
#define arch_barrier_extra() do \
	{ \
		write_to_strongly_ordered_memory(); \
	} while (0)
#else
#define arch_barrier_extra() do {} while (0)
#endif

#ifdef CONFIG_CACHE_L2X0
extern void l2x0_cache_sync(void);
extern void l2x0_cache_flush_all(void);
#define finish_arch_switch(prev)     do { l2x0_cache_sync(); } while (0)
#endif

#endif
#endif

#if defined CONFIG_ARCH_MSM_SCORPION || defined CONFIG_ARCH_MSM_SCORPIONMP
#define arch_has_speculative_dfetch()  1
#else
#define arch_has_speculative_dfetch()  0
#endif

#endif

