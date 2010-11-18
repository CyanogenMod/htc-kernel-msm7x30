/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _GSL_DRIVER_H
#define _GSL_DRIVER_H

#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <asm/atomic.h>

#include "kgsl_device.h"
#include "kgsl_sharedmem.h"

#define DRIVER_NAME "kgsl"
#define CHIP_REV_251 0x020501

/* Flags to control whether to flush or invalidate a cached memory range */
#define KGSL_CACHE_INV		0x00000000
#define KGSL_CACHE_CLEAN	0x00000001
#define KGSL_CACHE_FLUSH	0x00000002

#define KGSL_CACHE_USER_ADDR	0x00000010
#define KGSL_CACHE_VMALLOC_ADDR	0x00000020

enum kgsl_clk_freq {
	KGSL_AXI_HIGH_2D = 0,
	KGSL_AXI_HIGH_3D = 1,
	KGSL_2D_MIN_FREQ = 2,
	KGSL_2D_MAX_FREQ = 3,
	KGSL_3D_MIN_FREQ = 4,
	KGSL_3D_MAX_FREQ = 5,
	KGSL_NUM_FREQ
};

struct kgsl_driver {
	struct miscdevice misc;
	struct platform_device *pdev;
	atomic_t open_count;
	struct mutex mutex;

	int yamato_interrupt_num;
	int yamato_have_irq;
	int g12_interrupt_num;
	int g12_have_irq;

	struct clk *g12_grp_pclk;
	struct clk *g12_grp_clk;
	struct clk *yamato_grp_pclk;
	struct clk *yamato_grp_clk;
	struct clk *yamato_grp_src_clk;
	struct clk *imem_clk;
	unsigned int power_flags;
	unsigned int is_suspended;
	unsigned int clk_freq[KGSL_NUM_FREQ];

	struct kgsl_devconfig g12_config;
	struct kgsl_devconfig yamato_config;

	uint32_t flags_debug;

	struct kgsl_sharedmem shmem;
	struct kgsl_device yamato_device;
	struct kgsl_device g12_device;

	struct list_head client_list;

	/* Global list of pagetables */
	struct list_head pagetable_list;
	/* Mutex for accessing the pagetable list */
	struct mutex pt_mutex;
};

extern struct kgsl_driver kgsl_driver;

struct kgsl_mem_entry {
	struct kgsl_memdesc memdesc;
	struct file *pmem_file;
	struct list_head list;
	struct list_head free_list;
	uint32_t free_timestamp;
	/* back pointer to private structure under whose context this
	* allocation is made */
	struct kgsl_file_private *priv;
};

enum kgsl_status {
	KGSL_SUCCESS = 0,
	KGSL_FAILURE = 1
};

#define KGSL_TRUE 1
#define KGSL_FALSE 0

#define KGSL_G12_PRE_HWACCESS() \
while (1) { \
	if (kgsl_driver.g12_device.hwaccess_blocked == KGSL_FALSE) { \
		break; \
	} \
	if (kgsl_driver.is_suspended != KGSL_TRUE) { \
		kgsl_g12_wake(&kgsl_driver.g12_device); \
		break; \
	} \
	mutex_unlock(&kgsl_driver.mutex); \
	wait_for_completion(&kgsl_driver.g12_device.hwaccess_gate); \
	mutex_lock(&kgsl_driver.mutex); \
}
#define KGSL_G12_POST_HWACCESS() mutex_unlock(&kgsl_driver.mutex)

#define KGSL_PRE_HWACCESS() \
while (1) { \
	mutex_lock(&kgsl_driver.mutex); \
	if (kgsl_driver.yamato_device.hwaccess_blocked == KGSL_FALSE) { \
		break; \
	} \
	if (kgsl_driver.is_suspended != KGSL_TRUE) { \
		kgsl_yamato_wake(&kgsl_driver.yamato_device); \
		break; \
	} \
	mutex_unlock(&kgsl_driver.mutex); \
	wait_for_completion(&kgsl_driver.yamato_device.hwaccess_gate); \
}

#define KGSL_POST_HWACCESS() \
	mutex_unlock(&kgsl_driver.mutex)

#ifdef CONFIG_MSM_KGSL_MMU_PAGE_FAULT
#define MMU_CONFIG 2
#else
#define MMU_CONFIG 1
#endif

void kgsl_remove_mem_entry(struct kgsl_mem_entry *entry, bool preserve);

int kgsl_pwrctrl(unsigned int pwrflag);
int kgsl_yamato_sleep(struct kgsl_device *device, const int idle);
int kgsl_g12_last_release_locked(void);
int kgsl_g12_first_open_locked(void);
int kgsl_g12_sleep(struct kgsl_device *device, const int idle);
int kgsl_g12_wake(struct kgsl_device *device);
int kgsl_idle(struct kgsl_device *device, unsigned int timeout);
int kgsl_setstate(struct kgsl_device *device, uint32_t flags);
int kgsl_regread(struct kgsl_device *device, unsigned int offsetwords,
			unsigned int *value);
int kgsl_regwrite(struct kgsl_device *device, unsigned int offsetwords,
			unsigned int value);

#ifdef CONFIG_MSM_KGSL_DRM
extern int kgsl_drm_init(struct platform_device *dev);
extern void kgsl_drm_exit(void);
extern void kgsl_gpu_mem_flush(void);
#else
static inline int kgsl_drm_init(struct platform_device *dev)
{
	return 0;
}

static inline void kgsl_drm_exit(void)
{
}
#endif

#endif /* _GSL_DRIVER_H */
