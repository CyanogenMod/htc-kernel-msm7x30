/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <mach/clk.h>
#include <mach/dal_axi.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/android_pmem.h>
#include <linux/pm_qos_params.h>
#include <linux/highmem.h>
#include <linux/vmalloc.h>
#include <asm/cacheflush.h>

#include <linux/delay.h>
#include <asm/atomic.h>
#include <mach/internal_power_rail.h>

#include "kgsl.h"
#include "kgsl_drawctxt.h"
#include "kgsl_ringbuffer.h"
#include "kgsl_cmdstream.h"

#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_cmdstream.h"

#include "kgsl_log.h"
//#include "kgsl_drm.h"

#define KGSL_MAX_PRESERVED_BUFFERS		10
#define KGSL_MAX_SIZE_OF_PRESERVED_BUFFER	0x10000

struct kgsl_file_private {
	struct list_head list;
	struct list_head mem_list;
	uint32_t yamato_ctxt_id_mask;
	uint32_t g12_ctxt_id_mask;
	struct kgsl_pagetable *pagetable;
	unsigned long vmalloc_size;
	struct list_head preserve_entry_list;
	int preserve_list_size;
};

static void kgsl_put_phys_file(struct file *file);

static int kgsl_runpending(struct kgsl_device *device)
{
	if (device->flags & KGSL_FLAGS_INITIALIZED)
		kgsl_cmdstream_memqueue_drain(device);
	return 0;
}

#ifdef CONFIG_MSM_KGSL_MMU
static long kgsl_cache_range_op(unsigned long addr, int size,
					unsigned int flags)
{
#ifdef CONFIG_OUTER_CACHE
	unsigned long end;
#endif
	BUG_ON(addr & (KGSL_PAGESIZE - 1));
	BUG_ON(size & (KGSL_PAGESIZE - 1));

	if (flags & KGSL_CACHE_FLUSH)
		dmac_flush_range((const void *)addr,
				(const void *)(addr + size));
	else
		if (flags & KGSL_CACHE_CLEAN)
			dmac_clean_range((const void *)addr,
					(const void *)(addr + size));
		else
			dmac_inv_range((const void *)addr,
					(const void *)(addr + size));

#ifdef CONFIG_OUTER_CACHE
	for (end = addr; end < (addr + size); end += KGSL_PAGESIZE) {
		pte_t *pte_ptr, pte;
		unsigned long physaddr;
		if (flags & KGSL_CACHE_VMALLOC_ADDR)
			physaddr = vmalloc_to_pfn((void *)end);
		else
			if (flags & KGSL_CACHE_USER_ADDR) {
				pte_ptr = kgsl_get_pte_from_vaddr(end);
				if (!pte_ptr)
					return -EINVAL;
				pte = *pte_ptr;
				physaddr = pte_pfn(pte);
				pte_unmap(pte_ptr);
			} else
				return -EINVAL;

		physaddr <<= PAGE_SHIFT;
		if (flags & KGSL_CACHE_FLUSH)
			outer_flush_range(physaddr, physaddr + KGSL_PAGESIZE);
		else
			if (flags & KGSL_CACHE_CLEAN)
				outer_clean_range(physaddr,
					physaddr + KGSL_PAGESIZE);
			else
				outer_inv_range(physaddr,
					physaddr + KGSL_PAGESIZE);
	}
#endif
	return 0;
}

static long kgsl_clean_cache_all(struct kgsl_file_private *private)
{
	int result = 0;
	struct kgsl_mem_entry *entry = NULL;

	kgsl_runpending(&kgsl_driver.yamato_device);
	kgsl_runpending(&kgsl_driver.g12_device);

	list_for_each_entry(entry, &private->mem_list, list) {
		if (KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH & entry->memdesc.priv) {
			result =
			    kgsl_cache_range_op((unsigned long)entry->
						   memdesc.hostptr,
						   entry->memdesc.size,
				KGSL_CACHE_CLEAN | KGSL_CACHE_USER_ADDR);
			if (result)
				goto done;
		}
	}
done:
	return result;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

/*this is used for logging, so that we can call the dev_printk
 functions without export struct kgsl_driver everywhere*/
struct device *kgsl_driver_getdevnode(void)
{
	BUG_ON(kgsl_driver.pdev == NULL);
	return &kgsl_driver.pdev->dev;
}

int kgsl_regread(struct kgsl_device *device, unsigned int offsetwords,
			unsigned int *value)
{
	int status = -ENXIO;
	if (device->ftbl.device_regread)
		status = device->ftbl.device_regread(device, offsetwords,
					value);

	return status;
}

int kgsl_regwrite(struct kgsl_device *device, unsigned int offsetwords,
			unsigned int value)
{
	int status = -ENXIO;
	if (device->ftbl.device_regwrite)
		status = device->ftbl.device_regwrite(device, offsetwords,
					value);

	return status;
}

int kgsl_setstate(struct kgsl_device *device, uint32_t flags)
{
	int status = -ENXIO;

	if (flags && device->ftbl.device_setstate) {
		status = device->ftbl.device_setstate(device, flags);
		device->mmu.tlb_flags &= ~flags;
	} else
		status = 0;

	return status;
}

int kgsl_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = -ENXIO;

	if (device->ftbl.device_idle)
		status = device->ftbl.device_idle(device, timeout);

	return status;
}


int kgsl_pwrctrl(unsigned int pwrflag)
{
	char tname[TASK_COMM_LEN];

	switch (pwrflag) {
	case KGSL_PWRFLAGS_YAMATO_CLK_OFF:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_YAMATO_CLK_ON) {
			if (kgsl_driver.yamato_grp_pclk)
				clk_disable(kgsl_driver.yamato_grp_pclk);

			clk_disable(kgsl_driver.yamato_grp_clk);
			clk_disable(kgsl_driver.imem_clk);
			if (kgsl_driver.clk_freq[KGSL_3D_MIN_FREQ])
				clk_set_min_rate(kgsl_driver.yamato_grp_src_clk,
					kgsl_driver.clk_freq[KGSL_3D_MIN_FREQ]);
			if (kgsl_driver.clk_freq[KGSL_AXI_HIGH_3D])
				pm_qos_update_requirement(
					PM_QOS_SYSTEM_BUS_FREQ,
					"kgsl_3d", PM_QOS_DEFAULT_VALUE);
			kgsl_driver.power_flags &=
					~(KGSL_PWRFLAGS_YAMATO_CLK_ON);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_YAMATO_CLK_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_YAMATO_CLK_ON:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_YAMATO_CLK_OFF) {
			/* 2d composition doesnt require highest clock */
			if (kgsl_driver.clk_freq[KGSL_AXI_HIGH_3D]) {
				if (strcmp(get_task_comm(tname, current), "SurfaceFlinger")) {
					pm_qos_update_requirement(
						PM_QOS_SYSTEM_BUS_FREQ, "kgsl_3d",
						kgsl_driver.clk_freq[KGSL_AXI_HIGH_3D]);
				}
                        }
			if (kgsl_driver.clk_freq[KGSL_3D_MAX_FREQ]) {
				if (strcmp(get_task_comm(tname, current), "SurfaceFlinger")) {
					clk_set_min_rate(kgsl_driver.yamato_grp_src_clk,
						kgsl_driver.clk_freq[KGSL_3D_MAX_FREQ]);
				}
				else {
					clk_set_min_rate(kgsl_driver.yamato_grp_src_clk,
						kgsl_driver.clk_freq[KGSL_3D_MIN_FREQ]);
				}
                        }

			if (kgsl_driver.yamato_grp_pclk)
				clk_enable(kgsl_driver.yamato_grp_pclk);
			clk_enable(kgsl_driver.yamato_grp_clk);
			clk_enable(kgsl_driver.imem_clk);

			kgsl_driver.power_flags &=
				~(KGSL_PWRFLAGS_YAMATO_CLK_OFF);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_YAMATO_CLK_ON;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_G12_CLK_OFF:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_G12_CLK_ON) {
			if (kgsl_driver.g12_grp_pclk)
				clk_disable(kgsl_driver.g12_grp_pclk);
			if (kgsl_driver.g12_grp_clk != NULL) {
				clk_disable(kgsl_driver.g12_grp_clk);
				if (kgsl_driver.clk_freq[KGSL_2D_MIN_FREQ])
					clk_set_min_rate(
					kgsl_driver.g12_grp_clk,
					kgsl_driver.clk_freq[KGSL_2D_MIN_FREQ]);
			}
			if (kgsl_driver.clk_freq[KGSL_AXI_HIGH_2D])
				pm_qos_update_requirement(
					PM_QOS_SYSTEM_BUS_FREQ,
					"kgsl_2d", PM_QOS_DEFAULT_VALUE);
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_G12_CLK_ON);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_G12_CLK_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_G12_CLK_ON:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_G12_CLK_OFF) {
			if (kgsl_driver.clk_freq[KGSL_AXI_HIGH_2D])
				pm_qos_update_requirement(
					PM_QOS_SYSTEM_BUS_FREQ, "kgsl_2d",
					kgsl_driver.clk_freq[KGSL_AXI_HIGH_2D]);
			if (kgsl_driver.g12_grp_pclk)
				clk_enable(kgsl_driver.g12_grp_pclk);
			if (kgsl_driver.g12_grp_clk != NULL) {
				if (kgsl_driver.clk_freq[KGSL_2D_MAX_FREQ])
					clk_set_min_rate(
					kgsl_driver.g12_grp_clk,
					kgsl_driver.clk_freq[KGSL_2D_MAX_FREQ]);
				clk_enable(kgsl_driver.g12_grp_clk);
			}

			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_G12_CLK_OFF);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_G12_CLK_ON;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_POWER_OFF:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_POWER_ON) {
			internal_pwr_rail_ctl(PWR_RAIL_GRP_CLK, KGSL_FALSE);
			internal_pwr_rail_mode(PWR_RAIL_GRP_CLK,
					PWR_RAIL_CTL_AUTO);
			if (kgsl_driver.g12_device.hwaccess_blocked
				== KGSL_FALSE) {
				internal_pwr_rail_ctl(PWR_RAIL_GRP_2D_CLK,
					KGSL_FALSE);
				internal_pwr_rail_mode(PWR_RAIL_GRP_2D_CLK,
					PWR_RAIL_CTL_AUTO);
			}
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_POWER_ON);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_POWER_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_POWER_ON:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_POWER_OFF) {
			internal_pwr_rail_mode(PWR_RAIL_GRP_CLK,
					PWR_RAIL_CTL_MANUAL);
			internal_pwr_rail_ctl(PWR_RAIL_GRP_CLK, KGSL_TRUE);
			if (kgsl_driver.g12_device.hwaccess_blocked
				== KGSL_FALSE) {
				internal_pwr_rail_mode(PWR_RAIL_GRP_2D_CLK,
					PWR_RAIL_CTL_MANUAL);
				internal_pwr_rail_ctl(PWR_RAIL_GRP_2D_CLK,
					KGSL_TRUE);
			}
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_POWER_OFF);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_POWER_ON;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_YAMATO_IRQ_ON:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_YAMATO_IRQ_OFF) {
			enable_irq(kgsl_driver.yamato_interrupt_num);
			kgsl_driver.power_flags &=
				~(KGSL_PWRFLAGS_YAMATO_IRQ_OFF);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_YAMATO_IRQ_ON;
		}

		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_YAMATO_IRQ_OFF:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_YAMATO_IRQ_ON) {
			disable_irq(kgsl_driver.yamato_interrupt_num);
			kgsl_driver.power_flags &=
				~(KGSL_PWRFLAGS_YAMATO_IRQ_ON);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_YAMATO_IRQ_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_G12_IRQ_ON:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_G12_IRQ_OFF) {
			enable_irq(kgsl_driver.g12_interrupt_num);
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_G12_IRQ_OFF);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_G12_IRQ_ON;
		}

		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_G12_IRQ_OFF:
		if (kgsl_driver.power_flags & KGSL_PWRFLAGS_G12_IRQ_ON) {
			disable_irq(kgsl_driver.g12_interrupt_num);
			kgsl_driver.power_flags &= ~(KGSL_PWRFLAGS_G12_IRQ_ON);
			kgsl_driver.power_flags |= KGSL_PWRFLAGS_G12_IRQ_OFF;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

/*Suspend function*/
static int kgsl_suspend(struct platform_device *dev, pm_message_t state)
{
	mutex_lock(&kgsl_driver.mutex);
	if (kgsl_driver.power_flags != 0) {
		if (kgsl_driver.yamato_device.hwaccess_blocked == KGSL_FALSE)
			kgsl_yamato_suspend(&kgsl_driver.yamato_device);

		if (kgsl_driver.g12_device.hwaccess_blocked == KGSL_FALSE)
			kgsl_g12_suspend(&kgsl_driver.g12_device);

		kgsl_driver.is_suspended = KGSL_TRUE;
	}
	mutex_unlock(&kgsl_driver.mutex);
	return KGSL_SUCCESS;
}

/*Resume function*/
static int kgsl_resume(struct platform_device *dev)
{
	mutex_lock(&kgsl_driver.mutex);
	if (kgsl_driver.power_flags != 0) {
		kgsl_yamato_wake(&kgsl_driver.yamato_device);
		if (kgsl_driver.g12_device.flags & KGSL_FLAGS_STARTED)
			kgsl_g12_wake(&kgsl_driver.g12_device);
		kgsl_driver.is_suspended = KGSL_FALSE;
	}
	mutex_unlock(&kgsl_driver.mutex);
	return KGSL_SUCCESS;
}

/* file operations */
static int kgsl_first_open_locked(void)
{
	int result = 0;

	BUG_ON(kgsl_driver.yamato_grp_clk == NULL);
	BUG_ON(kgsl_driver.imem_clk == NULL);

	kgsl_driver.power_flags = KGSL_PWRFLAGS_YAMATO_CLK_OFF |
			KGSL_PWRFLAGS_POWER_OFF | KGSL_PWRFLAGS_YAMATO_IRQ_OFF;
	if (kgsl_driver.g12_device.hwaccess_blocked == KGSL_FALSE)
		kgsl_driver.power_flags |= KGSL_PWRFLAGS_G12_CLK_OFF |
			KGSL_PWRFLAGS_G12_IRQ_OFF;

	/* Turn the clocks on before the power.  Required for some platforms,
	   has no adverse effect on the others */
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_POWER_ON);

	kgsl_driver.is_suspended = KGSL_FALSE;

	INIT_LIST_HEAD(&kgsl_driver.pagetable_list);
	mutex_init(&kgsl_driver.pt_mutex);

	/* init devices */
	result = kgsl_yamato_init(&kgsl_driver.yamato_device,
					&kgsl_driver.yamato_config);
	if (result != 0)
		goto done;

	result = kgsl_yamato_start(&kgsl_driver.yamato_device, 0);
	if (result != 0)
		goto done;
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_ON);

done:
	return result;
}

static int kgsl_last_release_locked(void)
{
	BUG_ON(kgsl_driver.yamato_grp_clk == NULL);
	BUG_ON(kgsl_driver.imem_clk == NULL);

	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_OFF);

	kgsl_yamato_stop(&kgsl_driver.yamato_device);

	/* close yamato */
	kgsl_yamato_close(&kgsl_driver.yamato_device);

	/* For some platforms, power needs to go off before clocks */
	kgsl_pwrctrl(KGSL_PWRFLAGS_POWER_OFF);
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_OFF);

	kgsl_driver.power_flags = 0;

	return 0;
}

static int kgsl_release(struct inode *inodep, struct file *filep)
{
	int result = 0;
	unsigned int i;
	struct kgsl_mem_entry *entry, *entry_tmp;
	struct kgsl_file_private *private = NULL;

	KGSL_PRE_HWACCESS();

	private = filep->private_data;
	BUG_ON(private == NULL);
	filep->private_data = NULL;
	list_del(&private->list);

	for (i = 0; i < KGSL_CONTEXT_MAX; i++) {
		if (private->yamato_ctxt_id_mask & (1 << i))
			kgsl_drawctxt_destroy(&kgsl_driver.yamato_device, i);
		if (private->g12_ctxt_id_mask & (1 << i))
			kgsl_g12_drawctxt_destroy(&kgsl_driver.g12_device, i);
	}

	list_for_each_entry_safe(entry, entry_tmp, &private->mem_list, list)
		kgsl_remove_mem_entry(entry, false);

	entry = NULL;
	entry_tmp = NULL;
	list_for_each_entry_safe(entry, entry_tmp,
			&private->preserve_entry_list, list)
		kgsl_remove_mem_entry(entry, false);

	if (private->pagetable != NULL) {
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
		kgsl_yamato_cleanup_pt(&kgsl_driver.yamato_device,
					private->pagetable);
		kgsl_mmu_putpagetable(private->pagetable);
#endif
		private->pagetable = NULL;
	}
	kfree(private);

	if (atomic_dec_return(&kgsl_driver.open_count) == 0) {
		KGSL_DRV_VDBG("last_release\n");
		kgsl_g12_last_release_locked();
		result = kgsl_last_release_locked();
	}

	KGSL_POST_HWACCESS();

	return result;
}

static int kgsl_open(struct inode *inodep, struct file *filep)
{
	int result = 0;
	struct kgsl_file_private *private = NULL;

	KGSL_DRV_DBG("file %p pid %d\n", filep, task_pid_nr(current));


	if (filep->f_flags & O_EXCL) {
		KGSL_DRV_ERR("O_EXCL not allowed\n");
		return -EBUSY;
	}

	private = kzalloc(sizeof(*private), GFP_KERNEL);
	if (private == NULL) {
		KGSL_DRV_ERR("cannot allocate file private data\n");
		return -ENOMEM;
	}

	mutex_lock(&kgsl_driver.mutex);

	private->yamato_ctxt_id_mask = 0;
	private->g12_ctxt_id_mask = 0;
	INIT_LIST_HEAD(&private->mem_list);
	INIT_LIST_HEAD(&private->preserve_entry_list);
	private->preserve_list_size = 0;

	filep->private_data = private;

	list_add(&private->list, &kgsl_driver.client_list);

	if (atomic_inc_return(&kgsl_driver.open_count) == 1) {
		result = kgsl_first_open_locked();
		kgsl_g12_first_open_locked();
		if (result != 0)
			goto done;
	}
	/*NOTE: this must happen after first_open */
#ifdef CONFIG_MSM_KGSL_MMU
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	private->pagetable =
		kgsl_mmu_getpagetable(&kgsl_driver.yamato_device.mmu,
					current->pid);
	if (private->pagetable == NULL) {
		result = -ENOMEM;
		goto done;
	}
	result = kgsl_yamato_setup_pt(&kgsl_driver.yamato_device,
					private->pagetable);
	if (result) {
		kgsl_mmu_putpagetable(private->pagetable);
		private->pagetable = NULL;
		goto done;
	}
#else
	private->pagetable = kgsl_driver.yamato_device.mmu.hwpagetable;
#endif
	private->vmalloc_size = 0;
#endif
done:
	mutex_unlock(&kgsl_driver.mutex);
	if (result != 0)
		kgsl_release(inodep, filep);
	return result;
}


/*call with driver locked */
static struct kgsl_mem_entry *
kgsl_sharedmem_find(struct kgsl_file_private *private, unsigned int gpuaddr)
{
	struct kgsl_mem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->mem_list, list) {
		if (entry->memdesc.gpuaddr == gpuaddr) {
			result = entry;
			break;
		}
	}
	return result;
}

/*call with driver locked */
struct kgsl_mem_entry *
kgsl_sharedmem_find_region(struct kgsl_file_private *private,
				unsigned int gpuaddr,
				size_t size)
{
	struct kgsl_mem_entry *entry = NULL, *result = NULL;

	BUG_ON(private == NULL);

	list_for_each_entry(entry, &private->mem_list, list) {
		if (gpuaddr >= entry->memdesc.gpuaddr &&
		    ((gpuaddr + size) <=
			(entry->memdesc.gpuaddr + entry->memdesc.size))) {
			result = entry;
			break;
		}
	}

	return result;
}

/*call all ioctl sub functions with driver locked*/
static long kgsl_ioctl_cmdwindow_write(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_cmdwindow_write param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.device_id == KGSL_DEVICE_YAMATO) {
		result = -EINVAL;
	} else if (param.device_id == KGSL_DEVICE_G12) {
		KGSL_G12_PRE_HWACCESS();
		result = kgsl_g12_cmdwindow_write(&kgsl_driver.g12_device,
					     param.target,
					     param.addr,
					     param.data);
	} else {
		result = -EINVAL;
	}

	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_device_getproperty(struct kgsl_file_private *private,
					 void __user *arg)
{
	int result = 0;
	struct kgsl_device_getproperty param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	if ((param.device_id == KGSL_DEVICE_YAMATO) ||
				(param.device_id == KGSL_DEVICE_ANY))
		result = kgsl_yamato_getproperty(&kgsl_driver.yamato_device,
					 param.type,
					 param.value, param.sizebytes);
	else if (param.device_id == KGSL_DEVICE_G12)
		result = kgsl_g12_getproperty(&kgsl_driver.g12_device,
					 param.type,
					 param.value, param.sizebytes);
	else
		result = -EINVAL;

done:
	return result;
}

static long kgsl_ioctl_device_regread(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_device_regread param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	if (param.device_id == KGSL_DEVICE_YAMATO) {
		result = kgsl_yamato_regread(&kgsl_driver.yamato_device,
				     param.offsetwords, &param.value);

	} else if (param.device_id == KGSL_DEVICE_G12) {
		KGSL_G12_PRE_HWACCESS();
		result = kgsl_g12_regread(&kgsl_driver.g12_device,
				     param.offsetwords, &param.value);

	} else {
		result = -EINVAL;
	}

	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}


static long kgsl_ioctl_device_waittimestamp(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_device_waittimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	/* Don't wait forever, set a max value for now */
	if (param.timeout == -1)
		param.timeout = 10 * MSEC_PER_SEC;
	if (param.device_id == KGSL_DEVICE_YAMATO) {
		result = kgsl_yamato_waittimestamp(&kgsl_driver.yamato_device,
				     param.timestamp,
				     param.timeout);

		kgsl_runpending(&kgsl_driver.yamato_device);
	} else if (param.device_id == KGSL_DEVICE_G12) {
		KGSL_G12_PRE_HWACCESS();
		mutex_unlock(&kgsl_driver.mutex);
		result = kgsl_g12_waittimestamp(&kgsl_driver.g12_device,
				     param.timestamp,
				     param.timeout);
		mutex_lock(&kgsl_driver.mutex);

		kgsl_runpending(&kgsl_driver.g12_device);
	} else {
		result = -EINVAL;
	}

done:
	return result;
}

static long kgsl_ioctl_rb_issueibcmds(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_ringbuffer_issueibcmds param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.device_id == KGSL_DEVICE_YAMATO) {
		if (param.drawctxt_id >= KGSL_CONTEXT_MAX ||
			(private->yamato_ctxt_id_mask & 1 <<
			param.drawctxt_id) == 0) {
			result = -EINVAL;
			KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d\n",
				      param.drawctxt_id);
			result = -EINVAL;
			goto done;
		}

		if (kgsl_sharedmem_find_region(private, param.ibaddr,
				param.sizedwords*sizeof(uint32_t)) == NULL) {
			KGSL_DRV_ERR("invalid cmd buffer ibaddr %08x " \
					"sizedwords %d\n",
					param.ibaddr, param.sizedwords);
			result = -EINVAL;
			goto done;
		}

		result = kgsl_ringbuffer_issueibcmds(&kgsl_driver.yamato_device,
					     param.drawctxt_id,
					     param.ibaddr,
					     param.sizedwords,
					     &param.timestamp,
					     param.flags);
	} else if (param.device_id == KGSL_DEVICE_G12) {
		KGSL_G12_PRE_HWACCESS();
		if ((private->g12_ctxt_id_mask & 1 << param.drawctxt_id) == 0) {

			result = -EINVAL;
			KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d\n",
						param.drawctxt_id);
			result = -EINVAL;
			goto done;
		}

		if (kgsl_sharedmem_find_region(private, param.ibaddr,
				param.sizedwords*sizeof(uint32_t)) == NULL) {
			KGSL_DRV_ERR("invalid cmd buffer ibaddr %08x " \
					"sizedwords %d\n",
					param.ibaddr, param.sizedwords);
			result = -EINVAL;
			goto done;
		}

		result = kgsl_g12_cmdstream_issueibcmds(&kgsl_driver.g12_device,
						     private->pagetable,
						     param.drawctxt_id,
						     param.ibaddr,
						     param.sizedwords,
						     &param.timestamp,
						     param.flags);
	} else {
		result = -EINVAL;
	}
	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_cmdstream_readtimestamp(struct kgsl_file_private
						*private, void __user *arg)
{
	int result = 0;
	struct kgsl_cmdstream_readtimestamp param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.device_id == KGSL_DEVICE_YAMATO) {
		param.timestamp =
			kgsl_cmdstream_readtimestamp(
						&kgsl_driver.yamato_device,
							param.type);
	} else if (param.device_id == KGSL_DEVICE_G12) {
		struct kgsl_device *device;
		KGSL_G12_PRE_HWACCESS();
		device = &kgsl_driver.g12_device;

		param.timestamp = device->timestamp;
	} else {
		result = -EINVAL;
	}

	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_ioctl_cmdstream_freememontimestamp(struct kgsl_file_private
						*private, void __user *arg)
{
	int result = 0;
	struct kgsl_cmdstream_freememontimestamp param;
	struct kgsl_mem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	entry = kgsl_sharedmem_find(private, param.gpuaddr);
	if (entry == NULL) {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
		goto done;
	}
#ifdef CONFIG_MSM_KGSL_MMU
	if (entry->memdesc.priv & KGSL_MEMFLAGS_VMALLOC_MEM)
		entry->memdesc.priv &= ~KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH;
#endif
	if (param.device_id == KGSL_DEVICE_YAMATO) {
		result = kgsl_cmdstream_freememontimestamp(
						&kgsl_driver.yamato_device,
							entry,
							param.timestamp,
							param.type);

		kgsl_runpending(&kgsl_driver.yamato_device);
	} else if (param.device_id == KGSL_DEVICE_G12) {
		KGSL_G12_PRE_HWACCESS();
		result = kgsl_cmdstream_freememontimestamp(
						&kgsl_driver.g12_device,
							entry,
							param.timestamp,
							param.type);

		kgsl_runpending(&kgsl_driver.g12_device);
	} else {
		result = -EINVAL;
	}
done:
	return result;
}

static long kgsl_ioctl_drawctxt_create(struct kgsl_file_private *private,
				      void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_create param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.device_id == KGSL_DEVICE_YAMATO) {
		result = kgsl_drawctxt_create(&kgsl_driver.yamato_device,
					private->pagetable,
					param.flags,
					&param.drawctxt_id);
		if (result != 0)
			goto done;

		if (copy_to_user(arg, &param, sizeof(param))) {
			result = -EFAULT;
			goto done;
		}

		private->yamato_ctxt_id_mask |= 1 << param.drawctxt_id;
	} else if (param.device_id == KGSL_DEVICE_G12) {
		KGSL_G12_PRE_HWACCESS();
		result = kgsl_g12_drawctxt_create(&kgsl_driver.g12_device,
					private->g12_ctxt_id_mask,
					&param.drawctxt_id);
		if (result != 0)
			goto done;

		if (copy_to_user(arg, &param, sizeof(param))) {
			result = -EFAULT;
			goto done;
		}

		/* if KGSL_CONTEXT_MAX is more than 16,
		then linked list should be used for them */
		private->g12_ctxt_id_mask |= 1 << param.drawctxt_id;
	} else {
		result = -EINVAL;
	}

done:
	return result;
}

static long kgsl_ioctl_drawctxt_destroy(struct kgsl_file_private *private,
				       void __user *arg)
{
	int result = 0;
	struct kgsl_drawctxt_destroy param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	if (param.device_id == KGSL_DEVICE_YAMATO) {
		if (param.drawctxt_id >= KGSL_CONTEXT_MAX ||
				(private->yamato_ctxt_id_mask & 1 <<
				param.drawctxt_id) == 0) {
			result = -EINVAL;
			goto done;
		}

		result = kgsl_drawctxt_destroy(&kgsl_driver.yamato_device,
					param.drawctxt_id);
		if (result == 0)
			private->yamato_ctxt_id_mask &=
					~(1 << param.drawctxt_id);
	} else if (param.device_id == KGSL_DEVICE_G12) {
		KGSL_G12_PRE_HWACCESS();
		if ((private->g12_ctxt_id_mask & 1 << param.drawctxt_id) == 0) {
			result = -EINVAL;
			goto done;
		}

		result = kgsl_g12_drawctxt_destroy(&kgsl_driver.g12_device,
					param.drawctxt_id);
		if (result == 0)
			private->g12_ctxt_id_mask &= ~(1 <<
						param.drawctxt_id);

		kgsl_runpending(&kgsl_driver.g12_device);
	} else {
		result = -EINVAL;
	}
done:
	return result;
}

void kgsl_remove_mem_entry(struct kgsl_mem_entry *entry, bool preserve)
{
	/* If allocation is vmalloc and preserve is requested then save
	* the allocation in a free list to be used later instead of
	* freeing it here */
	if (KGSL_MEMFLAGS_VMALLOC_MEM & entry->memdesc.priv &&
		preserve &&
		entry->priv->preserve_list_size < KGSL_MAX_PRESERVED_BUFFERS &&
		entry->memdesc.size <= KGSL_MAX_SIZE_OF_PRESERVED_BUFFER) {
		if (entry->free_list.prev) {
			list_del(&entry->free_list);
			entry->free_list.prev = NULL;
		}
		if (entry->list.prev) {
			list_del(&entry->list);
			entry->list.prev = NULL;
		}
		list_add(&entry->list, &entry->priv->preserve_entry_list);
		entry->priv->preserve_list_size++;
		return;
	}
	kgsl_mmu_unmap(entry->memdesc.pagetable,
			entry->memdesc.gpuaddr & KGSL_PAGEMASK,
			entry->memdesc.size);
	if (KGSL_MEMFLAGS_VMALLOC_MEM & entry->memdesc.priv) {
		vfree((void *)entry->memdesc.physaddr);
		entry->priv->vmalloc_size -= entry->memdesc.size;
	} else
		kgsl_put_phys_file(entry->pmem_file);

	/* remove the entry from list and free_list if it exists */
	if (entry->list.prev)
		list_del(&entry->list);
	if (entry->free_list.prev)
		list_del(&entry->free_list);

	kfree(entry);

}

static long kgsl_ioctl_sharedmem_free(struct kgsl_file_private *private,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_sharedmem_free param;
	struct kgsl_mem_entry *entry = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
	entry = kgsl_sharedmem_find(private, param.gpuaddr);

	if (entry == NULL) {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
		goto done;
	}

	kgsl_remove_mem_entry(entry, false);
done:
	return result;
}

#ifdef CONFIG_MSM_KGSL_MMU
static long kgsl_ioctl_sharedmem_from_vmalloc(struct kgsl_file_private *private,
					      void __user *arg)
{
	int result = 0, len, found = 0;
	struct kgsl_sharedmem_from_vmalloc param;
	struct kgsl_mem_entry *entry = NULL, *entry_tmp = NULL;
	void *vmalloc_area;
	struct vm_area_struct *vma;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto error;
	}

	if (!param.hostptr) {
		KGSL_DRV_ERR
		    ("Invalid host pointer of malloc passed: param.hostptr "
		     "%08x\n", param.hostptr);
		result = -EINVAL;
		goto error;
	}

	vma = find_vma(current->mm, param.hostptr);
	if (!vma) {
		KGSL_MEM_ERR("Could not find vma for address %x\n",
			     param.hostptr);
		result = -EINVAL;
		goto error;
	}
	len = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff || !KGSL_IS_PAGE_ALIGNED(len) ||
	    !KGSL_IS_PAGE_ALIGNED(vma->vm_start)) {
		KGSL_MEM_ERR
		("kgsl vmalloc mapping must be at offset 0 and page aligned\n");
		result = -EINVAL;
		goto error;
	}
	if (vma->vm_start != param.hostptr) {
		KGSL_MEM_ERR
		    ("vma start address is not equal to mmap address\n");
		result = -EINVAL;
		goto error;
	}

	if ((private->vmalloc_size + len) > KGSL_GRAPHICS_MEMORY_LOW_WATERMARK
	    && !param.force_no_low_watermark) {
		result = -ENOMEM;
		goto error;
	}

	list_for_each_entry_safe(entry, entry_tmp,
				&private->preserve_entry_list, list) {
		if (entry->memdesc.size == len) {
			list_del(&entry->list);
			found = 1;
			break;
		}
	}

	if (!found) {
		entry = kzalloc(sizeof(struct kgsl_mem_entry), GFP_KERNEL);
		if (entry == NULL) {
			result = -ENOMEM;
			goto error;
		}

		/* allocate memory and map it to user space */
		vmalloc_area = vmalloc_user(len);
		if (!vmalloc_area) {
			KGSL_MEM_ERR("vmalloc failed\n");
			result = -ENOMEM;
			goto error_free_entry;
		}
		kgsl_cache_range_op((unsigned int)vmalloc_area, len,
				KGSL_CACHE_INV | KGSL_CACHE_VMALLOC_ADDR);

		result =
		    kgsl_mmu_map(private->pagetable,
			(unsigned long)vmalloc_area, len,
			GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
			&entry->memdesc.gpuaddr, KGSL_MEMFLAGS_ALIGN4K);
		if (result != 0)
			goto error_free_vmalloc;

		entry->memdesc.pagetable = private->pagetable;
		entry->memdesc.size = len;
		entry->memdesc.priv = KGSL_MEMFLAGS_VMALLOC_MEM |
			    KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH;
		entry->memdesc.physaddr = (unsigned long)vmalloc_area;
		entry->priv = private;
		private->vmalloc_size += len;

	} else {
		KGSL_MEM_INFO("Reusing memory entry: %x, size: %x\n",
				(unsigned int)entry, entry->memdesc.size);
		entry->priv->preserve_list_size--;
		vmalloc_area = (void *)entry->memdesc.physaddr;
	}

	if (!kgsl_cache_enable)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	result = remap_vmalloc_range(vma, vmalloc_area, 0);
	if (result) {
		KGSL_MEM_ERR("remap_vmalloc_range returned %d\n", result);
		goto error_unmap_entry;
	}

	entry->memdesc.hostptr = (void *)param.hostptr;

	param.gpuaddr = entry->memdesc.gpuaddr;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto error_unmap_entry;
	}
	list_add(&entry->list, &private->mem_list);

	return 0;

error_unmap_entry:
	kgsl_mmu_unmap(private->pagetable, entry->memdesc.gpuaddr,
		       entry->memdesc.size);

error_free_vmalloc:
	vfree(vmalloc_area);

error_free_entry:
	kfree(entry);

error:
	return result;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

static int kgsl_get_phys_file(int fd, unsigned long *start, unsigned long *len,
			      struct file **filep)
{
	struct file *fbfile;
	int put_needed;
	unsigned long vstart = 0;
	int ret = 0;
	dev_t rdev;
	struct fb_info *info;

	*filep = NULL;
	if (!get_pmem_file(fd, start, &vstart, len, filep))
		return 0;

	fbfile = fget_light(fd, &put_needed);
	if (fbfile == NULL)
		return -1;

	rdev = fbfile->f_dentry->d_inode->i_rdev;
	info = MAJOR(rdev) == FB_MAJOR ? registered_fb[MINOR(rdev)] : NULL;
	if (info) {
		*start = info->fix.smem_start;
		*len = info->fix.smem_len;
		ret = 0;
	} else
		ret = -1;
	fput_light(fbfile, put_needed);

	return ret;
}

static void kgsl_put_phys_file(struct file *file)
{
	KGSL_DRV_DBG("put phys file %p\n", file);
	if (file)
		put_pmem_file(file);
}

static int kgsl_ioctl_sharedmem_from_pmem(struct kgsl_file_private *private,
						void __user *arg)
{
	int result = 0;
	struct kgsl_sharedmem_from_pmem param;
	struct kgsl_mem_entry *entry = NULL;
	unsigned long start = 0, len = 0;
	struct file *pmem_file = NULL;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto error;
	}

	if (kgsl_get_phys_file(param.pmem_fd, &start, &len, &pmem_file)) {
		result = -EINVAL;
		goto error;

	} else {
		if (!param.len)
			param.len = len;

		if (param.offset + param.len > len) {
			KGSL_DRV_ERR("%s: region too large "
					"0x%x + 0x%x >= 0x%lx\n",
				     __func__, param.offset, param.len, len);
			result = -EINVAL;
			goto error_put_pmem;
		}
	}

	KGSL_MEM_INFO("get phys file %p start 0x%lx len 0x%lx\n",
		      pmem_file, start, len);
	KGSL_DRV_DBG("locked phys file %p\n", pmem_file);

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (entry == NULL) {
		result = -ENOMEM;
		goto error_put_pmem;
	}

	entry->pmem_file = pmem_file;

	entry->memdesc.pagetable = private->pagetable;

	/* Any MMU mapped memory must have a length in multiple of PAGESIZE */
	entry->memdesc.size = ALIGN(param.len, PAGE_SIZE);
	/*we shouldn't need to write here from kernel mode */
	entry->memdesc.hostptr = NULL;
	/* ensure that MMU mappings are at page boundary */
	entry->memdesc.physaddr = start + (param.offset & KGSL_PAGEMASK);
	result = kgsl_mmu_map(private->pagetable, entry->memdesc.physaddr,
			entry->memdesc.size, GSL_PT_PAGE_RV | GSL_PT_PAGE_WV,
			&entry->memdesc.gpuaddr,
			KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS);
	if (result)
		goto error_free_entry;

	/* If the offset is not at 4K boundary then add the correct offset
	 * value to gpuaddr */
	entry->memdesc.gpuaddr += (param.offset & ~KGSL_PAGEMASK);
	param.gpuaddr = entry->memdesc.gpuaddr;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto error_unmap_entry;
	}
	list_add(&entry->list, &private->mem_list);
	return result;

error_unmap_entry:
	kgsl_mmu_unmap(entry->memdesc.pagetable,
			entry->memdesc.gpuaddr & KGSL_PAGEMASK,
			entry->memdesc.size);
error_free_entry:
	kfree(entry);

error_put_pmem:
	kgsl_put_phys_file(pmem_file);

error:
	return result;
}

#ifdef CONFIG_MSM_KGSL_MMU
/*This function flushes a graphics memory allocation from CPU cache
 *when caching is enabled with MMU*/
static long kgsl_ioctl_sharedmem_flush_cache(struct kgsl_file_private *private,
				       void __user *arg)
{
	int result = 0;
	struct kgsl_mem_entry *entry;
	struct kgsl_sharedmem_free param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	entry = kgsl_sharedmem_find(private, param.gpuaddr);
	if (!entry) {
		KGSL_DRV_ERR("invalid gpuaddr %08x\n", param.gpuaddr);
		result = -EINVAL;
		goto done;
	}
	result = kgsl_cache_range_op((unsigned long)entry->memdesc.hostptr,
					entry->memdesc.size,
				KGSL_CACHE_CLEAN | KGSL_CACHE_USER_ADDR);
	/* Mark memory as being flushed so we don't flush it again */
	entry->memdesc.priv &= ~KGSL_MEMFLAGS_MEM_REQUIRES_FLUSH;
done:
	return result;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

static long kgsl_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result = 0;
	struct kgsl_file_private *private = filep->private_data;
	struct kgsl_drawctxt_set_bin_base_offset binbase;

	BUG_ON(private == NULL);

	KGSL_DRV_VDBG("filep %p cmd 0x%08x arg 0x%08lx\n", filep, cmd, arg);
	KGSL_PRE_HWACCESS();
	switch (cmd) {

	case IOCTL_KGSL_DEVICE_GETPROPERTY:
		result =
		    kgsl_ioctl_device_getproperty(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_REGREAD:
		result = kgsl_ioctl_device_regread(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_DEVICE_WAITTIMESTAMP:
		result = kgsl_ioctl_device_waittimestamp(private,
							(void __user *)arg);
		/* order reads to the buffer written to by the GPU */
		rmb();
		break;

	case IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS:
#ifdef CONFIG_MSM_KGSL_MMU
		if (kgsl_cache_enable)
			kgsl_clean_cache_all(private);
#endif
#ifdef CONFIG_MSM_KGSL_DRM
		kgsl_gpu_mem_flush();
#endif
		result = kgsl_ioctl_rb_issueibcmds(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_CMDSTREAM_READTIMESTAMP:
		result =
		    kgsl_ioctl_cmdstream_readtimestamp(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP:
		result =
		    kgsl_ioctl_cmdstream_freememontimestamp(private,
						    (void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_CREATE:
		result = kgsl_ioctl_drawctxt_create(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_DESTROY:
		result =
		    kgsl_ioctl_drawctxt_destroy(private, (void __user *)arg);
		break;

	case IOCTL_KGSL_SHAREDMEM_FREE:
		result = kgsl_ioctl_sharedmem_free(private, (void __user *)arg);
		break;

#ifdef CONFIG_MSM_KGSL_MMU
	case IOCTL_KGSL_SHAREDMEM_FROM_VMALLOC:
		kgsl_runpending(&kgsl_driver.yamato_device);
		kgsl_runpending(&kgsl_driver.g12_device);
		result = kgsl_ioctl_sharedmem_from_vmalloc(private,
							   (void __user *)arg);
		break;

	case IOCTL_KGSL_SHAREDMEM_FLUSH_CACHE:
		if (kgsl_cache_enable)
			result =
			    kgsl_ioctl_sharedmem_flush_cache(private,
						       (void __user *)arg);
		break;
#endif
	case IOCTL_KGSL_SHAREDMEM_FROM_PMEM:
		kgsl_runpending(&kgsl_driver.yamato_device);
		kgsl_runpending(&kgsl_driver.g12_device);
		result = kgsl_ioctl_sharedmem_from_pmem(private,
							(void __user *)arg);
		break;

	case IOCTL_KGSL_DRAWCTXT_SET_BIN_BASE_OFFSET:
		if (copy_from_user(&binbase, (void __user *)arg,
				   sizeof(binbase))) {
			result = -EFAULT;
			break;
		}

		if ((binbase.device_id == KGSL_DEVICE_YAMATO) &&
					(private->yamato_ctxt_id_mask &
					(1 << binbase.drawctxt_id))) {
			result = kgsl_drawctxt_set_bin_base_offset(
					&kgsl_driver.yamato_device,
					binbase.drawctxt_id,
					binbase.offset);
		} else {
			result = -EINVAL;
			KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d\n",
				     binbase.drawctxt_id);
		}
		break;

	case IOCTL_KGSL_CMDWINDOW_WRITE:
		result = kgsl_ioctl_cmdwindow_write(private,
							(void __user *)arg);
		break;
	default:
		KGSL_DRV_ERR("invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	KGSL_POST_HWACCESS();
	KGSL_DRV_VDBG("result %d\n", result);
	return result;
}

static int kgsl_mmap(struct file *file, struct vm_area_struct *vma)
{
	int result;
	struct kgsl_memdesc *memdesc = NULL;
	unsigned long vma_size = vma->vm_end - vma->vm_start;
	unsigned long vma_offset = vma->vm_pgoff << PAGE_SHIFT;
	struct kgsl_device *device = NULL;

	mutex_lock(&kgsl_driver.mutex);

	device = &kgsl_driver.yamato_device;

	/*allow yamato memstore to be mapped read only */
	if (vma_offset == device->memstore.physaddr) {
		if (vma->vm_flags & VM_WRITE) {
			result = -EPERM;
			goto done;
		}
		memdesc = &device->memstore;
	}

	if (memdesc->size != vma_size) {
		KGSL_MEM_ERR("file %p bad size %ld, should be %d\n",
			file, vma_size, memdesc->size);
		result = -EINVAL;
		goto done;
	}
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	result = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma_size, vma->vm_page_prot);
	if (result != 0) {
		KGSL_MEM_ERR("remap_pfn_range returned %d\n",
				result);
		goto done;
	}
done:
	mutex_unlock(&kgsl_driver.mutex);
	return result;
}

static const struct file_operations kgsl_fops = {
	.owner = THIS_MODULE,
	.release = kgsl_release,
	.open = kgsl_open,
	.mmap = kgsl_mmap,
	.unlocked_ioctl = kgsl_ioctl,
};


struct kgsl_driver kgsl_driver = {
	.misc = {
		 .name = DRIVER_NAME,
		 .minor = MISC_DYNAMIC_MINOR,
		 .fops = &kgsl_fops,
	 },
	.open_count = ATOMIC_INIT(0),
	.mutex = __MUTEX_INITIALIZER(kgsl_driver.mutex),
};


static void kgsl_driver_cleanup(void)
{

	struct kgsl_memregion *regspace = &kgsl_driver.yamato_device.regspace;

	regspace = &kgsl_driver.yamato_device.regspace;
	if (regspace->mmio_virt_base) {
		iounmap(regspace->mmio_virt_base);
		release_mem_region(regspace->mmio_phys_base,
				   regspace->sizebytes);
		memset(&regspace, 0, sizeof(*regspace));
	}

	regspace = &kgsl_driver.g12_device.regspace;
	if (regspace->mmio_virt_base) {
		iounmap(regspace->mmio_virt_base);
		release_mem_region(regspace->mmio_phys_base,
				   regspace->sizebytes);
		memset(&regspace, 0, sizeof(*regspace));
	}

	if (kgsl_driver.yamato_interrupt_num > 0) {
		if (kgsl_driver.yamato_have_irq) {
			free_irq(kgsl_driver.yamato_interrupt_num, NULL);
			kgsl_driver.yamato_have_irq = 0;
		}
		kgsl_driver.yamato_interrupt_num = 0;
	}

	if (kgsl_driver.g12_interrupt_num > 0) {
		if (kgsl_driver.g12_have_irq) {
			free_irq(kgsl_driver.g12_interrupt_num, NULL);
			kgsl_driver.g12_have_irq = 0;
		}
		kgsl_driver.g12_interrupt_num = 0;
	}

	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ, "kgsl_3d");

	if (kgsl_driver.yamato_grp_pclk) {
		clk_put(kgsl_driver.yamato_grp_pclk);
		kgsl_driver.yamato_grp_pclk = NULL;
	}

	/* shutdown memory apertures */
	kgsl_sharedmem_close(&kgsl_driver.shmem);

	if (kgsl_driver.yamato_grp_clk) {
		clk_put(kgsl_driver.yamato_grp_clk);
		kgsl_driver.yamato_grp_clk = NULL;
	}

	if (kgsl_driver.imem_clk != NULL) {
		clk_put(kgsl_driver.imem_clk);
		kgsl_driver.imem_clk = NULL;
	}

	if (kgsl_driver.g12_grp_pclk) {
		clk_put(kgsl_driver.g12_grp_pclk);
		kgsl_driver.g12_grp_pclk = NULL;
	}

	if (kgsl_driver.g12_grp_clk) {
		clk_put(kgsl_driver.g12_grp_clk);
		kgsl_driver.g12_grp_clk = NULL;
		pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ, "kgsl_2d");
	}

	kgsl_driver.pdev = NULL;

}


static int __devinit kgsl_platform_probe(struct platform_device *pdev)
{
	int result = 0;
	struct clk *clk, *grp_clk;
	struct resource *res = NULL;
	struct kgsl_platform_data *pdata = NULL;

	kgsl_debug_init();

	INIT_LIST_HEAD(&kgsl_driver.client_list);
	/*acquire clocks */
	BUG_ON(kgsl_driver.yamato_grp_clk != NULL);
	BUG_ON(kgsl_driver.g12_grp_clk != NULL);
	BUG_ON(kgsl_driver.imem_clk != NULL);

	kgsl_driver.pdev = pdev;
	pdata = pdev->dev.platform_data;

	clk = clk_get(&pdev->dev, "grp_pclk");
	if (IS_ERR(clk))
		clk = NULL;
	kgsl_driver.yamato_grp_pclk = clk;

	clk = clk_get(&pdev->dev, "grp_clk");
	if (IS_ERR(clk)) {
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(grp_clk) returned %d\n", result);
		goto done;
	}
	kgsl_driver.yamato_grp_clk = grp_clk = clk;

	clk = clk_get(&pdev->dev, "grp_src_clk");
	if (IS_ERR(clk)) {
		clk = grp_clk; /* Fallback to slave */
	}
	kgsl_driver.yamato_grp_src_clk = clk;


	/* put the AXI bus into asynchronous mode with the graphics cores */
	if (pdata != NULL) {
		if ((pdata->set_grp3d_async != NULL) &&
			(pdata->max_grp3d_freq) &&
			(!pdata->set_grp3d_async()))
			clk_set_min_rate(clk, pdata->max_grp3d_freq);
	}

	clk = clk_get(&pdev->dev, "imem_clk");
	if (IS_ERR(clk)) {
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(imem_clk) returned %d\n", result);
		goto done;
	}
	kgsl_driver.imem_clk = clk;

	clk = clk_get(&pdev->dev, "grp_2d_pclk");
	if (IS_ERR(clk))
		clk = NULL;
	kgsl_driver.g12_grp_pclk = clk;

	clk = clk_get(&pdev->dev, "grp_2d_clk");
	if (IS_ERR(clk)) {
		clk = NULL;
		kgsl_driver.g12_device.hwaccess_blocked = KGSL_TRUE;
		result = PTR_ERR(clk);
		KGSL_DRV_ERR("clk_get(grp_2d_clk) returned %d\n", result);
	}
	kgsl_driver.g12_grp_clk = clk;

	if (pdata != NULL && clk != NULL) {
		if ((pdata->set_grp2d_async != NULL) &&
			(pdata->max_grp2d_freq) &&
			(!pdata->set_grp2d_async()))
			clk_set_min_rate(clk, pdata->max_grp2d_freq);
	}

	kgsl_driver.power_flags = 0;

	if (pdata) {
		kgsl_driver.clk_freq[KGSL_AXI_HIGH_3D] = pdata->high_axi_3d;
		kgsl_driver.clk_freq[KGSL_AXI_HIGH_2D] = pdata->high_axi_2d;
		kgsl_driver.clk_freq[KGSL_2D_MIN_FREQ] = pdata->min_grp2d_freq;
		kgsl_driver.clk_freq[KGSL_2D_MAX_FREQ] = pdata->max_grp2d_freq;
		kgsl_driver.clk_freq[KGSL_3D_MIN_FREQ] = pdata->min_grp3d_freq;
		kgsl_driver.clk_freq[KGSL_3D_MAX_FREQ] = pdata->max_grp3d_freq;
	}

	pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ, "kgsl_3d",
				PM_QOS_DEFAULT_VALUE);

	/*acquire yamato interrupt */
	kgsl_driver.yamato_interrupt_num =
			platform_get_irq_byname(pdev, "kgsl_yamato_irq");

	if (kgsl_driver.yamato_interrupt_num <= 0) {
		KGSL_DRV_ERR("platform_get_irq_byname() returned %d\n",
			       kgsl_driver.yamato_interrupt_num);
		result = -EINVAL;
		goto done;
	}
	result = request_irq(kgsl_driver.yamato_interrupt_num, kgsl_yamato_isr,
				IRQF_TRIGGER_HIGH, DRIVER_NAME, NULL);
	if (result) {
		KGSL_DRV_ERR("request_irq(%d) returned %d\n",
			      kgsl_driver.yamato_interrupt_num, result);
		goto done;
	}
	kgsl_driver.yamato_have_irq = 1;
	disable_irq(kgsl_driver.yamato_interrupt_num);

	/*acquire g12 interrupt */
	if (kgsl_driver.g12_grp_clk) {
		kgsl_driver.g12_interrupt_num =
			platform_get_irq_byname(pdev, "kgsl_g12_irq");

		if (kgsl_driver.g12_interrupt_num <= 0) {
			KGSL_DRV_ERR("platform_get_irq_byname() returned %d\n",
						kgsl_driver.g12_interrupt_num);
			result = -EINVAL;
			goto done;
		}
		result = request_irq(kgsl_driver.g12_interrupt_num,
					kgsl_g12_isr,
					IRQF_TRIGGER_HIGH,
					DRIVER_NAME, NULL);
		if (result) {
			KGSL_DRV_ERR("request_irq(%d) returned %d\n",
				      kgsl_driver.g12_interrupt_num, result);
			goto done;
		}
		kgsl_driver.g12_have_irq = 1;
		disable_irq(kgsl_driver.g12_interrupt_num);

		/* g12 config */
		pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ, "kgsl_2d",
				PM_QOS_DEFAULT_VALUE);
		result = kgsl_g12_config(&kgsl_driver.g12_config, pdev);
		if (result != 0)
			goto done;
	}

	/* yamato config */
	result = kgsl_yamato_config(&kgsl_driver.yamato_config, pdev);
	if (result != 0)
		goto done;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "kgsl_phys_memory");
	if (res == NULL) {
		result = -EINVAL;
		goto done;
	}

	kgsl_driver.shmem.physbase = res->start;
	kgsl_driver.shmem.size = resource_size(res);

	/* init memory apertures */
	result = kgsl_sharedmem_init(&kgsl_driver.shmem);
	if (result != 0)
		goto done;

	result = kgsl_drm_init(pdev);

done:
	if (result)
		kgsl_driver_cleanup();
	else
		result = misc_register(&kgsl_driver.misc);

	return result;
}

static int kgsl_platform_remove(struct platform_device *pdev)
{

	kgsl_driver_cleanup();
	kgsl_drm_exit();
	misc_deregister(&kgsl_driver.misc);

	return 0;
}

static struct platform_driver kgsl_platform_driver = {
	.probe = kgsl_platform_probe,
	.remove = __devexit_p(kgsl_platform_remove),
	.suspend = kgsl_suspend,
	.resume = kgsl_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME
	}
};

static int __init kgsl_mod_init(void)
{
	return platform_driver_register(&kgsl_platform_driver);
}

static void __exit kgsl_mod_exit(void)
{
	platform_driver_unregister(&kgsl_platform_driver);
}

#ifdef MODULE
module_init(kgsl_mod_init);
#else
late_initcall(kgsl_mod_init);
#endif
module_exit(kgsl_mod_exit);

MODULE_DESCRIPTION("Graphics driver for QSD8x50, MSM7x27, and MSM7x30");
MODULE_VERSION("1.1");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:kgsl");
