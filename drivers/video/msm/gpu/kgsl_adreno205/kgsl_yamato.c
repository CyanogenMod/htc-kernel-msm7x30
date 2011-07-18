/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>

#include "kgsl_drawctxt.h"
#include "kgsl.h"
#include "kgsl_yamato.h"
#include "kgsl_log.h"
#include "kgsl_pm4types.h"
#include "kgsl_cmdstream.h"
#include "kgsl_postmortem.h"

#include "yamato_reg.h"

#define GSL_RBBM_INT_MASK \
	 (RBBM_INT_CNTL__RDERR_INT_MASK |  \
	  RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK)

#define GSL_SQ_INT_MASK \
	(SQ_INT_CNTL__PS_WATCHDOG_MASK | \
	 SQ_INT_CNTL__VS_WATCHDOG_MASK)

/* Yamato MH arbiter config*/
#define KGSL_CFG_YAMATO_MHARB \
	(0x10 \
		| (0 << MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
		| (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

static int kgsl_yamato_start(struct kgsl_device *device);
static int kgsl_yamato_stop(struct kgsl_device *device);
static int kgsl_yamato_sleep(struct kgsl_device *device, const int idle);

static int kgsl_yamato_gmeminit(struct kgsl_yamato_device *yamato_device)
{
	struct kgsl_device *device = &yamato_device->dev;
	union reg_rb_edram_info rb_edram_info;
	unsigned int gmem_size;
	unsigned int edram_value = 0;

	/* make sure edram range is aligned to size */
	BUG_ON(yamato_device->gmemspace.gpu_base &
				(yamato_device->gmemspace.sizebytes - 1));

	/* get edram_size value equivalent */
	gmem_size = (yamato_device->gmemspace.sizebytes >> 14);
	while (gmem_size >>= 1)
		edram_value++;

	rb_edram_info.val = 0;

	rb_edram_info.f.edram_size = edram_value;
	if (device->chip_id != KGSL_CHIPID_LEIA_REV470)
		rb_edram_info.f.edram_mapping_mode = 0; /* EDRAM_MAP_UPPER */

	/* must be aligned to size */
	rb_edram_info.f.edram_range = (yamato_device->gmemspace.gpu_base >> 14);

	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, rb_edram_info.val);

	return 0;
}

static int kgsl_yamato_gmemclose(struct kgsl_device *device)
{
	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, 0x00000000);

	return 0;
}

static void kgsl_yamato_rbbm_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int rderr = 0;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	kgsl_yamato_regread(device, REG_RBBM_INT_STATUS, &status);

	if (status & RBBM_INT_CNTL__RDERR_INT_MASK) {
		union rbbm_read_error_u rerr;
		kgsl_yamato_regread(device, REG_RBBM_READ_ERROR, &rderr);
		rerr.val = rderr;
		if (rerr.f.read_address == REG_CP_INT_STATUS &&
			rerr.f.read_error &&
			rerr.f.read_requester)
			KGSL_DRV_WARN("rbbm read error interrupt: %08x\n",
					rderr);
		else
			KGSL_DRV_FATAL("rbbm read error interrupt: %08x\n",
					rderr);
	} else if (status & RBBM_INT_CNTL__DISPLAY_UPDATE_INT_MASK) {
		KGSL_DRV_DBG("rbbm display update interrupt\n");
	} else if (status & RBBM_INT_CNTL__GUI_IDLE_INT_MASK) {
		KGSL_DRV_DBG("rbbm gui idle interrupt\n");
	} else {
		KGSL_CMD_DBG("bad bits in REG_CP_INT_STATUS %08x\n", status);
	}

	status &= GSL_RBBM_INT_MASK;
	kgsl_yamato_regwrite(device, REG_RBBM_INT_ACK, status);

	KGSL_DRV_VDBG("return\n");
}

static void kgsl_yamato_sq_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	kgsl_yamato_regread(device, REG_SQ_INT_STATUS, &status);

	if (status & SQ_INT_CNTL__PS_WATCHDOG_MASK)
		KGSL_DRV_DBG("sq ps watchdog interrupt\n");
	else if (status & SQ_INT_CNTL__VS_WATCHDOG_MASK)
		KGSL_DRV_DBG("sq vs watchdog interrupt\n");
	else
		KGSL_DRV_DBG("bad bits in REG_SQ_INT_STATUS %08x\n", status);


	status &= GSL_SQ_INT_MASK;
	kgsl_yamato_regwrite(device, REG_SQ_INT_ACK, status);

	KGSL_DRV_VDBG("return\n");
}

irqreturn_t kgsl_yamato_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;
	struct kgsl_device *device;
	unsigned int status;

	device = (struct kgsl_device *) data;

	BUG_ON(device == NULL);
	BUG_ON(device->regspace.sizebytes == 0);
	BUG_ON(device->regspace.mmio_virt_base == 0);

	kgsl_yamato_regread(device, REG_MASTER_INT_SIGNAL, &status);

	if (status & MASTER_INT_SIGNAL__MH_INT_STAT) {
		kgsl_mh_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__CP_INT_STAT) {
		kgsl_cp_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__RBBM_INT_STAT) {
		kgsl_yamato_rbbm_intrcallback(device);
		result = IRQ_HANDLED;
	}

	if (status & MASTER_INT_SIGNAL__SQ_INT_STAT) {
		kgsl_yamato_sq_intrcallback(device);
		result = IRQ_HANDLED;
	}
	/* Reset the time-out in our idle timer */
	mod_timer(&device->idle_timer, jiffies + device->interval_timeout);
	return result;
}

static int kgsl_yamato_cleanup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	if (device->mmu.defaultpagetable == pagetable)
		device->mmu.defaultpagetable = NULL;

	kgsl_mmu_unmap(pagetable, device->ringbuffer.buffer_desc.gpuaddr,
			device->ringbuffer.buffer_desc.size);

	kgsl_mmu_unmap(pagetable, device->ringbuffer.memptrs_desc.gpuaddr,
			device->ringbuffer.memptrs_desc.size);

	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);

	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);

	return 0;
}

static int kgsl_yamato_setup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	int result = 0;
	unsigned int flags = KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K;

	BUG_ON(device->ringbuffer.buffer_desc.physaddr == 0);
	BUG_ON(device->ringbuffer.memptrs_desc.physaddr == 0);
	BUG_ON(device->memstore.physaddr == 0);
#ifdef CONFIG_MSM_KGSL_MMU
	BUG_ON(device->mmu.dummyspace.physaddr == 0);
#endif
	if (device->mmu.defaultpagetable == NULL)
		device->mmu.defaultpagetable = pagetable;

	result = kgsl_mmu_map_global(pagetable, &device->ringbuffer.buffer_desc,
				     GSL_PT_PAGE_RV, flags);
	if (result)
		goto error;

	result = kgsl_mmu_map_global(pagetable,
				     &device->ringbuffer.memptrs_desc,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto unmap_buffer_desc;

	result = kgsl_mmu_map_global(pagetable, &device->memstore,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto unmap_memptrs_desc;

	result = kgsl_mmu_map_global(pagetable, &device->mmu.dummyspace,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto unmap_memstore_desc;

	return result;

unmap_memstore_desc:
	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);

unmap_memptrs_desc:
	kgsl_mmu_unmap(pagetable, device->ringbuffer.memptrs_desc.gpuaddr,
			device->ringbuffer.memptrs_desc.size);
unmap_buffer_desc:
	kgsl_mmu_unmap(pagetable, device->ringbuffer.buffer_desc.gpuaddr,
			device->ringbuffer.buffer_desc.size);
error:
	return result;
}

static int kgsl_yamato_setstate(struct kgsl_device *device, uint32_t flags)
{
	struct kgsl_yamato_device *yamato_device = (struct kgsl_yamato_device *)
								device;
	unsigned int link[32];
	unsigned int *cmds = &link[0];
	int sizedwords = 0;
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

#ifndef CONFIG_MSM_KGSL_MMU
	return 0;
#endif
	KGSL_MEM_DBG("device %p ctxt %p pt %p\n",
			device,
			yamato_device->drawctxt_active,
			device->mmu.hwpagetable);
	/* if possible, set via command stream,
	* otherwise set via direct register writes
	*/
	if (yamato_device->drawctxt_active) {
		KGSL_MEM_DBG("cmds\n");
		if (flags & KGSL_MMUFLAGS_PTUPDATE) {
			/* wait for graphics pipe to be idle */
			*cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
			*cmds++ = 0x00000000;

			/* set page table base */
			*cmds++ = pm4_type0_packet(REG_MH_MMU_PT_BASE, 1);
			*cmds++ = device->mmu.hwpagetable->base.gpuaddr;
			sizedwords += 4;
		}

		if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
			if (!(flags & KGSL_MMUFLAGS_PTUPDATE)) {
				*cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE,
								1);
				*cmds++ = 0x00000000;
				sizedwords += 2;
			}
			*cmds++ = pm4_type0_packet(REG_MH_MMU_INVALIDATE, 1);
			*cmds++ = mh_mmu_invalidate;
			sizedwords += 2;
		}

		if (flags & KGSL_MMUFLAGS_PTUPDATE) {
			/* HW workaround: to resolve MMU page fault interrupts
			* caused by the VGT.It prevents the CP PFP from filling
			* the VGT DMA request fifo too early,thereby ensuring
			* that the VGT will not fetch vertex/bin data until
			* after the page table base register has been updated.
			*
			* Two null DRAW_INDX_BIN packets are inserted right
			* after the page table base update, followed by a
			* wait for idle. The null packets will fill up the
			* VGT DMA request fifo and prevent any further
			* vertex/bin updates from occurring until the wait
			* has finished. */
			*cmds++ = pm4_type3_packet(PM4_SET_CONSTANT, 2);
			*cmds++ = (0x4 << 16) |
				(REG_PA_SU_SC_MODE_CNTL - 0x2000);
			*cmds++ = 0;	  /* disable faceness generation */
			*cmds++ = pm4_type3_packet(PM4_SET_BIN_BASE_OFFSET, 1);
			*cmds++ = device->mmu.dummyspace.gpuaddr;
			*cmds++ = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
			*cmds++ = 0;	  /* viz query info */
			*cmds++ = 0x0003C004; /* draw indicator */
			*cmds++ = 0;	  /* bin base */
			*cmds++ = 3;	  /* bin size */
			*cmds++ = device->mmu.dummyspace.gpuaddr; /* dma base */
			*cmds++ = 6;	  /* dma size */
			*cmds++ = pm4_type3_packet(PM4_DRAW_INDX_BIN, 6);
			*cmds++ = 0;	  /* viz query info */
			*cmds++ = 0x0003C004; /* draw indicator */
			*cmds++ = 0;	  /* bin base */
			*cmds++ = 3;	  /* bin size */
			/* dma base */
			*cmds++ = device->mmu.dummyspace.gpuaddr;
			*cmds++ = 6;	  /* dma size */
			*cmds++ = pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1);
			*cmds++ = 0x00000000;
			sizedwords += 21;
		}

		if (flags & (KGSL_MMUFLAGS_PTUPDATE | KGSL_MMUFLAGS_TLBFLUSH)) {
			*cmds++ = pm4_type3_packet(PM4_INVALIDATE_STATE, 1);
			*cmds++ = 0x7fff; /* invalidate all base pointers */
			sizedwords += 2;
		}

		kgsl_ringbuffer_issuecmds(device, KGSL_CMD_FLAGS_PMODE,
					&link[0], sizedwords);
	} else {
		KGSL_MEM_DBG("regs\n");

		if (flags & KGSL_MMUFLAGS_PTUPDATE) {
			kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);
			kgsl_yamato_regwrite(device, REG_MH_MMU_PT_BASE,
				     device->mmu.hwpagetable->base.gpuaddr);
		}

		if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
			kgsl_yamato_regwrite(device, REG_MH_MMU_INVALIDATE,
					   mh_mmu_invalidate);
		}
	}

	return 0;
}

static unsigned int
kgsl_yamato_getchipid(struct kgsl_device *device)
{
	unsigned int chipid;
	unsigned int coreid, majorid, minorid, patchid, revid;

	/* YDX */
	kgsl_yamato_regread(device, REG_RBBM_PERIPHID1, &coreid);
	coreid &= 0xF;

	kgsl_yamato_regread(device, REG_RBBM_PERIPHID2, &majorid);
	majorid = (majorid >> 4) & 0xF;

	kgsl_yamato_regread(device, REG_RBBM_PATCH_RELEASE, &revid);
	/* this is a 16bit field, but extremely unlikely it would ever get
	* this high
	*/
	minorid = ((revid >> 0)  & 0xFF);


	patchid = ((revid >> 16) & 0xFF);

	chipid  = ((coreid << 24) | (majorid << 16) |
			(minorid << 8) | (patchid << 0));

	/* Hardware revision 211 (8650) returns the wrong chip ID */
	if (chipid == KGSL_CHIPID_YAMATODX_REV21)
		chipid = KGSL_CHIPID_YAMATODX_REV211;

	/* Workaround Hardware revision issue of Z470 */
	if (chipid == KGSL_CHIPID_LEIA_REV470_TEMP)
		chipid = KGSL_CHIPID_LEIA_REV470;


	return chipid;
}

int __init
kgsl_yamato_init(struct kgsl_device *device, struct kgsl_devconfig *config)
{
	struct kgsl_yamato_device *yamato_device = (struct kgsl_yamato_device *)
								device;
	int status = -EINVAL;
	struct kgsl_memregion *regspace = &device->regspace;
	//unsigned int memflags = KGSL_MEMFLAGS_ALIGNPAGE | KGSL_MEMFLAGS_CONPHYS;

	KGSL_DRV_VDBG("enter (device=%p, config=%p)\n", device, config);

	if (device->flags & KGSL_FLAGS_INITIALIZED) {
		KGSL_DRV_VDBG("return %d\n", 0);
		return 0;
	}

	init_waitqueue_head(&yamato_device->ib1_wq);
	setup_timer(&device->idle_timer, kgsl_timer, (unsigned long)device);
	INIT_WORK(&device->idle_check_ws, kgsl_idle_check);

	memcpy(regspace, &config->regspace, sizeof(device->regspace));
	if (regspace->mmio_phys_base == 0 || regspace->sizebytes == 0) {
		KGSL_DRV_ERR("dev %d invalid regspace\n", device->id);
		goto error;
	}
	if (!request_mem_region(regspace->mmio_phys_base,
				regspace->sizebytes, DRIVER_NAME)) {
		KGSL_DRV_ERR("request_mem_region failed for register memory\n");
		status = -ENODEV;
		goto error;
	}

	regspace->mmio_virt_base = ioremap(regspace->mmio_phys_base,
					   regspace->sizebytes);
	KGSL_MEM_INFO("ioremap(regs) = %p\n", regspace->mmio_virt_base);
	if (regspace->mmio_virt_base == NULL) {
		KGSL_DRV_ERR("ioremap failed for register memory\n");
		status = -ENODEV;
		goto error_release_mem;
	}

	status = request_irq(kgsl_driver.yamato_interrupt_num, kgsl_yamato_isr,
			     IRQF_TRIGGER_HIGH, DRIVER_NAME, device);
	if (status) {
		KGSL_DRV_ERR("request_irq(%d) returned %d\n",
			      kgsl_driver.yamato_interrupt_num, status);
		goto error_iounmap;
	}
	kgsl_driver.yamato_have_irq = 1;
	disable_irq(kgsl_driver.yamato_interrupt_num);

	KGSL_DRV_INFO("dev %d regs phys 0x%08x size 0x%08x virt %p\n",
			device->id, regspace->mmio_phys_base,
			regspace->sizebytes, regspace->mmio_virt_base);


	memcpy(&yamato_device->gmemspace, &config->gmemspace,
			sizeof(yamato_device->gmemspace));

	device->id = KGSL_DEVICE_YAMATO;
	init_completion(&device->hwaccess_gate);
	device->interval_timeout = INTERVAL_YAMATO_TIMEOUT;

	ATOMIC_INIT_NOTIFIER_HEAD(&device->ts_notifier_list);

	kgsl_yamato_getfunctable(&device->ftbl);
	if (config->mmu_config) {
		device->mmu.config    = config->mmu_config;
		device->mmu.mpu_base  = config->mpu_base;
		device->mmu.mpu_range = config->mpu_range;
		device->mmu.va_base	  = config->va_base;
		device->mmu.va_range  = config->va_range;
	}

	status = kgsl_mmu_init(device);
	if (status != 0) {
		status = -ENODEV;
		goto error_free_irq;
	}

	status = kgsl_cmdstream_init(device);
	if (status != 0) {
		status = -ENODEV;
		goto error_close_mmu;
	}

	status = kgsl_sharedmem_alloc_coherent(&device->memstore,
					       sizeof(device->memstore));
	if (status != 0)  {
		status = -ENODEV;
		goto error_close_cmdstream;
	}
	status = kgsl_ringbuffer_init(device);
	if (status != 0)
		goto error_free_memstore;

	status = kgsl_drawctxt_init(device);
	if (status != 0) {
		goto error_close_rb;
	}

	device->flags |= KGSL_FLAGS_INITIALIZED;
	return 0;

error_close_rb:
	kgsl_ringbuffer_close(&device->ringbuffer);
error_free_memstore:
	kgsl_sharedmem_free(&device->memstore);
error_close_cmdstream:
	kgsl_cmdstream_close(device);
error_close_mmu:
	kgsl_mmu_close(device);
error_free_irq:
	free_irq(kgsl_driver.yamato_interrupt_num, NULL);
	kgsl_driver.yamato_have_irq = 0;
error_iounmap:
	iounmap(regspace->mmio_virt_base);
	regspace->mmio_virt_base = NULL;
error_release_mem:
	release_mem_region(regspace->mmio_phys_base, regspace->sizebytes);
error:
	return status;
}

int kgsl_yamato_close(struct kgsl_device *device)
{
	struct kgsl_memregion *regspace = &device->regspace;

	kgsl_ringbuffer_close(&device->ringbuffer);
	if (device->memstore.hostptr)
		kgsl_sharedmem_free(&device->memstore);

	kgsl_mmu_close(device);

	kgsl_cmdstream_close(device);

	if (regspace->mmio_virt_base != NULL) {
		KGSL_MEM_INFO("iounmap(regs) = %p\n", regspace->mmio_virt_base);
		iounmap(regspace->mmio_virt_base);
		regspace->mmio_virt_base = NULL;
		release_mem_region(regspace->mmio_phys_base,
					regspace->sizebytes);
	}
	free_irq(kgsl_driver.yamato_interrupt_num, NULL);
	kgsl_driver.yamato_have_irq = 0;

	KGSL_DRV_VDBG("return %d\n", 0);
	device->flags &= ~KGSL_FLAGS_INITIALIZED;
	return 0;
}

static int kgsl_yamato_start(struct kgsl_device *device)
{
	int status = -EINVAL;
	struct kgsl_yamato_device *yamato_device = (struct kgsl_yamato_device *)
							device;
	int init_reftimestamp = 0x7fffffff;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	if (!(device->flags & KGSL_FLAGS_INITIALIZED)) {
		KGSL_DRV_ERR("Trying to start uninitialized device.\n");
		return -EINVAL;
	}

	if (device->flags & KGSL_FLAGS_STARTED) {
		KGSL_DRV_VDBG("already started");
		return 0;
	}

	kgsl_driver.power_flags |= KGSL_PWRFLAGS_YAMATO_CLK_OFF |
		KGSL_PWRFLAGS_YAMATO_POWER_OFF | KGSL_PWRFLAGS_YAMATO_IRQ_OFF;

	/* Turn the clocks on before the power.  Required for some platforms,
	   has no adverse effect on the others */
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_POWER_ON);

	kgsl_driver.is_suspended = KGSL_FALSE;

	device->chip_id = kgsl_yamato_getchipid(device);

	if (kgsl_mmu_start(device))
		goto error_clk_off;

	/*We need to make sure all blocks are powered up and clocked before
	*issuing a soft reset.  The overrides will then be turned off (set to 0)
	*/
	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE1, 0xfffffffe);
	if (device->chip_id == CHIP_REV_251)
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0x000000ff);
	else
		kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0xffffffff);

	kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0xFFFFFFFF);

	/* The core is in an indeterminate state until the reset completes
	 * after 50ms.
	 */
	msleep(50);

	kgsl_yamato_regwrite(device, REG_RBBM_SOFT_RESET, 0x00000000);

	kgsl_yamato_regwrite(device, REG_RBBM_CNTL, 0x00004442);

	kgsl_yamato_regwrite(device, REG_MH_ARBITER_CONFIG,
				KGSL_CFG_YAMATO_MHARB);

	kgsl_yamato_regwrite(device, REG_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030f27);
	kgsl_yamato_regwrite(device, REG_MH_CLNT_INTF_CTRL_CONFIG2, 0x00472747);

	kgsl_yamato_regwrite(device, REG_SQ_VS_PROGRAM, 0x00000000);
	kgsl_yamato_regwrite(device, REG_SQ_PS_PROGRAM, 0x00000000);


	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE1, 0);
	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0);

	kgsl_sharedmem_set(&device->memstore, 0, 0, device->memstore.size);

	kgsl_sharedmem_writel(&device->memstore,
			     KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
			     init_reftimestamp);

	kgsl_yamato_regwrite(device, REG_RBBM_DEBUG, 0x00080000);


	KGSL_DRV_DBG("enabling RBBM interrupts mask 0x%08lx\n",
		     GSL_RBBM_INT_MASK);
	kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, GSL_RBBM_INT_MASK);

	/* make sure SQ interrupts are disabled */
	kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

	kgsl_yamato_gmeminit(yamato_device);

	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_ON);

	status = kgsl_ringbuffer_start(&device->ringbuffer);
	if (status != 0)
		goto error_irq_off;

	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);
	device->flags |= KGSL_FLAGS_STARTED;
	device->hwaccess_blocked = KGSL_FALSE;
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	pr_info("msm_kgsl: initialized dev=%d mmu=%s "
		"per_process_pagetable=on\n",
		device->id, kgsl_mmu_isenabled(&device->mmu) ? "on" : "off");
#else
	pr_info("msm_kgsl: initialized dev=%d mmu=%s "
		"per_process_pagetable=off\n",
		device->id, kgsl_mmu_isenabled(&device->mmu) ? "on" : "off");
#endif
	KGSL_DRV_VDBG("return %d\n", status);
	return status;

error_clk_off:
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_POWER_OFF);
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_OFF);
error_irq_off:
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_ON);
	kgsl_mmu_stop(device);
	return status;
}

static int kgsl_yamato_stop(struct kgsl_device *device)
{
	del_timer(&device->idle_timer);
	if (device->flags & KGSL_FLAGS_STARTED) {
		kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, 0);

		kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

		kgsl_drawctxt_close(device);

		kgsl_ringbuffer_stop(&device->ringbuffer);

		kgsl_yamato_gmemclose(device);

		kgsl_mmu_stop(device);

		kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_OFF);
		/* For some platforms, power needs to go off before clocks */
		kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_POWER_OFF);
		kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_OFF);
		device->hwaccess_blocked = KGSL_TRUE;

		device->flags &= ~KGSL_FLAGS_STARTED;
	}

	return 0;
}

static struct kgsl_yamato_device *kgsl_get_yamato_device(void)
{
	static struct kgsl_yamato_device yamato_device;

	return &yamato_device;
}

struct kgsl_device *kgsl_get_yamato_generic_device(void)
{
	struct kgsl_yamato_device *yamato_device;

	yamato_device = kgsl_get_yamato_device();
	return &yamato_device->dev;
}

static int kgsl_yamato_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;
	struct kgsl_yamato_device *yamato_device = (struct kgsl_yamato_device *)
							device;

	switch (type) {
	case KGSL_PROP_DEVICE_INFO:
		{
			struct kgsl_devinfo devinfo;

			if (sizebytes != sizeof(devinfo)) {
				status = -EINVAL;
				break;
			}

			memset(&devinfo, 0, sizeof(devinfo));
			devinfo.device_id = device->id+1;
			devinfo.chip_id = device->chip_id;
			devinfo.mmu_enabled = kgsl_mmu_isenabled(&device->mmu);
			devinfo.gmem_hostbaseaddr = (unsigned int)
					yamato_device->gmemspace.mmio_virt_base;
			devinfo.gmem_gpubaseaddr = yamato_device->gmemspace.
					gpu_base;
			devinfo.gmem_sizebytes = yamato_device->gmemspace.
					sizebytes;

			if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
					0) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_DEVICE_SHADOW:
		{
			struct kgsl_shadowprop shadowprop;

			if (sizebytes != sizeof(shadowprop)) {
				status = -EINVAL;
				break;
			}
			memset(&shadowprop, 0, sizeof(shadowprop));
			if (device->memstore.hostptr) {
				/*NOTE: with mmu enabled, gpuaddr doesn't mean
				 * anything to mmap().
				 */
				shadowprop.gpuaddr = device->memstore.physaddr;
				shadowprop.size = device->memstore.size;
				shadowprop.flags = KGSL_FLAGS_INITIALIZED;
			}
			if (copy_to_user(value, &shadowprop,
				sizeof(shadowprop))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_MMU_ENABLE:
		{
#ifdef CONFIG_MSM_KGSL_MMU
			int mmuProp = 1;
#else
			int mmuProp = 0;
#endif
			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &mmuProp, sizeof(mmuProp))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_INTERRUPT_WAITS:
		{
			int int_waits = 1;
			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &int_waits, sizeof(int))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	default:
		status = -EINVAL;
	}

	return status;
}

/* Caller must hold the driver mutex. */
int kgsl_yamato_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = -EINVAL;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;
	struct kgsl_mmu_debug mmu_dbg;
	unsigned int rbbm_status;
	int idle_count = 0;
#define IDLE_COUNT_MAX 1500000

	KGSL_DRV_VDBG("enter (device=%p, timeout=%d)\n", device, timeout);

	(void)timeout;

	/* first, wait until the CP has consumed all the commands in
	 * the ring buffer
	 */
	if (rb->flags & KGSL_FLAGS_STARTED) {
		do {
			idle_count++;
			GSL_RB_GET_READPTR(rb, &rb->rptr);

		} while (rb->rptr != rb->wptr && idle_count < IDLE_COUNT_MAX);
		if (idle_count == IDLE_COUNT_MAX)
			goto err;
	}
	/* now, wait for the GPU to finish its operations */
	for (idle_count = 0; idle_count < IDLE_COUNT_MAX; idle_count++) {
		kgsl_yamato_regread(device, REG_RBBM_STATUS, &rbbm_status);

		if (rbbm_status == 0x110) {
			status = 0;
			goto done;
		}
	}

err:
	KGSL_DRV_ERR("spun too long waiting for RB to idle\n");
	kgsl_postmortem_dump(device);
	kgsl_ringbuffer_dump(rb);
	kgsl_mmu_debug(&device->mmu, &mmu_dbg);
	BUG();

done:
	KGSL_DRV_VDBG("return %d\n", status);

	return status;
}

static unsigned int kgsl_yamato_isidle(struct kgsl_device *device)
{
	int status = KGSL_FALSE;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;
	unsigned int rbbm_status;

	if (rb->flags & KGSL_FLAGS_STARTED) {
		/* Is the ring buffer is empty? */
		GSL_RB_GET_READPTR(rb, &rb->rptr);
		if (rb->rptr == rb->wptr) {
			/* Is the core idle? */
			kgsl_yamato_regread(device, REG_RBBM_STATUS,
					    &rbbm_status);
			if (rbbm_status == 0x110)
				status = KGSL_TRUE;
		}
	}

	return status;
}

/******************************************************************/
/* Caller must hold the driver mutex. */
static int kgsl_yamato_sleep(struct kgsl_device *device, const int idle)
{
	int status = KGSL_SUCCESS;

	/* Skip this request if we're already sleeping. */
	if (device->hwaccess_blocked == KGSL_FALSE) {
		/* See if the device is idle. If it is, we can shut down */
		/* the core clock until the next attempt to access the HW. */
		if (idle == KGSL_TRUE || kgsl_yamato_isidle(device)) {
			kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_OFF);
			/* Turn off the core clocks */
			status = kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_OFF);

			/* Block further access to this core until it's awake */
			device->hwaccess_blocked = KGSL_TRUE;
		} else {
			status = KGSL_FAILURE;
		}
	}

	return status;
}

/******************************************************************/
/* Caller must hold the driver mutex. */
static int kgsl_yamato_wake(struct kgsl_device *device)
{
	int status = KGSL_SUCCESS;

	/* Turn on the core clocks */
	status = kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_ON);

	/* Re-enable HW access */
	device->hwaccess_blocked = KGSL_FALSE;
	complete_all(&device->hwaccess_gate);
	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);

	KGSL_DRV_VDBG("<-- kgsl_yamato_wake(). Return value %d\n", status);

	return status;
}


/******************************************************************/
/* Caller must hold the driver mutex. */
static int kgsl_yamato_suspend(struct kgsl_device *device)
{
	int status;

	/* Wait for the device to become idle */
	status = kgsl_yamato_idle(device, IDLE_COUNT_MAX);

	if (status == KGSL_SUCCESS) {
		/* Put the device to sleep. */
		status = kgsl_yamato_sleep(device, KGSL_TRUE);
		/* Don't let the timer wake us during suspended sleep. */
		del_timer(&device->idle_timer);
		/* Get the completion ready to be waited upon. */
		INIT_COMPLETION(device->hwaccess_gate);
	}

	return status;
}

int kgsl_yamato_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;

	KGSL_PRE_HWACCESS();
	if (offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes) {
		KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
		return -ERANGE;
	}

	reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	*value = readl(reg);

	return 0;
}

int kgsl_yamato_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	unsigned int *reg;
	KGSL_PRE_HWACCESS();
	if (offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes) {
		KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
		return -ERANGE;
	}

	reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	writel(value, reg);

	return 0;
}

static int kgsl_check_interrupt_timestamp(struct kgsl_device *device,
					unsigned int timestamp)
{
	int status;
	unsigned int ref_ts, enableflag;

	status = kgsl_check_timestamp(device, timestamp);
	if (!status) {
		mutex_lock(&kgsl_driver.mutex);
		kgsl_sharedmem_readl(&device->memstore, &enableflag,
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable));
		rmb();

		if (enableflag) {
			kgsl_sharedmem_readl(&device->memstore, &ref_ts,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts));
			rmb();
			if (timestamp_cmp(ref_ts, timestamp)) {
				kgsl_sharedmem_writel(&device->memstore,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
				timestamp);
				wmb();
			}
		} else {
			unsigned int cmds[2];
			kgsl_sharedmem_writel(&device->memstore,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
				timestamp);
			enableflag = 1;
			kgsl_sharedmem_writel(&device->memstore,
				KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable),
				enableflag);
			wmb();
			/* submit a dummy packet so that even if all
			* commands upto timestamp get executed we will still
			* get an interrupt */
			cmds[0] = pm4_type3_packet(PM4_NOP, 1);
			cmds[1] = 0;
			kgsl_ringbuffer_issuecmds(device, 0, &cmds[0], 2);
		}
		mutex_unlock(&kgsl_driver.mutex);
	}

	return status;
}

/*
 wait_event_interruptible_timeout checks for the exit condition before
 placing a process in wait q. For conditional interrupts we expect the
 process to already be in its wait q when its exit condition checking
 function is called.
*/
#define kgsl_wait_event_interruptible_timeout(wq, condition, timeout)	\
({									\
	long __ret = timeout;						\
	__wait_event_interruptible_timeout(wq, condition, __ret); 	\
	__ret;								\
})

/* MUST be called with the kgsl_driver.mutex held */
static int kgsl_yamato_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	long status = 0;
	struct kgsl_yamato_device *yamato_device = (struct kgsl_yamato_device *)
								device;

	if (!kgsl_check_timestamp(device, timestamp)) {
		mutex_unlock(&kgsl_driver.mutex);
		/* We need to make sure that the process is placed in wait-q
		 * before its condition is called */
		status = kgsl_wait_event_interruptible_timeout(
				yamato_device->ib1_wq,
				kgsl_check_interrupt_timestamp(device,
					timestamp), msecs_to_jiffies(msecs));
		mutex_lock(&kgsl_driver.mutex);

		if (status > 0)
			status = 0;
		else if (status == 0) {
			if (!kgsl_check_timestamp(device, timestamp)) {
				status = -ETIMEDOUT;
				kgsl_postmortem_dump(device);
			}
		}
	}

	return (int)status;
}

int __init kgsl_yamato_config(struct kgsl_devconfig *devconfig,
				struct platform_device *pdev)
{
	int result = 0;
	struct resource *res = NULL;

	memset(devconfig, 0, sizeof(*devconfig));

	/*find memory regions */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"kgsl_reg_memory");
	if (res == NULL) {
		KGSL_DRV_ERR("platform_get_resource_byname failed\n");
		result = -EINVAL;
		goto done;
	}
	KGSL_DRV_DBG("registers at %08x to %08x\n", res->start, res->end);
	devconfig->regspace.mmio_phys_base = res->start;
	devconfig->regspace.sizebytes = resource_size(res);

	devconfig->gmemspace.gpu_base = 0;
	devconfig->gmemspace.sizebytes = SZ_256K;

	/*note: for all of these behavior masks:
	 *	0 = do not translate
	 *	1 = translate within va_range, otherwise use physical
	 *	2 = translate within va_range, otherwise fault
	 */
	devconfig->mmu_config = 1 /* mmu enable */
		    | (MMU_CONFIG << MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT)
		    | (MMU_CONFIG << MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT);

	/*TODO: these should probably be configurable from platform device
	 * stuff */
	devconfig->va_base = 0x66000000;
#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	devconfig->va_range = SZ_32M;
#else
	devconfig->va_range = 0x0F000000;
#endif

	/* turn off memory protection unit by setting acceptable physical
	 * address range to include all pages. Apparrently MPU causing
	 * problems.
	 */
	devconfig->mpu_base = 0x00000000;
	devconfig->mpu_range = 0xFFFFF000;

	result = 0;
done:
	return result;
}


static long kgsl_yamato_ioctl(struct kgsl_device_private *dev_priv,
			unsigned int cmd,
			unsigned long arg)
{
	int result = 0;
	struct kgsl_drawctxt_set_bin_base_offset binbase;

	switch (cmd) {
	case IOCTL_KGSL_DRAWCTXT_SET_BIN_BASE_OFFSET:
		if (copy_from_user(&binbase, (void __user *)arg,
				   sizeof(binbase))) {
			result = -EFAULT;
			break;
		}

		if (dev_priv->ctxt_id_mask & (1 << binbase.drawctxt_id)) {
			result = kgsl_drawctxt_set_bin_base_offset(
					dev_priv->device,
					binbase.drawctxt_id,
					binbase.offset);
		} else {
			result = -EINVAL;
			KGSL_DRV_ERR("invalid drawctxt drawctxt_id %d"
				     " device_id=%d\n",
				     binbase.drawctxt_id, dev_priv->device->id);
		}
		break;

	default:
		KGSL_DRV_ERR("invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	return result;

}

int kgsl_yamato_getfunctable(struct kgsl_functable *ftbl)
{
	if (ftbl == NULL)
		return KGSL_FAILURE;
	ftbl->device_regread = kgsl_yamato_regread;
	ftbl->device_regwrite = kgsl_yamato_regwrite;
	ftbl->device_setstate = kgsl_yamato_setstate;
	ftbl->device_idle = kgsl_yamato_idle;
	ftbl->device_sleep = kgsl_yamato_sleep;
	ftbl->device_suspend = kgsl_yamato_suspend;
	ftbl->device_wake = kgsl_yamato_wake;
	ftbl->device_start = kgsl_yamato_start;
	ftbl->device_stop = kgsl_yamato_stop;
	ftbl->device_getproperty = kgsl_yamato_getproperty;
	ftbl->device_waittimestamp = kgsl_yamato_waittimestamp;
	ftbl->device_cmdstream_readtimestamp = kgsl_cmdstream_readtimestamp;
	ftbl->device_issueibcmds = kgsl_ringbuffer_issueibcmds;
	ftbl->device_drawctxt_create = kgsl_drawctxt_create;
	ftbl->device_drawctxt_destroy = kgsl_drawctxt_destroy;
	ftbl->device_ioctl = kgsl_yamato_ioctl;
	ftbl->device_setup_pt = kgsl_yamato_setup_pt;
	ftbl->device_cleanup_pt = kgsl_yamato_cleanup_pt;

	return KGSL_SUCCESS;
}
