/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_pm4types.h"
#include "kgsl_cmdstream.h"

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

#define FIRST_TIMEOUT (HZ / 2)
#define INTERVAL_TIMEOUT (HZ / 5)

static struct timer_list idle_timer;
static struct work_struct idle_check;

static int kgsl_yamato_getfunctable(struct kgsl_functable *ftbl);

void kgsl_yamato_timer(unsigned long data)
{
	/* Have work run in a non-interrupt context. */
	schedule_work(&idle_check);
}

void kgsl_yamato_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = &kgsl_driver.yamato_device;

	mutex_lock(&kgsl_driver.mutex);
	if (device->hwaccess_blocked == KGSL_FALSE
	    && device->flags & KGSL_FLAGS_STARTED) {
		if (kgsl_yamato_sleep(device, KGSL_FALSE) == KGSL_FAILURE)
			mod_timer(&idle_timer, jiffies + INTERVAL_TIMEOUT);
	}
	mutex_unlock(&kgsl_driver.mutex);
}

#define CLK_BASE		0xAB800000
#define GLBL_CLK_ENA		0x0
#define GLBL_CLK_STATE		0x04
#define GRP_NS_REG		0x84
#define CLK_HALT_STATEA		0x108
#define VDD_GRP_2D_GFS_CTL	0x274
#define VDD_GRP_GFS_CTL		0x284

static void kgsl_status_dump(struct kgsl_device *device)
{
	void *clk_base = NULL;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;

	clk_base = ioremap(CLK_BASE, 4096);

	if (clk_base == NULL)
		return;

	GSL_RB_GET_READPTR(rb, &rb->rptr);
	printk("power_flags = %08x\n", kgsl_driver.power_flags);
	printk("device->hwaccess_blocked = %08x\n", device->hwaccess_blocked);
	printk("device->flags = %08x\n", device->flags);
	printk("rb->flags = %08x\n", rb->flags);
	printk("rb->rptr = %08x\n", rb->rptr);
	printk("GLBL_CLK_ENA = %08x\n", readl(clk_base + GLBL_CLK_ENA));
	printk("GLBL_CLK_STATE = %08x\n", readl(clk_base + GLBL_CLK_STATE));
	printk("GRP_NS_REG = %08x\n", readl(clk_base + GRP_NS_REG));
	printk("CLK_HALT_STATEA = %08x\n", readl(clk_base + CLK_HALT_STATEA));
	printk("VDD_GRP_2D_GFS_CTL = %08x\n", readl(clk_base + VDD_GRP_2D_GFS_CTL));
	printk("VDD_GRP_GFS_CTL = %08x\n", readl(clk_base + VDD_GRP_GFS_CTL));

	iounmap(clk_base);
}
static void kgsl_register_dump(struct kgsl_device *device)
{
	if (kgsl_cmd_log >= 3) {
		unsigned int reg_value;

		kgsl_yamato_regread(device, REG_RBBM_STATUS, &reg_value);
		KGSL_CMD_ERR("RBBM_STATUS = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_RB_BASE, &reg_value);
		KGSL_CMD_ERR("CP_RB_BASE = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_RB_CNTL, &reg_value);
		KGSL_CMD_ERR("CP_RB_CNTL = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_RB_RPTR_ADDR, &reg_value);
		KGSL_CMD_ERR("CP_RB_RPTR_ADDR = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_RB_RPTR, &reg_value);
		KGSL_CMD_ERR("CP_RB_RPTR = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_RB_WPTR, &reg_value);
		KGSL_CMD_ERR("CP_RB_WPTR = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_RB_RPTR_WR, &reg_value);
		KGSL_CMD_ERR("CP_RB_RPTR_WR = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_INT_CNTL, &reg_value);
		KGSL_CMD_ERR("CP_INT_CNTL = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_INT_STATUS, &reg_value);
		KGSL_CMD_ERR("CP_INT_STATUS = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_ME_CNTL, &reg_value);
		KGSL_CMD_ERR("CP_ME_CNTL = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_ME_STATUS, &reg_value);
		KGSL_CMD_ERR("CP_ME_STATUS = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_RBBM_PM_OVERRIDE1, &reg_value);
		KGSL_CMD_ERR("RBBM_PM_OVERRIDE1 = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_RBBM_PM_OVERRIDE2, &reg_value);
		KGSL_CMD_ERR("RBBM_PM_OVERRIDE2 = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_RBBM_INT_CNTL, &reg_value);
		KGSL_CMD_ERR("RBBM_INT_CNTL = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_RBBM_INT_STATUS, &reg_value);
		KGSL_CMD_ERR("RBBM_INT_STATUS = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_MASTER_INT_SIGNAL, &reg_value);
		KGSL_CMD_ERR("MASTER_INT_SIGNAL = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_IB1_BASE, &reg_value);
		KGSL_CMD_ERR("CP_IB1_BASE = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_IB1_BUFSZ, &reg_value);
		KGSL_CMD_ERR("CP_IB1_BUFSZ = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_IB2_BASE, &reg_value);
		KGSL_CMD_ERR("CP_IB2_BASE = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_IB2_BUFSZ, &reg_value);
		KGSL_CMD_ERR("CP_IB2_BUFSZ = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_CP_STAT, &reg_value);
		KGSL_CMD_ERR("CP_STAT = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_SCRATCH_REG0, &reg_value);
		KGSL_CMD_ERR("SCRATCH_REG0 = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_COHER_SIZE_PM4, &reg_value);
		KGSL_CMD_ERR("COHER_SIZE_PM4 = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_COHER_BASE_PM4, &reg_value);
		KGSL_CMD_ERR("COHER_BASE_PM4 = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_COHER_STATUS_PM4, &reg_value);
		KGSL_CMD_ERR("COHER_STATUS_PM4 = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_RBBM_READ_ERROR, &reg_value);
		KGSL_CMD_ERR("RBBM_READ_ERROR = %08x\n", reg_value);
		kgsl_yamato_regread(device, REG_MH_AXI_ERROR, &reg_value);
		KGSL_CMD_ERR("MH_AXI_ERROR = %08x\n", reg_value);
	}
}

static int kgsl_yamato_gmeminit(struct kgsl_device *device)
{
	union reg_rb_edram_info rb_edram_info;
	unsigned int gmem_size;
	unsigned int edram_value = 0;

	/* make sure edram range is aligned to size */
	BUG_ON(device->gmemspace.gpu_base & (device->gmemspace.sizebytes - 1));

	/* get edram_size value equivalent */
	gmem_size = (device->gmemspace.sizebytes >> 14);
	while (gmem_size >>= 1)
		edram_value++;

	rb_edram_info.val = 0;

	rb_edram_info.f.edram_size = edram_value;
	rb_edram_info.f.edram_mapping_mode = 0; /* EDRAM_MAP_UPPER */
	/* must be aligned to size */
	rb_edram_info.f.edram_range = (device->gmemspace.gpu_base >> 14);

	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, rb_edram_info.val);

	return 0;
}

static int kgsl_yamato_gmemclose(struct kgsl_device *device)
{
	kgsl_yamato_regwrite(device, REG_RB_EDRAM_INFO, 0x00000000);

	return 0;
}

void kgsl_yamato_rbbm_intrcallback(struct kgsl_device *device)
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

void kgsl_yamato_sq_intrcallback(struct kgsl_device *device)
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

	struct kgsl_device *device = &kgsl_driver.yamato_device;
	unsigned int status;

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
	mod_timer(&idle_timer, jiffies + INTERVAL_TIMEOUT);
	return result;
}

int kgsl_yamato_cleanup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
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

int kgsl_yamato_setup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	int result = 0;
	unsigned int gpuaddr;

	BUG_ON(device->ringbuffer.buffer_desc.physaddr == 0);
	BUG_ON(device->ringbuffer.memptrs_desc.physaddr == 0);
	BUG_ON(device->memstore.physaddr == 0);
#ifdef CONFIG_MSM_KGSL_MMU
	BUG_ON(device->mmu.dummyspace.physaddr == 0);
#endif

	result = kgsl_mmu_map(pagetable,
				device->ringbuffer.buffer_desc.physaddr,
				device->ringbuffer.buffer_desc.size,
				GSL_PT_PAGE_RV, &gpuaddr,
				KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K);

	if (result)
		goto error;

	if (device->ringbuffer.buffer_desc.gpuaddr == 0)
		device->ringbuffer.buffer_desc.gpuaddr = gpuaddr;
	BUG_ON(device->ringbuffer.buffer_desc.gpuaddr != gpuaddr);

	result = kgsl_mmu_map(pagetable,
				device->ringbuffer.memptrs_desc.physaddr,
				device->ringbuffer.memptrs_desc.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, &gpuaddr,
				KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K);
	if (result)
		goto unmap_buffer_desc;

	if (device->ringbuffer.memptrs_desc.gpuaddr == 0)
		device->ringbuffer.memptrs_desc.gpuaddr = gpuaddr;
	BUG_ON(device->ringbuffer.memptrs_desc.gpuaddr != gpuaddr);

	result = kgsl_mmu_map(pagetable, device->memstore.physaddr,
				device->memstore.size,
				GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, &gpuaddr,
				KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K);
	if (result)
		goto unmap_memptrs_desc;

	if (device->memstore.gpuaddr == 0)
		device->memstore.gpuaddr = gpuaddr;
	BUG_ON(device->memstore.gpuaddr != gpuaddr);

	result = kgsl_mmu_map(pagetable,
			device->mmu.dummyspace.physaddr,
			device->mmu.dummyspace.size,
			GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, &gpuaddr,
			KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K);

	if (result)
		goto unmap_memstore_desc;

	if (device->mmu.dummyspace.gpuaddr == 0)
		device->mmu.dummyspace.gpuaddr = gpuaddr;
	BUG_ON(device->mmu.dummyspace.gpuaddr != gpuaddr);

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

int kgsl_yamato_setstate(struct kgsl_device *device, uint32_t flags)
{
	unsigned int link[32];
	unsigned int *cmds = &link[0];
	int sizedwords = 0;
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

#ifndef CONFIG_MSM_KGSL_MMU
	return 0;
#endif
	KGSL_MEM_DBG("device %p ctxt %p pt %p\n",
			device,
			device->drawctxt_active,
			device->mmu.hwpagetable);
	/* if possible, set via command stream,
	* otherwise set via direct register writes
	*/
	if (device->drawctxt_active) {
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

	return chipid;
}

int kgsl_yamato_init(struct kgsl_device *device, struct kgsl_devconfig *config)
{
	int status = -EINVAL;
	int init_reftimestamp = 0x7fffffff;
	struct kgsl_memregion *regspace = &device->regspace;
	unsigned int memflags = KGSL_MEMFLAGS_ALIGNPAGE | KGSL_MEMFLAGS_CONPHYS;

	KGSL_DRV_VDBG("enter (device=%p, config=%p)\n", device, config);

	if (device->flags & KGSL_FLAGS_INITIALIZED) {
		KGSL_DRV_VDBG("return %d\n", 0);
		return 0;
	}

	init_waitqueue_head(&device->ib1_wq);

	if (regspace->mmio_virt_base == NULL) {
		memcpy(regspace, &config->regspace, sizeof(device->regspace));
		if (regspace->mmio_phys_base == 0 || regspace->sizebytes == 0) {
			KGSL_DRV_ERR("dev %d invalid regspace\n", device->id);
			goto error;
		}
		if (!request_mem_region(regspace->mmio_phys_base,
					regspace->sizebytes, DRIVER_NAME)) {
			KGSL_DRV_ERR("request_mem_region failed for memory\n");
			status = -ENODEV;
			goto error;
		}

		regspace->mmio_virt_base = ioremap(regspace->mmio_phys_base,
				regspace->sizebytes);
		KGSL_MEM_INFO("ioremap(regs) = %p\n", regspace->mmio_virt_base);
		if (regspace->mmio_virt_base == NULL) {
			KGSL_DRV_ERR("ioremap failed for register memory\n");
			release_mem_region(regspace->mmio_phys_base,
					   regspace->sizebytes);
			memset(regspace, 0, sizeof(regspace));
			status = -ENODEV;
			goto error;
		}

		KGSL_DRV_INFO("dev %d regs phys 0x%08x size 0x%08x virt %p\n",
				device->id, regspace->mmio_phys_base,
				regspace->sizebytes, regspace->mmio_virt_base);
	}

	memcpy(&device->gmemspace, &config->gmemspace,
			sizeof(device->gmemspace));

	device->id = KGSL_DEVICE_YAMATO;
	init_completion(&device->hwaccess_gate);
	device->hwaccess_blocked = KGSL_FALSE;

	kgsl_yamato_getfunctable(&device->ftbl);
	if (config->mmu_config) {
		device->mmu.config    = config->mmu_config;
		device->mmu.mpu_base  = config->mpu_base;
		device->mmu.mpu_range = config->mpu_range;
		device->mmu.va_base	  = config->va_base;
		device->mmu.va_range  = config->va_range;
	}

	device->chip_id = kgsl_yamato_getchipid(device);

	/*We need to make sure all blocks are powered up and clocked before
	*issuing a soft reset.  The overrides will be turned off (set to 0)
	*later in kgsl_yamato_start.
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

	kgsl_yamato_regwrite(device, REG_SQ_VS_PROGRAM, 0x00000000);
	kgsl_yamato_regwrite(device, REG_SQ_PS_PROGRAM, 0x00000000);

	status = kgsl_mmu_init(device);
	if (status != 0) {
		status = -ENODEV;
		goto error;
	}

	status = kgsl_cmdstream_init(device);
	if (status != 0) {
		status = -ENODEV;
		goto error_close_mmu;
	}

	status = kgsl_sharedmem_alloc(memflags, sizeof(device->memstore),
				&device->memstore);
	if (status != 0)  {
		status = -ENODEV;
		goto error_close_cmdstream;
	}
	kgsl_sharedmem_set(&device->memstore, 0, 0, device->memstore.size);

	kgsl_sharedmem_writel(&device->memstore,
			     KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
			     init_reftimestamp);

	kgsl_yamato_regwrite(device, REG_RBBM_DEBUG, 0x00080000);

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	pr_info("msm_kgsl: initialized dev=%d mmu=%s "
		"per_process_pagetable=on\n",
		device->id, kgsl_mmu_isenabled(&device->mmu) ? "on" : "off");
#else
	pr_info("msm_kgsl: initialized dev=%d mmu=%s "
		"per_process_pagetable=off\n",
		device->id, kgsl_mmu_isenabled(&device->mmu) ? "on" : "off");
#endif
	device->flags |= KGSL_FLAGS_INITIALIZED;
	return 0;

error_close_cmdstream:
	kgsl_cmdstream_close(device);
error_close_mmu:
	kgsl_mmu_close(device);
error:
	return status;
}

int kgsl_yamato_close(struct kgsl_device *device)
{
	if (device->memstore.hostptr)
		kgsl_sharedmem_free(&device->memstore);

	kgsl_mmu_close(device);

	kgsl_cmdstream_close(device);

	KGSL_DRV_VDBG("return %d\n", 0);
	device->flags &= ~KGSL_FLAGS_INITIALIZED;
	return 0;
}

int kgsl_yamato_start(struct kgsl_device *device, uint32_t flags)
{
	int status = -EINVAL;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	if (!(device->flags & KGSL_FLAGS_INITIALIZED)) {
		KGSL_DRV_ERR("Trying to start uninitialized device.\n");
		return -EINVAL;
	}

	device->refcnt++;

	if (device->flags & KGSL_FLAGS_STARTED) {
		KGSL_DRV_VDBG("already started");
		return 0;
	}

	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE1, 0);
	kgsl_yamato_regwrite(device, REG_RBBM_PM_OVERRIDE2, 0);

	KGSL_DRV_DBG("enabling RBBM interrupts mask 0x%08lx\n",
		     GSL_RBBM_INT_MASK);
	kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, GSL_RBBM_INT_MASK);

	/* make sure SQ interrupts are disabled */
	kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

	kgsl_yamato_gmeminit(device);

	status = kgsl_ringbuffer_init(device);
	if (status != 0) {
		kgsl_yamato_stop(device);
		return status;
	}

	status = kgsl_drawctxt_init(device);
	if (status != 0) {
		kgsl_yamato_stop(device);
		return status;
	}

	device->flags |= KGSL_FLAGS_STARTED;
	init_timer(&idle_timer);
	idle_timer.function = kgsl_yamato_timer;
	idle_timer.expires = jiffies + FIRST_TIMEOUT;
	add_timer(&idle_timer);
	INIT_WORK(&idle_check, kgsl_yamato_idle_check);

	KGSL_DRV_VDBG("return %d\n", status);
	return status;
}

int kgsl_yamato_stop(struct kgsl_device *device)
{
	del_timer(&idle_timer);
	if (device->flags & KGSL_FLAGS_STARTED) {

		kgsl_yamato_regwrite(device, REG_RBBM_INT_CNTL, 0);

		kgsl_yamato_regwrite(device, REG_SQ_INT_CNTL, 0);

		kgsl_drawctxt_close(device);

		kgsl_ringbuffer_close(&device->ringbuffer);

		kgsl_yamato_gmemclose(device);

		device->flags &= ~KGSL_FLAGS_STARTED;
	}

	return 0;
}

int kgsl_yamato_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;

	switch (type) {
	case KGSL_PROP_DEVICE_INFO:
		{
			struct kgsl_devinfo devinfo;

			if (sizebytes != sizeof(devinfo)) {
				status = -EINVAL;
				break;
			}

			memset(&devinfo, 0, sizeof(devinfo));
			devinfo.device_id = device->id;
			devinfo.chip_id = device->chip_id;
			devinfo.mmu_enabled = kgsl_mmu_isenabled(&device->mmu);
			devinfo.gmem_hostbaseaddr =
				(unsigned int)device->gmemspace.mmio_virt_base;
			devinfo.gmem_gpubaseaddr = device->gmemspace.gpu_base;
			devinfo.gmem_sizebytes = device->gmemspace.sizebytes;

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
	kgsl_register_dump(device);
	kgsl_status_dump(device);
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
int kgsl_yamato_sleep(struct kgsl_device *device, const int idle)
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
int kgsl_yamato_wake(struct kgsl_device *device)
{
	int status = KGSL_SUCCESS;
	/* Turn on the core clocks */
	status = kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_CLK_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_YAMATO_IRQ_ON);

	/* Re-enable HW access */
	device->hwaccess_blocked = KGSL_FALSE;
	complete_all(&device->hwaccess_gate);
	mod_timer(&idle_timer, jiffies + FIRST_TIMEOUT);

	KGSL_DRV_VDBG("<-- kgsl_yamato_wake(). Return value %d\n", status);

	return status;
}

/******************************************************************/
/* Caller must hold the driver mutex. */
int kgsl_yamato_suspend(struct kgsl_device *device)
{
	int status;

	/* Wait for the device to become idle */
	status = kgsl_yamato_idle(device, IDLE_COUNT_MAX);

	if (status == KGSL_SUCCESS) {
		/* Put the device to sleep. */
		status = kgsl_yamato_sleep(device, KGSL_TRUE);
		/* Don't let the timer wake us during suspended sleep. */
		del_timer(&idle_timer);
		/* Get the completion ready to be waited upon. */
		INIT_COMPLETION(device->hwaccess_gate);
	}

	return status;
}

int kgsl_yamato_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;

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

	if (offsetwords*sizeof(uint32_t) >= device->regspace.sizebytes) {
		KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
		return -ERANGE;
	}

	reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	writel(value, reg);

	return 0;
}

static inline int _wait_timestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	long status;

	status = wait_event_interruptible_timeout(device->ib1_wq,
			kgsl_cmdstream_check_timestamp(device, timestamp),
			msecs_to_jiffies(msecs));

	if (status > 0)
		status = 0;
	else if (status == 0) {
		if (!kgsl_cmdstream_check_timestamp(device, timestamp)) {
			status = -ETIMEDOUT;
			kgsl_register_dump(device);
			kgsl_status_dump(device);
			KGSL_CMD_ERR("timestamp = %x\n", timestamp);
			KGSL_CMD_ERR("ts_processed = %x\n",
				kgsl_cmdstream_readtimestamp(device,
						    KGSL_TIMESTAMP_RETIRED));
		}
	}

	return (int)status;
}

/* MUST be called with the kgsl_driver.mutex held */
int kgsl_yamato_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	long status = 0;
	uint32_t ref_ts;
	unsigned int enableflag = 1;
	unsigned int cmd[2];

	KGSL_DRV_INFO("enter (device=%p,timestamp=%d,timeout=0x%08x)\n",
			 device, timestamp, msecs);

	if (!kgsl_cmdstream_check_timestamp(device, timestamp)) {
		kgsl_sharedmem_readl(&device->memstore, &ref_ts,
			KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts));
		if (timestamp_cmp(ref_ts, timestamp)) {
			kgsl_sharedmem_writel(&device->memstore,
				KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts),
				timestamp);
		}

		cmd[0] = pm4_type3_packet(PM4_INTERRUPT, 1);
		cmd[1] = CP_INT_CNTL__IB1_INT_MASK;
		kgsl_ringbuffer_issuecmds(device, KGSL_CMD_FLAGS_NO_TS_CMP,
						cmd, 2);
		kgsl_sharedmem_writel(&device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable),
			enableflag);

		mutex_unlock(&kgsl_driver.mutex);
		status = _wait_timestamp(device, timestamp, msecs);
		mutex_lock(&kgsl_driver.mutex);
	}

	KGSL_DRV_INFO("return %ld\n", status);
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
	devconfig->va_range = SZ_128M;
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

static int kgsl_yamato_getfunctable(struct kgsl_functable *ftbl)
{
	if (ftbl == NULL)
		return KGSL_FAILURE;
	ftbl->device_regread = kgsl_yamato_regread;
	ftbl->device_regwrite = kgsl_yamato_regwrite;
	ftbl->device_setstate = kgsl_yamato_setstate;
	ftbl->device_idle = kgsl_yamato_idle;

	return KGSL_SUCCESS;
}
