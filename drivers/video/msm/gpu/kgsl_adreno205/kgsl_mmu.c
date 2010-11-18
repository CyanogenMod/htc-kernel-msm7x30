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
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#ifdef CONFIG_MSM_KGSL_MMU
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#endif
#include "kgsl_mmu.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "yamato_reg.h"
#include "g12_reg.h"
#include "kgsl_device.h"

struct kgsl_pte_debug {
	unsigned int read:1;
	unsigned int write:1;
	unsigned int dirty:1;
	unsigned int reserved:9;
	unsigned int phyaddr:20;
};

#define GSL_PTE_SIZE	4
#define GSL_PT_EXTRA_ENTRIES	16


#define GSL_PT_PAGE_BITS_MASK	0x00000007
#define GSL_PT_PAGE_ADDR_MASK	(~(KGSL_PAGESIZE - 1))

#define GSL_MMU_INT_MASK \
	(MH_INTERRUPT_MASK__AXI_READ_ERROR | \
	 MH_INTERRUPT_MASK__AXI_WRITE_ERROR)

static const struct kgsl_mmu_reg mmu_reg[KGSL_DEVICE_MAX] = {
	{
		.config = REG_MH_MMU_CONFIG,
		.mpu_base = REG_MH_MMU_MPU_BASE,
		.mpu_end = REG_MH_MMU_MPU_END,
		.va_range = REG_MH_MMU_VA_RANGE,
		.pt_page = REG_MH_MMU_PT_BASE,
		.page_fault = REG_MH_MMU_PAGE_FAULT,
		.tran_error = REG_MH_MMU_TRAN_ERROR,
		.invalidate = REG_MH_MMU_INVALIDATE,
		.interrupt_mask = REG_MH_INTERRUPT_MASK,
		.interrupt_status = REG_MH_INTERRUPT_STATUS,
		.interrupt_clear = REG_MH_INTERRUPT_CLEAR
	},
	{
		.config = ADDR_MH_MMU_CONFIG,
		.mpu_base = ADDR_MH_MMU_MPU_BASE,
		.mpu_end = ADDR_MH_MMU_MPU_END,
		.va_range = ADDR_MH_MMU_VA_RANGE,
		.pt_page = ADDR_MH_MMU_PT_BASE,
		.page_fault = ADDR_MH_MMU_PAGE_FAULT,
		.tran_error = ADDR_MH_MMU_TRAN_ERROR,
		.invalidate = ADDR_MH_MMU_INVALIDATE,
		.interrupt_mask = ADDR_MH_INTERRUPT_MASK,
		.interrupt_status = ADDR_MH_INTERRUPT_STATUS,
		.interrupt_clear = ADDR_MH_INTERRUPT_CLEAR
	}
};

uint32_t kgsl_pt_entry_get(struct kgsl_pagetable *pt, uint32_t va)
{
	return (va - pt->va_base) >> KGSL_PAGESIZE_SHIFT;
}

uint32_t kgsl_pt_map_get(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte];
}

void kgsl_pt_map_set(struct kgsl_pagetable *pt, uint32_t pte, uint32_t val)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] = val;
}
#define GSL_PT_MAP_DEBUG(pte)	((struct kgsl_pte_debug *) \
		&gsl_pt_map_get(pagetable, pte))

void kgsl_pt_map_setbits(struct kgsl_pagetable *pt, uint32_t pte, uint32_t bits)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] |= bits;
}

void kgsl_pt_map_setaddr(struct kgsl_pagetable *pt, uint32_t pte,
					uint32_t pageaddr)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	uint32_t val = baseptr[pte];
	val &= ~GSL_PT_PAGE_ADDR_MASK;
	val |= (pageaddr & GSL_PT_PAGE_ADDR_MASK);
	baseptr[pte] = val;
}

void kgsl_pt_map_resetall(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] &= GSL_PT_PAGE_DIRTY;
}

void kgsl_pt_map_resetbits(struct kgsl_pagetable *pt, uint32_t pte,
				uint32_t bits)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	baseptr[pte] &= ~(bits & GSL_PT_PAGE_BITS_MASK);
}

int kgsl_pt_map_isdirty(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte] & GSL_PT_PAGE_DIRTY;
}

uint32_t kgsl_pt_map_getaddr(struct kgsl_pagetable *pt, uint32_t pte)
{
	uint32_t *baseptr = (uint32_t *)pt->base.hostptr;
	return baseptr[pte] & GSL_PT_PAGE_ADDR_MASK;
}

void kgsl_mh_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0;
	unsigned int reg;
	struct kgsl_mmu_debug dbg;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	kgsl_regread(device, mmu_reg[device->id-1].interrupt_status, &status);

	if (status & MH_INTERRUPT_MASK__AXI_READ_ERROR) {
		KGSL_MEM_FATAL("axi read error interrupt\n");
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__AXI_WRITE_ERROR) {
		KGSL_MEM_FATAL("axi write error interrupt\n");
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__MMU_PAGE_FAULT) {
		kgsl_regread(device, mmu_reg[device->id-1].page_fault, &reg);
		KGSL_MEM_FATAL("mmu page fault interrupt: %08x\n", reg);
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else {
		KGSL_MEM_DBG("bad bits in REG_MH_INTERRUPT_STATUS %08x\n",
			     status);
	}

	kgsl_regwrite(device, mmu_reg[device->id-1].interrupt_clear, status);

	/*TODO: figure out how to handle errror interupts.
	* specifically, page faults should probably nuke the client that
	* caused them, but we don't have enough info to figure that out yet.
	*/

	KGSL_MEM_VDBG("return\n");
}

#ifdef DEBUG
void kgsl_mmu_debug(struct kgsl_mmu *mmu, struct kgsl_mmu_debug *regs)
{
	unsigned id = mmu->device->id;

	memset(regs, 0, sizeof(struct kgsl_mmu_debug));
	kgsl_regread(mmu->device, mmu_reg[id-1].config, &regs->config);
	kgsl_regread(mmu->device, mmu_reg[id-1].mpu_base, &regs->mpu_base);
	kgsl_regread(mmu->device, mmu_reg[id-1].mpu_end, &regs->mpu_end);
	kgsl_regread(mmu->device, mmu_reg[id-1].va_range, &regs->va_range);
//	kgsl_regread(mmu->device, mmu_reg[id-1].pt_base, &regs->pt_base);
	kgsl_regread(mmu->device, mmu_reg[id-1].page_fault, &regs->page_fault);
	kgsl_regread(mmu->device, mmu_reg[id-1].tran_error, &regs->trans_error);
//	kgsl_regread(mmu->device, mmu_reg[id-1].axi_error, &regs->axi_error);
	kgsl_regread(mmu->device, mmu_reg[id-1].interrupt_mask,
				 &regs->interrupt_mask);
	kgsl_regread(mmu->device, mmu_reg[id-1].interrupt_status,
				&regs->interrupt_status);


	KGSL_MEM_DBG("mmu config %08x mpu_base %08x mpu_end %08x\n",
		     regs->config, regs->mpu_base, regs->mpu_end);
	KGSL_MEM_DBG("mmu va_range %08x pt_base %08x \n",
		     regs->va_range, regs->pt_base);
	KGSL_MEM_DBG("mmu page_fault %08x tran_err %08x\n",
		     regs->page_fault, regs->trans_error);
	KGSL_MEM_DBG("mmu int mask %08x int status %08x\n",
			regs->interrupt_mask, regs->interrupt_status);
}
#endif

static struct kgsl_pagetable *kgsl_mmu_createpagetableobject(
				struct kgsl_mmu *mmu,
				unsigned int name)
{
	int status = 0;
	struct kgsl_pagetable *pagetable = NULL;
	uint32_t flags;

	KGSL_MEM_VDBG("enter (mmu=%p)\n", mmu);

	pagetable = kzalloc(sizeof(struct kgsl_pagetable), GFP_KERNEL);
	if (pagetable == NULL) {
		KGSL_MEM_ERR("Unable to allocate pagetable object.\n");
		return NULL;
	}

	pagetable->refcnt = 1;

	pagetable->name = name;
	pagetable->va_base = mmu->va_base;
	pagetable->va_range = mmu->va_range;
	pagetable->last_superpte = 0;
	pagetable->max_entries = (mmu->va_range >> KGSL_PAGESIZE_SHIFT)
				 + GSL_PT_EXTRA_ENTRIES;

	pagetable->tlbflushfilter.size = (mmu->va_range /
				(PAGE_SIZE * GSL_PT_SUPER_PTE * 8)) + 1;
	pagetable->tlbflushfilter.base = (unsigned int *)
			kzalloc(pagetable->tlbflushfilter.size, GFP_KERNEL);
	if (!pagetable->tlbflushfilter.base) {
		KGSL_MEM_ERR("Failed to create tlbflushfilter\n");
		goto err_alloc;
	}
	GSL_TLBFLUSH_FILTER_RESET();

	pagetable->pool = gen_pool_create(KGSL_PAGESIZE_SHIFT, -1);
	if (pagetable->pool == NULL) {
		KGSL_MEM_ERR("Unable to allocate virtualaddr pool.\n");
		goto err_flushfilter;
	}

	if (gen_pool_add(pagetable->pool, pagetable->va_base,
				pagetable->va_range, -1)) {
		KGSL_MEM_ERR("gen_pool_create failed for pagetable %p\n",
				pagetable);
		goto err_pool;
	}

	/* allocate page table memory */
	flags = (KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS
		 | KGSL_MEMFLAGS_STRICTREQUEST);
	status = kgsl_sharedmem_alloc(flags,
				      pagetable->max_entries * GSL_PTE_SIZE,
				      &pagetable->base);

	if (status == 0) {
		/* reset page table entries
		 * -- all pte's are marked as not dirty initially
		 */
		kgsl_sharedmem_set(&pagetable->base, 0, 0,
				   pagetable->base.size);
	}
	pagetable->base.gpuaddr = pagetable->base.physaddr;

	list_add(&pagetable->list, &kgsl_driver.pagetable_list);

	KGSL_MEM_VDBG("return %p\n", pagetable);
	return pagetable;

err_pool:
	gen_pool_destroy(pagetable->pool);
err_flushfilter:
	kfree(pagetable->tlbflushfilter.base);
err_alloc:
	kfree(pagetable);

	return NULL;
}

static void kgsl_mmu_destroypagetable(struct kgsl_pagetable *pagetable)
{
	KGSL_MEM_VDBG("enter (pagetable=%p)\n", pagetable);

	list_del(&pagetable->list);

	if (pagetable) {
		if (pagetable->base.gpuaddr)
			kgsl_sharedmem_free(&pagetable->base);

		if (pagetable->pool) {
			gen_pool_destroy(pagetable->pool);
			pagetable->pool = NULL;
		}

		if (pagetable->tlbflushfilter.base) {
			pagetable->tlbflushfilter.size = 0;
			kfree(pagetable->tlbflushfilter.base);
			pagetable->tlbflushfilter.base = NULL;
		}

		kfree(pagetable);

	}
	KGSL_MEM_VDBG("return 0x%08x\n", 0);
}

struct kgsl_pagetable *kgsl_mmu_getpagetable(struct kgsl_mmu *mmu,
					     unsigned long name)
{
	struct kgsl_pagetable *pt;

	if (mmu == NULL)
		return NULL;

	mutex_lock(&kgsl_driver.pt_mutex);

	list_for_each_entry(pt,	&kgsl_driver.pagetable_list, list) {
		if (pt->name == name) {
			pt->refcnt++;
			mutex_unlock(&kgsl_driver.pt_mutex);
			return pt;
		}
	}

	pt = kgsl_mmu_createpagetableobject(mmu, name);
	mutex_unlock(&kgsl_driver.pt_mutex);

	return pt;
}

void kgsl_mmu_putpagetable(struct kgsl_pagetable *pagetable)
{

	if (pagetable == NULL)
		return;

	mutex_lock(&kgsl_driver.pt_mutex);

	if (!--pagetable->refcnt)
		kgsl_mmu_destroypagetable(pagetable);

	mutex_unlock(&kgsl_driver.pt_mutex);
}

int kgsl_mmu_setstate(struct kgsl_device *device,
				struct kgsl_pagetable *pagetable)
{
	int status = 0;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p, pagetable=%p)\n", device, pagetable);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* page table not current, then setup mmu to use new
		 *  specified page table
		 */
		KGSL_MEM_INFO("from %p to %p\n", mmu->hwpagetable, pagetable);
		if (mmu->hwpagetable != pagetable) {
			mmu->hwpagetable = pagetable;

			/* call device specific set page table */
			status = kgsl_setstate(mmu->device,
				KGSL_MMUFLAGS_TLBFLUSH |
				KGSL_MMUFLAGS_PTUPDATE);

		}
	}

	KGSL_MEM_VDBG("return %d\n", status);

	return status;
}

int kgsl_mmu_init(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	uint32_t flags;
	struct kgsl_mmu *mmu = &device->mmu;
#ifdef _DEBUG
	struct kgsl_mmu_debug regs;
#endif /* _DEBUG */

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		KGSL_MEM_INFO("MMU already initialized.\n");
		return 0;
	}

	mmu->device = device;

#ifndef CONFIG_MSM_KGSL_MMU
	mmu->config = 0x00000000;
#endif

	/* setup MMU and sub-client behavior */
	kgsl_regwrite(device, mmu_reg[device->id-1].config, mmu->config);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK);
	kgsl_regwrite(device, mmu_reg[device->id-1].interrupt_mask,
				GSL_MMU_INT_MASK);

	mmu->flags |= KGSL_FLAGS_INITIALIZED0;

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	/* idle device */
	kgsl_idle(device,  KGSL_TIMEOUT_DEFAULT);

	/* make sure aligned to pagesize */
	BUG_ON(mmu->mpu_base & (KGSL_PAGESIZE - 1));
	BUG_ON((mmu->mpu_base + mmu->mpu_range) & (KGSL_PAGESIZE - 1));

	/* define physical memory range accessible by the core */
	kgsl_regwrite(device, mmu_reg[device->id-1].mpu_base, mmu->mpu_base);
	kgsl_regwrite(device, mmu_reg[device->id-1].mpu_end,
			mmu->mpu_base + mmu->mpu_range);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);
	kgsl_regwrite(device, mmu_reg[device->id-1].interrupt_mask,
			GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);

	mmu->flags |= KGSL_FLAGS_INITIALIZED;
	mmu->tlb_flags = 0;

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {
		/*make sure virtual address range is a multiple of 64Kb */
		BUG_ON(mmu->va_range & ((1 << 16) - 1));

		/* allocate memory used for completing r/w operations that
		 * cannot be mapped by the MMU
		 */
		flags = (KGSL_MEMFLAGS_ALIGN4K | KGSL_MEMFLAGS_CONPHYS
			 | KGSL_MEMFLAGS_STRICTREQUEST);
		status = kgsl_sharedmem_alloc(flags, 64, &mmu->dummyspace);
		if (status != 0) {
			KGSL_MEM_ERR
			    ("Unable to allocate dummy space memory.\n");
			kgsl_mmu_close(device);
			return status;
		}

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);

		/* TRAN_ERROR needs a 32 byte (32 byte aligned) chunk of memory
		 * to complete transactions in case of an MMU fault. Note that
		 * we'll leave the bottom 32 bytes of the dummyspace for other
		 * purposes (e.g. use it when dummy read cycles are needed
		 * for other blocks */
		kgsl_regwrite(device, mmu_reg[device->id-1].tran_error,
						mmu->dummyspace.physaddr + 32);

		mmu->defaultpagetable = kgsl_mmu_getpagetable(mmu,
							 KGSL_MMU_GLOBAL_PT);

		if (!mmu->defaultpagetable) {
			KGSL_MEM_ERR("Failed to create global page table\n");
			kgsl_mmu_close(device);
			return -ENOMEM;
		}
		mmu->hwpagetable = mmu->defaultpagetable;

		kgsl_regwrite(device, mmu_reg[device->id-1].pt_page,
					mmu->hwpagetable->base.gpuaddr);
		kgsl_regwrite(device, mmu_reg[device->id-1].va_range,
				(mmu->hwpagetable->va_base |
				(mmu->hwpagetable->va_range >> 16)));
		status = kgsl_setstate(device, KGSL_MMUFLAGS_TLBFLUSH);

		if (status) {
			kgsl_mmu_close(device);
			return status;
		}
		mmu->flags |= KGSL_FLAGS_STARTED;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

#ifdef CONFIG_MSM_KGSL_MMU

pte_t *kgsl_get_pte_from_vaddr(unsigned int virtaddr)
{
	pgd_t *pgd_ptr = NULL;
	pmd_t *pmd_ptr = NULL;
	pte_t *pte_ptr = NULL;

	pgd_ptr = pgd_offset(current->mm, virtaddr);
	if (pgd_none(*pgd) || pgd_bad(*pgd)) {
		KGSL_MEM_ERR
		    ("Invalid pgd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pmd_ptr = pmd_offset(pgd_ptr, virtaddr);
	if (pmd_none(*pmd_ptr) || pmd_bad(*pmd_ptr)) {
		KGSL_MEM_ERR
		    ("Invalid pmd entry found while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}

	pte_ptr = pte_offset_map(pmd_ptr, virtaddr);
	if (!pte_ptr) {
		KGSL_MEM_ERR
		    ("Unable to map pte entry while trying to convert virtual "
		     "address to physical\n");
		return 0;
	}
	return pte_ptr;
}

int
kgsl_mmu_map(struct kgsl_pagetable *pagetable,
				unsigned int address,
				int range,
				unsigned int protflags,
				unsigned int *gpuaddr,
				unsigned int flags)
{
	int numpages;
	unsigned int pte, ptefirst, ptelast, physaddr;
	int flushtlb, alloc_size;
	int phys_contiguous = flags & KGSL_MEMFLAGS_CONPHYS;
	unsigned int align = flags & KGSL_MEMFLAGS_ALIGN_MASK;

	KGSL_MEM_VDBG("enter (pt=%p, physaddr=%08x, range=%08d, gpuaddr=%p)\n",
		      pagetable, address, range, gpuaddr);


	BUG_ON(protflags & ~(GSL_PT_PAGE_RV | GSL_PT_PAGE_WV));
	BUG_ON(protflags == 0);
	BUG_ON(range <= 0);

	/* Only support 4K and 8K alignment for now */
	if (align != KGSL_MEMFLAGS_ALIGN8K && align != KGSL_MEMFLAGS_ALIGN4K) {
		KGSL_MEM_ERR("Cannot map memory according to "
			     "requested flags: %08x\n", flags);
		return -EINVAL;
	}

	/* Make sure address being mapped is at 4K boundary */
	if (!IS_ALIGNED(address, KGSL_PAGESIZE) || range & ~KGSL_PAGEMASK) {
		KGSL_MEM_ERR("Cannot map address not aligned "
			     "at page boundary: address: %08x, range: %08x\n",
			     address, range);
		return -EINVAL;
	}
	alloc_size = range;
	if (align == KGSL_MEMFLAGS_ALIGN8K)
		alloc_size += KGSL_PAGESIZE;

	*gpuaddr = gen_pool_alloc(pagetable->pool, alloc_size);
	if (*gpuaddr == 0) {
		KGSL_MEM_ERR("gen_pool_alloc failed: %d\n", alloc_size);
		return -ENOMEM;
	}

	if (align == KGSL_MEMFLAGS_ALIGN8K) {
		if (*gpuaddr & ((1 << 13) - 1)) {
			/* Not 8k aligned, align it */
			gen_pool_free(pagetable->pool, *gpuaddr, KGSL_PAGESIZE);
			*gpuaddr = *gpuaddr + KGSL_PAGESIZE;
		} else
			gen_pool_free(pagetable->pool, *gpuaddr + range,
				      KGSL_PAGESIZE);
	}

	numpages = (range >> KGSL_PAGESIZE_SHIFT);

	ptefirst = kgsl_pt_entry_get(pagetable, *gpuaddr);
	ptelast = ptefirst + numpages;

	pte = ptefirst;
	flushtlb = 0;

	/* tlb needs to be flushed when the first and last pte are not at
	* superpte boundaries */
	if ((ptefirst & (GSL_PT_SUPER_PTE - 1)) != 0 ||
		((ptelast + 1) & (GSL_PT_SUPER_PTE-1)) != 0)
		flushtlb = 1;

	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		uint32_t val = kgsl_pt_map_getaddr(pagetable, pte);
		BUG_ON(val != 0 && val != GSL_PT_PAGE_DIRTY);
#endif
		if ((pte & (GSL_PT_SUPER_PTE-1)) == 0)
			if (GSL_TLBFLUSH_FILTER_ISDIRTY(pte / GSL_PT_SUPER_PTE))
				flushtlb = 1;
		/* mark pte as in use */
		if (phys_contiguous)
			physaddr = address;
		else {
			physaddr = vmalloc_to_pfn((void *)address);
			physaddr <<= PAGE_SHIFT;
		}

		if (physaddr) {
			kgsl_pt_map_set(pagetable, pte, physaddr | protflags);
		} else {
			KGSL_MEM_ERR
			("Unable to find physaddr for vmallloc address: %x\n",
			     address);
			kgsl_mmu_unmap(pagetable, *gpuaddr, range);
			return -EFAULT;
		}
		address += KGSL_PAGESIZE;
	}

	KGSL_MEM_INFO("pt %p p %08x g %08x pte f %d l %d n %d f %d\n",
		      pagetable, address, *gpuaddr, ptefirst, ptelast,
		      numpages, flushtlb);

	mb();

	/* Invalidate tlb only if current page table used by GPU is the
	 * pagetable that we used to allocate */
	if (flushtlb) {
		if ((kgsl_driver.yamato_device.flags & KGSL_FLAGS_INITIALIZED)
				&& (pagetable == kgsl_driver.yamato_device.mmu.
				hwpagetable)) {
			kgsl_driver.yamato_device.mmu.tlb_flags |=
				KGSL_MMUFLAGS_TLBFLUSH;
		}
		if ((kgsl_driver.g12_device.flags & KGSL_FLAGS_INITIALIZED) &&
				(pagetable == kgsl_driver.g12_device.mmu.
				hwpagetable)) {
			kgsl_driver.g12_device.mmu.tlb_flags |=
				KGSL_MMUFLAGS_TLBFLUSH;
		}
		GSL_TLBFLUSH_FILTER_RESET();
	}


	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}

int
kgsl_mmu_unmap(struct kgsl_pagetable *pagetable, unsigned int gpuaddr,
		int range)
{
	unsigned int numpages;
	unsigned int pte, ptefirst, ptelast, superpte;

	KGSL_MEM_VDBG("enter (pt=%p, gpuaddr=0x%08x, range=%d)\n",
			pagetable, gpuaddr, range);

	BUG_ON(range <= 0);

	numpages = (range >> KGSL_PAGESIZE_SHIFT);
	if (range & (KGSL_PAGESIZE - 1))
		numpages++;

	ptefirst = kgsl_pt_entry_get(pagetable, gpuaddr);
	ptelast = ptefirst + numpages;

	KGSL_MEM_INFO("pt %p gpu %08x pte first %d last %d numpages %d\n",
		      pagetable, gpuaddr, ptefirst, ptelast, numpages);

	superpte = ptefirst - (ptefirst & (GSL_PT_SUPER_PTE-1));
	GSL_TLBFLUSH_FILTER_SETDIRTY(superpte / GSL_PT_SUPER_PTE);
	for (pte = ptefirst; pte < ptelast; pte++) {
#ifdef VERBOSE_DEBUG
		/* check if PTE exists */
		BUG_ON(!kgsl_pt_map_getaddr(pagetable, pte));
#endif
		kgsl_pt_map_set(pagetable, pte, GSL_PT_PAGE_DIRTY);
		superpte = pte - (pte & (GSL_PT_SUPER_PTE - 1));
		if (pte == superpte)
			GSL_TLBFLUSH_FILTER_SETDIRTY(superpte /
				GSL_PT_SUPER_PTE);
	}

	mb();

	gen_pool_free(pagetable->pool, gpuaddr, range);

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
#endif /*CONFIG_MSM_KGSL_MMU*/

int kgsl_mmu_close(struct kgsl_device *device)
{
	/*
	 *  close device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;
#ifdef _DEBUG
	int i;
#endif /* _DEBUG */

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		/* disable mh interrupts */
		KGSL_MEM_DBG("disabling mmu interrupts\n");
		/* disable MMU */
		kgsl_regwrite(device, mmu_reg[device->id-1].interrupt_mask, 0);
		kgsl_regwrite(device, mmu_reg[device->id-1].config, 0x00000000);

		if (mmu->dummyspace.gpuaddr)
			kgsl_sharedmem_free(&mmu->dummyspace);

		mmu->flags &= ~KGSL_FLAGS_STARTED;
		mmu->flags &= ~KGSL_FLAGS_INITIALIZED;
		mmu->flags &= ~KGSL_FLAGS_INITIALIZED0;
		if (mmu->defaultpagetable) {
			kgsl_mmu_putpagetable(mmu->defaultpagetable);
			if (mmu->hwpagetable == mmu->defaultpagetable)
				mmu->hwpagetable = NULL;
			mmu->defaultpagetable = NULL;
		}
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
