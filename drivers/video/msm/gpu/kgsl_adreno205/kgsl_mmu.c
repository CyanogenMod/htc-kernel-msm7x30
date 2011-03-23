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
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#ifdef CONFIG_MSM_KGSL_MMU
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#endif
#include "kgsl_mmu.h"
#include "kgsl_drawctxt.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "yamato_reg.h"
#include "g12_reg.h"
#include "kgsl_device.h"
#include "kgsl_g12.h"
#include "kgsl_yamato.h"

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

	kgsl_regread(device, mmu_reg[device->id].interrupt_status, &status);

	if (status & MH_INTERRUPT_MASK__AXI_READ_ERROR) {
		KGSL_MEM_FATAL("axi read error interrupt\n");
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__AXI_WRITE_ERROR) {
		KGSL_MEM_FATAL("axi write error interrupt\n");
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else if (status & MH_INTERRUPT_MASK__MMU_PAGE_FAULT) {
		kgsl_regread(device, mmu_reg[device->id].page_fault, &reg);
		KGSL_MEM_FATAL("mmu page fault interrupt: %08x\n", reg);
		kgsl_mmu_debug(&device->mmu, &dbg);
	} else {
		KGSL_MEM_DBG("bad bits in REG_MH_INTERRUPT_STATUS %08x\n",
			     status);
	}

	kgsl_regwrite(device, mmu_reg[device->id].interrupt_clear, status);

	/*TODO: figure out how to handle errror interupts.
	* specifically, page faults should probably nuke the client that
	* caused them, but we don't have enough info to figure that out yet.
	*/

	KGSL_MEM_VDBG("return\n");
}

#ifdef DEBUG
void kgsl_mmu_debug(struct kgsl_mmu *mmu, struct kgsl_mmu_debug *regs)
{
	unint32_t id = mmu->device->id;

	memset(regs, 0, sizeof(struct kgsl_mmu_debug));
	kgsl_regread(mmu->device, mmu_reg[id].config, &regs->config);
	kgsl_regread(mmu->device, mmu_reg[id].mpu_base, &regs->mpu_base);
	kgsl_regread(mmu->device, mmu_reg[id].mpu_end, &regs->mpu_end);
	kgsl_regread(mmu->device, mmu_reg[id].va_range, &regs->va_range);
	kgsl_regread(mmu->device, mmu_reg[id].pt_base, &regs->pt_base);
	kgsl_regread(mmu->device, mmu_reg[id].page_fault, &regs->page_fault);
	kgsl_regread(mmu->device, mmu_reg[id].tran_error, &regs->trans_error);
	kgsl_regread(mmu->device, mmu_reg[id].axi_error, &regs->axi_error);
	kgsl_regread(mmu->device, mmu_reg[id].interrupt_mask,
				 &regs->interrupt_mask);
	kgsl_regread(mmu->device, mmu_reg[id].interrupt_status,
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
	//uint32_t flags;

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
	status = kgsl_sharedmem_alloc_coherent(&pagetable->base,
				      pagetable->max_entries * GSL_PTE_SIZE);
	if (status != 0)
		goto err_pool;

	/* reset page table entries
	 * -- all pte's are marked as not dirty initially
	 */
	kgsl_sharedmem_set(&pagetable->base, 0, 0, pagetable->base.size);

	pagetable->base.gpuaddr = pagetable->base.physaddr;

	status = kgsl_setup_pt(pagetable);
	if (status)
		goto err_free_sharedmem;

	list_add(&pagetable->list, &kgsl_driver.pagetable_list);

	KGSL_MEM_VDBG("return %p\n", pagetable);
	return pagetable;

err_free_sharedmem:
	kgsl_sharedmem_free(&pagetable->base);
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
		kgsl_cleanup_pt(pagetable);
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
	//uint32_t flags;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		KGSL_MEM_INFO("MMU already initialized.\n");
		return 0;
	}

	mmu->device = device;

#ifndef CONFIG_MSM_KGSL_MMU
	mmu->config = 0x00000000;
#endif

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	/* make sure aligned to pagesize */
	BUG_ON(mmu->mpu_base & (KGSL_PAGESIZE - 1));
	BUG_ON((mmu->mpu_base + mmu->mpu_range) & (KGSL_PAGESIZE - 1));

	mmu->tlb_flags = 0;

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {
		/*make sure virtual address range is a multiple of 64Kb */
		BUG_ON(mmu->va_range & ((1 << 16) - 1));

		/* allocate memory used for completing r/w operations that
		 * cannot be mapped by the MMU
		 */
		status = kgsl_sharedmem_alloc_coherent(&mmu->dummyspace, 64);
		if (status != 0) {
			KGSL_MEM_ERR
			    ("Unable to allocate dummy space memory.\n");
			goto error;
		}

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);

	}
	mmu->flags |= KGSL_FLAGS_INITIALIZED;

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;

error:
	return status;
}

int kgsl_mmu_start(struct kgsl_device *device)
{
	/*
	 * intialize device mmu
	 *
	 * call this with the global lock held
	 */
	int status;
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		KGSL_MEM_INFO("MMU already started.\n");
		return 0;
	}

	/* MMU not enabled */
	if ((mmu->config & 0x1) == 0) {
		KGSL_MEM_VDBG("return %d\n", 0);
		return 0;
	}

	mmu->flags |= KGSL_FLAGS_STARTED;

	/* setup MMU and sub-client behavior */
	kgsl_regwrite(device, mmu_reg[device->id].config, mmu->config);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK);
	kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask,
				GSL_MMU_INT_MASK);

	/* idle device */
	kgsl_idle(device,  KGSL_TIMEOUT_DEFAULT);

	/* define physical memory range accessible by the core */
	kgsl_regwrite(device, mmu_reg[device->id].mpu_base, mmu->mpu_base);
	kgsl_regwrite(device, mmu_reg[device->id].mpu_end,
			mmu->mpu_base + mmu->mpu_range);

	/* enable axi interrupts */
	KGSL_MEM_DBG("enabling mmu interrupts mask=0x%08lx\n",
		     GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);
	kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask,
			GSL_MMU_INT_MASK | MH_INTERRUPT_MASK__MMU_PAGE_FAULT);

	/* sub-client MMU lookups require address translation */
	if ((mmu->config & ~0x1) > 0) {

		kgsl_sharedmem_set(&mmu->dummyspace, 0, 0,
				   mmu->dummyspace.size);

		/* TRAN_ERROR needs a 32 byte (32 byte aligned) chunk of memory
		 * to complete transactions in case of an MMU fault. Note that
		 * we'll leave the bottom 32 bytes of the dummyspace for other
		 * purposes (e.g. use it when dummy read cycles are needed
		 * for other blocks */
		kgsl_regwrite(device, mmu_reg[device->id].tran_error,
						mmu->dummyspace.physaddr + 32);

		BUG_ON(mmu->defaultpagetable == NULL);
		mmu->hwpagetable = mmu->defaultpagetable;

		kgsl_regwrite(device, mmu_reg[device->id].pt_page,
			      mmu->hwpagetable->base.gpuaddr);
		kgsl_regwrite(device, mmu_reg[device->id].va_range,
			      (mmu->hwpagetable->va_base |
			      (mmu->hwpagetable->va_range >> 16)));
		status = kgsl_setstate(device, KGSL_MMUFLAGS_TLBFLUSH);
		if (status) {
			KGSL_MEM_ERR("Failed to setstate TLBFLUSH\n");
			goto error;
		}
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
error:
	/* disable MMU */
	kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask, 0);
	kgsl_regwrite(device, mmu_reg[device->id].config, 0x00000000);
	return status;
}



#ifdef CONFIG_MSM_KGSL_MMU

unsigned int kgsl_virtaddr_to_physaddr(unsigned int virtaddr)
{
	unsigned int physaddr = 0;
	pgd_t *pgd_ptr = NULL;
	pmd_t *pmd_ptr = NULL;
	pte_t *pte_ptr = NULL, pte;

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
	pte = *pte_ptr;
	physaddr = pte_pfn(pte);
	pte_unmap(pte_ptr);
	physaddr <<= PAGE_SHIFT;
	return physaddr;
}

int
kgsl_mmu_map(struct kgsl_pagetable *pagetable,
				unsigned int address,
				int range,
				unsigned int protflags,
				unsigned int *gpuaddr,
				unsigned int flags)
{
	int numpages, i;
	unsigned int pte, ptefirst, ptelast, physaddr;
	int flushtlb, alloc_size;
	unsigned int align = flags & KGSL_MEMFLAGS_ALIGN_MASK;
	struct kgsl_device *device;

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
		if (flags & KGSL_MEMFLAGS_CONPHYS)
			physaddr = address;
		else if (flags & KGSL_MEMFLAGS_VMALLOC_MEM) {
			physaddr = vmalloc_to_pfn((void *)address);
			physaddr <<= PAGE_SHIFT;
		} else if (flags & KGSL_MEMFLAGS_HOSTADDR)
			physaddr = kgsl_virtaddr_to_physaddr(address);
		else
			physaddr = 0;

		if (physaddr) {
			kgsl_pt_map_set(pagetable, pte, physaddr | protflags);
		} else {
			KGSL_MEM_ERR
			("Unable to find physaddr for address: %x\n",
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
		for (i = 0; i < kgsl_driver.num_devs; i++) {
			device = kgsl_driver.devp[i];
			if (device != NULL) {
				if ((device->flags & KGSL_FLAGS_INITIALIZED) &&
				    (pagetable == device->mmu.hwpagetable)) {
					device->mmu.tlb_flags |=
							KGSL_MMUFLAGS_TLBFLUSH;
				}
			}
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

int kgsl_mmu_map_global(struct kgsl_pagetable *pagetable,
			struct kgsl_memdesc *memdesc, unsigned int protflags,
			unsigned int flags)
{
	int result = -EINVAL;
	unsigned int gpuaddr = 0;

	if (memdesc == NULL)
		goto error;

	result = kgsl_mmu_map(pagetable, memdesc->physaddr, memdesc->size,
				protflags, &gpuaddr, flags);
	if (result)
		goto error;

	/*global mappings must have the same gpu address in all pagetables*/
	if (memdesc->gpuaddr == 0)
		memdesc->gpuaddr = gpuaddr;

	else if (memdesc->gpuaddr != gpuaddr) {
		KGSL_MEM_ERR("pt %p addr mismatch phys 0x%08x gpu 0x%0x 0x%08x",
				pagetable, memdesc->physaddr,
				memdesc->gpuaddr, gpuaddr);
		goto error_unmap;
	}
	return result;
error_unmap:
	kgsl_mmu_unmap(pagetable, gpuaddr, memdesc->size);
error:
	return result;
}

int kgsl_mmu_stop(struct kgsl_device *device)
{
	/*
	 *  stop device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		/* disable mh interrupts */
		KGSL_MEM_DBG("disabling mmu interrupts\n");
		/* disable MMU */
		kgsl_regwrite(device, mmu_reg[device->id].interrupt_mask, 0);
		kgsl_regwrite(device, mmu_reg[device->id].config, 0x00000000);

		mmu->flags &= ~KGSL_FLAGS_STARTED;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;


}

int kgsl_mmu_close(struct kgsl_device *device)
{
	/*
	 *  close device mmu
	 *
	 *  call this with the global lock held
	 */
	struct kgsl_mmu *mmu = &device->mmu;

	KGSL_MEM_VDBG("enter (device=%p)\n", device);

	if (mmu->flags & KGSL_FLAGS_INITIALIZED0) {
		if (mmu->dummyspace.gpuaddr)
			kgsl_sharedmem_free(&mmu->dummyspace);

		mmu->flags &= ~KGSL_FLAGS_INITIALIZED;
		mmu->flags &= ~KGSL_FLAGS_INITIALIZED0;
	}

	KGSL_MEM_VDBG("return %d\n", 0);

	return 0;
}
