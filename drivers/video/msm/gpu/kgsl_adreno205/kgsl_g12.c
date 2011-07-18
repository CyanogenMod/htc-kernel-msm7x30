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
#include <linux/workqueue.h>
#include <linux/notifier.h>

#include "kgsl.h"
#include "kgsl_g12.h"
#include "kgsl_log.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_g12_cmdstream.h"
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_sharedmem.h"
#include "kgsl_g12_vgv3types.h"

#include "g12_reg.h"

#define GSL_VGC_INT_MASK \
	 (REG_VGC_IRQSTATUS__MH_MASK | \
	  REG_VGC_IRQSTATUS__G2D_MASK | \
	  REG_VGC_IRQSTATUS__FIFO_MASK)

/* G12 MH arbiter config*/
#define KGSL_G12_CFG_G12_MHARB \
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

#define KGSL_G12_TIMESTAMP_EPSILON 20000
#define KGSL_G12_IDLE_COUNT_MAX 1000000

static int kgsl_g12_start(struct kgsl_device *device);
static int kgsl_g12_stop(struct kgsl_device *device);
static int kgsl_g12_idle(struct kgsl_device *device, unsigned int timeout);
static int kgsl_g12_sleep(struct kgsl_device *device, const int idle);
static int kgsl_g12_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs);

irqreturn_t kgsl_g12_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;
	unsigned int status;
	struct kgsl_device *device;
	struct kgsl_g12_device *g12_device;

	device = (struct kgsl_device *) data;
	g12_device =  (struct kgsl_g12_device *) device;

	kgsl_g12_regread(device, ADDR_VGC_IRQSTATUS >> 2, &status);

	if (status & GSL_VGC_INT_MASK) {
		kgsl_g12_regwrite(device,
			ADDR_VGC_IRQSTATUS >> 2, status & GSL_VGC_INT_MASK);

		result = IRQ_HANDLED;

		if (status & REG_VGC_IRQSTATUS__FIFO_MASK)
			KGSL_DRV_ERR("g12 fifo interrupt\n");
		if (status & REG_VGC_IRQSTATUS__MH_MASK)
			kgsl_mh_intrcallback(device);
		if (status & REG_VGC_IRQSTATUS__G2D_MASK) {
			int count;

			KGSL_DRV_VDBG("g12 g2d interrupt\n");
			kgsl_g12_regread(device,
					 ADDR_VGC_IRQ_ACTIVE_CNT >> 2,
					 &count);

			count >>= 8;
			count &= 255;
			g12_device->timestamp += count;

			wake_up_interruptible(&(g12_device->wait_timestamp_wq));

			atomic_notifier_call_chain(
				&(device->ts_notifier_list),
				KGSL_DEVICE_G12, NULL);
		}
	}

	mod_timer(&device->idle_timer, jiffies + device->interval_timeout);

	return result;
}

static int kgsl_g12_cleanup_pt(struct kgsl_device *device,
			       struct kgsl_pagetable *pagetable)
{
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;

	if (device->mmu.defaultpagetable == pagetable)
		device->mmu.defaultpagetable = NULL;

	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);

	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);

	kgsl_mmu_unmap(pagetable, g12_device->ringbuffer.cmdbufdesc.gpuaddr,
			g12_device->ringbuffer.cmdbufdesc.size);
	return 0;
}

static int kgsl_g12_setup_pt(struct kgsl_device *device,
			     struct kgsl_pagetable *pagetable)
{
	int result = 0;
	unsigned int flags = KGSL_MEMFLAGS_CONPHYS | KGSL_MEMFLAGS_ALIGN4K;
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;

	if (device->mmu.defaultpagetable == NULL)
		device->mmu.defaultpagetable = pagetable;

	result = kgsl_mmu_map_global(pagetable, &device->mmu.dummyspace,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto error;

	result = kgsl_mmu_map_global(pagetable, &device->memstore,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV, flags);
	if (result)
		goto error_unmap_dummy;

	result = kgsl_mmu_map_global(pagetable,
				     &g12_device->ringbuffer.cmdbufdesc,
				     GSL_PT_PAGE_RV, flags);
	if (result)
		goto error_unmap_memstore;
	return result;

error_unmap_dummy:
	kgsl_mmu_unmap(pagetable, device->mmu.dummyspace.gpuaddr,
			device->mmu.dummyspace.size);
error_unmap_memstore:
	kgsl_mmu_unmap(pagetable, device->memstore.gpuaddr,
			device->memstore.size);
error:
	return result;
}

int kgsl_g12_setstate(struct kgsl_device *device, uint32_t flags)
{
#ifdef CONFIG_MSM_KGSL_MMU
	unsigned int mh_mmu_invalidate = 0x00000003; /*invalidate all and tc */

	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
		kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);
		kgsl_g12_regwrite(device, ADDR_MH_MMU_PT_BASE,
				     device->mmu.hwpagetable->base.gpuaddr);
		kgsl_g12_regwrite(device, ADDR_MH_MMU_VA_RANGE,
				     (device->mmu.hwpagetable->
				      va_base | (device->mmu.hwpagetable->
						 va_range >> 16)));
		kgsl_g12_regwrite(device, ADDR_MH_MMU_INVALIDATE,
				     mh_mmu_invalidate);
	}

	if (flags & KGSL_MMUFLAGS_TLBFLUSH)
		kgsl_g12_regwrite(device, ADDR_MH_MMU_INVALIDATE,
			     mh_mmu_invalidate);
#endif
	return 0;
}

int __init
kgsl_g12_init(struct kgsl_device *device,
		struct kgsl_devconfig *config)
{
	int status = -EINVAL;
	struct kgsl_memregion *regspace = &device->regspace;
	//unsigned int memflags = KGSL_MEMFLAGS_ALIGNPAGE |	KGSL_MEMFLAGS_CONPHYS;
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;


	KGSL_DRV_VDBG("enter (device=%p, config=%p)\n", device, config);

	if (device->flags & KGSL_FLAGS_INITIALIZED) {
		KGSL_DRV_VDBG("return %d\n", 0);
		return 0;
	}

	device->flags |= KGSL_FLAGS_INITIALIZED;

	/* initilization of timestamp wait */
	init_waitqueue_head(&(g12_device->wait_timestamp_wq));

	memcpy(regspace, &config->regspace, sizeof(device->regspace));
	if (regspace->mmio_phys_base == 0 || regspace->sizebytes == 0) {
		KGSL_DRV_ERR("dev %d invalid regspace\n", device->id);
		status = -ENODEV;
		goto error;
	}
	if (!request_mem_region(regspace->mmio_phys_base,
				regspace->sizebytes, DRIVER_NAME)) {
		KGSL_DRV_ERR("request_mem_region failed for " \
					"register memory\n");
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

	status = request_irq(kgsl_driver.g12_interrupt_num, kgsl_g12_isr,
			     IRQF_TRIGGER_HIGH, DRIVER_NAME, device);
	if (status) {
		KGSL_DRV_ERR("request_irq(%d) returned %d\n",
			      kgsl_driver.g12_interrupt_num, status);
		goto error_iounmap;
	}
	kgsl_driver.g12_have_irq = 1;
	disable_irq(kgsl_driver.g12_interrupt_num);

	KGSL_DRV_INFO("dev_id %d regs phys 0x%08x size 0x%08x virt %p\n",
			device->id, regspace->mmio_phys_base,
			regspace->sizebytes, regspace->mmio_virt_base);


	device->id = KGSL_DEVICE_G12;
	init_completion(&device->hwaccess_gate);
	kgsl_g12_getfunctable(&device->ftbl);
	device->interval_timeout = INTERVAL_G12_TIMEOUT;

	ATOMIC_INIT_NOTIFIER_HEAD(&device->ts_notifier_list);

	setup_timer(&device->idle_timer, kgsl_timer, (unsigned long) device);
	INIT_WORK(&device->idle_check_ws, kgsl_idle_check);

	INIT_LIST_HEAD(&device->ringbuffer.memqueue);

	printk(KERN_INFO "kgsl mmu config 0x%x\n", config->mmu_config);
	if (config->mmu_config) {
		device->mmu.config    = config->mmu_config;
		device->mmu.mpu_base  = config->mpu_base;
		device->mmu.mpu_range = config->mpu_range;
		device->mmu.va_base   = config->va_base;
		device->mmu.va_range  = config->va_range;
	}

	status = kgsl_g12_cmdstream_init(device);
	if (status != 0)
		goto error_free_irq;

	status = kgsl_mmu_init(device);
	if (status != 0)
		goto error_close_cmdstream;

	status = kgsl_sharedmem_alloc_coherent(&device->memstore,
						sizeof(device->memstore));
	if (status != 0)
		goto error_close_mmu;

	kgsl_sharedmem_set(&device->memstore, 0, 0, device->memstore.size);

	return 0;

error_close_mmu:
	kgsl_mmu_close(device);
error_close_cmdstream:
	kgsl_g12_cmdstream_close(device);
error_free_irq:
	free_irq(kgsl_driver.g12_interrupt_num, NULL);
	kgsl_driver.g12_have_irq = 0;
error_iounmap:
	iounmap(regspace->mmio_virt_base);
	regspace->mmio_virt_base = NULL;
error_release_mem:
	release_mem_region(regspace->mmio_phys_base, regspace->sizebytes);
error:
	return status;
}

int kgsl_g12_close(struct kgsl_device *device)
{
	struct kgsl_memregion *regspace = &device->regspace;

	if (device->memstore.hostptr)
		kgsl_sharedmem_free(&device->memstore);

	kgsl_mmu_close(device);

	kgsl_g12_cmdstream_close(device);

	if (regspace->mmio_virt_base != NULL) {
		KGSL_MEM_INFO("iounmap(regs) = %p\n",
				regspace->mmio_virt_base);
		iounmap(regspace->mmio_virt_base);
		regspace->mmio_virt_base = NULL;
		release_mem_region(regspace->mmio_phys_base,
					regspace->sizebytes);
	}
	free_irq(kgsl_driver.g12_interrupt_num, NULL);
	kgsl_driver.g12_have_irq = 0;

	KGSL_DRV_VDBG("return %d\n", 0);
	device->flags &= ~KGSL_FLAGS_INITIALIZED;
	return 0;
}

static int kgsl_g12_start(struct kgsl_device *device)
{
	int status = 0;
	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	if (!(device->flags & KGSL_FLAGS_INITIALIZED)) {
		KGSL_DRV_ERR("Trying to start uninitialized device.\n");
		return -EINVAL;
	}

	if (device->flags & KGSL_FLAGS_STARTED) {
		KGSL_DRV_VDBG("already started");
		return 0;
	}
	kgsl_driver.power_flags |= KGSL_PWRFLAGS_G12_CLK_OFF |
		KGSL_PWRFLAGS_G12_POWER_OFF | KGSL_PWRFLAGS_G12_IRQ_OFF;

	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_POWER_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_ON);

	/* Set up MH arbiter.  MH offsets are considered to be dword
	 * based, therefore no down shift. */
	kgsl_g12_regwrite(device, ADDR_MH_ARBITER_CONFIG,
			  KGSL_G12_CFG_G12_MHARB);

	kgsl_g12_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG1, 0x00030F27);
	kgsl_g12_regwrite(device, ADDR_MH_CLNT_INTF_CTRL_CONFIG2, 0x004B274F);

	kgsl_g12_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0x3);

	status = kgsl_mmu_start(device);
	if (status)
		goto error_clk_off;

	status = kgsl_g12_cmdstream_start(device);
	if (status)
		goto error_mmu_stop;

	device->hwaccess_blocked = KGSL_FALSE;
	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);
	device->flags |= KGSL_FLAGS_STARTED;
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_IRQ_ON);
	return 0;
error_clk_off:
	kgsl_g12_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0);
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_OFF);
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_POWER_OFF);
error_mmu_stop:
	kgsl_mmu_stop(device);
	return status;
}

static int kgsl_g12_stop(struct kgsl_device *device)
{
	kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);

	del_timer(&device->idle_timer);

	kgsl_mmu_stop(device);

	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_IRQ_OFF);
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_OFF);
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_POWER_OFF);
	device->hwaccess_blocked = KGSL_TRUE;

	device->flags &= ~KGSL_FLAGS_STARTED;
	return 0;
}

static struct kgsl_g12_device *kgsl_get_g12_device(void)
{
	static struct kgsl_g12_device g12_device;

	return &g12_device;
}

struct kgsl_device *kgsl_get_g12_generic_device(void)
{
	struct kgsl_g12_device *g12_device = kgsl_get_g12_device();

	return &g12_device->dev;
}

static int kgsl_g12_getproperty(struct kgsl_device *device,
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
		devinfo.device_id = device->id+1;
		devinfo.chip_id = device->chip_id;
		devinfo.mmu_enabled = kgsl_mmu_isenabled(&device->mmu);

		if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
				0) {
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

	default:
	KGSL_DRV_ERR("invalid property: %d\n", type);
	status = -EINVAL;

	}
	return status;
}

static int kgsl_g12_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = KGSL_SUCCESS;
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;

	KGSL_DRV_VDBG("enter (device=%p, timeout=%d)\n", device, timeout);

	if (device->flags & KGSL_FLAGS_STARTED) {
		if (g12_device->current_timestamp > g12_device->timestamp)
			status = kgsl_g12_waittimestamp(device,
					g12_device->current_timestamp, timeout);
	}

	if (status)
		KGSL_DRV_ERR("Error, kgsl_g12_waittimestamp() timed out\n");

	KGSL_DRV_VDBG("return %d\n", status);

	return status;
}

static unsigned int kgsl_g12_isidle(struct kgsl_g12_device *g12_device)
{
	int status = 0;
	int timestamp = g12_device->timestamp;

	if (timestamp == g12_device->current_timestamp)
		status = KGSL_TRUE;

	return status;
}

/******************************************************************/
/* Caller must hold the driver mutex. */
static int kgsl_g12_sleep(struct kgsl_device *device, const int idle)
{
	int status = KGSL_SUCCESS;
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;

	KGSL_DRV_DBG("kgsl_g12_sleep!!!\n");

	/* Skip this request if we're already sleeping. */
	if (device->hwaccess_blocked == KGSL_FALSE) {
		/* See if the device is idle. If it is, we can shut down */
		/* the core clock until the next attempt to access the HW. */
		if (idle || kgsl_g12_isidle(g12_device)) {
			kgsl_pwrctrl(KGSL_PWRFLAGS_G12_IRQ_OFF);
			/* Turn off the core clocks */
			status = kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_OFF);

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
static int kgsl_g12_wake(struct kgsl_device *device)
{
	int status = KGSL_SUCCESS;

	/* Turn on the core clocks */
	status = kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_IRQ_ON);

	/* Re-enable HW access */
	device->hwaccess_blocked = KGSL_FALSE;
	complete_all(&device->hwaccess_gate);
	mod_timer(&device->idle_timer, jiffies + FIRST_TIMEOUT);

	KGSL_DRV_VDBG("<-- kgsl_g12_wake(). Return value %d\n", status);

	return status;
}

/******************************************************************/
/* Caller must hold the driver mutex. */
static int kgsl_g12_suspend(struct kgsl_device *device)
{
	int status;

	/* Wait for the device to become idle */
	status = kgsl_g12_idle(device, KGSL_G12_IDLE_COUNT_MAX);

	if (status == KGSL_SUCCESS) {
		/* Put the device to sleep. */
		status = kgsl_g12_sleep(device, true);
		/* Don't let the timer wake us during suspended sleep. */
		del_timer(&device->idle_timer);
		/* Get the completion ready to be waited upon. */
		INIT_COMPLETION(device->hwaccess_gate);
	}

	return status;
}

int kgsl_g12_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;
	KGSL_PRE_HWACCESS();
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_AXI_HALT_CONTROL) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		kgsl_g12_regwrite(device, (ADDR_VGC_MH_READ_ADDR >> 2),
				  offsetwords);
		reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ ADDR_VGC_MH_DATA_ADDR);
	} else {
		if (offsetwords * sizeof(uint32_t) >=
				device->regspace.sizebytes) {
			KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
			return -ERANGE;
		}

		reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
	}

	*value = readl(reg);

	return 0;
}

int kgsl_g12_regwrite(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int value)
{
	unsigned int *reg;

	KGSL_PRE_HWACCESS();
	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_CLNT_INTF_CTRL_CONFIG2) ||
	    (offsetwords >= ADDR_MH_MMU_CONFIG &&
	     offsetwords <= ADDR_MH_MMU_MPU_END)) {
		kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_MMU,
					 offsetwords, value);
	} else {
		if (offsetwords*sizeof(uint32_t) >=
				device->regspace.sizebytes) {
			KGSL_DRV_ERR("invalid offset %d\n", offsetwords);
			return -ERANGE;
		}

		reg = (unsigned int *)(device->regspace.mmio_virt_base
				+ (offsetwords << 2));
		writel(value, reg);
		/* Drain write buffer */
		dsb();

		/* Memory fence to ensure all data has posted.  On some systems,
		 * like 7x27, the register block is not allocated as strongly
		 * ordered memory.  Adding a memory fence ensures ordering
		 * during ringbuffer submits.*/
		mb();
	}

	return 0;
}

static int kgsl_g12_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;
	long timeout = 0;

	KGSL_DRV_INFO("enter (device=%p,timestamp=%d,timeout=0x%08x)\n",
			device, timestamp, msecs);

	KGSL_DRV_INFO("current (device=%p,timestamp=%d)\n",
			device, g12_device->timestamp);

	timeout = wait_event_interruptible_timeout(
			g12_device->wait_timestamp_wq,
			kgsl_check_timestamp((struct kgsl_device *) g12_device,
					     timestamp),
			msecs_to_jiffies(msecs));

	if (timeout > 0)
		status = 0;
	else if (timeout == 0)
		status = -ETIMEDOUT;
	else
		status = timeout;

	KGSL_DRV_INFO("return %d\n", status);
	return status;
}

int __init kgsl_g12_config(struct kgsl_devconfig *devconfig,
				struct platform_device *pdev)
{
	int result = 0;
	struct resource *res = NULL;

	memset(devconfig, 0, sizeof(*devconfig));

	/*find memory regions */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"kgsl_2d0_reg_memory");
	if (res == NULL) {
		KGSL_DRV_ERR("platform_get_resource_byname failed\n");
		result = -EINVAL;
		goto done;
	}
	KGSL_DRV_DBG("registers at %08x to %08x\n", res->start, res->end);
	devconfig->regspace.mmio_phys_base = res->start;
	devconfig->regspace.sizebytes = resource_size(res);

	/*note: for all of these behavior masks:
	 *	0 = do not translate
	 *	1 = translate within va_range, otherwise use phyisical
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

	devconfig->va_base = 0x66000000;
	devconfig->va_range = SZ_32M;

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

static long kgsl_g12_ioctl_cmdwindow_write(struct kgsl_device_private *dev_priv,
				     void __user *arg)
{
	int result = 0;
	struct kgsl_cmdwindow_write param;

	if (copy_from_user(&param, arg, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}

	result = kgsl_g12_cmdwindow_write(dev_priv->device,
					     param.target,
					     param.addr,
					     param.data);

	if (result != 0)
		goto done;

	if (copy_to_user(arg, &param, sizeof(param))) {
		result = -EFAULT;
		goto done;
	}
done:
	return result;
}

static long kgsl_g12_ioctl(struct kgsl_device_private *dev_priv,
			unsigned int cmd,
			unsigned long arg)
{
	int result = 0;

	switch (cmd) {
	case IOCTL_KGSL_CMDWINDOW_WRITE:
		result = kgsl_g12_ioctl_cmdwindow_write(dev_priv,
							(void __user *)arg);
		break;
	default:
		KGSL_DRV_ERR("invalid ioctl code %08x\n", cmd);
		result = -EINVAL;
		break;
	}
	return result;

}

int kgsl_g12_getfunctable(struct kgsl_functable *ftbl)
{

	if (ftbl == NULL)
		return KGSL_FAILURE;
	ftbl->device_regread = kgsl_g12_regread;
	ftbl->device_regwrite = kgsl_g12_regwrite;
	ftbl->device_setstate = kgsl_g12_setstate;
	ftbl->device_idle = kgsl_g12_idle;
	ftbl->device_sleep = kgsl_g12_sleep;
	ftbl->device_suspend = kgsl_g12_suspend;
	ftbl->device_wake = kgsl_g12_wake;
	ftbl->device_start = kgsl_g12_start;
	ftbl->device_stop = kgsl_g12_stop;
	ftbl->device_getproperty = kgsl_g12_getproperty;
	ftbl->device_waittimestamp = kgsl_g12_waittimestamp;
	ftbl->device_cmdstream_readtimestamp = kgsl_g12_cmdstream_readtimestamp;
	ftbl->device_issueibcmds = kgsl_g12_cmdstream_issueibcmds;
	ftbl->device_drawctxt_create = kgsl_g12_drawctxt_create;
	ftbl->device_drawctxt_destroy = kgsl_g12_drawctxt_destroy;
	ftbl->device_ioctl = kgsl_g12_ioctl;
	ftbl->device_setup_pt = kgsl_g12_setup_pt;
	ftbl->device_cleanup_pt = kgsl_g12_cleanup_pt;

	return KGSL_SUCCESS;
}
