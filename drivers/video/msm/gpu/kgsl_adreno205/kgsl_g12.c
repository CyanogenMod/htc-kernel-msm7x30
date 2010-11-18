/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/workqueue.h>

#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_g12_cmdstream.h"
#include "kgsl_cmdstream.h"
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

#define FIRST_TIMEOUT (HZ / 2)
#define INTERVAL_TIMEOUT (HZ / 10)

#define KGSL_G12_TIMESTAMP_EPSILON 20000
#define KGSL_G12_IDLE_COUNT_MAX 1000000

static struct timer_list idle_timer;
static struct work_struct idle_check;

static int kgsl_g12_getfunctable(struct kgsl_functable *ftbl);

void kgsl_g12_timer(unsigned long data)
{
	/* Have work run in a non-interrupt context. */
	schedule_work(&idle_check);
}

int kgsl_g12_last_release_locked(void)
{
	KGSL_DRV_INFO("kgsl_g12_last_release_locked()\n");

	if (kgsl_driver.g12_device.flags & KGSL_FLAGS_STARTED) {
		kgsl_g12_stop(&kgsl_driver.g12_device);
		kgsl_pwrctrl(KGSL_PWRFLAGS_G12_IRQ_OFF);
		kgsl_g12_close(&kgsl_driver.g12_device);
		kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_OFF);
		kgsl_driver.g12_device.hwaccess_blocked = KGSL_FALSE;
	}

	return KGSL_SUCCESS;
}

int kgsl_g12_first_open_locked(void)
{
	int result = KGSL_SUCCESS;

	KGSL_DRV_INFO("kgsl_g12_first_open()\n");

	if (kgsl_driver.g12_device.hwaccess_blocked == KGSL_FALSE) {
		kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_ON);

		result = kgsl_g12_init(&kgsl_driver.g12_device,
			&kgsl_driver.g12_config);
		if (result != 0)
			goto done;

		result = kgsl_g12_start(&kgsl_driver.g12_device, 0);
		if (result != 0)
			goto done;

		kgsl_pwrctrl(KGSL_PWRFLAGS_G12_IRQ_ON);
	}
 done:
	return result;

}

void kgsl_g12_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = &kgsl_driver.g12_device;

	KGSL_DRV_DBG("kgsl_g12_idle_check\n");
	mutex_lock(&kgsl_driver.mutex);
	if (device->flags & KGSL_FLAGS_STARTED) {
		if (kgsl_g12_sleep(device, false) == KGSL_FAILURE)
			mod_timer(&idle_timer, jiffies + INTERVAL_TIMEOUT);
	}
	mutex_unlock(&kgsl_driver.mutex);
}

irqreturn_t kgsl_g12_isr(int irq, void *data)
{
	irqreturn_t result = IRQ_NONE;

	struct kgsl_device *device = &kgsl_driver.g12_device;
	unsigned int status;
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
			device->timestamp += count;

			wake_up_interruptible(&(device->wait_timestamp_wq));
		}
	}

	mod_timer(&idle_timer, jiffies + INTERVAL_TIMEOUT);

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

int
kgsl_g12_init(struct kgsl_device *device,
		struct kgsl_devconfig *config)
{
	int status = -EINVAL;
	struct kgsl_memregion *regspace = &device->regspace;
	unsigned int memflags = KGSL_MEMFLAGS_ALIGNPAGE |
				KGSL_MEMFLAGS_CONPHYS;

	KGSL_DRV_VDBG("enter (device=%p, config=%p)\n", device, config);

	if (device->flags & KGSL_FLAGS_INITIALIZED) {
		KGSL_DRV_VDBG("return %d\n", 0);
		return 0;
	}

	device->flags |= KGSL_FLAGS_INITIALIZED;

	/* initilization of timestamp wait */
	init_waitqueue_head(&(device->wait_timestamp_wq));

	if (regspace->mmio_virt_base == NULL) {
		memcpy(regspace, &config->regspace, sizeof(device->regspace));
		if (regspace->mmio_phys_base == 0 || regspace->sizebytes == 0) {
			KGSL_DRV_ERR("dev %d invalid regspace\n", device->id);
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
			release_mem_region(regspace->mmio_phys_base,
					   regspace->sizebytes);
			memset(regspace, 0, sizeof(regspace));
			status = -ENODEV;
			goto error;
		}
	}

	KGSL_DRV_INFO("dev %d regs phys 0x%08x size 0x%08x virt %p\n",
			device->id, regspace->mmio_phys_base,
			regspace->sizebytes, regspace->mmio_virt_base);


	device->id = KGSL_DEVICE_G12;
	init_completion(&device->hwaccess_gate);
	kgsl_g12_getfunctable(&device->ftbl);

	printk(KERN_INFO "kgsl mmu config 0x%x\n", config->mmu_config);
	if (config->mmu_config) {
		device->mmu.config    = config->mmu_config;
		device->mmu.mpu_base  = config->mpu_base;
		device->mmu.mpu_range = config->mpu_range;
		device->mmu.va_base   = config->va_base;
		device->mmu.va_range  = config->va_range;
	}

	/* Set up MH arbiter.  MH offsets are considered to be dword
	 * based, therefore no down shift. */
	kgsl_g12_regwrite(device, ADDR_MH_ARBITER_CONFIG,
			  KGSL_G12_CFG_G12_MHARB);

	kgsl_g12_regwrite(device, (ADDR_VGC_IRQENABLE >> 2), 0x3);

	status = kgsl_mmu_init(device);

	if (status != 0) {
		status = -ENODEV;
		goto error;
	}

	status = kgsl_sharedmem_alloc(memflags, sizeof(device->memstore),
					&device->memstore);

	if (status != 0)  {
		status = -ENODEV;
		goto error_close_mmu;
	}

	kgsl_sharedmem_set(&device->memstore, 0, 0, device->memstore.size);

	return 0;

error_close_mmu:
	kgsl_mmu_close(device);
error:
	return status;
}

int kgsl_g12_close(struct kgsl_device *device)
{
	kgsl_g12_cmdwindow_close(device);

	if (device->memstore.hostptr)
		kgsl_sharedmem_free(&device->memstore);

	kgsl_mmu_close(device);

	KGSL_DRV_VDBG("return %d\n", 0);
	device->flags &= ~KGSL_FLAGS_INITIALIZED;
	return 0;
}

int kgsl_g12_start(struct kgsl_device *device, uint32_t flags)
{
	int status = -EINVAL;

	KGSL_DRV_VDBG("enter (device=%p)\n", device);

	if (!(device->flags & KGSL_FLAGS_INITIALIZED)) {
		KGSL_DRV_ERR("Trying to start uninitialized device.\n");
		return -EINVAL;
	}

	if (device->flags & KGSL_FLAGS_STARTED) {
		KGSL_DRV_VDBG("already started");
		return 0;
	}

	status = kgsl_g12_cmdwindow_init(device);
	if (status != 0) {
		kgsl_g12_stop(device);
		return status;
	}

	device->flags |= KGSL_FLAGS_STARTED;
	init_timer(&idle_timer);
	idle_timer.function = kgsl_g12_timer;
	idle_timer.expires = jiffies + FIRST_TIMEOUT;
	add_timer(&idle_timer);
	INIT_WORK(&idle_check, kgsl_g12_idle_check);

	INIT_LIST_HEAD(&device->ringbuffer.memqueue);

	KGSL_DRV_VDBG("return %d\n", status);
	return status;
}

int kgsl_g12_stop(struct kgsl_device *device)
{
	del_timer(&idle_timer);
	if (device->flags & KGSL_FLAGS_STARTED) {
		kgsl_g12_idle(device, KGSL_TIMEOUT_DEFAULT);
		device->flags &= ~KGSL_FLAGS_STARTED;
	}

	return 0;
}

int kgsl_g12_getproperty(struct kgsl_device *device,
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

int kgsl_g12_idle(struct kgsl_device *device, unsigned int timeout)
{
	int status = KGSL_SUCCESS;

	KGSL_DRV_VDBG("enter (device=%p, timeout=%d)\n", device, timeout);

	if (device->flags & KGSL_FLAGS_STARTED) {
		if (device->current_timestamp > device->timestamp)
			status = kgsl_g12_waittimestamp(device,
					device->current_timestamp, timeout);
	}

	if (status)
		KGSL_DRV_ERR("Error, kgsl_g12_waittimestamp() timed out\n");

	KGSL_DRV_VDBG("return %d\n", status);

	return status;
}

static unsigned int kgsl_g12_isidle(struct kgsl_device *device)
{
	int status = 0;
	int timestamp = device->timestamp;

	if (timestamp == device->current_timestamp)
		status = KGSL_TRUE;

	return status;
}

/******************************************************************/
/* Caller must hold the driver mutex. */
int kgsl_g12_sleep(struct kgsl_device *device, const int idle)
{
	int status = KGSL_SUCCESS;

	KGSL_DRV_DBG("kgsl_g12_sleep!!!\n");

	/* Skip this request if we're already sleeping. */
	if (device->hwaccess_blocked == KGSL_FALSE) {
		/* See if the device is idle. If it is, we can shut down */
		/* the core clock until the next attempt to access the HW. */
		if (idle || kgsl_g12_isidle(device)) {
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
int kgsl_g12_wake(struct kgsl_device *device)
{
	int status = KGSL_SUCCESS;

	/* Turn on the core clocks */
	status = kgsl_pwrctrl(KGSL_PWRFLAGS_G12_CLK_ON);
	kgsl_pwrctrl(KGSL_PWRFLAGS_G12_IRQ_ON);

	/* Re-enable HW access */
	device->hwaccess_blocked = KGSL_FALSE;
	complete_all(&device->hwaccess_gate);
	mod_timer(&idle_timer, jiffies + FIRST_TIMEOUT);

	KGSL_DRV_VDBG("<-- kgsl_g12_wake(). Return value %d\n", status);

	return status;
}

/******************************************************************/
/* Caller must hold the driver mutex. */
int kgsl_g12_suspend(struct kgsl_device *device)
{
	int status;

	/* Wait for the device to become idle */
	status = kgsl_g12_idle(device, KGSL_G12_IDLE_COUNT_MAX);

	if (status == KGSL_SUCCESS) {
		/* Put the device to sleep. */
		status = kgsl_g12_sleep(device, true);
		/* Don't let the timer wake us during suspended sleep. */
		del_timer(&idle_timer);
		/* Get the completion ready to be waited upon. */
		INIT_COMPLETION(device->hwaccess_gate);
	}

	return status;
}

int kgsl_g12_regread(struct kgsl_device *device, unsigned int offsetwords,
				unsigned int *value)
{
	unsigned int *reg;

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

	if ((offsetwords >= ADDR_MH_ARBITER_CONFIG &&
	     offsetwords <= ADDR_MH_AXI_HALT_CONTROL) ||
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

int kgsl_g12_waittimestamp(struct kgsl_device *device,
				unsigned int timestamp,
				unsigned int msecs)
{
	int status = -EINVAL;
	long timeout = 0;

	KGSL_DRV_INFO("enter (device=%p,timestamp=%d,timeout=0x%08x)\n",
			device, timestamp, msecs);

	KGSL_DRV_INFO("current (device=%p,timestamp=%d)\n",
			device, device->timestamp);

	timeout = wait_event_interruptible_timeout(device->wait_timestamp_wq,
			kgsl_g12_cmdstream_check_timestamp(device, timestamp),
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
			"kgsl_g12_reg_memory");
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

static int kgsl_g12_getfunctable(struct kgsl_functable *ftbl)
{
	if (ftbl == NULL)
		return KGSL_FAILURE;
	ftbl->device_regread = kgsl_g12_regread;
	ftbl->device_regwrite = kgsl_g12_regwrite;
	ftbl->device_setstate = kgsl_g12_setstate;
	ftbl->device_idle = kgsl_g12_idle;

	return KGSL_SUCCESS;
}
