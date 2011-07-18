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
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_yamato.h"
#include "kgsl_log.h"
#include "kgsl_pm4types.h"
#include "kgsl_ringbuffer.h"
#include "kgsl_postmortem.h"
#include "kgsl_cmdstream.h"

#include "yamato_reg.h"

#define VALID_STATUS_COUNT_MAX	10
#define GSL_RB_NOP_SIZEDWORDS				2
/* protected mode error checking below register address 0x800
*  note: if CP_INTERRUPT packet is used then checking needs
*  to change to below register address 0x7C8
*/
#define GSL_RB_PROTECTED_MODE_CONTROL		0x200001F2

#define GSL_CP_INT_MASK \
	(CP_INT_CNTL__SW_INT_MASK | \
	CP_INT_CNTL__T0_PACKET_IN_IB_MASK | \
	CP_INT_CNTL__OPCODE_ERROR_MASK | \
	CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK | \
	CP_INT_CNTL__RESERVED_BIT_ERROR_MASK | \
	CP_INT_CNTL__IB_ERROR_MASK | \
	CP_INT_CNTL__IB2_INT_MASK | \
	CP_INT_CNTL__IB1_INT_MASK | \
	CP_INT_CNTL__RB_INT_MASK)

#define YAMATO_PFP_FW "yamato_pfp.fw"
#define YAMATO_PM4_FW "yamato_pm4.fw"
#define LEIA_PFP_470_FW "leia_pfp_470.fw"
#define LEIA_PM4_470_FW "leia_pm4_470.fw"

/*  ringbuffer size log2 quadwords equivalent */
inline unsigned int kgsl_ringbuffer_sizelog2quadwords(unsigned int sizedwords)
{
	unsigned int sizelog2quadwords = 0;
	int i = sizedwords >> 1;

	while (i >>= 1)
		sizelog2quadwords++;

	return sizelog2quadwords;
}


/* functions */
void kgsl_cp_intrcallback(struct kgsl_device *device)
{
	unsigned int status = 0, num_reads = 0, master_status = 0;
	struct kgsl_yamato_device *yamato_device = (struct kgsl_yamato_device *)
								device;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;

	KGSL_CMD_VDBG("enter (device=%p)\n", device);

	kgsl_yamato_regread(device, REG_MASTER_INT_SIGNAL, &master_status);
	while (!status && (num_reads < VALID_STATUS_COUNT_MAX) &&
		(master_status & MASTER_INT_SIGNAL__CP_INT_STAT)) {
		kgsl_yamato_regread(device, REG_CP_INT_STATUS, &status);
		kgsl_yamato_regread(device, REG_MASTER_INT_SIGNAL,
					&master_status);
		num_reads++;
	}
	if (num_reads > 1)
		KGSL_DRV_WARN("Looped %d times to read REG_CP_INT_STATUS\n",
				num_reads);
	if (!status) {
		if (master_status & MASTER_INT_SIGNAL__CP_INT_STAT) {
			/* This indicates that we could not read CP_INT_STAT.
			 * As a precaution just wake up processes so
			 * they can check their timestamps. Since, we
			 * did not ack any interrupts this interrupt will
			 * be generated again */
			KGSL_DRV_WARN("Unable to read CP_INT_STATUS\n");
			wake_up_interruptible_all(&yamato_device->ib1_wq);
		} else
			KGSL_DRV_WARN("Spurious interrput detected\n");
		return;
	}

	if (status & CP_INT_CNTL__RB_INT_MASK) {
		/* signal intr completion event */
		unsigned int enableflag = 0;
		kgsl_sharedmem_writel(&rb->device->memstore,
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable),
			enableflag);
		wmb();
		KGSL_CMD_WARN("ringbuffer rb interrupt\n");
	}

	if (status & CP_INT_CNTL__T0_PACKET_IN_IB_MASK) {
		KGSL_CMD_FATAL("ringbuffer TO packet in IB interrupt\n");
		kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, 0);
		kgsl_postmortem_dump(device);
	}
	if (status & CP_INT_CNTL__OPCODE_ERROR_MASK) {
		KGSL_CMD_FATAL("ringbuffer opcode error interrupt\n");
		kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, 0);
		kgsl_postmortem_dump(device);
	}
	if (status & CP_INT_CNTL__PROTECTED_MODE_ERROR_MASK) {
		KGSL_CMD_FATAL("ringbuffer protected mode error interrupt\n");
		kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, 0);
		kgsl_postmortem_dump(device);
	}
	if (status & CP_INT_CNTL__RESERVED_BIT_ERROR_MASK) {
		KGSL_CMD_FATAL("ringbuffer reserved bit error interrupt\n");
		kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, 0);
		kgsl_postmortem_dump(device);
	}
	if (status & CP_INT_CNTL__IB_ERROR_MASK) {
		KGSL_CMD_FATAL("ringbuffer IB error interrupt\n");
		kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, 0);
		kgsl_postmortem_dump(device);
	}
	if (status & CP_INT_CNTL__SW_INT_MASK)
		KGSL_CMD_DBG("ringbuffer software interrupt\n");

	if (status & CP_INT_CNTL__IB2_INT_MASK)
		KGSL_CMD_DBG("ringbuffer ib2 interrupt\n");

	if (status & (~GSL_CP_INT_MASK))
		KGSL_CMD_DBG("bad bits in REG_CP_INT_STATUS %08x\n", status);

	/* only ack bits we understand */
	status &= GSL_CP_INT_MASK;
	kgsl_yamato_regwrite(device, REG_CP_INT_ACK, status);

	if (status & (CP_INT_CNTL__IB1_INT_MASK | CP_INT_CNTL__RB_INT_MASK)) {
		KGSL_CMD_WARN("ringbuffer ib1/rb interrupt\n");
		wake_up_interruptible_all(&yamato_device->ib1_wq);
		atomic_notifier_call_chain(&(device->ts_notifier_list),
					   KGSL_DEVICE_YAMATO,
					   NULL);
	}

	KGSL_CMD_VDBG("return\n");
}


void kgsl_ringbuffer_watchdog(struct kgsl_device *device)
{
	struct kgsl_ringbuffer *rb = NULL;

	BUG_ON(device == NULL);

	rb = &device->ringbuffer;

	KGSL_CMD_VDBG("enter\n");

	if ((rb->flags & KGSL_FLAGS_STARTED) == 0) {
		KGSL_CMD_VDBG("not started\n");
		return;
	}

	GSL_RB_GET_READPTR(rb, &rb->rptr);

	if (rb->rptr == rb->wptr) {
		/* clear rptr sample for interval n */
		rb->watchdog.flags &= ~KGSL_FLAGS_ACTIVE;
		goto done;
	}
	/* ringbuffer is currently not empty */
	/* and a rptr sample was taken during interval n-1 */
	if (rb->watchdog.flags & KGSL_FLAGS_ACTIVE) {
		/* and the rptr did not advance between
		* interval n-1 and n */
		if (rb->rptr == rb->watchdog.rptr_sample) {
			/* then the core has hung */
			KGSL_CMD_FATAL("Watchdog detected core hung.\n");
			goto done;
		}
		/* save rptr sample for interval n */
		rb->watchdog.flags |= KGSL_FLAGS_ACTIVE;
		rb->watchdog.rptr_sample = rb->rptr;
	}
done:
	KGSL_CMD_VDBG("return\n");
}

static void kgsl_ringbuffer_submit(struct kgsl_ringbuffer *rb)
{
	BUG_ON(rb->wptr == 0);

	GSL_RB_UPDATE_WPTR_POLLING(rb);
	/* Drain write buffer and data memory barrier */
	dsb();
	wmb();

	/* Memory fence to ensure all data has posted.  On some systems,
	* like 7x27, the register block is not allocated as strongly ordered
	* memory.  Adding a memory fence ensures ordering during ringbuffer
	* submits.*/
	mb();

	kgsl_yamato_regwrite(rb->device, REG_CP_RB_WPTR, rb->wptr);

	rb->flags |= KGSL_FLAGS_ACTIVE;
}

static int
kgsl_ringbuffer_waitspace(struct kgsl_ringbuffer *rb, unsigned int numcmds,
			  int wptr_ahead)
{
	int nopcount;
	unsigned int freecmds;
	unsigned int *cmds;

	KGSL_CMD_VDBG("enter (rb=%p, numcmds=%d, wptr_ahead=%d)\n",
		      rb, numcmds, wptr_ahead);

	/* if wptr ahead, fill the remaining with NOPs */
	if (wptr_ahead) {
		/* -1 for header */
		nopcount = rb->sizedwords - rb->wptr - 1;

		cmds = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		GSL_RB_WRITE(cmds, pm4_nop_packet(nopcount));

		/* Make sure that rptr is not 0 before submitting
		 * commands at the end of ringbuffer. We do not
		 * want the rptr and wptr to become equal when
		 * the ringbuffer is not empty */
		do {
			GSL_RB_GET_READPTR(rb, &rb->rptr);
		} while (!rb->rptr);

		rb->wptr++;

		kgsl_ringbuffer_submit(rb);

		rb->wptr = 0;
	}

	/* wait for space in ringbuffer */
	do {
		GSL_RB_GET_READPTR(rb, &rb->rptr);

		freecmds = rb->rptr - rb->wptr;

	} while ((freecmds != 0) && (freecmds <= numcmds));

	KGSL_CMD_VDBG("return %d\n", 0);

	return 0;
}


static unsigned int *kgsl_ringbuffer_allocspace(struct kgsl_ringbuffer *rb,
					     unsigned int numcmds)
{
	unsigned int	*ptr = NULL;
	int				status = 0;

	BUG_ON(numcmds >= rb->sizedwords);

	/* check for available space */
	if (rb->wptr >= rb->rptr) {
		/* wptr ahead or equal to rptr */
		/* reserve dwords for nop packet */
		if ((rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			status = kgsl_ringbuffer_waitspace(rb, numcmds, 1);
	} else {
		/* wptr behind rptr */
		if ((rb->wptr + numcmds) >= rb->rptr)
			status  = kgsl_ringbuffer_waitspace(rb, numcmds, 0);
		/* check for remaining space */
		/* reserve dwords for nop packet */
		if ((rb->wptr + numcmds) > (rb->sizedwords -
				GSL_RB_NOP_SIZEDWORDS))
			status = kgsl_ringbuffer_waitspace(rb, numcmds, 1);
	}

	if (status == 0) {
		ptr = (unsigned int *)rb->buffer_desc.hostptr + rb->wptr;
		rb->wptr += numcmds;
	}

	return ptr;
}

static int kgsl_ringbuffer_load_pm4_ucode(struct kgsl_device *device)
{
	int status = 0;
	int i;
	const struct firmware *fw = NULL;
	unsigned int *fw_ptr = NULL;
	size_t fw_word_size = 0;

	if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
		status = request_firmware(&fw, LEIA_PM4_470_FW,
			kgsl_driver.base_dev[KGSL_DEVICE_YAMATO]);
		if (status != 0) {
			KGSL_DRV_ERR(
				"request_firmware failed for %s  \
				 with error %d\n",
				LEIA_PM4_470_FW, status);
			goto error;
		}
	} else {
		status = request_firmware(&fw, YAMATO_PM4_FW,
			kgsl_driver.base_dev[KGSL_DEVICE_YAMATO]);
		if (status != 0) {
			KGSL_DRV_ERR(
				"request_firmware failed for %s  \
				 with error %d\n",
				YAMATO_PM4_FW, status);
			goto error;
		}
	}
	/*this firmware must come in 3 word chunks. plus 1 word of version*/
	if ((fw->size % (sizeof(uint32_t)*3)) != 4) {
		KGSL_DRV_ERR("bad firmware size %d.\n", fw->size);
		status = -EINVAL;
		goto error_release_fw;
	}
	fw_ptr = (unsigned int *)fw->data;
	fw_word_size = fw->size/sizeof(uint32_t);
	KGSL_DRV_INFO("loading pm4 ucode version: %d\n", fw_ptr[0]);

	kgsl_yamato_regwrite(device, REG_CP_DEBUG, 0x02000000);
	kgsl_yamato_regwrite(device, REG_CP_ME_RAM_WADDR, 0);
	for (i = 1; i < fw_word_size; i++)
		kgsl_yamato_regwrite(device, REG_CP_ME_RAM_DATA, fw_ptr[i]);

error_release_fw:
	release_firmware(fw);
error:
	return status;
}

static int kgsl_ringbuffer_load_pfp_ucode(struct kgsl_device *device)
{
	int status = 0;
	int i;
	const struct firmware *fw = NULL;
	unsigned int *fw_ptr = NULL;
	size_t fw_word_size = 0;

	if (device->chip_id == KGSL_CHIPID_LEIA_REV470) {
		status = request_firmware(&fw, LEIA_PFP_470_FW,
				kgsl_driver.base_dev[KGSL_DEVICE_YAMATO]);
		if (status != 0) {
			KGSL_DRV_ERR(
				"request_firmware for %s \
				 failed with error %d\n",
				LEIA_PFP_470_FW, status);
			return status;
		}
	} else {
		status = request_firmware(&fw, YAMATO_PFP_FW,
				kgsl_driver.base_dev[KGSL_DEVICE_YAMATO]);
		if (status != 0) {
			KGSL_DRV_ERR(
				"request_firmware for %s \
				 failed with error %d\n",
				YAMATO_PFP_FW, status);
			return status;
		}
	}
	/*this firmware must come in 1 word chunks. */
	if ((fw->size % sizeof(uint32_t)) != 0) {
		KGSL_DRV_ERR("bad firmware size %d.\n", fw->size);
		release_firmware(fw);
		return -EINVAL;
	}
	fw_ptr = (unsigned int *)fw->data;
	fw_word_size = fw->size/sizeof(uint32_t);

	KGSL_DRV_INFO("loading pfp ucode version: %d\n", fw_ptr[0]);

	kgsl_yamato_regwrite(device, REG_CP_PFP_UCODE_ADDR, 0);
	for (i = 1; i < fw_word_size; i++)
		kgsl_yamato_regwrite(device, REG_CP_PFP_UCODE_DATA, fw_ptr[i]);

	release_firmware(fw);
	return status;
}

int kgsl_ringbuffer_start(struct kgsl_ringbuffer *rb)
{
	int status;
	/*cp_rb_cntl_u cp_rb_cntl; */
	union reg_cp_rb_cntl cp_rb_cntl;
	unsigned int *cmds, rb_cntl;
	struct kgsl_device *device = rb->device;

	KGSL_CMD_VDBG("enter (rb=%p)\n", rb);

	if (rb->flags & KGSL_FLAGS_STARTED) {
		KGSL_CMD_VDBG("return %d\n", 0);
		return 0;
	}
	kgsl_sharedmem_set(&rb->memptrs_desc, 0, 0,
				sizeof(struct kgsl_rbmemptrs));

	kgsl_sharedmem_set(&rb->buffer_desc, 0, 0xAA,
				(rb->sizedwords << 2));

	kgsl_yamato_regwrite(device, REG_CP_RB_WPTR_BASE,
			     (rb->memptrs_desc.gpuaddr
			      + GSL_RB_MEMPTRS_WPTRPOLL_OFFSET));

	/* setup WPTR delay */
	kgsl_yamato_regwrite(device, REG_CP_RB_WPTR_DELAY, 0 /*0x70000010 */);

	/*setup REG_CP_RB_CNTL */
	kgsl_yamato_regread(device, REG_CP_RB_CNTL, &rb_cntl);
	cp_rb_cntl.val = rb_cntl;
	/* size of ringbuffer */
	cp_rb_cntl.f.rb_bufsz =
		kgsl_ringbuffer_sizelog2quadwords(rb->sizedwords);
	/* quadwords to read before updating mem RPTR */
	cp_rb_cntl.f.rb_blksz = rb->blksizequadwords;
	cp_rb_cntl.f.rb_poll_en = GSL_RB_CNTL_POLL_EN; /* WPTR polling */
	/* mem RPTR writebacks */
	cp_rb_cntl.f.rb_no_update =  GSL_RB_CNTL_NO_UPDATE;

	kgsl_yamato_regwrite(device, REG_CP_RB_CNTL, cp_rb_cntl.val);

	kgsl_yamato_regwrite(device, REG_CP_RB_BASE, rb->buffer_desc.gpuaddr);

	kgsl_yamato_regwrite(device, REG_CP_RB_RPTR_ADDR,
			     rb->memptrs_desc.gpuaddr +
			     GSL_RB_MEMPTRS_RPTR_OFFSET);

	/* explicitly clear all cp interrupts */
	kgsl_yamato_regwrite(device, REG_CP_INT_ACK, 0xFFFFFFFF);

	/* setup scratch/timestamp */
	kgsl_yamato_regwrite(device, REG_SCRATCH_ADDR,
			     device->memstore.gpuaddr +
			     KGSL_DEVICE_MEMSTORE_OFFSET(soptimestamp));

	kgsl_yamato_regwrite(device, REG_SCRATCH_UMSK,
			     GSL_RB_MEMPTRS_SCRATCH_MASK);

	/* load the CP ucode */

	status = kgsl_ringbuffer_load_pm4_ucode(device);
	if (status != 0) {
		KGSL_DRV_ERR("kgsl_ringbuffer_load_pm4_ucode failed  %d\n",
				status);
		return status;
	}


	/* load the prefetch parser ucode */
	status = kgsl_ringbuffer_load_pfp_ucode(device);
	if (status != 0) {
		KGSL_DRV_ERR("kgsl_ringbuffer_load_pm4_ucode failed %d\n",
				status);
		return status;
	}

	kgsl_yamato_regwrite(device, REG_CP_QUEUE_THRESHOLDS, 0x000C0804);

	rb->rptr = 0;
	rb->wptr = 0;

	rb->timestamp = 0;
	GSL_RB_INIT_TIMESTAMP(rb);

	INIT_LIST_HEAD(&rb->memqueue);

	/* clear ME_HALT to start micro engine */
	kgsl_yamato_regwrite(device, REG_CP_ME_CNTL, 0);

	/* ME_INIT */
	cmds = kgsl_ringbuffer_allocspace(rb, 19);

	GSL_RB_WRITE(cmds, PM4_HDR_ME_INIT);
	/* All fields present (bits 9:0) */
	GSL_RB_WRITE(cmds, 0x000003ff);
	/* Disable/Enable Real-Time Stream processing (present but ignored) */
	GSL_RB_WRITE(cmds, 0x00000000);
	/* Enable (2D <-> 3D) implicit synchronization (present but ignored) */
	GSL_RB_WRITE(cmds, 0x00000000);

	GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(REG_RB_SURFACE_INFO));
	GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SC_WINDOW_OFFSET));
	GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(REG_VGT_MAX_VTX_INDX));
	GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(REG_SQ_PROGRAM_CNTL));
	GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(REG_RB_DEPTHCONTROL));
	GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SU_POINT_SIZE));
	GSL_RB_WRITE(cmds, GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SC_LINE_CNTL));
	GSL_RB_WRITE(cmds,
	     GSL_HAL_SUBBLOCK_OFFSET(REG_PA_SU_POLY_OFFSET_FRONT_SCALE));

	/* Vertex and Pixel Shader Start Addresses in instructions
	* (3 DWORDS per instruction) */
	GSL_RB_WRITE(cmds, 0x80000180);
	/* Maximum Contexts */
	GSL_RB_WRITE(cmds, 0x00000001);
	/* Write Confirm Interval and The CP will wait the
	* wait_interval * 16 clocks between polling  */
	GSL_RB_WRITE(cmds, 0x00000000);

	/* NQ and External Memory Swap */
	GSL_RB_WRITE(cmds, 0x00000000);
	/* Protected mode error checking */
	GSL_RB_WRITE(cmds, GSL_RB_PROTECTED_MODE_CONTROL);
	/* Disable header dumping and Header dump address */
	GSL_RB_WRITE(cmds, 0x00000000);
	/* Header dump size */
	GSL_RB_WRITE(cmds, 0x00000000);

	kgsl_ringbuffer_submit(rb);

	/* idle device to validate ME INIT */
	status = kgsl_yamato_idle(device, KGSL_TIMEOUT_DEFAULT);

	KGSL_CMD_DBG("enabling CP interrupts: mask %08lx\n", GSL_CP_INT_MASK);
	kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, GSL_CP_INT_MASK);
	if (status == 0)
		rb->flags |= KGSL_FLAGS_STARTED;

	KGSL_CMD_VDBG("return %d\n", status);

	return status;
}

int kgsl_ringbuffer_stop(struct kgsl_ringbuffer *rb)
{
	KGSL_CMD_VDBG("enter (rb=%p)\n", rb);

	if (rb->flags & KGSL_FLAGS_STARTED) {
		KGSL_CMD_DBG("disabling CP interrupts: mask %08x\n", 0);
		kgsl_yamato_regwrite(rb->device, REG_CP_INT_CNTL, 0);

		/* ME_HALT */
		kgsl_yamato_regwrite(rb->device, REG_CP_ME_CNTL, 0x10000000);

		kgsl_cmdstream_memqueue_drain(rb->device);

		rb->flags &= ~KGSL_FLAGS_STARTED;
		kgsl_ringbuffer_dump(rb);
	}

	KGSL_CMD_VDBG("return %d\n", 0);

	return 0;
}

int kgsl_ringbuffer_init(struct kgsl_device *device)
{
	int status;
	//uint32_t flags;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;

	KGSL_CMD_VDBG("enter (device=%p)\n", device);

	rb->device = device;
	rb->sizedwords = (2 << kgsl_cfg_rb_sizelog2quadwords);
	rb->blksizequadwords = kgsl_cfg_rb_blksizequadwords;

	/* allocate memory for ringbuffer */
	status = kgsl_sharedmem_alloc_coherent(&rb->buffer_desc,
					       (rb->sizedwords << 2));
	if (status != 0) {
		kgsl_ringbuffer_close(rb);
		KGSL_CMD_VDBG("return %d\n", status);
		return status;
	}

	/* allocate memory for polling and timestamps */
	/* This really can be at 4 byte alignment boundry but for using MMU
	 * we need to make it at page boundary */
	status = kgsl_sharedmem_alloc_coherent(&rb->memptrs_desc,
					       sizeof(struct kgsl_rbmemptrs));
	if (status != 0) {
		kgsl_ringbuffer_close(rb);
		KGSL_CMD_VDBG("return %d\n", status);
		return status;
	}

	/* overlay structure on memptrs memory */
	rb->memptrs = (struct kgsl_rbmemptrs *) rb->memptrs_desc.hostptr;

	rb->flags |= KGSL_FLAGS_INITIALIZED;

	KGSL_CMD_VDBG("return %d\n", 0);
	return 0;
}

int kgsl_ringbuffer_close(struct kgsl_ringbuffer *rb)
{
	KGSL_CMD_VDBG("enter (rb=%p)\n", rb);

	if (rb->buffer_desc.hostptr)
		kgsl_sharedmem_free(&rb->buffer_desc);

	if (rb->memptrs_desc.hostptr)
		kgsl_sharedmem_free(&rb->memptrs_desc);

	rb->flags &= ~KGSL_FLAGS_INITIALIZED;

	memset(rb, 0, sizeof(struct kgsl_ringbuffer));

	KGSL_CMD_VDBG("return %d\n", 0);
	return 0;
}

static uint32_t
kgsl_ringbuffer_addcmds(struct kgsl_ringbuffer *rb,
				unsigned int flags, unsigned int *cmds,
				int sizedwords)
{
	unsigned int *ringcmds;
	unsigned int timestamp;
	unsigned int total_sizedwords = sizedwords + 6;
	unsigned int i;

	/* reserve space to temporarily turn off protected mode
	*  error checking if needed
	*/
	total_sizedwords += flags & KGSL_CMD_FLAGS_PMODE ? 4 : 0;
	total_sizedwords += !(flags & KGSL_CMD_FLAGS_NO_TS_CMP) ? 9 : 0;

	ringcmds = kgsl_ringbuffer_allocspace(rb, total_sizedwords);

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		/* disable protected mode error checking */
		GSL_RB_WRITE(ringcmds,
			pm4_type3_packet(PM4_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(ringcmds, 0);
	}

	for (i = 0; i < sizedwords; i++) {
		GSL_RB_WRITE(ringcmds, *cmds);
		cmds++;
	}

	if (flags & KGSL_CMD_FLAGS_PMODE) {
		/* re-enable protected mode error checking */
		GSL_RB_WRITE(ringcmds,
			pm4_type3_packet(PM4_SET_PROTECTED_MODE, 1));
		GSL_RB_WRITE(ringcmds, 1);
	}

	rb->timestamp++;
	timestamp = rb->timestamp;

	/* start-of-pipeline and end-of-pipeline timestamps */
	GSL_RB_WRITE(ringcmds, pm4_type0_packet(REG_CP_TIMESTAMP, 1));
	GSL_RB_WRITE(ringcmds, rb->timestamp);
	GSL_RB_WRITE(ringcmds, pm4_type3_packet(PM4_EVENT_WRITE, 3));
	GSL_RB_WRITE(ringcmds, CACHE_FLUSH_TS);
	GSL_RB_WRITE(ringcmds,
		     (rb->device->memstore.gpuaddr +
		      KGSL_DEVICE_MEMSTORE_OFFSET(eoptimestamp)));
	GSL_RB_WRITE(ringcmds, rb->timestamp);

	if (!(flags & KGSL_CMD_FLAGS_NO_TS_CMP)) {
		/*  Add idle packet so avoid RBBM errors */
		GSL_RB_WRITE(ringcmds, pm4_type3_packet(PM4_WAIT_FOR_IDLE, 1));
		GSL_RB_WRITE(ringcmds, 0x00000000);
		/* Conditional execution based on memory values */
		GSL_RB_WRITE(ringcmds, pm4_type3_packet(PM4_COND_EXEC, 4));
		GSL_RB_WRITE(ringcmds, (rb->device->memstore.gpuaddr +
			KGSL_DEVICE_MEMSTORE_OFFSET(ts_cmp_enable)) >> 2);
		GSL_RB_WRITE(ringcmds, (rb->device->memstore.gpuaddr +
			KGSL_DEVICE_MEMSTORE_OFFSET(ref_wait_ts)) >> 2);
		GSL_RB_WRITE(ringcmds, rb->timestamp);
		/* # of conditional command DWORDs */
		GSL_RB_WRITE(ringcmds, 2);
		GSL_RB_WRITE(ringcmds, pm4_type3_packet(PM4_INTERRUPT, 1));
		GSL_RB_WRITE(ringcmds, CP_INT_CNTL__RB_INT_MASK);
	}

	kgsl_ringbuffer_submit(rb);

	GSL_RB_STATS(rb->stats.words_total += sizedwords);
	GSL_RB_STATS(rb->stats.issues++);

	KGSL_CMD_VDBG("return %d\n", timestamp);

	/* return timestamp of issued coREG_ands */
	return timestamp;
}

uint32_t
kgsl_ringbuffer_issuecmds(struct kgsl_device *device,
						unsigned int flags,
						unsigned int *cmds,
						int sizedwords)
{
	unsigned int timestamp;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;

	KGSL_CMD_VDBG("enter (device->id=%d, flags=%d, cmds=%p, "
		"sizedwords=%d)\n", device->id, flags, cmds, sizedwords);

	timestamp = kgsl_ringbuffer_addcmds(rb, flags, cmds, sizedwords);

	KGSL_CMD_VDBG("return %d\n)", timestamp);
	return timestamp;
}

int
kgsl_ringbuffer_issueibcmds(struct kgsl_device_private *dev_priv,
				int drawctxt_index,
				uint32_t ibaddr,
				int sizedwords,
				uint32_t *timestamp,
				unsigned int flags)
{
	unsigned int link[3];
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_yamato_device *yamato_device = (struct kgsl_yamato_device *)
							device;

	KGSL_CMD_VDBG("enter (device_id=%d, drawctxt_index=%d, ibaddr=0x%08x,"
			" sizedwords=%d, timestamp=%p)\n",
			device->id, drawctxt_index, ibaddr,
			sizedwords, timestamp);


	if (!(device->ringbuffer.flags & KGSL_FLAGS_STARTED) ||
				(drawctxt_index >= KGSL_CONTEXT_MAX)) {
		KGSL_CMD_VDBG("return %d\n", -EINVAL);
		return -EINVAL;
	}

	BUG_ON(ibaddr == 0);
	BUG_ON(sizedwords == 0);

	link[0] = PM4_HDR_INDIRECT_BUFFER_PFD;
	link[1] = ibaddr;
	link[2] = sizedwords;

	kgsl_setstate(device, device->mmu.tlb_flags);

	kgsl_drawctxt_switch(yamato_device,
			&yamato_device->drawctxt[drawctxt_index], flags);

	*timestamp = kgsl_ringbuffer_addcmds(&device->ringbuffer,
					0, &link[0], 3);


	KGSL_CMD_INFO("ctxt %d g %08x sd %d ts %d\n",
			drawctxt_index, ibaddr, sizedwords, *timestamp);

	KGSL_CMD_VDBG("return %d\n", 0);

	return 0;
}

#ifdef DEBUG
void kgsl_ringbuffer_debug(struct kgsl_ringbuffer *rb,
				struct kgsl_rb_debug *rb_debug)
{
	memset(rb_debug, 0, sizeof(struct kgsl_rb_debug));

	rb_debug->mem_rptr = rb->memptrs->rptr;
	rb_debug->mem_wptr_poll = rb->memptrs->wptr_poll;
	kgsl_yamato_regread(rb->device, REG_CP_RB_BASE,
			    (unsigned int *)&rb_debug->cp_rb_base);
	kgsl_yamato_regread(rb->device, REG_CP_RB_CNTL,
			    (unsigned int *)&rb_debug->cp_rb_cntl);
	kgsl_yamato_regread(rb->device, REG_CP_RB_RPTR_ADDR,
			    (unsigned int *)&rb_debug->cp_rb_rptr_addr);
	kgsl_yamato_regread(rb->device, REG_CP_RB_RPTR,
			    (unsigned int *)&rb_debug->cp_rb_rptr);
	kgsl_yamato_regread(rb->device, REG_CP_RB_RPTR_WR,
			    (unsigned int *)&rb_debug->cp_rb_rptr_wr);
	kgsl_yamato_regread(rb->device, REG_CP_RB_WPTR,
			    (unsigned int *)&rb_debug->cp_rb_wptr);
	kgsl_yamato_regread(rb->device, REG_CP_RB_WPTR_DELAY,
			    (unsigned int *)&rb_debug->cp_rb_wptr_delay);
	kgsl_yamato_regread(rb->device, REG_CP_RB_WPTR_BASE,
			    (unsigned int *)&rb_debug->cp_rb_wptr_base);
	kgsl_yamato_regread(rb->device, REG_CP_IB1_BASE,
			    (unsigned int *)&rb_debug->cp_ib1_base);
	kgsl_yamato_regread(rb->device, REG_CP_IB1_BUFSZ,
			    (unsigned int *)&rb_debug->cp_ib1_bufsz);
	kgsl_yamato_regread(rb->device, REG_CP_IB2_BASE,
			    (unsigned int *)&rb_debug->cp_ib2_base);
	kgsl_yamato_regread(rb->device, REG_CP_IB2_BUFSZ,
			    (unsigned int *)&rb_debug->cp_ib2_bufsz);
	kgsl_yamato_regread(rb->device, REG_CP_ST_BASE,
			    (unsigned int *)&rb_debug->cp_st_base);
	kgsl_yamato_regread(rb->device, REG_CP_ST_BUFSZ,
			    (unsigned int *)&rb_debug->cp_st_bufsz);
	kgsl_yamato_regread(rb->device, REG_CP_CSQ_RB_STAT,
			    (unsigned int *)&rb_debug->cp_csq_rb_stat);
	kgsl_yamato_regread(rb->device, REG_CP_CSQ_IB1_STAT,
			    (unsigned int *)&rb_debug->cp_csq_ib1_stat);
	kgsl_yamato_regread(rb->device, REG_CP_CSQ_IB2_STAT,
			    (unsigned int *)&rb_debug->cp_csq_ib2_stat);
	kgsl_yamato_regread(rb->device, REG_SCRATCH_UMSK,
			    (unsigned int *)&rb_debug->scratch_umsk);
	kgsl_yamato_regread(rb->device, REG_SCRATCH_ADDR,
			    (unsigned int *)&rb_debug->scratch_addr);
	kgsl_yamato_regread(rb->device, REG_CP_ME_CNTL,
			    (unsigned int *)&rb_debug->cp_me_cntl);
	kgsl_yamato_regread(rb->device, REG_CP_ME_STATUS,
			    (unsigned int *)&rb_debug->cp_me_status);
	kgsl_yamato_regread(rb->device, REG_CP_DEBUG,
			    (unsigned int *)&rb_debug->cp_debug);
	kgsl_yamato_regread(rb->device, REG_CP_STAT,
			    (unsigned int *)&rb_debug->cp_stat);
	kgsl_yamato_regread(rb->device, REG_CP_INT_STATUS,
			    (unsigned int *)&rb_debug->cp_int_status);
	kgsl_yamato_regread(rb->device, REG_CP_INT_CNTL,
			    (unsigned int *)&rb_debug->cp_int_cntl);
	kgsl_yamato_regread(rb->device, REG_RBBM_STATUS,
			    (unsigned int *)&rb_debug->rbbm_status);
	kgsl_yamato_regread(rb->device, REG_RBBM_INT_STATUS,
			    (unsigned int *)&rb_debug->rbbm_int_status);
	GSL_RB_GET_SOP_TIMESTAMP(rb, (unsigned int *)&rb_debug->sop_timestamp);
	GSL_RB_GET_EOP_TIMESTAMP(rb, (unsigned int *)&rb_debug->eop_timestamp);

}
#endif /*DEBUG*/

#ifdef DEBUG
void kgsl_ringbuffer_dump(struct kgsl_ringbuffer *rb)
{
	struct kgsl_rb_debug rb_debug;
	kgsl_ringbuffer_debug(rb, &rb_debug);

	KGSL_CMD_DBG("rbbm_status %08x rbbm_int_status %08x"
			" mem_rptr %08x mem_wptr_poll %08x\n",
			rb_debug.rbbm_status,
			rb_debug.rbbm_int_status,
			rb_debug.mem_rptr, rb_debug.mem_wptr_poll);

	KGSL_CMD_DBG("rb_base %08x rb_cntl %08x rb_rptr_addr %08x rb_rptr %08x"
			" rb_rptr_wr %08x\n",
			rb_debug.cp_rb_base, rb_debug.cp_rb_cntl,
			rb_debug.cp_rb_rptr_addr, rb_debug.cp_rb_rptr,
			rb_debug.cp_rb_rptr_wr);

	KGSL_CMD_DBG("rb_wptr %08x rb_wptr_delay %08x rb_wptr_base %08x"
			" ib1_base %08x ib1_bufsz %08x\n",
			rb_debug.cp_rb_wptr, rb_debug.cp_rb_wptr_delay,
			rb_debug.cp_rb_wptr_base, rb_debug.cp_ib1_base,
			rb_debug.cp_ib1_bufsz);

	KGSL_CMD_DBG("ib2_base  %08x ib2_bufsz %08x st_base %08x st_bufsz %08x"
			" cp_me_cntl %08x cp_me_status %08x\n",
			rb_debug.cp_ib2_base, rb_debug.cp_ib2_bufsz,
			rb_debug.cp_st_base, rb_debug.cp_st_bufsz,
			rb_debug.cp_me_cntl, rb_debug.cp_me_status);

	KGSL_CMD_DBG("cp_debug %08x cp_stat %08x cp_int_status %08x"
			" cp_int_cntl %08x\n",
			rb_debug.cp_debug, rb_debug.cp_stat,
			rb_debug.cp_int_status, rb_debug.cp_int_cntl);

	KGSL_CMD_DBG("sop_timestamp: %d eop_timestamp: %d\n",
			rb_debug.sop_timestamp, rb_debug.eop_timestamp);

}
#endif /* DEBUG */
