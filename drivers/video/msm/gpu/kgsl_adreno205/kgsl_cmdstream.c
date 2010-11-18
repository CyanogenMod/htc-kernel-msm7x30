/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_cmdstream.h"
#include "kgsl_sharedmem.h"

int kgsl_cmdstream_init(struct kgsl_device *device)
{
	return 0;
}

int kgsl_cmdstream_close(struct kgsl_device *device)
{
	return 0;
}

uint32_t
kgsl_cmdstream_readtimestamp(struct kgsl_device *device,
			     enum kgsl_timestamp_type type)
{
	uint32_t timestamp = 0;

	KGSL_CMD_VDBG("enter (device_id=%d, type=%d)\n", device->id, type);

	if (type == KGSL_TIMESTAMP_CONSUMED)
		KGSL_CMDSTREAM_GET_SOP_TIMESTAMP(device,
						 (unsigned int *)&timestamp);
	else if (type == KGSL_TIMESTAMP_RETIRED)
		KGSL_CMDSTREAM_GET_EOP_TIMESTAMP(device,
						 (unsigned int *)&timestamp);
	rmb();

	KGSL_CMD_VDBG("return %d\n", timestamp);

	return timestamp;
}

int kgsl_cmdstream_check_timestamp(struct kgsl_device *device,
				   unsigned int timestamp)
{
	unsigned int ts_processed;

	ts_processed = kgsl_cmdstream_readtimestamp(device,
						    KGSL_TIMESTAMP_RETIRED);
	return timestamp_cmp(ts_processed, timestamp);
}

void kgsl_cmdstream_memqueue_drain(struct kgsl_device *device)
{
	struct kgsl_mem_entry *entry, *entry_tmp;
	uint32_t ts_processed;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;

	/* get current EOP timestamp */
	if (device == &kgsl_driver.yamato_device)
		ts_processed =
		   kgsl_cmdstream_readtimestamp(device, KGSL_TIMESTAMP_RETIRED);
	else
		ts_processed = device->timestamp;

	list_for_each_entry_safe(entry, entry_tmp, &rb->memqueue, free_list) {
		/*NOTE: this assumes that the free list is sorted by
		 * timestamp, but I'm not yet sure that it is a valid
		 * assumption
		 */
		if (!timestamp_cmp(ts_processed, entry->free_timestamp))
			break;
		KGSL_MEM_DBG("ts_processed %d ts_free %d gpuaddr %x)\n",
			     ts_processed, entry->free_timestamp,
			     entry->memdesc.gpuaddr);
		kgsl_remove_mem_entry(entry, true);
	}
}

int
kgsl_cmdstream_freememontimestamp(struct kgsl_device *device,
				  struct kgsl_mem_entry *entry,
				  uint32_t timestamp,
				  enum kgsl_timestamp_type type)
{
	struct kgsl_ringbuffer *rb = &device->ringbuffer;
	KGSL_MEM_DBG("enter (dev %p gpuaddr %x ts %d)\n",
		     device, entry->memdesc.gpuaddr, timestamp);

	list_add_tail(&entry->free_list, &rb->memqueue);
	entry->free_timestamp = timestamp;

	return 0;
}
