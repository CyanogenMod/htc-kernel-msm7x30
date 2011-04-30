/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

#include "kgsl.h"
#include "kgsl_cmdstream.h"
#include "kgsl_sharedmem.h"
#include "kgsl_yamato.h"

int kgsl_cmdstream_init(struct kgsl_device *device)
{
	return 0;
}

int kgsl_cmdstream_close(struct kgsl_device *device)
{
	struct kgsl_mem_entry *entry, *entry_tmp;

	list_for_each_entry_safe(entry, entry_tmp, &device->memqueue, list) {
		list_del(&entry->list);
		kgsl_destroy_mem_entry(entry);
	}
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

void kgsl_cmdstream_memqueue_drain(struct kgsl_device *device)
{
	struct kgsl_mem_entry *entry, *entry_tmp;
	uint32_t ts_processed;

	BUG_ON(!mutex_is_locked(&kgsl_driver.mutex));

	/* get current EOP timestamp */
	ts_processed = device->ftbl.device_cmdstream_readtimestamp(
					device,
					KGSL_TIMESTAMP_RETIRED);

	list_for_each_entry_safe(entry, entry_tmp, &device->memqueue, list) {
		KGSL_MEM_DBG("ts_processed %d ts_free %d gpuaddr %x)\n",
			     ts_processed, entry->free_timestamp,
			     entry->memdesc.gpuaddr);
		if (!timestamp_cmp(ts_processed, entry->free_timestamp))
			break;

		list_del(&entry->list);
		kgsl_destroy_mem_entry(entry);
	}
}

/* to be called when a process is destroyed, this walks the memqueue and
 * frees any entryies that belong to the dying process
 */
void kgsl_cmdstream_memqueue_cleanup(struct kgsl_device *device,
				     struct kgsl_process_private *private)
{
	struct kgsl_mem_entry *entry, *entry_tmp;

	BUG_ON(!mutex_is_locked(&kgsl_driver.mutex));

	list_for_each_entry_safe(entry, entry_tmp, &device->memqueue, list) {
		if (entry->priv == private) {
			list_del(&entry->list);
			kgsl_destroy_mem_entry(entry);
		}
	}
}

void
kgsl_cmdstream_freememontimestamp(struct kgsl_device *device,
				  struct kgsl_mem_entry *entry,
				  uint32_t timestamp,
				  enum kgsl_timestamp_type type)
{
	BUG_ON(!mutex_is_locked(&kgsl_driver.mutex));
	KGSL_MEM_DBG("enter (dev %p gpuaddr %x ts %d)\n",
		     device, entry->memdesc.gpuaddr, timestamp);

	entry->free_timestamp = timestamp;

	list_add_tail(&entry->list, &device->memqueue);
}
