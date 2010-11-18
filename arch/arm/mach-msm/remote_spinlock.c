/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <asm/system.h>

#include <mach/remote_spinlock.h>
#include <mach/dal.h>
#include "smd_private.h"

#define SMEM_SPINLOCK_COUNT 8
#define SMEM_SPINLOCK_ARRAY_SIZE (SMEM_SPINLOCK_COUNT * sizeof(uint32_t))

#if defined(CONFIG_ARCH_MSM7X30)
static int remote_spinlock_smem_init(int id, _remote_spinlock_t *lock)
{
	_remote_spinlock_t spinlock_start;

	if (id >= SMEM_SPINLOCK_COUNT)
		return -EINVAL;

	spinlock_start = smem_alloc(SMEM_SPINLOCK_ARRAY,
				    SMEM_SPINLOCK_ARRAY_SIZE);
	if (spinlock_start == NULL)
		return -ENXIO;

	*lock = spinlock_start + id;

	return 0;
}

static int
remote_spinlock_dal_init(const char *chunk_name, _remote_spinlock_t *lock)
{
	void *dal_smem_start, *dal_smem_end;
	uint32_t dal_smem_size;
	struct dal_chunk_header *cur_header;

	if (!chunk_name)
		return -EINVAL;


#if defined(CONFIG_QCT_LTE)
	dal_smem_start = smem_get_entry(SMEM_DAL_AREA, &dal_smem_size);
#else
	dal_smem_start = smem_item(SMEM_DAL_AREA, &dal_smem_size);
#endif

	if (!dal_smem_start)
		return -ENXIO;

	dal_smem_end = dal_smem_start + dal_smem_size;

	/* Find first chunk header */
	cur_header = (struct dal_chunk_header *)
			(((uint32_t)dal_smem_start + (4095)) & ~4095);
	*lock = NULL;
	while (cur_header->size != 0
		&& ((uint32_t)(cur_header + 1) < (uint32_t)dal_smem_end)) {

		/* Check if chunk name matches */
		if (!strncmp(cur_header->name, chunk_name,
						DAL_CHUNK_NAME_LENGTH)) {
			*lock = (_remote_spinlock_t)&cur_header->lock;
			return 0;
		}
		cur_header = (void *)cur_header + cur_header->size;
	}

	pr_err("%s: DAL remote lock \"%s\" not found.\n", __func__,
		chunk_name);
	return -EINVAL;
}

#define DEK_LOCK_REQUEST		1
#define DEK_LOCK_YIELD			(!DEK_LOCK_REQUEST)
#define DEK_YIELD_TURN_SELF		0
static inline void __raw_remote_dek_spin_lock(raw_remote_spinlock_t *lock)
{
	lock->dek.self_lock = DEK_LOCK_REQUEST;

	while (lock->dek.other_lock) {

		if (lock->dek.next_yield == DEK_YIELD_TURN_SELF)
			lock->dek.self_lock = DEK_LOCK_YIELD;

		while (lock->dek.other_lock)
			;

		lock->dek.self_lock = DEK_LOCK_REQUEST;
	}
	lock->dek.next_yield = DEK_YIELD_TURN_SELF;

	smp_mb();
}

static inline int __raw_remote_dek_spin_trylock(raw_remote_spinlock_t *lock)
{
	lock->dek.self_lock = DEK_LOCK_REQUEST;

	if (lock->dek.other_lock) {
		lock->dek.self_lock = DEK_LOCK_YIELD;
		return 0;
	}

	lock->dek.next_yield = DEK_YIELD_TURN_SELF;

	smp_mb();
	return 1;
}

static inline void __raw_remote_dek_spin_unlock(raw_remote_spinlock_t *lock)
{
	smp_mb();

	lock->dek.self_lock = DEK_LOCK_YIELD;
}
#endif

int _remote_spin_lock_init(remote_spin_lock_id_t id, _remote_spinlock_t *lock)
{
#if defined(CONFIG_ARCH_MSM7X30)
	BUG_ON(id == NULL);

	if (id[0] == 'D' && id[1] == ':') {
		/* DAL chunk name starts after "D:" */
		return remote_spinlock_dal_init(&id[2], lock);
	} else if (id[0] == 'S' && id[1] == ':') {
		/* Single-digit SMEM lock ID follows "S:" */
		BUG_ON(id[3] != '\0');
		return remote_spinlock_smem_init((((uint8_t)id[2])-'0'), lock);
	} else
		return -EINVAL;
#else
	_remote_spinlock_t spinlock_start;

	if (id >= SMEM_SPINLOCK_COUNT)
		return -EINVAL;

	spinlock_start = smem_alloc(SMEM_SPINLOCK_ARRAY,
				    SMEM_SPINLOCK_ARRAY_SIZE);
	if (spinlock_start == NULL)
		return -ENXIO;

	*lock = spinlock_start + id;

	return 0;
#endif
}

#if defined(CONFIG_ARCH_MSM7X30)
void _remote_spin_lock(_remote_spinlock_t *lock)
{
	__raw_remote_dek_spin_lock((raw_remote_spinlock_t *) (*lock));
}

void _remote_spin_unlock(_remote_spinlock_t *lock)
{
	__raw_remote_dek_spin_unlock((raw_remote_spinlock_t *) (*lock));
}

int _remote_spin_trylock(_remote_spinlock_t *lock)
{
	return (__raw_remote_dek_spin_trylock((raw_remote_spinlock_t *) (*lock)));
}

int _remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock)
{
	BUG_ON(id == NULL);

	lock->delay_us = id->delay_us;
	return _remote_spin_lock_init(id->r_spinlock_id, &(lock->r_spinlock));
}
EXPORT_SYMBOL(_remote_mutex_init);

void _remote_mutex_lock(_remote_mutex_t *lock)
{
	while (!_remote_spin_trylock(&(lock->r_spinlock))) {
		if (lock->delay_us >= 1000)
			msleep(lock->delay_us/1000);
		else
			udelay(lock->delay_us);
	}
}
EXPORT_SYMBOL(_remote_mutex_lock);

void _remote_mutex_unlock(_remote_mutex_t *lock)
{
	_remote_spin_unlock(&(lock->r_spinlock));
}
EXPORT_SYMBOL(_remote_mutex_unlock);

int _remote_mutex_trylock(_remote_mutex_t *lock)
{
	return _remote_spin_trylock(&(lock->r_spinlock));
}
EXPORT_SYMBOL(_remote_mutex_trylock);

#endif
