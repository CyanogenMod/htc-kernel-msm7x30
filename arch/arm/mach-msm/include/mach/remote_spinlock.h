/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Part of this this code is based on the standard ARM spinlock
 * implementation (asm/spinlock.h) found in the 2.6.29 kernel.
 */

#ifndef __ASM__ARCH_QC_REMOTE_SPINLOCK_H
#define __ASM__ARCH_QC_REMOTE_SPINLOCK_H

#include <linux/types.h>

#if defined(CONFIG_ARCH_MSM7X30)
#define SMEM_DAL_SPINLOCK_BASE 0x1000
struct dek_spinlock {
	volatile uint8_t self_lock;
	volatile uint8_t other_lock;
	volatile uint8_t next_yield;
	uint8_t pad;
};

typedef union {
	volatile uint32_t lock;
	struct dek_spinlock dek;
} raw_remote_spinlock_t;
#else
typedef struct {
	volatile uint32_t lock;
} raw_remote_spinlock_t;
#endif

typedef raw_remote_spinlock_t *_remote_spinlock_t;

#if defined(CONFIG_ARCH_MSM7X30)
#define remote_spin_lock_id_t const char *
#else
#define remote_spin_lock_id_t uint32_t
#endif
static inline void __raw_remote_ex_spin_lock(raw_remote_spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]\n"
"	teqeq	%0, #0\n"
"	bne	1b"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc");

	smp_mb();
}

static inline void __raw_remote_ex_spin_unlock(raw_remote_spinlock_t *lock)
{
	smp_mb();

	__asm__ __volatile__(
"	str	%1, [%0]\n"
	:
	: "r" (&lock->lock), "r" (0)
	: "cc");
}

static inline void __raw_remote_swp_spin_lock(raw_remote_spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	swp	%0, %2, [%1]\n"
"	teq	%0, #0\n"
"	bne	1b"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc");

	smp_mb();
}

static inline void __raw_remote_swp_spin_unlock(raw_remote_spinlock_t *lock)
{
	smp_mb();

	__asm__ __volatile__(
"	str	%1, [%0]"
	:
	: "r" (&lock->lock), "r" (0)
	: "cc");
}


int _remote_spin_lock_init(remote_spin_lock_id_t id, _remote_spinlock_t *lock);

/* Only use SWP-based spinlocks for ARM11 apps processors where the LDREX/STREX
 * instructions are unable to lock shared memory for exclusive access. */
#if defined(CONFIG_ARCH_MSM7X30)
void _remote_spin_lock(_remote_spinlock_t *lock);
void _remote_spin_unlock(_remote_spinlock_t *lock);
int _remote_spin_trylock(_remote_spinlock_t *lock);

/* Remote mutex definitions. */

typedef struct {
	_remote_spinlock_t	r_spinlock;
	uint32_t		delay_us;
} _remote_mutex_t;

struct remote_mutex_id {
	remote_spin_lock_id_t	r_spinlock_id;
	uint32_t		delay_us;
};

int _remote_mutex_init(struct remote_mutex_id *id, _remote_mutex_t *lock);
void _remote_mutex_lock(_remote_mutex_t *lock);
void _remote_mutex_unlock(_remote_mutex_t *lock);
int _remote_mutex_trylock(_remote_mutex_t *lock);
#elif defined(CONFIG_ARCH_MSM_ARM11)
#define _remote_spin_lock(lock)		__raw_remote_swp_spin_lock(*lock)
#define _remote_spin_unlock(lock)	__raw_remote_swp_spin_unlock(*lock)
#else
#define _remote_spin_lock(lock)		__raw_remote_ex_spin_lock(*lock)
#define _remote_spin_unlock(lock)	__raw_remote_ex_spin_unlock(*lock)
#endif	/* CONFIG_ARCH_MSM_ARM11 */
#endif /* __ASM__ARCH_QC_REMOTE_SPINLOCK_H */

