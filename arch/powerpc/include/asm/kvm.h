/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Copyright IBM Corp. 2007
 *
 * Authors: Hollis Blanchard <hollisb@us.ibm.com>
 */

#ifndef __LINUX_KVM_POWERPC_H
#define __LINUX_KVM_POWERPC_H

#include <linux/types.h>

struct kvm_regs {
	__u64 pc;
	__u64 cr;
	__u64 ctr;
	__u64 lr;
	__u64 xer;
	__u64 msr;
	__u64 srr0;
	__u64 srr1;
	__u64 pid;

	__u64 sprg0;
	__u64 sprg1;
	__u64 sprg2;
	__u64 sprg3;
	__u64 sprg4;
	__u64 sprg5;
	__u64 sprg6;
	__u64 sprg7;

	__u64 gpr[32];
};

#define KVM_SREGS_E_IMPL_NONE	0
#define KVM_SREGS_E_IMPL_FSL	1

#define KVM_SREGS_E_FSL_PIDn	(1 << 0) /* PID1/PID2 */

/*
 * Feature bits indicate which sections of the sregs struct are valid,
 * both in KVM_GET_SREGS and KVM_SET_SREGS.  On KVM_SET_SREGS, registers
 * corresponding to unset feature bits will not be modified.  This allows
 * restoring a checkpoint made without that feature, while keeping the
 * default values of the new registers.
 *
 * KVM_SREGS_E_BASE contains:
 * CSRR0/1 (refers to SRR2/3 on 40x)
 * ESR
 * DEAR
 * MCSR
 * TSR
 * TCR
 * DEC
 * TB
 * VRSAVE (USPRG0)
 */
#define KVM_SREGS_E_BASE		(1 << 0)

/*
 * KVM_SREGS_E_ARCH206 contains:
 *
 * PIR
 * MCSRR0/1
 * DECAR
 * IVPR
 */
#define KVM_SREGS_E_ARCH206		(1 << 1)

/*
 * Contains EPCR, plus the upper half of 64-bit registers
 * that are 32-bit on 32-bit implementations.
 */
#define KVM_SREGS_E_64			(1 << 2)

#define KVM_SREGS_E_SPRG8		(1 << 3)
#define KVM_SREGS_E_MCIVPR		(1 << 4)

/*
 * IVORs are used -- contains IVOR0-15, plus additional IVORs
 * in combination with an appropriate feature bit.
 */
#define KVM_SREGS_E_IVOR		(1 << 5)

/*
 * Contains MAS0-4, MAS6-7, TLBnCFG, MMUCFG.
 * Also TLBnPS if MMUCFG[MAVN] = 1.
 */
#define KVM_SREGS_E_ARCH206_MMU		(1 << 6)

/* DBSR, DBCR, IAC, DAC, DVC */
#define KVM_SREGS_E_DEBUG		(1 << 7)

/* Enhanced debug -- DSRR0/1, SPRG9 */
#define KVM_SREGS_E_ED			(1 << 8)

/* Embedded Floating Point (SPE) -- IVOR32-34 if KVM_SREGS_E_IVOR */
#define KVM_SREGS_E_SPE			(1 << 9)

/* External Proxy (EXP) -- EPR */
#define KVM_SREGS_EXP			(1 << 10)

/* External PID (E.PD) -- EPSC/EPLC */
#define KVM_SREGS_E_PD			(1 << 11)

/* Processor Control (E.PC) -- IVOR36-37 if KVM_SREGS_E_IVOR */
#define KVM_SREGS_E_PC			(1 << 12)

/* Page table (E.PT) -- EPTCFG */
#define KVM_SREGS_E_PT			(1 << 13)

/* Embedded Performance Monitor (E.PM) -- IVOR35 if KVM_SREGS_E_IVOR */
#define KVM_SREGS_E_PM			(1 << 14)

/*
 * Special updates:
 *
 * Some registers may change even while a vcpu is not running.
 * To avoid losing these changes, by default these registers are
 * not updated by KVM_SET_SREGS.  To force an update, set the bit
 * in u.e.update_special corresponding to the register to be updated.
 *
 * The update_special field is zero on return from KVM_GET_SREGS.
 *
 * When restoring a checkpoint, the caller can set update_special
 * to 0xffffffff to ensure that everything is restored, even new features
 * that the caller doesn't know about.
 */
#define KVM_SREGS_E_UPDATE_MCSR		(1 << 0)
#define KVM_SREGS_E_UPDATE_TSR		(1 << 1)
#define KVM_SREGS_E_UPDATE_DEC		(1 << 2)
#define KVM_SREGS_E_UPDATE_DBSR		(1 << 3)

/*
 * In KVM_SET_SREGS, reserved/pad fields must be left untouched from a
 * previous KVM_GET_REGS.
 *
 * Unless otherwise indicated, setting any register with KVM_SET_SREGS
 * directly sets its value.  It does not trigger any special semantics such
 * as write-one-to-clear.  Calling KVM_SET_SREGS on an unmodified struct
 * just received from KVM_GET_SREGS is always a no-op.
 */
struct kvm_sregs {
	__u32 pvr;
	union {
		struct {
			__u64 sdr1;
			struct {
				struct {
					__u64 slbe;
					__u64 slbv;
				} slb[64];
			} ppc64;
			struct {
				__u32 sr[16];
				__u64 ibat[8]; 
				__u64 dbat[8]; 
			} ppc32;
		} s;
		struct {
			union {
				struct { /* KVM_SREGS_E_IMPL_FSL */
					__u32 features; /* KVM_SREGS_E_FSL_ */
					__u32 svr;
					__u64 mcar;
					__u32 hid0;

					/* KVM_SREGS_E_FSL_PIDn */
					__u32 pid1, pid2;
				} fsl;
				__u8 pad[256];
			} impl;

			__u32 features; /* KVM_SREGS_E_ */
			__u32 impl_id;	/* KVM_SREGS_E_IMPL_ */
			__u32 update_special; /* KVM_SREGS_E_UPDATE_ */
			__u32 pir;	/* read-only */
			__u64 sprg8;
			__u64 sprg9;	/* E.ED */
			__u64 csrr0;
			__u64 dsrr0;	/* E.ED */
			__u64 mcsrr0;
			__u32 csrr1;
			__u32 dsrr1;	/* E.ED */
			__u32 mcsrr1;
			__u32 esr;
			__u64 dear;
			__u64 ivpr;
			__u64 mcivpr;
			__u64 mcsr;	/* KVM_SREGS_E_UPDATE_MCSR */

			__u32 tsr;	/* KVM_SREGS_E_UPDATE_TSR */
			__u32 tcr;
			__u32 decar;
			__u32 dec;	/* KVM_SREGS_E_UPDATE_DEC */

			/*
			 * Userspace can read TB directly, but the
			 * value reported here is consistent with "dec".
			 *
			 * Read-only.
			 */
			__u64 tb;

			__u32 dbsr;	/* KVM_SREGS_E_UPDATE_DBSR */
			__u32 dbcr[3];
			__u32 iac[4];
			__u32 dac[2];
			__u32 dvc[2];
			__u8 num_iac;	/* read-only */
			__u8 num_dac;	/* read-only */
			__u8 num_dvc;	/* read-only */
			__u8 pad;

			__u32 epr;	/* EXP */
			__u32 vrsave;	/* a.k.a. USPRG0 */
			__u32 epcr;	/* KVM_SREGS_E_64 */

			__u32 mas0;
			__u32 mas1;
			__u64 mas2;
			__u64 mas7_3;
			__u32 mas4;
			__u32 mas6;

			__u32 ivor_low[16]; /* IVOR0-15 */
			__u32 ivor_high[18]; /* IVOR32+, plus room to expand */

			__u32 mmucfg;	/* read-only */
			__u32 eptcfg;	/* E.PT, read-only */
			__u32 tlbcfg[4];/* read-only */
			__u32 tlbps[4]; /* read-only */

			__u32 eplc, epsc; /* E.PD */
		} e;
		__u8 pad[1020];
	} u;
};

struct kvm_fpu {
	__u64 fpr[32];
};

struct kvm_debug_exit_arch {
};

/* for KVM_SET_GUEST_DEBUG */
struct kvm_guest_debug_arch {
};

#define KVM_REG_MASK		0x001f
#define KVM_REG_EXT_MASK	0xffe0
#define KVM_REG_GPR		0x0000
#define KVM_REG_FPR		0x0020
#define KVM_REG_QPR		0x0040
#define KVM_REG_FQPR		0x0060

#define KVM_INTERRUPT_SET	-1U
#define KVM_INTERRUPT_UNSET	-2U
#define KVM_INTERRUPT_SET_LEVEL	-3U

#endif /* __LINUX_KVM_POWERPC_H */
