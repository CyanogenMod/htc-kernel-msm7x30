/* Copyright (c) 2002,2007-2011, Code Aurora Forum. All rights reserved.
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
 */
#ifndef __ADRENO_PM4TYPES_H
#define __ADRENO_PM4TYPES_H


#define CP_PKT_MASK	0xc0000000

#define CP_TYPE0_PKT	((unsigned int)0 << 30)
#define CP_TYPE1_PKT	((unsigned int)1 << 30)
#define CP_TYPE2_PKT	((unsigned int)2 << 30)
#define CP_TYPE3_PKT	((unsigned int)3 << 30)


/* type3 packets */
/* initialize CP's micro-engine */
#define CP_ME_INIT		0x48

/* skip N 32-bit words to get to the next packet */
#define CP_NOP			0x10

/* indirect buffer dispatch.  prefetch parser uses this packet type to determine
*  whether to pre-fetch the IB
*/
#define CP_INDIRECT_BUFFER	0x3f

/* indirect buffer dispatch.  same as IB, but init is pipelined */
#define CP_INDIRECT_BUFFER_PFD	0x37

/* wait for the IDLE state of the engine */
#define CP_WAIT_FOR_IDLE	0x26

/* wait until a register or memory location is a specific value */
#define CP_WAIT_REG_MEM	0x3c

/* wait until a register location is equal to a specific value */
#define CP_WAIT_REG_EQ		0x52

/* wait until a register location is >= a specific value */
#define CP_WAT_REG_GTE		0x53

/* wait until a read completes */
#define CP_WAIT_UNTIL_READ	0x5c

/* wait until all base/size writes from an IB_PFD packet have completed */
#define CP_WAIT_IB_PFD_COMPLETE 0x5d

/* register read/modify/write */
#define CP_REG_RMW		0x21

/* reads register in chip and writes to memory */
#define CP_REG_TO_MEM		0x3e

/* write N 32-bit words to memory */
#define CP_MEM_WRITE		0x3d

/* write CP_PROG_COUNTER value to memory */
#define CP_MEM_WRITE_CNTR	0x4f

/* conditional execution of a sequence of packets */
#define CP_COND_EXEC		0x44

/* conditional write to memory or register */
#define CP_COND_WRITE		0x45

/* generate an event that creates a write to memory when completed */
#define CP_EVENT_WRITE		0x46

/* generate a VS|PS_done event */
#define CP_EVENT_WRITE_SHD	0x58

/* generate a cache flush done event */
#define CP_EVENT_WRITE_CFL	0x59

/* generate a z_pass done event */
#define CP_EVENT_WRITE_ZPD	0x5b


/* initiate fetch of index buffer and draw */
#define CP_DRAW_INDX		0x22

/* draw using supplied indices in packet */
#define CP_DRAW_INDX_2		0x36

/* initiate fetch of index buffer and binIDs and draw */
#define CP_DRAW_INDX_BIN	0x34

/* initiate fetch of bin IDs and draw using supplied indices */
#define CP_DRAW_INDX_2_BIN	0x35


/* begin/end initiator for viz query extent processing */
#define CP_VIZ_QUERY		0x23

/* fetch state sub-blocks and initiate shader code DMAs */
#define CP_SET_STATE		0x25

/* load constant into chip and to memory */
#define CP_SET_CONSTANT	0x2d

/* load sequencer instruction memory (pointer-based) */
#define CP_IM_LOAD		0x27

/* load sequencer instruction memory (code embedded in packet) */
#define CP_IM_LOAD_IMMEDIATE	0x2b

/* load constants from a location in memory */
#define CP_LOAD_CONSTANT_CONTEXT 0x2e

/* selective invalidation of state pointers */
#define CP_INVALIDATE_STATE	0x3b


/* dynamically changes shader instruction memory partition */
#define CP_SET_SHADER_BASES	0x4A

/* sets the 64-bit BIN_MASK register in the PFP */
#define CP_SET_BIN_MASK	0x50

/* sets the 64-bit BIN_SELECT register in the PFP */
#define CP_SET_BIN_SELECT	0x51


/* updates the current context, if needed */
#define CP_CONTEXT_UPDATE	0x5e

/* generate interrupt from the command stream */
#define CP_INTERRUPT		0x40


/* copy sequencer instruction memory to system memory */
#define CP_IM_STORE            0x2c

/*
 * for a20x
 * program an offset that will added to the BIN_BASE value of
 * the 3D_DRAW_INDX_BIN packet
 */
#define CP_SET_BIN_BASE_OFFSET     0x4B

/*
 * for a22x
 * sets draw initiator flags register in PFP, gets bitwise-ORed into
 * every draw initiator
 */
#define CP_SET_DRAW_INIT_FLAGS      0x4B

#define CP_SET_PROTECTED_MODE  0x5f /* sets the register protection mode */


/* packet header building macros */
#define cp_type0_packet(regindx, cnt) \
	(CP_TYPE0_PKT | (((cnt)-1) << 16) | ((regindx) & 0x7FFF))

#define cp_type0_packet_for_sameregister(regindx, cnt) \
	((CP_TYPE0_PKT | (((cnt)-1) << 16) | ((1 << 15) | \
		((regindx) & 0x7FFF)))

#define cp_type1_packet(reg0, reg1) \
	 (CP_TYPE1_PKT | ((reg1) << 12) | (reg0))

#define cp_type3_packet(opcode, cnt) \
	 (CP_TYPE3_PKT | (((cnt)-1) << 16) | (((opcode) & 0xFF) << 8))

#define cp_predicated_type3_packet(opcode, cnt) \
	 (CP_TYPE3_PKT | (((cnt)-1) << 16) | (((opcode) & 0xFF) << 8) | 0x1)

#define cp_nop_packet(cnt) \
	 (CP_TYPE3_PKT | (((cnt)-1) << 16) | (CP_NOP << 8))


/* packet headers */
#define CP_HDR_ME_INIT	cp_type3_packet(CP_ME_INIT, 18)
#define CP_HDR_INDIRECT_BUFFER_PFD cp_type3_packet(CP_INDIRECT_BUFFER_PFD, 2)
#define CP_HDR_INDIRECT_BUFFER	cp_type3_packet(CP_INDIRECT_BUFFER, 2)

/* dword base address of the GFX decode space */
#define SUBBLOCK_OFFSET(reg) ((unsigned int)((reg) - (0x2000)))

/* gmem command buffer length */
#define CP_REG(reg) ((0x4 << 16) | (SUBBLOCK_OFFSET(reg)))

#endif	/* __ADRENO_PM4TYPES_H */
