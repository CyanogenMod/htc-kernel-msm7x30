/* include/linux/Ntrig_g41.h - NTRIG_G41 Touch driver
 *
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_NTRIG_H
#define _LINUX_NTRIG_H

#define NTRIG_G41_NAME "ntrig_g41"
#define NTRIG_PACKET_SIZE (136)

#define NTG_PMB_FF		0xFFFFFFFF
#define NTG_PREAMBLE 	0x7EE75AA5

#define CMD_GOTO_BL_VAL_LOAD_FW		(0x0100)
#define CMD_GOTO_BL_VAL_STAY_IN_BL		(0x0101)

#define DFU_CMD_NOT_RELEVANT_DATA 	(0x0)

#define DFU_CMD_START_BYTE			(0x7E)
#define DFU_CMD_GROUP					(0x7)
#define DFU_CMD_FW_TYPE				(0x1)

enum DfuMessageType {
	DFU_CMD_MSG_TYPE_CALL 			= 0x01,
	DFU_CMD_MSG_TYPE_CALL_RESPONSE 	= 0x81
};

enum DfuMessageCode {
	DFU_CMD_MSG_CODE_START				= 0x10,
	DFU_CMD_MSG_CODE_BLOCK_HEADER		= 0x11,
	DFU_CMD_MSG_CODE_DATA_FRAME		= 0x12,
	DFU_CMD_MSG_CODE_DATA_COMPLETE	= 0x13,
	DFU_CMD_MSG_CODE_LOAD_FW			= 0x14,
	DFU_CMD_MSG_CODE_TEST_FW			= 0x17,
	DFU_CMD_MSG_CODE_SET_SELF_VALID		= 0x19
};


#pragma pack(1)

typedef struct {
	char startByte;
	short int address;
	short int totalLength;
	char msgType;
	char msgGroup;
	char msgCode;
	unsigned long returnCode;
} ntg_CmdDfuHeader;

typedef struct {
	ntg_CmdDfuHeader dfuHeader;
	char reserved1;
	char reserved2;
	unsigned long startAddress;
	char fwType;
	char checksum;
} ntg_CmdDfuStart;


typedef struct {
	ntg_CmdDfuHeader dfuHeader;
	unsigned long dataLength;
	unsigned long destinationInRam;
	char lastBlockIndication;
	char checksum;
} ntg_CmdDfuSendBlock;
#if 0
struct ntg_CmdGoToBl {
	char requestType;
	char request;
	short int value;
	unsigned short int index;
	unsigned short int length;
};
#endif
struct ntg_touch_header {
	u8	type;
	u16 length;
	u8	flag;
	u8	channel;
	u8	function;
};

struct ntg_touch_ver {
	u8 repId;
	u8 reserved_0;
	u32 value;
	u16 reserved_1;
};

struct ntg_touch_calibrare_status {
	u8 repId;
	u32 value;
};

struct ntg_touch_Stouch {
	u8	repID;
	u8	bit;
	u16 posX;
	u16 posY;
	u16 posDx;
	u16 posDy;
};

struct ntg_touch_Ptouch {
	u8	repID;
	u8	bit;
	u16 posX;
	u16 posY;
	u16 tip;
	u16 unused;
};

typedef struct  {
	u16	figID;
	u16	posX;
	u16	psoY;
	u16	posDx;
	u16	posDy;
	u8	remove;
	u8 	palm;
} nt_Mtouch;

typedef struct {
	u8 figCNT;
	u8 reseved[3];
	nt_Mtouch Mtouch[6];
} ntg_touch_Mtouch;

struct ntg_command {
	u8 reqTy;
	u8 req;
	u16 vlaue;
	u16 index;
	u16 length;
};

struct ntg_touch_event {
	struct ntg_touch_header header;
	struct ntg_touch_ver	version;
	struct ntg_touch_Stouch  stouch;
	struct ntg_touch_Ptouch  ptouch;
	ntg_CmdDfuStart  DfuStCmd;
	ntg_touch_Mtouch mtouch;
};

#if 0
struct ntg_CmdFmt {
	struct ntg_touch_header head;
	struct ntg_command		cmd;
};
#endif

struct ntg_DfuCmd{
	u8 startAddr;
	u16 Addr;
	u16 Length;
	u8 msgType;
	u8 msgGup;
	u8 msgCode;
	u32 retCode;
	u8 reseved0;
	u8 reserved1;
};

struct ntg_DfuFmt{
	struct ntg_touch_header head;
	struct ntg_DfuCmd	dfu;
};

struct ntrig_finger_data {
	int x;
	int y;
};

struct ntrig_spi_platform_data {
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;

	uint8_t abs_pressure_min;
	uint8_t abs_pressure_max;
	uint8_t abs_width_min;
	uint8_t abs_width_max;
	int	pmic_in;
	int 	pmic_out;
	int 	spi_enable;
};

#pragma pack()


#endif
