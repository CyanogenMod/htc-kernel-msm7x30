//------------------------------------------------------------------------------
//
// Copyright (c) 2004-2010 Atheros Corporation.  All rights reserved.
// 
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
//
//
//------------------------------------------------------------------------------
//
// This file is the include file for Atheros PS and patch parser.
// It implements APIs to parse data buffer with patch and PS information and convert it to HCI commands.
//

#ifndef __AR3KPSPARSER_H
#define __AR3KPSPARSER_H




#include <linux/fs.h>
#include <linux/slab.h>
#include "athdefs.h"
#ifdef HCI_TRANSPORT_SDIO
#include "a_config.h"
#include "a_osapi.h"
#define ATH_MODULE_NAME misc
#include "a_debug.h"
#include "common_drv.h"
#include "hci_transport_api.h"
#include "ar3kconfig.h"
#else
#ifndef A_PRINTF
#define A_PRINTF(args...)     printk(KERN_ALERT args)
#endif /* A_PRINTF */
#include "debug_linux.h"

/* Helper data type declaration */

#define ATH_DEBUG_ERR          (1 << 0)
#define ATH_DEBUG_WARN         (1 << 1)
#define ATH_DEBUG_INFO         (1 << 2)



#define false   0
#define true    1

#ifndef A_MALLOC
#define A_MALLOC(size)  kmalloc((size),GFP_KERNEL)
#endif /* A_MALLOC */
#endif /* HCI_TRANSPORT_UART */

/* String manipulation APIs */
#ifndef A_STRTOUL
#define A_STRTOUL               simple_strtoul
#endif  /* A_STRTOL */

#ifndef A_STRTOL 
#define A_STRTOL                simple_strtol
#endif /* A_STRTOL */


/* The maximum number of bytes possible in a patch entry */
#define MAX_PATCH_SIZE                    20000

/* Maximum HCI packets that will be formed from the Patch file */
#define MAX_NUM_PATCH_ENTRY               (MAX_PATCH_SIZE/MAX_BYTE_LENGTH) + 1







struct ps_cmd_packet
{
    u8 *Hcipacket;
    int packetLen;
};

/* Parses a Patch information buffer and store it in global structure */
int AthDoParsePatch(u8 *, u32 );

/* parses a PS information buffer and stores it in a global structure */
int AthDoParsePS(u8 *, u32 );

/* 
 *  Uses the output of Both AthDoParsePS and AthDoParsePatch APIs to form HCI command array with
 *  all the PS and patch commands.
 *  The list will have the below mentioned commands in order.
 *  CRC command packet
 *  Download patch command(s)
 *  Enable patch Command
 *  PS Reset Command
 *  PS Tag Command(s)
 *
 */  
int AthCreateCommandList(struct ps_cmd_packet **, u32 *);

/* Cleanup the dynamically allicated HCI command list */
int AthFreeCommandList(struct ps_cmd_packet **HciPacketList, u32 numPackets);
#endif /* __AR3KPSPARSER_H */
