/****************************************************************************/
/*                                                                          */
/*  Name:       target.h                                                    */
/*                                                                          */
/*  Function    this file contains definitions that will probably           */
/*              change for each Bluetooth target system. This includes      */
/*              such things as buffer pool sizes, number of tasks,          */
/*              little endian/big endian conversions, etc...                */
/*                                                                          */
/*  NOTE        This file should always be included first.                  */
/*                                                                          */
/*  Date        Modification                                                */
/*  ------------------------                                                */
/*  7/6/99      Ash  Create                                                 */
/*  8/3/99      Ash  Modifications made for BT specification V1.0a          */
/*                                                                          */
/*  Copyright (c) 1999, Widcomm Inc., All Rights Reserved.                  */
/*  Widcomm Bluetooth Core. Proprietary and confidential.                   */
/*                                                                          */
/****************************************************************************/

#ifndef _TARGET_H
#define _TARGET_H

#define BT_API
#define RFC_API
#define L2C_API

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/spinlock.h>
#include <linux/ioctl.h>
#include <linux/kmod.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/timer.h>

#include <asm/uaccess.h>

#include "data_types.h"
#include "bt_types.h"

#define INT         int
#define UINT    unsigned int
#define DWORD   unsigned long

#ifndef BOOL
#define BOOL int
#endif

#ifndef USHORT
#define USHORT  unsigned short
#endif

#ifndef ULONG
#define ULONG unsigned long
#endif

#ifndef VOID
#define VOID void
#endif

#define malloc(x) kmalloc((x), GFP_ATOMIC)
#define free(x) kfree((x))

#define __cdecl

#define GKI_USE_DYNAMIC_BUFFERS     TRUE           /* TRUE if using dynamic buffers */

#define GKI_NUM_FIXED_BUF_POOLS     2
#define GKI_DEF_BUFPOOL_PERM_MASK   0xfffc          /* Pool ID 0 is public */

/* Define the total number of buffer pools used, fixed and dynamic (Maximum is 16).
*/
#define GKI_NUM_TOTAL_BUF_POOLS    4

#define GKI_MAX_TASKS           2
#define BTU_TASK                0
#define HCI_WRITE_TASK     1

#define GKI_BUF0_SIZE           300
#define GKI_BUF0_MAX            50
#define GKI_POOL_ID_0           0                   /* Public pool*/

#define GKI_BUF1_SIZE           1740
#define GKI_BUF1_MAX            50
#define GKI_POOL_ID_1           1

#define GKI_BUF3_SIZE           1800

#define GKI_BUF3_MAX            50
#define GKI_POOL_ID_3           3

#define RFCOMM_EVT_POOL_ID      GKI_POOL_ID_0   /* Port Up/Down control signals*/
#define HCI_SCO_POOL_ID         GKI_POOL_ID_0   /* all SCO data to/from the device*/
#define HCI_CMD_POOL_ID         GKI_POOL_ID_0   /* HCI commands and events*/
#define L2CAP_CMD_POOL_ID       GKI_POOL_ID_0
#define BTM_CMD_POOL_ID         GKI_POOL_ID_0
#define RFCOMM_CMD_POOL_ID      GKI_POOL_ID_1
#define L2CIF_POOL_ID           GKI_POOL_ID_1
#define BTMIF_POOL_ID           GKI_POOL_ID_1
#define SDP_POOL_ID             GKI_POOL_ID_1
#define HCI_ACL_POOL_ID         GKI_POOL_ID_1   /*all data from the device*/

/* Set this flag to non zero if you want to do buffer ownership checks.
** Note that to do these checks, you should have all tasks that free
** buffers as GKI tasks.
*/
#define GKI_ENABLE_OWNER_CHECK  0


/* Set this flag to non zero if you want to do buffer corruption checks.
** If set, GKI will check buffer tail corruption every time it processes
** a buffer. This is very usefull for debug, and is minimal overhead in
** a running system.
*/
#define GKI_ENABLE_BUF_CORRUPTION_CHECK  1

/***********************************************************************
** Timer related definitions. These should be edited per system. The
** macros should convert milliseconds and seconds to system ticks.
** Applications should use these to calculate the number of ticks to
** sleep for.
**
** The macros below are for a 1ms system tick.

*/
#define GKI_NUM_TIMERS      1

#define GKI_MS_TO_TICKS(x)   (((HZ * x) / 1000))
#define GKI_SECS_TO_TICKS(x) ((HZ * x))
#define GKI_TICKS_TO_MS(x)   (1000/HZ)


#define L2CAP_MTU_SIZE            500

void LogMsg (int TraceMask, const char *format, ...);
#endif

