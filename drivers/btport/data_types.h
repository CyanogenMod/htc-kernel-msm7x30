/****************************************************************************
**
**  Name        data_types.h
**  $Header: /Bluetooth/gki/data_types.h 7     9/13/00 11:01a Jjose $
**
**  Function    this file contains common data type definitions used
**              throughout the Widcomm Bluetooth code
**
**  Date       Modification
**  -----------------------
**  3/12/99    Create
**	07/27/00	Added nettohs macro for Little Endian
**
**  Copyright (c) 1999, 2000, Widcomm Inc., All Rights Reserved.
**  Proprietary and confidential.
**
*****************************************************************************/

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include "stdio.h"
#include "stdlib.h"
#include <stdarg.h>
#include "string.h"

#ifndef NULL
#define NULL     0
#endif

#ifndef FALSE
#define FALSE  0
#endif

typedef unsigned char   UINT8;
typedef unsigned short  UINT16;
typedef unsigned int    UINT32, *PUINT32;
typedef signed   int    INT32, *PINT32;
typedef signed   char   INT8;
typedef signed   short  INT16;


#if defined(_WIDE_CHAR) || defined(_UNICODE)
typedef unsigned short  WIDECHAR;      /* for _BT_WIN32 possibly use wchar_t */
#define WIDE_NULL_CHAR  ((WIDECHAR)0)
#else
typedef unsigned char   WIDECHAR;
#define WIDE_NULL_CHAR  '\0'
#endif

typedef UINT32          TIME_STAMP;


#ifndef _WINDOWS_VXD
typedef unsigned char   BOOLEAN;

#ifndef TRUE
#define TRUE   (!FALSE)
#endif
#endif

typedef unsigned char   UBYTE;

#define PACKED
#define INLINE

#ifndef BIG_ENDIAN
#define BIG_ENDIAN FALSE
#endif

#define UINT16_LOW_BYTE(x)      ((x) & 0xff)
#define UINT16_HI_BYTE(x)       ((x) >> 8)

/* make up for deficits in the CE compilers*/
#ifdef _WIN32_WCE
#define offsetof(s, m) ((size_t)&(((s *)0)->m))
typedef unsigned long (__cdecl * ThreadMain)(void *);
/*#define _beginthreadCustom(pfn, security, parameter) (CreateThread(NULL, 0, (ThreadMain)pfn, parameter, 0, NULL))*/
/*#define _endthread() (ExitThread(-1))*/
/*#define _beginthreadex(pfn, security, parameter) (CreateThread(NULL, NULL, pfn, parameter, 0, NULL))*/
#endif

#define ARRAY_SIZEOF(x) (sizeof(x)/sizeof(x[0]))

#endif

