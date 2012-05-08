/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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
#ifndef _VCD_UTIL_H_
#define _VCD_UTIL_H_
#include "vidc_type.h"
#include "vcd_api.h"

#if DEBUG

#define VCD_MSG_LOW(xx_fmt...)		printk(KERN_INFO "\n\t* [VID] " xx_fmt)
#define VCD_MSG_MED(xx_fmt...)		printk(KERN_INFO "\n  * [VID] " xx_fmt)
#define VCD_MSG_HIGH(xx_fmt...)		printk(KERN_WARNING "\n [VID] " xx_fmt)

#else

#define VCD_MSG_LOW(xx_fmt...)
#define VCD_MSG_MED(xx_fmt...)
#define VCD_MSG_HIGH(xx_fmt...)

#endif

#define VCD_MSG_ERROR(xx_fmt...)	printk(KERN_ERR "\n [VID] err: " xx_fmt)
#define VCD_MSG_FATAL(xx_fmt...)	printk(KERN_ERR "\n [VID] <FATAL> " xx_fmt)

#define VCD_FAILED_RETURN(rc, xx_fmt...)		\
	do {						\
		if (VCD_FAILED(rc)) {			\
			printk(KERN_ERR  xx_fmt);	\
			return rc;			\
		}					\
	} while	(0)

#define VCD_FAILED_DEVICE_FATAL(rc) \
	(rc == VCD_ERR_HW_FATAL ? true : false)
#define VCD_FAILED_CLIENT_FATAL(rc) \
	(rc == VCD_ERR_CLIENT_FATAL ? true : false)

#define VCD_FAILED_FATAL(rc)  \
	((VCD_FAILED_DEVICE_FATAL(rc) || VCD_FAILED_CLIENT_FATAL(rc)) \
	? true : false)

#endif
