/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Andy Shevchenko <andy@smile.org.ua>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_VERSION_H
#define _SQN_VERSION_H

#include <linux/autoconf.h>

#ifdef CONFIG_SDIO_SQN
#define SQN_MODULE_NAME		"sequans_sdio"
#elif defined (CONFIG_USB_SQN)
#define SQN_MODULE_NAME		"sequans_usb"
#else
#define SQN_MODULE_NAME		"sequans_xxx"
#endif

#define SQN_MODULE_VERSION	"1.2.153"

#endif /* _SQN_VERSION_H */
