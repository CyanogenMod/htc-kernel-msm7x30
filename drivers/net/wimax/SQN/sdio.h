/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_SDIO_WRAPPERS_H
#define _SQN_SDIO_WRAPPERS_H

#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

int sqn_sdio_dump_net_pkt(int on);

#endif  /* _SQN_SDIO_WRAPPERS_H */
