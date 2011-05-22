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


#ifndef _SQN_FIRMWARE_H
#define _SQN_FIRMWARE_H


#define SQN_DEFAULT_FW_NAME	"sequans_boot.bin"
extern char *fw1130_name;
extern char *fw1210_name;


int sqn_load_firmware(struct sdio_func *func);

#endif /* _SQN_FIRMWARE_H */
