/*
 * This is part of the Sequans SQN1130 driver.
 * Copyright 2008 SEQUANS Communications
 * Written by Andy Shevchenko <andy@smile.org.ua>,
 *            Dmitriy Chumak <chumakd@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#ifndef _SQN_MSG_H
#define _SQN_MSG_H

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>

#include "version.h"

#define sqn_pr(level, fmt, arg...)			\
do {							\
	char kthread_name[TASK_COMM_LEN] = { 0 };	\
	/* task_lock(current); */			\
	strncpy(kthread_name, current->comm		\
		, sizeof(current->comm));		\
	/* task_unlock(current); */			\
	printk(level "%s: %s: %s: " fmt			\
	       , SQN_MODULE_NAME, kthread_name		\
	       , __func__				\
	       , ##arg);				\
} while (0)


#define SQN_DEBUG_DUMP 1

#ifdef SQN_DEBUG_DUMP
#define DEBUG_LEVEL	KERN_INFO
#define sqn_pr_dbg_dump(prefix, data, len)			\
do {								\
	unsigned int i = 0;					\
	unsigned int width = 16;				\
	unsigned int len_ = (unsigned int)(len);		\
	while (i < len_) {					\
		if (i % width == 0)				\
			printk(DEBUG_LEVEL "%s: %s: %04x ",	\
			       SQN_MODULE_NAME, (prefix), i);	\
		printk("%02x ", ((unsigned char *)(data))[i++]);\
		if ((i % width == 0) || (i == len_))		\
			printk("\n");				\
	}							\
} while (0)
#else /* !SQN_DEBUG_DUMP */

#define sqn_pr_dbg_dump(prefix, data, len)	do {} while (0)
#endif /* SQN_DEBUG_DUMP */


#if defined(DEBUG)

#ifdef	SQN_DEBUG_LEVEL_INFO
#define DEBUG_LEVEL	KERN_INFO
#else
#define DEBUG_LEVEL	KERN_DEBUG
#endif

#define sqn_pr_dbg(fmt, arg...)  sqn_pr(DEBUG_LEVEL, fmt, ##arg)

#ifdef SQN_DEBUG_TRACE

#define sqn_pr_enter()	sqn_pr_dbg("%s\n", "enter")
#define sqn_pr_leave()	sqn_pr_dbg("%s\n", "leave")

#else /* !SQN_DEBUG_TRACE */

#define sqn_pr_enter()	do {} while (0)
#define sqn_pr_leave()	do {} while (0)

#endif /* SQN_DEBUG_TRACE */

#else /* !DEBUG */

#define sqn_pr_dbg(fmt, arg...)			do {} while (0)

#define sqn_pr_enter()	do {} while (0)
#define sqn_pr_leave()	do {} while (0)

#endif /* DEBUG */


#define sqn_pr_info(fmt, arg...)				\
	pr_info("%s: " fmt, SQN_MODULE_NAME, ##arg)

#define sqn_pr_warn(fmt, arg...)				\
	pr_warning("%s: " fmt, SQN_MODULE_NAME, ##arg)

#define sqn_pr_err(fmt, arg...)					\
	pr_err("%s: " fmt, SQN_MODULE_NAME, ##arg)


void sqn_pr_info_dump(char *prefix, unsigned char *data, unsigned int len);
void sqn_pr_info_dump_rawdata(char *prefix, unsigned char *data, unsigned int len);
int sqn_filter_packet_check(char *prefix, unsigned char *data, unsigned int len);

/*
#define sqn_pr_info_dump(prefix, data, len)			\
do {								\
	unsigned int i = 0;					\
	unsigned int width = 16;				\
	unsigned int len_ = (unsigned int)(len);		\
    sqn_pr_info_trace(prefix, data, len);   \
	while (i < len_) {					\
		if (i % width == 0)				\
			printk(KERN_INFO "%s: %s: %04x ",	\
			       SQN_MODULE_NAME, (prefix), i);	\
		printk("%02x ", ((unsigned char *)(data))[i++]);\
		if ((i % width == 0) || (i == len_))		\
			printk("\n");				\
	}							\
} while (0)
*/

#endif /* _SQN_MSG_H */
