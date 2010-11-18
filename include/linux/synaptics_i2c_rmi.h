/*
 * include/linux/synaptics_i2c_rmi.h - platform data structure for f75375s sensor
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef _LINUX_SYNAPTICS_I2C_RMI_H
#define _LINUX_SYNAPTICS_I2C_RMI_H

#define SYNAPTICS_I2C_RMI_NAME "synaptics-rmi-ts"
#define SYNAPTICS_T1007_NAME "synaptics-t1007"
#define SYNAPTICS_T1021_NAME "synaptics-t1021"
#define SYNAPTICS_3K_NAME "synaptics-3k"

enum {
	SYNAPTICS_FLIP_X = 1UL << 0,
	SYNAPTICS_FLIP_Y = 1UL << 1,
	SYNAPTICS_SWAP_XY = 1UL << 2,
	SYNAPTICS_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

enum {
	FINGER_1_REPORT = 1 << 0,
	FINGER_2_REPORT = 1 << 1,
};

struct synaptics_virtual_key {
	int keycode;
	int range_min;
	int range_max;
};

struct synaptics_i2c_rmi_platform_data {
	uint32_t version;	/* Use this entry for panels with */
				/* (major << 8 | minor) version or above. */
				/* If non-zero another array entry follows */
	int (*power)(int on);	/* Only valid in first array entry */
	struct synaptics_virtual_key *virtual_key;
	uint8_t virtual_key_num;
	uint8_t sensitivity;
	uint8_t finger_support;
	uint32_t gap_area;
	uint32_t key_area;
	uint32_t flags;
	unsigned long irqflags;
	uint32_t inactive_left; /* 0x10000 = screen width */
	uint32_t inactive_right; /* 0x10000 = screen width */
	uint32_t inactive_top; /* 0x10000 = screen height */
	uint32_t inactive_bottom; /* 0x10000 = screen height */
	uint32_t snap_left_on; /* 0x10000 = screen width */
	uint32_t snap_left_off; /* 0x10000 = screen width */
	uint32_t snap_right_on; /* 0x10000 = screen width */
	uint32_t snap_right_off; /* 0x10000 = screen width */
	uint32_t snap_top_on; /* 0x10000 = screen height */
	uint32_t snap_top_off; /* 0x10000 = screen height */
	uint32_t snap_bottom_on; /* 0x10000 = screen height */
	uint32_t snap_bottom_off; /* 0x10000 = screen height */
	uint32_t fuzz_x; /* 0x10000 = screen width */
	uint32_t fuzz_y; /* 0x10000 = screen height */
	uint16_t abs_x_min;
	uint16_t abs_x_max;
	uint16_t abs_y_min;
	uint16_t abs_y_max;
	int fuzz_p;
	int fuzz_w;
	uint32_t display_width;
	uint32_t display_height;
	int8_t sensitivity_adjust;
	uint32_t dup_threshold;
	uint32_t margin_inactive_pixel[4];
};

struct page_description {
	uint8_t addr;
	uint8_t value;
};

struct syn_finger_data {
	int x;
	int y;
	int w;
	int z;
};

#endif /* _LINUX_SYNAPTICS_I2C_RMI_H */
