/* arch/arm/mach-msm/include/mach/htc_acdb.h
 *
 * Copyright (C) 2010 HTC, Inc.
 * Author: Alan Liang <alan_liang@htc.com>
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

#ifndef _HTC_ACDB_H_
#define _HTC_ACDB_H_

struct audio_config_data {
	uint32_t device_id;
	uint32_t sample_rate;
	uint32_t offset;
	uint32_t length;
};

struct audio_config_database {
	uint8_t magic[8];
	uint32_t entry_count;
	uint32_t unused;
	struct audio_config_data entry[0];
};

struct acdb_config {
	uint32_t device_id;
	uint32_t sample_rate;
	uint32_t len;
	char data[2048];
	uint32_t checksum;
};

#endif
