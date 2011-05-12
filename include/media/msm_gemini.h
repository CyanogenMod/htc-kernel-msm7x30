/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LINUX_MSM_GEMINI_H
#define __LINUX_MSM_GEMINI_H

#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>

#define MSM_GMN_IOCTL_MAGIC 'g'

#define MSM_GMN_IOCTL_GET_HW_VERSION \
	_IOW(MSM_GMN_IOCTL_MAGIC, 1, struct msm_gemini_hw_cmd *)

#define MSM_GMN_IOCTL_RESET \
	_IOW(MSM_GMN_IOCTL_MAGIC, 2, struct msm_gemini_ctrl_cmd *)

#define MSM_GMN_IOCTL_STOP \
	_IOW(MSM_GMN_IOCTL_MAGIC, 3, struct msm_gemini_hw_cmds *)

#define MSM_GMN_IOCTL_START \
	_IOW(MSM_GMN_IOCTL_MAGIC, 4, struct msm_gemini_hw_cmds *)

#define MSM_GMN_IOCTL_INPUT_BUF_ENQUEUE \
	_IOW(MSM_GMN_IOCTL_MAGIC, 5, struct msm_gemini_buf *)

#define MSM_GMN_IOCTL_INPUT_GET \
	_IOW(MSM_GMN_IOCTL_MAGIC, 6, struct msm_gemini_buf *)

#define MSM_GMN_IOCTL_INPUT_GET_UNBLOCK \
	_IOW(MSM_GMN_IOCTL_MAGIC, 7, int)

#define MSM_GMN_IOCTL_OUTPUT_BUF_ENQUEUE \
	_IOW(MSM_GMN_IOCTL_MAGIC, 8, struct msm_gemini_buf *)

#define MSM_GMN_IOCTL_OUTPUT_GET \
	_IOW(MSM_GMN_IOCTL_MAGIC, 9, struct msm_gemini_buf *)

#define MSM_GMN_IOCTL_OUTPUT_GET_UNBLOCK \
	_IOW(MSM_GMN_IOCTL_MAGIC, 10, int)

#define MSM_GMN_IOCTL_EVT_GET \
	_IOW(MSM_GMN_IOCTL_MAGIC, 11, struct msm_gemini_ctrl_cmd *)

#define MSM_GMN_IOCTL_EVT_GET_UNBLOCK \
	_IOW(MSM_GMN_IOCTL_MAGIC, 12, int)

#define MSM_GMN_IOCTL_HW_CMD \
	_IOW(MSM_GMN_IOCTL_MAGIC, 13, struct msm_gemini_hw_cmd *)

#define MSM_GMN_IOCTL_HW_CMDS \
	_IOW(MSM_GMN_IOCTL_MAGIC, 14, struct msm_gemini_hw_cmds *)

#define MSM_GMN_IOCTL_TEST_DUMP_REGION \
	_IOW(MSM_GMN_IOCTL_MAGIC, 15, unsigned long)

#define MSM_GEMINI_MODE_REALTIME_ENCODE 0
#define MSM_GEMINI_MODE_OFFLINE_ENCODE 1
#define MSM_GEMINI_MODE_REALTIME_ROTATION 2
#define MSM_GEMINI_MODE_OFFLINE_ROTATION 3
struct msm_gemini_ctrl_cmd {
	uint32_t type;
	uint32_t len;
	void     *value;
};

#define MSM_GEMINI_EVT_RESET 0
#define MSM_GEMINI_EVT_FRAMEDONE	1
#define MSM_GEMINI_EVT_ERR 2

struct msm_gemini_buf {
	uint32_t type;
	int      fd;

	void     *vaddr;

	uint32_t y_off;
	uint32_t y_len;
	uint32_t framedone_len;

	uint32_t cbcr_off;
	uint32_t cbcr_len;

	uint32_t num_of_mcu_rows;
};

#define MSM_GEMINI_HW_CMD_TYPE_READ      0
#define MSM_GEMINI_HW_CMD_TYPE_WRITE     1
#define MSM_GEMINI_HW_CMD_TYPE_WRITE_OR  2
#define MSM_GEMINI_HW_CMD_TYPE_UWAIT     3
#define MSM_GEMINI_HW_CMD_TYPE_MWAIT     4
#define MSM_GEMINI_HW_CMD_TYPE_MDELAY    5
#define MSM_GEMINI_HW_CMD_TYPE_UDELAY    6
struct msm_gemini_hw_cmd {

	uint32_t type:4;

	/* n microseconds of timeout for WAIT */
	/* n microseconds of time for DELAY */
	/* repeat n times for READ/WRITE */
	/* max is 0xFFF, 4095 */
	uint32_t n:12;
	uint32_t offset:16;
	uint32_t mask;
	union {
		uint32_t data;   /* for single READ/WRITE/WAIT, n = 1 */
		uint32_t *pdata;   /* for multiple READ/WRITE/WAIT, n > 1 */
	};
};

struct msm_gemini_hw_cmds {
	uint32_t m; /* number of elements in the hw_cmd array */
	struct msm_gemini_hw_cmd hw_cmd[1];
};

#endif /* __LINUX_MSM_GEMINI_H */
