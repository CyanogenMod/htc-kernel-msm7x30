/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 */

#include <linux/module.h>
#include "msm_gemini_hw.h"
#include "msm_gemini_core.h"
#include "msm_gemini_platform.h"
#include "msm_gemini_common.h"

static struct msm_gemini_hw_pingpong fe_pingpong_buf;
static struct msm_gemini_hw_pingpong we_pingpong_buf;
static int we_pingpong_index;
static int reset_done_ack;
static spinlock_t reset_lock;
static wait_queue_head_t reset_wait;

int msm_gemini_core_reset(uint8_t op_mode, void *base, int size)
{
	unsigned long flags;
	int rc = 0;
	int tm = 500; /*500ms*/
	memset(&fe_pingpong_buf, 0, sizeof(fe_pingpong_buf));
	fe_pingpong_buf.is_fe = 1;
	we_pingpong_index = 0;
	memset(&we_pingpong_buf, 0, sizeof(we_pingpong_buf));
	spin_lock_irqsave(&reset_lock, flags);
	reset_done_ack = 0;
	spin_unlock_irqrestore(&reset_lock, flags);
	msm_gemini_hw_reset(base, size);
	rc = wait_event_interruptible_timeout(
			reset_wait,
			reset_done_ack,
			msecs_to_jiffies(tm));

	if (!reset_done_ack) {
		GMN_DBG("%s: reset ACK failed %d", __func__, rc);
		return -EBUSY;
	}

	GMN_DBG("%s: reset_done_ack rc %d", __func__, rc);
	spin_lock_irqsave(&reset_lock, flags);
	reset_done_ack = 0;
	spin_unlock_irqrestore(&reset_lock, flags);

	if (op_mode == MSM_GEMINI_MODE_REALTIME_ENCODE) {
		/* Nothing needed for fe buffer cfg, config we only */
		msm_gemini_hw_we_buffer_cfg(1);
	} else {
		/* Nothing needed for fe buffer cfg, config we only */
		msm_gemini_hw_we_buffer_cfg(0);
	}

	/* @todo wait for reset done irq */

	return 0;
}

void msm_gemini_core_release(void)
{
	int i = 0;
	for (i = 0; i < 2; i++) {
		if (we_pingpong_buf.buf_status[i]) {
			msm_gemini_platform_p2v(we_pingpong_buf.buf[i].file);
		}
	}
}

void msm_gemini_core_init(void)
{
	init_waitqueue_head(&reset_wait);
	spin_lock_init(&reset_lock);
}

int msm_gemini_core_fe_start(void)
{
	msm_gemini_hw_fe_start();
	return 0;
}

/* fetch engine */
int msm_gemini_core_fe_buf_update(struct msm_gemini_core_buf *buf)
{
	GMN_DBG("%s:%d] 0x%08x %d 0x%08x %d\n", __func__, __LINE__,
		(int) buf->y_buffer_addr, buf->y_len,
		(int) buf->cbcr_buffer_addr, buf->cbcr_len);
	return msm_gemini_hw_pingpong_update(&fe_pingpong_buf, buf);
}

void *msm_gemini_core_fe_pingpong_irq(int gemini_irq_status, void *context)
{
	return msm_gemini_hw_pingpong_irq(&fe_pingpong_buf);
}

/* write engine */
int msm_gemini_core_we_buf_update(struct msm_gemini_core_buf *buf)
{
	int rc;
	GMN_DBG("%s:%d] 0x%08x 0x%08x %d\n", __func__, __LINE__,
		(int) buf->y_buffer_addr, (int) buf->cbcr_buffer_addr,
		buf->y_len);
	we_pingpong_buf.buf_status[we_pingpong_index] = 0;
	we_pingpong_index = (we_pingpong_index + 1)%2;
	rc = msm_gemini_hw_pingpong_update(&we_pingpong_buf, buf);
	return 0;
}

void *msm_gemini_core_we_pingpong_irq(int gemini_irq_status, void *context)
{
	GMN_DBG("%s:%d]\n", __func__, __LINE__);

	return msm_gemini_hw_pingpong_irq(&we_pingpong_buf);
}

void *msm_gemini_core_framedone_irq(int gemini_irq_status, void *context)
{
	struct msm_gemini_hw_buf *buf_p;

	GMN_DBG("%s:%d]\n", __func__, __LINE__);

	buf_p = msm_gemini_hw_pingpong_active_buffer(&we_pingpong_buf);
	if (buf_p) {
		buf_p->framedone_len = msm_gemini_hw_encode_output_size();

		GMN_DBG("%s:%d] framedone_len %d\n", __func__, __LINE__,
			buf_p->framedone_len);
	}
	return buf_p;
}

void *msm_gemini_core_reset_ack_irq(int gemini_irq_status, void *context)
{
	/* @todo return the status back to msm_gemini_core_reset */
	GMN_DBG("%s:%d]\n", __func__, __LINE__);
	return NULL;
}

void *msm_gemini_core_err_irq(int gemini_irq_status, void *context)
{
	GMN_PR_ERR("%s:%d]\n", __func__, gemini_irq_status);
	return NULL;
}

static int (*msm_gemini_irq_handler) (int, void *, void *);

irqreturn_t msm_gemini_core_irq(int irq_num, void *context)
{
	void *data = NULL;
	unsigned long flags;
	int gemini_irq_status;

	GMN_DBG("%s:%d] irq_num = %d\n", __func__, __LINE__, irq_num);

	gemini_irq_status = msm_gemini_hw_irq_get_status();

	GMN_DBG("%s:%d] gemini_irq_status = %0x\n", __func__, __LINE__,
		gemini_irq_status);

	/*For reset and framedone IRQs, clear all bits*/
	if (gemini_irq_status & 0x400) {
		spin_lock_irqsave(&reset_lock, flags);
		reset_done_ack = 1;
		spin_unlock_irqrestore(&reset_lock, flags);
		wake_up(&reset_wait);
		msm_gemini_hw_irq_clear(HWIO_JPEG_IRQ_CLEAR_RMSK,
			JPEG_IRQ_CLEAR_ALL);
	} else if (gemini_irq_status & 0x1) {
		msm_gemini_hw_irq_clear(HWIO_JPEG_IRQ_CLEAR_RMSK,
			JPEG_IRQ_CLEAR_ALL);
	} else {
		msm_gemini_hw_irq_clear(HWIO_JPEG_IRQ_CLEAR_RMSK,
			gemini_irq_status);
	}

	if (msm_gemini_hw_irq_is_frame_done(gemini_irq_status)) {
		data = msm_gemini_core_framedone_irq(gemini_irq_status,
			context);
		if (msm_gemini_irq_handler)
			msm_gemini_irq_handler(
				MSM_GEMINI_HW_MASK_COMP_FRAMEDONE,
				context, data);
	}

	if (msm_gemini_hw_irq_is_fe_pingpong(gemini_irq_status)) {
		data = msm_gemini_core_fe_pingpong_irq(gemini_irq_status,
			context);
		if (msm_gemini_irq_handler)
			msm_gemini_irq_handler(MSM_GEMINI_HW_MASK_COMP_FE,
				context, data);
	}

	if (msm_gemini_hw_irq_is_we_pingpong(gemini_irq_status) &&
	    !msm_gemini_hw_irq_is_frame_done(gemini_irq_status)) {
		data = msm_gemini_core_we_pingpong_irq(gemini_irq_status,
			context);
		if (msm_gemini_irq_handler)
			msm_gemini_irq_handler(MSM_GEMINI_HW_MASK_COMP_WE,
				context, data);
	}

	if (msm_gemini_hw_irq_is_reset_ack(gemini_irq_status)) {
		data = msm_gemini_core_reset_ack_irq(gemini_irq_status,
			context);
		if (msm_gemini_irq_handler)
			msm_gemini_irq_handler(
				MSM_GEMINI_HW_MASK_COMP_RESET_ACK,
				context, data);
	}

	/* Unexpected/unintended HW interrupt */
	if (msm_gemini_hw_irq_is_err(gemini_irq_status)) {
		data = msm_gemini_core_err_irq(gemini_irq_status, context);
		if (msm_gemini_irq_handler)
			msm_gemini_irq_handler(MSM_GEMINI_HW_MASK_COMP_ERR,
				context, data);
	}

	return IRQ_HANDLED;
}

void msm_gemini_core_irq_install(int (*irq_handler) (int, void *, void *))
{
	msm_gemini_irq_handler = irq_handler;
}

void msm_gemini_core_irq_remove(void)
{
	msm_gemini_irq_handler = NULL;
}
