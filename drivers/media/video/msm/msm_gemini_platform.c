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
#include <linux/pm_qos_params.h>
#include <linux/clk.h>
#include <mach/clk.h>
#include <linux/io.h>
#include <linux/android_pmem.h>
/*#include <mach/msm_reqs.h>*/

#include "msm_gemini_platform.h"
#include "msm_gemini_common.h"

#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
/* NPA Flow ID */
#define MSM_SYSTEM_BUS_RATE	MSM_AXI_FLOW_JPEG_12MP
#else
/* AXI rate in KHz */
#define MSM_SYSTEM_BUS_RATE	160000
#endif

void msm_gemini_platform_p2v(struct file  *file)
{
	put_pmem_file(file);
}

uint32_t msm_gemini_platform_v2p(int fd, uint32_t len, struct file **file_p)
{
	unsigned long paddr;
	unsigned long kvstart;
	unsigned long size;
	int rc;

	rc = get_pmem_file(fd, &paddr, &kvstart, &size, file_p);
	if (rc < 0) {
		GMN_PR_ERR("%s: get_pmem_file fd %d error %d\n", __func__, fd,
			rc);
		return 0;
	}

	/* validate user input */
	if (len > size) {
		GMN_PR_ERR("%s: invalid offset + len\n", __func__);
		return 0;
	}

	return paddr;
}

static struct clk *jpeg_clk;
static struct clk *jpeg_pclk;

int msm_gemini_platform_clk_enable(void)
{
	/* MP*fps*(1 + %blanking)
	   2MP: 24MHz  ------ 2 x 10 x 1.2
	   3MP: 36MHz  ------ 3 x 10 x 1.2
	   5MP: 60MHz  ------ 5 x 10 x 1.2
	   8MP: 96MHz  ------ 8 x 10 x 1.2
	  12MP: 144MHz ------12 x 10 x 1.2
	 */
	int rc = -1;
	u32 rate = 144000000;

	if (jpeg_clk  == NULL) {
		jpeg_clk  = clk_get(NULL, "jpeg_clk");
		if (jpeg_clk  == NULL) {
			GMN_PR_ERR("%s:%d] fail rc = %d\n", __func__, __LINE__,
				rc);
			goto fail;
		}
	}

	rc = clk_set_min_rate(jpeg_clk, rate);
	if (rc) {
		GMN_PR_ERR("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
		goto fail;
	}

	rc = clk_enable(jpeg_clk);
	if (rc) {
		GMN_PR_ERR("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
		goto fail;
	}

	if (jpeg_pclk == NULL) {
		jpeg_pclk = clk_get(NULL, "jpeg_pclk");
		if (jpeg_pclk == NULL) {
			GMN_PR_ERR("%s:%d] fail rc = %d\n", __func__, __LINE__,
				rc);
			goto fail;
		}
	}

	rc = clk_enable(jpeg_pclk);
	if (rc) {
		GMN_PR_ERR("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
		goto fail;
	}

	/*rc = pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		"msm_gemini", MSM_SYSTEM_BUS_RATE);
	if (rc) {
		GMN_PR_ERR("request AXI bus QOS fails. rc = %d\n", rc);
		goto fail;
	}*/

GMN_DBG("%s:%d]\n", __func__, __LINE__);
	return rc;

fail:
	GMN_PR_ERR("%s:%d] fail rc = %d\n", __func__, __LINE__, rc);
	return rc;
}

int msm_gemini_platform_clk_disable(void)
{
	clk_disable(jpeg_clk);
	clk_put(jpeg_clk);
	jpeg_clk = NULL;

	clk_disable(jpeg_pclk);
	clk_put(jpeg_pclk);
	jpeg_pclk = NULL;

	//pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ, "msm_gemini");
	return 0;
}

int msm_gemini_platform_init(struct platform_device *pdev,
	struct resource **mem,
	void **base,
	int *irq,
	irqreturn_t (*handler) (int, void *),
	void *context)
{
	int rc = -1;
	int gemini_irq;
	struct resource *gemini_mem, *gemini_io, *gemini_irq_res;
	void *gemini_base;

	gemini_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!gemini_mem) {
		GMN_PR_ERR("%s: no mem resource?\n", __func__);
		return -ENODEV;
	}

	gemini_irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!gemini_irq_res) {
		GMN_PR_ERR("no irq resource?\n");
		return -ENODEV;
	}
	gemini_irq = gemini_irq_res->start;

	gemini_io = request_mem_region(gemini_mem->start,
		resource_size(gemini_mem), pdev->name);
	if (!gemini_io) {
		GMN_PR_ERR("%s: region already claimed\n", __func__);
		return -EBUSY;
	}

	gemini_base = ioremap(gemini_mem->start, resource_size(gemini_mem));
	if (!gemini_base) {
		rc = -ENOMEM;
		GMN_PR_ERR("%s: ioremap failed\n", __func__);
		goto fail1;
	}

	rc = request_irq(gemini_irq, handler, IRQF_TRIGGER_RISING, "gemini",
		context);
	if (rc) {
		GMN_PR_ERR("%s: request_irq failed, %d, JPEG = %d\n", __func__,
			gemini_irq, INT_JPEG);
		goto fail2;
	}

	rc = msm_gemini_platform_clk_enable();
	if (rc) {
		GMN_PR_ERR("%s: clk failed rc = %d\n", __func__, rc);
		goto fail3;
	}

	*mem  = gemini_mem;
	*base = gemini_base;
	*irq  = gemini_irq;
	GMN_DBG("%s:%d] success\n", __func__, __LINE__);

	return rc;

fail3:
	free_irq(gemini_irq, context);
fail2:
	iounmap(gemini_base);
fail1:
	release_mem_region(gemini_mem->start, resource_size(gemini_mem));
	GMN_DBG("%s:%d] fail\n", __func__, __LINE__);
	return rc;
}

int msm_gemini_platform_release(struct resource *mem, void *base, int irq,
	void *context)
{
	int result;

	result = msm_gemini_platform_clk_disable();

	free_irq(irq, context);

	iounmap(base);
	release_mem_region(mem->start, resource_size(mem));

	GMN_DBG("%s:%d] success\n", __func__, __LINE__);
	return result;
}

