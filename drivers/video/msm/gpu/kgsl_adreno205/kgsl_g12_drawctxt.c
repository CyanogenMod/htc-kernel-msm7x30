/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
 * Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
 *
 */
#include <linux/string.h>
#include <linux/types.h>
#include <linux/msm_kgsl.h>

#include "kgsl_g12_drawctxt.h"
#include "kgsl_sharedmem.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_vgv3types.h"
#include "g12_reg.h"

struct kgsl_g12_z1xx g_z1xx = {0};

static void addmarker(struct kgsl_g12_z1xx *z1xx)
{
	if (z1xx) {
		unsigned int *p = z1xx->cmdbuf[z1xx->curr];
		/* todo: use symbolic values */
		p[z1xx->offs++] = 0x7C000176;
		p[z1xx->offs++] = (0x8000 | 5);
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = 0x7C000176;
		p[z1xx->offs++] = 5;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
	}
}

int
kgsl_g12_drawctxt_create(struct kgsl_device *device,
			uint32_t ctxt_id_mask,
			unsigned int *drawctxt_id)
{
	int i;
	int cmd;
	int result;
	unsigned int ctx_id;

	if (g_z1xx.numcontext == 0) {
		for (i = 0; i < GSL_HAL_NUMCMDBUFFERS; i++) {
			int flags = 0;
			if (kgsl_sharedmem_alloc(flags, GSL_HAL_CMDBUFFERSIZE,
					&g_z1xx.cmdbufdesc[i]) !=  0)
				return -ENOMEM;


			g_z1xx.cmdbuf[i] = kzalloc(GSL_HAL_CMDBUFFERSIZE,
						   GFP_KERNEL);


			g_z1xx.curr = i;
			g_z1xx.offs = 0;
			addmarker(&g_z1xx);
			kgsl_sharedmem_write(&g_z1xx.cmdbufdesc[i], 0,
					 g_z1xx.cmdbuf[i],
					 (512 + 13) *
					 sizeof(unsigned int));
		}
		g_z1xx.curr = 0;
		cmd = (int)(((VGV3_NEXTCMD_JUMP) &
			VGV3_NEXTCMD_NEXTCMD_FMASK)
			<< VGV3_NEXTCMD_NEXTCMD_FSHIFT);

		/* set cmd stream buffer to hw */
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_MODE, 4);
		if (result != 0)
			return result;
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_NEXTADDR,
					 g_z1xx.cmdbufdesc[0].physaddr);
		if (result != 0)
			return result;
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_NEXTCMD, cmd | 5);
		if (result != 0)
			return result;

		cmd = (int)(((1) & VGV3_CONTROL_MARKADD_FMASK)
			<< VGV3_CONTROL_MARKADD_FSHIFT);
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_CONTROL, cmd);
		if (result != 0)
			return result;
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_CONTROL, 0);
		if (result != 0)
			return result;
	}
	ctx_id = ffz(ctxt_id_mask);

	g_z1xx.numcontext++;
	if (g_z1xx.numcontext > KGSL_G12_CONTEXT_MAX) {
		*drawctxt_id = 0;
		return KGSL_FAILURE;

	}
	*drawctxt_id = ctx_id;

	return KGSL_SUCCESS;
}

int
kgsl_g12_drawctxt_destroy(struct kgsl_device *device,
			unsigned int drawctxt_id)
{
	if (drawctxt_id >= KGSL_G12_CONTEXT_MAX)
		return KGSL_FAILURE;

	g_z1xx.numcontext--;
	if (g_z1xx.numcontext == 0) {
		int i;
		for (i = 0; i < GSL_HAL_NUMCMDBUFFERS; i++) {
			kgsl_sharedmem_free(&g_z1xx.cmdbufdesc[i]);
			kfree(g_z1xx.cmdbuf[i]);
		}

		memset(&g_z1xx, 0, sizeof(struct kgsl_g12_z1xx));
	}

	if (g_z1xx.numcontext < 0) {
		g_z1xx.numcontext = 0;
		return KGSL_FAILURE;
	}

	return KGSL_SUCCESS;
}
