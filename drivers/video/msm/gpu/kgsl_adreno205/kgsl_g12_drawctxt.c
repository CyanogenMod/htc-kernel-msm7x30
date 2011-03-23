/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/string.h>
#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include <linux/slab.h>

#include "kgsl_g12_drawctxt.h"
#include "kgsl_sharedmem.h"
#include "kgsl.h"
#include "kgsl_g12.h"
#include "kgsl_log.h"
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_vgv3types.h"
#include "g12_reg.h"

int
kgsl_g12_drawctxt_create(struct kgsl_device_private *dev_priv,
			uint32_t unused,
			unsigned int *drawctxt_id)
{
	unsigned int ctx_id;
	struct kgsl_device *device = dev_priv->device;
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;

	if (g12_device->ringbuffer.numcontext >= KGSL_G12_CONTEXT_MAX) {
		*drawctxt_id = 0;
		return KGSL_FAILURE;

	}
	g12_device->ringbuffer.numcontext++;
	ctx_id = ffz(g12_device->ringbuffer.ctxt_id_mask);
	g12_device->ringbuffer.ctxt_id_mask |= 1 << ctx_id;

	*drawctxt_id = ctx_id;

	return KGSL_SUCCESS;
}

int
kgsl_g12_drawctxt_destroy(struct kgsl_device *device,
			unsigned int drawctxt_id)
{
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;
	if (drawctxt_id >= KGSL_G12_CONTEXT_MAX)
		return KGSL_FAILURE;

	if (g12_device->ringbuffer.numcontext == 0)
		return KGSL_FAILURE;

	if ((g12_device->ringbuffer.ctxt_id_mask & (1 << drawctxt_id)) == 0)
		return KGSL_FAILURE;

	if (g12_device->ringbuffer.prevctx == drawctxt_id)
		g12_device->ringbuffer.prevctx = KGSL_G12_INVALID_CONTEXT;

	g12_device->ringbuffer.ctxt_id_mask &= ~(1 << drawctxt_id);
	g12_device->ringbuffer.numcontext--;

	return KGSL_SUCCESS;
}
