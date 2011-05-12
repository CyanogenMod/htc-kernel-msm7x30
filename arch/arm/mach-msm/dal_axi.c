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

#include <mach/dal_axi.h>

/* The AXI device ID */
#define DALDEVICEID_AXI   0x02000053
#define DALRPC_PORT_NAME  "SMD_DAL00"

enum {
	DALRPC_AXI_CONFIGURE_BRIDGE = DALDEVICE_FIRST_DEVICE_API_IDX + 11
};

enum {
	DAL_AXI_BRIDGE_CFG_CGR_SS_2DGRP_SYNC_MODE = 14,
	DAL_AXI_BRIDGE_CFG_CGR_SS_2DGRP_ASYNC_MODE,
	DAL_AXI_BRIDGE_CFG_CGR_SS_2DGRP_ISOSYNC_MODE,
	DAL_AXI_BRIDGE_CFG_CGR_SS_2DGRP_DEBUG_EN,
	DAL_AXI_BRIDGE_CFG_CGR_SS_2DGRP_DEBUG_DIS,
	DAL_AXI_BRIDGE_CFG_CGR_SS_3DGRP_SYNC_MODE,
	DAL_AXI_BRIDGE_CFG_CGR_SS_3DGRP_ASYNC_MODE,
	DAL_AXI_BRIDGE_CFG_CGR_SS_3DGRP_ISOSYNC_MODE,
	DAL_AXI_BRIDGE_CFG_CGR_SS_3DGRP_DEBUG_EN,
	DAL_AXI_BRIDGE_CFG_CGR_SS_3DGRP_DEBUG_DIS,
};

static int axi_configure_bridge_grfx_sync_mode(int bridge_mode)
{
	int rc;
	void *dev_handle;

	/* get device handle */
	rc = daldevice_attach(
		DALDEVICEID_AXI, DALRPC_PORT_NAME,
		DALRPC_DEST_MODEM, &dev_handle
	);
	if (rc) {
		printk(KERN_ERR "%s: failed to attach AXI bus device (%d)\n",
			__func__, rc);
		goto fail_dal_attach_detach;
	}

	/* call ConfigureBridge */
	rc = dalrpc_fcn_0(
		DALRPC_AXI_CONFIGURE_BRIDGE, dev_handle,
		bridge_mode
	);
	if (rc) {
		printk(KERN_ERR "%s: AXI bus device (%d) failed to be configured\n",
			__func__, rc);
		goto fail_dal_fcn_0;
	}

	/* close device handle */
	rc = daldevice_detach(dev_handle);
	if (rc) {
		printk(KERN_ERR "%s: failed to detach AXI bus device (%d)\n",
			__func__, rc);
		goto fail_dal_attach_detach;
	}

	return 0;

fail_dal_fcn_0:
	(void)daldevice_detach(dev_handle);
fail_dal_attach_detach:

	return rc;
}



int set_grp2d_async(void)
{
	return axi_configure_bridge_grfx_sync_mode(
		DAL_AXI_BRIDGE_CFG_CGR_SS_2DGRP_ASYNC_MODE);
}

int set_grp3d_async(void)
{
	return axi_configure_bridge_grfx_sync_mode(
		DAL_AXI_BRIDGE_CFG_CGR_SS_3DGRP_ASYNC_MODE);
}
