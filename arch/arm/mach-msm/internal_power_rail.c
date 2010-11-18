/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>

#include <mach/internal_power_rail.h>

#include "proc_comm.h"

static DEFINE_SPINLOCK(power_rail_lock);

static struct internal_rail {
	uint32_t id;
	uint32_t mode;
} rails[] = {
	{ PWR_RAIL_GRP_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_GRP_2D_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_MFC_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_ROTATOR_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_VDC_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_VFE_CLK, PWR_RAIL_CTL_AUTO },
	{ PWR_RAIL_VPE_CLK, PWR_RAIL_CTL_AUTO },
};

static struct internal_rail *find_rail(unsigned rail_id)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(rails); i++)
		if (rails[i].id == rail_id)
			return rails + i;

	return NULL;
}

/* Enable or disable an internal power rail */
int internal_pwr_rail_ctl(unsigned rail_id, bool enable)
{
	int cmd, rc;

	cmd = enable ? 	PCOM_CLKCTL_RPC_RAIL_ENABLE :
			PCOM_CLKCTL_RPC_RAIL_DISABLE;

	rc = msm_proc_comm(cmd, &rail_id, NULL);

	return rc;

}
EXPORT_SYMBOL(internal_pwr_rail_ctl);

/* Enable or disable a rail if the rail is in auto mode. */
int internal_pwr_rail_ctl_auto(unsigned rail_id, bool enable)
{
	int rc = 0;
	unsigned long flags;
	struct internal_rail *rail = find_rail(rail_id);

	BUG_ON(!rail);

	spin_lock_irqsave(&power_rail_lock, flags);
	if (rail->mode == PWR_RAIL_CTL_AUTO)
		rc = internal_pwr_rail_ctl(rail_id, enable);
	spin_unlock_irqrestore(&power_rail_lock, flags);

	return rc;
}

/* Specify an internal power rail control mode (ex. auto, manual) */
int internal_pwr_rail_mode(unsigned rail_id, enum rail_ctl_mode mode)
{
	int rc;
	unsigned long flags;
	struct internal_rail *rail = find_rail(rail_id);

	spin_lock_irqsave(&power_rail_lock, flags);
	rc = msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
	if (rc)
		goto out;
	if (rail_id) {
		rc = -EINVAL;
		goto out;
	}

	if (rail)
		rail->mode = mode;
out:
	spin_unlock_irqrestore(&power_rail_lock, flags);
	return rc;
}
EXPORT_SYMBOL(internal_pwr_rail_mode);

