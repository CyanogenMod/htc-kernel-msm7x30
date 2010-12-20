/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include "vidc_type.h"
#include "vcd_power_sm.h"
#include "vcd_core.h"
#include "vcd.h"

u32 vcd_power_event(
	struct vcd_dev_ctxt *dev_ctxt,
     struct vcd_clnt_ctxt *cctxt, u32 event)
{
	u32 rc = VCD_S_SUCCESS;

	VCD_MSG_MED("Device power state = %d", dev_ctxt->pwr_clk_state);
	VCD_MSG_MED("event = 0x%x", event);
	switch (event) {

	case VCD_EVT_PWR_DEV_INIT_BEGIN:
	case VCD_EVT_PWR_DEV_INIT_END:
	case VCD_EVT_PWR_DEV_INIT_FAIL:
	case VCD_EVT_PWR_DEV_TERM_BEGIN:
	case VCD_EVT_PWR_DEV_TERM_END:
	case VCD_EVT_PWR_DEV_TERM_FAIL:
	case VCD_EVT_PWR_DEV_SLEEP_BEGIN:
	case VCD_EVT_PWR_DEV_SLEEP_END:
	case VCD_EVT_PWR_DEV_SET_PERFLVL:
	case VCD_EVT_PWR_DEV_HWTIMEOUT:
		{
			rc = vcd_device_power_event(dev_ctxt, event,
				cctxt);
			break;
		}

	case VCD_EVT_PWR_CLNT_CMD_BEGIN:
	case VCD_EVT_PWR_CLNT_CMD_END:
	case VCD_EVT_PWR_CLNT_CMD_FAIL:
	case VCD_EVT_PWR_CLNT_PAUSE:
	case VCD_EVT_PWR_CLNT_RESUME:
	case VCD_EVT_PWR_CLNT_FIRST_FRAME:
	case VCD_EVT_PWR_CLNT_LAST_FRAME:
	case VCD_EVT_PWR_CLNT_ERRFATAL:
		{
			rc = vcd_client_power_event(dev_ctxt, cctxt, event);
			break;
		}

	}

	if (VCD_FAILED(rc))
		VCD_MSG_ERROR("vcd_power_event: event 0x%x failed", event);


	return rc;

}

u32 vcd_device_power_event(struct vcd_dev_ctxt *dev_ctxt, u32 event,
	struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_ERR_FAIL;
	u32 set_perf_lvl;

	switch (event) {

	case VCD_EVT_PWR_DEV_INIT_BEGIN:
	{
		if (dev_ctxt->pwr_clk_state ==
			VCD_PWRCLK_STATE_OFF) {
			if (res_trk_get_max_perf_level(&dev_ctxt->
				max_perf_lvl)) {
				if (res_trk_power_up()) {
					dev_ctxt->pwr_clk_state =
					VCD_PWRCLK_STATE_ON_NOTCLOCKED;
					dev_ctxt->curr_perf_lvl = 0;
					dev_ctxt->reqd_perf_lvl = 0;
					dev_ctxt->active_clnts = 0;
					dev_ctxt->
						set_perf_lvl_pending = false;
					rc = vcd_enable_clock(dev_ctxt,
						cctxt);
					if (VCD_FAILED(rc)) {
						(void)res_trk_power_down();
						dev_ctxt->pwr_clk_state =
							VCD_PWRCLK_STATE_OFF;
					}
				}
			}
		}

		break;
	}

	case VCD_EVT_PWR_DEV_INIT_END:
	case VCD_EVT_PWR_DEV_TERM_FAIL:
	case VCD_EVT_PWR_DEV_SLEEP_BEGIN:
	case VCD_EVT_PWR_DEV_HWTIMEOUT:
		{
			rc = vcd_gate_clock(dev_ctxt);

			break;
		}

	case VCD_EVT_PWR_DEV_INIT_FAIL:
	case VCD_EVT_PWR_DEV_TERM_END:
		{
			if (dev_ctxt->pwr_clk_state !=
				VCD_PWRCLK_STATE_OFF) {
				(void)vcd_disable_clock(dev_ctxt);
				(void)res_trk_power_down();

				dev_ctxt->pwr_clk_state =
				    VCD_PWRCLK_STATE_OFF;
				dev_ctxt->curr_perf_lvl = 0;
				dev_ctxt->reqd_perf_lvl = 0;
				dev_ctxt->active_clnts = 0;
				dev_ctxt->set_perf_lvl_pending = false;
				rc = VCD_S_SUCCESS;
			}

			break;
		}

	case VCD_EVT_PWR_DEV_TERM_BEGIN:
	case VCD_EVT_PWR_DEV_SLEEP_END:
		{
			rc = vcd_un_gate_clock(dev_ctxt);

			break;
		}

	case VCD_EVT_PWR_DEV_SET_PERFLVL:
		{
			set_perf_lvl =
			    dev_ctxt->reqd_perf_lvl >
			    0 ? dev_ctxt->
			    reqd_perf_lvl : VCD_MIN_PERF_LEVEL;
			rc = vcd_set_perf_level(dev_ctxt, set_perf_lvl);
			break;
		}
	}
	return rc;
}

u32 vcd_client_power_event(
	struct vcd_dev_ctxt *dev_ctxt,
    struct vcd_clnt_ctxt *cctxt, u32 event)
{
	u32 rc = VCD_ERR_FAIL;

	switch (event) {

	case VCD_EVT_PWR_CLNT_CMD_BEGIN:
		{
			rc = vcd_un_gate_clock(dev_ctxt);
			break;
		}

	case VCD_EVT_PWR_CLNT_CMD_END:
		{
			rc = vcd_gate_clock(dev_ctxt);
			break;
		}

	case VCD_EVT_PWR_CLNT_CMD_FAIL:
		{
			if (!vcd_core_is_busy(dev_ctxt))
				rc = vcd_gate_clock(dev_ctxt);

			break;
		}

	case VCD_EVT_PWR_CLNT_PAUSE:
	case VCD_EVT_PWR_CLNT_LAST_FRAME:
	case VCD_EVT_PWR_CLNT_ERRFATAL:
		{
			if (cctxt) {
				rc = VCD_S_SUCCESS;
				if (cctxt->status.req_perf_lvl) {
					dev_ctxt->reqd_perf_lvl -=
						cctxt->reqd_perf_lvl;
					cctxt->status.req_perf_lvl = false;
					rc = vcd_set_perf_level(dev_ctxt,
						dev_ctxt->reqd_perf_lvl);
				}
			}

			break;
		}

	case VCD_EVT_PWR_CLNT_RESUME:
	case VCD_EVT_PWR_CLNT_FIRST_FRAME:
		{
			if (cctxt) {
				rc = VCD_S_SUCCESS;
				if (!cctxt->status.req_perf_lvl) {
					dev_ctxt->reqd_perf_lvl +=
						cctxt->reqd_perf_lvl;
					cctxt->status.req_perf_lvl = true;

					rc = vcd_set_perf_level(dev_ctxt,
						dev_ctxt->reqd_perf_lvl);
				}
			}
			break;
		}
	}

	return rc;
}

u32 vcd_enable_clock(struct vcd_dev_ctxt *dev_ctxt,
	struct vcd_clnt_ctxt *cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	u32 set_perf_lvl;

	if (dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_OFF) {
		VCD_MSG_ERROR("vcd_enable_clock(): Already in state "
			"VCD_PWRCLK_STATE_OFF\n");
		rc = VCD_ERR_FAIL;
	} else if (dev_ctxt->pwr_clk_state ==
		VCD_PWRCLK_STATE_ON_NOTCLOCKED) {
		set_perf_lvl =
				dev_ctxt->reqd_perf_lvl >
				0 ? dev_ctxt->
				reqd_perf_lvl : VCD_MIN_PERF_LEVEL;
		rc = vcd_set_perf_level(dev_ctxt, set_perf_lvl);
		if (!VCD_FAILED(rc)) {
			if (res_trk_enable_clocks()) {
				dev_ctxt->pwr_clk_state =
					VCD_PWRCLK_STATE_ON_CLOCKED;
			}
		} else {
			rc = VCD_ERR_FAIL;
		}

	}

	if (!VCD_FAILED(rc))
		dev_ctxt->active_clnts++;

	return rc;
}

u32 vcd_disable_clock(struct vcd_dev_ctxt *dev_ctxt)
{
	u32 rc = VCD_S_SUCCESS;

	if (dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_OFF) {
		VCD_MSG_ERROR("vcd_disable_clock(): Already in state "
			"VCD_PWRCLK_STATE_OFF\n");
		rc = VCD_ERR_FAIL;
	} else if (dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKED ||
		dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKGATED) {
		dev_ctxt->active_clnts--;

		if (!dev_ctxt->active_clnts) {
			if (!res_trk_disable_clocks())
				rc = VCD_ERR_FAIL;

			dev_ctxt->pwr_clk_state =
			    VCD_PWRCLK_STATE_ON_NOTCLOCKED;
			dev_ctxt->curr_perf_lvl = 0;
		}
	}

	return rc;
}

u32 vcd_set_perf_level(struct vcd_dev_ctxt *dev_ctxt, u32 perf_lvl)
{
	u32 rc = VCD_S_SUCCESS;
	if (!vcd_core_is_busy(dev_ctxt)) {
		if (res_trk_set_perf_level(perf_lvl,
			&dev_ctxt->curr_perf_lvl, dev_ctxt)) {
			dev_ctxt->set_perf_lvl_pending = false;
		} else {
			rc = VCD_ERR_FAIL;
			dev_ctxt->set_perf_lvl_pending = true;
		}

	} else {
		dev_ctxt->set_perf_lvl_pending = true;
	}

	return rc;
}

u32 vcd_update_clnt_perf_lvl(
	struct vcd_clnt_ctxt *cctxt,
     struct vcd_property_frame_rate *fps, u32 frm_p_units)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_dev_ctxt *dev_ctxt = cctxt->dev_ctxt;
	u32 new_perf_lvl;
	new_perf_lvl =
	    frm_p_units * fps->fps_numerator / fps->fps_denominator;
	if (cctxt->status.req_perf_lvl) {
		dev_ctxt->reqd_perf_lvl =
		    dev_ctxt->reqd_perf_lvl - cctxt->reqd_perf_lvl +
		    new_perf_lvl;
		rc = vcd_set_perf_level(cctxt->dev_ctxt,
			dev_ctxt->reqd_perf_lvl);
	}
	cctxt->reqd_perf_lvl = new_perf_lvl;
	return rc;
}

u32 vcd_gate_clock(struct vcd_dev_ctxt *dev_ctxt)
{
	u32 rc = VCD_S_SUCCESS;

	if (dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_OFF ||
		dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_ON_NOTCLOCKED) {
		VCD_MSG_ERROR("%s(): Clk is Off or Not Clked yet\n", __func__);
		return VCD_ERR_FAIL;
	}

	if (dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKGATED)
		return rc;
#if ENA_CLK_GATE
	if (res_trk_disable_clocks())
		dev_ctxt->pwr_clk_state = VCD_PWRCLK_STATE_ON_CLOCKGATED;
	else
		rc = VCD_ERR_FAIL;
#endif
	return rc;
}

u32 vcd_un_gate_clock(struct vcd_dev_ctxt *dev_ctxt)
{
	u32 rc = VCD_S_SUCCESS;
#if ENA_CLK_GATE
	if (dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_OFF ||
		dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_ON_NOTCLOCKED) {
		VCD_MSG_ERROR("%s(): Clk is Off or Not Clked yet\n", __func__);
		return VCD_ERR_FAIL;
	}

	if (dev_ctxt->pwr_clk_state == VCD_PWRCLK_STATE_ON_CLOCKED)
		return rc;

	if (res_trk_enable_clocks())
		dev_ctxt->pwr_clk_state = VCD_PWRCLK_STATE_ON_CLOCKED;
	else
		rc = VCD_ERR_FAIL;
#endif
	return rc;
}

