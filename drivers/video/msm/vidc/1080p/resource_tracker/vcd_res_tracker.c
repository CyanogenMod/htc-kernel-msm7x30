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
#include <linux/firmware.h>
#include <linux/pm_qos_params.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <mach/internal_power_rail.h>
#include <mach/clk.h>
#include <mach/msm_reqs.h>
#include <linux/interrupt.h>
#include "vidc.h"
#include "vcd_res_tracker.h"
#include "vidc_init.h"

#if ENA_VIDC_CLK_SCALING
static unsigned int vidc_clk_table[3] = {
	48000000, 133330000, 200000000
};
#else
static unsigned int vidc_clk_table[3] = {
	200000000, 200000000, 200000000
};
#endif

static struct res_trk_context resource_context;
static int clock_enabled;

#define VIDC_FW	"vidc_1080p.fw"

unsigned char *vidc_video_codec_fw;
u32 vidc_video_codec_fw_size;

static u32 res_trk_disable_pwr_rail(void)
{
	mutex_lock(&resource_context.lock);
	if (clock_enabled == 1) {
		mutex_unlock(&resource_context.lock);
		return true;
	}
	if (resource_context.clock_enabled) {
		mutex_unlock(&resource_context.lock);
		VCDRES_MSG_LOW("\n Calling CLK disable in Power Down\n");
		res_trk_disable_clocks();
		mutex_lock(&resource_context.lock);
	}
	clk_put(resource_context.vcodec_clk);
	/*TODO: Power rail functions needs to added here*/
	if (!resource_context.rail_enabled) {
		mutex_unlock(&resource_context.lock);
		return false;
	}
	resource_context.rail_enabled = 0;
	mutex_unlock(&resource_context.lock);
	return true;
}

u32 res_trk_enable_clocks(void)
{
	VCDRES_MSG_LOW("\n in res_trk_enable_clocks()");

	mutex_lock(&resource_context.lock);
	if (clock_enabled == 1) {
		mutex_unlock(&resource_context.lock);
		return true;
	}
	if (!resource_context.clock_enabled) {
		VCDRES_MSG_LOW("Enabling IRQ in %s()\n", __func__);
		enable_irq(resource_context.irq_num);

		VCDRES_MSG_LOW("%s(): Enabling the clocks ...\n", __func__);

		if (clk_enable(resource_context.vcodec_clk)) {
			VCDRES_MSG_ERROR("vidc pclk Enable failed\n");
			mutex_unlock(&resource_context.lock);
			return false;
		}
	}
	resource_context.clock_enabled = 1;
	mutex_unlock(&resource_context.lock);
	return true;
}

static u32 res_trk_sel_clk_rate(unsigned long hclk_rate)
{
	mutex_lock(&resource_context.lock);
	if (clock_enabled == 1) {
		mutex_unlock(&resource_context.lock);
		return true;
	}
	if (clk_set_rate(resource_context.vcodec_clk,
		hclk_rate)) {
		VCDRES_MSG_ERROR("vidc hclk set rate failed\n");
		mutex_unlock(&resource_context.lock);
		return false;
	}
	resource_context.vcodec_clk_rate = hclk_rate;
	mutex_unlock(&resource_context.lock);
	return true;
}

static u32 res_trk_get_clk_rate(unsigned long *phclk_rate)
{
	if (!phclk_rate) {
		VCDRES_MSG_ERROR("%s(): phclk_rate is NULL\n", __func__);
		return false;
	}
	mutex_lock(&resource_context.lock);
	*phclk_rate = clk_get_rate(resource_context.vcodec_clk);
	if (!(*phclk_rate)) {
		VCDRES_MSG_ERROR("vidc hclk get rate failed\n");
		mutex_unlock(&resource_context.lock);
		return false;
	}
	mutex_unlock(&resource_context.lock);
	return true;
}

u32 res_trk_disable_clocks(void)
{
	VCDRES_MSG_LOW("in res_trk_disable_clocks()\n");

	mutex_lock(&resource_context.lock);
	if (clock_enabled == 1) {
		mutex_unlock(&resource_context.lock);
		return true;
	}
	if (!resource_context.clock_enabled) {
		mutex_unlock(&resource_context.lock);
		return false;
	}
	VCDRES_MSG_LOW("Disabling IRQ in %s()\n", __func__);
	disable_irq_nosync(resource_context.irq_num);
	VCDRES_MSG_LOW("%s(): Disabling the clocks ...\n", __func__);

	resource_context.clock_enabled = 0;
	clk_disable(resource_context.vcodec_clk);
	mutex_unlock(&resource_context.lock);
	return true;
}

static u32 res_trk_enable_pwr_rail(void)
{
	mutex_lock(&resource_context.lock);
	if (clock_enabled == 1) {
		mutex_unlock(&resource_context.lock);
		return true;
	}
	if (!resource_context.rail_enabled) {
		resource_context.vcodec_clk = clk_get(resource_context.device,
			"vcodec_clk");
		if (IS_ERR(resource_context.vcodec_clk)) {
			VCDRES_MSG_ERROR("%s(): vcodec_clk get failed\n"
				, __func__);
			mutex_unlock(&resource_context.lock);
			return false;
		}
		/*TODO: Set clk_rate to lowest value,Currenlty set to highest
		  value during bringup*/
		if (clk_set_rate(resource_context.vcodec_clk,
			vidc_clk_table[0])) {
			VCDRES_MSG_ERROR("set rate failed in power up\n");
			mutex_unlock(&resource_context.lock);
			return false;
		}
		mutex_unlock(&resource_context.lock);
		res_trk_enable_clocks();
		mutex_lock(&resource_context.lock);
	}
	/*TODO: Power rail functions needs to be added*/
	resource_context.rail_enabled = 1;
	clock_enabled = 1;
	mutex_unlock(&resource_context.lock);
	return true;
}

u32 res_trk_power_up(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_enable");
	VCDRES_MSG_LOW("clk_regime_sel_rail_control");
	VCDRES_MSG_MED("\n res_trk_power_up():: Calling "
		"vidc_enable_pwr_rail()\n");
	return res_trk_enable_pwr_rail();
}

u32 res_trk_power_down(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_disable");
	VCDRES_MSG_MED("\n res_trk_power_down():: Calling "
		"res_trk_disable_pwr_rail()\n");
	return res_trk_disable_pwr_rail();
}

u32 res_trk_get_max_perf_level(u32 *pn_max_perf_lvl)
{
	if (!pn_max_perf_lvl) {
		VCDRES_MSG_ERROR("%s(): pn_max_perf_lvl is NULL\n",
			__func__);
		return false;
	}
	*pn_max_perf_lvl = RESTRK_1080P_MAX_PERF_LEVEL;
	return true;
}

u32 res_trk_set_perf_level(u32 req_perf_lvl, u32 *pn_set_perf_lvl,
	struct vcd_dev_ctxt *dev_ctxt)
{
	u32 vidc_freq = 0;
	if (!pn_set_perf_lvl || !dev_ctxt) {
		VCDRES_MSG_ERROR("%s(): NULL pointer! dev_ctxt(%p)\n",
			__func__, dev_ctxt);
		return false;
	}
	VCDRES_MSG_LOW("%s(), req_perf_lvl = %d", __func__, req_perf_lvl);
	if (req_perf_lvl <= RESTRK_1080P_VGA_PERF_LEVEL) {
		vidc_freq = vidc_clk_table[0];
		*pn_set_perf_lvl = RESTRK_1080P_VGA_PERF_LEVEL;
	} else if (req_perf_lvl <= RESTRK_1080P_720P_PERF_LEVEL) {
		vidc_freq = vidc_clk_table[1];
		*pn_set_perf_lvl = RESTRK_1080P_720P_PERF_LEVEL;
	} else {
		vidc_freq = vidc_clk_table[2];
		*pn_set_perf_lvl = RESTRK_1080P_MAX_PERF_LEVEL;
	}
	resource_context.perf_level = *pn_set_perf_lvl;
	VCDRES_MSG_HIGH("\n VIDC: vidc_freq = %u, req_perf_lvl = %u",
		vidc_freq, req_perf_lvl);
#ifdef USE_RES_TRACKER
    if (req_perf_lvl != RESTRK_1080P_MIN_PERF_LEVEL) {
		VCDRES_MSG_HIGH("\n %s(): Setting vidc freq to %u",
			__func__, vidc_freq);
		if (!res_trk_sel_clk_rate(vidc_freq)) {
			VCDRES_MSG_ERROR("%s(): res_trk_sel_clk_rate FAILED\n",
				__func__);
			*pn_set_perf_lvl = 0;
			return false;
		}
	}
#endif
	VCDRES_MSG_HIGH("%s() set perl level : %d", __func__, *pn_set_perf_lvl);
	return true;
}

u32 res_trk_get_curr_perf_level(u32 *pn_perf_lvl)
{
	unsigned long freq;

	if (!pn_perf_lvl) {
		VCDRES_MSG_ERROR("%s(): pn_perf_lvl is NULL\n",
			__func__);
		return false;
	}
	VCDRES_MSG_LOW("clk_regime_msm_get_clk_freq_hz");
	if (!res_trk_get_clk_rate(&freq)) {
		VCDRES_MSG_ERROR("%s(): res_trk_get_clk_rate FAILED\n",
			__func__);
		*pn_perf_lvl = 0;
		return false;
	}
	*pn_perf_lvl = resource_context.perf_level;
	VCDRES_MSG_MED("%s(): freq = %lu, *pn_perf_lvl = %u", __func__,
		freq, *pn_perf_lvl);
	return true;
}

u32 res_trk_download_firmware(void)
{
	const struct firmware *fw_video = NULL;
	int rc = 0;

	VCDRES_MSG_HIGH("%s(): Request firmware download\n",
		__func__);
	mutex_lock(&resource_context.lock);
	rc = request_firmware(&fw_video, VIDC_FW,
		resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_FW, rc);
		mutex_unlock(&resource_context.lock);
		return false;
	}
	vidc_video_codec_fw = (unsigned char *)fw_video->data;
	vidc_video_codec_fw_size = (u32) fw_video->size;
	mutex_unlock(&resource_context.lock);
	return true;
}

void res_trk_init(struct device *device, u32 irq)
{
	if (resource_context.device || resource_context.irq_num ||
		!device) {
		VCDRES_MSG_ERROR("%s() Resource Tracker Init error\n",
			__func__);
	} else {
		memset(&resource_context, 0, sizeof(resource_context));
		mutex_init(&resource_context.lock);
		resource_context.device = device;
		resource_context.irq_num = irq;
		resource_context.perf_level = 0;
	}
}
