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
#include "vidc_type.h"
#include "vcd_res_tracker.h"
#include "vidc_init.h"

#define MSM_AXI_QOS_NAME "msm_vidc_reg"

#define QVGA_PERF_LEVEL (300 * 30)
#define VGA_PERF_LEVEL (1200 * 30)
#define WVGA_PERF_LEVEL (1500 * 30)

static unsigned int mfc_clk_freq_table[3] = {
	61440000, 122880000, 170667000
};

#ifndef CONFIG_MSM_NPA_SYSTEM_BUS
static unsigned int axi_clk_freq_table_enc[2] = {
	122880, 192000
};
static unsigned int axi_clk_freq_table_dec[2] = {
	122880, 192000
};
#else
static unsigned int axi_clk_freq_table_enc[2] = {
	MSM_AXI_FLOW_VIDEO_RECORDING_720P,
	MSM_AXI_FLOW_VIDEO_RECORDING_720P
};
static unsigned int axi_clk_freq_table_dec[2] = {
	MSM_AXI_FLOW_VIDEO_PLAYBACK_720P,
	MSM_AXI_FLOW_VIDEO_PLAYBACK_720P
};
#endif

static struct res_trk_context resource_context;

#define VIDC_BOOT_FW			"vidc_720p_command_control.fw"
#define VIDC_MPG4_DEC_FW		"vidc_720p_mp4_dec_mc.fw"
#define VIDC_H263_DEC_FW		"vidc_720p_h263_dec_mc.fw"
#define VIDC_H264_DEC_FW		"vidc_720p_h264_dec_mc.fw"
#define VIDC_MPG4_ENC_FW		"vidc_720p_mp4_enc_mc.fw"
#define VIDC_H264_ENC_FW		"vidc_720p_h264_enc_mc.fw"
#define VIDC_VC1_DEC_FW		"vidc_720p_vc1_dec_mc.fw"

unsigned char *vidc_command_control_fw;
u32 vidc_command_control_fw_size;

unsigned char *vidc_mpg4_dec_fw;
u32 vidc_mpg4_dec_fw_size;

unsigned char *vidc_h263_dec_fw;
u32 vidc_h263_dec_fw_size;

unsigned char *vidc_h264_dec_fw;
u32 vidc_h264_dec_fw_size;

unsigned char *vidc_mpg4_enc_fw;
u32 vidc_mpg4_enc_fw_size;

unsigned char *vidc_h264_enc_fw;
u32 vidc_h264_enc_fw_size;

unsigned char *vidc_vc1_dec_fw;
u32 vidc_vc1_dec_fw_size;

static u32 res_trk_disable_pwr_rail(void)
{
	int rc = -1;
	mutex_lock(&resource_context.lock);

	if (resource_context.clock_enabled) {
		mutex_unlock(&resource_context.lock);
		VCDRES_MSG_LOW("\n Calling CLK disable in Power Down\n");
		res_trk_disable_clocks();
		mutex_lock(&resource_context.lock);
	}

	if (!resource_context.rail_enabled) {
		mutex_unlock(&resource_context.lock);
		return false;
	}

	resource_context.rail_enabled = 0;
	rc = clk_reset(resource_context.pclk, CLK_RESET_ASSERT);
	if (rc) {
		VCDRES_MSG_ERROR("\n clk_reset failed %d\n", rc);
		mutex_unlock(&resource_context.lock);
		return false;
	}
	msleep(20);

	rc = internal_pwr_rail_ctl(PWR_RAIL_MFC_CLK, 0);
	if (rc) {
		VCDRES_MSG_ERROR("\n clk_reset failed %d\n", rc);
		mutex_unlock(&resource_context.lock);
		return false;
	}

	clk_put(resource_context.hclk_div2);
	clk_put(resource_context.hclk);
	clk_put(resource_context.pclk);
	mutex_unlock(&resource_context.lock);

	return true;
}

u32 res_trk_enable_clocks(void)
{
	VCDRES_MSG_LOW("\n in res_trk_enable_clocks()");

	mutex_lock(&resource_context.lock);
	if (!resource_context.clock_enabled) {
		VCDRES_MSG_LOW("Enabling IRQ in %s()\n", __func__);
		enable_irq(resource_context.irq_num);

		VCDRES_MSG_LOW("%s(): Enabling the clocks ...\n", __func__);

		if (clk_enable(resource_context.pclk)) {
			VCDRES_MSG_ERROR("vidc pclk Enable failed\n");

			clk_put(resource_context.hclk);
			clk_put(resource_context.hclk_div2);
			mutex_unlock(&resource_context.lock);
			return false;
		}

		if (clk_enable(resource_context.hclk)) {
			VCDRES_MSG_ERROR("vidc  hclk Enable failed\n");
			clk_put(resource_context.pclk);
			clk_put(resource_context.hclk_div2);
			mutex_unlock(&resource_context.lock);
			return false;
		}

		if (clk_enable(resource_context.hclk_div2)) {
			VCDRES_MSG_ERROR("vidc  hclk Enable failed\n");
			clk_put(resource_context.hclk);
			clk_put(resource_context.pclk);
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
	if (clk_set_rate(resource_context.hclk,
		hclk_rate)) {
		VCDRES_MSG_ERROR("vidc hclk set rate failed\n");
		mutex_unlock(&resource_context.lock);
		return false;
	}
	resource_context.hclk_rate = hclk_rate;
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
	*phclk_rate = clk_get_rate(resource_context.hclk);
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

	if (!resource_context.clock_enabled) {
		mutex_unlock(&resource_context.lock);
		return false;
	}

	VCDRES_MSG_LOW("Disabling IRQ in %s()\n", __func__);
	disable_irq_nosync(resource_context.irq_num);
	VCDRES_MSG_LOW("%s(): Disabling the clocks ...\n", __func__);

	resource_context.clock_enabled = 0;
	clk_disable(resource_context.hclk);
	clk_disable(resource_context.hclk_div2);
	clk_disable(resource_context.pclk);
	mutex_unlock(&resource_context.lock);

	return true;
}

static u32 res_trk_enable_pwr_rail(void)
{
	mutex_lock(&resource_context.lock);
	if (!resource_context.rail_enabled) {
		int rc = -1;
		rc = internal_pwr_rail_mode(PWR_RAIL_MFC_CLK,
			PWR_RAIL_CTL_MANUAL);
		if (rc) {
			VCDRES_MSG_ERROR("%s(): internal_pwr_rail_mode \
					failed %d\n", __func__, rc);
			mutex_unlock(&resource_context.lock);
			return false;
		}
		VCDRES_MSG_LOW("%s(): internal_pwr_rail_mode Success %d\n",
			__func__, rc);

		resource_context.pclk = clk_get(resource_context.device,
			"mfc_pclk");

		if (IS_ERR(resource_context.pclk)) {
			VCDRES_MSG_ERROR("%s(): mfc_pclk get failed\n"
							 , __func__);

			mutex_unlock(&resource_context.lock);
			return false;
		}

		resource_context.hclk = clk_get(resource_context.device,
			"mfc_clk");

		if (IS_ERR(resource_context.hclk)) {
			VCDRES_MSG_ERROR("%s(): mfc_clk get failed\n"
							 , __func__);

			clk_put(resource_context.pclk);
			mutex_unlock(&resource_context.lock);
			return false;
		}

		resource_context.hclk_div2 =
			clk_get(resource_context.device, "mfc_div2_clk");

		if (IS_ERR(resource_context.pclk)) {
			VCDRES_MSG_ERROR("%s(): mfc_div2_clk get failed\n"
							 , __func__);

			clk_put(resource_context.pclk);
			clk_put(resource_context.hclk);
			mutex_unlock(&resource_context.lock);
			return false;
		}

		rc = internal_pwr_rail_ctl(PWR_RAIL_MFC_CLK, 1);
		if (rc) {
			VCDRES_MSG_ERROR("\n internal_pwr_rail_ctl failed %d\n"
							 , rc);
			mutex_unlock(&resource_context.lock);
			return false;
		}
		VCDRES_MSG_LOW("%s(): internal_pwr_rail_ctl Success %d\n"
					   , __func__, rc);
		msleep(20);

		rc = clk_reset(resource_context.pclk, CLK_RESET_DEASSERT);
		if (rc) {
			VCDRES_MSG_ERROR("\n clk_reset failed %d\n", rc);
			mutex_unlock(&resource_context.lock);
			return false;
		}
		msleep(20);
	}
	resource_context.rail_enabled = 1;
	mutex_unlock(&resource_context.lock);
	return true;
}

static u32 res_trk_convert_freq_to_perf_lvl(u64 freq)
{
	u64 perf_lvl;
	u64 temp;

	VCDRES_MSG_MED("\n %s():: freq = %u\n", __func__, (u32)freq);

	if (!freq)
		return 0;

	temp = freq * 1000;
	do_div(temp, VCD_RESTRK_HZ_PER_1000_PERFLVL);
	perf_lvl = (u32)temp;
	VCDRES_MSG_MED("\n %s(): perf_lvl = %u\n", __func__,
		(u32)perf_lvl);

	return (u32)perf_lvl;
}

static u32 res_trk_convert_perf_lvl_to_freq(u64 perf_lvl)
{
	u64 freq, temp;

	VCDRES_MSG_MED("\n %s():: perf_lvl = %u\n", __func__,
		(u32)perf_lvl);
	temp = (perf_lvl * VCD_RESTRK_HZ_PER_1000_PERFLVL) + 999;
	do_div(temp, 1000);
	freq = (u32)temp;
	VCDRES_MSG_MED("\n %s(): freq = %u\n", __func__, (u32)freq);

	return (u32)freq;
}

u32 res_trk_power_up(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_enable");
	VCDRES_MSG_LOW("clk_regime_sel_rail_control");
#ifdef AXI_CLK_SCALING
{
	int rc;
	VCDRES_MSG_MED("\n res_trk_power_up():: "
		"Calling AXI add requirement\n");
	rc = pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		MSM_AXI_QOS_NAME, PM_QOS_DEFAULT_VALUE);
	if (rc < 0)	{
		VCDRES_MSG_ERROR("Request AXI bus QOS fails. rc = %d\n",
			rc);
		return false;
	}
}
#endif

	VCDRES_MSG_MED("\n res_trk_power_up():: Calling "
		"vidc_enable_pwr_rail()\n");
	return res_trk_enable_pwr_rail();
}

u32 res_trk_power_down(void)
{
	VCDRES_MSG_LOW("clk_regime_rail_disable");
#ifdef AXI_CLK_SCALING
	VCDRES_MSG_MED("\n res_trk_power_down()::"
		"Calling AXI remove requirement\n");
	pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ,
		MSM_AXI_QOS_NAME);
#endif
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

	*pn_max_perf_lvl = VCD_RESTRK_MAX_PERF_LEVEL;
	return true;
}

u32 res_trk_set_perf_level(u32 req_perf_lvl, u32 *pn_set_perf_lvl,
	struct vcd_dev_ctxt *dev_ctxt)
{
	struct vcd_clnt_ctxt *cctxt_itr = NULL;
	u32 axi_freq = 0, mfc_freq = 0, calc_mfc_freq = 0;
	int rc = -1;
	u8 enc_clnt_present = false;

	if (!pn_set_perf_lvl || !dev_ctxt) {
		VCDRES_MSG_ERROR("%s(): NULL pointer! dev_ctxt(%p)\n",
			__func__, dev_ctxt);
		return false;
	}

	VCDRES_MSG_LOW("%s(), req_perf_lvl = %d", __func__, req_perf_lvl);
	calc_mfc_freq = res_trk_convert_perf_lvl_to_freq(
		(u64)req_perf_lvl);

	if (calc_mfc_freq < VCD_RESTRK_MIN_FREQ_POINT)
		calc_mfc_freq = VCD_RESTRK_MIN_FREQ_POINT;
	else if (calc_mfc_freq > VCD_RESTRK_MAX_FREQ_POINT)
		calc_mfc_freq = VCD_RESTRK_MAX_FREQ_POINT;

	cctxt_itr = dev_ctxt->cctxt_list_head;
	while (cctxt_itr) {
		VCDRES_MSG_LOW("\n cctxt_itr = %p", cctxt_itr);
		if (!cctxt_itr->decoding) {
				VCDRES_MSG_LOW("\n Encoder client");
				enc_clnt_present = true;
				break;
		} else {
				VCDRES_MSG_LOW("\n Decoder client");
		}
		cctxt_itr = cctxt_itr->next;
	}

	if (enc_clnt_present) {
		if (req_perf_lvl >= VGA_PERF_LEVEL) {
			mfc_freq = mfc_clk_freq_table[2];
			axi_freq = axi_clk_freq_table_enc[1];
		} else {
			mfc_freq = mfc_clk_freq_table[0];
			axi_freq = axi_clk_freq_table_enc[0];
		}
		VCDRES_MSG_HIGH("\n ENCODER: axi_freq = %u"
			", mfc_freq = %u, calc_mfc_freq = %u,"
			" req_perf_lvl = %u", axi_freq,
			mfc_freq, calc_mfc_freq,
			req_perf_lvl);
	} else {
		if (req_perf_lvl <= QVGA_PERF_LEVEL) {
			mfc_freq = mfc_clk_freq_table[0];
			axi_freq = axi_clk_freq_table_dec[0];
		} else {
			axi_freq = axi_clk_freq_table_dec[0];
			if (req_perf_lvl <= VGA_PERF_LEVEL)
				mfc_freq = mfc_clk_freq_table[0];
			else if (req_perf_lvl <= WVGA_PERF_LEVEL)
				mfc_freq = mfc_clk_freq_table[1];
			else {
				mfc_freq = mfc_clk_freq_table[2];
				axi_freq = axi_clk_freq_table_dec[1];
			}
		}
		VCDRES_MSG_HIGH("\n DECODER: axi_freq = %u"
			", mfc_freq = %u, calc_mfc_freq = %u,"
			" req_perf_lvl = %u", axi_freq,
			mfc_freq, calc_mfc_freq,
			req_perf_lvl);
	}

#ifdef AXI_CLK_SCALING
    if (req_perf_lvl != VCD_RESTRK_MIN_PERF_LEVEL) {
		VCDRES_MSG_HIGH("\n %s(): Setting AXI freq to %u",
			__func__, axi_freq);
		rc = pm_qos_update_requirement(PM_QOS_SYSTEM_BUS_FREQ,
			MSM_AXI_QOS_NAME, axi_freq);

		if (rc < 0)	{
			VCDRES_MSG_ERROR("\n Update AXI bus QOS fails,"
				"rc = %d\n", rc);
			return false;
		}
	}
#endif

#ifdef USE_RES_TRACKER
    if (req_perf_lvl != VCD_RESTRK_MIN_PERF_LEVEL) {
		VCDRES_MSG_HIGH("\n %s(): Setting MFC freq to %u",
			__func__, mfc_freq);
		if (!res_trk_sel_clk_rate(mfc_freq)) {
			VCDRES_MSG_ERROR("%s(): res_trk_sel_clk_rate FAILED\n",
				__func__);
			*pn_set_perf_lvl = 0;
			return false;
		}
	}
#endif

	*pn_set_perf_lvl =
	    res_trk_convert_freq_to_perf_lvl((u64) mfc_freq);
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

	*pn_perf_lvl = res_trk_convert_freq_to_perf_lvl((u64) freq);
	VCDRES_MSG_MED("%s(): freq = %lu, *pn_perf_lvl = %u", __func__,
		freq, *pn_perf_lvl);
	return true;
}

u32 res_trk_download_firmware(void)
{
	const struct firmware *fw_boot = NULL;
	const struct firmware *fw_mpg4_dec = NULL;
	const struct firmware *fw_h263_dec = NULL;
	const struct firmware *fw_h264_dec = NULL;
	const struct firmware *fw_mpg4_enc = NULL;
	const struct firmware *fw_h264_enc = NULL;
	const struct firmware *fw_vc1_dec = NULL;
	int rc = 0;
	u32 status = true;

	VCDRES_MSG_HIGH("%s(): Request firmware download\n",
		__func__);
	mutex_lock(&resource_context.lock);
	rc = request_firmware(&fw_boot, VIDC_BOOT_FW,
						  resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_BOOT_FW, rc);
		mutex_unlock(&resource_context.lock);
		return false;
	}
	vidc_command_control_fw = (unsigned char *)fw_boot->data;
	vidc_command_control_fw_size = (u32) fw_boot->size;

	rc = request_firmware(&fw_mpg4_dec, VIDC_MPG4_DEC_FW,
						  resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_MPG4_DEC_FW, rc);
		status = false;
		goto boot_fw_free;
	}
	vidc_mpg4_dec_fw = (unsigned char *)fw_mpg4_dec->data;
	vidc_mpg4_dec_fw_size = (u32) fw_mpg4_dec->size;


	rc = request_firmware(&fw_h263_dec, VIDC_H263_DEC_FW,
						  resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_H263_DEC_FW, rc);
		status = false;
		goto mp4dec_fw_free;
	}
	vidc_h263_dec_fw = (unsigned char *)fw_h263_dec->data;
	vidc_h263_dec_fw_size = (u32) fw_h263_dec->size;

	rc = request_firmware(&fw_h264_dec, VIDC_H264_DEC_FW,
						  resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_H264_DEC_FW, rc);
		status = false;
		goto h263dec_fw_free;
	}
	vidc_h264_dec_fw = (unsigned char *)fw_h264_dec->data;
	vidc_h264_dec_fw_size = (u32) fw_h264_dec->size;

	rc = request_firmware(&fw_mpg4_enc, VIDC_MPG4_ENC_FW,
						  resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_MPG4_ENC_FW, rc);
		status = false;
		goto h264dec_fw_free;
	}
	vidc_mpg4_enc_fw = (unsigned char *)fw_mpg4_enc->data;
	vidc_mpg4_enc_fw_size = (u32) fw_mpg4_enc->size;

	rc = request_firmware(&fw_h264_enc, VIDC_H264_ENC_FW,
						  resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_H264_ENC_FW, rc);
		status = false;
		goto mp4enc_fw_free;
	}
	vidc_h264_enc_fw = (unsigned char *)fw_h264_enc->data;
	vidc_h264_enc_fw_size = (u32) fw_h264_enc->size;

	rc = request_firmware(&fw_vc1_dec, VIDC_VC1_DEC_FW,
						  resource_context.device);
	if (rc) {
		VCDRES_MSG_ERROR("request_firmware for %s error %d\n",
				VIDC_VC1_DEC_FW, rc);
		status = false;
		goto h264enc_fw_free;
	}
	vidc_vc1_dec_fw = (unsigned char *)fw_vc1_dec->data;
	vidc_vc1_dec_fw_size = (u32) fw_vc1_dec->size;
	mutex_unlock(&resource_context.lock);
	return status;

h264enc_fw_free:
	release_firmware(fw_h264_enc);
mp4enc_fw_free:
	release_firmware(fw_mpg4_enc);
h264dec_fw_free:
	release_firmware(fw_h264_dec);
h263dec_fw_free:
	release_firmware(fw_h263_dec);
mp4dec_fw_free:
	release_firmware(fw_mpg4_dec);
boot_fw_free:
	release_firmware(fw_boot);
	mutex_unlock(&resource_context.lock);
	return false;
}

void res_trk_init(struct device *device, u32 irq)
{
	if (resource_context.device || resource_context.irq_num ||
		!device) {
		VCDRES_MSG_ERROR("%s() Resource Tracker Init error\n",
				__func__);
		return;
	}
	memset(&resource_context, 0, sizeof(resource_context));
	mutex_init(&resource_context.lock);
	resource_context.device = device;
	resource_context.irq_num = irq;
}
