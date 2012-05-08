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

#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <linux/io.h>
#include "msm_vpe1_7x30.h"
#include <mach/msm_reqs.h>
#include <linux/pm_qos_params.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <mach/clk.h>
#include <asm/div64.h>

static int vpe_update_scaler(struct video_crop_t *pcrop);
static struct vpe_device_type  vpe_device_data;
static struct vpe_device_type  *vpe_device;
struct vpe_ctrl_type    *vpe_ctrl;
char *vpe_general_cmd[] = {
	"VPE_DUMMY_0",  /* 0 */
	"VPE_SET_CLK",
	"VPE_RESET",
	"VPE_START",
	"VPE_ABORT",
	"VPE_OPERATION_MODE_CFG",  /* 5 */
	"VPE_INPUT_PLANE_CFG",
	"VPE_OUTPUT_PLANE_CFG",
	"VPE_INPUT_PLANE_UPDATE",
	"VPE_SCALE_CFG_TYPE",
	"VPE_ROTATION_CFG_TYPE",  /* 10 */
	"VPE_AXI_OUT_CFG",
	"VPE_CMD_DIS_OFFSET_CFG",
};

#define CHECKED_COPY_FROM_USER(in) {					\
	if (copy_from_user((in), (void __user *)cmd->value,		\
			cmd->length)) {					\
		rc = -EFAULT;						\
		break;							\
	}								\
}

#define msm_dequeue_vpe(queue, member) ({			\
	unsigned long flags;					\
	struct msm_device_queue *__q = (queue);			\
	struct msm_queue_cmd *qcmd = 0;				\
	spin_lock_irqsave(&__q->lock, flags);			\
	if (!list_empty(&__q->list)) {				\
		__q->len--;					\
		qcmd = list_first_entry(&__q->list,		\
				struct msm_queue_cmd, member);	\
		list_del_init(&qcmd->member);			\
	}							\
	spin_unlock_irqrestore(&__q->lock, flags);		\
	qcmd;							\
})

/*
static   struct vpe_cmd_type vpe_cmd[] = {
		{VPE_DUMMY_0, 0},
		{VPE_SET_CLK, 0},
		{VPE_RESET, 0},
		{VPE_START, 0},
		{VPE_ABORT, 0},
		{VPE_OPERATION_MODE_CFG, VPE_OPERATION_MODE_CFG_LEN},
		{VPE_INPUT_PLANE_CFG, VPE_INPUT_PLANE_CFG_LEN},
		{VPE_OUTPUT_PLANE_CFG, VPE_OUTPUT_PLANE_CFG_LEN},
		{VPE_INPUT_PLANE_UPDATE, VPE_INPUT_PLANE_UPDATE_LEN},
		{VPE_SCALE_CFG_TYPE, VPE_SCALER_CONFIG_LEN},
		{VPE_ROTATION_CFG_TYPE, 0},
		{VPE_AXI_OUT_CFG, 0},
		{VPE_CMD_DIS_OFFSET_CFG, VPE_DIS_OFFSET_CFG_LEN},
};
*/

static long long vpe_do_div(long long num, long long den)
{
	do_div(num, den);
	return num;
}

static int vpe_start(void)
{
	/*  enable the frame irq, bit 0 = Display list 0 ROI done */
	msm_io_w(1, vpe_device->vpebase + VPE_INTR_ENABLE_OFFSET);
	msm_io_dump(vpe_device->vpebase + 0x10000, 0x250);
	/* this triggers the operation. */
	msm_io_w(1, vpe_device->vpebase + VPE_DL0_START_OFFSET);

	return 0;
}

static int vpe_clock_enable(void)
{
	struct clk *clk = NULL;
	clk = clk_get(NULL, "vpe_clk");
	clk_set_rate(clk, 153600000);
	clk_enable(clk);

	return 0;
}

static int vpe_clock_disable(void)
{
	struct clk *clk = NULL;
	clk = clk_get(NULL, "vpe_clk");
	clk_disable(clk);
	clk_put(clk);

	return 0;
}

void vpe_reset_state_variables(void)
{
	/* initialize local variables for state control, etc.*/
	vpe_ctrl->op_mode = 0;
	vpe_ctrl->state = 0;
	spin_lock_init(&vpe_ctrl->tasklet_lock);
	spin_lock_init(&vpe_ctrl->state_lock);
	INIT_LIST_HEAD(&vpe_ctrl->tasklet_q);
}

static int vpe_reset(void)
{
	uint32_t vpe_version;
	uint32_t rc;

	vpe_reset_state_variables();
	vpe_version = msm_io_r(vpe_device->vpebase + VPE_HW_VERSION_OFFSET);
	CDBG("[CAM] vpe_version = 0x%x\n", vpe_version);

	/* disable all interrupts.*/
	msm_io_w(0, vpe_device->vpebase + VPE_INTR_ENABLE_OFFSET);
	/* clear all pending interrupts*/
	msm_io_w(0x1fffff, vpe_device->vpebase + VPE_INTR_CLEAR_OFFSET);

	/* write sw_reset to reset the core. */
	msm_io_w(0x10, vpe_device->vpebase + VPE_SW_RESET_OFFSET);

	/* then poll the reset bit, it should be self-cleared. */
	while (1) {
		rc =
		msm_io_r(vpe_device->vpebase + VPE_SW_RESET_OFFSET) & 0x10;
		if (rc == 0)
			break;
	}

	/*  at this point, hardware is reset. Then pogram to default
		values. */
	msm_io_w(VPE_AXI_RD_ARB_CONFIG_VALUE,
			vpe_device->vpebase + VPE_AXI_RD_ARB_CONFIG_OFFSET);

	msm_io_w(VPE_CGC_ENABLE_VALUE,
			vpe_device->vpebase + VPE_CGC_EN_OFFSET);

	msm_io_w(1, vpe_device->vpebase + VPE_CMD_MODE_OFFSET);

	msm_io_w(VPE_DEFAULT_OP_MODE_VALUE,
			vpe_device->vpebase + VPE_OP_MODE_OFFSET);

	msm_io_w(VPE_DEFAULT_SCALE_CONFIG,
			vpe_device->vpebase + VPE_SCALE_CONFIG_OFFSET);

	return 0;
}

int msm_vpe_cfg_update(void *pinfo)
{
	uint32_t  rot_flag, rc = 0;
	struct video_crop_t *pcrop = (struct video_crop_t *)pinfo;

	rot_flag = msm_io_r(vpe_device->vpebase +
						VPE_OP_MODE_OFFSET) & 0xE00;
	if (pinfo != NULL) {
		CDBG("[CAM] Crop info in2_w = %d, in2_h = %d "
			"out2_h = %d out2_w = %d \n", pcrop->in2_w,
			pcrop->in2_h,
			pcrop->out2_h, pcrop->out2_w);
		rc = vpe_update_scaler(pcrop);
	}
	CDBG("[CAM] return rc = %d rot_flag = %d\n", rc, rot_flag);
	rc |= rot_flag;

	return rc;
}

void vpe_update_scale_coef(uint32_t *p)
{
	uint32_t i, offset;
	offset = *p;
	for (i = offset; i < (VPE_SCALE_COEFF_NUM + offset); i++) {
		msm_io_w(*(++p), vpe_device->vpebase + VPE_SCALE_COEFF_LSBn(i));
		msm_io_w(*(++p), vpe_device->vpebase + VPE_SCALE_COEFF_MSBn(i));
	}
}

void vpe_input_plane_config(uint32_t *p)
{
	msm_io_w(*p, vpe_device->vpebase + VPE_SRC_FORMAT_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_SRC_UNPACK_PATTERN1_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_SRC_IMAGE_SIZE_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_SRC_YSTRIDE1_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_SRC_SIZE_OFFSET);
	vpe_ctrl->in_h_w = *p;
	msm_io_w(*(++p), vpe_device->vpebase + VPE_SRC_XY_OFFSET);
}

void vpe_output_plane_config(uint32_t *p)
{
	msm_io_w(*p, vpe_device->vpebase + VPE_OUT_FORMAT_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_OUT_PACK_PATTERN1_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_OUT_YSTRIDE1_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_OUT_SIZE_OFFSET);
	msm_io_w(*(++p), vpe_device->vpebase + VPE_OUT_XY_OFFSET);
	vpe_ctrl->pcbcr_dis_offset = *(++p);
}

static int vpe_operation_config(uint32_t *p)
{
	uint32_t  outw, outh, temp;
	msm_io_w(*p, vpe_device->vpebase + VPE_OP_MODE_OFFSET);

	temp = msm_io_r(vpe_device->vpebase + VPE_OUT_SIZE_OFFSET);
	outw = temp & 0xFFF;
	outh = (temp & 0xFFF0000) >> 16;

	if (*p++ & 0xE00) {
		/* rotation enabled. */
		vpe_ctrl->out_w = outh;
		vpe_ctrl->out_h = outw;
	} else {
		vpe_ctrl->out_w = outw;
		vpe_ctrl->out_h = outh;
	}
	vpe_ctrl->dis_en = *p;
	return 0;
}

/* Later we can separate the rotation and scaler calc. If
*  rotation is enabled, simply swap the destination dimension.
*  And then pass the already swapped output size to this
*  function. */
static int vpe_update_scaler(struct video_crop_t *pcrop)
{
	uint32_t out_ROI_width, out_ROI_height;
	uint32_t src_ROI_width, src_ROI_height;

	uint32_t rc = 0;  /* default to no zoom. */
	/*
	* phase_step_x, phase_step_y, phase_init_x and phase_init_y
	* are represented in fixed-point, unsigned 3.29 format
	*/
	uint32_t phase_step_x = 0;
	uint32_t phase_step_y = 0;
	uint32_t phase_init_x = 0;
	uint32_t phase_init_y = 0;

	uint32_t src_roi, src_x, src_y, src_xy, temp;
	uint32_t yscale_filter_sel, xscale_filter_sel;
	uint32_t scale_unit_sel_x, scale_unit_sel_y;
	uint64_t numerator, denominator;

	if ((pcrop->in2_w >= pcrop->out2_w) &&
		(pcrop->in2_h >= pcrop->out2_h)) {
		CDBG("[CAM] =======VPE no zoom needed.\n");

		temp = msm_io_r(vpe_device->vpebase + VPE_OP_MODE_OFFSET)
		& 0xfffffffc;
		msm_io_w(temp, vpe_device->vpebase + VPE_OP_MODE_OFFSET);


		msm_io_w(0, vpe_device->vpebase + VPE_SRC_XY_OFFSET);

		CDBG("[CAM] vpe_ctrl->in_h_w = %d \n", vpe_ctrl->in_h_w);
		msm_io_w(vpe_ctrl->in_h_w , vpe_device->vpebase +
				VPE_SRC_SIZE_OFFSET);

		return rc;
	}
	/* If fall through then scaler is needed.*/

	CDBG("[CAM] ========VPE zoom needed.\n");
	/* assumption is both direction need zoom. this can be
	improved. */
	temp =
		msm_io_r(vpe_device->vpebase + VPE_OP_MODE_OFFSET) | 0x3;
	msm_io_w(temp, vpe_device->vpebase + VPE_OP_MODE_OFFSET);

	src_ROI_width = pcrop->in2_w;
	src_ROI_height = pcrop->in2_h;
	out_ROI_width = pcrop->out2_w;
	out_ROI_height = pcrop->out2_h;

	CDBG("[CAM] src w = 0x%x, h=0x%x, dst w = 0x%x, h =0x%x.\n",
		src_ROI_width, src_ROI_height, out_ROI_width,
		out_ROI_height);
	src_roi = (src_ROI_height << 16) + src_ROI_width;

	msm_io_w(src_roi, vpe_device->vpebase + VPE_SRC_SIZE_OFFSET);

	src_x = (out_ROI_width - src_ROI_width)/2;
	src_y = (out_ROI_height - src_ROI_height)/2;

	CDBG("[CAM] src_x = %d, src_y=%d.\n", src_x, src_y);

	src_xy = src_y*(1<<16) + src_x;
	msm_io_w(src_xy, vpe_device->vpebase +
			VPE_SRC_XY_OFFSET);
	CDBG("[CAM] src_xy = %d, src_roi=%d.\n", src_xy, src_roi);

	/* decide whether to use FIR or M/N for scaling */
	if ((out_ROI_width == 1 && src_ROI_width < 4) ||
		(src_ROI_width < 4 * out_ROI_width - 3))
		scale_unit_sel_x = 0;/* use FIR scalar */
	else
		scale_unit_sel_x = 1;/* use M/N scalar */

	if ((out_ROI_height == 1 && src_ROI_height < 4) ||
		(src_ROI_height < 4 * out_ROI_height - 3))
		scale_unit_sel_y = 0;/* use FIR scalar */
	else
		scale_unit_sel_y = 1;/* use M/N scalar */

	/* calculate phase step for the x direction */

	/* if destination is only 1 pixel wide,
	the value of phase_step_x
	is unimportant. Assigning phase_step_x to
	src ROI width as an arbitrary value. */
	if (out_ROI_width == 1)
		phase_step_x = (uint32_t) ((src_ROI_width) <<
						SCALER_PHASE_BITS);

		/* if using FIR scalar */
	else if (scale_unit_sel_x == 0) {

		/* Calculate the quotient ( src_ROI_width - 1 )
		/ ( out_ROI_width - 1)
		with u3.29 precision. Quotient is rounded up to
		the larger 29th decimal point. */
		numerator = (uint64_t)(src_ROI_width - 1) <<
			SCALER_PHASE_BITS;
		/* never equals to 0 because of the
		"(out_ROI_width == 1 )"*/
		denominator = (uint64_t)(out_ROI_width - 1);
		/* divide and round up to the larger 29th
		decimal point. */
		phase_step_x = (uint32_t) vpe_do_div((numerator +
					denominator - 1), denominator);
	} else if (scale_unit_sel_x == 1) { /* if M/N scalar */
		/* Calculate the quotient ( src_ROI_width ) /
		( out_ROI_width)
		with u3.29 precision. Quotient is rounded down to the
		smaller 29th decimal point. */
		numerator = (uint64_t)(src_ROI_width) <<
			SCALER_PHASE_BITS;
		denominator = (uint64_t)(out_ROI_width);
		phase_step_x =
			(uint32_t) vpe_do_div(numerator, denominator);
	}
	/* calculate phase step for the y direction */

	/* if destination is only 1 pixel wide, the value of
		phase_step_x is unimportant. Assigning phase_step_x
		to src ROI width as an arbitrary value. */
	if (out_ROI_height == 1)
		phase_step_y =
		(uint32_t) ((src_ROI_height) << SCALER_PHASE_BITS);

	/* if FIR scalar */
	else if (scale_unit_sel_y == 0) {
		/* Calculate the quotient ( src_ROI_height - 1 ) /
		( out_ROI_height - 1)
		with u3.29 precision. Quotient is rounded up to the
		larger 29th decimal point. */
		numerator = (uint64_t)(src_ROI_height - 1) <<
			SCALER_PHASE_BITS;
		/* never equals to 0 because of the "
		( out_ROI_height == 1 )" case */
		denominator = (uint64_t)(out_ROI_height - 1);
		/* Quotient is rounded up to the larger
		29th decimal point. */
		phase_step_y =
		(uint32_t) vpe_do_div(
			(numerator + denominator - 1), denominator);
	} else if (scale_unit_sel_y == 1) { /* if M/N scalar */
		/* Calculate the quotient ( src_ROI_height )
		/ ( out_ROI_height)
		with u3.29 precision. Quotient is rounded down
		to the smaller 29th decimal point. */
		numerator = (uint64_t)(src_ROI_height) <<
			SCALER_PHASE_BITS;
		denominator = (uint64_t)(out_ROI_height);
		phase_step_y = (uint32_t) vpe_do_div(
			numerator, denominator);
	}

	/* decide which set of FIR coefficients to use */
	if (phase_step_x > HAL_MDP_PHASE_STEP_2P50)
		xscale_filter_sel = 0;
	else if (phase_step_x > HAL_MDP_PHASE_STEP_1P66)
		xscale_filter_sel = 1;
	else if (phase_step_x > HAL_MDP_PHASE_STEP_1P25)
		xscale_filter_sel = 2;
	else
		xscale_filter_sel = 3;

	if (phase_step_y > HAL_MDP_PHASE_STEP_2P50)
		yscale_filter_sel = 0;
	else if (phase_step_y > HAL_MDP_PHASE_STEP_1P66)
		yscale_filter_sel = 1;
	else if (phase_step_y > HAL_MDP_PHASE_STEP_1P25)
		yscale_filter_sel = 2;
	else
		yscale_filter_sel = 3;

	/* calculate phase init for the x direction */

	/* if using FIR scalar */
	if (scale_unit_sel_x == 0) {
		if (out_ROI_width == 1)
			phase_init_x =
				(uint32_t) ((src_ROI_width - 1) <<
							SCALER_PHASE_BITS);
		else
			phase_init_x = 0;
	} else if (scale_unit_sel_x == 1) /* M over N scalar  */
		phase_init_x = 0;

	/* calculate phase init for the y direction
	if using FIR scalar */
	if (scale_unit_sel_y == 0) {
		if (out_ROI_height == 1)
			phase_init_y =
			(uint32_t) ((src_ROI_height -
						1) << SCALER_PHASE_BITS);
		else
			phase_init_y = 0;
	} else if (scale_unit_sel_y == 1) /* M over N scalar   */
		phase_init_y = 0;

	CDBG("[CAM] phase step x = %d, step y = %d.\n",
		 phase_step_x, phase_step_y);
	CDBG("[CAM] phase init x = %d, init y = %d.\n",
		 phase_init_x, phase_init_y);

	msm_io_w(phase_step_x, vpe_device->vpebase +
			VPE_SCALE_PHASEX_STEP_OFFSET);
	msm_io_w(phase_step_y, vpe_device->vpebase +
			VPE_SCALE_PHASEY_STEP_OFFSET);

	msm_io_w(phase_init_x, vpe_device->vpebase +
			VPE_SCALE_PHASEX_INIT_OFFSET);

	msm_io_w(phase_init_y, vpe_device->vpebase +
			VPE_SCALE_PHASEY_INIT_OFFSET);

	return 1;
}

static int vpe_update_scaler_with_dis(struct video_crop_t *pcrop,
				struct dis_offset_type *dis_offset)
{
	uint32_t out_ROI_width, out_ROI_height;
	uint32_t src_ROI_width, src_ROI_height;

	uint32_t rc = 0;  /* default to no zoom. */
	/*
	* phase_step_x, phase_step_y, phase_init_x and phase_init_y
	* are represented in fixed-point, unsigned 3.29 format
	*/
	uint32_t phase_step_x = 0;
	uint32_t phase_step_y = 0;
	uint32_t phase_init_x = 0;
	uint32_t phase_init_y = 0;

	uint32_t src_roi, temp;
	int32_t  src_x, src_y, src_xy;
	uint32_t yscale_filter_sel, xscale_filter_sel;
	uint32_t scale_unit_sel_x, scale_unit_sel_y;
	uint64_t numerator, denominator;
	int32_t  zoom_dis_x, zoom_dis_y;

	CDBG("[CAM] %s: pcrop->in2_w = %d, pcrop->in2_h = %d\n", __func__,
		 pcrop->in2_w, pcrop->in2_h);
	CDBG("[CAM] %s: pcrop->out2_w = %d, pcrop->out2_h = %d\n", __func__,
		 pcrop->out2_w, pcrop->out2_h);

	if ((pcrop->in2_w >= pcrop->out2_w) &&
		(pcrop->in2_h >= pcrop->out2_h)) {
		CDBG("[CAM] =======VPE no zoom needed, DIS is still enabled. \n");

		temp = msm_io_r(vpe_device->vpebase + VPE_OP_MODE_OFFSET)
		& 0xfffffffc;
		msm_io_w(temp, vpe_device->vpebase + VPE_OP_MODE_OFFSET);

		/* no zoom, use dis offset directly. */
		src_xy = dis_offset->dis_offset_y * (1<<16) +
			dis_offset->dis_offset_x;

		msm_io_w(src_xy, vpe_device->vpebase + VPE_SRC_XY_OFFSET);

		CDBG("[CAM] vpe_ctrl->in_h_w = 0x%x \n", vpe_ctrl->in_h_w);
		msm_io_w(vpe_ctrl->in_h_w, vpe_device->vpebase +
				 VPE_SRC_SIZE_OFFSET);
		return rc;
	}
	/* If fall through then scaler is needed.*/

	CDBG("[CAM] ========VPE zoom needed + DIS enabled.\n");
	/* assumption is both direction need zoom. this can be
	 improved. */
	temp = msm_io_r(vpe_device->vpebase +
					VPE_OP_MODE_OFFSET) | 0x3;
	msm_io_w(temp, vpe_device->vpebase +
			VPE_OP_MODE_OFFSET);
	zoom_dis_x = dis_offset->dis_offset_x *
		pcrop->in2_w / pcrop->out2_w;
	zoom_dis_y = dis_offset->dis_offset_y *
		pcrop->in2_h / pcrop->out2_h;

	src_x = zoom_dis_x + (pcrop->out2_w-pcrop->in2_w)/2;
	src_y = zoom_dis_y + (pcrop->out2_h-pcrop->in2_h)/2;



	out_ROI_width = vpe_ctrl->out_w;
	out_ROI_height = vpe_ctrl->out_h;

	src_ROI_width = out_ROI_width * pcrop->in2_w / pcrop->out2_w;
	src_ROI_height = out_ROI_height * pcrop->in2_h / pcrop->out2_h;

	/* clamp to output size.  This is because along
	processing, we mostly do truncation, therefore
	dis_offset tends to be
	smaller values.  The intention was to make sure that the
	offset does not exceed margin.   But in the case it could
	result src_roi bigger, due to subtract a smaller value. */
	CDBG("[CAM] src w = 0x%x, h=0x%x, dst w = 0x%x, h =0x%x.\n",
		src_ROI_width, src_ROI_height, out_ROI_width,
		out_ROI_height);

	src_roi = (src_ROI_height << 16) + src_ROI_width;

	msm_io_w(src_roi, vpe_device->vpebase + VPE_SRC_SIZE_OFFSET);

	CDBG("[CAM] src_x = %d, src_y=%d.\n", src_x, src_y);

	src_xy = src_y*(1<<16) + src_x;
	msm_io_w(src_xy, vpe_device->vpebase +
			VPE_SRC_XY_OFFSET);
	CDBG("[CAM] src_xy = 0x%x, src_roi=0x%x.\n", src_xy, src_roi);

	/* decide whether to use FIR or M/N for scaling */
	if ((out_ROI_width == 1 && src_ROI_width < 4) ||
		(src_ROI_width < 4 * out_ROI_width - 3))
		scale_unit_sel_x = 0;/* use FIR scalar */
	else
		scale_unit_sel_x = 1;/* use M/N scalar */

	if ((out_ROI_height == 1 && src_ROI_height < 4) ||
		(src_ROI_height < 4 * out_ROI_height - 3))
		scale_unit_sel_y = 0;/* use FIR scalar */
	else
		scale_unit_sel_y = 1;/* use M/N scalar */
	/* calculate phase step for the x direction */

	/* if destination is only 1 pixel wide, the value of
	phase_step_x is unimportant. Assigning phase_step_x
	to src ROI width as an arbitrary value. */
	if (out_ROI_width == 1)
		phase_step_x = (uint32_t) ((src_ROI_width) <<
							SCALER_PHASE_BITS);
	else if (scale_unit_sel_x == 0) { /* if using FIR scalar */
		/* Calculate the quotient ( src_ROI_width - 1 )
		/ ( out_ROI_width - 1)with u3.29 precision.
		Quotient is rounded up to the larger
		29th decimal point. */
		numerator =
			(uint64_t)(src_ROI_width - 1) <<
			SCALER_PHASE_BITS;
		/* never equals to 0 because of the "
		(out_ROI_width == 1 )"*/
		denominator = (uint64_t)(out_ROI_width - 1);
		/* divide and round up to the larger 29th
		decimal point. */
		phase_step_x = (uint32_t) vpe_do_div(
			(numerator + denominator - 1), denominator);
	} else if (scale_unit_sel_x == 1) { /* if M/N scalar */
		/* Calculate the quotient
		( src_ROI_width ) / ( out_ROI_width)
		with u3.29 precision. Quotient is rounded
		down to the smaller 29th decimal point. */
		numerator = (uint64_t)(src_ROI_width) <<
			SCALER_PHASE_BITS;
		denominator = (uint64_t)(out_ROI_width);
		phase_step_x =
			(uint32_t) vpe_do_div(numerator, denominator);
	}
	/* calculate phase step for the y direction */

	/* if destination is only 1 pixel wide, the value of
		phase_step_x is unimportant. Assigning phase_step_x
		to src ROI width as an arbitrary value. */
	if (out_ROI_height == 1)
		phase_step_y =
		(uint32_t) ((src_ROI_height) << SCALER_PHASE_BITS);
	else if (scale_unit_sel_y == 0) { /* if FIR scalar */
		/* Calculate the quotient
		( src_ROI_height - 1 ) / ( out_ROI_height - 1)
		with u3.29 precision. Quotient is rounded up to the
		larger 29th decimal point. */
		numerator = (uint64_t)(src_ROI_height - 1) <<
			SCALER_PHASE_BITS;
		/* never equals to 0 because of the
		"( out_ROI_height == 1 )" case */
		denominator = (uint64_t)(out_ROI_height - 1);
		/* Quotient is rounded up to the larger 29th
		decimal point. */
		phase_step_y =
		(uint32_t) vpe_do_div(
		(numerator + denominator - 1), denominator);
	} else if (scale_unit_sel_y == 1) { /* if M/N scalar */
		/* Calculate the quotient ( src_ROI_height ) / ( out_ROI_height)
		with u3.29 precision. Quotient is rounded down to the smaller
		29th decimal point. */
		numerator = (uint64_t)(src_ROI_height) <<
			SCALER_PHASE_BITS;
		denominator = (uint64_t)(out_ROI_height);
		phase_step_y = (uint32_t) vpe_do_div(
			numerator, denominator);
	}

	/* decide which set of FIR coefficients to use */
	if (phase_step_x > HAL_MDP_PHASE_STEP_2P50)
		xscale_filter_sel = 0;
	else if (phase_step_x > HAL_MDP_PHASE_STEP_1P66)
		xscale_filter_sel = 1;
	else if (phase_step_x > HAL_MDP_PHASE_STEP_1P25)
		xscale_filter_sel = 2;
	else
		xscale_filter_sel = 3;

	if (phase_step_y > HAL_MDP_PHASE_STEP_2P50)
		yscale_filter_sel = 0;
	else if (phase_step_y > HAL_MDP_PHASE_STEP_1P66)
		yscale_filter_sel = 1;
	else if (phase_step_y > HAL_MDP_PHASE_STEP_1P25)
		yscale_filter_sel = 2;
	else
		yscale_filter_sel = 3;

	/* calculate phase init for the x direction */

	/* if using FIR scalar */
	if (scale_unit_sel_x == 0) {
		if (out_ROI_width == 1)
			phase_init_x =
			(uint32_t) ((src_ROI_width - 1) <<
						SCALER_PHASE_BITS);
		else
			phase_init_x = 0;

	} else if (scale_unit_sel_x == 1) /* M over N scalar  */
		phase_init_x = 0;

	/* calculate phase init for the y direction
	if using FIR scalar */
	if (scale_unit_sel_y == 0) {
		if (out_ROI_height == 1)
			phase_init_y =
			(uint32_t) ((src_ROI_height -
						1) << SCALER_PHASE_BITS);
		else
			phase_init_y = 0;

	} else if (scale_unit_sel_y == 1) /* M over N scalar   */
		phase_init_y = 0;

	CDBG("[CAM] phase step x = %d, step y = %d.\n",
		phase_step_x, phase_step_y);
	CDBG("[CAM] phase init x = %d, init y = %d.\n",
		phase_init_x, phase_init_y);

	msm_io_w(phase_step_x, vpe_device->vpebase +
			VPE_SCALE_PHASEX_STEP_OFFSET);

	msm_io_w(phase_step_y, vpe_device->vpebase +
			VPE_SCALE_PHASEY_STEP_OFFSET);

	msm_io_w(phase_init_x, vpe_device->vpebase +
			VPE_SCALE_PHASEX_INIT_OFFSET);

	msm_io_w(phase_init_y, vpe_device->vpebase +
			VPE_SCALE_PHASEY_INIT_OFFSET);

	return 1;
}

void msm_send_frame_to_vpe(uint32_t pyaddr, uint32_t pcbcraddr,
				struct timespec *ts)
{
	uint32_t temp_pyaddr, temp_pcbcraddr;
	CDBG("[CAM] vpe input, pyaddr = 0x%x, pcbcraddr = 0x%x\n",
		pyaddr, pcbcraddr);
	msm_io_w(pyaddr, vpe_device->vpebase + VPE_SRCP0_ADDR_OFFSET);
	msm_io_w(pcbcraddr, vpe_device->vpebase + VPE_SRCP1_ADDR_OFFSET);
	if (vpe_ctrl->state == 1)
		CDBG("[CAM] =====VPE is busy!!!  Wrong!========\n");
	else
		vpe_ctrl->ts = *ts;

	if (vpe_ctrl->dis_en) {
		/* Changing the VPE output CBCR address,
		to make Y/CBCR continuous */
		vpe_ctrl->pcbcr_before_dis = msm_io_r(vpe_device->vpebase +
			VPE_OUTP1_ADDR_OFFSET);
		temp_pyaddr = msm_io_r(vpe_device->vpebase +
			VPE_OUTP0_ADDR_OFFSET);
		temp_pcbcraddr = temp_pyaddr + vpe_ctrl->pcbcr_dis_offset;
		msm_io_w(temp_pcbcraddr, vpe_device->vpebase +
			VPE_OUTP1_ADDR_OFFSET);
	}

	vpe_ctrl->state = 1;
	vpe_start();
}

static int vpe_proc_general(struct msm_vpe_cmd *cmd)
{
	int rc = 0;
	uint32_t *cmdp = NULL;
	struct msm_queue_cmd *qcmd = NULL;
	struct msm_vpe_buf_info *vpe_buf;
	struct msm_sync *sync = (struct msm_sync *)vpe_ctrl->syncdata;
	CDBG("[CAM] vpe_proc_general: cmdID = %s, length = %d\n",
		vpe_general_cmd[cmd->id], cmd->length);
	switch (cmd->id) {
	case VPE_RESET:
	case VPE_ABORT:
		rc = vpe_reset();
		break;
	case VPE_START:
		rc = vpe_start();
		break;

	case VPE_INPUT_PLANE_CFG:
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto vpe_proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto vpe_proc_general_done;
		}
		vpe_input_plane_config(cmdp);
		break;

	case VPE_OPERATION_MODE_CFG:
		CDBG("[CAM] cmd->length = %d \n", cmd->length);
		if (cmd->length != VPE_OPERATION_MODE_CFG_LEN) {
			rc = -EINVAL;
			goto vpe_proc_general_done;
		}
		cmdp = kmalloc(VPE_OPERATION_MODE_CFG_LEN,
					GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto vpe_proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			VPE_OPERATION_MODE_CFG_LEN)) {
			rc = -EFAULT;
			goto vpe_proc_general_done;
		}
		rc = vpe_operation_config(cmdp);
		CDBG("[CAM] rc = %d \n", rc);
		break;

	case VPE_OUTPUT_PLANE_CFG:
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto vpe_proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto vpe_proc_general_done;
		}
		vpe_output_plane_config(cmdp);
		break;

	case VPE_SCALE_CFG_TYPE:
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto vpe_proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto vpe_proc_general_done;
		}
		vpe_update_scale_coef(cmdp);
		break;

	case VPE_CMD_DIS_OFFSET_CFG: {
		struct msm_vfe_resp *vdata;
		/* first get the dis offset and frame id. */
		cmdp = kmalloc(cmd->length, GFP_ATOMIC);
		if (!cmdp) {
			rc = -ENOMEM;
			goto vpe_proc_general_done;
		}
		if (copy_from_user(cmdp,
			(void __user *)(cmd->value),
			cmd->length)) {
			rc = -EFAULT;
			goto vpe_proc_general_done;
		}
		/* get the offset. */
		vpe_ctrl->dis_offset = *(struct dis_offset_type *)cmdp;
		qcmd = msm_dequeue_vpe(&sync->vpe_q, list_vpe_frame);
		if (!qcmd) {
			pr_err("[CAM]%s: no video frame.\n", __func__);
			return -EAGAIN;
		}
		vdata = (struct msm_vfe_resp *)(qcmd->command);
		vpe_buf = &vdata->vpe_bf;
		vpe_update_scaler_with_dis(&(vpe_buf->vpe_crop),
					&(vpe_ctrl->dis_offset));

		msm_send_frame_to_vpe(vpe_buf->y_phy, vpe_buf->cbcr_phy,
						&(vpe_buf->ts));

		if (!qcmd || !atomic_read(&qcmd->on_heap))
			return -EAGAIN;
		if (!atomic_sub_return(1, &qcmd->on_heap))
			kfree(qcmd);
		break;
	}

	default:
		break;
	}
vpe_proc_general_done:
	kfree(cmdp);
	return rc;
}

static void vpe_addr_convert(struct msm_vpe_phy_info *pinfo,
	enum vpe_resp_msg type, void *data, void **ext, int32_t *elen)
{
	uint8_t outid;
	switch (type) {
	case VPE_MSG_OUTPUT_V:
		pinfo->output_id =
			((struct vpe_message *)data)->_u.msgOut.output_id;

		switch (type) {
		case VPE_MSG_OUTPUT_V:
			outid = OUTPUT_TYPE_V;
			CDBG("[CAM] In vpe_addr_convert outid = %d \n", outid);
			break;

		default:
			outid = 0xff;
			break;
		}
		pinfo->output_id = outid;
		pinfo->y_phy =
			((struct vpe_message *)data)->_u.msgOut.yBuffer;
		pinfo->cbcr_phy =
			((struct vpe_message *)data)->_u.msgOut.cbcrBuffer;
		*ext  = vpe_ctrl->extdata;
		*elen = vpe_ctrl->extlen;
		break;

	default:
		break;
	} /* switch */
}

void vpe_proc_ops(uint8_t id, void *msg, size_t len)
{
	struct msm_vpe_resp *rp;

	rp = vpe_ctrl->resp->vpe_alloc(sizeof(struct msm_vpe_resp),
		vpe_ctrl->syncdata, GFP_ATOMIC);
	if (!rp) {
		CDBG("[CAM] rp: cannot allocate buffer\n");
		return;
	}
	CDBG("[CAM] vpe_proc_ops, msgId = %d rp->evt_msg.msg_id = %d \n",
		id, rp->evt_msg.msg_id);
	rp->evt_msg.type   = MSM_CAMERA_MSG;
	rp->evt_msg.msg_id = id;
	rp->evt_msg.len    = len;
	rp->evt_msg.data   = msg;

	switch (rp->evt_msg.msg_id) {
	case MSG_ID_VPE_OUTPUT_V:
		rp->type = VPE_MSG_OUTPUT_V;
		vpe_addr_convert(&(rp->phy), VPE_MSG_OUTPUT_V,
			rp->evt_msg.data, &(rp->extdata),
			&(rp->extlen));
		break;

	default:
		rp->type = VPE_MSG_GENERAL;
		break;
	}
	CDBG("[CAM] %s: time = %ld\n",
			__func__, vpe_ctrl->ts.tv_nsec);
	vpe_ctrl->resp->vpe_resp(rp, MSM_CAM_Q_VPE_MSG,
					vpe_ctrl->syncdata,
					&(vpe_ctrl->ts), GFP_ATOMIC);
}

int vpe_config_axi(struct axidata *ad)
{
	uint32_t p1;
	struct msm_pmem_region *regp1 = NULL;
	CDBG("[CAM] vpe_config_axi:bufnum1 = %d.\n", ad->bufnum1);

	if (ad->bufnum1 != 1)
		return -EINVAL;

	regp1 = &(ad->region[0]);
	/* for video  Y address */
	p1 = (regp1->paddr + regp1->info.y_off);
	msm_io_w(p1, vpe_device->vpebase + VPE_OUTP0_ADDR_OFFSET);
	/* for video  CbCr address */
	p1 = (regp1->paddr + regp1->info.cbcr_off);
	msm_io_w(p1, vpe_device->vpebase + VPE_OUTP1_ADDR_OFFSET);

	return 0;
}

int msm_vpe_config(struct msm_vpe_cfg_cmd *cmd, void *data)
{
	struct msm_vpe_cmd vpecmd;
	int rc = 0;
	if (copy_from_user(&vpecmd,
			(void __user *)(cmd->value),
			sizeof(vpecmd))) {
		pr_err("[CAM]%s %d: copy_from_user failed\n", __func__,
				__LINE__);
		return -EFAULT;
	}
	CDBG("[CAM] %s: cmd_type %d\n", __func__, cmd->cmd_type);
	switch (cmd->cmd_type) {
	case CMD_VPE:
		rc = vpe_proc_general(&vpecmd);
		CDBG("[CAM] rc = %d\n", rc);
		break;

	case CMD_AXI_CFG_VPE: {
		struct axidata *axid;
		uint32_t *axio = NULL;
		axid = data;
		if (!axid)
			return -EFAULT;
		vpe_config_axi(axid);
		kfree(axio);
		break;
	}
	default:
		break;
	}
	return rc;
}

static void vpe_send_outmsg(uint8_t msgid, uint32_t pyaddr,
	uint32_t pcbcraddr)
{
	struct vpe_message msg;
	uint8_t outid;
	msg._d = outid = msgid;
	msg._u.msgOut.output_id   = msgid;
	msg._u.msgOut.yBuffer     = pyaddr;
	msg._u.msgOut.cbcrBuffer  = pcbcraddr;
	vpe_proc_ops(outid, &msg, sizeof(struct vpe_message));
	return;
}

int msm_vpe_reg(struct msm_vpe_callback *presp)
{

	if (presp && presp->vpe_resp)
		vpe_ctrl->resp = presp;
		/*
		CDBG("[CAM] vpe_ctrl->resp = %x \n", vpe_ctrl->resp);
		*/

	return 0;
}

static void vpe_do_tasklet(unsigned long data)
{
	unsigned long flags;
	uint32_t pyaddr, pcbcraddr, src_y, src_cbcr, temp;

	struct vpe_isr_queue_cmd_type *qcmd = NULL;

	CDBG("[CAM] === vpe_do_tasklet start === \n");

	spin_lock_irqsave(&vpe_ctrl->tasklet_lock, flags);
	qcmd = list_first_entry(&vpe_ctrl->tasklet_q,
		struct vpe_isr_queue_cmd_type, list);

	if (!qcmd) {
		spin_unlock_irqrestore(&vpe_ctrl->tasklet_lock, flags);
		return;
	}

	list_del(&qcmd->list);
	spin_unlock_irqrestore(&vpe_ctrl->tasklet_lock, flags);

	/* interrupt to be processed,  *qcmd has the payload.  */
	if (qcmd->irq_status & 0x1) {
		CDBG("[CAM] vpe plane0 frame done.\n");

		pyaddr =
			msm_io_r(vpe_device->vpebase + VPE_OUTP0_ADDR_OFFSET);
		pcbcraddr =
			msm_io_r(vpe_device->vpebase + VPE_OUTP1_ADDR_OFFSET);

		if (vpe_ctrl->dis_en)
			pcbcraddr = vpe_ctrl->pcbcr_before_dis;

		src_y =
			msm_io_r(vpe_device->vpebase + VPE_SRCP0_ADDR_OFFSET);
		src_cbcr =
			msm_io_r(vpe_device->vpebase + VPE_SRCP1_ADDR_OFFSET);

		msm_io_w(src_y,
				vpe_device->vpebase + VPE_OUTP0_ADDR_OFFSET);
		msm_io_w(src_cbcr,
				vpe_device->vpebase + VPE_OUTP1_ADDR_OFFSET);

		temp = msm_io_r(
		vpe_device->vpebase + VPE_OP_MODE_OFFSET) & 0xFFFFFFFC;
		msm_io_w(temp, vpe_device->vpebase + VPE_OP_MODE_OFFSET);
		CDBG("[CAM] vpe send out msg.\n");
		/*  now pass this frame to msm_camera.c. */
		vpe_send_outmsg(MSG_ID_VPE_OUTPUT_V, pyaddr, pcbcraddr);
		vpe_ctrl->state = 0;   /* put it back to idle. */
	}
	kfree(qcmd);
}
DECLARE_TASKLET(vpe_tasklet, vpe_do_tasklet, 0);

static irqreturn_t vpe_parse_irq(int irq_num, void *data)
{
	unsigned long flags;
	uint32_t irq_status = 0;
	struct vpe_isr_queue_cmd_type *qcmd;

	CDBG("[CAM] vpe_parse_irq.\n");
	/* read and clear back-to-back. */
	irq_status = msm_io_r_mb(vpe_device->vpebase +
							VPE_INTR_STATUS_OFFSET);
	msm_io_w_mb(irq_status, vpe_device->vpebase +
				VPE_INTR_CLEAR_OFFSET);

	msm_io_w(0, vpe_device->vpebase + VPE_INTR_ENABLE_OFFSET);

	if (irq_status == 0) {
		pr_err("[CAM] vpe_parse_irq: irq_status = 0"
						"!!!! Something is wrong!\n");
		return IRQ_HANDLED;
	}
	irq_status &= 0x1;
	/* apply mask. only interested in bit 0.  */
	if (irq_status) {
		qcmd = kzalloc(sizeof(struct vpe_isr_queue_cmd_type),
			GFP_ATOMIC);
		if (!qcmd) {
			pr_err("[CAM] vpe_parse_irq: qcmd malloc failed!\n");
			return IRQ_HANDLED;
		}
		/* must be 0x1 now. so in bottom half we don't really
		need to check. */
		qcmd->irq_status = irq_status & 0x1;
		spin_lock_irqsave(&vpe_ctrl->tasklet_lock, flags);
		list_add_tail(&qcmd->list, &vpe_ctrl->tasklet_q);
		spin_unlock_irqrestore(&vpe_ctrl->tasklet_lock, flags);
		tasklet_schedule(&vpe_tasklet);
	}
	return IRQ_HANDLED;
}

static int vpe_enable_irq(void)
{
	uint32_t   rc = 0;
	rc = request_irq(vpe_device->vpeirq,
				vpe_parse_irq,
				IRQF_TRIGGER_HIGH, "vpe", 0);
	return rc;
}

int msm_vpe_open(void)
{
	int rc = 0;

	CDBG("[CAM] %s: In \n", __func__);

	vpe_ctrl = kzalloc(sizeof(struct vpe_ctrl_type), GFP_KERNEL);
	if (!vpe_ctrl) {
		pr_err("[CAM]%s: no memory!\n", __func__);
		return -ENOMEM;
	}
	/* don't change the order of clock and irq.*/
	CDBG("[CAM] %s: enable_clock \n", __func__);
#ifdef CONFIG_MSM_CAMERA_7X30
	rc = vpe_clock_enable();
#else
	rc = msm_camio_vpe_clk_enable();
#endif
	CDBG("[CAM] %s: enable_irq \n", __func__);
	vpe_enable_irq();

	/* initialize the data structure - lock, queue etc. */
	spin_lock_init(&vpe_ctrl->tasklet_lock);
	INIT_LIST_HEAD(&vpe_ctrl->tasklet_q);

	CDBG("[CAM] %s: Out \n", __func__);

	return rc;
}

int msm_vpe_release(void)
{
	/* clean up....*/
	/* drain the queue, etc. */
	/* don't change the order of clock and irq.*/
	int rc = 0;

	pr_info("[CAM] %s: In\n", __func__);

	free_irq(vpe_device->vpeirq, 0);
#ifdef CONFIG_MSM_CAMERA_7X30
	rc = vpe_clock_disable();
#else
	rc = msm_camio_vpe_clk_disable();
#endif
	kfree(vpe_ctrl);

	pr_info("[CAM] %s: Out\n", __func__);

	return 0;
}

static int __msm_vpe_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct resource   *vpemem, *vpeirq, *vpeio;
	void __iomem      *vpebase;

	/* first allocate */

	vpe_device = &vpe_device_data;
	memset(vpe_device, 0, sizeof(struct vpe_device_type));

	/* does the device exist? */
	vpeirq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!vpeirq) {
		pr_err("[CAM]%s: no vpe irq resource.\n", __func__);
		rc = -ENODEV;
		goto vpe_free_device;
	}
	vpemem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!vpemem) {
		pr_err("[CAM]%s: no vpe mem resource!\n", __func__);
		rc = -ENODEV;
		goto vpe_free_device;
	}
	vpeio = request_mem_region(vpemem->start,
			resource_size(vpemem), pdev->name);
	if (!vpeio) {
		pr_err("[CAM]%s: VPE region already claimed.\n", __func__);
		rc = -EBUSY;
		goto vpe_free_device;
	}

	vpebase =
		ioremap(vpemem->start,
				(vpemem->end -vpemem->start) + 1);
	if (!vpebase) {
		pr_err("[CAM]%s: vpe ioremap failed.\n", __func__);
		rc = -ENOMEM;
		goto vpe_release_mem_region;
	}

	/* Fall through, _probe is successful. */
	vpe_device->vpeirq = vpeirq->start;
	vpe_device->vpemem = vpemem;
	vpe_device->vpeio = vpeio;
	vpe_device->vpebase = vpebase;
	return rc;  /* this rc should be zero.*/

	iounmap(vpe_device->vpebase);  /* this path should never occur */

/* from this part it is error handling. */
vpe_release_mem_region:
	release_mem_region(vpemem->start, (vpemem->end -vpemem->start) + 1);
vpe_free_device:
	return rc;  /* this rc should have error code. */
}

static int __msm_vpe_remove(struct platform_device *pdev)
{
	struct resource	*vpemem;
	vpemem = vpe_device->vpemem;

	iounmap(vpe_device->vpebase);
	release_mem_region(vpemem->start,
					(vpemem->end -vpemem->start) + 1);
	return 0;
}

static struct platform_driver msm_vpe_driver = {
	.probe = __msm_vpe_probe,
	.remove = __msm_vpe_remove,
	.driver = {
		.name = "msm_vpe",
		.owner = THIS_MODULE,
	},
};

static int __init msm_vpe_init(void)
{
	return platform_driver_register(&msm_vpe_driver);
}
module_init(msm_vpe_init);

static void __exit msm_vpe_exit(void)
{
	platform_driver_unregister(&msm_vpe_driver);
}
module_exit(msm_vpe_exit);

MODULE_DESCRIPTION("msm vpe 1.0 driver");
MODULE_VERSION("msm vpe driver 1.0");
MODULE_LICENSE("GPL v2");
