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
#include "vcd.h"

#define NORMALIZATION_FACTOR 3600
#define ADJUST_CLIENT_ROUNDS(client, rounds) \
do {\
	if ((client)->n_rounds < round_adjustment) {\
		(client)->n_rounds = 0;\
		VCD_MSG_HIGH("%s(): WARNING: Scheduler list unsorted",\
			__func__);\
	} else\
		(client)->n_rounds -= round_adjustment;\
} while (0)

u32 vcd_sched_create(struct list_head *p_sched_list)
{
	u32 rc = VCD_S_SUCCESS;
	if (!p_sched_list) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else
		INIT_LIST_HEAD(p_sched_list);
	return rc;
}

void vcd_sched_destroy(struct list_head *p_sched_clnt_list)
{
	struct vcd_sched_clnt_ctx_type *p_sched_clnt, *p_sched_clnt_next;
	if (p_sched_clnt_list)
		list_for_each_entry_safe(p_sched_clnt,
			p_sched_clnt_next, p_sched_clnt_list, list) {
			list_del_init(&p_sched_clnt->list);
			p_sched_clnt->b_clnt_active = FALSE;
		}
}

void insert_client_in_list(struct list_head *p_sched_clnt_list,
	struct vcd_sched_clnt_ctx_type *p_sched_new_clnt, bool b_tail)
{
	struct vcd_sched_clnt_ctx_type *p_sched_clnt;
	if (!list_empty(p_sched_clnt_list)) {
		if (b_tail)
			p_sched_clnt = list_entry(p_sched_clnt_list->prev,
				struct vcd_sched_clnt_ctx_type, list);
		else
			p_sched_clnt = list_first_entry(p_sched_clnt_list,
				struct vcd_sched_clnt_ctx_type, list);
		p_sched_new_clnt->n_rounds = p_sched_clnt->n_rounds;
	} else
		p_sched_new_clnt->n_rounds = 0;
	if (b_tail)
		list_add_tail(&p_sched_new_clnt->list, p_sched_clnt_list);
	else
		list_add(&p_sched_new_clnt->list, p_sched_clnt_list);
}

u32 vcd_sched_add_client(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	struct vcd_property_hdr_type prop_hdr;
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt;
	u32 rc = VCD_S_SUCCESS;
	if (!p_cctxt) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (p_cctxt->sched_clnt_hdl)
		VCD_MSG_HIGH(
			"%s(): Scheduler client already exists!", __func__);
	else {
		p_sched_cctxt = (struct vcd_sched_clnt_ctx_type *)
			vcd_malloc(sizeof(struct vcd_sched_clnt_ctx_type));
		if (p_sched_cctxt) {

			prop_hdr.prop_id = DDL_I_FRAME_PROC_UNITS;
			prop_hdr.n_size = sizeof(p_cctxt->n_frm_p_units);
			rc = ddl_get_property(p_cctxt->ddl_handle, &prop_hdr,
						  &p_cctxt->n_frm_p_units);
			VCD_FAILED_RETURN(rc,
				"Failed: Get DDL_I_FRAME_PROC_UNITS");
			if (p_cctxt->b_decoding) {
				p_cctxt->frm_rate.n_fps_numerator =
					VCD_DEC_INITIAL_FRAME_RATE;
				p_cctxt->frm_rate.n_fps_denominator = 1;
			} else {
				prop_hdr.prop_id = VCD_I_FRAME_RATE;
				prop_hdr.n_size = sizeof(p_cctxt->frm_rate);
				rc = ddl_get_property(p_cctxt->ddl_handle,
						&prop_hdr, &p_cctxt->frm_rate);
				VCD_FAILED_RETURN(rc,
					"Failed: Get VCD_I_FRAME_RATE");
			}
			p_cctxt->n_reqd_perf_lvl = p_cctxt->n_frm_p_units *
				p_cctxt->frm_rate.n_fps_numerator /
				p_cctxt->frm_rate.n_fps_denominator;

			p_cctxt->sched_clnt_hdl = p_sched_cctxt;
			memset(p_sched_cctxt, 0,
				sizeof(struct vcd_sched_clnt_ctx_type));
			p_sched_cctxt->n_o_tkns = 0;
			p_sched_cctxt->r_p_frm = NORMALIZATION_FACTOR *
				p_cctxt->frm_rate.n_fps_denominator /
				p_cctxt->frm_rate.n_fps_numerator;
			p_sched_cctxt->b_clnt_active = TRUE;
			p_sched_cctxt->p_clnt_data = p_cctxt;
			INIT_LIST_HEAD(&p_sched_cctxt->ip_frm_list);

			insert_client_in_list(
				&p_cctxt->p_dev_ctxt->sched_clnt_list,
				p_sched_cctxt, false);
		}
	}
	return rc;
}

u32 vcd_sched_remove_client(struct vcd_sched_clnt_ctx_type *p_sched_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_clnt_ctxt_type_t *p_cctxt;
	if (!p_sched_cctxt) {
		VCD_MSG_ERROR("%s(): Invalid handle ptr", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (!list_empty(&p_sched_cctxt->ip_frm_list)) {
		VCD_MSG_ERROR(
			"%s(): Cannot remove client, queue no empty", __func__);
		rc = VCD_ERR_ILLEGAL_OP;
	} else {
		p_cctxt = p_sched_cctxt->p_clnt_data;
		list_del(&p_sched_cctxt->list);
		memset(p_sched_cctxt, 0,
			sizeof(struct vcd_sched_clnt_ctx_type));
		vcd_free(p_sched_cctxt);
	}
	return rc;
}

u32 vcd_sched_update_config(struct vcd_clnt_ctxt_type_t *p_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	if (!p_cctxt || !p_cctxt->sched_clnt_hdl) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else {
		p_cctxt->sched_clnt_hdl->n_rounds /=
			p_cctxt->sched_clnt_hdl->r_p_frm;
		p_cctxt->sched_clnt_hdl->r_p_frm =
			NORMALIZATION_FACTOR *
			p_cctxt->frm_rate.n_fps_denominator /
			p_cctxt->frm_rate.n_fps_numerator;
		p_cctxt->sched_clnt_hdl->n_rounds *=
			p_cctxt->sched_clnt_hdl->r_p_frm;
	}
	return rc;
}

u32 vcd_sched_queue_buffer(
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt,
	struct vcd_buffer_entry_type *p_buffer, u32 b_tail)
{
	u32 rc = VCD_S_SUCCESS;
	if (!p_sched_cctxt || !p_buffer) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (b_tail)
		list_add_tail(&p_buffer->sched_list,
				&p_sched_cctxt->ip_frm_list);
	else
		list_add(&p_buffer->sched_list, &p_sched_cctxt->ip_frm_list);
	return rc;
}

u32 vcd_sched_dequeue_buffer(
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt,
	struct vcd_buffer_entry_type **pp_buffer)
{
	u32 rc = VCD_S_SCHED_QEMPTY;
	if (!p_sched_cctxt || !pp_buffer) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else {
		*pp_buffer = NULL;
		if (!list_empty(&p_sched_cctxt->ip_frm_list)) {
			*pp_buffer = list_first_entry(
					&p_sched_cctxt->ip_frm_list,
					struct vcd_buffer_entry_type,
					sched_list);
			list_del(&(*pp_buffer)->sched_list);
			rc = VCD_S_SUCCESS;
		}
	}
	return rc;
}

u32 vcd_sched_mark_client_eof(struct vcd_sched_clnt_ctx_type *p_sched_cctxt)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_buffer_entry_type *p_buffer = NULL;
	if (!p_sched_cctxt) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (!list_empty(&p_sched_cctxt->ip_frm_list)) {
		p_buffer = list_entry(p_sched_cctxt->ip_frm_list.prev,
			struct vcd_buffer_entry_type, sched_list);
		p_buffer->frame.n_flags |= VCD_FRAME_FLAG_EOS;
	} else
		rc = VCD_S_SCHED_QEMPTY;
	return rc;
}

u32 vcd_sched_suspend_resume_clnt(
	struct vcd_clnt_ctxt_type_t *p_cctxt, u32 b_state)
{
	u32 rc = VCD_S_SUCCESS;
	struct vcd_sched_clnt_ctx_type *p_sched_cctxt;
	if (!p_cctxt || !p_cctxt->sched_clnt_hdl) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else {
		p_sched_cctxt = p_cctxt->sched_clnt_hdl;
		if (b_state != p_sched_cctxt->b_clnt_active) {
			p_sched_cctxt->b_clnt_active = b_state;
			if (b_state)
				insert_client_in_list(&p_cctxt->p_dev_ctxt->\
					sched_clnt_list, p_sched_cctxt, false);
			else
				list_del_init(&p_sched_cctxt->list);
		}
	}
	return rc;
}

u32 vcd_sched_get_client_frame(struct list_head *p_sched_clnt_list,
	struct vcd_clnt_ctxt_type_t **pp_cctxt,
	struct vcd_buffer_entry_type **pp_buffer)
{
	u32 rc = VCD_S_SCHED_QEMPTY, round_adjustment = 0;
	struct vcd_sched_clnt_ctx_type *p_sched_clnt, *p_clnt_nxt;
	if (!p_sched_clnt_list || !pp_cctxt || !pp_buffer) {
		VCD_MSG_ERROR("%s(): Invalid parameter", __func__);
		rc = VCD_ERR_ILLEGAL_PARM;
	} else if (!list_empty(p_sched_clnt_list)) {
		*pp_cctxt = NULL;
		*pp_buffer = NULL;
		list_for_each_entry_safe(p_sched_clnt,
			p_clnt_nxt, p_sched_clnt_list, list) {
			if (&p_sched_clnt->list == p_sched_clnt_list->next)
				round_adjustment = p_sched_clnt->n_rounds;
			if (*pp_cctxt) {
				if ((*pp_cctxt)->sched_clnt_hdl->n_rounds >=
					p_sched_clnt->n_rounds)
					list_move(&(*pp_cctxt)->sched_clnt_hdl\
						->list, &p_sched_clnt->list);
				ADJUST_CLIENT_ROUNDS(p_sched_clnt,
					round_adjustment);
			} else if (p_sched_clnt->n_o_tkns &&
				!list_empty(&p_sched_clnt->ip_frm_list)) {
				*pp_cctxt = p_sched_clnt->p_clnt_data;
				p_sched_clnt->n_rounds += p_sched_clnt->r_p_frm;
			} else
				ADJUST_CLIENT_ROUNDS(p_sched_clnt,
					round_adjustment);
		}
		if (*pp_cctxt) {
			rc = vcd_sched_dequeue_buffer(
				(*pp_cctxt)->sched_clnt_hdl, pp_buffer);
			if (rc == VCD_S_SUCCESS) {
				(*pp_cctxt)->sched_clnt_hdl->n_o_tkns--;
				ADJUST_CLIENT_ROUNDS((*pp_cctxt)->\
					sched_clnt_hdl, round_adjustment);
			}
		}
	}
	return rc;
}
