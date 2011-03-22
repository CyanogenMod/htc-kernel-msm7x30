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

#include "vcd_ddl.h"
#include "vcd_ddl_shared_mem.h"
#include <linux/delay.h>

static void ddl_decoder_input_done_callback(
	struct ddl_client_context *ddl, u32 frame_transact_end);
static u32 ddl_decoder_ouput_done_callback(
	struct ddl_client_context *ddl, u32 frame_transact_end);
static u32 ddl_get_decoded_frame(struct vcd_frame_data  *frame,
	enum vidc_1080p_decode_frame frame_type);
static u32 ddl_get_encoded_frame(struct vcd_frame_data *frame,
	enum vcd_codec codec,
	enum vidc_1080p_encode_frame frame_type);
static void ddl_get_dec_profile_level(struct ddl_decoder_data *decoder,
	u32 profile, u32 level);
static void ddl_handle_enc_frame_done(struct ddl_client_context *ddl);

static void ddl_fw_status_done_callback(struct ddl_context *ddl_context)
{
	DDL_MSG_MED("ddl_fw_status_done_callback");
	if (!DDLCOMMAND_STATE_IS(ddl_context, DDL_CMD_DMA_INIT)) {
		DDL_MSG_ERROR("UNKWN_DMADONE");
	} else {
		DDL_MSG_LOW("FW_STATUS_DONE");
		vidc_1080p_set_host2risc_cmd(VIDC_1080P_HOST2RISC_CMD_SYS_INIT,
			ddl_context->fw_ctxt_memory_size, 0, 0, 0);
	}
}

static void ddl_sys_init_done_callback(struct ddl_context *ddl_context,
	u32 fw_size)
{
	u32 vcd_status = VCD_S_SUCCESS;

	DDL_MSG_MED("ddl_sys_init_done_callback");
	if (!DDLCOMMAND_STATE_IS(ddl_context, DDL_CMD_DMA_INIT)) {
		DDL_MSG_ERROR("UNKNOWN_SYS_INIT_DONE");
	} else {
		ddl_context->cmd_state = DDL_CMD_INVALID;
		DDL_MSG_LOW("SYS_INIT_DONE");
		vidc_1080p_get_fw_version(&ddl_context->fw_version);
		if (ddl_context->fw_memory_size >= fw_size) {
			ddl_context->device_state = DDL_DEVICE_INITED;
			vcd_status = VCD_S_SUCCESS;
		} else
			vcd_status = VCD_ERR_FAIL;
		ddl_context->ddl_callback(VCD_EVT_RESP_DEVICE_INIT,
			vcd_status, NULL, 0, NULL,
			ddl_context->client_data);
		DDL_IDLE(ddl_context);
	}
}

static void ddl_decoder_eos_done_callback(
	struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;

	if (!ddl->decoding) {
		DDL_MSG_ERROR("STATE-CRITICAL-EOSDONE");
		ddl_client_fatal_cb(ddl);
	} else {
		ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME;
		DDL_MSG_LOW("EOS_DONE");
		ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE,
			VCD_S_SUCCESS, NULL, 0, (u32 *)ddl,
			ddl->client_data);
		ddl_release_command_channel(ddl_context,
			ddl->command_channel);
	}
}

static u32 ddl_channel_set_callback(struct ddl_context *ddl_context,
	u32 instance_id)
{
	struct ddl_client_context *ddl;
	u32 ret = false;

	DDL_MSG_MED("ddl_channel_open_callback");
	ddl = ddl_get_current_ddl_client_for_command(ddl_context,
			DDL_CMD_CHANNEL_SET);
	if (ddl) {
		ddl->cmd_state = DDL_CMD_INVALID;
		if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_CHDONE)) {
			DDL_MSG_ERROR("STATE-CRITICAL-CHSET");
			ddl_release_command_channel(ddl_context,
			ddl->command_channel);
		} else {
			DDL_MSG_LOW("CH_SET_DONE");
			DDL_MSG_LOW("ddl_state_transition: %s ~~>\
				DDL_CLIENT_WAIT_FOR_INITCODEC",
				ddl_get_state_string(ddl->client_state));
			ddl->client_state = DDL_CLIENT_WAIT_FOR_INITCODEC;
			ddl->channel_id = instance_id;
			if (ddl->decoding) {
				if (ddl->codec_data.decoder.header_in_start)
					ddl_vidc_decode_init_codec(ddl);
				else {
					ddl_context->ddl_callback(
						VCD_EVT_RESP_START,
						VCD_S_SUCCESS, NULL, 0,
						(u32 *)ddl,
						ddl->client_data);
					ddl_release_command_channel(
						ddl_context,
						ddl->command_channel);
					ret = true;
				}
			} else
				ddl_vidc_encode_init_codec(ddl);
		}
	}
	return ret;
}

static u32 ddl_encoder_seq_done_callback(struct ddl_context *ddl_context,
	struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder;

	DDL_MSG_MED("ddl_encoder_seq_done_callback");
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-INITCODEC");
		ddl_client_fatal_cb(ddl);
		return true;
	}
	ddl->cmd_state = DDL_CMD_INVALID;
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_FRAME",
	ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME;
	DDL_MSG_LOW("INIT_CODEC_DONE");
	encoder = &ddl->codec_data.encoder;
	vidc_1080p_get_encoder_sequence_header_size(
		&encoder->seq_header_length);
	ddl_context->ddl_callback(VCD_EVT_RESP_START, VCD_S_SUCCESS,
		NULL, 0, (u32 *) ddl, ddl->client_data);
	ddl_release_command_channel(ddl_context,
		ddl->command_channel);
	return true;
}

static u32 ddl_decoder_seq_done_callback(struct ddl_context *ddl_context,
	struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	struct vidc_1080p_seq_hdr_info seq_hdr_info;
	u32 process_further = true;

	DDL_MSG_MED("ddl_decoder_seq_done_callback");
	if (!ddl->decoding ||
		!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_INITCODECDONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-HDDONE");
		ddl_client_fatal_cb(ddl);
	} else {
		ddl->cmd_state = DDL_CMD_INVALID;
		DDL_MSG_LOW("ddl_state_transition: %s ~~>\
			DDL_CLIENT_WAIT_FOR_DPB",\
			ddl_get_state_string(ddl->client_state));
		ddl->client_state = DDL_CLIENT_WAIT_FOR_DPB;
		DDL_MSG_LOW("HEADER_DONE");
		vidc_1080p_get_decode_seq_start_result(&seq_hdr_info);
		decoder->frame_size.width = seq_hdr_info.img_size_x;
		decoder->frame_size.height = seq_hdr_info.img_size_y;
		decoder->min_dpb_num = seq_hdr_info.min_num_dpb;
		vidc_sm_get_min_yc_dpb_sizes(
			&ddl->shared_mem[ddl->command_channel],
			&seq_hdr_info.min_luma_dpb_size,
			&seq_hdr_info.min_chroma_dpb_size);
		decoder->y_cb_cr_size = seq_hdr_info.min_luma_dpb_size +
			seq_hdr_info.min_chroma_dpb_size;
		decoder->dpb_buf_size.size_yuv = decoder->y_cb_cr_size;
		decoder->dpb_buf_size.size_y =
			seq_hdr_info.min_luma_dpb_size;
		decoder->dpb_buf_size.size_c =
			seq_hdr_info.min_chroma_dpb_size;
		decoder->progressive_only = 1 - seq_hdr_info.progressive;
		if (!seq_hdr_info.img_size_x || !seq_hdr_info.img_size_y) {
			DDL_MSG_ERROR("FATAL:ZeroImageSize");
			ddl_client_fatal_cb(ddl);
			return process_further;
		}
		vidc_sm_get_profile_info(&ddl->shared_mem
			[ddl->command_channel],
			&seq_hdr_info.profile, &seq_hdr_info.level);
		ddl_get_dec_profile_level(decoder, seq_hdr_info.profile,
			seq_hdr_info.level);
		ddl_calculate_stride(&decoder->frame_size,
			!decoder->progressive_only);
		vidc_sm_get_crop_info(
			&ddl->shared_mem[ddl->command_channel],
			&seq_hdr_info.crop_left_offset,
			&seq_hdr_info.crop_right_offset,
			&seq_hdr_info.crop_top_offset,
			&seq_hdr_info.crop_bottom_offset);
		seq_hdr_info.crop_exists = (seq_hdr_info.crop_left_offset ||
			seq_hdr_info.crop_right_offset ||
			seq_hdr_info.crop_top_offset ||
			seq_hdr_info.crop_bottom_offset);
		if (seq_hdr_info.crop_exists) {
			decoder->frame_size.width -=
				seq_hdr_info.crop_right_offset +
				seq_hdr_info.crop_left_offset;
			decoder->frame_size.height -=
				seq_hdr_info.crop_top_offset +
				seq_hdr_info.crop_bottom_offset;
		}
		ddl_set_default_decoder_buffer_req(decoder, false);
		if (decoder->header_in_start) {
			decoder->client_frame_size = decoder->frame_size;
			decoder->client_output_buf_req =
				decoder->actual_output_buf_req;
			if ((decoder->frame_size.width *
				decoder->frame_size.height) >=
				 VCD_DDL_WVGA_BUF_SIZE) {
				if ((decoder->actual_output_buf_req.\
					actual_count + 2) < 10)
					decoder->client_output_buf_req.\
						actual_count = 10;
				else
					decoder->client_output_buf_req.\
						actual_count += 2;
			} else
				decoder->client_output_buf_req.\
					actual_count = decoder->\
					actual_output_buf_req.\
					actual_count + 5;
			decoder->client_input_buf_req =
				decoder->actual_input_buf_req;
			ddl_context->ddl_callback(VCD_EVT_RESP_START,
				VCD_S_SUCCESS, NULL, 0, (u32 *) ddl,
				ddl->client_data);
			ddl_release_command_channel(ddl_context,
				ddl->command_channel);
		} else {
			u32 seq_hdr_only_frame = false;
			u32 need_reconfig = true;
			struct vcd_frame_data *input_vcd_frm =
				&ddl->input_frame.vcd_frm;

			if ((decoder->frame_size.width ==
				decoder->client_frame_size.width) &&
				(decoder->frame_size.height ==
				decoder->client_frame_size.height) &&
				(decoder->actual_output_buf_req.sz <=
				decoder->client_output_buf_req.sz) &&
				(decoder->actual_output_buf_req.actual_count <=
				 decoder->client_output_buf_req.actual_count) &&
				(decoder->frame_size.scan_lines ==
				decoder->client_frame_size.scan_lines) &&
				(decoder->frame_size.stride ==
				 decoder->client_frame_size.stride))
					need_reconfig = false;
			if (((input_vcd_frm->flags &
				VCD_FRAME_FLAG_CODECCONFIG) &&
				(!(input_vcd_frm->flags &
				VCD_FRAME_FLAG_SYNCFRAME))) ||
				input_vcd_frm->data_len ==
				seq_hdr_info.dec_frm_size) {
				seq_hdr_only_frame = true;
				input_vcd_frm->offset +=
					seq_hdr_info.dec_frm_size;
				input_vcd_frm->data_len -=
					seq_hdr_info.dec_frm_size;
				input_vcd_frm->flags |=
					VCD_FRAME_FLAG_CODECCONFIG;
				ddl->input_frame.frm_trans_end =
					!need_reconfig;
				ddl_context->ddl_callback(
					VCD_EVT_RESP_INPUT_DONE,
					VCD_S_SUCCESS, &ddl->input_frame,
					sizeof(struct ddl_frame_data_tag),
					(u32 *) ddl, ddl->client_data);
			}
			if (need_reconfig) {
				struct ddl_frame_data_tag *payload =
					&ddl->input_frame;
				u32 payload_size =
					sizeof(struct ddl_frame_data_tag);

				decoder->client_frame_size =
					decoder->frame_size;
				decoder->client_output_buf_req =
					decoder->actual_output_buf_req;
				decoder->client_input_buf_req =
					decoder->actual_input_buf_req;
				if (seq_hdr_only_frame) {
					payload = NULL;
					payload_size = 0;
				}
				ddl_context->ddl_callback(
					VCD_EVT_IND_OUTPUT_RECONFIG,
					VCD_S_SUCCESS, payload,
					payload_size, (u32 *) ddl,
					ddl->client_data);
			}
			if (!need_reconfig && !seq_hdr_only_frame) {
				if (!ddl_vidc_decode_set_buffers(ddl))
					process_further = false;
				else {
					DDL_MSG_ERROR("ddl_vidc_decode_set_\
						buffers failed");
					ddl_client_fatal_cb(ddl);
				}
			} else
				ddl_release_command_channel(ddl_context,
					ddl->command_channel);
		}
	}
	return process_further;
}

static u32 ddl_sequence_done_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl;
	u32 channel_inst_id, ret;

	vidc_1080p_get_returned_channel_inst_id(&channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	ddl = ddl_get_current_ddl_client_for_channel_id(ddl_context,
			ddl_context->response_cmd_ch_id);
	if (!ddl) {
		DDL_MSG_ERROR("UNKWN_SEQ_DONE");
		ret = true;
	} else {
		if (ddl->decoding)
			ret = ddl_decoder_seq_done_callback(ddl_context,
					ddl);
		else
			ret = ddl_encoder_seq_done_callback(ddl_context,
					ddl);
	}
	return ret;
}

static u32 ddl_dpb_buffers_set_done_callback(
	struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl;
	u32 channel_inst_id, ret_status = true;

	DDL_MSG_MED("ddl_dpb_buffers_set_done_callback");
	vidc_1080p_get_returned_channel_inst_id(&channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	ddl = ddl_get_current_ddl_client_for_command(ddl_context,
			DDL_CMD_DECODE_SET_DPB);
	if (ddl) {
		ddl->cmd_state = DDL_CMD_INVALID;
		if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPBDONE)) {
			DDL_MSG_ERROR("STATE-CRITICAL-DPBDONE");
			ddl_client_fatal_cb(ddl);
		} else {
			DDL_MSG_LOW("INTR_DPBDONE");
			DDL_MSG_LOW("ddl_state_transition: %s ~~>\
				DDL_CLIENT_WAIT_FOR_FRAME",\
				ddl_get_state_string(ddl->client_state));
			ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME;
			ddl->codec_data.decoder.dec_disp_info.\
				img_size_x = 0;
			ddl->codec_data.decoder.dec_disp_info.\
				img_size_y = 0;
			ddl_vidc_decode_frame_run(ddl);
			ret_status = false;
		}
	}
	return ret_status;
}

static void ddl_encoder_frame_run_callback(
	struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_encoder_data *encoder =
		&(ddl->codec_data.encoder);
	struct vcd_frame_data *output_frame =
		&(ddl->output_frame.vcd_frm);
	u32 bottom_frame_tag;

	DDL_MSG_MED("ddl_encoder_frame_run_callback");
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE) &&
		!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-ENCFRMRUN");
		ddl_client_fatal_cb(ddl);
	} else {
		DDL_MSG_LOW("ENC_FRM_RUN_DONE");
		ddl->cmd_state = DDL_CMD_INVALID;
		vidc_1080p_get_encode_frame_info(&encoder->enc_frame_info);
		vidc_sm_get_frame_tags(&ddl->shared_mem
			[ddl->command_channel],
			&output_frame->ip_frm_tag, &bottom_frame_tag);
		if (encoder->enc_frame_info.enc_frame_size ||
			(encoder->enc_frame_info.enc_frame ==
			VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED) ||
			DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
			u8 *input_buffer_address = NULL;
#ifdef DDL_PROFILE
			ddl_calc_core_time(1);
#endif
			output_frame->data_len =
				encoder->enc_frame_info.enc_frame_size;
			output_frame->flags |= VCD_FRAME_FLAG_ENDOFFRAME;
			ddl_get_encoded_frame(output_frame,
				encoder->codec.codec,
				encoder->enc_frame_info.enc_frame);
			ddl_vidc_encode_dynamic_property(ddl, false);
			ddl->input_frame.frm_trans_end = false;
			input_buffer_address = ddl_context->dram_base_a.\
				align_physical_addr +
				encoder->enc_frame_info.enc_luma_address;
			ddl_get_input_frame_from_pool(ddl,
				input_buffer_address);
			ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE,
				VCD_S_SUCCESS, &(ddl->input_frame),
				sizeof(struct ddl_frame_data_tag),
				(u32 *)ddl, ddl->client_data);
			ddl->output_frame.frm_trans_end =
				DDLCLIENT_STATE_IS(ddl,
				DDL_CLIENT_WAIT_FOR_EOS_DONE) ? false : true;
			ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
				VCD_S_SUCCESS, &(ddl->output_frame),
				sizeof(struct ddl_frame_data_tag),
				(u32 *)ddl, ddl->client_data);

			if (DDLCLIENT_STATE_IS(ddl,
				DDL_CLIENT_WAIT_FOR_EOS_DONE) &&
				encoder->i_period.b_frames) {
				if ((ddl->extra_output_buf_count < 0) ||
					(ddl->extra_output_buf_count >
					encoder->i_period.b_frames)) {
					DDL_MSG_ERROR("Invalid B frame output"
								"buffer index");
				} else {
					struct vidc_1080p_enc_frame_start_param
						enc_param;
					ddl->output_frame = ddl->\
					extra_output_frame[ddl->\
					extra_output_buf_count];
					ddl->\
					extra_output_buf_count--;
					output_frame =
					&ddl->output_frame.\
					vcd_frm;
					memset(&enc_param, 0,
						sizeof(enc_param));
					enc_param.cmd_seq_num =
						++ddl_context->cmd_seq_num;
					enc_param.inst_id = ddl->instance_id;
					enc_param.shared_mem_addr_offset =
					   DDL_ADDR_OFFSET(ddl_context->\
						dram_base_a, ddl->shared_mem
						[ddl->command_channel]);
					enc_param.stream_buffer_addr_offset =
						DDL_OFFSET(ddl_context->\
						dram_base_a.\
						align_physical_addr,
						output_frame->physical);
					enc_param.stream_buffer_size =
					encoder->client_output_buf_req.sz;
					enc_param.encode =
					VIDC_1080P_ENC_TYPE_LAST_FRAME_DATA;
<<<<<<< HEAD
				ddl->cmd_state = DDL_CMD_ENCODE_FRAME;
				ddl_context->vidc_encode_frame_start
				[ddl->command_channel]
					(&enc_param);
			} else {
				DDL_MSG_LOW("ddl_state_transition: %s ~~> \
					DDL_CLIENT_WAIT_FOR_FRAME",
=======
					ddl->cmd_state = DDL_CMD_ENCODE_FRAME;
					ddl_context->vidc_encode_frame_start
						[ddl->command_channel]
						(&enc_param);
				} } else {
				DDL_MSG_LOW("ddl_state_transition: %s ~~>"
					"DDL_CLIENT_WAIT_FOR_FRAME",
>>>>>>> a3677e5... vidc: venc: Add Bframe support
					ddl_get_state_string(
					ddl->client_state));
				ddl->client_state =
					DDL_CLIENT_WAIT_FOR_FRAME;
				ddl_release_command_channel(ddl_context,
				ddl->command_channel);
			}
		} else {
			ddl_context->ddl_callback(
				VCD_EVT_RESP_TRANSACTION_PENDING,
				VCD_S_SUCCESS, NULL, 0, (u32 *)ddl,
				ddl->client_data);
			DDL_MSG_LOW("ddl_state_transition: %s ~~> \
				DDL_CLIENT_WAIT_FOR_FRAME",
			ddl_get_state_string(ddl->client_state));
			ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME;
			ddl_release_command_channel(ddl_context,
			ddl->command_channel);
		}
	}
}

static u32 ddl_decoder_frame_run_callback(
	struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct vidc_1080p_dec_disp_info *dec_disp_info =
		&ddl->codec_data.decoder.dec_disp_info;
	u32 callback_end = false, ret_status = true, eos_present = false;

	DDL_MSG_MED("ddl_decoder_frame_run_callback");
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-DECFRMRUN");
		ddl_client_fatal_cb(ddl);
	} else {
		DDL_MSG_LOW("DEC_FRM_RUN_DONE");
		ddl->cmd_state = DDL_CMD_INVALID;
		vidc_1080p_get_display_frame_result(dec_disp_info);
		ddl_vidc_decode_dynamic_property(ddl, false);
		if (dec_disp_info->resl_change) {
			DDL_MSG_ERROR("DEC_RECONFIG_NOT_SUPPORTED");
			ddl_client_fatal_cb(ddl);
		} else {
			if ((VCD_FRAME_FLAG_EOS &
				ddl->input_frame.vcd_frm.flags)) {
				callback_end = false;
				eos_present = true;
			}
			if (dec_disp_info->display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_ONLY ||
				dec_disp_info->display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_AND_DISPLAY) {
				if (!eos_present)
					callback_end =
					(dec_disp_info->display_status ==
					VIDC_1080P_DISPLAY_STATUS_DECODE_ONLY);
				ddl_decoder_input_done_callback(ddl,
					callback_end);
			}
			if (dec_disp_info->display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_AND_DISPLAY ||
				dec_disp_info->display_status ==
				VIDC_1080P_DISPLAY_STATUS_DISPLAY_ONLY) {
				u32 vcd_status;
				if (!eos_present)
					callback_end = (dec_disp_info->\
					display_status ==
				VIDC_1080P_DISPLAY_STATUS_DECODE_AND_DISPLAY);

				vcd_status = ddl_decoder_ouput_done_callback(
					ddl, callback_end);
				if (vcd_status)
					return true;
			}
			if (dec_disp_info->display_status ==
				VIDC_1080P_DISPLAY_STATUS_DISPLAY_ONLY) {
				ddl_vidc_decode_frame_run(ddl);
				ret_status = false;
			} else if (eos_present) {
				ddl_vidc_decode_eos_run(ddl);
				ret_status = false;
			} else {
				ddl->client_state =
					DDL_CLIENT_WAIT_FOR_FRAME;
				ddl_release_command_channel(ddl_context,
					ddl->command_channel);
			}
		}
	}
	return ret_status;
}

static u32 ddl_eos_frame_done_callback(
	struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	struct vidc_1080p_dec_disp_info *dec_disp_info =
		&decoder->dec_disp_info;
	struct ddl_mask *dpb_mask = &decoder->dpb_mask;
	u32 ret_status = true;

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
		DDL_MSG_ERROR("STATE-CRITICAL-EOSFRMRUN");
		ddl_client_fatal_cb(ddl);
	} else {
		DDL_MSG_LOW("EOS_FRM_RUN_DONE");
		ddl->cmd_state = DDL_CMD_INVALID;
		vidc_1080p_get_display_frame_result(dec_disp_info);
		ddl_vidc_decode_dynamic_property(ddl, false);
		if (dec_disp_info->display_status ==
			VIDC_1080P_DISPLAY_STATUS_DPB_EMPTY) {
			ddl_decoder_eos_done_callback(ddl);
		} else {
			struct vidc_1080p_dec_frame_start_param dec_param;
			if (dec_disp_info->display_status ==
				VIDC_1080P_DISPLAY_STATUS_DISPLAY_ONLY) {
				u32 vcd_status;

				vcd_status = ddl_decoder_ouput_done_callback(
					ddl, false);
				if (vcd_status)
					return true;
			} else
				DDL_MSG_ERROR("EOS-STATE-CRITICAL-\
					WRONG-DISP-STATUS");

			ddl_decoder_dpb_transact(decoder, NULL,
				DDL_DPB_OP_SET_MASK);
			ddl->cmd_state = DDL_CMD_EOS;

			memset(&dec_param, 0, sizeof(dec_param));

			dec_param.cmd_seq_num =
				++ddl_context->cmd_seq_num;
			dec_param.inst_id = ddl->instance_id;
			dec_param.shared_mem_addr_offset =
				DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				ddl->shared_mem[ddl->command_channel]);
			dec_param.release_dpb_bit_mask =
				dpb_mask->hw_mask;
			dec_param.decode =
				VIDC_1080P_DEC_TYPE_LAST_FRAME_DATA;

			ddl_context->vidc_decode_frame_start[ddl->\
				command_channel](&dec_param);
			ret_status = false;
		}
	}
	return ret_status;
}

static u32 ddl_frame_run_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl;
	u32 channel_inst_id;
	u32 return_status = true;

	vidc_1080p_get_returned_channel_inst_id(&channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	ddl = ddl_get_current_ddl_client_for_channel_id(ddl_context,
			ddl_context->response_cmd_ch_id);
	if (ddl) {
		if (ddl->cmd_state == DDL_CMD_DECODE_FRAME)
			return_status = ddl_decoder_frame_run_callback(ddl);
		else if (ddl->cmd_state == DDL_CMD_ENCODE_FRAME)
			ddl_encoder_frame_run_callback(ddl);
		else if (ddl->cmd_state == DDL_CMD_EOS)
			return_status = ddl_eos_frame_done_callback(ddl);
		else {
			DDL_MSG_ERROR("UNKWN_FRAME_DONE");
			return_status = false;
		}
	} else
		return_status = false;

	return return_status;
}

static void ddl_channel_end_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl;

	DDL_MSG_MED("ddl_channel_end_callback");
	ddl = ddl_get_current_ddl_client_for_command(ddl_context,
			DDL_CMD_CHANNEL_END);
	if (ddl) {
		ddl->cmd_state = DDL_CMD_INVALID;
		if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_CHEND)) {
			DDL_MSG_LOW("STATE-CRITICAL-CHEND");
		} else {
			DDL_MSG_LOW("CH_END_DONE");
			ddl_release_client_internal_buffers(ddl);
			ddl_context->ddl_callback(VCD_EVT_RESP_STOP,
				VCD_S_SUCCESS, NULL, 0, (u32 *)ddl,
				ddl->client_data);
			DDL_MSG_LOW("ddl_state_transition: %s ~~>\
				DDL_CLIENT_OPEN",\
				ddl_get_state_string(ddl->client_state));
			ddl->client_state = DDL_CLIENT_OPEN;
		}
		ddl_release_command_channel(ddl_context,
			ddl->command_channel);
	}
}

static void ddl_edfu_callback(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl;
	u32 channel_inst_id;

	DDL_MSG_MED("ddl_edfu_callback");
	vidc_1080p_get_returned_channel_inst_id(&channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	ddl = ddl_get_current_ddl_client_for_channel_id(ddl_context,
			ddl_context->response_cmd_ch_id);
	if (ddl) {
		if (ddl->cmd_state != DDL_CMD_ENCODE_FRAME)
			DDL_MSG_LOW("UNKWN_EDFU");
	}
}

static void ddl_encoder_eos_done(struct ddl_context *ddl_context)
{
	struct ddl_client_context *ddl;
	u32 channel_inst_id;

	vidc_1080p_get_returned_channel_inst_id(&channel_inst_id);
	vidc_1080p_clear_returned_channel_inst_id();
	ddl = ddl_get_current_ddl_client_for_channel_id(ddl_context,
			ddl_context->response_cmd_ch_id);
<<<<<<< HEAD
	if (ddl) {
		if (DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)) {
			DDL_MSG_LOW("ENC_EOS_DONE");
			ddl->cmd_state = DDL_CMD_INVALID;
			DDL_MSG_LOW("ddl_state_transition: %s ~~> \
				DDL_CLIENT_WAIT_FOR_FRAME",
=======
	if (!ddl || (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE))) {
		DDL_MSG_ERROR("STATE-CRITICAL-EOSFRMDONE");
		ddl_client_fatal_cb(ddl);
	} else {
		struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
		vidc_1080p_get_encode_frame_info(&encoder->enc_frame_info);
		ddl_handle_enc_frame_done(ddl);
		DDL_MSG_LOW("encoder_eos_done");
		ddl->cmd_state = DDL_CMD_INVALID;
		DDL_MSG_LOW("ddl_state_transition: %s ~~>"
				"DDL_CLIENT_WAIT_FOR_FRAME",
>>>>>>> a3677e5... vidc: venc: Add Bframe support
				ddl_get_state_string(ddl->client_state));
		ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME;
		DDL_MSG_LOW("eos_done");
		ddl_context->ddl_callback(VCD_EVT_RESP_EOS_DONE,
				VCD_S_SUCCESS, NULL, 0,
				(u32 *)ddl, ddl->client_data);
		ddl_release_command_channel(ddl_context,
			ddl->command_channel);
	}
}

static u32 ddl_process_intr_status(struct ddl_context *ddl_context,
	u32 intr_status)
{
	u32 return_status = true;
	switch (intr_status) {
	case VIDC_1080P_RISC2HOST_CMD_OPEN_CH_RET:
		return_status = ddl_channel_set_callback(ddl_context,
			ddl_context->response_cmd_ch_id);
	break;
	case VIDC_1080P_RISC2HOST_CMD_CLOSE_CH_RET:
		ddl_channel_end_callback(ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_SEQ_DONE_RET:
		return_status = ddl_sequence_done_callback(ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_FRAME_DONE_RET:
		return_status = ddl_frame_run_callback(ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_SYS_INIT_RET:
		ddl_sys_init_done_callback(ddl_context,
			ddl_context->response_cmd_ch_id);
	break;
	case VIDC_1080P_RISC2HOST_CMD_FW_STATUS_RET:
		ddl_fw_status_done_callback(ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_EDFU_INT_RET:
		ddl_edfu_callback(ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_ENC_COMPLETE_RET:
		ddl_encoder_eos_done(ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_ERROR_RET:
		DDL_MSG_ERROR("OP_FAILED_INTR");
		return_status = ddl_handle_core_errors(ddl_context);
	break;
	case VIDC_1080P_RISC2HOST_CMD_INIT_BUFFERS_RET:
		return_status =
			ddl_dpb_buffers_set_done_callback(ddl_context);
	break;
	default:
		DDL_MSG_LOW("UNKWN_INTR");
	break;
	}
	return return_status;
}

void ddl_read_and_clear_interrupt(void)
{
	struct ddl_context *ddl_context;
	struct ddl_hw_interface  *ddl_hw_response;

	ddl_context = ddl_get_context();
	if (!ddl_context->core_virtual_base_addr) {
		DDL_MSG_LOW("SPURIOUS_INTERRUPT");
	} else {
		ddl_hw_response = &ddl_context->ddl_hw_response;
		vidc_1080p_get_risc2host_cmd(&ddl_hw_response->cmd,
			&ddl_hw_response->arg1, &ddl_hw_response->arg2,
			&ddl_hw_response->arg3, &ddl_hw_response->arg4);
		vidc_1080p_clear_risc2host_cmd();
		vidc_1080p_clear_interrupt();
		ddl_context->cmd_err_status =
			ddl_hw_response->arg2 & 0xffff;
		ddl_context->disp_pic_err_status =
			(ddl_hw_response->arg2 & 0xffff0000) >> 16;
		ddl_context->response_cmd_ch_id = ddl_hw_response->arg1;
	}
}

u32 ddl_process_core_response(void)
{
	struct ddl_context *ddl_context;
	struct ddl_hw_interface *ddl_hw_response;
	u32 return_status = false;

	ddl_context = ddl_get_context();
	if (!ddl_context->core_virtual_base_addr) {
		DDL_MSG_LOW("SPURIOUS_INTERRUPT");
	} else {
		ddl_hw_response = &ddl_context->ddl_hw_response;
		if (ddl_hw_response->cmd == DDL_INVALID_INTR_STATUS) {
			DDL_MSG_ERROR("INTERRUPT_NOT_READ");
		} else {
			return_status = ddl_process_intr_status(ddl_context,
				ddl_hw_response->cmd);
			if (ddl_context->interrupt_clr)
				(*ddl_context->interrupt_clr)();
			ddl_hw_response->cmd = DDL_INVALID_INTR_STATUS;
		}
	}
	return return_status;
}

static void ddl_decoder_input_done_callback(
	struct ddl_client_context *ddl, u32 frame_transact_end)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	struct vidc_1080p_dec_disp_info *dec_disp_info =
		&decoder->dec_disp_info;
	struct vcd_frame_data *input_vcd_frm =
		&(ddl->input_frame.vcd_frm);

	vidc_1080p_get_decoded_frame_size(
		&dec_disp_info->input_bytes_consumed);
	vidc_1080p_get_decode_frame(&dec_disp_info->input_frame);
	ddl_get_decoded_frame(input_vcd_frm,
		dec_disp_info->input_frame);
	vidc_1080p_get_decode_frame_result(dec_disp_info);
	input_vcd_frm->interlaced = (dec_disp_info->display_coding !=
		VIDC_1080P_DISPLAY_CODING_PROGRESSIVE_SCAN);
	input_vcd_frm->offset += dec_disp_info->input_bytes_consumed;
	input_vcd_frm->data_len -= dec_disp_info->input_bytes_consumed;
	ddl->input_frame.frm_trans_end = frame_transact_end;
	ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE, VCD_S_SUCCESS,
		&ddl->input_frame, sizeof(struct ddl_frame_data_tag),
		(u32 *)ddl, ddl->client_data);
}

static u32 ddl_decoder_ouput_done_callback(
	struct ddl_client_context *ddl, u32 frame_transact_end)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	struct vidc_1080p_dec_disp_info *dec_disp_info =
		&(decoder->dec_disp_info);
	struct ddl_frame_data_tag *output_frame = &(ddl->output_frame);
	struct vcd_frame_data *output_vcd_frm =
		&(output_frame->vcd_frm);
	u32 vcd_status, free_luma_dpb = 0, disp_pict = 0;

	output_vcd_frm->physical =
		(u8 *) (dec_disp_info->display_y_addr << 11);

	vidc_sm_get_displayed_picture_frame(&ddl->shared_mem
		[ddl->command_channel], &disp_pict);
	if (!disp_pict)
		output_vcd_frm->frame = VCD_FRAME_NOTCODED;
	else
		output_vcd_frm->frame = VCD_FRAME_YUV;

	if (decoder->codec.codec == VCD_CODEC_MPEG4 ||
		decoder->codec.codec == VCD_CODEC_VC1 ||
		decoder->codec.codec == VCD_CODEC_VC1_RCV ||
		(decoder->codec.codec >= VCD_CODEC_DIVX_3 &&
		decoder->codec.codec <= VCD_CODEC_XVID)) {
		if (output_vcd_frm->frame == VCD_FRAME_NOTCODED) {
			vidc_sm_get_available_luma_dpb_address(
				&ddl->shared_mem[ddl->command_channel],
				&free_luma_dpb);
			if (free_luma_dpb)
				output_vcd_frm->physical =
					(u8 *)(free_luma_dpb << 11);
		}
	}
	vcd_status = ddl_decoder_dpb_transact(decoder, output_frame,
			DDL_DPB_OP_MARK_BUSY);
	if (vcd_status) {
		DDL_MSG_ERROR("CORRUPTED_OUTPUT_BUFFER_ADDRESS");
		ddl_hw_fatal_cb(ddl);
	} else {
		vidc_sm_get_frame_tags(&ddl->shared_mem
			[ddl->command_channel],
			&dec_disp_info->tag_top,
			&dec_disp_info->tag_bottom);
		output_vcd_frm->ip_frm_tag = dec_disp_info->tag_top;

		vidc_sm_get_picture_times(&ddl->shared_mem
			[ddl->command_channel],
			&dec_disp_info->pic_time_top,
			&dec_disp_info->pic_time_bottom);

		vidc_sm_get_crop_info(&ddl->shared_mem
			[ddl->command_channel],
			&dec_disp_info->crop_left_offset,
			&dec_disp_info->crop_right_offset,
			&dec_disp_info->crop_top_offset,
			&dec_disp_info->crop_bottom_offset);

		if (dec_disp_info->crop_left_offset ||
			dec_disp_info->crop_right_offset ||
			dec_disp_info->crop_top_offset ||
			dec_disp_info->crop_bottom_offset)
			dec_disp_info->crop_exists = true;
		else
			dec_disp_info->crop_exists = false;

		if (dec_disp_info->crop_exists) {
			output_vcd_frm->dec_op_prop.disp_frm.left =
				dec_disp_info->crop_left_offset;
			output_vcd_frm->dec_op_prop.disp_frm.top =
				dec_disp_info->crop_top_offset;
			output_vcd_frm->dec_op_prop.disp_frm.right =
				decoder->frame_size.width -
				dec_disp_info->crop_right_offset;
			output_vcd_frm->dec_op_prop.disp_frm.bottom =
				decoder->frame_size.height -
				dec_disp_info->crop_bottom_offset;
		} else {
			output_vcd_frm->dec_op_prop.disp_frm.left = 0;
			output_vcd_frm->dec_op_prop.disp_frm.top = 0;
			output_vcd_frm->dec_op_prop.disp_frm.right =
				decoder->frame_size.width;
			output_vcd_frm->dec_op_prop.disp_frm.bottom =
				decoder->frame_size.height;
		}
		if (dec_disp_info->display_coding ==
			VIDC_1080P_DISPLAY_CODING_PROGRESSIVE_SCAN) {
			output_vcd_frm->interlaced = false;
			output_vcd_frm->intrlcd_ip_frm_tag =
				VCD_FRAMETAG_INVALID;
		} else {
			output_vcd_frm->interlaced = true;
			if (!dec_disp_info->tag_bottom)
				output_vcd_frm->intrlcd_ip_frm_tag =
					VCD_FRAMETAG_INVALID;
			else
				output_vcd_frm->intrlcd_ip_frm_tag =
					dec_disp_info->tag_bottom;
		}
		output_vcd_frm->offset = 0;
		output_vcd_frm->data_len = decoder->y_cb_cr_size;
		if (free_luma_dpb) {
			output_vcd_frm->data_len = 0;
			output_vcd_frm->flags |= VCD_FRAME_FLAG_DECODEONLY;
		}
		output_vcd_frm->flags |= VCD_FRAME_FLAG_ENDOFFRAME;
		output_frame->frm_trans_end = frame_transact_end;
#ifdef DDL_PROFILE
		ddl_calc_core_time(0);
#endif
		ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
			vcd_status, output_frame,
			sizeof(struct ddl_frame_data_tag),
			(u32 *)ddl, ddl->client_data);
	}
	return vcd_status;
}

static u32 ddl_get_decoded_frame(struct vcd_frame_data  *frame,
	enum vidc_1080p_decode_frame frame_type)
{
	u32 status = true;

	switch (frame_type) {
	case VIDC_1080P_DECODE_FRAMETYPE_I:
		frame->flags |= VCD_FRAME_FLAG_SYNCFRAME;
		frame->frame = VCD_FRAME_I;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_P:
		frame->frame = VCD_FRAME_P;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_B:
		frame->frame = VCD_FRAME_B;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_NOT_CODED:
		frame->frame = VCD_FRAME_NOTCODED;
		frame->data_len = 0;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_OTHERS:
		frame->frame = VCD_FRAME_YUV;
	break;
	case VIDC_1080P_DECODE_FRAMETYPE_32BIT:
	default:
		DDL_MSG_ERROR("UNKNOWN-FRAMETYPE");
		status = false;
	break;
	}
	return status;
}

static u32 ddl_get_encoded_frame(struct vcd_frame_data *frame,
	enum vcd_codec codec,
	enum vidc_1080p_encode_frame frame_type)
{
	u32 status = true;

	if (codec == VCD_CODEC_H264) {
		switch (frame_type) {
		case VIDC_1080P_ENCODE_FRAMETYPE_NOT_CODED:
			frame->frame = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_I:
			frame->flags |= VCD_FRAME_FLAG_SYNCFRAME;
			frame->frame = VCD_FRAME_I;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_P:
			frame->frame = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_B:
			frame->frame = VCD_FRAME_B;
			frame->flags |= VCD_FRAME_FLAG_BFRAME;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED:
			frame->frame = VCD_FRAME_NOTCODED;
			frame->data_len = 0;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_OTHERS:
			DDL_MSG_LOW("FRAMETYPE-OTHERS");
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_32BIT:
		default:
			DDL_MSG_LOW("UNKNOWN-FRAMETYPE");
			status = false;
		break;
		}
	} else if (codec == VCD_CODEC_MPEG4) {
		switch (frame_type) {
		case VIDC_1080P_ENCODE_FRAMETYPE_NOT_CODED:
			frame->frame = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_I:
			frame->flags |= VCD_FRAME_FLAG_SYNCFRAME;
			frame->frame = VCD_FRAME_I;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_P:
			frame->frame = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_B:
			frame->frame = VCD_FRAME_B;
			frame->flags |= VCD_FRAME_FLAG_BFRAME;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED:
			frame->frame = VCD_FRAME_NOTCODED;
			frame->data_len = 0;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_OTHERS:
			DDL_MSG_LOW("FRAMETYPE-OTHERS");
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_32BIT:
		default:
			DDL_MSG_LOW("UNKNOWN-FRAMETYPE");
			status = false;
		break;
		}
	} else if (codec == VCD_CODEC_H263) {
		switch (frame_type) {
		case VIDC_1080P_ENCODE_FRAMETYPE_NOT_CODED:
			frame->frame = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_I:
			frame->flags |= VCD_FRAME_FLAG_SYNCFRAME;
			frame->frame = VCD_FRAME_I;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_P:
			frame->frame = VCD_FRAME_P;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_SKIPPED:
			frame->frame = VCD_FRAME_NOTCODED;
			frame->data_len = 0;
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_OTHERS:
			DDL_MSG_LOW("FRAMETYPE-OTHERS");
		break;
		case VIDC_1080P_ENCODE_FRAMETYPE_32BIT:
		default:
			DDL_MSG_LOW("UNKNOWN-FRAMETYPE");
			status = false;
		break;
		}
	} else
		status = false;
	return status;
}

static void ddl_get_mpeg4_dec_level(enum vcd_codec_level *level,
	u32 level_codec, enum vcd_codec_profile mpeg4_profile)
{
	switch (level_codec) {
	case VIDC_1080P_MPEG4_LEVEL0:
		*level = VCD_LEVEL_MPEG4_0;
	break;
	case VIDC_1080P_MPEG4_LEVEL0b:
		*level = VCD_LEVEL_MPEG4_0b;
	break;
	case VIDC_1080P_MPEG4_LEVEL1:
		*level = VCD_LEVEL_MPEG4_1;
	break;
	case VIDC_1080P_MPEG4_LEVEL2:
		*level = VCD_LEVEL_MPEG4_2;
	break;
	case VIDC_1080P_MPEG4_LEVEL3:
		*level = VCD_LEVEL_MPEG4_3;
	break;
	case VIDC_1080P_MPEG4_LEVEL3b:
		if (mpeg4_profile == VCD_PROFILE_MPEG4_SP)
			*level = VCD_LEVEL_MPEG4_7;
		else
			*level = VCD_LEVEL_MPEG4_3b;
	break;
	case VIDC_1080P_MPEG4_LEVEL4a:
		*level = VCD_LEVEL_MPEG4_4a;
	break;
	case VIDC_1080P_MPEG4_LEVEL5:
		*level = VCD_LEVEL_MPEG4_5;
	break;
	case VIDC_1080P_MPEG4_LEVEL6:
		*level = VCD_LEVEL_MPEG4_6;
	break;
	default:
		*level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_h264_dec_level(enum vcd_codec_level *level,
	u32 level_codec)
{
	switch (level_codec) {
	case VIDC_1080P_H264_LEVEL1:
		*level = VCD_LEVEL_H264_1;
	break;
	case VIDC_1080P_H264_LEVEL1b:
		*level = VCD_LEVEL_H264_1b;
	break;
	case VIDC_1080P_H264_LEVEL1p1:
		*level = VCD_LEVEL_H264_1p1;
	break;
	case VIDC_1080P_H264_LEVEL1p2:
		*level = VCD_LEVEL_H264_1p2;
	break;
	case VIDC_1080P_H264_LEVEL1p3:
		*level = VCD_LEVEL_H264_1p3;
	break;
	case VIDC_1080P_H264_LEVEL2:
		*level = VCD_LEVEL_H264_2;
	break;
	case VIDC_1080P_H264_LEVEL2p1:
		*level = VCD_LEVEL_H264_2p1;
	break;
	case VIDC_1080P_H264_LEVEL2p2:
		*level = VCD_LEVEL_H264_2p2;
	break;
	case VIDC_1080P_H264_LEVEL3:
		*level = VCD_LEVEL_H264_3;
	break;
	case VIDC_1080P_H264_LEVEL3p1:
		*level = VCD_LEVEL_H264_3p1;
	break;
	case VIDC_1080P_H264_LEVEL3p2:
		*level = VCD_LEVEL_H264_3p2;
	break;
	case VIDC_1080P_H264_LEVEL4:
		*level = VCD_LEVEL_H264_4;
	break;
	default:
		*level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_h263_dec_level(enum vcd_codec_level *level,
	u32 level_codec)
{
	switch (level_codec) {
	case VIDC_1080P_H263_LEVEL10:
		*level = VCD_LEVEL_H263_10;
	break;
	case VIDC_1080P_H263_LEVEL20:
		*level = VCD_LEVEL_H263_20;
	break;
	case VIDC_1080P_H263_LEVEL30:
		*level = VCD_LEVEL_H263_30;
	break;
	case VIDC_1080P_H263_LEVEL40:
		*level = VCD_LEVEL_H263_40;
	break;
	case VIDC_1080P_H263_LEVEL45:
		*level = VCD_LEVEL_H263_45;
	break;
	case VIDC_1080P_H263_LEVEL50:
		*level = VCD_LEVEL_H263_50;
	break;
	case VIDC_1080P_H263_LEVEL60:
		*level = VCD_LEVEL_H263_60;
	break;
	case VIDC_1080P_H263_LEVEL70:
		*level = VCD_LEVEL_H263_70;
	break;
	default:
		*level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_vc1_dec_level(enum vcd_codec_level *level,
	u32 level_codec, enum vcd_codec_profile vc1_profile)
{
	if (vc1_profile == VCD_PROFILE_VC1_ADVANCE) {
		switch (level_codec) {
		case VIDC_SM_LEVEL_VC1_ADV_0:
			*level = VCD_LEVEL_VC1_A_0;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_1:
			*level = VCD_LEVEL_VC1_A_1;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_2:
			*level = VCD_LEVEL_VC1_A_2;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_3:
			*level = VCD_LEVEL_VC1_A_3;
		break;
		case VIDC_SM_LEVEL_VC1_ADV_4:
			*level = VCD_LEVEL_VC1_A_4;
		break;
		default:
			*level = VCD_LEVEL_UNKNOWN;
		break;
		}
	} else if (vc1_profile == VCD_PROFILE_VC1_MAIN) {
		switch (level_codec) {
		case VIDC_SM_LEVEL_VC1_LOW:
			*level = VCD_LEVEL_VC1_M_LOW;
		break;
		case VIDC_SM_LEVEL_VC1_MEDIUM:
			*level = VCD_LEVEL_VC1_M_MEDIUM;
		break;
		case VIDC_SM_LEVEL_VC1_HIGH:
			*level = VCD_LEVEL_VC1_M_HIGH;
		break;
		default:
			*level = VCD_LEVEL_UNKNOWN;
		break;
		}
	} else if (vc1_profile == VCD_PROFILE_VC1_SIMPLE) {
		switch (level_codec) {
		case VIDC_SM_LEVEL_VC1_LOW:
			*level = VCD_LEVEL_VC1_S_LOW;
		break;
		case VIDC_SM_LEVEL_VC1_MEDIUM:
			*level = VCD_LEVEL_VC1_S_MEDIUM;
		break;
		default:
			*level = VCD_LEVEL_UNKNOWN;
		break;
		}
	}
}

static void ddl_get_mpeg2_dec_level(enum vcd_codec_level *level,
	u32 level_codec)
{
	switch (level_codec) {
	case VIDC_SM_LEVEL_MPEG2_LOW:
		*level = VCD_LEVEL_MPEG2_LOW;
	break;
	case VIDC_SM_LEVEL_MPEG2_MAIN:
		*level = VCD_LEVEL_MPEG2_MAIN;
	break;
	case VIDC_SM_LEVEL_MPEG2_HIGH_1440:
		*level = VCD_LEVEL_MPEG2_HIGH_14;
	break;
	case VIDC_SM_LEVEL_MPEG2_HIGH:
		*level = VCD_LEVEL_MPEG2_HIGH;
	break;
	default:
		*level = VCD_LEVEL_UNKNOWN;
	break;
	}
}

static void ddl_get_dec_profile_level(struct ddl_decoder_data *decoder,
	u32 profile_codec, u32 level_codec)
{
	enum vcd_codec_profile profile = VCD_PROFILE_UNKNOWN;
	enum vcd_codec_level level = VCD_LEVEL_UNKNOWN;

	switch (decoder->codec.codec) {
	case VCD_CODEC_MPEG4:
	case VCD_CODEC_XVID:
		if (profile_codec == VIDC_SM_PROFILE_MPEG4_SIMPLE)
			profile = VCD_PROFILE_MPEG4_SP;
		else if (profile_codec == VIDC_SM_PROFILE_MPEG4_ADV_SIMPLE)
			profile = VCD_PROFILE_MPEG4_ASP;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_mpeg4_dec_level(&level, level_codec, profile);
	break;
	case VCD_CODEC_H264:
		if (profile_codec == VIDC_SM_PROFILE_H264_BASELINE)
			profile = VCD_PROFILE_H264_BASELINE;
		else if (profile_codec == VIDC_SM_PROFILE_H264_MAIN)
			profile = VCD_PROFILE_H264_MAIN;
		else if (profile_codec == VIDC_SM_PROFILE_H264_HIGH)
			profile = VCD_PROFILE_H264_HIGH;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_h264_dec_level(&level, level_codec);
	break;
	case VCD_CODEC_H263:
		if (profile_codec == VIDC_SM_PROFILE_H263_BASELINE)
			profile = VCD_PROFILE_H263_BASELINE;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_h263_dec_level(&level, level_codec);
	break;
	case VCD_CODEC_MPEG2:
		if (profile_codec == VIDC_SM_PROFILE_MPEG2_MAIN)
			profile = VCD_PROFILE_MPEG2_MAIN;
		else if (profile_codec == VIDC_SM_PROFILE_MPEG2_SIMPLE)
			profile = VCD_PROFILE_MPEG2_SIMPLE;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_mpeg2_dec_level(&level, level_codec);
	break;
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		if (profile_codec == VIDC_SM_PROFILE_VC1_SIMPLE)
			profile = VCD_PROFILE_VC1_SIMPLE;
		else if (profile_codec == VIDC_SM_PROFILE_VC1_MAIN)
			profile = VCD_PROFILE_VC1_MAIN;
		else if (profile_codec == VIDC_SM_PROFILE_VC1_ADVANCED)
			profile = VCD_PROFILE_VC1_ADVANCE;
		else
			profile = VCD_PROFILE_UNKNOWN;
		ddl_get_vc1_dec_level(&level, level_codec, profile);
	break;
	default:
		if (!profile_codec)
			profile = VCD_PROFILE_UNKNOWN;
		if (!level)
			level = VCD_LEVEL_UNKNOWN;
	break;
	}
	decoder->profile.profile = profile;
	decoder->level.level = level;
}

static void ddl_handle_enc_frame_done(struct ddl_client_context *ddl)
{
	struct ddl_context       *ddl_context = ddl->ddl_context;
	struct ddl_encoder_data  *encoder = &(ddl->codec_data.encoder);
	struct vcd_frame_data    *output_frame = &(ddl->output_frame.vcd_frm);
	u32 bottom_frame_tag;
	u8  *input_buffer_address = NULL;

	vidc_sm_get_frame_tags(&ddl->shared_mem[ddl->command_channel],
		&output_frame->ip_frm_tag, &bottom_frame_tag);
	output_frame->data_len = encoder->enc_frame_info.enc_frame_size;
	output_frame->flags |= VCD_FRAME_FLAG_ENDOFFRAME;
	(void)ddl_get_encoded_frame(output_frame,
		encoder->codec.codec, encoder->enc_frame_info.enc_frame);
	ddl_process_encoder_metadata(ddl);
	ddl_vidc_encode_dynamic_property(ddl, false);
	ddl->input_frame.frm_trans_end = false;
	input_buffer_address = ddl_context->dram_base_a.align_physical_addr +
			encoder->enc_frame_info.enc_luma_address;
	ddl_get_input_frame_from_pool(ddl, input_buffer_address);

	ddl_context->ddl_callback(VCD_EVT_RESP_INPUT_DONE,
		VCD_S_SUCCESS, &(ddl->input_frame),
		sizeof(struct ddl_frame_data_tag),
		(u32 *) ddl, ddl->client_data);

	ddl->output_frame.frm_trans_end =
		DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_EOS_DONE)
			? false : true;

	ddl_context->ddl_callback(VCD_EVT_RESP_OUTPUT_DONE,
		VCD_S_SUCCESS, &(ddl->output_frame),
		sizeof(struct ddl_frame_data_tag),
		(u32 *) ddl, ddl->client_data);

}
