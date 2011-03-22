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

#define MAX_DPB_SIZE_L4PT0_MBS    DDL_KILO_BYTE(32)
#define MAX_FRAME_SIZE_L4PT0_MBS  DDL_KILO_BYTE(8)

static u32 ddl_set_dec_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *property_hdr, void *property_value);
static u32 ddl_set_enc_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *property_hdr, void *property_value);
static u32 ddl_get_dec_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *property_hdr, void *property_value);
static u32 ddl_get_enc_property(struct ddl_client_context *pddl,
	struct vcd_property_hdr *property_hdr, void *property_value);
static u32 ddl_set_enc_dynamic_property(struct ddl_encoder_data *encoder,
	struct vcd_property_hdr *property_hdr, void *property_value);
static void ddl_set_default_enc_property(struct ddl_client_context *ddl);
static void ddl_set_default_enc_profile(
	struct ddl_encoder_data *encoder);
static void ddl_set_default_enc_level(struct ddl_encoder_data *encoder);
static void ddl_set_default_enc_vop_timing(
	struct ddl_encoder_data *encoder);
static void ddl_set_default_enc_intra_period(
	struct ddl_encoder_data *encoder);
static void ddl_set_default_enc_rc_params(
	struct ddl_encoder_data *encoder);
static u32 ddl_valid_buffer_requirement(
	struct vcd_buffer_requirement *original_buf_req,
	struct vcd_buffer_requirement *req_buf_req);
static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data *decoder);
static u32 ddl_set_dec_buffers(struct ddl_decoder_data *decoder,
	struct ddl_property_dec_pic_buffers *dpb);

u32 ddl_set_property(u32 *ddl_handle,
	struct vcd_property_hdr *property_hdr, void *property_value)
{
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl =
		(struct ddl_client_context *) ddl_handle;
	u32 vcd_status;

	DDL_MSG_HIGH("ddl_set_property");
	if (!property_hdr || !property_value) {
		DDL_MSG_ERROR("ddl_set_prop:Bad_argument");
		return VCD_ERR_ILLEGAL_PARM;
	}
	ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(ddl_context)) {
		DDL_MSG_ERROR("ddl_set_prop:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}
	if (!ddl) {
		DDL_MSG_ERROR("ddl_set_prop:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (ddl->decoding)
		vcd_status = ddl_set_dec_property(ddl, property_hdr,
				property_value);
	else
		vcd_status = ddl_set_enc_property(ddl, property_hdr,
				property_value);
	if (vcd_status)
		DDL_MSG_ERROR("ddl_set_prop:FAILED");
	return vcd_status;
}

u32 ddl_get_property(u32 *ddl_handle,
	struct vcd_property_hdr *property_hdr, void *property_value)
{
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl =
		(struct ddl_client_context *) ddl_handle;
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;

	DDL_MSG_HIGH("ddl_get_property");
	if (!property_hdr || !property_value)
		return VCD_ERR_ILLEGAL_PARM;
	if (property_hdr->prop_id == DDL_I_CAPABILITY) {
		if (sizeof(struct ddl_property_capability) ==
			property_hdr->sz) {
			struct ddl_property_capability *ddl_capability =
				(struct ddl_property_capability *)
				property_value;

			ddl_capability->max_num_client = VCD_MAX_NO_CLIENT;
			ddl_capability->exclusive = VCD_COMMAND_EXCLUSIVE;
			ddl_capability->frame_command_depth =
				VCD_FRAME_COMMAND_DEPTH;
			ddl_capability->general_command_depth =
				VCD_GENEVIDC_COMMAND_DEPTH;
			ddl_capability->ddl_time_out_in_ms =
				DDL_HW_TIMEOUT_IN_MS;
			vcd_status = VCD_S_SUCCESS;
		}
		return vcd_status;
	}
	ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(ddl_context))
		return VCD_ERR_ILLEGAL_OP;
	if (!ddl)
		return VCD_ERR_BAD_HANDLE;
	if (ddl->decoding)
		vcd_status = ddl_get_dec_property(ddl, property_hdr,
				property_value);
	else
		vcd_status = ddl_get_enc_property(ddl, property_hdr,
				property_value);
	if (vcd_status)
		DDL_MSG_ERROR("ddl_get_prop:FAILED");
	else
		DDL_MSG_ERROR("ddl_get_prop:SUCCESS");
	return vcd_status;
}

u32 ddl_decoder_ready_to_start(struct ddl_client_context *ddl,
	struct vcd_sequence_hdr  *header)
{
	struct ddl_decoder_data *decoder =
		&(ddl->codec_data.decoder);

	if (!decoder->codec.codec) {
		DDL_MSG_ERROR("ddl_dec_start_check:Codec_not_set");
		return false;
	}
	if ((!header) && (!decoder->client_frame_size.height ||
		!decoder->client_frame_size.width)) {
		DDL_MSG_ERROR("ddl_dec_start_check:\
			Client_height_width_default");
		return false;
	}
	return true;
}

u32 ddl_encoder_ready_to_start(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);

	if (!encoder->codec.codec || !encoder->frame_size.height ||
		!encoder->frame_size.width ||
		!encoder->frame_rate.fps_denominator ||
		!encoder->frame_rate.fps_numerator ||
		!encoder->target_bit_rate.target_bitrate)
		return false;
	return true;
}

static u32 ddl_set_dec_property(struct ddl_client_context *ddl,
	struct vcd_property_hdr *property_hdr, void *property_value)
{
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	u32  vcd_status = VCD_ERR_ILLEGAL_PARM ;

	switch (property_hdr->prop_id) {
	case DDL_I_DPB_RELEASE:
		if ((sizeof(struct ddl_frame_data_tag) ==
			property_hdr->sz) &&
			(decoder->dp_buf.no_of_dec_pic_buf))
			vcd_status = ddl_decoder_dpb_transact(decoder,
				(struct ddl_frame_data_tag *)
				property_value, DDL_DPB_OP_MARK_FREE);
	break;
	case DDL_I_DPB:
	{
		struct ddl_property_dec_pic_buffers *dpb =
		(struct ddl_property_dec_pic_buffers *) property_value;

		if ((sizeof(struct ddl_property_dec_pic_buffers) ==
			property_hdr->sz) &&
			(DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_INITCODEC) ||
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPB)) &&
			(dpb->no_of_dec_pic_buf ==
			decoder->client_output_buf_req.actual_count))
			vcd_status = ddl_set_dec_buffers(decoder, dpb);
	}
	break;
	case DDL_I_REQ_OUTPUT_FLUSH:
		if (sizeof(u32) == property_hdr->sz) {
			decoder->dynamic_prop_change |=
				DDL_DEC_REQ_OUTPUT_FLUSH;
			decoder->dpb_mask.client_mask = 0;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_INPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buffer_req =
			(struct vcd_buffer_requirement *)property_value;

		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz &&
			(DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_INITCODEC) ||
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPB) ||
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) &&
			(ddl_valid_buffer_requirement(
			&decoder->min_input_buf_req, buffer_req))) {
			decoder->client_input_buf_req = *buffer_req;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case DDL_I_OUTPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buffer_req =
			(struct vcd_buffer_requirement *)property_value;

		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz &&
			(DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_INITCODEC) ||
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPB) ||
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) &&
			(ddl_valid_buffer_requirement(
			&decoder->min_output_buf_req, buffer_req))) {
				decoder->client_output_buf_req =
					*buffer_req;
				vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_CODEC:
	{
		struct vcd_property_codec *codec =
			(struct vcd_property_codec *)property_value;
		if (sizeof(struct vcd_property_codec) ==
			property_hdr->sz &&
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) &&
			ddl_codec_type_transact(ddl, false,
			codec->codec)) {
			if (decoder->codec.codec != codec->codec) {
				decoder->codec = *codec;
				ddl_set_default_dec_property(ddl);
			}
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_POST_FILTER:
		if (sizeof(struct vcd_property_post_filter) ==
			property_hdr->sz &&
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) && (
			decoder->codec.codec == VCD_CODEC_MPEG4 ||
			decoder->codec.codec == VCD_CODEC_MPEG2)) {
			decoder->post_filter =
				*(struct vcd_property_post_filter *)
				property_value;
			vcd_status = VCD_S_SUCCESS;
	}
	break;
	case VCD_I_FRAME_SIZE:
	{
		struct vcd_property_frame_size *frame_size =
		(struct vcd_property_frame_size *) property_value;
		if ((sizeof(struct vcd_property_frame_size) ==
			property_hdr->sz) &&
			(DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) &&
			(DDL_ALLOW_DEC_FRAMESIZE(frame_size->width,
			frame_size->height))) {
			if (decoder->client_frame_size.height !=
				frame_size->height ||
				decoder->client_frame_size.width !=
				frame_size->width) {
				decoder->client_frame_size = *frame_size;
				ddl_set_default_decoder_buffer_req(decoder,
					true);
			}
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_BUFFER_FORMAT:
	{
		struct vcd_property_buffer_format *tile =
			(struct vcd_property_buffer_format *)
			property_value;
		if (sizeof(struct vcd_property_buffer_format) ==
			property_hdr->sz &&
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) &&
			tile->buffer_format == VCD_BUFFER_FORMAT_TILE_4x2) {
			if (tile->buffer_format !=
				decoder->buf_format.buffer_format) {
				decoder->buf_format = *tile;
				ddl_set_default_decoder_buffer_req(
					decoder, true);
			}
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_H264_MV_BUFFER:
	{
		int index, buffer_size;
		u8 *phys_addr;
		u8 *virt_addr;
		struct vcd_property_h264_mv_buffer *mv_buff =
			(struct vcd_property_h264_mv_buffer *)
		property_value;
		DDL_MSG_LOW("Entered VCD_I_H264_MV_BUFFER Virt: %p, Phys %p,"
					"fd: %d size: %d count: %d\n",
					mv_buff->kernel_virtual_addr,
					mv_buff->physical_addr,
					mv_buff->pmem_fd,
					mv_buff->size, mv_buff->count);
		if ((property_hdr->sz == sizeof(struct
			vcd_property_h264_mv_buffer)) &&
			(DDLCLIENT_STATE_IS(ddl,
			DDL_CLIENT_WAIT_FOR_INITCODEC) ||
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPB) ||
			DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN))) {
			phys_addr = mv_buff->physical_addr;
			virt_addr = mv_buff->kernel_virtual_addr;
			buffer_size = mv_buff->size/mv_buff->count;

			for (index = 0; index < mv_buff->count; index++) {
				ddl->codec_data.decoder.hw_bufs.
					h264_mv[index].align_physical_addr
					= phys_addr;
				ddl->codec_data.decoder.hw_bufs.
					h264_mv[index].align_virtual_addr
					= virt_addr;
				ddl->codec_data.decoder.hw_bufs.
					h264_mv[index].buffer_size
					= buffer_size;
				ddl->codec_data.decoder.hw_bufs.
					h264_mv[index].physical_base_addr
					= phys_addr;
				ddl->codec_data.decoder.hw_bufs.
					h264_mv[index].virtual_base_addr
					= virt_addr;
				DDL_MSG_LOW("Assigned %d buffer for "
							"virt: %p, phys %p for "
							"h264_mv_buffers "
							"of size: %d\n",
							index, virt_addr,
							phys_addr, buffer_size);
				phys_addr += buffer_size;
				virt_addr += buffer_size;
			}
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_FREE_H264_MV_BUFFER:
		{
			memset(&decoder->hw_bufs.h264_mv, 0, sizeof(struct
					ddl_buf_addr) * DDL_MAX_BUFFER_COUNT);
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		DDL_MSG_ERROR("Meta Data Interface is Not supported");
		vcd_status = VCD_S_SUCCESS;
		break;
	case VCD_I_FRAME_RATE:
		vcd_status = VCD_S_SUCCESS;
		break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

static u32 ddl_check_valid_enc_level(struct vcd_property_codec *codec,
	struct vcd_property_profile *profile,
	struct vcd_property_level *level)
{
	u32 status = false;

	if (codec && profile && level) {
		switch (codec->codec) {
		case VCD_CODEC_MPEG4:
			status = (profile->profile ==
				VCD_PROFILE_MPEG4_SP) &&
				(level->level >= VCD_LEVEL_MPEG4_0) &&
				(level->level <= VCD_LEVEL_MPEG4_7) &&
				(VCD_LEVEL_MPEG4_3b != level->level);
			status = status ||
				((profile->profile ==
				VCD_PROFILE_MPEG4_ASP) &&
				(level->level >= VCD_LEVEL_MPEG4_0) &&
				(level->level <= VCD_LEVEL_MPEG4_5));
		break;
		case VCD_CODEC_H264:
		status = (level->level >= VCD_LEVEL_H264_1) &&
				(level->level <= VCD_LEVEL_H264_4);
		break;
		case VCD_CODEC_H263:
		status = (level->level >= VCD_LEVEL_H263_10) &&
			(level->level <= VCD_LEVEL_H263_70);
		break;
		default:
		break;
		}
	}
	return status;
}

static u32 ddl_set_enc_property(struct ddl_client_context *ddl,
	struct vcd_property_hdr *property_hdr,
	void *property_value)
{
	struct ddl_encoder_data *encoder =
		&(ddl->codec_data.encoder);
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;

	if (DDLCLIENT_STATE_IS(ddl,
		DDL_CLIENT_WAIT_FOR_FRAME)) {
		vcd_status = ddl_set_enc_dynamic_property(encoder,
				property_hdr, property_value);
		return vcd_status;
	}
	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) {
		DDL_MSG_ERROR("ddl_set_enc_property:\
			Fails_as_not_in_open_state");
		return VCD_ERR_ILLEGAL_OP;
	}
	switch (property_hdr->prop_id) {
	case VCD_I_TARGET_BITRATE:
	{
		struct vcd_property_target_bitrate *bitrate =
		(struct vcd_property_target_bitrate *)property_value;
		if (sizeof(struct vcd_property_target_bitrate) ==
			property_hdr->sz && bitrate->target_bitrate) {
			encoder->target_bit_rate = *bitrate;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_FRAME_RATE:
	{
		struct vcd_property_frame_rate *frame_rate =
		(struct vcd_property_frame_rate *) property_value;
		if (sizeof(struct vcd_property_frame_rate) ==
			property_hdr->sz &&
			frame_rate->fps_denominator &&
			frame_rate->fps_numerator) {
			encoder->frame_rate = *frame_rate;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_FRAME_SIZE:
	{
		struct vcd_property_frame_size *frame_size =
		(struct vcd_property_frame_size *) property_value;
		if ((sizeof(struct vcd_property_frame_size) ==
			property_hdr->sz) &&
			(DDL_ALLOW_ENC_FRAMESIZE(frame_size->width,
			frame_size->height))) {
			ddl_calculate_stride(frame_size, false);
			encoder->frame_size = *frame_size;
			ddl_set_default_encoder_buffer_req(encoder);
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_CODEC:
	{
		struct vcd_property_codec *codec =
		(struct vcd_property_codec *) property_value;
		if ((sizeof(struct vcd_property_codec) ==
		property_hdr->sz) &&
		(ddl_codec_type_transact(ddl, false, codec->codec))) {
			if (codec->codec != encoder->codec.codec) {
				encoder->codec = *codec;
				ddl_set_default_enc_property(ddl);
			}
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_REQ_IFRAME:
		vcd_status = VCD_S_SUCCESS;
		break;
	case VCD_I_INTRA_PERIOD:
	{
		struct vcd_property_i_period *i_period =
			(struct vcd_property_i_period *)property_value;
		if (sizeof(struct vcd_property_i_period) ==
			property_hdr->sz &&
			i_period->b_frames <= DDL_MAX_NUM_OF_B_FRAME) {
			encoder->i_period = *i_period;
			encoder->client_input_buf_req.min_count =
				i_period->b_frames + 1;
			encoder->client_input_buf_req.actual_count =
				DDL_MAX(encoder->client_input_buf_req.\
				actual_count, encoder->\
				client_input_buf_req.min_count);
			encoder->client_output_buf_req.min_count =
				i_period->b_frames + 2;
			encoder->client_output_buf_req.actual_count =
				DDL_MAX(encoder->client_output_buf_req.\
				actual_count, encoder->\
				client_output_buf_req.min_count);
			ddl->extra_output_buf_count =
				i_period->b_frames - 1;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_PROFILE:
	{
		struct vcd_property_profile *profile =
			(struct vcd_property_profile *)property_value;

		if ((sizeof(struct vcd_property_profile) ==
			property_hdr->sz) && ((
			(encoder->codec.codec == VCD_CODEC_MPEG4) && (
			profile->profile == VCD_PROFILE_MPEG4_SP ||
			profile->profile == VCD_PROFILE_MPEG4_ASP)) ||
			((encoder->codec.codec == VCD_CODEC_H264) &&
			(profile->profile >= VCD_PROFILE_H264_BASELINE) &&
			(profile->profile <= VCD_PROFILE_H264_HIGH)) ||
			((encoder->codec.codec == VCD_CODEC_H263) &&
			(profile->profile == VCD_PROFILE_H263_BASELINE)))) {
			encoder->profile = *profile;
			vcd_status = VCD_S_SUCCESS;
			if (profile->profile == VCD_PROFILE_H264_BASELINE)
				encoder->entropy_control.entropy_sel =
					VCD_ENTROPY_SEL_CAVLC;
			else
				encoder->entropy_control.entropy_sel =
					VCD_ENTROPY_SEL_CABAC;
		}
	}
	break;
	case VCD_I_LEVEL:
	{
		struct vcd_property_level *level =
			(struct vcd_property_level *) property_value;

		if ((sizeof(struct vcd_property_level) ==
			property_hdr->sz) && (ddl_check_valid_enc_level
			(&encoder->codec,
			&encoder->profile, level))) {
			encoder->level = *level;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_MULTI_SLICE:
	{
		struct vcd_property_multi_slice *multi_slice =
			(struct vcd_property_multi_slice *)
				property_value;

		switch (multi_slice->m_slice_sel) {
		case VCD_MSLICE_OFF:
			vcd_status = VCD_S_SUCCESS;
		break;
		case VCD_MSLICE_BY_GOB:
			if (encoder->codec.codec == VCD_CODEC_H263)
				vcd_status = VCD_S_SUCCESS;
		break;
		case VCD_MSLICE_BY_MB_COUNT:
			if (multi_slice->m_slice_size >= 1 &&
				(multi_slice->m_slice_size <=
				DDL_NO_OF_MB(encoder->frame_size.width,
				encoder->frame_size.height)))
				vcd_status = VCD_S_SUCCESS;
		break;
		case VCD_MSLICE_BY_BYTE_COUNT:
			if (multi_slice->m_slice_size >=
				DDL_MINIMUM_BYTE_PER_SLICE)
				vcd_status = VCD_S_SUCCESS;
		break;
		default:
		break;
		}
		if (sizeof(struct vcd_property_multi_slice) ==
			property_hdr->sz && !vcd_status)
			encoder->multi_slice = *multi_slice;
	}
	break;
	case VCD_I_RATE_CONTROL:
	{
		struct vcd_property_rate_control *rate_control =
			(struct vcd_property_rate_control *)
			property_value;
		if (sizeof(struct vcd_property_rate_control) ==
			property_hdr->sz &&
			rate_control->rate_control >=
			VCD_RATE_CONTROL_OFF &&
			rate_control->rate_control <=
			VCD_RATE_CONTROL_CBR_CFR) {
			encoder->rc = *rate_control;
			ddl_set_default_enc_rc_params(encoder);
			vcd_status = VCD_S_SUCCESS;
		}

	}
	break;
	case VCD_I_SHORT_HEADER:
		if (sizeof(struct vcd_property_short_header) ==
			property_hdr->sz &&
			encoder->codec.codec ==
			VCD_CODEC_MPEG4) {
			encoder->short_header =
			*(struct vcd_property_short_header *)
				property_value;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_VOP_TIMING:
	{
		struct vcd_property_vop_timing *vop_time =
			(struct vcd_property_vop_timing *)
				property_value;

		if ((sizeof(struct vcd_property_vop_timing) ==
			property_hdr->sz) &&
			(encoder->frame_rate.fps_numerator <=
			vop_time->vop_time_resolution) &&
			(encoder->codec.codec == VCD_CODEC_MPEG4)) {
			encoder->vop_timing = *vop_time;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_HEADER_EXTENSION:
		if (sizeof(u32) == property_hdr->sz &&
			encoder->codec.codec == VCD_CODEC_MPEG4) {
			encoder->hdr_ext_control = *(u32 *)property_value;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_ENTROPY_CTRL:
	{
		struct vcd_property_entropy_control *entropy_control =
			(struct vcd_property_entropy_control *)
			property_value;
		if (sizeof(struct vcd_property_entropy_control) ==
			property_hdr->sz &&
			encoder->codec.codec == VCD_CODEC_H264 &&
			entropy_control->entropy_sel >=
			VCD_ENTROPY_SEL_CAVLC &&
			entropy_control->entropy_sel <=
			VCD_ENTROPY_SEL_CABAC) {
			encoder->entropy_control = *entropy_control;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_DEBLOCKING:
	{
		struct vcd_property_db_config *db_config =
			(struct vcd_property_db_config *) property_value;
		if (sizeof(struct vcd_property_db_config) ==
			property_hdr->sz  &&
			encoder->codec.codec == VCD_CODEC_H264 &&
			db_config->db_config >=
			VCD_DB_ALL_BLOCKING_BOUNDARY &&
			db_config->db_config <=
			VCD_DB_SKIP_SLICE_BOUNDARY) {
			encoder->db_control = *db_config;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_QP_RANGE:
	{
		struct vcd_property_qp_range *qp =
			(struct vcd_property_qp_range *)property_value;

		if ((sizeof(struct vcd_property_qp_range) ==
			property_hdr->sz) && (qp->min_qp <=
			qp->max_qp) && ((encoder->codec.codec ==
			VCD_CODEC_H264 && qp->max_qp <= DDL_MAX_H264_QP) ||
			(qp->max_qp <= DDL_MAX_MPEG4_QP))) {
			encoder->qp_range = *qp;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_SESSION_QP:
	{
		struct vcd_property_session_qp *qp =
			(struct vcd_property_session_qp *)property_value;
		if ((sizeof(struct vcd_property_session_qp) ==
			property_hdr->sz) &&
			(qp->i_frame_qp >= encoder->qp_range.min_qp) &&
			(qp->i_frame_qp <= encoder->qp_range.max_qp) &&
			(qp->p_frame_qp >= encoder->qp_range.min_qp) &&
			(qp->p_frame_qp <= encoder->qp_range.max_qp)) {
			encoder->session_qp = *qp;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_RC_LEVEL_CONFIG:
	{
		struct vcd_property_rc_level *rc_level =
			(struct vcd_property_rc_level *) property_value;
		if (sizeof(struct vcd_property_rc_level) ==
			property_hdr->sz &&
			(encoder->rc.rate_control >=
			VCD_RATE_CONTROL_VBR_VFR ||
			encoder->rc.rate_control <=
			VCD_RATE_CONTROL_CBR_VFR) &&
			(!rc_level->mb_level_rc ||
			encoder->codec.codec == VCD_CODEC_H264)) {
			encoder->rc_level = *rc_level;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_FRAME_LEVEL_RC:
	{
		struct vcd_property_frame_level_rc_params
			*frame_level_rc =
			(struct vcd_property_frame_level_rc_params *)
			property_value;
		if ((sizeof(struct vcd_property_frame_level_rc_params) ==
			property_hdr->sz) &&
			(frame_level_rc->reaction_coeff) &&
			(encoder->rc_level.frame_level_rc)) {
			encoder->frame_level_rc = *frame_level_rc;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_ADAPTIVE_RC:
		if ((sizeof(struct vcd_property_adaptive_rc_params) ==
			property_hdr->sz) &&
			(encoder->codec.codec == VCD_CODEC_H264) &&
			(encoder->rc_level.mb_level_rc)) {
			encoder->adaptive_rc =
				*(struct vcd_property_adaptive_rc_params *)
				property_value;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_INTRA_REFRESH:
	{
		struct vcd_property_intra_refresh_mb_number
			*intra_refresh_mb_num =
			(struct vcd_property_intra_refresh_mb_number *)
			property_value;
		u32 frame_mb_num = DDL_NO_OF_MB(encoder->frame_size.width,
					encoder->frame_size.height);

		if (sizeof(struct vcd_property_intra_refresh_mb_number) ==
			property_hdr->sz &&
			intra_refresh_mb_num->cir_mb_number <=
			frame_mb_num) {
			encoder->intra_refresh = *intra_refresh_mb_num;
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	}
	case VCD_I_BUFFER_FORMAT:
	{
		struct vcd_property_buffer_format *buffer_format =
			(struct vcd_property_buffer_format *)
			property_value;

		if (sizeof(struct vcd_property_buffer_format) ==
			property_hdr->sz &&
			((buffer_format->buffer_format ==
			VCD_BUFFER_FORMAT_NV12_16M2KA) ||
			(VCD_BUFFER_FORMAT_TILE_4x2 ==
			buffer_format->buffer_format))) {
			encoder->buf_format = *buffer_format;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case DDL_I_INPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buffer_req =
			(struct vcd_buffer_requirement *)property_value;
		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz && (ddl_valid_buffer_requirement(
			&encoder->input_buf_req, buffer_req))) {
			encoder->client_input_buf_req = *buffer_req;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case DDL_I_OUTPUT_BUF_REQ:
	{
		struct vcd_buffer_requirement *buffer_req =
			(struct vcd_buffer_requirement *)property_value;
		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz && (ddl_valid_buffer_requirement(
			&encoder->output_buf_req, buffer_req))) {
			encoder->client_output_buf_req = *buffer_req;
			encoder->client_output_buf_req.sz =
				DDL_ALIGN(buffer_req->sz,
				DDL_KILO_BYTE(4));
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_RECON_BUFFERS:
	{
		int index;
		struct vcd_property_enc_recon_buffer *recon_buffers =
			(struct vcd_property_enc_recon_buffer *)property_value;
		for (index = 0; index < 4; index++) {
			if (!encoder->hw_bufs.dpb_y[index].align_physical_addr)
				break;
			else
				continue;
			}
		if (property_hdr->sz == sizeof(struct
			vcd_property_enc_recon_buffer)) {
			encoder->hw_bufs.dpb_y[index].align_physical_addr =
				recon_buffers->physical_addr;
			encoder->hw_bufs.dpb_y[index].align_virtual_addr =
				recon_buffers->kernel_virtual_addr;
			encoder->hw_bufs.dpb_y[index].buffer_size =
				recon_buffers->buffer_size;
			encoder->hw_bufs.dpb_c[index].align_physical_addr =
			recon_buffers->physical_addr + ddl_get_yuv_buf_size(
				encoder->frame_size.width, encoder->frame_size.
				height, DDL_YUV_BUF_TYPE_TILE);
			encoder->hw_bufs.dpb_c[index].align_virtual_addr =
				recon_buffers->kernel_virtual_addr +
				recon_buffers->ysize;
			DDL_MSG_LOW("Y::KVirt: %p,KPhys: %p"
						"UV::KVirt: %p,KPhys: %p\n",
			encoder->hw_bufs.dpb_y[index].align_virtual_addr,
			encoder->hw_bufs.dpb_y[index].align_physical_addr,
			encoder->hw_bufs.dpb_c[index].align_virtual_addr,
			encoder->hw_bufs.dpb_c[index].align_physical_addr);
			vcd_status = VCD_S_SUCCESS;
			}
	}
	break;
	case VCD_I_FREE_RECON_BUFFERS:
	{
		memset(&encoder->hw_bufs.dpb_y, 0,
			sizeof(struct ddl_buf_addr) * 4);
		memset(&encoder->hw_bufs.dpb_c, 0,
			sizeof(struct ddl_buf_addr) * 4);
		vcd_status = VCD_S_SUCCESS;
		break;
	}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		DDL_MSG_ERROR("Meta Data Interface is Not supported");
		vcd_status = VCD_S_SUCCESS;
	break;
	default:
		DDL_MSG_ERROR("INVALID ID %d\n", (int)property_hdr->prop_id);
		vcd_status = VCD_ERR_ILLEGAL_OP;
	break;
	}
	return vcd_status;
}

static u32 ddl_get_dec_property(struct ddl_client_context *ddl,
	struct vcd_property_hdr *property_hdr, void *property_value)
{
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	DDL_MSG_HIGH("property_hdr->prop_id:%x\n", property_hdr->prop_id);
	switch (property_hdr->prop_id) {
	case VCD_I_FRAME_SIZE:
		if (sizeof(struct vcd_property_frame_size) ==
			property_hdr->sz) {
			ddl_calculate_stride(&decoder->client_frame_size,
				!decoder->progressive_only);
			*(struct vcd_property_frame_size *)
				property_value =
					decoder->client_frame_size;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_PROFILE:
		if (sizeof(struct vcd_property_profile) ==
			property_hdr->sz) {
			*(struct vcd_property_profile *)property_value =
				decoder->profile;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_LEVEL:
		if (sizeof(struct vcd_property_level) ==
			property_hdr->sz) {
			*(struct vcd_property_level *)property_value =
				decoder->level;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_PROGRESSIVE_ONLY:
		if (sizeof(u32) == property_hdr->sz) {
			*(u32 *)property_value =
				decoder->progressive_only;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_INPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz) {
			*(struct vcd_buffer_requirement *)
				property_value =
					decoder->client_input_buf_req;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_OUTPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz) {
			if (decoder->client_output_buf_req.sz) {
				*(struct vcd_buffer_requirement *)
					property_value =
					decoder->client_output_buf_req;
				vcd_status = VCD_S_SUCCESS;
			} else
				vcd_status = VCD_ERR_ILLEGAL_OP;
		}
	break;
	case VCD_I_CODEC:
	if (sizeof(struct vcd_property_codec) ==
		property_hdr->sz) {
		if (decoder->codec.codec) {
			*(struct vcd_property_codec *) property_value =
				decoder->codec;
			vcd_status = VCD_S_SUCCESS;
		} else
			vcd_status = VCD_ERR_ILLEGAL_OP;
	}
	break;
	case VCD_I_BUFFER_FORMAT:
		if (sizeof(struct vcd_property_buffer_format) ==
			property_hdr->sz) {
			*(struct vcd_property_buffer_format *)
				property_value = decoder->buf_format;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_POST_FILTER:
		if (sizeof(struct vcd_property_post_filter) ==
			property_hdr->sz) {
			*(struct vcd_property_post_filter *)
				property_value =
					decoder->post_filter;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_SEQHDR_ALIGN_BYTES:
		if (sizeof(u32) == property_hdr->sz) {
			*(u32 *)property_value =
				DDL_LINEAR_BUFFER_ALIGN_BYTES;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_FRAME_PROC_UNITS:
		if (sizeof(u32) == property_hdr->sz &&
			decoder->client_frame_size.width &&
			decoder->client_frame_size.height) {
			*(u32 *) property_value = DDL_NO_OF_MB(
				decoder->client_frame_size.width,
				decoder->client_frame_size.height);
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_DPB_RETRIEVE:
		if (sizeof(struct ddl_frame_data_tag) ==
			property_hdr->sz) {
			vcd_status = ddl_decoder_dpb_transact(decoder,
				(struct ddl_frame_data_tag *)
				property_value, DDL_DPB_OP_RETRIEVE);
		}
	break;
	case VCD_I_GET_H264_MV_SIZE:
		if (property_hdr->sz == sizeof(struct
			vcd_property_buffer_size)) {
			struct vcd_property_buffer_size *mv_size =
			(struct vcd_property_buffer_size *) property_value;
			mv_size->size = ddl_get_yuv_buf_size(mv_size->width,
				mv_size->height, DDL_YUV_BUF_TYPE_TILE);
			mv_size->alignment = DDL_TILE_BUFFER_ALIGN_BYTES;
			DDL_MSG_LOW("w: %d, h: %d, S: %d, "
						"A: %d", mv_size->width,
						mv_size->height, mv_size->size,
						mv_size->alignment);
			vcd_status = VCD_S_SUCCESS;
		}
		break;
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		DDL_MSG_ERROR("Meta Data Interface is Not supported");
		vcd_status = VCD_S_SUCCESS;
	break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
	break;
	}
	return vcd_status;
}

static u32 ddl_get_enc_property(struct ddl_client_context *ddl,
	struct vcd_property_hdr *property_hdr, void *property_value)
{
	struct ddl_encoder_data *encoder = &ddl->codec_data.encoder;
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;

	switch (property_hdr->prop_id) {
	case VCD_I_CODEC:
		if (sizeof(struct vcd_property_codec) ==
			property_hdr->sz) {
			*(struct vcd_property_codec *) property_value =
				encoder->codec;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_FRAME_SIZE:
		if (sizeof(struct vcd_property_frame_size) ==
			property_hdr->sz) {
			*(struct vcd_property_frame_size *)
				property_value = encoder->frame_size;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_FRAME_RATE:
		if (sizeof(struct vcd_property_frame_rate) ==
			property_hdr->sz) {
			*(struct vcd_property_frame_rate *)
				property_value = encoder->frame_rate;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_TARGET_BITRATE:
		if (sizeof(struct vcd_property_target_bitrate) ==
			property_hdr->sz) {
			*(struct vcd_property_target_bitrate *)
				property_value = encoder->target_bit_rate;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_RATE_CONTROL:
		if (sizeof(struct vcd_property_rate_control) ==
			property_hdr->sz) {
			*(struct vcd_property_rate_control *)
				property_value = encoder->rc;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_PROFILE:
		if (sizeof(struct vcd_property_profile) ==
			property_hdr->sz) {
			*(struct vcd_property_profile *) property_value =
				encoder->profile;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_LEVEL:
		if (sizeof(struct vcd_property_level) ==
			property_hdr->sz) {
			*(struct vcd_property_level *) property_value =
				encoder->level;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_MULTI_SLICE:
		if (sizeof(struct vcd_property_multi_slice) ==
			property_hdr->sz) {
			*(struct vcd_property_multi_slice *)
				property_value = encoder->multi_slice;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_SEQ_HEADER:
	{
		struct vcd_sequence_hdr *seq_hdr =
			(struct vcd_sequence_hdr *) property_value;

		if (!encoder->seq_header_length) {
			seq_hdr->sequence_header_len =
				encoder->seq_header_length;
			vcd_status = VCD_ERR_NO_SEQ_HDR;
		} else if (sizeof(struct vcd_sequence_hdr) ==
			property_hdr->sz &&
			encoder->seq_header_length <=
			seq_hdr->sequence_header_len) {
			memcpy(seq_hdr->sequence_header,
				encoder->seq_header.align_virtual_addr,
				encoder->seq_header_length);
			seq_hdr->sequence_header_len =
				encoder->seq_header_length;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case DDL_I_SEQHDR_PRESENT:
		if (sizeof(u32) == property_hdr->sz) {
			if ((encoder->codec.codec ==
				VCD_CODEC_MPEG4 &&
				!encoder->short_header.short_header) ||
				encoder->codec.codec == VCD_CODEC_H264)
				*(u32 *) property_value = 0x1;
			else
				*(u32 *) property_value = 0x0;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_VOP_TIMING:
		if (sizeof(struct vcd_property_vop_timing) ==
			property_hdr->sz) {
			*(struct vcd_property_vop_timing *)
				property_value = encoder->vop_timing;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_SHORT_HEADER:
		if (sizeof(struct vcd_property_short_header) ==
			property_hdr->sz) {
			if (encoder->codec.codec == VCD_CODEC_MPEG4) {
				*(struct vcd_property_short_header *)
					property_value =
						encoder->short_header;
				vcd_status = VCD_S_SUCCESS;
			} else
				vcd_status = VCD_ERR_ILLEGAL_OP;
		}
	break;
	case VCD_I_ENTROPY_CTRL:
		if (sizeof(struct vcd_property_entropy_control) ==
			property_hdr->sz) {
			if (encoder->codec.codec == VCD_CODEC_H264) {
				*(struct vcd_property_entropy_control *)
					property_value =
						encoder->entropy_control;
				vcd_status = VCD_S_SUCCESS;
			} else
				vcd_status = VCD_ERR_ILLEGAL_OP;
		}
	break;
	case VCD_I_DEBLOCKING:
		if (sizeof(struct vcd_property_db_config) ==
			property_hdr->sz) {
			if (encoder->codec.codec == VCD_CODEC_H264) {
				*(struct vcd_property_db_config *)
					property_value =
						encoder->db_control;
				vcd_status = VCD_S_SUCCESS;
			} else
				vcd_status = VCD_ERR_ILLEGAL_OP;
		}
	break;
	case VCD_I_INTRA_PERIOD:
		if (sizeof(struct vcd_property_i_period) ==
			property_hdr->sz) {
			*(struct vcd_property_i_period *)
				property_value = encoder->i_period;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_QP_RANGE:
		if (sizeof(struct vcd_property_qp_range) ==
			property_hdr->sz) {
			*(struct vcd_property_qp_range *)
				property_value = encoder->qp_range;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_SESSION_QP:
		if (sizeof(struct vcd_property_session_qp) ==
			property_hdr->sz) {
			*(struct vcd_property_session_qp *)
				property_value = encoder->session_qp;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_RC_LEVEL_CONFIG:
		if (sizeof(struct vcd_property_rc_level) ==
			property_hdr->sz) {
			*(struct vcd_property_rc_level *)
				property_value = encoder->rc_level;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_FRAME_LEVEL_RC:
		if (sizeof(struct vcd_property_frame_level_rc_params) ==
			property_hdr->sz) {
			*(struct vcd_property_frame_level_rc_params *)
			property_value = encoder->frame_level_rc;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_ADAPTIVE_RC:
		if (sizeof(struct vcd_property_adaptive_rc_params) ==
			property_hdr->sz) {
			*(struct vcd_property_adaptive_rc_params *)
				property_value = encoder->adaptive_rc;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_INTRA_REFRESH:
		if (sizeof(struct vcd_property_intra_refresh_mb_number) ==
			property_hdr->sz) {
			*(struct vcd_property_intra_refresh_mb_number *)
				property_value = encoder->intra_refresh;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_INPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz) {
			if (encoder->input_buf_req.sz) {
				*(struct vcd_buffer_requirement *)
					property_value =
						encoder->client_input_buf_req;
				vcd_status = VCD_S_SUCCESS;
			} else
				vcd_status = VCD_ERR_ILLEGAL_OP;
		}
	break;
	case DDL_I_OUTPUT_BUF_REQ:
		if (sizeof(struct vcd_buffer_requirement) ==
			property_hdr->sz) {
			if (encoder->output_buf_req.sz) {
				*(struct vcd_buffer_requirement *)
					property_value =
					encoder->client_output_buf_req;
				vcd_status = VCD_S_SUCCESS;
			} else
				vcd_status = VCD_ERR_ILLEGAL_OP;
		}
	break;
	case VCD_I_BUFFER_FORMAT:
		if (sizeof(struct vcd_property_buffer_format) ==
			property_hdr->sz) {
			*(struct vcd_property_buffer_format *)
				property_value =
			encoder->buf_format;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case DDL_I_FRAME_PROC_UNITS:
		if (sizeof(u32) == property_hdr->sz &&
			encoder->frame_size.width &&
			encoder->frame_size.height) {
			*(u32 *)property_value = DDL_NO_OF_MB(
				encoder->frame_size.width,
				encoder->frame_size.height);
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_HEADER_EXTENSION:
		if (sizeof(u32) == property_hdr->sz &&
			encoder->codec.codec == VCD_CODEC_MPEG4) {
			*(u32 *) property_value =
				encoder->hdr_ext_control;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_GET_RECON_BUFFER_SIZE:
	{
		u32 ysize, uvsize;
		if (property_hdr->sz == sizeof(struct
			vcd_property_buffer_size)) {
			struct vcd_property_buffer_size *recon_buff_size =
			(struct vcd_property_buffer_size *) property_value;

			ysize = ddl_get_yuv_buf_size(recon_buff_size->width,
				recon_buff_size->height, DDL_YUV_BUF_TYPE_TILE);
			uvsize = ddl_get_yuv_buf_size(recon_buff_size->width,
				recon_buff_size->height/2,
				DDL_YUV_BUF_TYPE_TILE);
			recon_buff_size->size = ysize + uvsize;
			recon_buff_size->alignment =
				DDL_TILE_BUFFER_ALIGN_BYTES;
			DDL_MSG_LOW("w: %d, h: %d, S: %d, A: %d",
			recon_buff_size->width, recon_buff_size->height,
			recon_buff_size->size, recon_buff_size->alignment);
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		DDL_MSG_ERROR("Meta Data Interface is Not supported");
		vcd_status = VCD_S_SUCCESS;
	break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

static u32 ddl_set_enc_dynamic_property(struct ddl_encoder_data
	*encoder, struct vcd_property_hdr *property_hdr,
	void *property_value)
{
	u32  vcd_status = VCD_ERR_ILLEGAL_PARM;

	switch (property_hdr->prop_id) {
	case VCD_I_REQ_IFRAME:
		if (sizeof(struct vcd_property_req_i_frame) ==
			property_hdr->sz) {
			encoder->dynamic_prop_change |= DDL_ENC_REQ_IFRAME;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_TARGET_BITRATE:
		if (sizeof(struct vcd_property_target_bitrate) ==
			property_hdr->sz) {
			encoder->target_bit_rate =
				*(struct vcd_property_target_bitrate *)
				property_value;
			encoder->dynamic_prop_change |=
				DDL_ENC_CHANGE_BITRATE;
			vcd_status = VCD_S_SUCCESS;
		}
	break;
	case VCD_I_INTRA_PERIOD:
	{
		struct vcd_property_i_period *i_period =
			(struct vcd_property_i_period *)property_value;

		if (sizeof(struct vcd_property_i_period) ==
			property_hdr->sz) {
			encoder->i_period = *i_period;
			encoder->dynamic_prop_change |=
				DDL_ENC_CHANGE_IPERIOD;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	case VCD_I_FRAME_RATE:
	{
		struct vcd_property_frame_rate *frame_rate =
			(struct vcd_property_frame_rate *)
			property_value;
		if (sizeof(struct vcd_property_frame_rate) ==
			property_hdr->sz &&
			frame_rate->fps_denominator &&
			frame_rate->fps_numerator &&
			frame_rate->fps_denominator <=
			frame_rate->fps_numerator) {
			encoder->frame_rate = *frame_rate;
			encoder->dynamic_prop_change |=
				DDL_ENC_CHANGE_FRAMERATE;
			vcd_status = VCD_S_SUCCESS;
		}
	}
	break;
	default:
		vcd_status = VCD_ERR_ILLEGAL_OP;
		break;
	}
	return vcd_status;
}

void ddl_set_default_dec_property(struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *decoder =
		&(ddl->codec_data.decoder);

	if (decoder->codec.codec == VCD_CODEC_MPEG4 ||
		decoder->codec.codec == VCD_CODEC_MPEG2)
		decoder->post_filter.post_filter = true;
	else
		decoder->post_filter.post_filter = false;
	decoder->buf_format.buffer_format = VCD_BUFFER_FORMAT_TILE_4x2;
	decoder->client_frame_size.height = VCD_DDL_TEST_DEFAULT_HEIGHT;
	decoder->client_frame_size.width  = VCD_DDL_TEST_DEFAULT_WIDTH;
	decoder->client_frame_size.stride = VCD_DDL_TEST_DEFAULT_WIDTH;
	decoder->client_frame_size.scan_lines = VCD_DDL_TEST_DEFAULT_HEIGHT;
	decoder->progressive_only = 1;

	decoder->meta_data_enable_flag = 0;
	ddl_set_default_decoder_buffer_req(decoder, true);
}

static void ddl_set_default_enc_property(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);

	ddl_set_default_enc_profile(encoder);
	ddl_set_default_enc_level(encoder);
	encoder->rc.rate_control = VCD_RATE_CONTROL_VBR_VFR;
	ddl_set_default_enc_rc_params(encoder);
	ddl_set_default_enc_intra_period(encoder);
	encoder->intra_refresh.cir_mb_number = 0;
	ddl_set_default_enc_vop_timing(encoder);
	encoder->multi_slice.m_slice_sel = VCD_MSLICE_OFF;
	ddl->b_count = 0;
	encoder->short_header.short_header    = false;
	encoder->entropy_control.entropy_sel  = VCD_ENTROPY_SEL_CAVLC;
	encoder->entropy_control.cabac_model  = VCD_CABAC_MODEL_NUMBER_0;
	encoder->db_control.db_config         =
		VCD_DB_ALL_BLOCKING_BOUNDARY;
	encoder->db_control.slice_alpha_offset = 0;
	encoder->db_control.slice_beta_offset = 0;
	encoder->re_con_buf_format.buffer_format =
		VCD_BUFFER_FORMAT_TILE_1x1;
	encoder->buf_format.buffer_format = VCD_BUFFER_FORMAT_NV12_16M2KA;
	encoder->hdr_ext_control = 0;
	encoder->mb_info_enable  = false;
	encoder->num_references_for_p_frame = DDL_MIN_NUM_REF_FOR_P_FRAME;
	encoder->meta_data_enable_flag = 0;
	ddl_set_default_encoder_buffer_req(encoder);
}

static void ddl_set_default_enc_profile(struct ddl_encoder_data *encoder)
{
	enum vcd_codec codec = encoder->codec.codec;

	if (codec == VCD_CODEC_MPEG4)
		encoder->profile.profile = VCD_PROFILE_MPEG4_SP;
	else if (codec == VCD_CODEC_H264)
		encoder->profile.profile = VCD_PROFILE_H264_BASELINE;
	else
		encoder->profile.profile = VCD_PROFILE_H263_BASELINE;
}

static void ddl_set_default_enc_level(struct ddl_encoder_data *encoder)
{
	enum vcd_codec codec = encoder->codec.codec;

	if (codec == VCD_CODEC_MPEG4)
		encoder->level.level = VCD_LEVEL_MPEG4_1;
	else if (codec == VCD_CODEC_H264)
		encoder->level.level = VCD_LEVEL_H264_1;
	else
		encoder->level.level = VCD_LEVEL_H263_10;
}

static void ddl_set_default_enc_vop_timing(
	struct ddl_encoder_data *encoder)
{
	encoder->vop_timing.vop_time_resolution =
		(encoder->frame_rate.fps_numerator << 1) /
		encoder->frame_rate.fps_denominator;
}

static void ddl_set_default_enc_intra_period(
	struct ddl_encoder_data *encoder)
{
	switch (encoder->rc.rate_control) {
	default:
	case VCD_RATE_CONTROL_VBR_VFR:
	case VCD_RATE_CONTROL_VBR_CFR:
	case VCD_RATE_CONTROL_CBR_VFR:
	case VCD_RATE_CONTROL_OFF:
		encoder->i_period.p_frames =
			((encoder->frame_rate.fps_numerator << 1) /
			encoder->frame_rate.fps_denominator) - 1;
	break;
	case VCD_RATE_CONTROL_CBR_CFR:
		encoder->i_period.p_frames =
			((encoder->frame_rate.fps_numerator >> 1) /
			encoder->frame_rate.fps_denominator) - 1;
	break;
	}
	encoder->i_period.b_frames = DDL_DEFAULT_NUM_OF_B_FRAME;
}

static void ddl_set_default_enc_rc_params(
	struct ddl_encoder_data *encoder)
{
	enum vcd_codec codec = encoder->codec.codec;
	encoder->rc_level.frame_level_rc = true;
	encoder->qp_range.min_qp = 0x1;
	if (codec == VCD_CODEC_H264) {
		encoder->qp_range.min_qp = 0x0;
		encoder->qp_range.max_qp = 0x33;
		encoder->session_qp.i_frame_qp = 0x19;
		encoder->session_qp.p_frame_qp = 0x19;
		encoder->session_qp.b_frame_qp = 0x19;
		encoder->rc_level.mb_level_rc  = true;
		encoder->adaptive_rc.activity_region_flag  = true;
		encoder->adaptive_rc.dark_region_as_flag   = true;
		encoder->adaptive_rc.smooth_region_as_flag = true;
		encoder->adaptive_rc.static_region_as_flag = true;
	} else {
		encoder->qp_range.max_qp       = 0x1f;
		encoder->qp_range.min_qp       = 0x1;
		encoder->session_qp.i_frame_qp = 0x14;
		encoder->session_qp.p_frame_qp = 0x14;
		encoder->session_qp.b_frame_qp = 0x14;
		encoder->rc_level.frame_level_rc = true;
		encoder->rc_level.mb_level_rc  = false;
	}
	switch (encoder->rc.rate_control) {
	case VCD_RATE_CONTROL_VBR_CFR:
		encoder->r_cframe_skip = 0;
		encoder->frame_level_rc.reaction_coeff = 0x1f4;
	break;
	case VCD_RATE_CONTROL_CBR_VFR:
		encoder->r_cframe_skip = 1;
		if (codec != VCD_CODEC_H264) {
			encoder->session_qp.i_frame_qp = 0xf;
			encoder->session_qp.p_frame_qp = 0xf;
			encoder->session_qp.b_frame_qp = 0xf;
		}
		encoder->frame_level_rc.reaction_coeff = 0x6;
	break;
	case VCD_RATE_CONTROL_CBR_CFR:
		encoder->r_cframe_skip = 0;
		encoder->frame_level_rc.reaction_coeff = 0x6;
	break;
	case VCD_RATE_CONTROL_OFF:
		encoder->r_cframe_skip = 0;
		encoder->rc_level.frame_level_rc = false;
		encoder->rc_level.mb_level_rc = false;
	break;
	case VCD_RATE_CONTROL_VBR_VFR:
	default:
		encoder->r_cframe_skip = 1;
		encoder->frame_level_rc.reaction_coeff = 0x1f4;
	break;
	}
}

void ddl_set_default_encoder_buffer_req(struct ddl_encoder_data *encoder)
{
	u32 y_cb_cr_size, y_size;
	memset(&encoder->hw_bufs.dpb_y, 0, sizeof(struct ddl_buf_addr) * 4);
	memset(&encoder->hw_bufs.dpb_c, 0, sizeof(struct ddl_buf_addr) * 4);

	y_cb_cr_size = ddl_get_yuv_buffer_size(&encoder->frame_size,
				&encoder->buf_format, false, &y_size);
	encoder->input_buf_size.size_yuv = y_cb_cr_size;
	encoder->input_buf_size.size_y   = y_size;
	encoder->input_buf_size.size_c   = y_cb_cr_size - y_size;
	memset(&encoder->input_buf_req , 0 ,
		sizeof(struct vcd_buffer_requirement));
	encoder->input_buf_req.min_count    = 3;
	encoder->input_buf_req.actual_count =
		encoder->input_buf_req.min_count;
	encoder->input_buf_req.max_count    = DDL_MAX_BUFFER_COUNT;
	encoder->input_buf_req.sz = y_cb_cr_size;
	if (encoder->buf_format.buffer_format ==
		VCD_BUFFER_FORMAT_NV12_16M2KA)
		encoder->input_buf_req.align =
			DDL_LINEAR_BUFFER_ALIGN_BYTES;
	else if (VCD_BUFFER_FORMAT_TILE_4x2 ==
		encoder->buf_format.buffer_format)
		encoder->input_buf_req.align = DDL_TILE_BUFFER_ALIGN_BYTES;
	encoder->client_input_buf_req = encoder->input_buf_req;
	memset(&encoder->output_buf_req , 0 ,
		sizeof(struct vcd_buffer_requirement));
	encoder->output_buf_req.min_count    =
		encoder->i_period.b_frames + 2;
	encoder->output_buf_req.actual_count =
		encoder->output_buf_req.min_count + 3;
	encoder->output_buf_req.max_count    = DDL_MAX_BUFFER_COUNT;
	encoder->output_buf_req.align	= DDL_LINEAR_BUFFER_ALIGN_BYTES;
	if (y_cb_cr_size >= VCD_DDL_720P_YUV_BUF_SIZE)
		y_cb_cr_size = y_cb_cr_size>>1;
	encoder->output_buf_req.sz =
		DDL_ALIGN(y_cb_cr_size, DDL_KILO_BYTE(4));
	encoder->client_output_buf_req = encoder->output_buf_req;
}

void ddl_set_default_decoder_buffer_req(struct ddl_decoder_data *decoder,
	u32 estimate)
{
	struct vcd_property_frame_size *frame_size;
	struct vcd_buffer_requirement *input_buf_req;
	struct vcd_buffer_requirement *output_buf_req;
	u32  min_dpb, y_cb_cr_size/*, y_size*/;

	if (!decoder->codec.codec)
		return;
	if (estimate) {
		frame_size = &decoder->client_frame_size;
		output_buf_req = &decoder->client_output_buf_req;
		input_buf_req = &decoder->client_input_buf_req;
		min_dpb = ddl_decoder_min_num_dpb(decoder);
		y_cb_cr_size = ddl_get_yuv_buffer_size(frame_size,
					&decoder->buf_format,
					(!decoder->progressive_only), NULL);
	} else {
		frame_size = &decoder->frame_size;
		output_buf_req = &decoder->actual_output_buf_req;
		input_buf_req = &decoder->actual_input_buf_req;
		min_dpb = decoder->min_dpb_num;
		y_cb_cr_size = decoder->y_cb_cr_size;
	}
	memset(output_buf_req, 0,
		sizeof(struct vcd_buffer_requirement));
	if ((frame_size->width * frame_size->height) >=
		 VCD_DDL_WVGA_BUF_SIZE) {
		output_buf_req->actual_count = min_dpb + 2;
		if (output_buf_req->actual_count < 10)
			output_buf_req->actual_count = 10;
	} else
		output_buf_req->actual_count = min_dpb + 5;

	output_buf_req->min_count = min_dpb;
	output_buf_req->max_count = DDL_MAX_BUFFER_COUNT;
	output_buf_req->sz = y_cb_cr_size;
	DDL_MSG_LOW("output_buf_req->sz : %d", output_buf_req->sz);
	if (decoder->buf_format.buffer_format != VCD_BUFFER_FORMAT_NV12)
		output_buf_req->align = DDL_TILE_BUFFER_ALIGN_BYTES;
	else
		output_buf_req->align = DDL_LINEAR_BUFFER_ALIGN_BYTES;
	decoder->min_output_buf_req = *output_buf_req;
	memset(input_buf_req, 0,
		sizeof(struct vcd_buffer_requirement));
	input_buf_req->min_count = 1;
	input_buf_req->actual_count = input_buf_req->min_count + 3;
	input_buf_req->max_count = DDL_MAX_BUFFER_COUNT;
	input_buf_req->sz = (1024*1024*3) >> 1;
	input_buf_req->align = DDL_LINEAR_BUFFER_ALIGN_BYTES;
	decoder->min_input_buf_req = *input_buf_req;
}

u32 ddl_get_yuv_buffer_size(struct vcd_property_frame_size *frame_size,
	struct vcd_property_buffer_format *buf_format,
	u32 interlace, u32 *pn_c_offset)
{
	struct vcd_property_frame_size frame_sz = *frame_size;
	u32 total_memory_size = 0, c_offset = 0;
	ddl_calculate_stride(&frame_sz, interlace);
	if (/*(buf_format->buffer_format == VCD_BUFFER_FORMAT_TILE_1x1) ||*/
		(buf_format->buffer_format == VCD_BUFFER_FORMAT_TILE_4x2)) {
		u32 component_mem_size, width_round_up;
		u32 height_round_up, height_chroma = (frame_sz.scan_lines >> 1);

		width_round_up =
			DDL_TILE_ALIGN(frame_sz.stride, DDL_TILE_ALIGN_WIDTH);
		height_round_up =
			DDL_TILE_ALIGN(frame_sz.scan_lines,
						   DDL_TILE_ALIGN_HEIGHT);
		component_mem_size = width_round_up * height_round_up;
		component_mem_size = DDL_TILE_ALIGN(component_mem_size,
			DDL_TILE_MULTIPLY_FACTOR);
		c_offset = component_mem_size;
		total_memory_size = ((component_mem_size +
					DDL_TILE_BUF_ALIGN_GUARD_BYTES) &
					DDL_TILE_BUF_ALIGN_MASK);
		height_round_up = DDL_TILE_ALIGN(height_chroma,
					DDL_TILE_ALIGN_HEIGHT);
		component_mem_size = width_round_up * height_round_up;
		component_mem_size = DDL_TILE_ALIGN(component_mem_size,
					DDL_TILE_MULTIPLY_FACTOR);
		total_memory_size += component_mem_size;
	} else {
		total_memory_size = frame_sz.scan_lines * frame_sz.stride;
		c_offset = DDL_ALIGN(total_memory_size,
			DDL_LINEAR_MULTIPLY_FACTOR);
		total_memory_size = c_offset + DDL_ALIGN(
			total_memory_size >> 1, DDL_LINEAR_MULTIPLY_FACTOR);
	}
	if (pn_c_offset)
		*pn_c_offset = c_offset;
	return total_memory_size;
}


void ddl_calculate_stride(struct vcd_property_frame_size *frame_size,
	u32 interlace)
{
	frame_size->stride = DDL_ALIGN(frame_size->width,
					DDL_LINEAR_ALIGN_WIDTH);
	if (interlace)
		frame_size->scan_lines = DDL_ALIGN(frame_size->height,
						DDL_TILE_ALIGN_HEIGHT);
	else
		frame_size->scan_lines = DDL_ALIGN(frame_size->height,
						DDL_LINEAR_ALIGN_HEIGHT);
}


static u32 ddl_valid_buffer_requirement(struct vcd_buffer_requirement
	*original_buf_req, struct vcd_buffer_requirement *req_buf_req)
{
	u32 status = false;

	if (original_buf_req->max_count >= req_buf_req->actual_count &&
		original_buf_req->min_count <=
		req_buf_req->actual_count &&
		!((original_buf_req->align - (u32)0x1) &
		req_buf_req->align) &&
		/*original_buf_req->align <= req_buf_req->align,*/
		original_buf_req->sz <= req_buf_req->sz)
		status = true;
	else
		DDL_MSG_ERROR("ddl_valid_buf_req:Failed");
	return status;
}

static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data *decoder)
{
	u32 min_dpb = 0;

	switch (decoder->codec.codec) {
	case VCD_CODEC_H264:
	{
		u32 yuv_size_in_mb = DDL_MIN(DDL_NO_OF_MB(
			decoder->client_frame_size.height,
			decoder->client_frame_size.width),
			MAX_FRAME_SIZE_L4PT0_MBS);
		min_dpb = DDL_MIN((MAX_DPB_SIZE_L4PT0_MBS /
				yuv_size_in_mb), 16);
		min_dpb += 2;
		if (min_dpb <= 6)
			min_dpb += 2;
	}
	break;
	case VCD_CODEC_H263:
		min_dpb = 3;
	break;
	default:
	case VCD_CODEC_MPEG1:
	case VCD_CODEC_MPEG2:
	case VCD_CODEC_MPEG4:
	case VCD_CODEC_DIVX_3:
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		min_dpb = 4;
	break;
	}
	return min_dpb;
}

static u32 ddl_set_dec_buffers(struct ddl_decoder_data *decoder,
	struct ddl_property_dec_pic_buffers *dpb)
{
	u32 vcd_status  = VCD_S_SUCCESS, loopc;


	for (loopc = 0; !vcd_status &&
		loopc < dpb->no_of_dec_pic_buf; ++loopc) {
		if ((!DDL_ADDR_IS_ALIGNED(dpb->dec_pic_buffers[loopc].
			vcd_frm.physical,
			decoder->client_output_buf_req.align)) ||
			(dpb->dec_pic_buffers[loopc].vcd_frm.alloc_len <
			decoder->client_output_buf_req.sz))
			vcd_status = VCD_ERR_ILLEGAL_PARM;
	}
	if (vcd_status) {
		DDL_MSG_ERROR("ddl_set_prop:\
			Dpb_align_fail_or_alloc_size_small");
		return vcd_status;
	}
	if (decoder->dp_buf.no_of_dec_pic_buf) {
		kfree(decoder->dp_buf.dec_pic_buffers);
		decoder->dp_buf.no_of_dec_pic_buf = 0;
	}
	decoder->dp_buf.dec_pic_buffers =
		kmalloc(dpb->no_of_dec_pic_buf *
			sizeof(struct ddl_frame_data_tag), GFP_KERNEL);
	if (!decoder->dp_buf.dec_pic_buffers) {
		DDL_MSG_ERROR("ddl_dec_set_prop:Dpb_container_alloc_failed");
		return VCD_ERR_ALLOC_FAIL;
	}
	decoder->dp_buf.no_of_dec_pic_buf = dpb->no_of_dec_pic_buf;
	for (loopc = 0; loopc < dpb->no_of_dec_pic_buf; ++loopc)
		decoder->dp_buf.dec_pic_buffers[loopc] =
			dpb->dec_pic_buffers[loopc];
	decoder->dpb_mask.client_mask = 0;
	decoder->dpb_mask.hw_mask = 0;
	decoder->dynamic_prop_change = 0;
	return VCD_S_SUCCESS;
}

void ddl_set_initial_default_values(struct ddl_client_context *ddl)
{

	if (ddl->decoding) {
		ddl->codec_data.decoder.codec.codec = VCD_CODEC_MPEG4;
		ddl_set_default_dec_property(ddl);
	} else {
		struct ddl_encoder_data *encoder =
			&(ddl->codec_data.encoder);
		encoder->codec.codec = VCD_CODEC_MPEG4;
		encoder->target_bit_rate.target_bitrate = 64000;
		encoder->frame_size.width = VCD_DDL_TEST_DEFAULT_WIDTH;
		encoder->frame_size.height = VCD_DDL_TEST_DEFAULT_HEIGHT;
		encoder->frame_size.scan_lines =
			VCD_DDL_TEST_DEFAULT_HEIGHT;
		encoder->frame_size.stride = VCD_DDL_TEST_DEFAULT_WIDTH;
		encoder->frame_rate.fps_numerator = 30;
		encoder->frame_rate.fps_denominator = 1;
		ddl_set_default_enc_property(ddl);
	}
}
