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
#include "vcd_ddl_utils.h"
#include "vcd_ddl.h"

#ifdef DDL_PROFILE
static unsigned int ddl_dec_t1, ddl_enc_t1;
static unsigned int ddl_dec_ttotal, ddl_enc_ttotal;
static unsigned int ddl_dec_count, ddl_enc_count;
#endif

#define DDL_FW_CHANGE_ENDIAN

#ifdef DDL_BUF_LOG
static void ddl_print_buffer(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str);
static void ddl_print_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf);
static void ddl_print_buffer_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str);
#endif

void *ddl_pmem_alloc(struct ddl_buf_addr *addr, size_t sz, u32 alignment)
{
	u32 alloc_size, offset = 0;

	alloc_size = (sz + alignment);
	addr->physical_base_addr = (u8 *) pmem_kalloc(alloc_size,
		PMEM_MEMTYPE_SMI | PMEM_ALIGNMENT_4K);
	if (!addr->physical_base_addr) {
		DDL_MSG_ERROR("%s() : pmem alloc failed (%d)\n", __func__,
			alloc_size);
		return NULL;
	}
	DDL_MSG_LOW("%s() : pmem alloc physical base addr/sz 0x%x / %d \n",\
		__func__, (u32)addr->physical_base_addr, alloc_size);
	addr->virtual_base_addr = (u8 *)ioremap((unsigned long)
		addr->physical_base_addr, alloc_size);
	if (!addr->virtual_base_addr) {
		DDL_MSG_ERROR("%s() : ioremap failed, virtual(%x)\n", __func__,
			(u32)addr->virtual_base_addr);
		return NULL;
	}
	DDL_MSG_LOW("%s() : pmem alloc virtual base addr/sz 0x%x / %d \n",\
		__func__, (u32)addr->virtual_base_addr, alloc_size);
	addr->align_physical_addr = (u8 *) DDL_ALIGN((u32)
		addr->physical_base_addr, alignment);
	offset = (u32)(addr->align_physical_addr -
			addr->physical_base_addr);
	addr->align_virtual_addr = addr->virtual_base_addr + offset;
	addr->buffer_size = sz;
	DDL_MSG_LOW("%s() : pmem alloc physical aligned addr/sz 0x%x/ %d \n",\
		__func__, (u32)addr->align_physical_addr, sz);
	DDL_MSG_LOW("%s() : pmem alloc virtual aligned addr/sz 0x%x / %d \n",\
		__func__, (u32)addr->virtual_base_addr, sz);
	return addr->virtual_base_addr;
}

void ddl_pmem_free(struct ddl_buf_addr *addr)
{
	DDL_MSG_LOW("ddl_pmem_free:");
	if (addr->virtual_base_addr)
		iounmap((void *)addr->virtual_base_addr);
	if ((addr->physical_base_addr) &&
		pmem_kfree((s32) addr->physical_base_addr)) {
		DDL_MSG_LOW("\n %s(): Error in Freeing Physical Address %p",\
			__func__, addr->physical_base_addr);
	}
	addr->physical_base_addr   = NULL;
	addr->virtual_base_addr    = NULL;
	addr->align_virtual_addr   = NULL;
	addr->align_physical_addr  = NULL;
	addr->buffer_size = 0;
}

#ifdef DDL_BUF_LOG

static void ddl_print_buffer(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str)
{
	struct ddl_buf_addr *base_ram;
	s32  offset;
	size_t sz, KB = 0;

	base_ram = &ddl_context->dram_base_a;
	offset = (s32) DDL_ADDR_OFFSET(*base_ram, *buf);
	sz = buf->buffer_size;
	if (sz > 0) {
		if (!(sz % 1024)) {
			sz /= 1024;
			KB++;
			if (!(sz % 1024)) {
				sz /= 1024;
				KB++;
			}
		}
	}
	DDL_MSG_LOW("\n%12s [%2d]:  0x%08x [0x%04x],  0x%08x(%d%s),  %s",
		str, idx, (u32) buf->align_physical_addr,
		(offset > 0) ? offset : 0, buf->buffer_size, sz,
		((2 == KB) ? "MB" : (1 == KB) ? "KB" : ""),
		(((u32) buf->virtual_base_addr) ? "Alloc" : ""));
}

static void ddl_print_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf)
{
	struct ddl_buf_addr *a = &ddl_context->dram_base_a;
	struct ddl_buf_addr *b = &ddl_context->dram_base_b;

	if (!buf->align_physical_addr || !buf->buffer_size)
		return;
	if (buf->align_physical_addr >= a->align_physical_addr &&
		buf->align_physical_addr + buf->buffer_size <=
		a->align_physical_addr + a->buffer_size)
		DDL_MSG_LOW(" -A [0x%x]-", DDL_ADDR_OFFSET(*a, *buf));
	else if (buf->align_physical_addr >= b->align_physical_addr &&
		buf->align_physical_addr + buf->buffer_size <=
		b->align_physical_addr + b->buffer_size)
		DDL_MSG_LOW(" -B [0x%x]-", DDL_ADDR_OFFSET(*b, *buf));
	else
		DDL_MSG_LOW(" -?-");
}

static void ddl_print_buffer_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str)
{
	DDL_MSG_LOW("\n");
	ddl_print_buffer(ddl_context, buf, idx, str);
	ddl_print_port(ddl_context, buf);
}

void ddl_list_buffers(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context;
	u32 i;

	ddl_context = ddl->ddl_context;
	DDL_MSG_LOW("\n\n");
	DDL_MSG_LOW("\n      Buffer     :     Start    [offs],      Size    \
	(Size),     Alloc/Port");
	DDL_MSG_LOW("\n-------------------------------------------------------\
	-------------------------");
	ddl_print_buffer(ddl_context, &ddl_context->dram_base_a, 0,
		"dram_base_a");
	ddl_print_buffer(ddl_context, &ddl_context->dram_base_b, 0,
		"dram_base_b");
	if (ddl->codec_data.hdr.decoding) {
		struct ddl_dec_buffers  *dec_bufs =
			&ddl->codec_data.decoder.hw_bufs;
		for (i = 0; i < 32; i++)
			ddl_print_buffer_port(ddl_context,
				&dec_bufs->h264Mv[i], i, "h264Mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->h264Vert_nb_mv, 0, "h264Vert_nb_mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->h264Nb_ip, 0, "h264Nb_ip");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->nb_dcac, 0, "nb_dcac");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->upnb_mv, 0, "upnb_mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->sub_anchor_mv, 0, "sub_anchor_mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->overlay_xform, 0, "overlay_xform");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->bit_plane3, 0, "bit_plane3");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->bit_plane2, 0, "bit_plane2");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->bit_plane1, 0, "bit_plane1");
		ddl_print_buffer_port(ddl_context,
			dec_bufs->stx_parser, 0, "stx_parser");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->desc, 0, "desc");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->context, 0, "context");
	} else {
		struct ddl_enc_buffers  *enc_bufs =
			&ddl->codec_data.encoder.hw_bufs;

		for (i = 0; i < 4; i++)
			ddl_print_buffer_port(ddl_context,
				&enc_bufs->dpb_y[i], i, "dpb_y");
		for (i = 0; i < 4; i++)
			ddl_print_buffer_port(ddl_context,
				&enc_bufs->dpb_c[i], i, "dpb_c");
		ddl_print_buffer_port(ddl_context, &enc_bufs->mv, 0, "mv");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->col_zero, 0, "col_zero");
		ddl_print_buffer_port(ddl_context, &enc_bufs->md, 0, "md");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->pred, 0, "pred");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->nbor_info, 0, "nbor_info");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->acdc_coef, 0, "acdc_coef");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->context, 0, "context");
	}
}
#endif

#ifdef DDL_FW_CHANGE_ENDIAN
static void ddl_fw_change_endian(u8 *fw, u32 fw_size)
{
	u32 i = 0;
	u8  temp;
	for (i = 0; i < fw_size; i = i + 4) {
		temp = fw[i];
		fw[i] = fw[i+3];
		fw[i+3] = temp;
		temp = fw[i+1];
		fw[i+1] = fw[i+2];
		fw[i+2] = temp;
	}
	return;
}
#endif

u32 ddl_fw_init(struct ddl_buf_addr *dram_base)
{

	u8 *dest_addr;

	dest_addr = DDL_GET_ALIGNED_VITUAL(*dram_base);
	if (vidc_video_codec_fw_size > dram_base->buffer_size ||
		!vidc_video_codec_fw)
		return false;
	DDL_MSG_LOW("FW Addr / FW Size : %x/%d", (u32)vidc_video_codec_fw,
		vidc_video_codec_fw_size);
	memcpy(dest_addr, vidc_video_codec_fw,
		vidc_video_codec_fw_size);
#ifdef DDL_FW_CHANGE_ENDIAN
	ddl_fw_change_endian(dest_addr, vidc_video_codec_fw_size);
#endif
	return true;
}

void ddl_fw_release(void)
{

}

#ifdef DDL_PROFILE
void ddl_get_core_start_time(u8 codec)
{
	u32 *ddl_t1 = NULL;
	if (!codec)
		ddl_t1 = &ddl_dec_t1;
	else if (codec == 1)
		ddl_t1 = &ddl_enc_t1;

	if (!*ddl_t1) {
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		*ddl_t1 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
	}
}

void ddl_calc_core_time(u8 codec)
{
	u32 *ddl_t1 = NULL, *ddl_ttotal = NULL,
		*ddl_count = NULL;
	if (!codec) {
		DDL_MSG_ERROR("\n1080p Core Decode ");
		ddl_t1 = &ddl_dec_t1;
		ddl_ttotal = &ddl_dec_ttotal;
		ddl_count = &ddl_dec_count;
	} else if (codec == 1) {
		DDL_MSG_ERROR("\n1080p Core Encode ");
		ddl_t1 = &ddl_enc_t1;
		ddl_ttotal = &ddl_enc_ttotal;
		ddl_count = &ddl_enc_count;
	}

	if (*ddl_t1) {
		int ddl_t2;
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		ddl_t2 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
		*ddl_ttotal += (ddl_t2 - *ddl_t1);
		*ddl_count = *ddl_count + 1;
		DDL_MSG_ERROR("time %u, average time %u, count %u",
			ddl_t2 - *ddl_t1, (*ddl_ttotal)/(*ddl_count),
			*ddl_count);
		*ddl_t1 = 0;
	}
}

void ddl_reset_time_variables(u8 codec)
{
	if (!codec) {
		DDL_MSG_ERROR("\n Reset Decoder time variables");
		ddl_dec_t1 = 0;
		ddl_dec_ttotal = 0;
		ddl_dec_count = 0;
	} else if (codec == 1) {
		DDL_MSG_ERROR("\n Reset Encoder time variables ");
		ddl_enc_t1 = 0;
		ddl_enc_ttotal = 0;
		ddl_enc_count = 0;
	}
}
#endif
