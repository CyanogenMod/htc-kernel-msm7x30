/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
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
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _VCD_DDL_CORE_H_
#define _VCD_DDL_CORE_H_

#define DDL_LINEAR_BUF_ALIGN_MASK         0xFFFFF800U
#define DDL_LINEAR_BUF_ALIGN_GUARD_BYTES  0x7FF
#define DDL_LINEAR_BUFFER_ALIGN_BYTES     2048
#define DDL_TILE_BUF_ALIGN_MASK           0xFFFFE000U
#define DDL_TILE_BUF_ALIGN_GUARD_BYTES    0x1FFF
#define DDL_TILE_BUFFER_ALIGN_BYTES       8192


#define DDL_MAX_FRAME_WIDTH   1920
#define DDL_MAX_FRAME_HEIGHT  1088

#define DDL_SW_RESET_SLEEP               1
#define VCD_MAX_NO_CLIENT                2
#define VCD_MAX_NO_CODEC                 0
#define VCD_MAX_NO_CODECTYPE             7
#define VCD_FRAME_COMMAND_DEPTH          1
#define VCD_GENEVIDC_COMMAND_DEPTH        1
#define VCD_COMMAND_EXCLUSIVE            true
#define DDL_HW_TIMEOUT_IN_MS             1000
#define DDL_STREAMBUF_ALIGN_GUARD_BYTES  0x7FF

#define DDL_CONTEXT_MEMORY (1024 * 15 * (VCD_MAX_NO_CLIENT + 1))

#define DDL_DB_LINE_BUF_SIZE\
	(((((DDL_MAX_FRAME_WIDTH * 4) - 1) / 256) + 1) * 8 * 1024)

#define DDL_DECODE_H264_VSPTEMP_BUFSIZE   0x51c00
#define DDL_ENC_MIN_DPB_BUFFERS           2
#define DDL_ENC_MAX_DPB_BUFFERS           4
#define DDL_DBG_CORE_DUMP_SIZE            (10 * 1024)
#define DDL_BUFEND_PAD                    256
#define DDL_ENC_SEQHEADER_SIZE            (256+DDL_BUFEND_PAD)
#define DDL_MAX_BUFFER_COUNT              32
#define DDL_MPEG_REFBUF_COUNT             2
#define DDL_MPEG_COMV_BUF_NO              2
#define DDL_H263_COMV_BUF_NO              0
#define DDL_COMV_BUFLINE_NO               128
#define DDL_VC1_COMV_BUFLINE_NO           32
#define DDL_MINIMUM_BYTE_PER_SLICE        1920
#define DDL_MAX_H264_QP            51
#define DDL_MAX_MPEG4_QP           31
#define DDL_FRAMESIZE_DIV_FACTOR   0xF

#define DDL_NO_OF_MB(nWidth, nHeight) \
	((((nWidth) + 15) >> 4) * (((nHeight) + 15) >> 4))
#define DDL_ALLOW_DEC_FRAMESIZE(width, height) \
	(((width <= DDL_MAX_FRAME_WIDTH) && \
	(height <= DDL_MAX_FRAME_HEIGHT)) && \
	((width >= 32 && height >= 16) || \
	(width >= 16 && height >= 32)))
#define DDL_ALLOW_ENC_FRAMESIZE(width, height) \
	(((width <= DDL_MAX_FRAME_WIDTH) && \
	(height <= DDL_MAX_FRAME_HEIGHT)) && \
	((width >= 32 && height >= 32)))
#define DDL_LINEAR_ALIGN_WIDTH      16
#define DDL_LINEAR_ALIGN_HEIGHT     16
#define DDL_LINEAR_MULTIPLY_FACTOR  2048
#define DDL_TILE_ALIGN_WIDTH        128
#define DDL_TILE_ALIGN_HEIGHT       32
#define DDL_TILE_MULTIPLY_FACTOR    8192
#define DDL_TILE_ALIGN(val, grid) \
	(((val) + (grid) - 1) / (grid) * (grid))

#define VCD_DDL_720P_YUV_BUF_SIZE     ((1280*720*3) >> 1)
#define VCD_DDL_WVGA_BUF_SIZE         (800*480)

#define VCD_DDL_TEST_MAX_WIDTH        (DDL_MAX_FRAME_WIDTH)
#define VCD_DDL_TEST_MAX_HEIGHT       (DDL_MAX_FRAME_HEIGHT)

#define VCD_DDL_TEST_MAX_NUM_H264_DPB  8

#define VCD_DDL_TEST_NUM_ENC_INPUT_BUFS   6
#define VCD_DDL_TEST_NUM_ENC_OUTPUT_BUFS  4

#define VCD_DDL_TEST_DEFAULT_WIDTH       176
#define VCD_DDL_TEST_DEFAULT_HEIGHT      144

#define DDL_PIXEL_CACHE_NOT_IDLE          0x4000
#define DDL_PIXEL_CACHE_STATUS_READ_RETRY 10
#define DDL_PIXEL_CACHE_STATUS_READ_SLEEP 20

#endif
