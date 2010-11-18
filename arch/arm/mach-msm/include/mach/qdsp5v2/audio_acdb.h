/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _MACH_QDSP5_V2_AUDIO_ACDB_H
#define _MACH_QDSP5_V2_AUDIO_ACDB_H

s32 acdb_calibrate_device(void *data);
s32 acdb_get_calibration(void);
s32 acdb_send_calibration(void);
void device_cb(u32 evt_id, union auddev_evt_data *evt, void *private);
void audpp_cb(void *private, u32 id, u16 *msg);
void audpreproc_cb(void *private, u32 id, void *msg);

s32 acdb_initialize_data(void);
s32 initialize_rpc(void);
s32 initialize_memory(void);
s32 register_device_cb(void);
s32 register_audpp_cb(void);
s32 register_audpreproc_cb(void);
s32 acdb_calibrate_audpp(void);
s32 acdb_calibrate_audpreproc(void);
s32 acdb_fill_audpp_iir(void);
s32 acdb_fill_audpp_mbadrc(void);
s32 acdb_fill_audpreproc_agc(void);
s32 acdb_fill_audpreproc_iir(void);

int htc_reinit_acdb(char* filename);
#endif
