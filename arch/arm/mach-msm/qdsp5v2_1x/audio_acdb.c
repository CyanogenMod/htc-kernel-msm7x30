/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <mach/debug_audio_mm.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/android_pmem.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <linux/firmware.h>
#include <linux/slab.h>

#include <mach/dal.h>
#include <mach/qdsp5v2_1x/audio_dev_ctl.h>
#include <mach/qdsp5v2_1x/audpp.h>
#include <mach/qdsp5v2_1x/audpreproc.h>
#include <mach/qdsp5v2_1x/qdsp5audppcmdi.h>
#include <mach/qdsp5v2_1x/qdsp5audpreproccmdi.h>
#include <mach/qdsp5v2_1x/qdsp5audpreprocmsg.h>
#include <mach/qdsp5v2_1x/qdsp5audppmsg.h>
#include <mach/qdsp5v2_1x/audio_acdb.h>
#include <mach/qdsp5v2_1x/audio_acdbi.h>
#include <mach/qdsp5v2_1x/acdb_commands.h>
#include <mach/qdsp5v2_1x/audio_acdb_def.h>

#include <mach/htc_acdb.h>
#include <mach/htc_acoustic_7x30.h>

/* this is the ACDB device ID */
#define DALDEVICEID_ACDB		0x02000069
#define ACDB_PORT_NAME			"SMD_DAL00"
#define ACDB_CPU			SMD_APPS_MODEM
#define ACDB_BUF_SIZE			4096

#define MAX_RETRY	10

/* rpc table index */
enum {
	ACDB_DalACDB_ioctl = DALDEVICE_FIRST_DEVICE_API_IDX
};

enum {
	CAL_DATA_READY		= 0x1,
	AUDPP_READY		= 0x2,
	AUDPREPROC_READY	= 0x4,
	WAITING_FOR_CAL		= 0x8
};


struct acdb_data {
	void *handle;

	u32 phys_addr;
	u8 *virt_addr;

	struct task_struct *cb_thread_task;

	u32 acdb_state;
	u8 cal_data_ready;
	u8 audpp_ready;
	u8 audpreproc_ready;
	u8 waiting_for_cal;
	u8 enable;
	struct audpp_event_callback audpp_cb;
	struct audpreproc_event_callback audpreproc_cb;

	struct auddev_evt_audcal_info *device_info;

	struct audpp_cmd_cfg_object_params_pcm *pp_iir;
	struct audpp_cmd_cfg_object_params_mbadrc *pp_mbadrc;
	struct audpreproc_cmd_cfg_agc_params *preproc_agc;
	struct audpreproc_cmd_cfg_iir_tuning_filter_params *preproc_iir;
	struct acdb_mbadrc_block mbadrc_block;
	u8 preproc_stream_id;

	wait_queue_head_t wait;
	struct mutex acdb_mutex;
	u32 acdb_compl;
	u32 wait_for_cal_compl;
	struct acdb_result acdb_result;
};


static struct acdb_data		acdb_data;
static int htc_acdb_enable;


static const struct file_operations acdb_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
};


struct miscdevice acdb_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_acdb",
	.fops	= &acdb_fops,
};


void *htc_acdb_data;
static void *audio_data;
static dma_addr_t audio_phys;
static char acdb_file[64] = "default.acdb";
const struct firmware *acdb_fw;
int mbadrc_num_bands = ARRAY_SIZE(acdb_data.mbadrc_block.band_config);

static int htc_acdb_init(char *filename)
{
	const struct audio_config_database *db;
	const struct firmware *fw;
	int n;

	printk("acdb: load '%s'\n", filename);
	if (request_firmware(&fw, filename, acdb_misc.this_device) < 0) {
		printk("acdb: load 'default.acdb' failed...\n");
		return -ENODEV;
	}

	if (!fw)
		return -ENODEV;
	db = (void*) fw->data;

	if (fw->size < sizeof(struct audio_config_database)) {
		pr_aud_err("acdb: undersized database\n");
		goto fail;
	}
	if (strcmp(db->magic, "ACDB1.0")) {
		pr_aud_err("acdb: invalid magic\n");
		goto fail;
	}
	if (db->entry_count > 1024) {
		pr_aud_err("acdb: too many entries\n");
		goto fail;
	}
	if (fw->size < (sizeof(struct audio_config_database) +
			db->entry_count * sizeof(struct audio_config_data))) {
		pr_aud_err("acdb: undersized TOC\n");
		goto fail;
	}
	for (n = 0; n < db->entry_count; n++) {
		if (db->entry[n].length > 4096) {
			pr_aud_err("acdb: entry %d too large (%d)\n",
			       n, db->entry[n].length);
			goto fail;
		}
		if ((db->entry[n].offset + db->entry[n].length) > fw->size) {
			pr_aud_err("acdb: entry %d outside of data\n", n);
			goto fail;
		}
	}
	if (htc_acdb_data)
		release_firmware(acdb_fw);
	htc_acdb_data = (void *) fw->data;
	acdb_fw = fw;
	htc_acdb_enable = 1;
	return 0;
fail:
	release_firmware(fw);
	return -ENODEV;
}

int htc_reinit_acdb(char* filename) {
	int res;

	if (strlen(filename) < 0) {
		res = -EINVAL;
		goto done;
	}
	mutex_lock(&acdb_data.acdb_mutex);
	res = htc_acdb_init(filename);
	if (!res)
		strcpy(acdb_file, filename);
	mutex_unlock(&acdb_data.acdb_mutex);
done:
	return res;

}
static int acdb_get_config_table(uint32_t device_id, uint32_t sample_rate)
{
	struct audio_config_database *db;
	int n, res;

	if (!htc_acdb_data) {
		res = htc_acdb_init(acdb_file);
		if (res)
			return res;
	}

	db = htc_acdb_data;
	for (n = 0; n < db->entry_count; n++) {
		if (db->entry[n].device_id != device_id)
			continue;
		if (db->entry[n].sample_rate != sample_rate)
			continue;
		break;
	}
		if (n == db->entry_count) {
		pr_aud_err("acdb: no entry for device %d, rate %d.\n",
		       device_id, sample_rate);
		return 0;
	}

	pr_aud_info("acdb: %d bytes for device %d, rate %d.\n",
		db->entry[n].length, device_id, sample_rate);

	memcpy(audio_data, htc_acdb_data + db->entry[n].offset, db->entry[n].length);

	acdb_data.acdb_result.used_bytes =  db->entry[n].length;
	acdb_data.virt_addr = audio_data;
	acdb_data.phys_addr = audio_phys;
	return db->entry[n].length;

}


static int __init acdb_init(void)
{

	s32 result = 0;

	memset(&acdb_data, 0, sizeof(acdb_data));

	acdb_data.cb_thread_task = kthread_run(acdb_calibrate_device,
		NULL, "acdb_cb_thread");

	if (IS_ERR(acdb_data.cb_thread_task)) {
		MM_AUD_ERR("ACDB=> Could not register cb thread\n");
		result = -ENODEV;
		goto err;
	}

	init_waitqueue_head(&acdb_data.wait);
	audio_data = dma_alloc_coherent(NULL, 9000, &audio_phys, GFP_KERNEL);

	return misc_register(&acdb_misc);
err:
	return result;
}


s32 acdb_calibrate_device(void *data)
{
	s32 result = 0;
	s32 rc = 0;

	/* initialize driver */
	result = acdb_initialize_data();
	if (result)
		goto done;

	while (!kthread_should_stop()) {
		MM_DBG("Waiting for Device Ready Event\n");
		wait_event_interruptible(acdb_data.wait,
					acdb_data.acdb_compl);
		acdb_data.acdb_compl = 0;

		mutex_lock(&acdb_data.acdb_mutex);
		result = acdb_get_calibration();
		if (result < 0) {
			mutex_unlock(&acdb_data.acdb_mutex);
			MM_AUD_ERR("Not able to get calibration data,\
					continue\n");
			continue;
		}
		if (!((acdb_data.device_info->dev_type == RX_DEVICE)
			&& (acdb_data.acdb_state & AUDPP_READY)) &&
			!((acdb_data.device_info->dev_type == TX_DEVICE)
			&& (acdb_data.acdb_state & AUDPREPROC_READY))) {
			MM_DBG("Waiting for either AUDPP/AUDPREPROC Event\n");

			mutex_unlock(&acdb_data.acdb_mutex);
			rc = wait_event_interruptible(acdb_data.wait,
					      acdb_data.acdb_compl);
			acdb_data.acdb_compl = 0;
			mutex_lock(&acdb_data.acdb_mutex);
		}

		if (acdb_data.acdb_state & CAL_DATA_READY)
			result = acdb_send_calibration();

		mutex_unlock(&acdb_data.acdb_mutex);
	}

done:
	return 0;
}


s32 acdb_initialize_data(void)
{
	s32	result = 0;

	mutex_init(&acdb_data.acdb_mutex);

	result = initialize_rpc();
	if (result)
		goto err;

	result = initialize_memory();
	if (result)
		goto err1;

	result = register_device_cb();
	if (result)
		goto err2;

	result = register_audpp_cb();
	if (result)
		goto err3;

	result = register_audpreproc_cb();
	if (result)
		goto err4;

	return result;

err4:
	result = audpreproc_unregister_event_callback(&acdb_data.audpreproc_cb);
	if (result)
		MM_AUD_ERR("ACDB=> Could not unregister audpreproc callback\n");
err3:
	result = audpp_unregister_event_callback(&acdb_data.audpp_cb);
	if (result)
		MM_AUD_ERR("ACDB=> Could not unregister audpp callback\n");
err2:
	result = auddev_unregister_evt_listner(AUDDEV_CLNT_AUDIOCAL, 0);
	if (result)
		MM_AUD_ERR("ACDB=> Could not unregister device callback\n");
err1:
	daldevice_detach(acdb_data.handle);
	acdb_data.handle = NULL;
err:
	return result;
}


s32 initialize_rpc(void)
{
	s32 result = 0;

	result = daldevice_attach(DALDEVICEID_ACDB, ACDB_PORT_NAME,
			ACDB_CPU, &acdb_data.handle);

	if (result) {
		MM_AUD_ERR("ACDB=> Device Attach failed\n");
		result = -ENODEV;
		goto done;
	}
done:
	return result;
}


s32 initialize_memory(void)
{
	s32 result = 0;

	/*initialize local cache */

	acdb_data.virt_addr = dma_alloc_coherent(NULL, ACDB_BUF_SIZE,
				 &acdb_data.phys_addr, GFP_KERNEL);

	if (acdb_data.virt_addr == NULL) {
		MM_AUD_ERR("ACDB=> Could not allocate acdb buffer\n");
		result = -ENOMEM;
		goto done;
	}

	memset(acdb_data.virt_addr, 0, sizeof(*acdb_data.virt_addr));

	acdb_data.device_info = kmalloc(sizeof(*acdb_data.device_info),
		GFP_KERNEL);
	if (acdb_data.device_info == NULL) {
		MM_AUD_ERR("ACDB=> Could not allocate device\
				controller memory\n");
		result = -ENOMEM;
		goto done;
	}

	acdb_data.pp_iir = kmalloc(sizeof(*acdb_data.pp_iir),
		GFP_KERNEL);
	if (acdb_data.pp_iir == NULL) {
		MM_AUD_ERR("ACDB=> Could not allocate postproc iir memory\n");
		result = -ENOMEM;
		goto done;
	}

	acdb_data.pp_mbadrc = kmalloc(sizeof(*acdb_data.pp_mbadrc), GFP_KERNEL);
	if (acdb_data.pp_mbadrc == NULL) {
		MM_AUD_ERR("ACDB=> Could not allocate postproc\
				mbadrc memory\n");
		result = -ENOMEM;
		goto done;
	}

	acdb_data.preproc_agc = kmalloc(sizeof(*acdb_data.preproc_agc),
						GFP_KERNEL);
	if (acdb_data.preproc_agc == NULL) {
		MM_AUD_ERR("ACDB=> Could not allocate preproc agc memory\n");
		result = -ENOMEM;
		goto done;
	}

	acdb_data.preproc_iir = kmalloc(sizeof(*acdb_data.preproc_iir),
		GFP_KERNEL);
	if (acdb_data.preproc_iir == NULL) {
		MM_AUD_ERR("ACDB=> Could not allocate preproc iir memory\n");
		result = -ENOMEM;
		goto done;
	}
done:
	return result;
}


s32 register_device_cb(void)
{
	s32 result = 0;

	result = auddev_register_evt_listner(AUDDEV_EVT_DEV_RDY,
		AUDDEV_CLNT_AUDIOCAL, 0, device_cb, (void *)&acdb_data);

	if (result) {
		MM_AUD_ERR("ACDB=> Could not register device callback\n");
		result = -ENODEV;
		goto done;
	}
done:
	return result;
}

s32 register_audpp_cb(void)
{
	s32 result = 0;

	acdb_data.audpp_cb.fn = audpp_cb;
	acdb_data.audpp_cb.private = NULL;
	result = audpp_register_event_callback(&acdb_data.audpp_cb);
	if (result) {
		MM_AUD_ERR("ACDB=> Could not register audpp callback\n");
		result = -ENODEV;
		goto done;
	}
done:
	return result;
}

s32 register_audpreproc_cb(void)
{
	s32 result = 0;

	acdb_data.audpreproc_cb.fn = audpreproc_cb;
	acdb_data.audpreproc_cb.private = NULL;
	result = audpreproc_register_event_callback(&acdb_data.audpreproc_cb);
	if (result) {
		MM_AUD_ERR("ACDB=> Could not register audpreproc callback\n");
		result = -ENODEV;
		goto done;
	}

done:
	return result;
}

void device_cb(u32 evt_id, union auddev_evt_data *evt, void *private)
{
	struct auddev_evt_audcal_info	audcal_info;

	if (!(evt_id == AUDDEV_EVT_DEV_RDY))
		goto done;

	audcal_info = evt->audcal_info;
	mutex_lock(&acdb_data.acdb_mutex);
	if (acdb_data.acdb_state & CAL_DATA_READY) {
		if (audcal_info.dev_type == acdb_data.device_info->dev_type) {
			/* Wait for current calibration to finish */
			/* before proccessing new cal data */
			MM_DBG("acdb send calibration is process\n");
			mutex_unlock(&acdb_data.acdb_mutex);
			goto done;
		} else {
			/* Overwite cal data with new values */
			acdb_data.acdb_state &= ~CAL_DATA_READY;
		}
	}
	memcpy(acdb_data.device_info, &audcal_info, sizeof(audcal_info));
	mutex_unlock(&acdb_data.acdb_mutex);
	acdb_data.acdb_compl = 1;
	wake_up(&acdb_data.wait);
done:
	return;
}


s32 acdb_get_calibration(void)
{
	struct acdb_cmd_get_device_table	acdb_cmd;
	s32					result = 0;
	int sz = -1;
	uint32_t iterations = 0;

	sz = acdb_get_config_table(acdb_data.device_info->acdb_id,
				acdb_data.device_info->sample_rate);

	if (sz > 0) {
		acdb_data.acdb_state |= CAL_DATA_READY;
		acdb_data.enable = 1;
	} else {
		MM_DBG("acdb state = %d\n", acdb_data.acdb_state);
		acdb_cmd.command_id = ACDB_GET_DEVICE_TABLE;
		acdb_cmd.device_id = acdb_data.device_info->acdb_id;
		acdb_cmd.network_id = 0x0108B153;
		acdb_cmd.sample_rate_id = acdb_data.device_info->sample_rate;
		acdb_cmd.total_bytes = ACDB_BUF_SIZE;
		acdb_cmd.phys_buf = (u32 *)acdb_data.phys_addr;

		do {
			result = dalrpc_fcn_8(ACDB_DalACDB_ioctl,
					acdb_data.handle,
					(const void *)&acdb_cmd,
					sizeof(acdb_cmd),
					&acdb_data.acdb_result,
					sizeof(acdb_data.acdb_result));

			if (result < 0) {
				MM_AUD_ERR("ACDB=> Device table RPC failure"
					" result = %d\n", result);
				result = -EINVAL;
				goto done;
			}

			/* following check is introduced to handle boot up race
			condition between AUDCAL SW peers running on apps
			and modem (ACDB_RES_BADSTATE indicates modem AUDCAL SW is
			not in initialized sate) we need to retry to get ACDB
			values */
			if (acdb_data.acdb_result.result == ACDB_RES_BADSTATE) {
				msleep(500);
				iterations++;
			} else if (acdb_data.acdb_result.result == ACDB_RES_SUCCESS) {
				MM_DBG("Modem query for acdb values is successful"
					" (iterations = %d)\n", iterations);
				acdb_data.acdb_state |= CAL_DATA_READY;
				acdb_data.enable = 1;
				pr_aud_info("%d: change acdb_State to %d\n",
						__LINE__,acdb_data.acdb_state);
				goto done;
			} else {
				MM_AUD_ERR("ACDB=> modem failed"
						"to fill acdb values,"
						" reuslt = %d, (iterations = %d)\n",
					acdb_data.acdb_result.result,
					iterations);
				result = -EINVAL;
				goto done;
			}
		} while (iterations < MAX_RETRY);

		MM_AUD_ERR("ACDB=> AUDCAL SW on modem is not"
			" in initialized state (%d)\n",
			acdb_data.acdb_result.result);
		result = -EINVAL;
	}

done:
	return result;
}


void audpp_cb(void *private, u32 id, u16 *msg)
{
	if (id != AUDPP_MSG_CFG_MSG)
		goto done;

	if (msg[0] == AUDPP_MSG_ENA_DIS) {
		acdb_data.acdb_state &= ~AUDPP_READY;
		MM_DBG("acdb state = %d\n", acdb_data.acdb_state);
		goto done;
	}

	acdb_data.acdb_state |= AUDPP_READY;
	if (acdb_data.acdb_state & CAL_DATA_READY) {
		acdb_data.acdb_compl = 1;
		wake_up(&acdb_data.wait);
	}
done:
	return;
}


void audpreproc_cb(void *private, u32 id, void *msg)
{
	struct audpreproc_cmd_enc_cfg_done_msg *tmp;

	if (id != AUDPREPROC_CMD_ENC_CFG_DONE_MSG)
		goto done;

	tmp = (struct audpreproc_cmd_enc_cfg_done_msg *)msg;
	acdb_data.preproc_stream_id = tmp->stream_id;
	MM_DBG("rec_enc_type = %x\n", tmp->rec_enc_type);
	if ((tmp->rec_enc_type & 0x8000) ==
				AUD_PREPROC_CONFIG_DISABLED) {
		acdb_data.acdb_state &= ~AUDPREPROC_READY;
		goto done;
	}

	acdb_data.acdb_state |= AUDPREPROC_READY;

	if (acdb_data.acdb_state & CAL_DATA_READY) {
		acdb_data.acdb_compl = 1;
		wake_up(&acdb_data.wait);
	}
done:
	return;
}


s32 acdb_send_calibration(void)
{
	s32	result = 0;

	if ((acdb_data.device_info->dev_type & RX_DEVICE) == 1) {
		result = acdb_calibrate_audpp();
		if (result)
			goto done;
	} else if ((acdb_data.device_info->dev_type & TX_DEVICE) == 2) {
		result = acdb_calibrate_audpreproc();
		if (result)
			goto done;
	}

	if (acdb_data.acdb_state & WAITING_FOR_CAL) {
		acdb_data.wait_for_cal_compl = 1;
		wake_up(&acdb_data.wait);
	}
	acdb_data.acdb_state &= ~CAL_DATA_READY;
	acdb_data.enable = 0;
done:
	return result;
}

s32 acdb_calibrate_audpp(void)
{
	s32	result = 0;

	result = acdb_fill_audpp_iir();
	if (!IS_ERR_VALUE(result)) {
	result = audpp_dsp_set_rx_iir(acdb_data.device_info->dev_id,
				acdb_data.pp_iir->active_flag,
					acdb_data.pp_iir, COPP);
	if (result) {
		MM_AUD_ERR("ACDB=> Failed to send IIR data to postproc\n");
		result = -EINVAL;
		goto done;
		} else
			MM_DBG("AUDPP is calibrated with IIR parameters"
					" for COPP ID %d\n",
						acdb_data.device_info->dev_id);
	}
	result = acdb_fill_audpp_mbadrc();
	if (!IS_ERR_VALUE(result)) {
	result = audpp_dsp_set_mbadrc(acdb_data.device_info->dev_id,
					acdb_data.pp_mbadrc->enable,
					acdb_data.pp_mbadrc, COPP);
	if (result) {
			MM_AUD_ERR("ACDB=> Failed to send MBADRC data to"
					" postproc\n");
		result = -EINVAL;
		goto done;
		} else
			MM_DBG("AUDPP is calibrated with MBADRC parameters"
					" for COPP ID %d\n",
					acdb_data.device_info->dev_id);
	}
done:
	return result;
}

struct acdb_iir_block *get_audpp_irr_block()
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);
		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_IIR_RX) {
				if (prs_hdr->iid == IID_AUDIO_IIR_COEFF)
					return (struct acdb_iir_block *)
						(acdb_data.virt_addr + index
						 + sizeof(struct header));
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}


s32 acdb_fill_audpp_iir(void)
{
	struct acdb_iir_block *acdb_iir;
	s32 i = 0;

	acdb_iir = get_audpp_irr_block();
	if (acdb_iir == NULL) {
		MM_AUD_ERR("unable to find  audpp iir block returning\n");
		return -1;
	}
	memset(acdb_data.pp_iir, 0, sizeof(*acdb_data.pp_iir));

	acdb_data.pp_iir->common.cmd_id = AUDPP_CMD_CFG_OBJECT_PARAMS;
	acdb_data.pp_iir->common.stream = AUDPP_CMD_COPP_STREAM;
	acdb_data.pp_iir->common.stream_id = 0;
	acdb_data.pp_iir->common.obj_cfg = AUDPP_CMD_OBJ0_UPDATE;
	acdb_data.pp_iir->common.command_type = 0;

	acdb_data.pp_iir->active_flag = acdb_iir->enable_flag;
	acdb_data.pp_iir->num_bands = acdb_iir->stage_count;
	for (; i < acdb_iir->stage_count; i++) {
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b0_filter_lsw =
			acdb_iir->stages[i].b0_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b0_filter_msw =
			acdb_iir->stages[i].b0_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b1_filter_lsw =
			acdb_iir->stages[i].b1_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b1_filter_msw =
			acdb_iir->stages[i].b1_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b2_filter_lsw =
			acdb_iir->stages[i].b2_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			numerator_filter[i].numerator_b2_filter_msw =
			acdb_iir->stages[i].b2_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a0_filter_lsw =
			acdb_iir->stages_a[i].a1_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a0_filter_msw =
			acdb_iir->stages_a[i].a1_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a1_filter_lsw =
			acdb_iir->stages_a[i].a2_lo;
		acdb_data.pp_iir->params_filter.filter_4_params.
			denominator_filter[i].denominator_a1_filter_msw =
			acdb_iir->stages_a[i].a2_hi;
		acdb_data.pp_iir->params_filter.filter_4_params.
			shift_factor_filter[i].shift_factor_0 =
			acdb_iir->shift_factor[i];
		acdb_data.pp_iir->params_filter.filter_4_params.pan_filter[i].
			pan_filter_0 = acdb_iir->pan[i];
	}
	return 0;
}

void get_aupp_mbadrc_block(u32 *phy_addr)
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);

		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_MBADRC_RX) {
				if ((prs_hdr->iid == IID_MBADRC_EXT_BUFF)
					|| (prs_hdr->iid ==
						IID_MBADRC_BAND_CONFIG)
					|| (prs_hdr->iid ==
						IID_MBADRC_PARAMETERS)) {
					if (prs_hdr->iid ==
							 IID_MBADRC_EXT_BUFF) {
						MM_DBG(" \
						Got IID \
						 = IID_MBADRC_EXT_BUFF\n");
						*phy_addr = acdb_data.phys_addr\
							+ index
						+ sizeof(struct header);
						memcpy(acdb_data. \
							mbadrc_block.ext_buf,
							(acdb_data.virt_addr
								 + index +
								sizeof(
							struct header)),
								 196*2);
						MM_DBG("phy_addr = %x",
							 *phy_addr);
						index += prs_hdr->data_len +
							sizeof(struct header);
					} else if (prs_hdr->iid
						 == IID_MBADRC_BAND_CONFIG) {
						MM_DBG("Got IID \
						== IID_MBADRC_BAND_CONFIG\n");
						if (acdb_data. \
							mbadrc_block.parameters\
							.mbadrc_num_bands > mbadrc_num_bands) {
							MM_AUD_ERR("mbadrc bands \
							number too much.");
							return;
						}
						memcpy(acdb_data. \
						mbadrc_block.band_config,
							(acdb_data. \
							virt_addr +
								index +
							sizeof(struct header)),
							sizeof(struct
							mbadrc_band_config_type\
							) * acdb_data. \
							mbadrc_block.parameters\
							.mbadrc_num_bands);
						index += prs_hdr->data_len +
							sizeof(struct header);
					} else if (prs_hdr->iid
						 == IID_MBADRC_PARAMETERS) {
						struct mbadrc_parameter \
								*tmp;
						tmp = (
						struct mbadrc_parameter *) \
							(acdb_data.virt_addr
							+ index +
							sizeof(struct header));
						MM_DBG("Got IID\
						 == IID_MBADRC_PARAMETERS\n");
						acdb_data.mbadrc_block.
						parameters.mbadrc_enable =
							tmp->mbadrc_enable;
						acdb_data.mbadrc_block. \
						parameters.mbadrc_num_bands =
							tmp->mbadrc_num_bands;
						acdb_data.mbadrc_block. \
							parameters. \
						mbadrc_down_sample_level =
						tmp->mbadrc_down_sample_level;
						acdb_data.mbadrc_block.
						parameters.mbadrc_delay =
							tmp->mbadrc_delay;
						index += prs_hdr->data_len
							+ sizeof(struct \
								 header);
					}
				}
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
}

s32 acdb_fill_audpp_mbadrc(void)
{
	u32		mbadrc_phys_addr = 0;

	get_aupp_mbadrc_block(&mbadrc_phys_addr);

	if (IS_ERR_VALUE(mbadrc_phys_addr)) {
		MM_AUD_ERR("failed to get mbadrc block\n");
		return -1;
	}

	memset(acdb_data.pp_mbadrc, 0, sizeof(*acdb_data.pp_mbadrc));

	acdb_data.pp_mbadrc->common.cmd_id = AUDPP_CMD_CFG_OBJECT_PARAMS;
	acdb_data.pp_mbadrc->common.stream = AUDPP_CMD_COPP_STREAM;
	acdb_data.pp_mbadrc->common.stream_id = 0;
	acdb_data.pp_mbadrc->common.obj_cfg = AUDPP_CMD_OBJ0_UPDATE;
	acdb_data.pp_mbadrc->common.command_type = 0;

	acdb_data.pp_mbadrc->enable = acdb_data.mbadrc_block.\
					parameters.mbadrc_enable;
	acdb_data.pp_mbadrc->num_bands =
				acdb_data.mbadrc_block.\
					parameters.mbadrc_num_bands;
	acdb_data.pp_mbadrc->down_samp_level =
				acdb_data.mbadrc_block.parameters.\
					mbadrc_down_sample_level;
	acdb_data.pp_mbadrc->adrc_delay =
				acdb_data.mbadrc_block.parameters.\
					mbadrc_delay;

	if (acdb_data.mbadrc_block.parameters.mbadrc_num_bands > 1)
		acdb_data.pp_mbadrc->ext_buf_size = (97 * 2) +
			(33 * 2 * (acdb_data.mbadrc_block.parameters.\
					mbadrc_num_bands - 2));

	acdb_data.pp_mbadrc->ext_partition = 0;
	acdb_data.pp_mbadrc->ext_buf_lsw = (u16)(mbadrc_phys_addr\
						 & 0xFFFF);
	acdb_data.pp_mbadrc->ext_buf_msw = (u16)((mbadrc_phys_addr\
						 & 0xFFFF0000) >> 16);
	if (acdb_data.mbadrc_block.parameters.mbadrc_num_bands > mbadrc_num_bands) {
		MM_AUD_ERR("mbadrc bands number too much.");
		return -1;
	}
	memcpy(acdb_data.pp_mbadrc->adrc_band, acdb_data.mbadrc_block.\
					band_config,
		sizeof(struct mbadrc_band_config_type) *
			acdb_data.mbadrc_block.parameters.mbadrc_num_bands);
	return 0;
}


s32 acdb_calibrate_audpreproc(void)
{
	s32	result = 0;

	result = acdb_fill_audpreproc_agc();
	if (!IS_ERR_VALUE(result)) {
	result = audpreproc_dsp_set_agc(acdb_data.preproc_agc, sizeof(
					struct audpreproc_cmd_cfg_agc_params));
	if (result) {
		MM_AUD_ERR("ACDB=> Failed to send AGC data to preproc)\n");
		result = -EINVAL;
		goto done;
		} else
			MM_DBG("AUDPREC is calibrated with AGC parameters"
				" for COPP ID %d and AUDREC session %d\n",
					acdb_data.device_info->dev_id,
					acdb_data.preproc_stream_id);
	}
	result = acdb_fill_audpreproc_iir();
	if (!IS_ERR_VALUE(result)) {
		result = audpreproc_dsp_set_iir(acdb_data.preproc_iir,
				sizeof(struct\
				audpreproc_cmd_cfg_iir_tuning_filter_params));
	if (result) {
		MM_AUD_ERR("ACDB=> Failed to send IIR data to preproc\n");
		result = -EINVAL;
		goto done;
		} else
			MM_DBG("audpreproc is calibrated with iir parameters"
			" for COPP ID %d and AUREC session %d\n",
					acdb_data.device_info->dev_id,
					acdb_data.preproc_stream_id);
	}
done:
	return result;
}

struct acdb_agc_block *get_audpreproc_agc_block()
{
	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);
		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_AGC_TX) {
				if (prs_hdr->iid == IID_AUDIO_AGC_PARAMETERS) {
					MM_DBG("GOT ABID_AUDIO_AGC_TX\n");
					return (struct acdb_agc_block *)
						(acdb_data.virt_addr + index
						 + sizeof(struct header));
				}
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}

s32 acdb_fill_audpreproc_agc(void)
{
	struct acdb_agc_block	*acdb_agc;

	acdb_agc = get_audpreproc_agc_block();
	if (!acdb_agc) {
		MM_DBG("unable to find preproc agc parameters winding up\n");
		return -1;
	}
	memset(acdb_data.preproc_agc, 0, sizeof(*acdb_data.preproc_agc));
	acdb_data.preproc_agc->cmd_id = AUDPREPROC_CMD_CFG_AGC_PARAMS;
	acdb_data.preproc_agc->stream_id = acdb_data.preproc_stream_id;
	/* 0xFE00 to configure all parameters */
	acdb_data.preproc_agc->tx_agc_param_mask = 0xFFFF;

	if (acdb_agc->enable_status)
		acdb_data.preproc_agc->tx_agc_enable_flag =
			AUDPREPROC_CMD_TX_AGC_ENA_FLAG_ENA;
	else
		acdb_data.preproc_agc->tx_agc_enable_flag =
			AUDPREPROC_CMD_TX_AGC_ENA_FLAG_DIS;

	acdb_data.preproc_agc->comp_rlink_static_gain =
		acdb_agc->comp_rlink_static_gain;
	acdb_data.preproc_agc->comp_rlink_aig_flag =
		acdb_agc->comp_rlink_aig_flag;
	acdb_data.preproc_agc->expander_rlink_th =
		acdb_agc->exp_rlink_threshold;
	acdb_data.preproc_agc->expander_rlink_slope =
		acdb_agc->exp_rlink_slope;
	acdb_data.preproc_agc->compressor_rlink_th =
		acdb_agc->comp_rlink_threshold;
	acdb_data.preproc_agc->compressor_rlink_slope =
		acdb_agc->comp_rlink_slope;

	/* 0xFFF0 to configure all parameters */
	acdb_data.preproc_agc->tx_adc_agc_param_mask = 0xFFFF;

	acdb_data.preproc_agc->comp_rlink_aig_attackk =
		acdb_agc->comp_rlink_aig_attack_k;
	acdb_data.preproc_agc->comp_rlink_aig_leak_down =
		acdb_agc->comp_rlink_aig_leak_down;
	acdb_data.preproc_agc->comp_rlink_aig_leak_up =
		acdb_agc->comp_rlink_aig_leak_up;
	acdb_data.preproc_agc->comp_rlink_aig_max =
		acdb_agc->comp_rlink_aig_max;
	acdb_data.preproc_agc->comp_rlink_aig_min =
		acdb_agc->comp_rlink_aig_min;
	acdb_data.preproc_agc->comp_rlink_aig_releasek =
		acdb_agc->comp_rlink_aig_release_k;
	acdb_data.preproc_agc->comp_rlink_aig_leakrate_fast =
		acdb_agc->comp_rlink_aig_sm_leak_rate_fast;
	acdb_data.preproc_agc->comp_rlink_aig_leakrate_slow =
		acdb_agc->comp_rlink_aig_sm_leak_rate_slow;
	acdb_data.preproc_agc->comp_rlink_attackk_msw =
		acdb_agc->comp_rlink_attack_k_msw;
	acdb_data.preproc_agc->comp_rlink_attackk_lsw =
		acdb_agc->comp_rlink_attack_k_lsw;
	acdb_data.preproc_agc->comp_rlink_delay =
		acdb_agc->comp_rlink_delay;
	acdb_data.preproc_agc->comp_rlink_releasek_msw =
		acdb_agc->comp_rlink_release_k_msw;
	acdb_data.preproc_agc->comp_rlink_releasek_lsw =
		acdb_agc->comp_rlink_release_k_lsw;
	acdb_data.preproc_agc->comp_rlink_rms_tav =
		acdb_agc->comp_rlink_rms_trav;
	return 0;
}

struct acdb_iir_block *get_audpreproc_irr_block()
{

	struct header *prs_hdr;
	u32 index = 0;

	while (index < acdb_data.acdb_result.used_bytes) {
		prs_hdr = (struct header *)(acdb_data.virt_addr + index);

		if (prs_hdr->dbor_signature == DBOR_SIGNATURE) {
			if (prs_hdr->abid == ABID_AUDIO_IIR_TX) {
				if (prs_hdr->iid == IID_AUDIO_IIR_COEFF)
					return (struct acdb_iir_block *)
						(acdb_data.virt_addr + index
						 + sizeof(struct header));
			} else {
				index += prs_hdr->data_len +
						sizeof(struct header);
			}
		} else {
			break;
		}
	}
	return NULL;
}


s32 acdb_fill_audpreproc_iir(void)
{
	struct acdb_iir_block	*acdb_iir;


	acdb_iir =  get_audpreproc_irr_block();
	if (!acdb_iir) {
		MM_DBG("unable to find preproc iir parameters winding up\n");
		return -1;
	}
	memset(acdb_data.preproc_iir, 0, sizeof(*acdb_data.preproc_iir));

	acdb_data.preproc_iir->cmd_id =
		AUDPREPROC_CMD_CFG_IIR_TUNING_FILTER_PARAMS;
	acdb_data.preproc_iir->stream_id = acdb_data.preproc_stream_id;
	acdb_data.preproc_iir->active_flag = acdb_iir->enable_flag;
	acdb_data.preproc_iir->num_bands = acdb_iir->stage_count;

	acdb_data.preproc_iir->numerator_coeff_b0_filter0_lsw =
		acdb_iir->stages[0].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter0_msw =
		acdb_iir->stages[0].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter0_lsw =
		acdb_iir->stages[0].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter0_msw =
		acdb_iir->stages[0].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter0_lsw =
		acdb_iir->stages[0].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter0_msw =
		acdb_iir->stages[0].b2_hi;

	acdb_data.preproc_iir->numerator_coeff_b0_filter1_lsw =
		acdb_iir->stages[1].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter1_msw =
		acdb_iir->stages[1].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter1_lsw =
		acdb_iir->stages[1].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter1_msw =
		acdb_iir->stages[1].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter1_lsw =
		acdb_iir->stages[1].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter1_msw =
		acdb_iir->stages[1].b2_hi;

	acdb_data.preproc_iir->numerator_coeff_b0_filter2_lsw =
		acdb_iir->stages[2].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter2_msw =
		acdb_iir->stages[2].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter2_lsw =
		acdb_iir->stages[2].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter2_msw =
		acdb_iir->stages[2].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter2_lsw =
		acdb_iir->stages[2].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter2_msw =
		acdb_iir->stages[2].b2_hi;

	acdb_data.preproc_iir->numerator_coeff_b0_filter3_lsw =
		acdb_iir->stages[3].b0_lo;
	acdb_data.preproc_iir->numerator_coeff_b0_filter3_msw =
		acdb_iir->stages[3].b0_hi;
	acdb_data.preproc_iir->numerator_coeff_b1_filter3_lsw =
		acdb_iir->stages[3].b1_lo;
	acdb_data.preproc_iir->numerator_coeff_b1_filter3_msw =
		acdb_iir->stages[3].b1_hi;
	acdb_data.preproc_iir->numerator_coeff_b2_filter3_lsw =
		acdb_iir->stages[3].b2_lo;
	acdb_data.preproc_iir->numerator_coeff_b2_filter3_msw =
		acdb_iir->stages[3].b2_hi;

	acdb_data.preproc_iir->denominator_coeff_a0_filter0_lsw =
		acdb_iir->stages_a[0].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter0_msw =
		acdb_iir->stages_a[0].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter0_lsw =
		acdb_iir->stages_a[0].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter0_msw =
		acdb_iir->stages_a[0].a2_hi;

	acdb_data.preproc_iir->denominator_coeff_a0_filter1_lsw =
		acdb_iir->stages_a[1].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter1_msw =
		acdb_iir->stages_a[1].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter1_lsw =
		acdb_iir->stages_a[1].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter1_msw =
		acdb_iir->stages_a[1].a2_hi;
	acdb_data.preproc_iir->denominator_coeff_a0_filter2_lsw =
		acdb_iir->stages_a[2].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter2_msw =
		acdb_iir->stages_a[2].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter2_lsw =
		acdb_iir->stages_a[2].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter2_msw =
		acdb_iir->stages_a[2].a2_hi;
	acdb_data.preproc_iir->denominator_coeff_a0_filter3_lsw =
		acdb_iir->stages_a[3].a1_lo;
	acdb_data.preproc_iir->denominator_coeff_a0_filter3_msw =
		acdb_iir->stages_a[3].a1_hi;
	acdb_data.preproc_iir->denominator_coeff_a1_filter3_lsw =
		acdb_iir->stages_a[3].a2_lo;
	acdb_data.preproc_iir->denominator_coeff_a1_filter3_msw =
		acdb_iir->stages_a[3].a2_hi;

	acdb_data.preproc_iir->shift_factor_filter0 =
		acdb_iir->shift_factor[0];
	acdb_data.preproc_iir->shift_factor_filter1 =
		acdb_iir->shift_factor[1];
	acdb_data.preproc_iir->shift_factor_filter2 =
		acdb_iir->shift_factor[2];
	acdb_data.preproc_iir->shift_factor_filter3 =
		acdb_iir->shift_factor[3];

	acdb_data.preproc_iir->pan_of_filter0 =
		acdb_iir->pan[0];
	acdb_data.preproc_iir->pan_of_filter1 =
		acdb_iir->pan[1];
	acdb_data.preproc_iir->pan_of_filter2 =
		acdb_iir->pan[2];
	acdb_data.preproc_iir->pan_of_filter3 =
		acdb_iir->pan[3];

	return 0;
}

static void __exit acdb_exit(void)
{
	s32	result = 0;

	result = auddev_unregister_evt_listner(AUDDEV_CLNT_AUDIOCAL, 0);
	if (result)
		MM_AUD_ERR("ACDB=> Could not unregister device callback\n");

	result = audpp_unregister_event_callback(&acdb_data.audpp_cb);
	if (result)
		MM_AUD_ERR("ACDB=> Could not unregister audpp callback\n");

	result = audpreproc_unregister_event_callback(&acdb_data.\
				audpreproc_cb);
	if (result)
		MM_AUD_ERR("ACDB=> Could not unregister audpreproc callback\n");

	result = kthread_stop(acdb_data.cb_thread_task);
	if (result)
		MM_AUD_ERR("ACDB=> Could not stop kthread\n");

	if (acdb_data.phys_addr)
		pmem_kfree(acdb_data.phys_addr);

	kfree(acdb_data.device_info);
	kfree(acdb_data.pp_iir);
	kfree(acdb_data.pp_mbadrc);
	kfree(acdb_data.preproc_agc);
	kfree(acdb_data.preproc_iir);

	mutex_destroy(&acdb_data.acdb_mutex);
	memset(&acdb_data, 0, sizeof(acdb_data));
}

late_initcall(acdb_init);
module_exit(acdb_exit);

MODULE_DESCRIPTION("MSM 7x30 Audio ACDB driver");
MODULE_LICENSE("Dual BSD/GPL");
