/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/diagchar.h>
#include <linux/platform_device.h>
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_cntl.h"
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif

#define HDR_SIZ 8

static void diag_smd_cntl_send_req(int proc_num)
{
	int data_len = 0, type = -1, count_bytes = 0, j, r, flag = 0;
	struct bindpkt_params_per_process *pkt_params =
		 kzalloc(sizeof(struct bindpkt_params_per_process), GFP_KERNEL);
	struct diag_ctrl_msg *msg;
	struct cmd_code_range *range;
	struct bindpkt_params *temp;
	void *buf = NULL, *dump_buf = NULL;
	smd_channel_t *smd_ch = NULL;

	DIAG_INFO("%s: %s\n", __func__,
		(proc_num == MODEM_PROC)?"MODEM_PROC":
		(proc_num == QDSP_PROC)?"QDSP_PROC":"WCNSS_PROC");

	if (proc_num == MODEM_PROC) {
		buf = driver->buf_in_cntl;
		smd_ch = driver->ch_cntl;
	} else if (proc_num == QDSP_PROC) {
		buf = driver->buf_in_qdsp_cntl;
		smd_ch = driver->chqdsp_cntl;
	} else if (proc_num == WCNSS_PROC) {
		buf = driver->buf_in_wcnss_cntl;
		smd_ch = driver->ch_wcnss_cntl;
	}

	if (!smd_ch || !buf) {
		kfree(pkt_params);
		return;
	}

	r = smd_read_avail(smd_ch);
	if (r > IN_BUF_SIZE) {
		if (r < MAX_IN_BUF_SIZE) {
			pr_err("diag: SMD CNTL sending pkt upto %d bytes", r);
			buf = krealloc(buf, r, GFP_KERNEL);
		} else {
			pr_err("diag: CNTL pkt > %d bytes", MAX_IN_BUF_SIZE);
			kfree(pkt_params);
			return;
		}
	}
	if (buf && r > 0) {
		smd_read(smd_ch, buf, r);
		while (count_bytes + HDR_SIZ <= r) {
			type = *(uint32_t *)(buf);
			data_len = *(uint32_t *)(buf + 4);
			count_bytes = count_bytes+HDR_SIZ+data_len;
			if (type == DIAG_CTRL_MSG_REG && r >= count_bytes) {
				msg = buf+HDR_SIZ;
				if (!msg->count_entries) {
					DIAG_ERR("version: %d, cmd_code: %d,"
						" subsysid: %d, count_entries: %d,"
						" port:%d\n", msg->version,
						msg->cmd_code, msg->subsysid,
						msg->count_entries, msg->port);
					dump_buf = kmalloc(r, GFP_KERNEL);
					memcpy(dump_buf, buf, r);
					continue;
				}
				range = buf+HDR_SIZ+
						sizeof(struct diag_ctrl_msg);
				pkt_params->count = msg->count_entries;
				temp = kzalloc(pkt_params->count * sizeof(struct
						 bindpkt_params), GFP_KERNEL);
				for (j = 0; j < pkt_params->count; j++) {
					temp->cmd_code = msg->cmd_code;
					temp->subsys_id = msg->subsysid;
					temp->client_id = proc_num;
					temp->proc_id = proc_num;
					temp->cmd_code_lo = range->cmd_code_lo;
					temp->cmd_code_hi = range->cmd_code_hi;
					range++;
					temp++;
				}
				temp -= pkt_params->count;
				pkt_params->params = temp;
				flag = 1;
				diagchar_ioctl(NULL, DIAG_IOCTL_COMMAND_REG,
						 (unsigned long)pkt_params);
				kfree(temp);
				buf = buf + HDR_SIZ + data_len;
			}
		}
	}
	if (dump_buf) {
		print_hex_dump(KERN_DEBUG, "diag_debug_buf:",
			16, 1, DUMP_PREFIX_ADDRESS, dump_buf, r, 1);
		kfree(dump_buf);
	}
	kfree(pkt_params);
	if (flag) {
		/* Poll SMD CNTL channels to check for data */
		queue_work(driver->diag_wq, &(driver->diag_read_smd_cntl_work));
		queue_work(driver->diag_wq,
			 &(driver->diag_read_smd_qdsp_cntl_work));
		queue_work(driver->diag_wq,
			 &(driver->diag_read_smd_wcnss_cntl_work));
	}
}

void diag_read_smd_cntl_work_fn(struct work_struct *work)
{
	diag_smd_cntl_send_req(MODEM_PROC);
}

void diag_read_smd_qdsp_cntl_work_fn(struct work_struct *work)
{
	diag_smd_cntl_send_req(QDSP_PROC);
}

void diag_read_smd_wcnss_cntl_work_fn(struct work_struct *work)
{
	diag_smd_cntl_send_req(WCNSS_PROC);
}

static void diag_smd_cntl_notify(void *ctxt, unsigned event)
{
	queue_work(driver->diag_wq, &(driver->diag_read_smd_cntl_work));
}

static void diag_smd_qdsp_cntl_notify(void *ctxt, unsigned event)
{
	queue_work(driver->diag_wq, &(driver->diag_read_smd_qdsp_cntl_work));
}

static void diag_smd_wcnss_cntl_notify(void *ctxt, unsigned event)
{
	queue_work(driver->diag_wq, &(driver->diag_read_smd_wcnss_cntl_work));
}

static int diag_smd_cntl_probe(struct platform_device *pdev)
{
	int r = 0;

	/* open control ports only on 8960 */
	if (chk_config_get_id() == AO8960_TOOLS_ID) {
		if (pdev->id == SMD_APPS_MODEM)
			r = smd_open("DIAG_CNTL", &driver->ch_cntl, driver,
							diag_smd_cntl_notify);
		if (pdev->id == SMD_APPS_QDSP)
			r = smd_named_open_on_edge("DIAG_CNTL", SMD_APPS_QDSP
				, &driver->chqdsp_cntl, driver,
					 diag_smd_qdsp_cntl_notify);
		if (pdev->id == SMD_APPS_WCNSS)
			r = smd_named_open_on_edge("APPS_RIVA_CTRL",
				SMD_APPS_WCNSS, &driver->ch_wcnss_cntl,
					driver, diag_smd_wcnss_cntl_notify);
		pr_debug("diag: open CNTL port, ID = %d,r = %d\n", pdev->id, r);
	}
	return 0;
}

static int diagfwd_cntl_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diagfwd_cntl_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diagfwd_cntl_dev_pm_ops = {
	.runtime_suspend = diagfwd_cntl_runtime_suspend,
	.runtime_resume = diagfwd_cntl_runtime_resume,
};

static struct platform_driver msm_smd_ch1_cntl_driver = {

	.probe = diag_smd_cntl_probe,
	.driver = {
			.name = "DIAG_CNTL",
			.owner = THIS_MODULE,
			.pm   = &diagfwd_cntl_dev_pm_ops,
		   },
};

static struct platform_driver diag_smd_lite_cntl_driver = {

	.probe = diag_smd_cntl_probe,
	.driver = {
			.name = "APPS_RIVA_CTRL",
			.owner = THIS_MODULE,
			.pm   = &diagfwd_cntl_dev_pm_ops,
		   },
};

void diagfwd_cntl_init(void)
{
	if (driver->buf_in_cntl == NULL) {
		driver->buf_in_cntl = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_cntl == NULL)
			goto err;
	}
	if (driver->buf_in_qdsp_cntl == NULL) {
		driver->buf_in_qdsp_cntl = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_qdsp_cntl == NULL)
			goto err;
	}
	if (driver->buf_in_wcnss_cntl == NULL) {
		driver->buf_in_wcnss_cntl = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (driver->buf_in_wcnss_cntl == NULL)
			goto err;
	}
	platform_driver_register(&msm_smd_ch1_cntl_driver);
	platform_driver_register(&diag_smd_lite_cntl_driver);

	return;
err:
		pr_err("diag: Could not initialize diag buffers");
		kfree(driver->buf_in_cntl);
		kfree(driver->buf_in_qdsp_cntl);
		kfree(driver->buf_in_wcnss_cntl);
}

void diagfwd_cntl_exit(void)
{
	smd_close(driver->ch_cntl);
	smd_close(driver->chqdsp_cntl);
	smd_close(driver->ch_wcnss_cntl);
	driver->ch_cntl = 0;
	driver->chqdsp_cntl = 0;
	driver->ch_wcnss_cntl = 0;
	platform_driver_unregister(&msm_smd_ch1_cntl_driver);
	platform_driver_unregister(&diag_smd_lite_cntl_driver);

	kfree(driver->buf_in_cntl);
	kfree(driver->buf_in_qdsp_cntl);
	kfree(driver->buf_in_wcnss_cntl);
}
