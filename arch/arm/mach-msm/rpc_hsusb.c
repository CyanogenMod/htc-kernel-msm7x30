/* linux/arch/arm/mach-msm/rpc_hsusb.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/err.h>
#include <mach/rpc_hsusb.h>
#include <asm/mach-types.h>

static struct msm_rpc_endpoint *usb_ep;

#define MSM_RPC_CHG_PROG 0x3000001a

struct msm_chg_rpc_ids {
	unsigned long	vers_comp;
	unsigned	chg_usb_charger_connected_proc;
	unsigned	chg_usb_charger_disconnected_proc;
	unsigned	chg_usb_i_is_available_proc;
	unsigned	chg_usb_i_is_not_available_proc;
};

struct msm_hsusb_rpc_ids {
	unsigned long	prog;
	unsigned long	vers_comp;
	unsigned long	init_phy;
	unsigned long	vbus_pwr_up;
	unsigned long	vbus_pwr_down;
	unsigned long	update_product_id;
	unsigned long	update_serial_num;
	unsigned long	update_is_serial_num_null;
	unsigned long	reset_rework_installed;
	unsigned long	enable_pmic_ulpi_data0;
	unsigned long	disable_pmic_ulpi_data0;
};

static struct msm_hsusb_rpc_ids usb_rpc_ids;

static int msm_hsusb_init_rpc_ids(unsigned long vers)
{

		usb_rpc_ids.prog			= 0x30000064;
		usb_rpc_ids.init_phy			= 2;
		usb_rpc_ids.vbus_pwr_up			= 6;
		usb_rpc_ids.vbus_pwr_down		= 7;
		usb_rpc_ids.update_product_id		= 8;
		usb_rpc_ids.update_serial_num		= 9;
		usb_rpc_ids.update_is_serial_num_null	= 10;
		usb_rpc_ids.reset_rework_installed	= 17;
		usb_rpc_ids.enable_pmic_ulpi_data0	= 18;
		usb_rpc_ids.disable_pmic_ulpi_data0	= 19;

	if (vers == 0x00010001) {
		usb_rpc_ids.vers_comp			= 0x00010001;
		return 0;
	} else if (vers == 0x00010002) {
		usb_rpc_ids.vers_comp			= 0x00010002;
		return 0;
	} else if (vers == 0x00010003) {
		usb_rpc_ids.vers_comp			= 0x00010003;
		return 0;
	} else {
		pr_info("%s: no matches found for version\n",
			__func__);
		return -ENODATA;
	}
}

/* rpc connect for hsusb */
int msm_hsusb_rpc_connect(void)
{
	int ver;
	int i;

	if (usb_ep && !IS_ERR(usb_ep)) {
		pr_info("%s: usb_ep already connected\n", __func__);
		return 0;
	}

	/* Initialize rpc ids */
	for (i = 0, ver = 0x00010003; i < 3; i++) {
		if (msm_hsusb_init_rpc_ids(ver-i)) {
			pr_err("%s: rpc ids initialization failed\n"
				, __func__);
			return -ENODATA;
		}

		usb_ep = msm_rpc_connect(usb_rpc_ids.prog,
						usb_rpc_ids.vers_comp,
						MSM_RPC_UNINTERRUPTIBLE);

		if (IS_ERR(usb_ep)) {
			pr_err("%s: connect compatible failed vers = %lx\n",
					__func__, usb_rpc_ids.vers_comp);

		} else {
			pr_err("%s: rpc connect success vers = %lx\n",
					__func__, usb_rpc_ids.vers_comp);
			return 0;
		}
	}
	return -EAGAIN;
}
EXPORT_SYMBOL(msm_hsusb_rpc_connect);

/* rpc call for phy_reset */
void msm_hsusb_phy_reset(void)
{
	int rc = 0;
	struct hsusb_phy_start_req {
		struct rpc_request_hdr hdr;
	} req;

	if (!usb_ep || IS_ERR(usb_ep)) {
		pr_err("%s: phy_reset rpc failed before call,"
			"rc = %ld\n", __func__, PTR_ERR(usb_ep));
		return;
	}

	rc = msm_rpc_call(usb_ep, usb_rpc_ids.init_phy,
				&req, sizeof(req), 5 * HZ);

	if (rc < 0) {
		pr_err("%s: phy_reset rpc failed! rc = %d\n",
			__func__, rc);
	} else
		pr_info("msm_hsusb_phy_reset\n");

	return;
}
EXPORT_SYMBOL(msm_hsusb_phy_reset);

/* rpc call to close connection */
int msm_hsusb_rpc_close(void)
{
	int rc = 0;

	if (IS_ERR(usb_ep)) {
		pr_err("%s: rpc_close failed before call, rc = %ld\n",
			__func__, PTR_ERR(usb_ep));
		return -EAGAIN;
	}

	rc = msm_rpc_close(usb_ep);
	usb_ep = NULL;

	if (rc < 0) {
		pr_err("%s: close rpc failed! rc = %d\n",
			__func__, rc);
		return -EAGAIN;
	} else
		pr_debug("rpc close success\n");

	return rc;
}
EXPORT_SYMBOL(msm_hsusb_rpc_close);

