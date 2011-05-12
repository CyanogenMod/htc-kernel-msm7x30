/* linux/include/asm-arm/arch-msm/hsusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_HSUSB_H
#define __ASM_ARCH_MSM_HSUSB_H

#include <linux/types.h>

/* platform device data for msm_hsusb driver */

#ifdef CONFIG_USB_FUNCTION
/* matches a product ID to a list of enabled functions */
struct msm_hsusb_product {
	/* product ID for usb_device_descriptor.idProduct */
	__u16 product_id;

	/* bit mask of enabled usb_functions, matching ordering
	** in msm_hsusb_platform_data.functions
	*/
	__u32 functions;
};
#endif

enum chg_type {
	USB_CHG_TYPE__SDP,
	USB_CHG_TYPE__CARKIT,
	USB_CHG_TYPE__WALLCHARGER,
	USB_CHG_TYPE__INVALID
};

enum pre_emphasis_level {
	PRE_EMPHASIS_DEFAULT,
	PRE_EMPHASIS_DISABLE,
	PRE_EMPHASIS_WITH_10_PERCENT = (1 << 5),
	PRE_EMPHASIS_WITH_20_PERCENT = (3 << 4),
};
enum cdr_auto_reset {
	CDR_AUTO_RESET_DEFAULT,
	CDR_AUTO_RESET_ENABLE,
	CDR_AUTO_RESET_DISABLE,
};

enum se1_gate_state {
	SE1_GATING_DEFAULT,
	SE1_GATING_ENABLE,
	SE1_GATING_DISABLE,
};

enum hs_drv_amplitude {
	HS_DRV_AMPLITUDE_DEFAULT,
	HS_DRV_AMPLITUDE_ZERO_PERCENT,
	HS_DRV_AMPLITUDE_25_PERCENTI = (1 << 2),
	HS_DRV_AMPLITUDE_5_PERCENT = (1 << 3),
	HS_DRV_AMPLITUDE_75_PERCENT = (3 << 2),
};

/* used to configure the analog switch to select b/w host and peripheral */
enum usb_switch_control {
	USB_SWITCH_PERIPHERAL = 0,	/* Configure switch in peripheral mode*/
	USB_SWITCH_HOST,		/* Host mode */
	USB_SWITCH_DISABLE,		/* No mode selected, shutdown power */
};

struct msm_hsusb_platform_data {
	/* hard reset the ULPI PHY */
	void (*phy_reset)(void);
	void (*phy_shutdown)(void);

	/* (de)assert the reset to the usb core */
	void (*hw_reset)(bool enable);

	/* for notification when USB is connected or disconnected */
	void (*usb_connected)(int);
	/* 1 : uart, 0 : usb */
	void (*usb_uart_switch)(int);
	void (*config_usb_id_gpios)(bool enable);
	void (*usb_hub_enable)(bool);
	void (*serial_debug_gpios)(int);
	int (*china_ac_detect)(void);
	void (*disable_usb_charger)(void);
	/* val, reg pairs terminated by -1 */
	int *phy_init_seq;
	void (*change_phy_voltage)(int);
	int (*ldo_init) (int init);
	int (*ldo_enable) (int enable);
	int (*rpc_connect)(int);
	/* 1 : mhl, 0 : usb */
	void (*usb_mhl_switch)(bool);
#ifdef CONFIG_USB_FUNCTION
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID.
	** This can be overridden dynamically based on the disabled
	** state of the functions using the product_table.
	*/
	__u16 product_id;

	__u16 version;
	char *product_name;
	char *manufacturer_name;

	/* list of function drivers to bind to this configuration */
	int num_functions;
	char **functions;

	/* if num_products is zero, then the default value in product_id
	** is used for the configuration descriptor.
	*/
	int num_products;
	struct msm_hsusb_product *products;
#endif
	char *serial_number;
	int usb_id_pin_gpio;
	int dock_pin_gpio;
	int id_pin_irq;
	bool enable_car_kit_detect;
	__u8 accessory_detect;
	bool dock_detect;

	int ac_9v_gpio;
};

struct msm_otg_platform_data {
	int (*rpc_connect)(int);
	int (*phy_reset)(void __iomem *);
	unsigned int core_clk;
	int pmic_vbus_irq;
	/* if usb link is in sps there is no need for
	 * usb pclk as dayatona fabric clock will be
	 * used instead
	 */
	int usb_in_sps;
	enum pre_emphasis_level	pemp_level;
	enum cdr_auto_reset	cdr_autoreset;
	enum hs_drv_amplitude	drv_ampl;
	enum se1_gate_state	se1_gating;
	int			phy_reset_sig_inverted;
	int			phy_can_powercollapse;
	int			pclk_required_during_lpm;

	/* HSUSB core in 8660 has the capability to gate the
	 * pclk when not being used. Though this feature is
	 * now being disabled because of H/w issues
	 */
	int			pclk_is_hw_gated;

	int (*ldo_init) (int init);
	int (*ldo_enable) (int enable);
	int (*ldo_set_voltage) (int mV);

	u32 			swfi_latency;
	/* pmic notfications apis */
	int (*pmic_vbus_notif_init) (void (*callback)(int online), int init);
	int (*pmic_id_notif_init) (void (*callback)(int online), int init);
	int (*pmic_register_vbus_sn) (void (*callback)(int online));
	void (*pmic_unregister_vbus_sn) (void (*callback)(int online));
	int (*pmic_enable_ldo) (int);
	int (*init_gpio)(int on);
	void (*setup_gpio)(enum usb_switch_control mode);
	u8      otg_mode;
	u8	usb_mode;
	void (*vbus_power) (unsigned phy_info, int on);

	/* charger notification apis */
	void (*chg_connected)(enum chg_type chg_type);
	void (*chg_vbus_draw)(unsigned ma);
	int  (*chg_init)(int init);
	int (*config_vddcx)(int enable);

	struct pm_qos_request_list *pm_qos_req_dma;
	struct pm_qos_request_list *pm_qos_req_bus;
};

struct msm_usb_host_platform_data {
	unsigned phy_info;
	unsigned int power_budget;
	void (*config_gpio)(unsigned int config);
	void (*vbus_power) (unsigned phy_info, int on);
	int  (*vbus_init)(int init);
	struct pm_qos_request_list *pm_qos_req_bus;
};

int usb_get_connect_type(void);
#endif
