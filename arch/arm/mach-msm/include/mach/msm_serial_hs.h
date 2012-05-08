/*
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_MSM_SERIAL_HS_H
#define __ASM_ARCH_MSM_SERIAL_HS_H

#include<linux/serial_core.h>

/* Optional platform device data for msm_serial_hs driver.
 * Used to configure low power wakeup */
struct msm_serial_hs_platform_data {
	int wakeup_irq;  /* wakeup irq */
	/* bool: inject char into rx tty on wakeup */
	unsigned char inject_rx_on_wakeup;
	char rx_to_inject;
	int (*gpio_config)(int);

	unsigned char cpu_lock_supported;

	/* for bcm BT */
	int rx_wakeup_irq;  /* wakeup irq */
	unsigned char bt_wakeup_pin_supported;
	unsigned char bt_wakeup_pin;	/* Device to Chip */
	unsigned char host_wakeup_pin;	/* Chip to Device */
};
#if 1		//Add by evan.xu@2012-02-02
/* API for TI_ST */
extern void ti_msm_hs_request_clock_off(struct uart_port *uport);
extern void ti_msm_hs_request_clock_on(struct uart_port *uport);
extern void ti_dc_msm_hs_request_clock_off(struct uart_port *uport);
extern void ti_dc_msm_hs_request_clock_on(struct uart_port *uport);
#endif

extern void imc_msm_hs_request_clock_on(struct uart_port *uport);
unsigned int msm_hs_tx_empty(struct uart_port *uport);
void msm_hs_request_clock_off(struct uart_port *uport);
void msm_hs_request_clock_on(struct uart_port *uport);
void msm_hs_set_mctrl(struct uart_port *uport,
				    unsigned int mctrl);
#endif
