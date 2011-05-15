/* linux/arch/arm/mach-msm/board-speedy.h
 * Copyright (C) 2007-2009 HTC Corporation.
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SPEEDY_H
#define __ARCH_ARM_MACH_MSM_BOARD_SPEEDY_H

#include <mach/board.h>

#define SPEEDY_GPIO_UART2_RX	51
#define SPEEDY_GPIO_UART2_TX	52

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x04000000
#define MSM_LINUX_SIZE1		0x0C000000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0BF00000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE    0x000F0000

/* CIQ */
#ifdef CONFIG_BUILD_CIQ
#define MSM_PMEM_CIQ_BASE   MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE
#define MSM_PMEM_CIQ_SIZE   SZ_64K
#define MSM_PMEM_CIQ1_BASE  MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ1_SIZE  MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ2_BASE  MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ2_SIZE  MSM_PMEM_CIQ_SIZE
#define MSM_PMEM_CIQ3_BASE  MSM_PMEM_CIQ_BASE
#define MSM_PMEM_CIQ3_SIZE  MSM_PMEM_CIQ_SIZE
#endif

#define MSM_PMEM_ADSP_BASE  	0x2BF00000
#define MSM_PMEM_ADSP_SIZE	0x01800000
#define PMEM_KERNEL_EBI1_BASE   0x2D700000
#define PMEM_KERNEL_EBI1_SIZE   0x00600000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x2DD00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_FB_BASE		0x2FD00000
#define MSM_FB_SIZE		0x00300000

#define SPEEDY_GPIO_WIFI_IRQ		(147)
#define SPEEDY_GPIO_WIFI_SHUTDOWN_N	(36)

#define SPEEDY_GPIO_KEYPAD_POWER_KEY	(46)

#define SPEEDY_GPIO_FLASH_EN		(97)
#define SPEEDY_GPIO_UP_RESET_N		(54)

#define SPEEDY_GPIO_TP_INT_N		(20)
#define SPEEDY_GPIO_TP_EN		(105)

#define SPEEDY_LAYOUTS			{ \
		{ {0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ {0, -1, 0}, { -1, 0,  0}, {0, 0, 1} }, \
		{ {1,  0, 0}, { 0,  -1, 0}, {0, 0, -1} }, \
		{ {1,  0, 0}, { 0,  0,  1}, {0, 1, 0} }  \
					}

#define SPEEDY_LCD_RSTz			(126)
/* BT */
#define SPEEDY_GPIO_BT_UART1_RTS	(134)
#define SPEEDY_GPIO_BT_UART1_CTS	(135)
#define SPEEDY_GPIO_BT_UART1_RX		(136)
#define SPEEDY_GPIO_BT_UART1_TX		(137)
#define SPEEDY_GPIO_BT_PCM_OUT          (138)
#define SPEEDY_GPIO_BT_PCM_IN           (139)
#define SPEEDY_GPIO_BT_PCM_SYNC         (140)
#define SPEEDY_GPIO_BT_PCM_CLK          (141)
#define SPEEDY_GPIO_BT_RESET_N		(37)
#define SPEEDY_GPIO_BT_HOST_WAKE	(44)
#define SPEEDY_GPIO_BT_CHIP_WAKE	(50)
#define SPEEDY_GPIO_BT_SHUTDOWN_N	(35)

/* USB */
#define SPEEDY_GPIO_USB_ID_PIN		(49)
#define SPEEDY_GPIO_USB_ID1_PIN		(145)

#define SPEEDY_SPI_DO			(47)
#define SPEEDY_SPI_DI			(48)
#define SPEEDY_SPI_CLK			(45)

#define SPEEDY_GPIO_PS_HOLD		(29)

#define SPEEDY_SLIDING_INTZ		(18)

/* 35mm headset */
#define SPEEDY_GPIO_35MM_HEADSET_DET	(26)

/* Camera */
#define SPEEDY_CAM_RST			(31)
#define SPEEDY_CAM_PWD			(34)

#define SPEEDY_GPIO_EMMC_RST		  (88)

/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define SPEEDY_KYPD_SNS1		PMGPIO(1)
#define SPEEDY_KYPD_SNS2		PMGPIO(2)
#define SPEEDY_KYPD_SNS3		PMGPIO(3)
#define SPEEDY_KYPD_SNS4		PMGPIO(4)
#define SPEEDY_KYPD_SNS5		PMGPIO(5)
#define SPEEDY_KYPD_SNS6		PMGPIO(6)
#define SPEEDY_KYPD_SNS7		PMGPIO(7)
#define SPEEDY_WiMAX_HOST_WAKEUP    PMGPIO(17)
#define SPEEDY_AUD_SPK_ENO		PMGPIO(18)
#define SPEEDY_AUD_HANDSET_ENO		PMGPIO(19)
#define SPEEDY_GPIO_PS_EN		PMGPIO(20)
#define SPEEDY_GPIO_GSENSOR_INT_N_XB	PMGPIO(20)
#define SPEEDY_TP_RSTz			PMGPIO(21)
#define SPEEDY_GPIO_PS_INT_N		PMGPIO(22)
#define SPEEDY_GPIO_UP_INT_N		PMGPIO(23)
#define SPEEDY_GREEN_LED		PMGPIO(24)
#define SPEEDY_AMBER_LED		PMGPIO(25)
#define SPEEDY_KEYPAD_LED		PMGPIO(26)
#define SPEEDY_VOL_UP			PMGPIO(27)
#define SPEEDY_VOL_DN			PMGPIO(29)
#define SPEEDY_GPIO_SDMC_CD_N		PMGPIO(34)
#define SPEEDY_GPIO_LS_EN		PMGPIO(35)
#define SPEEDY_GPIO_CHG_INT		PMGPIO(35)
#define SPEEDY_GPIO_uP_RST_XB	PMGPIO(36)
#define SPEEDY_GPIO_GSENSOR_INT_N_XA	PMGPIO(37)
#define SPEEDY_GPIO_COMPASS_INT_N_XB	PMGPIO(37)
#define SPEEDY_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)


int __init speedy_init_mmc(unsigned int sys_rev);
void __init speedy_audio_init(void);
int __init speedy_init_keypad(void);
int __init speedy_wifi_init(void);
int __init speedy_init_panel(unsigned int sys_rev);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_SPEEDY_H */
