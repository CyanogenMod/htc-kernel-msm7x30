/* linux/arch/arm/mach-msm/board-mecha.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_MECHA_H
#define __ARCH_ARM_MACH_MSM_BOARD_MECHA_H

#include <mach/board.h>

#define MECHA_GPIO_UART3_RX	53
#define MECHA_GPIO_UART3_TX	54
#define MECHA_GPIO_UART3_RTS	55
#define MECHA_GPIO_UART3_CTS	57

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x05200000
#define MSM_LINUX_SIZE1		0x0AE00000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0B100000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_BASE  	0x2B100000
#define MSM_PMEM_ADSP_SIZE	0x02400000
#define PMEM_KERNEL_EBI1_BASE   0x2D700000
#define PMEM_KERNEL_EBI1_SIZE   0x00600000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x2DD00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_FB_BASE		0x2FD00000
#define MSM_FB_SIZE		0x00300000

#define MECHA_GPIO_MDM2AP_STATUS	(77)

#define MECHA_GPIO_WIFI_IRQ		(180)
#define MECHA_GPIO_WIFI_SHUTDOWN_N	(122)

#define MECHA_GPIO_KEYPAD_POWER_KEY	(46)

#define MECHA_GPIO_AUD_AMP_EN		(128)
#define MECHA_GPIO_FLASH_EN_XA		(128)
#define MECHA_GPIO_TORCH_EN		(129)

#define MECHA_GPIO_TP_INT_N		(147)

#define MECHA_LAYOUTS			{ \
		{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 1,  0, 0}, { 0,  -1, 0}, {0, 0, -1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}
#define MECHA_AUD_CODEC_RST		(125)
/* LCD */
#define	MECHA_LCD_PCLK			(90)
#define	MECHA_LCD_DE			(91)
#define	MECHA_LCD_VSYNC			(92)
#define	MECHA_LCD_HSYNC			(93)
#define	MECHA_LCD_G0			(18)
#define	MECHA_LCD_G1			(19)
#define	MECHA_LCD_G2			(94)
#define	MECHA_LCD_G3			(95)
#define	MECHA_LCD_G4			(96)
#define	MECHA_LCD_G5			(97)
#define	MECHA_LCD_G6			(98)
#define	MECHA_LCD_G7			(99)
#define	MECHA_LCD_B0			(20)
#define	MECHA_LCD_B1			(21)
#define	MECHA_LCD_B2			(22)
#define	MECHA_LCD_B3			(100)
#define	MECHA_LCD_B4			(101)
#define	MECHA_LCD_B5			(102)
#define	MECHA_LCD_B6			(103)
#define	MECHA_LCD_B7			(104)
#define	MECHA_LCD_R0			(23)
#define	MECHA_LCD_R1			(24)
#define	MECHA_LCD_R2			(25)
#define	MECHA_LCD_R3			(105)
#define	MECHA_LCD_R4			(106)
#define	MECHA_LCD_R5			(107)
#define	MECHA_LCD_R6			(108)
#define	MECHA_LCD_R7			(109)
#define MECHA_LCD_RSTz			(126)
/* BT */
#define MECHA_GPIO_BT_UART1_RTS	        (134)
#define MECHA_GPIO_BT_UART1_CTS	        (135)
#define MECHA_GPIO_BT_UART1_RX		(136)
#define MECHA_GPIO_BT_UART1_TX		(137)
#define MECHA_GPIO_BT_PCM_OUT           (138)
#define MECHA_GPIO_BT_PCM_IN            (139)
#define MECHA_GPIO_BT_PCM_SYNC          (140)
#define MECHA_GPIO_BT_PCM_CLK           (141)
#define MECHA_GPIO_BT_RESET_N		(120)
#define MECHA_GPIO_BT_HOST_WAKE	        (145)
#define MECHA_GPIO_BT_CHIP_WAKE	        (123)
#define MECHA_GPIO_BT_SHUTDOWN_N	(121)

/* USB */
#define MECHA_GPIO_AUD_UART_SWITCH	(2)
#define MECHA_GPIO_USB_AUD_UART_SWITCH	(37)
#define MECHA_GPIO_USB_ID_PIN		(49)	// Fix Me
#define MECHA_GPIO_USB_ID1_PIN		(145)	// Fix Me, PMGPIO(13)

#define MECHA_SPI_DO			(47)
#define MECHA_SPI_DI			(48)
#define MECHA_SPI_CLK			(45)

#define MECHA_GPIO_PS_HOLD		(29)

/* Camera */
#define CAM1_PWD			(30)
#define CAM1_VCM_PWD	(35)
#define CAM2_PWD			(146)
#define CAM2_RST			(31)
#define CLK_SWITCH		(144)

/* Audio */
#define MECHA_AUD_MICPATH_SEL		(127)

/* EMMC */
#define MECHA_GPIO_EMMC_RST		(88)

/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define MECHA_GPIO_CHG_INT		PMGPIO(1)
#define MECHA_USB_HUB_PWR		PMGPIO(2)
#define MECHA_GPIO_FLASH_EN		PMGPIO(5)
#define MECHA_GPIO_AUD_AMP_EN_XA	PMGPIO(5)
#define MECHA_AUD_SPK_SD		PMGPIO(12)
#define MECHA_GPIO_USB_ID		PMGPIO(13)
#define MECHA_ALS_SHUTDOWN		PMGPIO(19)
#define MECHA_GPIO_PS_EN		PMGPIO(20)
#define MECHA_GPIO_GSENSOR_INT_N	PMGPIO(7)
#define MECHA_TP_RSTz			PMGPIO(21)
#define MECHA_GPIO_PS_INT_N		PMGPIO(15)
#define MECHA_GPIO_UP_INT_N		PMGPIO(23)
#define MECHA_GREEN_LED			PMGPIO(24)
#define MECHA_AMBER_LED			PMGPIO(25)
#define MECHA_AUD_HP_DETz		PMGPIO(26)
#define MECHA_VOL_UP			PMGPIO(16)
#define MECHA_VOL_DN			PMGPIO(17)
#define MECHA_GPIO_USB_HUB_SWITCH	PMGPIO(6) /*for XC*/
#define MECHA_GPIO_SDMC_CD_N		PMGPIO(6) /*for XA, XB*/
#define MECHA_USB_HUB_RESET		PMGPIO(35)
#define MECHA_GPIO_COMPASS_INT_N	PMGPIO(37)
#define MECHA_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)
#define MECHA_GPIO_UP_RESET_N		PMGPIO(36)

int __init mecha_init_mmc(unsigned int sys_rev);
void __init mecha_audio_init(void);
int __init mecha_init_keypad(void);
int __init mecha_wifi_init(void);
int __init mecha_init_panel(void);
unsigned int mecha_get_engineerid(void);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_MECHA_H */
