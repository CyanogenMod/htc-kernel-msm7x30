/* linux/arch/arm/mach-msm/board-glacier.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_GLACIER_H
#define __ARCH_ARM_MACH_MSM_BOARD_GLACIER_H

#include <mach/board.h>

#define MSM_LINUX_BASE1		0x04000000
#define MSM_LINUX_SIZE1		0x0C000000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0B900000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_BASE  	0x2B900000
#define MSM_PMEM_ADSP_SIZE	0x01D00000
#define PMEM_KERNEL_EBI1_BASE   0x2D600000
#define PMEM_KERNEL_EBI1_SIZE   0x00700000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x2DD00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_FB_BASE		0x2FD00000
#define MSM_FB_SIZE		0x00300000

#define GLACIER_GPIO_WIFI_IRQ             147
#define GLACIER_GPIO_WIFI_SHUTDOWN_N       39

#define GLACIER_GPIO_UART2_RX 		51
#define GLACIER_GPIO_UART2_TX 		52

#define GLACIER_GPIO_KEYPAD_POWER_KEY		46
/*
#define GLACIER_GPIO_KEYPAD_MENU_KEY		113
#define GLACIER_GPIO_KEYPAD_HOME_KEY		18
#define GLACIER_GPIO_KEYPAD_BACK_KEY		19
*/

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

/* Proximity */
#define GLACIER_GPIO_PROXIMITY_EN		19
#define GLACIER_GPIO_PROXIMITY_INT_N		18

#define GLACIER_GPIO_COMPASS_INT		42
#define GLACIER_LAYOUTS			{ \
		{ { 0,  1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ { 1,  0, 0}, { 0,  0,  1}, {0, 1,  0} }  \
					}

#define GLACIER_PMIC_GPIO_INT		(27)
#define GLACIER_GPIO_UP_RESET_N		(43)
#define GLACIER_GPIO_UP_INT_N            (142)

#define GLACIER_CAM2_RSTz		(21)

#define GLACIER_GPIO_PS_HOLD	(29)

#define GLACIER_LCD_2V85_EN            (22)
#define GLACIER_MDDI_RSTz		(162)
#define GLACIER_LCD_ID0               (124)
#define GLACIER_LCD_ID1               (126)
#define GLACIER_LCD_ID2               (127)
#define GLACIER_LCD_PCLK               (90)
#define GLACIER_LCD_DE                 (91)
#define GLACIER_LCD_VSYNC              (92)
#define GLACIER_LCD_HSYNC              (93)
/*
#define GLACIER_LCD_G2                 (94)
#define GLACIER_LCD_G3                 (95)
#define GLACIER_LCD_G4                 (96)
#define GLACIER_LCD_G5                 (97)
#define GLACIER_LCD_G6                 (98)
#define GLACIER_LCD_G7                 (99)
#define GLACIER_LCD_B3                 (100)
#define GLACIER_LCD_B4                 (101)
#define GLACIER_LCD_B5                 (102)
#define GLACIER_LCD_B6                 (103)
#define GLACIER_LCD_B7                 (104)
#define GLACIER_LCD_R3                 (105)
#define GLACIER_LCD_R4                 (106)
#define GLACIER_LCD_R5                 (107)
#define GLACIER_LCD_R6                 (108)
#define GLACIER_LCD_R7                 (109)
*/
/* Audio */
#define GLACIER_AUD_MICPATH_SEL         121
#define GLACIER_AUD_A1026_INT           120
#define GLACIER_AUD_A1026_WAKEUP        123
#define GLACIER_AUD_A1026_RESET         122
#define GLACIER_AUD_A1026_CLK   -1

/* Flashlight */
#define GLACIER_GPIO_FLASHLIGHT_FLASH	128
#define GLACIER_GPIO_FLASHLIGHT_TORCH	129

/* BT */
#define GLACIER_GPIO_BT_UART1_RTS      (134)
#define GLACIER_GPIO_BT_UART1_CTS      (135)
#define GLACIER_GPIO_BT_UART1_RX       (136)
#define GLACIER_GPIO_BT_UART1_TX       (137)
#define GLACIER_GPIO_BT_PCM_OUT        (138)
#define GLACIER_GPIO_BT_PCM_IN         (139)
#define GLACIER_GPIO_BT_PCM_SYNC       (140)
#define GLACIER_GPIO_BT_PCM_CLK        (141)
#define GLACIER_GPIO_BT_RESET_N        (41)
#define GLACIER_GPIO_BT_HOST_WAKE      (44)
#define GLACIER_GPIO_BT_CHIP_WAKE      (50)
#define GLACIER_GPIO_BT_SHUTDOWN_N     (38)

#define GLACIER_GPIO_USB_ID_PIN			(145)
#define GLACIER_GPIO_DOCK_PIN			(37)

#define GLACIER_CAM_PWD                (34)
#define GLACIER_CAM_RST                (31)
#define GLACIER_CLK_SWITCH             (23) /* camera select pin */
#define GLACIER_CAM2_PWD			   (24)
#define GLACIER_CAM2_RST			   (21)

#define GLACIER_OJ_RSTz                (36)
#define GLACIER_OJ_MOTION              (26)

#define GLACIER_GPIO_TP_ATT_N	(20)

/* EMMC */
#define GLACIER_GPIO_EMMC_RST		   (88)

/* PMIC 8058 GPIO */
#define PMGPIO(x) (x-1)
#define GLACIER_OJ_ACTION		PMGPIO(10)
#define GLACIER_HOME_KEY		PMGPIO(11)
#define GLACIER_MENU_KEY		PMGPIO(12)
#define GLACIER_BACK_KEY		PMGPIO(13)
#define GLACIER_SEND_KEY		PMGPIO(14)
#define GLACIER_GPIO_CHG_INT		PMGPIO(16)
#define GLACIER_AUD_SPK_ENO		PMGPIO(18)
#define GLACIER_AUD_HP_EN		PMGPIO(19)
#define GLACIER_PS_SHDN			PMGPIO(20)
#define GLACIER_TP_RSTz			PMGPIO(21)
#define GLACIER_LED_3V3_EN		PMGPIO(22)
#define GLACIER_SDMC_CD_N		PMGPIO(23)
#define GLACIER_VOL_UP			PMGPIO(24)
#define GLACIER_VOL_DN			PMGPIO(25)
#define GLACIER_AUD_HP_DETz		PMGPIO(26)
#define GLACIER_CAM_A2V85_EN    PMGPIO(33)/* ANALOG POWER of Glacier*/
#define GLACIER_CAM_STEP2		PMGPIO(35)
#define GLACIER_CAM_STEP1		PMGPIO(36)
#define GLACIER_WFM_ANT_SW		PMGPIO(37)
#define GLACIER_SLEEP_CLK2		PMGPIO(39)

unsigned int glacier_get_engineerid(void);
int glacier_init_mmc(unsigned int sys_rev);
void __init glacier_audio_init(void);
int glacier_init_keypad(void);
int __init glacier_wifi_init(void);
int __init glacier_init_panel(void);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_GLACIER_H */
