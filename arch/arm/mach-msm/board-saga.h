/* linux/arch/arm/mach-msm/board-saga.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SAGA_H
#define __ARCH_ARM_MACH_MSM_BOARD_SAGA_H

#include <mach/board.h>

#define MSM_LINUX_BASE1			0x04400000
#define MSM_LINUX_SIZE1			0x0BC00000
#define MSM_LINUX_BASE2			0x20000000
#define MSM_LINUX_SIZE2			0x0B300000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE		0x00100000
#define MSM_GPU_MEM_SIZE		0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_BASE      0x2B300000
#define MSM_PMEM_ADSP_SIZE      0x02300000
#define PMEM_KERNEL_EBI1_BASE   0x2D600000
#define PMEM_KERNEL_EBI1_SIZE   0x00700000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE		0x2DD00000
#define MSM_PMEM_MDP_SIZE		0x02000000

#define MSM_FB_BASE				0x2FD00000
#define MSM_FB_SIZE				0x00300000

#define SAGA_GPIO_WIFI_IRQ             147
#define SAGA_GPIO_WIFI_SHUTDOWN_N       39

#define SAGA_GPIO_UART2_RX 				51
#define SAGA_GPIO_UART2_TX 				52

#define SAGA_GPIO_KEYPAD_POWER_KEY		46


/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

/* Proximity */
#define SAGA_GPIO_PROXIMITY_INT_N		18

#define SAGA_GPIO_COMPASS_INT			42
#define SAGA_GPIO_GSENSOR_INT_N			180
#define SAGA_LAYOUTS_XA_XB		{ \
		{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 1,  0, 0}, { 0, -1,  0}, {0, 0, -1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}
#define SAGA_LAYOUTS_XC			{ \
		{ { 0,  1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}

#define SAGA_PMIC_GPIO_INT			(27)

#define SAGA_GPIO_UP_INT_N          (142)

#define SAGA_GPIO_PS_HOLD			(29)

#define SAGA_MDDI_RSTz				(162)
#define SAGA_LCD_ID0				(124)
#define SAGA_LCD_RSTz_ID1			(126)
#define SAGA_LCD_PCLK_1             (90)
#define SAGA_LCD_DE                 (91)
#define SAGA_LCD_VSYNC              (92)
#define SAGA_LCD_HSYNC              (93)
#define SAGA_LCD_G0                 (94)
#define SAGA_LCD_G1                 (95)
#define SAGA_LCD_G2                 (96)
#define SAGA_LCD_G3                 (97)
#define SAGA_LCD_G4                 (98)
#define SAGA_LCD_G5                 (99)
#define SAGA_LCD_B0					(22)
#define SAGA_LCD_B1                 (100)
#define SAGA_LCD_B2                 (101)
#define SAGA_LCD_B3                 (102)
#define SAGA_LCD_B4                 (103)
#define SAGA_LCD_B5                 (104)
#define SAGA_LCD_R0					(25)
#define SAGA_LCD_R1                 (105)
#define SAGA_LCD_R2                 (106)
#define SAGA_LCD_R3                 (107)
#define SAGA_LCD_R4                 (108)
#define SAGA_LCD_R5                 (109)

#define SAGA_LCD_SPI_CSz			(87)

/* Audio */
#define SAGA_AUD_SPI_CSz            (89)
#define SAGA_AUD_MICPATH_SEL_XA     (121)

/* Flashlight */
#define SAGA_GPIO_FLASHLIGHT_FLASH	128
#define SAGA_GPIO_FLASHLIGHT_TORCH	129

/* BT */
#define SAGA_GPIO_BT_UART1_RTS      (134)
#define SAGA_GPIO_BT_UART1_CTS      (135)
#define SAGA_GPIO_BT_UART1_RX       (136)
#define SAGA_GPIO_BT_UART1_TX       (137)
#define SAGA_GPIO_BT_PCM_OUT        (138)
#define SAGA_GPIO_BT_PCM_IN         (139)
#define SAGA_GPIO_BT_PCM_SYNC       (140)
#define SAGA_GPIO_BT_PCM_CLK        (141)
#define SAGA_GPIO_BT_RESET_N        (41)
#define SAGA_GPIO_BT_HOST_WAKE      (26)
#define SAGA_GPIO_BT_CHIP_WAKE      (50)
#define SAGA_GPIO_BT_SHUTDOWN_N     (38)

#define SAGA_GPIO_USB_ID_PIN		(145)

#define SAGA_VCM_PD					(34)
#define SAGA_CAM1_PD                (31)
#define SAGA_CLK_SEL                (23) /* camera select pin */
#define SAGA_CAM2_PD			    (24)
#define SAGA_CAM2_RSTz				(21)

#define SAGA_CODEC_3V_EN			(36)
#define SAGA_GPIO_TP_ATT_N			(20)

/* PMIC 8058 GPIO */
#define PMGPIO(x) (x-1)
#define SAGA_AUD_MICPATH_SEL_XB		PMGPIO(10)
#define SAGA_AUD_EP_EN				PMGPIO(13)
#define SAGA_VOL_UP				PMGPIO(14)
#define SAGA_VOL_DN				PMGPIO(15)
#define SAGA_GPIO_CHG_INT		PMGPIO(16)
#define SAGA_AUD_HP_DETz		PMGPIO(17)
#define SAGA_AUD_SPK_EN			PMGPIO(18)
#define SAGA_AUD_HP_EN			PMGPIO(19)
#define SAGA_PS_SHDN			PMGPIO(20)
#define SAGA_TP_RSTz			PMGPIO(21)
#define SAGA_LED_3V3_EN			PMGPIO(22)
#define SAGA_SDMC_CD_N			PMGPIO(23)
#define SAGA_GREEN_LED			PMGPIO(24)
#define SAGA_AMBER_LED			PMGPIO(25)
#define SAGA_AUD_A3254_RSTz		PMGPIO(36)
#define SAGA_WIFI_SLOW_CLK		PMGPIO(38)
#define SAGA_SLEEP_CLK2			PMGPIO(39)

unsigned int saga_get_engineerid(void);
int saga_init_mmc(unsigned int sys_rev);
void __init saga_audio_init(void);
int saga_init_keypad(void);
int __init saga_wifi_init(void);
int __init saga_init_panel(void);
int __init saga_rfkill_init(void);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_SAGA_H */
