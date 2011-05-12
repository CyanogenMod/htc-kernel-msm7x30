/* linux/arch/arm/mach-msm/board-spade.h
 *
 * Copyright (C) 2010-2011 HTC Corporation.
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_SPADE_H
#define __ARCH_ARM_MACH_MSM_BOARD_SPADE_H

#include <mach/board.h>

#define SPADE_PROJECT_NAME	"ace"

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

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
#define MSM_PMEM_ADSP_SIZE	0x01D00000 /* for 8M(4:3) + gpu effect */
#define PMEM_KERNEL_EBI1_BASE   0x2D600000
#define PMEM_KERNEL_EBI1_SIZE   0x00700000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x2DD00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_FB_BASE		0x2FD00000
#define MSM_FB_SIZE		0x00300000

/* GPIO definition */

/* Direct Keys */
#define SPADE_GPIO_KEYPAD_POWER_KEY  (46)

/* Battery */
#define SPADE_GPIO_MBAT_IN           (40)
#define SPADE_GPIO_MCHG_EN_N         (162)
#define SPADE_GPIO_ISET		     (127)

/* Wifi */
#define SPADE_GPIO_WIFI_IRQ          (147)
#define SPADE_GPIO_WIFI_EN           (39)

#define SPADE_GPIO_UART2_RX          (51)
#define SPADE_GPIO_UART2_TX          (52)

/* Sensors */
#define SPADE_GPIO_PROXIMITY_INT_N   (180)
#define SPADE_GPIO_COMPASS_INT       (42)

#define SPADE_COMPASS_LAYOUTS		{ \
	{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ { 0, -1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
	{ { 1,  0, 0}, { 0, -1,  0}, {0, 0, -1} }, \
	{ { 1,  0, 0}, { 0,  0,  1}, {0, 1,  0} }  \
					}
/* Microp */
#define SPADE_GPIO_UP_RESET_N        (43)
#define SPADE_GPIO_UP_INT_N          (142)

/* TP */
#define SPADE_GPIO_TP_ATT_N          (40)
#define SPADE_GPIO_TP_3V3_ENABLE	(55)

/* LCD */
#define SPADE_LCD_PCLK               (90)
#define SPADE_LCD_DE                 (91)
#define SPADE_LCD_VSYNC              (92)
#define SPADE_LCD_HSYNC              (93)

#define SPADE_LCD_G0                 (18)
#define SPADE_LCD_G1                 (19)
#define SPADE_LCD_G2                 (94)
#define SPADE_LCD_G3                 (95)
#define SPADE_LCD_G4                 (96)
#define SPADE_LCD_G5                 (97)
#define SPADE_LCD_G6                 (98)
#define SPADE_LCD_G7                 (99)

#define SPADE_LCD_B0                 (20)
#define SPADE_LCD_B1                 (21)
#define SPADE_LCD_B2                 (22)
#define SPADE_LCD_B3                 (100)
#define SPADE_LCD_B4                 (101)
#define SPADE_LCD_B5                 (102)
#define SPADE_LCD_B6                 (103)
#define SPADE_LCD_B7                 (104)

#define SPADE_LCD_R0                 (23)
#define SPADE_LCD_R1                 (24)
#define SPADE_LCD_R2                 (25)
#define SPADE_LCD_R3                 (105)
#define SPADE_LCD_R4                 (106)
#define SPADE_LCD_R5                 (107)
#define SPADE_LCD_R6                 (108)
#define SPADE_LCD_R7                 (109)
#define SPADE_LCD_RSTz               (126)

/* Audio */
#define SPADE_AUD_CODEC_RST          (34)
#define SPADE_AUD_MIC_BIAS           (56)
#define SPADE_AUD_A1026_INT          (120)
#define SPADE_AUD_MICPATH_SEL        (121)
#define SPADE_AUD_A1026_RESET        (122)
#define SPADE_AUD_A1026_WAKEUP       (123)

/* BT */
#define SPADE_GPIO_BT_SHUTDOWN_N     (38)
#define SPADE_GPIO_BT_RESET_N        (41)
#define SPADE_GPIO_BT_HOST_WAKE      (44)
#define SPADE_GPIO_BT_CHIP_WAKE      (50)
#define SPADE_GPIO_BT_UART1_RTS      (134)
#define SPADE_GPIO_BT_UART1_CTS      (135)
#define SPADE_GPIO_BT_UART1_RX       (136)
#define SPADE_GPIO_BT_UART1_TX       (137)
#define SPADE_GPIO_BT_PCM_OUT        (138)
#define SPADE_GPIO_BT_PCM_IN         (139)
#define SPADE_GPIO_BT_PCM_SYNC       (140)
#define SPADE_GPIO_BT_PCM_CLK        (141)

/* USB */
#define SPADE_GPIO_USB_ID_PIN        (145)
#define SPADE_DISABLE_USB_CHARGER               (125)

/* Camera */
#define SPADE_CAM_RST                (31)
#define SPADE_CAM_PWD                (124)
#define SPADE_CAM_A2V85_EN_XA        (37)

/* Flashlight */
#define SPADE_GPIO_FLASHLIGHT_FLASH  (128)
#define SPADE_GPIO_FLASHLIGHT_TORCH  (129)

#define SPADE_SPI_CLK                (45)
#define SPADE_SPI_DO                 (47)
#define SPADE_SPI_DI                 (48)
#define SPADE_SPI_CS2                (87)

/* EMMC */
#define SPADE_GPIO_EMMC_RST			 (88)

/* PMIC */
#define PMIC_GPIO_INT_XC             (27)
#define PMIC_GPIO_INT                (179)
#define SPADE_GPIO_PS_HOLD           (29)

/* PMIC GPIO definition */
#define PMGPIO(x) (x-1)
#define SPADE_AUD_EP_EN        PMGPIO(16)
#define SPADE_AUD_SPK_ENO      PMGPIO(18)
#define SPADE_AUD_HP_EN        PMGPIO(19)
#define SPADE_TP_RSTz          PMGPIO(21)
#define SPADE_LED_3V3_EN       PMGPIO(22)
#define SPADE_SDMC_CD_N        PMGPIO(23)
#define SPADE_VOL_UP           PMGPIO(24)
#define SPADE_VOL_DN           PMGPIO(25)
#define SPADE_AUD_HP_DETz      PMGPIO(26)
#define SPADE_GSENSOR_INTz     PMGPIO(35)
#define SPADE_SLEEP_CLK2       PMGPIO(39)

int spade_panel_sleep_in(void);
#ifdef CONFIG_MICROP_COMMON
void __init spade_microp_init(void);
#endif
int spade_init_mmc(unsigned int sys_rev);
void __init spade_audio_init(void);
int __init spade_init_keypad(void);
int __init spade_wifi_init(void);
int __init spade_init_panel(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_SPADE_H */
