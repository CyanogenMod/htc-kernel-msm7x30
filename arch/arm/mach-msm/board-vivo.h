/* linux/arch/arm/mach-msm/board-vivo.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_VIVO_H
#define __ARCH_ARM_MACH_MSM_BOARD_VIVO_H

#include <mach/board.h>

#define VIVO_GPIO_UART2_RX 	51
#define VIVO_GPIO_UART2_TX 	52

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x04400000
#define MSM_LINUX_SIZE1		0x0BC00000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x0B700000
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_BASE  	0x2B700000
#define MSM_PMEM_ADSP_SIZE	0x02000000
#define PMEM_KERNEL_EBI1_BASE   0x2D700000
#define PMEM_KERNEL_EBI1_SIZE   0x00600000

#define MSM_PMEM_CAMERA_BASE	0x2DD00000
#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_MDP_BASE	0x2DD00000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_FB_BASE		0x2FD00000
#define MSM_FB_SIZE		0x00300000

#define VIVO_GPIO_WIFI_IRQ             147
#define VIVO_GPIO_WIFI_SHUTDOWN_N       39

#define VIVO_GPIO_KEYPAD_POWER_KEY		46
//#define VIVO_GPIO_KEYPAD_MENU_KEY		113
//#define VIVO_GPIO_KEYPAD_HOME_KEY		18
//#define VIVO_GPIO_KEYPAD_BACK_KEY		19

//#define VIVO_GPIO_FLASH_EN			97//PMIC GPIO15
#define VIVO_GPIO_TORCH_EN			98

//#define VIVO_GPIO_UP_RESET_N		43//PMIC GPIO36
//#define VIVO_GPIO_UP_INT_N			142//PMIC GPIO23

#define VIVO_GPIO_TP_EN			105

//#define VIVO_GPIO_COMPASS_INT		42//PMIC GPIO37
#define VIVO_GPIO_CHG_INT	180

#define VIVO_LAYOUTS			{ \
		{ { 0,  1, 0}, { -1, 0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { -1, 0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}
#define VIVO_MDDI_TE			(30)
#define VIVO_LCD_RSTz		(126)
#define VIVO_LCD_ID1			(128)
#define VIVO_LCD_ID0			(129)
//#define VIVO_LCD_PCLK               (90)
//#define VIVO_LCD_DE                 (91)
//#define VIVO_LCD_VSYNC              (92)
//#define VIVO_LCD_HSYNC              (93)
//#define VIVO_LCD_G2                 (94)
//#define VIVO_LCD_G3                 (95)
//#define VIVO_LCD_G4                 (96)
//#define VIVO_LCD_G5                 (97)
//#define VIVO_LCD_G6                 (98)
//#define VIVO_LCD_G7                 (99)
//#define VIVO_LCD_B3                 (100)
//#define VIVO_LCD_B4                 (101)
//#define VIVO_LCD_B5                 (102)
//#define VIVO_LCD_B6                 (103)
//#define VIVO_LCD_B7                 (104)
//#define VIVO_LCD_R3                 (105)
//#define VIVO_LCD_R4                 (106)
//#define VIVO_LCD_R5                 (107)
//#define VIVO_LCD_R6                 (108)
//#define VIVO_LCD_R7                 (109)

/* BT */
#define VIVO_GPIO_BT_UART1_RTS      (134)
#define VIVO_GPIO_BT_UART1_CTS      (135)
#define VIVO_GPIO_BT_UART1_RX       (136)
#define VIVO_GPIO_BT_UART1_TX       (137)
#define VIVO_GPIO_BT_PCM_OUT        (138)
#define VIVO_GPIO_BT_PCM_IN         (139)
#define VIVO_GPIO_BT_PCM_SYNC       (140)
#define VIVO_GPIO_BT_PCM_CLK        (141)
#define VIVO_GPIO_BT_RESET_N        (41)
#define VIVO_GPIO_BT_HOST_WAKE      (44)
#define VIVO_GPIO_BT_CHIP_WAKE      (50)
#define VIVO_GPIO_BT_SHUTDOWN_N     (38)

/* USB */
#define VIVO_GPIO_USB_ID_PIN			(49)
#define VIVO_GPIO_USB_ID1_PIN			(145)
#define VIVO_AUDIOz_UART_SW			(95)
#define VIVO_USBz_AUDIO_SW				(96)

//#define VIVO_SPI_CS2                (87)
//#define VIVO_SPI_DO                 (47)
//#define VIVO_SPI_DI                 (48)
//#define VIVO_SPI_CLK                (45)

#define VIVO_GPIO_PS_HOLD	(29)

//#define VIVO_SLIDING_INTZ	(18)//Vivo doesn't have sliding keyboard

#define VIVO_AUD_MICPATH_SEL		(127)

/* 35mm headset */
#define VIVO_GPIO_35MM_HEADSET_DET	(26)

/* EMMC */
#define VIVO_GPIO_EMMC_RST			  (88)

/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define VIVO_GPIO_TP_INT_N		PMGPIO(1)
#define VIVO_GPIO_GSENSOR_INT	PMGPIO(7)
#define VIVO_AUD_SPK_SD		PMGPIO(12)
#define VIVO_GPIO_FLASH_EN	PMGPIO(15)
#define VIVO_VOL_UP			PMGPIO(16)
#define VIVO_VOL_DN			PMGPIO(17)
#define VIVO_AUD_AMP_EN		PMGPIO(26)
//#define VIVO_AUD_HANDSET_ENO		PMGPIO(19)//Vivo doesn't have this pin, sync mecha audio.
#define VIVO_GPIO_PS_EN		PMGPIO(20)
#define VIVO_TP_RSTz			PMGPIO(21)
#define VIVO_GPIO_PS_INT_N		PMGPIO(22)
#define VIVO_GPIO_UP_INT_N		PMGPIO(23)
#define VIVO_GREEN_LED		PMGPIO(24)
#define VIVO_AMBER_LED		PMGPIO(25)
//#define VIVO_KEYPAD_LED		PMGPIO(26)//Vivo doesn't have sliding keyboard
#define VIVO_GPIO_SDMC_CD_N		PMGPIO(34)
#define VIVO_GPIO_LS_EN		PMGPIO(35)
#define VIVO_GPIO_uP_RST PMGPIO(36)
#define VIVO_GPIO_COMPASS_INT_N PMGPIO(37)
#define VIVO_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)

/* Camera */
#define CAM1_PWD			(35)
#define CAM1_VCM_PWD	(34)
#define CAM2_PWD			(146)
#define CAM2_RST			(31)
#define CLK_SWITCH		(144)

int vivo_init_mmc(unsigned int sys_rev);
void __init vivo_audio_init(void);
int vivo_init_keypad(void);
int __init vivo_wifi_init(void);

//vivo and vivow will share one panel code
int __init vivow_init_panel(unsigned int sys_rev);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_VIVO_H */
