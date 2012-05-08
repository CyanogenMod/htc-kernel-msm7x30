/* linux/arch/arm/mach-msm/board-runnymede.h
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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_runnymede_H
#define __ARCH_ARM_MACH_MSM_BOARD_runnymede_H

#include <mach/board.h>

#define runnymede_GPIO_UART2_RX 	51
#define runnymede_GPIO_UART2_TX 	52

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)

#define MSM_LINUX_BASE1		0x14400000
#define MSM_LINUX_SIZE1		0x0BC00000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x10000000
#define MSM_LINUX_BASE3		0x40000000
#define MSM_LINUX_SIZE3		0x10000000
#define MSM_LINUX_BASE4		0x50000000
#define MSM_LINUX_SIZE4		0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_AUDIO_SIZE	0x00200000

#define MSM_PMEM_ADSP_SIZE	0x03A00000		/*98MB -> 63MB -> 55MB -> 58MB*/

#define MSM_PMEM_ADSP2_SIZE     0x002C0000

#define PMEM_KERNEL_EBI1_SIZE   0x00900000

#define MSM_PMEM_CAMERA_SIZE	0x00000000

#define MSM_PMEM_SF_SIZE	0x01000000

#define MSM_FB_SIZE		0x00500000

#define runnymede_GPIO_WIFI_IRQ             147
#define runnymede_GPIO_WIFI_SHUTDOWN_N       39

#define runnymede_GPIO_KEYPAD_POWER_KEY		46

#define runnymede_GPIO_FLASH_EN			128
#define runnymede_GPIO_TORCH_EN			129

#define RUNNYMEDE_GPIO_GYRO_INT         (62)
#define RUNNYMEDE_GPIO_PS_INT_N         (18)

#define RUNNYMEDE_MDDI_TE		(30)
#define RUNNYMEDE_LCD_RSTz		(127)
#define RUNNYMEDE_LCD_ID1		(22)
#define RUNNYMEDE_LCD_ID0		(126)

/* BT */
#define RUNNYMEDE_GPIO_BT_CHIP_WAKE      (31)
#define RUNNYMEDE_GPIO_BT_SHUTDOWN_N     (38)
#define RUNNYMEDE_GPIO_BT_RESET_N        (41)
#define RUNNYMEDE_GPIO_BT_HOST_WAKE      (44)
#define RUNNYMEDE_GPIO_BT_UART1_RTS      (134)
#define RUNNYMEDE_GPIO_BT_UART1_CTS      (135)
#define RUNNYMEDE_GPIO_BT_UART1_RX       (136)
#define RUNNYMEDE_GPIO_BT_UART1_TX       (137)


#define runnymede_GPIO_BT_PCM_OUT        (138)
#define runnymede_GPIO_BT_PCM_IN         (139)
#define runnymede_GPIO_BT_PCM_SYNC       (140)
#define runnymede_GPIO_BT_PCM_CLK        (141)

/* USB */
#define runnymede_GPIO_USB_ID1_PIN			(145)

#define runnymede_GPIO_PS_HOLD	(29)

#define runnymede_AUD_MICPATH_SEL		(121)

/* Accessory */
#define runnymede_GPIO_35MM_HEADSET_DET		(26)

/*P sensor*/
#define runnymede_GPIO_PS_INT_N			(18)
/* EMMC */
#define runnymede_GPIO_EMMC_RST			  (88)

/* Touch */
#define runnymede_GPIO_TP_ATT_N			(20)
#define runnymede_GPIO_TP_3V3_EN		(99)

/* BT Dock */
#define runnymede_GPIO_BT_DOCK		(44)

/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define runnymede_AUD_STEREO_REC		PMGPIO(5)
#define runnymede_COMPASS_INT			PMGPIO(12)
#define runnymede_GPIO_GSENSOR_INT		PMGPIO(13)
#define runnymede_CHG_INT				PMGPIO(16)
#define runnymede_AUD_SPK_SD			PMGPIO(18)
#define runnymede_VOL_UP				PMGPIO(23)
#define runnymede_VOL_DN				PMGPIO(37)
#define runnymede_AUD_AMP_EN			PMGPIO(19)
#define runnymede_GPIO_PS_EN			PMGPIO(20)
#define runnymede_TP_RSTz				PMGPIO(21)
#define runnymede_AUD_REMO_PRES_N		PMGPIO(22)
#define runnymede_GREEN_LED				PMGPIO(24)
#define runnymede_AMBER_LED				PMGPIO(25)

#define runnymede_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)

/* Camera */
#define CAM_MCLK_1			(15)
#define CAM_I2C_SCL			(16)
#define CAM_I2C_SDA			(17)
#define CAM_STEP1			(36)
#define CAM_STEP2			(35)
#define CAM1_PWD			(0)
#define CAM1_VCM_PWD		(1)
#define CAM2_PWD			(105)
#define CAM2_RST			(106)
#define CLK_SWITCH			(107)
#define RUNNYMEDE_CAM_ID			(19)

/*display*/
extern struct platform_device msm_device_mdp;
extern struct platform_device msm_device_mddi0;
extern int panel_type;
extern unsigned long msm_fb_base;

int runnymede_init_mmc(unsigned int sys_rev);
void __init runnymede_audio_init(void);
int __init runnymede_init_keypad(void);
int __init runnymede_wifi_init(void);

/*runnymede and runnymedew will share one panel code*/
int __init runnymede_init_panel(void);
#endif /* __ARCH_ARM_MACH_MSM_BOARD_runnymede_H */
