/*
 * linux/arch/arm/mach-omap2/board-ldp.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 * Nishant Kamat <nskamat@ti.com>
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>
#include <linux/io.h>
#include <linux/smsc911x.h>
#include <linux/mmc/host.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <mach/gpio.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <mach/board-zoom.h>

#include <asm/delay.h>
#include <plat/usb.h>
#include <plat/gpmc-smsc911x.h>

#include "board-flash.h"
#include "mux.h"
#include "hsmmc.h"
#include "control.h"
#include "common-board-devices.h"

#define LDP_SMSC911X_CS		1
#define LDP_SMSC911X_GPIO	152
#define DEBUG_BASE		0x08000000
#define LDP_ETHR_START		DEBUG_BASE

static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_1),
	KEY(1, 0, KEY_2),
	KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4),
	KEY(1, 1, KEY_5),
	KEY(2, 1, KEY_6),
	KEY(3, 1, KEY_F5),
	KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8),
	KEY(2, 2, KEY_9),
	KEY(3, 2, KEY_F6),
	KEY(0, 3, KEY_F7),
	KEY(1, 3, KEY_0),
	KEY(2, 3, KEY_F8),
	PERSISTENT_KEY(4, 5),
	KEY(4, 4, KEY_VOLUMEUP),
	KEY(5, 5, KEY_VOLUMEDOWN),
	0
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data ldp_kp_twl4030_data = {
	.keymap_data	= &board_map_data,
	.rows		= 6,
	.cols		= 6,
	.rep		= 1,
};

static struct gpio_keys_button ldp_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_ENTER,
		.gpio			= 101,
		.desc			= "enter sw",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	[1] = {
		.code			= KEY_F1,
		.gpio			= 102,
		.desc			= "func 1",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	[2] = {
		.code			= KEY_F2,
		.gpio			= 103,
		.desc			= "func 2",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	[3] = {
		.code			= KEY_F3,
		.gpio			= 104,
		.desc			= "func 3",
		.active_low		= 1,
		.debounce_interval 	= 30,
	},
	[4] = {
		.code			= KEY_F4,
		.gpio			= 105,
		.desc			= "func 4",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	[5] = {
		.code			= KEY_LEFT,
		.gpio			= 106,
		.desc			= "left sw",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	[6] = {
		.code			= KEY_RIGHT,
		.gpio			= 107,
		.desc			= "right sw",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	[7] = {
		.code			= KEY_UP,
		.gpio			= 108,
		.desc			= "up sw",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	[8] = {
		.code			= KEY_DOWN,
		.gpio			= 109,
		.desc			= "down sw",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
};

static struct gpio_keys_platform_data ldp_gpio_keys = {
	.buttons		= ldp_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(ldp_gpio_keys_buttons),
	.rep			= 1,
};

static struct platform_device ldp_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &ldp_gpio_keys,
	},
};

static struct omap_smsc911x_platform_data smsc911x_cfg = {
	.cs             = LDP_SMSC911X_CS,
	.gpio_irq       = LDP_SMSC911X_GPIO,
	.gpio_reset     = -EINVAL,
	.flags		= SMSC911X_USE_32BIT,
};

static inline void __init ldp_init_smsc911x(void)
{
	gpmc_smsc911x_init(&smsc911x_cfg);
}

static struct platform_device ldp_lcd_device = {
	.name		= "ldp_lcd",
	.id		= -1,
};

static struct omap_lcd_config ldp_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_board_config_kernel ldp_config[] __initdata = {
	{ OMAP_TAG_LCD,		&ldp_lcd_config },
};

static void __init omap_ldp_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
}

static struct twl4030_usb_data ldp_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data ldp_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
};

static struct twl4030_madc_platform_data ldp_madc_data = {
	.irq_line	= 1,
};

static struct regulator_consumer_supply ldp_vmmc1_supply = {
	.supply			= "vmmc",
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data ldp_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldp_vmmc1_supply,
};

/* ads7846 on SPI */
static struct regulator_consumer_supply ldp_vaux1_supplies[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
};

/* VAUX1 */
static struct regulator_init_data ldp_vaux1 = {
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= ARRAY_SIZE(ldp_vaux1_supplies),
	.consumer_supplies		= ldp_vaux1_supplies,
};

static struct twl4030_platform_data ldp_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &ldp_madc_data,
	.usb		= &ldp_usb_data,
	.vmmc1		= &ldp_vmmc1,
	.vaux1		= &ldp_vaux1,
	.gpio		= &ldp_gpio_data,
	.keypad		= &ldp_kp_twl4030_data,
};

static int __init omap_i2c_init(void)
{
	omap3_pmic_init("twl4030", &ldp_twldata);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

static struct platform_device *ldp_devices[] __initdata = {
	&ldp_lcd_device,
	&ldp_gpio_keys_device,
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct mtd_partition ldp_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),	/* 512KB, 0x80000 */
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * (64 * 2048),	/* 1.25MB, 0x140000 */
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",
		.offset		= MTDPART_OFS_APPEND,   /* Offset = 0x1c0000 */
		.size		= 2 * (64 * 2048),	/* 256KB, 0x40000 */
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x0200000*/
		.size		= 240 * (64 * 2048),	/* 30M, 0x1E00000 */
	},
	{
		.name		= "File System - NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x2000000 */
		.size		= MTDPART_SIZ_FULL,	/* 96MB, 0x6000000 */
	},

};

static void __init omap_ldp_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap_board_config = ldp_config;
	omap_board_config_size = ARRAY_SIZE(ldp_config);
	ldp_init_smsc911x();
	omap_i2c_init();
	platform_add_devices(ldp_devices, ARRAY_SIZE(ldp_devices));
	omap_ads7846_init(1, 54, 310, NULL);
	omap_serial_init();
	usb_musb_init(NULL);
	board_nand_init(ldp_nand_partitions,
		ARRAY_SIZE(ldp_nand_partitions), ZOOM_NAND_CS, 0);

	omap2_hsmmc_init(mmc);
	/* link regulators to MMC adapters */
	ldp_vmmc1_supply.dev = mmc[0].dev;
}

MACHINE_START(OMAP_LDP, "OMAP LDP board")
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_ldp_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_ldp_init,
	.timer		= &omap_timer,
MACHINE_END
