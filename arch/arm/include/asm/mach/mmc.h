/*
 *  arch/arm/include/asm/mach/mmc.h
 */
#ifndef ASMARM_MACH_MMC_H
#define ASMARM_MACH_MMC_H

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>

struct embedded_sdio_data {
        struct sdio_cis cis;
        struct sdio_cccr cccr;
        struct sdio_embedded_func *funcs;
        int num_funcs;
};

struct mmc_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	u32 (*translate_vdd)(struct device *, unsigned int);
	unsigned int (*status)(struct device *);
	unsigned int status_irq;
	struct embedded_sdio_data *embedded_sdio;
        unsigned int sdiowakeup_irq;
	int (*register_status_notify)(void (*callback)(int card_present, void *dev_id), void *dev_id);
	unsigned int *slot_type;
	unsigned long irq_flags;
	unsigned dat0_gpio;
#ifdef CONFIG_MMC_SUPPORT_EXTERNEL_DRIVER
	int use_ext_sdiodrv;
	unsigned char *ext_sdiodrv_name;
#endif
	unsigned long mmc_bus_width;
	unsigned char non_hot_plug;
	int (*wpswitch) (struct device *);
	int dummy52_required;
	unsigned int msmsdcc_fmin;
	unsigned int msmsdcc_fmid;
	unsigned int msmsdcc_fmax;
	bool nonremovable;
	bool pclk_src_dfab;
};

#endif
