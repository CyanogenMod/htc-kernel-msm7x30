/*
 *  linux/drivers/mmc/core/sd.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/kthread.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"
#include "sd_ops.h"

extern void mmc_power_up(struct mmc_host *host);
extern void mmc_power_off(struct mmc_host *host);

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

/*
 * Given the decoded CSD structure, decode the raw CID to our CID structure.
 */
static void mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	memset(&card->cid, 0, sizeof(struct mmc_cid));

	/*
	 * SD doesn't currently have a version field so we will
	 * have to assume we can parse this.
	 */
	card->cid.manfid		= UNSTUFF_BITS(resp, 120, 8);
	card->cid.oemid			= UNSTUFF_BITS(resp, 104, 16);
	card->cid.prod_name[0]		= UNSTUFF_BITS(resp, 96, 8);
	card->cid.prod_name[1]		= UNSTUFF_BITS(resp, 88, 8);
	card->cid.prod_name[2]		= UNSTUFF_BITS(resp, 80, 8);
	card->cid.prod_name[3]		= UNSTUFF_BITS(resp, 72, 8);
	card->cid.prod_name[4]		= UNSTUFF_BITS(resp, 64, 8);
	card->cid.hwrev			= UNSTUFF_BITS(resp, 60, 4);
	card->cid.fwrev			= UNSTUFF_BITS(resp, 56, 4);
	card->cid.serial		= UNSTUFF_BITS(resp, 24, 32);
	card->cid.year			= UNSTUFF_BITS(resp, 12, 8);
	card->cid.month			= UNSTUFF_BITS(resp, 8, 4);

	card->cid.year += 2000; /* SD cards year offset */
}

/*
 * Given a 128-bit response, decode to our card CSD structure.
 */
static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, csd_struct;
	u32 *resp = card->raw_csd;

	csd_struct = UNSTUFF_BITS(resp, 126, 2);

	switch (csd_struct) {
	case 0:
		m = UNSTUFF_BITS(resp, 115, 4);
		e = UNSTUFF_BITS(resp, 112, 3);
		csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
		csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

		e = UNSTUFF_BITS(resp, 47, 3);
		m = UNSTUFF_BITS(resp, 62, 12);
		csd->capacity	  = (1 + m) << (e + 2);

		csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
		csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
		csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
		csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
		csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
		csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
		csd->write_partial = UNSTUFF_BITS(resp, 21, 1);
		break;
	case 1:
		/*
		 * This is a block-addressed SDHC card. Most
		 * interesting fields are unused and have fixed
		 * values. To avoid getting tripped by buggy cards,
		 * we assume those fixed values ourselves.
		 */
		mmc_card_set_blockaddr(card);

		csd->tacc_ns	 = 0; /* Unused */
		csd->tacc_clks	 = 0; /* Unused */

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

		m = UNSTUFF_BITS(resp, 48, 22);
		csd->capacity     = (1 + m) << 10;

		csd->read_blkbits = 9;
		csd->read_partial = 0;
		csd->write_misalign = 0;
		csd->read_misalign = 0;
		csd->r2w_factor = 4; /* Unused */
		csd->write_blkbits = 9;
		csd->write_partial = 0;
		break;
	default:
		printk(KERN_ERR "%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd_struct);
		return -EINVAL;
	}

	return 0;
}

/*
 * Given a 64-bit response, decode to our card SCR structure.
 */
static int mmc_decode_scr(struct mmc_card *card)
{
	struct sd_scr *scr = &card->scr;
	unsigned int scr_struct;
	u32 resp[4];

	resp[3] = card->raw_scr[1];
	resp[2] = card->raw_scr[0];

	scr_struct = UNSTUFF_BITS(resp, 60, 4);
	if (scr_struct != 0) {
		printk(KERN_ERR "%s: unrecognised SCR structure version %d\n",
			mmc_hostname(card->host), scr_struct);
		return -EINVAL;
	}

	scr->sda_vsn = UNSTUFF_BITS(resp, 56, 4);
	scr->bus_widths = UNSTUFF_BITS(resp, 48, 4);

	return 0;
}

/*
 * Fetches and decodes switch information
 */
static int mmc_read_switch(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
		return 0;

	if (!(card->csd.cmdclass & CCC_SWITCH)) {
		printk(KERN_WARNING "%s: card lacks mandatory switch "
			"function, performance might suffer.\n",
			mmc_hostname(card->host));
		return 0;
	}

	err = -EIO;

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		printk(KERN_ERR "%s: could not allocate a buffer for "
			"switch capabilities.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_sd_switch(card, 0, 0, 1, status);
	if (err) {
		/* If the host or the card can't do the switch,
		 * fail more gracefully. */
		if ((err != -EINVAL)
		 && (err != -ENOSYS)
		 && (err != -EFAULT))
			goto out;

		printk(KERN_WARNING "%s: problem reading switch "
			"capabilities, performance might suffer.\n",
			mmc_hostname(card->host));
		err = 0;

		goto out;
	}

	if (status[13] & 0x02)
		card->sw_caps.hs_max_dtr = 50000000;

out:
	kfree(status);

	return err;
}

/*
 * Test if the card supports high-speed mode and, if so, switch to it.
 */
static int mmc_switch_hs(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
		return 0;

	if (!(card->csd.cmdclass & CCC_SWITCH))
		return 0;

	if (!(card->host->caps & MMC_CAP_SD_HIGHSPEED))
		return 0;

	if (card->sw_caps.hs_max_dtr == 0)
		return 0;

	err = -EIO;

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		printk(KERN_ERR "%s: could not allocate a buffer for "
			"switch capabilities.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_sd_switch(card, 1, 0, 1, status);
	if (err)
		goto out;

	if ((status[16] & 0xF) != 1) {
		printk(KERN_WARNING "%s: Problem switching card "
			"into high-speed mode!\n",
			mmc_hostname(card->host));
	} else {
		mmc_card_set_highspeed(card);
		mmc_set_timing(card->host, MMC_TIMING_SD_HS);
	}

out:
	kfree(status);

	return err;
}

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(scr, "%08x%08x\n", card->raw_scr[0], card->raw_scr[1]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);


static struct attribute *sd_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_scr.attr,
	&dev_attr_date.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_serial.attr,
	NULL,
};

static struct attribute_group sd_std_attr_group = {
	.attrs = sd_std_attrs,
};

static const struct attribute_group *sd_attr_groups[] = {
	&sd_std_attr_group,
	NULL,
};

static struct device_type sd_type = {
	.groups = sd_attr_groups,
};

/*
 * Handle the detection and initialisation of a card.
 *
 * In the case of a resume, "oldcard" will contain the card
 * we're trying to reinitialise.
 */
static int mmc_sd_init_card(struct mmc_host *host, u32 ocr,
	struct mmc_card *oldcard)
{
	struct mmc_card *card;
	int err;
	u32 cid[4];
	unsigned int max_dtr;
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	int retries;
#endif
	BUG_ON(!host);
	WARN_ON(!host->claimed);

	/*
	 * Since we're changing the OCR value, we seem to
	 * need to tell some cards to go back to the idle
	 * state.  We wait 1ms to give cards time to
	 * respond.
	 */
	mmc_go_idle(host);

	/*
	 * If SD_SEND_IF_COND indicates an SD 2.0
	 * compliant card and we should set bit 30
	 * of the ocr to indicate that we can handle
	 * block-addressed SDHC cards.
	 */
	err = mmc_send_if_cond(host, ocr);
	if (!err)
		ocr |= 1 << 30;

	err = mmc_send_app_op_cond(host, ocr, NULL);
	if (err)
		goto err;

	/*
	 * Fetch CID from card.
	 */
	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);
	else
		err = mmc_all_send_cid(host, cid);
	if (err)
		goto err;

	if (oldcard) {
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0) {
			err = -ENOENT;
			goto err;
		}

		card = oldcard;
	} else {
		/*
		 * Allocate card structure.
		 */
		card = mmc_alloc_card(host, &sd_type);
		if (IS_ERR(card)) {
			err = PTR_ERR(card);
			goto err;
		}

		card->type = MMC_TYPE_SD;
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
	}

	/*
	 * For native busses:  get card RCA and quit open drain mode.
	 */
	if (!mmc_host_is_spi(host)) {
		err = mmc_send_relative_addr(host, &card->rca);
		if (err)
			goto free_card;

		mmc_set_bus_mode(host, MMC_BUSMODE_PUSHPULL);
	}

	if (!oldcard) {
		/*
		 * Fetch CSD from card.
		 */
		err = mmc_send_csd(card, card->raw_csd);
		if (err)
			goto free_card;

		err = mmc_decode_csd(card);
		if (err)
			goto free_card;

		mmc_decode_cid(card);
	}

	/*
	 * Select card, as all following commands rely on that.
	 */
	if (!mmc_host_is_spi(host)) {
		err = mmc_select_card(card);
		if (err)
			goto free_card;
	}

	if (!oldcard) {
		/*
		 * Fetch SCR from card.
		 */
		err = mmc_app_send_scr(card, card->raw_scr);
		if (err)
			goto free_card;

		err = mmc_decode_scr(card);
		if (err < 0)
			goto free_card;
		/*
		 * Fetch switch information from card.
		 */
#ifdef CONFIG_MMC_PARANOID_SD_INIT
		for (retries = 1; retries <= 3; retries++) {
			err = mmc_read_switch(card);
			if (!err) {
				if (retries > 1) {
					printk(KERN_WARNING
					       "%s: recovered\n", 
					       mmc_hostname(host));
				}
				break;
			} else {
				printk(KERN_WARNING
				       "%s: read switch failed (attempt %d)\n",
				       mmc_hostname(host), retries);
			}
		}
#else
		err = mmc_read_switch(card);
#endif

		if (err)
			goto free_card;
	}

	/*
	 * For SPI, enable CRC as appropriate.
	 * This CRC enable is located AFTER the reading of the
	 * card registers because some SDHC cards are not able
	 * to provide valid CRCs for non-512-byte blocks.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err)
			goto free_card;
	}

	/*
	 * Attempt to change to high-speed (if supported)
	 */
	err = mmc_switch_hs(card);
	if (err)
		goto free_card;

	/*
	 * Compute bus speed.
	 */
	max_dtr = (unsigned int)-1;

	if (mmc_card_highspeed(card)) {
		if (max_dtr > card->sw_caps.hs_max_dtr) {
			max_dtr = card->sw_caps.hs_max_dtr;
			printk(KERN_WARNING "%s: high speed mode max_dtr = %d\n",
				       mmc_hostname(host), max_dtr);
		}
	} else if (max_dtr > card->csd.max_dtr) {
		max_dtr = card->csd.max_dtr;
		printk(KERN_WARNING "%s: non-hight speed mode max_dtr = %d\n",
			mmc_hostname(host), max_dtr);
	}

	mmc_set_clock(host, max_dtr);

	/*
	 * Switch to wider bus (if supported).
	 */
	if ((host->caps & MMC_CAP_4_BIT_DATA) &&
		(card->scr.bus_widths & SD_SCR_BUS_WIDTH_4)) {
		err = mmc_app_set_bus_width(card, MMC_BUS_WIDTH_4);
		if (err)
			goto free_card;

		mmc_set_bus_width(host, MMC_BUS_WIDTH_4);
	}

	/*
	 * Check if read-only switch is active.
	 */
	if (!oldcard) {
		if (!host->ops->get_ro || host->ops->get_ro(host) < 0) {
			printk(KERN_WARNING "%s: host does not "
				"support reading read-only "
				"switch. assuming write-enable.\n",
				mmc_hostname(host));
		} else {
			if (host->ops->get_ro(host) > 0)
				mmc_card_set_readonly(card);
		}
	}

	if (!oldcard)
		host->card = card;

	return 0;

free_card:
	if (!oldcard)
		mmc_remove_card(card);
err:

	return err;
}

/*
 * Host is being removed. Free up the current card.
 */
static void mmc_sd_remove(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_remove_card(host->card);
	host->card = NULL;
}

/*
 * Card detection callback from host.
 */
static void mmc_sd_detect(struct mmc_host *host)
{
	int err = 0;
#ifdef CONFIG_MMC_PARANOID_SD_INIT
        int retries = 5;
#endif

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);

	/*
	 * Just check if our card has been removed.
	 */
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	while(retries) {
		err = mmc_send_status(host->card, NULL);
		if (err) {
			retries--;
			udelay(5);
			continue;
		}
		break;
	}
	if (!retries) {
		printk(KERN_ERR "%s(%s): Unable to re-detect card (%d)\n",
		       __func__, mmc_hostname(host), err);
	}
#else
	err = mmc_send_status(host->card, NULL);
#endif
	mmc_release_host(host);

	if (err) {
		mmc_sd_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);
	}
}

/*
 * Suspend callback from host.
 */
static int mmc_sd_suspend(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	if (!mmc_host_is_spi(host))
		mmc_deselect_cards(host);
	host->card->state &= ~MMC_STATE_HIGHSPEED;
	mmc_release_host(host);

	return 0;
}

/*
 * Resume callback from host.
 *
 * This function tries to determine if the same card is still present
 * and, if so, restore all state to it.
 */
static int mmc_sd_resume(struct mmc_host *host)
{
	int err;
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	int retries;
#endif

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	retries = 5;
	while (retries) {
		err = mmc_sd_init_card(host, host->ocr, host->card);

		if (err) {
			printk(KERN_ERR "%s: Re-init card rc = %d (retries = %d)\n",
			       mmc_hostname(host), err, retries);
			mmc_power_off(host);
			mdelay(5);
			mmc_power_up(host);
			retries--;
			continue;
		}
		break;
	}
#else
	err = mmc_sd_init_card(host, host->ocr, host->card);
#endif
	mmc_release_host(host);

	return err;
}

static void mmc_sd_power_restore(struct mmc_host *host)
{
	host->card->state &= ~MMC_STATE_HIGHSPEED;
	mmc_claim_host(host);
	mmc_sd_init_card(host, host->ocr, host->card);
	mmc_release_host(host);
}

#ifdef CONFIG_MMC_UNSAFE_RESUME

static const struct mmc_bus_ops mmc_sd_ops = {
	.remove = mmc_sd_remove,
	.detect = mmc_sd_detect,
	.suspend = mmc_sd_suspend,
	.resume = mmc_sd_resume,
	.power_restore = mmc_sd_power_restore,
};

static void mmc_sd_attach_bus_ops(struct mmc_host *host)
{
	mmc_attach_bus(host, &mmc_sd_ops);
}

#else

static const struct mmc_bus_ops mmc_sd_ops = {
	.remove = mmc_sd_remove,
	.detect = mmc_sd_detect,
	.suspend = NULL,
	.resume = NULL,
	.power_restore = mmc_sd_power_restore,
};

static const struct mmc_bus_ops mmc_sd_ops_unsafe = {
	.remove = mmc_sd_remove,
	.detect = mmc_sd_detect,
	.suspend = mmc_sd_suspend,
	.resume = mmc_sd_resume,
	.power_restore = mmc_sd_power_restore,
};

static void mmc_sd_attach_bus_ops(struct mmc_host *host)
{
	const struct mmc_bus_ops *bus_ops;

	if (host->caps & MMC_CAP_NONREMOVABLE || !mmc_assume_removable)
		bus_ops = &mmc_sd_ops_unsafe;
	else
		bus_ops = &mmc_sd_ops;
	mmc_attach_bus(host, bus_ops);
}

#endif

/*
 * Starting point for SD card init.
 */
int mmc_attach_sd(struct mmc_host *host, u32 ocr)
{
	int err;
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	int retries;
#endif

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	mmc_sd_attach_bus_ops(host);

	/*
	 * We need to get OCR a different way for SPI.
	 */
	if (mmc_host_is_spi(host)) {
		mmc_go_idle(host);

		err = mmc_spi_read_ocr(host, 0, &ocr);
		if (err)
			goto err;
	}

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		printk(KERN_WARNING "%s: card claims to support voltages "
		       "below the defined range. These will be ignored.\n",
		       mmc_hostname(host));
		ocr &= ~0x7F;
	}

	if (ocr & MMC_VDD_165_195) {
		printk(KERN_WARNING "%s: SD card claims to support the "
		       "incompletely defined 'low voltage range'. This "
		       "will be ignored.\n", mmc_hostname(host));
		ocr &= ~MMC_VDD_165_195;
	}

	host->ocr = mmc_select_voltage(host, ocr);

	/*
	 * Can we support the voltage(s) of the card(s)?
	 */
	if (!host->ocr) {
		err = -EINVAL;
		goto err;
	}

	/*
	 * Detect and init the card.
	 */
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	retries = 5;
	while (retries) {
		err = mmc_sd_init_card(host, host->ocr, NULL);
		if (err) {
			retries--;
			continue;
		}
		break;
	}

	if (!retries) {
		printk(KERN_ERR "%s: mmc_sd_init_card() failure (err = %d)\n",
		       mmc_hostname(host), err);
		goto err;
	}
#else
	err = mmc_sd_init_card(host, host->ocr, NULL);
	if (err)
		goto err;
#endif

	mmc_release_host(host);

	err = mmc_add_card(host->card);
	if (err)
		goto remove_card;

	return 0;

remove_card:
	mmc_remove_card(host->card);
	host->card = NULL;
	mmc_claim_host(host);
err:
	mmc_detach_bus(host);
	mmc_release_host(host);

	printk(KERN_ERR "%s: error %d whilst initialising SD card\n",
		mmc_hostname(host), err);

	return err;
}

