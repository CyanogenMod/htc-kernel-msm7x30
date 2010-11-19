/*
 * BCMSDH Function Driver for the native SDIO/MMC driver in the Linux Kernel
 *
 * Copyright (C) 1999-2010, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: bcmsdh_sdmmc.c,v 1.1.2.5.6.30.4.1 2010/09/02 23:12:21 Exp $
 */
#include <typedefs.h>

#include <bcmdevs.h>
#include <bcmendian.h>
#include <bcmutils.h>
#include <osl.h>
#include <sdio.h>	/* SDIO Device and Protocol Specs */
#include <sdioh.h>	/* SDIO Host Controller Specification */
#include <bcmsdbus.h>	/* bcmsdh to/from specific controller APIs */
#include <sdiovar.h>	/* ioctl/iovars */

#include <linux/mmc/core.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include <dngl_stats.h>
#include <dhd.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_PM_SLEEP)
#include <linux/suspend.h>
extern volatile bool dhd_mmc_suspend;
#endif
#include "bcmsdh_sdmmc.h"

#ifndef BCMSDH_MODULE
extern int sdio_function_init(void);
extern void sdio_function_cleanup(void);
#endif /* BCMSDH_MODULE */

#if !defined(OOB_INTR_ONLY)
static void IRQHandler(struct sdio_func *func);
static void IRQHandlerF2(struct sdio_func *func);
#endif /* !defined(OOB_INTR_ONLY) */
static int sdioh_sdmmc_get_cisaddr(sdioh_info_t *sd, uint32 regaddr);
extern int sdio_reset_comm(struct mmc_card *card);

extern PBCMSDH_SDMMC_INSTANCE gInstance;

uint sd_sdmode = SDIOH_MODE_SD4;	/* Use SD4 mode by default */
uint sd_f2_blocksize = 512;		/* Default blocksize */

uint sd_divisor = 2;			/* Default 48MHz/2 = 24MHz */

uint sd_power = 1;		/* Default to SD Slot powered ON */
uint sd_clock = 1;		/* Default to SD Clock turned ON */
uint sd_hiok = FALSE;	/* Don't use hi-speed mode by default */
uint sd_msglevel = 0x01;
uint sd_use_dma = TRUE;
DHD_PM_RESUME_WAIT_INIT(sdioh_request_byte_wait);
DHD_PM_RESUME_WAIT_INIT(sdioh_request_word_wait);
DHD_PM_RESUME_WAIT_INIT(sdioh_request_packet_wait);
DHD_PM_RESUME_WAIT_INIT(sdioh_request_buffer_wait);

#define DMA_ALIGN_MASK	0x03

int sdioh_sdmmc_card_regread(sdioh_info_t *sd, int func, uint32 regaddr, int regsize, uint32 *data);

static int
sdioh_sdmmc_card_enablefuncs(sdioh_info_t *sd)
{
	int err_ret;
	uint32 fbraddr;
	uint8 func;

	sd_trace(("%s\n", __FUNCTION__));

	/* Get the Card's common CIS address */
	sd->com_cis_ptr = sdioh_sdmmc_get_cisaddr(sd, SDIOD_CCCR_CISPTR_0);
	sd->func_cis_ptr[0] = sd->com_cis_ptr;
	sd_info(("%s: Card's Common CIS Ptr = 0x%x\n", __FUNCTION__, sd->com_cis_ptr));

	/* Get the Card's function CIS (for each function) */
	for (fbraddr = SDIOD_FBR_STARTADDR, func = 1;
	     func <= sd->num_funcs; func++, fbraddr += SDIOD_FBR_SIZE) {
		sd->func_cis_ptr[func] = sdioh_sdmmc_get_cisaddr(sd, SDIOD_FBR_CISPTR_0 + fbraddr);
		sd_info(("%s: Function %d CIS Ptr = 0x%x\n",
		         __FUNCTION__, func, sd->func_cis_ptr[func]));
	}

	sd->func_cis_ptr[0] = sd->com_cis_ptr;
	sd_info(("%s: Card's Common CIS Ptr = 0x%x\n", __FUNCTION__, sd->com_cis_ptr));

	/* Enable Function 1 */
	sdio_claim_host(gInstance->func[1]);
	err_ret = sdio_enable_func(gInstance->func[1]);
	sdio_release_host(gInstance->func[1]);
	if (err_ret) {
		sd_err(("bcmsdh_sdmmc: Failed to enable F1 Err: 0x%08x", err_ret));
	}

	return FALSE;
}

/*
 *	Public entry points & extern's
 */
extern sdioh_info_t *
sdioh_attach(osl_t *osh, void *bar0, uint irq)
{
	sdioh_info_t *sd;
	int err_ret;

	sd_trace(("%s\n", __FUNCTION__));

	if (gInstance == NULL) {
		sd_err(("%s: SDIO Device not present\n", __FUNCTION__));
		return NULL;
	}

	if ((sd = (sdioh_info_t *)MALLOC(osh, sizeof(sdioh_info_t))) == NULL) {
		sd_err(("sdioh_attach: out of memory, malloced %d bytes\n", MALLOCED(osh)));
		return NULL;
	}
	bzero((char *)sd, sizeof(sdioh_info_t));
	sd->osh = osh;
	if (sdioh_sdmmc_osinit(sd) != 0) {
		sd_err(("%s:sdioh_sdmmc_osinit() failed\n", __FUNCTION__));
		MFREE(sd->osh, sd, sizeof(sdioh_info_t));
		return NULL;
	}

	sd->num_funcs = 2;
	sd->sd_blockmode = TRUE;
	sd->use_client_ints = TRUE;
	sd->client_block_size[0] = 64;

	gInstance->sd = sd;

	/* Claim host controller */
	sdio_claim_host(gInstance->func[1]);

	sd->client_block_size[1] = 64;
	err_ret = sdio_set_block_size(gInstance->func[1], 64);
	if (err_ret) {
		sd_err(("bcmsdh_sdmmc: Failed to set F1 blocksize\n"));
	}

	/* Release host controller F1 */
	sdio_release_host(gInstance->func[1]);

	if (gInstance->func[2]) {
		/* Claim host controller F2 */
		sdio_claim_host(gInstance->func[2]);

		sd->client_block_size[2] = sd_f2_blocksize;
		err_ret = sdio_set_block_size(gInstance->func[2], sd_f2_blocksize);
		if (err_ret) {
			sd_err(("bcmsdh_sdmmc: Failed to set F2 blocksize to %d\n",
				sd_f2_blocksize));
		}

		/* Release host controller F2 */
		sdio_release_host(gInstance->func[2]);
	}

	sdioh_sdmmc_card_enablefuncs(sd);

	sd_trace(("%s: Done\n", __FUNCTION__));
	return sd;
}


extern SDIOH_API_RC
sdioh_detach(osl_t *osh, sdioh_info_t *sd)
{
	sd_trace(("%s\n", __FUNCTION__));

	if (sd) {

		/* Disable Function 2 */
		sdio_claim_host(gInstance->func[2]);
		sdio_disable_func(gInstance->func[2]);
		sdio_release_host(gInstance->func[2]);

		/* Disable Function 1 */
		sdio_claim_host(gInstance->func[1]);
		sdio_disable_func(gInstance->func[1]);
		sdio_release_host(gInstance->func[1]);

		/* deregister irq */
		sdioh_sdmmc_osfree(sd);

		MFREE(sd->osh, sd, sizeof(sdioh_info_t));
	}
	return SDIOH_API_RC_SUCCESS;
}

#if defined(OOB_INTR_ONLY) && defined(HW_OOB)

extern SDIOH_API_RC
sdioh_enable_func_intr(void)
{
	uint8 reg;
	int err;

	if (gInstance->func[0]) {
		sdio_claim_host(gInstance->func[0]);

		reg = sdio_readb(gInstance->func[0], SDIOD_CCCR_INTEN, &err);
		if (err) {
			sd_err(("%s: error for read SDIO_CCCR_IENx : 0x%x\n", __FUNCTION__, err));
			sdio_release_host(gInstance->func[0]);
			return SDIOH_API_RC_FAIL;
		}

		/* Enable F1 and F2 interrupts, set master enable */
		reg |= (INTR_CTL_FUNC1_EN | INTR_CTL_FUNC2_EN | INTR_CTL_MASTER_EN);

		sdio_writeb(gInstance->func[0], reg, SDIOD_CCCR_INTEN, &err);
		sdio_release_host(gInstance->func[0]);

		if (err) {
			sd_err(("%s: error for write SDIO_CCCR_IENx : 0x%x\n", __FUNCTION__, err));
			return SDIOH_API_RC_FAIL;
		}
	}

	return SDIOH_API_RC_SUCCESS;
}

extern SDIOH_API_RC
sdioh_disable_func_intr(void)
{
	uint8 reg;
	int err;

	if (gInstance->func[0]) {
		sdio_claim_host(gInstance->func[0]);
		reg = sdio_readb(gInstance->func[0], SDIOD_CCCR_INTEN, &err);
		if (err) {
			sd_err(("%s: error for read SDIO_CCCR_IENx : 0x%x\n", __FUNCTION__, err));
			sdio_release_host(gInstance->func[0]);
			return SDIOH_API_RC_FAIL;
		}

		reg &= ~(INTR_CTL_FUNC1_EN | INTR_CTL_FUNC2_EN);
		/* Disable master interrupt with the last function interrupt */
		if (!(reg & 0xFE))
			reg = 0;
		sdio_writeb(gInstance->func[0], reg, SDIOD_CCCR_INTEN, &err);

		sdio_release_host(gInstance->func[0]);
		if (err) {
			sd_err(("%s: error for write SDIO_CCCR_IENx : 0x%x\n", __FUNCTION__, err));
			return SDIOH_API_RC_FAIL;
		}
	}
	return SDIOH_API_RC_SUCCESS;
}
#endif /* defined(OOB_INTR_ONLY) && defined(HW_OOB) */

/* Configure callback to client when we recieve client interrupt */
extern SDIOH_API_RC
sdioh_interrupt_register(sdioh_info_t *sd, sdioh_cb_fn_t fn, void *argh)
{
	sd_trace(("%s: Entering\n", __FUNCTION__));
	if (fn == NULL) {
		sd_err(("%s: interrupt handler is NULL, not registering\n", __FUNCTION__));
		return SDIOH_API_RC_FAIL;
	}
#if !defined(OOB_INTR_ONLY)
	sd->intr_handler = fn;
	sd->intr_handler_arg = argh;
	sd->intr_handler_valid = TRUE;

	/* register and unmask irq */
	if (gInstance->func[2]) {
		sdio_claim_host(gInstance->func[2]);
		sdio_claim_irq(gInstance->func[2], IRQHandlerF2);
		sdio_release_host(gInstance->func[2]);
	}

	if (gInstance->func[1]) {
		sdio_claim_host(gInstance->func[1]);
		sdio_claim_irq(gInstance->func[1], IRQHandler);
		sdio_release_host(gInstance->func[1]);
	}
#elif defined(HW_OOB)
	sdioh_enable_func_intr();
#endif /* defined(OOB_INTR_ONLY) */
	return SDIOH_API_RC_SUCCESS;
}

extern SDIOH_API_RC
sdioh_interrupt_deregister(sdioh_info_t *sd)
{
	sd_trace(("%s: Entering\n", __FUNCTION__));

#if !defined(OOB_INTR_ONLY)
	if (gInstance->func[1]) {
		/* register and unmask irq */
		sdio_claim_host(gInstance->func[1]);
		sdio_release_irq(gInstance->func[1]);
		sdio_release_host(gInstance->func[1]);
	}

	if (gInstance->func[2]) {
		/* Claim host controller F2 */
		sdio_claim_host(gInstance->func[2]);
		sdio_release_irq(gInstance->func[2]);
		/* Release host controller F2 */
		sdio_release_host(gInstance->func[2]);
	}

	sd->intr_handler_valid = FALSE;
	sd->intr_handler = NULL;
	sd->intr_handler_arg = NULL;
#elif defined(HW_OOB)
	sdioh_disable_func_intr();
#endif /*  !defined(OOB_INTR_ONLY) */
	return SDIOH_API_RC_SUCCESS;
}

extern SDIOH_API_RC
sdioh_interrupt_query(sdioh_info_t *sd, bool *onoff)
{
	sd_trace(("%s: Entering\n", __FUNCTION__));
	*onoff = sd->client_intr_enabled;
	return SDIOH_API_RC_SUCCESS;
}

#if defined(DHD_DEBUG)
extern bool
sdioh_interrupt_pending(sdioh_info_t *sd)
{
	return (0);
}
#endif

uint
sdioh_query_iofnum(sdioh_info_t *sd)
{
	return sd->num_funcs;
}

/* IOVar table */
enum {
	IOV_MSGLEVEL = 1,
	IOV_BLOCKMODE,
	IOV_BLOCKSIZE,
	IOV_DMA,
	IOV_USEINTS,
	IOV_NUMINTS,
	IOV_NUMLOCALINTS,
	IOV_HOSTREG,
	IOV_DEVREG,
	IOV_DIVISOR,
	IOV_SDMODE,
	IOV_HISPEED,
	IOV_HCIREGS,
	IOV_POWER,
	IOV_CLOCK,
	IOV_RXCHAIN
};

const bcm_iovar_t sdioh_iovars[] = {
	{"sd_msglevel", IOV_MSGLEVEL,	0,	IOVT_UINT32,	0 },
	{"sd_blockmode", IOV_BLOCKMODE, 0,	IOVT_BOOL,	0 },
	{"sd_blocksize", IOV_BLOCKSIZE, 0,	IOVT_UINT32,	0 }, /* ((fn << 16) | size) */
	{"sd_dma",	IOV_DMA,	0,	IOVT_BOOL,	0 },
	{"sd_ints", 	IOV_USEINTS,	0,	IOVT_BOOL,	0 },
	{"sd_numints",	IOV_NUMINTS,	0,	IOVT_UINT32,	0 },
	{"sd_numlocalints", IOV_NUMLOCALINTS, 0, IOVT_UINT32,	0 },
	{"sd_hostreg",	IOV_HOSTREG,	0,	IOVT_BUFFER,	sizeof(sdreg_t) },
	{"sd_devreg",	IOV_DEVREG, 	0,	IOVT_BUFFER,	sizeof(sdreg_t) },
	{"sd_divisor",	IOV_DIVISOR,	0,	IOVT_UINT32,	0 },
	{"sd_power",	IOV_POWER,	0,	IOVT_UINT32,	0 },
	{"sd_clock",	IOV_CLOCK,	0,	IOVT_UINT32,	0 },
	{"sd_mode", 	IOV_SDMODE, 	0,	IOVT_UINT32,	100},
	{"sd_highspeed", IOV_HISPEED,	0,	IOVT_UINT32,	0 },
	{"sd_rxchain",  IOV_RXCHAIN,    0, 	IOVT_BOOL,	0 },
	{NULL, 0, 0, 0, 0 }
};

int
sdioh_iovar_op(sdioh_info_t *si, const char *name,
                           void *params, int plen, void *arg, int len, bool set)
{
	const bcm_iovar_t *vi = NULL;
	int bcmerror = 0;
	int val_size;
	int32 int_val = 0;
	bool bool_val;
	uint32 actionid;

	ASSERT(name);
	ASSERT(len >= 0);

	/* Get must have return space; Set does not take qualifiers */
	ASSERT(set || (arg && len));
	ASSERT(!set || (!params && !plen));

	sd_trace(("%s: Enter (%s %s)\n", __FUNCTION__, (set ? "set" : "get"), name));

	if ((vi = bcm_iovar_lookup(sdioh_iovars, name)) == NULL) {
		bcmerror = BCME_UNSUPPORTED;
		goto exit;
	}

	if ((bcmerror = bcm_iovar_lencheck(vi, arg, len, set)) != 0)
		goto exit;

	/* Set up params so get and set can share the convenience variables */
	if (params == NULL) {
		params = arg;
		plen = len;
	}

	if (vi->type == IOVT_VOID)
		val_size = 0;
	else if (vi->type == IOVT_BUFFER)
		val_size = len;
	else
		val_size = sizeof(int);

	if (plen >= (int)sizeof(int_val))
		bcopy(params, &int_val, sizeof(int_val));

	bool_val = (int_val != 0) ? TRUE : FALSE;

	actionid = set ? IOV_SVAL(vi->varid) : IOV_GVAL(vi->varid);
	switch (actionid) {
	case IOV_GVAL(IOV_MSGLEVEL):
		int_val = (int32)sd_msglevel;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_MSGLEVEL):
		sd_msglevel = int_val;
		break;

	case IOV_GVAL(IOV_BLOCKMODE):
		int_val = (int32)si->sd_blockmode;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_BLOCKMODE):
		si->sd_blockmode = (bool)int_val;
		/* Haven't figured out how to make non-block mode with DMA */
		break;

	case IOV_GVAL(IOV_BLOCKSIZE):
		if ((uint32)int_val > si->num_funcs) {
			bcmerror = BCME_BADARG;
			break;
		}
		int_val = (int32)si->client_block_size[int_val];
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_BLOCKSIZE):
	{
		uint func = ((uint32)int_val >> 16);
		uint blksize = (uint16)int_val;
		uint maxsize;

		if (func > si->num_funcs) {
			bcmerror = BCME_BADARG;
			break;
		}

		switch (func) {
		case 0: maxsize = 32; break;
		case 1: maxsize = BLOCK_SIZE_4318; break;
		case 2: maxsize = BLOCK_SIZE_4328; break;
		default: maxsize = 0;
		}
		if (blksize > maxsize) {
			bcmerror = BCME_BADARG;
			break;
		}
		if (!blksize) {
			blksize = maxsize;
		}

		/* Now set it */
		si->client_block_size[func] = blksize;

		break;
	}

	case IOV_GVAL(IOV_RXCHAIN):
		int_val = FALSE;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_DMA):
		int_val = (int32)si->sd_use_dma;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_DMA):
		si->sd_use_dma = (bool)int_val;
		break;

	case IOV_GVAL(IOV_USEINTS):
		int_val = (int32)si->use_client_ints;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_USEINTS):
		si->use_client_ints = (bool)int_val;
		if (si->use_client_ints)
			si->intmask |= CLIENT_INTR;
		else
			si->intmask &= ~CLIENT_INTR;

		break;

	case IOV_GVAL(IOV_DIVISOR):
		int_val = (uint32)sd_divisor;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_DIVISOR):
		sd_divisor = int_val;
		break;

	case IOV_GVAL(IOV_POWER):
		int_val = (uint32)sd_power;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_POWER):
		sd_power = int_val;
		break;

	case IOV_GVAL(IOV_CLOCK):
		int_val = (uint32)sd_clock;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_CLOCK):
		sd_clock = int_val;
		break;

	case IOV_GVAL(IOV_SDMODE):
		int_val = (uint32)sd_sdmode;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_SDMODE):
		sd_sdmode = int_val;
		break;

	case IOV_GVAL(IOV_HISPEED):
		int_val = (uint32)sd_hiok;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_SVAL(IOV_HISPEED):
		sd_hiok = int_val;
		break;

	case IOV_GVAL(IOV_NUMINTS):
		int_val = (int32)si->intrcount;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_NUMLOCALINTS):
		int_val = (int32)0;
		bcopy(&int_val, arg, val_size);
		break;

	case IOV_GVAL(IOV_HOSTREG):
	{
		sdreg_t *sd_ptr = (sdreg_t *)params;

		if (sd_ptr->offset < SD_SysAddr || sd_ptr->offset > SD_MaxCurCap) {
			sd_err(("%s: bad offset 0x%x\n", __FUNCTION__, sd_ptr->offset));
			bcmerror = BCME_BADARG;
			break;
		}

		sd_trace(("%s: rreg%d at offset %d\n", __FUNCTION__,
		                  (sd_ptr->offset & 1) ? 8 : ((sd_ptr->offset & 2) ? 16 : 32),
		                  sd_ptr->offset));
		if (sd_ptr->offset & 1)
			int_val = 8; /* sdioh_sdmmc_rreg8(si, sd_ptr->offset); */
		else if (sd_ptr->offset & 2)
			int_val = 16; /* sdioh_sdmmc_rreg16(si, sd_ptr->offset); */
		else
			int_val = 32; /* sdioh_sdmmc_rreg(si, sd_ptr->offset); */

		bcopy(&int_val, arg, sizeof(int_val));
		break;
	}

	case IOV_SVAL(IOV_HOSTREG):
	{
		sdreg_t *sd_ptr = (sdreg_t *)params;

		if (sd_ptr->offset < SD_SysAddr || sd_ptr->offset > SD_MaxCurCap) {
			sd_err(("%s: bad offset 0x%x\n", __FUNCTION__, sd_ptr->offset));
			bcmerror = BCME_BADARG;
			break;
		}

		sd_trace(("%s: wreg%d value 0x%08x at offset %d\n", __FUNCTION__, sd_ptr->value,
		                  (sd_ptr->offset & 1) ? 8 : ((sd_ptr->offset & 2) ? 16 : 32),
		                  sd_ptr->offset));
		break;
	}

	case IOV_GVAL(IOV_DEVREG):
	{
		sdreg_t *sd_ptr = (sdreg_t *)params;
		uint8 data = 0;

		if (sdioh_cfg_read(si, sd_ptr->func, sd_ptr->offset, &data)) {
			bcmerror = BCME_SDIO_ERROR;
			break;
		}

		int_val = (int)data;
		bcopy(&int_val, arg, sizeof(int_val));
		break;
	}

	case IOV_SVAL(IOV_DEVREG):
	{
		sdreg_t *sd_ptr = (sdreg_t *)params;
		uint8 data = (uint8)sd_ptr->value;

		if (sdioh_cfg_write(si, sd_ptr->func, sd_ptr->offset, &data)) {
			bcmerror = BCME_SDIO_ERROR;
			break;
		}
		break;
	}

	default:
		bcmerror = BCME_UNSUPPORTED;
		break;
	}
exit:

	return bcmerror;
}

#if defined(OOB_INTR_ONLY) && defined(HW_OOB)

SDIOH_API_RC
sdioh_enable_hw_oob_intr(sdioh_info_t *sd, bool enable)
{
	SDIOH_API_RC status;
	uint8 data;

	if (enable)
		data = 3;	/* enable hw oob interrupt */
	else
		data = 4;	/* disable hw oob interrupt */
	data |= 4;		/* Active HIGH */

	status = sdioh_request_byte(sd, SDIOH_WRITE, 0, 0xf2, &data);
	return status;
}
#endif /* defined(OOB_INTR_ONLY) && defined(HW_OOB) */

extern SDIOH_API_RC
sdioh_cfg_read(sdioh_info_t *sd, uint fnc_num, uint32 addr, uint8 *data)
{
	SDIOH_API_RC status;
	/* No lock needed since sdioh_request_byte does locking */
	status = sdioh_request_byte(sd, SDIOH_READ, fnc_num, addr, data);
	return status;
}

extern SDIOH_API_RC
sdioh_cfg_write(sdioh_info_t *sd, uint fnc_num, uint32 addr, uint8 *data)
{
	/* No lock needed since sdioh_request_byte does locking */
	SDIOH_API_RC status;
	status = sdioh_request_byte(sd, SDIOH_WRITE, fnc_num, addr, data);
	return status;
}

static int
sdioh_sdmmc_get_cisaddr(sdioh_info_t *sd, uint32 regaddr)
{
	/* read 24 bits and return valid 17 bit addr */
	int i;
	uint32 scratch, regdata;
	uint8 *ptr = (uint8 *)&scratch;
	for (i = 0; i < 3; i++) {
		if ((sdioh_sdmmc_card_regread (sd, 0, regaddr, 1, &regdata)) != SUCCESS)
			sd_err(("%s: Can't read!\n", __FUNCTION__));

		*ptr++ = (uint8) regdata;
		regaddr++;
	}

	/* Only the lower 17-bits are valid */
	scratch = ltoh32(scratch);
	scratch &= 0x0001FFFF;
	return (scratch);
}

extern SDIOH_API_RC
sdioh_cis_read(sdioh_info_t *sd, uint func, uint8 *cisd, uint32 length)
{
	uint32 count;
	int offset;
	uint32 foo;
	uint8 *cis = cisd;

	sd_trace(("%s: Func = %d\n", __FUNCTION__, func));

	if (!sd->func_cis_ptr[func]) {
		bzero(cis, length);
		sd_err(("%s: no func_cis_ptr[%d]\n", __FUNCTION__, func));
		return SDIOH_API_RC_FAIL;
	}

	sd_err(("%s: func_cis_ptr[%d]=0x%04x\n", __FUNCTION__, func, sd->func_cis_ptr[func]));

	for (count = 0; count < length; count++) {
		offset =  sd->func_cis_ptr[func] + count;
		if (sdioh_sdmmc_card_regread (sd, 0, offset, 1, &foo) < 0) {
			sd_err(("%s: regread failed: Can't read CIS\n", __FUNCTION__));
			return SDIOH_API_RC_FAIL;
		}

		*cis = (uint8)(foo & 0xff);
		cis++;
	}

	return SDIOH_API_RC_SUCCESS;
}

extern SDIOH_API_RC
sdioh_request_byte(sdioh_info_t *sd, uint rw, uint func, uint regaddr, uint8 *byte)
{
	int err_ret;

	sd_info(("%s: rw=%d, func=%d, addr=0x%05x\n", __FUNCTION__, rw, func, regaddr));

	DHD_PM_RESUME_WAIT(sdioh_request_byte_wait);
	DHD_PM_RESUME_RETURN_ERROR(SDIOH_API_RC_FAIL);
	if(rw) { /* CMD52 Write */
		if (func == 0) {
			/* Can only directly write to some F0 registers.  Handle F2 enable
			 * as a special case.
			 */
			if (regaddr == SDIOD_CCCR_IOEN) {
				if (gInstance->func[2]) {
					sdio_claim_host(gInstance->func[2]);
					if (*byte & SDIO_FUNC_ENABLE_2) {
						/* Enable Function 2 */
						err_ret = sdio_enable_func(gInstance->func[2]);
						if (err_ret) {
							sd_err(("bcmsdh_sdmmc: enable F2 failed:%d",
								err_ret));
						}
					} else {
						/* Disable Function 2 */
						err_ret = sdio_disable_func(gInstance->func[2]);
						if (err_ret) {
							sd_err(("bcmsdh_sdmmc: Disab F2 failed:%d",
								err_ret));
						}
					}
					sdio_release_host(gInstance->func[2]);
				}
			}
#if defined(MMC_SDIO_ABORT)
			/* to allow abort command through F1 */
			else if (regaddr == SDIOD_CCCR_IOABORT) {
				sdio_claim_host(gInstance->func[func]);
				/*
				* this sdio_f0_writeb() can be replaced with another api
				* depending upon MMC driver change.
				* As of this time, this is temporaray one
				*/
				sdio_writeb(gInstance->func[func], *byte, regaddr, &err_ret);
				sdio_release_host(gInstance->func[func]);
			}
#endif /* MMC_SDIO_ABORT */
			else if (regaddr < 0xF0) {
				sd_err(("bcmsdh_sdmmc: F0 Wr:0x%02x: write disallowed\n", regaddr));
			} else {
				/* Claim host controller, perform F0 write, and release */
				sdio_claim_host(gInstance->func[func]);
				sdio_f0_writeb(gInstance->func[func], *byte, regaddr, &err_ret);
				sdio_release_host(gInstance->func[func]);
			}
		} else {
			/* Claim host controller, perform Fn write, and release */
			sdio_claim_host(gInstance->func[func]);
			sdio_writeb(gInstance->func[func], *byte, regaddr, &err_ret);
			sdio_release_host(gInstance->func[func]);
		}
	} else { /* CMD52 Read */
		/* Claim host controller, perform Fn read, and release */
		sdio_claim_host(gInstance->func[func]);

		if (func == 0) {
			*byte = sdio_f0_readb(gInstance->func[func], regaddr, &err_ret);
		} else {
			*byte = sdio_readb(gInstance->func[func], regaddr, &err_ret);
		}

		sdio_release_host(gInstance->func[func]);
	}

	if (err_ret) {
		sd_err(("bcmsdh_sdmmc: Failed to %s byte F%d:@0x%05x=%02x, Err: %d\n",
		                        rw ? "Write" : "Read", func, regaddr, *byte, err_ret));
	}

	return ((err_ret == 0) ? SDIOH_API_RC_SUCCESS : SDIOH_API_RC_FAIL);
}

extern SDIOH_API_RC
sdioh_request_word(sdioh_info_t *sd, uint cmd_type, uint rw, uint func, uint addr,
                                   uint32 *word, uint nbytes)
{
	int err_ret = SDIOH_API_RC_FAIL;

	if (func == 0) {
		sd_err(("%s: Only CMD52 allowed to F0.\n", __FUNCTION__));
		return SDIOH_API_RC_FAIL;
	}

	sd_info(("%s: cmd_type=%d, rw=%d, func=%d, addr=0x%05x, nbytes=%d\n",
	         __FUNCTION__, cmd_type, rw, func, addr, nbytes));

	DHD_PM_RESUME_WAIT(sdioh_request_word_wait);
	DHD_PM_RESUME_RETURN_ERROR(SDIOH_API_RC_FAIL);
	/* Claim host controller */
	sdio_claim_host(gInstance->func[func]);

	if(rw) { /* CMD52 Write */
		if (nbytes == 4) {
			sdio_writel(gInstance->func[func], *word, addr, &err_ret);
		} else if (nbytes == 2) {
			sdio_writew(gInstance->func[func], (*word & 0xFFFF), addr, &err_ret);
		} else {
			sd_err(("%s: Invalid nbytes: %d\n", __FUNCTION__, nbytes));
		}
	} else { /* CMD52 Read */
		if (nbytes == 4) {
			*word = sdio_readl(gInstance->func[func], addr, &err_ret);
		} else if (nbytes == 2) {
			*word = sdio_readw(gInstance->func[func], addr, &err_ret) & 0xFFFF;
		} else {
			sd_err(("%s: Invalid nbytes: %d\n", __FUNCTION__, nbytes));
		}
	}

	/* Release host controller */
	sdio_release_host(gInstance->func[func]);

	if (err_ret) {
		sd_err(("bcmsdh_sdmmc: Failed to %s word, Err: 0x%08x",
		                        rw ? "Write" : "Read", err_ret));
	}

	return ((err_ret == 0) ? SDIOH_API_RC_SUCCESS : SDIOH_API_RC_FAIL);
}

static SDIOH_API_RC
sdioh_request_packet(sdioh_info_t *sd, uint fix_inc, uint write, uint func,
                     uint addr, void *pkt)
{
	bool fifo = (fix_inc == SDIOH_DATA_FIX);
	uint32	SGCount = 0;
	int err_ret = 0;

	void *pnext;

	sd_trace(("%s: Enter\n", __FUNCTION__));

	ASSERT(pkt);
	DHD_PM_RESUME_WAIT(sdioh_request_packet_wait);
	DHD_PM_RESUME_RETURN_ERROR(SDIOH_API_RC_FAIL);

	/* Claim host controller */
	sdio_claim_host(gInstance->func[func]);
	for (pnext = pkt; pnext; pnext = PKTNEXT(sd->osh, pnext)) {
		uint pkt_len = PKTLEN(sd->osh, pnext);
		pkt_len += 3;
		pkt_len &= 0xFFFFFFFC;

#ifdef CONFIG_MMC_MSM7X00A
		if ((pkt_len % 64) == 32) {
			sd_trace(("%s: Rounding up TX packet +=32\n", __FUNCTION__));
			pkt_len += 32;
		}
#endif /* CONFIG_MMC_MSM7X00A */
		/* Make sure the packet is aligned properly. If it isn't, then this
		 * is the fault of sdioh_request_buffer() which is supposed to give
		 * us something we can work with.
		 */
		ASSERT(((uint32)(PKTDATA(sd->osh, pkt)) & DMA_ALIGN_MASK) == 0);

		if ((write) && (!fifo)) {
			err_ret = sdio_memcpy_toio(gInstance->func[func], addr,
				((uint8*)PKTDATA(sd->osh, pnext)),
				pkt_len);
		} else if (write) {
			err_ret = sdio_memcpy_toio(gInstance->func[func], addr,
				((uint8*)PKTDATA(sd->osh, pnext)),
				pkt_len);
		} else if (fifo) {
			err_ret = sdio_readsb(gInstance->func[func],
				((uint8*)PKTDATA(sd->osh, pnext)),
				addr,
				pkt_len);
		} else {
			err_ret = sdio_memcpy_fromio(gInstance->func[func],
				((uint8*)PKTDATA(sd->osh, pnext)),
				addr,
				pkt_len);
		}

		if (err_ret) {
			sd_err(("%s: %s FAILED %p[%d], addr=0x%05x, pkt_len=%d, ERR=0x%08x\n",
				__FUNCTION__,
				(write) ? "TX" : "RX",
				pnext, SGCount, addr, pkt_len, err_ret));
		} else {
			sd_trace(("%s: %s xfr'd %p[%d], addr=0x%05x, len=%d\n",
				__FUNCTION__,
				(write) ? "TX" : "RX",
				pnext, SGCount, addr, pkt_len));
		}

		if (!fifo) {
			addr += pkt_len;
		}
		SGCount ++;

	}

	/* Release host controller */
	sdio_release_host(gInstance->func[func]);

	sd_trace(("%s: Exit\n", __FUNCTION__));
	return ((err_ret == 0) ? SDIOH_API_RC_SUCCESS : SDIOH_API_RC_FAIL);
}


/*
 * This function takes a buffer or packet, and fixes everything up so that in the
 * end, a DMA-able packet is created.
 *
 * A buffer does not have an associated packet pointer, and may or may not be aligned.
 * A packet may consist of a single packet, or a packet chain.  If it is a packet chain,
 * then all the packets in the chain must be properly aligned.  If the packet data is not
 * aligned, then there may only be one packet, and in this case, it is copied to a new
 * aligned packet.
 *
 */
extern SDIOH_API_RC
sdioh_request_buffer(sdioh_info_t *sd, uint pio_dma, uint fix_inc, uint write, uint func,
                     uint addr, uint reg_width, uint buflen_u, uint8 *buffer, void *pkt)
{
	SDIOH_API_RC Status;
	void *mypkt = NULL;

	sd_trace(("%s: Enter\n", __FUNCTION__));

	DHD_PM_RESUME_WAIT(sdioh_request_buffer_wait);
	DHD_PM_RESUME_RETURN_ERROR(SDIOH_API_RC_FAIL);
	/* Case 1: we don't have a packet. */
	if (pkt == NULL) {
		sd_data(("%s: Creating new %s Packet, len=%d\n",
		         __FUNCTION__, write ? "TX" : "RX", buflen_u));
#ifdef DHD_USE_STATIC_BUF
		if (!(mypkt = PKTGET_STATIC(sd->osh, buflen_u, write ? TRUE : FALSE))) {
#else
		if (!(mypkt = PKTGET(sd->osh, buflen_u, write ? TRUE : FALSE))) {
#endif /* DHD_USE_STATIC_BUF */
			sd_err(("%s: PKTGET failed: len %d\n",
			           __FUNCTION__, buflen_u));
			return SDIOH_API_RC_FAIL;
		}

		/* For a write, copy the buffer data into the packet. */
		if (write) {
			bcopy(buffer, PKTDATA(sd->osh, mypkt), buflen_u);
		}

		Status = sdioh_request_packet(sd, fix_inc, write, func, addr, mypkt);

		/* For a read, copy the packet data back to the buffer. */
		if (!write) {
			bcopy(PKTDATA(sd->osh, mypkt), buffer, buflen_u);
		}
#ifdef DHD_USE_STATIC_BUF
		PKTFREE_STATIC(sd->osh, mypkt, write ? TRUE : FALSE);
#else
		PKTFREE(sd->osh, mypkt, write ? TRUE : FALSE);
#endif /* DHD_USE_STATIC_BUF */
	} else if (((uint32)(PKTDATA(sd->osh, pkt)) & DMA_ALIGN_MASK) != 0) {
		/* Case 2: We have a packet, but it is unaligned. */

		/* In this case, we cannot have a chain. */
		ASSERT(PKTNEXT(sd->osh, pkt) == NULL);

		sd_data(("%s: Creating aligned %s Packet, len=%d\n",
		         __FUNCTION__, write ? "TX" : "RX", PKTLEN(sd->osh, pkt)));
#ifdef DHD_USE_STATIC_BUF
		if (!(mypkt = PKTGET_STATIC(sd->osh, PKTLEN(sd->osh, pkt), write ? TRUE : FALSE))) {
#else
		if (!(mypkt = PKTGET(sd->osh, PKTLEN(sd->osh, pkt), write ? TRUE : FALSE))) {
#endif /* DHD_USE_STATIC_BUF */
			sd_err(("%s: PKTGET failed: len %d\n",
			           __FUNCTION__, PKTLEN(sd->osh, pkt)));
			return SDIOH_API_RC_FAIL;
		}

		/* For a write, copy the buffer data into the packet. */
		if (write) {
			bcopy(PKTDATA(sd->osh, pkt),
			      PKTDATA(sd->osh, mypkt),
			      PKTLEN(sd->osh, pkt));
		}

		Status = sdioh_request_packet(sd, fix_inc, write, func, addr, mypkt);

		/* For a read, copy the packet data back to the buffer. */
		if (!write) {
			bcopy(PKTDATA(sd->osh, mypkt),
			      PKTDATA(sd->osh, pkt),
			      PKTLEN(sd->osh, mypkt));
		}
#ifdef DHD_USE_STATIC_BUF
		PKTFREE_STATIC(sd->osh, mypkt, write ? TRUE : FALSE);
#else
		PKTFREE(sd->osh, mypkt, write ? TRUE : FALSE);
#endif /* DHD_USE_STATIC_BUF */
	} else { /* case 3: We have a packet and it is aligned. */
		sd_data(("%s: Aligned %s Packet, direct DMA\n",
		         __FUNCTION__, write ? "Tx" : "Rx"));
		Status = sdioh_request_packet(sd, fix_inc, write, func, addr, pkt);
	}

	return (Status);
}

/* this function performs "abort" for both of host & device */
extern int
sdioh_abort(sdioh_info_t *sd, uint func)
{
#if defined(MMC_SDIO_ABORT)
	char t_func = (char) func;
#endif /* defined(MMC_SDIO_ABORT) */
	sd_trace(("%s: Enter\n", __FUNCTION__));

#if defined(MMC_SDIO_ABORT)
	/* issue abort cmd52 command through F1 */
	sdioh_request_byte(sd, SD_IO_OP_WRITE, SDIO_FUNC_0, SDIOD_CCCR_IOABORT, &t_func);
#endif /* defined(MMC_SDIO_ABORT) */

	sd_trace(("%s: Exit\n", __FUNCTION__));
	return SDIOH_API_RC_SUCCESS;
}

/* Reset and re-initialize the device */
int sdioh_sdio_reset(sdioh_info_t *si)
{
	sd_trace(("%s: Enter\n", __FUNCTION__));
	sd_trace(("%s: Exit\n", __FUNCTION__));
	return SDIOH_API_RC_SUCCESS;
}

/* Disable device interrupt */
void
sdioh_sdmmc_devintr_off(sdioh_info_t *sd)
{
	sd_trace(("%s: %d\n", __FUNCTION__, sd->use_client_ints));
	sd->intmask &= ~CLIENT_INTR;
}

/* Enable device interrupt */
void
sdioh_sdmmc_devintr_on(sdioh_info_t *sd)
{
	sd_trace(("%s: %d\n", __FUNCTION__, sd->use_client_ints));
	sd->intmask |= CLIENT_INTR;
}

/* Read client card reg */
int
sdioh_sdmmc_card_regread(sdioh_info_t *sd, int func, uint32 regaddr, int regsize, uint32 *data)
{

	if ((func == 0) || (regsize == 1)) {
		uint8 temp = 0;

		sdioh_request_byte(sd, SDIOH_READ, func, regaddr, &temp);
		*data = temp;
		*data &= 0xff;
		sd_data(("%s: byte read data=0x%02x\n",
		         __FUNCTION__, *data));
	} else {
		sdioh_request_word(sd, 0, SDIOH_READ, func, regaddr, data, regsize);
		if (regsize == 2)
			*data &= 0xffff;

		sd_data(("%s: word read data=0x%08x\n",
		         __FUNCTION__, *data));
	}

	return SUCCESS;
}

#if !defined(OOB_INTR_ONLY)
/* bcmsdh_sdmmc interrupt handler */
static void IRQHandler(struct sdio_func *func)
{
	sdioh_info_t *sd;

	sd_trace(("bcmsdh_sdmmc: ***IRQHandler\n"));
	sd = gInstance->sd;

	ASSERT(sd != NULL);
	sdio_release_host(gInstance->func[0]);

	if (sd->use_client_ints) {
		sd->intrcount++;
		ASSERT(sd->intr_handler);
		ASSERT(sd->intr_handler_arg);
		(sd->intr_handler)(sd->intr_handler_arg);
	} else {
		sd_err(("bcmsdh_sdmmc: ***IRQHandler\n"));

		sd_err(("%s: Not ready for intr: enabled %d, handler %p\n",
		        __FUNCTION__, sd->client_intr_enabled, sd->intr_handler));
	}

	sdio_claim_host(gInstance->func[0]);
}

/* bcmsdh_sdmmc interrupt handler for F2 (dummy handler) */
static void IRQHandlerF2(struct sdio_func *func)
{
	sdioh_info_t *sd;

	sd_trace(("bcmsdh_sdmmc: ***IRQHandlerF2\n"));

	sd = gInstance->sd;

	ASSERT(sd != NULL);
}
#endif /* !defined(OOB_INTR_ONLY) */

#ifdef NOTUSED
/* Write client card reg */
static int
sdioh_sdmmc_card_regwrite(sdioh_info_t *sd, int func, uint32 regaddr, int regsize, uint32 data)
{

	if ((func == 0) || (regsize == 1)) {
		uint8 temp;

		temp = data & 0xff;
		sdioh_request_byte(sd, SDIOH_READ, func, regaddr, &temp);
		sd_data(("%s: byte write data=0x%02x\n",
		         __FUNCTION__, data));
	} else {
		if (regsize == 2)
			data &= 0xffff;

		sdioh_request_word(sd, 0, SDIOH_READ, func, regaddr, &data, regsize);

		sd_data(("%s: word write data=0x%08x\n",
		         __FUNCTION__, data));
	}

	return SUCCESS;
}
#endif /* NOTUSED */

int
sdioh_start(sdioh_info_t *si, int stage)
{
	int ret;
	sdioh_info_t *sd = gInstance->sd;

	/* Need to do this stages as we can't enable the interrupt till
		downloading of the firmware is complete, other wise polling
		sdio access will come in way
	*/
	if (gInstance->func[0]) {
			if (stage == 0) {
		/* Since the power to the chip is killed, we will have
			re enumerate the device again. Set the block size
			and enable the fucntion 1 for in preparation for
			downloading the code
		*/
		/* sdio_reset_comm() - has been fixed in latest kernel/msm.git for Linux
		   2.6.27. The implementation prior to that is buggy, and needs broadcom's
		   patch for it
		*/
		if ((ret = sdio_reset_comm(gInstance->func[0]->card)))
			sd_err(("%s Failed, error = %d\n", __FUNCTION__, ret));
		else {
			sd->num_funcs = 2;
			sd->sd_blockmode = TRUE;
			sd->use_client_ints = TRUE;
			sd->client_block_size[0] = 64;

			/* Claim host controller */
			sdio_claim_host(gInstance->func[1]);

			sd->client_block_size[1] = 64;
			if (sdio_set_block_size(gInstance->func[1], 64)) {
				sd_err(("bcmsdh_sdmmc: Failed to set F1 blocksize\n"));
			}

			/* Release host controller F1 */
			sdio_release_host(gInstance->func[1]);

			if (gInstance->func[2]) {
				/* Claim host controller F2 */
				sdio_claim_host(gInstance->func[2]);

				sd->client_block_size[2] = sd_f2_blocksize;
				if (sdio_set_block_size(gInstance->func[2],
					sd_f2_blocksize)) {
					sd_err(("bcmsdh_sdmmc: Failed to set F2 "
						"blocksize to %d\n", sd_f2_blocksize));
				}

				/* Release host controller F2 */
				sdio_release_host(gInstance->func[2]);
			}

			sdioh_sdmmc_card_enablefuncs(sd);
			}
		} else {
#if !defined(OOB_INTR_ONLY)
			sdio_claim_host(gInstance->func[0]);
			sdio_claim_irq(gInstance->func[2], IRQHandlerF2);
			sdio_claim_irq(gInstance->func[1], IRQHandler);
			sdio_release_host(gInstance->func[0]);
#else /* defined(OOB_INTR_ONLY) */
#if defined(HW_OOB)
			sdioh_enable_func_intr();
#endif
			bcmsdh_oob_intr_set(TRUE);
#endif /* !defined(OOB_INTR_ONLY) */
		}
	}
	else
		sd_err(("%s Failed\n", __FUNCTION__));

	return (0);
}

int
sdioh_stop(sdioh_info_t *si)
{
	/* MSM7201A Android sdio stack has bug with interrupt
		So internaly within SDIO stack they are polling
		which cause issue when device is turned off. So
		unregister interrupt with SDIO stack to stop the
		polling
	*/
	if (gInstance->func[0]) {
#if !defined(OOB_INTR_ONLY)
		sdio_claim_host(gInstance->func[0]);
		sdio_release_irq(gInstance->func[1]);
		sdio_release_irq(gInstance->func[2]);
		sdio_release_host(gInstance->func[0]);
#else /* defined(OOB_INTR_ONLY) */
#if defined(HW_OOB)
		sdioh_disable_func_intr();
#endif
		bcmsdh_oob_intr_set(FALSE);
#endif /* !defined(OOB_INTR_ONLY) */
	}
	else
		sd_err(("%s Failed\n", __FUNCTION__));
	return (0);
}
