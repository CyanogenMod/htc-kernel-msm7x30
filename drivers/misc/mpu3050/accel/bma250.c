/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

/**
 *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   bma250.c
 *      @brief  Accelerometer setup and handling methods.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlos.h"
#include "mlsl.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-acc"

/* full scale setting - register and mask */
#define BOSCH_CTRL_REG      (0x0F)
#define BOSCH_INT_REG       (0x16)
#define BOSCH_PWR_REG       (0x11)
#define BMA250_REG_SOFT_RESET (0x14)
#define BMA250_BW_REG        (0x10)    /* BMA250 : BW setting register */

#define ACCEL_BOSCH_CTRL_MASK              (0x0F)
#define ACCEL_BOSCH_CTRL_MASK_FSR          (0xF8)
#define ACCEL_BOSCH_INT_MASK_WUP           (0xF8)
#define ACCEL_BOSCH_INT_MASK_IRQ           (0xDF)
#define BMA250_BW_MASK      (0xE0)    /* BMA250 : BW setting mask */

#define D(x...) printk(KERN_DEBUG "[GSNR][BMA250] " x)
#define I(x...) printk(KERN_INFO "[GSNR][BMA250] " x)
#define E(x...) printk(KERN_ERR "[GSNR][BMA250 ERROR] " x)
#define DIF(x...) \
	if (debug_flag) \
		printk(KERN_DEBUG "[GSNR][BMA250 DEBUG] " x)

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

struct bma250_config {
	unsigned int odr; /* Output data rate mHz */
	unsigned int fsr; /* full scale range mg */
	unsigned int irq_type;
	unsigned int power_mode;
	unsigned char ctrl_reg; /* range */
	unsigned char bw_reg; /* bandwidth */
	unsigned char int_reg;
};

struct bma250_private_data {
	struct bma250_config suspend;
	struct bma250_config resume;
	unsigned char state;
};


/*********************************************
    Accelerometer Initialization Functions
**********************************************/

/**
 * Sets the IRQ to fire when one of the IRQ events occur.  Threshold and
 * duration will not be used uless the type is MOT or NMOT.
 *
 * @param config configuration to apply to, suspend or resume
 * @param irq_type The type of IRQ.  Valid values are
 * - MPU_SLAVE_IRQ_TYPE_NONE
 * - MPU_SLAVE_IRQ_TYPE_MOTION
 * - MPU_SLAVE_IRQ_TYPE_DATA_READY
 *
 */
static int bma250_set_irq(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct bma250_config *config,
			int apply,
			long irq_type)
{
	unsigned char irq_bits = 0;
	int result = ML_SUCCESS;

	/* TODO Use irq when necessary */
	return ML_SUCCESS;

	if (irq_type == MPU_SLAVE_IRQ_TYPE_MOTION)
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;

	config->irq_type = (unsigned char)irq_type;

	if (irq_type == MPU_SLAVE_IRQ_TYPE_DATA_READY) {
		irq_bits = 0x20;
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
	} else {
		irq_bits = 0x00;
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
	}

	config->int_reg &= ACCEL_BOSCH_INT_MASK_IRQ;
	config->int_reg |= irq_bits;

	if (apply) {

		if (!config->power_mode) {
			/* BMA250: Software reset */
			result = MLSLSerialWriteSingle(mlsl_handle,
					pdata->address, BMA250_REG_SOFT_RESET,
					0xB6);
			ERROR_CHECK(result);
			MLOSSleep(1);
		}

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_CTRL_REG, config->ctrl_reg);
		ERROR_CHECK(result);

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_INT_REG, config->int_reg);
		ERROR_CHECK(result);

		if (!config->power_mode) {
			result = MLSLSerialWriteSingle(mlsl_handle,
				pdata->address, BOSCH_PWR_REG, 0x80);
			ERROR_CHECK(result);
			MLOSSleep(1);
		}
	}
	return result;
}

/**
 * Set the Output data rate for the particular configuration
 *
 * @param config Config to modify with new ODR
 * @param odr Output data rate in units of 1/1000Hz
 */
static int bma250_set_odr(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct bma250_config *config,
			int apply,
			long odr)
{
	unsigned char odr_bits = 0;
	unsigned char wup_bits = 0;
	int result = ML_SUCCESS;

	/* TO DO use dynamic bandwidth when stability safe */
	/*if (odr > 100000) {
		config->odr = 125000;
		odr_bits = 0x0C;
		config->power_mode = 1;
	} else if (odr > 50000) {
		config->odr = 62500;
		odr_bits = 0x0B;
		config->power_mode = 1;
	} else if (odr > 20000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 15000) {
		config->odr = 15630;
		odr_bits = 0x09;
		config->power_mode = 1;
	} else if (odr > 0) {
		config->odr = 7810;
		odr_bits = 0x08;
		config->power_mode = 1;
	} else {
		config->odr = 0;
		wup_bits = 0x00;
		config->power_mode = 0;
	}*/
	if (odr > 100000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 50000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 20000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 15000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 0) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else {
		config->odr = 0;
		wup_bits = 0x00;
		config->power_mode = 0;
	}

	switch (config->power_mode) {
	case 1:
		config->bw_reg &= BMA250_BW_MASK;
		config->bw_reg |= odr_bits;
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
		break;
	case 0:
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
		config->int_reg |= wup_bits;
		break;
	default:
		break;
	}

	MPL_LOGV("ODR: %d \n", config->odr);
	if (apply) {
			/* BMA250: Software reset */
			result = MLSLSerialWriteSingle(mlsl_handle,
					pdata->address, BMA250_REG_SOFT_RESET,
					0xB6);
			ERROR_CHECK(result);
			MLOSSleep(1);

			result = MLSLSerialWriteSingle(mlsl_handle,
					pdata->address, BMA250_BW_REG,
					config->bw_reg);
			ERROR_CHECK(result);

			/* TODO Use irq when necessary */
			/*
			result = MLSLSerialWriteSingle(mlsl_handle,
					pdata->address,	BOSCH_INT_REG,
					config->int_reg);
			ERROR_CHECK(result);*/

			if (!config->power_mode) {
				result = MLSLSerialWriteSingle(mlsl_handle,
						pdata->address,	BOSCH_PWR_REG,
						0x80);
				ERROR_CHECK(result);
				MLOSSleep(1);
			}
	}

	return result;
}

/**
 * Set the full scale range of the accels
 *
 * @param config pointer to configuration
 * @param fsr requested full scale range
 */
static int bma250_set_fsr(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct bma250_config *config,
			int apply,
			long fsr)
{
	unsigned char fsr_bits;
	int result = ML_SUCCESS;

	/* TO DO use dynamic range when stability safe */
	/*if (fsr <= 2048) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 4096) {
		fsr_bits = 0x05;
		config->fsr = 4096;
	} else if (fsr <= 8192) {
		fsr_bits = 0x08;
		config->fsr = 8192;
	} else if (fsr <= 16384) {
		fsr_bits = 0x0C;
		config->fsr = 16384;
	} else {
		fsr_bits = 0x03;
		config->fsr = 2048;
	}*/
	if (fsr <= 2048) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 4096) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 8192) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 16384) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else {
		fsr_bits = 0x03;
		config->fsr = 2048;
	}

	config->ctrl_reg &= ACCEL_BOSCH_CTRL_MASK_FSR;
	config->ctrl_reg |= fsr_bits;

	MPL_LOGV("FSR: %d \n", config->fsr);
	if (apply) {

		if (!config->power_mode) {
			/* BMA250: Software reset */
			result = MLSLSerialWriteSingle(mlsl_handle,
				pdata->address, BMA250_REG_SOFT_RESET, 0xB6);
			ERROR_CHECK(result);
			MLOSSleep(1);
		}

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_CTRL_REG, config->ctrl_reg);
		ERROR_CHECK(result);

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_CTRL_REG, config->ctrl_reg);
		ERROR_CHECK(result);

		if (!config->power_mode) {
			result = MLSLSerialWriteSingle(mlsl_handle,
				pdata->address,	BOSCH_PWR_REG, 0x80);
			ERROR_CHECK(result);
			MLOSSleep(1);
		}
	}
	return result;
}

static int bma250_suspend(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{

	int result = 0;
	unsigned char ctrl_reg;
	unsigned char int_reg;

	struct bma250_private_data *private_data = pdata->private_data;
	ctrl_reg = private_data->suspend.ctrl_reg;
	int_reg = private_data->suspend.int_reg;

	private_data->state = 1;

	/* TO DO sync from bma150 of MPL3.3.0, comment follows */
	/* BMA250: Software reset */
	/*result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_REG_SOFT_RESET, 0xB6);
	ERROR_CHECK(result);
	MLOSSleep(1);

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_CTRL_REG, ctrl_reg);
	ERROR_CHECK(result);*/

	/* TODO Use irq when necessary */
	/*result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_INT_REG, int_reg);
	ERROR_CHECK(result);*/

	if (!private_data->suspend.power_mode) {
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_PWR_REG, 0x80);
		ERROR_CHECK(result);
	}

	return result;
}


static int bma250_resume(void *mlsl_handle,
			 struct ext_slave_descr *slave,
			 struct ext_slave_platform_data *pdata)
{

	int result;
	unsigned char ctrl_reg;
	unsigned char bw_reg;
	unsigned char int_reg;

	struct bma250_private_data *private_data = pdata->private_data;
	ctrl_reg = private_data->resume.ctrl_reg;
	bw_reg = private_data->resume.bw_reg;
	int_reg = private_data->resume.int_reg;

	private_data->state = 0;

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_REG_SOFT_RESET, 0xB6);  /* BMA250: Software reset */
	ERROR_CHECK(result);
	MLOSSleep(1);

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_CTRL_REG, ctrl_reg);
	ERROR_CHECK(result);

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_BW_REG, bw_reg);
	ERROR_CHECK(result);

	/* TODO Use irq when necessary */
	/*result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_INT_REG, int_reg);
	ERROR_CHECK(result);*/

	if (!private_data->resume.power_mode) {
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_PWR_REG, 0x80);
		ERROR_CHECK(result);
	}

	return result;
}

static int bma250_read(void *mlsl_handle,
		       struct ext_slave_descr *slave,
		       struct ext_slave_platform_data *pdata,
		       unsigned char *data)
{
	int result;

	result = MLSLSerialRead(mlsl_handle, pdata->address,
		slave->reg, slave->len, data);

	return result;
}

static int bma250_init(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
	tMLError result;
	unsigned char reg = 0;
	unsigned char bw_reg = 0;

	struct bma250_private_data *private_data;
	private_data = (struct bma250_private_data *)
		MLOSMalloc(sizeof(struct bma250_private_data));

	if (!private_data)
		return ML_ERROR_MEMORY_EXAUSTED;



	pdata->private_data = private_data;

	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_REG_SOFT_RESET, 0xB6);  /* BMA250: Software reset */
	ERROR_CHECK(result);
	MLOSSleep(1);

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, BOSCH_CTRL_REG, 1,
				&reg);
	ERROR_CHECK(result);

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, BMA250_BW_REG, 1,
				&bw_reg);
	ERROR_CHECK(result);

	private_data->resume.ctrl_reg = reg;
	private_data->suspend.ctrl_reg = reg;

	private_data->resume.bw_reg = bw_reg;
	private_data->suspend.bw_reg = bw_reg;

	/* TODO Use irq when necessary */
	/*result =
	    MLSLSerialRead(mlsl_handle, pdata->address, BOSCH_INT_REG, 1, &reg);
	ERROR_CHECK(result);*/

	private_data->resume.int_reg = reg;
	private_data->suspend.int_reg = reg;

	private_data->resume.power_mode = 1;
	private_data->suspend.power_mode = 0;

	private_data->state = 0;

	bma250_set_odr(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 0);
	bma250_set_odr(mlsl_handle, pdata, &private_data->resume,
			TRUE, 25000);
	bma250_set_fsr(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 2048);
	bma250_set_fsr(mlsl_handle, pdata, &private_data->resume,
			FALSE, 2048);

	/* TODO Use irq when necessary */
	/*bma250_set_irq(mlsl_handle, pdata, &private_data->suspend,
			FALSE,
			MPU_SLAVE_IRQ_TYPE_NONE);
	bma250_set_irq(mlsl_handle, pdata, &private_data->resume,
			FALSE,
			MPU_SLAVE_IRQ_TYPE_NONE);*/

	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address, BOSCH_PWR_REG,
					0x80);
	ERROR_CHECK(result);

	return result;
}

static int bma250_exit(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
	if (pdata->private_data)
		return MLOSFree(pdata->private_data);
	else
		return ML_SUCCESS;
}

static int bma250_config(void *mlsl_handle,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config *data)
{
	struct bma250_private_data *private_data = pdata->private_data;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		return bma250_set_odr(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		return bma250_set_odr(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		return bma250_set_fsr(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		return bma250_set_fsr(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		return bma250_set_irq(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		return bma250_set_irq(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};
	return ML_SUCCESS;
}

static int bma250_get_config(void *mlsl_handle,
				struct ext_slave_descr *slave,
				struct ext_slave_platform_data *pdata,
				struct ext_slave_config *data)
{
	struct bma250_private_data *private_data = pdata->private_data;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.odr;
		break;
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.odr;
		break;
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.fsr;
		break;
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.fsr;
		break;
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.irq_type;
		break;
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.irq_type;
		break;
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return ML_SUCCESS;
}

static struct ext_slave_descr bma250_descr = {
	/*.init             = */ bma250_init,
	/*.exit             = */ bma250_exit,
	/*.suspend          = */ bma250_suspend,
	/*.resume           = */ bma250_resume,
	/*.read             = */ bma250_read,
	/*.config           = */ bma250_config,
	/*.get_config       = */ bma250_get_config,
	/*.name             = */ "bma250",
	/*.type             = */ EXT_SLAVE_TYPE_ACCELEROMETER,
	/*.id               = */ ACCEL_ID_BMA250,
	/*.reg              = */ 0x02,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
	/*.range            = */ {2, 0},
};

struct ext_slave_descr *bma250_get_slave_descr(void)
{
	return &bma250_descr;
}
EXPORT_SYMBOL(bma250_get_slave_descr);

#ifdef __KERNEL__
MODULE_AUTHOR("Invensense");
MODULE_DESCRIPTION("User space IRQ handler for MPU3xxx devices");
MODULE_LICENSE("GPL");
MODULE_ALIAS("bma");
#endif

/**
 *  @}
 */
