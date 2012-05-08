/*
 * Definitions for BMA250 G-sensor chip.
 */
#ifndef BMA250_H
#define BMA250_H

#include <linux/ioctl.h>

#define BMA250_I2C_NAME "bma250"
#define BMA250_I2C_NAME_REMOVE_ECOMPASS "bma250_no_ecmps"

#define BMAIO			0xA1

/* BMA250 register address */
#define bma250_CHIP_ID_REG	0x00
#define bma250_X_AXIS_LSB_REG	0x2
#define bma250_RANGE_SEL_REG	0x0F
#define bma250_BW_SEL_REG	0x10
#define bma250_MODE_CTRL_REG	0x11

/* IOCTLs*/
#define BMA_IOCTL_INIT			_IO(BMAIO, 0x31)
#define BMA_IOCTL_WRITE			_IOW(BMAIO, 0x32, char[5])
#define BMA_IOCTL_READ			_IOWR(BMAIO, 0x33, char[5])
#define BMA_IOCTL_READ_ACCELERATION	_IOWR(BMAIO, 0x34, short[7])
#define BMA_IOCTL_SET_MODE		_IOW(BMAIO, 0x35, short)
#define BMA_IOCTL_GET_INT		_IOR(BMAIO, 0x36, short)
#define BMA_IOCTL_GET_CHIP_LAYOUT	_IOR(BMAIO, 0x37, short)
#define BMA_IOCTL_GET_CALI_MODE		_IOR(BMAIO, 0x38, short)
#define BMA_IOCTL_SET_CALI_MODE		_IOW(BMAIO, 0x39, short)
#define BMA_IOCTL_READ_CALI_VALUE       _IOR(BMAIO, 0x3a, char[3])
#define BMA_IOCTL_WRITE_CALI_VALUE      _IOW(BMAIO, 0x3b, int)
#define BMA_IOCTL_GET_UPDATE_USER_CALI_DATA    _IOR(BMAIO, 0x3c, short)
#define BMA_IOCTL_SET_UPDATE_USER_CALI_DATA    _IOW(BMAIO, 0x3d, short)

/* range and bandwidth */
#define BMA_RANGE_2G		0x3
#define BMA_RANGE_4G		0x5
#define BMA_RANGE_8G		0x8
#define BMA_RANGE_16G		0xC

#define BMA_BW_7_81HZ		0x8
#define BMA_BW_15_63HZ		0x9
#define BMA_BW_31_25HZ		0xA
#define BMA_BW_62_5HZ		0xB
#define BMA_BW_125HZ		0xC
#define BMA_BW_250HZ		0xD
#define BMA_BW_500HZ		0xE
#define BMA_BW_1000HZ		0xF

/* BMA250 API error codes */
#define E_OUT_OF_RANGE          (char)(-2)

/* mode settings */
#define bma250_MODE_NORMAL      0
#define bma250_MODE_SUSPEND     1

extern unsigned int gs_kvalue;

struct bma250_platform_data {
	int intr;
	int chip_layout;
	int calibration_mode;
	int gs_kvalue;
	unsigned int (*G_Sensor_Compass_POR)(void);
	short layouts[4][3][3];
};

#endif
