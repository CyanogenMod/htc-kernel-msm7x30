#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_SENSORS_AKM8975)   += akm8975.o
obj-$(CONFIG_SENSORS_AKM8975_PANA_GYRO)   += akm8975_pana_gyro.o
obj-$(CONFIG_SENSORS_PANASONIC_GYRO)	+= ewtzmu2.o
obj-$(CONFIG_SENSORS_BMA250)	+= bma250.o
obj-$(CONFIG_SENSORS_ONLY_BMA250)      += bma250_without_Ecompass.o
obj-$(CONFIG_SENSORS_I2C_AK_BMA250)	+= akm_bma250.o
obj-$(CONFIG_SENSORS_NFC_PN544) += pn544.o
obj-$(CONFIG_INPUT_CAPELLA_CM3629)	+= cm3629.o
obj-$(CONFIG_INPUT_CAPELLA_CM3628)	+= cm3628.o
obj-$(CONFIG_VP_A1026)		+= a1026.o
obj-$(CONFIG_VP_A1028)		+= a1028.o
obj-$(CONFIG_AMP_TPA2051D3)		+= tpa2051d3.o
obj-$(CONFIG_INPUT_ISL29028)	+= isl29028.o
obj-$(CONFIG_INPUT_ISL29029)	+= isl29029.o
obj-$(CONFIG_TPS65200)          += tps65200.o
obj-$(CONFIG_CODEC_AIC3254)      += aic3254.o
obj-$(CONFIG_CODEC_AIC3008)      += aic3008.o
ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

