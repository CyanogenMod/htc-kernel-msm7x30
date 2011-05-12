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

obj-$(CONFIG_SENSORS_AKM8973)	+= akm8973.o
obj-$(CONFIG_SENSORS_AKM8975)   += akm8975.o
obj-$(CONFIG_SENSORS_AKM8976)	+= akm8976.o
obj-$(CONFIG_SENSORS_BMA150)    += bma150.o
obj-$(CONFIG_SENSORS_BMA250)	+= bma250.o

obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_SENSORS_PCA963X)	+= pca963x.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
#obj-$(CONFIG_SENSORS_MT9T013)	+= mt9t013.o
obj-$(CONFIG_VP_A1026)		+= a1026.o
obj-$(CONFIG_AMP_TPA2051D3)	+= tpa2051d3.o
obj-$(CONFIG_AMP_TPA2026)	+= tpa2026.o
obj-$(CONFIG_AMP_TPA2028)	+= tpa2028.o
obj-$(CONFIG_AMP_TPA6130A)	+= tpa6130.o
obj-$(CONFIG_AMP_TPA2018D1)	+= tpa2018d1.o
obj-$(CONFIG_ARCH_MSM_FLASHLIGHT)	+= adp1650_flashlight.o
obj-$(CONFIG_INPUT_ISL29028)	+= isl29028.o
obj-$(CONFIG_INPUT_ISL29029)	+= isl29029.o
obj-$(CONFIG_SMB329)            += smb329.o
obj-$(CONFIG_TPS65200)          += tps65200.o
obj-$(CONFIG_INPUT_CAPELLA_CM3628)          += cm3628.o
obj-$(CONFIG_SENSORS_CAPSENSE)	+= cap_sense.o
obj-$(CONFIG_SENSORS_CAPSENSE_EXPRESS)	+= cap_sense_express.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

