#
# This is part of the Sequans SQN1130 driver.
# Copyright 2008 SEQUANS Communications
# Written by Andy Shevchenko <andy@smile.org.ua>,
#            Dmitriy Chumak <chumakd@gmail.com>
#

ifeq ($(KERNELRELEASE),)

# The following two variables are used to build SDIO
# driver for foreign architecture, for example ARM,
# change them to meet your build environment
SQN_KERNEL_HEADERS_DIR	:= /usr/src/linux-headers-$(shell uname -r)
SQN_CROSS_COMPILER	:= arm-eabi-


help:
	@echo -e "Available targets are: \n\n\
	\thelp		-- print this help message\n\n\
	\tsdio_arm_release -- build release SDIO driver for ARM architecture\n\
	\tsdio_arm_debug	-- build debug SDIO driver for ARM architecture\n\
	\tsdio		-- build SDIO driver for x86 architecture\n\
	\tusb		-- build USB driver\n\n\
	\tusb_debian_pkg -- make debian package for USB driver\n\
	\tusb_ubuntu_pkg -- make ubuntu package for USB driver\n\n\
	\tclean		-- clean object files and binaries"

sdio_arm_release:
	$(MAKE) -C $(SQN_KERNEL_HEADERS_DIR)		\
		ARCH=arm				\
		CROSS_COMPILE=$(SQN_CROSS_COMPILER)	\
		M=$(shell pwd)				\
		CONFIG_USB_SQN=''			\
		CONFIG_SDIO_SQN=m			\
		SQN_TARGET=CONFIG_SDIO_SQN		\

sdio_arm_debug:
	$(MAKE) -C $(SQN_KERNEL_HEADERS_DIR)		\
		ARCH=arm				\
		CROSS_COMPILE=$(SQN_CROSS_COMPILER)	\
		M=$(shell pwd)				\
		CONFIG_USB_SQN=''			\
		CONFIG_SDIO_SQN=m			\
		SQN_TARGET=CONFIG_SDIO_SQN		\
		EXTRA_CFLAGS='-DCONFIG_SDIO_SQN -DDEBUG -Wno-unused-function		\
		   -DSQN_DEBUG_TRACE -DSQN_DEBUG_DUMP -DSQN_DEBUG_LEVEL_INFO'

sdio:
	$(MAKE) -C $(SQN_KERNEL_HEADERS_DIR)		\
		M=$(shell pwd)				\
		CONFIG_USB_SQN=''			\
		CONFIG_SDIO_SQN=m			\
		SQN_TARGET=CONFIG_SDIO_SQN		\

usb:
	$(MAKE) -C $(SQN_KERNEL_HEADERS_DIR)		\
		M=$(shell pwd)				\
		CONFIG_USB_SQN=m			\
		CONFIG_SDIO_SQN=''			\
		SQN_TARGET=CONFIG_USB_SQN		\

usb_debian_pkg:
	@./build_deb.sh debian

usb_ubuntu_pkg:
	@./build_deb.sh ubuntu

clean:
	rm -vrf *.o *.ko .*.cmd *.mod.c .*.d modules.order Module.* .tmp_versions *~

else # KERNELRELEASE

SQN_TARGET	:= SQN_SDIO
SQN_SDIO	:= m
SQN_USB		:=

obj-$(SQN_SDIO)	+= sequans_sdio.o
obj-$(SQN_USB)	+= sequans_usb.o

sequans_sdio-objs := sdio.o sdio-driver.o sdio-fw.o sdio-pm.o thp.o sdio_netlink.o msg.o

sequans_usb-objs  := usb-driver.o thp.o


### Compile debug version of the module
#EXTRA_CFLAGS	+= -D$(SQN_TARGET) -DDEBUG -Wno-unused-function		\
		   -DSQN_DEBUG_TRACE -DSQN_DEBUG_DUMP -DSQN_DEBUG_LEVEL_INFO

#EXTRA_CFLAGS	+= -D$(SQN_TARGET) -DDEBUG -Wno-unused-function		\
		   -DSQN_DEBUG_TRACE -DSQN_DEBUG_LEVEL_INFO

#EXTRA_CFLAGS	+= -D$(SQN_TARGET) -DDEBUG -Wno-unused-function


### Compile release version of the module
EXTRA_CFLAGS	+= -D$(SQN_TARGET) -Wno-unused-function -Wno-unused-label	\
		   -Wno-unused-variable

EXTRA_CFLAGS	+= -DANDROID_KERNEL

endif
