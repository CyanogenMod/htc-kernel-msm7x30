#bcm4330

#DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2         \
        -DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
        -DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE -DDHD_BCMEVENTS  \
        -DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS      \
        -Wall -Wstrict-prototypes -Werror  -DSDIO_ISR_THREAD 		      \
        -DEMBEDDED_PLATFORM -DARP_OFFLOAD_SUPPORT -DPKT_FILTER_SUPPORT        \
        -DKEEP_ALIVE -DCONFIG_FIRST_SCAN -DAP_ONLY -DCUSTOM_OOB_GPIO_NUM=299  \
        -DOOB_INTR_ONLY -DMMC_SDIO_ABORT                       \
        -I/home/takara/work/HTC/bcm4330b1 -I/home/takara/work/HTC/bcm4330b1/include
        #-DPNO_SUPPORT -DCSCAN -DSET_RANDOM_MAC_SOFTAP -DGET_CUSTOM_MAC_ENABLE 


DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32  \
	-DBCMFILEIMAGE -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64            \
	-DDHD_FIRSTREAD=64 -DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DBDC           \
	-DTOE -DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DOEM_ANDROID   \
	-DBCMDBG -DDHD_USE_STATIC_BUF -DSOFTAP -DCONFIG_FIRST_SCAN -DAP_ONLY      \
	-DCUSTOM_OOB_GPIO_NUM=46 -DOOB_INTR_ONLY -DMMC_SDIO_ABORT -DEMBEDDED_PLATFORM -DCUSTOMER_HW2 -DDHD_PRINT_DEBUG   \
	-DPNO_SUPPORT -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWIFI_ACT_FRAME -DKEEP_ALIVE \
	-DCSCAN -DBCM4329_LOW_POWER -DBCMWAPI_WPI -DBCMWAPI_WAI -DHTC_KlocWork \
	-Wall -Wstrict-prototypes -Werror                                    \
	-Idrivers/net/wireless/bcm4330b2 -Idrivers/net/wireless/bcm4330b2/include
#	-I$(M) -I$(M)/include
#	-I/home/takara/work/HTC/cmp/diff/bcm4330b1 -I/home/takara/work/HTC/cmp/diff/bcm4330b1/include



DHDOFILES = dhd_linux.o linux_osl.o bcmutils.o dhd_common.o dhd_custom_gpio.o \
        wl_iw.o siutils.o sbutils.o aiutils.o hndpmu.o bcmwifi.o dhd_sdio.o   \
        dhd_linux_sched.o dhd_cdc.o bcmsdh_sdmmc.o bcmsdh.o bcmsdh_linux.o    \
        bcmsdh_sdmmc_linux.o bcmevent.o dhd_bta.o



#obj-m += bcm4329.o
#bcm4329-objs += $(DHDOFILES)
#obj-m += bcm4330.o
obj-$(CONFIG_BCM4330B2) += bcm4330.o
bcm4330-objs += $(DHDOFILES)
EXTRA_CFLAGS = $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug
