
#bcm4330

#DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2         \
        -DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
        -DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE -DDHD_BCMEVENTS  \
        -DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS      \
        -Wall -Wstrict-prototypes -Werror  -DSDIO_ISR_THREAD 		      \
        -DEMBEDDED_PLATFORM -DARP_OFFLOAD_SUPPORT -DPKT_FILTER_SUPPORT        \
        -DKEEP_ALIVE -DCONFIG_FIRST_SCAN -DAP_ONLY -DCUSTOM_OOB_GPIO_NUM=299  \
        -DOOB_INTR_ONLY -DMMC_SDIO_ABORT                                      \
        -I/home/takara/work/HTC/bcm4330b1 -I/home/takara/work/HTC/bcm4330b1/include
        #-DPNO_SUPPORT -DCSCAN -DSET_RANDOM_MAC_SOFTAP -DGET_CUSTOM_MAC_ENABLE 


DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32  \
	-DBCMFILEIMAGE -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64            \
	-DDHD_FIRSTREAD=64 -DDHDTHREAD -DDHD_GPL -DDHD_SCHED -DBDC           \
	-DTOE -DDHD_BCMEVENTS -DSHOW_EVENTS -DDONGLEOVERLAYS -DOEM_ANDROID   \
	-DBCMDBG -DDHD_USE_STATIC_BUF -DCONFIG_FIRST_SCAN -DAP_ONLY \
	-DCUSTOM_OOB_GPIO_NUM=46 -DOOB_INTR_ONLY -DMMC_SDIO_ABORT -DEMBEDDED_PLATFORM -DCUSTOMER_HW2 -DDHD_PRINT_DEBUG   \
	-DPNO_SUPPORT -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS -DWIFI_ACT_FRAME -DKEEP_ALIVE \
	-DCSCAN -DBCM4329_LOW_POWER  -DBCMWAPI_WAI -DBCMWAPI_WPI -DHTC_KlocWork -DWL_CFG80211 -DWLP2P -DWL_COMPAT_WIRELESS\
	-Wall -Wstrict-prototypes -Werror                                    \
	-Idrivers/net/wireless/bcmdhd -Idrivers/net/wireless/bcmdhd/include
	#-I$(M) -I$(M)/include
	# CFG 80211 and P2P compile flag
	#-DWL_CFG80211 -DWLP2P -DWL_COMPAT_WIRELESS \

#ifdef P2P
DHDCFLAGS += -DWL_CFG80211 -DWLP2P -DWL_COMPAT_WIRELESS 
#endif

ifdef COMPAT_WIRELESS
NOSTDINC_FLAGS := -I$(COMPAT_WIRELESS)/include/ \
        -include $(COMPAT_WIRELESS)/include/linux/compat-2.6.h
endif

DHDOFILES = dhd_linux.o linux_osl.o bcmutils.o dhd_common.o dhd_custom_gpio.o \
        siutils.o sbutils.o aiutils.o hndpmu.o bcmwifi.o dhd_sdio.o   \
        dhd_linux_sched.o dhd_cdc.o bcmsdh_sdmmc.o bcmsdh.o bcmsdh_linux.o    \
        bcmsdh_sdmmc_linux.o bcmevent.o dhd_bta.o wl_android.o wldev_common.o wl_linux_mon.o wl_cfg80211.o wl_cfgp2p.o dhd_cfg80211.o

#ifdef P2P
DHDOFILES += wl_linux_mon.o wl_cfg80211.o wl_cfgp2p.o
#endif
	




obj-$(CONFIG_BCMDHD) += bcmdhd.o
bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug
