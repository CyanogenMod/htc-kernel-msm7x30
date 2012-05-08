#Whether to build debug version
BUILD_DEBUG_VERSION := 1

#Enable this flag to build driver in diag version
BUILD_DIAG_VERSION := 0
#Enable this flag to build ftm driver
BUILD_FTM_DRIVER := 0

#KERNEL include dir
KERNEL_INC ?= ./include/linux

#WLAN_DIR 		:= $(WLAN_PRIMA)/CORE
WLAN_DIR			:= drivers/net/wireless/qualcomm_wlan/prima/CORE

############## HDD ###################
HDD_INC := $(WLAN_DIR)/HDD/inc/
HDD_OBJS := wlan_hdd_main.o  \
            wlan_hdd_scan.o  \
            wlan_hdd_assoc.o  \
            wlan_hdd_mib.o  \
            wlan_hdd_wext.o \
            wlan_hdd_tx_rx.o   \
            wlan_hdd_dp_utils.o \
            wlan_hdd_early_suspend.o  \
            wlan_hdd_wmm.o \
	    wlan_hdd_cfg.o \
            wlan_hdd_wowl.o \
            wlan_hdd_innav.o \
            wlan_hdd_ftm.o  \
            wlan_hdd_hostapd.o  \
            wlan_hdd_softap_tx_rx.o \
            wlan_hdd_dev_pwr.o \
            bap_hdd_main.o

ifeq ($(CONFIG_CFG80211), y)
HDD_OBJS += wlan_hdd_cfg80211.o \
            wlan_hdd_p2p.o
endif

############ BAP #####################
BAP_INC := 	-I$(WLAN_DIR)/BAP/inc \
		-I$(WLAN_DIR)/BAP/src

BAP_SRC_DIR := ../../BAP/src
BAP_OBJS := $(BAP_SRC_DIR)/bapModule.o        \
         $(BAP_SRC_DIR)/btampHCI.o         \
         $(BAP_SRC_DIR)/bapApiLinkCntl.o   \
         $(BAP_SRC_DIR)/bapApiHCBB.o       \
         $(BAP_SRC_DIR)/bapApiInfo.o       \
         $(BAP_SRC_DIR)/bapApiStatus.o     \
         $(BAP_SRC_DIR)/bapApiDebug.o      \
         $(BAP_SRC_DIR)/bapApiData.o       \
         $(BAP_SRC_DIR)/bapApiExt.o        \
         $(BAP_SRC_DIR)/bapApiTimer.o      \
         $(BAP_SRC_DIR)/btampFsm.o         \
         $(BAP_SRC_DIR)/bapRsn8021xAuthFsm.o \
         $(BAP_SRC_DIR)/bapRsn8021xSuppRsnFsm.o \
         $(BAP_SRC_DIR)/bapRsn8021xPrf.o     \
         $(BAP_SRC_DIR)/bapRsnAsfPacket.o    \
         $(BAP_SRC_DIR)/bapRsnSsmEapol.o     \
         $(BAP_SRC_DIR)/bapRsnSsmReplayCtr.o \
         $(BAP_SRC_DIR)/bapRsnTxRx.o         \
         $(BAP_SRC_DIR)/bapRsnSsmAesKeyWrap.o	\
	 $(BAP_SRC_DIR)/bapApiLinkSupervision.o

############# VOSS ################################
VOSS_INC := -I$(WLAN_DIR)/VOSS/inc/ \
           -I$(WLAN_DIR)/VOSS/src

VOSS_SRC_DIR := ../../VOSS/src
VOSS_OBJS :=    $(VOSS_SRC_DIR)/vos_types.o \
                $(VOSS_SRC_DIR)/vos_event.o  \
                $(VOSS_SRC_DIR)/vos_getBin.o \
                $(VOSS_SRC_DIR)/vos_list.o   \
                $(VOSS_SRC_DIR)/vos_lock.o   \
                $(VOSS_SRC_DIR)/vos_memory.o \
                $(VOSS_SRC_DIR)/vos_mq.o     \
                $(VOSS_SRC_DIR)/vos_nvitem.o \
                $(VOSS_SRC_DIR)/vos_packet.o \
                $(VOSS_SRC_DIR)/vos_power.o  \
                $(VOSS_SRC_DIR)/vos_threads.o \
                $(VOSS_SRC_DIR)/vos_timer.o   \
                $(VOSS_SRC_DIR)/vos_trace.o  \
                $(VOSS_SRC_DIR)/vos_api.o    \
                $(VOSS_SRC_DIR)/vos_sched.o   \
                $(VOSS_SRC_DIR)/vos_utils.o

ifeq ($(BUILD_DIAG_VERSION),1)
VOSS_OBJS += $(VOSS_SRC_DIR)/vos_diag.o
endif

############# TL ##################### 
TL_INC := 	-I$(WLAN_DIR)/TL/inc \
         -I$(WLAN_DIR)/TL/src

TL_SRC_DIR := ../../TL/src
TL_OBJS := 	$(TL_SRC_DIR)/wlan_qct_tl.o \
             $(TL_SRC_DIR)/wlan_qct_tl_ba.o \
             $(TL_SRC_DIR)/wlan_qct_tl_hosupport.o

############# SYS ##################### 
SYS_INC := 	-I$(WLAN_DIR)/SYS/common/inc \
        -I$(WLAN_DIR)/SYS/legacy/src/pal/inc \
        -I$(WLAN_DIR)/SYS/legacy/src/platform/inc \
        -I$(WLAN_DIR)/SYS/legacy/src/system/inc \
        -I$(WLAN_DIR)/SYS/legacy/src/utils/inc \

SYS_COMMON_SRC_DIR := ../../SYS/common/src
SYS_LEGACY_SRC_DIR := ../../SYS/legacy/src
SYS_OBJS :=	$(SYS_COMMON_SRC_DIR)/wlan_qct_sys.o \
        $(SYS_LEGACY_SRC_DIR)/pal/src/palApiComm.o \
        $(SYS_LEGACY_SRC_DIR)/pal/src/palTimer.o \
        $(SYS_LEGACY_SRC_DIR)/platform/src/VossWrapper.o \
        $(SYS_LEGACY_SRC_DIR)/system/src/macInitApi.o \
        $(SYS_LEGACY_SRC_DIR)/system/src/sysEntryFunc.o \
        $(SYS_LEGACY_SRC_DIR)/system/src/sysWinStartup.o \
        $(SYS_LEGACY_SRC_DIR)/utils/src/dot11f.o \
        $(SYS_LEGACY_SRC_DIR)/utils/src/logApi.o \
        $(SYS_LEGACY_SRC_DIR)/utils/src/logDump.o \
        $(SYS_LEGACY_SRC_DIR)/utils/src/macTrace.o \
        $(SYS_LEGACY_SRC_DIR)/utils/src/parserApi.o \
        $(SYS_LEGACY_SRC_DIR)/utils/src/utilsApi.o \
        $(SYS_LEGACY_SRC_DIR)/utils/src/utilsParser.o

############# MAC ##################### 
MAC_INC := 	-I$(WLAN_DIR)/MAC/inc \
        -I$(WLAN_DIR)/MAC/src/include \
        -I$(WLAN_DIR)/MAC/src/pe/include \
        -I$(WLAN_DIR)/MAC/src/dph \
        -I$(WLAN_DIR)/MAC/src/pe/lim \
        -I$(WLAN_DIR)/MAC/src/dvt

MAC_SRC_DIR :=  ../../MAC/src

MAC_CFG_OBJS := $(MAC_SRC_DIR)/cfg/cfgApi.o \
        $(MAC_SRC_DIR)/cfg/cfgProcMsg.o \
        $(MAC_SRC_DIR)/cfg/cfgSendMsg.o \
        $(MAC_SRC_DIR)/cfg/cfgDebug.o \
        $(MAC_SRC_DIR)/cfg/cfgParamName.o

MAC_DPH_OBJS := $(MAC_SRC_DIR)/dph/dphHashTable.o
MAC_LIM_OBJS := $(MAC_SRC_DIR)/pe/lim/limAIDmgmt.o               \
            $(MAC_SRC_DIR)/pe/lim/limApi.o                   \
            $(MAC_SRC_DIR)/pe/lim/limAssocUtils.o            \
            $(MAC_SRC_DIR)/pe/lim/limIbssPeerMgmt.o          \
            $(MAC_SRC_DIR)/pe/lim/limLinkMonitoringAlgo.o    \
            $(MAC_SRC_DIR)/pe/lim/limProcessActionFrame.o    \
            $(MAC_SRC_DIR)/pe/lim/limProcessAssocReqFrame.o  \
            $(MAC_SRC_DIR)/pe/lim/limProcessAssocRspFrame.o  \
            $(MAC_SRC_DIR)/pe/lim/limProcessAuthFrame.o      \
            $(MAC_SRC_DIR)/pe/lim/limProcessBeaconFrame.o    \
            $(MAC_SRC_DIR)/pe/lim/limProcessCfgUpdates.o     \
            $(MAC_SRC_DIR)/pe/lim/limProcessDeauthFrame.o    \
            $(MAC_SRC_DIR)/pe/lim/limProcessDisassocFrame.o  \
            $(MAC_SRC_DIR)/pe/lim/limProcessLmmMessages.o    \
            $(MAC_SRC_DIR)/pe/lim/limProcessMessageQueue.o   \
            $(MAC_SRC_DIR)/pe/lim/limProcessMlmReqMessages.o \
            $(MAC_SRC_DIR)/pe/lim/limProcessMlmRspMessages.o \
            $(MAC_SRC_DIR)/pe/lim/limProcessProbeReqFrame.o  \
            $(MAC_SRC_DIR)/pe/lim/limProcessProbeRspFrame.o  \
            $(MAC_SRC_DIR)/pe/lim/limProcessSmeReqMessages.o \
            $(MAC_SRC_DIR)/pe/lim/limPropExtsUtils.o         \
            $(MAC_SRC_DIR)/pe/lim/limRoamingAlgo.o           \
            $(MAC_SRC_DIR)/pe/lim/limScanResultUtils.o       \
            $(MAC_SRC_DIR)/pe/lim/limSecurityUtils.o         \
            $(MAC_SRC_DIR)/pe/lim/limSendManagementFrames.o  \
            $(MAC_SRC_DIR)/pe/lim/limSendMessages.o          \
            $(MAC_SRC_DIR)/pe/lim/limSendSmeRspMessages.o    \
            $(MAC_SRC_DIR)/pe/lim/limSerDesUtils.o           \
            $(MAC_SRC_DIR)/pe/lim/limSmeReqUtils.o           \
            $(MAC_SRC_DIR)/pe/lim/limStaHashApi.o            \
            $(MAC_SRC_DIR)/pe/lim/limTimerUtils.o            \
            $(MAC_SRC_DIR)/pe/lim/limUtils.o                 \
            $(MAC_SRC_DIR)/pe/lim/limLogDump.o               \
            $(MAC_SRC_DIR)/pe/lim/limDebug.o             \
            $(MAC_SRC_DIR)/pe/lim/limTrace.o         \
            $(MAC_SRC_DIR)/pe/lim/limAdmitControl.o \
            $(MAC_SRC_DIR)/pe/lim/limSession.o \
            $(MAC_SRC_DIR)/pe/lim/limSessionUtils.o \
            $(MAC_SRC_DIR)/pe/lim/limFT.o \
            $(MAC_SRC_DIR)/pe/lim/limP2P.o

MAC_PMM_OBJS = $(MAC_SRC_DIR)/pe/pmm/pmmApi.o    \
            $(MAC_SRC_DIR)/pe/pmm/pmmDebug.o     \
            $(MAC_SRC_DIR)/pe/pmm/pmmAP.o

MAC_SCH_OBJS = $(MAC_SRC_DIR)/pe/sch/schApi.o           \
            $(MAC_SRC_DIR)/pe/sch/schBeaconGen.o     \
            $(MAC_SRC_DIR)/pe/sch/schBeaconProcess.o \
            $(MAC_SRC_DIR)/pe/sch/schDebug.o         \
            $(MAC_SRC_DIR)/pe/sch/schMessage.o

MAC_RRM_OBJS = $(MAC_SRC_DIR)/pe/rrm/rrmApi.o

MAC_OBJS = $(MAC_CFG_OBJS) \
           $(MAC_DPH_OBJS) \
           $(MAC_LIM_OBJS) \
           $(MAC_PMM_OBJS) \
           $(MAC_SCH_OBJS) \
           $(MAC_RRM_OBJS)

############# SME ##################### 
SME_INC := -I$(WLAN_DIR)/SME/inc \
          -I$(WLAN_DIR)/SME/src/csr

SME_SRC_DIR := ../../SME/src

SME_CCM_OBJS := $(SME_SRC_DIR)/ccm/ccmApi.o \
               $(SME_SRC_DIR)/ccm/ccmLogDump.o
SME_CSR_OBJS := $(SME_SRC_DIR)/csr/csrLinkList.o \
        $(SME_SRC_DIR)/csr/csrApiScan.o \
        $(SME_SRC_DIR)/csr/csrApiRoam.o \
        $(SME_SRC_DIR)/csr/csrCmdProcess.o \
        $(SME_SRC_DIR)/csr/csrUtil.o \
        $(SME_SRC_DIR)/csr/csrLogDump.o \
        $(SME_SRC_DIR)/csr/csrNeighborRoam.o 

SME_PMC_OBJS := $(SME_SRC_DIR)/pmc/pmcApi.o \
        $(SME_SRC_DIR)/pmc/pmc.o \
        $(SME_SRC_DIR)/pmc/pmcLogDump.o

SME_QOS_OBJS := $(SME_SRC_DIR)/QoS/sme_Qos.o

SME_CMN_OBJS := $(SME_SRC_DIR)/sme_common/sme_Api.o $(SME_SRC_DIR)/sme_common/sme_FTApi.o

SME_BTC_OBJS := $(SME_SRC_DIR)/btc/btcApi.o

SME_MEAS_OBJS := $(SME_SRC_DIR)/meas/measApi.o
SME_P2P_OBJS = $(SME_SRC_DIR)/p2p/p2p_Api.o
SME_RRM_OBJS := $(SME_SRC_DIR)/rrm/sme_rrm.o

SME_OBJS :=  $(SME_CCM_OBJS) \
        $(SME_CSR_OBJS) \
        $(SME_PMC_OBJS) \
        $(SME_QOS_OBJS) \
        $(SME_CMN_OBJS) \
        $(SME_BTC_OBJS) \
        $(SME_MEAS_OBJS) \
        $(SME_P2P_OBJS) \
	$(SME_RRM_OBJS)

############### SVC  #################

SVC_INC :=  	-I$(WLAN_DIR)/SVC/inc/
SVC_EXT_INC :=  -I$(WLAN_DIR)/SVC/external
NLINK_SRC_DIR := ../../SVC/src/nlink
BTC_SRC_DIR := ../../SVC/src/btc
PTT_SRC_DIR := ../../SVC/src/ptt
NLINK_OBJS :=    $(NLINK_SRC_DIR)/wlan_nlink_srv.o
BTC_OBJS :=      $(BTC_SRC_DIR)/wlan_btc_svc.o
PTT_OBJS :=      $(PTT_SRC_DIR)/wlan_ptt_sock_svc.o

############# SAP ##################### 
SAP_INC = -I$(WLAN_DIR)/SAP/inc \
          -I$(WLAN_DIR)/SAP/src

SAP_SRC_DIR = ../../SAP/src

SAP_OBJS = $(SAP_SRC_DIR)/sapApiLinkCntl.o \
	$(SAP_SRC_DIR)/sapFsm.o \
	$(SAP_SRC_DIR)/sapModule.o\
	$(SAP_SRC_DIR)/sapChSelect.o

############# WDA ##################### 
WDA_INC = -I$(WLAN_DIR)/WDA/inc \
          -I$(WLAN_DIR)/WDA/inc/legacy \
          -I$(WLAN_DIR)/WA/src

WDA_SRC_DIR = ../../WDA/src

WDA_OBJS = $(WDA_SRC_DIR)/wlan_qct_wda.o \
	$(WDA_SRC_DIR)/wlan_qct_wda_debug.o \
	$(WDA_SRC_DIR)/wlan_qct_wda_ds.o \
	$(WDA_SRC_DIR)/wlan_qct_wda_legacy.o \
	$(WDA_SRC_DIR)/wlan_nv.o

############# DXE ##################### 
DXE_INC = -I$(WLAN_DIR)/DXE/inc \
          -I$(WLAN_DIR)/DXE/src

DXE_SRC_DIR = ../../DXE/src

DXE_OBJS = $(DXE_SRC_DIR)/wlan_qct_dxe.o \
   $(DXE_SRC_DIR)/wlan_qct_dxe_cfg_i.o \

############## WDI CP ################################
WDI_CP_INC     = -I$(WLAN_DIR)/WDI/CP/inc/ \

WDI_CP_SRC_DIR = ../../WDI/CP/src
WDI_CP_OBJS    = $(WDI_CP_SRC_DIR)/wlan_qct_wdi.o  \
            $(WDI_CP_SRC_DIR)/wlan_qct_wdi_dp.o  \
            $(WDI_CP_SRC_DIR)/wlan_qct_wdi_sta.o  \

############## WDI DP ################################
WDI_DP_INC     = -I$(WLAN_DIR)/WDI/DP/inc/ \

WDI_DP_SRC_DIR = ../../WDI/DP/src
WDI_DP_OBJS    = $(WDI_DP_SRC_DIR)/wlan_qct_wdi_bd.o  \
                 $(WDI_DP_SRC_DIR)/wlan_qct_wdi_ds.o  \

############## WDI TRP ################################
WDI_TRP_INC     = -I$(WLAN_DIR)/WDI/TRP/CTS/inc/ \
                  -I$(WLAN_DIR)/WDI/TRP/DTS/inc/ 

WDI_TRP_CTS_SRC_DIR = ../../WDI/TRP/CTS/src
WDI_TRP_CTS_OBJS    = $(WDI_TRP_CTS_SRC_DIR)/wlan_qct_wdi_cts.o 

WDI_TRP_DTS_SRC_DIR = ../../WDI/TRP/DTS/src
WDI_TRP_DTS_OBJS    = $(WDI_TRP_DTS_SRC_DIR)/wlan_qct_wdi_dts.o 

WDI_TRP_OBJS        = $(WDI_TRP_CTS_OBJS) \
                      $(WDI_TRP_DTS_OBJS)

############## WDI WPAL ################################
WDI_WPAL_INC     = -I$(WLAN_DIR)/WDI/WPAL/inc

WDI_WPAL_SRC_DIR = ../../WDI/WPAL/src
WDI_WPAL_OBJS    = $(WDI_WPAL_SRC_DIR)/wlan_qct_pal_packet.o  \
            $(WDI_WPAL_SRC_DIR)/wlan_qct_pal_timer.o  \
            $(WDI_WPAL_SRC_DIR)/wlan_qct_pal_trace.o  \
            $(WDI_WPAL_SRC_DIR)/wlan_qct_pal_api.o  \
            $(WDI_WPAL_SRC_DIR)/wlan_qct_pal_sync.o \
            $(WDI_WPAL_SRC_DIR)/wlan_qct_pal_msg.o \
            $(WDI_WPAL_SRC_DIR)/wlan_qct_pal_device.o \

############# WDI ##################### 
WDI_INC = $(WDI_WPAL_INC)\
          $(WDI_CP_INC)\
          $(WDI_DP_INC)\
          $(WDI_TRP_INC)

WDI_OBJS = $(WDI_WPAL_OBJS) \
   $(WDI_CP_OBJS) \
   $(WDI_DP_OBJS) \
   $(WDI_TRP_OBJS) 

############# RIVA ##################### 
RIVA_INC    =    -I$(WLAN_DIR)/../riva/inc/

EXTRA_CFLAGS += $(addprefix -I, $(KERNEL_INC))
EXTRA_CFLAGS += $(addprefix -I, $(HDD_INC))
EXTRA_CFLAGS += $(BAP_INC)
EXTRA_CFLAGS += $(SAP_INC)
EXTRA_CFLAGS += $(VOSS_INC)
EXTRA_CFLAGS += $(TL_INC)
EXTRA_CFLAGS += $(SYS_INC)
EXTRA_CFLAGS += $(MAC_INC)
EXTRA_CFLAGS += $(SME_INC)
EXTRA_CFLAGS += $(SVC_INC)
EXTRA_CFLAGS += $(SVC_EXT_INC)
EXTRA_CFLAGS += $(DXE_INC)
EXTRA_CFLAGS += $(WDA_INC)
EXTRA_CFLAGS += $(WDI_INC)
EXTRA_CFLAGS += $(RIVA_INC)
EXTRA_LDFLAGS += --strip-debug

#ifneq (,$(findstring arm-eabi-4.4, $(ANDROID_EABI_TOOLCHAIN)))
EXTRA_CFLAGS += -Wframe-larger-than=8192
#endif


CDEFINES  := -DANI_PRODUCT_TYPE_CLIENT=1 \
        -DANI_BUS_TYPE_PLATFORM=1 \
        -DANI_LITTLE_BYTE_ENDIAN \
        -DANI_LITTLE_BIT_ENDIAN \
        -DWLAN_STA=1 \
        -DAP=2 \
        -DWNI_POLARIS_FW_PRODUCT=1 \
        -DQC_WLAN_CHIPSET_PRIMA \
        -DINTEGRATION_READY \
        -DVOSS_ENABLED \
        -DDOT11F_LITTLE_ENDIAN_HOST \
        -DGEN6_ONWARDS \
        -DANI_COMPILER_TYPE_GCC \
        -DANI_OS_TYPE_ANDROID=6 \
        -DWNI_POLARIS_FW_OS=6 \
        -DADVANCED=3 \
        -DWNI_POLARIS_FW_PACKAGE=9 \
        -DTRACE_RECORD \
        -DPE_DEBUG_LOGW \
        -DPE_DEBUG_LOGE \
        -DDEBUG \
        -DANI_LOGDUMP \
        -DWLAN_PERF \
        -DUSE_LOCKED_QUEUE \
        -DPTT_SOCK_SVC_ENABLE \
        -DFEATURE_WLAN_UAPSD_FW_TRG_FRAMES \
        -DWLAN_SOFTAP_FEATURE \
        -Wall\
        -DWLAN_DEBUG \
        -D__linux__ \
        -DMSM_PLATFORM \
        -DFEATURE_WLAN_INTEGRATED_SOC \
        -DBAP_DEBUG \
        -DHAL_SELF_STA_PER_BSS=1 \
        -DANI_MANF_DIAG \
        -DWLAN_BTAMP_FEATURE \
        -DRSSI_HACK \
        -DWLAN_FEATURE_VOWIFI_11R \
        -DWLAN_FEATURE_NEIGHBOR_ROAMING \
        -DWLAN_FEATURE_NEIGHBOR_ROAMING_DEBUG \
        -DWLAN_FEATURE_VOWIFI_11R_DEBUG \
        -DFEATURE_WLAN_WAPI \
        -DFEATURE_INNAV_SUPPORT\
        -DSOFTAP_CHANNEL_RANGE \
		-DWLAN_FEATURE_CIQ_METRICS 
        #-DFEATURE_WLAN_BTAMP_UT \
        #-DVOLANS_PERF \
        #-DWLAN_FEATURE_VOWIFI \
        #-DRSSI_HACK \
        #-DWLAN_FEATURE_VOWIFI_11R \
        #-DWLAN_FEATURE_NEIGHBOR_ROAMING \
        #-DWLAN_FEATURE_VOWIFI_11R_DEBUG \
        #-DWLAN_VOWIFI_DEBUG \


# there are still pieces of code which are conditional upon these
# need to investigate all of them to see which should also be
# conditional upon QC_WLAN_CHIPSET_PRIMA
CDEFINES += -DANI_CHIPSET_VOLANS 

ifeq ($(BUILD_DEBUG_VERSION),1)
CDEFINES += -DWLAN_DEBUG \
		-DTRACE_RECORD \
		-DPE_DEBUG_LOGW \
		-DPE_DEBUG_LOGE \
		-DDEBUG 
endif

ifeq ($(CONFIG_CFG80211), y)
CDEFINES += -DWLAN_FEATURE_P2P
CDEFINES += -DWLAN_FEATURE_WFD
endif

ifeq ($(BUILD_DIAG_VERSION),1)
CDEFINES += -DFEATURE_WLAN_DIAG_SUPPORT
CDEFINES += -DFEATURE_WLAN_DIAG_SUPPORT_CSR
CDEFINES += -DFEATURE_WLAN_DIAG_SUPPORT_LIM
endif

#ifneq (, $(filter msm8960, $(TARGET_PRODUCT)))
EXTRA_CFLAGS += -march=armv7-a
CDEFINES += -DMSM_PLATFORM_8960
#endif

ifneq (, $(filter msm8660_surf msm8660_csfb, $(TARGET_PRODUCT)))
EXTRA_CFLAGS += -march=armv7-a
CDEFINES += -DMSM_PLATFORM_8660
endif

ifneq (, $(filter msm7630_surf msm7630_fusion, $(TARGET_PRODUCT)))
EXTRA_CFLAGS += -march=armv7-a
CDEFINES += -DMSM_PLATFORM_7x30
endif

ifneq (, $(filter msm7627_surf msm7627_7x_surf, $(TARGET_PRODUCT)))
EXTRA_CFLAGS += -march=armv6
CDEFINES += -DMSM_PLATFORM_7x27
endif


ifneq (, $(filter msm7627_ffa msm7627_7x_ffa, $(TARGET_PRODUCT)))
EXTRA_CFLAGS += -march=armv6
CDEFINES += -DMSM_PLATFORM_7x27_FFA
endif

#default TARGET_TYPE is RF
TARGET_TYPE :=RF

ifeq ($(TARGET_TYPE),RF)
CDEFINES += -DVOLANS_RF
MODNAME := prima_wlan
else ifeq ($(TARGET_TYPE),BB)
CDEFINES += -DVOLANS_BB
MODNAME := prima_wlan_bb
else
CDEFINES += -DVOLANS_FPGA
MODNAME := prima_wlan_fpga
endif

ifeq ($(BUILD_FTM_DRIVER),1)
CDEFINES += -DANI_MANF_DIAG \
	-UWLAN_FTM_STUB \
	-DANI_PHY_DEBUG
MODNAME := $(MODNAME)_ftm
endif

ifeq ($(WLAN_DBG),1)
CDEFINES += -DWLAN_DEBUG
endif

KBUILD_CPPFLAGS += $(CDEFINES)

# Objects required by the driver file
OBJS := $(VOSS_OBJS) \
                $(BAP_OBJS)  \
                $(TL_OBJS)   \
                $(SME_OBJS)  \
                $(SYS_OBJS)  \
                $(SAP_OBJS)  \
                $(MAC_OBJS)  \
                $(HAL_OBJS)  \
                $(HDD_OBJS)  \
                $(NLINK_OBJS)\
                $(BTC_OBJS)  \
                $(PTT_OBJS)  \
                $(WDA_OBJS)  \
                $(WDI_OBJS)  \
                $(DXE_OBJS)  \


# Module information used by KBuild framework
obj-m   := $(MODNAME).o
$(MODNAME)-y += $(OBJS)
