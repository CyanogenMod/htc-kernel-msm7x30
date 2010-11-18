obj-y += io.o irq.o timer.o dma.o memory.o
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o
obj-y += devices.o pwrtest.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += pmic.o
obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock-arm11.o idle.o
obj-$(CONFIG_ARCH_QSD8X50) += arch-init-scorpion.o acpuclock-scorpion.o
obj-$(CONFIG_ARCH_MSM7X30) += acpuclock-7x30.o internal_power_rail.o
obj-$(CONFIG_ACPUCLOCK_DEBUG) += acpuclock_debug.o
obj-$(CONFIG_ARCH_MSM7X30) += clock-7x30.o arch-init-7x30.o socinfo.o
obj-$(CONFIG_ARCH_MSM7X30) += spm.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += idle-v7.o msm_fault_handlers.o
obj-y += clock.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += drv_callback.o
obj-y += radio_feedback.o
obj-$(CONFIG_ARCH_QSD8X50) += pmic.o htc_wifi_nvs.o htc_bluetooth.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o

obj-$(CONFIG_MSM_SMD) += htc_port_list.o
obj-$(CONFIG_ARCH_MSM7X30) += rpc_hsusb.o

obj-y += remote_spinlock.o
ifndef CONFIG_QCT_LTE
obj-$(CONFIG_ARCH_MSM7X30) += rpc_pmapp.o smd_rpcrouter_clients.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_SMD) += smem_log.o
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
ifndef CONFIG_ARCH_MSM7X30
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
else
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter-7x30.o
endif
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
ifndef CONFIG_ARCH_MSM7X30
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
else
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers-7x30.o
endif
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_xdr.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
else
obj-$(CONFIG_QCT_LTE) += lte/
endif

obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o
obj-$(CONFIG_ARCH_MSM7X30) += dal_axi.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_ADSP_COMP) += qdsp5_comp/
obj-$(CONFIG_MSM7KV2_AUDIO) += qdsp5v2/
obj-$(CONFIG_QSD_AUDIO) += qdsp6/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o

obj-$(CONFIG_HTC_ACOUSTIC) += htc_acoustic.o
obj-$(CONFIG_HTC_ACOUSTIC_QSD) += htc_acoustic_qsd.o
obj-$(CONFIG_MSM7KV2_AUDIO) += htc_acoustic_7x30.o htc_acdb_7x30.o
obj-$(CONFIG_SENSORS_AKM8976) += htc_akm_cal.o

obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o board-halibut-panel.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut-keypad.o fish_battery.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-keypad.o fish_battery.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-panel.o
obj-$(CONFIG_MACH_SWORDFISH) += board-swordfish-mmc.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += htc_akm_cal.o htc_wifi_nvs.o htc_acoustic.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o board-trout-wifi.o
obj-$(CONFIG_MACH_TROUT) += devices_htc.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi.o board-mahimahi-panel.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-keypad.o board-mahimahi-mmc.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-rfkill.o htc_wifi_nvs.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-wifi.o board-mahimahi-audio.o
obj-$(CONFIG_MACH_MAHIMAHI) += msm_vibrator.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-microp.o
obj-$(CONFIG_MACH_MAHIMAHI) += htc_acoustic_qsd.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-flashlight.o

obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec.o panel-samsungwvga-tl2796a.o panel-sonywvga-s6d16a0x21.o board-incrediblec-tv.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-keypad.o board-incrediblec-mmc.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-rfkill.o board-incrediblec-audio.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_INCREDIBLEC) += board-incrediblec-microp.o
obj-$(CONFIG_MACH_INCREDIBLEC) += proc_engineerid.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic.o board-supersonic-panel.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-keypad.o board-supersonic-mmc.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-rfkill.o board-supersonic-audio.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_SUPERSONIC) += board-supersonic-microp.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo.o panel-samsungwvga-tl2796a.o panel-sonywvga-s6d16a0x21.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-keypad.o board-bravo-mmc.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-rfkill.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-audio.o board-bravo-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_BRAVO) += board-bravo-microp.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc.o panel-samsungwvga-tl2796a.o panel-sonywvga-s6d16a0x21.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-keypad.o board-bravoc-mmc.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-rfkill.o board-bravoc-audio.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-wifi.o htc_awb_cal.o
obj-$(CONFIG_MACH_BRAVOC) += board-bravoc-microp.o

# MSM7x30 boards
obj-$(CONFIG_ARCH_MSM7X30) += panel-samsungwvga-tl2796a.o panel-samsungwvga-s6e63m0.o panel-sonywvga-s6d16a0x21-7x30.o
obj-$(CONFIG_ARCH_MSM7X30) += htc_wifi_nvs.o htc_awb_cal.o

obj-$(CONFIG_MACH_GLACIER)+= board-glacier.o
obj-$(CONFIG_MACH_GLACIER)+= board-glacier-keypad.o
obj-$(CONFIG_MACH_GLACIER)+= board-glacier-microp.o
obj-$(CONFIG_MACH_GLACIER)+= board-glacier-audio.o
obj-$(CONFIG_MACH_GLACIER)+= board-glacier-mmc.o board-glacier-wifi.o
obj-$(CONFIG_MACH_GLACIER)+= board-glacier-rfkill.o
obj-$(CONFIG_MACH_GLACIER)+= htc_fmtx_rfkill.o
obj-$(CONFIG_MACH_GLACIER)+= board-glacier-panel.o htc_bluetooth.o

# MSM7200A boards
obj-$(CONFIG_MACH_HERO) += board-hero.o board-hero-panel.o board-hero-keypad.o
obj-$(CONFIG_MACH_HERO) += board-hero-rfkill.o
obj-$(CONFIG_MACH_HERO) += board-hero-mmc.o board-hero-wifi.o htc_wifi_nvs.o htc_awb_cal.o

# MSM7225 boards
obj-$(CONFIG_MACH_BUZZ) += board-buzz.o board-buzz-panel.o board-buzz-keypad.o board-buzz-microp.o
obj-$(CONFIG_MACH_BUZZ) += board-buzz-rfkill.o board-buzz-mmc.o board-buzz-wifi.o
obj-$(CONFIG_MACH_BEE) += board-bee.o board-bee-panel.o board-bee-keypad.o board-bee-microp.o
obj-$(CONFIG_MACH_BEE) += board-bee-rfkill.o board-bee-mmc.o board-bee-wifi.o

# MSM7227 boards
obj-$(CONFIG_MACH_LEGEND) += board-legend-rfkill.o
obj-$(CONFIG_MACH_LEGEND) += board-legend.o board-legend-keypad.o board-legend-panel.o
obj-$(CONFIG_MACH_LEGEND) += board-legend-mmc.o board-legend-wifi.o board-legend-microp.o
obj-$(CONFIG_MACH_LATTE) += board-latte-rfkill.o
obj-$(CONFIG_MACH_LATTE) += board-latte.o board-latte-keypad.o board-latte-panel.o
obj-$(CONFIG_MACH_LATTE) += board-latte-mmc.o board-latte-wifi.o board-latte-microp.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty-rfkill.o htc_bluetooth.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty.o board-liberty-keypad.o board-liberty-panel.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty-mmc.o board-liberty-wifi.o board-liberty-microp.o

# MSM7600 boards
obj-$(CONFIG_MACH_DESIREC) += board-desirec.o board-desirec-panel.o board-desirec-keypad.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-rfkill.o
obj-$(CONFIG_MACH_DESIREC) += board-desirec-mmc.o board-desirec-wifi.o htc_wifi_nvs.o
obj-$(CONFIG_MACH_HEROC) += board-heroc.o board-heroc-panel.o board-heroc-keypad.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-rfkill.o
obj-$(CONFIG_MACH_HEROC) += board-heroc-mmc.o board-heroc-wifi.o htc_wifi_nvs.o

obj-$(CONFIG_ARCH_MSM7X00A) += msm_vibrator.o devices_htc.o
obj-$(CONFIG_ARCH_MSM7225) += msm_vibrator.o devices_htc.o htc_wifi_nvs.o htc_awb_cal.o htc_bluetooth.o
obj-$(CONFIG_ARCH_MSM7227) += msm_vibrator.o devices_htc.o htc_wifi_nvs.o htc_awb_cal.o
obj-$(CONFIG_ARCH_QSD8X50) += msm_vibrator.o devices_htc.o
obj-$(CONFIG_ARCH_MSM7X30) += msm_vibrator.o devices_htc.o
obj-$(CONFIG_ARCH_MSM_FLASHLIGHT) += msm_flashlight.o

obj-$(CONFIG_TROUT_BATTCHG_DOCK) += htc_one_wire.o
obj-$(CONFIG_TROUT_BATTCHG) += htc_battery_trout.o
obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o

obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_MSM_RMT_STORAGE_SERVER) += rmt_storage_server.o
obj-$(CONFIG_MSM_RMT_STORAGE_CLIENT) += rmt_storage_client.o
ifeq ($(CONFIG_MSM_SSBI),y)
else
obj-$(CONFIG_PMIC8058) += pmic8058-gpio.o pmic8058-mpp.o
endif
obj-$(CONFIG_MSM_NPA) += npa.o npa_resources.o msm_pm_qos.o msm_reqs.o
obj-$(CONFIG_MSM_NPA_REMOTE) += npa_remote.o
obj-$(CONFIG_MSM_NPA_REMOTE_ONCRPC) += npa_remote_rpc_client.o

obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_PERFLOCK) += htc_set_perflock.o
obj-$(CONFIG_MICROP_COMMON) += atmega_microp_common.o
obj-$(CONFIG_TOUCHSCREEN_TSSC_MANAGER) += htc_tp_cal.o

obj-$(CONFIG_HTC_HEADSET_MGR) += htc_headset_mgr.o
obj-$(CONFIG_HTC_HEADSET_H2W) += htc_headset_h2w.o
obj-$(CONFIG_HTC_HEADSET_GPIO) += htc_headset_gpio.o
obj-$(CONFIG_HTC_HEADSET_MICROP) += htc_headset_microp.o
obj-$(CONFIG_HTC_HEADSET_PMIC) += htc_headset_pmic.o

obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-tpa2018d1.o
obj-$(CONFIG_MACH_MAHIMAHI) += board-mahimahi-smb329.o
obj-$(CONFIG_MSM_SSBI) += ssbi.o
