obj-y += modem_notifier.o

obj-$(CONFIG_MSM_SDIO_AL) += sdio_al.o
obj-$(CONFIG_MSM_SDIO_AL_TEST) += sdio_al_test.o
obj-$(CONFIG_MSM_SMD) += smem_log.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += rpc_pmapp.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_clients.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_xdr.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVER_TIME_REMOTE) += rpc_server_time_remote.o

obj-$(CONFIG_MSM_ONCRPCROUTER) += rpcrouter_smd_xprt.o
obj-$(CONFIG_MSM_RPC_SDIO_XPRT) += rpcrouter_sdio_xprt.o

obj-$(CONFIG_MSM_SDIO_CTL) += sdio_ctl.o

ifeq ($(CONFIG_MACH_MECHA),y)
obj-$(CONFIG_MSM_SDIO_DIAG) += sdio_diag.o
endif

obj-$(CONFIG_MSM_SMD_NMEA) += smd_nmea.o
obj-$(CONFIG_MSM_SMD_PKT) += smd_pkt.o

quiet_cmd_mkrpcsym = MKCAP   $@
      cmd_mkrpcsym = $(PERL) $(srctree)/$(src)/mkrpcsym.pl $< $@

target += smd_rpc_sym.c
$(obj)/smd_rpc_sym.c: $(src)/smd_rpc_sym $(src)/mkrpcsym.pl
	$(call if_changed,mkrpcsym)

obj-$(CONFIG_DEBUG_FS) += smd_rpc_sym.o

