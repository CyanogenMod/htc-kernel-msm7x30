#
# Makefile for TI's shared transport line discipline
# and its protocol drivers (BT, FM, GPS)
#
obj-m 				+= fm_drv.o
fm_drv-objs     		:= fmdrv_common.o fmdrv_rx.o fmdrv_v4l2.o
fm_drv-objs			:= fmdrv_chr.o
#obj-$(CONFIG_ST_GPS)		+= gps_drv.o
#obj-m += ti_hci_drv.o
#ti_hci_drv-objs         := hci_if_drv.o

