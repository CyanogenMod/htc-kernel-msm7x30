GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
ifeq ($(GCC_VERSION),0404)
CFLAGS_REMOVE_msm_vfe8x.o = -Wframe-larger-than=1024
endif

ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  obj-$(CONFIG_MSM_CAMERA) += msm_isp.o msm.o msm_mem.o msm_mctl.o msm_mctl_buf.o msm_mctl_pp.o
  obj-$(CONFIG_MSM_CAMERA) += rawchip/ io/ sensors/ actuators/
else
ifeq ($(CONFIG_ARCH_MSM8X60),y)
  obj-$(CONFIG_MSM_CAMERA) += msm_camera-8x60.o sensors/

  ifeq ($(CONFIG_CAMERA_3D),y)
    obj-$(CONFIG_MSM_CAMERA) += msm_camera_liteon.o sensors/
  endif

else
ifeq ($(CONFIG_ARCH_MSM7X30),y)
  obj-$(CONFIG_MSM_CAMERA) += msm_camera-7x30.o sensors/ rawchip/
else
  obj-$(CONFIG_MSM_CAMERA) += msm_camera.o
endif
endif
endif
obj-$(CONFIG_MSM_CAMERA) += msm_axi_qos.o

ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
	obj-$(CONFIG_MSM_CAMERA) += gemini/
	obj-$(CONFIG_MSM_CAMERA_FLASH) += flash.o
else
	obj-$(CONFIG_MSM_CAMERA) += gemini_8x60/
ifeq ($(CONFIG_ARCH_MSM7X27A),y)
	obj-$(CONFIG_MSM_CAMERA_FLASH) += flash.o
else
	obj-$(CONFIG_MSM_CAMERA_FLASH) += flash_8x60.o
endif
endif
obj-$(CONFIG_ARCH_MSM_ARM11) += msm_vfe7x.o msm_io7x.o
obj-$(CONFIG_ARCH_MSM7X27A) += msm_vfe7x27a.o msm_io_7x27a.o
obj-$(CONFIG_ARCH_MSM7X30) += msm_vfe_7x30.o msm_io_7x30.o msm_vpe1_7x30.o
obj-$(CONFIG_ARCH_QSD8X50) += msm_vfe8x.o msm_vfe8x_proc.o msm_io8x.o

ifdef CONFIG_S5K4E5YX
obj-$(CONFIG_MACH_BLISS) += s5k4e5yx.o s5k4e5yx_reg_bls.o
obj-$(CONFIG_MACH_BLISSC) += s5k4e5yx.o s5k4e5yx_reg_bls.o
obj-$(CONFIG_MACH_PRIMOU) += s5k4e5yx.o s5k4e5yx_reg_pro.o
obj-$(CONFIG_MACH_PRIMOC) += s5k4e5yx.o s5k4e5yx_reg_pro.o
obj-$(CONFIG_MACH_KINGDOM) += s5k4e5yx.o s5k4e5yx_reg.o
endif


ifdef CONFIG_CAMERA_ZSL
	obj-$(CONFIG_ARCH_MSM8X60) += msm_io_8x60.o msm_vfe_8x60_ZSL.o msm_vpe1_8x60.o
else
	ifdef CONFIG_CAMERA_3D
		obj-$(CONFIG_ARCH_MSM8X60) += msm_io_8x60.o msm_vfe_8x60.o msm_vpe1_8x60.o
		obj-$(CONFIG_ARCH_MSM8X60) += msm_vfe31_liteon.o msm_vpe1_liteon.o
	else
		obj-$(CONFIG_ARCH_MSM8X60) += msm_io_8x60.o msm_vfe_8x60.o msm_vpe1_8x60.o
	endif
endif

obj-$(CONFIG_ARCH_MSM8960) += msm_io_8960.o msm_ispif.o msm_vfe32.o msm_vpe.o
obj-$(CONFIG_MT9T013) += mt9t013.o mt9t013_reg.o
obj-$(CONFIG_SN12M0PZ) += sn12m0pz.o sn12m0pz_reg.o
obj-$(CONFIG_MT9P012) += mt9p012_reg.o
ifdef CONFIG_MACH_PRIMOTD
obj-$(CONFIG_S5K4E1) += s5k4e1_td.o s5k4e1_reg_td.o
else
obj-$(CONFIG_S5K4E1) += s5k4e1.o s5k4e1_reg.o
endif
obj-$(CONFIG_MSM_CAMERA_AF_FOXCONN) += mt9p012_fox.o
obj-$(CONFIG_MSM_CAMERA_AF_BAM) += mt9p012_bam.o
obj-$(CONFIG_MT9P012_KM) += mt9p012_km.o mt9p012_km_reg.o
obj-$(CONFIG_MT9E013) += mt9e013.o mt9e013_reg.o
obj-$(CONFIG_S5K3E2FX) += s5k3e2fx.o
obj-$(CONFIG_S5K3H1GX) += s5k3h1gx.o s5k3h1gx_reg_pyd.o

#FIXME: Merge the two ifeq causes VX6953 preview not coming up.
ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  obj-$(CONFIG_VX6953) += vx6953_v4l2.o vx6953_reg_v4l2.o
  obj-$(CONFIG_MT9V113) += mt9v113_v4l2.o mt9v113_reg_lens_9251.o
else
  ifdef CONFIG_MACH_BLISS
    obj-$(CONFIG_MT9V113) += mt9v113.o mt9v113_reg_lens_9251.o
  else
    ifdef CONFIG_MACH_BLISSC
      obj-$(CONFIG_MT9V113) += mt9v113.o mt9v113_reg_lens_9251.o
    else
      obj-$(CONFIG_MT9V113) += mt9v113.o mt9v113_reg_lens_9218.o
    endif
  endif
  obj-$(CONFIG_VX6953) += vx6953.o vx6953_reg.o
  obj-$(CONFIG_IMX074) += imx074.o imx074_reg.o
endif
obj-$(CONFIG_QS_MT9P017) += qs_mt9p017.o qs_mt9p017_reg.o
obj-$(CONFIG_VB6801) += vb6801.o
obj-$(CONFIG_IMX072) += imx072.o imx072_reg.o
obj-$(CONFIG_WEBCAM_OV9726) += ov9726.o ov9726_reg.o
obj-$(CONFIG_WEBCAM_OV7692) += ov7692.o
obj-$(CONFIG_MT9D112) += mt9d112.o mt9d112_reg.o

obj-$(CONFIG_MT9D113) += mt9d113.o mt9d113_reg.o
