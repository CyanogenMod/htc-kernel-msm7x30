ccflags-y := -Iinclude/drm

msm_kgsl_core-$(CONFIG_GPU_MSM_KGSL) = \
	kgsl.o \
	kgsl_sharedmem.o \
	kgsl_pwrctrl.o

msm_kgsl_core-$(CONFIG_DEBUG_FS) += kgsl_debugfs.o
msm_kgsl_core-$(CONFIG_MSM_KGSL_MMU) += kgsl_mmu.o
msm_kgsl_core-$(CONFIG_MSM_KGSL_CFF_DUMP) += kgsl_cffdump.o
msm_kgsl_core-$(CONFIG_MSM_KGSL_DRM) += kgsl_drm.o

msm_adreno-$(CONFIG_GPU_MSM_KGSL) += \
	adreno_ringbuffer.o \
	adreno_drawctxt.o \
	adreno_postmortem.o \
	adreno.o

msm_adreno-$(CONFIG_DEBUG_FS) += adreno_debugfs.o

msm_z180-$(CONFIG_MSM_KGSL_2D) += \
	kgsl_g12_drawctxt.o \
	kgsl_g12_cmdstream.o \
	kgsl_g12.o

msm_kgsl_core-objs = $(msm_kgsl_core-y)
msm_adreno-objs = $(msm_kgsl_adreno-y)
msm_z180-objs = $(msm_kgsl_z180-y)

obj-$(CONFIG_GPU_MSM_KGSL) += \
	msm_kgsl_core.o \
	msm_adreno.o
