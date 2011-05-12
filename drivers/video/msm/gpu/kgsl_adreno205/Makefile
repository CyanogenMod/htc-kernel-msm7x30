ccflags-y := -Iinclude/drm

msm_kgsl-y = \
	kgsl_drawctxt.o \
	kgsl_cmdstream.o \
	kgsl.o \
	kgsl_log.o \
	kgsl_mmu.o \
	kgsl_postmortem.o \
	kgsl_ringbuffer.o \
	kgsl_sharedmem.o \
	kgsl_yamato.o \
	kgsl_g12_drawctxt.o \
	kgsl_g12_cmdwindow.o \
	kgsl_g12_cmdstream.o \
	kgsl_g12.o

msm_kgsl-$(CONFIG_MSM_KGSL_DRM) += kgsl_drm.o

msm_kgsl-objs = $(msm_kgsl-y)

obj-$(CONFIG_GPU_MSM_KGSL) += msm_kgsl.o
