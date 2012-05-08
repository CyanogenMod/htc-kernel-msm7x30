#ifndef __ARCH_ARM_MACH_MSM_8X60_LTE_HTC_UTIL_H
#define __ARCH_ARM_MACH_MSM_8X60_LTE_HTC_UTIL_H
void htc_PM_monitor_init(void);
void htc_monitor_init(void);
void htc_idle_stat_add(int sleep_mode, u32 time);
void htc_xo_block_CLKs_count(void);
#endif
