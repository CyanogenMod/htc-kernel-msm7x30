#ifndef RESUME_TRACE_H
#define RESUME_TRACE_H

enum {
	TRACE_DPM_PREPARE = 1U << 0,
	TRACE_DPM_SUSPEND = 1U << 1,
	TRACE_DPM_SUSPEND_NOIRQ = 1U << 2,
	TRACE_SYSDEV_SUSPEND = 1U << 3,
	TRACE_SYSDEV_RESUME = 1U << 4,
	TRACE_DPM_RESUME_NOIRQ = 1U << 5,
	TRACE_DPM_RESUME = 1U << 6,
	TRACE_DPM_COMPLETE = 1U << 7,
	TRACE_PM_WARN = 1U << 8,
};

#ifdef CONFIG_PM_TRACE
#include <asm/resume-trace.h>

extern int pm_trace_enabled;
extern int pm_trace_mask;

static inline int pm_trace_is_enabled(void)
{
       return pm_trace_enabled;
}

struct device;
extern void set_trace_device(struct device *);
extern void generate_resume_trace(const void *tracedata, unsigned int user);

#ifdef CONFIG_PM_TRACE_RTC
#define TRACE_DEVICE(dev) do { \
	if (pm_trace_enabled) \
		set_trace_device(dev); \
	} while(0)
#else
#define TRACE_DEVICE(dev) do { } while (0)
#endif

#define TRACE_MASK(type, format, arg...) do {\
	if (pm_trace_mask & type) \
		pr_info("[PM.%x] " format, type, ## arg); \
	} while(0)

#else

static inline int pm_trace_is_enabled(void) { return 0; }

#define TRACE_DEVICE(dev) do { } while (0)
#define TRACE_RESUME(dev) do { } while (0)
#define TRACE_MASK(type, format, arg...) do { } while (0)

#endif

#endif
