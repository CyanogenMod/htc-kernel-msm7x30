/* interface for the pm_qos_power infrastructure of the linux kernel.
 *
 * Mark Gross <mgross@linux.intel.com>
 */
#ifndef __PM_QOS_PARAMS_H__
#define __PM_QOS_PARAMS_H__

#include <linux/list.h>
#include <linux/notifier.h>
#include <linux/miscdevice.h>

#define PM_QOS_RESERVED 0
#define PM_QOS_CPU_DMA_LATENCY 1
#define PM_QOS_NETWORK_LATENCY 2
#define PM_QOS_NETWORK_THROUGHPUT 3
#define PM_QOS_SYSTEM_BUS_FREQ 4

#define PM_QOS_NUM_CLASSES 5
#define PM_QOS_DEFAULT_VALUE -1

struct requirement_list {
	struct list_head list;
	union {
		s32 value;
		s32 usec;
		s32 kbps;
	};
	char *name;
	void *data;
};

struct pm_qos_object {
	struct requirement_list requirements;
	struct blocking_notifier_head *notifiers;
	struct miscdevice pm_qos_power_miscdev;
	char *name;
	s32 default_value;
	atomic_t target_value;
	s32 (*comparitor)(s32, s32);
	struct pm_qos_plugin {
		void *data;
		int (*add_fn)(struct pm_qos_object *, char *, s32, void **);
		int (*update_fn)(struct pm_qos_object *, char *, s32, void **);
		int (*remove_fn)(struct pm_qos_object *, char *, s32, void **);
	} *plugin;
};

int pm_qos_register_plugin(int pm_qos_class, struct pm_qos_plugin *plugin);

int pm_qos_add_requirement(int qos, char *name, s32 value);
int pm_qos_update_requirement(int qos, char *name, s32 new_value);
void pm_qos_remove_requirement(int qos, char *name);

int pm_qos_requirement(int qos);

int pm_qos_add_notifier(int qos, struct notifier_block *notifier);
int pm_qos_remove_notifier(int qos, struct notifier_block *notifier);

#endif /* __PM_QOS_PARAMS_H__ */
