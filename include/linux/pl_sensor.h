/*for P/L sensor common header file for each vender chip*/
#ifndef __LINUX_PL_SENSOR_H
#define __LINUX_PL_SENSOR_H

extern struct blocking_notifier_head psensor_notifier_list;

extern int register_notifier_by_psensor(struct notifier_block *nb);
extern int unregister_notifier_by_psensor(struct notifier_block *nb);

#endif

