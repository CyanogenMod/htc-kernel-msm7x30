#ifndef _DEBUG_BY_VIBRATOR_H
#define _DEBUG_BY_VIBRATOR_H

#if defined(CONFIG_MSM_RPC_VIBRATOR)
#define DEBUG_BY_VIBRATOR
#endif

#define DISABLE			0
#define ERR_MODE		1
#define CRASH_MODE		2
#define EXPIRED_TIMEOUT		300

extern int debug_by_vibrator(int mode, const char *name);
extern int get_vibrator_enabled(void);

#endif /*_DEBUG_BY_VIBRATOR_H*/
