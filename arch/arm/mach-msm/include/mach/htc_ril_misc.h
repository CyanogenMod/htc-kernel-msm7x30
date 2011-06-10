#ifndef __HTC_RIL_H__
#define __HTC_RIL_H__

#define PRINTRTC  do { \
struct timespec ts; \
struct rtc_time tm; \
getnstimeofday(&ts); \
rtc_time_to_tm(ts.tv_sec, &tm); \
printk("At (%d-%02d-%02d %02d:%02d:%02d.%09lu) ", \
tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#endif /*__HTC_RIL_H__ */

