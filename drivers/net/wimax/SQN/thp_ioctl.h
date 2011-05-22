#ifndef _THP_IOCTL_H_
#define _THP_IOCTL_H_

//ioctl group number
//must be a nonnegative 8-bit number
#define		WIMAX_DEV_IOCTLID	'w'

#define		CMD_BASE		65
//ioctl type within the group
//should be sequentially assigned numbers for each different ioctl operation
//must be a nonnegative 8-bit number
#define		CMD_DROP_PACKETS	  CMD_BASE+0
#define		CMD_SIWTCH_UART       CMD_BASE+1
#define		CMD_SIWTCH_NETLOG     CMD_BASE+2

//write only
//arg=1, drop tx/rx packets
//arg=0, normal mode
#define		IOCTL_DROP_PACKETS	    _IOW(WIMAX_DEV_IOCTLID, CMD_DROP_PACKETS, int)
#define     IOCTL_SWITCH_UART	    _IOW(WIMAX_DEV_IOCTLID, CMD_SIWTCH_UART,  int)
#define     IOCTL_SWITCH_NETLOG	    _IOW(WIMAX_DEV_IOCTLID, CMD_SIWTCH_NETLOG,  int)

#endif //_THP_IOCTL_H_
