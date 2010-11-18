/*
* FILE FUNCTION: BTLINUXPORT Driver External Definitions
*
* REVISION HISTORY:
*/
#ifndef _BTLINUXPORT_
#define _BTLINUXPORT_

#define BTLINUX_PORT_OPENED         0
#define BTLINUX_PORT_CLOSED         1
#define BTLINUX_PORT_ENABLE_MS      2
#define BTLINUX_PORT_SET_BREAK_ON   3
#define BTLINUX_PORT_SET_BREAK_OFF  4
#define BTLINUX_PORT_MODEM_CONTROL  5
#define BTLINUX_PORT_MODEM_STATUS   6
#define BTLINUX_PORT_TX_EMPTY       7


#define MODEM_CNTRL_DTRDSR_MASK        0x0001
#define MODEM_CNTRL_CTSRTS_MASK        0x0002
#define MODEM_CNTRL_RNG_MASK           0x0004
#define MODEM_CNTRL_CAR_MASK           0x0008

typedef struct _BTLINUXPORTEvent {
    int eventCode;
    union {
       short modemControlReg;
    } u;
} BTLINUXPORTEvent, *pBTLINUXPORTEvent;

/*
 ioctl defs
*/
#define IOCTL_BTPORT_GET_EVENT            0x1001
#define IOCTL_BTPORT_SET_EVENT_RESULT     0x1002
#define IOCTL_BTPORT_HANDLE_MCTRL         0x1003

#define USB_DIAG_IOC_MAGIC 0xFF

#define USB_DIAG_FUNC_IOC_ENABLE_SET	_IOW(USB_DIAG_IOC_MAGIC, 1, int)
#define USB_DIAG_FUNC_IOC_ENABLE_GET	_IOR(USB_DIAG_IOC_MAGIC, 2, int)

#define USB_DIAG_FUNC_IOC_REGISTER_SET  _IOW(USB_DIAG_IOC_MAGIC, 3, char *)
#define USB_DIAG_FUNC_IOC_AMR_SET	_IOW(USB_DIAG_IOC_MAGIC, 4, int)
#endif /* _BTUSBEXT_*/

