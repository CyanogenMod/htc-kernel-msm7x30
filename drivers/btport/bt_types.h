/****************************************************************************/
/*                                                                           */
/*  Name        bt_types.h                                                  */
/*                                                                          */
/*  Function    this file contains definitions that are shared between      */
/*              units in the Bluetooth system such as events...             */
/*                                                                          */
/*  Copyright (c) 1999-2002, Widcomm Inc., All Rights Reserved.             */
/*  Widcomm Bluetooth Core. Proprietary and confidential.                   */
/*                                                                          */
/****************************************************************************/

#ifndef BT_TYPES_H
#define BT_TYPES_H

#include "data_types.h"

#ifdef BLUESTACK_TESTER
    #include "bte_stack_entry.h"
#endif

/* READ WELL !!
**
** This section defines global events. These are events that cross layers.
** Any event that passes between layers MUST be one of these events. Tasks
** can use their own events internally, but a FUNDAMENTAL design issue is
** that global events MUST be one of these events defined below.
**
** The convention used is the the event name contains the layer that the
** event is going to.
*/
#define BT_EVT_MASK                 0xFF00

/* To Bluetooth Upper Layers        */
/************************************/
#define BT_EVT_TO_BTU_L2C_EVT       0x0900      /* L2CAP event */
#define BT_EVT_TO_BTU_HCI_EVT       0x1000      /* HCI Event                        */
#define BT_EVT_TO_BTU_HCI_ACL       0x1100      /* ACL Data from HCI                */
#define BT_EVT_TO_BTU_HCI_SCO       0x1200      /* SCO Data from HCI                */
#define BT_EVT_TO_BTU_HCIT_ERR      0x1300      /* HCI Transport Error              */

#define BT_EVT_TO_BTU_SP_EVT        0x1400      /* Serial Port Event                */
#define BT_EVT_TO_BTU_SP_DATA       0x1500      /* Serial Port Data                 */

#define BT_EVT_TO_BTU_HCI_CMD       0x1600      /* HCI command from upper layer     */

/* OBEX */
#define BT_EVT_TO_BTU_OBEX_REQ      0x1700      /* OBEX Request from application    */
#define BT_EVT_TO_BTU_OBEX_CNF      0x1800      /* OBEX Confirmation from appl      */

#define BT_EVT_TO_BTU_L2C_SEG_XMIT  0x1900      /* L2CAP segment(s) transmitted     */

#define BT_EVT_PROXY_INCOMING_MSG   0x1A00      /* BlueStackTester event: incoming message from target */


/* To LM                            */
/************************************/
#define BT_EVT_TO_LM_HCI_CMD        0x2000      /* HCI Command                      */
#define BT_EVT_TO_LM_HCI_ACL        0x2100      /* HCI ACL Data                     */
#define BT_EVT_TO_LM_HCI_SCO        0x2200      /* HCI SCO Data                     */
#define BT_EVT_TO_LM_HCIT_ERR       0x2300      /* HCI Transport Error              */
#define BT_EVT_TO_LM_LC_EVT         0x2400      /* LC event                         */
#define BT_EVT_TO_LM_LC_LMP         0x2500      /* LC Received LMP command frame    */
#define BT_EVT_TO_LM_LC_ACL         0x2600      /* LC Received ACL data             */
#define BT_EVT_TO_LM_LC_SCO         0x2700      /* LC Received SCO data             */
#define BT_EVT_TO_LM_LC_ACL_TX      0x2800      /* LMP data transmit complete       */
#define BT_EVT_TO_LM_LC_LMPC_TX     0x2900      /* LMP Command transmit complete    */
#define BT_EVT_TO_LM_LOCAL_ACL_LB   0x2a00      /* Data to be locally loopbacked    */
#define BT_EVT_TO_LM_HCI_ACL_ACK    0x2b00      /* HCI ACL Data ack                 */

#define BT_EVT_TO_TCS_CMDS          0x3000

#define BT_EVT_TO_SYNC_SRVR_CMDS    0x3100		/* Sync Server Events */
#define BT_EVT_TO_TELP_CMDS         0x3200
#define BT_EVT_TO_OBEX_LL_CMDS      0x3300      /* obex events from lower layer     */
#define BT_EVT_TO_SYNC_CLNT_CMDS    0x3400		/* Sync Client Events */

/*LAP EVENTS */

#define BT_EVT_TO_BTU_LAP			0x4000

#define BT_EVT_TO_LAP_UL_OPEN_CLI  (0x0001 | BT_EVT_TO_BTU_LAP)  /* PPP receives upper layer open */
#define BT_EVT_TO_LAP_UL_OPEN_SRV  (0x0002 | BT_EVT_TO_BTU_LAP)  /* PPP receives upper layer open */
#define BT_EVT_TO_LAP_UL_CLOSE     (0x0003 | BT_EVT_TO_BTU_LAP)  /* PPP receives upper layer close */
#define BT_EVT_TO_LAP_UL_DATA      (0x0004 | BT_EVT_TO_BTU_LAP)  /* PPP receives upper layer data */
#define BT_EVT_TO_LAP_DEBUG_LVL    (0x0005 | BT_EVT_TO_BTU_LAP)  /* set PPP debug level */
#define BT_EVT_TO_LAP_DEBUG_PRINT  (0x0006 | BT_EVT_TO_BTU_LAP)  /* dump PPP debug info */
#define BT_EVT_TO_LAP_IP_ADDR      (0x0007 | BT_EVT_TO_BTU_LAP)  /* PPP obtains an IP Address */
#define BT_EVT_TO_LAP_UL_SHUTDOWN  (0x0008 | BT_EVT_TO_BTU_LAP)  /* PPP receives upper layer server shutdown */
#define BT_EVT_TO_LAP_UL_INIT_CONN (0x0009 | BT_EVT_TO_BTU_LAP)  /* PPP receives initialization request */
#define BT_EVT_TO_LAP_UL_INIT_IP   (0x000a | BT_EVT_TO_BTU_LAP)  /* PPP receives IP Layer initializations */
#define BT_EVT_TO_LAP_UL_IP_DATA   (0x000b | BT_EVT_TO_BTU_LAP)  /* PPP receives IP Layer data */
#define BT_EVT_TO_LAP_UL_PAP_DATA  (0x000c | BT_EVT_TO_BTU_LAP)  /* PPP receives PAP data */
#define BT_EVT_TO_LAP_AUTH_REP     (0x000d | BT_EVT_TO_BTU_LAP)  /* PPP obtains an authentication reply */
#define BT_EVT_TO_LAP_AUTH_REQ     (0x000e | BT_EVT_TO_BTU_LAP)  /* PPP obtains a repeated authentication request */

/* TCI Events */

#define BT_EVT_BTU_TCI              0x5000

#define BT_EVT_TO_BTU_TCI_CMD      (0x0001 | BT_EVT_BTU_TCI) /* TCI in BTU task recieves command */
#define BT_EVT_TO_BTU_TCI_ACL      (0x0002 | BT_EVT_BTU_TCI) /* TCI in BTU task recieves ACL data */
#define BT_EVT_TO_BTU_TCI_EVT      (0x0003 | BT_EVT_BTU_TCI) /* TCI in BTU task recieves event */
#define BT_EVT_TO_TCI_TCI_CMD      (0x0004 | BT_EVT_BTU_TCI) /* TCI Task recieves command */
#define BT_EVT_TO_TCI_TCI_EVT      (0x0005 | BT_EVT_BTU_TCI) /* TCI Task recieves event */
#define BT_EVT_TO_TCI_TCI_ACL      (0x0006 | BT_EVT_BTU_TCI) /* TCI Task recieves ACL data */
#define BT_EVT_TO_TCI_TX_DONE      (0x0007 | BT_EVT_BTU_TCI) /* TCI transmition to serial port done */
#define BT_EVT_TO_TCI_RX_READY     (0x0008 | BT_EVT_BTU_TCI) /* TCI data at serial port ready to be read */


/* HSP Events */

#define BT_EVT_BTU_HSP2             0x6000
#define BT_EVT_TO_BTU_HSP2_EVT     (0x0001 | BT_EVT_BTU_HSP2)

/* HFP Events */

#define BT_EVT_BTU_HFP              0x8000
#define BT_EVT_TO_BTU_HFP_EVT      (0x0001 | BT_EVT_BTU_HFP)

/* SIM Access Profile Events */
#define BT_EVT_TO_BTU_SAP           0x3a00       /* SIM Access Profile events */

/* Define the header of each buffer used in the Bluetooth stack.
*/
typedef struct {
    UINT16          event;
    UINT16          len;
    UINT16          offset;
    UINT16          layer_specific;
} BT_HDR;

#define BT_HDR_SIZE (sizeof (BT_HDR))

#define BT_PSM_SDP                      0x0001
#define BT_PSM_RFCOMM                   0x0003
#define BT_PSM_TCS                      0x0005
#define BT_PSM_CTP                      0x0007
#define BT_PSM_BNEP                     0x000F
#define BT_PSM_HIDC                     0x0011
#define BT_PSM_HIDI                     0x0013
#define BT_PSM_AVCTP                    0x0017
#define BT_PSM_AVDTP                    0x0019


/* These macros extract the HCI opcodes from a buffer
*/
#define HCI_GET_CMD_HDR_OPCODE(p)    (UINT16)((*((UINT8 *)((p) + 1) + p->offset) + (*((UINT8 *)((p) + 1) + p->offset + 1) << 8)))
#define HCI_GET_CMD_HDR_PARAM_LEN(p) (UINT8)  (*((UINT8 *)((p) + 1) + p->offset + 2))

#define HCI_GET_EVT_HDR_OPCODE(p)    (UINT8)(*((UINT8 *)((p) + 1) + p->offset))
#define HCI_GET_EVT_HDR_PARAM_LEN(p) (UINT8)  (*((UINT8 *)((p) + 1) + p->offset + 1))


/********************************************************************************
** Macros to get and put bytes to and from a stream (Little Endian format).
*/
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (UINT8)(u32); *(p)++ = (UINT8)((u32) >> 8); *(p)++ = (UINT8)((u32) >> 16); *(p)++ = (UINT8)((u32) >> 24); }
#define UINT24_TO_STREAM(p, u24) {*(p)++ = (UINT8)(u24); *(p)++ = (UINT8)((u24) >> 8); *(p)++ = (UINT8)((u24) >> 16); }
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (UINT8)(u16); *(p)++ = (UINT8)((u16) >> 8); }
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (UINT8)(u8); }
#define ARRAY16_TO_STREAM(p, a)  {register int _i; for (_i = 0; _i < 16;           _i++) *(p)++ = (UINT8) a[15 - _i]; }
#define ARRAY8_TO_STREAM(p, a)   {register int _i; for (_i = 0; _i < 8;            _i++) *(p)++ = (UINT8) a[7 - _i]; }
#define BDADDR_TO_STREAM(p, a)   {register int _i; for (_i = 0; _i < BD_ADDR_LEN;  _i++) *(p)++ = (UINT8) a[BD_ADDR_LEN - 1 - _i]; }
#define LAP_TO_STREAM(p, a)      {register int _i; for (_i = 0; _i < LAP_LEN;      _i++) *(p)++ = (UINT8) a[LAP_LEN - 1 - _i]; }
#define DEVCLASS_TO_STREAM(p, a) {register int _i; for (_i = 0; _i < DEV_CLASS_LEN; _i++) *(p)++ = (UINT8) a[DEV_CLASS_LEN - 1 - _i]; }
#define ARRAY_TO_STREAM(p, a, l) {register int _i; for (_i = 0; _i < l;            _i++) *(p)++ = (UINT8) a[_i]; }

#define STREAM_TO_UINT8(u8, p)   {u8 = (UINT8)(*(p)); (p) += 1; }
#define STREAM_TO_UINT16(u16, p) {u16 = ((UINT16)(*(p)) + (((UINT16)(*((p) + 1))) << 8)); (p) += 2; }
#define STREAM_TO_UINT32(u32, p) {u32 = (((UINT32)(*(p))) + ((((UINT32)(*((p) + 1)))) << 8) + ((((UINT32)(*((p) + 2)))) << 16) + ((((UINT32)(*((p) + 3)))) << 24)); (p) += 4; }
#define STREAM_TO_BDADDR(a, p)   {register int _i; register UINT8 *pbda = (UINT8 *)a + BD_ADDR_LEN - 1; for (_i = 0; _i < BD_ADDR_LEN; _i++) *pbda-- = *p++; }
#define STREAM_TO_ARRAY16(a, p)  {register int _i; register UINT8 *_pa = (UINT8 *)a + 15; for (_i = 0; _i < 16; _i++) *_pa-- = *p++; }
#define STREAM_TO_ARRAY8(a, p)   {register int _i; register UINT8 *_pa = (UINT8 *)a + 7; for (_i = 0; _i < 8; _i++) *_pa-- = *p++; }
#define STREAM_TO_DEVCLASS(a, p) {register int _i; register UINT8 *_pa = (UINT8 *)a + DEV_CLASS_LEN - 1; for (_i = 0; _i < DEV_CLASS_LEN; _i++) *_pa-- = *p++; }
#define STREAM_TO_LAP(a, p)      {register int _i; register UINT8 *plap = (UINT8 *)a + LAP_LEN - 1; for (_i = 0; _i < LAP_LEN; _i++) *plap-- = *p++; }
#define STREAM_TO_ARRAY(a, p, l) {register int _i; register UINT8 *_pa = (UINT8 *)a; for (_i = 0; _i < l; _i++) *_pa++ = *p++; }

/********************************************************************************
** Macros to get and put bytes to and from a field (Little Endian format).
** These are the same as to stream, except the pointer is not incremented.
*/
#define UINT32_TO_FIELD(p, u32) {*(UINT8 *)(p) = (UINT8)(u32); *((UINT8 *)(p)+1) = (UINT8)((u32) >> 8); *((UINT8 *)(p)+2) = (UINT8)((u32) >> 16); *((UINT8 *)(p)+3) = (UINT8)((u32) >> 24); }
#define UINT24_TO_FIELD(p, u24) {*(UINT8 *)(p) = (UINT8)(u24); *((UINT8 *)(p)+1) = (UINT8)((u24) >> 8); *((UINT8 *)(p)+2) = (UINT8)((u24) >> 16); }
#define UINT16_TO_FIELD(p, u16) {*(UINT8 *)(p) = (UINT8)(u16); *((UINT8 *)(p)+1) = (UINT8)((u16) >> 8); }
#define UINT8_TO_FIELD(p, u8)   {*(UINT8 *)(p) = (UINT8)(u8); }


/********************************************************************************
** Macros to get and put bytes to and from a stream (Big Endian format)
*/
#define UINT32_TO_BE_STREAM(p, u32) {*(p)++ = (UINT8)((u32) >> 24);  *(p)++ = (UINT8)((u32) >> 16); *(p)++ = (UINT8)((u32) >> 8); *(p)++ = (UINT8)(u32); }
#define UINT24_TO_BE_STREAM(p, u24) {*(p)++ = (UINT8)((u24) >> 16); *(p)++ = (UINT8)((u24) >> 8); *(p)++ = (UINT8)(u24); }
#define UINT16_TO_BE_STREAM(p, u16) {*(p)++ = (UINT8)((u16) >> 8); *(p)++ = (UINT8)(u16); }
#define UINT8_TO_BE_STREAM(p, u8)   {*(p)++ = (UINT8)(u8); }
#define ARRAY_TO_BE_STREAM(p, a, l) {register int _i; for (_i = 0; _i < l; _i++) *p++ = (UINT8) a[_i]; }

#define BE_STREAM_TO_UINT8(u8, p)   {u8 = (UINT8)(*(p)); (p) += 1; }
#define BE_STREAM_TO_UINT16(u16, p) {u16 = (UINT16)(((UINT16)(*(p)) << 8) + (UINT16)(*((p) + 1))); (p) += 2; }
#define BE_STREAM_TO_UINT32(u32, p) {u32 = ((UINT32)(*((p) + 3)) + ((UINT32)(*((p) + 2)) << 8) + ((UINT32)(*((p) + 1)) << 16) + ((UINT32)(*(p)) << 24)); (p) += 4; }
#define BE_STREAM_TO_ARRAY(p, a, l) {register int _i; for (_i = 0; _i < l; _i++) ((UINT8 *) a)[_i] = *p++; }


/********************************************************************************
** Macros to get and put bytes to and from a field (Big Endian format).
** These are the same as to stream, except the pointer is not incremented.
*/
#define UINT32_TO_BE_FIELD(p, u32) {*(UINT8 *)(p) = (UINT8)((u32) >> 24);  *((UINT8 *)(p)+1) = (UINT8)((u32) >> 16); *((UINT8 *)(p)+2) = (UINT8)((u32) >> 8); *((UINT8 *)(p)+3) = (UINT8)(u32); }
#define UINT24_TO_BE_FIELD(p, u24) {*(UINT8 *)(p) = (UINT8)((u24) >> 16); *((UINT8 *)(p)+1) = (UINT8)((u24) >> 8); *((UINT8 *)(p)+2) = (UINT8)(u24); }
#define UINT16_TO_BE_FIELD(p, u16) {*(UINT8 *)(p) = (UINT8)((u16) >> 8); *((UINT8 *)(p)+1) = (UINT8)(u16); }
#define UINT8_TO_BE_FIELD(p, u8)   {*(UINT8 *)(p) = (UINT8)(u8); }


/* Common Bluetooth field definitions
*/
#define BD_ADDR_LEN     6             		/* Device address length */
typedef UINT8 BD_ADDR[BD_ADDR_LEN];			/* Device address */
typedef UINT8 *BD_ADDR_PTR;					/* Pointer to Device Address */

#define LINK_KEY_LEN    16
typedef UINT8 LINK_KEY[LINK_KEY_LEN];       /* Link Key */

#define PIN_CODE_LEN    16
typedef UINT8 PIN_CODE[PIN_CODE_LEN];       /* Pin Code (upto 128 bits) MSB is 0 */
typedef UINT8 *PIN_CODE_PTR;				/* Pointer to Pin Code */

#define DEV_CLASS_LEN   3
typedef UINT8 DEV_CLASS[DEV_CLASS_LEN];     /* Device class */
typedef UINT8 *DEV_CLASS_PTR;				/* Pointer to Device class */

#define BD_NAME_LEN     248
typedef UINT8 BD_NAME[BD_NAME_LEN];         /* Device name */
typedef UINT8 *BD_NAME_PTR;					/* Pointer to Device name */

#define BD_FEATURES_LEN 8
typedef UINT8 BD_FEATURES[BD_FEATURES_LEN]; /* LMP features supported by device */

#define EVENT_MASK_LEN  8
typedef UINT8 EVENT_MASK[EVENT_MASK_LEN];   /* Event Mask */

#define LAP_LEN         3
typedef UINT8 LAP[LAP_LEN];                 /* IAC as passed to Inquiry (LAP) */
typedef UINT8 INQ_LAP[LAP_LEN];				/* IAC as passed to Inquiry (LAP) */

#define RAND_NUM_LEN    16
typedef UINT8 RAND_NUM[RAND_NUM_LEN];

#define ACO_LEN         12
typedef UINT8 ACO[ACO_LEN];                 /* Authenticated ciphering offset */

#define COF_LEN         12
typedef UINT8 COF[COF_LEN];                 /* ciphering offset number */

typedef struct {
	UINT8				qos_flags;			/* TBD */
	UINT8				service_type;		/* see below */
	UINT32				token_rate;			/* bytes/second */
	UINT32				token_bucket_size;	/* bytes */
	UINT32				peak_bandwidth;		/* bytes/second */
	UINT32				latency;			/* microseconds */
	UINT32				delay_variation;	/* microseconds */
} FLOW_SPEC;

/* Values for service_type */
#define NO_TRAFFIC		0
#define BEST_EFFORT		1
#define GUARANTEED		2

/* Service class of the CoD */
#define SERV_CLASS_NETWORKING               (1 << 1)
#define SERV_CLASS_RENDERING                (1 << 2)
#define SERV_CLASS_CAPTURING                (1 << 3)
#define SERV_CLASS_OBJECT_TRANSFER          (1 << 4)
#define SERV_CLASS_OBJECT_AUDIO             (1 << 5)
#define SERV_CLASS_OBJECT_TELEPHONY         (1 << 6)
#define SERV_CLASS_OBJECT_INFORMATION       (1 << 7)

/* Second byte */
#define SERV_CLASS_LIMITED_DISC_MODE        (0x20)

/* Major Device class 5 bits of the second byte */
#define MAJOR_DEV_CLASS_ANY                 0x00
#define MAJOR_DEV_CLASS_MASK                0x1F

#define MAJOR_DEV_CLASS_MISC                0x00
#define MAJOR_DEV_CLASS_COMPUTER            0x01
#define MAJOR_DEV_CLASS_PHONE               0x02
#define MAJOR_DEV_CLASS_LAN_ACCESS          0x03
#define MAJOR_DEV_CLASS_AUDIO               0x04
#define MAJOR_DEV_CLASS_PERIPHERAL          0x05
#define MAJOR_DEV_CLASS_IMAGING             0x06
#define MAJOR_DEV_CLASS_UNSPECIFIED         0x1F

/* Major Device class 6 bits of the second byte */
#define MINOR_DEV_CLASS_ANY                 0x00
#define MINOR_DEV_CLASS_MASK                0xFC

#define MINOR_DEV_CLASS_COMP_UNCLASSIFIED   0x00
#define MINOR_DEV_CLASS_COMP_WORKSTATION    (0x01 << 2)
#define MINOR_DEV_CLASS_COMP_SERVER         (0x02 << 2)
#define MINOR_DEV_CLASS_COMP_LAPTOP         (0x03 << 2)
#define MINOR_DEV_CLASS_COMP_HANDHELD       (0x04 << 2)
#define MINOR_DEV_CLASS_COMP_PALM           (0x05 << 2)

#define MINOR_DEV_CLASS_PHONE_UNCLASSIFIED  0x00
#define MINOR_DEV_CLASS_PHONE_CELLULAR      (0x01 << 2)
#define MINOR_DEV_CLASS_PHONE_CORDLESS      (0x02 << 2)
#define MINOR_DEV_CLASS_PHONE_SMART         (0x03 << 2)
#define MINOR_DEV_CLASS_PHONE_MODEM         (0x04 << 2)

/*minor device class for imaging*/
#define MINOR_DEV_CLASS_IMAGING_UNCLASSIFIED  0x00
#define MINOR_DEV_CLASS_IMAGING_DISPLAY       (0x04 << 2)
#define MINOR_DEV_CLASS_IMAGING_CAMERA        (0x08 << 2)
#define MINOR_DEV_CLASS_IMAGING_SCANNER       (0x10 << 2)
#define MINOR_DEV_CLASS_IMAGING_PRINTER       (0x20 << 2)

#define MINOR_DEV_CLASS_AUDIO_UNCLASSIFIED  0x00

#define MINOR_DEV_CLASS_AUDIO_UNCLASSIFIED  0x00
#define MINOR_DEV_CLASS_AUDIO_HEADSET       (0x01 << 2)
#define MINOR_DEV_CLASS_AUDIO_HANDSFREE     (0x02 << 2)
#define MINOR_DEV_CLASS_AUDIO_MICROPHONE        (0x04 << 2)
#define MINOR_DEV_CLASS_AUDIO_LOUDSPEAKER       (0x05 << 2)
#define MINOR_DEV_CLASS_AUDIO_HEADPHONES    (0x06 << 2)
#define MINOR_DEV_CLASS_AUDIO_PORTABLE_AUDIO    (0x07 << 2)
#define MINOR_DEV_CLASS_AUDIO_CAR_AUDIO         (0x08 << 2)
#define MINOR_DEV_CLASS_AUDIO_SET_TOP_BOX       (0x09 << 2)
#define MINOR_DEV_CLASS_AUDI0_HIFI_DEVICE       (0x0A << 2)
#define MINOR_DEV_CLASS_AUDIO_VCR               (0x0B << 2)
#define MINOR_DEV_CLASS_AUDIO_VIDEO_CAMERA      (0x0C << 2)
#define MINOR_DEV_CLASS_AUDIO_CAMCORDER         (0x0D << 2)
#define MINOR_DEV_CLASS_AUDIO_VIDEO_MONITOR     (0x0E << 2)
#define MINOR_DEV_CLASS_AUDIO_VIDEO_DISPLAY_LOUD (0x0F << 2)
#define MINOR_DEV_CLASS_AUDIO_VIDEO_CONFERENCING (0x10 << 2)
#define MINOR_DEV_CLASS_AUDIO_GAMING_TOY        (0x12 << 2)


/* Minor device class for peripherals */
#define MINOR_DEV_CLASS_PERIPHERAL_UNCLASSIFIED  0x00
#define MINOR_DEV_CLASS_PERIPHERAL_JOYSTICK      (0x01 << 2)
#define MINOR_DEV_CLASS_PERIPHERAL_GAMEPAD       (0x02 << 2)
#define MINOR_DEV_CLASS_PERIPHERAL_REMOTE_CTRL   (0x04 << 2)
#define MINOR_DEV_CLASS_PERIPHERAL_SENSOR        (0x08 << 2)
#define MINOR_DEV_CLASS_PERIPHERAL_KEYBOARD      (0x10 << 2)
#define MINOR_DEV_CLASS_PERIPHERAL_MOUSE         (0x20 << 2)
#define MINOR_DEV_CLASS_PERIPHERAL_KM_COMBO      (MINOR_DEV_CLASS_PERIPHERAL_MOUSE | MINOR_DEV_CLASS_PERIPHERAL_KEYBOARD)

/* FTP definitions
*/
#define FTP_FOLDER_LIST_UTF8     0
#define FTP_FOLDER_LIST_UTF16_1  1
#define FTP_FOLDER_LIST_UTF16_2  2
#define FTP_FOLDER_LIST_UNICODE  3

enum FtpFolder {
    FTP_ROOT_FOLDER =        1,
    FTP_PARENT_FOLDER,
    FTP_SUBFOLDER
};
typedef enum FtpFolder tFtpFolder;


/* Field size definitions. Note that byte lengths are rounded up.
*/
#define ACCESS_CODE_BIT_LEN             72
#define ACCESS_CODE_BYTE_LEN            9
#define SHORTENED_ACCESS_CODE_BIT_LEN   68

typedef UINT8 ACCESS_CODE[ACCESS_CODE_BYTE_LEN];

#define SYNTH_TX				1			/* want synth code to TRANSMIT at this freq */
#define SYNTH_RX				2			/* want synth code to RECEIVE at this freq */

#define SYNC_REPS 1				/* repeats of sync word transmitted to start of burst */

/* Bluetooth CLK27 */
#define BT_CLK27            (2 << 26)

/* Bluetooth CLK12 is 1.28 sec */
#define BT_CLK12_TO_MS(x)    ((x) * 1280)
#define BT_MS_TO_CLK12(x)	 ((x) / 1280)
#define BT_CLK12_TO_SLOTS(x) ((x) << 11)

/* Bluetooth CLK is 0.625 msec */
#define BT_CLK_TO_MS(x)      (((x) * 5 + 3) / 8)
#define BT_MS_TO_CLK(x)      (((x) * 8 + 2) / 5)

#define BT_CLK_TO_MICROSECS(x)  (((x) * 5000 + 3) / 8)
#define BT_MICROSECS_TO_CLK(x)  (((x) * 8 + 2499) / 5000)

/* Maximum UUID size - 16 bytes, and structure to hold any type of UUID.
*/
#define MAX_UUID_SIZE              16
typedef struct {
#define LEN_UUID_16     2
#define LEN_UUID_32     4
#define LEN_UUID_128    16

	UINT16 len;

	union {
		UINT16      uuid16;
		UINT32      uuid32;
		UINT8       uuid128[MAX_UUID_SIZE];
	} uu;
} tBT_UUID;


/* Connection statistics
*/
/*
 Structure to hold connection stats

 SDK defines this struct in BtIfDefinitions.h also
 so make sure we don't step on each other
*/
#ifndef BT_CONN_STATS_DEFINED
#define BT_CONN_STATS_DEFINED
typedef struct {
    UINT32   bIsConnected;
    INT32    Rssi;
    UINT32   BytesSent;
    UINT32   BytesRcvd;
    UINT32   Duration;
} tBT_CONN_STATS;
#endif


/* Define trace levels
*/
#define BT_TRACE_LEVEL_NONE    0            /* No trace messages to be generated    */
#define BT_TRACE_LEVEL_ERROR   1           /* Error condition trace messages       */
#define BT_TRACE_LEVEL_WARNING 2          /* Warning condition trace messages     */
#define BT_TRACE_LEVEL_API     3             /* API traces                           */
#define BT_TRACE_LEVEL_EVENT   4           /* Debug messages for events            */
#define BT_TRACE_LEVEL_DEBUG   5            /* Full debug messages                  */

#define MAX_TRACE_LEVEL         5

/* Define trace message types
*/
#define TRACE_TYPE_GENERIC        0
#define TRACE_TYPE_ERROR          1
#define TRACE_TYPE_HCI_CMD_RECV   2
#define TRACE_TYPE_HCI_CMD_XMIT   3
#define TRACE_TYPE_HCI_EVT_RECV   4
#define TRACE_TYPE_HCI_EVT_XMIT   5
#define TRACE_TYPE_HCI_ACL_RECV   6
#define TRACE_TYPE_HCI_ACL_XMIT   7
#define TRACE_TYPE_LMP_RECV       8
#define TRACE_TYPE_LMP_XMIT       9
#define TRACE_TYPE_TARGET_TRACE   10
#define TRACE_TYPE_L2CAP_RECV     11
#define TRACE_TYPE_L2CAP_XMIT     12
#define TRACE_TYPE_SDP_RECV       13
#define TRACE_TYPE_SDP_XMIT       14
#define TRACE_TYPE_RFCOMM_RECV    15
#define TRACE_TYPE_RFCOMM_XMIT    16
#define TRACE_TYPE_TCS_RECV       17
#define TRACE_TYPE_TCS_XMIT       18
#define TRACE_TYPE_OBEX_RECV      19
#define TRACE_TYPE_OBEX_XMIT      20
#define TRACE_TYPE_OAPP_RECV      21
#define TRACE_TYPE_OAPP_XMIT      22
#define TRACE_TYPE_TCI_RECV		  23
#define TRACE_TYPE_TCI_XMIT       24
#define TRACE_TYPE_BNEP_RECV      25
#define TRACE_TYPE_BNEP_XMIT      26
#define TRACE_TYPE_PPP_RECV       27
#define TRACE_TYPE_PPP_XMIT       28
#define TRACE_TYPE_ALL            100     /* These should always be at the end */
#define TRACE_TYPE_ALL_RECV       101     /* All RECV packet types */
#define TRACE_TYPE_ALL_XMIT       102     /* All XMIT packet types */
/*
 Define this to be one less than the number of subsystems above, because
  we don't actually store a value for the ALLs
*/
#define MAX_TRACE_TYPES             30

/* Define color for script type
*/

#define SCR_COLOR_DEFAULT		0
#define SCR_COLOR_TYPE_COMMENT	1
#define SCR_COLOR_TYPE_COMMAND	2
#define SCR_COLOR_TYPE_EVENT	3
#define SCR_COLOR_TYPE_SELECT	4

/* Define protocol trace flag values
*/
#define SCR_PROTO_TRACE_HCI_SUMMARY	0x0001
#define SCR_PROTO_TRACE_HCI_DATA	0x0002
#define SCR_PROTO_TRACE_L2CAP		0x0004
#define SCR_PROTO_TRACE_RFCOMM		0x0008
#define SCR_PROTO_TRACE_SDP			0x0010
#define SCR_PROTO_TRACE_TCS			0x0020
#define SCR_PROTO_TRACE_OBEX		0x0040
#define SCR_PROTO_TRACE_OAPP        0x0080 /* OBEX Application Profile */
#define SCR_PROTO_TRACE_TCI			0x0100
#define SCR_PROTO_TRACE_BNEP        0x0200
#define SCR_PROTO_TRACE_PPP         0x0400
#define SCR_PROTO_TRACE_AV          0x0800
#define SCR_PROTO_TRACE_HID         0x1000

#define MAX_SCRIPT_TYPE				5


/* Define a function for logging
*/
typedef void (BT_LOG_FUNC) (int trace_type, const char *fmt_str, ...);


/* Define values for Widcomm non-standard features. These may be used
** when the other side is also a Widcomm stack. To enable any feature(s),
** there should be a definition for WIDCOMM_ENHANCED_FEATURES in target.h.
*/
#ifndef L2CAP_COMPRESSION_INCLUDED
#define L2CAP_COMPRESSION_INCLUDED   TRUE
#endif

#define WIDCOMM_FEATURE_REQ_ID      73
#define WIDCOMM_FEATURE_RSP_ID     173

#define L2CAP_FEATURE_REQ_ID     WIDCOMM_FEATURE_REQ_ID
#define L2CAP_FEATURE_RSP_ID     WIDCOMM_FEATURE_RSP_ID

/* Define the supported features. This is a bit map.
*/
#define L2CAP_COMPRESSION         0x0001

#if (L2CAP_COMPRESSION_INCLUDED == TRUE)
#define L2CAP_ENHANCED_FEATURES   L2CAP_COMPRESSION
#endif

/* Define the Protocol Element that describes compression, and the default
** values. These may be overwritten in target.h
*/
#define WIDCOMM_PE_TYPE_COMPRESSION		1
#define WIDCOMM_PE_LEN_COMPRESSION		2
#define L2CAP_PE_TYPE_COMPRESSION		WIDCOMM_PE_TYPE_COMPRESSION

#ifndef L2CAP_CPMR_DEFAULT_MEM_LEVEL
#define L2CAP_CPMR_DEFAULT_MEM_LEVEL	2
#define L2CAP_CPMR_MAX_MEM_LEVEL		8
#define L2CAP_CPMR_DEFAULT_WBITS      10
#define L2CAP_CPMR_MAX_WBITS			15
#endif

/* Mask of enhancd features that are L2CAP related
*/
#define WIDCOMM_FEATURES_L2CAP_MASK     (L2CAP_COMPRESSION)

#endif

