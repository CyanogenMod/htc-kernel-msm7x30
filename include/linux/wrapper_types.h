#ifndef __WRAPPER_TYPES_H_
#define __WRAPPER_TYPES_H_

#define DRIVER_ZONE     "[D:BATT]"
#define DRIVER_BATTERY  (1<<9)

/* The left side of these typedefs are machine and compiler dependent */
typedef signed      char INT8;
typedef unsigned    char UINT8;
typedef signed      short INT16;
typedef unsigned    short UINT16;
typedef signed      int INT32;
typedef unsigned    int UINT32;
typedef struct _INT64{

    UINT32 u0, u1, u2; INT32 u3;
} INT64;
typedef struct _UINT64{

    UINT32 u0, u1, u2, u3;
} UINT64;
typedef struct _INT128{

    UINT32 u0, u1, u2; INT32 u3;
} INT128;
typedef struct _UINT128{

    UINT32 u0, u1, u2, u3;
} UINT128;

typedef INT8 * PINT8;
typedef UINT8 *PUINT8;
typedef INT16 * PINT16;
typedef UINT16 * PUINT16;
typedef INT32 *PINT32;
typedef UINT32 *PUINT32;
typedef INT64 * PINT64;
typedef UINT64 * PUINT64;
typedef INT128 * PINT128;
typedef UINT128 * PUINT128;
typedef const void *PCVOID;
typedef void **PPVOID;
/*typedef unsigned char uchar;*/
typedef void (*PFNVOID)( void);
typedef char CHAR;
typedef UINT8 BYTE;
typedef UINT8 UCHAR;
typedef UINT32 ULONG;
typedef INT32 LONG;
typedef UINT32 DWORD;
typedef UINT32 UINT;
typedef UINT32 BOOL;
typedef UINT16 WORD;
typedef UINT16 USHORT;
typedef INT32 INT;
typedef void *LPVOID;
typedef void *PVOID;
typedef void VOID;
typedef void *HLOCAL;
typedef CHAR * LPCHAR;
typedef CHAR * PBYTE;
typedef INT16 * LPCWSTR;
typedef UINT8 *LPTSTR;
typedef UINT8 *LPCSTR;
typedef UINT8 *LPCTSTR;
typedef BYTE * LPBYTE;
typedef DWORD *LPDWORD;
typedef DWORD *PDWORD;
typedef CHAR TCHAR;
typedef UINT32 HINSTANCE;
typedef UINT32 HKEY;
typedef UINT32 HMODULE;
typedef UINT32 FARPROC;
typedef UINT32 HRESULT;
typedef UINT32 HREGNOTIFY;
typedef UINT32 LPSTARTUPINFO;
typedef UINT32 LPPROCESS_INFORMATION;
typedef UINT32 REGISTRYNOTIFYCALLBACK;
typedef UINT32 NOTIFICATIONCONDITION;
typedef UINT32 REGSAM;
typedef UINT32 PHKEY;

#define FALSE       0
#define TRUE        1
#define INFINITE    0x7fffffffffffffffL

#define MAKEWORD(a, b)      ((WORD)(((BYTE)(a)) | ((WORD)((BYTE)(b))) << 8))
#define LOBYTE(w)           ((BYTE)(w))
#define HIBYTE(w)           ((BYTE)(((WORD)(w) >> 8) & 0xFF))
#define CEILING(n, precision)   ((n+precision-1)/precision)  /* ex: CEILING(989, 10)=99, means 98.9 mapping to 99*/

#endif
