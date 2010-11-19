/*
 * Misc useful os-independent macros and functions.
 *
 * Copyright (C) 1999-2010, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 * $Id: bcmutils.h,v 13.184.4.6.2.1.18.25 2010/04/26 06:05:24 Exp $
 */


#ifndef	_bcmutils_h_
#define	_bcmutils_h_

#ifdef __cplusplus
extern "C" {
#endif


#define _BCM_U	0x01	
#define _BCM_L	0x02	
#define _BCM_D	0x04	
#define _BCM_C	0x08	
#define _BCM_P	0x10	
#define _BCM_S	0x20	
#define _BCM_X	0x40	
#define _BCM_SP	0x80	

extern const unsigned char bcm_ctype[];
#define bcm_ismask(x)	(bcm_ctype[(int)(unsigned char)(x)])

#define bcm_isalnum(c)	((bcm_ismask(c)&(_BCM_U|_BCM_L|_BCM_D)) != 0)
#define bcm_isalpha(c)	((bcm_ismask(c)&(_BCM_U|_BCM_L)) != 0)
#define bcm_iscntrl(c)	((bcm_ismask(c)&(_BCM_C)) != 0)
#define bcm_isdigit(c)	((bcm_ismask(c)&(_BCM_D)) != 0)
#define bcm_isgraph(c)	((bcm_ismask(c)&(_BCM_P|_BCM_U|_BCM_L|_BCM_D)) != 0)
#define bcm_islower(c)	((bcm_ismask(c)&(_BCM_L)) != 0)
#define bcm_isprint(c)	((bcm_ismask(c)&(_BCM_P|_BCM_U|_BCM_L|_BCM_D|_BCM_SP)) != 0)
#define bcm_ispunct(c)	((bcm_ismask(c)&(_BCM_P)) != 0)
#define bcm_isspace(c)	((bcm_ismask(c)&(_BCM_S)) != 0)
#define bcm_isupper(c)	((bcm_ismask(c)&(_BCM_U)) != 0)
#define bcm_isxdigit(c)	((bcm_ismask(c)&(_BCM_D|_BCM_X)) != 0)
#define bcm_tolower(c)	(bcm_isupper((c)) ? ((c) + 'a' - 'A') : (c))
#define bcm_toupper(c)	(bcm_islower((c)) ? ((c) + 'A' - 'a') : (c))



struct bcmstrbuf {
	char *buf;	
	unsigned int size;	
	char *origbuf;	
	unsigned int origsize;	
};


#ifdef BCMDRIVER
#include <osl.h>

#define GPIO_PIN_NOTDEFINED 	0x20	


#define SPINWAIT(exp, us) { \
	uint countdown = (us) + 9; \
	while ((exp) && (countdown >= 10)) {\
		OSL_DELAY(10); \
		countdown -= 10; \
	} \
}



#ifndef PKTQ_LEN_DEFAULT
#define PKTQ_LEN_DEFAULT        128	
#endif
#ifndef PKTQ_MAX_PREC
#define PKTQ_MAX_PREC           16	
#endif

typedef struct pktq_prec {
	void *head;     
	void *tail;     
	uint16 len;     
	uint16 max;     
} pktq_prec_t;



struct pktq {
	uint16 num_prec;        
	uint16 hi_prec;         
	uint16 max;             
	uint16 len;             
	
	struct pktq_prec q[PKTQ_MAX_PREC];
};


struct spktq {
	uint16 num_prec;        
	uint16 hi_prec;         
	uint16 max;             
	uint16 len;             
	
	struct pktq_prec q[1];
};

#define PKTQ_PREC_ITER(pq, prec)        for (prec = (pq)->num_prec - 1; prec >= 0; prec--)




struct ether_addr;

extern int ether_isbcast(const void *ea);
extern int ether_isnulladdr(const void *ea);



#define pktq_psetmax(pq, prec, _max)    ((pq)->q[prec].max = (_max))
#define pktq_plen(pq, prec)             ((pq)->q[prec].len)
#define pktq_pavail(pq, prec)           ((pq)->q[prec].max - (pq)->q[prec].len)
#define pktq_pfull(pq, prec)            ((pq)->q[prec].len >= (pq)->q[prec].max)
#define pktq_pempty(pq, prec)           ((pq)->q[prec].len == 0)

#define pktq_ppeek(pq, prec)            ((pq)->q[prec].head)
#define pktq_ppeek_tail(pq, prec)       ((pq)->q[prec].tail)

extern void *pktq_penq(struct pktq *pq, int prec, void *p);
extern void *pktq_penq_head(struct pktq *pq, int prec, void *p);
extern void *pktq_pdeq(struct pktq *pq, int prec);
extern void *pktq_pdeq_tail(struct pktq *pq, int prec);

extern bool pktq_pdel(struct pktq *pq, void *p, int prec);


extern void pktq_pflush(osl_t *osh, struct pktq *pq, int prec, bool dir);

extern void pktq_flush(osl_t *osh, struct pktq *pq, bool dir);



extern int pktq_mlen(struct pktq *pq, uint prec_bmp);
extern void *pktq_mdeq(struct pktq *pq, uint prec_bmp, int *prec_out);



#define pktq_len(pq)                    ((int)(pq)->len)
#define pktq_max(pq)                    ((int)(pq)->max)
#define pktq_avail(pq)                  ((int)((pq)->max - (pq)->len))
#define pktq_full(pq)                   ((pq)->len >= (pq)->max)
#define pktq_empty(pq)                  ((pq)->len == 0)


#define pktenq(pq, p)		pktq_penq(((struct pktq *)pq), 0, (p))
#define pktenq_head(pq, p)	pktq_penq_head(((struct pktq *)pq), 0, (p))
#define pktdeq(pq)		pktq_pdeq(((struct pktq *)pq), 0)
#define pktdeq_tail(pq)		pktq_pdeq_tail(((struct pktq *)pq), 0)
#define pktqinit(pq, len) pktq_init(((struct pktq *)pq), 1, len)

extern void pktq_init(struct pktq *pq, int num_prec, int max_len);

extern void *pktq_deq(struct pktq *pq, int *prec_out);
extern void *pktq_deq_tail(struct pktq *pq, int *prec_out);
extern void *pktq_peek(struct pktq *pq, int *prec_out);
extern void *pktq_peek_tail(struct pktq *pq, int *prec_out);



extern uint pktcopy(osl_t *osh, void *p, uint offset, int len, uchar *buf);
extern uint pktfrombuf(osl_t *osh, void *p, uint offset, int len, uchar *buf);
extern uint pkttotlen(osl_t *osh, void *p);
extern void *pktlast(osl_t *osh, void *p);
extern uint pktsegcnt(osl_t *osh, void *p);


extern uint pktsetprio(void *pkt, bool update_vtag);
#define	PKTPRIO_VDSCP	0x100		
#define	PKTPRIO_VLAN	0x200		
#define	PKTPRIO_UPD	0x400		
#define	PKTPRIO_DSCP	0x800		


extern int bcm_atoi(char *s);
extern ulong bcm_strtoul(char *cp, char **endp, uint base);
extern char *bcmstrstr(char *haystack, char *needle);
extern char *bcmstrcat(char *dest, const char *src);
extern char *bcmstrncat(char *dest, const char *src, uint size);
extern ulong wchar2ascii(char *abuf, ushort *wbuf, ushort wbuflen, ulong abuflen);
char* bcmstrtok(char **string, const char *delimiters, char *tokdelim);
int bcmstricmp(const char *s1, const char *s2);
int bcmstrnicmp(const char* s1, const char* s2, int cnt);



extern char *bcm_ether_ntoa(const struct ether_addr *ea, char *buf);
extern int bcm_ether_atoe(char *p, struct ether_addr *ea);


struct ipv4_addr;
extern char *bcm_ip_ntoa(struct ipv4_addr *ia, char *buf);


extern void bcm_mdelay(uint ms);

extern char *getvar(char *vars, const char *name);
extern int getintvar(char *vars, const char *name);
extern uint getgpiopin(char *vars, char *pin_name, uint def_pin);
#define bcm_perf_enable()
#define bcmstats(fmt)
#define	bcmlog(fmt, a1, a2)
#define	bcmdumplog(buf, size)	*buf = '\0'
#define	bcmdumplogent(buf, idx)	-1

#define bcmtslog(tstamp, fmt, a1, a2)
#define bcmprinttslogs()
#define bcmprinttstamp(us)




typedef struct bcm_iovar {
	const char *name;	
	uint16 varid;		
	uint16 flags;		
	uint16 type;		
	uint16 minlen;		
} bcm_iovar_t;




#define IOV_GET 0 
#define IOV_SET 1 


#define IOV_GVAL(id)		((id)*2)
#define IOV_SVAL(id)		(((id)*2)+IOV_SET)
#define IOV_ISSET(actionid)	((actionid & IOV_SET) == IOV_SET)



extern const bcm_iovar_t *bcm_iovar_lookup(const bcm_iovar_t *table, const char *name);
extern int bcm_iovar_lencheck(const bcm_iovar_t *table, void *arg, int len, bool set);

#endif	


#define IOVT_VOID	0	
#define IOVT_BOOL	1	
#define IOVT_INT8	2	
#define IOVT_UINT8	3	
#define IOVT_INT16	4	
#define IOVT_UINT16	5	
#define IOVT_INT32	6	
#define IOVT_UINT32	7	
#define IOVT_BUFFER	8	
#define BCM_IOVT_VALID(type) (((unsigned int)(type)) <= IOVT_BUFFER)


#define BCM_IOV_TYPE_INIT { \
	"void", \
	"bool", \
	"int8", \
	"uint8", \
	"int16", \
	"uint16", \
	"int32", \
	"uint32", \
	"buffer", \
	"" }

#define BCM_IOVT_IS_INT(type) (\
	(type == IOVT_BOOL) || \
	(type == IOVT_INT8) || \
	(type == IOVT_UINT8) || \
	(type == IOVT_INT16) || \
	(type == IOVT_UINT16) || \
	(type == IOVT_INT32) || \
	(type == IOVT_UINT32))



#define BCME_STRLEN 		64	
#define VALID_BCMERROR(e)  ((e <= 0) && (e >= BCME_LAST))




#define BCME_OK				0	
#define BCME_ERROR			-1	
#define BCME_BADARG			-2	
#define BCME_BADOPTION			-3	
#define BCME_NOTUP			-4	
#define BCME_NOTDOWN			-5	
#define BCME_NOTAP			-6	
#define BCME_NOTSTA			-7	
#define BCME_BADKEYIDX			-8	
#define BCME_RADIOOFF 			-9	
#define BCME_NOTBANDLOCKED		-10	
#define BCME_NOCLK			-11	
#define BCME_BADRATESET			-12	
#define BCME_BADBAND			-13	
#define BCME_BUFTOOSHORT		-14	
#define BCME_BUFTOOLONG			-15	
#define BCME_BUSY			-16	
#define BCME_NOTASSOCIATED		-17	
#define BCME_BADSSIDLEN			-18	
#define BCME_OUTOFRANGECHAN		-19	
#define BCME_BADCHAN			-20	
#define BCME_BADADDR			-21	
#define BCME_NORESOURCE			-22	
#define BCME_UNSUPPORTED		-23	
#define BCME_BADLEN			-24	
#define BCME_NOTREADY			-25	
#define BCME_EPERM			-26	
#define BCME_NOMEM			-27	
#define BCME_ASSOCIATED			-28	
#define BCME_RANGE			-29	
#define BCME_NOTFOUND			-30	
#define BCME_WME_NOT_ENABLED		-31	
#define BCME_TSPEC_NOTFOUND		-32	
#define BCME_ACM_NOTSUPPORTED		-33	
#define BCME_NOT_WME_ASSOCIATION	-34	
#define BCME_SDIO_ERROR			-35	
#define BCME_DONGLE_DOWN		-36	
#define BCME_VERSION			-37	
#define BCME_TXFAIL			-38	
#define BCME_RXFAIL			-39	
#define BCME_NODEVICE			-40	
#define BCME_UNFINISHED			-41	
#define BCME_LAST			BCME_UNFINISHED


#define BCMERRSTRINGTABLE {		\
	"OK",				\
	"Undefined error",		\
	"Bad Argument",			\
	"Bad Option",			\
	"Not up",			\
	"Not down",			\
	"Not AP",			\
	"Not STA",			\
	"Bad Key Index",		\
	"Radio Off",			\
	"Not band locked",		\
	"No clock",			\
	"Bad Rate valueset",		\
	"Bad Band",			\
	"Buffer too short",		\
	"Buffer too long",		\
	"Busy",				\
	"Not Associated",		\
	"Bad SSID len",			\
	"Out of Range Channel",		\
	"Bad Channel",			\
	"Bad Address",			\
	"Not Enough Resources",		\
	"Unsupported",			\
	"Bad length",			\
	"Not Ready",			\
	"Not Permitted",		\
	"No Memory",			\
	"Associated",			\
	"Not In Range",			\
	"Not Found",			\
	"WME Not Enabled",		\
	"TSPEC Not Found",		\
	"ACM Not Supported",		\
	"Not WME Association",		\
	"SDIO Bus Error",		\
	"Dongle Not Accessible",	\
	"Incorrect version",		\
	"TX Failure",			\
	"RX Failure",			\
	"Device Not Present",		\
	"Command not finished",		\
}

#ifndef ABS
#define	ABS(a)			(((a) < 0)?-(a):(a))
#endif 

#ifndef MIN
#define	MIN(a, b)		(((a) < (b))?(a):(b))
#endif 

#ifndef MAX
#define	MAX(a, b)		(((a) > (b))?(a):(b))
#endif 

#define CEIL(x, y)		(((x) + ((y)-1)) / (y))
#define	ROUNDUP(x, y)		((((x)+((y)-1))/(y))*(y))
#define	ISALIGNED(a, x)		(((a) & ((x)-1)) == 0)
#define ALIGN_ADDR(addr, boundary) (void *)(((uintptr)(addr) + (boundary) - 1) \
	                                         & ~((boundary) - 1))
#define	ISPOWEROF2(x)		((((x)-1)&(x)) == 0)
#define VALID_MASK(mask)	!((mask) & ((mask) + 1))
#ifndef OFFSETOF
#define	OFFSETOF(type, member)	((uint)(uintptr)&((type *)0)->member)
#endif 
#ifndef ARRAYSIZE
#define ARRAYSIZE(a)		(sizeof(a)/sizeof(a[0]))
#endif


#ifndef setbit
#ifndef NBBY		      
#define	NBBY	8	
#endif 
#define	setbit(a, i)	(((uint8 *)a)[(i)/NBBY] |= 1<<((i)%NBBY))
#define	clrbit(a, i)	(((uint8 *)a)[(i)/NBBY] &= ~(1<<((i)%NBBY)))
#define	isset(a, i)	(((const uint8 *)a)[(i)/NBBY] & (1<<((i)%NBBY)))
#define	isclr(a, i)	((((const uint8 *)a)[(i)/NBBY] & (1<<((i)%NBBY))) == 0)
#endif 

#define	NBITS(type)	(sizeof(type) * 8)
#define NBITVAL(nbits)	(1 << (nbits))
#define MAXBITVAL(nbits)	((1 << (nbits)) - 1)
#define	NBITMASK(nbits)	MAXBITVAL(nbits)
#define MAXNBVAL(nbyte)	MAXBITVAL((nbyte) * 8)


#define MUX(pred, true, false) ((pred) ? (true) : (false))


#define MODDEC(x, bound) MUX((x) == 0, (bound) - 1, (x) - 1)
#define MODINC(x, bound) MUX((x) == (bound) - 1, 0, (x) + 1)


#define MODDEC_POW2(x, bound) (((x) - 1) & ((bound) - 1))
#define MODINC_POW2(x, bound) (((x) + 1) & ((bound) - 1))


#define MODADD(x, y, bound) \
    MUX((x) + (y) >= (bound), (x) + (y) - (bound), (x) + (y))
#define MODSUB(x, y, bound) \
    MUX(((int)(x)) - ((int)(y)) < 0, (x) - (y) + (bound), (x) - (y))


#define MODADD_POW2(x, y, bound) (((x) + (y)) & ((bound) - 1))
#define MODSUB_POW2(x, y, bound) (((x) - (y)) & ((bound) - 1))


#define CRC8_INIT_VALUE  0xff		
#define CRC8_GOOD_VALUE  0x9f		
#define CRC16_INIT_VALUE 0xffff		
#define CRC16_GOOD_VALUE 0xf0b8		
#define CRC32_INIT_VALUE 0xffffffff	
#define CRC32_GOOD_VALUE 0xdebb20e3	


typedef struct bcm_bit_desc {
	uint32	bit;
	const char* name;
} bcm_bit_desc_t;


typedef struct bcm_tlv {
	uint8	id;
	uint8	len;
	uint8	data[1];
} bcm_tlv_t;


#define bcm_valid_tlv(elt, buflen) ((buflen) >= 2 && (int)(buflen) >= (int)(2 + (elt)->len))


#define ETHER_ADDR_STR_LEN	18	


#ifdef IL_BIGENDIAN
static INLINE uint32
load32_ua(uint8 *a)
{
	return ((a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3]);
}

static INLINE void
store32_ua(uint8 *a, uint32 v)
{
	a[0] = (v >> 24) & 0xff;
	a[1] = (v >> 16) & 0xff;
	a[2] = (v >> 8) & 0xff;
	a[3] = v & 0xff;
}

static INLINE uint16
load16_ua(uint8 *a)
{
	return ((a[0] << 8) | a[1]);
}

static INLINE void
store16_ua(uint8 *a, uint16 v)
{
	a[0] = (v >> 8) & 0xff;
	a[1] = v & 0xff;
}

#else 

static INLINE uint32
load32_ua(uint8 *a)
{
	return ((a[3] << 24) | (a[2] << 16) | (a[1] << 8) | a[0]);
}

static INLINE void
store32_ua(uint8 *a, uint32 v)
{
	a[3] = (v >> 24) & 0xff;
	a[2] = (v >> 16) & 0xff;
	a[1] = (v >> 8) & 0xff;
	a[0] = v & 0xff;
}

static INLINE uint16
load16_ua(uint8 *a)
{
	return ((a[1] << 8) | a[0]);
}

static INLINE void
store16_ua(uint8 *a, uint16 v)
{
	a[1] = (v >> 8) & 0xff;
	a[0] = v & 0xff;
}

#endif 



static INLINE void
xor_128bit_block(const uint8 *src1, const uint8 *src2, uint8 *dst)
{
	if (
#ifdef __i386__
	    1 ||
#endif
	    (((uintptr)src1 | (uintptr)src2 | (uintptr)dst) & 3) == 0) {
		
		
		((uint32 *)dst)[0] = ((uint32 *)src1)[0] ^ ((uint32 *)src2)[0];
		((uint32 *)dst)[1] = ((uint32 *)src1)[1] ^ ((uint32 *)src2)[1];
		((uint32 *)dst)[2] = ((uint32 *)src1)[2] ^ ((uint32 *)src2)[2];
		((uint32 *)dst)[3] = ((uint32 *)src1)[3] ^ ((uint32 *)src2)[3];
	} else {
		
		int k;
		for (k = 0; k < 16; k++)
			dst[k] = src1[k] ^ src2[k];
	}
}



extern uint8 hndcrc8(uint8 *p, uint nbytes, uint8 crc);
extern uint16 hndcrc16(uint8 *p, uint nbytes, uint16 crc);
extern uint32 hndcrc32(uint8 *p, uint nbytes, uint32 crc);

#if defined(DHD_DEBUG) || defined(WLMSG_PRHDRS) || defined(WLMSG_PRPKT) || \
	defined(WLMSG_ASSOC)
extern int bcm_format_flags(const bcm_bit_desc_t *bd, uint32 flags, char* buf, int len);
extern int bcm_format_hex(char *str, const void *bytes, int len);
extern void prhex(const char *msg, uchar *buf, uint len);
#endif 
extern char *bcm_brev_str(uint32 brev, char *buf);
extern void printbig(char *buf);


extern bcm_tlv_t *bcm_next_tlv(bcm_tlv_t *elt, int *buflen);
extern bcm_tlv_t *bcm_parse_tlvs(void *buf, int buflen, uint key);
extern bcm_tlv_t *bcm_parse_ordered_tlvs(void *buf, int buflen, uint key);


extern const char *bcmerrorstr(int bcmerror);


typedef uint32 mbool;
#define mboolset(mb, bit)		((mb) |= (bit))		
#define mboolclr(mb, bit)		((mb) &= ~(bit))	
#define mboolisset(mb, bit)		(((mb) & (bit)) != 0)	
#define	mboolmaskset(mb, mask, val)	((mb) = (((mb) & ~(mask)) | (val)))


extern uint16 bcm_qdbm_to_mw(uint8 qdbm);
extern uint8 bcm_mw_to_qdbm(uint16 mw);


struct fielddesc {
	const char *nameandfmt;
	uint32 	offset;
	uint32 	len;
};

extern void bcm_binit(struct bcmstrbuf *b, char *buf, uint size);
extern int bcm_bprintf(struct bcmstrbuf *b, const char *fmt, ...);
extern void bcm_inc_bytes(uchar *num, int num_bytes, uint8 amount);
extern int bcm_cmp_bytes(uchar *arg1, uchar *arg2, uint8 nbytes);
extern void bcm_print_bytes(char *name, const uchar *cdata, int len);

typedef  uint32 (*bcmutl_rdreg_rtn)(void *arg0, uint arg1, uint32 offset);
extern uint bcmdumpfields(bcmutl_rdreg_rtn func_ptr, void *arg0, uint arg1, struct fielddesc *str,
                          char *buf, uint32 bufsize);

extern uint bcm_mkiovar(char *name, char *data, uint datalen, char *buf, uint len);
extern uint bcm_bitcount(uint8 *bitmap, uint bytelength);

#if defined(WLTINYDUMP) || defined(WLMSG_INFORM) || defined(WLMSG_ASSOC) || \
	defined(WLMSG_PRPKT) || defined(WLMSG_WSEC)
extern int bcm_format_ssid(char* buf, const uchar ssid[], uint ssid_len);
#endif 


#define SSID_FMT_BUF_LEN	((4 * DOT11_MAX_SSID_LEN) + 1)

#ifdef __cplusplus
	}
#endif

#endif	
