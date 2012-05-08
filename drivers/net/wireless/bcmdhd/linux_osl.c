/*
 * Linux OS Independent Layer
 *
 * Copyright (C) 1999-2011, Broadcom Corporation
 * 
 *         Unless you and Broadcom execute a separate written software license
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
 *
 * $Id: linux_osl.c 281175 2011-09-01 09:46:46Z $
 */


#define LINUX_PORT

#include <typedefs.h>
#include <bcmendian.h>
#include <linuxver.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <linux/delay.h>
#include <pcicfg.h>

#ifdef BCMASSERT_LOG
#include <bcm_assert_log.h>
#endif

#include <linux/fs.h>

#define PCI_CFG_RETRY 		10

#define OS_HANDLE_MAGIC		0x1234abcd	
#define BCM_MEM_FILENAME_LEN 	24		

#ifdef DHD_USE_STATIC_BUF
#define STATIC_BUF_MAX_NUM	16
#define STATIC_BUF_SIZE		(PAGE_SIZE * 2)
#define STATIC_BUF_TOTAL_LEN	(STATIC_BUF_MAX_NUM * STATIC_BUF_SIZE)

typedef struct bcm_static_buf {
	struct semaphore static_sem;
	unsigned char *buf_ptr;
	unsigned char buf_use[STATIC_BUF_MAX_NUM];
} bcm_static_buf_t;

static bcm_static_buf_t *bcm_static_buf = 0;

#define STATIC_PKT_MAX_NUM	8

typedef struct bcm_static_pkt {
	struct sk_buff *skb_4k[STATIC_PKT_MAX_NUM];
	struct sk_buff *skb_8k[STATIC_PKT_MAX_NUM];
	struct semaphore osl_pkt_sem;
	unsigned char pkt_use[STATIC_PKT_MAX_NUM * 2];
} bcm_static_pkt_t;

static bcm_static_pkt_t *bcm_static_skb = 0;
#endif 

typedef struct bcm_mem_link {
	struct bcm_mem_link *prev;
	struct bcm_mem_link *next;
	uint	size;
	int	line;
	char	file[BCM_MEM_FILENAME_LEN];
} bcm_mem_link_t;

struct osl_info {
	osl_pubinfo_t pub;
#ifdef CTFPOOL
	ctfpool_t *ctfpool;
#endif 
	uint magic;
	void *pdev;
	atomic_t malloced;
	uint failed;
	uint bustype;
	bcm_mem_link_t *dbgmem_list;
};




uint32 g_assert_type = FALSE;

static int16 linuxbcmerrormap[] =
{	0, 			
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL,		
	-EINVAL, 		
	-EINVAL, 		
	-EINVAL, 		
	-E2BIG,			
	-E2BIG,			
	-EBUSY, 		
	-EINVAL, 		
	-EINVAL, 		
	-EINVAL, 		
	-EINVAL, 		
	-EFAULT, 		
	-ENOMEM, 		
	-EOPNOTSUPP,		
	-EMSGSIZE,		
	-EINVAL,		
	-EPERM,			
	-ENOMEM, 		
	-EINVAL, 		
	-ERANGE, 		
	-EINVAL, 		
	-EINVAL, 		
	-EINVAL, 		
	-EINVAL, 		
	-EINVAL,		
	-EIO,			
	-ENODEV,		
	-EINVAL,		
	-EIO,			
	-EIO,			
	-ENODEV,		
	-EINVAL,		
	-ENODATA,		



#if BCME_LAST != -42
#error "You need to add a OS error translation in the linuxbcmerrormap \
	for new error code defined in bcmutils.h"
#endif
};


int
osl_error(int bcmerror)
{
	if (bcmerror > 0)
		bcmerror = 0;
	else if (bcmerror < BCME_LAST)
		bcmerror = BCME_ERROR;

	
	return linuxbcmerrormap[-bcmerror];
}

extern uint8* dhd_os_prealloc(void *osh, int section, int size);

osl_t *
osl_attach(void *pdev, uint bustype, bool pkttag)
{
	osl_t *osh;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25))
	gfp_t flags;

	flags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	osh = kmalloc(sizeof(osl_t), flags);
#else
	osh = kmalloc(sizeof(osl_t), GFP_ATOMIC);
#endif 
	ASSERT(osh);
#ifdef HTC_KlocWork
    if(osh == NULL) {
        printf("[HTCKW] osl_attach: osh == NULL\n");
        return NULL;
    }
#endif
	bzero(osh, sizeof(osl_t));

	
	ASSERT(ABS(BCME_LAST) == (ARRAYSIZE(linuxbcmerrormap) - 1));

	osh->magic = OS_HANDLE_MAGIC;
	atomic_set(&osh->malloced, 0);
	osh->failed = 0;
	osh->dbgmem_list = NULL;
	osh->pdev = pdev;
	osh->pub.pkttag = pkttag;
	osh->bustype = bustype;

	switch (bustype) {
		case PCI_BUS:
		case SI_BUS:
		case PCMCIA_BUS:
			osh->pub.mmbus = TRUE;
			break;
		case JTAG_BUS:
		case SDIO_BUS:
		case USB_BUS:
		case SPI_BUS:
		case RPC_BUS:
			osh->pub.mmbus = FALSE;
			break;
		default:
			ASSERT(FALSE);
			break;
	}

#if defined(DHD_USE_STATIC_BUF)
	if (!bcm_static_buf) {
		if (!(bcm_static_buf = (bcm_static_buf_t *)dhd_os_prealloc(osh, 3, STATIC_BUF_SIZE+
			STATIC_BUF_TOTAL_LEN))) {
			printf("can not alloc static buf!\n");
#ifdef HTC_KlocWork
			return NULL;
#endif
		}
		else
			printf("alloc static buf at %x!\n", (unsigned int)bcm_static_buf);


		sema_init(&bcm_static_buf->static_sem, 1);

		bcm_static_buf->buf_ptr = (unsigned char *)bcm_static_buf + STATIC_BUF_SIZE;
	}

	if (!bcm_static_skb) {
		int i;
		void *skb_buff_ptr = 0;
		bcm_static_skb = (bcm_static_pkt_t *)((char *)bcm_static_buf + 2048);
		skb_buff_ptr = dhd_os_prealloc(osh, 4, 0);

#ifdef HTC_KlocWork
    if(skb_buff_ptr != NULL)
#endif
		bcopy(skb_buff_ptr, bcm_static_skb, sizeof(struct sk_buff *) * 16);
		for (i = 0; i < STATIC_PKT_MAX_NUM * 2; i++)
			bcm_static_skb->pkt_use[i] = 0;

		sema_init(&bcm_static_skb->osl_pkt_sem, 1);
	}
#endif 

	return osh;
}

void
osl_detach(osl_t *osh)
{
	if (osh == NULL)
		return;

	ASSERT(osh->magic == OS_HANDLE_MAGIC);
	kfree(osh);
}

static struct sk_buff *osl_alloc_skb(unsigned int len)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)
	gfp_t flags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;

	return __dev_alloc_skb(len, flags);
#else
	return dev_alloc_skb(len);
#endif
}

#ifdef CTFPOOL

void *
osl_ctfpool_add(osl_t *osh)
{
	struct sk_buff *skb;

	if ((osh == NULL) || (osh->ctfpool == NULL))
		return NULL;

	spin_lock_bh(&osh->ctfpool->lock);
	ASSERT(osh->ctfpool->curr_obj <= osh->ctfpool->max_obj);

	
	if (osh->ctfpool->curr_obj == osh->ctfpool->max_obj) {
		spin_unlock_bh(&osh->ctfpool->lock);
		return NULL;
	}

	
	skb = osl_alloc_skb(osh->ctfpool->obj_size);
	if (skb == NULL) {
		printf("%s: skb alloc of len %d failed\n", __FUNCTION__,
		       osh->ctfpool->obj_size);
		spin_unlock_bh(&osh->ctfpool->lock);
		return NULL;
	}

	
	skb->next = (struct sk_buff *)osh->ctfpool->head;
	osh->ctfpool->head = skb;
	osh->ctfpool->fast_frees++;
	osh->ctfpool->curr_obj++;

	
	CTFPOOLPTR(osh, skb) = (void *)osh->ctfpool;

	
	PKTFAST(osh, skb) = FASTBUF;

	spin_unlock_bh(&osh->ctfpool->lock);

	return skb;
}


void
osl_ctfpool_replenish(osl_t *osh, uint thresh)
{
	if ((osh == NULL) || (osh->ctfpool == NULL))
		return;

	
	while ((osh->ctfpool->refills > 0) && (thresh--)) {
		osl_ctfpool_add(osh);
		osh->ctfpool->refills--;
	}
}


int32
osl_ctfpool_init(osl_t *osh, uint numobj, uint size)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25))
	gfp_t flags;

	flags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	osh->ctfpool = kmalloc(sizeof(ctfpool_t), flags);
#else
	osh->ctfpool = kmalloc(sizeof(ctfpool_t), GFP_ATOMIC);
#endif 
	ASSERT(osh->ctfpool);
	bzero(osh->ctfpool, sizeof(ctfpool_t));

	osh->ctfpool->max_obj = numobj;
	osh->ctfpool->obj_size = size;

	spin_lock_init(&osh->ctfpool->lock);

	while (numobj--) {
		if (!osl_ctfpool_add(osh))
			return -1;
		osh->ctfpool->fast_frees--;
	}

	return 0;
}


void
osl_ctfpool_cleanup(osl_t *osh)
{
	struct sk_buff *skb, *nskb;

	if ((osh == NULL) || (osh->ctfpool == NULL))
		return;

	spin_lock_bh(&osh->ctfpool->lock);

	skb = osh->ctfpool->head;

	while (skb != NULL) {
		nskb = skb->next;
		dev_kfree_skb(skb);
		skb = nskb;
		osh->ctfpool->curr_obj--;
	}

	ASSERT(osh->ctfpool->curr_obj == 0);
	osh->ctfpool->head = NULL;
	spin_unlock_bh(&osh->ctfpool->lock);

	kfree(osh->ctfpool);
	osh->ctfpool = NULL;
}

void
osl_ctfpool_stats(osl_t *osh, void *b)
{
	struct bcmstrbuf *bb;

	if ((osh == NULL) || (osh->ctfpool == NULL))
		return;

#ifdef DHD_USE_STATIC_BUF
	if (bcm_static_buf) {
		bcm_static_buf = 0;
	}
	if (bcm_static_skb) {
		bcm_static_skb = 0;
	}
#endif 

	bb = b;

	ASSERT((osh != NULL) && (bb != NULL));

	bcm_bprintf(bb, "max_obj %d obj_size %d curr_obj %d refills %d\n",
	            osh->ctfpool->max_obj, osh->ctfpool->obj_size,
	            osh->ctfpool->curr_obj, osh->ctfpool->refills);
	bcm_bprintf(bb, "fast_allocs %d fast_frees %d slow_allocs %d\n",
	            osh->ctfpool->fast_allocs, osh->ctfpool->fast_frees,
	            osh->ctfpool->slow_allocs);
}

static inline struct sk_buff *
osl_pktfastget(osl_t *osh, uint len)
{
	struct sk_buff *skb;

	
	if (osh->ctfpool == NULL)
		return NULL;

	spin_lock_bh(&osh->ctfpool->lock);
	if (osh->ctfpool->head == NULL) {
		ASSERT(osh->ctfpool->curr_obj == 0);
		osh->ctfpool->slow_allocs++;
		spin_unlock_bh(&osh->ctfpool->lock);
		return NULL;
	}

	ASSERT(len <= osh->ctfpool->obj_size);

	
	skb = (struct sk_buff *)osh->ctfpool->head;
	osh->ctfpool->head = (void *)skb->next;

	osh->ctfpool->fast_allocs++;
	osh->ctfpool->curr_obj--;
	ASSERT(CTFPOOLHEAD(osh, skb) == (struct sock *)osh->ctfpool->head);
	spin_unlock_bh(&osh->ctfpool->lock);

	
	skb->next = skb->prev = NULL;
	skb->data = skb->head + 16;
	skb->tail = skb->head + 16;

	skb->len = 0;
	skb->cloned = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14)
	skb->list = NULL;
#endif
	atomic_set(&skb->users, 1);

	return skb;
}
#endif 


void * BCMFASTPATH
osl_pktget(osl_t *osh, uint len)
{
	struct sk_buff *skb;

#ifdef CTFPOOL
	
	skb = osl_pktfastget(osh, len);
	if ((skb != NULL) || ((skb = osl_alloc_skb(len)) != NULL)) {
#else 
	if ((skb = osl_alloc_skb(len))) {
#endif 
		skb_put(skb, len);
		skb->priority = 0;


		osh->pub.pktalloced++;
	}

	return ((void*) skb);
}

#ifdef CTFPOOL
static inline void
osl_pktfastfree(osl_t *osh, struct sk_buff *skb)
{
	ctfpool_t *ctfpool;

	ctfpool = (ctfpool_t *)CTFPOOLPTR(osh, skb);
	ASSERT(ctfpool != NULL);

	
	spin_lock_bh(&ctfpool->lock);
	skb->next = (struct sk_buff *)ctfpool->head;
	ctfpool->head = (void *)skb;

	ctfpool->fast_frees++;
	ctfpool->curr_obj++;

	ASSERT(ctfpool->curr_obj <= ctfpool->max_obj);
	spin_unlock_bh(&ctfpool->lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 14)
	skb->tstamp.tv.sec = 0;
#else
	skb->stamp.tv_sec = 0;
#endif

	
	skb->dev = NULL;
	skb->dst = NULL;
	memset(skb->cb, 0, sizeof(skb->cb));
	skb->ip_summed = 0;
	skb->destructor = NULL;
}
#endif 


void BCMFASTPATH
osl_pktfree(osl_t *osh, void *p, bool send)
{
	struct sk_buff *skb, *nskb;

	skb = (struct sk_buff*) p;

	if (send && osh->pub.tx_fn)
		osh->pub.tx_fn(osh->pub.tx_ctx, p, 0);

	
	while (skb) {
		nskb = skb->next;
		skb->next = NULL;


#ifdef CTFPOOL
		if (PKTISFAST(osh, skb))
			osl_pktfastfree(osh, skb);
		else {
#else 
		{
#endif 

			if (skb->destructor)
				
				dev_kfree_skb_any(skb);
			else
				
				dev_kfree_skb(skb);
		}

		osh->pub.pktalloced--;

		skb = nskb;
	}
}

#ifdef DHD_USE_STATIC_BUF
void *
osl_pktget_static(osl_t *osh, uint len)
{
	int i;
	struct sk_buff *skb;

	if (len > (PAGE_SIZE * 2)) {
		printf("%s: attempt to allocate huge packet (0x%x)\n", __FUNCTION__, len);
		return osl_pktget(osh, len);
	}

	down(&bcm_static_skb->osl_pkt_sem);

	if (len <= PAGE_SIZE) {
		for (i = 0; i < STATIC_PKT_MAX_NUM; i++) {
			if (bcm_static_skb->pkt_use[i] == 0)
				break;
		}

		if (i != STATIC_PKT_MAX_NUM) {
			bcm_static_skb->pkt_use[i] = 1;
			up(&bcm_static_skb->osl_pkt_sem);
			skb = bcm_static_skb->skb_4k[i];
			skb->tail = skb->data + len;
			skb->len = len;
			return skb;
		}
	}


	for (i = 0; i < STATIC_PKT_MAX_NUM; i++) {
		if (bcm_static_skb->pkt_use[i+STATIC_PKT_MAX_NUM] == 0)
			break;
	}

	if (i != STATIC_PKT_MAX_NUM) {
		bcm_static_skb->pkt_use[i+STATIC_PKT_MAX_NUM] = 1;
		up(&bcm_static_skb->osl_pkt_sem);
		skb = bcm_static_skb->skb_8k[i];
		skb->tail = skb->data + len;
		skb->len = len;
		return skb;
	}

	up(&bcm_static_skb->osl_pkt_sem);
	printf("%s: all static pkt in use!\n", __FUNCTION__);
	return osl_pktget(osh, len);
}

void
osl_pktfree_static(osl_t *osh, void *p, bool send)
{
	int i;

	for (i = 0; i < STATIC_PKT_MAX_NUM; i++) {
		if (p == bcm_static_skb->skb_4k[i]) {
			down(&bcm_static_skb->osl_pkt_sem);
			bcm_static_skb->pkt_use[i] = 0;
			up(&bcm_static_skb->osl_pkt_sem);
			return;
		}
	}

	for (i = 0; i < STATIC_PKT_MAX_NUM; i++) {
		if (p == bcm_static_skb->skb_8k[i]) {
			down(&bcm_static_skb->osl_pkt_sem);
			bcm_static_skb->pkt_use[i + STATIC_PKT_MAX_NUM] = 0;
			up(&bcm_static_skb->osl_pkt_sem);
			return;
		}
	}

	return osl_pktfree(osh, p, send);
}
#endif 

uint32
osl_pci_read_config(osl_t *osh, uint offset, uint size)
{
	uint val = 0;
	uint retry = PCI_CFG_RETRY;

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	
	ASSERT(size == 4);

	do {
		pci_read_config_dword(osh->pdev, offset, &val);
		if (val != 0xffffffff)
			break;
	} while (retry--);


	return (val);
}

void
osl_pci_write_config(osl_t *osh, uint offset, uint size, uint val)
{
	uint retry = PCI_CFG_RETRY;

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	
	ASSERT(size == 4);

	do {
		pci_write_config_dword(osh->pdev, offset, val);
		if (offset != PCI_BAR0_WIN)
			break;
		if (osl_pci_read_config(osh, offset, size) == val)
			break;
	} while (retry--);

}


uint
osl_pci_bus(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

	return ((struct pci_dev *)osh->pdev)->bus->number;
}


uint
osl_pci_slot(osl_t *osh)
{
	ASSERT(osh && (osh->magic == OS_HANDLE_MAGIC) && osh->pdev);

	return PCI_SLOT(((struct pci_dev *)osh->pdev)->devfn);
}

static void
osl_pcmcia_attr(osl_t *osh, uint offset, char *buf, int size, bool write)
{
}

void
osl_pcmcia_read_attr(osl_t *osh, uint offset, void *buf, int size)
{
	osl_pcmcia_attr(osh, offset, (char *) buf, size, FALSE);
}

void
osl_pcmcia_write_attr(osl_t *osh, uint offset, void *buf, int size)
{
	osl_pcmcia_attr(osh, offset, (char *) buf, size, TRUE);
}

void *
osl_malloc(osl_t *osh, uint size)
{
	void *addr;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25))
	gfp_t flags;

	
	if (osh)
		ASSERT(osh->magic == OS_HANDLE_MAGIC);

	flags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	if ((addr = kmalloc(size, flags)) == NULL) {
#else
	if ((addr = kmalloc(size, GFP_ATOMIC)) == NULL) {
#endif 
		if (osh)
			osh->failed++;
		return (NULL);
	}
	if (osh)
		atomic_add(size, &osh->malloced);

	return (addr);
}

void
osl_mfree(osl_t *osh, void *addr, uint size)
{
	if (osh) {
		ASSERT(osh->magic == OS_HANDLE_MAGIC);
		atomic_sub(size, &osh->malloced);
	}
	kfree(addr);
}

uint
osl_malloced(osl_t *osh)
{
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	return (atomic_read(&osh->malloced));
}

uint
osl_malloc_failed(osl_t *osh)
{
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	return (osh->failed);
}



uint
osl_dma_consistent_align(void)
{
	return (PAGE_SIZE);
}

void*
osl_dma_alloc_consistent(osl_t *osh, uint size, uint16 align_bits, uint *alloced, ulong *pap)
{
	uint16 align = (1 << align_bits);
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	if (!ISALIGNED(DMA_CONSISTENT_ALIGN, align))
		size += align;
	*alloced = size;

	return (pci_alloc_consistent(osh->pdev, size, (dma_addr_t*)pap));
}

void
osl_dma_free_consistent(osl_t *osh, void *va, uint size, ulong pa)
{
	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));

	pci_free_consistent(osh->pdev, size, va, (dma_addr_t)pa);
}

uint BCMFASTPATH
osl_dma_map(osl_t *osh, void *va, uint size, int direction)
{
	int dir;

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	dir = (direction == DMA_TX)? PCI_DMA_TODEVICE: PCI_DMA_FROMDEVICE;
	return (pci_map_single(osh->pdev, va, size, dir));
}

void BCMFASTPATH
osl_dma_unmap(osl_t *osh, uint pa, uint size, int direction)
{
	int dir;

	ASSERT((osh && (osh->magic == OS_HANDLE_MAGIC)));
	dir = (direction == DMA_TX)? PCI_DMA_TODEVICE: PCI_DMA_FROMDEVICE;
	pci_unmap_single(osh->pdev, (uint32)pa, size, dir);
}

#if defined(BCMASSERT_LOG)
void
osl_assert(char *exp, char *file, int line)
{
	char tempbuf[256];
	char *basename;

	basename = strrchr(file, '/');
	
	if (basename)
		basename++;

	if (!basename)
		basename = file;

#ifdef BCMASSERT_LOG
	snprintf(tempbuf, 64, "\"%s\": file \"%s\", line %d\n",
		exp, basename, line);

	bcm_assert_log(tempbuf);
#endif 


}
#endif 

void
osl_delay(uint usec)
{
	uint d;

	while (usec > 0) {
		d = MIN(usec, 1000);
		udelay(d);
		usec -= d;
	}
}



void *
osl_pktdup(osl_t *osh, void *skb)
{
	void * p;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25))
	gfp_t flags;

	flags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	if ((p = skb_clone((struct sk_buff *)skb, flags)) == NULL)
#else
	if ((p = skb_clone((struct sk_buff*)skb, GFP_ATOMIC)) == NULL)
#endif 
		return NULL;

#ifdef CTFPOOL
	if (PKTISFAST(osh, skb)) {
		ctfpool_t *ctfpool;

		
		ctfpool = (ctfpool_t *)CTFPOOLPTR(osh, skb);
		ASSERT(ctfpool != NULL);
		PKTCLRFAST(osh, p);
		PKTCLRFAST(osh, skb);
		ctfpool->refills++;
	}
#endif 

	
	if (osh->pub.pkttag)
		bzero((void*)((struct sk_buff *)p)->cb, OSL_PKTTAG_SZ);

	
	osh->pub.pktalloced++;
	return (p);
}







void *
osl_os_open_image(char *filename)
{
	struct file *fp;

	fp = filp_open(filename, O_RDONLY, 0);
	
	 if (IS_ERR(fp))
		 fp = NULL;

	 return fp;
}

int
osl_os_get_image_block(char *buf, int len, void *image)
{
	struct file *fp = (struct file *)image;
	int rdlen;

	if (!image)
		return 0;

	rdlen = kernel_read(fp, fp->f_pos, buf, len);
	if (rdlen > 0)
		fp->f_pos += rdlen;

	return rdlen;
}

void
osl_os_close_image(void *image)
{
	if (image)
		filp_close((struct file *)image, NULL);
}
