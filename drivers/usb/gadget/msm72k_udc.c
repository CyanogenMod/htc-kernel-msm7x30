/*
 * Driver for HighSpeed USB Client Controller in MSM7K
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include <linux/irq.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/switch.h>

#include <asm/mach-types.h>

#include <mach/board.h>
#include <mach/msm_hsusb.h>
#include <linux/device.h>
#include <mach/msm_hsusb_hw.h>
#ifdef CONFIG_ARCH_MSM7X30
#include <mach/rpc_hsusb.h>
#endif
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
#include <mach/htc_headset_mgr.h>
#endif
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
#include <mach/cable_detect.h>
#endif
#include <mach/clk.h>

static const char driver_name[] = "msm72k_udc";

/* #define DEBUG */
/* #define VERBOSE */

#define MSM_USB_BASE ((unsigned) ui->addr)

#define	DRIVER_DESC		"MSM 72K USB Peripheral Controller"

#define EPT_FLAG_IN        0x0001

#define SETUP_BUF_SIZE      4096


static const char *const ep_name[] = {
	"ep0out", "ep1out", "ep2out", "ep3out",
	"ep4out", "ep5out", "ep6out", "ep7out",
	"ep8out", "ep9out", "ep10out", "ep11out",
	"ep12out", "ep13out", "ep14out", "ep15out",
	"ep0in", "ep1in", "ep2in", "ep3in",
	"ep4in", "ep5in", "ep6in", "ep7in",
	"ep8in", "ep9in", "ep10in", "ep11in",
	"ep12in", "ep13in", "ep14in", "ep15in"
};

static struct usb_info *the_usb_info;
/* current state of VBUS */
static int vbus;
static int use_mfg_serialno;
static char mfg_df_serialno[16];
static int disable_charger;

#if defined (CONFIG_DOCK_ACCESSORY_DETECT) || defined(CONFIG_USB_ACCESSORY_DETECT)
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
extern int htc_get_usb_accessory_adc_level(uint32_t *buffer);
#endif


static struct switch_dev dock_switch = {
	.name = "dock",
};

#define DOCK_STATE_UNDOCKED     0
#define DOCK_STATE_DESK         (1 << 0)
#define DOCK_STATE_CAR          (1 << 1)

#define DOCK_DET_DELAY		HZ/4
#endif

#include <linux/wakelock.h>
#include <mach/perflock.h>

static struct wake_lock vbus_idle_wake_lock;
static struct perf_lock usb_perf_lock;

struct msm_request {
	struct usb_request req;

	/* saved copy of req.complete */
	void	(*gadget_complete)(struct usb_ep *ep,
					struct usb_request *req);


	struct usb_info *ui;
	struct msm_request *next;

	unsigned busy:1;
	unsigned live:1;
	unsigned alloced:1;
	unsigned dead:1;

	dma_addr_t dma;
	dma_addr_t item_dma;

	struct ept_queue_item *item;
};

#define to_msm_request(r) container_of(r, struct msm_request, req)
#define to_msm_endpoint(r) container_of(r, struct msm_endpoint, ep)

struct msm_endpoint {
	struct usb_ep ep;
	struct usb_info *ui;
	struct msm_request *req; /* head of pending requests */
	struct msm_request *last;
	unsigned flags;

	/* bit number (0-31) in various status registers
	** as well as the index into the usb_info's array
	** of all endpoints
	*/
	unsigned char bit;
	unsigned char num;

	/* pointers to DMA transfer list area */
	/* these are allocated from the usb_info dma space */
	struct ept_queue_head *head;
};

static void usb_do_work(struct work_struct *w);
static void do_usb_hub_disable(struct work_struct *w);
static void check_charger(struct work_struct *w);
#ifdef CONFIG_USB_ACCESSORY_DETECT
static void accessory_detect_work(struct work_struct *w);
#endif
#ifdef CONFIG_DOCK_ACCESSORY_DETECT
static void dock_isr_work(struct work_struct *w);
static void dock_detect_work(struct work_struct *w);
static void dock_detect_init(struct usb_info *ui);
#endif

#define USB_STATE_IDLE    0
#define USB_STATE_ONLINE  1
#define USB_STATE_OFFLINE 2

#define USB_FLAG_START          0x0001
#define USB_FLAG_VBUS_ONLINE    0x0002
#define USB_FLAG_VBUS_OFFLINE   0x0004
#define USB_FLAG_RESET          0x0008
#define USB_FLAG_CONFIGURED     0x0020

struct usb_info {
	/* lock for register/queue/device state changes */
	spinlock_t lock;

	/* single request used for handling setup transactions */
	struct usb_request *setup_req;

	struct platform_device *pdev;
	int irq;
	void *addr;

	unsigned state;
	unsigned flags;

	atomic_t online;
	atomic_t running;

	struct dma_pool *pool;

	/* dma page to back the queue heads and items */
	unsigned char *buf;
	dma_addr_t dma;

	struct ept_queue_head *head;

	/* used for allocation */
	unsigned next_item;
	unsigned next_ifc_num;

	/* endpoints are ordered based on their status bits,
	** so they are OUT0, OUT1, ... OUT15, IN0, IN1, ... IN15
	*/
	struct msm_endpoint ept[32];

	int *phy_init_seq;
	void (*phy_reset)(void);
	void (*hw_reset)(bool en);
	void (*usb_uart_switch)(int);
	void (*serial_debug_gpios)(int);
	void (*usb_hub_enable)(bool);
	int (*china_ac_detect)(void);
	void (*disable_usb_charger)(void);
	void (*change_phy_voltage)(int);
	int (*ldo_init) (int init);
	int (*ldo_enable) (int enable);
	void (*usb_mhl_switch)(bool);

	/* for notification when USB is connected or disconnected */
	void (*usb_connected)(int);

	struct workqueue_struct *usb_wq;
	struct work_struct work;
	struct delayed_work chg_work;
	struct work_struct detect_work;
	struct work_struct notifier_work;
	struct work_struct usb_hub_work;
	unsigned phy_status;
	unsigned phy_fail_count;

	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct switch_dev sdev;

#define ep0out ept[0]
#define ep0in  ept[16]

	struct clk *clk;
	struct clk *coreclk;
	struct clk *pclk;
	struct clk *otgclk;
	struct clk *ebi1clk;

	atomic_t ep0_dir;
	atomic_t test_mode;

	atomic_t remote_wakeup;
	enum usb_connect_type connect_type;
	u8 in_lpm;

	/* for accessory detection */
	bool dock_detect;
	u8 accessory_detect;
	u8 mfg_usb_carkit_enable;
	int idpin_irq;
	int usb_id_pin_gpio;

	int dockpin_irq;
	int dock_pin_gpio;
	uint8_t dock_pin_state;
	struct delayed_work dock_work_isr;
	struct delayed_work dock_work;

	void (*config_usb_id_gpios)(bool output_enable);
	/* 0: none, 1: carkit, 2: usb headset, 4: mhl */
	u8 accessory_type;
	struct timer_list	ac_detect_timer;
	int			ac_detect_count;
	int ac_9v_gpio;
};

static const struct usb_ep_ops msm72k_ep_ops;


static int msm72k_pullup(struct usb_gadget *_gadget, int is_active);
static int msm72k_set_halt(struct usb_ep *_ep, int value);
static void flush_endpoint(struct msm_endpoint *ept);

static DEFINE_MUTEX(notify_sem);
static void send_usb_connect_notify(struct work_struct *w)
{
	static struct t_usb_status_notifier *notifier;
	struct usb_info *ui = container_of(w, struct usb_info,
		notifier_work);
	if (!ui)
		return;

	USB_INFO("send connect type %d\n", ui->connect_type);
	mutex_lock(&notify_sem);
	list_for_each_entry(notifier,
		&g_lh_usb_notifier_list,
		notifier_link) {
			if (notifier->func != NULL) {
				/* Notify other drivers about connect type. */
				/* use slow charging for unknown type*/
				if (ui->connect_type == CONNECT_TYPE_UNKNOWN)
					notifier->func(CONNECT_TYPE_USB);
				else
					notifier->func(ui->connect_type);
			}
		}
	mutex_unlock(&notify_sem);
}

int usb_register_notifier(struct t_usb_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link,
		&g_lh_usb_notifier_list);
	mutex_unlock(&notify_sem);
	return 0;
}

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", driver_name);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->state ? "online" : "offline");
}

static int usb_ep_get_stall(struct msm_endpoint *ept)
{
	unsigned int n;
	struct usb_info *ui = ept->ui;

	n = readl(USB_ENDPTCTRL(ept->num));
	if (ept->flags & EPT_FLAG_IN)
		return (CTRL_TXS & n) ? 1 : 0;
	else
		return (CTRL_RXS & n) ? 1 : 0;
}

static unsigned ulpi_read(struct usb_info *ui, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout)) ;

	if (timeout == 0) {
		USB_ERR("ulpi_read: timeout %08x\n", readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct usb_info *ui, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout)) ;

	if (timeout == 0) {
		USB_ERR("ulpi_write: timeout\n");
		return -1;
	}

	return 0;
}

static void ulpi_init(struct usb_info *ui)
{
	int *seq = ui->phy_init_seq;

	if (!seq)
		return;

	while (seq[0] >= 0) {
		USB_INFO("ulpi: write 0x%02x to 0x%02x\n", seq[0], seq[1]);
		ulpi_write(ui, seq[0], seq[1]);
		seq += 2;
	}
}

static void init_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++) {
		struct msm_endpoint *ept = ui->ept + n;

		ept->ui = ui;
		ept->bit = n;
		ept->num = n & 15;
		ept->ep.name = ep_name[n];
		ept->ep.ops = &msm72k_ep_ops;

		if (ept->bit > 15) {
			/* IN endpoint */
			ept->head = ui->head + (ept->num << 1) + 1;
			ept->flags = EPT_FLAG_IN;
		} else {
			/* OUT endpoint */
			ept->head = ui->head + (ept->num << 1);
			ept->flags = 0;
		}

	}
}

static void config_ept(struct msm_endpoint *ept)
{
	unsigned cfg = CONFIG_MAX_PKT(ept->ep.maxpacket) | CONFIG_ZLT;

	if (ept->bit == 0)
		/* ep0 out needs interrupt-on-setup */
		cfg |= CONFIG_IOS;

	ept->head->config = cfg;
	ept->head->next = TERMINATE;

#if 0
	if (ept->ep.maxpacket)
		INFO("ept #%d %s max:%d head:%p bit:%d\n",
		    ept->num, (ept->flags & EPT_FLAG_IN) ? "in" : "out",
		    ept->ep.maxpacket, ept->head, ept->bit);
#endif
}

static void configure_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++)
		config_ept(ui->ept + n);
}

struct usb_request *usb_ept_alloc_req(struct msm_endpoint *ept,
			unsigned bufsize, gfp_t gfp_flags)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		goto fail1;

	req->item = dma_pool_alloc(ui->pool, gfp_flags, &req->item_dma);
	if (!req->item)
		goto fail2;

	if (bufsize) {
		req->req.buf = kmalloc(bufsize, gfp_flags);
		if (!req->req.buf)
			goto fail3;
		req->alloced = 1;
	}

	return &req->req;

fail3:
	dma_pool_free(ui->pool, req->item, req->item_dma);
fail2:
	kfree(req);
fail1:
	return 0;
}

static void do_free_req(struct usb_info *ui, struct msm_request *req)
{
	if (req->alloced)
		kfree(req->req.buf);

	dma_pool_free(ui->pool, req->item, req->item_dma);
	kfree(req);
}


static void usb_ept_enable(struct msm_endpoint *ept, int yes,
		unsigned char ep_type)
{
	struct usb_info *ui = ept->ui;
	int in = ept->flags & EPT_FLAG_IN;
	unsigned n;

	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		n = (n & (~CTRL_TXT_MASK));
		if (yes) {
			n |= CTRL_TXE | CTRL_TXR;
		} else {
			n &= (~CTRL_TXE);
		}
		if (yes) {
			switch (ep_type) {
			case USB_ENDPOINT_XFER_BULK:
				n |= CTRL_TXT_BULK;
				break;
			case USB_ENDPOINT_XFER_INT:
				n |= CTRL_TXT_INT;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				n |= CTRL_TXT_ISOCH;
				break;
			default:
				USB_ERR("%s: unsupported ep_type %d for %s\n",
					__func__, ep_type, ept->ep.name);
				break;
			}
		}
	} else {
		n = (n & (~CTRL_RXT_MASK));
		if (yes) {
			n |= CTRL_RXE | CTRL_RXR;
		} else {
			n &= ~(CTRL_RXE);
		}
		if (yes) {
			switch (ep_type) {
			case USB_ENDPOINT_XFER_BULK:
				n |= CTRL_RXT_BULK;
				break;
			case USB_ENDPOINT_XFER_INT:
				n |= CTRL_RXT_INT;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				n |= CTRL_RXT_ISOCH;
				break;
			default:
				USB_ERR("%s: unsupported ep_type %d for %s\n",
					__func__, ep_type, ept->ep.name);
				break;
			}
		}
	}
	writel(n, USB_ENDPTCTRL(ept->num));

#if 0
	INFO("ept %d %s %s\n",
	       ept->num, in ? "in" : "out", yes ? "enabled" : "disabled");
#endif
}

static void usb_ept_start(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req = ept->req;
	int i, cnt;
	unsigned n = 1 << ept->bit;

	BUG_ON(req->live);

	/* link the hw queue head to the request's transaction item */
	ept->head->next = req->item_dma;
	ept->head->info = 0;

	/* during high throughput testing it is observed that
	 * ept stat bit is not set even thoguh all the data
	 * structures are updated properly and ept prime bit
	 * is set. To workaround the issue, try to check if
	 * ept stat bit otherwise try to re-prime the ept
	 */
	for (i = 0; i < 5; i++) {
		writel(n, USB_ENDPTPRIME);
		for (cnt = 0; cnt < 3000; cnt++) {
			if (!(readl(USB_ENDPTPRIME) & n) &&
					(readl(USB_ENDPTSTAT) & n))
				goto DONE;
			udelay(1);
		}
	}

	if (!(readl(USB_ENDPTSTAT) & n)) {
		USB_ERR("Unable to prime the ept%d%s\n",
				ept->num,
				ept->flags & EPT_FLAG_IN ? "in" : "out");
		return;
	}

DONE:
	/* mark this chain of requests as live */
	while (req) {
		req->live = 1;
		req = req->next;
	}
}

int usb_ept_queue_xfer(struct msm_endpoint *ept, struct usb_request *_req)
{
	unsigned long flags;
	struct msm_request *req = to_msm_request(_req);
	struct msm_request *last;
	struct usb_info *ui = ept->ui;
	struct ept_queue_item *item = req->item;
	unsigned length = req->req.length;

	if (length > 0x4000)
		return -EMSGSIZE;

	spin_lock_irqsave(&ui->lock, flags);

	if (req->busy) {
		req->req.status = -EBUSY;
		spin_unlock_irqrestore(&ui->lock, flags);
		USB_INFO("usb_ept_queue_xfer() tried to queue busy request\n");
		return -EBUSY;
	}

	if (!atomic_read(&ui->online) && (ept->num != 0)) {
		req->req.status = -ESHUTDOWN;
		spin_unlock_irqrestore(&ui->lock, flags);
		USB_INFO("usb_ept_queue_xfer() called while offline\n");
		return -ESHUTDOWN;
	}

	req->busy = 1;
	req->live = 0;
	req->next = 0;
	req->req.status = -EBUSY;

	req->dma = dma_map_single(NULL, req->req.buf, length,
				  (ept->flags & EPT_FLAG_IN) ?
				  DMA_TO_DEVICE : DMA_FROM_DEVICE);

	/* prepare the transaction descriptor item for the hardware */
	item->next = TERMINATE;
	item->info = INFO_BYTES(length) | INFO_IOC | INFO_ACTIVE;
	item->page0 = req->dma;
	item->page1 = (req->dma + 0x1000) & 0xfffff000;
	item->page2 = (req->dma + 0x2000) & 0xfffff000;
	item->page3 = (req->dma + 0x3000) & 0xfffff000;

	/* Add the new request to the end of the queue */
	last = ept->last;
	if (last) {
		/* Already requests in the queue. add us to the
		 * end, but let the completion interrupt actually
		 * start things going, to avoid hw issues
		 */
		last->next = req;

		/* only modify the hw transaction next pointer if
		 * that request is not live
		 */
		if (!last->live)
			last->item->next = req->item_dma;
	} else {
		/* queue was empty -- kick the hardware */
		ept->req = req;
		usb_ept_start(ept);
	}
	ept->last = req;

	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
}

/* --- endpoint 0 handling --- */

static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_request *r = to_msm_request(req);
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;

	req->complete = r->gadget_complete;
	r->gadget_complete = 0;
	if	(req->complete)
		req->complete(&ui->ep0in.ep, req);
}

static void ep0_queue_ack_complete(struct usb_ep *ep,
	struct usb_request *_req)
{
	struct msm_request *r = to_msm_request(_req);
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;
	struct usb_request *req = ui->setup_req;

	/* queue up the receive of the ACK response from the host */
	if (_req->status == 0 && _req->actual == _req->length) {
		req->length = 0;
		if (atomic_read(&ui->ep0_dir) == USB_DIR_IN)
			usb_ept_queue_xfer(&ui->ep0out, req);
		else
			usb_ept_queue_xfer(&ui->ep0in, req);
		_req->complete = r->gadget_complete;
		r->gadget_complete = 0;
		if (_req->complete)
			_req->complete(&ui->ep0in.ep, _req);
	} else
		ep0_complete(ep, _req);
}

static void ep0_setup_ack_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;
	unsigned int temp;
	int test_mode = atomic_read(&ui->test_mode);

	if (!test_mode)
		return;

	switch (test_mode) {
	case J_TEST:
		USB_INFO("usb electrical test mode: (J)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_J_STATE, USB_PORTSC);
		break;

	case K_TEST:
		USB_INFO("usb electrical test mode: (K)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_K_STATE, USB_PORTSC);
		break;

	case SE0_NAK_TEST:
		USB_INFO("usb electrical test mode: (SE0-NAK)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_SE0_NAK, USB_PORTSC);
		break;

	case TST_PKT_TEST:
		USB_INFO("usb electrical test mode: (TEST_PKT)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_TST_PKT, USB_PORTSC);
		break;
	}
}

static void ep0_setup_ack(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	req->length = 0;
	req->complete = ep0_setup_ack_complete;
	usb_ept_queue_xfer(&ui->ep0in, req);
}

static void ep0_setup_stall(struct usb_info *ui)
{
	writel((1<<16) | (1<<0), USB_ENDPTCTRL(0));
}

static void ep0_setup_send(struct usb_info *ui, unsigned length)
{
	struct usb_request *req = ui->setup_req;
	struct msm_request *r = to_msm_request(req);
	struct msm_endpoint *ept = &ui->ep0in;

	req->length = length;
	req->complete = ep0_queue_ack_complete;
	r->gadget_complete = 0;
	usb_ept_queue_xfer(ept, req);
}

static void handle_setup(struct usb_info *ui)
{
	struct usb_ctrlrequest ctl;
	struct usb_request *req = ui->setup_req;
	int ret;

	memcpy(&ctl, ui->ep0out.head->setup_data, sizeof(ctl));
	writel(EPT_RX(0), USB_ENDPTSETUPSTAT);

	if (ctl.bRequestType & USB_DIR_IN)
		atomic_set(&ui->ep0_dir, USB_DIR_IN);
	else
		atomic_set(&ui->ep0_dir, USB_DIR_OUT);

	/* any pending ep0 transactions must be canceled */
	flush_endpoint(&ui->ep0out);
	flush_endpoint(&ui->ep0in);

#if 0
	INFO("setup: type=%02x req=%02x val=%04x idx=%04x len=%04x\n",
	       ctl.bRequestType, ctl.bRequest, ctl.wValue,
	       ctl.wIndex, ctl.wLength);
#endif

	if ((ctl.bRequestType & (USB_DIR_IN | USB_TYPE_MASK)) ==
					(USB_DIR_IN | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_GET_STATUS) {
			if (ctl.wLength != 2)
				goto stall;
			switch (ctl.bRequestType & USB_RECIP_MASK) {
			case USB_RECIP_ENDPOINT:
			{
				struct msm_endpoint *ept;
				unsigned num =
					ctl.wIndex & USB_ENDPOINT_NUMBER_MASK;
				u16 temp = 0;

				if (num == 0) {
					memset(req->buf, 0, 2);
					break;
				}
				if (ctl.wIndex & USB_ENDPOINT_DIR_MASK)
					num += 16;
				ept = &ui->ep0out + num;
				temp = usb_ep_get_stall(ept);
				temp = temp << USB_ENDPOINT_HALT;
				memcpy(req->buf, &temp, 2);
				break;
			}
			case USB_RECIP_DEVICE:
			{
				u16 temp = 0;

				temp |= (atomic_read(&ui->remote_wakeup)
						<< USB_DEVICE_REMOTE_WAKEUP);
				memcpy(req->buf, &temp, 2);
				break;
			}
			case USB_RECIP_INTERFACE:
				memset(req->buf, 0, 2);
				break;
			default:
				goto stall;
			}
			ep0_setup_send(ui, 2);
			return;
		}
	}
	if (ctl.bRequestType ==
		    (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)) {
		if ((ctl.bRequest == USB_REQ_CLEAR_FEATURE) ||
				(ctl.bRequest == USB_REQ_SET_FEATURE)) {
			if ((ctl.wValue == 0) && (ctl.wLength == 0)) {
				unsigned num = ctl.wIndex & 0x0f;

				if (num != 0) {
					struct msm_endpoint *ept;

					if (ctl.wIndex & 0x80)
						num += 16;
					ept = &ui->ep0out + num;

					if (ctl.bRequest == USB_REQ_SET_FEATURE)
						msm72k_set_halt(&ept->ep, 1);
					else
						msm72k_set_halt(&ept->ep, 0);
				}
				goto ack;
			}
		}
	}
	if (ctl.bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD)) {
		if (ctl.bRequest == USB_REQ_SET_CONFIGURATION) {
			atomic_set(&ui->online, !!ctl.wValue);
			ui->flags |= USB_FLAG_CONFIGURED;
			queue_work(ui->usb_wq, &ui->work);
		} else if (ctl.bRequest == USB_REQ_SET_ADDRESS) {
			/* write address delayed (will take effect
			** after the next IN txn)
			*/
			writel((ctl.wValue << 25) | (1 << 24), USB_DEVICEADDR);
			goto ack;
		} else if (ctl.bRequest == USB_REQ_SET_FEATURE) {
			switch (ctl.wValue) {
			case USB_DEVICE_TEST_MODE:
				switch (ctl.wIndex) {
				case J_TEST:
				case K_TEST:
				case SE0_NAK_TEST:
					if (!atomic_read(&ui->test_mode)) {
						disable_charger = 1;
						queue_delayed_work(ui->usb_wq, &ui->chg_work, 0);
					}
				case TST_PKT_TEST:
					atomic_set(&ui->test_mode, ctl.wIndex);
					goto ack;
				}
				goto stall;
			case USB_DEVICE_REMOTE_WAKEUP:
				atomic_set(&ui->remote_wakeup, 1);
				goto ack;
			}
		} else if ((ctl.bRequest == USB_REQ_CLEAR_FEATURE) &&
				(ctl.wValue == USB_DEVICE_REMOTE_WAKEUP)) {
			atomic_set(&ui->remote_wakeup, 0);
			goto ack;
		}
	}

	/* delegate if we get here */
	if (ui->driver) {
		ret = ui->driver->setup(&ui->gadget, &ctl);
		if (ret >= 0)
			return;
	}

stall:
	/* stall ep0 on error */
	ep0_setup_stall(ui);
	return;

ack:
	ep0_setup_ack(ui);
}

static void handle_endpoint(struct usb_info *ui, unsigned bit)
{
	struct msm_endpoint *ept = ui->ept + bit;
	struct msm_request *req;
	unsigned long flags;
	unsigned info;

#if 0
	INFO("handle_endpoint() %d %s req=%p(%08x)\n",
		ept->num, (ept->flags & EPT_FLAG_IN) ? "in" : "out",
		ept->req, ept->req ? ept->req->item_dma : 0);
#endif

	/* expire all requests that are no longer active */
	spin_lock_irqsave(&ui->lock, flags);
	while ((req = ept->req)) {
		info = req->item->info;

		/* if we've processed all live requests, time to
		 * restart the hardware on the next non-live request
		 */
		if (!req->live) {
			usb_ept_start(ept);
			break;
		}

		/* if the transaction is still in-flight, stop here */
		if (info & INFO_ACTIVE)
			break;

		/* advance ept queue to the next request */
		ept->req = req->next;
		if (ept->req == 0)
			ept->last = 0;

		dma_unmap_single(NULL, req->dma, req->req.length,
				 (ept->flags & EPT_FLAG_IN) ?
				 DMA_TO_DEVICE : DMA_FROM_DEVICE);

		if (info & (INFO_HALTED | INFO_BUFFER_ERROR | INFO_TXN_ERROR)) {
			/* XXX pass on more specific error code */
			req->req.status = -EIO;
			req->req.actual = 0;
			USB_INFO("msm72k_udc: ept %d %s error. info=%08x\n",
			       ept->num,
			       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
			       info);
		} else {
			req->req.status = 0;
			req->req.actual =
				req->req.length - ((info >> 16) & 0x7FFF);
		}
		req->busy = 0;
		req->live = 0;
		if (req->dead)
			do_free_req(ui, req);
		else if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(&ept->ep, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

#define FLUSH_WAIT_US	5
#define FLUSH_TIMEOUT	(2 * (USEC_PER_SEC / FLUSH_WAIT_US))
static void flush_endpoint_hw(struct usb_info *ui, unsigned bits)
{
	uint32_t unflushed = 0;
	uint32_t stat = 0;
	int cnt = 0;

	/* flush endpoint, canceling transactions
	** - this can take a "large amount of time" (per databook)
	** - the flush can fail in some cases, thus we check STAT
	**   and repeat if we're still operating
	**   (does the fact that this doesn't use the tripwire matter?!)
	*/
	while (cnt < FLUSH_TIMEOUT) {
		writel(bits, USB_ENDPTFLUSH);
		while (((unflushed = readl(USB_ENDPTFLUSH)) & bits) &&
		       cnt < FLUSH_TIMEOUT) {
			cnt++;
			udelay(FLUSH_WAIT_US);
		}

		stat = readl(USB_ENDPTSTAT);
		if (cnt >= FLUSH_TIMEOUT)
			goto err;
		if (!(stat & bits))
			goto done;
		cnt++;
		udelay(FLUSH_WAIT_US);
	}

err:
	USB_WARNING("%s: Could not complete flush! NOT GOOD! "
		   "stat: %x unflushed: %x bits: %x\n", __func__,
		   stat, unflushed, bits);
done:
	return;
}

static void flush_endpoint_sw(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req, *next_req = NULL;
	unsigned long flags;

	/* inactive endpoints have nothing to do here */
	if (ept->ep.maxpacket == 0)
		return;

	/* put the queue head in a sane state */
	ept->head->info = 0;
	ept->head->next = TERMINATE;

	/* cancel any pending requests */
	spin_lock_irqsave(&ui->lock, flags);
	req = ept->req;
	ept->req = 0;
	ept->last = 0;
	while (req != 0) {
		req->busy = 0;
		req->live = 0;
		req->req.status = -ECONNRESET;
		req->req.actual = 0;

		/*
		 * Gadget driver may free the request in completion
		 * handler. So keep a copy of next req pointer
		 * before calling completion handler.
		 */
		next_req = req->next;
		if (req->req.complete) {
			spin_unlock_irqrestore(&ui->lock, flags);
			req->req.complete(&ept->ep, &req->req);
			spin_lock_irqsave(&ui->lock, flags);
		}
		if (req->dead)
			do_free_req(ui, req);
		req = next_req;
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void flush_endpoint(struct msm_endpoint *ept)
{
	flush_endpoint_hw(ept->ui, (1 << ept->bit));
	flush_endpoint_sw(ept);
}

static void handle_notify_offline(struct usb_info *ui)
{
	if (ui->driver) {
		USB_INFO("%s: notify offline\n", __func__);
		ui->driver->disconnect(&ui->gadget);
	}
	/* cancel pending ep0 transactions */
	flush_endpoint(&ui->ep0out);
	flush_endpoint(&ui->ep0in);
}

static irqreturn_t usb_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;
	unsigned n;
	unsigned long flags;

	n = readl(USB_USBSTS);
	writel(n, USB_USBSTS);

	/* somehow we got an IRQ while in the reset sequence: ignore it */
	if (!atomic_read(&ui->running))
		return IRQ_HANDLED;

	if (n & STS_PCI) {
		switch (readl(USB_PORTSC) & PORTSC_PSPD_MASK) {
		case PORTSC_PSPD_FS:
			USB_INFO("portchange USB_SPEED_FULL\n");
			spin_lock_irqsave(&ui->lock, flags);
			ui->gadget.speed = USB_SPEED_FULL;
			spin_unlock_irqrestore(&ui->lock, flags);
			break;
		case PORTSC_PSPD_LS:
			USB_INFO("portchange USB_SPEED_LOW\n");
			spin_lock_irqsave(&ui->lock, flags);
			ui->gadget.speed = USB_SPEED_LOW;
			spin_unlock_irqrestore(&ui->lock, flags);
			break;
		case PORTSC_PSPD_HS:
			USB_INFO("portchange USB_SPEED_HIGH\n");
			spin_lock_irqsave(&ui->lock, flags);
			ui->gadget.speed = USB_SPEED_HIGH;
			spin_unlock_irqrestore(&ui->lock, flags);
			break;
		}
	}

	if (n & STS_URI) {
		USB_INFO("reset\n");

		writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
		writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
		writel(0xffffffff, USB_ENDPTFLUSH);
		writel(0, USB_ENDPTCTRL(1));

		if (atomic_read(&ui->online)) {
			/* marking us offline will cause ept queue attempts
			** to fail
			*/
			atomic_set(&ui->online, 0);

			handle_notify_offline(ui);
		}
		if (ui->connect_type != CONNECT_TYPE_USB) {
			ui->connect_type = CONNECT_TYPE_USB;
			queue_work(ui->usb_wq, &ui->notifier_work);
			ui->ac_detect_count = 0;
			del_timer_sync(&ui->ac_detect_timer);
		}
	}

	if (n & STS_SLI)
		USB_INFO("suspend\n");

	if (n & STS_UI) {
		n = readl(USB_ENDPTSETUPSTAT);
		if (n & EPT_RX(0))
			handle_setup(ui);

		n = readl(USB_ENDPTCOMPLETE);
		writel(n, USB_ENDPTCOMPLETE);
		while (n) {
			unsigned bit = __ffs(n);
			handle_endpoint(ui, bit);
			n = n & (~(1 << bit));
		}
	}
	return IRQ_HANDLED;
}

int usb_get_connect_type(void)
{
	if (!the_usb_info)
		return 0;
	return the_usb_info->connect_type;
}
EXPORT_SYMBOL(usb_get_connect_type);

void msm_hsusb_request_reset(void)
{
	struct usb_info *ui = the_usb_info;
	unsigned long flags;
	if (!ui)
		return;
	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_RESET;
	queue_work(ui->usb_wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);
}

static ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	if (!the_usb_info)
		return 0;
	length = sprintf(buf, "%d\n",
		(the_usb_info->connect_type == CONNECT_TYPE_USB)?1:0);
	return length;
}

static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL);

static ssize_t show_USB_ID_status(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int value = 1;
	unsigned length;
#if defined(CONFIG_CABLE_DETECT_ACCESSORY)
	value = cable_get_usb_id_level();
#else
	struct usb_info *ui = the_usb_info;

	if (!ui)
		return 0;
	if (ui->usb_id_pin_gpio != 0) {
		value = gpio_get_value(ui->usb_id_pin_gpio);
		USB_INFO("id pin status %d\n", value);
	}
#endif
	length = sprintf(buf, "%d", value);
	return length;
}

static DEVICE_ATTR(USB_ID_status, 0444,
	show_USB_ID_status, NULL);

static ssize_t show_usb_car_kit_enable(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct usb_info *ui = the_usb_info;
	int value = 1;
	unsigned length;

	if (!ui)
		return 0;
	if (ui->accessory_detect == 0) {
		value = 0;
	}
	USB_INFO("USB_car_kit_enable %d\n", ui->accessory_detect);
	length = sprintf(buf, "%d", value);
	return length;
}

static DEVICE_ATTR(usb_car_kit_enable, 0444,
	show_usb_car_kit_enable, NULL);/*for kar kit AP check if car kit enable*/

static ssize_t show_usb_phy_setting(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct usb_info *ui = the_usb_info;
	unsigned length = 0;
	int i;

	for (i = 0; i <= 0x14; i++)
		length += sprintf(buf + length, "0x%x = 0x%x\n", i, ulpi_read(ui, i));

	for (i = 0x30; i <= 0x37; i++)
		length += sprintf(buf + length, "0x%x = 0x%x\n", i, ulpi_read(ui, i));

	return length;
}

static ssize_t store_usb_phy_setting(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;
	char *token[10];
	unsigned reg;
	unsigned value;
	int i;

	USB_INFO("%s\n", buf);
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");

	reg = simple_strtoul(token[0], NULL, 16);
	value = simple_strtoul(token[1], NULL, 16);
	USB_INFO("Set 0x%x = 0x%x\n", reg, value);

	ulpi_write(ui, value, reg);

	return 0;
}

static DEVICE_ATTR(usb_phy_setting, 0664,
	show_usb_phy_setting, store_usb_phy_setting);

#ifdef CONFIG_USB_ACCESSORY_DETECT
static ssize_t show_mfg_carkit_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct usb_info *ui = the_usb_info;

	length = sprintf(buf, "%d", ui->mfg_usb_carkit_enable);
	USB_INFO("%s: %d\n", __func__, ui->mfg_usb_carkit_enable);
	return length;

}

static ssize_t store_mfg_carkit_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;
	unsigned char uc;

	if (buf[0] != '0' && buf[0] != '1') {
		USB_ERR("Can't enable/disable carkit\n");
		return -EINVAL;
	}
	uc = buf[0] - '0';
	USB_INFO("%s: %d\n", __func__, uc);
	ui->mfg_usb_carkit_enable = uc;
	if (uc == 1 && ui->accessory_type == 1 &&
		board_mfg_mode() == 1) {
		switch_set_state(&dock_switch, DOCK_STATE_CAR);
		USB_INFO("carkit: set state %d\n", DOCK_STATE_CAR);
	}
	return count;
}

static DEVICE_ATTR(usb_mfg_carkit_enable, 0644,
	show_mfg_carkit_enable, store_mfg_carkit_enable);
#endif

#if defined (CONFIG_DOCK_ACCESSORY_DETECT) || defined(CONFIG_USB_ACCESSORY_DETECT)
static ssize_t dock_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct usb_info *ui = the_usb_info;
	if (ui->accessory_type == 1)
		return sprintf(buf, "online\n");
	else if (ui->accessory_type == 3) /*desk dock*/
		return sprintf(buf, "online\n");
	else
		return sprintf(buf, "offline\n");
}
static DEVICE_ATTR(status, S_IRUGO | S_IWUSR, dock_status_show, NULL);
#endif

static void usb_prepare(struct usb_info *ui)
{
	int ret;
	spin_lock_init(&ui->lock);

	memset(ui->buf, 0, 4096);
	ui->head = (void *) (ui->buf + 0);

	/* only important for reset/reinit */
	memset(ui->ept, 0, sizeof(ui->ept));
	ui->next_item = 0;
	ui->next_ifc_num = 0;

	init_endpoints(ui);

	ui->ep0in.ep.maxpacket = 64;
	ui->ep0out.ep.maxpacket = 64;

	ui->setup_req =
		usb_ept_alloc_req(&ui->ep0in, SETUP_BUF_SIZE, GFP_KERNEL);

	ui->usb_wq = create_singlethread_workqueue("msm_hsusb");
	if (ui->usb_wq == 0) {
		USB_ERR("fail to create workqueue\n");
		return;
	}
	INIT_WORK(&ui->work, usb_do_work);
#ifdef CONFIG_USB_ACCESSORY_DETECT
	INIT_WORK(&ui->detect_work, accessory_detect_work);
#endif
#ifdef CONFIG_DOCK_ACCESSORY_DETECT
	if (ui->dock_detect) {
		INIT_DELAYED_WORK(&ui->dock_work_isr, dock_isr_work);
		INIT_DELAYED_WORK(&ui->dock_work, dock_detect_work);
		dock_detect_init(ui);
	}
#endif

	INIT_WORK(&ui->notifier_work, send_usb_connect_notify);
	INIT_DELAYED_WORK(&ui->chg_work, check_charger);

	if (ui->usb_hub_enable)
		INIT_WORK(&ui->usb_hub_work, do_usb_hub_disable);

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_cable_connect);
	if (ret != 0)
		USB_WARNING("dev_attr_usb_cable_connect failed\n");

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_USB_ID_status);
	if (ret != 0)
		USB_WARNING("dev_attr_USB_ID_status failed\n");

	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_phy_setting);
	if (ret != 0)
		USB_WARNING("dev_attr_usb_phy_setting failed\n");

#ifdef CONFIG_USB_ACCESSORY_DETECT
	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_mfg_carkit_enable);
	if (ret != 0)
		USB_WARNING("dev_attr_usb_mfg_carkit_enable failed\n");
#endif
	ret = device_create_file(&ui->pdev->dev,
		&dev_attr_usb_car_kit_enable);/*for kar kit AP check if car kit enable*/
	if (ret != 0)
		USB_WARNING("dev_attr_usb_car_kit_enable failed\n");

	ui->sdev.name = driver_name;
	ui->sdev.print_name = print_switch_name;
	ui->sdev.print_state = print_switch_state;

	ret = switch_dev_register(&ui->sdev);
	if (ret != 0)
		USB_WARNING("switch class can't be registered\n");

}

static int usb_wakeup_phy(struct usb_info *ui)
{
	int i;

	/*writel(readl(USB_USBCMD) & ~ULPI_STP_CTRL, USB_USBCMD);*/

	/* some circuits automatically clear PHCD bit */
	for (i = 0; i < 5 && (readl(USB_PORTSC) & PORTSC_PHCD); i++) {
		writel(readl(USB_PORTSC) & ~PORTSC_PHCD, USB_PORTSC);
		mdelay(1);
	}

	if ((readl(USB_PORTSC) & PORTSC_PHCD)) {
		USB_ERR("%s: cannot clear phcd bit\n", __func__);
		return -1;
	}

	return 0;
}

static void usb_suspend_phy(struct usb_info *ui)
{
	USB_INFO("%s\n", __func__);
#ifdef CONFIG_ARCH_MSM7X00A
	/* disable unused interrupt */
	ulpi_write(ui, 0x01, 0x0d);
	ulpi_write(ui, 0x01, 0x10);

	/* disable interface protect circuit to drop current consumption */
	ulpi_write(ui, (1 << 7), 0x08);
	/* clear the SuspendM bit -> suspend the PHY */
	ulpi_write(ui, 1 << 6, 0x06);
#else
#ifdef CONFIG_ARCH_MSM7X30
	ulpi_write(ui, 0x0, 0x0D);
	ulpi_write(ui, 0x0, 0x10);
#endif
	/* clear VBusValid and SessionEnd rising interrupts */
	ulpi_write(ui, (1 << 1) | (1 << 3), 0x0f);
	/* clear VBusValid and SessionEnd falling interrupts */
	ulpi_write(ui, (1 << 1) | (1 << 3), 0x12);
	ulpi_read(ui, 0x14); /* clear PHY interrupt latch register*/

	ulpi_write(ui, 0x08, 0x09);/* turn off PLL on integrated phy */

	/* set phy to be in lpm */
	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);
	mdelay(1);
	if (!(readl(USB_PORTSC) & PORTSC_PHCD))
		USB_INFO("%s: unable to set lpm\n", __func__);
#endif
}

static void usb_reset(struct usb_info *ui)
{
	USB_INFO("hsusb: reset controller\n");

	atomic_set(&ui->running, 0);

	/* disable usb interrupts */
	writel(0, USB_USBINTR);

	/* wait for a while after enable usb clk*/
	msleep(5);

	/* To prevent phantom packets being received by the usb core on
	 * some devices, put the controller into reset prior to
	 * resetting the phy. */
	writel(2, USB_USBCMD);
	msleep(10);

#if 0
	/* we should flush and shutdown cleanly if already running */
	writel(0xffffffff, USB_ENDPTFLUSH);
	msleep(2);
#endif

	if (ui->phy_reset)
		ui->phy_reset();

	msleep(100);

	/* RESET */
	writel(2, USB_USBCMD);
	msleep(10);

#ifdef CONFIG_ARCH_MSM7X00A
	/* INCR4 BURST mode */
	writel(0x01, USB_SBUSCFG);
#else
	/* bursts of unspecified length. */
	writel(0, USB_AHBBURST);
	/* Use the AHB transactor */
	writel(0, USB_AHBMODE);
#endif

	/* select DEVICE mode */
	writel(0x12, USB_USBMODE);
	msleep(1);

	/* select ULPI phy */
	writel(0x80000000, USB_PORTSC);

	ulpi_init(ui);

	writel(ui->dma, USB_ENDPOINTLISTADDR);

	configure_endpoints(ui);

	/* marking us offline will cause ept queue attempts to fail */
	atomic_set(&ui->online, 0);

	handle_notify_offline(ui);

	/* enable interrupts */
	writel(STS_URI | STS_SLI | STS_UI | STS_PCI, USB_USBINTR);

	/* go to RUN mode (D+ pullup enable) */
	msm72k_pullup(&ui->gadget, 1);

	atomic_set(&ui->running, 1);
}

static void usb_start(struct usb_info *ui)
{
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_START;
/*if msm_hsusb_set_vbus_state set 1, but usb did not init, the ui =NULL, */
/*it would cause reboot with usb, it did not swith to USB and ADB fail*/
/*So when USB start, check again*/
#ifndef CONFIG_ARCH_MSM8X60
	if (vbus) {
		ui->flags |= USB_FLAG_VBUS_ONLINE;
		if (ui->change_phy_voltage)
			ui->change_phy_voltage(1);
	} else {
		ui->flags |= USB_FLAG_VBUS_OFFLINE;
	}
	/* online->switch to USB, offline->switch to uart */
	if (ui->usb_uart_switch)
		ui->usb_uart_switch(!vbus);
#endif
	queue_work(ui->usb_wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);
}

static int usb_free(struct usb_info *ui, int ret)
{
	USB_INFO("%s(%d)\n", __func__, ret);

	if (ui->irq)
		free_irq(ui->irq, 0);
	if (ui->pool)
		dma_pool_destroy(ui->pool);
	if (ui->dma)
		dma_free_coherent(&ui->pdev->dev, 4096, ui->buf, ui->dma);
	if (ui->addr)
		iounmap(ui->addr);
	if (ui->clk)
		clk_put(ui->clk);
	if (ui->pclk)
		clk_put(ui->pclk);
	if (ui->otgclk)
		clk_put(ui->otgclk);
	if (ui->coreclk)
		clk_put(ui->coreclk);
	if (ui->ebi1clk)
		clk_put(ui->ebi1clk);
	kfree(ui);
	return ret;
}

static void usb_do_work_check_vbus(struct usb_info *ui)
{
	unsigned long iflags;

	spin_lock_irqsave(&ui->lock, iflags);
#if defined(CONFIG_USB_BYPASS_VBUS_NOTIFY)
	ui->flags |= USB_FLAG_VBUS_ONLINE;
	USB_INFO("fake vbus\n");
#else
	if (vbus)
		ui->flags |= USB_FLAG_VBUS_ONLINE;
	else
		ui->flags |= USB_FLAG_VBUS_OFFLINE;
#endif
	spin_unlock_irqrestore(&ui->lock, iflags);
}

static void usb_lpm_enter(struct usb_info *ui)
{
	unsigned long iflags;

	if (ui->in_lpm)
		return;

	USB_INFO("lpm enter\n");
	spin_lock_irqsave(&ui->lock, iflags);
	usb_suspend_phy(ui);
	if (ui->otgclk)
		clk_disable(ui->otgclk);
	clk_disable(ui->pclk);
	clk_disable(ui->clk);
	if (ui->coreclk)
		clk_disable(ui->coreclk);
	clk_set_rate(ui->ebi1clk, 0);
	ui->in_lpm = 1;
	spin_unlock_irqrestore(&ui->lock, iflags);

	if (board_mfg_mode() == 1) {/*for MFG adb unstable in FROYO ROM*/
		USB_INFO("idle_wake_unlock and perf unlock\n");
		wake_unlock(&vbus_idle_wake_lock);
		if (is_perf_lock_active(&usb_perf_lock))
			perf_unlock(&usb_perf_lock);
	}
}

static void usb_lpm_exit(struct usb_info *ui)
{
	unsigned long iflags;

	if (!ui->in_lpm)
		return;

	USB_INFO("lpm exit\n");
	spin_lock_irqsave(&ui->lock, iflags);
#ifndef CONFIG_ARCH_MSM8X60 /* FIXME */
	clk_set_rate(ui->ebi1clk, acpuclk_get_max_axi_rate());
#endif
	udelay(10);
	if (ui->coreclk)
		clk_enable(ui->coreclk);
	clk_enable(ui->clk);
	clk_enable(ui->pclk);
	if (ui->otgclk)
		clk_enable(ui->otgclk);
	usb_wakeup_phy(ui);
	ui->in_lpm = 0;
	spin_unlock_irqrestore(&ui->lock, iflags);

	if (board_mfg_mode() == 1) {/*for MFG adb unstable in FROYO ROM*/
		USB_INFO("idle_wake_lock and perf lock\n");
		wake_lock(&vbus_idle_wake_lock);
		if (!is_perf_lock_active(&usb_perf_lock))
			perf_lock(&usb_perf_lock);
	}
}

static void do_usb_hub_disable(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, usb_hub_work);

	if (ui->usb_hub_enable)
		ui->usb_hub_enable(false);
}

#ifdef CONFIG_DOCK_ACCESSORY_DETECT
static irqreturn_t dock_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;
	disable_irq_nosync(ui->dockpin_irq);
	cancel_delayed_work(&ui->dock_work);
	queue_delayed_work(ui->usb_wq, &ui->dock_work_isr, DOCK_DET_DELAY);
	return IRQ_HANDLED;
}
static void dock_isr_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w,
			struct usb_info, dock_work_isr.work);
	ui->dock_pin_state = gpio_get_value(ui->dock_pin_gpio);

	if (ui->dock_pin_state == 1)
		set_irq_type(ui->dockpin_irq, IRQF_TRIGGER_LOW);
	else
		set_irq_type(ui->dockpin_irq, IRQF_TRIGGER_HIGH);
	queue_delayed_work(ui->usb_wq, &ui->dock_work, DOCK_DET_DELAY);
	enable_irq(ui->dockpin_irq);
}
static void dock_detect_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, dock_work.work);
	int value;

	value = gpio_get_value(ui->dock_pin_gpio);
	USB_INFO("%s: dock_pin = %s\n", __func__, value ? "high" : "low");
	if (ui->dock_pin_state != value && (ui->dock_pin_state & 0x80) == 0) {
		USB_ERR("%s: dock_pin_state changed\n", __func__);
		return;
	}

	if (value == 0 && vbus) {
		if (ui->accessory_type == 3)
			return;
		set_irq_type(ui->dockpin_irq, IRQF_TRIGGER_HIGH);
		switch_set_state(&dock_switch, DOCK_STATE_DESK);
		ui->accessory_type = 3;
		USB_INFO("dock: set state %d\n", DOCK_STATE_DESK);
	} else {
		if (ui->accessory_type == 0)
			return;
		set_irq_type(ui->dockpin_irq, IRQF_TRIGGER_LOW);
		switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
		ui->accessory_type = 0;
		USB_INFO("dock: set state %d\n", DOCK_STATE_UNDOCKED);
	}
}
static void dock_detect_init(struct usb_info *ui)
{
	int ret;

	if (ui->dock_pin_gpio == 0)
		return;
	if (ui->dockpin_irq == 0)
		ui->dockpin_irq = gpio_to_irq(ui->dock_pin_gpio);
	if (!vbus)
		set_irq_flags(ui->dockpin_irq, IRQF_VALID | IRQF_NOAUTOEN);
	ret = request_irq(ui->dockpin_irq, dock_interrupt,
				IRQF_TRIGGER_LOW, "dock_irq", ui);
	if (ret < 0) {
		USB_ERR("[GPIO DOCK] %s: request_irq failed\n", __func__);
		return;
	}
	USB_INFO("%s: dock irq %d\n", __func__, ui->dockpin_irq);
	ret = set_irq_wake(ui->dockpin_irq, 1);
	if (ret < 0) {
		USB_ERR("[GPIO DOCK] %s: set_irq_wake failed\n", __func__);
		goto err;
	}

	if (switch_dev_register(&dock_switch) < 0) {
		USB_ERR("[GPIO DOCK] fail to register dock switch!\n");
		goto err;
	}

	ret = device_create_file(dock_switch.dev, &dev_attr_status);
	if (ret != 0)
		USB_WARNING("dev_attr_status failed\n");

	return;

err:
	free_irq(ui->dockpin_irq, 0);
}
#endif


#ifdef CONFIG_USB_ACCESSORY_DETECT
static void carkit_detect(struct usb_info *ui)
{
	unsigned n;
	int value;
	unsigned in_lpm;

	msleep(100);
	value = gpio_get_value(ui->usb_id_pin_gpio);
	USB_INFO("%s: usb ID pin = %d\n", __func__, value);
	in_lpm = ui->in_lpm;
	if (value == 0) {
		if (in_lpm)
			usb_lpm_exit(ui);

		n = readl(USB_OTGSC);
		/* ID pull-up register */
		writel(n | OTGSC_IDPU, USB_OTGSC);

		msleep(100);
		n =  readl(USB_OTGSC);

		if (n & OTGSC_ID) {
			USB_INFO("carkit inserted\n");
			if ((board_mfg_mode() == 0) || (board_mfg_mode() == 1 &&
				ui->mfg_usb_carkit_enable == 1)) {
				switch_set_state(&dock_switch, DOCK_STATE_CAR);
				USB_INFO("carkit: set state %d\n", DOCK_STATE_CAR);
			}
			ui->accessory_type = 1;
		} else
			ui->accessory_type = 0;
		if (in_lpm)
			usb_lpm_enter(ui);
	} else {
		if (ui->accessory_type == 1)
			USB_INFO("carkit removed\n");
		switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
		USB_INFO("carkit: set state %d\n", DOCK_STATE_UNDOCKED);
		ui->accessory_type = 0;
	}
}

#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
static void mhl_detect(struct usb_info *ui)
{
	uint32_t adc_value = 0xffffffff;

	if (ui->config_usb_id_gpios)
		ui->config_usb_id_gpios(1);

	htc_get_usb_accessory_adc_level(&adc_value);
	USB_INFO("[2nd] accessory adc = 0x%x\n", adc_value);

	if (adc_value >= 0x5999 && adc_value <= 0x76B0) {
		USB_INFO("MHL inserted\n");
		if (ui->usb_mhl_switch)
			ui->usb_mhl_switch(1);
		ui->accessory_type = 4;
	}
	if (ui->config_usb_id_gpios)
		ui->config_usb_id_gpios(0);
}

static void accessory_detect_by_adc(struct usb_info *ui)
{
	int value;

	msleep(100);

	value = gpio_get_value(ui->usb_id_pin_gpio);
	USB_INFO("%s: usb ID pin = %d\n", __func__, value);

	if (value == 0) {
		uint32_t adc_value = 0xffffffff;
		htc_get_usb_accessory_adc_level(&adc_value);
		USB_INFO("accessory adc = 0x%x\n", adc_value);
		if (adc_value >= 0x2112 && adc_value <= 0x3D53) {
			USB_INFO("headset inserted\n");
			ui->accessory_type = 2;
			headset_ext_detect(USB_AUDIO_OUT);
		} else if (adc_value >= 0x1174 && adc_value <= 0x1E38) {
			USB_INFO("carkit inserted\n");
			ui->accessory_type = 1;
			if ((board_mfg_mode() == 0) || (board_mfg_mode() == 1 &&
				ui->mfg_usb_carkit_enable == 1)) {
				switch_set_state(&dock_switch, DOCK_STATE_CAR);
				USB_INFO("carkit: set state %d\n", DOCK_STATE_CAR);
			}
		} else if (adc_value >= 0x0 && adc_value < 0x1174) {
			mhl_detect(ui);
		} else
			ui->accessory_type = 0;
	} else {
		switch (ui->accessory_type) {
		case 1:
			USB_INFO("carkit removed\n");
			switch_set_state(&dock_switch, DOCK_STATE_UNDOCKED);
			ui->accessory_type = 0;
			break;
		case 2:
			USB_INFO("headset removed\n");
			headset_ext_detect(USB_NO_HEADSET);
			ui->accessory_type = 0;
			break;
		case 3:
			/*MHL*/
			break;
		default:
			break;
		}
	}
}
#endif

static void accessory_detect_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, detect_work);
	int value;

	if (!ui->accessory_detect)
		return;

	if (ui->accessory_detect == 1)
		carkit_detect(ui);
#ifdef CONFIG_USB_ACCESSORY_DETECT_BY_ADC
	else if (ui->accessory_detect == 2)
		accessory_detect_by_adc(ui);
#endif

	value = gpio_get_value(ui->usb_id_pin_gpio);
	if (value == 0)
		set_irq_type(ui->idpin_irq, IRQF_TRIGGER_HIGH);
	else
		set_irq_type(ui->idpin_irq, IRQF_TRIGGER_LOW);
	enable_irq(ui->idpin_irq);
}

static irqreturn_t usbid_interrupt(int irq, void *data)
{
	struct usb_info *ui = data;

	disable_irq_nosync(ui->idpin_irq);
	USB_INFO("id interrupt\n");
	queue_work(ui->usb_wq, &ui->detect_work);
	return IRQ_HANDLED;
}

static void accessory_detect_init(struct usb_info *ui)
{
	int ret;
	USB_INFO("%s: id pin %d\n", __func__, ui->usb_id_pin_gpio);

	if (ui->usb_id_pin_gpio == 0)
		return;
	if (ui->idpin_irq == 0)
		ui->idpin_irq = gpio_to_irq(ui->usb_id_pin_gpio);

	ret = request_irq(ui->idpin_irq, usbid_interrupt,
				IRQF_TRIGGER_LOW,
				"car_kit_irq", ui);
	if (ret < 0) {
		USB_ERR("%s: request_irq failed\n", __func__);
		return;
	}

	ret = set_irq_wake(ui->idpin_irq, 1);
	if (ret < 0) {
		USB_ERR("%s: set_irq_wake failed\n", __func__);
		goto err;
	}

	if (switch_dev_register(&dock_switch) < 0) {
		USB_ERR(" fail to register dock switch!\n");
		goto err;
	}

	ret = device_create_file(dock_switch.dev, &dev_attr_status);
	if (ret != 0)
		USB_WARNING("dev_attr_status failed\n");

	return;
err:
	free_irq(ui->idpin_irq, 0);
}

#endif

#define DELAY_FOR_CHECK_CHG msecs_to_jiffies(300)

static void charger_detect_by_uart(struct usb_info *ui)
{
	int is_china_ac;

	if (!vbus)
		return;

	/*UART*/
	if (ui->usb_uart_switch)
		ui->usb_uart_switch(1);

	is_china_ac = ui->china_ac_detect();

	if (is_china_ac) {
		ui->connect_type = CONNECT_TYPE_AC;
		queue_work(ui->usb_wq, &ui->notifier_work);
		usb_lpm_enter(ui);
		USB_INFO("AC charger\n");
	} else {
		ui->connect_type = CONNECT_TYPE_UNKNOWN;
		queue_delayed_work(ui->usb_wq, &ui->chg_work,
			DELAY_FOR_CHECK_CHG);
		USB_INFO("not AC charger\n");

		/*set uart to gpo*/
		if (ui->serial_debug_gpios)
			ui->serial_debug_gpios(0);
		/*turn on USB HUB*/
		if (ui->usb_hub_enable)
			ui->usb_hub_enable(1);

		/*USB*/
		if (ui->usb_uart_switch)
			ui->usb_uart_switch(0);

		usb_lpm_exit(ui);
		usb_reset(ui);
	}
}

static void charger_detect(struct usb_info *ui)
{
	if (!vbus)
		return;
	msleep(10);
	/* detect shorted D+/D-, indicating AC power */
	if ((readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {
		USB_INFO("not AC charger\n");
		ui->connect_type = CONNECT_TYPE_UNKNOWN;
		queue_delayed_work(ui->usb_wq, &ui->chg_work,
			DELAY_FOR_CHECK_CHG);
		mod_timer(&ui->ac_detect_timer, jiffies + (3 * HZ));
	} else {
		if (ui->usb_id_pin_gpio != 0) {
			if (gpio_get_value(ui->usb_id_pin_gpio) == 0) {
				USB_INFO("9V AC charger\n");
				ui->connect_type = CONNECT_TYPE_9V_AC;
			} else {
				USB_INFO("AC charger\n");
				ui->connect_type = CONNECT_TYPE_AC;
			}
		} else {
			USB_INFO("AC charger\n");
			ui->connect_type = CONNECT_TYPE_AC;
		}
		queue_work(ui->usb_wq, &ui->notifier_work);
		writel(0x00080000, USB_USBCMD);
		msleep(10);
		usb_lpm_enter(ui);
		if (ui->change_phy_voltage)
			ui->change_phy_voltage(0);
	}
}

static void check_charger(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, chg_work.work);
	if (disable_charger) {
		USB_INFO("disable charger\n");
		if (ui->disable_usb_charger)
			ui->disable_usb_charger();
		disable_charger = 0;
		return;
	}
	/* unknown charger */
	if (vbus && ui->connect_type == CONNECT_TYPE_UNKNOWN)
		queue_work(ui->usb_wq, &ui->notifier_work);
}

static void charger_detect_by_9v_gpio(struct usb_info *ui)
{
	if (!vbus)
		return;
	msleep(10);
	if (gpio_get_value(ui->ac_9v_gpio) == 0) {
		printk(KERN_INFO "not 9V AC charger\n");
		ui->connect_type = CONNECT_TYPE_UNKNOWN;
	} else {
		printk(KERN_INFO "9V AC charger\n");
		ui->connect_type = CONNECT_TYPE_9V_AC;

		queue_work(ui->usb_wq, &ui->notifier_work);
		writel(0x00080000, USB_USBCMD);
		msleep(10);
		usb_lpm_enter(ui);
	}
}


static void usb_do_work(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, work);
	unsigned long iflags;
	unsigned flags, _vbus;

	for (;;) {
		spin_lock_irqsave(&ui->lock, iflags);
		flags = ui->flags;
		ui->flags = 0;
		_vbus = vbus;
		spin_unlock_irqrestore(&ui->lock, iflags);

		/* give up if we have nothing to do */
		if (flags == 0)
			break;

		switch (ui->state) {
		case USB_STATE_IDLE:
			if (flags & USB_FLAG_START) {
				USB_INFO("hsusb: IDLE -> ONLINE\n");

				usb_lpm_exit(ui);
				usb_reset(ui);
				if (ui->china_ac_detect)
					charger_detect_by_uart(ui);
				else if (ui->ac_9v_gpio)
					charger_detect_by_9v_gpio(ui);
				else
					charger_detect(ui);

				ui->state = USB_STATE_ONLINE;
#ifdef CONFIG_USB_ACCESSORY_DETECT
				if (ui->accessory_detect)
					accessory_detect_init(ui);
#endif
				usb_do_work_check_vbus(ui);
			}
			break;
		case USB_STATE_ONLINE:
			/* If at any point when we were online, we received
			 * the signal to go offline, we must honor it
			 */
			if (flags & USB_FLAG_VBUS_OFFLINE) {
				USB_INFO("hsusb: ONLINE -> OFFLINE\n");

				atomic_set(&ui->running, 0);
				atomic_set(&ui->online, 0);
				/* synchronize with irq context */
				spin_lock_irqsave(&ui->lock, iflags);
				writel(0x00080000, USB_USBCMD);
				spin_unlock_irqrestore(&ui->lock, iflags);

				if (ui->connect_type != CONNECT_TYPE_NONE) {
					ui->connect_type = CONNECT_TYPE_NONE;
					queue_work(ui->usb_wq, &ui->notifier_work);
				}
				if (ui->in_lpm) {
					usb_lpm_exit(ui);
					msleep(5);
				}

				handle_notify_offline(ui);

				if (ui->phy_reset)
					ui->phy_reset();

				/* power down phy, clock down usb */
				usb_lpm_enter(ui);
				ui->ac_detect_count = 0;
				del_timer_sync(&ui->ac_detect_timer);

				switch_set_state(&ui->sdev, 0);
				ui->state = USB_STATE_OFFLINE;
				usb_do_work_check_vbus(ui);
				break;
			}

			if (flags & USB_FLAG_CONFIGURED) {
				switch_set_state(&ui->sdev, atomic_read(&ui->online));
				break;
			}

			if (flags & USB_FLAG_RESET) {
				USB_INFO("hsusb: ONLINE -> RESET\n");
				if (ui->connect_type == CONNECT_TYPE_AC) {
					USB_INFO("hsusb: RESET -> ONLINE\n");
					break;
				}

				atomic_set(&ui->online, 0);
				spin_lock_irqsave(&ui->lock, iflags);
				msm72k_pullup(&ui->gadget, 0);
				spin_unlock_irqrestore(&ui->lock, iflags);
				usb_reset(ui);
				switch_set_state(&ui->sdev, 0);
				USB_INFO("hsusb: RESET -> ONLINE\n");
				break;
			}
			break;
		case USB_STATE_OFFLINE:
			/* If we were signaled to go online and vbus is still
			 * present when we received the signal, go online.
			 */
			if ((flags & USB_FLAG_VBUS_ONLINE) && _vbus) {
				USB_INFO("hsusb: OFFLINE -> ONLINE\n");

				if (ui->china_ac_detect)
					charger_detect_by_uart(ui);
				else if (ui->ac_9v_gpio) {
					usb_lpm_exit(ui);
					usb_reset(ui);
					charger_detect_by_9v_gpio(ui);
				} else {
					usb_lpm_exit(ui);
					usb_reset(ui);
					charger_detect(ui);
				}

				ui->state = USB_STATE_ONLINE;
				usb_do_work_check_vbus(ui);
			}
			break;
		}
	}
}

/* FIXME - the callers of this function should use a gadget API instead.
 * This is called from htc_battery.c and board-halibut.c
 * WARNING - this can get called before this driver is initialized.
 */
void msm_hsusb_set_vbus_state(int online)
{
	unsigned long flags = 0;
	struct usb_info *ui = the_usb_info;
	USB_INFO("%s: %d\n", __func__, online);

	if (ui)
		spin_lock_irqsave(&ui->lock, flags);
	if (vbus != online) {
		vbus = online;
		if (ui) {
			if (online) {
				ui->flags |= USB_FLAG_VBUS_ONLINE;
			} else {
				ui->flags |= USB_FLAG_VBUS_OFFLINE;
			}

			if (ui->change_phy_voltage)
				ui->change_phy_voltage(online);

			if (online) {
				/*USB*/
				if (ui->usb_uart_switch)
					ui->usb_uart_switch(0);
			} else {
				/*turn off USB HUB*/
				if (ui->usb_hub_enable)
					queue_work(ui->usb_wq, &ui->usb_hub_work);

				/*UART*/
				if (ui->usb_uart_switch)
					ui->usb_uart_switch(1);
				/*configure uart pin to alternate function*/
				if (ui->serial_debug_gpios)
					ui->serial_debug_gpios(1);

				/*path should be switched to usb after mhl cable is removed*/
				if (ui->usb_mhl_switch && ui->accessory_type == 4) {
					USB_INFO("MHL removed\n");
					ui->usb_mhl_switch(0);
					ui->accessory_type = 0;
				}
			}

			queue_work(ui->usb_wq, &ui->work);
		}
	}
	if (ui) {
		spin_unlock_irqrestore(&ui->lock, flags);
#ifdef CONFIG_DOCK_ACCESSORY_DETECT
		if (ui->dock_detect) {
			if (vbus)
				enable_irq(ui->dockpin_irq);
			else {
				disable_irq_nosync(ui->dockpin_irq);
				if (cancel_delayed_work_sync(&ui->dock_work_isr))
					enable_irq(ui->dockpin_irq);

				if (cancel_delayed_work_sync(&ui->dock_work)) {
					if (ui->dock_pin_state == 0)
						set_irq_type(ui->dockpin_irq,
							IRQF_TRIGGER_LOW);
				}
				if (ui->accessory_type == 3) {
					ui->dock_pin_state |= 0x80;
					queue_delayed_work(ui->usb_wq, &ui->dock_work, 0);
				}
			}
		}
#endif
	}
}

#if defined(CONFIG_DEBUG_FS) && 0

void usb_function_reenumerate(void)
{
	struct usb_info *ui = the_usb_info;

	/* disable and re-enable the D+ pullup */
	msm72k_pullup(&ui->gadget, false);
	msleep(10);
	msm72k_pullup(&ui->gadget, true);
}

static char debug_buffer[PAGE_SIZE];

static ssize_t debug_read_status(struct file *file, char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	char *buf = debug_buffer;
	unsigned long flags;
	struct msm_endpoint *ept;
	struct msm_request *req;
	int n;
	int i = 0;

	spin_lock_irqsave(&ui->lock, flags);

	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "regs: setup=%08x prime=%08x stat=%08x done=%08x\n",
		       readl(USB_ENDPTSETUPSTAT),
		       readl(USB_ENDPTPRIME),
		       readl(USB_ENDPTSTAT),
		       readl(USB_ENDPTCOMPLETE));
	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "regs:   cmd=%08x   sts=%08x intr=%08x port=%08x\n\n",
		       readl(USB_USBCMD),
		       readl(USB_USBSTS),
		       readl(USB_USBINTR),
		       readl(USB_PORTSC));


	for (n = 0; n < 32; n++) {
		ept = ui->ept + n;
		if (ept->ep.maxpacket == 0)
			continue;

		i += scnprintf(buf + i, PAGE_SIZE - i,
			       "ept%d %s cfg=%08x active=%08x next=%08x info=%08x\n",
			       ept->num, (ept->flags & EPT_FLAG_IN) ? "in " : "out",
			       ept->head->config, ept->head->active,
			       ept->head->next, ept->head->info);

		for (req = ept->req; req; req = req->next)
			i += scnprintf(buf + i, PAGE_SIZE - i,
				       "  req @%08x next=%08x info=%08x page0=%08x %c %c\n",
				       req->item_dma, req->item->next,
				       req->item->info, req->item->page0,
				       req->busy ? 'B' : ' ',
				       req->live ? 'L' : ' '
				);
	}

	i += scnprintf(buf + i, PAGE_SIZE - i,
		       "phy failure count: %d\n", ui->phy_fail_count);

	spin_unlock_irqrestore(&ui->lock, flags);

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static ssize_t debug_write_reset(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct usb_info *ui = file->private_data;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	ui->flags |= USB_FLAG_RESET;
	queue_work(ui->usb_wq, &ui->work);
	spin_unlock_irqrestore(&ui->lock, flags);

	return count;
}

static ssize_t debug_write_cycle(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	usb_function_reenumerate();
	return count;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

const struct file_operations debug_stat_ops = {
	.open = debug_open,
	.read = debug_read_status,
};

const struct file_operations debug_reset_ops = {
	.open = debug_open,
	.write = debug_write_reset,
};

const struct file_operations debug_cycle_ops = {
	.open = debug_open,
	.write = debug_write_cycle,
};

static void usb_debugfs_init(struct usb_info *ui)
{
	struct dentry *dent;
	dent = debugfs_create_dir("usb", 0);
	if (IS_ERR(dent))
		return;

	debugfs_create_file("status", 0444, dent, ui, &debug_stat_ops);
	debugfs_create_file("reset", 0220, dent, ui, &debug_reset_ops);
	debugfs_create_file("cycle", 0220, dent, ui, &debug_cycle_ops);
}
#else
static void usb_debugfs_init(struct usb_info *ui) {}
#endif

static int
msm72k_enable(struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	unsigned char ep_type =
			desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	if (ep_type == USB_ENDPOINT_XFER_BULK)
		_ep->maxpacket = le16_to_cpu(desc->wMaxPacketSize);
	else
		_ep->maxpacket = le16_to_cpu(64);
	config_ept(ept);
	usb_ept_enable(ept, 1, ep_type);
	return 0;
}

static int msm72k_disable(struct usb_ep *_ep)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);

	usb_ept_enable(ept, 0, 0);
	flush_endpoint(ept);
	return 0;
}

static struct usb_request *
msm72k_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	return usb_ept_alloc_req(to_msm_endpoint(_ep), 0, gfp_flags);
}

static void
msm72k_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct msm_request *req = to_msm_request(_req);
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	struct usb_info *ui = ept->ui;
	unsigned long flags;
	int dead = 0;

	spin_lock_irqsave(&ui->lock, flags);
	/* defer freeing resources if request is still busy */
	if (req->busy)
		dead = req->dead = 1;
	spin_unlock_irqrestore(&ui->lock, flags);

	/* if req->dead, then we will clean up when the request finishes */
	if (!dead)
		do_free_req(ui, req);
}

static int
msm72k_queue(struct usb_ep *_ep, struct usb_request *req, gfp_t gfp_flags)
{
	struct msm_endpoint *ep = to_msm_endpoint(_ep);
	struct usb_info *ui = ep->ui;

	if (ep == &ui->ep0in) {
		struct msm_request *r = to_msm_request(req);
		if (!req->length)
			goto ep_queue_done;
		else {
			if (atomic_read(&ui->ep0_dir) == USB_DIR_OUT) {
				ep = &ui->ep0out;
				ep->ep.driver_data = ui->ep0in.ep.driver_data;
			}
			/* ep0_queue_ack_complete queue a receive for ACK before
			** calling req->complete
			*/
			r->gadget_complete = req->complete;
			req->complete = ep0_queue_ack_complete;
		}
	}
ep_queue_done:
	return usb_ept_queue_xfer(ep, req);
}

static int msm72k_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct msm_endpoint *ep = to_msm_endpoint(_ep);
	struct msm_request *req = to_msm_request(_req);
	struct usb_info *ui = ep->ui;

	struct msm_request *cur, *prev;
	unsigned long flags;

	if (!_ep || !_req)
		return -EINVAL;

	spin_lock_irqsave(&ui->lock, flags);
	cur = ep->req;
	prev = NULL;

	while (cur != 0) {
		if (cur == req) {
			req->busy = 0;
			req->live = 0;
			req->req.status = -ECONNRESET;
			req->req.actual = 0;
			if (req->req.complete) {
				spin_unlock_irqrestore(&ui->lock, flags);
				req->req.complete(&ep->ep, &req->req);
				spin_lock_irqsave(&ui->lock, flags);
			}
			if (req->dead)
				do_free_req(ui, req);
			/* remove from linked list */
			if (prev)
				prev->next = cur->next;
			else
				ep->req = cur->next;
			if (ep->last == cur)
				ep->last = prev;
			/* break from loop */
			cur = NULL;
		} else {
			prev = cur;
			cur = cur->next;
		}
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	return 0;
}

static int
msm72k_set_halt(struct usb_ep *_ep, int value)
{
	struct msm_endpoint *ept = to_msm_endpoint(_ep);
	struct usb_info *ui = ept->ui;
	unsigned int in = ept->flags & EPT_FLAG_IN;
	unsigned int n;
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		if (value)
			n |= CTRL_TXS;
		else {
			n &= ~CTRL_TXS;
			n |= CTRL_TXR;
		}
	} else {
		if (value)
			n |= CTRL_RXS;
		else {
			n &= ~CTRL_RXS;
			n |= CTRL_RXR;
		}
	}
	writel(n, USB_ENDPTCTRL(ept->num));
	spin_unlock_irqrestore(&ui->lock, flags);

	return 0;
}

static int
msm72k_fifo_status(struct usb_ep *_ep)
{
	return -EOPNOTSUPP;
}

static void
msm72k_fifo_flush(struct usb_ep *_ep)
{
	flush_endpoint(to_msm_endpoint(_ep));
}

static const struct usb_ep_ops msm72k_ep_ops = {
	.enable		= msm72k_enable,
	.disable	= msm72k_disable,

	.alloc_request	= msm72k_alloc_request,
	.free_request	= msm72k_free_request,

	.queue		= msm72k_queue,
	.dequeue	= msm72k_dequeue,

	.set_halt	= msm72k_set_halt,
	.fifo_status	= msm72k_fifo_status,
	.fifo_flush	= msm72k_fifo_flush,
};

static int msm72k_get_frame(struct usb_gadget *_gadget)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);

	/* frame number is in bits 13:3 */
	return (readl(USB_FRINDEX) >> 3) & 0x000007FF;
}

/* VBUS reporting logically comes from a transceiver */
static int msm72k_udc_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	msm_hsusb_set_vbus_state(is_active);
	return 0;
}

/* drivers may have software control over D+ pullup */
static int msm72k_pullup(struct usb_gadget *_gadget, int is_active)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);

	u32 cmd = (8 << 16);

	/* disable/enable D+ pullup */
	if (is_active) {
		USB_INFO("msm_hsusb: enable pullup\n");
		writel(cmd | 1, USB_USBCMD);
	} else {
		USB_INFO("msm_hsusb: disable pullup\n");
		writel(cmd, USB_USBCMD);

#ifndef CONFIG_ARCH_MSM7X00A
		ulpi_write(ui, 0x48, 0x04);
#endif
	}

	return 0;
}

static int msm72k_wakeup(struct usb_gadget *_gadget)
{
	struct usb_info *ui = container_of(_gadget, struct usb_info, gadget);
	unsigned long flags;

	if (!atomic_read(&ui->remote_wakeup)) {
		USB_ERR("%s: remote wakeup not supported\n", __func__);
		return -ENOTSUPP;
	}

	if (!atomic_read(&ui->online)) {
		USB_ERR("%s: device is not configured\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&ui->lock, flags);
	if ((readl(USB_PORTSC) & PORTSC_SUSP) == PORTSC_SUSP) {
		USB_INFO("%s: enabling force resume\n", __func__);
		writel(readl(USB_PORTSC) | PORTSC_FPR, USB_PORTSC);
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	return 0;
}

static const struct usb_gadget_ops msm72k_ops = {
	.get_frame	= msm72k_get_frame,
	.vbus_session	= msm72k_udc_vbus_session,
	.pullup		= msm72k_pullup,
	/* .wakeup		= msm72k_wakeup, */
};

static ssize_t usb_remote_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_info *ui = the_usb_info;

	msm72k_wakeup(&ui->gadget);

	return count;
}
static DEVICE_ATTR(wakeup, S_IWUSR, 0, usb_remote_wakeup);

static void ac_detect_expired(unsigned long _data)
{
	struct usb_info *ui = (struct usb_info *) _data;
	u32 delay = 0;

	USB_INFO("%s: count = %d, connect_type = 0x%04x\n", __func__,
			ui->ac_detect_count, ui->connect_type);

	if (ui->connect_type == CONNECT_TYPE_USB || ui->ac_detect_count >= 3)
		return;

	/* detect shorted D+/D-, indicating AC power */
	if ((readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {

		/* Some carkit can't be recognized as AC mode.
		 * Add SW solution here to notify battery driver should
		 * work as AC charger when car mode activated.
		 */
		if (ui->accessory_type == 1) {
			USB_INFO("car mode charger\n");
			ui->connect_type = CONNECT_TYPE_AC;
			queue_work(ui->usb_wq, &ui->notifier_work);
			writel(0x00080000, USB_USBCMD);
			mdelay(10);
			usb_lpm_enter(ui);
			return;
		}

		ui->ac_detect_count++;
		/* detect delay: 3 sec, 5 sec, 10 sec */
		if (ui->ac_detect_count == 1)
			delay = 5 * HZ;
		else if (ui->ac_detect_count == 2)
			delay = 10 * HZ;

		mod_timer(&ui->ac_detect_timer, jiffies + delay);
	} else {
		if (ui->usb_id_pin_gpio != 0) {
			if (gpio_get_value(ui->usb_id_pin_gpio) == 0) {
				USB_INFO("9V AC charger\n");
				ui->connect_type = CONNECT_TYPE_9V_AC;
			} else {
				USB_INFO("AC charger\n");
				ui->connect_type = CONNECT_TYPE_AC;
			}
		} else {
			USB_INFO("AC charger\n");
			ui->connect_type = CONNECT_TYPE_AC;
		}
		queue_work(ui->usb_wq, &ui->notifier_work);
		writel(0x00080000, USB_USBCMD);
		mdelay(10);
		usb_lpm_enter(ui);
	}
}

static int msm72k_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct usb_info *ui;
	int irq;
	int ret;
	char *serialno = "000000000000";

	USB_INFO("msm72k_probe\n");
	ui = kzalloc(sizeof(struct usb_info), GFP_KERNEL);
	if (!ui)
		return -ENOMEM;

	spin_lock_init(&ui->lock);
	ui->pdev = pdev;

	if (pdev->dev.platform_data) {
		struct msm_hsusb_platform_data *pdata = pdev->dev.platform_data;
		ui->phy_reset = pdata->phy_reset;
		ui->phy_init_seq = pdata->phy_init_seq;
		ui->usb_connected = pdata->usb_connected;
		ui->usb_uart_switch = pdata->usb_uart_switch;
		ui->serial_debug_gpios = pdata->serial_debug_gpios;
		ui->usb_hub_enable = pdata->usb_hub_enable;
		ui->china_ac_detect = pdata->china_ac_detect;
		ui->disable_usb_charger = pdata->disable_usb_charger;
		ui->change_phy_voltage = pdata->change_phy_voltage;
		ui->ldo_init = pdata->ldo_init;
		ui->ldo_enable = pdata->ldo_enable;
		ui->usb_mhl_switch = pdata->usb_mhl_switch;
		ui->ac_9v_gpio = pdata->ac_9v_gpio;

		if (ui->ldo_init)
			ui->ldo_init(1);

		if (ui->ldo_enable)
			ui->ldo_enable(1);

		ui->accessory_detect = pdata->accessory_detect;
		USB_INFO("accessory detect %d\n", ui->accessory_detect);
		ui->usb_id_pin_gpio = pdata->usb_id_pin_gpio;
		USB_INFO("id_pin_gpio %d\n", pdata->usb_id_pin_gpio);

		ui->dock_detect = pdata->dock_detect;
		USB_INFO("dock detect %d\n", ui->dock_detect);
		ui->dock_pin_gpio = pdata->dock_pin_gpio;
		USB_INFO("dock pin gpio %d\n", ui->dock_pin_gpio);

		ui->idpin_irq = pdata->id_pin_irq;
		if (pdata->config_usb_id_gpios)
			ui->config_usb_id_gpios = pdata->config_usb_id_gpios;
	}

	irq = platform_get_irq(pdev, 0);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || (irq < 0))
		return usb_free(ui, -ENODEV);

	ui->addr = ioremap(res->start, 4096);
	if (!ui->addr)
		return usb_free(ui, -ENOMEM);

	ui->buf = dma_alloc_coherent(&pdev->dev, 4096, &ui->dma, GFP_KERNEL);
	if (!ui->buf)
		return usb_free(ui, -ENOMEM);

	ui->pool = dma_pool_create("msm72k_udc", NULL, 32, 32, 0);
	if (!ui->pool)
		return usb_free(ui, -ENOMEM);

	USB_INFO("msm72k_probe() io=%p, irq=%d, dma=%p(%x)\n",
	       ui->addr, irq, ui->buf, ui->dma);

#ifdef CONFIG_ARCH_MSM7X30
	msm_hsusb_rpc_connect();
#endif
	ui->clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(ui->clk))
		return usb_free(ui, PTR_ERR(ui->clk));

	ui->pclk = clk_get(&pdev->dev, "usb_hs_pclk");
	if (IS_ERR(ui->pclk))
		return usb_free(ui, PTR_ERR(ui->pclk));

	ui->otgclk = clk_get(&pdev->dev, "usb_otg_clk");
	if (IS_ERR(ui->otgclk))
		ui->otgclk = NULL;

	ui->coreclk = clk_get(&pdev->dev, "usb_hs_core_clk");
	if (IS_ERR(ui->coreclk))
		ui->coreclk = NULL;

	ui->ebi1clk = clk_get(NULL, "ebi1_clk");
	if (IS_ERR(ui->ebi1clk))
		return usb_free(ui, PTR_ERR(ui->ebi1clk));

	/* clear interrupts before requesting irq */
	if (ui->coreclk)
		clk_enable(ui->coreclk);
	clk_enable(ui->clk);
	clk_enable(ui->pclk);
	if (ui->otgclk)
		clk_enable(ui->otgclk);
	writel(0, USB_USBINTR);
	writel(0, USB_OTGSC);
	if (ui->otgclk)
		clk_disable(ui->otgclk);
	clk_disable(ui->pclk);
	clk_disable(ui->clk);
	if (ui->coreclk)
		clk_disable(ui->coreclk);

	ui->in_lpm = 1;
	ret = request_irq(irq, usb_interrupt, 0, pdev->name, ui);
	if (ret)
		return usb_free(ui, ret);
	enable_irq_wake(irq);
	ui->irq = irq;

	ui->gadget.ops = &msm72k_ops;
	ui->gadget.is_dualspeed = 1;
	device_initialize(&ui->gadget.dev);
	dev_set_name(&ui->gadget.dev, "gadget");
	ui->gadget.dev.parent = &pdev->dev;
	ui->gadget.dev.dma_mask = pdev->dev.dma_mask;

	the_usb_info = ui;

	usb_debugfs_init(ui);

	usb_prepare(ui);

	/* initialize mfg serial number */

	if (board_mfg_mode() == 1) {
		use_mfg_serialno = 1;
		wake_lock_init(&vbus_idle_wake_lock, WAKE_LOCK_IDLE, "usb_idle_lock");
		perf_lock_init(&usb_perf_lock, PERF_LOCK_HIGHEST, "usb");
	} else
		use_mfg_serialno = 0;
	strncpy(mfg_df_serialno, serialno, strlen(serialno));

	ui->ac_detect_count = 0;
	ui->ac_detect_timer.data = (unsigned long) ui;
	ui->ac_detect_timer.function = ac_detect_expired;
	init_timer(&ui->ac_detect_timer);
	return 0;
}

int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct usb_info *ui = the_usb_info;
	int			retval, n;

	if (!driver
			|| driver->speed < USB_SPEED_FULL
			|| !driver->bind
			|| !driver->disconnect
			|| !driver->setup)
		return -EINVAL;
	if (!ui)
		return -ENODEV;
	if (ui->driver)
		return -EBUSY;

	/* first hook up the driver ... */
	ui->driver = driver;
	ui->gadget.dev.driver = &driver->driver;
	ui->gadget.name = driver_name;
	INIT_LIST_HEAD(&ui->gadget.ep_list);
	ui->gadget.ep0 = &ui->ep0in.ep;
	INIT_LIST_HEAD(&ui->gadget.ep0->ep_list);
	ui->gadget.speed = USB_SPEED_UNKNOWN;

	for (n = 1; n < 16; n++) {
		struct msm_endpoint *ept = ui->ept + n;
		list_add_tail(&ept->ep.ep_list, &ui->gadget.ep_list);
		ept->ep.maxpacket = 512;
	}
	for (n = 17; n < 32; n++) {
		struct msm_endpoint *ept = ui->ept + n;
		list_add_tail(&ept->ep.ep_list, &ui->gadget.ep_list);
		ept->ep.maxpacket = 512;
	}

	retval = device_add(&ui->gadget.dev);
	if (retval)
		goto fail;

	retval = driver->bind(&ui->gadget);
	if (retval) {
		USB_INFO("bind to driver %s --> error %d\n",
				driver->driver.name, retval);
		device_del(&ui->gadget.dev);
		goto fail;
	}

	/* create sysfs node for remote wakeup */
	retval = device_create_file(&ui->gadget.dev, &dev_attr_wakeup);
	if (retval != 0)
		USB_INFO("failed to create sysfs entry: (wakeup) error:"
				" (%d)\n", retval);
	USB_INFO("msm72k_udc: registered gadget driver '%s'\n",
			driver->driver.name);

#if defined(CONFIG_USB_BYPASS_VBUS_NOTIFY)
	vbus = 1;
#endif
	usb_start(ui);

	return 0;

fail:
	ui->driver = NULL;
	ui->gadget.dev.driver = NULL;
	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct usb_info *dev = the_usb_info;

	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver || !driver->unbind)
		return -EINVAL;

	device_remove_file(&dev->gadget.dev, &dev_attr_wakeup);
	driver->unbind(&dev->gadget);
	dev->gadget.dev.driver = NULL;
	dev->driver = NULL;

	device_del(&dev->gadget.dev);

	USB_DEBUG("unregistered gadget driver '%s'\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);


static struct platform_driver usb_driver = {
	.probe = msm72k_probe,
	.driver = { .name = "msm_hsusb", },
};

static int __init init(void)
{
	return platform_driver_register(&usb_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	platform_driver_unregister(&usb_driver);
}
module_exit(cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Mike Lockwood, Brian Swetland");
MODULE_LICENSE("GPL");
