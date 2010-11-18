/*
 *  BtLinuxPort.c
 *
 *  Driver for Bluetooth RFCOMM ports
 *
 *  Based on Linux 2.6 serial driver architecture
 *
 *  V 1.0  --
 *  V 1.01  -- Enable opening of tty port independently of btport
 *                  Fixed kernel panic issue if no fasync fd was set.
 *                  Lots of cleanup and fixed warnings.
 *
 */

#include "target.h"
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>

#include "port_api.h"
#include "btport.h"

#include <linux/platform_device.h>
#include <mach/msm_smd.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>

#define DRIVER_VERSION "v1.01"
#define DRIVER_AUTHOR "Satyajit Roy <roys@broadcom.com>"
#define DRIVER_DESC "Bluetooth RFCOMM port driver"


#define PORT_BROADCOM 10
#define BTLINUXPORT_MAJOR 0

#define TXN_MAX 8192
#define RXN_MAX 8192

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 4
#define TX_REQ_MAX 4

/* Module information */
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define BTLINUX_SERIAL_NAME	"BtLinuxPort"
#define BTPORT_DEV_NAME "BtPort"

#define MY_NAME         BTLINUX_SERIAL_NAME
#define UART_NR         2   /* Maximum ports */
#define MAX_INT16               (0xFFFF)

#define _DEBUG_
#define _INFO_

#ifdef _INFO_
#define info(fmt, arg...)                        \
		printk("%s: %s: " fmt "\n",        \
			MY_NAME , __func__ , ## arg);
#else
#define info(fmt, arg...)
#endif


#ifdef _DEBUG_
#define dbg(fmt, arg...)                        \
		printk("%s: %s: " fmt "\n",        \
			MY_NAME , __func__ , ## arg);
#else
#define dbg(fmt, arg...)
#endif

#define err(format, arg...) printk("%s: " format "\n" , MY_NAME , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: " format "\n" , MY_NAME , ## arg)

/****************************************************************************
 *
 *   STRUCTS AND TYPEDEFS
 *
 ****************************************************************************/
#define TABLE_SIZE 10

struct bt_diag_context {
	int online;
	int error;
	int read_avail;
	int write_avail;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;
	atomic_t enable_excl;

	spinlock_t lock;
	spinlock_t lock_reg_num;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;
	struct list_head tx_done;
	wait_queue_head_t read_wq;

	smd_channel_t *ch;
	int is2ARM11;
	char is7E;

	unsigned char id_table[TABLE_SIZE];

	struct platform_device *pdev;
	u64 tx_count; /* To ARM9*/
	u64 rx_count; /* From ARM9*/
};

static struct bt_diag_context _context;

enum data_access {
	data_set_clear = 0,
	data_set_rx,
	data_get_rx,
	data_set_tx,
	data_get_tx,
};

struct bt_request {
	void *buf;          /* pointer to associated data buffer */
	unsigned length;    /* requested transfer length */
	void *context;
	void *device;

	struct list_head list;
};

/*#define RX_BUFFERS_SIZE 4096*/
#define RX_BUFFERS_SIZE 16384
#define TX_BUFFERS_SIZE 16384
struct uart_btlinux_port {
	struct uart_port    port;
	UINT16              line;
	UINT16              bt_port;
	UINT8               port_added;
	UINT8               port_opened;
	UINT8               open_count;
	BOOL                is_open;
	unsigned int        old_status;
	char                rcv_data[RX_BUFFERS_SIZE];
	int                 rcv_data_head;
	int                 rcv_data_tail;
	char                rcv_full;
	char                tx_data[TX_BUFFERS_SIZE];
	int                 tx_data_head;
	int                 tx_data_tail;
	BTLINUXPORTEvent    saved_event;
	wait_queue_head_t   port_wait_q;
	int                 event_result;
	int                 flag;
	wait_queue_head_t   rx_wait_q;
	BOOLEAN             wake_timeout;
	int                 event_data;
	int                 modem_control;
	struct fasync_struct *fasync;
	struct tasklet_struct tx_task;
	UINT8 minor;
};

int btport_major;
static struct uart_btlinux_port btlinux_ports[UART_NR];
static void send_tx_char(struct uart_btlinux_port *up);
char btspp_buf[8192];

#define tx_circ_empty(circ) ((circ)->tx_data_head == (circ)->tx_data_tail)
#define tx_circ_chars_pending(circ) (CIRC_CNT((circ)->tx_data_head, (circ)->tx_data_tail, TX_BUFFERS_SIZE))
#define tx_circ_chars_free(circ) (CIRC_SPACE((circ)->tx_data_head, (circ)->tx_data_tail, TX_BUFFERS_SIZE))

static struct wake_lock btport_wake_lock;

/****************************************************************************
 *
 *
 *
 ****************************************************************************/
struct bt_request *bt_alloc_req(unsigned bufsize)
{
	struct bt_request *req;

	req = kzalloc(sizeof(*req), GFP_ATOMIC);
	if (!req)
		goto fail1;

	if (bufsize) {
		req->buf = kmalloc(bufsize, GFP_ATOMIC);
		if (!req->buf)
			goto fail2;
	}

	return req;
fail2:
	kfree(req);
fail1:
	return 0;
}

void bt_free_req(struct bt_request *req)
{
	if (req->buf)
		kfree(req->buf);

	kfree(req);
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

void bt_put_req(struct bt_diag_context *ctxt, struct list_head *head, struct bt_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
struct bt_request *bt_get_req(struct bt_diag_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct bt_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
/*
		if (head == &ctxt->tx_idle)
			DBG("bt_get_req: tx_idle list_empty\n");
		else if (head == &ctxt->rx_idle)
			DBG("bt_get_req: rx_idle list_empty\n");
*/
		req = 0;
	} else {
		req = list_first_entry(head, struct bt_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}



static void btlinux_stop_rx(struct uart_port *port)
{
	dbg("unimplemented");
}

static void btlinux_stop_tx(struct uart_port *port/*, unsigned int tty_stop*/)
{
	dbg("unimplemented");
}

static unsigned int btlinux_tx_empty(struct uart_port *port)
{
	struct        circ_buf *xmit = &port->state->xmit;

	if (uart_circ_empty(xmit)) {
		info("tx buffer empty");
		return TIOCSER_TEMT;
	}
	return 0;
}

static void btlinux_start_tx(struct uart_port *port /*, unsigned int tty_start*/)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct uart_btlinux_port *up = (struct uart_btlinux_port *)port;
	int count;
	count = uart_circ_chars_pending(xmit);

	/*    dbg("line %d - buf size = %d", port->line, uart_circ_chars_pending(xmit));*/
	/*    printk("btlinux_start_tx: line %d port %x\n", port->line, up);*/

	if (!up->port_opened) {
		err("start_tx - port is not opened");
		return;
	}

	if (!up->is_open) {
		err("start_tx - bt port is not opened");
		btlinux_stop_tx(port/*, 0*/);
		return;
	}
	if (port->x_char) {
		info("x_char is transmitted");
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))  {
		info("tx buffer empty or tx stopped");
		btlinux_stop_tx(port/*, 0*/);
		return;
	}

	send_tx_char(up);

	/* wake up application waiting in a poll */
	wake_up_interruptible(&up->rx_wait_q);

}

static void send_tx_char (struct uart_btlinux_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	unsigned char *buf;
	int c, d, count;
	count = uart_circ_chars_pending(xmit);

	buf = &xmit->buf[xmit->tail];
	/* write to tx buffer */
	while (1) {
		c = CIRC_SPACE_TO_END(up->tx_data_head, up->tx_data_tail, TX_BUFFERS_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;

		if (xmit->head > xmit->tail) {
			memcpy(up->tx_data + up->tx_data_head, buf, c);
			buf += c;
			xmit->tail = (xmit->tail + c) & (UART_XMIT_SIZE - 1);
		} else {
			d = (UART_XMIT_SIZE - xmit->tail);
			if (c < d)
				d = c;
			memcpy(up->tx_data + up->tx_data_head, buf, d);
			xmit->tail = (xmit->tail + d) & (UART_XMIT_SIZE - 1);
			buf = &xmit->buf[xmit->tail];
			if (c > d) {
				memcpy(up->tx_data + up->tx_data_head + d, buf, (c-d));
				xmit->tail = (xmit->tail + (c-d)) & (UART_XMIT_SIZE - 1);
				buf = &xmit->buf[xmit->tail];
			}
		}
		up->tx_data_head = (up->tx_data_head + c) & (TX_BUFFERS_SIZE - 1);
		count -= c;
	}

	if (uart_circ_chars_pending(xmit) < 1024)
		uart_write_wakeup(&up->port);
	/*    dbg("tx head %d - tx tail = %d", up->tx_data_head, up->tx_data_tail);*/
}

/****************************************************************************
 *
 * set_mctrl: sets a new value for the MCR UART register
 *
 ****************************************************************************/

static void btlinux_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_btlinux_port *up = (struct uart_btlinux_port *)port;

	dbg("[USERSPACE --> KERNEL] mctrl %x, up %x, up->flag %x", mctrl, (unsigned int)up, up->flag);

	if (!up) {
		err("set_mctrl - null uart");
		return;
	}

	if (!up->port_opened) {
		err("set_mctrl - port is not opened");
		return;
	}

	if (!up->is_open) {
		err("set_mctrl - bt port is not opened");
		return;
	}
	up->saved_event.eventCode = BTLINUX_PORT_MODEM_CONTROL;
	up->saved_event.u.modemControlReg = 0;

	if (mctrl & TIOCM_DTR) {
		info(" DTR SET");
		up->saved_event.u.modemControlReg |= MODEM_CNTRL_DTRDSR_MASK;
	} else
		info(" DTR CLR");

	if (mctrl & TIOCM_RTS) {
		info(" RTS SET");
		up->saved_event.u.modemControlReg |= MODEM_CNTRL_CTSRTS_MASK;
	} else {
		info(" RTS CLR");
		up->saved_event.u.modemControlReg |= MODEM_CNTRL_CTSRTS_MASK;
	}

	if (mctrl & TIOCM_CAR) {
		info(" CAR SET");
		up->saved_event.u.modemControlReg |= MODEM_CNTRL_CAR_MASK;
	} else
		info(" CAR CLR");

	if (mctrl & TIOCM_RNG) {
		info(" RNG SET");
		up->saved_event.u.modemControlReg |= MODEM_CNTRL_RNG_MASK;
	} else
		info(" RNG CLR");

	if (mctrl & TIOCM_OUT1) {
		info(" OUT1 SET");
		up->saved_event.u.modemControlReg |= MODEM_CNTRL_CAR_MASK;
	} else
		info(" OUT1 CLR");

	if (up->fasync) {
		dbg("notify application using fasync (fd %d)", up->fasync->fa_fd);

		/* notify application */
		kill_fasync(&up->fasync, SIGUSR1, POLL_MSG);

		/* sync wait */
		wait_event(up->port_wait_q, up->flag != 0);
		dbg("wait done");
	} else
		err("fasync NULL !");
}

/****************************************************************************
 *
 * get_mctrl: gets the current MCR UART register value.
 *
 ****************************************************************************/

static unsigned int btlinux_get_mctrl(struct uart_port *port)
{
	/*UINT8   control;*/
	unsigned int ret = 0;

	struct uart_btlinux_port *up = (struct uart_btlinux_port *)port;

	if (!up) {
		dbg("%s: uart_btlinux_port is null\n", __func__);
		return -ENODEV;
	}

	if (!up->fasync) {
		dbg("%s: up->fasync is null\n", __func__);
		return -ENODEV;
	}

	dbg("#########  %s -- ######## fasync fd %d, line %d", __func__, up->fasync->fa_fd, up->line);

	if (!up->port_opened) {
		info("set_mctrl - port is not opened");
		return -EINVAL;
	}
#if 0
	if (!BTKRNL_PORT_GetModemStatus(up->bt_port, &control)) {
		err("Bad return from PORT_GetModemStatus");
		return -EINVAL;
	}

	if (control & PORT_DTRDSR_ON)
		ret |= TIOCM_DSR;
	if (control & PORT_CTSRTS_ON)
		ret |= TIOCM_CTS;
	if (control & PORT_RING_ON)
		ret |= TIOCM_RNG;
	if (control & PORT_DCD_ON)
		ret |= TIOCM_CAR;
#endif
	up->saved_event.eventCode = BTLINUX_PORT_MODEM_STATUS;

	/* notify application */
	/*dbg("btport_startup - fasync fd %d", up->fasync->fa_fd);*/
	kill_fasync(&up->fasync, SIGUSR1, POLL_MSG);

	/* sync wait */
	wait_event(up->port_wait_q, up->flag != 0);
	up->flag = 0;
	if (up->event_result == 0) {
		err("Bad return from PORT_GetModemStatus");
		return -EINVAL;
	}

	if (up->event_data & PORT_DTRDSR_ON) {
		ret |= TIOCM_DSR;
		dbg("%s: TIOCM_DSR 1\n", __func__);
	} else
		dbg("%s: TIOCM_DSR 0\n", __func__);

	if (up->event_data & PORT_CTSRTS_ON) {
		ret |= TIOCM_CTS;
		dbg("%s: TIOCM_CTS 1\n", __func__);
	} else
		dbg("%s: TIOCM_CTS 0\n", __func__);

	if (up->event_data & PORT_RING_ON) {
		ret |= TIOCM_RNG;
		dbg("%s: TIOCM_RNG 1\n", __func__);
	} else
		dbg("%s: TIOCM_RNG 0\n", __func__);

	if (up->event_data & PORT_DCD_ON) {
		ret |= TIOCM_CAR;
		dbg("%s: TIOCM_CAR 1\n", __func__);
	} else
		dbg("%s: TIOCM_CAR 0\n", __func__);

	return ret;
}

/****************************************************************************
 *
 *
 *
 ****************************************************************************/

static int btlinux_ioctl(struct uart_port *port, unsigned int cmd,
					unsigned long arg)
{
	struct tty_struct *tty;
	void __user *argp = (void __user *)arg;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	tty = port->state->port.tty;
#else
	tty = port->state->tty;
#endif

	dbg("cmd %d arg %x", cmd, (unsigned int)arg);

	switch (cmd) {
	case TCGETS:
		if (copy_to_user(argp, tty->termios, sizeof(struct termios)))
			return -EFAULT;
	break;

	case TCSETS:
		if (copy_from_user(tty->termios, argp, sizeof(struct termios)))
			return -EFAULT;
	break;

	case TIOCSBRK:
		err(" IOCTL_SERIAL_SET_BREAK_ON -- unhandled");
#if 0
		if (!BTKRNL_PORT_Control(up->bt_port, PORT_SET_BREAK)) {
		    dbg("Bad return from PORT_Control");
		    return -EFAULT;
		}
#endif
	/*            up->saved_event.eventCode = BTLINUX_PORT_SET_BREAK_ON;*/
	/* notify application */
	/*            kill_fasync(&up->fasync, SIGIO, POLL_IN);*/
	break;

	case TIOCCBRK:
		err(" IOCTL_SERIAL_SET_BREAK_OFF -- unhandled");
#if 0
		if (!BTKRNL_PORT_Control(up->bt_port, PORT_CLR_BREAK)) {
			dbg("Bad return from PORT_Control");
			return -EFAULT;
		}
#endif
	/*            up->saved_event.eventCode = BTLINUX_PORT_SET_BREAK_OFF;*/
	/* notify application */
	/*            kill_fasync(&up->fasync, SIGIO, POLL_IN);*/
	break;

	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

#if 0
static struct uart_btlinux_port *btlinux_port_from_line(UINT16 line)
{
	int i;
	for (i = 0; i < UART_NR; i++) {
		if (btlinux_ports[i].bt_port == line)
			return &btlinux_ports[i];
	}
	return NULL;
}
#endif


/****************************************************************************
 *
 *
 *
 ****************************************************************************/

void btlinux_handle_mctrl(struct uart_btlinux_port *linux_port, UINT8 signals, UINT8 values)
{
	UINT16 status = 0;

	dbg("[USERSPACE <-- KERNEL] -- signals %x, values %x", signals, values);

	if (values & PORT_DTRDSR_ON) {
		info("BTPORT_DTRDSR_ON");
		status |= TIOCM_DSR;
	};
	if (values & PORT_CTSRTS_ON) {
		info("BTPORT_CTSRTS_ON");
		status |= TIOCM_CTS;
	};
	if (values & PORT_RING_ON) {
		info("BTPORT_RING_ON");
		status |= TIOCM_RNG;
	};
	if (values & PORT_DCD_ON) {
		info("BTPORT_DCD_ON");
		status |= TIOCM_CAR;
	};

	if (signals & MODEM_CNTRL_DTRDSR_MASK) {
		dbg("MODEM_CNTRL_DTRDSR_MASK, SET DTR %d", status & TIOCM_DSR);
		linux_port->port.icount.dsr++;
	}

	if (signals & MODEM_CNTRL_CTSRTS_MASK) {
		dbg("MODEM_CNTRL_CTSRTS_MASK, SET CTS %d", status & TIOCM_CTS);
		uart_handle_cts_change(&linux_port->port, status & TIOCM_CTS);
	}

	if (signals & MODEM_CNTRL_RNG_MASK) {
		dbg("MODEM_CNTRL_RNG_MASK, SET DTR %d", status & TIOCM_RNG);
		linux_port->port.icount.rng++;
	}
	if (signals & MODEM_CNTRL_CAR_MASK) {
		dbg("MODEM_CNTRL_CAR_MASK, SET CAR %d", status & TIOCM_CAR);
		uart_handle_dcd_change(&linux_port->port, status & TIOCM_CAR);
	}
}


/****************************************************************************
 *
 *
 *
 ****************************************************************************/
static void smd_diag_notify(void *priv, unsigned event)
{
	int bytesInSmdBuf;
	struct uart_btlinux_port *up = &btlinux_ports[1];
	struct bt_diag_context *ctxt = &_context;

	bytesInSmdBuf = smd_read_avail(ctxt->ch);

	if (up && bytesInSmdBuf) {
		/*info("smd_diag_notify++\n");*/
		wake_up_interruptible(&up->rx_wait_q);
	}
}

static ssize_t diag_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct bt_diag_context *ctxt = &_context;
	int r = 0;
	int ret;
	struct bt_request *req;
	UINT8 *tmp;
	int i;

	if (_lock(&ctxt->read_excl))
		return 0;

	ret = wait_event_interruptible(ctxt->read_wq, (ctxt->read_avail || !ctxt->online));
	dbg("read_wq wake up\n");

	if (!ctxt->online || ret < 0) {
		dbg("%s: offline\n", __func__);

		while (1) {
			req = bt_get_req(ctxt, &ctxt->rx_done);
			if (req)
				bt_put_req(ctxt, &ctxt->rx_idle, req);
			else
				break;
		}

		ret = -EIO;
		_unlock(&ctxt->read_excl);
		return ret;
	}
	while (1) {
		req = bt_get_req(ctxt, &ctxt->rx_done);
		if (req) {
			tmp = (UINT8 *) req->buf;
			for (i = 0; i < req->length; i++, tmp++)
				printk(KERN_INFO "%x", *tmp);

			printk(KERN_INFO "\n");
			if (copy_to_user(buf, req->buf, req->length))
				dbg("diag:diag_read: copy_to_user fail\n");

			buf += req->length;
			r += req->length;
			bt_put_req(ctxt, &ctxt->rx_idle, req);
		} else
			break;
	}

	ctxt->read_avail = 0;

	_unlock(&ctxt->read_excl);

	return r;
}

static ssize_t diag_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct bt_diag_context *ctxt = &_context;
	struct bt_request *req = 0;
	int xfer;
	int access = 0;
	struct uart_btlinux_port *up = &btlinux_ports[1];
	char *tmp;
	int i;

	if (count == 0) {
		dbg("diag_write: count = 0\n");
		return 0;
	}

	if (_lock(&ctxt->write_excl))
		return 0;

	if (count > 8192)
		xfer = 8192;
	else
		xfer = count;

	dbg("xfer: %d\n", xfer);
	/* check to verify that the incomign data is good */
	access = !access_ok(VERIFY_READ, (void *)buf, count);
	if (access) {
		err("BTLINUXPORT %s: buffer access verification failed", __func__);
		_unlock(&ctxt->write_excl);
		return 0;
	}

	req = bt_get_req(ctxt, &ctxt->tx_idle);

	if (!req) {
		dbg("no request in tx_idle\n");
		_unlock(&ctxt->write_excl);
		return 0;
	} else {
		if (copy_from_user(req->buf, buf, xfer)) {
			err("BTLINUXPORT %s: copy from user error", __func__);
			bt_put_req(ctxt, &ctxt->tx_idle, req);
			_unlock(&ctxt->write_excl);
			return -EINVAL;
		} else {
			req->length = xfer;

			tmp = (UINT8 *) req->buf;

			dbg("[DM router -> btport driver]\n");
			for (i = 0; i < req->length; i++, tmp++)
				printk(KERN_INFO "%2x", *tmp);

			printk(KERN_INFO "\n");
			ctxt->write_avail = 1;
			bt_put_req(ctxt, &ctxt->tx_done, req);
			wake_up_interruptible(&up->rx_wait_q);

		}
	}

	_unlock(&ctxt->write_excl);

	return 0;
}

static int diag_open(struct inode *ip, struct file *fp)
{
	struct bt_diag_context *ctxt = &_context;

	dbg("diag_open\n");

	if (_lock(&ctxt->open_excl))
		return -EBUSY;

	return 0;
}

static int diag_release(struct inode *ip, struct file *fp)
{
	struct bt_diag_context *ctxt = &_context;

	dbg("diag_release\n");

	ctxt->read_avail = 0;
	ctxt->write_avail = 0;

	_unlock(&ctxt->open_excl);
	return 0;
}

static int
diag_ioctl(struct inode *inode, struct file *file,
	  unsigned int cmd, unsigned long arg)
{
	struct bt_diag_context *ctxt = &_context;
	void __user *argp = (void __user *)arg;
	int tmp_value;
	unsigned long flags;
	dbg("diag_ioctl\n");

	if (_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC) {
		dbg("IOC_TYPE not USB_DIAG_IOC_MAGIC\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case USB_DIAG_FUNC_IOC_ENABLE_SET:
		if (copy_from_user(&tmp_value, argp, sizeof(int))) {
			dbg("copy_from_user fail\n");
			return -EFAULT;
		}
		dbg("IOCTL USB_DIAG_FUNC_IOC_ENABLE_SET %d\n", tmp_value);
		if (tmp_value)	{
			/*TODO: enable BT interface*/
			dbg("smd_open: SMD_DIAG\n");
			smd_open("SMD_DIAG", &ctxt->ch, ctxt, smd_diag_notify);
			wake_lock(&btport_wake_lock);
		} else	{
			/*TODO: disable BT interface*/
			dbg("smd_close: SMD_DIAG\n");
			smd_close(ctxt->ch);
			wake_unlock(&btport_wake_lock);
		}
		break;
	case USB_DIAG_FUNC_IOC_ENABLE_GET:
		/*TODO: assign BT enable to tmp_value*/
		if (copy_to_user(argp, &tmp_value, sizeof(tmp_value)))
			return -EFAULT;
		break;
	case USB_DIAG_FUNC_IOC_REGISTER_SET:
		spin_lock_irqsave(&ctxt->lock_reg_num, flags);
		if (copy_from_user(ctxt->id_table, (unsigned char *)argp, sizeof(unsigned char)*TABLE_SIZE)) {
			spin_unlock_irqrestore(&ctxt->lock_reg_num, flags);
			return -EFAULT;
		}
		spin_unlock_irqrestore(&ctxt->lock_reg_num, flags);
		break;
	case USB_DIAG_FUNC_IOC_AMR_SET:
		if (copy_from_user(&ctxt->is2ARM11, argp, sizeof(int)))
			return -EFAULT;
		dbg("IOCTL USB_DIAG_FUNC_IOC_AMR_SET %d\n", ctxt->is2ARM11);
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static struct file_operations diag_fops = {
	.owner =   THIS_MODULE,
	.read =    diag_read,
	.write =   diag_write,
	.open =    diag_open,
	.ioctl =   diag_ioctl,
	.release = diag_release,
};

static struct miscdevice diag_device_fops = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "btdiag",
	.fops = &diag_fops,
};


static int btport_open(struct inode *inode, struct file *filp)
{
	struct uart_btlinux_port *btlinux_port;
	int minor;

	minor = iminor(inode);

	info("open subport %d", minor);

	if (minor >= UART_NR)
	    return -ENODEV;

	btlinux_port = &btlinux_ports[minor];

	if (btlinux_port->open_count)
		return -EBUSY;

	/* init the wait q  */
	init_waitqueue_head(&btlinux_port->rx_wait_q);

	dbg("wait queue initialized");

	btlinux_port->open_count = 1;
	btlinux_port->is_open = TRUE;

	btlinux_port->rcv_data_head = 0;
	btlinux_port->rcv_data_tail = 0;
	btlinux_port->rcv_full = 0;
	btlinux_port->minor = minor;

	filp->private_data = btlinux_port;

	return 0;
}

/****************************************************************************
 *
 *
 *
 ****************************************************************************/
static int btport_fasync(int fd, struct file *file, int on)
{
	int retval;
	struct uart_btlinux_port *btlinux_port = file->private_data;

	dbg("fd %d, on %d\n", fd, on);

	if (btlinux_port == NULL) {
	    printk(KERN_ERR "BTLINUXPORT %s - error, can't find device\n", __func__);
	    return -ENODEV;
	}
	retval = fasync_helper(fd, file, on, &btlinux_port->fasync);
	return retval < 0 ? retval : 0;
}

/****************************************************************************
 *
 *
 *
 ****************************************************************************/

static int btport_release(struct inode *inode, struct file *filp)
{
	struct uart_btlinux_port *btlinux_port;
	btlinux_port = (struct uart_btlinux_port *)filp->private_data;

	info("close subport %d", btlinux_port->line);
	if (btlinux_port == NULL) {
		err("BTPORT %s - error, can't find device", __func__);
		return -ENODEV;
	}

	if (btlinux_port->port_opened) {
		dbg("BTPORT %s - user port is still opened", __func__);
		uart_handle_cts_change(&btlinux_port->port, 0);
		uart_handle_dcd_change(&btlinux_port->port, 0);
	}

	btport_fasync(-1, filp, 0);

	btlinux_port->open_count--;
	btlinux_port->is_open = FALSE;


	/* send signal to read so that it can be released when driver
	* is closed.
	*/

	btlinux_port->wake_timeout = TRUE;

	/* ensure there is no process hanging on poll / select  */
	wake_up_interruptible(&btlinux_port->rx_wait_q);

	return 0;
}


/****************************************************************************
 *
 *
 *
 ****************************************************************************/

static int btport_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
      unsigned long arg)
{
	struct uart_btlinux_port *btlinux_port;
	int retval = 0;
	UINT8 signals;
	UINT8 values;


	btlinux_port = (struct uart_btlinux_port *)file->private_data;
	if (btlinux_port == NULL) {
		err("error, can't find device");
		return -ENODEV;
	}

	if (!btlinux_port->port_opened) {
		err("BTLINUXPORT %s - port not opened", __func__);
		return -ENODEV;
	}

	if (!btlinux_port->is_open)
		return -EINVAL;

	switch (cmd) {
	case IOCTL_BTPORT_GET_EVENT:
		info("IOCTL_BTPORT_GET_EVENT");
		if (copy_to_user((void *) arg, &btlinux_port->saved_event, sizeof(BTLINUXPORTEvent))) {
			err("BTLINUXPORT %s - copy_to_user failed", __func__);
			retval = -EFAULT;
			goto err_out;
		}
	break;
	case IOCTL_BTPORT_SET_EVENT_RESULT:
		dbg("IOCTL_BTPORT_SET_EVENT_RESULT");
		btlinux_port->event_result = 0;
		btlinux_port->flag = 1;
		if (copy_from_user(&btlinux_port->event_result, (void *) arg, sizeof(int))) {
		retval = -EFAULT;
		err("BTLINUXPORT IOCTL_BTPORT_SET_EVENT_RESULT failed getting 1st par");
		wake_up(&btlinux_port->port_wait_q);
		goto err_out;
		}

		if (copy_from_user(&btlinux_port->event_data, ((char *) arg) + sizeof(int), sizeof(int))) {
			retval = -EFAULT;
			err("BTLINUXPORT IOCTL_BTPORT_SET_EVENT_RESULT failed getting 2nd par");
			wake_up(&btlinux_port->port_wait_q);
			goto err_out;
		}
		wake_up(&btlinux_port->port_wait_q);
		break;

	case IOCTL_BTPORT_HANDLE_MCTRL:
		dbg("IOCTL_BTPORT_HANDLE_MCTRL");
		if (copy_from_user(&signals, (void *) arg, sizeof(UINT8))) {
			retval = -EFAULT;
			err("BTLINUXPORT IOCTL_BTPORT_HANDLE_MCTRL failed getting 1st par");
			goto err_out;
		}
		if (copy_from_user(&values, ((char *) arg) + sizeof(UINT8), sizeof(UINT8))) {
			retval = -EFAULT;
			err("BTLINUXPORT IOCTL_BTPORT_HANDLE_MCTRL failed getting 2nd par");
			goto err_out;
		}

		btlinux_handle_mctrl(btlinux_port, signals, values);

		break;
	}

	err_out:
	dbg("BTLINUXPORT btport_ioctl returning %d", retval);
	return retval;
}

/****************************************************************************
 *
 *
 *
 ****************************************************************************/
static ssize_t btport_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int           bytesRemain, bytesInCircBuf, nRead, nExtraRead;
	char          *pBuf;
	struct uart_btlinux_port *btlinux_port;
	unsigned long flags;
	int           bytesInSmdBuf;
	ssize_t ret = 0;
	struct bt_diag_context *ctxt = &_context;
	struct bt_request *req;
	char *tmp;
	int i;

	btlinux_port = (struct uart_btlinux_port *)file->private_data;

	/*dbg ("%s", __func__);*/

	if (btlinux_port == NULL) {
		err("BTLINUXPORT %s - error, can't find device", __func__);
		return -ENODEV;
	}

	/*BT SPP*/
	if (btlinux_port->minor == 1) {
		if (!ctxt->ch) {
			err("btport_write %s: SMD channel is null\n", __func__);
			return 0;
		} else {
			/*Data from DM router*/
			req = bt_get_req(ctxt, &ctxt->tx_done);
			if (req) {
				tmp = (UINT8 *) req->buf;

				dbg("[btport driver -> BTLD]\n");
				for (i = 0; i < req->length; i++, tmp++)
					printk(KERN_INFO "%2x", *tmp);

				printk(KERN_INFO "\n");
				if (copy_to_user(buffer, req->buf, req->length))
					dbg("copy_to_user error\n");

				bt_put_req(ctxt, &ctxt->tx_idle, req);
				ctxt->write_avail = 0;
				return req->length;
			}

			/*DATA from SMD*/
			bytesInSmdBuf = smd_read_avail(ctxt->ch);
			dbg("btport_read: avail %d", bytesInSmdBuf);

			if (bytesInSmdBuf >= count)
			    nRead = count;
			else
				nRead = bytesInSmdBuf;

			if (nRead > 0) {
				pBuf = &btlinux_port->tx_data[0];

				smd_read(ctxt->ch, pBuf, nRead);
				if (copy_to_user(buffer, pBuf, nRead))
					dbg("copy_to_user error\n");

				/*dbg ("btport_read buffer: %s\n", buffer);*/
			}
			return nRead;
		}
	}

	if (!btlinux_port->port_opened) {
		err("BTLINUXPORT %s - port not opened", __func__);
		return -ENODEV;
	}

	if (!btlinux_port->is_open)
		return -EINVAL;

	spin_lock_irqsave(&btlinux_port->port.lock, flags);

	if (btlinux_port->tx_data_head > btlinux_port->tx_data_tail) {
		bytesInCircBuf = btlinux_port->tx_data_head - btlinux_port->tx_data_tail;
		bytesRemain = 0;
	} else {
		bytesInCircBuf = TX_BUFFERS_SIZE - btlinux_port->tx_data_tail;
		bytesRemain = btlinux_port->tx_data_head;
	}
	/*    dbg ("request tx head %d tx tail %d port = %x", btlinux_port->tx_data_head, btlinux_port->tx_data_tail, btlinux_port);*/

	nExtraRead = 0;
	nRead = 0;
	pBuf = &btlinux_port->tx_data[btlinux_port->tx_data_tail];


	if (bytesInCircBuf >= count)
	    nRead = count;
	else {
		nRead = bytesInCircBuf;
		if (bytesRemain) {
			if (bytesRemain >= (count-nRead))
				nExtraRead = count-nRead;
			else
				nExtraRead = bytesRemain;
		}
	}
	if (!tx_circ_empty(btlinux_port)) {
		dbg("request read %d bytes from circ buf (%d) MAX = %lu", (int)count, bytesInCircBuf, UART_XMIT_SIZE);
		btlinux_port->tx_data_tail += nRead;
		btlinux_port->tx_data_tail = (btlinux_port->tx_data_tail) & (TX_BUFFERS_SIZE - 1);
		if (copy_to_user(buffer, pBuf, nRead)) {
			spin_unlock_irqrestore(&btlinux_port->port.lock, flags);
			err("BTLINUXPORT %s: copy to user error", __func__);
			ret = -EINVAL;
		}
		if (nExtraRead) {
			pBuf = &btlinux_port->tx_data[0];
			btlinux_port->tx_data_tail = nExtraRead;
			if (copy_to_user(buffer+nRead, pBuf, nExtraRead)) {
				spin_unlock_irqrestore(&btlinux_port->port.lock, flags);
				err("BTLINUXPORT %s: copy to user error", __func__);
				ret = -EINVAL;
			}
		}
		dbg("btport_read buffer: %s\n", buffer);
		ret = (nRead + nExtraRead);
	} else if (btlinux_port->wake_timeout) {
		spin_unlock_irqrestore(&btlinux_port->port.lock, flags);
		btlinux_port->wake_timeout = FALSE;
		ret = 0;
	} else {
		spin_unlock_irqrestore(&btlinux_port->port.lock, flags);
		ret = 0;
	}

	if (tx_circ_chars_pending(btlinux_port) < 1024)
		send_tx_char(btlinux_port);
	/*    info("%s%d <-- ttySA%d    wanted (%d bytes) got  (%d bytes) in uart buffer (%d bytes)", BTPORT_DEV_NAME, btlinux_port->line, btlinux_port->line, count, nRead+nExtraRead, uart_circ_chars_pending(xmit));*/
	spin_unlock_irqrestore(&btlinux_port->port.lock, flags);
	return ret;
}

/****************************************************************************
 *
 *
 *
 ****************************************************************************/
static void send_to_dmrouter(char *user_buffer,  int xfer)
{
	struct bt_request *req;
	struct bt_diag_context *ctxt = &_context;
	UINT8 *cmd;

	req = bt_get_req(ctxt, &ctxt->rx_idle);

	if (!req) {
		dbg("no request in rx_idle\n");
		return;
	} else {
		memcpy(req->buf, user_buffer, xfer);

		req->length = xfer;
		cmd = (UINT8 *)req->buf;
		dbg("cmd, buffer: %x\n", *cmd);

		ctxt->read_avail = 1;
		bt_put_req(ctxt, &ctxt->rx_done, req);
		wake_up(&ctxt->read_wq);
	}
}

static ssize_t btport_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *ppos)
{
	struct uart_btlinux_port *btlinux_port;
	struct tty_struct *tty;
	int nCopy, nRemaining, dataOut;
	int access = 0;
	struct bt_diag_context *ctxt = &_context;
	unsigned int cmd_id;
	int xfer;
	unsigned char *table = NULL;
	unsigned char tmp;

	btlinux_port = (struct uart_btlinux_port *)file->private_data;

	if (btlinux_port == NULL) {
		err("BTLINUXPORT %s - error, can't find device", __func__);
		return -ENODEV;
	}

	/*BT SPP*/
	if (btlinux_port->minor == 1) {
		if (!ctxt->ch) {
			err("btport_write %s: SMD channel is null\n", __func__);
			return 0;
		} else {

			/* check to verify that the incomign data is good */
			access = !access_ok(VERIFY_READ, (void *)user_buffer, count);
			if (access) {
				err("BTLINUXPORT %s: buffer access verification failed", __func__);
				return 0;
			}
			if (count == 0)
				return 0;

			if (count > 8192)
				xfer = 8192;
			else
				xfer = count;

			if (copy_from_user(btspp_buf, (void *)user_buffer, xfer)) {
				err("BTLINUXPORT %s: copy from user error", __func__);
				return -EINVAL;
			}

			cmd_id = btspp_buf[0];

			dbg("cmd_id, xfer: %x, %x\n", cmd_id, xfer);

			/*Send data to DM router*/
			if (cmd_id >= 0xfb && cmd_id <= 0xff) {
				send_to_dmrouter(btspp_buf, xfer);
				return 0;
			}

			table = ctxt->id_table;
			while ((tmp = *table++)) {
				if (tmp == cmd_id) {
					dbg("cmd_id in table: %x\n", cmd_id);
					send_to_dmrouter(btspp_buf, xfer);
					return 0;
				}
			}
			/*Sent data to ARM9*/
			dataOut = smd_write(ctxt->ch, btspp_buf, count);
			return dataOut;
		}
	}


	/*BT DUN*/
	if (!btlinux_port->port_opened) {
	/*       err("BTLINUXPORT %s - port not opened", __func__);*/
		return -ENODEV;
	}

	if (!btlinux_port->is_open)
		return -EINVAL;

	if (count == 0)
		return 0;

	/* check to verify that the incomign data is good */
	access = !access_ok(VERIFY_READ, (void *)user_buffer, count);
	if (access) {
		err("BTLINUXPORT %s: buffer access verification failed", __func__);
		return 0;
	}
	if (!btlinux_port->port.state) {
		err("btport_write - port info obtained");
		return -EINVAL;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	tty = btlinux_port->port.state->port.tty;
#else
	tty = btlinux_port->port.state->tty;
#endif

	if (!tty) {
		err("btport_write - tty not obtained");
		return -EINVAL;
	}

	dataOut = 0;
	if (!btlinux_port->rcv_full) {
		if (btlinux_port->rcv_data_tail >= btlinux_port->rcv_data_head) {
			nCopy = RX_BUFFERS_SIZE - btlinux_port->rcv_data_tail;
			nRemaining = btlinux_port->rcv_data_head;
		} else {
			nCopy = btlinux_port->rcv_data_head - btlinux_port->rcv_data_tail;
			nRemaining = 0;
		}

		if (nCopy >= count)
			nCopy = count;

		if (copy_from_user(&btlinux_port->rcv_data[btlinux_port->rcv_data_tail],
									(void *)user_buffer, nCopy)) {
			err("BTLINUXPORT %s: copy from user error", __func__);
			return -EINVAL;
		}

	    dataOut = nCopy;

	    btlinux_port->rcv_data_tail += nCopy;
	    btlinux_port->rcv_data_tail &= (RX_BUFFERS_SIZE - 1);

		if (btlinux_port->rcv_data_tail == btlinux_port->rcv_data_head) {
		    btlinux_port->rcv_full = 1;
		} else {
			if (nRemaining && nCopy < count) {
				int remData = count - nCopy;
				nCopy = nRemaining;

				if (nCopy >= remData)
					nCopy = remData;

				if (copy_from_user(&btlinux_port->rcv_data[btlinux_port->rcv_data_tail],
												(void *)(user_buffer+dataOut), nCopy)) {
					err("BTLINUXPORT %s: copy from user error", __func__);
					return -EINVAL;
				}
				dataOut += nCopy;
				btlinux_port->rcv_data_tail += nCopy;

				if (btlinux_port->rcv_data_tail == btlinux_port->rcv_data_head)
					btlinux_port->rcv_full = 1;
			}
		}
	}

	do {
		if (tty_buffer_request_room(tty, 1) == 0) {
			err("Flip buffer overflows");
			break;
		}
		if (tty_insert_flip_char(tty,
								btlinux_port->rcv_data[btlinux_port->rcv_data_head],
								TTY_NORMAL) == 0)
		    err("btlinux_write, flip buffer insert error");
		btlinux_port->rcv_data_head++;
		btlinux_port->rcv_data_head &= (RX_BUFFERS_SIZE - 1);
		btlinux_port->port.icount.rx++;
	} while (btlinux_port->rcv_data_head != btlinux_port->rcv_data_tail);

	if (btlinux_port->rcv_data_tail != btlinux_port->rcv_data_head)
	    btlinux_port->rcv_full = 0;

	tty->low_latency = 1;
	tty->icanon = 0;

	 /*dbg("<<< push data to uart port>>>");*/
	 tty_flip_buffer_push(tty);

	/* let user know how much data we sent to transport  */
	return dataOut;
}

/****************************************************************************
 *
 *
 *
 ****************************************************************************/

unsigned int btport_poll(struct file *file, struct poll_table_struct *p_pt)
{
	struct uart_btlinux_port *btlinux_port;
	unsigned mask = 0;  /* always make this writable  */
	struct        circ_buf *xmit;
	struct bt_diag_context *ctxt = &_context;
	int bytesInSmdBuf = 0;

	btlinux_port = (struct uart_btlinux_port *)file->private_data;

	if (btlinux_port == NULL) {
		err("BTLINUXPORT %s - error, can't find device", __func__);
		return -ENODEV;
	}

	if (!btlinux_port->is_open)
		return -EINVAL;

	if (!btlinux_port->port.state)
	    return mask;
	xmit = &btlinux_port->port.state->xmit;

	if (btlinux_port->minor == 0) {
		if (btlinux_port->tx_data_head == btlinux_port->tx_data_tail)
			poll_wait(file, &btlinux_port->rx_wait_q, p_pt);
		/* only set READ mask if data is queued from HCI */
		if (btlinux_port->tx_data_head != btlinux_port->tx_data_tail)
			mask |= POLLIN | POLLRDNORM;
	} else if (btlinux_port->minor == 1) {
		if (ctxt->ch)
			bytesInSmdBuf = smd_read_avail(ctxt->ch);

		if (!bytesInSmdBuf || !ctxt->ch)
			poll_wait(file, &btlinux_port->rx_wait_q, p_pt);

		if (ctxt->ch)
			bytesInSmdBuf = smd_read_avail(ctxt->ch);
		else
			return mask;

		/* only set READ mask if data is queued from HCI */
		if (bytesInSmdBuf || ctxt->write_avail)
			mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

static struct file_operations port_fops = {
	.owner =        THIS_MODULE,
	.read =         btport_read,
	.write =        btport_write,
	.poll =         btport_poll,
	.open =         btport_open,
	.release =      btport_release,
	.fasync =       btport_fasync,
	.ioctl =        btport_ioctl,
};


static void btlinux_set_termios(struct uart_port *port,
				struct ktermios *termios, struct ktermios *old)
{
	struct tty_struct *tty;
	/*struct uart_btlinux_port *up = (struct uart_btlinux_port *)port;*/

	dbg("%s", __func__);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	tty = port->state->port.tty;
#else
	tty = port->state->tty;
#endif
	memcpy(tty->termios, termios, sizeof(*termios));
}

static int btlinux_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	dbg("not implemented");
	return 0;
}

static void btlinux_release_port(struct uart_port *port)
{
	dbg("not implemented");
}

/* These do not need to do anything interesting either.  */
static void btlinux_config_port(struct uart_port *port, int flags)
{
	struct uart_btlinux_port *up = (struct uart_btlinux_port *)port;

	dbg(" btlinux_config_port");
	spin_lock_irqsave(&up->port.lock, (unsigned long)flags);

	up->port.type = PORT_BROADCOM;

	spin_unlock_irqrestore(&up->port.lock, (unsigned long)flags);
}

static void btlinux_pm(struct uart_port *port, unsigned int state,
	unsigned int oldstate)
{
	dbg("not implemented");
}

static const char *
btlinux_type(struct uart_port *port)
{
	dbg("not implemented");
	return "not impl";
}

static void btlinux_enable_ms(struct uart_port *port)
{
	dbg("not implemented");
}

static void btlinux_break_ctl(struct uart_port *port, int break_state)
{
	dbg("not implemented");
}


static int btlinux_startup(struct uart_port *port)
{
	struct uart_btlinux_port *up = (struct uart_btlinux_port *)port;
	struct bt_diag_context *ctxt = &_context;
	struct tty_struct *tty;

	dbg("btlinux_startup line %d", port->line);

	up->bt_port = port->line + 1;
	up->saved_event.eventCode = BTLINUX_PORT_OPENED;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	tty = port->state->port.tty;
#else
	tty = port->state->tty;
#endif

	if (!tty) {
	    err("btlinux_startup - tty not obtained");
	    return -EINVAL;
	}
	dbg("tty magic %d index %d name %s", tty->magic, tty->index, tty->name);

	info("%s is opened", tty->name);
	up->port_opened = 1;
	ctxt->online = 1;
	uart_write_wakeup(&up->port);

	return 0;
}

static void btlinux_shutdown(struct uart_port *port)
{
	struct uart_btlinux_port *up = (struct uart_btlinux_port *)port;
	struct bt_diag_context *ctxt = &_context;
	struct tty_struct *tty;

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 26)
	tty = port->state->port.tty;
#else
	tty = port->state->tty;
#endif

	info("%s is closed", tty->name);

	up->port_opened = 0;
	ctxt->online = 0;
	wake_up(&ctxt->read_wq);
}

static struct uart_ops btlinux_pops = {
	.tx_empty   = btlinux_tx_empty,
	.set_mctrl  = btlinux_set_mctrl,
	.get_mctrl  = btlinux_get_mctrl,
	.stop_tx    = btlinux_stop_tx,
	.start_tx   = btlinux_start_tx,
	.stop_rx    = btlinux_stop_rx,
	.enable_ms  = btlinux_enable_ms,
	.break_ctl  = btlinux_break_ctl,
	.startup    = btlinux_startup,
	.shutdown   = btlinux_shutdown,
	.ioctl      = btlinux_ioctl,
	.pm         = btlinux_pm,
	.type       = btlinux_type,
	.set_termios  = btlinux_set_termios,
	.release_port = btlinux_release_port,
	.config_port  = btlinux_config_port,
	.verify_port  = btlinux_verify_port,
};


static int __init btlinux_register_ports(struct uart_driver *drv)
{
	int i,  ret = 0;

	for (i = 0; i < (UART_NR); i++)  {
		struct uart_btlinux_port *up = &btlinux_ports[i];
		memset(up, 0, sizeof(*up));
		up->port.line = (unsigned int)i;
		up->port.ops = &btlinux_pops;
		up->port.iobase = 0x1;
		up->port.mapbase = 0x45;
		up->port.membase = (unsigned char __iomem *)0x55;
		up->port.flags |= UPF_BOOT_AUTOCONF;

		info("btlinux_register_ports -- add port: %d up %x up->port %x", up->port.line, (int)up, (int)&up->port);

		ret = uart_add_one_port(drv, &up->port);
		if (ret) {
			err("uart_add_one_port : error %d", ret);
			break;
		}
		init_waitqueue_head(&up->port_wait_q);
		up->port_added = 1;
	}

	if (ret) {
		for (i = 0; i < UART_NR; i++)  {
			struct uart_btlinux_port *up = &btlinux_ports[i];
			if (up->port_added) {
			    uart_remove_one_port(drv, &btlinux_ports[i].port);
			    up->port_added = 0;
			}
		}
	}
	return ret;
}

static struct uart_driver btlinux_port_reg = {
	.owner      = THIS_MODULE,
	.driver_name    = BTLINUX_SERIAL_NAME,
#ifdef CONFIG_DEVFS_FS
	    .dev_name     = "ttySA%d",
#else
	    .dev_name     = "ttySA",
#endif
	.major          = 200,/*TTY_MAJOR,*/
	.minor          = 104,
	.nr     = 4,
	.cons           = NULL,
};

static int msm_diag_probe(struct platform_device *pdev)
{
	struct bt_diag_context *ctxt = &_context;

	ctxt->pdev = pdev;
	info("btport: msm_diag_probe\n");

	return 0;
}

static struct platform_driver msm_smd_ch1_driver = {
	.probe = msm_diag_probe,
	.driver = {
		.name = "SMD_DIAG",
		.owner = THIS_MODULE,
	},
};

#if 0
static void diag_plat_release(struct device *dev) {}

static struct platform_device diag_plat_device = {
	.name		= "SMD_DIAG",
	.id		= -1,
	.dev		= {
		.release	= diag_plat_release,
	},
};
#endif

static int __init btlinux_port_init(void)
{
	int ret;
	struct bt_diag_context *ctxt = &_context;
	struct bt_request *req;
	int i;

	info("Loading BRCM rfcomm driver %s", DRIVER_VERSION);

	/* Register BtLinuxPort as char device */
	btport_major = register_chrdev(BTLINUXPORT_MAJOR, BTPORT_DEV_NAME,
									&port_fops);

	info("Registered btport chrdev, major number returned = %d", btport_major);

	if (btport_major < 0) {
		err("btlinux_port_init: unable to get major");
		return btport_major;
	}

	ret = uart_register_driver(&btlinux_port_reg);

	if (ret) {
		err("uart_register_driver returns %d", ret);
		unregister_chrdev(btport_major, BTPORT_DEV_NAME);
		return ret;
	}

	ret = btlinux_register_ports(&btlinux_port_reg);

	if (ret) {
		err("btlinux_register_ports returns %d", ret);
		unregister_chrdev(btport_major, BTPORT_DEV_NAME);
		uart_unregister_driver(&btlinux_port_reg);
	}
	ret = platform_driver_register(&msm_smd_ch1_driver);
	if (ret < 0)
		printk(KERN_ERR "%s: Register driver fail\n", __func__);

#if 0
	/*registed in usb diag*/
	ret = platform_device_register(&diag_plat_device);
	if (ret < 0) {
		printk(KERN_ERR "%s: Register device fail\n", __func__);
		platform_driver_unregister(&msm_smd_ch1_driver);
	}
#endif
	ctxt->is2ARM11 = 0;
	ctxt->is7E = 0x7E;

	init_waitqueue_head(&ctxt->read_wq);

	atomic_set(&ctxt->open_excl, 0);
	atomic_set(&ctxt->read_excl, 0);
	atomic_set(&ctxt->write_excl, 0);
	atomic_set(&ctxt->enable_excl, 0);

	spin_lock_init(&ctxt->lock);
	spin_lock_init(&ctxt->lock_reg_num);

	for (i = 0; i < TABLE_SIZE; i++)
		ctxt->id_table[i] = 0;
	INIT_LIST_HEAD(&ctxt->rx_idle);
	INIT_LIST_HEAD(&ctxt->tx_idle);
	INIT_LIST_HEAD(&ctxt->rx_done);
	INIT_LIST_HEAD(&ctxt->tx_done);
	misc_register(&diag_device_fops);
	info("Registered uart driver");

	for (i = 0; i < RX_REQ_MAX; i++) {
		req = bt_alloc_req(8192);
		if (req == 0)
			return -1;
		req->context = ctxt;
		bt_put_req(ctxt, &ctxt->rx_idle, req);
	}

	for (i = 0; i < TX_REQ_MAX; i++) {
		req = bt_alloc_req(8192);
		if (req == 0)
			return -1;
		req->context = ctxt;
		bt_put_req(ctxt, &ctxt->tx_idle, req);
	}

	wake_lock_init(&btport_wake_lock, WAKE_LOCK_SUSPEND, "BTPORT");

	return ret;
}

static void __exit btlinux_port_exit(void)
{
	int i;

	info("Unloading BRCM rfcomm driver");

	for (i = 0; i < UART_NR; i++)
		uart_remove_one_port(&btlinux_port_reg, &btlinux_ports[i].port);

	uart_unregister_driver(&btlinux_port_reg);
	unregister_chrdev(btport_major, BTPORT_DEV_NAME);
}

static int show_bt_dun_enabled(char *buffer, struct kernel_param *kp)
{
	struct uart_btlinux_port *btlinux_port;

	btlinux_port = &btlinux_ports[0];

	buffer[0] = '0' + btlinux_port->is_open;

	return 1;
}

module_param_call(bt_dun_enabled, NULL, show_bt_dun_enabled, NULL, 0664);

module_init(btlinux_port_init);
module_exit(btlinux_port_exit);

