/* drivers/serial/msm_serial_hs.c
 *
 * MSM 7k High speed uart driver
 *
 * Copyright (c) 2008 Google Inc.
 * Copyright (c) 2007-2011, Code Aurora Forum. All rights reserved.
 * Modified: Nick Pelly <npelly@google.com>
 *
 * All source code in this file is licensed under the following license
 * except where indicated.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * Has optional support for uart power management independent of linux
 * suspend/resume:
 *
 * RX wakeup.
 * UART wakeup can be triggered by RX activity (using a wakeup GPIO on the
 * UART RX pin). This should only be used if there is not a wakeup
 * GPIO on the UART CTS, and the first RX byte is known (for example, with the
 * Bluetooth Texas Instruments HCILL protocol), since the first RX byte will
 * always be lost. RTS will be asserted even while the UART is off in this mode
 * of operation. See msm_serial_hs_platform_data.rx_wakeup_irq.
 */

#include <linux/module.h>

#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/wait.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include <linux/atomic.h>
#include <linux/irq.h>
#include <asm/system.h>

#include <mach/hardware.h>
#include <mach/dma.h>
#include <mach/msm_serial_hs.h>
/* for brcm */
#include <linux/gpio.h>
#include <linux/uaccess.h>

#include "msm_serial_hs_hwreg.h"

#include <linux/poison.h>
#include <linux/delay.h>

static int hs_serial_debug_mask = 1;
module_param_named(debug_mask, hs_serial_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

/* tx_empty() no spin lock */
#if defined(CONFIG_MACH_RUNNYMEDE)
#define TX_EMPTY_NO_SPINLOCK 1
#endif

/* for brcm */
#define USE_BCM_BT_CHIP
#define T_LOW 0
#define T_HIGH 1
/* for brcm bt serial debug */
#define BT_SERIAL_DBG


enum flush_reason {
	FLUSH_NONE,
	FLUSH_DATA_READY,
	FLUSH_DATA_INVALID,  /* values after this indicate invalid data */
	FLUSH_IGNORE = FLUSH_DATA_INVALID,
	FLUSH_STOP,
	FLUSH_SHUTDOWN,
};

enum msm_hs_clk_states_e {
	MSM_HS_CLK_PORT_OFF,     /* port not in use */
	MSM_HS_CLK_OFF,          /* clock disabled */
	MSM_HS_CLK_REQUEST_OFF,  /* disable after TX and RX flushed */
	MSM_HS_CLK_ON,           /* clock enabled */
};

/* Track the forced RXSTALE flush during clock off sequence.
 * These states are only valid during MSM_HS_CLK_REQUEST_OFF */
enum msm_hs_clk_req_off_state_e {
	CLK_REQ_OFF_START,
	CLK_REQ_OFF_RXSTALE_ISSUED,
	CLK_REQ_OFF_FLUSH_ISSUED,
	CLK_REQ_OFF_RXSTALE_FLUSHED,
};

struct msm_hs_tx {
	unsigned int tx_ready_int_en;  /* ok to dma more tx */
	unsigned int dma_in_flight;    /* tx dma in progress */
	struct msm_dmov_cmd xfer;
	dmov_box *command_ptr;
	u32 *command_ptr_ptr;
	dma_addr_t mapped_cmd_ptr;
	dma_addr_t mapped_cmd_ptr_ptr;
	int tx_count;
	dma_addr_t dma_base;
	struct tasklet_struct tlet;
	#ifdef USE_BCM_BT_CHIP	/* brcm tx wakelock */
	struct wake_lock brcm_tx_wake_lock;
	#endif
};

struct msm_hs_rx {
	enum flush_reason flush;
	struct msm_dmov_cmd xfer;
	dma_addr_t cmdptr_dmaaddr;
	dmov_box *command_ptr;
	u32 *command_ptr_ptr;
	dma_addr_t mapped_cmd_ptr;
	wait_queue_head_t wait;
	dma_addr_t rbuffer;
	unsigned char *buffer;
	unsigned int buffer_pending;
	struct dma_pool *pool;
	struct wake_lock wake_lock;
	struct delayed_work flip_insert_work;
	struct tasklet_struct tlet;
	struct work_struct tty_work;
	struct wake_lock brcm_rx_wake_lock;
	unsigned int is_brcm_rx_wake_locked;
};

enum buffer_states {
	NONE_PENDING = 0x0,
	FIFO_OVERRUN = 0x1,
	PARITY_ERROR = 0x2,
	CHARS_NORMAL = 0x4,
};

/* optional low power wakeup, typically on a GPIO RX irq */
struct msm_hs_wakeup {
	int irq;  /* < 0 indicates low power wakeup disabled */
	unsigned char ignore;  /* bool */

	/* bool: inject char into rx tty on wakeup */
	unsigned char inject_rx;
	char rx_to_inject;
};

struct msm_hs_port {
	struct uart_port uport;
	unsigned long imr_reg;  /* shadow value of UARTDM_IMR */
	struct clk *clk;
	struct clk *pclk;
	struct msm_hs_tx tx;
	struct msm_hs_rx rx;
	/* gsbi uarts have to do additional writes to gsbi memory */
	/* block and top control status block. The following pointers */
	/* keep a handle to these blocks. */
	unsigned char __iomem	*mapped_gsbi;
	int dma_tx_channel;
	int dma_rx_channel;
	int dma_tx_crci;
	int dma_rx_crci;
	struct hrtimer clk_off_timer;  /* to poll TXEMT before clock off */
	ktime_t clk_off_delay;
	enum msm_hs_clk_states_e clk_state;
	enum msm_hs_clk_req_off_state_e clk_req_off_state;

	struct msm_hs_wakeup wakeup;
	/* optional callback to exit low power mode */
	void (*exit_lpm_cb)(struct uart_port *);

	struct wake_lock dma_wake_lock;  /* held while any DMA active */

	#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
	unsigned char bt_wakeup_pin_supported;
	unsigned char bt_wakeup_pin;
	unsigned char bt_wakeup_level;
	unsigned char bt_wakeup_assert_inadvance;
	unsigned char host_wakeup_pin;
	unsigned char host_wakeup_level;
	unsigned char host_want_sleep;
	int request_clk_off_delay;
	#endif
};

#define MSM_UARTDM_BURST_SIZE 16   /* DM burst size (in bytes) */
#define UARTDM_TX_BUF_SIZE UART_XMIT_SIZE
#define UARTDM_RX_BUF_SIZE 512
#define RETRY_TIMEOUT 5
#define UARTDM_NR 2

static struct msm_hs_port q_uart_port[UARTDM_NR];
static struct platform_driver msm_serial_hs_platform_driver;
static struct uart_driver msm_hs_driver;
static struct uart_ops msm_hs_ops;
static struct workqueue_struct *msm_hs_workqueue;

#define UARTDM_TO_MSM(uart_port) \
	container_of((uart_port), struct msm_hs_port, uport)

static ssize_t show_clock(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
#if 0
	int state = 1;
	enum msm_hs_clk_states_e clk_state;
	unsigned long flags;

	struct platform_device *pdev = container_of(dev, struct
						    platform_device, dev);
	struct msm_hs_port *msm_uport = &q_uart_port[pdev->id];

	spin_lock_irqsave(&msm_uport->uport.lock, flags);
	clk_state = msm_uport->clk_state;
	spin_unlock_irqrestore(&msm_uport->uport.lock, flags);

	if (clk_state <= MSM_HS_CLK_OFF)
		state = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", state);
#else
	return 0;
#endif
}

static ssize_t set_clock(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
#if 0
	int state;
	struct platform_device *pdev = container_of(dev, struct
						    platform_device, dev);
	struct msm_hs_port *msm_uport = &q_uart_port[pdev->id];

	state = buf[0] - '0';
	switch (state) {
	case 0: {
		msm_hs_request_clock_off(&msm_uport->uport);
		break;
	}
	case 1: {
		msm_hs_request_clock_on(&msm_uport->uport);
		break;
	}
	default: {
		return -EINVAL;
	}
	}
	return count;
#else
	return 0;
#endif
}

static DEVICE_ATTR(clock, S_IWUSR | S_IRUGO, show_clock, set_clock);

static inline unsigned int use_low_power_wakeup(struct msm_hs_port *msm_uport)
{
	return (msm_uport->wakeup.irq > 0);
}

static inline int is_gsbi_uart(struct msm_hs_port *msm_uport)
{
	/* assume gsbi uart if gsbi resource found in pdata */
	return ((msm_uport->mapped_gsbi != NULL));
}

static inline unsigned int msm_hs_read(struct uart_port *uport,
				       unsigned int offset)
{
	return readl_relaxed(uport->membase + offset);
}

static inline void msm_hs_write(struct uart_port *uport, unsigned int offset,
				 unsigned int value)
{
	writel_relaxed(value, uport->membase + offset);
}

static void msm_hs_release_port(struct uart_port *port)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *gsbi_resource;
	resource_size_t size;

	if (is_gsbi_uart(msm_uport)) {
		iowrite32(GSBI_PROTOCOL_IDLE, msm_uport->mapped_gsbi +
			  GSBI_CONTROL_ADDR);
		gsbi_resource = platform_get_resource_byname(pdev,
							     IORESOURCE_MEM,
							     "gsbi_resource");
		if (gsbi_resource == NULL) {
			printk(KERN_ERR "Can't get GSBI RES\n");
			return;
		}
		size = gsbi_resource->end - gsbi_resource->start + 1;
		release_mem_region(gsbi_resource->start, size);
		iounmap(msm_uport->mapped_gsbi);
		msm_uport->mapped_gsbi = NULL;
	}
}

static int msm_hs_request_port(struct uart_port *port)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *gsbi_resource;
	resource_size_t size;

	gsbi_resource = platform_get_resource_byname(pdev,
						     IORESOURCE_MEM,
						     "gsbi_resource");

	if (gsbi_resource) {
		size = gsbi_resource->end - gsbi_resource->start + 1;
		if (unlikely(!request_mem_region(gsbi_resource->start, size,
						 "msm_serial_hs")))
			return -EBUSY;
		msm_uport->mapped_gsbi = ioremap(gsbi_resource->start,
						 size);
		if (!msm_uport->mapped_gsbi) {
			release_mem_region(gsbi_resource->start, size);
			return -EBUSY;
		}
	}
	/* no gsbi uart */
	return 0;
}

static int __devexit msm_hs_remove(struct platform_device *pdev)
{

	struct msm_hs_port *msm_uport;
	struct device *dev;

	if (pdev->id < 0 || pdev->id >= UARTDM_NR) {
		printk(KERN_ERR "[BT]Invalid plaform device ID = %d\n",
				pdev->id);
		return -EINVAL;
	}

	msm_uport = &q_uart_port[pdev->id];
	dev = msm_uport->uport.dev;

	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_clock.attr);

	dma_unmap_single(dev, msm_uport->rx.mapped_cmd_ptr, sizeof(dmov_box),
			 DMA_TO_DEVICE);
	dma_pool_free(msm_uport->rx.pool, msm_uport->rx.buffer,
		      msm_uport->rx.rbuffer);
	dma_pool_destroy(msm_uport->rx.pool);

	dma_unmap_single(dev, msm_uport->rx.cmdptr_dmaaddr, sizeof(u32 *),
			 DMA_TO_DEVICE);
	dma_unmap_single(dev, msm_uport->tx.mapped_cmd_ptr_ptr, sizeof(u32 *),
			 DMA_TO_DEVICE);
	dma_unmap_single(dev, msm_uport->tx.mapped_cmd_ptr, sizeof(dmov_box),
			 DMA_TO_DEVICE);

	/* destroy tx wakelock */
	wake_lock_destroy(&msm_uport->tx.brcm_tx_wake_lock);
	/* destroy rx wakelock */
	wake_lock_destroy(&msm_uport->rx.brcm_rx_wake_lock);

	wake_lock_destroy(&msm_uport->rx.wake_lock);
	wake_lock_destroy(&msm_uport->dma_wake_lock);

	uart_remove_one_port(&msm_hs_driver, &msm_uport->uport);
	clk_put(msm_uport->clk);

	/* Free the tx resources */
	kfree(msm_uport->tx.command_ptr);
	kfree(msm_uport->tx.command_ptr_ptr);

	/* Free the rx resources */
	kfree(msm_uport->rx.command_ptr);
	kfree(msm_uport->rx.command_ptr_ptr);

	iounmap(msm_uport->uport.membase);

	return 0;
}

static int msm_hs_init_clk(struct uart_port *uport)
{
	int ret;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	wake_lock(&msm_uport->dma_wake_lock);
	/* Set up the MREG/NREG/DREG/MNDREG */
	ret = clk_set_rate(msm_uport->clk, uport->uartclk);
	if (ret) {
		printk(KERN_WARNING "[BT]Error setting clock rate on UART\n");
		return ret;
	}

	ret = clk_enable(msm_uport->clk);
	if (ret) {
		printk(KERN_ERR "[BT]Error could not turn on UART clk\n");
		return ret;
	}
	if (msm_uport->pclk) {
		ret = clk_enable(msm_uport->pclk);
		if (ret) {
			dev_err(uport->dev,
				"[BT]Error could not turn on UART pclk\n");
			return ret;
		}
	}

	msm_uport->clk_state = MSM_HS_CLK_ON;

	return 0;
}

/*
 * programs the UARTDM_CSR register with correct bit rates
 *
 * Interrupts should be disabled before we are called, as
 * we modify Set Baud rate
 * Set receive stale interrupt level, dependant on Bit Rate
 * Goal is to have around 8 ms before indicate stale.
 * roundup (((Bit Rate * .008) / 10) + 1
 */
static void msm_hs_set_bps_locked(struct uart_port *uport,
			       unsigned int bps)
{
	unsigned long rxstale;
	unsigned long data;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	switch (bps) {
	case 300:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x00);
		rxstale = 1;
		break;
	case 600:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x11);
		rxstale = 1;
		break;
	case 1200:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x22);
		rxstale = 1;
		break;
	case 2400:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x33);
		rxstale = 1;
		break;
	case 4800:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x44);
		rxstale = 1;
		break;
	case 9600:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x55);
		rxstale = 2;
		break;
	case 14400:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x66);
		rxstale = 3;
		break;
	case 19200:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x77);
		rxstale = 4;
		break;
	case 28800:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x88);
		rxstale = 6;
		break;
	case 38400:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x99);
		rxstale = 8;
		break;
	case 57600:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xaa);
		rxstale = 16;
		break;
	case 76800:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xbb);
		rxstale = 16;
		break;
	case 115200:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xcc);
		rxstale = 31;
		break;
	case 230400:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xee);
		rxstale = 31;
		break;
	case 460800:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xff);
		rxstale = 31;
		break;
	case 4000000:
	case 3686400:
	case 3200000:
	case 3500000:
	case 3000000:
	case 2500000:
	case 1500000:
	case 1152000:
	case 1000000:
	case 921600:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xff);
		rxstale = 31;
		break;
	default:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xff);
		/* default to 9600 */
		bps = 9600;
		rxstale = 2;
		break;
	}
	/*
	 * uart baud rate depends on CSR and MND Values
	 * we are updating CSR before and then calling
	 * clk_set_rate which updates MND Values. Hence
	 * dsb requires here.
	 */
	mb();
	if (bps > 460800)
		uport->uartclk = bps * 16;
	else
		uport->uartclk = 7372800;

	if (clk_set_rate(msm_uport->clk, uport->uartclk)) {
		printk(KERN_WARNING "[BT]Error setting clock rate on UART\n");
		return;
	}

	data = rxstale & UARTDM_IPR_STALE_LSB_BMSK;
	data |= UARTDM_IPR_STALE_TIMEOUT_MSB_BMSK & (rxstale << 2);

	msm_hs_write(uport, UARTDM_IPR_ADDR, data);
	/*
	 * It is suggested to do reset of transmitter and receiver after
	 * changing any protocol configuration. Here Baud rate and stale
	 * timeout are getting updated. Hence reset transmitter and receiver.
	 */
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_TX);
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_RX);
}


static void msm_hs_set_std_bps_locked(struct uart_port *uport,
			       unsigned int bps)
{
	unsigned long rxstale;
	unsigned long data;

	switch (bps) {
	case 9600:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x99);
		rxstale = 2;
		break;
	case 14400:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xaa);
		rxstale = 3;
		break;
	case 19200:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xbb);
		rxstale = 4;
		break;
	case 28800:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xcc);
		rxstale = 6;
		break;
	case 38400:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xdd);
		rxstale = 8;
		break;
	case 57600:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xee);
		rxstale = 16;
		break;
	case 115200:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0xff);
		rxstale = 31;
		break;
	default:
		msm_hs_write(uport, UARTDM_CSR_ADDR, 0x99);
		/* default to 9600 */
		bps = 9600;
		rxstale = 2;
		break;
	}

	data = rxstale & UARTDM_IPR_STALE_LSB_BMSK;
	data |= UARTDM_IPR_STALE_TIMEOUT_MSB_BMSK & (rxstale << 2);

	msm_hs_write(uport, UARTDM_IPR_ADDR, data);
}

/*
 * termios :  new ktermios
 * oldtermios:  old ktermios previous setting
 *
 * Configure the serial port
 */
static void msm_hs_set_termios(struct uart_port *uport,
				   struct ktermios *termios,
				   struct ktermios *oldtermios)
{
	unsigned int bps;
	unsigned long data;
	unsigned long flags;
	unsigned int c_cflag = termios->c_cflag;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	spin_lock_irqsave(&uport->lock, flags);
	clk_enable(msm_uport->clk);

	/*
	 * Disable Rx channel of UARTDM
	 * DMA Rx Stall happens if enqueue and flush of Rx command happens
	 * concurrently. Hence before changing the baud rate/protocol
	 * configuration and sending flush command to ADM, disable the Rx
	 * channel of UARTDM.
	 * Note: should not reset the receiver here immediately as it is not
	 * suggested to do disable/reset or reset/disable at the same time.
	 */
	data = msm_hs_read(uport, UARTDM_DMEN_ADDR);
	data &= ~UARTDM_RX_DM_EN_BMSK;
	msm_hs_write(uport, UARTDM_DMEN_ADDR, data);

	/* 300 is the minimum baud support by the driver  */
	bps = uart_get_baud_rate(uport, termios, oldtermios, 200, 4000000);

	/* Temporary remapping  200 BAUD to 3.2 mbps */
	if (bps == 200)
		bps = 3200000;

	uport->uartclk = clk_get_rate(msm_uport->clk);
	if (!uport->uartclk)
		msm_hs_set_std_bps_locked(uport, bps);
	else
		msm_hs_set_bps_locked(uport, bps);

	data = msm_hs_read(uport, UARTDM_MR2_ADDR);
	data &= ~UARTDM_MR2_PARITY_MODE_BMSK;
	/* set parity */
	if (PARENB == (c_cflag & PARENB)) {
		if (PARODD == (c_cflag & PARODD))
			data |= ODD_PARITY;
		else if (CMSPAR == (c_cflag & CMSPAR))
			data |= SPACE_PARITY;
		else
			data |= EVEN_PARITY;
	}

	/* Set bits per char */
	data &= ~UARTDM_MR2_BITS_PER_CHAR_BMSK;

	switch (c_cflag & CSIZE) {
	case CS5:
		data |= FIVE_BPC;
		break;
	case CS6:
		data |= SIX_BPC;
		break;
	case CS7:
		data |= SEVEN_BPC;
		break;
	default:
		data |= EIGHT_BPC;
		break;
	}
	/* stop bits */
	if (c_cflag & CSTOPB) {
		data |= STOP_BIT_TWO;
	} else {
		/* otherwise 1 stop bit */
		data |= STOP_BIT_ONE;
	}
	data |= UARTDM_MR2_ERROR_MODE_BMSK;
	/* write parity/bits per char/stop bit configuration */
	msm_hs_write(uport, UARTDM_MR2_ADDR, data);

	/* Configure HW flow control */
	data = msm_hs_read(uport, UARTDM_MR1_ADDR);

	data &= ~(UARTDM_MR1_CTS_CTL_BMSK | UARTDM_MR1_RX_RDY_CTL_BMSK);

	if (c_cflag & CRTSCTS) {
		data |= UARTDM_MR1_CTS_CTL_BMSK;
		data |= UARTDM_MR1_RX_RDY_CTL_BMSK;
	}

	msm_hs_write(uport, UARTDM_MR1_ADDR, data);

	uport->ignore_status_mask = termios->c_iflag & INPCK;
	uport->ignore_status_mask |= termios->c_iflag & IGNPAR;
	uport->read_status_mask = (termios->c_cflag & CREAD);

	msm_hs_write(uport, UARTDM_IMR_ADDR, 0);

	/* Set Transmit software time out */
	uart_update_timeout(uport, c_cflag, bps);

	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_RX);
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_TX);

	if (msm_uport->rx.flush == FLUSH_NONE) {
		wake_lock(&msm_uport->rx.wake_lock);
		msm_uport->rx.flush = FLUSH_IGNORE;
		/*
		 * Before using dmov APIs make sure that
		 * previous writel are completed. Hence
		 * dsb requires here.
		 */
		mb();
		/* do discard flush */
		msm_dmov_stop_cmd(msm_uport->dma_rx_channel,
				  &msm_uport->rx.xfer, 0);
	}

	msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
	/* calling other hardware component here clk_disable API. */
	mb();
	clk_disable(msm_uport->clk);
	spin_unlock_irqrestore(&uport->lock, flags);
}

/*
 *  Standard API, Transmitter
 *  Any character in the transmit shift register is sent
 */
unsigned int brcm_msm_hs_tx_empty(struct uart_port *uport)
{
	unsigned int data;
	unsigned int ret = 0;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

#ifndef TX_EMPTY_NO_SPINLOCK
	unsigned long flags;
	spin_lock_irqsave(&uport->lock, flags);
#endif
	clk_enable(msm_uport->clk);

	data = msm_hs_read(uport, UARTDM_SR_ADDR);
	if (data & UARTDM_SR_TXEMT_BMSK)
		ret = TIOCSER_TEMT;

	clk_disable(msm_uport->clk);
#ifndef TX_EMPTY_NO_SPINLOCK
	spin_unlock_irqrestore(&uport->lock, flags);
#endif
	return ret;
}

/*
 *  Standard API, Stop transmitter.
 *  Any character in the transmit shift register is sent as
 *  well as the current data mover transfer .
 */
static void msm_hs_stop_tx_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	msm_uport->tx.tx_ready_int_en = 0;
}

/*
 *  Standard API, Stop receiver as soon as possible.
 *
 *  Function immediately terminates the operation of the
 *  channel receiver and any incoming characters are lost. None
 *  of the receiver status bits are affected by this command and
 *  characters that are already in the receive FIFO there.
 */
static void msm_hs_stop_rx_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	unsigned int data;

	clk_enable(msm_uport->clk);

	/* disable dlink */
	data = msm_hs_read(uport, UARTDM_DMEN_ADDR);
	data &= ~UARTDM_RX_DM_EN_BMSK;
	msm_hs_write(uport, UARTDM_DMEN_ADDR, data);

	/* calling DMOV or CLOCK API. Hence mb() */
	mb();
	/* Disable the receiver */
	if (msm_uport->rx.flush == FLUSH_NONE) {
		wake_lock(&msm_uport->rx.wake_lock);
		/* do discard flush */
		msm_dmov_stop_cmd(msm_uport->dma_rx_channel,
				  &msm_uport->rx.xfer, 0);
	}
	if (msm_uport->rx.flush != FLUSH_SHUTDOWN)
		msm_uport->rx.flush = FLUSH_STOP;

	clk_disable(msm_uport->clk);
}

/*  Transmit the next chunk of data */
static void msm_hs_submit_tx_locked(struct uart_port *uport)
{
	int left;
	int tx_count;
	int aligned_tx_count;
	dma_addr_t src_addr;
	dma_addr_t aligned_src_addr;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	struct msm_hs_tx *tx = &msm_uport->tx;
	struct circ_buf *tx_buf = &msm_uport->uport.state->xmit;

	if (uart_circ_empty(tx_buf) || uport->state->port.tty->stopped) {
		msm_hs_stop_tx_locked(uport);
		return;
	}

	tx->dma_in_flight = 1;

	tx_count = uart_circ_chars_pending(tx_buf);

	if (UARTDM_TX_BUF_SIZE < tx_count)
		tx_count = UARTDM_TX_BUF_SIZE;

	left = UART_XMIT_SIZE - tx_buf->tail;

	if (tx_count > left)
		tx_count = left;

	src_addr = tx->dma_base + tx_buf->tail;
	/* Mask the src_addr to align on a cache
	 * and add those bytes to tx_count */
	aligned_src_addr = src_addr & ~(dma_get_cache_alignment() - 1);
	aligned_tx_count = tx_count + src_addr - aligned_src_addr;

	dma_sync_single_for_device(uport->dev, aligned_src_addr,
			aligned_tx_count, DMA_TO_DEVICE);

	tx->command_ptr->num_rows = (((tx_count + 15) >> 4) << 16) |
				     ((tx_count + 15) >> 4);
	tx->command_ptr->src_row_addr = src_addr;

	dma_sync_single_for_device(uport->dev, tx->mapped_cmd_ptr,
				   sizeof(dmov_box), DMA_TO_DEVICE);

	*tx->command_ptr_ptr = CMD_PTR_LP | DMOV_CMD_ADDR(tx->mapped_cmd_ptr);

	/* Save tx_count to use in Callback */
	tx->tx_count = tx_count;
	msm_hs_write(uport, UARTDM_NCF_TX_ADDR, tx_count);

	/* Disable the tx_ready interrupt */
	msm_uport->imr_reg &= ~UARTDM_ISR_TX_READY_BMSK;
	msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
	/* Calling next DMOV API. Hence mb() here. */
	mb();

	dma_sync_single_for_device(uport->dev, tx->mapped_cmd_ptr_ptr,
				   sizeof(u32 *), DMA_TO_DEVICE);

	if ((tx->xfer.list.next == LIST_POISON1)
			||	(tx->xfer.list.next == NULL))
		msm_dmov_enqueue_cmd(msm_uport->dma_tx_channel, &tx->xfer);
	else
		printk(KERN_ERR "[BT]Error: tx already started in dmov\n");
}

/* Start to receive the next chunk of data */
static void msm_hs_start_rx_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	unsigned int buffer_pending = msm_uport->rx.buffer_pending;
	unsigned int data;

	msm_uport->rx.buffer_pending = 0;
	if (buffer_pending && hs_serial_debug_mask)
		printk(KERN_ERR "[BT]Error: rx started in buffer state = %x",
		       buffer_pending);

	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_STALE_INT);
	msm_hs_write(uport, UARTDM_DMRX_ADDR, UARTDM_RX_BUF_SIZE);
	msm_hs_write(uport, UARTDM_CR_ADDR, STALE_EVENT_ENABLE);
	msm_uport->imr_reg |= UARTDM_ISR_RXLEV_BMSK;

	/*
	 * Enable UARTDM Rx Interface as previously it has been
	 * disable in set_termios before configuring baud rate.
	 */
	data = msm_hs_read(uport, UARTDM_DMEN_ADDR);
	data |= UARTDM_RX_DM_EN_BMSK;
	msm_hs_write(uport, UARTDM_DMEN_ADDR, data);
	msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
	/* Calling next DMOV API. Hence mb() here. */
	mb();

	msm_uport->rx.flush = FLUSH_NONE;

	if ((msm_uport->rx.xfer.list.next == LIST_POISON1)
			|| (msm_uport->rx.xfer.list.next == NULL))
		msm_dmov_enqueue_cmd(msm_uport->dma_rx_channel,
				&msm_uport->rx.xfer);
	else
		printk(KERN_ERR "[BT]Error: rx already started in dmov\n");

}

static void flip_insert_work(struct work_struct *work)
{
	unsigned long flags;
	int retval;
	struct msm_hs_port *msm_uport =
		container_of(work, struct msm_hs_port,
			     rx.flip_insert_work.work);
	struct tty_struct *tty = msm_uport->uport.state->port.tty;

	spin_lock_irqsave(&msm_uport->uport.lock, flags);
	if (msm_uport->rx.buffer_pending == NONE_PENDING) {
		if (hs_serial_debug_mask)
			printk(KERN_ERR "[BT]Error: No buffer pending in %s",
			       __func__);
		return;
	}
	if (msm_uport->rx.buffer_pending & FIFO_OVERRUN) {
		retval = tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		if (retval)
			msm_uport->rx.buffer_pending &= ~FIFO_OVERRUN;
	}
	if (msm_uport->rx.buffer_pending & PARITY_ERROR) {
		retval = tty_insert_flip_char(tty, 0, TTY_PARITY);
		if (retval)
			msm_uport->rx.buffer_pending &= ~PARITY_ERROR;
	}
	if (msm_uport->rx.buffer_pending & CHARS_NORMAL) {
		int rx_count, rx_offset;
		rx_count = (msm_uport->rx.buffer_pending & 0xFFFF0000) >> 16;
		rx_offset = (msm_uport->rx.buffer_pending & 0xFFD0) >> 5;
		retval = tty_insert_flip_string(tty, msm_uport->rx.buffer +
						rx_offset, rx_count);
		msm_uport->rx.buffer_pending &= (FIFO_OVERRUN |
						 PARITY_ERROR);
		if (retval != rx_count)
			msm_uport->rx.buffer_pending |= CHARS_NORMAL |
				retval << 8 | (rx_count - retval) << 16;
	}
	if (msm_uport->rx.buffer_pending)
		schedule_delayed_work(&msm_uport->rx.flip_insert_work,
				      msecs_to_jiffies(RETRY_TIMEOUT));
	else
		if ((msm_uport->clk_state == MSM_HS_CLK_ON) &&
		    (msm_uport->rx.flush <= FLUSH_IGNORE)) {
			if (hs_serial_debug_mask)
				printk(KERN_WARNING
				       "[BT]msm_serial_hs: "
				       "Pending buffers cleared. "
				       "Restarting\n");
			msm_hs_start_rx_locked(&msm_uport->uport);
		}
	spin_unlock_irqrestore(&msm_uport->uport.lock, flags);
	tty_flip_buffer_push(tty);
}

static void msm_serial_hs_rx_tlet(unsigned long tlet_ptr)
{
	int retval;
	int rx_count;
	unsigned long status;
	unsigned long flags;
	unsigned int error_f = 0;
	struct uart_port *uport;
	struct msm_hs_port *msm_uport;
	unsigned int flush;
	struct tty_struct *tty;

	msm_uport = container_of((struct tasklet_struct *)tlet_ptr,
				 struct msm_hs_port, rx.tlet);
	uport = &msm_uport->uport;
	tty = uport->state->port.tty;

	status = msm_hs_read(uport, UARTDM_SR_ADDR);

	spin_lock_irqsave(&uport->lock, flags);

	clk_enable(msm_uport->clk);
	msm_hs_write(uport, UARTDM_CR_ADDR, STALE_EVENT_DISABLE);

	/* overflow is not connect to data in a FIFO */
	if (unlikely((status & UARTDM_SR_OVERRUN_BMSK) &&
		     (uport->read_status_mask & CREAD))) {
		retval = tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		if (!retval)
			msm_uport->rx.buffer_pending |= TTY_OVERRUN;
		uport->icount.buf_overrun++;
		error_f = 1;
	}

	if (!(uport->ignore_status_mask & INPCK))
		status = status & ~(UARTDM_SR_PAR_FRAME_BMSK);

	if (unlikely(status & UARTDM_SR_PAR_FRAME_BMSK)) {
		/* Can not tell difference between parity & frame error */
		uport->icount.parity++;
		error_f = 1;
		if (uport->ignore_status_mask & IGNPAR) {
			retval = tty_insert_flip_char(tty, 0, TTY_PARITY);
			if (!retval)
				msm_uport->rx.buffer_pending |= TTY_PARITY;
		}
	}

	if (error_f)
		msm_hs_write(uport, UARTDM_CR_ADDR, RESET_ERROR_STATUS);

	if (msm_uport->clk_req_off_state == CLK_REQ_OFF_FLUSH_ISSUED)
		msm_uport->clk_req_off_state = CLK_REQ_OFF_RXSTALE_FLUSHED;
	flush = msm_uport->rx.flush;
	if (flush == FLUSH_IGNORE)
		if (!msm_uport->rx.buffer_pending)
			msm_hs_start_rx_locked(uport);

	if (flush == FLUSH_STOP) {
		msm_uport->rx.flush = FLUSH_SHUTDOWN;
		wake_up(&msm_uport->rx.wait);
	}
	if (flush >= FLUSH_DATA_INVALID)
		goto out;

	rx_count = msm_hs_read(uport, UARTDM_RX_TOTAL_SNAP_ADDR);

	/* order the read of rx.buffer */
	rmb();

	if (0 != (uport->read_status_mask & CREAD)) {
		retval = tty_insert_flip_string(tty, msm_uport->rx.buffer,
						rx_count);
		if (retval != rx_count) {
			msm_uport->rx.buffer_pending |= CHARS_NORMAL |
				retval << 5 | (rx_count - retval) << 16;
		}
	}

	/* order the read of rx.buffer and the start of next rx xfer */
	wmb();

	if (!msm_uport->rx.buffer_pending)
		msm_hs_start_rx_locked(uport);

out:
	if (msm_uport->rx.buffer_pending) {
		if (hs_serial_debug_mask)
			printk(KERN_WARNING
			       "[BT]msm_serial_hs: "
			       "tty buffer exhausted. "
			       "Stalling\n");
		schedule_delayed_work(&msm_uport->rx.flip_insert_work
				      , msecs_to_jiffies(RETRY_TIMEOUT));
	}
	clk_disable(msm_uport->clk);
	/* release wakelock in 500ms, not immediately, because higher layers
	 * don't always take wakelocks when they should */
	wake_lock_timeout(&msm_uport->rx.wake_lock, HZ / 2);
	/* tty_flip_buffer_push() might call msm_hs_start(), so unlock */
	spin_unlock_irqrestore(&uport->lock, flags);
	if (flush < FLUSH_DATA_INVALID)
		queue_work(msm_hs_workqueue, &msm_uport->rx.tty_work);
}

/* Enable the transmitter Interrupt */
static void msm_hs_start_tx_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	clk_enable(msm_uport->clk);

/*
	if (msm_uport->exit_lpm_cb)
		msm_uport->exit_lpm_cb(uport);
*/ /* remove in kernel 3.0 */

	if (msm_uport->tx.tx_ready_int_en == 0) {
		msm_uport->tx.tx_ready_int_en = 1;
		if (msm_uport->tx.dma_in_flight == 0)
			msm_hs_submit_tx_locked(uport);
	}

	clk_disable(msm_uport->clk);
}

/*
 *  This routine is called when we are done with a DMA transfer
 *
 *  This routine is registered with Data mover when we set
 *  up a Data Mover transfer. It is called from Data mover ISR
 *  when the DMA transfer is done.
 */
static void msm_hs_dmov_tx_callback(struct msm_dmov_cmd *cmd_ptr,
					unsigned int result,
					struct msm_dmov_errdata *err)
{
	struct msm_hs_port *msm_uport;

	WARN_ON(result != 0x80000002);  /* DMA did not finish properly */

	msm_uport = container_of(cmd_ptr, struct msm_hs_port, tx.xfer);

	tasklet_schedule(&msm_uport->tx.tlet);
}

static void msm_serial_hs_tx_tlet(unsigned long tlet_ptr)
{
	unsigned long flags;
	struct msm_hs_port *msm_uport = container_of((struct tasklet_struct *)
				tlet_ptr, struct msm_hs_port, tx.tlet);

	spin_lock_irqsave(&(msm_uport->uport.lock), flags);
	clk_enable(msm_uport->clk);

	msm_uport->imr_reg |= UARTDM_ISR_TX_READY_BMSK;
	msm_hs_write(&(msm_uport->uport), UARTDM_IMR_ADDR, msm_uport->imr_reg);
	/* Calling clk API. Hence mb() requires. */
	mb();

	clk_disable(msm_uport->clk);
	spin_unlock_irqrestore(&(msm_uport->uport.lock), flags);
}

/*
 * This routine is called when we are done with a DMA transfer or the
 * a flush has been sent to the data mover driver.
 *
 * This routine is registered with Data mover when we set up a Data Mover
 *  transfer. It is called from Data mover ISR when the DMA transfer is done.
 */
static void msm_hs_dmov_rx_callback(struct msm_dmov_cmd *cmd_ptr,
					unsigned int result,
					struct msm_dmov_errdata *err)
{
	struct msm_hs_port *msm_uport;

	msm_uport = container_of(cmd_ptr, struct msm_hs_port, rx.xfer);

	tasklet_schedule(&msm_uport->rx.tlet);
}

static void msm_hs_tty_flip_buffer_work(struct work_struct *work)
{
	struct msm_hs_port *msm_uport =
			container_of(work, struct msm_hs_port, rx.tty_work);
	struct tty_struct *tty = msm_uport->uport.state->port.tty;

	tty_flip_buffer_push(tty);
}

/*
 *  Standard API, Current states of modem control inputs
 *
 * Since CTS can be handled entirely by HARDWARE we always
 * indicate clear to send and count on the TX FIFO to block when
 * it fills up.
 *
 * - TIOCM_DCD
 * - TIOCM_CTS
 * - TIOCM_DSR
 * - TIOCM_RI
 *  (Unsupported) DCD and DSR will return them high. RI will return low.
 */
static unsigned int msm_hs_get_mctrl_locked(struct uart_port *uport)
{
	return TIOCM_DSR | TIOCM_CAR | TIOCM_CTS;
}

/*
 *  Standard API, Set or clear RFR_signal
 *
 * Set RFR high, (Indicate we are not ready for data), we disable auto
 * ready for receiving and then set RFR_N high. To set RFR to low we just turn
 * back auto ready for receiving and it should lower RFR signal
 * when hardware is ready
 */
static void msm_hs_set_mctrl_locked(struct uart_port *uport,
				    unsigned int mctrl)
{
	unsigned int set_rts;
	unsigned int data;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	clk_enable(msm_uport->clk);

	/* RTS is active low */
	set_rts = TIOCM_RTS & mctrl ? 0 : 1;

	data = msm_hs_read(uport, UARTDM_MR1_ADDR);
	if (set_rts) {
		/*disable auto ready-for-receiving */
		data &= ~UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hs_write(uport, UARTDM_MR1_ADDR, data);
		/* set RFR_N to high */
		msm_hs_write(uport, UARTDM_CR_ADDR, RFR_HIGH);
	} else {
		/* Enable auto ready-for-receiving */
		data |= UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hs_write(uport, UARTDM_MR1_ADDR, data);
	}
	/* Calling CLOCK API. Hence mb() requires. */
	mb();
	clk_disable(msm_uport->clk);
}

/*
void msm_hs_set_mctrl(struct uart_port *uport,
				    unsigned int mctrl)
{
	unsigned long flags;

	spin_lock_irqsave(&uport->lock, flags);
	msm_hs_set_mctrl_locked(uport, mctrl);
	spin_unlock_irqrestore(&uport->lock, flags);
}
EXPORT_SYMBOL(msm_hs_set_mctrl);
*/

/* Standard API, Enable modem status (CTS) interrupt  */
static void msm_hs_enable_ms_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	clk_enable(msm_uport->clk);

	/* Enable DELTA_CTS Interrupt */
	msm_uport->imr_reg |= UARTDM_ISR_DELTA_CTS_BMSK;
	msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
	/* Calling CLOCK API. Hence mb() requires here. */
	mb();

	clk_disable(msm_uport->clk);

}

/*
 *  Standard API, Break Signal
 *
 * Control the transmission of a break signal. ctl eq 0 => break
 * signal terminate ctl ne 0 => start break signal
 */
static void msm_hs_break_ctl(struct uart_port *uport, int ctl)
{
	unsigned long flags;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	spin_lock_irqsave(&uport->lock, flags);
	clk_enable(msm_uport->clk);
	msm_hs_write(uport, UARTDM_CR_ADDR, ctl ? START_BREAK : STOP_BREAK);
	/* Calling CLOCK API. Hence mb() requires here. */
	mb();
	clk_disable(msm_uport->clk);
	spin_unlock_irqrestore(&uport->lock, flags);
}

static void msm_hs_config_port(struct uart_port *uport, int cfg_flags)
{
	/* unsigned long flags;*/
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	if (cfg_flags & UART_CONFIG_TYPE) {
		uport->type = PORT_MSM;
		msm_hs_request_port(uport);
	}

	/* spin_lock_irqsave(&uport->lock, flags); */
	if (is_gsbi_uart(msm_uport)) {
		if (msm_uport->pclk)
			clk_enable(msm_uport->pclk);
		iowrite32(GSBI_PROTOCOL_UART, msm_uport->mapped_gsbi +
			  GSBI_CONTROL_ADDR);
		if (msm_uport->pclk)
			clk_disable(msm_uport->pclk);
	}
	/* spin_unlock_irqrestore(&uport->lock, flags); */
}

/*  Handle CTS changes (Called from interrupt handler) */
static void msm_hs_handle_delta_cts_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	clk_enable(msm_uport->clk);

	/* clear interrupt */
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_CTS);
	/* Calling CLOCK API. Hence mb() requires here. */
	mb();
	uport->icount.cts++;

	clk_disable(msm_uport->clk);

	/* clear the IOCTL TIOCMIWAIT if called */
	wake_up_interruptible(&uport->state->port.delta_msr_wait);
}

#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
static int read_host_wake_state(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	return gpio_get_value(msm_uport->host_wakeup_pin);
}
#endif

/* check if the TX path is flushed, and if so clock off
 * returns 0 did not clock off, need to retry (still sending final byte)
 *        -1 did not clock off, do not retry
 *         1 if we clocked off
 */
static int msm_hs_check_clock_off_locked(struct uart_port *uport)
{
	unsigned long sr_status;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	struct circ_buf *tx_buf = &uport->state->xmit;

	if (msm_uport->request_clk_off_delay > 0) {
		msm_uport->request_clk_off_delay--;
		return 0;  /* delay */
	}

	/* Cancel if tx tty buffer is not empty, dma is in flight,
	 * or tx fifo is not empty, or rx fifo is not empty */
	if (msm_uport->clk_state != MSM_HS_CLK_REQUEST_OFF ||
		!uart_circ_empty(tx_buf) || msm_uport->tx.dma_in_flight ||
		(msm_uport->imr_reg & UARTDM_ISR_TXLEV_BMSK) ||
		!(msm_uport->imr_reg & UARTDM_ISR_RXLEV_BMSK)) {
		return -1;
	}

	/* Make sure the uart is finished with the last byte */
	sr_status = msm_hs_read(uport, UARTDM_SR_ADDR);
	if (!(sr_status & UARTDM_SR_TXEMT_BMSK))
		return 0;  /* retry */

	#ifdef USE_BCM_BT_CHIP	/* set bt_chip_wakeup pin to high till now */
	/* host has nothing send to bt chip, so set bt_wakeup_pin to high */
	if (msm_uport->host_want_sleep) {
		if ((msm_uport->bt_wakeup_level == 0)
			|| (msm_uport->bt_wakeup_assert_inadvance == 1)) {
			gpio_set_value(msm_uport->bt_wakeup_pin,
				T_HIGH);
			msm_uport->bt_wakeup_level = 1;
			msm_uport->bt_wakeup_assert_inadvance = 0;
			#ifdef BT_SERIAL_DBG
			printk(KERN_INFO "[BT]CHK CLK OFF, BT_WAKE=HIGH\n");
			#endif
		}
	} else {
		/* host doesn't want to sleep, so exit anyway */
		msm_uport->clk_state = MSM_HS_CLK_ON;
		return -1;
	}

	/* now we can start to check rx part	*/
	/* check host_wake pin level */
	if (msm_uport->host_wakeup_level == 0) {
		msm_uport->clk_state = MSM_HS_CLK_ON;
		return -1;	/* host_wake is still low, no need to retry */
	} else if (!read_host_wake_state(uport)) {
		/* this means host wake is LOW now but isr haven't been called,
		so retry again */
		return 0;
	} else {

		if (sr_status & UARTDM_SR_RXRDY_BMSK)
			return 0;  /* some data in rx fifo, so retry */

		/* bt chip has nothing to send to host, so we disable rfr */
		{
			unsigned int data;
			data = msm_hs_read(uport, UARTDM_MR1_ADDR);

			if (data & UARTDM_MR1_RX_RDY_CTL_BMSK) {
				/*disable auto ready-for-receiving */
				data &= ~UARTDM_MR1_RX_RDY_CTL_BMSK;
				msm_hs_write(uport, UARTDM_MR1_ADDR, data);

				/* set RFR_N to high */
				msm_hs_write(uport, UARTDM_CR_ADDR, RFR_HIGH);
				/*complete above write. hence mb() here. */
				mb();
				#ifdef BT_SERIAL_DBG
				printk(KERN_INFO "[BT]- DIS RFR, %d -\n",
						msm_uport->clk_state);
				#endif
			}
		}
	}
	#endif

	/* Make sure forced RXSTALE flush complete */
	switch (msm_uport->clk_req_off_state) {
	case CLK_REQ_OFF_START:
		msm_uport->clk_req_off_state = CLK_REQ_OFF_RXSTALE_ISSUED;
		msm_hs_write(uport, UARTDM_CR_ADDR, FORCE_STALE_EVENT);
		/*
		 * Before returning make sure that device writel completed.
		 * Hence mb() requires here.
		 */
		mb();
		return 0;  /* RXSTALE flush not complete - retry */
	case CLK_REQ_OFF_RXSTALE_ISSUED:
	case CLK_REQ_OFF_FLUSH_ISSUED:
		return 0;  /* RXSTALE flush not complete - retry */
	case CLK_REQ_OFF_RXSTALE_FLUSHED:
		break;  /* continue */
	}

	if (msm_uport->rx.flush != FLUSH_SHUTDOWN) {
		#ifdef BT_SERIAL_DBG
		printk(KERN_INFO "[BT]CHK CLK OFF rx.flush:%d\n",
				msm_uport->rx.flush);
		#endif
		if (msm_uport->rx.flush == FLUSH_NONE)
			msm_hs_stop_rx_locked(uport);
		return 0;  /* come back later to really clock off */
	}

	/* we really want to clock off */
	clk_disable(msm_uport->clk);
	if (msm_uport->pclk)
		clk_disable(msm_uport->pclk);
	msm_uport->clk_state = MSM_HS_CLK_OFF;

	/* release rx wake lock */
	if (msm_uport->rx.is_brcm_rx_wake_locked == 1) {
		wake_unlock(&msm_uport->rx.brcm_rx_wake_lock);
		msm_uport->rx.is_brcm_rx_wake_locked = 0;
	}

	#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
	#ifdef BT_SERIAL_DBG
	printk(KERN_INFO "[BT]CLK OFF, %d\n", msm_uport->clk_state);
	#endif
	#else
	if (use_low_power_wakeup(msm_uport)) {
		msm_uport->wakeup.ignore = 1;
		enable_irq(msm_uport->wakeup.irq);
	}
	#endif
	wake_unlock(&msm_uport->dma_wake_lock);
	return 1;
}

static enum hrtimer_restart msm_hs_clk_off_retry(struct hrtimer *timer)
{
	unsigned long flags;
	int ret = HRTIMER_NORESTART;
	struct msm_hs_port *msm_uport = container_of(timer, struct msm_hs_port,
						     clk_off_timer);
	struct uart_port *uport = &msm_uport->uport;

	spin_lock_irqsave(&uport->lock, flags);

	if (!msm_hs_check_clock_off_locked(uport)) {
		hrtimer_forward_now(timer, msm_uport->clk_off_delay);
		ret = HRTIMER_RESTART;
	}

	spin_unlock_irqrestore(&uport->lock, flags);

	return ret;
}

static irqreturn_t msm_hs_isr(int irq, void *dev)
{
	unsigned long flags;
	unsigned long isr_status;
	struct msm_hs_port *msm_uport = (struct msm_hs_port *)dev;
	struct uart_port *uport = &msm_uport->uport;
	struct circ_buf *tx_buf = &uport->state->xmit;
	struct msm_hs_tx *tx = &msm_uport->tx;
	struct msm_hs_rx *rx = &msm_uport->rx;

	spin_lock_irqsave(&uport->lock, flags);

	isr_status = msm_hs_read(uport, UARTDM_MISR_ADDR);

	/* Uart RX starting */
	if (isr_status & UARTDM_ISR_RXLEV_BMSK) {
		wake_lock(&rx->wake_lock);  /* hold wakelock while rx dma */
		msm_uport->imr_reg &= ~UARTDM_ISR_RXLEV_BMSK;
		msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
		/* Complete device write for IMR. Hence mb() requires. */
		mb();
	}
	/* Stale rx interrupt */
	if (isr_status & UARTDM_ISR_RXSTALE_BMSK) {
		msm_hs_write(uport, UARTDM_CR_ADDR, STALE_EVENT_DISABLE);
		msm_hs_write(uport, UARTDM_CR_ADDR, RESET_STALE_INT);
		/*
		 * Complete device write before calling DMOV API. Hence
		 * mb() requires here.
		 */
		mb();

		if (msm_uport->clk_req_off_state == CLK_REQ_OFF_RXSTALE_ISSUED)
			msm_uport->clk_req_off_state =
				CLK_REQ_OFF_FLUSH_ISSUED;

		if (rx->flush == FLUSH_NONE) {
			rx->flush = FLUSH_DATA_READY;
			msm_dmov_flush(msm_uport->dma_rx_channel);
		}
	}
	/* tx ready interrupt */
	if (isr_status & UARTDM_ISR_TX_READY_BMSK) {
		/* Clear  TX Ready */
		msm_hs_write(uport, UARTDM_CR_ADDR, CLEAR_TX_READY);

		if (msm_uport->clk_state == MSM_HS_CLK_REQUEST_OFF) {
			msm_uport->imr_reg |= UARTDM_ISR_TXLEV_BMSK;
			msm_hs_write(uport, UARTDM_IMR_ADDR,
				     msm_uport->imr_reg);
		}
		/*
		 * Complete both writes before starting new TX.
		 * Hence mb() requires here.
		 */
		mb();
		/* Complete DMA TX transactions and submit new transactions */
		tx_buf->tail = (tx_buf->tail + tx->tx_count) & ~UART_XMIT_SIZE;

		tx->dma_in_flight = 0;

		uport->icount.tx += tx->tx_count;
		if (tx->tx_ready_int_en)
			msm_hs_submit_tx_locked(uport);

		if (uart_circ_chars_pending(tx_buf) < WAKEUP_CHARS)
			uart_write_wakeup(uport);
	}
	if (isr_status & UARTDM_ISR_TXLEV_BMSK) {
		/* TX FIFO is empty */
		msm_uport->imr_reg &= ~UARTDM_ISR_TXLEV_BMSK;
		msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
		/*
		 * Complete device write before starting clock_off request.
		 * Hence mb() requires here.
		 */
		mb();
		if (!msm_hs_check_clock_off_locked(uport))
			hrtimer_start(&msm_uport->clk_off_timer,
				      msm_uport->clk_off_delay,
				      HRTIMER_MODE_REL);
	}

	/* Change in CTS interrupt */
	if (isr_status & UARTDM_ISR_DELTA_CTS_BMSK)
		msm_hs_handle_delta_cts_locked(uport);

	spin_unlock_irqrestore(&uport->lock, flags);

	return IRQ_HANDLED;
}

void brcm_msm_hs_request_clock_off_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	if (msm_uport->clk_state == MSM_HS_CLK_ON) {
		msm_uport->clk_state = MSM_HS_CLK_REQUEST_OFF;
		msm_uport->clk_req_off_state = CLK_REQ_OFF_START;
		msm_uport->imr_reg |= UARTDM_ISR_TXLEV_BMSK;
		msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
		/*
		 * Complete device write before retuning back.
		 * Hence mb() requires here.
		 */
		mb();
	}
}
/* request to turn off uart clock once pending TX is flushed */
void brcm_msm_hs_request_clock_off(struct uart_port *uport)
{
	unsigned long flags;

	spin_lock_irqsave(&uport->lock, flags);
	brcm_msm_hs_request_clock_off_locked(uport);
	spin_unlock_irqrestore(&uport->lock, flags);
}

void brcm_msm_hs_request_clock_on_locked(struct uart_port *uport)
{
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	unsigned int data;
	int ret = 0;

	switch (msm_uport->clk_state) {
	case MSM_HS_CLK_OFF:
		wake_lock(&msm_uport->dma_wake_lock);
		clk_enable(msm_uport->clk);
		if (msm_uport->pclk)
			ret = clk_enable(msm_uport->pclk);
		#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
		#else
		disable_irq_nosync(msm_uport->wakeup.irq);
		/* fall-through */
		#endif
		if (unlikely(ret)) {
			dev_err(uport->dev, "[BT]Clock ON Failure"
				"Stalling HSUART\n");
			break;
		}
		/* else fall-through */
	case MSM_HS_CLK_REQUEST_OFF:
		msm_uport->request_clk_off_delay = 100;
		if (msm_uport->rx.flush == FLUSH_STOP ||
		    msm_uport->rx.flush == FLUSH_SHUTDOWN) {
			msm_hs_write(uport, UARTDM_CR_ADDR, RESET_RX);
			data = msm_hs_read(uport, UARTDM_DMEN_ADDR);
			data |= UARTDM_RX_DM_EN_BMSK;
			msm_hs_write(uport, UARTDM_DMEN_ADDR, data);
			/* Complete above device write. Hence mb() here. */
			mb();
		}
		hrtimer_try_to_cancel(&msm_uport->clk_off_timer);
		if (msm_uport->rx.flush == FLUSH_SHUTDOWN)
			msm_hs_start_rx_locked(uport);
		if (msm_uport->rx.flush == FLUSH_STOP)
			msm_uport->rx.flush = FLUSH_IGNORE;
		msm_uport->clk_state = MSM_HS_CLK_ON;

	#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
	case MSM_HS_CLK_ON:
	case MSM_HS_CLK_PORT_OFF:
		{
			unsigned int data;

			/* If disabled, enable auto ready-for-receiving again */
			data = msm_hs_read(uport, UARTDM_MR1_ADDR);
			if (!(data & UARTDM_MR1_RX_RDY_CTL_BMSK)) {
				data |= UARTDM_MR1_RX_RDY_CTL_BMSK;
				msm_hs_write(uport, UARTDM_MR1_ADDR, data);
				/* Complete above device write.
				Hence mb() here. */
				mb();
				#ifdef BT_SERIAL_DBG
				printk(KERN_INFO "[BT]- EN RFR -\n");
				#endif
			}
		}
		break;
	#else
		break;
	case MSM_HS_CLK_ON: break;
	case MSM_HS_CLK_PORT_OFF: break;
	#endif
	}
}

void brcm_msm_hs_request_clock_on(struct uart_port *uport)
{
	unsigned long flags;
	spin_lock_irqsave(&uport->lock, flags);
	brcm_msm_hs_request_clock_on_locked(uport);
	spin_unlock_irqrestore(&uport->lock, flags);
}

static int
msm_uartdm_ioctl(struct uart_port *uport, unsigned int cmd, unsigned long arg)
{
#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
	void __user *argp = (void __user *)arg;
	unsigned long tbt_wakeup_level;
	int i = 0;

	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	struct msm_hs_tx *tx = &msm_uport->tx;
	unsigned long tflags;

	switch (cmd) {
	case 0x8003:
		/* aquire tx wakelock */
		wake_lock(&tx->brcm_tx_wake_lock);

#if 1/* should we add a if(), only block when RFR is disable? */
		/* block at most 300 ms if CLK=MSM_HS_CLK_REQUEST_OFF */
		for (i=0; i<15; i++) {
			if(msm_uport->clk_state == MSM_HS_CLK_REQUEST_OFF) {
				msleep(20);
				printk(KERN_INFO "[BT]Wait CLK off, round:%d, CLK:%d\n",
					i, msm_uport->clk_state);
			} else {
				break;
			}
		}
#endif
		spin_lock_irqsave(&uport->lock, tflags);
		#ifdef BT_SERIAL_DBG
		printk(KERN_INFO "[BT]-- HOST BT_WAKE=LOW, %d --\n",
				msm_uport->clk_state);
		#endif

		if (msm_uport->bt_wakeup_pin_supported) {
			gpio_set_value(msm_uport->bt_wakeup_pin,
					T_LOW);
			msm_uport->bt_wakeup_level = 0;
			msm_uport->host_want_sleep = 0;
		}
		spin_unlock_irqrestore(&uport->lock, tflags);

		brcm_msm_hs_request_clock_on(uport);

		break;

	case 0x8004:
		#ifdef BT_SERIAL_DBG
		printk(KERN_INFO "[BT]-- HOST BT_WAKE=HIGH, %d --\n",
				msm_uport->clk_state);
		#endif

		msm_uport->host_want_sleep = 1;
		if (msm_uport->host_wakeup_level == 1)
			brcm_msm_hs_request_clock_off(uport);
		#ifdef BT_SERIAL_DBG
		else
			printk(KERN_INFO "[BT]BUT HOST_WAKE==LOW\n");
		#endif

		/* release tx wakelock */
		wake_lock_timeout(&msm_uport->tx.brcm_tx_wake_lock, HZ / 2);

		break;

	case 0x8005:
		/* the logic for rfkill1 is as follow:
		 * rfkill1 return 1,BT_WAKE = LOW,it means awake state
		 * rfkill1 return 0,BT_WAKE = HIGH,it means asleep state
		 * So in ioctl did the same way as rfkill1
		 */
		tbt_wakeup_level = !msm_uport->bt_wakeup_level;
		if (copy_to_user(argp, &tbt_wakeup_level,
				sizeof(tbt_wakeup_level)))
			return -EFAULT;
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
#endif
}

static irqreturn_t msm_hs_wakeup_isr(int irq, void *dev)
{
#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
	unsigned long flags;
	struct msm_hs_port *msm_uport = (struct msm_hs_port *)dev;
	struct uart_port *uport = &msm_uport->uport;

	spin_lock_irqsave(&uport->lock, flags);

	if (msm_uport->host_wakeup_level == 0) { /* host_wake is high */
		#ifdef BT_SERIAL_DBG
		printk(KERN_INFO "[BT]-- CHIP HOST_WAKE=HIGH, %d --\n",
				msm_uport->clk_state);
		#endif
		msm_uport->host_wakeup_level = 1;
		irq_set_irq_type(msm_uport->wakeup.irq, IRQF_TRIGGER_LOW);
		if ((msm_uport->host_want_sleep)
				&& (msm_uport->clk_state == MSM_HS_CLK_ON)) {
			#if 1	/* testing purpose, but do "not" remove it */
			if (msm_uport->bt_wakeup_level)
				msm_uport->request_clk_off_delay = 100;

			msm_uport->clk_state = MSM_HS_CLK_REQUEST_OFF;
			msm_uport->clk_req_off_state = CLK_REQ_OFF_START;
			/* should change to rx?? */
			msm_uport->imr_reg |= UARTDM_ISR_TXLEV_BMSK;
			msm_hs_write(uport,
					UARTDM_IMR_ADDR, msm_uport->imr_reg);
			/* Complete above device write. Hence mb() here. */
			mb();
			#endif
		}

	} else {	/* host_wake is low */
		#ifdef BT_SERIAL_DBG
		printk(KERN_INFO "[BT]-- CHIP HOST_WAKE=LOW, %d --\n",
				msm_uport->clk_state);
		#endif

		/* aquire rx wake lock */
		if (msm_uport->rx.is_brcm_rx_wake_locked == 0) {
			msm_uport->rx.is_brcm_rx_wake_locked = 1;
			wake_lock(&msm_uport->rx.brcm_rx_wake_lock);
		}

		#if 1	/* host asserts bt_wake */
		if ((msm_uport->bt_wakeup_level == 1)
			&& (msm_uport->bt_wakeup_assert_inadvance == 0)) {
			gpio_set_value(msm_uport->bt_wakeup_pin,
					T_LOW);
			msm_uport->bt_wakeup_assert_inadvance = 1;
			#ifdef BT_SERIAL_DBG
			printk(KERN_INFO "[BT]SET BT_WAKE=LOW IN ADV\n");
			#endif
		}
		#endif

		msm_uport->host_wakeup_level = 0;
		irq_set_irq_type(msm_uport->wakeup.irq, IRQF_TRIGGER_HIGH);
		brcm_msm_hs_request_clock_on_locked(uport);
/*
		* btld will set BT_CHIP_WAKEUP HIGH later in this case,
		 * so it is ok here.
		 * *
		msm_uport->host_want_sleep = 0;
*/
	}

	spin_unlock_irqrestore(&uport->lock, flags);

	return IRQ_HANDLED;
#else
	unsigned int wakeup = 0;
	unsigned long flags;
	struct msm_hs_port *msm_uport = (struct msm_hs_port *)dev;
	struct uart_port *uport = &msm_uport->uport;
	struct tty_struct *tty = NULL;

	spin_lock_irqsave(&uport->lock, flags);
	if (msm_uport->clk_state == MSM_HS_CLK_OFF) {
		/* ignore the first irq - it is a pending irq that occured
		 * before enable_irq() */
		if (msm_uport->wakeup.ignore)
			msm_uport->wakeup.ignore = 0;
		else
			wakeup = 1;
	}

	if (wakeup) {
		/* the uart was clocked off during an rx, wake up and
		 * optionally inject char into tty rx */
		brcm_msm_hs_request_clock_on_locked(uport);
		if (msm_uport->wakeup.inject_rx) {
			tty = uport->state->port.tty;
			tty_insert_flip_char(tty,
					     msm_uport->wakeup.rx_to_inject,
					     TTY_NORMAL);
			queue_work(msm_hs_workqueue, &msm_uport->rx.tty_work);
		}
	}

	spin_unlock_irqrestore(&uport->lock, flags);

	return IRQ_HANDLED;
#endif
}

static const char *msm_hs_type(struct uart_port *port)
{
	return ("MSM HS UART");
}

/* Called when port is opened */
static int msm_hs_startup(struct uart_port *uport)
{
	int ret;
	int rfr_level;
	unsigned long flags;
	unsigned int data;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	struct circ_buf *tx_buf = &uport->state->xmit;
	struct msm_hs_tx *tx = &msm_uport->tx;

	printk(KERN_INFO "[BT]== S UP ==\n");

	rfr_level = uport->fifosize;
	if (rfr_level > 16)
		rfr_level -= 16;

	tx->dma_base = dma_map_single(uport->dev, tx_buf->buf, UART_XMIT_SIZE,
				      DMA_TO_DEVICE);

	/* do not let tty layer execute RX in global workqueue, use a
	 * dedicated workqueue managed by this driver */
	uport->state->port.tty->low_latency = 1;

	/* turn on uart clk */
	ret = msm_hs_init_clk(uport);
	if (unlikely(ret))
		return ret;

	/* Set auto RFR Level */
	data = msm_hs_read(uport, UARTDM_MR1_ADDR);
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK;
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK;
	data |= (UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK & (rfr_level << 2));
	data |= (UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK & rfr_level);
	msm_hs_write(uport, UARTDM_MR1_ADDR, data);

	/* Make sure RXSTALE count is non-zero */
	data = msm_hs_read(uport, UARTDM_IPR_ADDR);
	if (!data) {
		data |= 0x1f & UARTDM_IPR_STALE_LSB_BMSK;
		msm_hs_write(uport, UARTDM_IPR_ADDR, data);
	}

	/* Enable Data Mover Mode */
	data = UARTDM_TX_DM_EN_BMSK | UARTDM_RX_DM_EN_BMSK;
	msm_hs_write(uport, UARTDM_DMEN_ADDR, data);

	/* Reset TX */
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_TX);
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_RX);
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_ERROR_STATUS);
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_BREAK_INT);
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_STALE_INT);
	msm_hs_write(uport, UARTDM_CR_ADDR, RESET_CTS);
	msm_hs_write(uport, UARTDM_CR_ADDR, RFR_LOW);
	/* Turn on Uart Receiver */
	msm_hs_write(uport, UARTDM_CR_ADDR, UARTDM_CR_RX_EN_BMSK);

	/* Turn on Uart Transmitter */
	msm_hs_write(uport, UARTDM_CR_ADDR, UARTDM_CR_TX_EN_BMSK);

	/* Initialize the tx */
	tx->tx_ready_int_en = 0;
	tx->dma_in_flight = 0;

	tx->xfer.complete_func = msm_hs_dmov_tx_callback;
	tx->xfer.exec_func = NULL;

	tx->command_ptr->cmd = CMD_LC |
	    CMD_DST_CRCI(msm_uport->dma_tx_crci) | CMD_MODE_BOX;

	tx->command_ptr->src_dst_len = (MSM_UARTDM_BURST_SIZE << 16)
					   | (MSM_UARTDM_BURST_SIZE);

	tx->command_ptr->row_offset = (MSM_UARTDM_BURST_SIZE << 16);

	tx->command_ptr->dst_row_addr =
	    msm_uport->uport.mapbase + UARTDM_TF_ADDR;

	msm_uport->imr_reg |= UARTDM_ISR_RXSTALE_BMSK;
	/* Enable reading the current CTS, no harm even if CTS is ignored */
	msm_uport->imr_reg |= UARTDM_ISR_CURRENT_CTS_BMSK;

	msm_hs_write(uport, UARTDM_TFWR_ADDR, 0);  /* TXLEV on empty TX fifo */
	/*
	 * Complete all device write related configuration before
	 * queuing RX request. Hence mb() requires here.
	 */
	mb();

	if (use_low_power_wakeup(msm_uport)) {
		ret = irq_set_irq_wake(msm_uport->wakeup.irq, 1);
		if (unlikely(ret))
			return ret;
	}

	ret = request_irq(uport->irq, msm_hs_isr, IRQF_TRIGGER_HIGH,
			  "msm_hs_uart", msm_uport);
	if (unlikely(ret))
		return ret;
	if (use_low_power_wakeup(msm_uport)) {
		msm_uport->host_wakeup_level = 1;
		msm_uport->rx.is_brcm_rx_wake_locked = 0;

		/* move from startup  **/
/*		if (unlikely(set_irq_wake(msm_uport->rx_wakeup.irq, 1)))
			return -ENXIO;
*/

		#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
		/* FIXME, should clear irq status first?? */
		#endif

		ret = request_irq(msm_uport->wakeup.irq,
				msm_hs_wakeup_isr,
				#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
				/* IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, */
				IRQF_TRIGGER_LOW,
				#else
				IRQF_TRIGGER_FALLING,
				#endif
				"msm_hs_wakeup", msm_uport);
		if (unlikely(ret))
			return ret;

		#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
		#else
		disable_irq(msm_uport->wakeup.irq);
		#endif
	}

	spin_lock_irqsave(&uport->lock, flags);

	/* Still need this, right? */
	msm_hs_write(uport, UARTDM_RFWR_ADDR, 0);
	/* Complete above device write. Hence mb() here. */
	mb();

	msm_hs_start_rx_locked(uport);

	spin_unlock_irqrestore(&uport->lock, flags);
/*
	ret = pm_runtime_set_active(uport->dev);
	if (ret)
		dev_err(uport->dev, "[BT]set active error:%d\n", ret);
	pm_runtime_enable(uport->dev);
*/

	return 0;
}

/* Initialize tx and rx data structures */
static int uartdm_init_port(struct uart_port *uport)
{
	int ret = 0;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);
	struct msm_hs_tx *tx = &msm_uport->tx;
	struct msm_hs_rx *rx = &msm_uport->rx;

	/* Allocate the command pointer. Needs to be 64 bit aligned */
	tx->command_ptr = kmalloc(sizeof(dmov_box), GFP_KERNEL | __GFP_DMA);
	if (!tx->command_ptr)
		return -ENOMEM;

	tx->command_ptr_ptr = kmalloc(sizeof(u32 *), GFP_KERNEL | __GFP_DMA);
	if (!tx->command_ptr_ptr) {
		ret = -ENOMEM;
		goto free_tx_command_ptr;
	}

	tx->mapped_cmd_ptr = dma_map_single(uport->dev, tx->command_ptr,
					    sizeof(dmov_box), DMA_TO_DEVICE);
	tx->mapped_cmd_ptr_ptr = dma_map_single(uport->dev,
						tx->command_ptr_ptr,
						sizeof(u32), DMA_TO_DEVICE);
	tx->xfer.cmdptr = DMOV_CMD_ADDR(tx->mapped_cmd_ptr_ptr);

	wake_lock_init(&tx->brcm_tx_wake_lock,
			WAKE_LOCK_SUSPEND, "msm_serial_hs_brcm_tx");

	wake_lock_init(&rx->brcm_rx_wake_lock,
			WAKE_LOCK_SUSPEND, "msm_serial_hs_brcm_rx");

	init_waitqueue_head(&rx->wait);
	wake_lock_init(&rx->wake_lock, WAKE_LOCK_SUSPEND, "msm_serial_hs_rx");
	wake_lock_init(&msm_uport->dma_wake_lock, WAKE_LOCK_SUSPEND,
			"msm_serial_hs_dma");

	tasklet_init(&rx->tlet, msm_serial_hs_rx_tlet,
			(unsigned long) &rx->tlet);
	tasklet_init(&tx->tlet, msm_serial_hs_tx_tlet,
			(unsigned long) &tx->tlet);

	rx->pool = dma_pool_create("rx_buffer_pool", uport->dev,
				   UARTDM_RX_BUF_SIZE, 16, 0);

	if (!rx->pool) {
		pr_err("%s(): cannot allocate rx_buffer_pool", __func__);
		ret = -ENOMEM;
		goto exit_tasket_init;
	}

	rx->buffer = dma_pool_alloc(rx->pool, GFP_KERNEL, &rx->rbuffer);
	if (!rx->buffer) {
		pr_err("%s(): cannot allocate rx->buffer", __func__);
		ret = -ENOMEM;
		goto free_pool;
	}

	/* Allocate the command pointer. Needs to be 64 bit aligned */
	rx->command_ptr = kmalloc(sizeof(dmov_box), GFP_KERNEL | __GFP_DMA);
	if (!rx->command_ptr) {
		pr_err("%s(): cannot allocate rx->command_ptr", __func__);
		ret = -ENOMEM;
		goto free_rx_buffer;
	}

	rx->command_ptr_ptr = kmalloc(sizeof(u32), GFP_KERNEL | __GFP_DMA);
	if (!rx->command_ptr_ptr) {
		pr_err("%s(): cannot allocate rx->command_ptr_ptr", __func__);
		ret = -ENOMEM;
		goto free_rx_command_ptr;
	}

	rx->command_ptr->num_rows = ((UARTDM_RX_BUF_SIZE >> 4) << 16) |
					 (UARTDM_RX_BUF_SIZE >> 4);

	rx->command_ptr->dst_row_addr = rx->rbuffer;

	/* Set up Uart Receive */
	msm_hs_write(uport, UARTDM_RFWR_ADDR, 0);

	rx->xfer.complete_func = msm_hs_dmov_rx_callback;

	rx->command_ptr->cmd = CMD_LC |
	    CMD_SRC_CRCI(msm_uport->dma_rx_crci) | CMD_MODE_BOX;

	rx->command_ptr->src_dst_len = (MSM_UARTDM_BURST_SIZE << 16)
					   | (MSM_UARTDM_BURST_SIZE);
	rx->command_ptr->row_offset =  MSM_UARTDM_BURST_SIZE;
	rx->command_ptr->src_row_addr = uport->mapbase + UARTDM_RF_ADDR;

	rx->mapped_cmd_ptr = dma_map_single(uport->dev, rx->command_ptr,
					    sizeof(dmov_box), DMA_TO_DEVICE);

	*rx->command_ptr_ptr = CMD_PTR_LP | DMOV_CMD_ADDR(rx->mapped_cmd_ptr);

	rx->cmdptr_dmaaddr = dma_map_single(uport->dev, rx->command_ptr_ptr,
					    sizeof(u32), DMA_TO_DEVICE);
	rx->xfer.cmdptr = DMOV_CMD_ADDR(rx->cmdptr_dmaaddr);

	INIT_WORK(&rx->tty_work, msm_hs_tty_flip_buffer_work);
	INIT_DELAYED_WORK(&rx->flip_insert_work, flip_insert_work);

	rx->xfer.list.next = LIST_POISON1;
	tx->xfer.list.next = LIST_POISON1;

	return ret;

free_rx_command_ptr:
	kfree(rx->command_ptr);

free_rx_buffer:
	dma_pool_free(msm_uport->rx.pool, msm_uport->rx.buffer,
			msm_uport->rx.rbuffer);

free_pool:
	dma_pool_destroy(msm_uport->rx.pool);

exit_tasket_init:
	wake_lock_destroy(&msm_uport->rx.wake_lock);
	wake_lock_destroy(&msm_uport->dma_wake_lock);
	tasklet_kill(&msm_uport->tx.tlet);
	tasklet_kill(&msm_uport->rx.tlet);
	dma_unmap_single(uport->dev, msm_uport->tx.mapped_cmd_ptr_ptr,
			sizeof(u32 *), DMA_TO_DEVICE);
	dma_unmap_single(uport->dev, msm_uport->tx.mapped_cmd_ptr,
			sizeof(dmov_box), DMA_TO_DEVICE);
	kfree(msm_uport->tx.command_ptr_ptr);

free_tx_command_ptr:
	kfree(msm_uport->tx.command_ptr);
	return ret;
}

static int msm_hs_probe(struct platform_device *pdev)
{
	int ret;
	struct uart_port *uport;
	struct msm_hs_port *msm_uport;
	struct resource *resource;
	struct msm_serial_hs_platform_data *pdata = pdev->dev.platform_data;

	/* for debug */
	printk(KERN_INFO "[BT]BRCM chip\n");

	if (pdev->id < 0 || pdev->id >= UARTDM_NR) {
		printk(KERN_ERR "[BT]Invalid plaform device ID = %d\n",
				pdev->id);
		return -EINVAL;
	}

	msm_uport = &q_uart_port[pdev->id];
	uport = &msm_uport->uport;

	uport->dev = &pdev->dev;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!resource))
		return -ENXIO;
	uport->mapbase = resource->start;  /* virtual address */

	uport->membase = ioremap(uport->mapbase, PAGE_SIZE);
	if (unlikely(!uport->membase))
		return -ENOMEM;

	uport->irq = platform_get_irq(pdev, 0);
	/*
	if (unlikely((int)uport->irq < 0))
		return -ENXIO;
	*/

	if (pdata == NULL || pdata->rx_wakeup_irq < 0)
		msm_uport->wakeup.irq = -1;
	else {
		msm_uport->wakeup.irq = pdata->rx_wakeup_irq;
		msm_uport->wakeup.ignore = 1;
		msm_uport->wakeup.inject_rx = pdata->inject_rx_on_wakeup;
		msm_uport->wakeup.rx_to_inject = pdata->rx_to_inject;

		if (unlikely(msm_uport->wakeup.irq < 0))
			return -ENXIO;
	}

	if (pdata == NULL)
		msm_uport->exit_lpm_cb = NULL;
	else {
		/* remove in kernel 3.0 */
		/* msm_uport->exit_lpm_cb = pdata->exit_lpm_cb; */

		#ifdef USE_BCM_BT_CHIP	/* bt for brcm */
		if (pdata->bt_wakeup_pin_supported) {
			msm_uport->bt_wakeup_pin_supported
				= pdata->bt_wakeup_pin_supported;
			msm_uport->bt_wakeup_pin = pdata->bt_wakeup_pin;
			msm_uport->bt_wakeup_level = 1;
			msm_uport->bt_wakeup_assert_inadvance = 0;
			msm_uport->host_wakeup_pin = pdata->host_wakeup_pin;
			msm_uport->host_want_sleep = 1;
			msm_uport->request_clk_off_delay = 0;
			msm_uport->host_wakeup_level = 1;
		}
		#endif
	}
	resource = platform_get_resource_byname(pdev, IORESOURCE_DMA,
						"uartdm_channels");
	if (unlikely(!resource))
		return -ENXIO;
	msm_uport->dma_tx_channel = resource->start;
	msm_uport->dma_rx_channel = resource->end;

	resource = platform_get_resource_byname(pdev, IORESOURCE_DMA,
						"uartdm_crci");
	if (unlikely(!resource))
		return -ENXIO;
	msm_uport->dma_tx_crci = resource->start;
	msm_uport->dma_rx_crci = resource->end;

	uport->iotype = UPIO_MEM;
	uport->fifosize = 64;
	uport->ops = &msm_hs_ops;
	uport->flags = UPF_BOOT_AUTOCONF;
	uport->uartclk = 7372800;
	msm_uport->imr_reg = 0x0;

	msm_uport->clk = clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(msm_uport->clk))
		return PTR_ERR(msm_uport->clk);

	msm_uport->pclk = clk_get(&pdev->dev, "iface_clk");
	/*
	 * Some configurations do not require explicit pclk control so
	 * do not flag error on pclk get failure.
	 */
	if (IS_ERR(msm_uport->pclk))
		msm_uport->pclk = NULL;

	ret = clk_set_rate(msm_uport->clk, uport->uartclk);
	if (ret) {
		printk(KERN_WARNING "[BT]Error setting clock rate on UART\n");
		return ret;
	}

	ret = uartdm_init_port(uport);
	if (unlikely(ret))
		return ret;

	/* configure the CR Protection to Enable */
	msm_hs_write(uport, UARTDM_CR_ADDR, CR_PROTECTION_EN);
	/*
	 * Enable Command register protection before going ahead as this hw
	 * configuration makes sure that issued cmd to CR register gets complete
	 * before next issued cmd start. Hence mb() requires here.
	 */
	mb();

	msm_uport->clk_state = MSM_HS_CLK_PORT_OFF;
	hrtimer_init(&msm_uport->clk_off_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	msm_uport->clk_off_timer.function = msm_hs_clk_off_retry;
	msm_uport->clk_off_delay = ktime_set(0, 1000000);  /* 1ms */

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_clock.attr);
	if (unlikely(ret))
		return ret;

	uport->line = pdev->id;
	return uart_add_one_port(&msm_hs_driver, uport);
}

static int __init msm_serial_hs_init(void)
{
	int ret;
	int i;

	/* Init all UARTS as non-configured */
	for (i = 0; i < UARTDM_NR; i++)
		q_uart_port[i].uport.type = PORT_UNKNOWN;

	msm_hs_workqueue = create_singlethread_workqueue("msm_hs_brcm_wq");

	ret = uart_register_driver(&msm_hs_driver);
	if (unlikely(ret)) {
		printk(KERN_WARNING "[BT]%s failed to load\n", __func__);
		return ret;
	}
	ret = platform_driver_register(&msm_serial_hs_platform_driver);
	if (ret) {
		printk(KERN_WARNING "[BT]%s failed to load\n", __func__);
		uart_unregister_driver(&msm_hs_driver);
		return ret;
	}

	printk(KERN_INFO "[BT]msm_serial_hs module loaded\n");
	return ret;
}

/*
 *  Called by the upper layer when port is closed.
 *     - Disables the port
 *     - Unhook the ISR
 */
static void msm_hs_shutdown(struct uart_port *uport)
{
	unsigned long flags;
	struct msm_hs_port *msm_uport = UARTDM_TO_MSM(uport);

	BUG_ON(msm_uport->rx.flush < FLUSH_STOP);
	tasklet_kill(&msm_uport->tx.tlet);
	wait_event(msm_uport->rx.wait, msm_uport->rx.flush == FLUSH_SHUTDOWN);
	tasklet_kill(&msm_uport->rx.tlet);
	cancel_delayed_work_sync(&msm_uport->rx.flip_insert_work);

	spin_lock_irqsave(&uport->lock, flags);
	clk_enable(msm_uport->clk);

	/* Disable the transmitter */
	msm_hs_write(uport, UARTDM_CR_ADDR, UARTDM_CR_TX_DISABLE_BMSK);
	/* Disable the receiver */
	msm_hs_write(uport, UARTDM_CR_ADDR, UARTDM_CR_RX_DISABLE_BMSK);

	/* disable irq wakeup when shutdown **/
	if (use_low_power_wakeup(msm_uport))
		irq_set_irq_wake(msm_uport->wakeup.irq, 0);


	/* Remove it here to make sure wakelock is released when shutdown */
	wake_lock_timeout(&msm_uport->tx.brcm_tx_wake_lock, HZ / 2);
	wake_lock_timeout(&msm_uport->rx.brcm_rx_wake_lock, HZ / 2);
	/* make sure wake_lock is released */
	wake_lock_timeout(&msm_uport->rx.wake_lock, HZ / 10);

	/* Free the interrupt */
	free_irq(uport->irq, msm_uport);
	if (use_low_power_wakeup(msm_uport))
		free_irq(msm_uport->wakeup.irq, msm_uport);

	msm_uport->imr_reg = 0;
	msm_hs_write(uport, UARTDM_IMR_ADDR, msm_uport->imr_reg);
	/*
	 * Complete all device write before actually disabling uartclk.
	 * Hence mb() requires here.
	 */
	mb();

	clk_disable(msm_uport->clk);  /* to balance local clk_enable() */
	if (msm_uport->clk_state != MSM_HS_CLK_OFF) {
		clk_disable(msm_uport->clk);  /* to balance clk_state */
		if (msm_uport->pclk)
			clk_disable(msm_uport->pclk);
		wake_unlock(&msm_uport->dma_wake_lock);
	}
	msm_uport->clk_state = MSM_HS_CLK_PORT_OFF;

	dma_unmap_single(uport->dev, msm_uport->tx.dma_base,
			 UART_XMIT_SIZE, DMA_TO_DEVICE);

	spin_unlock_irqrestore(&uport->lock, flags);

	if (cancel_work_sync(&msm_uport->rx.tty_work))
		msm_hs_tty_flip_buffer_work(&msm_uport->rx.tty_work);

/*
	pm_runtime_disable(uport->dev);
	pm_runtime_set_suspended(uport->dev);
*/
	printk(KERN_INFO "[BT]== S DN ==\n");

}

static void __exit msm_serial_hs_exit(void)
{
	printk(KERN_INFO "[BT]msm_serial_hs module removed\n");
	platform_driver_unregister(&msm_serial_hs_platform_driver);
	uart_unregister_driver(&msm_hs_driver);
	destroy_workqueue(msm_hs_workqueue);
}

static int msm_hs_runtime_idle(struct device *dev)
{
	/*
	 * returning success from idle results in runtime suspend to be
	 * called
	 */
	return 0;
}

static int msm_hs_runtime_resume(struct device *dev)
{
/*
	struct platform_device *pdev = container_of(dev, struct
						    platform_device, dev);
	struct msm_hs_port *msm_uport = &q_uart_port[pdev->id];
	msm_hs_request_clock_on(&msm_uport->uport);
*/
	return 0;
}

static int msm_hs_runtime_suspend(struct device *dev)
{
/*
	struct platform_device *pdev = container_of(dev, struct
						    platform_device, dev);
	struct msm_hs_port *msm_uport = &q_uart_port[pdev->id];
	msm_hs_request_clock_off(&msm_uport->uport);
*/
	return 0;
}

static const struct dev_pm_ops msm_hs_dev_pm_ops = {
	.runtime_suspend = msm_hs_runtime_suspend,
	.runtime_resume  = msm_hs_runtime_resume,
	.runtime_idle    = msm_hs_runtime_idle,
};

static struct platform_driver msm_serial_hs_platform_driver = {
	.probe = msm_hs_probe,
	.remove = msm_hs_remove,
	.driver = {
		   .name = "msm_serial_hs_brcm",
		.pm   = &msm_hs_dev_pm_ops,
	},
};

static struct uart_driver msm_hs_driver = {
	.owner = THIS_MODULE,
	.driver_name = "msm_serial_hs_brcm",
	.dev_name = "ttyHS",
	.nr = UARTDM_NR,
	.cons = 0,
};

static struct uart_ops msm_hs_ops = {
	.tx_empty = brcm_msm_hs_tx_empty,
	.set_mctrl = msm_hs_set_mctrl_locked,
	.get_mctrl = msm_hs_get_mctrl_locked,
	.stop_tx = msm_hs_stop_tx_locked,
	.start_tx = msm_hs_start_tx_locked,
	.stop_rx = msm_hs_stop_rx_locked,
	.enable_ms = msm_hs_enable_ms_locked,
	.break_ctl = msm_hs_break_ctl,
	.startup = msm_hs_startup,
	.shutdown = msm_hs_shutdown,
	.set_termios = msm_hs_set_termios,
	.type = msm_hs_type,
	.config_port = msm_hs_config_port,
	.ioctl = msm_uartdm_ioctl,
	.release_port = msm_hs_release_port,
	.request_port = msm_hs_request_port,
};

module_init(msm_serial_hs_init);
module_exit(msm_serial_hs_exit);
MODULE_DESCRIPTION("High Speed UART Driver for the MSM chipset");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL v2");
