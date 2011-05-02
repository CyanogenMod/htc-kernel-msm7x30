/*
 * USB Serial Converter Generic functions
 *
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>

static int debug;

#ifdef CONFIG_USB_SERIAL_GENERIC

static int generic_probe(struct usb_interface *interface,
			 const struct usb_device_id *id);

static __u16 vendor  = 0x05f9;
static __u16 product = 0xffff;

module_param(vendor, ushort, 0);
MODULE_PARM_DESC(vendor, "User specified USB idVendor");

module_param(product, ushort, 0);
MODULE_PARM_DESC(product, "User specified USB idProduct");

static struct usb_device_id generic_device_ids[2]; /* Initially all zeroes. */

/* we want to look at all devices, as the vendor/product id can change
 * depending on the command line argument */
static struct usb_device_id generic_serial_ids[] = {
	{.driver_info = 42},
	{}
};

static struct usb_driver generic_driver = {
	.name =		"usbserial_generic",
	.probe =	generic_probe,
	.disconnect =	usb_serial_disconnect,
	.id_table =	generic_serial_ids,
	.no_dynamic_id =	1,
};

/* All of the device info needed for the Generic Serial Converter */
struct usb_serial_driver usb_serial_generic_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"generic",
	},
	.id_table =		generic_device_ids,
	.usb_driver = 		&generic_driver,
	.num_ports =		1,
	.disconnect =		usb_serial_generic_disconnect,
	.release =		usb_serial_generic_release,
	.throttle =		usb_serial_generic_throttle,
	.unthrottle =		usb_serial_generic_unthrottle,
	.resume =		usb_serial_generic_resume,
};

static int generic_probe(struct usb_interface *interface,
			       const struct usb_device_id *id)
{
	const struct usb_device_id *id_pattern;

	id_pattern = usb_match_id(interface, generic_device_ids);
	if (id_pattern != NULL)
		return usb_serial_probe(interface, id);
	return -ENODEV;
}
#endif

int usb_serial_generic_register(int _debug)
{
	int retval = 0;

	debug = _debug;
#ifdef CONFIG_USB_SERIAL_GENERIC
	generic_device_ids[0].idVendor = vendor;
	generic_device_ids[0].idProduct = product;
	generic_device_ids[0].match_flags =
		USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT;

	/* register our generic driver with ourselves */
	retval = usb_serial_register(&usb_serial_generic_device);
	if (retval)
		goto exit;
	retval = usb_register(&generic_driver);
	if (retval)
		usb_serial_deregister(&usb_serial_generic_device);
exit:
#endif
	return retval;
}

void usb_serial_generic_deregister(void)
{
#ifdef CONFIG_USB_SERIAL_GENERIC
	/* remove our generic driver */
	usb_deregister(&generic_driver);
	usb_serial_deregister(&usb_serial_generic_device);
#endif
}

int usb_serial_generic_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	int result = 0;
	unsigned long flags;

	dbg("%s - port %d", __func__, port->number);

	/* clear the throttle flags */
	spin_lock_irqsave(&port->lock, flags);
	port->throttled = 0;
	port->throttle_req = 0;
	spin_unlock_irqrestore(&port->lock, flags);

	/* if we have a bulk endpoint, start reading from it */
	if (serial->num_bulk_in) {
		/* Start reading from the device */
		usb_fill_bulk_urb(port->read_urb, serial->dev,
				   usb_rcvbulkpipe(serial->dev,
						port->bulk_in_endpointAddress),
				   port->read_urb->transfer_buffer,
				   port->read_urb->transfer_buffer_length,
				   ((serial->type->read_bulk_callback) ?
				     serial->type->read_bulk_callback :
				     usb_serial_generic_read_bulk_callback),
				   port);
		result = usb_submit_urb(port->read_urb, GFP_KERNEL);
		if (result)
			dev_err(&port->dev,
			    "%s - failed resubmitting read urb, error %d\n",
							__func__, result);
	}

	return result;
}
EXPORT_SYMBOL_GPL(usb_serial_generic_open);

static void generic_cleanup(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;

	dbg("%s - port %d", __func__, port->number);

	if (serial->dev) {
		/* shutdown any bulk reads that might be going on */
		if (serial->num_bulk_out)
			usb_kill_urb(port->write_urb);
		if (serial->num_bulk_in)
			usb_kill_urb(port->read_urb);
	}
}

void usb_serial_generic_close(struct usb_serial_port *port)
{
	dbg("%s - port %d", __func__, port->number);
	generic_cleanup(port);
}

static int usb_serial_multi_urb_write(struct tty_struct *tty,
	struct usb_serial_port *port, const unsigned char *buf, int count)
{
	unsigned long flags;
	struct urb *urb;
	unsigned char *buffer;
	int status;
	int towrite;
	int bwrite = 0;

	dbg("%s - port %d", __func__, port->number);

	if (count == 0)
		dbg("%s - write request of 0 bytes", __func__);

	while (count > 0) {
		towrite = (count > port->bulk_out_size) ?
			port->bulk_out_size : count;
		spin_lock_irqsave(&port->lock, flags);
		if (port->urbs_in_flight >
		    port->serial->type->max_in_flight_urbs) {
			spin_unlock_irqrestore(&port->lock, flags);
			dbg("%s - write limit hit\n", __func__);
			return bwrite;
		}
		port->tx_bytes_flight += towrite;
		port->urbs_in_flight++;
		spin_unlock_irqrestore(&port->lock, flags);

		buffer = kmalloc(towrite, GFP_ATOMIC);
		if (!buffer) {
			dev_err(&port->dev,
			"%s ran out of kernel memory for urb ...\n", __func__);
			goto error_no_buffer;
		}

		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb) {
			dev_err(&port->dev, "%s - no more free urbs\n",
				__func__);
			goto error_no_urb;
		}

		/* Copy data */
		memcpy(buffer, buf + bwrite, towrite);
		usb_serial_debug_data(debug, &port->dev, __func__,
				      towrite, buffer);
		/* fill the buffer and send it */
		usb_fill_bulk_urb(urb, port->serial->dev,
			usb_sndbulkpipe(port->serial->dev,
					port->bulk_out_endpointAddress),
			buffer, towrite,
			usb_serial_generic_write_bulk_callback, port);

		status = usb_submit_urb(urb, GFP_ATOMIC);
		if (status) {
			dev_err(&port->dev,
				"%s - failed submitting write urb, error %d\n",
				__func__, status);
			goto error;
		}

		/* This urb is the responsibility of the host driver now */
		usb_free_urb(urb);
		dbg("%s write: %d", __func__, towrite);
		count -= towrite;
		bwrite += towrite;
	}
	return bwrite;

error:
	usb_free_urb(urb);
error_no_urb:
	kfree(buffer);
error_no_buffer:
	spin_lock_irqsave(&port->lock, flags);
	port->urbs_in_flight--;
	port->tx_bytes_flight -= towrite;
	spin_unlock_irqrestore(&port->lock, flags);
	return bwrite;
}

/**
 * usb_serial_generic_write_start - kick off an URB write
 * @port:	Pointer to the &struct usb_serial_port data
 *
 * Returns the number of bytes queued on success. This will be zero if there
 * was nothing to send. Otherwise, it returns a negative errno value
 */
static int usb_serial_generic_write_start(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	unsigned char *data;
	int result;
	int count;
	unsigned long flags;
	bool start_io;

	/* Atomically determine whether we can and need to start a USB
	 * operation. */
	spin_lock_irqsave(&port->lock, flags);
	if (port->write_urb_busy)
		start_io = false;
	else {
		start_io = (__kfifo_len(port->write_fifo) != 0);
		port->write_urb_busy = start_io;
	}
	spin_unlock_irqrestore(&port->lock, flags);

	if (!start_io)
		return 0;

	data = port->write_urb->transfer_buffer;
	count = kfifo_get(port->write_fifo, data, port->bulk_out_size);
	usb_serial_debug_data(debug, &port->dev, __func__, count, data);

	/* set up our urb */
	usb_fill_bulk_urb(port->write_urb, serial->dev,
			   usb_sndbulkpipe(serial->dev,
				port->bulk_out_endpointAddress),
			   port->write_urb->transfer_buffer, count,
			   ((serial->type->write_bulk_callback) ?
			     serial->type->write_bulk_callback :
			     usb_serial_generic_write_bulk_callback),
			   port);

	/* send the data out the bulk port */
	result = usb_submit_urb(port->write_urb, GFP_ATOMIC);
	if (result) {
		dev_err(&port->dev,
			"%s - failed submitting write urb, error %d\n",
						__func__, result);
		/* don't have to grab the lock here, as we will
		   retry if != 0 */
		port->write_urb_busy = 0;
	} else
		result = count;

	return result;
}

/**
 * usb_serial_generic_write - generic write function for serial USB devices
 * @tty:	Pointer to &struct tty_struct for the device
 * @port:	Pointer to the &usb_serial_port structure for the device
 * @buf:	Pointer to the data to write
 * @count:	Number of bytes to write
 *
 * Returns the number of characters actually written, which may be anything
 * from zero to @count. If an error occurs, it returns the negative errno
 * value.
 */
int usb_serial_generic_write(struct tty_struct *tty,
	struct usb_serial_port *port, const unsigned char *buf, int count)
{
	struct usb_serial *serial = port->serial;
	int result;

	dbg("%s - port %d", __func__, port->number);

	if (count == 0) {
		dbg("%s - write request of 0 bytes", __func__);
		return 0;
	}

	/* only do something if we have a bulk out endpoint */
	if (!serial->num_bulk_out)
		return 0;

	if (serial->type->max_in_flight_urbs)
		return usb_serial_multi_urb_write(tty, port,
						  buf, count);

	count = kfifo_put(port->write_fifo, buf, count);
	result = usb_serial_generic_write_start(port);

	if (result >= 0)
		result = count;

	return result;
}
EXPORT_SYMBOL_GPL(usb_serial_generic_write);

int usb_serial_generic_write_room(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct usb_serial *serial = port->serial;
	unsigned long flags;
	int room = 0;

	dbg("%s - port %d", __func__, port->number);
	spin_lock_irqsave(&port->lock, flags);
	if (serial->type->max_in_flight_urbs) {
		if (port->urbs_in_flight < serial->type->max_in_flight_urbs)
			room = port->bulk_out_size *
				(serial->type->max_in_flight_urbs -
				 port->urbs_in_flight);
	} else if (serial->num_bulk_out)
		room = port->write_fifo->size - __kfifo_len(port->write_fifo);
	spin_unlock_irqrestore(&port->lock, flags);

	dbg("%s - returns %d", __func__, room);
	return room;
}

int usb_serial_generic_chars_in_buffer(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct usb_serial *serial = port->serial;
	int chars = 0;
	unsigned long flags;

	dbg("%s - port %d", __func__, port->number);

	if (serial->type->max_in_flight_urbs) {
		spin_lock_irqsave(&port->lock, flags);
		chars = port->tx_bytes_flight;
		spin_unlock_irqrestore(&port->lock, flags);
	} else if (serial->num_bulk_out)
		chars = kfifo_len(port->write_fifo);

	dbg("%s - returns %d", __func__, chars);
	return chars;
}


void usb_serial_generic_resubmit_read_urb(struct usb_serial_port *port,
			gfp_t mem_flags)
{
	struct urb *urb = port->read_urb;
	struct usb_serial *serial = port->serial;
	int result;

	/* Continue reading from device */
	usb_fill_bulk_urb(urb, serial->dev,
			   usb_rcvbulkpipe(serial->dev,
					port->bulk_in_endpointAddress),
			   urb->transfer_buffer,
			   urb->transfer_buffer_length,
			   ((serial->type->read_bulk_callback) ?
			     serial->type->read_bulk_callback :
			     usb_serial_generic_read_bulk_callback), port);
	result = usb_submit_urb(urb, mem_flags);
	if (result)
		dev_err(&port->dev,
			"%s - failed resubmitting read urb, error %d\n",
							__func__, result);
}
EXPORT_SYMBOL_GPL(usb_serial_generic_resubmit_read_urb);

/* Push data to tty layer and resubmit the bulk read URB */
static void flush_and_resubmit_read_urb(struct usb_serial_port *port)
{
	struct urb *urb = port->read_urb;
	struct tty_struct *tty = tty_port_tty_get(&port->port);
	char *ch = (char *)urb->transfer_buffer;
	int i;

	if (!tty)
		goto done;

	/* The per character mucking around with sysrq path it too slow for
	   stuff like 3G modems, so shortcircuit it in the 99.9999999% of cases
	   where the USB serial is not a console anyway */
	if (!port->console || !port->sysrq)
		tty_insert_flip_string(tty, ch, urb->actual_length);
	else {
		/* Push data to tty */
		for (i = 0; i < urb->actual_length; i++, ch++) {
			if (!usb_serial_handle_sysrq_char(tty, port, *ch))
				tty_insert_flip_char(tty, *ch, TTY_NORMAL);
		}
	}
	tty_flip_buffer_push(tty);
	tty_kref_put(tty);
done:
	usb_serial_generic_resubmit_read_urb(port, GFP_ATOMIC);
}

void usb_serial_generic_read_bulk_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	unsigned char *data = urb->transfer_buffer;
	int status = urb->status;
	unsigned long flags;

	dbg("%s - port %d", __func__, port->number);

	if (unlikely(status != 0)) {
		dbg("%s - nonzero read bulk status received: %d",
		    __func__, status);
		return;
	}

	usb_serial_debug_data(debug, &port->dev, __func__,
						urb->actual_length, data);

	/* Throttle the device if requested by tty */
	spin_lock_irqsave(&port->lock, flags);
	port->throttled = port->throttle_req;
	if (!port->throttled) {
		spin_unlock_irqrestore(&port->lock, flags);
		flush_and_resubmit_read_urb(port);
	} else
		spin_unlock_irqrestore(&port->lock, flags);
}
EXPORT_SYMBOL_GPL(usb_serial_generic_read_bulk_callback);

void usb_serial_generic_write_bulk_callback(struct urb *urb)
{
	unsigned long flags;
	struct usb_serial_port *port = urb->context;
	int status = urb->status;

	dbg("%s - port %d", __func__, port->number);

	if (port->serial->type->max_in_flight_urbs) {
		kfree(urb->transfer_buffer);

		spin_lock_irqsave(&port->lock, flags);
		--port->urbs_in_flight;
		port->tx_bytes_flight -= urb->transfer_buffer_length;
		if (port->urbs_in_flight < 0)
			port->urbs_in_flight = 0;
		spin_unlock_irqrestore(&port->lock, flags);

		if (status) {
			dbg("%s - nonzero multi-urb write bulk status "
				"received: %d", __func__, status);
			return;
		}
	} else {
		port->write_urb_busy = 0;

		if (status) {
			dbg("%s - nonzero multi-urb write bulk status "
				"received: %d", __func__, status);
			kfifo_reset(port->write_fifo);
		} else
			usb_serial_generic_write_start(port);
	}

	usb_serial_port_softint(port);
}
EXPORT_SYMBOL_GPL(usb_serial_generic_write_bulk_callback);

void usb_serial_generic_throttle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	unsigned long flags;

	dbg("%s - port %d", __func__, port->number);

	/* Set the throttle request flag. It will be picked up
	 * by usb_serial_generic_read_bulk_callback(). */
	spin_lock_irqsave(&port->lock, flags);
	port->throttle_req = 1;
	spin_unlock_irqrestore(&port->lock, flags);
}

void usb_serial_generic_unthrottle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	int was_throttled;
	unsigned long flags;

	dbg("%s - port %d", __func__, port->number);

	/* Clear the throttle flags */
	spin_lock_irqsave(&port->lock, flags);
	was_throttled = port->throttled;
	port->throttled = port->throttle_req = 0;
	spin_unlock_irqrestore(&port->lock, flags);

	if (was_throttled) {
		/* Resume reading from device */
		flush_and_resubmit_read_urb(port);
	}
}

int usb_serial_handle_sysrq_char(struct tty_struct *tty,
			struct usb_serial_port *port, unsigned int ch)
{
	if (port->sysrq && port->console) {
		if (ch && time_before(jiffies, port->sysrq)) {
			handle_sysrq(ch, tty);
			port->sysrq = 0;
			return 1;
		}
		port->sysrq = 0;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(usb_serial_handle_sysrq_char);

int usb_serial_handle_break(struct usb_serial_port *port)
{
	if (!port->sysrq) {
		port->sysrq = jiffies + HZ*5;
		return 1;
	}
	port->sysrq = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(usb_serial_handle_break);

/**
 *	usb_serial_handle_dcd_change - handle a change of carrier detect state
 *	@port: usb_serial_port structure for the open port
 *	@tty: tty_struct structure for the port
 *	@status: new carrier detect status, nonzero if active
 */
void usb_serial_handle_dcd_change(struct usb_serial_port *usb_port,
				struct tty_struct *tty, unsigned int status)
{
	struct tty_port *port = &usb_port->port;

	dbg("%s - port %d, status %d", __func__, usb_port->number, status);

	if (status)
		wake_up_interruptible(&port->open_wait);
	else if (tty && !C_CLOCAL(tty))
		tty_hangup(tty);
}
EXPORT_SYMBOL_GPL(usb_serial_handle_dcd_change);

int usb_serial_generic_resume(struct usb_serial *serial)
{
	struct usb_serial_port *port;
	int i, c = 0, r;

	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		if (!port->port.count)
			continue;

		if (port->read_urb) {
			r = usb_submit_urb(port->read_urb, GFP_NOIO);
			if (r < 0)
				c++;
		}

		if (port->write_urb) {
			r = usb_serial_generic_write_start(port);
			if (r < 0)
				c++;
		}
	}

	return c ? -EIO : 0;
}
EXPORT_SYMBOL_GPL(usb_serial_generic_resume);

void usb_serial_generic_disconnect(struct usb_serial *serial)
{
	int i;

	dbg("%s", __func__);

	/* stop reads and writes on all ports */
	for (i = 0; i < serial->num_ports; ++i)
		generic_cleanup(serial->port[i]);
}

void usb_serial_generic_release(struct usb_serial *serial)
{
	dbg("%s", __func__);
}
